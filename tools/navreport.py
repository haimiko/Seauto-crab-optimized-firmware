#!/usr/bin/env python3
"""Offline analysis of diag recorder CSV/JSONL; no BLE or firmware writes.

python -I tools/navreport.py run.csv --out analysis/floor_run

Fields/units: crab_ctl.py DIAG_SAMPLE_FIELDS. Floor-only is
CTYPE/CSUB 1/1. Sonar 9999 means no echo.
Analysis filters below are quality filters, not firmware thresholds. Sonar closing
rate is a candidate, not measured ground speed; no calibration file is generated.
"""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path

FIELDS = ("t_ms", "yaw", "pitch", "roll", "sonar_cm", "ltrack", "rtrack", "pump",
          "ctype", "csub", "state", "step", "inwater", "fault", "batt", "pins",
          "guard", "bump")
SIGNED = {"yaw", "pitch", "roll", "ltrack", "rtrack"}
HALFWORDS = {"sonar_cm", "bump"}
UINT32 = {"t_ms", "seq"}


def integer(value, name):
    if isinstance(value, bool) or not isinstance(value, (int, str)):
        raise ValueError(f"{name}: expected an integer")
    try:
        result = int(value)
    except ValueError as error:
        raise ValueError(f"{name}: expected an integer") from error
    lo, hi = ((-32768, 32767) if name in SIGNED else
              (0, 65535) if name in HALFWORDS else
              (0, 0xFFFFFFFF) if name in UINT32 else (0, 255))
    if not lo <= result <= hi:
        raise ValueError(f"{name}: outside recorded field range {lo}..{hi}")
    return result


def validate(samples):
    result = []
    for number, raw in enumerate(samples):
        if not isinstance(raw, dict):
            raise ValueError(f"sample {number}: expected an object")
        missing = set(FIELDS) - raw.keys()
        if missing:
            raise ValueError(f"sample {number}: missing {', '.join(sorted(missing))}")
        sample = {k: integer(raw[k], k) for k in FIELDS}
        if "seq" in raw:
            sample["seq"] = integer(raw["seq"], "seq")
        if sample["inwater"] not in (0, 1):
            raise ValueError(f"sample {number}: inwater must be 0 or 1")
        result.append(sample)
    if result and any("seq" in s for s in result) and not all("seq" in s for s in result):
        raise ValueError("mixed presence of sequence numbers")
    return result


def load_recording(path, capture=None):
    """CLI recorder JSONL or CSV; bench event JSONL requires selection if ambiguous."""
    text = path.read_text(encoding="utf-8-sig")
    if path.suffix.lower() == ".csv":
        if capture is not None:
            raise ValueError("--capture applies only to bench-event JSONL")
        return validate(list(csv.DictReader(text.splitlines())))
    rows = [json.loads(line) for line in text.splitlines() if line.strip()]
    captures = [r["samples"] for r in rows if isinstance(r, dict) and
                r.get("event") == "recorder" and isinstance(r.get("samples"), list)]
    if captures:
        if len(captures) > 1 and capture is None:
            raise ValueError(f"{len(captures)} recorder downloads; choose --capture 1..{len(captures)}")
        index = 1 if capture is None else capture
        if not 1 <= index <= len(captures):
            raise ValueError("--capture outside available downloads")
        return validate(captures[index - 1])
    if capture is not None:
        raise ValueError("no bench recorder download to select")
    return validate(rows)


def angle_delta(new, old):
    """Shortest signed change, raw headings in 0.1 degree; sign is not left/right."""
    return ((new - old + 1800) % 3600 - 1800) / 10


def timeline(samples):
    elapsed, gaps, times = 0, [], []
    for index, sample in enumerate(samples):
        if index:
            previous = samples[index - 1]
            delta = (sample["t_ms"] - previous["t_ms"]) & 0xFFFFFFFF
            if delta >= 0x80000000:
                raise ValueError("uptime moved backwards: separate boots must be analyzed separately")
            elapsed += delta
            sequence_gap = False
            if "seq" in sample:
                advance = (sample["seq"] - previous["seq"]) & 0xFFFFFFFF
                if advance == 0 or advance >= 0x80000000:
                    raise ValueError("sequence repeated or moved backwards")
                sequence_gap = advance != 1
            if sequence_gap or delta > 5000:
                gaps.append(index)
        times.append(elapsed / 1000)
    return times, gaps


def floor(sample):
    return sample["inwater"] == 1 and (sample["ctype"], sample["csub"]) == (1, 1)


def unwrap(values):
    result = [values[0] / 10]
    for a, b in zip(values, values[1:]):
        result.append(result[-1] + angle_delta(b, a))
    return result


def fit(times, values):
    mx, my = sum(times) / len(times), sum(values) / len(values)
    xx = sum((x - mx) ** 2 for x in times)
    yy = sum((y - my) ** 2 for y in values)
    if xx == 0 or yy == 0:
        return None
    slope = sum((x - mx) * (y - my) for x, y in zip(times, values)) / xx
    residual = sum((y - (my + slope * (x - mx))) ** 2 for x, y in zip(times, values))
    return slope, max(0, 1 - residual / yy)


def sonar_candidate(block, times):
    """Conservative straight-approach fit; curved/sloped/changed targets still confound it."""
    if len(set(times)) < 6 or times[-1] - times[0] < 3:
        return None
    headings = unwrap([s["yaw"] for s in block])
    if max(headings) - min(headings) > 5:
        return None
    if any(max(s[k] for s in block) - min(s[k] for s in block) > 50
           for k in ("pitch", "roll")):
        return None
    ranges = [s["sonar_cm"] for s in block]
    result = fit(times, ranges)
    if result is None:
        return None
    slope, r2 = result
    if slope >= 0 or r2 < .90 or ranges[0] - ranges[-1] < 50:
        return None
    differences = [b - a for a, b in zip(ranges, ranges[1:]) if b != a]
    if not differences or sum(d <= 0 for d in differences) / len(differences) < .8:
        return None
    return {"start_s": times[0], "end_s": times[-1], "samples": len(block),
            "mean_track_command": sum((s["ltrack"] + s["rtrack"]) / 2 for s in block) / len(block),
            "sonar_closing_m_s": -slope / 100, "r_squared": r2,
            "classification": "candidate only; not ground speed or simulator calibration"}


def analyze(samples):
    samples = validate(samples)
    times, gaps = timeline(samples)
    gap_set = set(gaps)
    phases, candidates = [], []
    start = 0
    key = lambda s: (s["inwater"], s["ctype"], s["csub"], s["state"], s["step"], s["fault"])
    for end in range(1, len(samples) + 1):
        if end < len(samples) and key(samples[end]) == key(samples[start]) and end not in gap_set:
            continue
        block = samples[start:end]
        headings = unwrap([s["yaw"] for s in block])
        phases.append({"start_s": times[start], "end_s": times[end - 1],
                       "observed_span_s": times[end - 1] - times[start],
                       "samples": len(block), "floor_only": floor(block[0]),
                       **{k: block[0][k] for k in ("inwater", "ctype", "csub", "state", "step", "fault")},
                       "yaw_change_deg": headings[-1] - headings[0]})
        start = end
    # Invalid sonar/turns, missing samples and sparse on-change observations break a fit.
    block, block_times = [], []
    for index in range(len(samples) + 1):
        eligible = False
        if index < len(samples):
            s = samples[index]
            eligible = (floor(s) and s["fault"] == 0 and s["state"] == 0 and
                        s["step"] in (1, 5, 101) and s["pump"] > 0 and
                        min(s["ltrack"], s["rtrack"]) > 0 and abs(s["ltrack"] - s["rtrack"]) <= 10 and
                        0 < s["sonar_cm"] < 9999 and
                        all(s[k] not in (-32768, 32767) for k in ("yaw", "pitch", "roll")))
        boundary = (index in gap_set or index < len(samples) and block and
                    (key(s) != key(block[-1]) or times[index] - block_times[-1] > 1))
        if not eligible or boundary:
            if block:
                candidate = sonar_candidate(block, block_times)
                if candidate:
                    candidates.append(candidate)
                block, block_times = [], []
        if eligible:
            block.append(s)
            block_times.append(times[index])
    warnings = []
    count = sum(floor(s) for s in samples)
    if not samples:
        warnings.append("Empty recording; no run result can be inferred.")
    if not count:
        warnings.append("No in-water CTYPE/CSUB 1/1 samples: Floor-only run not established.")
    if gaps:
        warnings.append("Sequence gaps or time gaps >5 s; phases/fits split at those gaps.")
    if any(s["fault"] for s in samples):
        warnings.append("Fault present in recording; inspect before another run.")
    return {"sample_count": len(samples), "observed_span_s": times[-1] if times else 0,
            "floor_only_samples": count, "in_water_samples": sum(s["inwater"] for s in samples),
            "fault_codes": sorted({s["fault"] for s in samples if s["fault"]}),
            "first_uptime_ms": samples[0]["t_ms"] if samples else None,
            "last_uptime_ms": samples[-1]["t_ms"] if samples else None,
            "gap_indices": gaps, "phases": phases, "sonar_candidates": candidates,
            "warnings": warnings,
            "unmeasured": ["effective cleaning width", "true ground speed / slip",
                           "gyro drift against an external heading", "pool boundary / coverage"]}


def markdown(report):
    lines = ["# Offline Floor-only recorder report", "",
             f"Source: `{report['source']['path']}`; SHA-256 `{report['source']['sha256']}`.", "",
             f"{report['sample_count']} samples, observed span {report['observed_span_s']:.3f} s; "
             f"{report['floor_only_samples']} in-water Floor-only samples.",
             f"Recorded nonzero fault codes: {report['fault_codes'] or 'none'}.", "",
             "Sonar closing rates are candidates, not ground-speed measurements. No simulator "
             "calibration or pool coverage is inferred from them.", ""]
    lines.extend(f"- {w}" for w in report["warnings"])
    lines += ["", "| Start s | End s | Water | CTYPE/CSUB | State/step | Fault | Yaw change ° |",
              "|---|---|---|---|---|---|---|"]
    for phase in report["phases"]:
        lines.append(f"| {phase['start_s']:.3f} | {phase['end_s']:.3f} | {phase['inwater']} | "
                     f"{phase['ctype']}/{phase['csub']} | {phase['state']}/{phase['step']} | "
                     f"{phase['fault']} | {phase['yaw_change_deg']:.1f} |")
    lines += ["", "## Sonar approach candidates", "",
              "| Start s | End s | Mean track command | Closing m/s | R² |",
              "|---|---|---|---|---|"]
    for candidate in report["sonar_candidates"]:
        lines.append(f"| {candidate['start_s']:.3f} | {candidate['end_s']:.3f} | "
                     f"{candidate['mean_track_command']:.1f} | {candidate['sonar_closing_m_s']:.3f} | "
                     f"{candidate['r_squared']:.3f} |")
    lines += ["", "Still unmeasured: " + "; ".join(report["unmeasured"]) + ".", ""]
    return "\n".join(lines)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("recording", type=Path)
    parser.add_argument("--capture", type=int, help="1-based download in bench-event JSONL")
    parser.add_argument("--out", type=Path, default=Path("analysis/floor_report"))
    args = parser.parse_args()
    try:
        samples = load_recording(args.recording, args.capture)
        report = analyze(samples)
        report["source"] = {"path": str(args.recording),
                            "sha256": hashlib.sha256(args.recording.read_bytes()).hexdigest(),
                            "capture": args.capture}
        for name in ("report.json", "report.md"):
            if (args.out / name).resolve() == args.recording.resolve():
                raise ValueError("output would overwrite the input recording")
        args.out.mkdir(parents=True, exist_ok=True)
        (args.out / "report.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
        (args.out / "report.md").write_text(markdown(report), encoding="utf-8")
        print(f"{args.out}: {len(samples)} samples, {report['floor_only_samples']} Floor-only, "
              f"{len(report['sonar_candidates'])} sonar candidates; no automatic calibration")
    except (OSError, ValueError, TypeError) as error:
        parser.exit(1, f"navreport: {error}\n")


if __name__ == "__main__":
    main()
