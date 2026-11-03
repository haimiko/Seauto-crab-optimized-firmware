#!/usr/bin/env python3
"""Run 9: the stock program on camera, and stock against optimized on the same floor. Reads files only.

    python -I analysis/stock-vs-optimized/video_study.py

Needs the run data, which is not published: video/track.csv and map.bin here, run 8's track and both camera
calibrations under data/. So this is a record of the method.
Writes video/study.json, video/coverage.png, video/panels.png, video/same_floor.npz.
"""
import csv
import importlib.util
import json
import math
from pathlib import Path
import sys

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
RUN8 = HERE / "data" / "run8"
sys.path.insert(0, str(ROOT / "tools"))
import navmap as nm  # noqa: E402
spec = importlib.util.spec_from_file_location("videotrack", ROOT / "tools" / "videotrack.py")
v = importlib.util.module_from_spec(spec)
spec.loader.exec_module(v)

OFFSET_S = 57.5                 # video time minus cycle time: path log rhythm fit, correlation 0.68, rival 0.36
FLOOR_T_S = 5569.6              # cycle time: the path log's change to drive 35 then 77, as at run 8's known trip
START_8 = 188.981 + 61.5        # run 8: map start_ms + its offset, video time
FLOOR_8 = 5490.464 + 61.5       # run 8: navigation.since + its offset, video time
FIT_9 = {"tilt": 50.3, "roll": -11.5, "height": 2.12}
FIT_8 = {"tilt": 39.7, "roll": -2.0, "height": 1.64}
FOCAL = 1123.3
MARKS_8 = {"disc": (1325, 320), "junction": (245, 825), "skimmer": (190, 250)}    # the same three things in both views
MARKS_9 = {"disc": (1448, 412), "junction": (450, 960), "skimmer": (305, 230)}    # the disc is on the floor, the others at the waterline
MINUTES = (5, 10, 15, 20, 25, 30, 40, 50, 60, 70, 80, 88)
SIMULATED = {"diag-3.3": {20: 91.5, 30: 95.7}, "stock": {20: 79.3, 30: 91.6}}     # tools/navsim.py


def read_track(path):
    with open(path, newline="") as f:
        return np.array([(float(r["t_s"]), float(r["x_m"]), float(r["y_m"]), float(r["z"])) for r in csv.DictReader(f)])


def nadir(calibration_path, fit):
    """The floor point under the camera, in that calibration's plan."""
    cot = 1 / math.tan(math.radians(fit["tilt"]))
    roll = math.radians(fit["roll"])
    h = np.array(json.loads(Path(calibration_path).read_text())["homography_image_to_m"])
    n = h @ np.array((960 + FOCAL * math.sin(roll) * cot, 540 + FOCAL * math.cos(roll) * cot, 1.0))
    return n[:2] / n[2]


def similarity(c8, c9, n8, n9, hw):
    """Plan 8 to plan 9 from the three marks, with waterline marks pulled back by the water's apparent depth hw."""
    def marks(cal, n, height, table):
        out = []
        for name, xy in table.items():
            f = cal.to_pool([xy])[0].astype(np.float64)
            out.append(f if name == "disc" else n + (f - n) * (height - hw) / height)
        return np.array(out)
    a, b = marks(c8, n8, FIT_8["height"], MARKS_8), marks(c9, n9, FIT_9["height"], MARKS_9)
    ca, cb = a.mean(axis=0), b.mean(axis=0)
    za, zb = (a - ca) @ (1, 1j), (b - cb) @ (1, 1j)
    k = (np.conj(za) @ zb) / (np.conj(za) @ za)
    scale, theta = abs(k), math.atan2(k.imag, k.real)
    rot = np.array(((math.cos(theta), -math.sin(theta)), (math.sin(theta), math.cos(theta))))

    def move(p):
        return scale * (np.asarray(p, np.float64) - ca) @ rot.T + cb
    return move, {"hw_m": hw, "scale": round(float(scale), 3), "rotation_deg": round(math.degrees(theta), 1),
                  "mark_misses_m": np.hypot(*(move(a) - b).T).round(2).tolist()}


def curve(first, floor, start):
    area = int((floor > 0).sum())
    done = first[(floor > 0) & np.isfinite(first)]
    return {m: round(100 * float((done <= start + m * 60).sum()) / area, 1) for m in MINUTES}


def minutes_to(first, floor, start, share):
    done = np.sort(first[(floor > 0) & np.isfinite(first)])
    need = int(math.ceil(share * int((floor > 0).sum())))
    return round((float(done[need - 1]) - start) / 60, 1) if len(done) >= need else None


def same_floor(t8, t9, c8, c9, move, start_8, start_9, end_8, end_9):
    """Both runs painted on plan 9, limited to floor both cameras had in view and at least one run reached."""
    chain9 = [tuple(r) for r in t9[(t9[:, 0] >= start_9) & (t9[:, 0] <= end_9)]]
    moved = t8[(t8[:, 0] >= start_8) & (t8[:, 0] <= end_8)].copy()
    moved[:, 1:3] = move(moved[:, 1:3])
    chain8 = [tuple(r) for r in moved]
    _, pass9, first9, pool = v.coverage(chain9, c9)
    _, pass8, first8, _ = v.coverage(chain8, c9)
    view8 = np.zeros(pool.shape, np.uint8)
    cv2.fillPoly(view8, [((move(c8.outline) - (c9.x0, c9.y0)) / v.CELL_M).astype(np.int32)], 1)
    reached = cv2.morphologyEx((((pass8 > 0) | (pass9 > 0)) & (pool > 0)).astype(np.uint8), cv2.MORPH_CLOSE, np.ones((13, 13), np.uint8))
    floor = (reached > 0) & (view8 > 0) & (pool > 0)
    area = int(floor.sum())
    return {
        "floor_m2": round(area * v.CELL_M ** 2, 1),
        "optimized_percent": curve(first8, floor, start_8), "stock_percent": curve(first9, floor, start_9),
        "optimized_final_percent": round(100 * float((pass8[floor] > 0).mean()), 1), "stock_final_percent": round(100 * float((pass9[floor] > 0).mean()), 1),
        "optimized_min_to": {p: minutes_to(first8, floor, start_8, p / 100) for p in (50, 75, 90, 95)},
        "stock_min_to": {p: minutes_to(first9, floor, start_9, p / 100) for p in (50, 75, 90, 95)},
        "optimized_mean_passes": round(float(pass8[floor & (pass8 > 0)].mean()), 1), "stock_mean_passes": round(float(pass9[floor & (pass9 > 0)].mean()), 1),
    }, floor, pass8, pass9, (first8 - start_8) / 60, (first9 - start_9) / 60


def hidden(times, start, end, at_least=60.0):
    grid = np.arange(start, end, 1.0)
    index = np.searchsorted(times, grid).clip(1, len(times) - 1)
    seen = np.minimum(np.abs(times[index] - grid), np.abs(times[index - 1] - grid)) <= 3.0
    runs, first = [], None
    for t, flag in zip(grid, seen):
        if not flag and first is None:
            first = t
        if flag and first is not None:
            if t - first >= at_least:
                runs.append((round((first - start) / 60, 1), round((t - first) / 60, 1)))
            first = None
    return round(100 * float(seen.mean()), 1), runs


def lanes_from_log(log):
    return [{"video_s": p["t_s"] + OFFSET_S, "end_video_s": p["t_s"] + p["dur_s"] + OFFSET_S, "yaw": p["yaw"] / 10, "seconds": p["dur_s"]}
            for p in log if p["drive"] >= 60 and p["dur_s"] >= 6 and p["t_s"] < FLOOR_T_S]


def lane_spacing(track, lanes):
    times = track[:, 0]
    fitted = []
    for lane in lanes:
        use = (times >= lane["video_s"] + 1.0) & (times <= lane["end_video_s"] - 1.0)
        if use.sum() < 8 or use.sum() < 0.6 * (lane["end_video_s"] - lane["video_s"] - 2) * 2:
            fitted.append(None)
            continue
        points = track[use, 1:3]
        centre = np.median(points, axis=0)
        direction = np.linalg.svd(points - centre)[2][0]
        spread = np.abs((points - centre) - np.outer((points - centre) @ direction, direction))
        fitted.append({"centre": centre, "direction": direction, "yaw": lane["yaw"], "length": float(np.ptp((points - centre) @ direction)),
                       "wobble": float(np.median(np.hypot(*spread.T)))})
    rows = []
    for a, b in zip(fitted, fitted[1:]):
        if not a or not b or a["length"] < 1.0 or b["length"] < 1.0 or a["wobble"] > 0.15 or b["wobble"] > 0.15:
            continue
        if abs(abs((a["yaw"] - b["yaw"] + 180) % 360 - 180) - 180) > 30:
            continue
        normal = np.array((-a["direction"][1], a["direction"][0]))
        rows.append({"metres": round(abs(float((b["centre"] - a["centre"]) @ normal)), 2),
                     "angle_deg": round(math.degrees(math.acos(min(abs(float(a["direction"] @ b["direction"])), 1.0))), 1)})
    return rows


def log_agreement(track, log, pan, drift, speed=0.205):
    agree, miss, absent = 0, 0, 0
    for p in log:
        if p["drive"] < 60 or p["dur_s"] < 6 or p["t_s"] >= FLOOR_T_S:
            continue
        t0, t1 = p["t_s"] + OFFSET_S + 1.0, p["t_s"] + p["dur_s"] + OFFSET_S - 1.0
        part = track[(track[:, 0] >= t0) & (track[:, 0] <= t1)]
        if len(part) < (t1 - t0):
            absent += 1
            continue
        seen = part[-3:, 1:3].mean(axis=0) - part[:3, 1:3].mean(axis=0)
        heading = math.radians(pan + p["yaw"] / 10 - drift * p["t_s"] / 60)
        want = speed * (part[-2, 0] - part[1, 0]) * np.array((math.cos(heading), -math.sin(heading)))
        if np.hypot(*(seen - want)) < 0.45:
            agree += 1
        else:
            miss += 1
    return {"lanes_agree_within_0.45_m": agree, "lanes_disagree": miss, "lanes_with_no_track": absent}


def panels(track, cal, path):
    scale = 45
    low = cal.outline.min(axis=0) - 0.3
    size = ((cal.outline.max(axis=0) - low + 0.3) * scale).astype(int)
    tiles = []
    for m in range(0, 100, 10):
        image = np.full((size[1] + 26, size[0], 3), 255, np.uint8)
        offset = np.array((0, 26))
        cv2.polylines(image, [((cal.outline - low) * scale).astype(np.int32) + offset], True, (0, 150, 200), 1, cv2.LINE_AA)
        part = track[(track[:, 0] >= m * 60 + OFFSET_S) & (track[:, 0] < (m + 10) * 60 + OFFSET_S)]
        for a, b in zip(part, part[1:]):
            if b[0] - a[0] <= 3:
                cv2.line(image, tuple(int(x) for x in (a[1:3] - low) * scale + offset), tuple(int(x) for x in (b[1:3] - low) * scale + offset),
                         (150, 70, 20), 1, cv2.LINE_AA)
        cv2.putText(image, f"minute {m} to {m + 10}", (4, 17), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv2.LINE_AA)
        tiles.append(image)
    while len(tiles) % 5:
        tiles.append(np.full_like(tiles[0], 255))
    cv2.imwrite(str(path), np.vstack([np.hstack(tiles[i:i + 5]) for i in range(0, len(tiles), 5)]))


def log_stats(entries, floor_t):
    part = [p for p in entries if p["t_s"] < floor_t]
    span = part[-1]["t_s"] + part[-1]["dur_s"] - part[0]["t_s"]
    long_pieces = [p for p in part if p["drive"] >= 60 and p["dur_s"] >= 6]
    back = [p for p in part if p["drive"] < 0]
    return {"from_minute": round(part[0]["t_s"] / 60, 1), "span_min": round(span / 60, 1), "moving_percent": round(100 * sum(p["dur_s"] for p in part) / span, 1),
            "backing_percent": round(100 * sum(p["dur_s"] for p in back) / span, 1), "lanes_per_minute": round(len(long_pieces) / (span / 60), 2),
            "median_lane_s": float(np.median([p["dur_s"] for p in long_pieces])), "median_lane_drive": float(np.median([p["drive"] for p in long_pieces]))}


def main():
    calib8, calib9 = HERE / "data/calib_run8.json", HERE / "data/calib_run9.json"
    c8, c9 = v.load_calibration(calib8), v.load_calibration(calib9)
    n8, n9 = nadir(calib8, FIT_8), nadir(calib9, FIT_9)
    t9, t8 = read_track(HERE / "video/track.csv"), read_track(RUN8 / "video/track.csv")
    start, floor_video = OFFSET_S, FLOOR_T_S + OFFSET_S
    _, _, log = nm.parse_map((HERE / "map.bin").read_bytes())
    _, _, log8 = nm.parse_map((RUN8 / "map.bin").read_bytes())

    # the whole view, stock alone
    chain = [tuple(r) for r in t9 if start <= r[0] <= floor_video]
    summary, passes, first, pool = v.coverage(chain, c9)
    reached = cv2.morphologyEx(((pool > 0) & (passes > 0)).astype(np.uint8), cv2.MORPH_CLOSE, np.ones((13, 13), np.uint8)) > 0
    seen_percent, away = hidden(t9[:, 0], start, floor_video)
    seen8, away8 = hidden(t8[:, 0], START_8, FLOOR_8)
    v.render_coverage(HERE / "video/coverage.png", chain, c9, summary, passes, pool, "run 9, stock Floor, cycle start to battery floor")
    panels(t9, c9, HERE / "video/panels.png")

    # new floor per ten minutes, and the floor touched in each ten minutes
    new, area_each = {}, {}
    for m in range(0, 90, 10):
        a, b = start + m * 60, start + (m + 10) * 60
        new[m] = round(float(((first >= a) & (first < b) & (pool > 0)).sum()) * v.CELL_M ** 2, 1)
        part = [tuple(r) for r in t9 if a <= r[0] < b]
        area_each[m] = round(float((v.coverage(part, c9)[1] > 0).sum()) * v.CELL_M ** 2, 1) if len(part) > 1 else 0.0

    # the same floor, for several assumed water depths: each gives a slightly different match of the two plans
    results, kept = {}, None
    for hw in (0.7, 0.8, 0.9, 1.0):
        move, info = similarity(c8, c9, n8, n9, hw)
        result, *rest = same_floor(t8, t9, c8, c9, move, START_8, start, FLOOR_8, floor_video)
        results[str(hw)] = {**info, **result}
        if hw == 0.9:
            kept = rest

    lanes = lanes_from_log(log)
    spacing = lane_spacing(t9, lanes)
    gaps = [r["metres"] for r in spacing]
    fit = json.loads(calib9.read_text())["fit"]
    result = {
        "minutes_to_floor": round(FLOOR_T_S / 60, 1), "seen_percent_to_floor": seen_percent, "absences_over_a_minute_from_minute_and_length": away,
        "absent_minutes": round(sum(x[1] for x in away), 1), "run8_seen_percent_to_floor": seen8, "run8_absent_minutes": round(sum(x[1] for x in away8), 1),
        "outline_m2": summary["pool_m2"], "reached_floor_m2": round(int(reached.sum()) * v.CELL_M ** 2, 1),
        "percent_of_outline": curve(first, pool, start), "percent_of_reached": curve(first, reached, start),
        "holes_in_reached_m2": round(int((reached & (passes == 0)).sum()) * v.CELL_M ** 2, 2),
        "mean_passes": summary["mean_passes"], "new_floor_m2_per_10_min": new, "floor_touched_m2_per_10_min": area_each,
        "same_floor_by_assumed_depth": results, "simulated_whole_pool_percent": SIMULATED,
        "log_stock": log_stats(log, FLOOR_T_S), "log_optimized": log_stats(log8, FLOOR_8 - 61.5 - 188.981),
        "log_agreement": log_agreement(t9, log, 122.1, -4.52),
        "lane_spacing": {"pairs": len(gaps), "percentiles_10_25_50_75_90": np.percentile(gaps, [10, 25, 50, 75, 90]).round(2).tolist() if gaps else [],
                         "median_angle_deg": float(np.median([r["angle_deg"] for r in spacing])) if spacing else None},
        "camera_fit": fit,
    }
    (HERE / "video/study.json").write_bytes((json.dumps(result, indent=1) + "\n").encode())
    print(json.dumps(result, indent=1))
    floor, pass8, pass9, first8, first9 = kept
    np.savez_compressed(HERE / "video/same_floor.npz", floor=floor, pass8=pass8.astype(np.uint16), pass9=pass9.astype(np.uint16),
                        first8=first8.astype(np.float32), first9=first9.astype(np.float32), x0=c9.x0, y0=c9.y0)


if __name__ == "__main__":
    main()
