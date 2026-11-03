#!/usr/bin/env python3
"""navmap - render a visited-area map downloaded from the robot (map layout 3; reads
layout 2 too).

The map is dead-reckoned in "configured-speed distance". Its scale is corrected by the
robot's own sonar speed check (measured range drop / predicted distance on straight runs),
or by a speed you measured yourself. Gyro drift is corrected here, on the PC: the path log is
re-integrated with the drift rate that makes the map most compact, because repeated sweeps of
the same pool only line up at the right rate. The correction is applied only when the run is
long enough to be trusted (simulation: reliable after about 20 minutes).
Layout 3 counts motion only in the water, flat, with the sonar showing progress and not past
the last echo ahead; it reports the drive time it did not count and why. Layout 2 (diag-3.0)
counted all commanded drive.
Blue = visited cells, red = wall contacts, green = start, black = last position. 1 m grid.

    python -I tools/crab_ctl.py --address AA:BB:CC:DD:EE:FF diag-map --out map.bin
    python -I tools/navmap.py map.bin --png map.png [--speed-cm-s 24] [--drift 0 | --drift 1.5]
"""
import argparse
import json
import math
from pathlib import Path
import struct

HEADER_V2 = struct.Struct("<3I3iIii3I2I4I2i4II2i2I3ii3I")    # diag-3.0 map_t header, 36 words
FIELDS_V2 = ("magic", "version", "speed70", "x_um", "y_um", "yaw0", "last_ms", "last_yaw",
             "last_drive", "moving_ms", "overflow", "contacts", "sonar_cm", "expected_cm",
             "seg_t0", "seg_r0", "seg_t1", "seg_r1", "seg_yaw", "seg_drive", "last_cell",
             "cells", "flags", "start_ms", "log_count", "base_x_um", "base_y_um",
             "cur_t_ms", "cur_ms", "cur_yaw", "cur_turn_sum", "cur_drive", "cur_off_ms",
             "reserved1", "reserved2", "reserved3")
MAP_HEADER = struct.Struct("<3I3iIii3I2I4I2i2I4II2i2I3i3iI4iI2iIi4I2iI")   # layout 3, 54 words
MAP_FIELDS = FIELDS_V2[:20] + ("seg_rp", "seg_tp") + FIELDS_V2[20:33] + (
    "cur_drive_sum", "cur_dev_ms", "pend_ms", "pend_x_um", "pend_y_um", "pend_turn_sum",
    "pend_drive_sum", "pend_r0", "pend_d0", "pend_yaw0", "pend_r1", "pend_d1", "stalled_ms",
    "dry_ms", "sloped_ms", "stalls", "ahead_um", "ahead_yaw", "beyond_ms")
PIECE = struct.Struct("<HhHbB")
MAP_ADDR, MAP_BYTES, MAP_DIM, MAP_CELL_M, MAP_SEGMENTS = 0x20006800, 0x1800, 128, 0.25, 485
MAP_MAGIC, MAP_VERSION = 0x3150414D, 3
LAYOUTS = {2: (HEADER_V2, FIELDS_V2, 494), 3: (MAP_HEADER, MAP_FIELDS, MAP_SEGMENTS)}
MIN_CHECK_CM = 200              # below this much checked travel the sonar ratio is not trusted
DRIFT_CANDIDATES = [k / 4 for k in range(-16, 17)]     # deg/min
DRIFT_MIN_MOVING_S = 20 * 60    # shorter runs revisit too little to fix the drift rate
DRIFT_MIN_GAIN = 0.03           # the best rate must shrink the drawn area by 3 %


def parse_map(data):
    """Header dict, visited set of (x, y) cell indices, path pieces (oldest first)."""
    if len(data) != MAP_BYTES:
        raise ValueError(f"map is {len(data)} bytes, expected {MAP_BYTES}")
    magic, version = struct.unpack_from("<2I", data)
    if magic != MAP_MAGIC or version not in LAYOUTS:
        raise ValueError("no map (needs diag-3.0 or later and a cleaning cycle since power-on)")
    layout, fields, segments = LAYOUTS[version]
    header = dict(zip(fields, layout.unpack_from(data)))
    bits = data[layout.size:layout.size + MAP_DIM * MAP_DIM // 8]
    visited = {(i % MAP_DIM, i // MAP_DIM) for i in range(MAP_DIM * MAP_DIM)
               if bits[i >> 3] >> (i & 7) & 1}
    base = layout.size + len(bits)
    count = header["log_count"]
    order = range(count) if count <= segments else [(count + k) % segments for k in range(segments)]
    pieces, wrap, last = [], 0, -1
    for index in order:
        t_ds, yaw, dur_cs, drive, flags = PIECE.unpack_from(data, base + PIECE.size * index)
        if t_ds + wrap < last:                  # 16-bit decisecond clock wrapped (109 min)
            wrap += 65536
        last = t_ds + wrap
        pieces.append({"t_s": last / 10, "yaw": yaw, "dur_s": dur_cs / 100, "drive": drive,
                       "contact": bool(flags & 1), **end_marks(flags & 2, flags & 4)})
    if header["cur_drive"] and header["cur_ms"]:   # the piece still open at download time
        start = (header["cur_t_ms"] - header["start_ms"]) / 1000
        drive = round(header["cur_drive_sum"] / header["cur_ms"]) if version >= 3 else header["cur_drive"]
        pieces.append({"t_s": start, "dur_s": header["cur_ms"] / 1000, "drive": drive,
                       "yaw": header["cur_yaw"] + header["cur_turn_sum"] // header["cur_ms"],
                       "contact": False,
                       **(end_marks(header["flags"] & 32, header["flags"] & 64) if version >= 3 else {})})
    return header, visited, pieces


def end_marks(after_floor, held):
    return {**({"after_floor": True} if after_floor else {}), **({"held": True} if held else {})}


def integrate(header, pieces, drift, step=0.05):
    """Points every `step` m (map units) and contact points, heading corrected by drift."""
    x, y = header["base_x_um"] / 1e6, header["base_y_um"] / 1e6
    points, contacts = [(x, y)], []
    speed = header["speed70"] / 100
    for p in pieces:
        if p.get("after_floor"):        # the map stopped here too (diag-3.4): a short drive to a
            continue                    # wall, then commands to motors held stopped ("held")
        h = math.radians(p["yaw"] / 10 - drift * (p["t_s"] + p["dur_s"] / 2) / 60)
        length = speed * p["drive"] / 70 * p["dur_s"]
        n = max(1, int(abs(length) / step))
        for k in range(n):
            x += length / n * math.cos(h)
            y += length / n * math.sin(h)
            points.append((x, y))
        if p["contact"]:
            contacts.append((x, y))
    return points, contacts


def estimate_drift(header, pieces):
    """Drift rate (deg/min) that draws the path most compactly, and the area gain over 0."""
    def area(d):
        pts, _ = integrate(header, pieces, d, step=0.1)
        return len({(math.floor(px / MAP_CELL_M), math.floor(py / MAP_CELL_M)) for px, py in pts})
    scores = {d: area(d) for d in DRIFT_CANDIDATES}
    best = min(scores, key=lambda d: (scores[d], abs(d)))
    return best, 1 - scores[best] / scores[0.0]


def choose_drift(header, pieces, override=None):
    if override is not None:
        return override, f"set by hand: {override} deg/min"
    if not pieces:
        return 0.0, "no path log: drift not corrected"
    best, gain = estimate_drift(header, pieces)
    if header["moving_ms"] < DRIFT_MIN_MOVING_S * 1000:
        return 0.0, f"run too short for a drift estimate (best {best} deg/min, not applied)"
    if abs(best) >= DRIFT_CANDIDATES[-1]:
        return 0.0, f"drift estimate at the search limit ({best} deg/min), not applied"
    if gain < DRIFT_MIN_GAIN:
        return 0.0, f"no clear drift ({best} deg/min shrinks the map {gain:.0%}), not applied"
    return best, f"estimated {best} deg/min (map {gain:.0%} more compact)"


def corrected_cells(header, pieces, drift):
    """Visited cells (same marking as the firmware) and contacts from the corrected path."""
    points, contacts = integrate(header, pieces, drift)
    half = MAP_DIM / 2
    cells = set()
    for x, y in points:
        i, j = math.floor(x / MAP_CELL_M + half), math.floor(y / MAP_CELL_M + half)
        cells.update({(i, j), (i + 1, j), (i - 1, j), (i, j + 1), (i, j - 1)})
    return cells, [(round(x * 100), round(y * 100)) for x, y in contacts], points[-1]


def summary(header, visited, pieces, speed_cm_s=None, drift=None):
    """Scale, drift and plain statistics; scale converts map units to metres on the ground."""
    configured = header["speed70"]
    if speed_cm_s:
        scale, source = speed_cm_s / configured, f"measured speed {speed_cm_s} cm/s"
    elif header["expected_cm"] >= MIN_CHECK_CM:
        scale = header["sonar_cm"] / header["expected_cm"]
        source = f"sonar speed check over {header['expected_cm'] / 100:.1f} m"
    else:
        scale, source = 1.0, "no usable sonar check: configured speed assumed"
    applied, reason = choose_drift(header, pieces, drift)
    cells, contacts, last = corrected_cells(header, pieces, applied) if pieces else (visited, [], None)
    if last is None:
        last = (header["x_um"] / 1e6, header["y_um"] / 1e6)
    cell = MAP_CELL_M * scale
    info = {"scale": round(scale, 3), "scale_source": source, "configured_speed_cm_s": configured,
            "drift_deg_min": applied, "drift": reason, "pieces": len(pieces),
            "log_wrapped": header["log_count"] > LAYOUTS[header["version"]][2],
            "visited_m2": round(len(cells) * cell * cell, 1), "cells": sorted(cells),
            "contacts_cm": contacts, "contacts": header["contacts"],
            "moving_s": round(header["moving_ms"] / 1000), "outside_grid": header["overflow"],
            "last_position_m": [round(last[0] * scale, 2), round(last[1] * scale, 2)]}
    if header["version"] >= 3:    # drive time the map did not count as travel, by reason
        info["not_counted_s"] = {"no_progress": round(header["stalled_ms"] / 1000),
                                 "past_the_echo": round(header["beyond_ms"] / 1000),
                                 "out_of_water": round(header["dry_ms"] / 1000),
                                 "tilted": round(header["sloped_ms"] / 1000)}
        info["stalls"] = header["stalls"]
    return info


def render(info, path, title=""):
    from PIL import Image, ImageDraw
    scale, half = info["scale"], MAP_DIM / 2
    to_m = lambda i: (i - half) * MAP_CELL_M * scale             # cell index -> metres
    contacts = [(cx / 100 * scale, cy / 100 * scale) for cx, cy in info["contacts_cm"]]
    points = [(to_m(x), to_m(y)) for x, y in info["cells"]] + contacts + [(0, 0)]
    xs, ys = [p[0] for p in points], [p[1] for p in points]
    lo_x, hi_x, lo_y, hi_y = min(xs) - 1, max(xs) + 1.5, min(ys) - 1, max(ys) + 1.5
    px = 40                                                       # pixels per metre
    w, h = int((hi_x - lo_x) * px), int((hi_y - lo_y) * px)
    img = Image.new("RGB", (max(w, 640), h + 60), "white")
    d = ImageDraw.Draw(img)
    at = lambda x, y: (int((x - lo_x) * px), 60 + int((hi_y - y) * px))
    for m in range(int(lo_x), int(hi_x) + 1):
        d.line([at(m, lo_y), at(m, hi_y)], fill="#e3e8ee")
    for m in range(int(lo_y), int(hi_y) + 1):
        d.line([at(lo_x, m), at(hi_x, m)], fill="#e3e8ee")
    size = MAP_CELL_M * scale
    for x, y in info["cells"]:
        x0, y0 = at(to_m(x), to_m(y) + size)
        x1, y1 = at(to_m(x) + size, to_m(y))
        d.rectangle([x0, y0, x1, y1], fill="#2d99c3")
    for x, y in contacts:
        x, y = at(x, y)
        d.ellipse([x - 3, y - 3, x + 3, y + 3], fill="#d64541")
    x, y = at(0, 0)
    d.ellipse([x - 6, y - 6, x + 6, y + 6], outline="#1e8449", width=3)
    x, y = at(*info["last_position_m"])
    d.ellipse([x - 4, y - 4, x + 4, y + 4], fill="black")
    d.text((8, 6), title or "Visited-area map (dead reckoning, grid 1 m)", fill="black")
    d.text((8, 24), f"scale x{info['scale']} ({info['scale_source']}); drift: {info['drift']}",
           fill="#40536a")
    d.text((8, 42), f"visited {info['visited_m2']} m2, {info['contacts']} wall contacts, {info['moving_s']} s "
                    "moving; blue visited, red contact, green start, black last", fill="#40536a")
    img.save(path)


def main():
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("map", type=Path, help="raw map file from crab_ctl diag-map")
    p.add_argument("--png", type=Path)
    p.add_argument("--json", type=Path)
    p.add_argument("--speed-cm-s", type=float, help="your measured speed at drive 70, overrides the sonar check")
    p.add_argument("--drift", type=float, help="drift rate in deg/min to apply instead of the estimate (0 = none)")
    a = p.parse_args()
    try:
        header, visited, pieces = parse_map(a.map.read_bytes())
    except (OSError, ValueError) as error:
        p.error(str(error))
    info = summary(header, visited, pieces, a.speed_cm_s, a.drift)
    print(json.dumps({k: v for k, v in info.items() if k not in ("cells", "contacts_cm")}, indent=2))
    if a.json:
        a.json.write_bytes((json.dumps({"summary": info, "header": header, "pieces": pieces},
                                       indent=1) + "\n").encode("utf-8"))
    if a.png:
        render(info, a.png)


if __name__ == "__main__":
    main()
