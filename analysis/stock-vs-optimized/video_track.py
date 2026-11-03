#!/usr/bin/env python3
"""Run 9's track and camera fit: `tools/videotrack.py` with two things it does not do. Never connects to the robot.

    python -I analysis/stock-vs-optimized/video_track.py fit TRACK.csv CALIB.json OUT.json
    python -I analysis/stock-vs-optimized/video_track.py peaks CALIB.json PEAKS.pkl VIDEO...
    python -I analysis/stock-vs-optimized/video_track.py chain CALIB.json PEAKS.pkl OUT_DIR

The run data and the video are not published, so this is a record of the method.

`fit`: no `before` connection, so no recorder. The moving rhythm and the lane headings come from the
path log instead (cycle minute 29 to the end), and the lens is held at run 8's: same camera, and a free
fit let focal length and lens curve wander without changing the pose much.

`chain`: removes what is known to be wrong before chaining. A white plume across the water while someone
was at the pool edge (video 170 to 240 s, the robot was at the near wall throughout, checked by eye), and
everything once the robot is pinned at the wall after the battery floor, where it fades into the background
and only noise is left (from video 5640 s, it is in the same place at 5640, 5700, 5790, 5900 and 6040 s).
"""
import csv
import importlib.util
import json
import pickle
from collections import deque
from pathlib import Path
import sys

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(ROOT / "tools"))
import navmap as nm  # noqa: E402
spec = importlib.util.spec_from_file_location("videotrack", ROOT / "tools" / "videotrack.py")
v = importlib.util.module_from_spec(spec)
spec.loader.exec_module(v)

SPEED_M_S = 0.205               # map: 30 cm/s x drive 64/70 x sonar check 0.748 over 256.9 m
FOCAL, K1 = 1123.3, -0.4044948723204428     # run 8's calibration
FLOOR_T_S = 5569.6              # cycle time: the path log's change to drive 35 then 77, as at run 8's known trip
PINNED_FROM = 5640.0


def fit(track, calibration, out):
    from scipy.optimize import least_squares
    _, _, log = nm.parse_map((HERE / "map.bin").read_bytes())
    rough = json.loads(Path(calibration).read_text())
    cal = v.Calibration(rough)
    points = v.read_track(track)
    intervals = []
    for a, b in zip(log, log[1:]):
        end = a["t_s"] + a["dur_s"]
        intervals.append((a["t_s"], end, 101, 0, 0, 0))
        if b["t_s"] > end:
            intervals.append((end, b["t_s"], 103, 0, 0, 0))
    offset, best, rival = v.find_offset(points, [i for i in intervals if i[1] <= FLOOR_T_S])
    print("offset", round(offset, 2), "correlation", round(best, 3), "best rival", round(rival, 3))
    pixels, times, pieces = cal.seen(points[:, 1:3]), points[:, 0], []
    for p in log:
        if p["drive"] < 60 or p["dur_s"] < 6 or p["t_s"] > FLOOR_T_S - 10:
            continue
        t0, t1 = p["t_s"] + offset + 1.0, p["t_s"] + p["dur_s"] + offset - 1.0
        knots = [k for k in np.arange(t0, t1 + 1e-6, v.FIT_PIECE_S) if (np.abs(times - k) <= 1.0).sum() >= 2]
        if len(knots) < 2:
            continue
        px = [np.median(pixels[np.abs(times - k) <= 1.0], axis=0) for k in knots]
        for k0, p0, k_next, p1 in zip(knots, px, knots[1:], px[1:]):
            pieces.append((p["t_s"] / 60, k_next - k0, p["yaw"] / 10, *p0, *p1))
    pieces = np.array(pieces)
    centre, corner2 = cal.centre, cal.corner2

    def full(q):
        return (FOCAL, q[0], q[1], q[2], K1, q[3], q[4])

    def residuals(q, sign, use):
        c = full(q)
        part = pieces[use]
        w0, w1 = v.camera_floor(part[:, 3:5], c, centre, corner2), v.camera_floor(part[:, 5:7], c, centre, corner2)
        heading = np.radians(c[5] + sign * (part[:, 2] - c[6] * part[:, 0]))
        return (w1 - w0 - SPEED_M_S * part[:, 1:2] * np.column_stack((np.cos(heading), np.sin(heading)))).ravel()

    low, high, scale = (15, -25, 1.0, -720, -6), (88, 25, 8.0, 720, 6), (10, 3, 1, 20, 0.5)
    every = np.ones(len(pieces), bool)
    best_fit = None
    for sign in (1, -1):
        for pan in range(0, 360, 30):
            for tilt, height in ((50, 2.4), (60, 3.0), (42, 2.0)):
                f = least_squares(residuals, (tilt, 0, height, pan, -2.0), args=(sign, every), loss="soft_l1", f_scale=0.1,
                                  bounds=(low, high), x_scale=scale, max_nfev=200)
                if best_fit is None or f.cost < best_fit[0].cost:
                    best_fit = (f, sign)
    f, sign = best_fit
    use = every
    for _ in range(4):
        size = np.hypot(*residuals(f.x, sign, every).reshape(-1, 2).T)
        use = size < max(0.3, 2 * np.median(size[use]))
        f = least_squares(residuals, f.x, args=(sign, use), loss="soft_l1", f_scale=0.1, bounds=(low, high), x_scale=scale, max_nfev=200)
    size = np.hypot(*residuals(f.x, sign, use).reshape(-1, 2).T)
    c = full(f.x)
    corners = np.array(((100, 300), (1820, 300), (1820, 1000), (100, 1000)), np.float64)
    d = corners - centre
    ideal = d / (1 + K1 * (d * d).sum(axis=1, keepdims=True) / corner2) + centre
    floor = v.camera_floor(corners, c, centre, corner2) * (1, -1)
    to_m, _ = cv2.findHomography(ideal.astype(np.float32), floor.astype(np.float32), 0)
    data = {key: rough[key] for key in ("image_size", "pool_outline_image", "exclude_image", "mask_margin_m", "robot_m", "brush_m") if key in rough}
    data.update(k1=K1, homography_image_to_m=to_m.tolist())
    outline = v.Calibration(dict(data)).outline
    shift = np.array(((1, 0, -outline[:, 0].min()), (0, 1, -outline[:, 1].min()), (0, 0, 1.0)))
    data["homography_image_to_m"] = (shift @ to_m).tolist()
    data["fit"] = {"focal_px": FOCAL, "tilt_deg": round(float(c[1]), 1), "roll_deg": round(float(c[2]), 1), "height_m": round(float(c[3]), 2),
                   "gyro_drift_deg_min": round(float(c[6]), 2), "yaw_sign": sign, "speed_m_s": SPEED_M_S, "offset_s": round(float(offset), 1),
                   "pieces": len(pieces), "pieces_used": int(use.sum()), "median_residual_m": round(float(np.median(size)), 3),
                   "piece_m": round(SPEED_M_S * v.FIT_PIECE_S, 2), "source": "path log, no recorder; lens held at run 8's"}
    Path(out).write_bytes((json.dumps(data, indent=1) + "\n").encode())
    print(data["fit"])


def peaks(calibration, out, videos):
    cal = v.load_calibration(calibration)
    paths = [Path(p) for p in videos]
    start = 34.0            # before that it is being carried in
    backgrounds = v.Backgrounds(cal, paths, start, None)
    mask = cal.pool_mask(cal.mask_margin_m)
    found, recent = [], deque(maxlen=3)
    for t, frame in v.video_frames(paths, 0.5, start, None):
        recent.append((t, cv2.cvtColor(cal.warp(frame), cv2.COLOR_BGR2Lab)))
        if len(recent) < 3:
            continue
        centre = recent[1][0]
        steady = np.median(np.stack([lab for _, lab in recent]), axis=0)
        background, noise = backgrounds.at(centre)
        found.append((centre, v.candidates(steady, background, noise, cal, mask)))
    with open(out, "wb") as f:
        pickle.dump(found, f)
    print("frames", len(found))


def chain(calibration, peaks_path, out):
    with open(peaks_path, "rb") as f:
        found = pickle.load(f)
    kept = []
    for t, candidates in found:
        if t >= PINNED_FROM:
            continue
        if 170 <= t <= 240 and len(candidates):
            candidates = candidates[candidates[:, 1] >= 4.0]       # the plume near the fountain
        if 196 <= t <= 222 and len(candidates):
            candidates = candidates[candidates[:, 0] >= 1.5]       # the plume along the left wall
        kept.append((t, candidates))
    track = v.smooth(v.best_chain(v.drop_steady_spots(kept)))
    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    with open(out / "track.csv", "w", newline="") as f:
        writer = csv.writer(f, lineterminator="\n")
        writer.writerow(("t_s", "x_m", "y_m", "z"))
        for t, x, y, z in track:
            writer.writerow((f"{t:.2f}", f"{x:.3f}", f"{y:.3f}", f"{z:.1f}"))
    print("frames", len(kept), "track", len(track), "share %", round(100 * len(track) / len(kept), 1))


if __name__ == "__main__":
    command = sys.argv[1]
    if command == "fit":
        fit(*sys.argv[2:5])
    elif command == "peaks":
        peaks(sys.argv[2], sys.argv[3], sys.argv[4:])
    else:
        chain(*sys.argv[2:5])
