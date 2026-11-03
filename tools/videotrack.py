#!/usr/bin/env python3
"""Follow the robot through a fixed overhead video of the pool: the ground truth its own map lacks.

One camera, not moved during the run, with the floor in view. Nothing here touches the robot.

    frame      save one frame with a pixel grid, to read the calibration points from
    track      follow the robot; writes track.csv, coverage.png, coverage.json, overlay.jpg, checks.jpg
    compare    line track.csv up with a watertest session; writes compare.md, lanes.csv, shifts.csv
    calibrate  fit the camera from the robot's own lanes; no tape measure, needs a first rough track

Method. The image is warped to a top-down view with a homography from four or more known points.
A background (the median of frames spread over several minutes, so the moving robot vanishes from
it and slow changes of light stay in it) is subtracted, and a robot-sized centre-minus-surround
filter scores what is left: ripples and caustics are much smaller than the robot and average out,
a passing cloud is much larger and cancels. Each spot is judged against what that spot usually
scores, so rippling water, a swaying frond or a sun patch needs far more than calm water does. A
median over three samples first removes what lasts one sample (sun glints), which the robot,
moving 11 cm per sample, survives. Each sample gives a few candidate peaks, and the track is the
chain of candidates with the best total score at a speed the robot can reach, with no score above
the robot's own: whatever shows up once loses to what is there all the time. A robot out of sight
leaves a gap. No training or labelled frames are needed.

Limits. Read checks.jpg and overlay.jpg before believing a number. Water with the sky reflected in
it hides the robot, which leaves a gap and not a guess. Positions are for a flat floor; a robot
climbing a wall is drawn past the outline and clipped. A lane through ripples can be missed.
"""
import argparse
from collections import deque
import csv
import json
import math
from pathlib import Path

import cv2
import numpy as np

CHANNEL_WEIGHT = np.array((1.0, 2.0, 2.0), np.float32)  # Lab: chroma is steadier than lightness
BACKGROUND_WINDOW_S = 300.0     # each side: longer than the robot stays anywhere while cleaning
BACKGROUND_EVERY_S = 15.0
CANDIDATES = 12                 # peaks kept per frame
MIN_Z = 3.0                     # peak height against what the same spot usually scores
CHAIN_PASSES = 3                # each pass caps a peak's score at the z of the previous pass's chain
KEEP_SHARE = 0.25               # after the first pass, peaks under this share of the chain's z do not count
SATURATED_L = 245               # background this bright (0..255) is sky on the water: nothing shows through
FRAGMENT_S = 10.0               # a stretch of track shorter than this, and weak, is not the robot
SPOT_CELL_M = 0.4               # a robot does not sit in one such cell for most of SPOT_WINDOW_S while cleaning;
SPOT_WINDOW_S = 120.0           # a swaying frond or a flickering sun patch does
SPOT_SHARE = 0.6
WARN_TRACKED_PERCENT = 80        # below this, look at why: the robot out of view, or too faint to follow
LOOK_BACK = 40                  # frames with candidates a link may skip
EXCESS_COST = 2.0               # per metre beyond what the robot can reach in the time
SKIP_COST = 0.05                # per skipped frame
GAP_S = 6.0                     # no swath is drawn across a longer gap: the robot's way through it is unknown
CELL_M = 0.05
PASS_CHUNK_S = 10.0             # one swath chunk counts as one pass over a cell
EXPECTED_MOTION = {101: 1.0, 102: 1.0, 103: 0.0, 104: 1.0, 105: 0.0, 106: 1.0, 107: 0.0, 108: 0.0, 109: 0.0}
SHIFT_M = 0.30                  # nav spacing
STUCK_ECHO_CM = 150             # the firmware's shift check applies below this range
FIT_SPEED_M_S = 0.22            # lane speed at drive 70 (a measured speed check): sets the camera fit's scale
FIT_PIECE_S = 4.0


def video_frames(paths, every_s, start=0.0, end=None):
    """Yield (t_s, frame), decoding in order. Several files play as one (cameras split recordings)."""
    base, next_t = 0.0, start
    for path in paths:
        cap = cv2.VideoCapture(str(path))
        if not cap.isOpened():
            raise SystemExit(f"cannot open {path}")
        rate, index, local = cap.get(cv2.CAP_PROP_FPS) or 25.0, 0, 0.0
        while cap.grab():
            ms = cap.get(cv2.CAP_PROP_POS_MSEC)
            local = ms / 1000 if ms > 0 else index / rate
            index += 1
            t = base + local
            if end is not None and t > end:
                cap.release()
                return
            if t + 1e-6 < next_t:
                continue
            ok, frame = cap.retrieve()
            if ok:
                yield t, frame
            while next_t <= t + 1e-6:
                next_t += every_s
        base += local + 1 / rate
        cap.release()


def video_duration(paths):
    total = []
    for path in paths:
        cap = cv2.VideoCapture(str(path))
        if not cap.isOpened():
            raise SystemExit(f"cannot open {path}")
        total.append(cap.get(cv2.CAP_PROP_FRAME_COUNT) / (cap.get(cv2.CAP_PROP_FPS) or 25.0))
        cap.release()
    return total


def frames_at(paths, times):
    """Seek to each time. Close enough for a background or a picture, never used for a position."""
    durations, out = video_duration(paths), []
    for t in times:
        local = t
        for path, duration in zip(paths, durations):
            if local < duration or path == paths[-1]:
                cap = cv2.VideoCapture(str(path))
                cap.set(cv2.CAP_PROP_POS_MSEC, max(local, 0.0) * 1000)
                ok, frame = cap.read()
                cap.release()
                if ok:
                    out.append((t, frame))
                break
            local -= duration
    return out


class Calibration:
    """Image pixels to pool metres, and metres to the pixels of the top-down canvas."""

    def __init__(self, data, ppm=40):
        width, height = data.get("image_size", (1920, 1080))
        self.k1 = float(data.get("k1", 0.0))        # lens: ideal = seen / (1 + k1 r^2), r = 1 at the frame corner
        self.centre = np.array((width / 2, height / 2), np.float64)
        self.corner2 = float(self.centre @ self.centre)
        if "homography_image_to_m" in data:
            self.to_m = np.array(data["homography_image_to_m"], np.float64)
            pool = None
        else:
            pool = np.array(data["pool_points_m"], np.float32)
            image = self.ideal(data["image_points"]).astype(np.float32)
            if len(image) < 4 or len(image) != len(pool):
                raise SystemExit("calibration needs the same number (four or more) of image and pool points")
            self.to_m, _ = cv2.findHomography(image, pool, 0)
            if self.to_m is None:
                raise SystemExit("calibration points are degenerate")
        if "pool_outline_image" in data:
            self.outline = self.to_pool(data["pool_outline_image"])
        elif "pool_outline_m" in data:
            self.outline = np.array(data["pool_outline_m"], np.float32)
        elif pool is not None:
            self.outline = cv2.convexHull(pool).reshape(-1, 2)
        else:
            raise SystemExit("a calibration given as a homography also needs pool_outline_image")
        self.exclude = [self.to_pool(polygon) for polygon in data.get("exclude_image", [])]    # fronds and the like
        self.mask_margin_m = float(data.get("mask_margin_m", 0.3))
        self.robot_m = float(data.get("robot_m", 0.40))     # tools/navsim.py WIDTH: an assumption, not measured
        self.brush_m = float(data.get("brush_m", 0.40))     # the same, so coverage compares with the simulator
        self.ppm = ppm
        self.margin_m = 0.5
        self.x0, self.y0 = self.outline.min(axis=0) - self.margin_m
        x1, y1 = self.outline.max(axis=0) + self.margin_m
        self.size = int(math.ceil((x1 - self.x0) * ppm)), int(math.ceil((y1 - self.y0) * ppm))
        xs, ys = np.meshgrid(np.arange(self.size[0]) / ppm + self.x0, np.arange(self.size[1]) / ppm + self.y0)
        seen = self.seen(np.column_stack((xs.ravel(), ys.ravel())))
        self.map_x = seen[:, 0].reshape(xs.shape).astype(np.float32)
        self.map_y = seen[:, 1].reshape(xs.shape).astype(np.float32)

    def ideal(self, pixels):
        """Pixels as the camera saw them to pixels without the lens curve."""
        offset = np.array(pixels, np.float64).reshape(-1, 2) - self.centre
        return offset / (1 + self.k1 * (offset * offset).sum(axis=1, keepdims=True) / self.corner2) + self.centre

    def seen(self, xy_m):
        """Pool metres to pixels as the camera saw them."""
        points = np.asarray(xy_m, np.float64).reshape(-1, 1, 2)
        straight = cv2.perspectiveTransform(points, np.linalg.inv(self.to_m)).reshape(-1, 2) - self.centre
        if self.k1 == 0:
            return straight + self.centre
        r2 = (straight * straight).sum(axis=1, keepdims=True) / self.corner2
        root = np.sqrt(1 - 4 * self.k1 * r2)
        return 2 * straight / (1 + root) + self.centre           # the exact inverse of ideal()

    def to_pool(self, pixels):
        points = self.ideal(pixels).reshape(-1, 1, 2)
        return cv2.perspectiveTransform(points, self.to_m).reshape(-1, 2).astype(np.float32)

    def warp(self, frame):
        return cv2.remap(frame, self.map_x, self.map_y, cv2.INTER_LINEAR)

    def canvas_px(self, xy_m):
        return ((np.asarray(xy_m, np.float64) - (self.x0, self.y0)) * self.ppm).astype(np.int32)

    def image_px(self, xy_m):
        return self.seen(xy_m).astype(np.int32)

    def pool_mask(self, margin_m=0.0):
        mask = np.zeros(self.size[::-1], np.uint8)
        cv2.fillPoly(mask, [self.canvas_px(self.outline)], 255)
        if margin_m > 0:
            k = 2 * int(margin_m * self.ppm) + 1
            mask = cv2.dilate(mask, np.ones((k, k), np.uint8))
        for polygon in self.exclude:
            cv2.fillPoly(mask, [self.canvas_px(polygon)], 0)
        return mask


def load_calibration(path, ppm=40):
    return Calibration(json.loads(Path(path).read_text()), ppm)


def robot_score(diff, cal):
    """Robot-sized centre minus surround: ripples are smaller and average out, a cloud is larger and cancels."""
    inner = max(3, int(cal.robot_m * 0.7 * cal.ppm) | 1)
    outer = inner * 3 | 1
    surround = cv2.blur(diff, (inner, inner)) - cv2.blur(diff, (outer, outer))
    return np.sqrt(((surround * surround) * CHANNEL_WEIGHT).sum(axis=2))


class Backgrounds:
    """Median of warped frames around a time and what each spot usually scores, recomputed every half minute."""

    def __init__(self, cal, paths, start, end):
        self.cal = cal
        total = sum(video_duration(paths))
        stop = min(end, total) if end else total
        every = min(BACKGROUND_EVERY_S, max((stop - start) / 30, 1.0))
        self.times, self.bank = [], []
        for t, frame in frames_at(paths, np.arange(start, stop, every)):
            self.times.append(t)
            self.bank.append(cv2.cvtColor(cal.warp(frame), cv2.COLOR_BGR2Lab))
        if len(self.bank) < 3:
            raise SystemExit("too little video to build a background")
        self.times = np.array(self.times)
        self.block, self.current = None, None

    def at(self, t):
        block = int(t // 30)
        if block != self.block:
            near = np.flatnonzero(np.abs(self.times - t) <= BACKGROUND_WINDOW_S)
            if len(near) < 3:
                near = np.argsort(np.abs(self.times - t))[:3]
            self.current = np.median(np.stack([self.bank[i] for i in near]), axis=0).astype(np.float32)
            scores = [robot_score(self.bank[i].astype(np.float32) - self.current, self.cal) for i in near]
            self.noise = np.median(np.stack(scores), axis=0)    # the robot passes, ripples and fronds stay
            self.block = block
        return self.current, self.noise


def candidates(frame_lab, background, noise, cal, mask):
    """Peaks of the robot-sized filter as (x_m, y_m, z), the strongest first; z is against the spot's own noise."""
    inner = max(3, int(cal.robot_m * 0.7 * cal.ppm) | 1)
    usable = (mask > 0) & (background[..., 0] < SATURATED_L)
    if not usable.any():
        return np.zeros((0, 3))
    diff = frame_lab.astype(np.float32) - background
    diff[~usable] = 0                                   # sun on the rocks must not leak in through the filter
    usual = cv2.blur(noise, (inner, inner))
    floor = max(float(np.median(usual[usable])) * 0.7, 0.5)     # a quiet spot is not a certain one
    z = robot_score(diff, cal) / np.maximum(usual, floor)
    z[~usable] = 0
    peak = z == cv2.dilate(z, np.ones((inner, inner), np.uint8))
    ys, xs = np.nonzero(peak & (z >= MIN_Z))
    order = np.argsort(z[ys, xs])[::-1][:CANDIDATES]
    ys, xs = ys[order], xs[order]
    return np.column_stack((xs / cal.ppm + cal.x0, ys / cal.ppm + cal.y0, z[ys, xs]))


def drop_steady_spots(found):
    """Remove peaks in cells that fire through most of a two-minute window."""
    cleaned, start = [], found[0][0] if found else 0.0
    while start <= (found[-1][0] if found else -1.0):
        part = [(t, c) for t, c in found if start <= t < start + SPOT_WINDOW_S]
        counts = {}
        for _, c in part:
            for cell in {(int(x // SPOT_CELL_M), int(y // SPOT_CELL_M)) for x, y, _ in c}:
                counts[cell] = counts.get(cell, 0) + 1
        steady = {cell for cell, n in counts.items() if n > SPOT_SHARE * len(part)}
        for t, c in part:
            if steady and len(c):
                c = c[[(int(x // SPOT_CELL_M), int(y // SPOT_CELL_M)) not in steady for x, y, _ in c]]
            cleaned.append((t, c))
        start += SPOT_WINDOW_S
    return cleaned


def best_chain(found, max_speed=0.35, slack_m=0.15):
    """found: [(t, candidates)]. Returns [(t, x, y, z)], the best-scoring chain the robot could drive."""
    found = [(t, c) for t, c in found if len(c)]
    if not found:
        return []
    level = float(np.median([c[:, 2].max() for _, c in found]))
    chain = link_chain(found, level, max_speed, slack_m)
    for _ in range(CHAIN_PASSES - 1):
        scores = [c[3] for c in chain]                              # the robot in plain view, not the hidden stretches;
        level = min(float(np.percentile(scores, 75)), 2 * float(np.median(scores)))     # a few glints must not set it
        strong = [(t, c[c[:, 2] >= KEEP_SHARE * level]) for t, c in found]      # a hidden robot leaves a gap,
        strong = [(t, c) for t, c in strong if len(c)]                          # not a path through the noise
        chain = link_chain(strong, level, max_speed, slack_m)
    return drop_fragments(chain, level)


def drop_fragments(chain, level):
    """The robot in view stays in view for a while; a few peaks strung together in the noise do not."""
    kept, part = [], []

    def close():
        if part and (part[-1][0] - part[0][0] >= FRAGMENT_S or np.median([c[3] for c in part]) >= level):
            kept.extend(part)

    for c in chain:
        if part and c[0] - part[-1][0] > 3.0:
            close()
            part = []
        part.append(c)
    close()
    return kept


def link_chain(found, level, max_speed, slack_m):
    totals, links = [], []
    for i, (t, nodes) in enumerate(found):
        gain = np.minimum(nodes[:, 2] / level, 1.0)
        total, link, origin = gain.copy(), np.full(len(nodes), -1), np.full(len(nodes), -1)
        for j in range(max(0, i - LOOK_BACK), i):
            before_t, before = found[j]
            distance = np.hypot(nodes[:, 0, None] - before[:, 0], nodes[:, 1, None] - before[:, 1])
            excess = np.maximum(distance - max_speed * (t - before_t) - slack_m, 0)
            reach = totals[j] - EXCESS_COST * excess - SKIP_COST * (i - j - 1)
            best = reach.argmax(axis=1)
            value = gain + reach[np.arange(len(nodes)), best]
            better = value > total
            total[better] = value[better]
            link[better] = best[better]
            origin[better] = j
        totals.append(total)
        links.append((origin, link))
    i = max(range(len(found)), key=lambda k: totals[k].max())
    node, chain = int(totals[i].argmax()), []
    while i >= 0:
        t, nodes = found[i]
        chain.append((t, float(nodes[node, 0]), float(nodes[node, 1]), float(nodes[node, 2])))
        origin, link = links[i]
        i, node = int(origin[node]), int(link[node])
    return chain[::-1]


def smooth(chain):
    """Three-point median: takes out single-frame jitter, keeps the turns."""
    if len(chain) < 3:
        return chain
    a = np.array(chain)
    for column in (1, 2):
        a[1:-1, column] = np.median(np.stack((a[:-2, column], a[1:-1, column], a[2:, column])), axis=0)
    return [tuple(float(v) for v in row) for row in a]


def coverage(chain, cal, cell_m=CELL_M):
    """Swath painted along the track on a fine grid: passes per cell, first time per cell, the curve."""
    ppm = 1 / cell_m
    origin = (cal.x0, cal.y0)
    shape = (int(cal.size[1] / cal.ppm * ppm) + 1, int(cal.size[0] / cal.ppm * ppm) + 1)
    pool = np.zeros(shape, np.uint8)
    cv2.fillPoly(pool, [((cal.outline - origin) * ppm).astype(np.int32)], 1)
    passes, first = np.zeros(shape, np.uint16), np.full(shape, np.inf)
    thick = max(1, int(round(cal.brush_m * ppm)))

    def paint(points, when):
        if len(points) >= 2:
            mask = np.zeros(shape, np.uint8)
            cv2.polylines(mask, [((np.array(points) - origin) * ppm).astype(np.int32)], False, 1, thick)
            mask &= pool
            passes[mask > 0] += 1
            first[(mask > 0) & np.isinf(first)] = when

    chunk, chunk_t, previous = [], None, None
    for t, x, y, _ in chain:
        if previous is not None and t - previous > GAP_S:
            paint(chunk, chunk_t)
            chunk = []
        if not chunk:
            chunk_t = t
        chunk.append((x, y))
        if t - chunk_t >= PASS_CHUNK_S:
            paint(chunk, chunk_t)
            chunk, chunk_t = [(x, y)], t
        previous = t
    paint(chunk, chunk_t)
    area = int(pool.sum())
    start = chain[0][0] if chain else 0.0
    covered = np.sort(first[(pool > 0) & np.isfinite(first)])

    def minutes_to(share):
        need = int(math.ceil(share * area))
        return round((float(covered[need - 1]) - start) / 60, 1) if area and len(covered) >= need else None

    painted = passes[(pool > 0) & (passes > 0)]
    summary = {
        "pool_m2": round(area * cell_m * cell_m, 1),
        "covered_m2": round(len(covered) * cell_m * cell_m, 1),
        "covered_percent": round(100 * len(covered) / area, 1) if area else 0.0,
        "minutes_to": {f"{p}%": minutes_to(p / 100) for p in (50, 75, 90, 95)},
        "mean_passes": round(float(painted.mean()), 2) if len(painted) else 0.0,
    }
    return summary, passes, first, pool


def render_coverage(path, chain, cal, summary, passes, pool, title):
    """Passes per cell: red never, then lighter to darker blue; the black line is the track."""
    colours = ((235, 225, 200), (225, 190, 120), (200, 140, 60), (150, 90, 30), (90, 40, 10))
    image = np.full(passes.shape + (3,), 255, np.uint8)
    image[pool > 0] = (170, 170, 245)
    for i, (low, high) in enumerate(((1, 1), (2, 2), (3, 4), (5, 8), (9, 65535))):
        image[(passes >= low) & (passes <= high) & (pool > 0)] = colours[i]
    scale = 4
    image = cv2.resize(image, None, fx=scale, fy=scale, interpolation=cv2.INTER_NEAREST)
    ppm = scale / CELL_M
    if chain:
        points = ((np.array([(x, y) for _, x, y, _ in chain]) - (cal.x0, cal.y0)) * ppm).astype(np.int32)
        cv2.polylines(image, [points], False, (0, 0, 0), 1, cv2.LINE_AA)
        cv2.circle(image, tuple(int(v) for v in points[0]), 7, (0, 160, 0), -1)
        cv2.circle(image, tuple(int(v) for v in points[-1]), 7, (0, 0, 0), -1)
    band = np.full((64, image.shape[1], 3), 255, np.uint8)
    reach = ", ".join(f"{k} in {v} min" for k, v in summary["minutes_to"].items() if v is not None)
    cv2.putText(band, title, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 1, cv2.LINE_AA)
    text = (f"covered {summary['covered_percent']} % of {summary['pool_m2']} m2, "
            f"mean {summary['mean_passes']} passes; {reach or 'no share reached'}")
    cv2.putText(band, text, (8, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv2.LINE_AA)
    cv2.imwrite(str(path), np.vstack((band, image)))


def render_checks(out, paths, chain, cal):
    """What a person checks first: the track on the camera's own picture, and the robot in close-up."""
    if not chain:
        return
    times = [chain[int(k)][0] for k in np.linspace(0, len(chain) - 1, 12)]
    grabbed = frames_at(paths, times)
    if not grabbed:
        return
    overlay = grabbed[len(grabbed) // 2][1].copy()
    cv2.polylines(overlay, [cal.image_px(cal.outline)], True, (0, 255, 255), 2, cv2.LINE_AA)
    cv2.polylines(overlay, [cal.image_px([(x, y) for _, x, y, _ in chain])], False, (0, 0, 255), 1, cv2.LINE_AA)
    cv2.imwrite(str(out / "overlay.jpg"), overlay)
    stamps = np.array([c[0] for c in chain])
    tiles, half = [], int(1.5 * cal.ppm)
    for t, frame in grabbed:
        _, x, y, _ = chain[int(np.abs(stamps - t).argmin())]
        view = cal.warp(frame)
        cx, cy = (int(v) for v in cal.canvas_px((x, y)))
        cv2.circle(view, (cx, cy), int(cal.robot_m * cal.ppm * 0.8), (0, 0, 255), 1, cv2.LINE_AA)
        padded = cv2.copyMakeBorder(view, half, half, half, half, cv2.BORDER_CONSTANT)
        tile = cv2.resize(padded[cy:cy + 2 * half, cx:cx + 2 * half], (240, 240), interpolation=cv2.INTER_NEAREST)
        cv2.putText(tile, f"{t / 60:.1f} min", (6, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
        tiles.append(tile)
    while len(tiles) % 4:
        tiles.append(np.zeros_like(tiles[0]))
    cv2.imwrite(str(out / "checks.jpg"), np.vstack([np.hstack(tiles[i:i + 4]) for i in range(0, len(tiles), 4)]))


def track(paths, cal, out, every_s=0.5, start=0.0, end=None, max_speed=0.35, title="", log=print):
    out.mkdir(parents=True, exist_ok=True)
    backgrounds = Backgrounds(cal, paths, start, end)
    mask = cal.pool_mask(cal.mask_margin_m)
    found, reported, recent = [], start, deque(maxlen=3)
    for t, frame in video_frames(paths, every_s, start, end):
        recent.append((t, cv2.cvtColor(cal.warp(frame), cv2.COLOR_BGR2Lab)))
        if len(recent) < 3:
            continue
        centre = recent[1][0]
        steady = np.median(np.stack([lab for _, lab in recent]), axis=0)
        background, noise = backgrounds.at(centre)
        found.append((centre, candidates(steady, background, noise, cal, mask)))
        if t - reported >= 300:
            log(f"  {t / 60:.0f} min of video read")
            reported = t
    chain = smooth(best_chain(drop_steady_spots(found), max_speed))
    with open(out / "track.csv", "w", newline="") as f:
        writer = csv.writer(f, lineterminator="\n")
        writer.writerow(("t_s", "x_m", "y_m", "z"))
        for t, x, y, z in chain:
            writer.writerow((f"{t:.2f}", f"{x:.3f}", f"{y:.3f}", f"{z:.1f}"))
    summary, passes, _, pool = coverage(chain, cal)
    gaps = [b[0] - a[0] for a, b in zip(chain, chain[1:]) if b[0] - a[0] > GAP_S]
    path_m = sum(math.hypot(b[1] - a[1], b[2] - a[2]) for a, b in zip(chain, chain[1:])
                 if b[0] - a[0] <= GAP_S)
    summary.update(
        frames=len(found), tracked=len(chain),
        tracked_percent=round(100 * len(chain) / len(found), 1) if found else 0.0,
        first_s=round(chain[0][0], 1) if chain else None, last_s=round(chain[-1][0], 1) if chain else None,
        path_m=round(path_m, 1), gaps_over_limit=len(gaps), longest_gap_s=round(max(gaps), 1) if gaps else 0.0,
    )
    (out / "coverage.json").write_bytes((json.dumps(summary, indent=2) + "\n").encode())
    render_coverage(out / "coverage.png", chain, cal, summary, passes, pool, title or out.name)
    render_checks(out, paths, chain, cal)
    return summary, chain


def read_track(path):
    with open(path, newline="") as f:
        rows = [(float(r["t_s"]), float(r["x_m"]), float(r["y_m"])) for r in csv.DictReader(f)]
    if len(rows) < 10:
        raise SystemExit(f"{path}: too few track points")
    return np.array(rows)


def recorder_intervals(path):
    """(start_s, end_s, step, yaw_deg, echo_first, echo_next) from a recorder.csv, robot uptime."""
    with open(path, newline="") as f:
        rows = [{k: int(v) for k, v in r.items()} for r in csv.DictReader(f)]
    out = []
    for a, b in zip(rows, rows[1:]):
        if a["step"] >= 100 and a["inwater"]:
            out.append((a["t_ms"] / 1000, b["t_ms"] / 1000, a["step"], a["yaw"] / 10, a["sonar_cm"], b["sonar_cm"]))
    return out


def speed_signal(points, step):
    """Track speed over two seconds on a regular grid; not valid where the track has a gap."""
    t = points[:, 0]
    grid = np.arange(t[0], t[-1], step)
    x, y = np.interp(grid, t, points[:, 1]), np.interp(grid, t, points[:, 2])
    reach = max(1, int(round(1.0 / step)))
    speed = np.zeros(len(grid))
    speed[reach:-reach] = np.hypot(x[2 * reach:] - x[:-2 * reach], y[2 * reach:] - y[:-2 * reach]) / (2 * reach * step)
    index = np.searchsorted(t, grid).clip(1, len(t) - 1)
    return grid, speed, (t[index] - t[index - 1]) <= 5.0


def find_offset(points, intervals, step=0.5):
    """Video time minus robot time: where the recorder's moving/turning rhythm best fits the speed."""
    grid, speed, valid = speed_signal(points, step)
    start, end = intervals[0][0], intervals[-1][1]
    expected = np.zeros(int((end - start) / step) + 1)
    known = np.zeros(len(expected), bool)
    for a, b, phase, *_ in intervals:
        i, j = int((a - start) / step), int((b - start) / step)
        expected[i:j + 1] = EXPECTED_MOTION.get(phase, 0.0)
        known[i:j + 1] = phase in EXPECTED_MOTION
    need = max(int(0.5 * len(expected)), min(len(expected), int(60 / step)))
    results = []
    for shift in range(need - len(expected), len(grid) - need + 1):
        i0, i1 = max(shift, 0), min(shift + len(expected), len(grid))
        use = valid[i0:i1] & known[i0 - shift:i1 - shift]
        if use.sum() < need / 2:
            continue
        a, b = speed[i0:i1][use], expected[i0 - shift:i1 - shift][use]
        if a.std() == 0 or b.std() == 0:
            continue
        results.append((float(np.corrcoef(a, b)[0, 1]), grid[0] + shift * step - start))
    if not results:
        raise SystemExit("the video and the recorder do not overlap enough to line them up")
    results.sort(reverse=True)
    best, offset = results[0]
    rival = max((r for r, o in results if abs(o - offset) > 10), default=0.0)
    return offset, best, rival


def fit_heading(lanes):
    """Camera heading against gyro yaw: mirror, fixed offset and the gyro's drift in deg/min."""
    t = np.array([lane["t_s"] for lane in lanes]) / 60
    camera = np.radians([lane["camera_deg"] for lane in lanes])
    yaw = np.radians([lane["yaw_deg"] for lane in lanes])
    best = (-1.0, 1, 0.0, 0.0)
    for mirror in (1, -1):
        for drift in np.arange(-6, 6.001, 0.05):
            mean = np.exp(1j * (camera - mirror * (yaw - np.radians(drift) * (t - t[0])))).mean()
            if abs(mean) > best[0]:
                best = (float(abs(mean)), mirror, float(drift), math.degrees(np.angle(mean)))
    return dict(zip(("agreement", "mirror", "drift_deg_min", "offset_deg"), best))


def compare(track_path, session, out, offset=None, configured_cm_s=None):
    points = read_track(track_path)
    session = Path(session)
    intervals = recorder_intervals(session / "recorder.csv")
    if not intervals:
        raise SystemExit("this recorder has no controller phases, so it has no rhythm to line up by")
    correlation = rival = None
    if offset is None:
        offset, correlation, rival = find_offset(points, intervals)

    def at(t):
        return np.array((np.interp(t, points[:, 0], points[:, 1]), np.interp(t, points[:, 0], points[:, 2])))

    def inside(t):
        i = int(np.searchsorted(points[:, 0], t).clip(1, len(points) - 1))
        return points[0, 0] <= t <= points[-1, 0] and points[i, 0] - points[i - 1, 0] <= 5.0

    lanes, shifts = [], []
    for a, b, phase, yaw, echo, echo_next in intervals:
        v0, v1 = a + offset, b + offset
        if phase == 101 and b - a >= 3 and inside(v0 + 1) and inside(v1 - 1):
            delta = at(v1 - 1) - at(v0 + 1)
            distance = float(np.hypot(*delta))
            lanes.append(dict(t_s=round(v0, 1), seconds=round(b - a, 1), yaw_deg=yaw, metres=round(distance, 2),
                              cm_s=round(100 * distance / max(b - a - 2, 1), 1),
                              camera_deg=round(math.degrees(math.atan2(delta[1], delta[0])), 1)))
        elif phase == 104 and inside(v0 - 1) and inside(v1 + 1):
            # From mid-turn to mid-turn, where it stands still: a 1.5 s move is shorter than the
            # smoothing and than the error of the time alignment.
            blocked = 0 < echo < STUCK_ECHO_CM and (echo - echo_next) * 4 < SHIFT_M * 100
            shifts.append(dict(t_s=round(v0, 1), yaw_deg=yaw, echo_cm=echo, echo_next_cm=echo_next,
                               sonar_says_blocked=int(blocked), moved_m=round(float(np.hypot(*(at(v1 + 1) - at(v0 - 1)))), 2)))
    out.mkdir(parents=True, exist_ok=True)
    for name, rows in (("lanes.csv", lanes), ("shifts.csv", shifts)):
        with open(out / name, "w", newline="") as f:
            if rows:
                writer = csv.DictWriter(f, rows[0].keys(), lineterminator="\n")
                writer.writeheader()
                writer.writerows(rows)
    long_lanes = [lane for lane in lanes if lane["metres"] >= 1.0]
    result = {"offset_s": round(offset, 1), "correlation": correlation and round(correlation, 3),
              "best_rival": rival and round(rival, 3), "lanes": len(lanes), "shifts": len(shifts)}
    if long_lanes:
        result["median_lane_m"] = round(float(np.median([x["metres"] for x in long_lanes])), 2)
        result["median_speed_cm_s"] = round(float(np.median([x["cm_s"] for x in long_lanes])), 1)
        if configured_cm_s:
            result["map_scale"] = round(result["median_speed_cm_s"] / configured_cm_s, 3)
    if len(long_lanes) >= 6:
        result["heading"] = {k: round(v, 3) if isinstance(v, float) else v for k, v in fit_heading(long_lanes).items()}
    for label, flag in (("blocked", 1), ("free", 0)):
        moved = [s["moved_m"] for s in shifts if s["sonar_says_blocked"] == flag]
        if moved:
            result[f"shift_moved_m_when_sonar_says_{label}"] = round(float(np.median(moved)), 2)
    lines = ["# Camera against recorder", "", f"Track `{track_path}`, session `{session}`.", "", "| | |", "|---|---|"]
    lines += [f"| {key} | {value} |" for key, value in result.items()]
    lines += ["", "`lanes.csv`: each lane as the camera saw it. `shifts.csv`: each lane shift, and whether the",
              "sonar rule and the camera agree that it moved. Read `correlation` first: below about 0.5,",
              "or close to `best_rival`, the two are not lined up and nothing else here counts.", ""]
    (out / "compare.md").write_bytes("\n".join(lines).encode())
    (out / "compare.json").write_bytes((json.dumps(result, indent=2) + "\n").encode())
    return result


def lane_pieces(pixels, times, recorder, offset):
    """(minutes, seconds, yaw_deg, x0, y0, x1, y1): the lanes as pieces of about four seconds in pixels."""
    with open(recorder, newline="") as f:
        rows = [{k: int(x) for k, x in r.items()} for r in csv.DictReader(f)]
    start_ms = next(r["t_ms"] for r in rows if r["step"] == 101)
    pieces = []
    for a, b in zip(rows, rows[1:]):
        if a["step"] != 101 or not a["inwater"]:
            continue
        t0, t1 = a["t_ms"] / 1000 + offset + 1.5, b["t_ms"] / 1000 + offset - 2.0   # not the turn, not the wall climb
        knots = [k for k in np.arange(t0, t1 + 1e-6, FIT_PIECE_S) if (np.abs(times - k) <= 1.0).sum() >= 2]
        if len(knots) < 2:
            continue
        px = [np.median(pixels[np.abs(times - k) <= 1.0], axis=0) for k in knots]
        yaw = (a["yaw"] + b["yaw"]) / 20 if abs(a["yaw"] - b["yaw"]) < 200 else a["yaw"] / 10
        for k0, p0, k1, p1 in zip(knots, px, knots[1:], px[1:]):
            pieces.append(((a["t_ms"] - start_ms) / 60000, k1 - k0, yaw, *p0, *p1))
    return np.array(pieces)


def camera_floor(points, p, centre, corner2):
    """Pixels to floor metres for a camera above the origin, looking along +y, with x to its right."""
    focal, tilt, roll, height, k1 = p[0], math.radians(p[1]), math.radians(p[2]), p[3], p[4]
    d = points - centre
    u = d / (1 + k1 * (d * d).sum(axis=1, keepdims=True) / corner2) / focal
    x = math.cos(roll) * u[:, 0] - math.sin(roll) * u[:, 1]
    y = math.sin(roll) * u[:, 0] + math.cos(roll) * u[:, 1]
    forward = math.cos(tilt) - y * math.sin(tilt)
    down = y * math.cos(tilt) + math.sin(tilt)
    t = height / np.maximum(down, 1e-3)
    return np.column_stack((t * x, t * forward))


def fit_camera(pieces, centre, corner2, speed):
    """Camera and gyro drift that make every lane piece speed x time long, along its heading."""
    from scipy.optimize import least_squares        # only this command needs it

    def residuals(p, sign, use):
        part = pieces[use]
        w0, w1 = camera_floor(part[:, 3:5], p, centre, corner2), camera_floor(part[:, 5:7], p, centre, corner2)
        heading = np.radians(p[5] + sign * (part[:, 2] - p[6] * part[:, 0]))
        return (w1 - w0 - speed * part[:, 1:2] * np.column_stack((np.cos(heading), np.sin(heading)))).ravel()

    low, high = (450, 15, -20, 1.0, -0.6, -720, -6), (2000, 88, 20, 8.0, 0.3, 720, 6)
    scale = (200, 10, 3, 1, 0.05, 20, 0.5)
    every = np.ones(len(pieces), bool)
    best = None
    for sign in (1, -1):
        for pan in range(0, 360, 30):
            for focal, tilt in ((900, 48), (700, 55), (1200, 42)):
                fit = least_squares(residuals, (focal, tilt, 0, 3.0, -0.1, pan, -2.0), args=(sign, every), loss="soft_l1",
                                    f_scale=0.1, bounds=(low, high), x_scale=scale, max_nfev=300)
                if best is None or fit.cost < best[0].cost:
                    best = (fit, sign)
    fit, sign = best
    use = every
    for _ in range(4):                              # lanes the tracker got wrong must not bend the camera
        size = np.hypot(*residuals(fit.x, sign, every).reshape(-1, 2).T)
        use = size < max(0.3, 2 * np.median(size[use]))
        fit = least_squares(residuals, fit.x, args=(sign, use), loss="soft_l1", f_scale=0.1, bounds=(low, high),
                            x_scale=scale, max_nfev=300)
    size = np.hypot(*residuals(fit.x, sign, use).reshape(-1, 2).T)
    return fit.x, sign, int(use.sum()), float(np.median(size))


def calibrate(track_path, rough_path, session, out, offset=None, speed=FIT_SPEED_M_S):
    """A calibration from the rough track: map right is the camera's right, map up is away from it."""
    rough = json.loads(Path(rough_path).read_text())
    cal = Calibration(rough)
    points = read_track(track_path)
    recorder = Path(session) / "recorder.csv"
    if offset is None:
        offset, *_ = find_offset(points, recorder_intervals(recorder))
    pieces = lane_pieces(cal.seen(points[:, 1:3]), points[:, 0], recorder, offset)
    if len(pieces) < 20:
        raise SystemExit(f"only {len(pieces)} lane pieces in view: not enough to fit a camera")
    p, sign, used, residual = fit_camera(pieces, cal.centre, cal.corner2, speed)
    corners = np.array(((100, 300), (1820, 300), (1820, 1000), (100, 1000)), np.float64) * (2 * cal.centre) / (1920, 1080)
    d = corners - cal.centre
    ideal = d / (1 + p[4] * (d * d).sum(axis=1, keepdims=True) / cal.corner2) + cal.centre
    floor = camera_floor(corners, p, cal.centre, cal.corner2) * (1, -1)        # the canvas y axis points down
    to_m, _ = cv2.findHomography(ideal.astype(np.float32), floor.astype(np.float32), 0)
    data = {key: rough[key] for key in ("image_size", "pool_outline_image", "exclude_image", "mask_margin_m",
                                        "robot_m", "brush_m") if key in rough}
    data.update(k1=float(p[4]), homography_image_to_m=to_m.tolist())
    outline = Calibration(dict(data)).outline
    shift = np.array(((1, 0, -outline[:, 0].min()), (0, 1, -outline[:, 1].min()), (0, 0, 1.0)))
    data["homography_image_to_m"] = (shift @ to_m).tolist()
    data["fit"] = {"focal_px": round(float(p[0]), 1), "tilt_deg": round(float(p[1]), 1), "roll_deg": round(float(p[2]), 1),
                   "height_m": round(float(p[3]), 2), "gyro_drift_deg_min": round(float(p[6]), 2), "yaw_sign": sign,
                   "speed_m_s": speed, "offset_s": round(float(offset), 1), "pieces": len(pieces), "pieces_used": used,
                   "median_residual_m": round(residual, 3), "piece_m": round(speed * FIT_PIECE_S, 2)}
    Path(out).parent.mkdir(parents=True, exist_ok=True)
    Path(out).write_bytes((json.dumps(data, indent=1) + "\n").encode())
    return data


def save_frame(paths, at_s, out, calibration=None):
    grabbed = frames_at(paths, [at_s])
    if not grabbed:
        raise SystemExit(f"no frame at {at_s} s")
    frame = grabbed[0][1]
    out = Path(out)
    out.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(out), frame)
    grid = frame.copy()
    height, width = grid.shape[:2]
    for x in range(0, width, 100):
        cv2.line(grid, (x, 0), (x, height), (255, 255, 255), 1)
        cv2.putText(grid, str(x), (x + 2, 14), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1, cv2.LINE_AA)
    for y in range(0, height, 100):
        cv2.line(grid, (0, y), (width, y), (255, 255, 255), 1)
        cv2.putText(grid, str(y), (2, y + 14), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1, cv2.LINE_AA)
    cv2.imwrite(str(out.with_name(out.stem + "_grid" + out.suffix)), grid)
    if calibration:
        cal = load_calibration(calibration)
        view = cal.warp(frame)
        cv2.polylines(view, [cal.canvas_px(cal.outline)], True, (0, 255, 255), 1, cv2.LINE_AA)
        cv2.imwrite(str(out.with_name(out.stem + "_topdown" + out.suffix)), view)
    return width, height


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="command", required=True)
    one = sub.add_parser("frame", help="save one frame with a pixel grid (and top-down with --calib)")
    one.add_argument("video", nargs="+", type=Path)
    one.add_argument("--at-s", type=float, default=30.0)
    one.add_argument("--out", type=Path, required=True)
    one.add_argument("--calib", type=Path)
    two = sub.add_parser("track", help="follow the robot; files in recording order")
    two.add_argument("video", nargs="+", type=Path)
    two.add_argument("--calib", type=Path, required=True)
    two.add_argument("--out", type=Path, required=True)
    two.add_argument("--every-s", type=float, default=0.5, help="time between frames used")
    two.add_argument("--start-s", type=float, default=0.0, help="skip people carrying the robot")
    two.add_argument("--end-s", type=float)
    two.add_argument("--max-speed", type=float, default=0.35, help="m/s the track may not exceed for free")
    three = sub.add_parser("compare", help="line the track up with a watertest session")
    three.add_argument("--track", type=Path, required=True)
    three.add_argument("--session", type=Path, required=True)
    three.add_argument("--out", type=Path, required=True)
    three.add_argument("--offset-s", type=float, help="video time minus robot uptime, if already known")
    three.add_argument("--configured-cm-s", type=float, help="the speed the map assumed, gives the true map scale")
    four = sub.add_parser("calibrate", help="fit the camera from the robot's lanes, from a first rough track")
    four.add_argument("--track", type=Path, required=True)
    four.add_argument("--rough", type=Path, required=True, help="the calibration the track was made with")
    four.add_argument("--session", type=Path, required=True)
    four.add_argument("--out", type=Path, required=True)
    four.add_argument("--offset-s", type=float, help="video time minus robot uptime, if already known")
    four.add_argument("--speed", type=float, default=FIT_SPEED_M_S, help="lane speed in m/s: sets the scale")
    args = parser.parse_args()
    if args.command == "frame":
        width, height = save_frame(args.video, args.at_s, args.out, args.calib)
        print(f"{args.out}: {width} x {height} pixels; read the points from the _grid picture")
    elif args.command == "track":
        cal = load_calibration(args.calib)
        summary, _ = track(args.video, cal, args.out, args.every_s, args.start_s, args.end_s, args.max_speed)
        print(json.dumps(summary, indent=2))
        if summary["tracked_percent"] < WARN_TRACKED_PERCENT:
            print(f"NOTE: {summary['tracked_percent']} % of frames tracked. Either the robot left the part of the pool "
                  "the camera can see, or it hardly stands out of the water: check overlay.jpg and checks.jpg.")
        print(f"Check {args.out / 'checks.jpg'} and {args.out / 'overlay.jpg'} before believing these numbers.")
    elif args.command == "calibrate":
        data = calibrate(args.track, args.rough, args.session, args.out, args.offset_s, args.speed)
        print(json.dumps(data["fit"], indent=2))
        print(f"Run frame with --calib {args.out}: frame_topdown.png must show the pool as a true plan, not folded.")
    else:
        print(json.dumps(compare(args.track, args.session, args.out, args.offset_s, args.configured_cm_s), indent=2))


if __name__ == "__main__":
    main()
