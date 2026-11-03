#!/usr/bin/env python3
"""navsim - coverage simulation of the Crab's decoded cleaning patterns in a kidney pool.

Models decoded stock behaviour against the exact compiled nav_tick controller, on a
10 cm truth grid used only for scoring. The optimized controller cannot read that grid.

  stock-floor     CTYPE 1 lanes: wall by tilt -> STEP 20 (6 s: stop, 3 s back-off, stop), turn 90,
                  1.4 s shift between 1 s stops, turn 76 (166 deg U-turn), STEP 20 again, run.
                  40 s timeout -> 7 s STEP 0. Short-lane counter adds 180 to the second turn
                  (-45 every second flip). Each turn relative to the current (drifting) yaw.
                  Timings come from the decoded stock program.
  stock-standard  CTYPE 2 bounce (variant A: steering off, reverse 6-9 s, -20 deg per wall,
                  every 3rd wall -90 + probe + 70) for 60 min, then stock-floor lanes.
  optimized-floor sensor-only lanes, timed spacing/transfer, both sides then cross pass;
                  diag-3.3 cores also read the sonar and the compiled map, which runs
                  alongside them in every optimized mode (wall phases included)
  optimized-standard optimized floor20min, stock wall phase10min with flat backoff1.5s
  proposed        historical perfect-map experiment retained for reference, not run by CLI

Assumptions (not measured yet - the recorder run will replace them): speed 0.30 m/s at drive
70 (scaled linearly), reverse 0.21 m/s at -50, turn rate 60 deg/s + 1 s settle, robot width
0.40 m; --matrix varies width0.30..0.50m, speed0.20..0.40m/s, drift0.5/2deg/min.
Optimized observations have noise/dropouts; timed odometry has uncalibrated speed and
slip. No exact position, map, wall-normal reset or ground-speed sensor is used. Stock
primitives are the older approximation, without equivalent sensor noise/slip; these
comparisons cannot establish hardware coverage, safety, wall cleaning or battery life.
Measured in water: the sonar reads nothing under
21 cm or beyond about 4.5 m, about 30 cm more than the gap to the wall (34 cm pushing at it)
and often nothing at contact; a lane at drive 70 ends by climbing (tilt), a 1.5 s shift at
drive 60 pushes at the cove without tilting. Before that navsim's sonar reported 5 cm and
every contact was a wall observation, which hid the run 4 nook trap.

    python -I tools/navsim.py [--minutes 60] [--seeds 5] [--out analysis/navsim]
"""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import random
import importlib.util
import tempfile

import numpy as np

# ---------------------------------------------------------------- pool (metres)
CELL = 0.10
W, H = 11.0, 6.5
POOL = "kidney"
START = "random"
ERROR_PROFILE = "nominal"
ERRORS = {"nominal": (.5, 3., .03, .90), "adverse": (2., 10., .20, .70)}


def in_pool(x, y):
    """Kidney: two lobes joined by a waist, with a bite out of the top edge."""
    if POOL == "nook":
        # A real pool, coarsely: a 9 x 4 m body rounded at the left, and the three-walled
        # nook at the bottom right where run 4 was trapped (2 m wide, 1.8 m deep).
        body = 1 <= x <= 10 and 2.2 <= y <= 6.2 and (x >= 3 or (x-3)**2 + (y-4.2)**2 <= 4)
        return body or (8 <= x <= 10 and .4 <= y <= 2.2)
    if POOL != "kidney":
        outer = 1 <= x <= 10 and 1 <= y <= 5.5
        if POOL == "rectangle": return outer
        if POOL == "l-shape": return outer and not (x > 5.5 and y > 3)
        if POOL == "island": return outer and (x-5.5)**2 + (y-3.25)**2 > .8**2
        raise ValueError(f"unknown pool: {POOL}")
    a = (x - 3.6) ** 2 / 3.0 ** 2 + (y - 2.9) ** 2 / 2.2 ** 2 <= 1
    b = (x - 7.2) ** 2 / 2.4 ** 2 + (y - 3.4) ** 2 / 1.9 ** 2 <= 1
    bridge = 3.6 <= x <= 7.2 and 1.4 <= y <= 4.6
    bite = (x - 5.6) ** 2 / 1.6 ** 2 + (y - 5.6) ** 2 / 1.3 ** 2 <= 1
    return (a or b or bridge) and not bite


POOLS = ("kidney", "rectangle", "l-shape", "island", "nook")
STARTS = ("random", "edge", "center", "nook")
GRID = np.zeros((int(H / CELL), int(W / CELL)), dtype=bool)
for j in range(GRID.shape[0]):
    for i in range(GRID.shape[1]):
        GRID[j, i] = in_pool((i + 0.5) * CELL, (j + 0.5) * CELL)
POOL_CELLS = int(GRID.sum())


def environment(pool="kidney", start="random", errors="nominal"):
    """Plant/scoring setup only; geometry and start policy never enter nav_tick."""
    global POOL, START, ERROR_PROFILE, GRID, POOL_CELLS, CENTROID
    if pool not in POOLS:
        raise ValueError("unknown pool")
    if start not in STARTS or errors not in ERRORS:
        raise ValueError("unknown start or error profile")
    if start == "nook" and pool != "nook":
        raise ValueError("the nook start needs the nook pool")
    POOL, START, ERROR_PROFILE = pool, start, errors
    GRID = np.array([[in_pool((i+.5)*CELL, (j+.5)*CELL)
                      for i in range(int(W/CELL))] for j in range(int(H/CELL))])
    POOL_CELLS = int(GRID.sum())
    jj, ii = np.nonzero(GRID)
    CENTROID = ((ii.mean()+.5)*CELL, (jj.mean()+.5)*CELL)


def environment_parameters():
    yaw, sonar, dropout, slip = ERRORS[ERROR_PROFILE]
    return {"pool": POOL, "start": START, "errors": ERROR_PROFILE,
            "floor_area_m2": POOL_CELLS*CELL*CELL,
            "yaw_noise_sd_deg": yaw, "sonar_noise_sd_cm": sonar,
            "sonar_dropout_probability": dropout, "minimum_motion_fraction": slip,
            "maximum_motion_fraction": 1., "measured": False}

# ---------------------------------------------------------------- robot model
WIDTH = 0.40
V70 = 0.30          # m/s at drive 70
REVERSE_V50 = None  # None preserves the original proportional forward/reverse model
TURN_RATE = 60.0    # deg/s
DRIFT_PER_MIN = 0.5
CHECK_EVERY = 10.0  # s between coverage checkpoints

DEFAULTS = {"width_m": WIDTH, "v70_m_s": V70, "reverse_v50_m_s": None,
            "turn_rate_deg_s": TURN_RATE, "drift_deg_per_min": DRIFT_PER_MIN}
MODEL_SOURCE = {k: "assumed original model" for k in DEFAULTS}


def read_calibration(path):
    """Accept only explicitly supplied values with provenance; never consume sonar candidates."""
    data = json.loads(Path(path).read_text(encoding="utf-8"))
    if (not isinstance(data, dict) or type(data.get("schema_version")) is not int or
            data["schema_version"] != 1 or set(data) - {"schema_version", "measurements", "notes"}):
        raise ValueError("calibration must use schema_version 1 and measurements")
    values = data.get("measurements")
    if not isinstance(values, dict) or set(values) - DEFAULTS.keys():
        raise ValueError("unknown or missing calibration measurements object")
    for name, entry in values.items():
        if not isinstance(entry, dict) or set(entry) != {"value", "source"}:
            raise ValueError(f"{name}: requires value and source")
        value, source = entry["value"], entry["source"]
        if (isinstance(value, bool) or not isinstance(value, (int, float)) or
                not math.isfinite(value) or value < 0 or value == 0 and name != "drift_deg_per_min"):
            raise ValueError(f"{name}: requires finite positive value (drift may be zero)")
        if name == "width_m" and value > min(W, H):
            raise ValueError("width_m exceeds this simulation's pool bounds")
        if not isinstance(source, str) or not source.strip():
            raise ValueError(f"{name}: non-empty measurement source required")
    return values


def configure(values):
    """Reset defaults before applying validated measurements; rebuild the footprint."""
    global WIDTH, V70, REVERSE_V50, TURN_RATE, DRIFT_PER_MIN, STENCIL, MODEL_SOURCE
    merged = dict(DEFAULTS)
    MODEL_SOURCE = {k: "assumed original model" for k in DEFAULTS}
    for name, entry in values.items():
        merged[name] = entry["value"]
        MODEL_SOURCE[name] = entry["source"]
    WIDTH, V70 = merged["width_m"], merged["v70_m_s"]
    REVERSE_V50, TURN_RATE = merged["reverse_v50_m_s"], merged["turn_rate_deg_s"]
    DRIFT_PER_MIN = merged["drift_deg_per_min"]
    STENCIL = [(dx, dy) for dx in np.arange(-WIDTH / 2, WIDTH / 2 + 1e-9, CELL / 2)
               for dy in np.arange(-WIDTH / 2, WIDTH / 2 + 1e-9, CELL / 2)
               if dx * dx + dy * dy <= (WIDTH / 2) ** 2]


def model_parameters():
    values = (WIDTH, V70, REVERSE_V50, TURN_RATE, DRIFT_PER_MIN)
    return {name: {"value": value, "source": MODEL_SOURCE[name]}
            for name, value in zip(DEFAULTS, values)}

# paint stencil (cell offsets inside the robot's footprint)
_r = WIDTH / 2
STENCIL = [(dx, dy) for dx in np.arange(-_r, _r + 1e-9, CELL / 2)
           for dy in np.arange(-_r, _r + 1e-9, CELL / 2) if dx * dx + dy * dy <= _r * _r]


def speed(drive):
    if drive < 0 and REVERSE_V50 is not None:
        return REVERSE_V50 * drive / 50.0
    return V70 * drive / 70.0


class Sim:
    def __init__(self, seed, minutes):
        self.rng = random.Random(seed)
        self.budget = minutes * 60.0
        self.t = 0.0
        self.cov = np.zeros(GRID.shape, dtype=np.int32)
        self.yaw_err = 0.0
        while True:
            self.x, self.y = self.rng.uniform(0, W), self.rng.uniform(0, H)
            if in_pool(self.x, self.y):
                break
        self.hdg = self.rng.uniform(0, 360)
        if START == "nook":
            # Run 4: lanes across the nook, shifting toward its back wall (+90 from 180).
            self.x, self.y, self.hdg = 9.0, 1.2, (180 + self.rng.gauss(0, 5)) % 360
        elif START != "random":
            # Explicit adversarial starts use a footprint-clear grid point. This is
            # plant initialization, not localization information given to the core.
            jj, ii = np.nonzero(GRID)
            candidates = [((i+.5)*CELL, (j+.5)*CELL) for j,i in zip(jj,ii)
                          if all(in_pool((i+.5)*CELL+math.cos(a)*WIDTH/2,
                                         (j+.5)*CELL+math.sin(a)*WIDTH/2)
                                 for a in np.arange(0, 2*math.pi, math.pi/4))]
            target = CENTROID if START == "center" else (0, H/2)
            self.x, self.y = min(candidates, key=lambda p:
                                (p[0]-target[0])**2+(p[1]-target[1])**2)
        self.walls = 0
        self.turns = 0
        self.history = []          # (t, covered fraction)
        self._next_check = 0.0
        self.mapper = None         # optimized runs: the compiled map follows every primitive

    # --- primitives --------------------------------------------------------
    def done(self):
        return self.t >= self.budget

    def _tick(self, dt):
        self.t = min(self.budget, self.t + dt)
        if self.t >= self._next_check:
            self.history.append((self.t, self.coverage()[0]))
            self._next_check += CHECK_EVERY

    def paint(self):
        for dx, dy in STENCIL:
            i, j = int((self.x + dx) / CELL), int((self.y + dy) / CELL)
            if 0 <= i < GRID.shape[1] and 0 <= j < GRID.shape[0] and GRID[j, i]:
                self.cov[j, i] += 1

    def wait(self, s):
        self._tick(s)
        if self.mapper: self.mapper(0)

    def turn(self, deg):
        """Turn in place by deg (relative to the current, drifting heading)."""
        duration = abs(deg) / TURN_RATE + 1.0
        fraction = min(1., max(0., self.budget-self.t)/duration)
        self.hdg = (self.hdg + (deg + self.rng.gauss(0, 1.0))*fraction) % 360
        self._tick(duration)
        self.turns += 1
        if self.mapper: self.mapper(0)

    def drive(self, drive, max_s):
        """Drive at `drive` (negative = reverse) until a wall or max_s. Returns 'wall'/'time'."""
        v = abs(speed(drive))
        end = min(self.budget, self.t + max_s)
        if self.mapper: self.mapper(drive)
        while self.t < end:
            if self.done():
                return "time"
            dt = min(.05, end-self.t)
            self.yaw_err += DRIFT_PER_MIN * dt / 60.0
            h = math.radians(self.hdg + self.yaw_err + (0 if drive > 0 else 180))
            nx, ny = self.x + math.cos(h) * v * dt, self.y + math.sin(h) * v * dt
            ax, ay = nx + math.cos(h) * WIDTH / 2, ny + math.sin(h) * WIDTH / 2
            if not in_pool(ax, ay):
                self.walls += 1
                return "wall"
            self.x, self.y = nx, ny
            self.paint()
            self._tick(dt)
            if self.mapper: self.mapper(drive)
        return "time"

    def coverage(self):
        c = self.cov[GRID]
        return float((c > 0).mean()), float((c > 1).mean())

    def time_to(self, frac):
        for t, c in self.history:
            if c >= frac:
                return t
        return None


# ---------------------------------------------------------------- stock patterns
def stock_step20(s: Sim):
    """STEP 20: stop 1 s, reverse -50 until 4 s after entry
    (or roll >= +15 deg, not modelled), then stop until 6 s after entry."""
    t0 = s.t
    s.wait(1.0)
    s.drive(-50, 3.0)
    s.wait(max(0.0, 6.0 - (s.t - t0)))


def stock_step0(s: Sim):
    """STEP 0: target := yaw, stop 1 s, reverse -50 for 5 s, stop 1 s."""
    t0 = s.t
    s.wait(1.0)
    s.drive(-50, 5.0)
    s.wait(max(0.0, 7.0 - (s.t - t0)))


def stock_floor(s: Sim):
    """CTYPE 1 lanes as decoded, every wall by tilt.

    Per lane end: STEP 20 (6 s), turn +-90, STEP 3/7 shift (1 s stop, drive 60 for 1.4 s,
    1 s stop; contact aborts it and adds 2 short), turn +-76 (the 180 flip is added to
    this turn), STEP 20 again (6 s), run. A 40 s run timeout is STEP 0 (7 s, 5 s reverse);
    two timeouts in a row make the next run a wall at once. Short-lane counter 0x200000C8:
    +2 for a run under 9333 - 33.3 x battery% ms, -1 per shift with no change since the
    last shift (0 < c < 6), >= 6 adds 180 to the second turn, every second flip -45.
    Run speed is drive 70 = app speed 40 (55/65 at 20/30). Not modelled: the sonar < 40 cm
    shift abort, bump-found walls (90 deg second turn), climbing time at the wall.
    """
    sign = 1
    short = 0          # 0x200000C8
    short_seen = 0     # 0x2000031D: counter value at the previous shift
    flips = 0          # 0x20000134
    timeouts = 0       # 0x2000031E: consecutive 40 s timeouts
    stock_step0(s)
    while not s.done():
        t0 = s.t
        r = "wall" if timeouts >= 2 else s.drive(70, 40.0)   # step 1 or 5
        if r != "wall":
            timeouts += 1                                     # timeout -> STEP 0
            stock_step0(s)
            continue
        timeouts = 0
        if s.t - t0 < 9.333 - 0.0333 * 80:                    # battery 80 %
            short += 2
        stock_step20(s)
        s.turn(sign * 90)                                     # STEP 2/6
        s.wait(1.0)                                           # STEP 3/7
        if s.drive(60, 1.4) == "wall":                        # aborted shift
            short += 2
        s.wait(1.0)
        turn = sign * 76                                      # wall found by tilt
        if short_seen == short and 0 < short < 6:
            short -= 1
        short_seen = short
        if short >= 6:
            short = 0
            flips += 1
            turn += 180 - (45 if flips % 2 == 0 else 0)
        s.turn((turn + 180) % 360 - 180)                      # STEP 4/8, shortest way
        stock_step20(s)                                       # STEP 20 again
        sign = -sign


def stock_bounce(s: Sim, max_minutes, back_seconds=None):
    """CTYPE 2 variant A as decoded: steering off, long reversing, fixed small turns."""
    end = min(s.budget, s.t + max_minutes * 60)
    wallcount = 0
    s.wait(3.0)
    while s.t < end and not s.done():
        s.yaw_err += s.rng.gauss(0, 3.0)      # steering off: open-loop run, extra drift
        s.drive(90, 100.0)                     # run (speed 40 -> 90, max 100 s)
        s.wait(2.0)                            # keeps pushing 2 s after the trigger
        s.drive(-40, 2.0)                      # reverse down until flat (~2 s)
        wallcount += 1
        s.drive(-60, back_seconds if back_seconds is not None else
                (9.0 if wallcount < 2 else 6.0))   # flat step4 only; descent above unchanged
        if wallcount >= 3:
            s.turn(-90)                        # step 5
            s.drive(60, 5.0); s.wait(2.0); s.drive(-60, 5.0); s.wait(3.0)   # step 15 probe
            s.turn(70)                         # step 6
            wallcount = 0
        else:
            s.turn(-20)                        # step 6


def stock_standard(s: Sim):
    stock_bounce(s, 60)
    stock_floor(s)


def stock_bounce_b(s, max_minutes, back_seconds=6.0):
    """PB4-high wall variant, planar projection only (no wall area/energy scoring).

    Assume 6s climb to the stock timeout + 6s top hold, then 2s descent until flat;
    heading alignment costs1s. These times are assumptions, not measured geometry.
    Shortening affects only the subsequent flat step4, never those climb/descent times.
    """
    end = min(s.budget, s.t + max_minutes*60)
    count = 0
    while s.t < end and not s.done():
        s.drive(80, 100)
        s.wait(12)
        s.drive(-45, 2)
        s.wait(1)
        s.drive(-60, back_seconds)
        count += 1
        if count == 3:
            s.turn(-90)
            s.drive(60, 5); s.wait(2); s.drive(-60, 5); s.wait(3)
            s.turn(80)
            count = 0
        else:
            s.turn(-10)


def stock_standard_b(s):
    stock_bounce_b(s, 60)
    stock_floor(s)


# ---------------------------------------------------------------- proposed pattern
_jj, _ii = np.nonzero(GRID)
CENTROID = ((_ii.mean() + 0.5) * CELL, (_jj.mean() + 0.5) * CELL)


def unvisited_target(s: Sim, min_dist=0.5):
    """Nearest unvisited patch (an unvisited cell with >= 5 unvisited cells within 30 cm),
    as the robot's own coverage map would report it. None when the map is full."""
    unv = GRID & (s.cov == 0)
    if not unv.any():
        return None
    jj, ii = np.nonzero(unv)
    xs, ys = (ii + 0.5) * CELL, (jj + 0.5) * CELL
    d = np.hypot(xs - s.x, ys - s.y)
    for k in np.argsort(d):
        if d[k] < min_dist:
            continue
        near = np.hypot(xs - xs[k], ys - ys[k]) <= 0.3
        if near.sum() >= 5:
            return xs[k], ys[k]
    return None


def proposed(s: Sim):
    """Parallel lanes with exact U-turns, a distance-based shift, and a coverage map.

    One sweep = lanes along `axis`, each lane to the wall, shifting sideways by `shift`
    towards the side where the map still has unvisited floor. The sweep ends when the shift
    runs into the wall or lanes get very short. Then the robot drives to the nearest
    unvisited patch (from its map) and sweeps there with the lanes rotated 90 deg
    (cross-hatch). Once the map is full, repeats start half a lane offset.
    """
    shift = WIDTH * 0.85
    axis = s.hdg
    sweep = 0
    while not s.done():
        s.hdg = axis % 360
        # shift side: towards the unvisited area (centroid of the pool once it is all visited)
        unv = GRID & (s.cov == 0)
        if unv.sum() > 20:
            jj, ii = np.nonzero(unv)
            tx, ty = (ii.mean() + 0.5) * CELL, (jj.mean() + 0.5) * CELL
        else:
            tx, ty = CENTROID
        h = math.radians(axis)
        left = (-math.sin(h), math.cos(h))
        sign = 1 if left[0] * (tx - s.x) + left[1] * (ty - s.y) >= 0 else -1
        first = True
        short = 0
        while not s.done():
            t0 = s.t
            r = s.drive(70, 60.0)
            s.yaw_err = 0.0                       # heading re-anchored at the wall
            if r != "wall":
                break
            short = short + 1 if s.t - t0 < 2.0 else 0
            if short >= 3:                        # lanes too short: this side is finished
                break
            s.wait(0.5); s.drive(-50, 1.5)        # short back-off so the turn has room
            s.turn(sign * 90)
            d = shift / 2 if (first and sweep >= 2 and sweep % 2 == 0) else shift
            first = False
            if s.drive(60, d / speed(60)) == "wall":
                break                             # no room to shift: sweep complete
            s.turn(sign * 90)
            sign = -sign
        sweep += 1
        tgt = unvisited_target(s)
        if tgt is not None:                       # go to the nearest gap and sweep there
            ang = math.degrees(math.atan2(tgt[1] - s.y, tgt[0] - s.x))
            s.wait(0.5); s.drive(-50, 1.5)
            s.turn((ang - s.hdg + 180) % 360 - 180)
            s.yaw_err = 0.0
            s.drive(70, math.hypot(tgt[0] - s.x, tgt[1] - s.y) / speed(70))
            axis = s.hdg + 90 if sweep % 2 else s.hdg
        else:
            axis += 90                            # map full: cross-hatch repeats


PATTERNS = {"stock-floor": stock_floor, "stock-standard": stock_standard, "proposed": proposed}

# The historical 'proposed' function above is retained as an explicitly idealized
# reference, excluded from the deliverable comparison. It sees the ground-truth map.
COMPILED = None
# The compiled boot defaults (flags 1 from diag-3.4), but at the plant's speed: their 23 cm/s is the
# real robot's, and at V70 it would time a 45 cm shift. Studies may set the 8 FC4B values; None
# takes the compiled defaults as they are.
NAV_CONFIG = (1, 30, 30, 1000, 20, 10, 3, 1500)
NAV_MAPPED = True   # False: a diag-3.3 core without the map (as with 16 KB SRAM)


def load_tool(name):
    spec = importlib.util.spec_from_file_location(name, Path(__file__).with_name(name + ".py"))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


# Measured sonar: no reading under 21 cm or past about
# 4.5 m (longest 429 cm); 34 cm while pushing at the wall, 30-70 cm at the nook's cove.
SONAR_OFFSET_CM = 30     # reading minus the gap between the robot's front and the wall
SONAR_BLIND_CM = 21
SONAR_MAX_CM = 450
SONAR_CONTACT_CM = 10    # this close to the wall, half the readings are lost (9999)


def observed_range(s):
    """Synthetic forward sonar, with noise/dropouts; only the plant sees true pose."""
    _, noise, dropout, _ = ERRORS[ERROR_PROFILE]
    if s.rng.random() < dropout:
        return 9999
    h = math.radians(s.hdg)
    for gap in range(0, 1201, 5):
        reach = (WIDTH * 100 / 2 + gap) / 100
        if not in_pool(s.x + math.cos(h) * reach, s.y + math.sin(h) * reach):
            if gap < SONAR_CONTACT_CM and s.rng.random() < .5:
                return 9999
            reading = gap + SONAR_OFFSET_CM + s.rng.gauss(0, noise)
            return reading if SONAR_BLIND_CM <= reading <= SONAR_MAX_CM else 9999
    return 9999


def sensed(s):
    yaw_noise = ERRORS[ERROR_PROFILE][0]
    return round(((s.hdg + s.yaw_err + s.rng.gauss(0, yaw_noise) + 180) % 360 - 180) * 10)


def nav_core(s):
    """One compiled core per run, as the firmware keeps one: its map spans the whole run
    and the stock primitives (wall phases, timeout fallback) move it too."""
    if getattr(s, "core", None) is None:
        s.core = load_tool("navcore").Core(*COMPILED, config=NAV_CONFIG)
        s.core.map_reset()
        s.core_yielded = 0.0
        s.mapper = lambda drive: s.core.map_tick(round(s.t * 1000), sensed(s), drive, 0,
                                                 observed_range(s))
    return s.core


def optimized_lanes(s, end):
    """Execute compiled firmware core, never supply pose or coverage to its inputs.

    The plant has 5% random longitudinal slip and heading-response uncertainty; yaw
    drifts continuously and has 0.5 degree noise. Distance uses configured speed30cm/s,
    independent of actual plant speed. There is no re-anchoring at a wall.
    A floor phase restarts the lanes (phase 0) as the firmware's mode change does; the
    core's counters and the map carry on.
    """
    core = nav_core(s)
    core.uc.mem_write(load_tool("navcore").STATE_AT + 16, bytes(4))     # phase := 0
    wall = False
    while s.t < min(end, s.budget):
        dt = min(.1, min(end, s.budget)-s.t)
        motion_min = ERRORS[ERROR_PROFILE][3]
        sensed_yaw = sensed(s)
        sonar = observed_range(s)
        base, target = core.tick(round((s.t - s.core_yielded) * 1000), sensed_yaw,
                                 wall or sonar < 10, sonar, mapped=NAV_MAPPED)
        if core.state[4] == 9:
            s.timeouts = getattr(s, "timeouts", 0) + 1
            # The firmware falls back to vendor lanes on timeout. Bound this floor
            # phase too so Standard can still transition to walls.
            original = s.budget
            s.budget = min(end, original)
            stock_floor(s)
            s.budget = original
            break
        core.map_tick(round(s.t * 1000), sensed_yaw, base, 0, sonar)
        error = (target / 10 - (s.hdg + s.yaw_err) + 180) % 360 - 180
        turn = max(-TURN_RATE, min(TURN_RATE, error * 3)) * dt
        s.hdg = (s.hdg + turn * s.rng.uniform(.95, 1.05)) % 360
        s.yaw_err += DRIFT_PER_MIN * dt / 60
        h = math.radians(s.hdg)
        distance = speed(base) * dt * s.rng.uniform(motion_min, 1.0)
        nx, ny = s.x + math.cos(h) * distance, s.y + math.sin(h) * distance
        front = h + (math.pi if base < 0 else 0)
        blocked = base != 0 and not in_pool(nx + math.cos(front) * WIDTH / 2,
                                            ny + math.sin(front) * WIDTH / 2)
        if not blocked:
            s.x, s.y = nx, ny
        else:
            s.walls += 1
        s.paint()
        s._tick(dt)
        # Only a lane or transfer (drive 70) climbs the wall to a tilt; a shift (drive 60)
        # pushes at the cove unobserved, as all 72 shifts at the run 4 nook did.
        wall = blocked and base >= 70
        if wall:
            # Forward contact = tilt on the wall: the hook yields and the vendor STEP 20
            # runs its 1 s stop before the robot reverses flat (climb/descent unmodelled,
            # as for stock). Charged here so both programs pay the vendor's stop.
            t0 = s.t
            s.wait(1.0)
            s.core_yielded += s.t - t0


def optimized_floor(s):
    optimized_lanes(s, s.budget)


def optimized_standard(s):
    config = nav_core(s).config
    while not s.done():
        optimized_lanes(s, s.t + config[4] * 60)
        if not s.done():
            # Same stock wall maneuvers, only the flat back-off is shortened.
            wall_phase(s, stock_bounce, config[5], config[7]/1000)


def optimized_standard_b(s):
    config = nav_core(s).config
    while not s.done():
        optimized_lanes(s, s.t + config[4]*60)
        if not s.done():
            wall_phase(s, stock_bounce_b, config[5], config[7]/1000)


def wall_phase(s, pattern, minutes, back_seconds):
    """Bound primitives to this phase as well as the whole comparison run."""
    original = s.budget
    s.budget = min(original, s.t+minutes*60)
    try:
        pattern(s, minutes, back_seconds=back_seconds)
    finally:
        s.budget = original


PATTERNS = {"stock-floor": stock_floor, "optimized-floor": optimized_floor,
            "stock-standard": stock_standard, "optimized-standard": optimized_standard,
            "stock-standard-b": stock_standard_b, "optimized-standard-b": optimized_standard_b}


def render(sim: Sim, title, path):
    """Coverage map as a PNG (PIL): red = never, light blue = 1-2x, mid = 3-7x, dark = 8x+."""
    from PIL import Image, ImageDraw
    img = np.full(GRID.shape + (3,), 255, dtype=np.uint8)
    img[~GRID] = (215, 215, 215)
    c = sim.cov
    img[GRID & (c == 0)] = (240, 90, 90)
    img[GRID & (c >= 1)] = (140, 205, 255)
    img[GRID & (c >= 3)] = (25, 115, 215)
    img[GRID & (c >= 8)] = (5, 40, 115)
    scale = 8
    im = Image.fromarray(img[::-1]).resize((GRID.shape[1] * scale, GRID.shape[0] * scale), Image.NEAREST)
    canvas = Image.new("RGB", (im.width, im.height + 44), (255, 255, 255))
    canvas.paste(im, (0, 44))
    cov1, cov2 = sim.coverage()
    d = ImageDraw.Draw(canvas)
    d.text((8, 6), f"{title}   covered {cov1 * 100:.0f} %, twice+ {cov2 * 100:.0f} %, "
                   f"{sim.walls} walls, {sim.turns} turns", fill=(0, 0, 0))
    d.text((8, 24), "red = never cleaned, light blue = 1-2x, mid = 3-7x, dark = 8x+; grey = outside the pool",
           fill=(80, 80, 80))
    canvas.save(path)


def snapshot(sim: Sim, title, path):
    render(sim, title, path)


def comparison_visuals(examples, histories, minutes, out, seed_start=0):
    """Binary coverage maps and paired mean curves; visit counts have different time steps
    in the two plant models, so do not compare the older visit-density coloring."""
    from PIL import Image, ImageDraw, ImageFont
    def font(size):
        try: return ImageFont.truetype("C:/Windows/Fonts/segoeui.ttf", size)
        except OSError: return ImageFont.load_default()
    pairs = (("Floor-only", "stock-floor", "optimized-floor"),
             ("Standard / wall A", "stock-standard", "optimized-standard"),
             ("Standard / wall B", "stock-standard-b", "optimized-standard-b"))
    maps = Image.new("RGB", (1440, 1530), "white")
    draw = ImageDraw.Draw(maps)
    draw.text((30, 15), f"Simulated floor coverage after {minutes:g} minutes", fill="#172b45", font=font(30))
    draw.text((30, 60), f"Assumed {POOL}/{START}/{ERROR_PROFILE}, width {WIDTH:.2f}m, speed {V70:.2f}m/s, drift {DRIFT_PER_MIN:g}; seed{seed_start}", fill="#40536a", font=font(18))
    draw.text((30, 90), "Blue = covered floor   Red = missed floor   Grey = outside pool", fill="#40536a", font=font(18))
    for row, (label, stock, new) in enumerate(pairs):
        y = 145 + row*435
        for col, name in enumerate((stock, new)):
            sim = examples[name]
            pixels = np.full(GRID.shape + (3,), 232, dtype=np.uint8)
            pixels[GRID & (sim.cov == 0)] = (226, 82, 72)
            pixels[GRID & (sim.cov > 0)] = (45, 153, 195)
            pic = Image.fromarray(pixels[::-1]).resize((660,390), Image.Resampling.NEAREST)
            x = 30 + col*720
            maps.paste(pic, (x, y+35))
            draw.text((x, y), f"{label} / {'Stock' if col==0 else 'Optimized'}  {sim.coverage()[0]*100:.1f}%", fill="#172b45", font=font(22))
    draw.text((30, 1465), "Optimized controller receives noisy yaw/sonar, no true position or coverage map.", fill="#40536a", font=font(18))
    draw.text((30, 1495), "Wall phases are planar approximations; this scores floor area, not wall cleaning or battery use.", fill="#40536a", font=font(18))
    maps.save(Path(out, "coverage_comparison.png"))
    curves = Image.new("RGB", (1440, 1230), "white")
    draw = ImageDraw.Draw(curves)
    draw.text((30, 15), "Floor area covered over time / paired simulation", fill="#172b45", font=font(30))
    draw.text((30, 60), f"Same assumptions as maps; mean of {len(histories['stock-floor'])} seeds. Blue stock / orange optimized.", fill="#40536a", font=font(20))
    times = np.arange(0, minutes*60+1, 10)
    for row, (label, stock, new) in enumerate(pairs):
        left, right, top, bottom = 100, 1390, 140+row*345, 400+row*345
        draw.text((left, top-35), label, fill="#172b45", font=font(22))
        for percent in (0,25,50,75,100):
            y = bottom - (bottom-top)*percent/100
            draw.line((left,y,right,y), fill="#dce3eb", width=1)
            draw.text((35,y-12), f"{percent}%", fill="#40536a", font=font(18))
        for minute in range(0, int(minutes)+1, 5):
            x = left+(right-left)*minute/minutes
            draw.text((x-12,bottom+8), str(minute), fill="#40536a", font=font(18))
        for name, color in ((stock,"#2565a0"),(new,"#d47715")):
            values = np.mean([np.interp(times, [t for t,c in history], [c for t,c in history])
                              for history in histories[name]], axis=0)
            points = [(left+(right-left)*t/(minutes*60), bottom-(bottom-top)*v) for t,v in zip(times,values)]
            draw.line(points, fill=color, width=4)
    draw.text((600, 1180), "Elapsed minutes", fill="#40536a", font=font(20))
    curves.save(Path(out, "coverage_curves.png"))


def scenario(job):
    global COMPILED
    width, velocity, drift, seeds, minutes, compiled, names, seed_start, settings = job
    COMPILED = compiled
    environment(*settings)
    configure({k: {"value": v, "source": "unmeasured sensitivity scenario"}
               for k, v in (("width_m", width), ("v70_m_s", velocity),
                            ("drift_deg_per_min", drift))})
    rows = []
    for seed in range(seed_start, seed_start + seeds):
        for name in names:
            pattern = PATTERNS[name]
            s = Sim(seed, minutes)
            pattern(s)
            rows.append({"pattern": name, "seed": seed, "width_m": width,
                         "speed_m_s": velocity, "drift_deg_min": drift,
                         "coverage": s.coverage()[0], "t95_s": s.time_to(.95),
                         "coverage20": next((c for t,c in s.history if t >= 1200),
                                            s.coverage()[0]) if minutes >= 20 else None,
                         "timeouts": getattr(s, "timeouts", 0), "simulated_s": s.t,
                         "pool": POOL, "start": START, "errors": ERROR_PROFILE})
    return rows


def matrix(args):
    """Paired seeds, including adverse drift and uncalibrated timing. No cherry-picking."""
    rows = []
    names = list(PATTERNS)
    if args.wall_variant == "a": names = names[:4]
    if args.wall_variant == "b": names = names[:2] + names[4:]
    jobs = [(w,v,d,args.seeds,args.minutes,COMPILED,names,args.seed_start,
             (args.pool, args.start, args.errors))
            for w in (.30,.40,.50) for v in (.20,.30,.40) for d in (.5,2.0)]
    if args.stress:
        profiles = [("kidney", "random", "nominal", .3, .2, 2.),
                    ("kidney", "edge", "nominal", .4, .3, .5),
                    ("rectangle", "center", "nominal", .4, .3, .5),
                    ("l-shape", "random", "nominal", .4, .3, .5),
                    ("island", "random", "nominal", .4, .3, .5),
                    ("kidney", "random", "adverse", .3, .2, 2.),
                    ("l-shape", "edge", "adverse", .3, .2, 2.),
                    ("island", "center", "adverse", .4, .3, 2.)]
        jobs = [(w,v,d,args.seeds,args.minutes,COMPILED,names,args.seed_start,(p,s,e))
                for p,s,e,w,v,d in profiles]
    from concurrent.futures import ProcessPoolExecutor
    with ProcessPoolExecutor(max_workers=args.workers) as pool:
        for job, result in zip(jobs, pool.map(scenario, jobs)):
            rows.extend(result)
            print(f"simulated {job[8]}, width={job[0]}, speed={job[1]}, drift={job[2]}", flush=True)
    config = load_tool("navcore").Core(*COMPILED).config
    metadata = {"minutes": args.minutes, "seeds": args.seeds, "seed_start": args.seed_start,
                "controller": "exact compiled nav_tick; uncalibrated compiled boot defaults",
                "controller_config": dict(zip(load_tool("crab_ctl").DIAG_NAV_CONFIG, config)),
                "compiled_addon_sha256": hashlib.sha256(COMPILED[0]).hexdigest(),
                "wall_variant": args.wall_variant,
                "suite": "stress" if args.stress else "matrix",
                "scenario_parameters": [{"pool": j[8][0], "start": j[8][1],
                    "errors": j[8][2], "width_m": j[0], "speed_m_s": j[1],
                    "drift_deg_min": j[2]} for j in jobs],
                "error_profiles": {k: dict(zip(("yaw_noise_sd_deg", "sonar_noise_sd_cm",
                    "sonar_dropout_probability", "minimum_motion_fraction"), v)) for k,v in ERRORS.items()},
                "simulator_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                "timing": "all primitives stop at requested budget; partial final motion; stock CTYPE 1 step timings as decoded (STEP 20 6 s twice per U-turn, STEP 0 7 s); optimized pays the vendor STEP 20 1 s stop per forward contact with its phase clock frozen (diag-2.7)",
                "assumptions": "assumed geometry and starts; no measured calibration; optimized noisy yaw/sonar, imperfect motion per error_profiles; heading response idealized; stock sonar/tilt triggers approximated with less noise/slip; wall phases are planar projections, B climb6s/hold6s/descent2s assumed; no wall area/energy/full vendor safety model",
                "rows": rows}
    Path(args.out, "matrix.json").write_text(json.dumps(metadata, indent=2) + "\n", encoding="utf-8")
    lines = [f"{args.minutes:g} minute paired runs, {args.seeds} seeds starting{args.seed_start}, {len(jobs)} unmeasured scenarios",
             metadata["assumptions"], ""]
    lines.extend(summarize(rows, names))
    if args.stress:
        for j in jobs:
            p,s,e = j[8]
            subset = [r for r in rows if (r["pool"],r["start"],r["errors"]) == (p,s,e)]
            lines.extend(["", f"{p} / {s} / {e}: width{j[0]} speed{j[1]} drift{j[2]}"])
            lines.extend(summarize(subset, names))
    summary = "\n".join(lines) + "\n"
    print(summary)
    Path(args.out, "matrix_summary.txt").write_text(summary, encoding="utf-8")


def summarize(rows, names):
    lines = []
    for stock, new in (("stock-floor", "optimized-floor"), ("stock-standard", "optimized-standard"),
                       ("stock-standard-b", "optimized-standard-b")):
        if stock not in names: continue
        before = [r["coverage"] for r in rows if r["pattern"] == stock]
        after = [r["coverage"] for r in rows if r["pattern"] == new]
        delta = [b-a for a,b in zip(before, after)]
        lines.append(f"{stock} mean {np.mean(before)*100:.1f}% vs {new} {np.mean(after)*100:.1f}%; "
                     f"difference range {min(delta)*100:+.1f}..{max(delta)*100:+.1f}pp; "
                     f"improved {sum(d>0 for d in delta)}/{len(delta)} paired cases")
        for name in (stock, new):
            runs = [r for r in rows if r["pattern"] == name]
            reached = [r["t95_s"]/60 for r in runs if r["t95_s"] is not None]
            lines.append(f"  {name}: 95% reached {len(reached)}/{len(runs)}; "
                         f"median among reaching runs {np.median(reached):.2f} min" if reached else
                         f"  {name}: no run reached95%")
        before_runs = [r for r in rows if r["pattern"] == stock]
        after_runs = [r for r in rows if r["pattern"] == new]
        paired = [(a["t95_s"], b["t95_s"]) for a,b in zip(before_runs,after_runs)
                  if a["t95_s"] is not None and b["t95_s"] is not None]
        lines.append(f"  among {len(paired)} pairs both reaching95%: optimized faster "
                     f"{sum(b<a for a,b in paired)}, slower {sum(b>a for a,b in paired)}")
    return lines


def main():
    global COMPILED
    p = argparse.ArgumentParser()
    p.add_argument("--minutes", type=float, default=60)
    p.add_argument("--seeds", type=int, default=5)
    p.add_argument("--seed-start", type=int, help="first paired seed (default0; stress default6 for fresh holdouts)")
    p.add_argument("--out", default="analysis/navsim")
    p.add_argument("--calibration", type=Path, help="schema-1 measurements with value/source provenance")
    suite = p.add_mutually_exclusive_group()
    suite.add_argument("--matrix", action="store_true", help="speed/width/drift sensitivity for both modes")
    suite.add_argument("--stress", action="store_true", help="eight shape/start/error scenarios, default holdout seeds6+")
    p.add_argument("--pool", choices=POOLS, default="kidney")
    p.add_argument("--start", choices=STARTS, default="random")
    p.add_argument("--errors", choices=tuple(ERRORS), default="nominal")
    p.add_argument("--workers", type=int, default=1, help="offline matrix processes (1..4)")
    p.add_argument("--wall-variant", choices=("a", "b", "both"), default="both")
    a = p.parse_args()
    if a.seed_start is None: a.seed_start = 6 if a.stress else 0
    if (not math.isfinite(a.minutes) or a.minutes <= 0 or a.seeds < 1 or
            a.seed_start < 0 or not 1 <= a.workers <= 4):
        p.error("minutes must be finite and positive; seeds >=1, seed-start >=0, workers1..4")
    if a.calibration and (a.matrix or a.stress):
        p.error("matrix/stress set their own assumed speed/width/drift; use calibration for a single scenario")
    if a.stress and (a.pool, a.start, a.errors) != ("kidney", "random", "nominal"):
        p.error("stress sets its own pool/start/errors; use those options for a single scenario or matrix")
    try:
        environment(a.pool, a.start, a.errors)
        configure(read_calibration(a.calibration) if a.calibration else {})
    except (OSError, ValueError, TypeError) as error:
        p.error(f"calibration: {error}")
    os.makedirs(a.out, exist_ok=True)
    if not Path(__file__).with_name("mkdiag.py").exists():
        p.error("the optimized controller's firmware source is not published yet, so this simulator cannot run")
    builder = load_tool("mkdiag")
    with tempfile.TemporaryDirectory(prefix="navsim-", dir=builder.ROOT / "build") as tmp:
        COMPILED = builder.compile_addon(Path(tmp), builder.ROOT / "tools/arm-gnu-toolchain-14.3.rel1/bin")
    if a.matrix or a.stress:
        matrix(a)
        return
    marks = (5, 10, 15, 20, 30, 45, 60)
    lines = [f"pool {POOL_CELLS * CELL * CELL:.1f} m^2 ({POOL}), start {START}, errors {ERROR_PROFILE}, robot {WIDTH} m wide, "
             f"{V70} m/s; {a.minutes:g} min runs, mean of {a.seeds} seeds",
             "geometry assumed; optimized controller sees noisy yaw/sonar only, no position/map",
             "parameters (unprovided values retain assumptions):",
             *[f"  {name}: {entry['value']} ({entry['source']})"
               for name, entry in model_parameters().items()],
             "  reverse_v50_m_s=None means forward-speed proportional model; "
             "other drive commands also scale proportionally",
             "", "coverage % after N minutes:",
             f"{'pattern':16}" + "".join(f"{m:>7}" for m in marks) +
             f"   {'t90':>6}{'t95':>6}{'t99':>6}  (minutes to reach 90/95/99 %)"]
    examples, histories = {}, {}
    for name, fn in PATTERNS.items():
        histories[name] = []
        rows, t90, t95, t99 = [], [], [], []
        for seed in range(a.seed_start, a.seed_start + a.seeds):
            s = Sim(seed, a.minutes)
            fn(s)
            histories[name].append(s.history)
            # Do not present the end-of-run coverage as a measurement at a future mark.
            rows.append([next((c for t, c in s.history if t >= m * 60), s.history[-1][1])
                         if m <= a.minutes else math.nan for m in marks])
            for lst, f in ((t90, .90), (t95, .95), (t99, .99)):
                lst.append(s.time_to(f))
            if seed == a.seed_start:
                examples[name] = s
                render(s, f"{name}, {a.minutes:.0f} min", os.path.join(a.out, f"{name}.png"))
        mean = np.mean(rows, axis=0) * 100

        def fmt(lst):
            ok = [v for v in lst if v is not None]
            return f"{np.mean(ok) / 60:6.1f}" if len(ok) == len(lst) else f"{'>' + str(int(a.minutes)):>6}"
        lines.append(f"{name:16}" + "".join(f"{v:7.1f}" if math.isfinite(v) else f"{'--':>7}" for v in mean) +
                     f"   {fmt(t90)}{fmt(t95)}{fmt(t99)}")
    txt = "\n".join(lines)
    print(txt)
    Path(a.out, "summary.txt").write_text(txt + "\n", encoding="utf-8")
    metadata = {"minutes": a.minutes, "seeds": a.seeds, "seed_start": a.seed_start,
                "compiled_addon_sha256": hashlib.sha256(COMPILED[0]).hexdigest(),
                "controller_config": dict(zip(load_tool("crab_ctl").DIAG_NAV_CONFIG,
                                               load_tool("navcore").Core(*COMPILED).config)),
                "parameters": model_parameters(),
                "environment": environment_parameters(),
                "simulator_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                "timing": "all primitives stop at requested budget; partial final motion; stock CTYPE 1 step timings as decoded (STEP 20 6 s twice per U-turn, STEP 0 7 s); optimized pays the vendor STEP 20 1 s stop per forward contact with its phase clock frozen (diag-2.7)",
                "pool_geometry": f"assumed {POOL} model",
                "position_and_boundary_knowledge": "optimized core: none; noisy synthetic yaw/sonar",
                "calibration_path": str(a.calibration) if a.calibration else None,
                "calibration_sha256": hashlib.sha256(a.calibration.read_bytes()).hexdigest()
                    if a.calibration else None}
    Path(a.out, "parameters.json").write_text(json.dumps(metadata, indent=2) + "\n", encoding="utf-8")
    comparison_visuals(examples, histories, a.minutes, a.out, a.seed_start)


if __name__ == "__main__":
    main()
