"""The video tracker against a synthetic pool video with a known path; never connects to a robot.

The video has what a real one is expected to have: perspective, a low-contrast robot, ripples,
slow water patterns, glints as large as the robot and sudden clouds. It proves the method on those,
not on the real pool: only footage can do that.
"""
import csv
import importlib.util
import json
import math
from pathlib import Path
import tempfile
import unittest

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("videotrack", ROOT / "tools" / "videotrack.py")
v = importlib.util.module_from_spec(spec)
spec.loader.exec_module(v)

W, H, FPS, PPM = 640, 360, 10, 40
POOL = (9.0, 4.0)
IMAGE_POINTS = [[90, 70], [560, 60], [630, 340], [15, 345]]         # the corners, seen at an angle
POOL_POINTS = [[0, 0], [9, 0], [9, 4], [0, 4]]
SPEED = 0.22
GYRO_DRIFT = -2.5                                                   # deg/min
ROBOT_START_S = 5000.0                                              # robot uptime at video time zero


def planned_run(seconds=270, seed=4):
    """Lanes across the pool with the controller's rhythm: (phase, t0, t1, x0, y0, x1, y1, heading)."""
    rng = np.random.default_rng(seed)
    x, y, t, phases, sign = 0.8, 0.5, 8.0, [], 1
    while t < seconds:
        y1 = rng.uniform(2.8, 3.6) if sign > 0 else rng.uniform(0.4, 1.0)      # uneven, as real lanes are
        lane_t = abs(y1 - y) / SPEED
        phases.append((101, t, t + lane_t, x, y, x, y1, 90.0 * sign))
        t += lane_t
        y = y1
        phases.append((102, t, t + 1.0, x, y, x, y - sign * 0.2, 90.0 * sign))
        t += 1.0
        y -= sign * 0.2
        phases.append((103, t, t + 2.3, x, y, x, y, 0.0))
        t += 2.3
        phases.append((104, t, t + 1.5, x, y, x + 0.3, y, 0.0))
        t += 1.5
        x += 0.3
        sign = -sign
        phases.append((105, t, t + 2.3, x, y, x, y, 90.0 * sign))
        t += 2.3
    return phases


def position(phases, t):
    for _, t0, t1, x0, y0, x1, y1, _ in phases:
        if t0 <= t < t1:
            k = (t - t0) / (t1 - t0)
            return x0 + k * (x1 - x0), y0 + k * (y1 - y0)
    return (phases[0][3], phases[0][4]) if t < phases[0][1] else (phases[-1][5], phases[-1][6])


def write_video(path, phases, seconds):
    rng = np.random.default_rng(1)
    writer = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*"MJPG"), FPS, (W, H))
    if not writer.isOpened():
        raise unittest.SkipTest("no MJPG writer in this OpenCV")
    size = int(POOL[0] * PPM), int(POOL[1] * PPM)
    floor = np.full((size[1], size[0], 3), (205, 190, 140), np.float32)
    floor += cv2.GaussianBlur(rng.normal(0, 30, (size[1], size[0])).astype(np.float32), (0, 0), 6)[..., None]
    gx, gy = np.meshgrid(np.arange(size[0]) / PPM, np.arange(size[1]) / PPM)
    warp = cv2.getPerspectiveTransform(np.array(POOL_POINTS, np.float32) * PPM, np.array(IMAGE_POINTS, np.float32))
    half = int(0.21 * PPM)
    for k in range(int(seconds * FPS)):
        t = k / FPS
        ripple = 15 * np.sin(2 * math.pi * (gx / 0.27 + t / 1.3)) * np.sin(2 * math.pi * (gy / 0.31 - t / 1.7))
        water = 5 * np.sin(2 * math.pi * (gx / 2.1 + t / 9)) * np.sin(2 * math.pi * (gy / 1.7 - t / 11))
        image = floor + (ripple + water)[..., None] + (20 if t % 80 > 40 else 0)
        x, y = position(phases, t)
        cx, cy = int(x * PPM), int(y * PPM)
        image[max(cy - half, 0):cy + half, max(cx - half, 0):cx + half] -= 20
        for _ in range(5):
            gx0, gy0 = int(rng.integers(0, size[0] - 14)), int(rng.integers(0, size[1] - 14))
            image[gy0:gy0 + 12, gx0:gx0 + 12] = 255
        writer.write(cv2.warpPerspective(np.clip(image, 0, 255).astype(np.uint8), warp, (W, H), borderValue=(90, 110, 90)))
    writer.release()


def write_recorder(session, phases):
    session.mkdir(parents=True)
    fields = ("seq,t_ms,yaw,pitch,roll,sonar_cm,ltrack,rtrack,pump,ctype,csub,state,step,inwater,"
              "fault,batt,pins,guard,bump").split(",")
    with open(session / "recorder.csv", "w", newline="") as f:
        writer = csv.DictWriter(f, fields, lineterminator="\n")
        writer.writeheader()
        for seq, (phase, t0, _, _, _, _, _, heading) in enumerate(phases):
            row = dict.fromkeys(fields, 0)
            row.update(seq=seq, t_ms=int((ROBOT_START_S + t0) * 1000), yaw=int((heading + GYRO_DRIFT * t0 / 60) * 10),
                       sonar_cm=9999, step=phase, inwater=1, csub=1, ctype=1)
            writer.writerow(row)


class VideoTrackTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.dir = tempfile.TemporaryDirectory()
        root = Path(cls.dir.name)
        cls.seconds = 280
        cls.phases = planned_run(cls.seconds - 10)
        cls.video = root / "pool.avi"
        write_video(cls.video, cls.phases, cls.seconds)
        cls.session = root / "session"
        write_recorder(cls.session, cls.phases)
        cls.calib = root / "calib.json"
        cls.calib.write_text(json.dumps({"image_points": IMAGE_POINTS, "pool_points_m": POOL_POINTS}))
        cls.cal = v.load_calibration(cls.calib)
        cls.out = root / "out"
        cls.summary, cls.chain = v.track([cls.video], cls.cal, cls.out, log=lambda *_: None)

    @classmethod
    def tearDownClass(cls):
        cls.dir.cleanup()

    def test_track_follows_the_known_path(self):
        self.assertGreater(self.summary["tracked_percent"], 90)
        errors = [math.hypot(x - position(self.phases, t)[0], y - position(self.phases, t)[1])
                  for t, x, y, _ in self.chain]
        self.assertLess(float(np.sqrt(np.mean(np.square(errors)))), 0.12)
        self.assertLess(max(errors), 0.6)

    def test_coverage_matches_the_true_swath(self):
        truth = [(t, *position(self.phases, t), 0.0) for t in np.arange(0, self.seconds, 0.5)]
        expected, *_ = v.coverage(truth, self.cal)
        self.assertAlmostEqual(self.summary["covered_percent"], expected["covered_percent"], delta=4)
        self.assertAlmostEqual(self.summary["pool_m2"], 36.0, delta=1.0)       # edge cells are drawn whole
        self.assertLess(self.summary["covered_percent"], 60)    # the run is short: this is not a full pass

    def test_a_glint_chain_loses_to_the_robot(self):
        rng = np.random.default_rng(2)                                  # glints: stronger, never twice in one place
        robot = [(float(t), np.array([[1.0 + 0.1 * t, 2.0, 12.0], [*rng.uniform((0, 0), (9, 4)), 40.0]])) for t in range(30)]
        chain = v.best_chain(robot)
        self.assertEqual(len(chain), 30)
        self.assertGreaterEqual(sum(abs(y - 2.0) < 1e-6 for _, _, y, _ in chain), 28)

    def test_compare_recovers_offset_speed_drift_and_shifts(self):
        out = Path(self.dir.name) / "compare"
        result = v.compare(self.out / "track.csv", self.session, out, configured_cm_s=23)
        self.assertAlmostEqual(result["offset_s"], -ROBOT_START_S, delta=1.5)
        self.assertGreater(result["correlation"], 0.5)
        self.assertGreater(result["correlation"], result["best_rival"])
        self.assertAlmostEqual(result["median_speed_cm_s"], SPEED * 100, delta=2.5)
        self.assertAlmostEqual(result["map_scale"], SPEED * 100 / 23, delta=0.1)
        self.assertAlmostEqual(result["heading"]["drift_deg_min"], GYRO_DRIFT, delta=0.6)
        self.assertGreater(result["heading"]["agreement"], 0.9)
        self.assertAlmostEqual(result["shift_moved_m_when_sonar_says_free"], 0.3, delta=0.12)
        self.assertTrue((out / "compare.md").exists() and (out / "lanes.csv").exists())

    def test_outputs_exist_for_a_person_to_check(self):
        for name in ("track.csv", "coverage.json", "coverage.png", "overlay.jpg", "checks.jpg"):
            self.assertGreater((self.out / name).stat().st_size, 0, name)

    def test_frame_writes_the_grid_and_top_down_pictures(self):
        out = Path(self.dir.name) / "frame.png"
        self.assertEqual(v.save_frame([self.video], 3.0, out, self.calib), (W, H))
        self.assertTrue(out.with_name("frame_grid.png").exists() and out.with_name("frame_topdown.png").exists())

    def test_calibration_maps_the_points_it_was_given(self):
        for image, pool in zip(IMAGE_POINTS, POOL_POINTS):
            pixel = cv2.perspectiveTransform(np.array([[image]], np.float64), self.cal.to_m).reshape(2)
            self.assertAlmostEqual(float(pixel[0]), pool[0], delta=0.01)
            self.assertAlmostEqual(float(pixel[1]), pool[1], delta=0.01)
        with self.assertRaises(SystemExit):
            v.Calibration({"image_points": IMAGE_POINTS[:3], "pool_points_m": POOL_POINTS[:3]})

    def test_a_hidden_robot_leaves_a_gap_not_a_path_through_the_noise(self):
        rng = np.random.default_rng(3)
        found = []
        for k in range(600):
            t = k * 0.5
            hidden = 100 <= t < 200
            nodes = [rng.uniform((0, 0, 3), (9, 4, 8), (3, 3))] if hidden else \
                [np.array([[1.0 + 0.01 * k, 2.0, 40.0]]), rng.uniform((0, 0, 3), (9, 4, 8), (2, 3))]    # run 8: noise tops out near 6
            found.append((t, np.vstack(nodes)))
        chain = v.best_chain(found)
        self.assertFalse([c for c in chain if 101 <= c[0] < 199], "the chain followed noise through the hidden part")
        self.assertGreater(len(chain), 390)
        self.assertLess(max(abs(c[2] - 2.0) for c in chain), 1e-6)

    def test_a_steady_spot_loses_to_the_robot(self):
        found = [(k * 0.5, np.array([[7.0, 1.0, 50.0], [1.0 + 0.11 * k, 2.0, 30.0]])) for k in range(60)]
        chain = v.best_chain(v.drop_steady_spots(found))
        self.assertGreater(len(chain), 50)
        self.assertLess(max(abs(c[2] - 2.0) for c in chain), 1e-6)

    def test_a_rippling_spot_needs_a_stronger_signal_than_calm_water(self):
        cal = v.Calibration({"image_points": [[0, 0], [160, 0], [160, 160], [0, 160]], "pool_points_m": [[0, 0], [4, 0], [4, 4], [0, 4]],
                             "mask_margin_m": 0.0})
        background = np.full((cal.size[1], cal.size[0], 3), 120, np.float32)
        noise = np.full(background.shape[:2], 1.0, np.float32)
        noise[:cal.ppm * 2, :] = 12.0                                   # the far half always scores high: ripples
        half = int(0.2 * cal.ppm)

        def peak(x_m, y_m):
            image = background.copy()
            cx, cy = (int(p) for p in cal.canvas_px((x_m, y_m)))
            image[cy - half:cy + half, cx - half:cx + half] -= 30
            found = v.candidates(image, background, noise, cal, cal.pool_mask(0.0))
            near = found[np.hypot(found[:, 0] - x_m, found[:, 1] - y_m) < 0.3] if len(found) else found
            return float(near[:, 2].max()) if len(near) else 0.0

        self.assertGreater(peak(1.0, 3.0), 3 * max(peak(1.0, 1.0), 1.0))

    def test_short_weak_fragments_are_dropped(self):
        robot = [(k * 0.5, 1.0 + 0.1 * k, 2.0, 30.0) for k in range(40)]
        fragment = [(40.0 + k * 0.5, 5.0, 3.0, 12.0) for k in range(4)]
        strong = [(60.0 + k * 0.5, 2.0, 1.0, 40.0) for k in range(4)]
        kept = v.drop_fragments(robot + fragment + strong, 30.0)
        self.assertEqual(len(kept), 44)
        self.assertFalse([c for c in kept if c[3] == 12.0])

    def test_saturated_water_and_excluded_parts_give_no_peaks(self):
        data = {"image_points": [[0, 0], [160, 0], [160, 160], [0, 160]], "pool_points_m": [[0, 0], [4, 0], [4, 4], [0, 4]],
                "exclude_image": [[[100, 100], [140, 100], [140, 140], [100, 140]]], "mask_margin_m": 0.0}
        cal = v.Calibration(data)
        rng = np.random.default_rng(5)
        background = np.full((cal.size[1], cal.size[0], 3), 120, np.float32)
        noise = rng.normal(0, 2, background.shape).astype(np.float32)
        background[:cal.ppm * 2, :, 0] = 250                            # the half with sky on the water
        frame = background + noise
        half = int(0.2 * cal.ppm)

        def spot(x_m, y_m):
            image = frame.copy()
            cx, cy = (int(p) for p in cal.canvas_px((x_m, y_m)))
            image[cy - half:cy + half, cx - half:cx + half] -= 60
            return v.candidates(image, background, np.full(background.shape[:2], 1.0, np.float32), cal, cal.pool_mask(0.0))

        def near(peaks, x_m, y_m):
            return len(peaks) and np.hypot(peaks[:, 0] - x_m, peaks[:, 1] - y_m).min() < 0.3

        self.assertTrue(near(spot(1.0, 3.0), 1.0, 3.0))                 # open water: found
        self.assertFalse(near(spot(1.0, 0.8), 1.0, 0.8))                # in the glare: not a peak
        self.assertFalse(near(spot(3.0, 3.0), 3.0, 3.0))                # under the frond: not a peak

    def test_lens_correction_round_trips_and_straightens(self):
        k1 = -0.15
        cal = v.Calibration({"image_size": [640, 360], "k1": k1, "homography_image_to_m": np.linalg.inv(
            cv2.getPerspectiveTransform(np.array(POOL_POINTS, np.float32), np.array(IMAGE_POINTS, np.float32))).tolist(),
            "pool_outline_image": IMAGE_POINTS})
        floor = np.array([[1.0, 1.0], [4.0, 2.0], [8.0, 3.5]])
        seen = cal.seen(floor)
        self.assertLess(np.abs(cal.to_pool(seen) - floor).max(), 1e-3)
        line = np.column_stack((np.linspace(0.5, 8.5, 9), np.full(9, 0.3)))     # straight on the floor, curved by the lens
        curved = cal.seen(line)
        chord = (curved[-1] - curved[0]) / np.hypot(*(curved[-1] - curved[0]))
        bow = np.abs((curved - curved[0]) @ np.array((-chord[1], chord[0]))).max()
        self.assertGreater(bow, 3)                                      # pixels: the lens really bends it
        self.assertLess(np.abs(cal.to_pool(curved)[:, 1] - 0.3).max(), 1e-3)

    def test_calibrate_recovers_a_wide_camera_from_lanes_alone(self):
        truth = (1150.0, 40.0, -4.0, 2.0, -0.3)
        drift, pan, speed = -2.3, 118.0, 0.22
        centre, corner2 = np.array((960.0, 540.0)), 960.0 ** 2 + 540.0 ** 2
        corners = np.array(((100, 300), (1820, 300), (1820, 1000), (100, 1000)), np.float64)
        d = corners - centre
        ideal = d / (1 + truth[4] * (d * d).sum(axis=1, keepdims=True) / corner2) + centre
        floor = v.camera_floor(corners, truth, centre, corner2)
        to_m, _ = cv2.findHomography(ideal.astype(np.float32), floor.astype(np.float32), 0)
        outline = [[100, 300], [1820, 300], [1820, 1000], [100, 1000]]
        camera = v.Calibration({"homography_image_to_m": to_m.tolist(), "k1": truth[4], "pool_outline_image": outline})
        rng = np.random.default_rng(6)
        root = Path(self.dir.name) / "fit"
        session = root / "session"
        session.mkdir(parents=True)
        fields = ("seq,t_ms,yaw,pitch,roll,sonar_cm,ltrack,rtrack,pump,ctype,csub,state,step,inwater,"
                  "fault,batt,pins,guard,bump").split(",")
        low, high = floor.min(axis=0) + 0.6, floor.max(axis=0) - 0.6
        where, t, rows, track = rng.uniform(low, high), 1000.0, [], []
        seq = 0
        for lane in range(70):
            heading = rng.uniform(0, 360)
            step = speed * np.array((math.cos(math.radians(heading)), math.sin(math.radians(heading))))
            seconds = rng.uniform(8, 18)
            end = where + step * seconds
            if not (low[0] <= end[0] <= high[0] and low[1] <= end[1] <= high[1]):
                step, end = -step, where - step * seconds
                heading += 180
                if not (low[0] <= end[0] <= high[0] and low[1] <= end[1] <= high[1]):
                    where = rng.uniform(low, high)
                    continue
            yaw = heading - pan + drift * (t - 1000.0) / 60
            for k, phase in ((0, 101), (seconds, 103)):
                row = dict.fromkeys(fields, 0)
                row.update(seq=seq, t_ms=int((t + k) * 1000), yaw=int(((yaw + 180) % 360 - 180) * 10), sonar_cm=9999, step=phase, inwater=1)
                rows.append(row)
                seq += 1
            for k in np.arange(0, seconds, 0.5):
                track.append((t + k, *(where + step * k)))
            where, t = end, t + seconds + 3
        with open(session / "recorder.csv", "w", newline="") as f:
            writer = csv.DictWriter(f, fields, lineterminator="\n")
            writer.writeheader()
            writer.writerows(rows)
        track = np.array(track)
        pixels = camera.seen(track[:, 1:3]) + rng.normal(0, 4, (len(track), 2))     # pixels of tracker noise
        points = [[300, 300], [1600, 300], [1600, 1000], [300, 1000]]
        rough = root / "rough.json"
        rough.write_text(json.dumps({"image_points": points, "pool_points_m": [[0, 0], [6, 0], [6, 5], [0, 5]],
                                     "pool_outline_image": outline, "mask_margin_m": 0.1}))
        rough_cal = v.load_calibration(rough)
        with open(root / "track.csv", "w", newline="") as f:
            writer = csv.writer(f, lineterminator="\n")
            writer.writerow(("t_s", "x_m", "y_m", "z"))
            for (time, *_), metres in zip(track, rough_cal.to_pool(pixels)):
                writer.writerow((f"{time:.2f}", f"{metres[0]:.4f}", f"{metres[1]:.4f}", "30"))
        data = v.calibrate(root / "track.csv", rough, session, root / "fitted.json", offset=0.0)
        self.assertAlmostEqual(data["fit"]["gyro_drift_deg_min"], drift, delta=0.4)
        self.assertLess(data["fit"]["median_residual_m"], 0.1)
        fitted = v.Calibration(data)
        a = rng.uniform(low, high, (200, 2))
        b = rng.uniform(low, high, (200, 2))
        true_m = np.hypot(*(a - b).T)
        seen_m = np.hypot(*(fitted.to_pool(camera.seen(a)) - fitted.to_pool(camera.seen(b))).T)
        far_apart = true_m > 1.0
        self.assertLess(float(np.abs(seen_m[far_apart] / true_m[far_apart] - 1).mean()), 0.06)
        left, right = fitted.to_pool(camera.seen(np.array(((-1.0, 3.0), (1.0, 3.0)))))
        self.assertGreater(right[0], left[0])                           # map right is the camera's right, not mirrored

    def test_several_files_play_as_one(self):
        seen = [t for t, _ in v.video_frames([self.video, self.video], 20.0)]
        self.assertGreater(seen[-1], self.seconds)
        self.assertTrue(all(b > a for a, b in zip(seen, seen[1:])))


if __name__ == "__main__":
    unittest.main()
