"""Offline measurement-quality checks; never connects to a robot."""
import importlib.util
import json
import struct
import io
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]


def load(name):
    spec = importlib.util.spec_from_file_location(name, ROOT / "tools" / (name + ".py"))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


r, s, m = load("navreport"), load("navsim"), load("navmap")


def samples(step=1):
    return [{**dict.fromkeys(r.FIELDS, 0), "seq": i, "t_ms": i*500, "ctype": 1, "csub": 1,
             "inwater": 1, "step": step, "sonar_cm": 400-i*15, "ltrack": 70,
             "rtrack": 70, "pump": 60, "batt": 80} for i in range(9)]


class RecordingTests(unittest.TestCase):
    def test_valid_straight_sonar_is_only_a_candidate(self):
        for step in (1, 5, 101):
            report = r.analyze(samples(step))
            self.assertAlmostEqual(report["sonar_candidates"][0]["sonar_closing_m_s"], .30)
            self.assertIn("not ground speed", report["sonar_candidates"][0]["classification"])
            self.assertIn("true ground speed / slip", report["unmeasured"])

    def test_noise_turns_no_echo_sparse_or_missing_samples_prevent_fit(self):
        for field, value in (("sonar_cm", 9999), ("yaw", 400), ("fault", 4),
                             ("step", 103), ("ltrack", -70), ("seq", 20)):
            data = samples()
            data[4][field] = value
            if field == "seq":
                for i in range(5, 9):
                    data[i]["seq"] += 16
            self.assertEqual(r.analyze(data)["sonar_candidates"], [])

    def test_wrap_is_valid_and_new_boot_is_refused(self):
        data = samples()
        for i, row in enumerate(data):
            row["t_ms"] = (0xFFFFFF00 + i*500) & 0xFFFFFFFF
        self.assertEqual(r.analyze(data)["observed_span_s"], 4)
        data[5]["t_ms"] = 0
        with self.assertRaises(ValueError):
            r.analyze(data)

    def test_dry_and_empty_results_never_infer_coverage(self):
        self.assertTrue(r.analyze([])["warnings"])
        data = samples()
        for row in data: row["inwater"] = 0
        report = r.analyze(data)
        self.assertEqual(report["floor_only_samples"], 0)
        self.assertEqual(report["sonar_candidates"], [])

    def test_jsonl_selection_and_bad_fields(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/"run.jsonl"
            path.write_text("\n".join(json.dumps({"event": "recorder", "samples": samples()})
                                      for _ in range(2)), encoding="utf-8")
            with self.assertRaises(ValueError): r.load_recording(path)
            self.assertEqual(len(r.load_recording(path, 2)), 9)
        for bad in (True, 65536):
            data = samples(); data[0]["sonar_cm"] = bad
            with self.assertRaises(ValueError): r.validate(data)


class CalibrationTests(unittest.TestCase):
    def test_provenance_is_required_and_reset_restores_defaults(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/"calibration.json"
            data = {"schema_version": 1, "measurements": {
                "v70_m_s": {"value": .25, "source": "example external measurement"}}}
            path.write_text(json.dumps(data), encoding="utf-8")
            s.configure(s.read_calibration(path))
            self.assertEqual(s.V70, .25)
            s.configure({})
            self.assertEqual(s.V70, .30)
            for bad in (0, True, float("nan")):
                data["measurements"]["v70_m_s"]["value"] = bad
                path.write_text(json.dumps(data), encoding="utf-8")
                with self.assertRaises(ValueError): s.read_calibration(path)
            data["measurements"]["v70_m_s"] = {"value": .3, "source": ""}
            path.write_text(json.dumps(data), encoding="utf-8")
            with self.assertRaises(ValueError): s.read_calibration(path)

    def test_comparison_excludes_perfect_map_controller(self):
        self.assertNotIn("proposed", s.PATTERNS)
        self.assertIn("optimized-floor", s.PATTERNS)
        self.assertIn("optimized-standard", s.PATTERNS)


class SimulationRobustnessTests(unittest.TestCase):
    def tearDown(self):
        s.environment()
        s.configure({})

    def test_geometry_scoring_and_starts_reset_without_controller_input(self):
        areas = []
        for shape in ("kidney", "rectangle", "l-shape", "island"):
            s.environment(shape, "edge", "adverse")
            areas.append(s.POOL_CELLS)
            for start in ("edge", "center", "random"):
                s.environment(shape, start, "adverse")
                a, b = s.Sim(6, 1), s.Sim(6, 1)
                self.assertTrue(s.in_pool(a.x, a.y))
                self.assertEqual((a.x,a.y,a.hdg), (b.x,b.y,b.hdg))
                self.assertEqual(a.cov.shape, s.GRID.shape)
                self.assertEqual(s.environment_parameters()["measured"], False)
        self.assertEqual(len(set(areas)), 4)
        s.environment()
        self.assertEqual((s.POOL, s.START, s.ERROR_PROFILE), ("kidney", "random", "nominal"))
        with self.assertRaises(ValueError): s.environment("unknown")
        with self.assertRaises(ValueError): s.environment(errors="unknown")

    def test_wait_turn_drive_and_patterns_never_exceed_deadline(self):
        s.environment("rectangle", "center")
        for action in (lambda r:r.wait(10), lambda r:r.turn(180), lambda r:r.drive(70, 10)):
            robot = s.Sim(6, .001)
            action(robot)
            self.assertEqual(robot.t, robot.budget)
            self.assertTrue(all(t <= robot.budget for t,_ in robot.history))
        for pattern in (s.stock_floor, s.stock_standard, s.stock_standard_b):
            robot = s.Sim(6, .01)
            pattern(robot)
            self.assertEqual(robot.t, .6)

    def test_partial_turn_and_drive_use_only_remaining_time(self):
        s.environment("rectangle", "center")
        robot = s.Sim(6, .001)
        robot.hdg = 0
        # Nominal 180deg turn takes4s including settling; only60ms is available.
        with patch.object(robot.rng, "gauss", return_value=0): robot.turn(180)
        self.assertAlmostEqual(robot.hdg, 2.7)
        robot = s.Sim(6, .001)
        robot.hdg = 0
        s.DRIFT_PER_MIN = 0
        x = robot.x
        robot.drive(70, 10)
        self.assertAlmostEqual(robot.x-x, .3*.06)

    def test_wall_phase_restores_budget_and_stops_exactly_at_boundary(self):
        robot = s.Sim(6, 45)
        s.wall_phase(robot, s.stock_bounce_b, .01, 1.5)
        self.assertEqual(robot.t, .6)
        self.assertEqual(robot.budget, 2700)
        def fail(*args, **kwargs): raise ValueError("fixture")
        with self.assertRaises(ValueError): s.wall_phase(robot, fail, 1, 1)
        self.assertEqual(robot.budget, 2700)

    def test_error_profiles_affect_only_synthetic_observations(self):
        s.environment(errors="adverse")
        robot = s.Sim(6, 1)
        with patch.object(robot.rng, "random", return_value=.1):
            self.assertEqual(s.observed_range(robot), 9999)
        s.environment(errors="nominal")
        with patch.object(robot.rng, "random", return_value=.1):
            self.assertNotEqual(s.observed_range(robot), 9999)
        self.assertLess(s.ERRORS["nominal"][0], s.ERRORS["adverse"][0])

    def test_sonar_reads_as_measured_in_the_pool(self):
        # Pool runs 3 and 4: nothing under 21 cm or past about 4.5 m, about 30 cm more than
        # the gap between the robot's front and the wall, often nothing at contact.
        s.environment("rectangle", "center")
        robot = s.Sim(6, 1)
        robot.y, robot.hdg = 3.0, 0                              # facing the wall at x = 10
        # the model steps in 5 cm: gaps of 77 and 3 cm read as 80 and 5
        for x, roll, expected in ((9.03, .9, 110), (9.77, .9, 35), (9.77, .1, 9999), (4.0, .9, 9999)):
            robot.x = x
            with patch.object(robot.rng, "random", return_value=roll), \
                    patch.object(robot.rng, "gauss", return_value=0):
                self.assertEqual(round(s.observed_range(robot)), expected, (x, roll))
        readings = []
        for x in [1.5 + k * .01 for k in range(830)]:
            robot.x = x
            readings.append(s.observed_range(robot))
        self.assertGreaterEqual(min(readings), s.SONAR_BLIND_CM)
        self.assertTrue(any(r == 9999 for r in readings) and any(r < 60 for r in readings))

    def test_nook_pool_and_start_reproduce_run_four(self):
        s.environment("nook", "nook")
        robot = s.Sim(3, 1)
        self.assertEqual((robot.x, robot.y), (9.0, 1.2))
        self.assertLess(abs((robot.hdg - 180 + 180) % 360 - 180), 20)
        self.assertTrue(s.in_pool(9.0, .6) and s.in_pool(9.0, 5.0) and not s.in_pool(6.0, 1.0))
        self.assertAlmostEqual(s.POOL_CELLS * s.CELL ** 2, 38.0, delta=1.0)
        with self.assertRaises(ValueError): s.environment("kidney", "nook")

    def test_failed_runs_are_counted_in_comparison(self):
        names = ["stock-floor", "optimized-floor"]
        rows = [{"pattern": names[0], "coverage": .96, "t95_s": 100},
                {"pattern": names[1], "coverage": .80, "t95_s": None}]
        summary = "\n".join(s.summarize(rows, names))
        self.assertIn("optimized-floor: no run reached95%", summary)
        self.assertIn("improved 0/1", summary)
        self.assertIn("among 0 pairs", summary)

    def test_stock_floor_timing_follows_the_decoded_steps(self):
        class Script:
            """Stand-in plant: lane runs answer from a script, everything else takes its time."""
            def __init__(self, runs):
                self.t, self.runs, self.log = 0.0, list(runs), []
            def done(self):
                return not self.runs
            def wait(self, seconds):
                self.t += seconds; self.log.append(("wait", seconds))
            def turn(self, deg):
                self.t += abs(deg) / 60 + 1; self.log.append(("turn", deg))
            def drive(self, drive, max_s):
                if drive == 70:
                    result, seconds = self.runs.pop(0)
                else:
                    result, seconds = "time", max_s
                self.t += seconds; self.log.append(("drive", drive, seconds))
                return result
        # One 20 s lane ending at a wall: STEP 20 (6 s), turn 90, 3.4 s shift, turn 76, STEP 20.
        robot = Script([("wall", 20.0)])
        s.stock_floor(robot)
        self.assertEqual(robot.log[:3], [("wait", 1.0), ("drive", -50, 5.0), ("wait", 1.0)])
        uturn = robot.log[4:]
        self.assertEqual([e[1] for e in uturn if e[0] == "turn"], [90, 76])
        self.assertAlmostEqual(robot.t - 7 - 20, 6 + 2.5 + 3.4 + 76/60 + 1 + 6)
        # Two 40 s timeouts: STEP 0 (7 s) each, then the next run is a wall without driving.
        robot = Script([("time", 40.0), ("time", 40.0), ("wall", 99.0)])
        s.stock_floor(robot)
        lanes = [i for i, e in enumerate(robot.log) if e[0] == "drive" and e[1] == 70]
        self.assertEqual(len(lanes), 3)
        self.assertEqual(sum(1 for e in robot.log if e == ("drive", -50, 5.0)), 3)
        self.assertTrue(lanes[1] < robot.log.index(("drive", -50, 3.0)) < lanes[2])
        # Three short lanes: the third U-turn's second turn carries the 180 flip (76+180).
        robot = Script([("wall", 1.0)] * 3)
        s.stock_floor(robot)
        self.assertEqual([e[1] for e in robot.log if e[0] == "turn"],
                         [90, 76, -90, -76, 90, -104])

    def test_cli_refuses_silently_ignored_calibration_and_stress_overrides(self):
        for argv in (("--matrix", "--calibration", "unused.json"),
                     ("--stress", "--calibration", "unused.json"),
                     ("--stress", "--pool", "island"),
                     ("--stress", "--errors", "adverse"),
                     ("--stress", "--start", "center")):
            with patch("sys.argv", ["navsim", *argv]), patch("sys.stderr", new_callable=io.StringIO):
                with self.assertRaises(SystemExit) as result: s.main()
                self.assertEqual(result.exception.code, 2)


@unittest.skipUnless((ROOT / "tools/mkdiag.py").exists(), "needs the firmware source, not published yet")
class CompiledCoreSimulationTests(unittest.TestCase):
    """navsim with the compiled diag-3.3 core and map (built in a temporary directory)."""
    @classmethod
    def setUpClass(cls):
        builder = load("mkdiag")
        with tempfile.TemporaryDirectory(prefix="navtools-", dir=ROOT / "build") as directory:
            cls.compiled = builder.compile_addon(Path(directory), ROOT / "tools/arm-gnu-toolchain-14.3.rel1/bin")

    def tearDown(self):
        s.environment()
        s.configure({})
        s.COMPILED, s.NAV_CONFIG, s.NAV_MAPPED = None, None, True

    def test_lanes_leave_the_nook_run_four_was_trapped_in(self):
        # Run 4's restart in the nook with run 4's speed and settings: the shift check ends
        # the side at the nook's back wall and the lanes reach the pool body.
        s.environment("nook", "nook")
        s.configure({"v70_m_s": {"value": .22, "source": "pool runs 3/4"},
                     "turn_rate_deg_s": {"value": 36.0, "source": "pool run 4"}})
        s.COMPILED, s.NAV_CONFIG = self.compiled, (1, 30, 23, 1000, 20, 10, 3, 1500)
        robot = s.Sim(0, 10)
        s.optimized_floor(robot)
        state = robot.core.state
        self.assertGreaterEqual(state[25], 2)                  # shifts the sonar found blocked
        body = s.GRID.copy()
        body[:int(2.2 / s.CELL), :] = False                    # the pool body, outside the nook
        self.assertGreater((robot.cov[body] > 0).mean(), .5)

    def test_compiled_map_follows_stock_primitives_too(self):
        # Wall phases and the timeout fallback are stock primitives: the map must move with them.
        s.environment("rectangle", "center")
        s.COMPILED = self.compiled
        robot = s.Sim(6, 5)
        core = s.nav_core(robot)
        before = m.parse_map(core.map_bytes())[0]["moving_ms"]
        robot.drive(70, 3.0)
        robot.wait(.5)
        header = m.parse_map(core.map_bytes())[0]
        self.assertGreater(header["moving_ms"], before + 2000)
        self.assertGreater(header["log_count"], 0)


class WaterTestCliTests(unittest.TestCase):
    """tools/watertest.py refuses bad invocations before any Bluetooth activity."""

    def test_refusals_happen_before_connecting(self):
        wt = load("watertest")
        self.assertEqual(wt.setting("spacing_cm=32"), ("spacing_cm", 32))
        for text in ("flags=1", "bogus=1", "spacing_cm", "spacing_cm=x"):
            with self.assertRaises(Exception):
                wt.setting(text)
        for argv in (("before", "--session", "x"), ("after", "--session", "x"),
                     ("--address", "AA", "before", "--session", "x", "--set", "bogus=1"),
                     ("--address", "AA", "before", "--session", "x", "--nav", "fast")):
            with patch("sys.argv", ["watertest", *argv]), patch("sys.stderr", new_callable=io.StringIO), \
                 patch.object(wt, "connected", side_effect=AssertionError("must not connect")):
                with self.assertRaises(SystemExit) as result:
                    wt.main()
                self.assertEqual(result.exception.code, 2)


class MapRenderTests(unittest.TestCase):
    """tools/navmap.py on synthetic maps in map layout 3 and layout 2 (no robot,
    no emulator)."""

    def blob(self, sonar_cm=0, expected_cm=0, cells=((64, 64), (65, 64)), pieces=(), **values):
        fields = dict.fromkeys(m.MAP_FIELDS, 0)
        fields.update(magic=m.MAP_MAGIC, version=m.MAP_VERSION, speed70=30, x_um=250000,
                      y_um=-100000, sonar_cm=sonar_cm, expected_cm=expected_cm, cells=len(cells),
                      moving_ms=42000, log_count=len(pieces),
                      contacts=sum(1 for p in pieces if p[4]), **values)
        data = bytearray(m.MAP_BYTES)
        m.MAP_HEADER.pack_into(data, 0, *(fields[k] for k in m.MAP_FIELDS))
        for x, y in cells:
            bit = y * m.MAP_DIM + x
            data[m.MAP_HEADER.size + bit // 8] |= 1 << (bit % 8)
        for k, piece in enumerate(pieces):
            m.PIECE.pack_into(data, m.MAP_HEADER.size + m.MAP_DIM * m.MAP_DIM // 8 + 8 * k, *piece)
        return bytes(data)

    def test_parse_scale_choice_contacts_and_render(self):
        pieces = ((0, 0, 1000, 70, 0), (110, 900, 500, 70, 1))     # 3 m along +x, 1.5 m along +y
        header, visited, parsed = m.parse_map(self.blob(pieces=pieces))
        self.assertEqual([p["contact"] for p in parsed], [False, True])
        info = m.summary(header, visited, parsed)
        self.assertEqual((info["scale"], info["contacts_cm"], info["drift_deg_min"]), (1.0, [(300, 150)], 0.0))
        self.assertIn("too short", info["drift"])                   # 42 s of motion
        sonar = m.summary(*m.parse_map(self.blob(400, 600, pieces=pieces)))
        self.assertEqual((sonar["scale"], sonar["last_position_m"]), (0.667, [2.0, 1.0]))
        self.assertEqual(m.summary(*m.parse_map(self.blob(400, 600, pieces=pieces)), speed_cm_s=24)["scale"], 0.8)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory, "map.png")
            m.render(sonar, path)
            self.assertGreater(path.stat().st_size, 500)
        # 16-bit decisecond start times wrap after 109 minutes; parsing unwraps them in order.
        wrapped = m.parse_map(self.blob(pieces=((65000, 0, 100, 70, 0), (200, 0, 100, 70, 0))))[2]
        self.assertEqual([p["t_s"] for p in wrapped], [6500.0, 6573.6])

    def test_drift_rate_is_recovered_from_repeated_sweeps(self):
        # A 6 m x 4 m pool swept along x, then y, then x again (30+ minutes); the recorded
        # heading carries a 1.5 deg/min gyro drift. Re-integration must find that rate.
        true_drift, pieces, t = 1.5, [], 0.0
        def lane(heading, length):
            nonlocal t
            dur = length / 0.30
            sensed = heading + true_drift * (t + dur / 2) / 60
            pieces.append({"t_s": t, "yaw": round(((sensed + 180) % 360 - 180) * 10), "dur_s": dur,
                           "drive": 70, "contact": True})
            t += dur + 3
        for repeat in range(3):
            for k in range(13):                         # lanes along x, 0.3 m apart
                lane(0 if k % 2 == 0 else 180, 6.0)
                lane(90, 0.3)
            for k in range(13):                         # back down
                lane(0 if k % 2 else 180, 6.0)
                lane(-90, 0.3)
            for k in range(20):                         # lanes along y
                lane(90 if k % 2 == 0 else -90, 3.9)
                lane(0, 0.3)
            for k in range(20):
                lane(90 if k % 2 else -90, 3.9)
                lane(180, 0.3)
        header = {"speed70": 30, "base_x_um": 0, "base_y_um": 0, "moving_ms": int(t * 1000)}
        best, gain = m.estimate_drift(header, pieces)
        self.assertLessEqual(abs(best - true_drift), 0.25)
        self.assertGreater(gain, m.DRIFT_MIN_GAIN)
        self.assertEqual(m.choose_drift(header, pieces)[0], best)

    def test_refuses_wrong_size_or_missing_map_and_matches_the_tool(self):
        with self.assertRaises(ValueError): m.parse_map(self.blob()[:-1])
        with self.assertRaises(ValueError): m.parse_map(bytes(m.MAP_BYTES))
        cc = load("crab_ctl")
        self.assertEqual((cc.MAP_ADDR, cc.MAP_BYTES, cc.MAP_MAGIC), (m.MAP_ADDR, m.MAP_BYTES, m.MAP_MAGIC))
        self.assertEqual(m.MAP_HEADER.size + m.MAP_DIM ** 2 // 8 + m.MAP_SEGMENTS * m.PIECE.size, m.MAP_BYTES)
        self.assertEqual(cc.MAP_VERSIONS, tuple(sorted(m.LAYOUTS)))
        for layout, fields, segments in m.LAYOUTS.values():
            self.assertEqual((len(fields), layout.size + m.MAP_DIM ** 2 // 8 + segments * m.PIECE.size),
                             (layout.size // 4, m.MAP_BYTES))

    def test_reports_uncounted_drive_and_still_reads_diag30_maps(self):
        pieces = ((0, 0, 1000, 70, 0),)
        info = m.summary(*m.parse_map(self.blob(pieces=pieces, stalled_ms=61000, dry_ms=25400,
                                                sloped_ms=9000, stalls=61, beyond_ms=30200)))
        self.assertEqual((info["not_counted_s"], info["stalls"]),
                         ({"no_progress": 61, "past_the_echo": 30, "out_of_water": 25, "tilted": 9}, 61))
        # A diag-3.0 map (layout 2: 144-byte header, 494-piece ring), e.g. the first pool run.
        layout, fields, segments = m.LAYOUTS[2]
        values = dict.fromkeys(fields, 0)
        values.update(magic=m.MAP_MAGIC, version=2, speed70=30, cells=0, log_count=segments + 2,
                      cur_drive=70, cur_ms=500, cur_t_ms=9000)
        data = bytearray(m.MAP_BYTES)
        layout.pack_into(data, 0, *(values[k] for k in fields))
        for k in range(segments):                  # pieces 0 and 1 were overwritten by 494, 495
            n = k + segments if k < 2 else k
            m.PIECE.pack_into(data, layout.size + m.MAP_DIM ** 2 // 8 + 8 * k, 10 * n, 0, 100, 70, 0)
        header, visited, parsed = m.parse_map(bytes(data))
        self.assertEqual((header["version"], len(parsed), parsed[0]["t_s"], parsed[-2]["t_s"], parsed[-1]["drive"]),
                         (2, segments + 1, 2.0, 495.0, 70))
        info = m.summary(header, visited, parsed)
        self.assertTrue(info["log_wrapped"])
        self.assertNotIn("not_counted_s", info)


if __name__ == "__main__":
    unittest.main(verbosity=2)
