#!/usr/bin/env python3
"""watertest - one supervised water run with the diagnostic build, in two short connections.

    python -I tools/watertest.py --address AA:BB:CC:DD:EE:FF before --session analysis/water/run1 --nav floor
    (disconnect happens automatically; put the robot in the water, let it clean, take it out,
     keep it switched on)
    python -I tools/watertest.py --address AA:BB:CC:DD:EE:FF after --session analysis/water/run1
    python -I tools/watertest.py report --session analysis/water/run1      # offline, any time

before: one Bluetooth connection. Reads the add-on status, the app settings and (diag-2.8+)
the navigation status; optionally sets the volatile navigation mode/settings (FC4B, reset at
every boot); arms the recorder (cleared, on-change by default). Saves before.json.
after: one connection. Checks it is still the same boot (otherwise RAM, the recorder and the
map are gone), reads the robot's heading first (yaw is zero where the cycle started, so with
the robot set down facing the way it went in this is the heading drift of the whole run),
stops the recorder, downloads it, the navigation status and (diag-3.0+) the map with its path
log. Saves the files, then writes the offline report.
report: recorder report (navreport), map picture with drift correction (navmap), summary.md.

Never writes flash, firmware or the battery floor, never resets the robot. Changing the app's
cleaning mode is a vendor setting (the same as the app): only with --mode.
"""
import argparse
import asyncio
import csv
import datetime as dt
import hashlib
import importlib.util
import io
import json
from pathlib import Path
import struct
import sys


def load(name):
    spec = importlib.util.spec_from_file_location(name, Path(__file__).with_name(name + ".py"))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


cc = load("crab_ctl")
# +4 (diag-3.3): the optimized lanes without the map's memory (no scans or new-floor side ends)
NAV_FLAGS = {"stock": 0, "floor": 1, "standard": 2, "both": 3,
             "floor-nomemory": 5, "standard-nomemory": 6, "both-nomemory": 7}
APP_MODES = {2: "Floor", 3: "Walls", 4: "Waterline", 7: "Standard"}
MODE_FOR = {"floor": 2, "standard": 7}
ATTITUDE = 0x200003F8                  # vendor roll, pitch, yaw: int32 in 0.1 deg
FIND_TRIES = 3                          # a weak advertiser can miss one 10 s scan
STATUS_KEEP = ("build_id", "protocol", "sram32k", "boot_count", "ms", "faults", "probe_faults",
               "ring_cap", "seq", "period_ms", "rec_flags", "emergency_count", "battery_floor",
               "floor_tripped", "floor_saved", "bpr19_at_boot")


def stamp():
    return dt.datetime.now().astimezone().isoformat(timespec="seconds")


def save(path, data):
    path.write_bytes((json.dumps(data, indent=2) + "\n").encode("utf-8"))


async def app_settings(crab):
    out = {}
    for name in ("fw_version", "battery", "mode", "speed", "cycle_time"):
        try:
            out[name] = await crab.get(name)
        except Exception as error:                 # one missing register must not stop the run
            out[name] = f"error: {error}"
    return out


async def nav_status(crab):
    """FC4C state, or None on builds without the layout-2 navigation extension."""
    try:
        return await crab.diag(cc.diag_nav_status_frame())
    except ValueError:
        return None


async def attitude(crab):
    roll, pitch, yaw = struct.unpack("<3i", await crab.diag_read(ATTITUDE, 12, cc.DIAG_MAX_READ))
    return {"yaw_deg": yaw / 10, "pitch_deg": pitch / 10, "roll_deg": roll / 10}


async def map_bytes(crab, sram32k):
    """The map of the last cycle, or None (older build, no cycle since power-on, 16 KB chip)."""
    if not sram32k:
        return None
    try:
        head = await crab.diag_read(cc.MAP_ADDR, 8, cc.DIAG_MAX_READ)
    except cc.DiagException:
        return None
    version = int.from_bytes(head[4:8], "little")
    if int.from_bytes(head[:4], "little") != cc.MAP_MAGIC or version not in cc.MAP_VERSIONS:
        return None
    return await crab.diag_read(cc.MAP_ADDR, cc.MAP_BYTES, cc.DIAG_MAX_READ)


RESET_FLAGS = 0x40021024        # CRM_CTRLSTS: sticky reset-cause flags, bits 26..31
RESET_NAMES = ((26, "pin"), (27, "power-on"), (28, "software"), (29, "watchdog"),
               (30, "window-watchdog"), (31, "low-power"))


async def reset_flags(crab):
    """The chip's reset-cause flags, decoded. They accumulate from power-on and the vendor app
    never clears them; the bootloader's iwdg_init (0x0800103C) clears them all when, and only
    when, the watchdog caused the reset it is booting from. So "power-on" missing from the set
    means a watchdog reset since the robot was switched on."""
    value = int.from_bytes(await crab.diag_read(RESET_FLAGS, 4, cc.DIAG_MAX_READ), "little")
    names = [name for bit, name in RESET_NAMES if value >> bit & 1]
    return {"value": f"0x{value:08X}", "set": names,
            "watchdog_since_power_on": "power-on" not in names}


async def before(crab, session, nav=None, settings=(), mode=None, period_ms=0, on_change=True,
                 stop_full=False, log=print):
    session.mkdir(parents=True, exist_ok=True)
    if (session / "before.json").exists():
        raise ValueError(f"{session} already has a before.json: use a new session folder")
    status = await crab.diag(cc.diag_status_frame())
    await crab.diag_learn_rc_count()
    record = {"time": stamp(), "status": {k: status[k] for k in STATUS_KEEP}, "warnings": []}
    record["reset_flags"] = await reset_flags(crab)
    navigation = await nav_status(crab)
    if nav is not None or settings:
        if navigation is None:
            raise ValueError("this build has no navigation extension (needs diag-2.8 or later)")
        for name, value in settings:
            await crab.diag(cc.diag_nav_frame(name, value))
        if nav is not None:
            await crab.diag(cc.diag_nav_frame("flags", NAV_FLAGS[nav]))
        navigation = await nav_status(crab)
    if mode is not None:
        await crab.set("mode", MODE_FOR[mode])     # vendor setting, like choosing it in the app
    record["app"] = await app_settings(crab)
    record["navigation"] = navigation
    flags = cc.REC_CLEAR | (cc.REC_ONCHANGE if on_change else 0) | (cc.REC_STOPFULL if stop_full else 0)
    await crab.diag(cc.diag_control_frame(period_ms, flags))
    armed = await crab.diag(cc.diag_status_frame())
    record["recorder"] = {"period_ms": armed["period_ms"], "flags": armed["rec_flags"],
                          "capacity": armed["ring_cap"], "seq": armed["seq"]}
    if armed["period_ms"] != period_ms or armed["rec_flags"] != flags & ~cc.REC_CLEAR:
        raise ValueError(f"recorder did not arm as requested: {record['recorder']}")
    mode_now = record["app"].get("mode")
    if navigation and navigation["flags"] & 3:
        wanted = {1: (2,), 2: (7,), 3: (2, 7)}[navigation["flags"] & 3]
        if mode_now not in wanted:
            record["warnings"].append(
                f"app mode is {APP_MODES.get(mode_now, mode_now)}: the optimized program needs "
                + " or ".join(APP_MODES[m] for m in wanted) + " (choose it in the app, or --mode)")
    if status["faults"] or status["probe_faults"]:
        record["warnings"].append(f"add-on reports faults {status['faults']} / probe faults {status['probe_faults']}")
    save(session / "before.json", record)
    log(f"{session}: armed {status['build_id']}, recorder {record['recorder']['capacity']} samples "
        f"({'on change' if on_change else 'periodic'}), navigation flags "
        f"{navigation['flags'] if navigation else 'n/a'}, app mode {APP_MODES.get(mode_now, mode_now)}, "
        f"battery {record['app'].get('battery')} %")
    for warning in record["warnings"]:
        log(f"WARNING: {warning}")
    log("Now: put the robot in the water with its back to the wall; it starts on its own\n"
        "(Bluetooth is disconnected). Do not switch it off on the way: the navigation setting\n"
        "and the recorder live in RAM and a restart clears them. Note which way it faces.\n"
        "Optional, for calibration: time one straight run between two marks of known distance.\n"
        "Afterwards: take it out, keep it switched on (no button press, no charger), set it\n"
        "down level facing the same way it faced going in, within reach, and within 30 minutes\n"
        "run `after` with the same --session.")
    return record


def restart_since(first, status, at):
    """Why the add-on's RAM at `after` is not the one `before` armed, or None. A restart soon
    after `before` leaves a longer uptime at `after` (seen on a real run), so the recorder that
    `before` armed counts too: only a restart turns it off."""
    if status["build_id"] != first["status"]["build_id"]:
        return "the build changed"
    if status["ms"] < first["status"]["ms"] or (first.get("recorder", {}).get("flags")
                                               and not status["rec_flags"]):
        power_on = dt.datetime.fromisoformat(at) - dt.timedelta(milliseconds=status["ms"])
        later = (power_on - dt.datetime.fromisoformat(first["time"])).total_seconds()
        return (f"powered on at {power_on:%H:%M:%S}"
                + (f", {later:.0f} s after `before`" if later >= 0 else ""))
    return None


async def after(crab, session, log=print):
    first = json.loads((session / "before.json").read_text(encoding="utf-8"))
    if (session / "after.json").exists():
        raise ValueError(f"{session} already has an after.json")
    status = await crab.diag(cc.diag_status_frame())
    await crab.diag_learn_rc_count()
    record = {"time": stamp(), "status": {k: status[k] for k in STATUS_KEEP}, "warnings": []}
    record["attitude"] = await attitude(crab)      # first, while it sits as it was set down
    record["reset_flags"] = await reset_flags(crab)
    restart = restart_since(first, status, record["time"])
    same_boot = restart is None
    record["same_boot"] = same_boot
    if not same_boot:
        record["warnings"].append(f"the robot restarted since `before` ({restart}): the recorder "
                                  "and navigation settings were lost (RAM); a map is of the "
                                  "cycle after the restart")
        flags = record["reset_flags"]
        record["warnings"].append(
            f"reset flags {flags['value']} ({', '.join(flags['set']) or 'none'}): "
            + ("the power-on flag is gone, so the bootloader cleared them: a WATCHDOG reset"
               if flags["watchdog_since_power_on"] else
               "the power-on flag is still there: not a watchdog reset"
               + (" (a software reset is on record)" if "software" in flags["set"] else "")))
    await crab.diag(cc.diag_control_frame(0, 0))   # stop recording, keep the samples
    samples = await crab.diag_recorder(0) if same_boot else []
    record["recorder"] = {"samples": len(samples), "written": status["seq"],
                          "capacity": status["ring_cap"]}
    if same_boot and status["seq"] > status["ring_cap"]:
        record["warnings"].append(f"recorder overflowed: {status['seq']} written, the oldest "
                                  f"{status['seq'] - status['ring_cap']} were overwritten")
    record["navigation"] = await nav_status(crab)
    data = await map_bytes(crab, status["sram32k"])
    record["map"] = {"bytes": len(data) if data else 0,
                     "sha256": hashlib.sha256(data).hexdigest() if data else None}
    if same_boot and data is None:
        record["warnings"].append("no map (needs diag-3.0 and a cleaning cycle since power-on)")
    record["app"] = await app_settings(crab)
    if samples:
        text = io.StringIO()
        writer = csv.writer(text, lineterminator="\n")
        writer.writerow(("seq",) + cc.DIAG_SAMPLE_FIELDS)
        writer.writerows([s[k] for k in ("seq",) + cc.DIAG_SAMPLE_FIELDS] for s in samples)
        (session / "recorder.csv").write_bytes(text.getvalue().encode("utf-8"))
    if data:
        (session / "map.bin").write_bytes(data)
    save(session / "after.json", record)
    log(f"{session}: {len(samples)} recorder samples, map {'saved' if data else 'missing'}, "
        f"battery {record['app'].get('battery')} %")
    for warning in record["warnings"]:
        log(f"WARNING: {warning}")
    return record


def report(session, speed_cm_s=None, drift=None, log=print):
    """Offline: recorder report, map picture, summary.md. Safe to rerun."""
    nr, nm = load("navreport"), load("navmap")
    first = json.loads((session / "before.json").read_text(encoding="utf-8"))
    last = json.loads((session / "after.json").read_text(encoding="utf-8"))
    restart = restart_since(first, last["status"], last["time"])   # older `after`s missed some
    lines = [f"# Water run {session.name}", "",
             f"Firmware {first['status']['build_id']}; before {first['time']}, after {last['time']}.", ""]
    facts = [("Battery", f"{first['app'].get('battery')} % → {last['app'].get('battery')} %"),
             ("App mode", f"{APP_MODES.get(first['app'].get('mode'), first['app'].get('mode'))} → "
                          f"{APP_MODES.get(last['app'].get('mode'), last['app'].get('mode'))}"),
             ("Navigation flags", first["navigation"]["flags"] if first.get("navigation") else "not available"),
             ("Same boot at `after`", "yes" if last["same_boot"] and not restart
              else f"NO: {restart or 'RAM lost'}"),
             ("Add-on faults before / after", f"{first['status']['faults']} / {last['status']['faults']}"),
             ("Battery floor", f"{last['status']['battery_floor']} % (tripped: {last['status']['floor_tripped']})")]
    if last.get("reset_flags"):
        flags = last["reset_flags"]
        facts.append(("Reset flags at `after`",
                      f"{flags['value']}: {', '.join(flags['set']) or 'none'}"
                      + ("; the power-on flag is gone: a WATCHDOG reset since the robot was switched on"
                         if flags["watchdog_since_power_on"] else "; no watchdog reset since power-on")))
    if last.get("attitude") and (last["same_boot"] and not restart or last.get("map", {}).get("bytes")):
        a = last["attitude"]
        facts.append(("Heading at `after`", f"{a['yaw_deg']:+.1f}° from the cycle start (the run's heading "
                                            f"drift if it was set down facing the way it went in); "
                                            f"pitch {a['pitch_deg']:+.1f}°, roll {a['roll_deg']:+.1f}°"))
    nav = last.get("navigation")
    if nav:
        facts.append(("Controller", f"mode {nav['mode']}, phase {nav['phase']}, sweeps {nav['sweeps']}, "
                                    f"lanes {nav['lanes']}, turn timeouts {nav['timeouts']}, "
                                    f"blocked {nav['blocked']}"))
        if nav.get("nav_version", 2) >= 3:
            facts.append(("Controller (diag-3.3)",
                          f"shifts the sonar found blocked {nav['stuck']}; sides ended for no new "
                          f"floor {nav['stale']}, by the lane cap {nav['capped']}; scans {nav['scans']} "
                          f"({nav['steered']} chose a heading by the map)"))
    if (session / "recorder.csv").exists():
        samples = nr.load_recording(session / "recorder.csv")
        rec = nr.analyze(samples)
        rec["source"] = {"path": str(session / "recorder.csv"),
                         "sha256": hashlib.sha256((session / "recorder.csv").read_bytes()).hexdigest(),
                         "capture": None}
        save(session / "recorder_report.json", rec)
        (session / "recorder_report.md").write_bytes(nr.markdown(rec).encode("utf-8"))
        facts.append(("Recorder", f"{rec['sample_count']} samples over {rec['observed_span_s']:.0f} s; "
                                  f"fault codes {rec['fault_codes'] or 'none'}"))
    if (session / "map.bin").exists():
        header, visited, pieces = nm.parse_map((session / "map.bin").read_bytes())
        info = nm.summary(header, visited, pieces, speed_cm_s, drift)
        nm.render(info, session / "map.png", f"Water run {session.name}")
        save(session / "map.json", {k: v for k, v in info.items() if k != "cells"})
        facts += [("Map", f"{info['visited_m2']} m² visited, {info['contacts']} wall contacts, "
                          f"{info['moving_s']} s moving, {info['pieces']} path pieces"
                          + (" (log wrapped: oldest part not drift-corrected)" if info["log_wrapped"] else "")),
                  ("Map scale", f"×{info['scale']} ({info['scale_source']})"),
                  ("Map drive not counted", ", ".join(f"{k.replace('_', ' ')} {v} s" for k, v in
                                                       info["not_counted_s"].items())
                   if "not_counted_s" in info else "not reported (diag-3.0 map)"),
                  ("Gyro drift", info["drift"])]
    lines += ["| | |", "|---|---|"] + [f"| {k} | {v} |" for k, v in facts] + [""]
    warnings = ([f"before: {w}" for w in first.get("warnings", [])] +
                [f"after: {w}" for w in last.get("warnings", [])])
    if restart and last["same_boot"]:
        warnings.append(f"report: the robot restarted since `before` ({restart}); `after` did not "
                        "notice: the recorder and navigation settings were lost, the program was stock")
    if warnings:
        lines += ["## Warnings", ""] + [f"- {w}" for w in warnings] + [""]
    lines += ["## Files", "",
              "`before.json`, `after.json`: status and settings at both ends. `recorder.csv` and "
              "`recorder_report.md`: the recorder and its phases. `map.bin`, `map.png`, `map.json`: "
              "the map (drift decision and scale in `map.json`). Everything here is a real robot "
              "measurement except the map's dead reckoning, which is an estimate.", ""]
    (session / "summary.md").write_bytes("\n".join(lines).encode("utf-8"))
    log(f"{session / 'summary.md'} written")
    return facts


async def connected(address, work):
    """One connection: the add-on answers first, then `work(crab)`; always disconnect."""
    from bleak import BleakClient
    for _ in range(FIND_TRIES):                    # scanning sends nothing
        device = await cc.find_device(address)
        if device:
            break
    if not device:
        sys.exit(f"robot {address} not found: nothing was sent; close the phone app and retry")
    client = BleakClient(device)
    await client.connect()
    try:
        crab = cc.Crab(client)
        await crab.start()
        return await work(crab)
    finally:
        await client.disconnect()


def setting(text):
    name, _, value = text.partition("=")
    if name not in cc.DIAG_NAV_CONFIG or name == "flags" or not value.lstrip("-").isdigit():
        raise argparse.ArgumentTypeError(f"expected NAME=VALUE with NAME one of "
                                         f"{', '.join(n for n in cc.DIAG_NAV_CONFIG if n != 'flags')}")
    return name, int(value)


def main():
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    p.add_argument("--address", help="the robot's Bluetooth address (required for before/after)")
    sub = p.add_subparsers(dest="cmd", required=True)
    b = sub.add_parser("before", help="check, configure and arm; one connection")
    b.add_argument("--session", type=Path, required=True)
    b.add_argument("--nav", choices=tuple(NAV_FLAGS), help="volatile navigation mode (default: leave as is)")
    b.add_argument("--set", type=setting, action="append", default=[], metavar="NAME=VALUE",
                   help="volatile navigation setting, e.g. spacing_cm=30")
    b.add_argument("--mode", choices=tuple(MODE_FOR), help="also change the app's cleaning mode (vendor setting)")
    b.add_argument("--period-ms", type=int, default=0, help="periodic recorder samples (0 = on change only)")
    b.add_argument("--no-on-change", action="store_true", help="do not record state changes")
    b.add_argument("--stop-when-full", action="store_true", help="keep the start of the run instead of the end")
    a = sub.add_parser("after", help="download everything and report; one connection")
    a.add_argument("--session", type=Path, required=True)
    for parser in (a, sub.add_parser("report", help="offline report from a session folder")):
        if parser is not a:
            parser.add_argument("--session", type=Path, required=True)
        parser.add_argument("--speed-cm-s", type=float, help="measured speed at drive 70 (map scale)")
        parser.add_argument("--drift", type=float, help="drift in deg/min instead of the estimate (0 = none)")
    args = p.parse_args()
    try:
        if args.cmd == "report":
            report(args.session, args.speed_cm_s, args.drift)
            return
        if not args.address:
            p.error("--address is required for before/after")
        if args.cmd == "before":
            cc._diag_range(args.period_ms, 0, 65535, "period")
            asyncio.run(connected(args.address, lambda crab: before(
                crab, args.session, args.nav, args.set, args.mode, args.period_ms,
                not args.no_on_change, args.stop_when_full)))
        else:
            asyncio.run(connected(args.address, lambda crab: after(crab, args.session)))
            report(args.session, args.speed_cm_s, args.drift)
    except (OSError, ValueError, TimeoutError) as error:
        p.exit(1, f"watertest: {error}\n")


if __name__ == "__main__":
    main()
