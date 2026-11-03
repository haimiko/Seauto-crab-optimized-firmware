"""Offline checks for tools/crab_ctl.py — no robot needed.

    python -I tests/test_crab_ctl.py

1. Modbus frames: every frame the tool builds must equal the stock app's hard-coded frames
   (CRC included), byte for byte.
2. Replies the stock app logs must parse correctly, and
   a reply split across two BLE notifications must be reassembled.
3. Firmware update: the XMODEM-1K packets must equal a port of the app's own packet builder;
   image checks must refuse junk. No firmware is published: put images of your own in
   tests/images to check them too.
4. Diagnostic add-on: request frames, the replies the add-on gave under emulation, refusals,
   reassembly at every fragment boundary, and the guard against the firmware's other parsers.
"""
import asyncio
import importlib.util
import struct
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("crab_ctl", ROOT / "tools" / "crab_ctl.py")
m = importlib.util.module_from_spec(spec)
spec.loader.exec_module(m)

fails = checks = 0


def ok(cond, msg):
    global fails, checks
    checks += 1
    fails += 0 if cond else 1
    print(("OK   " if cond else "FAIL ") + msg)


def chk(desc, got, want):
    g = got.hex(" ")
    ok(g == want.lower(), f"{desc}: {g}" + ("" if g == want.lower() else f"  != {want}"))


# --- 1. frames hard-coded in the stock app ------------------------------------------------
chk("getCycleTime", m.read_frame(7, "holding", 1), "01 03 00 07 00 01 35 cb")
chk("getFirmware",  m.read_frame(1, "input", 2),   "01 04 00 01 00 02 20 0b")
chk("getModel",     m.read_frame(0, "input", 2),   "01 04 00 00 00 02 71 cb")
chk("getBattery",   m.read_frame(3, "input", 2),   "01 04 00 03 00 02 81 cb")
chk("getS/N",       m.read_frame(2, "input", 2),   "01 04 00 02 00 02 d0 0b")
chk("getMode",      m.read_frame(5, "holding", 1), "01 03 00 05 00 01 94 0b")
chk("getSpeed",     m.read_frame(6, "holding", 1), "01 03 00 06 00 01 64 0b")
chk("getOnOff",     m.read_frame(4, "holding", 1), "01 03 00 04 00 01 c5 cb")
chk("setCycle=60",  m.write_frame(7, 60),          "01 06 00 07 00 3c 38 1a")
chk("setCycle=180", m.write_frame(7, 180),         "01 06 00 07 00 b4 38 7c")
chk("setSpeed=20",  m.write_frame(6, 20),          "01 06 00 06 00 14 69 c4")
chk("setUnlock",    m.write_frame(9, 1),           "01 06 00 09 00 01 98 08")
chk("setVoice=1",   m.write_frame(8, 1),           "01 06 00 08 00 01 c9 c8")
chk("setVoice=0",   m.write_frame(8, 0),           "01 06 00 08 00 00 08 08")

# --- 2. replies from the app log ----------------------------------------------------------
ok(m.parse_value(bytes.fromhex("0104043dfd434c571d")) == 1040008012, "fw_version reply parses")
ok(m.parse_value(bytes.fromhex("0104040000005a7bbf")) == 90, "battery reply parses")
ok(m.parse_value(bytes.fromhex("010302001e384c")) == 30, "speed reply parses")
ok(m.check_crc(bytes.fromhex("010302001e384c")), "check_crc on a genuine reply")

c = m.Crab.__new__(m.Crab)
c._buf, c._waiters = bytearray(), []
loop = asyncio.new_event_loop()
fut = loop.create_future()
c._waiters.append(fut)
full = bytes.fromhex("010302001e384c")
c._on_notify(None, bytearray(full[:3]))
c._on_notify(None, bytearray(full[3:]))
ok(fut.done() and fut.result() == full, "reply split over two notifications is reassembled")


# --- 3. firmware update packets -----------------------------------------------------------
def app_crc(data):
    """Literal port of the CRC loop in the app's h.m()."""
    i5 = 0
    for b3 in data:
        for i7 in range(8):
            z3 = ((b3 >> (7 - i7)) & 1) == 1
            z4 = ((i5 >> 15) & 1) == 1
            i5 <<= 1
            if z4 ^ z3:
                i5 ^= 4129
    return i5 & 0xFFFF


def app_packets(file_bytes):
    """Port of the app's d() padding (0x1A to a 1024 multiple) + m() packet build."""
    data = bytearray(file_bytes)
    if len(data) % 1024:
        data += bytes([26]) * (1024 - len(data) % 1024)
    out, t = [], 1
    while len(data) - (t - 1) * 1024 > 0:
        i3 = (t - 1) * 1024
        p = bytearray(1029)
        p[0], p[1], p[2] = 2, t & 0xFF, (~t) & 0xFF
        p[3:1027] = data[i3:i3 + 1024]
        crc = app_crc(data[i3:i3 + 1024])
        p[1027], p[1028] = crc >> 8, crc & 0xFF
        out.append(bytes(p))
        t += 1
    return out


ok(m.crc16_xmodem(b"123456789") == 0x31C3, "CRC-16/XMODEM check value 0x31C3")
synthetic = bytes(range(256)) * 9 + bytes(100)
ok(list(m.xmodem_blocks(synthetic)) == app_packets(synthetic), "synthetic image: packets identical to the app's")
for path in sorted((ROOT / "tests" / "images").glob("*.bin")):
    img = path.read_bytes()
    mine, theirs = list(m.xmodem_blocks(img)), app_packets(img)
    ok(mine == theirs, f"{path.name}: {len(mine)} packets identical to the app's")
    ok(m.check_image(img) is None, f"{path.name}: accepted by check_image")
ok(m.check_image(b"\xff" * 4096) is not None, "erased/garbage image refused")
ok(m.check_image(b"PK" + bytes(4094)) is not None,
   "non-firmware file refused")
ok(m.check_image(b"\x00" * (m.APP_MAX + 1)) is not None, "image reaching 0x0801F800 refused")

# --- 4. diagnostic add-on (protocol 3), no BLE connection ---------------
# Request frames are pinned byte for byte; the replies are the ones the add-on produced
# under emulation (this section needs no compiler and no firmware).
chk("FC47 latch",      m.diag_addr_frame(0x20000B2C),          "01 47 20 00 0b 2c b8 e8")
chk("FC41 read",       m.diag_read_frame(0, 12),               "01 41 00 00 0c 00 38 c5")
chk("FC41 max window", m.diag_read_frame(0xFFFF, 200),         "01 41 ff ff c8 00 6a 21")
chk("FC42 recorder",   m.diag_recorder_frame(1),               "01 42 00 00 00 01 b8 05")
chk("FC43 status",     m.diag_status_frame(),                  "01 43 00 00 00 00 44 05")
chk("FC44 control",    m.diag_control_frame(25, m.REC_CLEAR | m.REC_STOPFULL),
    "01 44 00 19 05 00 23 52")
chk("FC44 stop",       m.diag_control_frame(0),                "01 44 00 00 00 00 f1 c5")
chk("FC45 write 1",    m.diag_write_frame(b"\xa5"),            "01 45 a5 00 00 00 ee c9")
chk("FC48 write 2",    m.diag_write_frame(b"\x34\x12"),        "01 48 34 12 00 00 4f f1")
chk("FC49 write 4",    m.diag_write_frame(bytes.fromhex("78 56 34 12")), "01 49 78 56 34 12 b2 79")
chk("FC46 reset",      m.diag_reset_frame(1),                  "01 46 01 00 00 00 89 f9")
chk("FC46 bootloader", m.diag_reset_frame(2),                  "01 46 02 00 00 00 89 bd")
chk("FC4A floor off",  m.diag_floor_frame(0),                  "01 4a 00 00 00 00 98 04")
chk("FC4A floor 20",   m.diag_floor_frame(20),                 "01 4a 14 00 00 00 9d f4")
chk("FC4A floor 100",  m.diag_floor_frame(100),                "01 4a 64 00 00 00 87 34")
ok(all(len(f) == 8 for f in (m.diag_addr_frame(0xFFFFFFFF), m.diag_read_frame(0xFFFF, 200),
                             m.diag_recorder_frame(0xFFFFFFFF), m.diag_control_frame(0xFFFF, 7),
                             m.diag_write_frame(b"\xff" * 4), m.diag_reset_frame(2), m.diag_floor_frame(20))),
   "every diagnostic request is 8 bytes (the firmware closes frames at 8)")

status = bytes.fromhex(
    "01 43 7e 64 69 61 67 2d 32 2e 32 00 00 04 1c 01 00 01 00 00 00 29 00 00 00 2a 00 00 00 "
    "00 00 00 00 01 00 00 00 01 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 "
    "00 00 00 00 00 00 00 00 00 00 00 00 00 40 00 20 49 02 00 00 00 00 00 00 00 00 00 00 "
    "00 00 00 00 01 00 00 00 00 00 00 00 00 3a 00 20 44 01 00 00 41 c2 0f 41 80 00 00 00 "
    "00 00 00 00 14 00 00 00 00 00 00 00 00 00 00 00 09 06")
ok(m.parse_diag_reply(status, m.diag_status_frame()) == {
    "build_id": "diag-2.2", "protocol": 4, "sample_size": 28, "sram32k": 1, "nibble_cleared": 0,
    "boot_count": 1, "loop_count": 41, "ms": 42, "bpr19_at_boot": 0, "bpr19": 1,
    "persist_word": 1, "faults": 0, "fault_pc": 0, "fault_lr": 0, "fault_cfsr": 0,
    "fault_hfsr": 0, "fault_bfar": 0, "probe_faults": 0, "ring": 0x20004000, "ring_cap": 585,
    "seq": 0, "period_ms": 0, "rec_flags": 0, "frames": 1, "addr": 0, "g_address": 0x20003A00,
    "g_size": 324, "cpuid": 0x410FC241, "flash_kb": 128,
    "emergency_count": 0, "battery_floor": 20, "floor_tripped": 0, "floor_saved": 0},
   "FC43 status reply decodes with default floor, not yet saved")
memory = bytes.fromhex("01 41 10 20 00 0b 2c f3 01 00 00 4c 43 fd 3d 00 ca 9a 3b 6b b7")
ok(m.parse_diag_reply(memory, m.diag_read_frame(0, 12)) ==
   {"address": 0x20000B2C, "data": (499).to_bytes(4, "little") + (1040008012).to_bytes(4, "little")
    + (1000000000).to_bytes(4, "little")}, "FC41 reply: address + model / version / serial")
recorder = bytes.fromhex(
    "01 42 3c 00 00 00 01 c0 01 00 00 2e fb 2d 00 7c fc 41 01 37 00 c9 ff 00 01 00 00 00 00 "
    "04 00 02 3c b3 00 d9 01 00 00 2e fb 2d 00 7c fc 41 01 37 00 c9 ff 00 01 00 00 00 00 04 "
    "00 02 3c b3 00 9e 33")
sample = {"t_ms": 448, "yaw": -1234, "pitch": 45, "roll": -900, "sonar_cm": 321, "ltrack": 55,
          "rtrack": -55, "pump": 0, "ctype": 1, "csub": 0, "state": 0, "step": 0, "inwater": 0,
          "fault": 4, "batt": 0, "pins": 2, "guard": 60, "bump": 179}
ok(m.parse_diag_reply(recorder, m.diag_recorder_frame(1)) ==
   {"sequence": 1, "samples": [sample, {**sample, "t_ms": 473}]}, "FC42 reply: two 28-byte samples")
ok(m.parse_diag_reply(bytes.fromhex("01 42 04 00 00 03 e8 f5 9c"), m.diag_recorder_frame(1000)) ==
   {"sequence": 1000, "samples": []}, "FC42 reply past the newest sample is empty")
control = m.diag_control_frame(25, m.REC_CLEAR | m.REC_STOPFULL)
ok(m.parse_diag_reply(control, control) ==
   {"period_ms": 25, "on_change": False, "stop_when_full": True, "cleared": True}, "FC44 echo parses")
for data, reply in ((b"\xa5", "01 45 01 20 00 3c 30 22 41"), (b"\x34\x12", "01 48 02 20 00 3c 30 67 5c"),
                    (bytes.fromhex("78 56 34 12"), "01 49 04 20 00 3c 30 ee 8d")):
    ok(m.parse_diag_reply(bytes.fromhex(reply), m.diag_write_frame(data)) ==
       {"written": len(data), "address": 0x20003C30}, f"{len(data)}-byte write acknowledgement parses")
for mode in (1, 2):
    ok(m.parse_diag_reply(m.diag_reset_frame(mode), m.diag_reset_frame(mode)) == {"mode": mode},
       f"FC46 mode {mode} echo parses")
floor_reply = bytes.fromhex("01 4a 14 00 00 00 9d f4")
ok(m.parse_diag_reply(floor_reply, m.diag_floor_frame(20)) == {"battery_floor": 20},
   "FC4A canned floor echo parses")


def refuses(callback, description, wanted=ValueError, text=""):
    try:
        callback()
    except wanted as exc:
        ok(text in str(exc), description)
    else:
        ok(False, description)


def refusal(reply, request):
    try:
        m.parse_diag_reply(bytes.fromhex(reply), request)
    except m.DiagException as exc:
        return exc.fc, exc.code, exc.extra
    return None


ok(refusal("01 c1 82 60 00 00 00 7d 0f", m.diag_read_frame(0, 4)) == (0x41, 2, 0x60000000),
   "bus-fault refusal names the address")
ok(refusal("01 c1 81 00 00 00 00 27 0f", m.diag_read_frame(0, 4)) == (0x41, 1, 0),
   "bad-parameter refusal parses")
ok(refusal("01 c5 81 08 00 a0 00 5c eb", m.diag_write_frame(b"\x01")) == (0x45, 1, 0x0800A000),
   "write outside SRAM refused with the latched address")
ok(refusal("01 ca 81 65 00 00 00 38 b8", m.diag_floor_frame(20)) == (0x4A, 1, 0x65000000),
   "invalid battery floor exception parses")
ok(refusal("01 ca 83 08 01 ec 14 40 db", m.diag_floor_frame(20)) == (0x4A, 3, 0x0801EC14),
   "flash save failure exception names the failing address")
refuses(lambda: m.parse_diag_reply(bytes.fromhex("01 ca 83 08 01 ec 14 40 db"), m.diag_floor_frame(20)),
        "flash save failure is explicit to the user", wanted=m.DiagException, text="flash save failed")
refuses(lambda: m.parse_diag_reply(bytes.fromhex("01 c3 01 b1 30"), m.diag_status_frame()),
        "stock firmware's exception is reported as 'no add-on'", text="no diagnostic add-on")
refuses(lambda: m.parse_diag_reply(bytes.fromhex("01 80 02 c0 01"), m.diag_status_frame()),
        "the vendor's answer to a garbled frame is not taken for a reply")

for length in (0, 201):
    refuses(lambda: m.diag_read_frame(0, length), f"FC41 length {length} refused")
refuses(lambda: m.diag_read_frame(0x10000, 4), "FC41 offset beyond 16 bits refused")
refuses(lambda: m.diag_addr_frame(-1), "FC47 negative address refused")
refuses(lambda: m.diag_control_frame(65536), "FC44 excess period refused")
refuses(lambda: m.diag_control_frame(10, 8), "FC44 unknown flag refused")
refuses(lambda: m.diag_write_frame(b"abc"), "3-byte write refused (1, 2 or 4 only)")
refuses(lambda: m.diag_write_frame(b""), "empty write refused")
refuses(lambda: m.diag_reset_frame(3), "FC46 invalid mode refused")
for percent in (-1, 101):
    refuses(lambda: m.diag_floor_frame(percent), f"FC4A floor {percent} refused")
refuses(lambda: m.parse_diag_reply(floor_reply, m.diag_floor_frame(21)), "wrong floor echo refused")
refuses(lambda: m.parse_diag_reply(memory[:-1] + b"\0"), "bad diagnostic CRC refused")
refuses(lambda: m.parse_diag_reply(memory, m.diag_status_frame()), "reply to another function refused")
refuses(lambda: m.parse_diag_reply(memory, m.diag_read_frame(0, 3)), "wrong memory count refused")
refuses(lambda: m.parse_diag_reply(status, m.diag_read_frame(0, 4)), "status as a read reply refused")
refuses(lambda: m.parse_diag_reply(m.frame(status[:-3]), m.diag_status_frame()),
        "status of another size refused", text="byte count")
refuses(lambda: m.parse_diag_reply(m.frame(bytes([1, 0x43, 126]) + status[3:13] + b"\x05" + status[14:-2]),
                                   m.diag_status_frame()),
        "status from another protocol revision refused", text="protocol 5")
refuses(lambda: m.parse_diag_reply(m.frame(bytes([1, 0x43, 122]) + b"diag-2.1\0\0" + b"\x03" + status[14:-6]),
                                   m.diag_status_frame()), "backup-only protocol-3 status refused", text="other add-on version")
refuses(lambda: m.parse_diag_reply(m.frame(bytes([1, 0x43, 110]) + status[3:13] + b"\x02" + status[14:-18]),
                                   m.diag_status_frame()), "old protocol-2 status refused", text="other add-on version")
refuses(lambda: m.parse_diag_reply(m.frame(bytes.fromhex("01 42 05 00 00 00 00 aa")),
                                   m.diag_recorder_frame(0)), "partial recorder sample refused")
refuses(lambda: m.parse_diag_reply(recorder, m.diag_recorder_frame(2)),
        "recorder reply from before the requested sequence refused")
refuses(lambda: m.parse_diag_reply(m.diag_control_frame(100), m.diag_control_frame(200)),
        "wrong control echo refused")
refuses(lambda: m.parse_diag_reply(m.diag_reset_frame(1), m.diag_reset_frame(2)),
        "wrong reset echo refused")
refuses(lambda: m.parse_diag_reply(bytes.fromhex("01 48 02 20 00 3c 30 67 5c"), m.diag_write_frame(b"a")),
        "acknowledgement of another write width refused")

# reassembly: every reply above, split at every byte, must come out whole and only once whole
vendor_exception, add_on_exception = "01 c3 01 b1 30", "01 c1 82 60 00 00 00 7d 0f"
for full in (memory, recorder, status, bytes.fromhex(vendor_exception),
             bytes.fromhex(add_on_exception), bytes.fromhex("01 49 04 20 00 3c 30 ee 8d"), control,
             floor_reply, bytes.fromhex("01 ca 81 65 00 00 00 38 b8"),
             bytes.fromhex("01 ca 83 08 01 ec 14 40 db")):
    for split in range(1, len(full)):
        c._buf, c._waiters = bytearray(), [loop.create_future()]
        waiter = c._waiters[0]
        c._on_notify(None, bytearray(full[:split]))
        premature = waiter.done()
        c._on_notify(None, bytearray(full[split:]))
        if premature or not waiter.done() or waiter.result() != full:
            ok(False, f"reply FC{full[1]:02X} ({len(full)} B) split at {split}")
            break
    else:
        ok(True, f"reply FC{full[1]:02X} ({len(full)} B) reassembles at every fragment boundary")

# the firmware's other parsers on the same bytes
ok(m.frame_hazard(m.F8) and m.frame_hazard(b"\x01" + m.F12) and m.frame_hazard(b"\x00\xcc\xdd\x00"),
   "reset / bootloader / remote-control byte patterns are recognised")
ok(not any(m.frame_hazard(f) for f in (m.diag_status_frame(), bytes(8), m.read_frame(1, "input", 2))),
   "status, resync and stock frames are free of them")
ok(m.rc_count_after(None, bytes(20)) == (None, False), "0xFF-frame counter: unknown until a 0xFF")
ok(m.rc_count_after(None, b"\xff" + bytes(8)) == (9, False), "... 0xFF restarts it at 1")
ok(m.rc_count_after(9, b"\xff") == (1, True), "... a 0xFF at 9 commits a remote-control frame")
ok(m.rc_count_after(8, b"\xff") == (1, False) and m.rc_count_after(10, b"\xff") == (1, False),
   "... and only at 9")
ok(m.rc_count_after(255, b"\x00") == (0, False), "... the counter is 8 bits wide")
latch, offset = m.diag_read_plan(m.RC_COUNT_ADDR, 1, plain=True)
ok(not any(0xFF in f or m.frame_hazard(f) for f in
           (m.diag_status_frame(), m.diag_addr_frame(latch), m.diag_read_frame(offset, 1))),
   "the session's first requests (status, counter read) carry no 0xFF")
latch, offset = m.diag_read_plan(0x0800CCDD, 16)
ok(latch + offset == 0x0800CCDD and not m.frame_hazard(m.diag_addr_frame(latch))
   and m.frame_hazard(m.diag_addr_frame(0x0800CCDD)) is not None,
   "a read at 0x0800CCDD is planned around the remote-control header")


class FakeRobot:
    """bleak client stand-in that answers like the add-on (replies captured above) and
    keeps the firmware's 0xFF-frame counter the way the firmware does."""
    mtu_size = 23

    def __init__(self, count):
        self.writes, self.count, self.commits, self.latch = [], count, 0, 0

    async def write_gatt_char(self, uuid, data, response):
        data = bytes(data)
        self.writes.append(data)
        self.count, hit = m.rc_count_after(self.count, data)
        self.commits += hit
        if len(data) != 8 or data[0] != 1:
            return
        if data[1] == 0x47:
            self.latch = int.from_bytes(data[2:6], "big")
            reply = data
        elif data[1] == 0x41 and self.latch + int.from_bytes(data[2:4], "big") == m.RC_COUNT_ADDR:
            reply = m.frame(bytes([1, 0x41, 5]) + m.RC_COUNT_ADDR.to_bytes(4, "big") + bytes([self.count]))
        elif data[1] == 0x49:
            reply = m.frame(bytes([1, 0x49, 4]) + self.latch.to_bytes(4, "big"))
        else:
            return
        for start in range(0, len(reply), 20):
            self.crab._on_notify(None, bytearray(reply[start:start + 20]))


async def first_ff_of_a_session(count):
    robot = FakeRobot(count)
    robot.crab = crab = m.Crab(robot)
    await crab.diag_write(0x20003C30, b"\x10\x20\x30\xff")     # 0xFF is the 6th byte of the request
    return robot


for count in range(256):
    robot = asyncio.run(first_ff_of_a_session(count))
    if robot.commits or robot.writes[-1] != m.diag_write_frame(b"\x10\x20\x30\xff"):
        ok(False, f"first 0xFF of a session with the firmware's counter at {count}")
        break
else:
    ok(True, "first 0xFF of a session is safe whatever the firmware's counter was (all 256 values)")
unguarded = FakeRobot(4)
unguarded.count, hit = m.rc_count_after(unguarded.count, m.diag_write_frame(b"\x10\x20\x30\xff"))
ok(hit, "... and the same request sent blind with the counter at 4 would have committed a frame")
loop.close()

# Navigation extensions retain the installed protocol-4 FC43 layout.
for name, value, expected in (
        ("flags", 3, "01 4b 00 00 03 00 a5 34"),
        ("spacing_cm", 30, "01 4b 01 00 1e 00 ad 98"),
        ("spacing_cm", 34, "01 4b 01 00 22 00 bc 98")):
    request = m.diag_nav_frame(name, value)
    ok(request == bytes.fromhex(expected), f"FC4B {name} exact bytes")
    ok(m.parse_diag_reply(request, request) == {"setting": name, "value": value, "persistent": False},
       f"FC4B {name} echo is explicitly volatile")
ok(m.diag_nav_status_frame() == bytes.fromhex("01 4c 00 00 00 00 10 04"), "FC4C exact status request")
refuses(lambda: m.parse_diag_reply(m.frame(bytes.fromhex("01 cc 01")), m.diag_nav_status_frame()),
        "installed old add-on's navigation refusal is explicit", text="requires diag-2.4")
for name, (lo, hi) in zip(m.DIAG_NAV_CONFIG, m.DIAG_NAV_LIMITS):
    for value in (lo - 1, hi + 1):
        refuses(lambda: m.diag_nav_frame(name, value), f"navigation {name} {value} refused")
refuses(lambda: m.diag_nav_frame("unknown", 1), "unknown navigation setting refused")
refuses(lambda: m.parse_diag_reply(m.diag_nav_frame("flags", 1), m.diag_nav_frame("flags", 2)),
        "wrong navigation echo refused")
nav_payload = struct.pack("<B8IIiii9I", 2, 3, 34, 30, 1000, 20, 10, 3, 1500,
                          100, -1790, 1790, -1, 4, 1, 6, 0, 6800, 7, 2, 0, 1)
nav_reply = m.frame(bytes([1, 0x4C, 85]) + nav_payload)
decoded = m.parse_diag_reply(nav_reply, m.diag_nav_status_frame())
ok(decoded["axis"] == -1790 and decoded["lateral"] == -1 and decoded["spacing_cm"] == 34
   and decoded["phase"] == 4 and decoded["mode"] == 7 and decoded["blocked"] == 1,
   "FC4C canned signed state/config reply")
refuses(lambda: m.parse_diag_reply(m.frame(bytes([1, 0x4C, 85, 3]) + nav_payload[1:]),
                                   m.diag_nav_status_frame()), "unknown navigation layout refused")
# diag-3.3: layout 3 appends the shift echo, map cells, new-cell history, scan word, eight scan
# scores and five counters; flags bit 2 (lanes without the map's memory) is accepted.
nav3_payload = nav_payload[:1].replace(b"\x02", b"\x03") + nav_payload[1:] + struct.pack(
    "<4I8B5I", 43 | 45 << 16, 812, 0x00010203, 0x100, 0, 5, 0, 9, 1, 0, 2, 3, 7, 2, 0, 3, 1)
decoded3 = m.parse_diag_reply(m.frame(bytes([1, 0x4C, 129]) + nav3_payload), m.diag_nav_status_frame())
ok(decoded3["nav_version"] == 3 and decoded3["axis"] == -1790 and decoded3["blocked"] == 1
   and decoded3["echo"] == 43 | 45 << 16 and decoded3["cells"] == 812 and decoded3["scan"] == 0x100
   and decoded3["scores"] == [0, 5, 0, 9, 1, 0, 2, 3]
   and (decoded3["stuck"], decoded3["stale"], decoded3["capped"], decoded3["scans"],
        decoded3["steered"]) == (7, 2, 0, 3, 1), "FC4C layout 3 (diag-3.3) decoded")
ok(m.parse_diag_reply(nav_reply, m.diag_nav_status_frame())["nav_version"] == 2,
   "FC4C layout 2 (diag-2.8..3.2) still decoded")
ok(m.diag_nav_frame("flags", 5)[2:6] == bytes([0, 0, 5, 0]), "flags 5: Floor without the map's memory")
refuses(lambda: m.parse_diag_reply(m.frame(bytes([1, 0x4C, 128]) + nav3_payload[:-1]),
                                   m.diag_nav_status_frame()), "short layout 3 refused")
old_layout = struct.pack("<B8IIiii8I", 1, 3, 34, 30, 1000, 20, 10, 3, 1500,
                         100, -1790, 1790, -1, 4, 1, 6, 0, 6800, 7, 2, 0)
refuses(lambda: m.parse_diag_reply(m.frame(bytes([1, 0x4C, 81]) + old_layout),
                                   m.diag_nav_status_frame()), "version-1 navigation layout refused")
nav_loop = asyncio.new_event_loop()
for split in range(1, len(nav_reply)):
    crab = m.Crab(None)
    future = nav_loop.create_future()
    crab._waiters.append(future)
    crab._on_notify(None, bytearray(nav_reply[:split]))
    if future.done():
        ok(False, "navigation status delivered prematurely")
        break
    crab._on_notify(None, bytearray(nav_reply[split:]))
    if not future.done() or future.result() != nav_reply:
        ok(False, "navigation status failed fragmentation")
        break
else:
    ok(True, "FC4C reassembles at every fragment boundary")
nav_loop.close()

print("\n" + (f"ALL PASS ({checks} checks)" if fails == 0 else f"{fails} FAILURES ({checks} checks)"))
raise SystemExit(1 if fails else 0)
