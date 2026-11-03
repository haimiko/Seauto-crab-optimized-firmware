#!/usr/bin/env python3
"""crab_ctl - desktop control/diagnostic tool for the SEAUTO Crab pool robot.

Talks to the robot's BLE module (service FFF0, write FFF1, notify FFF2) using the
Modbus-RTU-over-BLE protocol the stock app uses. Read/write the documented registers,
read status, or send a raw frame. First-version CLI.

Only intended for your own robot. Reads status, writes the same settings the stock
app exposes, and can send a firmware image with the stock app's update protocol — `ota` refuses images that don't look like a Crab app image.

    python -I tools/crab_ctl.py scan
    python -I tools/crab_ctl.py info
    python -I tools/crab_ctl.py get speed
    python -I tools/crab_ctl.py set speed 30
    python -I tools/crab_ctl.py monitor
    python -I tools/crab_ctl.py raw 01 03 00 07 00 01
    python -I tools/crab_ctl.py listen                  # print every notification (bootloader text)
    python -I tools/crab_ctl.py ota image.bin --yes     # firmware update over BLE

Requires: pip install bleak
"""
import argparse
import asyncio
import hashlib
import json
import struct
import sys

from bleak import BleakClient, BleakScanner

SVC_UUID = "0000fff0-0000-1000-8000-00805f9b34fb"
WRITE_UUID = "0000fff1-0000-1000-8000-00805f9b34fb"
NOTIFY_UUID = "0000fff2-0000-1000-8000-00805f9b34fb"
# BLE names the stock app accepts: seauto_BLE, doiot-BLE, seauto-BLE, shark2.0-BLE
NAME_HINTS = ("seauto", "doiot-ble", "shark2.0-ble", "crab")

# register map. width = Modbus register count.
#   name: (address, kind, width, writable)   kind: "input" (FC04) / "holding" (FC03/06)
REGS = {
    "model":          (0, "input",   2, False),
    "fw_version":     (1, "input",   2, False),
    "serial":         (2, "input",   2, False),
    "battery":        (3, "input",   2, False),
    "on_off":         (4, "holding", 1, True),
    "mode":           (5, "holding", 1, True),
    "speed":          (6, "holding", 1, True),
    "cycle_time":     (7, "holding", 1, True),
    "voice_reminder": (8, "holding", 1, True),
    "unlock":         (9, "holding", 1, True),
}
MODELS = {499: "Crab", 699: "Seal", 999: "Shark", 200: "SAT30", 1099: "Shark 2.0"}


def crc16(data: bytes) -> bytes:
    """CRC-16/Modbus, returned low byte first (wire order)."""
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return bytes([crc & 0xFF, crc >> 8])


def frame(body: bytes) -> bytes:
    return body + crc16(body)


def read_frame(addr: int, kind: str, width: int) -> bytes:
    fc = 4 if kind == "input" else 3
    return frame(bytes([1, fc, addr >> 8, addr & 0xFF, 0, width]))


def write_frame(addr: int, value: int) -> bytes:
    v = value & 0xFFFF
    return frame(bytes([1, 6, addr >> 8, addr & 0xFF, v >> 8, v & 0xFF]))


def check_crc(f: bytes) -> bool:
    return len(f) >= 4 and crc16(f[:-2]) == f[-2:]


def parse_value(resp: bytes):
    """Decode a FC03/04 read reply into an int, or None if not a clean read reply."""
    if len(resp) < 5 or resp[1] not in (3, 4):
        return None
    n = resp[2]
    data = resp[3:3 + n]
    if len(data) != n:
        return None
    return int.from_bytes(data, "big")


# --- diagnostic add-on (its source is not published yet) ---
# Every request is exactly 8 bytes, like the stock frames: the firmware's frame assembler
# closes any non-FC16 frame at 8 bytes. 32-bit addresses go
# through an address latch (FC47); reads carry a 16-bit offset from it.
DIAG_PROTOCOL = 4
DIAG_SAMPLE = struct.Struct("<IhhhHhh10BH")
DIAG_SAMPLE_FIELDS = ("t_ms", "yaw", "pitch", "roll", "sonar_cm", "ltrack", "rtrack",
                      "pump", "ctype", "csub", "state", "step", "inwater", "fault",
                      "batt", "pins", "guard", "bump")
DIAG_STATUS = struct.Struct("<10s4B28I")
DIAG_STATUS_FIELDS = ("protocol", "sample_size", "sram32k", "nibble_cleared",
                      "boot_count", "loop_count", "ms", "bpr19_at_boot", "bpr19",
                      "persist_word", "faults", "fault_pc", "fault_lr", "fault_cfsr",
                      "fault_hfsr", "fault_bfar", "probe_faults", "ring", "ring_cap", "seq",
                      "period_ms", "rec_flags", "frames", "addr", "g_address", "g_size",
                      "cpuid", "flash_kb", "emergency_count", "battery_floor", "floor_tripped",
                      "floor_saved")
REC_CLEAR, REC_ONCHANGE, REC_STOPFULL = 1, 2, 4
DIAG_WRITE_FC = {1: 0x45, 2: 0x48, 4: 0x49}     # bytes written -> function code
DIAG_MAX_READ = 200
DIAG_NAV_CONFIG = ("flags", "spacing_cm", "speed_cm_s", "back_ms", "floor_min", "wall_min",
                   "still_windows", "wall_back_ms")
# flags: bit0 Floor, bit1 Standard, bit2 lanes without the map's memory (diag-3.3; older
# add-ons refuse 4..7)
DIAG_NAV_LIMITS = ((0, 7), (10, 80), (10, 80), (100, 3000), (1, 120), (1, 30), (1, 15), (500, 9000))
DIAG_NAV_FIELDS_V2 = DIAG_NAV_CONFIG + ("since", "axis", "heading", "lateral", "phase", "side",
                                       "lanes", "short_lanes", "transfer_ms", "mode", "sweeps",
                                       "timeouts", "blocked")
DIAG_NAV_FIELDS = DIAG_NAV_FIELDS_V2 + ("echo", "cells", "fresh", "scan", "scores", "stuck",
                                        "stale", "capped", "scans", "steered")
DIAG_NAV_STATUS_V2 = struct.Struct("<B8IIiii9I")   # layout version 2 (diag-2.8..3.2)
DIAG_NAV_STATUS = struct.Struct("<B8IIiii9I4I8B5I")  # layout version 3
MAP_ADDR, MAP_BYTES, MAP_MAGIC = 0x20006800, 0x1800, 0x3150414D   # diag-3.0+ map + path log (tools/navmap.py)
MAP_VERSIONS = (2, 3)                                              # diag-3.0, diag-3.1 and later
F12 = b"\x1b[24~"   # firmware main loop: enter bootloader update mode
F8 = b"\x1b[19~"    # firmware main loop: plain reset


def _diag_range(value, low, high, name):
    if not low <= value <= high:
        raise ValueError(f"{name} must be {low}..{high}")
    return value


def _diag_frame(fc: int, params: bytes) -> bytes:
    if len(params) != 4:
        raise ValueError("a diagnostic request carries exactly 4 parameter bytes")
    return frame(bytes([1, fc]) + params)


def diag_addr_frame(address: int) -> bytes:
    return _diag_frame(0x47, struct.pack(">I", _diag_range(address, 0, 0xFFFFFFFF, "address")))


def diag_read_frame(offset: int, length: int) -> bytes:
    _diag_range(offset, 0, 0xFFFF, "offset")
    _diag_range(length, 1, DIAG_MAX_READ, "length")
    return _diag_frame(0x41, struct.pack(">HBB", offset, length, 0))


def diag_recorder_frame(sequence: int) -> bytes:
    return _diag_frame(0x42, struct.pack(">I", _diag_range(sequence, 0, 0xFFFFFFFF, "sequence")))


def diag_status_frame() -> bytes:
    return _diag_frame(0x43, bytes(4))


def diag_control_frame(period_ms: int, flags: int = 0) -> bytes:
    _diag_range(period_ms, 0, 0xFFFF, "period_ms")
    _diag_range(flags, 0, REC_CLEAR | REC_ONCHANGE | REC_STOPFULL, "flags")
    return _diag_frame(0x44, struct.pack(">HBB", period_ms, flags, 0))


def diag_write_frame(data: bytes) -> bytes:
    """Write 1, 2 or 4 bytes (memory order) at the latched address, as one store."""
    if len(data) not in DIAG_WRITE_FC:
        raise ValueError("a diagnostic write is 1, 2 or 4 bytes")
    return _diag_frame(DIAG_WRITE_FC[len(data)], bytes(data).ljust(4, b"\0"))


def diag_reset_frame(mode: int = 1) -> bytes:
    return _diag_frame(0x46, bytes([_diag_range(mode, 1, 2, "reset mode"), 0, 0, 0]))


def diag_floor_frame(percent: int) -> bytes:
    """Save discharge floor in flash (0 disables); prior trip stays latched until reset."""
    return _diag_frame(0x4A, bytes([_diag_range(percent, 0, 100, "battery floor"), 0, 0, 0]))


def diag_nav_frame(setting: str, value: int) -> bytes:
    """Volatile setting; flags 0 stock, 1 Floor, 2 Standard, 3 both, +4 without the map's
    memory (diag-3.3). No flash write."""
    if setting not in DIAG_NAV_CONFIG:
        raise ValueError("unknown navigation setting")
    key = DIAG_NAV_CONFIG.index(setting)
    _diag_range(value, *DIAG_NAV_LIMITS[key], setting)
    return _diag_frame(0x4B, struct.pack(">BHB", key, value, 0))


def diag_nav_status_frame() -> bytes:
    return _diag_frame(0x4C, bytes(4))


class DiagException(ValueError):
    def __init__(self, fc, code, extra):
        self.fc, self.code, self.extra = fc, code, extra
        what = {1: "bad parameter", 2: "bus fault at", 3: "flash save failed at"}.get(code, f"code {code}")
        super().__init__(f"diagnostic FC 0x{fc:02X} refused: {what} 0x{extra:08X}")


def parse_diag_reply(resp: bytes, request: bytes | None = None) -> dict:
    """Validate CRC, sizes and (given the request) that the reply answers it; decode it.

    Recorder fields use wire units: angles 0.1 degree, bump cm/s². Addresses, sequence
    numbers and exception details are big-endian; status, samples and memory are not.
    """
    if not check_crc(resp) or resp[0] != 1:
        raise ValueError("bad diagnostic reply CRC or slave")
    fc = resp[1] & 0x7F
    if not 0x41 <= fc <= 0x4C:
        raise ValueError("not a diagnostic function code")
    if request is not None and (len(request) != 8 or not check_crc(request)
                                or request[:2] != bytes([1, fc])):
        raise ValueError("diagnostic reply does not match request")
    if resp[1] & 0x80:
        if len(resp) == 5:
            if fc in (0x4B, 0x4C):
                raise ValueError("firmware lacks the navigation extension (requires diag-2.4)")
            raise ValueError("the robot answered with a stock Modbus exception: "
                             "its firmware has no diagnostic add-on")
        if len(resp) != 9 or not resp[2] & 0x80:
            raise ValueError("bad diagnostic exception")
        raise DiagException(fc, resp[2] & 0x7F, int.from_bytes(resp[3:7], "big"))
    if fc in (0x41, 0x42, 0x43, 0x4C):
        if len(resp) != resp[2] + 5:
            raise ValueError("bad diagnostic byte count")
        payload = resp[3:-2]
        if fc == 0x4C:
            if (payload[:1], len(payload)) == (b"\x02", DIAG_NAV_STATUS_V2.size):
                values = DIAG_NAV_STATUS_V2.unpack(payload)
                return {"nav_version": 2, **dict(zip(DIAG_NAV_FIELDS_V2, values[1:]))}
            if (payload[:1], len(payload)) != (b"\x03", DIAG_NAV_STATUS.size):
                raise ValueError("bad navigation status length/version (requires diag-2.8)")
            values = DIAG_NAV_STATUS.unpack(payload)
            words = len(DIAG_NAV_FIELDS_V2) + 4              # through "scan"
            return {"nav_version": 3, **dict(zip(DIAG_NAV_FIELDS, values[1:words + 1])),
                    "scores": list(values[words + 1:words + 9]),
                    **dict(zip(DIAG_NAV_FIELDS[-5:], values[words + 9:]))}
        if fc == 0x43:
            if len(payload) != DIAG_STATUS.size:
                raise ValueError("bad diagnostic status length (other add-on version?)")
            values = DIAG_STATUS.unpack(payload)
            result = dict(zip(DIAG_STATUS_FIELDS, values[1:]))
            result["build_id"] = values[0].split(b"\0", 1)[0].decode("ascii", "replace")
            if result["protocol"] != DIAG_PROTOCOL or result["sample_size"] != DIAG_SAMPLE.size:
                raise ValueError(f"add-on {result['build_id']} speaks protocol "
                                 f"{result['protocol']}, this tool {DIAG_PROTOCOL}")
            return result
        if len(payload) < 4:
            raise ValueError("diagnostic reply too short")
        first, body = int.from_bytes(payload[:4], "big"), payload[4:]
        if fc == 0x41:
            _diag_range(len(body), 1, DIAG_MAX_READ, "memory reply length")
            if request is not None and len(body) != request[4]:
                raise ValueError("memory reply length does not match request")
            return {"address": first, "data": body}
        if len(body) % DIAG_SAMPLE.size or len(body) > 7 * DIAG_SAMPLE.size:
            raise ValueError("bad recorder sample length")
        if request is not None and first < int.from_bytes(request[2:6], "big"):
            raise ValueError("recorder reply starts before the requested sequence")
        return {"sequence": first, "samples": [dict(zip(DIAG_SAMPLE_FIELDS, values))
                                               for values in DIAG_SAMPLE.iter_unpack(body)]}
    if fc in (0x44, 0x46, 0x47, 0x4A, 0x4B):
        if len(resp) != 8:
            raise ValueError("bad diagnostic echo length")
        if request is not None and resp != request:
            raise ValueError("diagnostic echo does not match request")
        if fc == 0x44:
            return {"period_ms": int.from_bytes(resp[2:4], "big"),
                    "on_change": bool(resp[4] & REC_ONCHANGE),
                    "stop_when_full": bool(resp[4] & REC_STOPFULL),
                    "cleared": bool(resp[4] & REC_CLEAR)}
        if fc == 0x46:
            return {"mode": _diag_range(resp[2], 1, 2, "reset reply mode")}
        if fc == 0x4A:
            if any(resp[3:6]):
                raise ValueError("bad battery floor echo parameters")
            return {"battery_floor": _diag_range(resp[2], 0, 100, "battery floor")}
        if fc == 0x4B:
            key, value = resp[2], int.from_bytes(resp[3:5], "big")
            if key >= len(DIAG_NAV_CONFIG) or resp[5]:
                raise ValueError("bad navigation echo parameters")
            _diag_range(value, *DIAG_NAV_LIMITS[key], DIAG_NAV_CONFIG[key])
            return {"setting": DIAG_NAV_CONFIG[key], "value": value, "persistent": False}
        return {"addr": int.from_bytes(resp[2:6], "big")}
    if len(resp) != 9 or DIAG_WRITE_FC.get(resp[2]) != fc:
        raise ValueError("bad diagnostic write acknowledgement")
    return {"written": resp[2], "address": int.from_bytes(resp[3:7], "big")}


def reply_length(head: bytes) -> int | None:
    """Total length of the reply that starts with `head` (>= 3 bytes); None if not known."""
    fc = head[1]
    if fc in (3, 4, 0x41, 0x42, 0x43, 0x4C):
        return 5 + head[2]
    if fc in (6, 0x44, 0x46, 0x47, 0x4A, 0x4B):
        return 8
    if fc in (0x45, 0x48, 0x49):
        return 9
    if fc & 0x80:
        return 9 if head[2] & 0x80 else 5   # the add-on flags its code byte; the vendor's are 1..3
    return None


# The firmware gives every received byte to four more parsers besides Modbus
#, so binary diagnostic
# requests must not look like their input:
#   ESC[19~ resets the robot, ESC[24~ reboots it into the bootloader;
#   CC DD + 6 bytes is a remote-control frame that sets the cleaning algorithm;
#   so is 0xFF + 8 bytes + 0xFF (a byte counter that restarts at every 0xFF).
def frame_hazard(data: bytes) -> str | None:
    """Reason these bytes must not be sent as a request, else None."""
    for pattern, what in ((F8, "the reset sequence ESC[19~"),
                          (F12, "the bootloader sequence ESC[24~"),
                          (b"\xcc\xdd", "a remote-control frame header (CC DD)")):
        if pattern in data:
            return what
    return None


def rc_count_after(count, data: bytes):
    """Model of the firmware's 0xFF-frame byte counter. Returns (count after
    `data`, True if a 0xFF arrived at count 9, which commits a remote-control frame).
    `count` is None while unknown, i.e. until the first 0xFF of a session."""
    hit = False
    for byte in data:
        if byte == 0xFF:
            hit = hit or count == 9
            count = 0
        if count is not None:
            count = (count + 1) & 0xFF
    return count, hit


def diag_read_plan(address: int, length: int, plain: bool = False) -> tuple[int, int]:
    """(latch address, offset) to read `length` bytes at `address` with request frames that
    are free of frame_hazard patterns (their CRC bytes count too); `plain` also keeps every
    0xFF out of them."""
    def usable(request):
        return frame_hazard(request) is None and not (plain and 0xFF in request)

    for offset in range(0, min(address, 0xFFFF) + 1):
        if usable(diag_addr_frame(address - offset)) and usable(diag_read_frame(offset, length)):
            return address - offset, offset
    raise ValueError(f"no safe request found for 0x{address:08X}")


RC_COUNT_ADDR = 0x2000043D      # the firmware's byte counter, the one rc_count_after models


class Crab:
    def __init__(self, client: BleakClient):
        self.client = client
        self._buf = bytearray()
        self._waiters = []
        self._rc_count = None       # firmware's 0xFF-frame byte counter; unknown at connect
        self._latch = None          # diagnostic address latch, once set in this session

    def _on_notify(self, _char, data: bytearray):
        self._buf += data
        # try to hand a complete, CRC-valid frame to the oldest waiter
        while len(self._buf) >= 5:
            # Known lengths prevent a CRC-valid prefix of a large recorder/status
            # reply being mistaken for a whole frame while BLE fragments arrive.
            known = reply_length(self._buf)
            for end in ([known] if known else range(5, len(self._buf) + 1)):
                if end > len(self._buf):
                    return
                if check_crc(bytes(self._buf[:end])):
                    f = bytes(self._buf[:end])
                    del self._buf[:end]
                    if self._waiters:
                        self._waiters.pop(0).set_result(f)
                    break
            else:
                return

    async def start(self):
        await self.client.start_notify(NOTIFY_UUID, self._on_notify)

    async def _write(self, data: bytes):
        self._rc_count, _ = rc_count_after(self._rc_count, data)
        await self.client.write_gatt_char(WRITE_UUID, data, response=False)

    async def transact(self, tx: bytes, timeout=3.0, guard=False) -> bytes:
        """Send one request and wait for one reply. `guard` (diagnostic requests) refuses
        byte patterns the firmware's other parsers act on and keeps a 0xFF from completing
        a remote-control frame by sending 0x00 filler bytes first (the frame assembler
        ignores bytes between frames)."""
        if guard:
            why = frame_hazard(tx)
            if why:
                raise ValueError(f"request {tx.hex(' ')} contains {why}; not sent")
            for filler in range(12):
                count, _ = rc_count_after(self._rc_count, bytes(filler))
                if not rc_count_after(count, tx)[1]:
                    break
            else:
                raise ValueError(f"request {tx.hex(' ')} cannot be sent safely; not sent")
            if filler:
                await self._write(bytes(filler))
        self._buf.clear()
        loop = asyncio.get_running_loop()
        fut = loop.create_future()
        self._waiters.append(fut)
        try:
            # One request = one BLE write = one UART burst (the firmware's receive buffer
            # is 30 bytes; every frame this tool builds is 8).
            chunk = max(20, min(244, (self.client.mtu_size or 23) - 3))
            for offset in range(0, len(tx), chunk):
                await self._write(tx[offset:offset + chunk])
            return await asyncio.wait_for(fut, timeout)
        finally:
            if fut in self._waiters:
                self._waiters.remove(fut)

    async def get(self, name: str):
        addr, kind, width, _ = REGS[name]
        resp = await self.transact(read_frame(addr, kind, width))
        return parse_value(resp)

    async def set(self, name: str, value: int) -> bytes:
        addr, kind, _, writable = REGS[name]
        if not writable:
            raise ValueError(f"{name} is read-only")
        return await self.transact(write_frame(addr, value))

    # ---- diagnostic add-on ----
    async def resync(self):
        """After a lost byte the firmware's frame assembler may sit inside a frame (it has
        no timeout): eight zero bytes complete any 8-byte frame, the rest is ignored."""
        await self._write(bytes(8))
        await asyncio.sleep(0.2)
        self._buf.clear()

    async def diag(self, tx: bytes, timeout=3.0, retries=1) -> dict:
        """One diagnostic request -> decoded reply. A timeout or a reply that does not
        answer the request is retried once after a resync (requests are idempotent);
        a refusal from the add-on (DiagException) is not."""
        for attempt in range(retries + 1):
            try:
                return parse_diag_reply(await self.transact(tx, timeout, guard=True), tx)
            except DiagException:
                raise
            except (asyncio.TimeoutError, ValueError):
                if attempt == retries or frame_hazard(tx):
                    raise
                await self.resync()

    async def diag_learn_rc_count(self):
        """Read the firmware's 0xFF-frame byte counter, with requests that carry no 0xFF.
        Until then its value is unknown and the first 0xFF of the session could complete a
        remote-control frame (1 chance in 256); from here on the guard in transact() is
        exact. The value read already includes the bytes of the request that read it.
        Moves the address latch, so it runs before a command sets the latch, never between
        a latch and the request that uses it. Does nothing once the counter is known."""
        if self._rc_count is not None:
            return
        latch, offset = diag_read_plan(RC_COUNT_ADDR, 1, plain=True)
        self._latch = None
        await self.diag(diag_addr_frame(latch))
        self._latch = latch
        reply = await self.diag(diag_read_frame(offset, 1))
        if reply["address"] != RC_COUNT_ADDR:
            self._latch = None
            raise ValueError(f"memory reply is for 0x{reply['address']:08X}")
        self._rc_count = reply["data"][0]

    async def diag_read(self, address: int, length: int, chunk: int = 64, progress=None) -> bytes:
        """Read memory; every reply is checked against the address it should come from."""
        await self.diag_learn_rc_count()
        out = bytearray()
        while len(out) < length:
            at = address + len(out)
            n = min(chunk, length - len(out))
            offset = None if self._latch is None else at - self._latch
            if offset is None or not 0 <= offset <= 0xFFFF or frame_hazard(diag_read_frame(offset, n)):
                latch, offset = diag_read_plan(at, n)
                self._latch = None
                await self.diag(diag_addr_frame(latch))
                self._latch = latch
            reply = await self.diag(diag_read_frame(offset, n))
            if reply["address"] != at:
                self._latch = None
                raise ValueError(f"memory reply is for 0x{reply['address']:08X}, "
                                 f"expected 0x{at:08X} (robot restarted?)")
            out += reply["data"]
            if progress:
                progress(len(out), length)
        return bytes(out)

    async def diag_write(self, address: int, data: bytes) -> None:
        """Write 1, 2 or 4 bytes of SRAM with a single store; the reply names the address."""
        request = diag_write_frame(data)
        await self.diag_learn_rc_count()
        self._latch = None
        await self.diag(diag_addr_frame(address))
        self._latch = address
        reply = await self.diag(request)
        if reply != {"written": len(data), "address": address}:
            raise ValueError(f"write acknowledged as {reply}")

    async def diag_recorder(self, sequence: int = 0, limit: int | None = None) -> list[dict]:
        """Samples from `sequence` up to the newest one (or `limit` of them), each with its
        sequence number. Older samples the ring has already overwritten are skipped."""
        await self.diag_learn_rc_count()
        samples = []
        while limit is None or len(samples) < limit:
            reply = await self.diag(diag_recorder_frame(sequence))
            if not reply["samples"]:
                break
            for index, sample in enumerate(reply["samples"]):
                samples.append({"seq": reply["sequence"] + index, **sample})
            sequence = reply["sequence"] + len(reply["samples"])
        return samples if limit is None else samples[:limit]


async def find_device(address=None, timeout=10.0):
    if address:
        d = await BleakScanner.find_device_by_address(address, timeout=timeout)
        return d
    devices = await BleakScanner.discover(timeout=timeout)
    for d in devices:
        name = (d.name or "").lower()
        if any(h in name for h in NAME_HINTS):
            return d
    return None


async def cmd_scan(_args):
    print(f"scanning {10}s for BLE devices...")
    devices = await BleakScanner.discover(timeout=10.0)
    for d in sorted(devices, key=lambda x: x.name or "~"):
        tag = "  <- Crab?" if any(h in (d.name or "").lower() for h in NAME_HINTS) else ""
        print(f"  {d.address}  {d.name or '(no name)'}{tag}")


async def _connect(args) -> tuple[BleakClient, Crab]:
    dev = await find_device(getattr(args, "address", None))
    if not dev:
        sys.exit("no Crab found (try: scan, or pass --address)")
    print(f"connecting to {dev.name or '?'} [{dev.address}] ...")
    client = BleakClient(dev)
    await client.connect()
    crab = Crab(client)
    await crab.start()
    return client, crab


def fmt(name: str, raw):
    if raw is None:
        return "no/!bad reply"
    if name == "model":
        return f"{raw} ({MODELS.get(raw, '?')})"
    if name == "fw_version":
        return str(raw)
    if name == "battery":
        return f"{raw}%"
    return str(raw)


async def cmd_info(args):
    client, crab = await _connect(args)
    try:
        for name in ("model", "fw_version", "serial", "battery",
                     "on_off", "mode", "speed", "cycle_time", "voice_reminder"):
            try:
                print(f"  {name:15} {fmt(name, await crab.get(name))}")
            except Exception as e:
                print(f"  {name:15} error: {e}")
    finally:
        await client.disconnect()


async def cmd_get(args):
    client, crab = await _connect(args)
    try:
        print(fmt(args.register, await crab.get(args.register)))
    finally:
        await client.disconnect()


async def cmd_set(args):
    client, crab = await _connect(args)
    try:
        resp = await crab.set(args.register, args.value)
        ok = check_crc(resp) and resp[:6] == write_frame(REGS[args.register][0], args.value)[:6]
        print(f"wrote {args.register} = {args.value}; echo {'OK' if ok else resp.hex(' ')}")
    finally:
        await client.disconnect()


async def cmd_monitor(args):
    client, crab = await _connect(args)
    try:
        print("battery / mode / speed / on_off  (Ctrl-C to stop)")
        while True:
            vals = {n: await crab.get(n) for n in ("battery", "mode", "speed", "on_off")}
            print(f"  batt={fmt('battery', vals['battery'])}  mode={vals['mode']}  "
                  f"speed={vals['speed']}  on={vals['on_off']}")
            await asyncio.sleep(args.interval)
    except (KeyboardInterrupt, asyncio.CancelledError):
        pass
    finally:
        await client.disconnect()


async def cmd_raw(args):
    body = bytes(int(x, 16) for x in args.bytes)
    tx = body if args.no_crc else frame(body)
    client, crab = await _connect(args)
    try:
        print(f"tx: {tx.hex(' ')}")
        resp = await crab.transact(tx)
        print(f"rx: {resp.hex(' ')}  crc={'ok' if check_crc(resp) else 'BAD'}  "
              f"value={parse_value(resp)}")
    finally:
        await client.disconnect()


_HEX_FIELDS = ("bpr19_at_boot", "bpr19", "persist_word", "fault_pc", "fault_lr", "fault_cfsr",
               "fault_hfsr", "fault_bfar", "ring", "addr", "g_address", "cpuid")


def hexdump(address: int, data: bytes) -> str:
    return "\n".join(f"{address + i:08X}  {data[i:i + 16].hex(' ')}" for i in range(0, len(data), 16))


async def cmd_diag(args):
    # Validate what can be validated before connecting.
    if args.cmd == "diag-read":
        _diag_range(args.length, 1, 0x100000, "length")
        _diag_range(args.chunk, 1, DIAG_MAX_READ, "chunk")
        diag_read_plan(args.memory_address, min(args.chunk, args.length))
    elif args.cmd == "diag-write":
        data = bytes.fromhex(" ".join(args.bytes))
        write_request = diag_write_frame(data)
        if not 0x20000000 <= args.memory_address <= 0x20008000 - len(data):
            raise ValueError("writes go to SRAM only: 0x20000000..0x20007FFF")
        for request in (diag_addr_frame(args.memory_address), write_request):
            if frame_hazard(request):
                sys.exit(f"request {request.hex(' ')} contains {frame_hazard(request)}; "
                         "write a neighbouring address or another width")
    elif args.cmd == "diag-control":
        flags = (REC_CLEAR * args.clear | REC_ONCHANGE * args.on_change
                 | REC_STOPFULL * args.stop_when_full)
        control_request = diag_control_frame(args.period_ms, flags)
    elif args.cmd == "diag-floor":
        floor_request = diag_floor_frame(args.percent)
    elif args.cmd == "diag-nav":
        nav_request = diag_nav_frame(args.setting, args.value)
    client, crab = await _connect(args)
    try:
        # Status first: proves the add-on is there and speaks this tool's protocol. Then the
        # firmware's 0xFF-frame counter, before any request that could carry a 0xFF.
        status = await crab.diag(diag_status_frame())
        await crab.diag_learn_rc_count()
        if args.cmd == "diag-status":
            print(json.dumps({k: f"0x{v:08X}" if k in _HEX_FIELDS else v
                              for k, v in status.items()}, indent=2))
        elif args.cmd == "diag-read":
            def progress(done, total):
                if args.out:
                    print(f"  {done}/{total} bytes", end="\r")
            data = await crab.diag_read(args.memory_address, args.length, args.chunk, progress)
            if args.out:
                with open(args.out, "wb") as f:
                    f.write(data)
                print(f"\n{args.out}: {len(data)} bytes from 0x{args.memory_address:08X}, "
                      f"SHA-256 {hashlib.sha256(data).hexdigest()}")
            else:
                print(hexdump(args.memory_address, data))
        elif args.cmd == "diag-recorder":
            samples = await crab.diag_recorder(args.sequence, args.count)
            if args.csv:
                with open(args.csv, "w", encoding="utf-8", newline="\n") as f:
                    f.write(",".join(("seq",) + DIAG_SAMPLE_FIELDS) + "\n")
                    for sample in samples:
                        f.write(",".join(str(sample[k]) for k in ("seq",) + DIAG_SAMPLE_FIELDS) + "\n")
                print(f"{args.csv}: {len(samples)} samples (recorder holds {status['ring_cap']}, "
                      f"{status['seq']} written)")
            else:
                for sample in samples:
                    print(json.dumps(sample))
        elif args.cmd == "diag-control":
            print(json.dumps(await crab.diag(control_request)))
        elif args.cmd == "diag-floor":
            print(json.dumps(await crab.diag(floor_request)))
        elif args.cmd == "diag-nav":
            print(json.dumps(await crab.diag(nav_request)))
        elif args.cmd == "diag-nav-status":
            print(json.dumps(await crab.diag(diag_nav_status_frame()), indent=2))
        elif args.cmd == "diag-map":
            # Read-only: the visited-area map of the current or last cycle (lost at power-off).
            if not status["sram32k"]:
                raise ValueError("this robot reports 16 KB SRAM: no map")
            data = await crab.diag_read(MAP_ADDR, MAP_BYTES, DIAG_MAX_READ,
                                        lambda done, total: print(f"  {done}/{total} bytes", end="\r"))
            if int.from_bytes(data[:4], "little") != MAP_MAGIC:
                raise ValueError("no map: needs diag-3.0 or later and a cleaning cycle since power-on")
            with open(args.out, "wb") as f:
                f.write(data)
            print(f"\n{args.out}: map {len(data)} bytes, SHA-256 {hashlib.sha256(data).hexdigest()}; "
                  f"render: python -I tools/navmap.py {args.out} --png map.png")
        elif args.cmd == "diag-write":
            await crab.diag_write(args.memory_address, data)
            print(f"wrote {len(data)} byte(s) at 0x{args.memory_address:08X}: {data.hex(' ')}")
        else:
            # No retry: the robot resets right after its echo, which can get lost.
            mode = 2 if args.bootloader else 1
            print(json.dumps(await crab.diag(diag_reset_frame(mode), retries=0)))
            print("robot is resetting" + (" into its bootloader (update mode): `ota <image> "
                                          "--yes --in-bootloader` brings it back" if mode == 2 else ""))
    finally:
        await client.disconnect()


# --- firmware update -------------------------------------------
# App side, as the stock app does it: send ESC[24~ (F12),
# wait for 'C', then XMODEM-1K blocks (no block 0), image padded to 1024 with 0x1A,
# CRC-16/XMODEM big-endian, EOT at the end. Replies: ACK 06 / NAK 15 / CAN 18.
APP_BASE = 0x0800A000
# The flash page at 0x0801F800 holds the voice on/off flag, so an
# image must end before it: 86 KB max (real images are ~58 KB).
APP_MAX = 0x0801F800 - APP_BASE
STX, EOT, ACK, NAK, CAN = 0x02, 0x04, 0x06, 0x15, 0x18


def crc16_xmodem(data: bytes) -> int:
    crc = 0
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if crc & 0x8000 else crc << 1
            crc &= 0xFFFF
    return crc


def xmodem_blocks(image: bytes):
    """Yield the 1029-byte XMODEM-1K packets exactly as the stock app builds them."""
    if len(image) % 1024:
        image = image + b"\x1a" * (1024 - len(image) % 1024)
    for i in range(len(image) // 1024):
        blk = (i + 1) & 0xFF
        data = image[i * 1024:(i + 1) * 1024]
        crc = crc16_xmodem(data)
        yield bytes([STX, blk, blk ^ 0xFF]) + data + bytes([crc >> 8, crc & 0xFF])


def check_image(image: bytes) -> str | None:
    """Return a reason to refuse the image, or None if it looks like a Crab app image."""
    if not 1024 <= len(image) <= APP_MAX:
        return f"size {len(image)} outside 1 KB..{APP_MAX // 1024} KB"
    sp = int.from_bytes(image[0:4], "little")
    reset = int.from_bytes(image[4:8], "little")
    if not 0x20000000 < sp <= 0x20018000:
        return f"initial SP 0x{sp:08X} is not in SRAM"
    if not (reset & 1 and APP_BASE <= (reset & ~1) < APP_BASE + len(image)):
        return f"reset vector 0x{reset:08X} is not Thumb code inside an image at 0x{APP_BASE:08X}"
    return None


def _printable(data: bytes) -> str:
    return "".join(chr(b) if 32 <= b < 127 else "." for b in data)


async def _raw_session(args):
    """Connect and collect raw notifications into a queue (no Modbus framing)."""
    dev = await find_device(getattr(args, "address", None))
    if not dev:
        sys.exit("no Crab found (try: scan, or pass --address)")
    print(f"connecting to {dev.name or '?'} [{dev.address}] ...")
    client = BleakClient(dev)
    await client.connect()
    q: asyncio.Queue = asyncio.Queue()
    log = open(args.log, "a", encoding="utf-8") if getattr(args, "log", None) else None

    def on_notify(_c, data: bytearray):
        b = bytes(data)
        if log:
            log.write(f"rx {b.hex(' ')}  |{_printable(b)}|\n")
            log.flush()
        q.put_nowait(b)

    await client.start_notify(NOTIFY_UUID, on_notify)
    return client, q, log


async def cmd_listen(args):
    client, q, log = await _raw_session(args)
    try:
        print("printing every notification (Ctrl-C to stop)")
        while True:
            b = await q.get()
            print(f"  {b.hex(' ')}  |{_printable(b)}|")
    except (KeyboardInterrupt, asyncio.CancelledError):
        pass
    finally:
        if log:
            log.close()
        await client.disconnect()


async def cmd_reboot(args):
    """Reset the robot with F8 and print what the bootloader/firmware send while it boots."""
    client, q, log = await _raw_session(args)
    try:
        print(f"sending F8 (ESC[19~) -> robot resets; listening {args.seconds:.0f} s"
              + (" (reconnecting if the link drops)" if args.reconnect else ""))
        await client.write_gatt_char(WRITE_UUID, F8, response=False)
        loop = asyncio.get_running_loop()
        t0 = loop.time()
        end = t0 + args.seconds

        def on_notify(_c, data: bytearray):
            b = bytes(data)
            if log:
                log.write(f"rx {b.hex(' ')}  |{_printable(b)}|\n")
                log.flush()
            q.put_nowait(b)

        while (left := end - loop.time()) > 0:
            try:
                b = await asyncio.wait_for(q.get(), min(left, 0.25))
                print(f"  +{loop.time() - t0:5.1f}s  {b.hex(' ')}  |{_printable(b)}|")
                continue
            except asyncio.TimeoutError:
                pass
            if client.is_connected or not args.reconnect:
                continue
            # link dropped (the robot resets its BLE module at boot): reconnect and keep listening
            print(f"  +{loop.time() - t0:5.1f}s  link dropped; reconnecting ...")
            while loop.time() < end:
                try:
                    client = BleakClient(args.address or client.address)
                    await asyncio.wait_for(client.connect(), 10.0)
                    await client.start_notify(NOTIFY_UUID, on_notify)
                    print(f"  +{loop.time() - t0:5.1f}s  reconnected")
                    break
                except Exception as e:
                    await asyncio.sleep(0.5)
    finally:
        if log:
            log.close()
        try:
            await client.disconnect()
        except Exception:
            pass


async def _wait_byte(q: asyncio.Queue, wanted: set, timeout: float):
    """Wait for a notification whose first byte is in `wanted`; return it or None."""
    loop = asyncio.get_running_loop()
    end = loop.time() + timeout
    while (left := end - loop.time()) > 0:
        try:
            b = await asyncio.wait_for(q.get(), left)
        except asyncio.TimeoutError:
            return None
        if b and b[0] in wanted:
            return b[0]
        print(f"    (ignored {b.hex(' ')} |{_printable(b)}|)")
    return None


async def cmd_ota(args):
    image = open(args.image, "rb").read()
    why = check_image(image)
    if why:
        sys.exit(f"refusing {args.image}: {why}")
    blocks = list(xmodem_blocks(image))
    print(f"{args.image}: {len(image)} bytes -> {len(blocks)} blocks of 1 KB")
    if not args.yes:
        sys.exit("dry run only (image checks passed). Re-run with --yes to send it.")

    client, q, log = await _raw_session(args)
    try:
        char = client.services.get_characteristic(WRITE_UUID)
        with_resp = "write" in char.properties
        chunk = max(20, min(244, (client.mtu_size or 23) - 3))
        print(f"BLE chunk {chunk} B, write-with-response={with_resp}")

        async def send(data: bytes):
            for i in range(0, len(data), chunk):
                await client.write_gatt_char(WRITE_UUID, data[i:i + chunk], response=with_resp)
                if not with_resp:
                    await asyncio.sleep(0.01)

        if not args.in_bootloader:
            print("sending F12 (ESC[24~) -> robot reboots into its bootloader")
            await send(F12)
        print("waiting for 'C' from the bootloader ...")
        if await _wait_byte(q, {ord("C")}, 30.0) is None:
            sys.exit("no 'C' within 30 s; nothing was written. (Try --in-bootloader if it is "
                     "already waiting, or `listen` to see what it sends.)")

        for n, pkt in enumerate(blocks, 1):
            for attempt in range(10):
                await send(pkt)
                r = await _wait_byte(q, {ACK, NAK, CAN}, 15.0)
                if r == ACK:
                    break
                if r == CAN:
                    sys.exit(f"bootloader cancelled (CAN) at block {n}")
                print(f"  block {n}: {'NAK' if r == NAK else 'timeout'}, resending")
            else:
                sys.exit(f"block {n} failed 10 times; stopping. The robot should still be in "
                         "its bootloader: re-run with --in-bootloader.")
            print(f"  block {n}/{len(blocks)} ok", end="\r")
        print()
        await send(bytes([EOT]))
        r = await _wait_byte(q, {ACK, NAK}, 15.0)
        print("EOT acknowledged - update complete" if r == ACK else f"EOT reply: {r}")
    finally:
        if log:
            log.close()
        await client.disconnect()


def main():
    p = argparse.ArgumentParser(description="SEAUTO Crab BLE control/diagnostic tool")
    p.add_argument("--address", help="BLE MAC/UUID of the robot (skip scan)")
    sub = p.add_subparsers(dest="cmd", required=True)
    sub.add_parser("scan", help="list nearby BLE devices")
    sub.add_parser("info", help="read all status registers")
    g = sub.add_parser("get", help="read one register")
    g.add_argument("register", choices=REGS)
    s = sub.add_parser("set", help="write one register")
    s.add_argument("register", choices=[k for k, v in REGS.items() if v[3]])
    s.add_argument("value", type=lambda x: int(x, 0))
    m = sub.add_parser("monitor", help="poll status in a loop")
    m.add_argument("--interval", type=float, default=2.0)
    r = sub.add_parser("raw", help="send a raw frame (hex bytes)")
    r.add_argument("bytes", nargs="+", help="e.g. 01 03 00 07 00 01")
    r.add_argument("--no-crc", action="store_true", help="do not append CRC")
    li = sub.add_parser("listen", help="print every raw notification (e.g. bootloader text)")
    li.add_argument("--log", help="also append notifications to this file")
    rb = sub.add_parser("reboot", help="reset the robot (F8) and print what it sends while booting")
    rb.add_argument("--seconds", type=float, default=20.0, help="how long to listen")
    rb.add_argument("--reconnect", action="store_true",
                    help="reconnect and keep listening if the BLE link drops during the reset")
    rb.add_argument("--log", default="reboot_session.log", help="raw notification log file")
    o = sub.add_parser("ota", help="send a firmware image (stock app's update protocol)")
    o.add_argument("image", help="app image to write at 0x0800A000")
    o.add_argument("--yes", action="store_true", help="actually send (default: checks only)")
    o.add_argument("--in-bootloader", action="store_true",
                   help="robot is already waiting in its bootloader: skip the F12 trigger")
    o.add_argument("--log", default="ota_session.log", help="raw notification log file")
    integer = lambda value: int(value, 0)
    dr = sub.add_parser("diag-read", help="read memory through the diagnostic add-on")
    dr.add_argument("memory_address", type=integer)
    dr.add_argument("length", type=integer)
    dr.add_argument("--chunk", type=integer, default=64, help="bytes per request, 1..200 (default 64)")
    dr.add_argument("--out", help="write the bytes to this file instead of printing them")
    de = sub.add_parser("diag-recorder", help="read recorder samples (angles 0.1 deg, bump cm/s2)")
    de.add_argument("sequence", type=integer, nargs="?", default=0, help="first sample (default 0)")
    de.add_argument("--count", type=integer, help="stop after this many samples")
    de.add_argument("--csv", help="write the samples to this CSV file")
    sub.add_parser("diag-status", help="diagnostic build, boot safeguard, crash trace, recorder")
    dc = sub.add_parser("diag-control", help="recorder: period in ms (0 = no periodic samples)")
    dc.add_argument("period_ms", type=integer)
    dc.add_argument("--clear", action="store_true", help="clear the recorder")
    dc.add_argument("--on-change", action="store_true",
                    help="also sample when the navigation state changes")
    dc.add_argument("--stop-when-full", action="store_true",
                    help="keep the start of a run instead of the latest samples")
    dw = sub.add_parser("diag-write", help="write 1, 2 or 4 SRAM bytes with one store")
    dw.add_argument("memory_address", type=integer)
    dw.add_argument("bytes", nargs="+", help="hex bytes in memory order, e.g. 01 or 10 27 00 00")
    ds = sub.add_parser("diag-reset", help="reset the robot through the diagnostic add-on")
    ds.add_argument("--bootloader", action="store_true", help="reset into update mode")
    df = sub.add_parser("diag-floor", help="save battery discharge floor, 0..100 percent (0 disables)")
    df.add_argument("percent", type=integer)
    dn = sub.add_parser("diag-nav", help="volatile navigation setting (default off; requires diag-2.4)")
    dn.add_argument("setting", choices=DIAG_NAV_CONFIG)
    dn.add_argument("value", type=integer)
    sub.add_parser("diag-nav-status", help="navigation configuration/state (requires diag-2.8)")
    dm = sub.add_parser("diag-map", help="download the visited-area map of the last cycle (requires diag-3.0+)")
    dm.add_argument("--out", required=True, help="raw map file for tools/navmap.py")
    args = p.parse_args()

    handler = {"scan": cmd_scan, "info": cmd_info, "get": cmd_get, "set": cmd_set,
               "monitor": cmd_monitor, "raw": cmd_raw, "listen": cmd_listen,
               "reboot": cmd_reboot, "ota": cmd_ota,
               **dict.fromkeys(("diag-read", "diag-recorder", "diag-status", "diag-control",
                                "diag-write", "diag-reset", "diag-floor", "diag-nav", "diag-nav-status",
                                "diag-map"), cmd_diag)}[args.cmd]
    try:
        asyncio.run(handler(args))
    except (ValueError, TimeoutError) as error:
        if not args.cmd.startswith("diag-"):
            raise
        sys.exit(f"{args.cmd}: {error}" if str(error) else
                 f"{args.cmd}: no reply (is the robot running the diagnostic image, and not "
                 "waiting in its bootloader?)")


if __name__ == "__main__":
    main()
