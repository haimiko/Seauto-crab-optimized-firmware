#!/usr/bin/env python3
"""Snapshot of a robot running the diagnostic build, over ONE BLE connection (read-only).

Reads the add-on's status, RAM 0x20000000-0x20004000 (vendor data + stack + add-on variables),
and the GPIO port registers; prints the variables the power-off / fault analysis uses and writes the RAM image to --out.

    python -I tools/diag_snapshot.py --address AA:BB:CC:DD:EE:FF --out analysis/ram_NAME.bin

One connection instead of one per crab_ctl.py command. Peripherals whose
registers change something when read (ADC data, UART data) are left alone on purpose.
"""
import argparse
import asyncio
import hashlib
import json
import os
import struct
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import crab_ctl as c  # noqa: E402
from bleak import BleakClient  # noqa: E402

RAM_BASE, RAM_LENGTH = 0x20000000, 0x4000
WORDS = (
    ("in-water flag", 0x20000060),
    ("cycle flag", 0x200000CC),
    ("fault byte", 0x200000DC),
    ("persistent word (BPR19 copy)", 0x20000330),
    ("guard", 0x20000338),
    ("power-off countdown, ms", 0x20000AD0),
    ("AD4 shutting down / AD6 idle auto-off armed", 0x20000AD4),
    ("button hold ms / repeat", 0x20000AD8),
    ("0x20000AE4", 0x20000AE4),
)
PORTS = (("GPIOA", 0x40010800), ("GPIOB", 0x40010C00), ("GPIOC", 0x40011000),
         ("GPIOD", 0x40011400), ("GPIOF", 0x40011C00))


def show(status):
    print(json.dumps({k: f"0x{v:08X}" if k in c._HEX_FIELDS else v for k, v in status.items()}))


async def main(args):
    dev = None
    for _ in range(4):          # the scan misses the robot now and then
        dev = await c.find_device(args.address, timeout=15.0)
        if dev:
            break
    if not dev:
        sys.exit("robot not found")
    client = BleakClient(dev)
    await client.connect()
    print(f"connected {time.strftime('%H:%M:%S')}")
    crab = c.Crab(client)
    await crab.start()
    try:
        show(await crab.diag(c.diag_status_frame()))
        await crab.diag_learn_rc_count()
        ram = await crab.diag_read(RAM_BASE, RAM_LENGTH, 200)
        if args.out:
            with open(args.out, "wb") as f:
                f.write(ram)
            print(f"{args.out}: {len(ram)} bytes from 0x{RAM_BASE:08X}, "
                  f"SHA-256 {hashlib.sha256(ram).hexdigest()}")
        for name, address in WORDS:
            print(f"  0x{address:08X} = 0x{struct.unpack_from('<I', ram, address - RAM_BASE)[0]:08X}  {name}")
        for name, base in PORTS:
            try:
                cfglr, cfghr, idt, odt = struct.unpack("<4I", await crab.diag_read(base, 16, 16))
                print(f"  {name}: config {cfghr:08X} {cfglr:08X}  inputs {idt:04X}  outputs {odt:04X}")
            except c.DiagException as e:
                print(f"  {name}: {e}")
        show(await crab.diag(c.diag_status_frame()))
    finally:
        await client.disconnect()
        print(f"disconnected {time.strftime('%H:%M:%S')}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--address", help="BLE address of the robot (default: scan by name)")
    parser.add_argument("--out", help="write the RAM image to this file")
    try:
        asyncio.run(main(parser.parse_args()))
    except (ValueError, asyncio.TimeoutError) as e:
        sys.exit(f"snapshot failed: {e or 'no reply (diagnostic image running?)'}")
