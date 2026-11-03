"""Emulate the firmware's C-library startup (__scatterload) to recover initialised RAM.

The RW data (register descriptor table, default settings, thresholds ...) is stored
compressed in flash and unpacked at boot.  This runs the firmware's own unpacker in the
unicorn emulator, then dumps RAM and decodes the Modbus register descriptor table.

usage: python -I tools/fwram.py <firmware.bin> <outdir>
outputs: <outdir>/ram_init.bin (RAM image from 0x20000000), <outdir>/ram_init.txt (report)
"""
import os
import struct
import sys

from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB, UC_HOOK_CODE
from unicorn.arm_const import UC_ARM_REG_SP, UC_ARM_REG_LR

FW, OUT = sys.argv[1], sys.argv[2]
os.makedirs(OUT, exist_ok=True)
img = open(FW, "rb").read()
BASE = 0x0800A000
RAM, RAM_SIZE = 0x20000000, 0x20000
SCATTERLOAD = 0x0800AB68          # __scatterload
STOP_AT = 0x0800AB80              # bl __main_after_scatterload (jump to main)
REGION_TABLE = (0x08018560, 0x08018580)

uc = Uc(UC_ARCH_ARM, UC_MODE_THUMB)
uc.mem_map(0x08000000, 0x80000)
uc.mem_write(BASE, img)
uc.mem_map(RAM, RAM_SIZE)
sp0 = struct.unpack_from("<I", img, 0)[0]
uc.reg_write(UC_ARM_REG_SP, sp0)
uc.reg_write(UC_ARM_REG_LR, 0xFFFFFFFF)

steps = [0]
def hook(u, addr, size, _):
    steps[0] += 1
    if addr == STOP_AT:
        u.emu_stop()
uc.hook_add(UC_HOOK_CODE, hook)
uc.emu_start(SCATTERLOAD | 1, 0xFFFFFFFF, count=50_000_000)

ram = bytes(uc.mem_read(RAM, RAM_SIZE))
open(os.path.join(OUT, "ram_init.bin"), "wb").write(ram)
w32 = lambda a: struct.unpack_from("<I", ram, a - RAM)[0]

lines = [f"emulated {steps[0]} instructions; stopped at 0x{STOP_AT:08X}", "",
         "scatter-load regions (src -> dst, size, handler):"]
for t in range(*REGION_TABLE, 16):
    src, dst, size, fn = struct.unpack_from("<IIII", img, t - BASE)
    lines.append(f"  0x{src:08X} -> 0x{dst:08X}  size=0x{size:X}  fn=0x{fn:08X}")

TYPES = {0: "u8", 1: "s8", 2: "u16", 3: "s16", 4: "u32", 5: "s32", 6: "?6"}
ACC = {2: "holding R/W", 3: "input RO"}
NAMES = ["model", "fw_version", "serial", "battery_%", "on_off", "mode", "speed",
         "cycle_time", "voice_reminder", "unlock"]
lines += ["", "Modbus register descriptors @0x20000B38 (12 bytes each):",
          "reg  b0  b1 type  access        ptr_a       ptr_b       value@ptr_a  name"]
for i in range(10):
    e = ram[0xB38 + 12 * i: 0xB38 + 12 * i + 12]
    b0, b1, t, acc = e[0], e[1], e[2], e[3]
    pa, pb = struct.unpack_from("<II", e, 4)
    val = ""
    if RAM <= pa < RAM + RAM_SIZE - 4:
        val = {0: ram[pa - RAM], 1: ram[pa - RAM], 2: struct.unpack_from("<H", ram, pa - RAM)[0],
               3: struct.unpack_from("<h", ram, pa - RAM)[0]}.get(t, w32(pa))
    lines.append(f"{i:>3} {b0:>3} {b1:>3} {TYPES.get(t, t):<5} {ACC.get(acc, str(acc)):<13} "
                 f"0x{pa:08X}  0x{pb:08X}  {val!s:<11}  {NAMES[i]}")

lines += ["", "non-zero initialised words in RAM (address: hex / decimal):"]
end = 0x4000
for a in range(0, end, 4):
    v = struct.unpack_from("<I", ram, a)[0]
    if v:
        lines.append(f"  0x{RAM + a:08X}: 0x{v:08X}  {v}")
open(os.path.join(OUT, "ram_init.txt"), "w", encoding="utf-8").write("\n".join(lines) + "\n")
print("\n".join(lines[:40]))
print(f"... full report: {os.path.join(OUT, 'ram_init.txt')}")
