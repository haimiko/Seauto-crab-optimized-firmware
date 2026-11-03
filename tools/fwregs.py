"""Decode the Modbus register descriptor table of the SEAUTO Crab firmware.

The table lives in RAM at 0x20000B38 (12 bytes per register) and is filled at boot by the
ARM C library scatter-loader from an RW init image in flash.  This script finds the
scatter-load region table, maps RAM -> flash init image, and decodes the descriptors.
Falls back to a pattern scan if the region table is not found.
"""
import struct, sys

data = open(sys.argv[1], "rb").read()
SIZE = len(data)
vt = struct.unpack_from("<96I", data, 0)
BASE = min((v & ~1) for v in vt if 0x08000000 <= (v & ~1) < 0x08080000) & 0xFFFFF000
END = BASE + SIZE
TABLE = 0x20000B38
NREGS = 10

def w(off):
    return struct.unpack_from("<I", data, off)[0]

def inflash(a):
    return BASE <= a < END

# ---- 1. scatter-load region table: tuples {src, dst, size, fn} ----
regions = []
for off in range(0, SIZE - 16, 4):
    src, dst, size, fn = struct.unpack_from("<IIII", data, off)
    if inflash(src) and 0x20000000 <= dst < 0x20010000 and 0 < size < 0x8000 and inflash(fn & ~1) and (fn & 1):
        # a real table has consecutive tuples; require the next tuple to look sane or be end
        regions.append((off, src, dst, size, fn))
print(f"base=0x{BASE:08X} end=0x{END:08X}")
print("scatter-load region candidates (file off: src -> dst, size, handler):")
for off, src, dst, size, fn in regions:
    kind = "copy" if inflash(src) and src != dst else "?"
    print(f"  0x{off:05X}: 0x{src:08X} -> 0x{dst:08X}  size=0x{size:X}  fn=0x{fn:08X}")

# RW copy region covering TABLE
rw = [r for r in regions if r[2] <= TABLE < r[2] + r[3]]
src_off = None
if rw:
    off, src, dst, size, fn = rw[0]
    src_off = (src - BASE) + (TABLE - dst)
    print(f"\nTABLE 0x{TABLE:08X} is in RW region dst=0x{dst:08X} src=0x{src:08X}; init image file offset 0x{src_off:X}")
else:
    print("\nno scatter region covers the table; falling back to pattern scan")
    # pattern: 12-byte entries with byte[3] in (2,3), byte[2] < 7, +4 and +8 pointers into RAM
    best = None
    for s in range(0, SIZE - 12 * NREGS, 4):
        n = 0
        while n < 16 and s + 12 * n + 12 <= SIZE:
            e = data[s + 12 * n: s + 12 * n + 12]
            t, acc = e[2], e[3]
            p1, p2 = struct.unpack_from("<II", e, 4)
            if acc in (2, 3) and t < 7 and 0x20000000 <= p1 < 0x20010000 and (p2 == 0 or 0x20000000 <= p2 < 0x20010000):
                n += 1
            else:
                break
        if n >= 6 and (best is None or n > best[1]):
            best = (s, n)
    if best:
        src_off = best[0]
        print(f"pattern scan: {best[1]} consecutive descriptors at file offset 0x{src_off:X} (flash 0x{BASE + src_off:08X})")

if src_off is None:
    sys.exit("descriptor table not found")

# ---- 2. decode ----
KNOWN = {
    0x20000B2C: "const 499 (modbus_init)",
    0x20000B30: "firmware version 1040008012",
    0x20000B34: "const 1000000000",
}
TYPES = {0: "u8", 1: "s8", 2: "u16", 3: "s16", 4: "u32", 5: "s32", 6: "partial"}
ACC = {2: "holding R/W (FC03/06)", 3: "input RO (FC04)"}
print(f"\nModbus register descriptors (table @0x{TABLE:08X}):")
print(f"{'reg':>3} {'b0':>3} {'b1':>3} {'type':<8} {'access':<22} {'read_ptr':<12} {'write_ptr':<12} note")
for i in range(NREGS + 2):
    o = src_off + 12 * i
    if o + 12 > SIZE:
        break
    b0, b1, t, acc = data[o], data[o + 1], data[o + 2], data[o + 3]
    p1, p2 = struct.unpack_from("<II", data, o + 4)
    note = KNOWN.get(p1, "") or KNOWN.get(p2, "")
    print(f"{i:>3} {b0:>3} {b1:>3} {TYPES.get(t, str(t)):<8} {ACC.get(acc, str(acc)):<22} 0x{p1:08X}   0x{p2:08X}   {note}")

# ---- 3. the three words right before the table (set by the firmware) ----
if rw:
    off, src, dst, size, fn = rw[0]
    for a in (0x20000B2C, 0x20000B30, 0x20000B34):
        fo = (src - BASE) + (a - dst)
        print(f"init image value at 0x{a:08X}: 0x{w(fo):08X}")
