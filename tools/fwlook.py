import math
import re
import struct
import sys
from collections import Counter


def entropy(b):
    if not b:
        return 0.0
    c = Counter(b)
    n = len(b)
    return -sum(v / n * math.log2(v / n) for v in c.values())


def hexdump(b, base=0, width=16):
    out = []
    for i in range(0, len(b), width):
        chunk = b[i:i + width]
        hx = " ".join(f"{x:02x}" for x in chunk)
        asc = "".join(chr(x) if 32 <= x < 127 else "." for x in chunk)
        out.append(f"{base + i:08x}  {hx:<{width * 3}} {asc}")
    return "\n".join(out)


def look(path):
    d = open(path, "rb").read()
    print("=" * 78)
    print(path, len(d), "bytes", f"(0x{len(d):x})")
    print(f"overall entropy: {entropy(d):.3f} bits/byte")
    print("-- first 0x100 --")
    print(hexdump(d[:0x100]))
    print("-- last 0x40 --")
    print(hexdump(d[-0x40:], len(d) - 0x40))

    print("-- entropy per 4 KiB block --")
    print(" ".join(f"{entropy(d[i:i + 4096]):.2f}" for i in range(0, len(d), 4096)))

    # Cortex-M vector table guess at a few candidate offsets
    for off in (0, 0x10, 0x20, 0x40, 0x80, 0x100, 0x200):
        if off + 8 > len(d):
            continue
        sp, rv = struct.unpack_from("<II", d, off)
        if (sp >> 24) in (0x20, 0x10) and (rv & 1) and (rv >> 24) in (0x00, 0x08, 0x01, 0x10):
            words = struct.unpack_from("<16I", d, off)
            print(f"-- plausible Cortex-M vector table at 0x{off:x}: SP=0x{sp:08x} Reset=0x{rv:08x}")
            print("   " + " ".join(f"{w:08x}" for w in words))

    strs = re.findall(rb"[\x20-\x7e]{5,}", d)
    print(f"-- ascii strings >=5 chars: {len(strs)} --")
    for s in strs[:120]:
        print("  ", s.decode("ascii"))
    return d


a = look(sys.argv[1])
b = look(sys.argv[2])

print("=" * 78)
n = min(len(a), len(b))
same = sum(1 for i in range(n) if a[i] == b[i])
print(f"byte-identical positions over common length {n}: {same} ({same / n * 100:.1f}%)")
first = next((i for i in range(n) if a[i] != b[i]), None)
print("first differing offset:", None if first is None else hex(first))
