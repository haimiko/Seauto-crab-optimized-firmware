import re
import struct
import sys
import zipfile

sys.stdout.reconfigure(encoding="utf-8")
apk, pattern = sys.argv[1], sys.argv[2]
rx = re.compile(pattern, re.I)


def uleb(b, o):
    r = s = 0
    while True:
        x = b[o]; o += 1
        r |= (x & 0x7F) << s; s += 7
        if not x & 0x80:
            return r, o


with zipfile.ZipFile(apk) as z:
    dexes = [n for n in z.namelist() if re.fullmatch(r"classes\d*\.dex", n)]
    for name in dexes:
        d = z.read(name)
        n, off = struct.unpack_from("<II", d, 0x38)
        hits = []
        for i in range(n):
            so = struct.unpack_from("<I", d, off + 4 * i)[0]
            _, p = uleb(d, so)
            e = d.index(b"\0", p)
            s = d[p:e].decode("utf-8", "replace")
            if rx.search(s):
                hits.append(s)
        print(f"== {name}: {n} strings, {len(hits)} matching ==")
        for s in hits:
            print("  ", s[:200])
