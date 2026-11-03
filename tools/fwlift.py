import struct, sys
from capstone import Cs, CS_ARCH_ARM, CS_MODE_THUMB
from capstone.arm import *

# usage: fwlift.py [START END [IMAGE BASE]]   (defaults: nav in the app image; e.g. the
#        bootloader: fwlift.py 08006360 08006594 bootloader_dump.bin 08000000)
FW = sys.argv[3] if len(sys.argv) > 3 else "stock.bin"
BASE = int(sys.argv[4], 16) if len(sys.argv) > 4 else 0x0800A000
data = open(FW, "rb").read()
START = int(sys.argv[1], 16) if len(sys.argv) > 1 else 0x0801019C
END = int(sys.argv[2], 16) if len(sys.argv) > 2 else 0x080134EC

md = Cs(CS_ARCH_ARM, CS_MODE_THUMB)
md.detail = True

NAMES = {
    0x20000018: "STATE", 0x200000A8: "STEP", 0x200000AC: "T_ms",
    0x200003F8: "ROLL", 0x200003FC: "PITCH", 0x20000400: "YAW",
    0x20000410: "BASE_SPD", 0x200000BC: "TGT_HDG", 
    0x200004BD: "CTYPE", 0x200004BC: "CSUB", 0x200001A8: "STILLCNT", 0x200004E8: "RECOV",
    0x200004E0: "MODEL", 0x200000F8: "BATT_EMPTY", 0x20000060: "F060",
    0x2000032C: "HDG_FLIP", 0x20000414: "KP_f", 0x20000418: "INTEG_f",
    0x20000B1A: "CYCLE_MIN", 0x200004C8: "LTRACK", 0x200004CC: "RTRACK",
}

def gname(a):
    if a in NAMES:
        return NAMES[a]
    if 0x20000000 <= a < 0x20010000:
        return "g%04X" % (a & 0xFFFF)
    if 0x40000000 <= a < 0x60000000:
        return "IO_%08X" % a
    return "M_%08X" % a

def rd32(a):
    return struct.unpack_from("<I", data, a - BASE)[0]

def dis(addr):
    off = addr - BASE
    for i in md.disasm(data[off:off + 4], addr, 1):
        return i
    return None

# ---------- recursive descent ----------
insns = {}
TABLES = {}
todo = [START]
leaders = {START}
while todo:
    a = todo.pop()
    while True:
        if a in insns or not (START <= a < END):
            break
        i = dis(a)
        if i is None:
            print("decode fail", hex(a)); break
        insns[a] = i
        m = i.mnemonic
        nxt = a + i.size
        ops = i.operands
        if m in ("b", "b.w") and not i.cc in (ARM_CC_INVALID,) and i.cc != ARM_CC_AL:
            pass
        if m.startswith("cbz") or m.startswith("cbnz"):
            t = ops[1].imm
            leaders.add(t); leaders.add(nxt); todo.append(t)
            a = nxt; continue
        if m.startswith("b") and m not in ("bl", "blx", "bic", "bics", "bfi", "bfc", "bkpt") and not m.startswith("bic"):
            t = ops[0].imm if ops and ops[0].type == ARM_OP_IMM else None
            cond = m.split(".")[0] not in ("b",)
            if t is not None:
                leaders.add(t); todo.append(t)
            if m.startswith("bx"):
                break
            if cond:
                leaders.add(nxt); a = nxt; continue
            leaders.add(nxt)
            break
        if m.startswith("pop") and any(op.type == ARM_OP_REG and op.reg == ARM_REG_PC for op in ops):
            leaders.add(nxt); break
        if m in ("tbb", "tbh"):
            tb = a + 4
            ents = []
            p = tb
            lim = 1 << 40
            while p < lim and len(ents) < 64:
                if m == "tbb":
                    t = tb + 2 * data[p - BASE]; p += 1
                else:
                    t = tb + 2 * struct.unpack_from("<H", data, p - BASE)[0]; p += 2
                ents.append(t); lim = min(lim, t)
            TABLES[a] = ents
            for t in ents:
                leaders.add(t); todo.append(t)
            break
        a = nxt

addrs = sorted(insns)

# ---------- lifting ----------
class V:
    __slots__ = ("c", "e")
    def __init__(self, c=None, e=None):
        self.c = c; self.e = e
    def s(self):
        if self.c is not None:
            c = self.c & 0xFFFFFFFF
            if c >= 0x80000000 and c < 0xFFFF0000:
                return "0x%X" % c
            if c >= 0xFFFF0000:
                return str(c - 0x100000000)
            if 0x20000000 <= c < 0x20010000:
                return "&" + gname(c)
            return str(c) if c < 4096 else "0x%X" % c
        return self.e
    def __repr__(self):
        return self.s()

def C(v): return V(c=v & 0xFFFFFFFF)
def E(s): return V(e=s)

REGN = {}
for r in range(ARM_REG_ENDING):
    try:
        REGN[r] = md.reg_name(r)
    except Exception:
        pass

LIB = {0x0800A3F4: "dadd", 0x0800A542: "dmul", 0x0800A704: "i2f", 0x0800A716: "u2f",
       0x0800A720: "i2d", 0x0800A826: "f2d", 0x0800A84C: "dcmp", 0x0800A87C: "dcmp_rev",
       0x0800A7B6: "d2i", 0x0800A264: "fadd?", 0x0800A314: "fmul?", 0x08014154: "ret1",
       0x08013CAC: "stuck_detect"}

def dval(lo, hi):
    if lo.c is not None and hi.c is not None:
        return "%g" % struct.unpack("<d", struct.pack("<II", lo.c, hi.c))[0]
    if hi.e == (lo.s() + ".hi"):
        return lo.s()
    if hi.e == "(%s.hi &~ 0x80000000)" % lo.s():
        return "fabs(%s)" % lo.s()
    return "(%s:%s)" % (lo.s(), hi.s())

def fval(v):
    if v.c is not None:
        return "%gf" % struct.unpack("<f", struct.pack("<I", v.c))[0]
    return v.s()

out = []
CONDS = {ARM_CC_EQ: "==", ARM_CC_NE: "!=", ARM_CC_HS: ">=u", ARM_CC_LO: "<u", ARM_CC_MI: "<0(mi)",
         ARM_CC_PL: ">=0(pl)", ARM_CC_VS: "vs", ARM_CC_VC: "vc", ARM_CC_HI: ">u", ARM_CC_LS: "<=u",
         ARM_CC_GE: ">=", ARM_CC_LT: "<", ARM_CC_GT: ">", ARM_CC_LE: "<="}

def clobber(regs):
    for r in (ARM_REG_R0, ARM_REG_R1, ARM_REG_R2, ARM_REG_R3, ARM_REG_R12, ARM_REG_LR):
        regs.pop(r, None)

def lift_block(start, end_excl, init_flags=None, init_regs=None):
    regs = dict(init_regs) if init_regs else {}
    flags = init_flags
    dirty = set()
    def get(r):
        if r == ARM_REG_PC:
            return None
        return regs.get(r, E(REGN[r]))
    def setr(r, v):
        regs[r] = v; dirty.add(r)
    def opval(op):
        if op.type == ARM_OP_IMM:
            return C(op.imm)
        if op.type == ARM_OP_REG:
            v = get(op.reg)
            if op.shift.type != ARM_SFT_INVALID and op.shift.value:
                st = {ARM_SFT_LSL: "<<", ARM_SFT_LSR: ">>u", ARM_SFT_ASR: ">>", ARM_SFT_ROR: "ror"}.get(op.shift.type, "?sh")
                if v.c is not None and st == "<<":
                    return C(v.c << op.shift.value)
                return E("(%s %s %d)" % (v.s(), st, op.shift.value))
            return v
        return E("?")
    def memaddr(op):
        m = op.mem
        base = m.base
        disp = m.disp
        if base == ARM_REG_SP:
            return ("sp", disp)
        b = get(base)
        if m.index != 0:
            idx = get(m.index)
            if b.c is not None:
                return ("e", "%s[%s]" % (gname(b.c), idx.s()))
            return ("e", "(%s+%s)" % (b.s(), idx.s()))
        if b.c is not None:
            return ("g", b.c + disp)
        return ("e", "(%s+%d)" % (b.s(), disp) if disp else b.s())
    def memname(ma, sz):
        k, v = ma
        if k == "sp":
            return "sp_%X" % v
        if k == "g":
            return gname(v)
        return "*" + v
    a = start
    lines = []
    while a < end_excl:
        i = insns[a]
        m = i.mnemonic
        base_m = m.split(".")[0]
        ops = i.operands
        txt = None
        try:
            if base_m == "ldr" and ops[1].type == ARM_OP_MEM and ops[1].mem.base == ARM_REG_PC:
                la = ((a + 4) & ~3) + ops[1].mem.disp
                setr(ops[0].reg, C(rd32(la)))
            elif base_m in ("ldr", "ldrb", "ldrh", "ldrsb", "ldrsh") and ops[1].type == ARM_OP_MEM:
                ma = memaddr(ops[1])
                pre = {"ldr": "", "ldrb": "u8 ", "ldrh": "u16 ", "ldrsb": "s8 ", "ldrsh": "s16 "}[base_m]
                setr(ops[0].reg, E(pre + memname(ma, 4)))
                if len(ops) > 2 or i.writeback:
                    lines.append("  ?? writeback " + m + " " + i.op_str)
            elif base_m in ("str", "strb", "strh") and ops[1].type == ARM_OP_MEM:
                ma = memaddr(ops[1])
                pre = {"str": "", "strb": "(u8)", "strh": "(u16)"}[base_m]
                v = get(ops[0].reg)
                txt = "%s = %s%s" % (memname(ma, 4), pre, v.s())
                if i.writeback:
                    txt += "   ?? writeback"
            elif base_m == "strd":
                ma = memaddr(ops[2])
                lo, hi = get(ops[0].reg), get(ops[1].reg)
                txt = "%s = D(%s)" % (memname(ma, 8), dval(lo, hi))
            elif base_m == "ldrd":
                ma = memaddr(ops[2])
                n = memname(ma, 8)
                setr(ops[0].reg, E(n)); setr(ops[1].reg, E(n + ".hi"))
            elif base_m == "ldm":
                b = get(ops[0].reg)
                for k, op in enumerate(ops[1:]):
                    if b.c is not None:
                        nm = gname(b.c + 4 * k)
                    else:
                        nm = "*(%s+%d)" % (b.s(), 4 * k)
                    setr(op.reg, E(nm))
                if b.c is not None and len(ops) == 3:
                    setr(ops[2].reg, E(gname(b.c) + ".hi"))
            elif base_m == "stm":
                b = get(ops[0].reg)
                vals = [get(op.reg) for op in ops[1:]]
                if b.c is not None:
                    if len(vals) == 2:
                        txt = "%s = D(%s)" % (gname(b.c), dval(vals[0], vals[1]))
                    else:
                        txt = "; ".join("%s = %s" % (gname(b.c + 4 * k), v.s()) for k, v in enumerate(vals))
                else:
                    txt = "stm %s <- %s" % (b.s(), vals)
            elif base_m in ("mov", "movs", "movw", "mvn", "mvns", "movt"):
                src = opval(ops[1])
                if base_m.startswith("mvn"):
                    src = C(~src.c) if src.c is not None else E("~" + src.s())
                if base_m == "movt":
                    old = get(ops[0].reg)
                    src = C((old.c & 0xFFFF) | (ops[1].imm << 16)) if old.c is not None else E("movt?")
                setr(ops[0].reg, src)
                if base_m.endswith("s"):
                    flags = ("val", src)
            elif base_m in ("add", "adds", "sub", "subs", "rsb", "rsbs", "mul", "muls", "sdiv", "udiv",
                            "lsl", "lsls", "lsr", "lsrs", "asr", "asrs", "and", "ands", "orr", "orrs",
                            "eor", "eors", "bic", "bics"):
                if len(ops) == 3:
                    x, y = opval(ops[1]), opval(ops[2])
                else:
                    x, y = get(ops[0].reg), opval(ops[1])
                op = base_m.rstrip("s") if base_m not in ("adds",) else "add"
                if base_m in ("asrs",): op = "asr"
                if base_m in ("lsrs",): op = "lsr"
                if base_m in ("lsls",): op = "lsl"
                if base_m in ("subs",): op = "sub"
                if base_m in ("rsbs",): op = "rsb"
                if base_m in ("ands",): op = "and"
                if base_m in ("orrs",): op = "orr"
                if base_m in ("eors",): op = "eor"
                if base_m in ("bics",): op = "bic"
                if base_m in ("muls",): op = "mul"
                sym = {"add": "+", "sub": "-", "mul": "*", "sdiv": "/", "udiv": "/u", "lsl": "<<", "lsr": ">>u",
                       "asr": ">>", "and": "&", "orr": "|", "eor": "^", "bic": "&~"}
                if op == "rsb":
                    x, y = y, x; op = "sub"
                res = None
                if x.c is not None and y.c is not None:
                    xc, yc = x.c, y.c
                    sx = xc - (1 << 32) if xc >= 1 << 31 else xc
                    sy = yc - (1 << 32) if yc >= 1 << 31 else yc
                    try:
                        res = C({"add": xc + yc, "sub": xc - yc, "mul": xc * yc, "lsl": xc << yc, "lsr": xc >> yc,
                                 "asr": sx >> yc, "and": xc & yc, "orr": xc | yc, "eor": xc ^ yc, "bic": xc & ~yc,
                                 "sdiv": int(sx / sy) if sy else 0, "udiv": xc // yc if yc else 0}[op])
                    except KeyError:
                        res = None
                if res is None:
                    if op == "sub" and x.c == 0:
                        res = E("-" + y.s())
                    else:
                        res = E("(%s %s %s)" % (x.s(), sym[op], y.s()))
                setr(ops[0].reg, res)
                if base_m.endswith("s") and base_m not in ("sdiv",):
                    flags = ("val", res)
            elif base_m == "mla":
                x, y, z = get(ops[1].reg), get(ops[2].reg), get(ops[3].reg)
                setr(ops[0].reg, E("(%s * %s + %s)" % (x.s(), y.s(), z.s())))
            elif base_m == "mls":
                x, y, z = get(ops[1].reg), get(ops[2].reg), get(ops[3].reg)
                setr(ops[0].reg, E("(%s - %s * %s)" % (z.s(), x.s(), y.s())))
            elif base_m in ("ubfx", "sbfx"):
                x = get(ops[1].reg)
                setr(ops[0].reg, E("%s(%s,%d,%d)" % (base_m, x.s(), ops[2].imm, ops[3].imm)))
            elif base_m in ("uxtb", "uxth", "sxtb", "sxth"):
                setr(ops[0].reg, E("%s(%s)" % (base_m, get(ops[1].reg).s())))
            elif base_m in ("cmp", "cmn", "tst"):
                x, y = get(ops[0].reg), opval(ops[1])
                if base_m == "cmn":
                    y = C(-y.c) if y.c is not None else E("-" + y.s())
                flags = (base_m if base_m == "tst" else "cmp", x, y)
            elif base_m in ("cbz", "cbnz"):
                x = get(ops[0].reg)
                txt = "if (%s %s 0) goto L_%08X" % (x.s(), "==" if base_m == "cbz" else "!=", ops[1].imm)
            elif base_m[0] == "b" and base_m not in ("bl", "blx", "bx", "bic", "bics") and ops and ops[0].type == ARM_OP_IMM:
                t = ops[0].imm
                if i.cc in (ARM_CC_AL, ARM_CC_INVALID):
                    txt = "goto L_%08X" % t
                else:
                    cs = CONDS[i.cc]
                    if flags is None:
                        cnd = "FLAGS?? %s" % cs
                    elif flags[0] == "cmp":
                        cnd = "%s %s %s" % (flags[1].s(), cs, flags[2].s())
                    elif flags[0] == "tst":
                        cnd = "(%s & %s) %s 0" % (flags[1].s(), flags[2].s(), cs)
                    elif flags[0] == "dcmp":
                        cnd = "D[%s] %s D[%s]" % (flags[1], cs, flags[2])
                    else:
                        cnd = "%s %s 0" % (flags[1].s(), cs)
                    txt = "if (%s) goto L_%08X" % (cnd, t)
            elif base_m == "bl":
                t = ops[0].imm
                nm = LIB.get(t, "sub_%08X" % t)
                r0, r1, r2, r3 = (get(r) for r in (ARM_REG_R0, ARM_REG_R1, ARM_REG_R2, ARM_REG_R3))
                if nm in ("dadd", "dmul"):
                    txt = "ret = %s(%s, %s)" % (nm, dval(r0, r1), dval(r2, r3))
                    res = (E(nm + "(%s, %s)" % (dval(r0, r1), dval(r2, r3))))
                    clobber(regs); setr(ARM_REG_R0, res); setr(ARM_REG_R1, E(res.s() + ".hi"))
                    txt = None
                elif nm in ("i2d", "f2d"):
                    res = E("%s(%s)" % (nm, r0.s() if nm == "i2d" else fval(r0)))
                    clobber(regs); setr(ARM_REG_R0, res); setr(ARM_REG_R1, E(res.s() + ".hi"))
                elif nm in ("i2f", "u2f"):
                    res = E("%s(%s)" % (nm, r0.s()))
                    clobber(regs); setr(ARM_REG_R0, res)
                elif nm in ("dcmp", "dcmp_rev"):
                    A, B = dval(r0, r1), dval(r2, r3)
                    if nm == "dcmp_rev":
                        A, B = B, A
                    flags = ("dcmp", A, B)
                    clobber(regs)
                else:
                    txt = "call %s(r0=%s, r1=%s, r2=%s, r3=%s)" % (nm, r0.s(), r1.s(), r2.s(), r3.s())
                    clobber(regs); setr(ARM_REG_R0, E("ret_" + nm))
            elif base_m in ("tbb", "tbh"):
                txt = "switch (%s) { %s }" % (get(ops[0].mem.index).s(), ", ".join("%d: L_%08X" % (k, t) for k, t in enumerate(TABLES[a])))
            elif base_m in ("push", "pop"):
                txt = m + " " + i.op_str
            else:
                txt = "RAW %s %s" % (m, i.op_str)
        except Exception as ex:
            txt = "ERR %s %s (%s)" % (m, i.op_str, ex)
        if txt:
            lines.append("  %08X  %s" % (a, txt))
        a += i.size
        if a not in insns:
            break
    return lines, regs, dirty, flags

blocks = sorted(l for l in leaders if l in insns)
# predecessor map
preds = {b: [] for b in blocks}
for bi, b in enumerate(blocks):
    nb = blocks[bi + 1] if bi + 1 < len(blocks) else END
    lastaddr = max(x for x in insns if b <= x < nb)
    li = insns[lastaddr]
    m = li.mnemonic
    succ = []
    isbr = m[0] == "b" and m not in ("bl", "blx", "bic", "bics") and li.operands and li.operands[0].type == ARM_OP_IMM
    if m.startswith(("cbz", "cbnz")):
        succ = [li.operands[1].imm, nb]
    elif isbr:
        succ = [li.operands[0].imm] + ([nb] if li.cc not in (ARM_CC_AL, ARM_CC_INVALID) else [])
    elif m.startswith("pop") or m in ("tbb", "tbh") or m.startswith("bx"):
        succ = TABLES.get(lastaddr, [])
    else:
        succ = [nb]
    for t in succ:
        if t in preds:
            preds[t].append(b)
END_STATE = {}
prev_flags = None
for bi, b in enumerate(blocks):
    nb = blocks[bi + 1] if bi + 1 < len(blocks) else END
    # end of block: up to next leader
    ir = None
    pf = prev_flags
    if len(preds[b]) == 1 and preds[b][0] in END_STATE:
        ir, pf = END_STATE[preds[b][0]]
    lines, regs, dirty, prev_flags = lift_block(b, nb, pf, ir)
    END_STATE[b] = (dict(regs), prev_flags)
    print("L_%08X:" % b)
    for l in lines:
        print(l)
    # live regs at end (non-address constants)
    live = []
    for r in sorted(dirty):
        v = regs.get(r)
        if v is None:
            continue
        if v.c is not None and 0x20000000 <= v.c < 0x20010000:
            continue
        if lines and v.s() in lines[-1] and not lines[-1].split()[1].startswith(("g", "S", "B", "T", "R", "K", "M", "*")):
            continue
        live.append("%s=%s" % (REGN[r], v.s()))
    last = insns[max(x for x in insns if b <= x < nb)]
    if live and not (last.mnemonic.startswith("b") and last.mnemonic not in ("bl",) and last.cc in (ARM_CC_AL, ARM_CC_INVALID) and False):
        print("      {live: %s}" % ", ".join(live))
