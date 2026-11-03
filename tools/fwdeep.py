import struct, sys
from collections import defaultdict, Counter
from capstone import Cs, CS_ARCH_ARM, CS_MODE_THUMB, CS_MODE_LITTLE_ENDIAN
from capstone.arm import ARM_OP_IMM, ARM_OP_MEM

data = open(sys.argv[1], "rb").read(); SIZE = len(data)
vt = struct.unpack_from("<96I", data, 0)
BASE = min((v & ~1) for v in vt if 0x08000000 <= (v & ~1) < 0x08080000) & 0xFFFFF000
END = BASE + SIZE
inimg = lambda a: BASE <= a < END
rdw = lambda a: struct.unpack_from("<I", data, a - BASE)[0] if BASE <= a <= END - 4 else None

md = Cs(CS_ARCH_ARM, CS_MODE_THUMB + CS_MODE_LITTLE_ENDIAN); md.detail = True
insns = {}; pos = 0xEC
while pos < SIZE - 1:
    for ins in md.disasm(data[pos:], BASE + pos):
        insns[ins.address] = ins; pos = ins.address - BASE + ins.size
    if pos < SIZE - 1: pos += 2
order = sorted(insns)

callcnt = Counter(); funcs = set()
for a in order:
    ins = insns[a]
    if ins.mnemonic in ("bl", "blx"):
        for op in ins.operands:
            if op.type == ARM_OP_IMM: callcnt[op.imm & ~1] += 1
    if ins.mnemonic == "push" and "lr" in ins.op_str: funcs.add(a)
for i, v in enumerate(vt):
    if i >= 1 and inimg(v & ~1): funcs.add(v & ~1)
funcs |= {t for t in callcnt if inimg(t)}
funcs = sorted(funcs)
def fend(f):
    i = funcs.index(f); return funcs[i + 1] if i + 1 < len(funcs) else END
def owner(a):
    lo = 0
    for f in funcs:
        if f <= a: lo = f
        else: break
    return lo

def lit_of(ins):
    if ins.mnemonic == "ldr" and len(ins.operands) == 2 and ins.operands[1].type == ARM_OP_MEM \
       and md.reg_name(ins.operands[1].mem.base) == "pc":
        return rdw(((ins.address + 4) & ~3) + ins.operands[1].mem.disp)
    return None

# ---- 1. immediates used in the USART3 ISR (Modbus register map / function codes) ----
U3 = 0x0800DCA8; U3E = fend(U3)
print(f"== USART3 ISR 0x{U3:08X}-0x{U3E:08X}: compare/move immediates ==")
imm = Counter()
for a in order:
    if not (U3 <= a < U3E): continue
    ins = insns[a]
    if ins.mnemonic.startswith(("cmp", "movs", "movw", "mov.w", "sub", "add")):
        for op in ins.operands:
            if op.type == ARM_OP_IMM and 0 < op.imm < 0x10000:
                imm[(ins.mnemonic.split('.')[0], op.imm)] += 1
for (m, v), c in sorted(imm.items(), key=lambda kv: (kv[0][1], kv[0][0])):
    print(f"   {m:5} #0x{v:04X} ({v:5})  x{c}")

# ---- 2. callee profile of main loop ----
MAIN = 0x0801019C; ME = fend(MAIN)
print(f"\n== main 0x{MAIN:08X}-0x{ME:08X} callees ==")
cc = Counter()
for a in order:
    if MAIN <= a < ME and insns[a].mnemonic in ("bl", "blx"):
        for op in insns[a].operands:
            if op.type == ARM_OP_IMM: cc[op.imm & ~1] += 1
for t, c in cc.most_common():
    print(f"   {c:3}x sub_{t:08X}")

# ---- 3. xrefs to key functions (bl/b targets + literal-pool function pointers) ----
KEYS = [0x080149AC, 0x0800AD08, 0x0801019C, 0x08017F50, 0x08015E0C, 0x0800D684, 0x0800C988, 0x0800B754, 0x0800DCA8]
print("\n== xrefs ==")
for k in KEYS:
    refs = []
    for a in order:
        ins = insns[a]
        for op in ins.operands:
            if op.type == ARM_OP_IMM and (op.imm & ~1) == k and ins.mnemonic.startswith("b"):
                refs.append(f"{ins.mnemonic}@0x{a:08X}(in sub_{owner(a):08X})")
        lv = lit_of(ins)
        if lv is not None and (lv & ~1) == k:
            refs.append(f"fptr@0x{a:08X}(in sub_{owner(a):08X})")
    print(f"   0x{k:08X}: {', '.join(refs) if refs else '(none)'}")

# ---- 4. tail-branches into other functions (b to a func start) ----
print("\n== unconditional tail branches to function starts ==")
fs = set(funcs)
for a in order:
    ins = insns[a]
    if ins.mnemonic in ("b", "b.w"):
        for op in ins.operands:
            if op.type == ARM_OP_IMM and (op.imm & ~1) in fs and owner(a) != (op.imm & ~1):
                print(f"   0x{a:08X} (sub_{owner(a):08X}) -> sub_{op.imm & ~1:08X}")

# ---- 5. global SRAM variables: most-referenced RAM literals ----
print("\n== top SRAM globals (literal-pool refs) ==")
ram = Counter(); ramf = defaultdict(set)
for a in order:
    v = lit_of(insns[a])
    if v is not None and 0x20000000 <= v < 0x20010000:
        ram[v] += 1; ramf[v].add(owner(a))
for v, c in ram.most_common(40):
    print(f"   0x{v:08X} x{c:3}  used by {len(ramf[v])} funcs")
print(f"\n   distinct RAM literal bases: {len(ram)}; SP top 0x{vt[0]:08X}; highest ref 0x{max(ram):08X}")
