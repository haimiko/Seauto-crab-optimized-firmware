import struct
import sys
import os
from collections import defaultdict, Counter
from capstone import Cs, CS_ARCH_ARM, CS_MODE_THUMB, CS_MODE_LITTLE_ENDIAN
from capstone.arm import ARM_OP_IMM, ARM_OP_MEM, ARM_OP_REG

FW = sys.argv[1]
OUTDIR = sys.argv[2]
# optional third argument: output file stem, for images other than the app (e.g. "bootloader")
STEM = sys.argv[3] if len(sys.argv) > 3 else "crab_1040008012"
TITLE = "SEAUTO Crab firmware 1040008012" if STEM == "crab_1040008012" else "SEAUTO Crab " + STEM
os.makedirs(OUTDIR, exist_ok=True)
data = open(FW, "rb").read()
SIZE = len(data)

# ---- resolve load base from the reset vector ----
sp0, reset = struct.unpack_from("<II", data, 0)
# handlers point into flash just above a base aligned to 0x1000; infer base
reset_addr = reset & ~1
BASE = reset_addr & 0xFFFFF000  # 0x0800A000 expected
# refine: smallest handler target rounded down
vt = struct.unpack_from("<64I", data, 0)
targets = [v & ~1 for v in vt if 0x08000000 <= (v & ~1) < 0x08080000]
BASE = min(targets) & 0xFFFFF000
FLASH_END = BASE + SIZE


def in_image(a):
    return BASE <= a < FLASH_END


def off(a):
    return a - BASE


# ---- peripheral map: Artery AT32F415 (STM32F1-compatible bases) ----
# Differences from a plain STM32F10x map: the only ADC is at 0x40012400 (the F1 "ADC2" slot),
# 0x40013C00 is the comparator, port F exists at 0x40011C00, TIM9/10/11 exist.
PERIPH = [
    (0x40000000, "TIM2"), (0x40000400, "TIM3"), (0x40000800, "TIM4"),
    (0x40000C00, "TIM5"), (0x40001000, "TIM6"), (0x40001400, "TIM7"),
    (0x40002800, "RTC"), (0x40002C00, "WWDG"), (0x40003000, "IWDG"),
    (0x40003800, "SPI2"), (0x40003C00, "SPI3"),
    (0x40004400, "USART2"), (0x40004800, "USART3"),
    (0x40004C00, "UART4"), (0x40005000, "UART5"),
    (0x40005400, "I2C1"), (0x40005800, "I2C2"),
    (0x40006400, "CAN1"), (0x40007000, "PWR"), (0x40007400, "DAC"),
    (0x40010000, "AFIO"), (0x40010400, "EXTI"),
    (0x40010800, "GPIOA"), (0x40010C00, "GPIOB"), (0x40011000, "GPIOC"),
    (0x40011400, "GPIOD"), (0x40011800, "GPIOE"), (0x40011C00, "GPIOF"),
    (0x40012400, "ADC1"),
    (0x40012C00, "TIM1"), (0x40013000, "SPI1"), (0x40013800, "USART1"),
    (0x40013C00, "CMP"),
    (0x40014C00, "TIM9"), (0x40015000, "TIM10"), (0x40015400, "TIM11"),
    (0x40020000, "DMA1"), (0x40020400, "DMA2"),
    (0x40021000, "RCC"), (0x40022000, "FLASH_IF"), (0x40023000, "CRC"),
    (0xE000E010, "SysTick"), (0xE000E100, "NVIC"), (0xE000ED00, "SCB"),
    (0x1FFFF000, "SysMemBoot"), (0x1FFFF800, "OptionBytes"),
]


def periph_name(a):
    best = None
    for b, n in PERIPH:
        if b <= a < b + 0x400:
            return n, a - b
    return None, None


EXC = {1: "Reset", 2: "NMI", 3: "HardFault", 4: "MemManage", 5: "BusFault",
       6: "UsageFault", 11: "SVCall", 12: "DebugMon", 14: "PendSV", 15: "SysTick"}

# ---- disassemble whole image (thumb) ----
md = Cs(CS_ARCH_ARM, CS_MODE_THUMB + CS_MODE_LITTLE_ENDIAN)
md.detail = True

insns = {}      # addr -> insn
order = []
VT_END = 0xEC  # skip the vector table itself (59 entries)
pos = VT_END
while pos < SIZE - 1:
    progressed = False
    for ins in md.disasm(data[pos:], BASE + pos):
        insns[ins.address] = ins
        order.append(ins.address)
        pos = ins.address - BASE + ins.size
        progressed = True
    if pos < SIZE - 1:
        pos += 2  # undecodable halfword (literal/data); step over it

# ---- collect references: BL/BLX targets, literal-pool pointers, periph refs ----
call_targets = Counter()
code_xref = defaultdict(set)      # target -> set(callers)
ldr_literals = {}                 # pc-rel literal address -> value loaded
periph_refs = defaultdict(list)   # periph name -> list of (addr, offset)
data_ptrs = Counter()             # pointers into flash (func/data tables)
ram_ptrs = Counter()

def read_word(a):
    o = off(a)
    if 0 <= o <= SIZE - 4:
        return struct.unpack_from("<I", data, o)[0]
    return None

for a in order:
    ins = insns[a]
    m = ins.mnemonic
    if m in ("bl", "blx"):
        for op in ins.operands:
            if op.type == ARM_OP_IMM:
                t = op.imm
                call_targets[t] += 1
                code_xref[t].add(a)
    # PC-relative literal loads: ldr rX, [pc, #imm]
    if m == "ldr" and len(ins.operands) == 2:
        src = ins.operands[1]
        if src.type == ARM_OP_MEM and src.mem.base == md.reg_name(src.mem.base) and False:
            pass
    # capstone resolves pc-rel as [pc, #x]; use the written literal target
    if m == "ldr":
        # find the literal address capstone computed in op_str like "[pc, #0x..]"
        ops = ins.operands
        if len(ops) == 2 and ops[1].type == ARM_OP_MEM:
            base_reg = ops[1].mem.base
            if md.reg_name(base_reg) == "pc":
                lit = (ins.address + 4) & ~3
                lit += ops[1].mem.disp
                val = read_word(lit)
                if val is not None:
                    ldr_literals[lit] = val
                    n, poff = periph_name(val)
                    if n:
                        periph_refs[n].append((ins.address, poff))
                    elif 0x20000000 <= val < 0x20010000:
                        ram_ptrs[val] += 1
                    elif in_image(val & ~1):
                        data_ptrs[val] += 1

# ---- function discovery ----
funcs = set()
funcs.add(reset_addr)
for i, v in enumerate(vt):
    t = v & ~1
    if i >= 1 and in_image(t):
        funcs.add(t)
for t in call_targets:
    if in_image(t & ~1):
        funcs.add(t & ~1)
# prologue scan: push {..., lr}
for a in order:
    ins = insns[a]
    if ins.mnemonic == "push" and "lr" in ins.op_str:
        funcs.add(a)

funcs = sorted(x for x in funcs if in_image(x))

# vector table label map
vec_label = {}
for i, v in enumerate(vt):
    t = v & ~1
    if not in_image(t):
        continue
    if i in EXC:
        vec_label[t] = EXC[i]
    elif i >= 16:
        vec_label[t] = f"IRQ{i-16}"

# ---- write full disassembly ----
listing = os.path.join(OUTDIR, STEM + ".lst")
fnames = {}
for idx, f in enumerate(funcs):
    fnames[f] = vec_label.get(f, f"sub_{f:08X}")
fnames[reset_addr] = "Reset_Handler"

with open(listing, "w", encoding="utf-8") as out:
    out.write(f"; {TITLE}  base=0x{BASE:08X} size=0x{SIZE:X} end=0x{FLASH_END:08X}\n")
    out.write(f"; initial SP=0x{sp0:08X}  reset=0x{reset_addr:08X}\n")
    out.write(f"; {len(funcs)} functions discovered\n\n")
    func_set = set(funcs)
    for a in order:
        if a in func_set:
            nm = fnames.get(a, f"sub_{a:08X}")
            xr = code_xref.get(a) or code_xref.get(a | 1)
            xrs = f"  ; xrefs={len(xr)}" if xr else ""
            out.write(f"\n{nm}:{xrs}\n")
        if a in vec_label and a not in func_set:
            out.write(f"\n{vec_label[a]}:\n")
        ins = insns[a]
        tgt = ""
        if ins.mnemonic in ("bl", "blx", "b", "bx", "b.w") or ins.mnemonic.startswith("b"):
            for op in ins.operands:
                if op.type == ARM_OP_IMM and in_image(op.imm & ~1):
                    nn = fnames.get(op.imm & ~1)
                    if nn:
                        tgt = f"  -> {nn}"
        lit = ""
        if ins.mnemonic == "ldr" and len(ins.operands) == 2 and ins.operands[1].type == ARM_OP_MEM and md.reg_name(ins.operands[1].mem.base) == "pc":
            la = ((ins.address + 4) & ~3) + ins.operands[1].mem.disp
            v = ldr_literals.get(la)
            if v is not None:
                pn, po = periph_name(v)
                if pn:
                    lit = f"  ; ={pn}+0x{po:X} (0x{v:08X})"
                else:
                    lit = f"  ; =0x{v:08X}"
        out.write(f"  {ins.address:08X}: {ins.mnemonic:<8}{ins.op_str}{tgt}{lit}\n")

# ---- summary report ----
report = os.path.join(OUTDIR, STEM + "_report.txt")
with open(report, "w", encoding="utf-8") as r:
    r.write(TITLE + " - static analysis summary\n")
    r.write("=" * 60 + "\n\n")
    r.write(f"File size           : {SIZE} bytes (0x{SIZE:X})\n")
    r.write(f"Load base           : 0x{BASE:08X}\n")
    r.write(f"Flash span          : 0x{BASE:08X} - 0x{FLASH_END:08X}\n")
    r.write(f"Initial SP          : 0x{sp0:08X}  (SRAM top)\n")
    r.write(f"Reset handler       : 0x{reset_addr:08X}\n")
    r.write(f"Instructions decoded: {len(order)}\n")
    r.write(f"Functions discovered: {len(funcs)}\n\n")

    # interrupt vector table
    r.write("Interrupt vector table (populated entries)\n")
    r.write("-" * 60 + "\n")
    for i, v in enumerate(vt):
        t = v & ~1
        if v == 0 or not in_image(t):
            continue
        label = EXC.get(i, f"IRQ{i-16}" if i >= 16 else f"res{i}")
        r.write(f"  [{i:3}] 0x{v:08X}  {label}\n")
    r.write("\n")

    # peripherals
    r.write("Peripherals referenced (by literal-pool base address)\n")
    r.write("-" * 60 + "\n")
    for n in sorted(periph_refs, key=lambda k: -len(periph_refs[k])):
        offs = sorted(set(o for _, o in periph_refs[n]))
        r.write(f"  {n:10} refs={len(periph_refs[n]):4}  reg offsets: {', '.join('0x%X'%o for o in offs[:16])}{' ...' if len(offs)>16 else ''}\n")
    r.write("\n")

    # most-called functions (likely core helpers)
    r.write("Most-referenced call targets (hot functions)\n")
    r.write("-" * 60 + "\n")
    for t, c in call_targets.most_common(30):
        r.write(f"  {c:4} calls -> {fnames.get(t & ~1, 'sub_%08X' % (t & ~1))}  (0x{t & ~1:08X})\n")
    r.write("\n")

    # mnemonic histogram
    r.write("Instruction mnemonic histogram (top 30)\n")
    r.write("-" * 60 + "\n")
    mh = Counter(insns[a].mnemonic for a in order)
    for mn, c in mh.most_common(30):
        r.write(f"  {mn:10} {c}\n")

print("BASE=0x%08X SIZE=0x%X funcs=%d insns=%d" % (BASE, SIZE, len(funcs), len(order)))
print("periph:", ", ".join("%s(%d)" % (n, len(periph_refs[n])) for n in sorted(periph_refs, key=lambda k:-len(periph_refs[k]))))
print("listing:", listing)
print("report :", report)
