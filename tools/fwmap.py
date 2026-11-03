import struct, sys, os
from collections import defaultdict, Counter
from capstone import Cs, CS_ARCH_ARM, CS_MODE_THUMB, CS_MODE_LITTLE_ENDIAN
from capstone.arm import ARM_OP_IMM, ARM_OP_MEM

FW = sys.argv[1]
data = open(FW, "rb").read()
SIZE = len(data)
sp0, reset = struct.unpack_from("<II", data, 0)
vt = struct.unpack_from("<96I", data, 0)
BASE = min((v & ~1) for v in vt if 0x08000000 <= (v & ~1) < 0x08080000) & 0xFFFFF000
END = BASE + SIZE
off = lambda a: a - BASE
inimg = lambda a: BASE <= a < END

PERIPH = [
    (0x40000000,"TIM2"),(0x40000400,"TIM3"),(0x40000800,"TIM4"),(0x40000C00,"TIM5"),
    (0x40001000,"TIM6"),(0x40001400,"TIM7"),(0x40002800,"RTC"),(0x40002C00,"WWDG"),
    (0x40003000,"IWDG"),(0x40003800,"SPI2"),(0x40003C00,"SPI3"),(0x40004400,"USART2"),
    (0x40004800,"USART3"),(0x40004C00,"UART4"),(0x40005000,"UART5"),(0x40005400,"I2C1"),
    (0x40005800,"I2C2"),(0x40006400,"CAN1"),(0x40007000,"PWR"),(0x40007400,"DAC"),
    (0x40010000,"AFIO"),(0x40010400,"EXTI"),(0x40010800,"GPIOA"),(0x40010C00,"GPIOB"),
    (0x40011000,"GPIOC"),(0x40011400,"GPIOD"),(0x40011800,"GPIOE"),(0x40011C00,"GPIOF"),
    (0x40012400,"ADC1"),(0x40012C00,"TIM1"),(0x40013000,"SPI1"),(0x40013800,"USART1"),
    (0x40013C00,"CMP"),(0x40014C00,"TIM9"),(0x40015000,"TIM10"),(0x40015400,"TIM11"),
    (0x40020000,"DMA1"),(0x40020400,"DMA2"),(0x40021000,"RCC"),(0x40022000,"FLASH_IF"),
    (0x40023000,"CRC"),(0xE000E010,"SysTick"),(0xE000E100,"NVIC"),(0xE000ED00,"SCB"),
]
def periph(a):
    for b,n in PERIPH:
        if b <= a < b+0x400: return n
    return None

md = Cs(CS_ARCH_ARM, CS_MODE_THUMB+CS_MODE_LITTLE_ENDIAN); md.detail=True
insns={}; order=[]
pos=0xEC
while pos < SIZE-1:
    got=False
    for ins in md.disasm(data[pos:], BASE+pos):
        insns[ins.address]=ins; order.append(ins.address)
        pos=ins.address-BASE+ins.size; got=True
    if pos < SIZE-1: pos+=2
order.sort()

def rdw(a):
    o=off(a)
    return struct.unpack_from("<I",data,o)[0] if 0<=o<=SIZE-4 else None

# function set
calls=defaultdict(set); callcnt=Counter()
for a in order:
    ins=insns[a]
    if ins.mnemonic in ("bl","blx"):
        for op in ins.operands:
            if op.type==ARM_OP_IMM:
                calls[a].add(op.imm&~1); callcnt[op.imm&~1]+=1
funcs=set([reset&~1])
for i,v in enumerate(vt):
    if i>=1 and inimg(v&~1): funcs.add(v&~1)
for t in callcnt:
    if inimg(t): funcs.add(t)
for a in order:
    if insns[a].mnemonic=="push" and "lr" in insns[a].op_str: funcs.add(a)
funcs=sorted(f for f in funcs if inimg(f))

# IRQ labels (STM32F103 high-density)
IRQN={6:"EXTI0",7:"EXTI1",8:"EXTI2",9:"EXTI3",10:"EXTI4",11:"DMA1_CH1",12:"DMA1_CH2",
13:"DMA1_CH3",14:"DMA1_CH4",15:"DMA1_CH5",16:"DMA1_CH6",17:"DMA1_CH7",18:"ADC1_2",
23:"EXTI9_5",24:"TIM1_BRK",25:"TIM1_UP",26:"TIM1_TRG",27:"TIM1_CC",28:"TIM2",29:"TIM3",
30:"TIM4",35:"SPI1",36:"SPI2",37:"USART1",38:"USART2",39:"USART3",40:"EXTI15_10",
50:"TIM5",53:"UART4",54:"UART5",55:"TIM6",56:"TIM7",57:"DMA2_CH1",58:"DMA2_CH2",
59:"DMA2_CH3",60:"DMA2_CH45"}
EXC={1:"Reset",2:"NMI",3:"HardFault",11:"SVCall",14:"PendSV",15:"SysTick"}
label={}
for i,v in enumerate(vt):
    t=v&~1
    if not inimg(t): continue
    if i in EXC: label.setdefault(t,EXC[i])
    elif i>=16 and (i-16) in IRQN: label.setdefault(t, "ISR_"+IRQN[i-16])

# per-function feature extraction
fidx={f:i for i,f in enumerate(funcs)}
def func_end(f):
    i=fidx[f]
    return funcs[i+1] if i+1<len(funcs) else END
feat={}
for f in funcs:
    e=func_end(f)
    pset=Counter(); callees=[]
    a=f
    body=[x for x in order if f<=x<e]
    for a in body:
        ins=insns[a]
        if ins.mnemonic=="ldr" and len(ins.operands)==2 and ins.operands[1].type==ARM_OP_MEM and md.reg_name(ins.operands[1].mem.base)=="pc":
            la=((a+4)&~3)+ins.operands[1].mem.disp; v=rdw(la)
            if v is not None:
                pn=periph(v)
                if pn: pset[pn]+=1
        if ins.mnemonic in ("bl","blx"):
            for op in ins.operands:
                if op.type==ARM_OP_IMM: callees.append(op.imm&~1)
    feat[f]=dict(size=e-f, periph=pset, callees=callees, ninsn=len(body))

# heuristic naming
def guess(f):
    if f in label: return label[f]
    p=feat[f]["periph"]
    keys=set(p)
    if {"RCC","FLASH_IF"}<=keys or ("RCC" in keys and p.get("RCC",0)>=6): return "clock/flash_init?"
    if "FLASH_IF" in keys: return "flash_program?"
    if {"UART5"}&keys and p.get("UART5",0)>=2: return "uart5_io?"
    if {"USART1"}&keys and p.get("USART1",0)>=2: return "usart1_io?"
    if {"USART3"}&keys and p.get("USART3",0)>=2: return "usart3_io?"
    if "ADC2" in keys or "ADC1" in keys: return "adc_read?"
    if "TIM1" in keys: return "tim1/pwm?"
    if keys & {"TIM3","TIM4","TIM5"}: return "timer_cfg?"
    if keys & {"DMA1","DMA2"}: return "dma_cfg?"
    if "IWDG" in keys: return "watchdog?"
    if "RTC" in keys: return "rtc?"
    if keys=={"GPIOA"} or keys<= {"GPIOA","GPIOB","GPIOC","GPIOD","GPIOE","AFIO"} and keys: return "gpio?"
    return ""

print(f"base=0x{BASE:08X} end=0x{END:08X} funcs={len(funcs)}")
print(f"bootloader region: 0x08000000-0x{BASE:08X} ({(BASE-0x08000000)//1024} KiB), app: {(END-BASE)//1024} KiB\n")

print("ISR handlers:")
for f in funcs:
    if f in label and label[f].startswith("ISR"):
        p=feat[f]["periph"]
        print(f"  0x{f:08X} {label[f]:14} size={feat[f]['size']:5}  periph={dict(p)}")
print()

# biggest functions
print("Largest functions (likely main logic / protocol state machines):")
for f in sorted(funcs,key=lambda x:-feat[x]['size'])[:20]:
    g=guess(f)
    p=dict(feat[f]['periph'])
    print(f"  0x{f:08X} size={feat[f]['size']:5} insn={feat[f]['ninsn']:4} calls={callcnt[f]:3} {g:16} {p if p else ''}")
print()

# subsystem rollup: which funcs touch which peripheral
print("Functions touching each peripheral:")
bysub=defaultdict(list)
for f in funcs:
    for pn in feat[f]['periph']:
        bysub[pn].append(f)
for pn in sorted(bysub,key=lambda k:-len(bysub[k])):
    print(f"  {pn:10} {len(bysub[pn])} funcs")
