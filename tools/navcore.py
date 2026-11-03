"""Offline execution of the exact compiled sensor-only nav_tick and map_tick Thumb functions.

No vendor peripherals here: navsim supplies imperfect observations and models motion.
It needs a compiled add-on, whose source is not published yet.
diag-3.3 cores (symbol nav_layout 3) also take the raw sonar and the map; older cores
(diag-2.8..3.2, 13-word state) are still executable for comparisons.
"""
import struct
from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB, UC_MODE_MCLASS
from unicorn.arm_const import (UC_ARM_REG_R0, UC_ARM_REG_R1, UC_ARM_REG_R2,
                               UC_ARM_REG_R3, UC_ARM_REG_SP, UC_ARM_REG_LR, UC_ARM_REG_PC)

STATE = struct.Struct("<Iiii9I4I8B5I")  # nav_state_t, layout 3 (diag-3.3)
STATE_V2 = struct.Struct("<Iiii9I")     # layout 2, 13 words (diag-2.8..3.2)
MAP_BYTES = 0x1800                      # the map (layout 3); parsed by tools/navmap.py
STATE_AT, CONFIG_AT, OUTPUT_AT, MAP_AT, STACK_AT, RETURN = (
    0x20001000, 0x20001100, 0x20001200, 0x20002000, 0x20007000, 0x0801EB00)


class Core:
    def __init__(self, code, symbols, config=None):
        self.uc = Uc(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS)
        self.uc.mem_map(0x08000000, 0x20000)
        self.uc.mem_write(0x080186C0, code)
        self.uc.mem_map(0x20000000, 0x8000)
        self.symbols = symbols
        self.entry = symbols["nav_tick"]
        self.layout = struct.unpack_from("<I", code, symbols["nav_layout"] - 0x080186C0)[0] \
            if "nav_layout" in symbols else 2
        self.layout_struct = STATE if self.layout >= 3 else STATE_V2
        self.config = tuple(config) if config is not None else struct.unpack_from(
            "<8I", code, symbols["nav_defaults"] - 0x080186C0)
        self.reset()

    def reset(self):
        self.uc.mem_write(STATE_AT, bytes(STATE.size))
        self.uc.mem_write(CONFIG_AT, struct.pack("<8I", *self.config))

    @property
    def state(self):
        """Layout 3: 13 words as before, then echo, cells, fresh, scan, 8 scores, stuck,
        stale, capped, scans, steered."""
        return self.layout_struct.unpack(self.uc.mem_read(STATE_AT, self.layout_struct.size))

    def _call(self, entry, registers, stacked, name):
        uc = self.uc
        uc.reg_write(UC_ARM_REG_SP, STACK_AT)
        uc.reg_write(UC_ARM_REG_LR, RETURN | 1)
        for reg, value in zip((UC_ARM_REG_R0, UC_ARM_REG_R1, UC_ARM_REG_R2, UC_ARM_REG_R3), registers):
            uc.reg_write(reg, value & 0xFFFFFFFF)
        if stacked:
            uc.mem_write(STACK_AT, struct.pack(f"<{len(stacked)}I", *(v & 0xFFFFFFFF for v in stacked)))
        uc.emu_start(entry | 1, RETURN, count=20000)
        # An instruction limit is a failure, not a stale output.
        if uc.reg_read(UC_ARM_REG_PC) != RETURN:
            raise RuntimeError(f"{name} exceeded instruction budget")

    def tick(self, now, yaw, wall, sonar=0, mapped=False):
        """sonar: forward range in cm as the firmware variable holds it (0 or >= 1000: none);
        mapped: hand the core the map at MAP_AT (map_reset first). Older cores ignore both."""
        if self.layout >= 3:
            stacked = (bool(wall), round(sonar), MAP_AT if mapped else 0, OUTPUT_AT)
        else:
            stacked = (bool(wall), OUTPUT_AT)
        self._call(self.entry, (STATE_AT, CONFIG_AT, now, yaw), stacked, "nav_tick")
        return struct.unpack("<2i", self.uc.mem_read(OUTPUT_AT, 8))

    def map_reset(self, speed70=None):
        self._call(self.symbols["map_reset"], (MAP_AT, self.config[2] if speed70 is None else speed70),
                   (), "map_reset")

    def map_tick(self, now, yaw, drive, tilt, sonar, wet=True):
        # tilt: the larger of |roll| and |pitch| in 0.1 degree, as diag_map_c passes it.
        # The firmware's sonar variable is whole centimetres; navsim's model is fractional.
        self._call(self.symbols["map_tick"], (MAP_AT, now, yaw, drive), (tilt, round(sonar), bool(wet)),
                   "map_tick")

    def map_bytes(self):
        return bytes(self.uc.mem_read(MAP_AT, MAP_BYTES))
