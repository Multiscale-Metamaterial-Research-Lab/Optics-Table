# =============================================================================
# ADF5355 Dual Board Controller (Revised)
# =============================================================================

import math
import time
import threading
import queue
from dataclasses import dataclass
from typing import List, Literal, Optional

import tkinter as tk
from tkinter import ttk, messagebox, filedialog

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    serial = None

MOD1 = 1 << 24  # 16777216


@dataclass
class ADF5355Config:
    ref_in_hz: float = 100e6
    r_counter: int = 1
    ref_doubler: bool = False
    ref_div2: bool = False
    ref_mode_differential: bool = False
    cp_current_code: int = 0b0010
    muxout_code: int = 0b110
    pd_polarity_positive: bool = True
    rfouta_power_code: int = 0b10
    enable_mtld: bool = False
    enable_negative_bleed: bool = False
    enable_gated_bleed: bool = False
    bleed_current_code: int = 0
    phase_adjust: bool = False
    phase_resync: bool = False
    phase_value: int = 1
    le_sync: bool = True
    ld_mode_integer: bool = False
    ld_precision_code: int = 0b11
    ld_cycle_count_code: int = 0b00
    lol_mode: bool = False
    adc_enable: bool = True
    adc_conversion_enable: bool = True


class PicoADF5355:
    def __init__(self, port: str, baudrate: int = 115200, timeout: float = 1.0):
        if serial is None:
            raise RuntimeError("pyserial is not installed. Run: pip install pyserial")
        self.ser = serial.Serial(port, baudrate=baudrate, timeout=timeout)
        time.sleep(2.0)
        try:
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
        except Exception:
            pass

    def close(self):
        if self.ser and self.ser.is_open:
            self.ser.close()

    def _send(self, cmd: str):
        if not cmd.endswith("\n"):
            cmd += "\n"
        self.ser.write(cmd.encode("ascii"))
        self.ser.flush()

    def _readline(self) -> str:
        return self.ser.readline().decode(errors="ignore").strip()

    def read_all_lines(self, max_lines: int = 50, idle_timeout: float = 0.15):
        lines = []
        t0 = time.time()
        while len(lines) < max_lines:
            line = self._readline()
            if line:
                lines.append(line)
                t0 = time.time()
            elif (time.time() - t0) > idle_timeout:
                break
        return lines

    def startup(self):
        self._send("$STAR")

    def select_board(self, board: Literal["A", "B"]):
        if board.upper() == "A":
            self._send("$BRDA")
        elif board.upper() == "B":
            self._send("$BRDB")
        else:
            raise ValueError("board must be 'A' or 'B'")
        return self._readline()

    def write_register(self, reg_index: int, value: int):
        if not (0 <= reg_index <= 12):
            raise ValueError("reg_index must be 0..12")
        value &= 0xFFFFFFFF
        self._send(f"$REGS{reg_index:02d}{value}")
        return self._readline()

    def write_registers(self, regs: List[int]):
        if len(regs) != 13:
            raise ValueError("Need exactly 13 registers")
        replies = []
        for i, value in enumerate(regs):
            replies.append(self.write_register(i, value))
        return replies

    def init_device(self):
        self._send("$INIT")
        return self._readline()

    def update_frequency(self):
        self._send("$FREQ")
        return self._readline()

    def read_lock_status(self):
        self._send("$LOCK")
        return self._readline()

    def dump_active(self):
        self._send("$DUMP")
        return self.read_all_lines()


class ADF5355RegisterBuilder:
    def __init__(self, cfg: Optional[ADF5355Config] = None):
        self.cfg = cfg if cfg is not None else ADF5355Config()

    @staticmethod
    def _set_bits(word: int, value: int, msb: int, lsb: int) -> int:
        width = msb - lsb + 1
        mask = ((1 << width) - 1) << lsb
        word &= ~mask
        word |= (value << lsb) & mask
        return word

    @staticmethod
    def _set_bit(word: int, bit: int, state: bool) -> int:
        if state:
            return word | (1 << bit)
        return word & ~(1 << bit)

    def f_pfd(self) -> float:
        d = 1 if self.cfg.ref_doubler else 0
        t = 1 if self.cfg.ref_div2 else 0
        return self.cfg.ref_in_hz * (1 + d) / (self.cfg.r_counter * (1 + t))

    @staticmethod
    def _gcd_float(a: float, b: float) -> float:
        ai = int(round(a))
        bi = int(round(b))
        return math.gcd(ai, bi)

    @staticmethod
    def _rf_divider_code(divider: int) -> int:
        table = {1: 0b000, 2: 0b001, 4: 0b010, 8: 0b011, 16: 0b100, 32: 0b101, 64: 0b110}
        if divider not in table:
            raise ValueError("RF divider must be one of 1,2,4,8,16,32,64")
        return table[divider]

    @staticmethod
    def _choose_rf_divider_for_rfouta(freq_hz: float) -> int:
        for div in [1, 2, 4, 8, 16, 32, 64]:
            vco = freq_hz * div
            if 3.4e9 <= vco <= 6.8e9:
                return div
        raise ValueError("Requested RFOUTA frequency is outside achievable range")

    def _calc_int_frac(self, target_n: float, channel_spacing_hz: float = 1.0):
        fpfd = self.f_pfd()
        int_val = int(math.floor(target_n))
        frac = target_n - int_val
        frac1_full = frac * MOD1
        frac1 = int(math.floor(frac1_full))
        remainder = frac1_full - frac1

        if abs(remainder) < 1e-12:
            mod2 = 2
            frac2 = 0
        else:
            gcd_val = self._gcd_float(fpfd, channel_spacing_hz)
            mod2 = int(round(fpfd / gcd_val)) if gcd_val > 0 else 2
            mod2 = max(2, min(mod2, 16383))
            frac2 = int(round(remainder * mod2))
            if frac2 >= mod2:
                frac2 = mod2 - 1

        return int_val, frac1, frac2, mod2

    def build_registers(
        self,
        output_freq_hz: float,
        output: Literal["A", "B"] = "B",
        output_power_code: Optional[int] = None,
        channel_spacing_hz: float = 1.0,
    ) -> List[int]:
        fpfd = self.f_pfd()

        if output.upper() == "B":
            if not (6.8e9 <= output_freq_hz <= 13.6e9):
                raise ValueError("RFOUTB must be between 6.8 GHz and 13.6 GHz")
            vco_hz = output_freq_hz / 2.0
            rf_divider = 1
            rfouta_enable = False
            rfoutb_disable_bit = 0
            feedback_fundamental = True
            target_n = vco_hz / fpfd

        elif output.upper() == "A":
            if not (53.125e6 <= output_freq_hz <= 6.8e9):
                raise ValueError("RFOUTA must be between 53.125 MHz and 6.8 GHz")
            rf_divider = self._choose_rf_divider_for_rfouta(output_freq_hz)
            vco_hz = output_freq_hz * rf_divider
            rfouta_enable = True
            rfoutb_disable_bit = 1
            feedback_fundamental = False
            target_n = output_freq_hz / fpfd
        else:
            raise ValueError("output must be 'A' or 'B'")

        if not (3.4e9 <= vco_hz <= 6.8e9):
            raise ValueError("Computed VCO is outside 3.4 to 6.8 GHz")

        int_val, frac1, frac2, mod2 = self._calc_int_frac(target_n, channel_spacing_hz)

        prescaler_89 = int_val >= 75
        if (not prescaler_89) and int_val < 23:
            raise ValueError(
                f"INT too low ({int_val}) for 4/5 prescaler. Increase PFD or choose different settings."
            )

        regs = [0] * 13

        r0 = 0
        r0 = self._set_bits(r0, int_val, 19, 4)
        r0 = self._set_bit(r0, 21, True)
        r0 = self._set_bit(r0, 20, prescaler_89)
        r0 |= 0x0
        regs[0] = r0

        r1 = 0
        r1 = self._set_bits(r1, frac1, 27, 4)
        r1 |= 0x1
        regs[1] = r1

        r2 = 0
        r2 = self._set_bits(r2, frac2, 31, 18)
        r2 = self._set_bits(r2, mod2, 17, 4)
        r2 |= 0x2
        regs[2] = r2

        r3 = 0
        r3 = self._set_bit(r3, 30, False)
        r3 = self._set_bit(r3, 29, self.cfg.phase_resync)
        r3 = self._set_bit(r3, 28, self.cfg.phase_adjust)
        r3 = self._set_bits(r3, self.cfg.phase_value & 0xFFFFFF, 27, 4)
        r3 |= 0x3
        regs[3] = r3

        r4 = 0
        r4 = self._set_bits(r4, self.cfg.muxout_code & 0b111, 29, 27)
        r4 = self._set_bit(r4, 26, self.cfg.ref_doubler)
        r4 = self._set_bit(r4, 25, self.cfg.ref_div2)
        r4 = self._set_bits(r4, self.cfg.r_counter, 24, 15)
        r4 = self._set_bit(r4, 14, True)
        r4 = self._set_bits(r4, self.cfg.cp_current_code & 0xF, 13, 10)
        r4 = self._set_bit(r4, 9, self.cfg.ref_mode_differential)
        r4 = self._set_bit(r4, 8, True)
        r4 = self._set_bit(r4, 7, self.cfg.pd_polarity_positive)
        r4 = self._set_bit(r4, 6, False)
        r4 = self._set_bit(r4, 5, False)
        r4 = self._set_bit(r4, 4, False)
        r4 |= 0x4
        regs[4] = r4

        regs[5] = 0x00800025

        if output_power_code is None:
            output_power_code = self.cfg.rfouta_power_code

        r6 = 0
        r6 = self._set_bit(r6, 30, self.cfg.enable_gated_bleed)
        r6 = self._set_bit(r6, 29, self.cfg.enable_negative_bleed)
        r6 = self._set_bit(r6, 28, True)
        r6 = self._set_bits(r6, 0b010, 27, 25)
        r6 = self._set_bit(r6, 24, feedback_fundamental)
        r6 = self._set_bits(r6, self._rf_divider_code(rf_divider), 23, 21)
        r6 = self._set_bits(r6, self.cfg.bleed_current_code & 0xFF, 20, 13)
        r6 = self._set_bit(r6, 11, self.cfg.enable_mtld)
        r6 = self._set_bit(r6, 10, rfoutb_disable_bit)
        r6 = self._set_bits(r6, 0, 9, 7)
        r6 = self._set_bit(r6, 6, rfouta_enable)
        r6 = self._set_bits(r6, output_power_code & 0b11, 5, 4)
        r6 |= 0x6
        regs[6] = r6

        r7 = 0
        r7 = self._set_bit(r7, 25, self.cfg.le_sync)
        r7 = self._set_bits(r7, self.cfg.ld_cycle_count_code & 0b11, 9, 8)
        r7 = self._set_bit(r7, 7, self.cfg.lol_mode)
        r7 = self._set_bits(r7, self.cfg.ld_precision_code & 0b11, 6, 5)
        r7 = self._set_bit(r7, 4, self.cfg.ld_mode_integer)
        r7 |= 0x7
        regs[7] = r7

        regs[8] = 0x102D0428

        vco_band_div = math.ceil(fpfd / 2.4e6)
        timeout = math.ceil((fpfd * 50e-6) / 30.0)
        alc_wait = 30
        synth_lock_timeout = 12

        r9 = 0
        r9 = self._set_bits(r9, max(1, min(vco_band_div, 255)), 31, 24)
        r9 = self._set_bits(r9, max(1, min(timeout, 1023)), 23, 14)
        r9 = self._set_bits(r9, alc_wait, 13, 9)
        r9 = self._set_bits(r9, synth_lock_timeout, 8, 4)
        r9 |= 0x9
        regs[9] = r9

        adc_clk_div = math.ceil(((fpfd / 100000.0) - 2.0) / 4.0)
        adc_clk_div = max(1, min(adc_clk_div, 255))

        r10 = 0
        r10 |= (0b11 << 22)
        r10 = self._set_bits(r10, adc_clk_div, 13, 6)
        r10 = self._set_bit(r10, 5, self.cfg.adc_conversion_enable)
        r10 = self._set_bit(r10, 4, self.cfg.adc_enable)
        r10 |= 0xA
        regs[10] = r10

        regs[11] = 0x0061300B

        r12 = 0
        r12 = self._set_bits(r12, 1, 31, 16)
        r12 |= 0x00000810
        r12 |= 0xC
        regs[12] = r12

        return regs

    def calculate_summary(
        self,
        output_freq_hz: float,
        output: Literal["A", "B"] = "B",
        channel_spacing_hz: float = 1.0,
    ) -> dict:
        fpfd = self.f_pfd()

        if output.upper() == "B":
            if not (6.8e9 <= output_freq_hz <= 13.6e9):
                raise ValueError("RFOUTB must be between 6.8 GHz and 13.6 GHz")
            vco_hz = output_freq_hz / 2.0
            rf_divider = 1
            feedback_mode = "fundamental"
            target_n = vco_hz / fpfd
        else:
            if not (53.125e6 <= output_freq_hz <= 6.8e9):
                raise ValueError("RFOUTA must be between 53.125 MHz and 6.8 GHz")
            rf_divider = self._choose_rf_divider_for_rfouta(output_freq_hz)
            vco_hz = output_freq_hz * rf_divider
            feedback_mode = "divided"
            target_n = output_freq_hz / fpfd

        int_val, frac1, frac2, mod2 = self._calc_int_frac(target_n, channel_spacing_hz)

        return {
            "output_path": f"RFOUT{output.upper()}",
            "requested_hz": output_freq_hz,
            "vco_hz": vco_hz,
            "fpfd_hz": fpfd,
            "rf_divider": rf_divider,
            "feedback_mode": feedback_mode,
            "int": int_val,
            "frac1": frac1,
            "frac2": frac2,
            "mod2": mod2,
        }


class ADF5355DualGUI(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("ADF5355 Dual Board Controller")
        self.geometry("1360x900")
        self.minsize(1200, 780)

        self.pico: Optional[PicoADF5355] = None
        self.current_regs: List[int] = [0] * 13
        self.serial_queue: queue.Queue = queue.Queue()
        self.recalc_after_id = None
        self.last_valid_settings = None
        self.last_recalc_error = None

        self.style = ttk.Style(self)
        try:
            self.style.theme_use("clam")
        except Exception:
            pass

        self.style.configure("LockOn.TLabel", foreground="#0a7d22")
        self.style.configure("LockOff.TLabel", foreground="#b22222")
        self.style.configure("LockUnknown.TLabel", foreground="#6b7280")
        self.style.configure("Warning.TLabel", foreground="#b45309")

        self._build_vars()
        self._build_ui()
        self.protocol("WM_DELETE_WINDOW", self.on_close)

        self.refresh_ports()
        self.recalculate()
        self.after(150, self.process_serial_queue)

    def _build_vars(self):
        self.port_var = tk.StringVar()
        self.status_var = tk.StringVar(value="Disconnected")
        self.lock_a_var = tk.StringVar(value="A: ?")
        self.lock_b_var = tk.StringVar(value="B: ?")

        self.board_var = tk.StringVar(value="A")
        self.output_var = tk.StringVar(value="B")

        self.freq_var = tk.StringVar(value="10.525")
        self.units_var = tk.StringVar(value="GHz")
        self.channel_spacing_var = tk.StringVar(value="1")
        self.channel_spacing_units_var = tk.StringVar(value="Hz")

        self.force_init_var = tk.BooleanVar(value=True)

        self.ref_freq_var = tk.StringVar(value="100")
        self.ref_units_var = tk.StringVar(value="MHz")
        self.r_counter_var = tk.StringVar(value="1")
        self.ref_doubler_var = tk.BooleanVar(value=False)
        self.ref_div2_var = tk.BooleanVar(value=False)
        self.ref_diff_var = tk.BooleanVar(value=False)

        self.cp_code_var = tk.StringVar(value="0b0010")
        self.muxout_var = tk.StringVar(value="110")
        self.rfouta_power_var = tk.StringVar(value="+2 dBm")
        self.mtld_var = tk.BooleanVar(value=False)
        self.neg_bleed_var = tk.BooleanVar(value=False)
        self.gated_bleed_var = tk.BooleanVar(value=False)
        self.bleed_code_var = tk.StringVar(value="0")
        self.le_sync_var = tk.BooleanVar(value=True)

        self.summary_text_vars = {
            "fpfd": tk.StringVar(),
            "vco": tk.StringVar(),
            "rfdiv": tk.StringVar(),
            "feedback": tk.StringVar(),
            "int": tk.StringVar(),
            "frac1": tk.StringVar(),
            "frac2": tk.StringVar(),
            "mod2": tk.StringVar(),
        }

        self.mux_warning_var = tk.StringVar(value="")

    def _build_ui(self):
        top = ttk.Frame(self, padding=10)
        top.pack(fill="x")

        ttk.Label(top, text="ADF5355 Dual Board Controller", font=("Segoe UI", 16, "bold")).pack(side="left")

        status_frame = ttk.Frame(top)
        status_frame.pack(side="right")

        self.lock_a_label = ttk.Label(
            status_frame, textvariable=self.lock_a_var,
            font=("Segoe UI", 10, "bold"), style="LockUnknown.TLabel"
        )
        self.lock_a_label.pack(side="right", padx=(8, 0))

        self.lock_b_label = ttk.Label(
            status_frame, textvariable=self.lock_b_var,
            font=("Segoe UI", 10, "bold"), style="LockUnknown.TLabel"
        )
        self.lock_b_label.pack(side="right", padx=(8, 0))

        ttk.Label(status_frame, textvariable=self.status_var, font=("Segoe UI", 11)).pack(side="right", padx=(8, 0))

        main = ttk.Panedwindow(self, orient="horizontal")
        main.pack(fill="both", expand=True, padx=10, pady=(0, 10))

        left = ttk.Frame(main, padding=10)
        right = ttk.Frame(main, padding=10)
        main.add(left, weight=3)
        main.add(right, weight=2)

        self._build_connection_frame(left)
        self._build_synth_frame(left)
        self._build_summary_frame(left)
        self._build_actions_frame(left)
        self._build_log_frame(left)
        self._build_registers_frame(right)

    def _build_connection_frame(self, parent):
        frm = ttk.LabelFrame(parent, text="Connection", padding=10)
        frm.pack(fill="x", pady=(0, 8))

        ttk.Label(frm, text="COM Port").grid(row=0, column=0, sticky="w")
        self.port_combo = ttk.Combobox(frm, textvariable=self.port_var, width=18, state="readonly")
        self.port_combo.grid(row=0, column=1, padx=6, sticky="w")
        ttk.Button(frm, text="Refresh", command=self.refresh_ports).grid(row=0, column=2, padx=4)
        ttk.Button(frm, text="Connect", command=self.connect_port).grid(row=0, column=3, padx=4)
        ttk.Button(frm, text="Disconnect", command=self.disconnect_port).grid(row=0, column=4, padx=4)

        ttk.Label(frm, text="Board").grid(row=1, column=0, sticky="w", pady=(8, 0))
        ttk.Radiobutton(frm, text="Board A", variable=self.board_var, value="A",
                        command=self.on_param_change).grid(row=1, column=1, sticky="w", pady=(8, 0))
        ttk.Radiobutton(frm, text="Board B", variable=self.board_var, value="B",
                        command=self.on_param_change).grid(row=1, column=2, sticky="w", pady=(8, 0))

    def _build_synth_frame(self, parent):
        frm = ttk.LabelFrame(parent, text="Synthesizer Setup", padding=10)
        frm.pack(fill="x", pady=(0, 8))

        r = 0

        ttk.Label(frm, text="Output Path").grid(row=r, column=0, sticky="w")
        ttk.Radiobutton(frm, text="RFOUTA", variable=self.output_var, value="A",
                        command=self.on_param_change).grid(row=r, column=1, sticky="w")
        ttk.Radiobutton(frm, text="RFOUTB", variable=self.output_var, value="B",
                        command=self.on_param_change).grid(row=r, column=2, sticky="w")

        r += 1
        ttk.Label(frm, text="Output Frequency").grid(row=r, column=0, sticky="w", pady=(8, 0))
        ttk.Entry(frm, textvariable=self.freq_var, width=16).grid(row=r, column=1, sticky="w", pady=(8, 0))
        ttk.Combobox(frm, textvariable=self.units_var, values=["Hz", "kHz", "MHz", "GHz"],
                     width=8, state="readonly").grid(row=r, column=2, sticky="w", pady=(8, 0))

        r += 1
        ttk.Label(frm, text="Channel Spacing").grid(row=r, column=0, sticky="w", pady=(8, 0))
        ttk.Entry(frm, textvariable=self.channel_spacing_var, width=16).grid(row=r, column=1, sticky="w", pady=(8, 0))
        ttk.Combobox(frm, textvariable=self.channel_spacing_units_var, values=["Hz", "kHz", "MHz"],
                     width=8, state="readonly").grid(row=r, column=2, sticky="w", pady=(8, 0))

        r += 1
        ttk.Label(frm, text="Reference Frequency").grid(row=r, column=0, sticky="w", pady=(8, 0))
        ttk.Entry(frm, textvariable=self.ref_freq_var, width=16).grid(row=r, column=1, sticky="w", pady=(8, 0))
        ttk.Combobox(frm, textvariable=self.ref_units_var, values=["Hz", "kHz", "MHz", "GHz"],
                     width=8, state="readonly").grid(row=r, column=2, sticky="w", pady=(8, 0))

        r += 1
        ttk.Label(frm, text="R Counter").grid(row=r, column=0, sticky="w", pady=(8, 0))
        ttk.Entry(frm, textvariable=self.r_counter_var, width=16).grid(row=r, column=1, sticky="w", pady=(8, 0))

        r += 1
        ttk.Checkbutton(frm, text="Reference Doubler", variable=self.ref_doubler_var,
                        command=self.on_param_change).grid(row=r, column=0, sticky="w", pady=(8, 0))
        ttk.Checkbutton(frm, text="Reference ÷2", variable=self.ref_div2_var,
                        command=self.on_param_change).grid(row=r, column=1, sticky="w", pady=(8, 0))
        ttk.Checkbutton(frm, text="Differential Reference", variable=self.ref_diff_var,
                        command=self.on_param_change).grid(row=r, column=2, sticky="w", pady=(8, 0))

        r += 1
        ttk.Label(frm, text="RFOUTA Power").grid(row=r, column=0, sticky="w", pady=(8, 0))
        ttk.Combobox(frm, textvariable=self.rfouta_power_var,
                     values=["-4 dBm", "-1 dBm", "+2 dBm", "+5 dBm"],
                     width=12, state="readonly").grid(row=r, column=1, sticky="w", pady=(8, 0))

        r += 1
        ttk.Label(frm, text="Charge Pump Code").grid(row=r, column=0, sticky="w", pady=(8, 0))
        ttk.Combobox(frm, textvariable=self.cp_code_var,
                     values=[f"0b{i:04b}" for i in range(16)],
                     width=12, state="readonly").grid(row=r, column=1, sticky="w", pady=(8, 0))

        r += 1
        ttk.Label(frm, text="MUXOUT Code").grid(row=r, column=0, sticky="w", pady=(8, 0))
        ttk.Combobox(frm, textvariable=self.muxout_var,
                     values=["000", "001", "010", "011", "100", "101", "110"],
                     width=12, state="readonly").grid(row=r, column=1, sticky="w", pady=(8, 0))

        r += 1
        self.mux_warning_label = ttk.Label(frm, textvariable=self.mux_warning_var, style="Warning.TLabel")
        self.mux_warning_label.grid(row=r, column=0, columnspan=3, sticky="w", pady=(2, 0))

        r += 1
        ttk.Checkbutton(frm, text="Mute Till Lock Detect", variable=self.mtld_var,
                        command=self.on_param_change).grid(row=r, column=0, sticky="w", pady=(8, 0))
        ttk.Checkbutton(frm, text="Negative Bleed", variable=self.neg_bleed_var,
                        command=self.on_param_change).grid(row=r, column=1, sticky="w", pady=(8, 0))
        ttk.Checkbutton(frm, text="Gated Bleed", variable=self.gated_bleed_var,
                        command=self.on_param_change).grid(row=r, column=2, sticky="w", pady=(8, 0))

        r += 1
        ttk.Label(frm, text="Bleed Code").grid(row=r, column=0, sticky="w", pady=(8, 0))
        ttk.Entry(frm, textvariable=self.bleed_code_var, width=16).grid(row=r, column=1, sticky="w", pady=(8, 0))
        ttk.Checkbutton(frm, text="LE Sync", variable=self.le_sync_var,
                        command=self.on_param_change).grid(row=r, column=2, sticky="w", pady=(8, 0))

        for child in frm.winfo_children():
            if isinstance(child, (ttk.Entry, ttk.Combobox)):
                child.bind("<KeyRelease>", self.on_param_change)
                child.bind("<<ComboboxSelected>>", self.on_param_change)

    def _build_summary_frame(self, parent):
        frm = ttk.LabelFrame(parent, text="Calculated Summary", padding=10)
        frm.pack(fill="x", pady=(0, 8))

        labels = [
            ("PFD Frequency", "fpfd"),
            ("VCO Frequency", "vco"),
            ("RF Divider", "rfdiv"),
            ("Feedback Mode", "feedback"),
            ("INT", "int"),
            ("FRAC1", "frac1"),
            ("FRAC2", "frac2"),
            ("MOD2", "mod2"),
        ]

        for i, (label, key) in enumerate(labels):
            ttk.Label(frm, text=label).grid(row=i // 2, column=(i % 2) * 2, sticky="w", padx=(0, 8), pady=2)
            ttk.Label(frm, textvariable=self.summary_text_vars[key], width=28).grid(
                row=i // 2, column=(i % 2) * 2 + 1, sticky="w", pady=2
            )

    def _build_actions_frame(self, parent):
        frm = ttk.LabelFrame(parent, text="Actions", padding=10)
        frm.pack(fill="x", pady=(0, 8))

        ttk.Checkbutton(frm, text="Force Full Initialization", variable=self.force_init_var).grid(row=0, column=0, sticky="w")
        ttk.Button(frm, text="Recalculate", command=self.recalculate).grid(row=0, column=1, padx=5)
        ttk.Button(frm, text="Refresh Lock", command=self.refresh_lock_status).grid(row=0, column=2, padx=5)
        ttk.Button(frm, text="Program Device", command=self.program_device).grid(row=0, column=3, padx=5)
        ttk.Button(frm, text="Read Active Dump", command=self.read_active_dump).grid(row=0, column=4, padx=5)
        ttk.Button(frm, text="Save Registers", command=self.save_registers).grid(row=0, column=5, padx=5)
        ttk.Button(frm, text="Copy Register Dump", command=self.copy_register_dump).grid(row=0, column=6, padx=5)

    def _build_log_frame(self, parent):
        frm = ttk.LabelFrame(parent, text="Status", padding=10)
        frm.pack(fill="both", expand=True, pady=(0, 8))

        self.log_box = tk.Text(frm, height=10, wrap="word", state="disabled")
        self.log_box.pack(fill="both", expand=True)
        self.log("Application started.")

    def _build_registers_frame(self, parent):
        frm = ttk.LabelFrame(parent, text="Register View", padding=10)
        frm.pack(fill="both", expand=True)

        columns = ("reg", "hex", "dec")
        self.tree = ttk.Treeview(frm, columns=columns, show="headings", height=18)
        self.tree.heading("reg", text="Register")
        self.tree.heading("hex", text="Hex")
        self.tree.heading("dec", text="Decimal")
        self.tree.column("reg", width=80, anchor="center")
        self.tree.column("hex", width=150, anchor="center")
        self.tree.column("dec", width=180, anchor="center")
        self.tree.pack(fill="both", expand=True)

        for i in range(13):
            self.tree.insert("", "end", iid=str(i), values=(f"R{i}", "0x00000000", "0"))

    def log(self, text: str):
        self.log_box.configure(state="normal")
        self.log_box.insert("end", text + "\n")
        self.log_box.see("end")
        self.log_box.configure(state="disabled")

    def refresh_ports(self):
        ports = []
        if serial is not None:
            ports = [p.device for p in serial.tools.list_ports.comports()]
        self.port_combo["values"] = ports
        if ports:
            if self.port_var.get() not in ports:
                self.port_var.set(ports[0])
        else:
            self.port_var.set("")

    def connect_port(self):
        port = self.port_var.get().strip()
        if not port:
            messagebox.showerror("No Port", "Select a COM port first.")
            return
        try:
            self.pico = PicoADF5355(port)
            self.pico.startup()
            self.status_var.set(f"Connected: {port}")
            self.log(f"Connected to {port}.")
            self.refresh_lock_status()
        except Exception as e:
            self.pico = None
            messagebox.showerror("Connection Error", str(e))
            self.log(f"Connection failed: {e}")

    def disconnect_port(self):
        if self.pico:
            try:
                self.pico.close()
            except Exception:
                pass
        self.pico = None
        self.status_var.set("Disconnected")
        self.set_lock_display("A", None)
        self.set_lock_display("B", None)
        self.log("Disconnected.")

    @staticmethod
    def unit_scale(unit: str) -> float:
        return {"Hz": 1.0, "kHz": 1e3, "MHz": 1e6, "GHz": 1e9}[unit]

    def build_config_from_ui(self) -> ADF5355Config:
        power_map = {"-4 dBm": 0b00, "-1 dBm": 0b01, "+2 dBm": 0b10, "+5 dBm": 0b11}

        cp_code = int(self.cp_code_var.get().replace("0b", ""), 2)
        mux_code = int(self.muxout_var.get(), 2)
        bleed_code = int(self.bleed_code_var.get())
        r_counter = int(self.r_counter_var.get())
        ref_hz = float(self.ref_freq_var.get()) * self.unit_scale(self.ref_units_var.get())

        if cp_code < 0 or cp_code > 15:
            raise ValueError("Charge Pump Code must be between 0 and 15")
        if mux_code < 0 or mux_code > 7:
            raise ValueError("MUXOUT code must be between 000 and 111")
        if bleed_code < 0 or bleed_code > 255:
            raise ValueError("Bleed Code must be between 0 and 255")
        if r_counter < 1 or r_counter > 1023:
            raise ValueError("R Counter must be between 1 and 1023")
        if ref_hz <= 0:
            raise ValueError("Reference frequency must be greater than 0")

        return ADF5355Config(
            ref_in_hz=ref_hz,
            r_counter=r_counter,
            ref_doubler=self.ref_doubler_var.get(),
            ref_div2=self.ref_div2_var.get(),
            ref_mode_differential=self.ref_diff_var.get(),
            cp_current_code=cp_code,
            muxout_code=mux_code,
            pd_polarity_positive=True,
            rfouta_power_code=power_map[self.rfouta_power_var.get()],
            enable_mtld=self.mtld_var.get(),
            enable_negative_bleed=self.neg_bleed_var.get(),
            enable_gated_bleed=self.gated_bleed_var.get(),
            bleed_current_code=bleed_code,
            le_sync=self.le_sync_var.get(),
        )

    def get_request_values(self):
        freq_hz = float(self.freq_var.get()) * self.unit_scale(self.units_var.get())
        spacing_hz = float(self.channel_spacing_var.get()) * self.unit_scale(self.channel_spacing_units_var.get())

        if freq_hz <= 0:
            raise ValueError("Output frequency must be greater than 0")
        if spacing_hz <= 0:
            raise ValueError("Channel spacing must be greater than 0")

        return freq_hz, spacing_hz

    def compute_current_settings(self):
        cfg = self.build_config_from_ui()
        freq_hz, spacing_hz = self.get_request_values()
        builder = ADF5355RegisterBuilder(cfg)
        summary = builder.calculate_summary(freq_hz, self.output_var.get(), spacing_hz)
        regs = builder.build_registers(freq_hz, self.output_var.get(), cfg.rfouta_power_code, spacing_hz)
        return cfg, freq_hz, spacing_hz, summary, regs

    def recalculate(self):
        self.recalc_after_id = None
        try:
            cfg, freq_hz, spacing_hz, summary, regs = self.compute_current_settings()
            self.current_regs = regs
            self.last_valid_settings = (cfg, freq_hz, spacing_hz, summary, regs)
            self.last_recalc_error = None

            self.summary_text_vars["fpfd"].set(f"{summary['fpfd_hz']/1e6:.6f} MHz")
            self.summary_text_vars["vco"].set(f"{summary['vco_hz']/1e9:.9f} GHz")
            self.summary_text_vars["rfdiv"].set(str(summary["rf_divider"]))
            self.summary_text_vars["feedback"].set(summary["feedback_mode"])
            self.summary_text_vars["int"].set(str(summary["int"]))
            self.summary_text_vars["frac1"].set(str(summary["frac1"]))
            self.summary_text_vars["frac2"].set(str(summary["frac2"]))
            self.summary_text_vars["mod2"].set(str(summary["mod2"]))
            self.update_register_tree(regs)

            if cfg.muxout_code != 0b110:
                self.mux_warning_var.set("Warning: Lock readback requires MUXOUT = 110 (Digital Lock Detect).")
            else:
                self.mux_warning_var.set("")

        except Exception as e:
            for key in self.summary_text_vars:
                self.summary_text_vars[key].set("Error")
            self.mux_warning_var.set("")
            if str(e) != self.last_recalc_error:
                self.log(f"Recalculate error: {e}")
                self.last_recalc_error = str(e)

    def update_register_tree(self, regs: List[int]):
        for i, reg in enumerate(regs):
            self.tree.item(str(i), values=(f"R{i}", f"0x{reg:08X}", str(reg)))

    def on_param_change(self, event=None):
        if self.recalc_after_id is not None:
            try:
                self.after_cancel(self.recalc_after_id)
            except Exception:
                pass
        self.recalc_after_id = self.after(80, self.recalculate)

    def program_device(self):
        if self.pico is None:
            messagebox.showerror("Not Connected", "Connect to the Pico first.")
            return

        try:
            cfg, freq_hz, spacing_hz, summary, regs = self.compute_current_settings()
            self.current_regs = regs
            self.last_valid_settings = (cfg, freq_hz, spacing_hz, summary, regs)
            self.update_register_tree(regs)

            if cfg.muxout_code != 0b110:
                proceed = messagebox.askyesno(
                    "MUXOUT Warning",
                    "MUXOUT is not set to 110 (Digital Lock Detect).\n\n"
                    "The lock indicators may be invalid.\n\n"
                    "Continue programming anyway?"
                )
                if not proceed:
                    return

        except Exception as e:
            messagebox.showerror("Invalid Settings", str(e))
            return

        board = self.board_var.get()
        regs_snapshot = list(self.current_regs)
        force_init = self.force_init_var.get()

        def worker():
            try:
                reply = self.pico.select_board(board)
                if reply:
                    self.serial_queue.put(("log", f"Board select reply: {reply}"))

                replies = self.pico.write_registers(regs_snapshot)
                for rep in replies:
                    if rep:
                        self.serial_queue.put(("log", rep))
                self.serial_queue.put(("log", f"Registers staged to board {board}."))

                if force_init:
                    init_reply = self.pico.init_device()
                    if init_reply:
                        self.serial_queue.put(("log", init_reply))
                else:
                    freq_reply = self.pico.update_frequency()
                    if freq_reply:
                        self.serial_queue.put(("log", freq_reply))

                lock_reply = self.pico.read_lock_status()
                if lock_reply:
                    self.serial_queue.put(("lock", lock_reply))

                self.serial_queue.put(("status", f"Programmed Board {board}"))

            except Exception as e:
                self.serial_queue.put(("error", str(e)))

        threading.Thread(target=worker, daemon=True).start()

    def refresh_lock_status(self):
        if self.pico is None:
            return

        def worker():
            try:
                lock_reply = self.pico.read_lock_status()
                if lock_reply:
                    self.serial_queue.put(("lock", lock_reply))
            except Exception as e:
                self.serial_queue.put(("log", f"Lock read error: {e}"))

        threading.Thread(target=worker, daemon=True).start()

    def read_active_dump(self):
        if self.pico is None:
            messagebox.showerror("Not Connected", "Connect to the Pico first.")
            return

        def worker():
            try:
                board = self.board_var.get()
                reply = self.pico.select_board(board)
                if reply:
                    self.serial_queue.put(("log", f"Board select reply: {reply}"))
                lines = self.pico.dump_active()
                for line in lines:
                    self.serial_queue.put(("log", line))
            except Exception as e:
                self.serial_queue.put(("error", str(e)))

        threading.Thread(target=worker, daemon=True).start()

    def process_serial_queue(self):
        try:
            while True:
                kind, payload = self.serial_queue.get_nowait()
                if kind == "log":
                    self.log(payload)
                elif kind == "status":
                    self.status_var.set(payload)
                    self.log(payload)
                elif kind == "lock":
                    self.apply_lock_status(payload)
                    self.log(payload)
                elif kind == "error":
                    self.log(f"Programming error: {payload}")
                    messagebox.showerror("Programming Error", payload)
        except queue.Empty:
            pass

        self.after(150, self.process_serial_queue)

    def set_lock_display(self, board: str, state):
        if board == "A":
            label = self.lock_a_label
            if state is None:
                self.lock_a_var.set("A: ?")
                label.configure(style="LockUnknown.TLabel")
            elif state:
                self.lock_a_var.set("A: LOCK")
                label.configure(style="LockOn.TLabel")
            else:
                self.lock_a_var.set("A: UNLOCK")
                label.configure(style="LockOff.TLabel")
        elif board == "B":
            label = self.lock_b_label
            if state is None:
                self.lock_b_var.set("B: ?")
                label.configure(style="LockUnknown.TLabel")
            elif state:
                self.lock_b_var.set("B: LOCK")
                label.configure(style="LockOn.TLabel")
            else:
                self.lock_b_var.set("B: UNLOCK")
                label.configure(style="LockOff.TLabel")

    def apply_lock_status(self, text: str):
        try:
            if text.startswith("LOCK:"):
                parts = text.split(":")
                a_val = None
                b_val = None
                for part in parts[1:]:
                    if part.startswith("A="):
                        a_val = part.split("=")[1].strip()
                    elif part.startswith("B="):
                        b_val = part.split("=")[1].strip()
                if a_val is not None:
                    self.set_lock_display("A", a_val == "1")
                if b_val is not None:
                    self.set_lock_display("B", b_val == "1")

            elif ":LOCKED" in text or ":UNLOCKED" in text:
                parts = text.split(":")
                if len(parts) >= 3:
                    board = parts[1].strip()
                    state = parts[2].strip()
                    if board == "A":
                        self.set_lock_display("A", state == "LOCKED")
                    elif board == "B":
                        self.set_lock_display("B", state == "LOCKED")

        except Exception as e:
            self.log(f"Lock parse error: {e} :: {text}")

    def copy_register_dump(self):
        dump = "\n".join([f"R{i:02d} = 0x{reg:08X} ({reg})" for i, reg in enumerate(self.current_regs)])
        self.clipboard_clear()
        self.clipboard_append(dump)
        self.log("Register dump copied to clipboard.")

    def save_registers(self):
        path = filedialog.asksaveasfilename(
            defaultextension=".txt",
            filetypes=[("Text Files", "*.txt"), ("All Files", "*.*")]
        )
        if not path:
            return

        with open(path, "w", encoding="utf-8") as f:
            f.write("ADF5355 Register Dump\n")
            f.write(f"Board: {self.board_var.get()}\n")
            f.write(f"Output: RFOUT{self.output_var.get()}\n\n")
            for i, reg in enumerate(self.current_regs):
                f.write(f"R{i:02d} = 0x{reg:08X} ({reg})\n")

        self.log(f"Saved register dump to {path}")

    def on_close(self):
        try:
            if self.pico:
                self.pico.close()
        except Exception:
            pass
        self.destroy()


if __name__ == "__main__":
    app = ADF5355DualGUI()
    app.mainloop()
