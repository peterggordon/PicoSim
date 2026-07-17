# main.py
# Entry point for PicoSim GUI application
#
# This version restores the full Trends tab implementation from main_bkp.py
# while keeping the newer control code (V420 toggle, V422 pulse, V61 feedback LEDs).

import tkinter as tk
from tkinter import ttk, messagebox, filedialog, simpledialog
import threading
import time
from collections import deque
from datetime import datetime, timedelta
import re
import pathlib

from network import FINSClient
from data_parser import parse_snapshot, FIELD_MAP
from gui_components import ask_setpoint, ValveToggle
from data_logger import DataLogger, HighResPressureCapture, LOG_FOLDER
# Matplotlib for embedded plots in the Trends tab
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import matplotlib as mpl
from itertools import cycle


# GUI version (shown in the window title)
APP_VERSION = "1.0.3"

MAX_HEAT_NOTE_MSG = (
    "Note: Maximum heating setpoint is not high enough.  "
    "Target setpoint will not be reached."
)


# ----------------------------
# Command maps (control side)
# ----------------------------

SETPOINT_COMMANDS = {
    # Temperature control registers (user-verified PLC ground truth)
    #   - TE1 button = chamber heater maximum (DM16)
    #   - TE2 button = chamber target setpoint (CIO122)
    #   - TE210 button = Line 2 / Source B target setpoint (DM145)
    #   - TE211 button = Line 2 / Source B heater maximum (DM10)
    "TE1":   ("DM", 16),
    "TE2":   ("CIO", 122),
    "TE110": ("CIO", 0x006B),
    "TE210": ("DM", 145),
    "TE211": ("DM", 10),
    "TE310": ("CIO", 0x006E),

    "PT1":   ("CIO", 0x0070),
    "PT2":   ("CIO", 0x0071),
    "PT100": ("CIO", 0x0072),
    "PT200": ("CIO", 0x0073),
    "PT300": ("CIO", 0x0074),
    "PT400": ("CIO", 0x0075),
    "PT420": ("CIO", 0x0076),

    "MFC1":   ("CIO", 0x0077),
    "MFC100": ("CIO", 0x0078),
    "MFC200": ("CIO", 0x0079),
    "MFC300": ("CIO", 0x007B),
    "MFC400": ("CIO", 0x007E),
}

# Lid control (treated as command+state bits, since no reliable feedback was found)
LID_OPEN_CMD  = (12, 0x07)  # set to 1 to open (paired with LID_CLOSE_CMD=0)
LID_CLOSE_CMD = (12, 0x08)  # set to 1 when closed (set to 0 to allow opening)

VALVE_COMMANDS = {
    # NOTE: User-verified command bits are in CIO105.xx
    # IMPORTANT: V1/V2 mapping here reflects the corrected behavior requested by the user:
    #   - V1 = CIO105.00 (1=open)
    #   - V2 = CIO105.01 (1=open)
    "V1":   (105, 0x00),
    "V2":   (105, 0x01),

    # Other valves on the same CIO word (best-known from prior work)
        "V6":   (105, 0x02),  # command bit; NOTE: inverted logic (0=open, 1=closed) handled in code below

    "V110": (105, 0x03),
    "V210": (105, 0x05),  # user-verified: CIO105.05
    "V211": (105, 0x0C),
    "V310": (105, 0x0D),

    # Line 4 mapping (solved):
    # V420 hold-open enable = CIO102.09
    # V422 pulse            = CIO107.10
    "V420": (102, 0x09),
    "V422": (107, 0x0A),
}


# ----------------------------
# Trends tab configuration
# ----------------------------

# Canonical tag lists for the Trends tab (actuals only)
TREND_TEMP_TAGS = ["TE1", "TE2", "TE110", "TE210", "TE211", "TE310"]
TREND_PRESS_TAGS = ["PT1", "PT2", "PT100", "PT200", "PT300", "PT400", "PT420"]
TREND_FLOW_TAGS = ["MFC1", "MFC100", "MFC200", "MFC300", "MFC400", "MFM420"]
TREND_ALL_TAGS = TREND_TEMP_TAGS + TREND_PRESS_TAGS + TREND_FLOW_TAGS

# For hi-res run logging: include every field parsed from the snapshot.
# (This matches what DataLogger.log_snapshot() writes, just at higher cadence.)
HIRES_RUN_TAGS = sorted({v[0] for v in FIELD_MAP.values()})

# Top-of-window status header (visible on all tabs)
STATUS_TAGS = [
    "TE1", "TE2", "TE110", "TE210", "TE211", "TE310",
    "PT1", "PT2", "PT100", "PT200", "PT300", "PT400", "PT420",
    "MFC1", "MFC100", "MFC200", "MFC300", "MFC400", "MFM420",
]


def unit_for_tag(tag: str) -> str:
    """Return display units for a given process tag."""
    if tag.startswith("TE"):
        return "°C"
    if tag.startswith("PT"):
        return "hPa"
    if tag.startswith("MFC") or tag.startswith("MFM"):
        return "sccm"
    return ""

# Default y-axis ranges for optional fixed scaling
TEMP_Y_DEFAULT = (0.0, 300.0)    # °C
PRESS_Y_DEFAULT = (0.0, 1100.0)  # hPa
FLOW_Y_DEFAULT = (0.0, 2000.0)   # sccm

# Length of the history window in seconds
TREND_WINDOW_SECONDS = 60



# High-rate plotting buffer for pressure traces (fed by the ~10 Hz snapshot poll).
# Plotting only; 1 Hz buffers + DATA.txt logging remain unchanged.
PRESS_HI_HZ = 20.0  
PRESS_HI_MAXLEN = int(TREND_WINDOW_SECONDS * PRESS_HI_HZ) + 5
class PicoSimApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title(f"PicoSim v{APP_VERSION}")
        self.geometry("1200x800")

        try:
            self.fins = FINSClient()
            self.ip_address = self.fins.sock.getsockname()[0] if self.fins.sock else "Unavailable"
        except Exception as e:
            self.fins = None
            self.ip_address = "Unavailable"
            print(f"Warning: Network initialization failed — {e}")

        self.online = False
        self.snapshot_data = {}
        self._vent_active = False
        self._evac_active = False



        # Cache of selected CIO words we read explicitly for state inference (e.g., lid/valve command bits)
        self.cio_cache = {}
        # --- V61 feedback state ---
        self._last_v61_poll = 0.0

        # --- Trends/logging cadence ---
        self._last_trend_log_time = 0.0  # 1 Hz

        # --- Top status header (visible on all tabs) ---
        self._build_status_header()

        # --- Tabs ---
        self.tabs = ttk.Notebook(self)
        self.tabs.pack(expand=1, fill="both")

        self.network_tab = ttk.Frame(self.tabs)
        self.trends_tab = ttk.Frame(self.tabs)
        self.recipe_tab = ttk.Frame(self.tabs)

        self.tabs.add(self.network_tab, text="Network")
        self.tabs.add(self.trends_tab, text="Manual")
        self.tabs.add(self.recipe_tab, text="Recipe")

        self._build_network_tab()
        self._build_trends_tab()
        self._build_recipe_tab()


        # Hi-res run capture (CSV) — starts/stops automatically with recipe runs.
        # Captures all snapshot fields at a higher cadence than the normal 1 Hz DATA.txt log.
        try:
            self.run_hires_capture = HighResPressureCapture(
                log_folder_getter=lambda: getattr(getattr(self, "logger", None), "folder", LOG_FOLDER),
                snapshot_getter=lambda: dict(getattr(self, "snapshot_data", {}) or {}),
                tags=HIRES_RUN_TAGS,
                target_hz=float(PRESS_HI_HZ),
                flush_every=50,
                filename_prefix="RUN_HIRES",
            )
        except Exception as e:
            self.run_hires_capture = None
            print(f"Warning: run hi-res capture init failed — {e}")
        # Cached CIO words for simple state reflection (command bits treated as state)
        self.cio_cache = {}
        self._last_cio_poll = 0.0

        # Manual tab: readback of max/setpoints displayed next to TE buttons
        self._last_setpoint_meta_poll = 0.0

        # --- Background polling thread ---
        self.running = True
        self.poll_thread = threading.Thread(target=self._poll_loop, daemon=True)
        self.poll_thread.start()

        # Ensure window-close handler exists even if later indentation edits regress.
        def _on_close_handler():
            # Stop background work
            try:
                self.running = False
            except Exception:
                pass

            # Stop hi-res run capture
            try:
                self._stop_run_hires_capture()
            except Exception:
                pass
            # Best-effort logger close
            if getattr(self, "logger", None) is not None:
                try:
                    self.logger.close()
                except Exception:
                    pass
            # Close UDP socket if present
            if getattr(self, "fins", None) is not None:
                try:
                    self.fins.close()
                except Exception:
                    pass
            try:
                self.destroy()
            except Exception:
                pass

        # Expose as instance attribute for app.protocol(...)
        self.on_close = _on_close_handler


    # ----------------------------
    # Hi-res run logging (CSV)
    # ----------------------------

    def _sanitize_filename_component(self, s: str, max_len: int = 60) -> str:
        """Make a safe filename component (letters/digits/underscore) for log prefixes."""
        s = (s or "").strip()
        if not s:
            return "Recipe"
        s = re.sub(r"[^A-Za-z0-9]+", "_", s).strip("_")
        if not s:
            s = "Recipe"
        return s[:max_len]

    def _get_recipe_title_for_logging(self, recipe: dict) -> str:
        """Best-effort title used for hi-res run log filenames."""
        for k in ("title", "name", "recipe_name"):
            v = recipe.get(k)
            if isinstance(v, str) and v.strip():
                return v.strip()

        p = recipe.get("_source_path")
        if isinstance(p, str) and p.strip():
            return pathlib.Path(p).stem

        p = getattr(self, "current_recipe_path", None)
        if isinstance(p, str) and p.strip():
            return pathlib.Path(p).stem

        return "Recipe"

    def _build_run_hires_prefix(self, recipe: dict, queue_active: bool) -> str:
        title = self._get_recipe_title_for_logging(recipe)
        safe = self._sanitize_filename_component(title)
        if queue_active:
            try:
                n = len(getattr(self, "_queue_run_list", []) or [])
            except Exception:
                n = 0
            suffix = f"_x{n}" if n else ""
            return f"QUEUE_{safe}{suffix}"
        return f"RUN_{safe}"


    def _start_run_hires_capture(self, recipe: dict) -> None:
        """Start hi-res run CSV capture. For queue runs, stays open across recipes."""
        cap = getattr(self, "run_hires_capture", None)
        if cap is None:
            return
        if not getattr(self, "online", False):
            return
        try:
            if cap.is_running():
                return

            queue_active = bool(getattr(self, "_queue_run_list", None))
            cap.filename_prefix = self._build_run_hires_prefix(recipe, queue_active=queue_active)

            fname = cap.start()
            if fname:
                print(f"Run hi-res capture started: {fname}")
        except Exception as e:
            print(f"Run hi-res capture start failed: {e}")


    def _stop_run_hires_capture(self) -> None:
        """Stop the hi-res run CSV capture if running."""
        cap = getattr(self, "run_hires_capture", None)
        if cap is None:
            return
        try:
            if cap.is_running():
                cap.stop()
                print(
                    f"Run hi-res capture stopped: {cap.filename} "
                    f"({cap.samples_written} samples, ~{cap.effective_hz:.1f} Hz achieved)"
                )
        except Exception as e:
            print(f"Run hi-res capture stop failed: {e}")
    # ----------------------------
    # Status header (shown above the Notebook; effectively "on every tab")
    # ----------------------------

    def _seconds_to_hms(self, seconds: float) -> str:
        """Format seconds as HH:MM:SS (robust to bad inputs)."""
        try:
            s = int(round(float(seconds)))
        except Exception:
            s = 0
        h = s // 3600
        m = (s % 3600) // 60
        ss = s % 60
        return f"{h:02d}:{m:02d}:{ss:02d}"

    def _build_status_header(self):
        """
        Build a compact 2-row status table that stays visible regardless of which tab is active.

        Row 0: tag names
        Row 1: live values ("----" if unavailable or "0000")
        """
        self.status_header_frame = ttk.Frame(self)
        self.status_header_frame.pack(side="top", fill="x", padx=6, pady=(6, 0))

        # Use tk.Label for consistent borders across ttk themes
        header_font = ("Arial", 9, "bold")
        value_font = ("Arial", 9)

        self.status_header_value_labels = {}

        for col, tag in enumerate(STATUS_TAGS):
            h = tk.Label(
                self.status_header_frame,
                text=tag,
                font=header_font,
                borderwidth=1,
                relief="solid",
                padx=6,
                pady=2,
            )
            h.grid(row=0, column=col, sticky="nsew")

            v = tk.Label(
                self.status_header_frame,
                text=f"---- {unit_for_tag(tag)}",
                font=value_font,
                borderwidth=1,
                relief="solid",
                padx=6,
                pady=2,
            )
            v.grid(row=1, column=col, sticky="nsew")

            self.status_header_value_labels[tag] = v
            self.status_header_frame.grid_columnconfigure(col, weight=1)

    def _update_status_header(self):
        """
        Update the status header from the latest snapshot_data.

        Rule: if the value is exactly "0000", treat as no data and show "----".
        """
        if not hasattr(self, "status_header_value_labels"):
            return

        for tag, lbl in self.status_header_value_labels.items():
            val = self.snapshot_data.get(tag, "0000")
            unit = unit_for_tag(tag)
            if val == "0000":
                lbl.config(text=f"---- {unit}" if unit else "----")
            else:
                lbl.config(text=f"{val} {unit}" if unit else str(val))

# Manual tab
    # ----------------------------

    def _build_manual_controls(self, parent):
        """
        Build the Manual controls UI into the provided parent frame.

        This was originally the entire Manual tab; it is now embedded on the left
        side of the Trends tab, so we can have Manual + Monitoring in one place.
        """
        ttk.Label(parent, text="Setpoints", font=("Arial", 14)).grid(
            row=0, column=0, padx=10, pady=10, sticky="w"
        )

        # --- Setpoints (individual) ---
        # Live readback labels (read from PLC) for heater max / setpoints.
        # These are informational only; the actual set action is via the buttons.
        self._manual_setpoint_meta_vars = {}

        controls = [
            ("TE1", "°C"), ("TE2", "°C"), ("TE210", "°C"), ("TE211", "°C"),
            ("MFC1", "sccm"), ("MFC100", "sccm"), ("MFC200", "sccm"),
            ("MFC300", "sccm"), ("MFC400", "sccm"),
        ]

        for i, (label, unit) in enumerate(controls):
            row = i + 1
            ttk.Label(parent, text=label).grid(row=row, column=0, padx=10, sticky="w")
            b = ttk.Button(
                parent,
                text=f"Set {label}",
                command=lambda l=label, u=unit: self._popup_setpoint(l, u),
            )
            b.grid(row=row, column=1, padx=10, pady=5, sticky="w")

            # Meta readback (max/setpoint) for TE controls
            if label in ("TE1", "TE2", "TE210", "TE211"):
                if label == "TE1":
                    default = "(Max = ---- °C)"
                elif label == "TE2":
                    default = "(Setpoint = ---- °C)"
                elif label == "TE210":
                    default = "(Line 2 Setpoint = ---- °C)"
                else:  # TE211
                    default = "(Line 2 Max = ---- °C)"

                v = tk.StringVar(value=default)
                ttk.Label(parent, textvariable=v, foreground="gray").grid(
                    row=row,
                    column=2,
                    padx=(0, 10),
                    sticky="w",
                )
                self._manual_setpoint_meta_vars[label] = v

        # --- Convenience: set all line flows (100/200/300/400) ---
        flow_row = len(controls) + 1
        ttk.Label(parent, text="All Line Flows").grid(row=flow_row, column=0, padx=10, sticky="w")
        self.all_line_flow_var = tk.StringVar(value="40")
        ttk.Entry(parent, textvariable=self.all_line_flow_var, width=8).grid(
            row=flow_row, column=1, padx=10, pady=5, sticky="w"
        )
        ttk.Label(parent, text="sccm").grid(row=flow_row, column=1, padx=(70, 0), sticky="w")

        ttk.Button(
            parent,
            text="Set all line flows",
            command=self._set_all_line_flows,
        ).grid(row=flow_row + 1, column=1, padx=10, pady=(0, 8), sticky="w")

        # --- Valves ---
        ttk.Label(parent, text="Valves", font=("Arial", 14)).grid(
            row=0, column=3, padx=20, pady=10, sticky="w"
        )

        # Standard valves (simple toggles)
        standard_valves = ["V1", "V2", "V110", "V210", "V211", "V310", "V420"]
        self.valve_states = {}

        for i, v in enumerate(standard_valves):
            row = i + 1
            vt = ValveToggle(parent, v, self._toggle_valve)
            vt.grid(row=row, column=3, padx=10, sticky="w")
            self.valve_states[v] = vt

        # V6: inverted semantics (checked = CLOSED, unchecked = OPEN)
        v6_row = len(standard_valves) + 1
        v6_frame = ttk.Frame(parent)
        v6_frame.grid(row=v6_row, column=3, padx=10, sticky="w")

        self.v6_closed_var = tk.BooleanVar(value=False)  # False => OPEN
        self.v6_chk = ttk.Checkbutton(
            v6_frame,
            text="V6",
            variable=self.v6_closed_var,
            command=self._on_v6_toggle,
        )
        self.v6_chk.pack(side="left")

        self.v6_indicator = tk.Canvas(v6_frame, width=12, height=12, highlightthickness=0)
        self.v6_indicator.pack(side="left", padx=4)

        self.v6_state_text = ttk.Label(v6_frame, text="(OPEN)")
        self.v6_state_text.pack(side="left", padx=(4, 0))

        self._update_v6_indicator()

        # V422: momentary pulse (check -> pulse for duration -> auto-uncheck)
        pulse_row = v6_row + 1

        self.v422_pulse_var = tk.BooleanVar(value=False)
        self.v422_pulse_duration_s = tk.StringVar(value="0.25")  # seconds

        pulse_frame = ttk.Frame(parent)
        pulse_frame.grid(row=pulse_row, column=3, padx=10, sticky="w")

        self.v422_pulse_btn = ttk.Checkbutton(
            pulse_frame,
            text="V422 (pulse)",
            variable=self.v422_pulse_var,
            command=self._on_v422_pulse,
        )
        self.v422_pulse_btn.pack(side="left")

        ttk.Label(pulse_frame, text="t (s):").pack(side="left", padx=(8, 2))
        ttk.Entry(pulse_frame, textvariable=self.v422_pulse_duration_s, width=6).pack(side="left")

        self.v422_indicator = tk.Canvas(pulse_frame, width=12, height=12, highlightthickness=0)
        self.v422_indicator.pack(side="left", padx=4)
        self._set_led(self.v422_indicator, False)

        # Lid controls (command+state inferred from bits 12.07 / 12.08)
        lid_row = pulse_row + 1
        lid_frame = ttk.Frame(parent)
        lid_frame.grid(row=lid_row, column=3, padx=10, pady=(10, 0), sticky="w")

        ttk.Label(lid_frame, text="Lid:").pack(side="left")

        self.open_lid_btn = ttk.Button(lid_frame, text="OPEN LID", command=self._open_lid)
        self.open_lid_btn.pack(side="left", padx=(8, 4))

        self.close_lid_btn = ttk.Button(lid_frame, text="CLOSE LID", command=self._close_lid)
        self.close_lid_btn.pack(side="left", padx=4)

        self._update_lid_buttons()


        # --- Vent / Evac controls (Manual tab) ---
        vent_frame = ttk.Frame(parent)
        vent_frame.grid(row=lid_row + 1, column=3, padx=10, pady=(10, 0), sticky="w")

        self.vent_btn = ttk.Button(vent_frame, text="Vent", width=10, command=self._start_vent)
        self.vent_btn.pack(side="left", padx=(0, 6))

        self.vent_cancel_btn = ttk.Button(vent_frame, text="Cancel", width=10, command=self._cancel_vent)
        self.vent_cancel_btn.pack(side="left")

        evac_frame = ttk.Frame(parent)
        evac_frame.grid(row=lid_row + 2, column=3, padx=10, pady=(6, 0), sticky="w")

        self.evac_btn = ttk.Button(evac_frame, text="Evac", width=10, command=self._start_evac)
        self.evac_btn.pack(side="left", padx=(0, 6))

        self.evac_cancel_btn = ttk.Button(evac_frame, text="Cancel", width=10, command=self._cancel_evac)
        self.evac_cancel_btn.pack(side="left")

        # Status text for vent/evac actions
        self.vent_evac_status = ttk.Label(parent, text="", foreground="gray")
        self.vent_evac_status.grid(row=lid_row + 3, column=3, padx=10, pady=(6, 0), sticky="w")


    def _popup_setpoint(self, label, unit):
        """Prompt user for a setpoint and write it to the PLC.

        Temperatures (TE*) are entered in °C and written as 0.1 °C units.
        For heating, TE2/TE210 are targets while TE1/TE211 are maximum heater limits.
        """
        if not self.online or not self.fins:
            print("Offline — cannot setpoint")
            return

        if label in ("TE1", "TE2", "TE210", "TE211"):
            val_c = ask_setpoint("Setpoint Entry", label, unit, allow_float=True)
            if val_c is None:
                return
            try:
                val_raw = int(round(float(val_c) * 10.0))
            except Exception:
                messagebox.showerror("Invalid value", "Enter a numeric temperature.")
                return

            if label not in SETPOINT_COMMANDS:
                print(f"No command mapping found for {label}")
                return
            area, address = SETPOINT_COMMANDS[label]
            success = self.fins.write_word(address, val_raw, area=area)
            print(f"Set {label} to {val_c} {unit}: {'OK' if success else 'FAILED'}")

            # Best-effort note if max limits are too low.
            self._warn_if_max_heating_low()
            return

        value = ask_setpoint("Setpoint Entry", label, unit)
        if value is None:
            return
        if label not in SETPOINT_COMMANDS:
            print(f"No command mapping found for {label}")
            return

        area, address = SETPOINT_COMMANDS[label]
        success = self.fins.write_word(address, int(value), area=area)
        print(f"Set {label} to {value} {unit}: {'OK' if success else 'FAILED'}")

    def _warn_if_max_heating_low(self):
        """Warn (once per user action) if a max heater limit is <= target setpoint.

        Ground truth:
          - Chamber target setpoint: CIO122 (0.1 °C)
          - Chamber heater max:      DM16  (0.1 °C)
          - Line 2 target setpoint:  DM145 (0.1 °C)
          - Line 2 heater max:       DM10  (0.1 °C)
        """
        if not self.online or not self.fins:
            return

        try:
            chamber_sp = self.fins.read_word(122, area="CIO")
            chamber_max = self.fins.read_word(16, area="DM")
            line2_sp = self.fins.read_word(145, area="DM")
            line2_max = self.fins.read_word(10, area="DM")
        except Exception:
            return

        if chamber_sp is not None and chamber_max is not None:
            if int(chamber_max) <= int(chamber_sp):
                messagebox.showwarning("Note", MAX_HEAT_NOTE_MSG)
                return

        if line2_sp is not None and line2_max is not None:
            if int(line2_max) <= int(line2_sp):
                messagebox.showwarning("Note", MAX_HEAT_NOTE_MSG)


    def _toggle_valve(self, name, is_open):
        if not self.online or not self.fins:
            print("Offline — cannot toggle valve")
            return
        if name not in VALVE_COMMANDS:
            print(f"No command mapping for valve {name}")
            return
        word, bit = VALVE_COMMANDS[name]

        # V6 is inverted (0=open, 1=closed)
        if name == "V6":
            value = 0 if is_open else 1
        else:
            value = 1 if is_open else 0

        success = self.fins.write_bit(word, bit, value)
        print(f"Set {name} to {'OPEN' if is_open else 'CLOSE'}: {'OK' if success else 'FAILED'}")

    # --- LED helpers ---
    def _set_led(self, canvas: tk.Canvas, is_on: bool):
        self._set_led_color(canvas, "green" if is_on else "red")

    def _set_led_color(self, canvas: tk.Canvas, color: str):
        canvas.delete("all")
        canvas.create_oval(2, 2, 10, 10, fill=color, outline=color)

    def _on_v422_pulse(self):
        if not self.v422_pulse_var.get():
            return

        if not self.online or not self.fins:
            print("Offline — cannot pulse V422")
            self.v422_pulse_var.set(False)
            self._set_led(self.v422_indicator, False)
            return

        try:
            duration_s = float(self.v422_pulse_duration_s.get().strip())
            if duration_s <= 0:
                raise ValueError
        except ValueError:
            print("Invalid V422 pulse duration (must be a positive number, seconds).")
            self.v422_pulse_var.set(False)
            self._set_led(self.v422_indicator, False)
            return

        duration_s = min(duration_s, 10.0)

        self.v422_pulse_btn.state(["disabled"])
        t = threading.Thread(target=self._v422_pulse_worker, args=(duration_s,), daemon=True)
        t.start()

    def _v422_pulse_worker(self, duration_s: float):
        ok_on = False
        ok_off = False
        try:
            word, bit = VALVE_COMMANDS["V422"]
            ok_on = self.fins.write_bit(word, bit, 1)
            self.after(0, lambda: self._set_led(self.v422_indicator, ok_on))
            time.sleep(duration_s)
            ok_off = self.fins.write_bit(word, bit, 0)
        finally:
            def _reset_ui():
                self.v422_pulse_var.set(False)
                self._set_led(self.v422_indicator, False)
                self.v422_pulse_btn.state(["!disabled"])
                if not ok_on:
                    print("V422 pulse: FAILED to set ON")
                if ok_on and not ok_off:
                    print("V422 pulse: FAILED to set OFF")

            self.after(0, _reset_ui)

    # ----------------------------
    # Network tab
    # ----------------------------

    def _build_network_tab(self):
        self.network_status = ttk.Label(
            self.network_tab,
            text="Status: OFFLINE",
            foreground="red",
            font=("Arial", 14),
        )
        self.network_status.pack(pady=20)

        self.connect_btn = ttk.Button(
            self.network_tab,
            text="Connect to Tool",
            command=self._connect_to_tool,
        )
        self.connect_btn.pack()

        self.ip_label = ttk.Label(self.network_tab, text=f"Tool IP: {self.ip_address}", font=("Arial", 12))
        self.ip_label.pack(pady=10)

    def _connect_to_tool(self):
        if not self.fins:
            self.network_status.configure(text="Status: OFFLINE - No FINS client", foreground="red")
            return

        response = self.fins.read_snapshot()
        if response:
            self.online = True
            self.network_status.configure(text="Status: ONLINE", foreground="green")
        else:
            self.online = False
            self.network_status.configure(text="Status: OFFLINE - No response", foreground="red")

    # ----------------------------
    # Trends tab (restored)
    # ----------------------------

    def _build_trends_tab(self):
        """
        Build the merged Manual + Trends layout:

        Left side (two columns):
          - Manual controls (original Manual tab layout), unchanged.
          - Live Values + plot toggles + y-range controls + logging.

        Right side:
          - Embedded matplotlib figure with 3 stacked subplots:
              Temps, Pressures, Flows, sharing a common time axis.
        """
        manual_frame = ttk.Frame(self.trends_tab)
        manual_frame.pack(side="left", fill="y", padx=10, pady=10)
        self._build_manual_controls(manual_frame)

        live_frame = ttk.Frame(self.trends_tab)
        live_frame.pack(side="left", fill="y", padx=10, pady=10)

        right_frame = ttk.Frame(self.trends_tab)
        right_frame.pack(side="right", expand=True, fill="both", padx=10, pady=10)


        # --- Current values display + checkboxes ---
        ttk.Label(live_frame, text="Live Values", font=("Arial", 12, "bold")).grid(
            row=0, column=0, columnspan=2, sticky="w", pady=(0, 5)
        )

        self.trend_value_labels = {}
        self.trend_check_vars = {}

        row = 1

        def add_group(title, tags):
            nonlocal row
            ttk.Label(live_frame, text=title, font=("Arial", 11, "bold")).grid(
                row=row, column=0, columnspan=2, sticky="w", pady=(5, 0)
            )
            row += 1
            for tag in tags:
                # Value label
                val_label = ttk.Label(live_frame, text=f"{tag}: ----", width=18)
                val_label.grid(row=row, column=0, sticky="w", pady=1)
                self.trend_value_labels[tag] = val_label

                # Checkbox to enable plotting
                var = tk.BooleanVar(value=False)
                chk = ttk.Checkbutton(live_frame, text="Plot", variable=var, command=self._update_trend_visibility)
                chk.grid(row=row, column=1, sticky="w", pady=1)
                self.trend_check_vars[tag] = var

                row += 1

        add_group("Temperatures (°C)", TREND_TEMP_TAGS)
        add_group("Pressures (hPa)", TREND_PRESS_TAGS)
        add_group("Flows (sccm)", TREND_FLOW_TAGS)

        # --- Y-axis controls ---
        row += 1
        ttk.Label(live_frame, text="Y-axis scaling", font=("Arial", 11, "bold")).grid(
            row=row, column=0, columnspan=2, sticky="w", pady=(8, 0)
        )
        row += 1

        self.use_fixed_ranges = tk.BooleanVar(value=False)

        fixed_chk = ttk.Checkbutton(
            live_frame,
            text="Use fixed ranges (overrides autoscale)",
            variable=self.use_fixed_ranges,
            command=self._redraw_trend_plots,
        )
        fixed_chk.grid(row=row, column=0, columnspan=2, sticky="w")
        row += 1

        # Min / max entries per family
        self.temp_ymin_var = tk.DoubleVar(value=TEMP_Y_DEFAULT[0])
        self.temp_ymax_var = tk.DoubleVar(value=TEMP_Y_DEFAULT[1])
        self.press_ymin_var = tk.DoubleVar(value=PRESS_Y_DEFAULT[0])
        self.press_ymax_var = tk.DoubleVar(value=PRESS_Y_DEFAULT[1])
        self.flow_ymin_var = tk.DoubleVar(value=FLOW_Y_DEFAULT[0])
        self.flow_ymax_var = tk.DoubleVar(value=FLOW_Y_DEFAULT[1])

        def add_y_range_row(label, vmin, vmax):
            nonlocal row
            ttk.Label(live_frame, text=label).grid(row=row, column=0, sticky="w", pady=2)
            frame = ttk.Frame(live_frame)
            frame.grid(row=row, column=1, sticky="w", pady=2)
            ttk.Entry(frame, width=6, textvariable=vmin).grid(row=0, column=0, padx=(0, 2))
            ttk.Label(frame, text="to").grid(row=0, column=1)
            ttk.Entry(frame, width=6, textvariable=vmax).grid(row=0, column=2, padx=(2, 0))
            row += 1

        add_y_range_row("Temp (°C)", self.temp_ymin_var, self.temp_ymax_var)
        add_y_range_row("Press (hPa)", self.press_ymin_var, self.press_ymax_var)
        add_y_range_row("Flow (sccm)", self.flow_ymin_var, self.flow_ymax_var)

        # --- Logger buttons (folder controls) ---
        row += 1
        ttk.Label(live_frame, text="Data Logging", font=("Arial", 11, "bold")).grid(
            row=row, column=0, columnspan=2, sticky="w", pady=(8, 0)
        )
        row += 1

        # Dedicated frame for DataLogger UI (it uses pack internally)
        logger_frame = ttk.Frame(live_frame)
        logger_frame.grid(row=row, column=0, columnspan=2, sticky="w")
        self.logger = DataLogger(parent_frame=logger_frame)
        row += 1

        # --- Matplotlib figure / canvas on right ---
        self.trend_figure = Figure(figsize=(6, 5), dpi=100)
        self.ax_temp = self.trend_figure.add_subplot(3, 1, 1)
        self.ax_press = self.trend_figure.add_subplot(3, 1, 2, sharex=self.ax_temp)
        self.ax_flow = self.trend_figure.add_subplot(3, 1, 3, sharex=self.ax_temp)

        self.ax_temp.set_ylabel("°C")
        self.ax_press.set_ylabel("hPa")
        self.ax_flow.set_ylabel("sccm")
        self.ax_flow.set_xlabel("Time (seconds ago)")

        # Data buffers and Line2D objects
        self.trend_buffers = {
            tag: {"t": deque(maxlen=TREND_WINDOW_SECONDS), "v": deque(maxlen=TREND_WINDOW_SECONDS)}
            for tag in TREND_ALL_TAGS
        }

        # High-rate buffers for pressure plotting (fed from snapshot polling).
        self.press_hi_buffers = {
            tag: {"t": deque(maxlen=PRESS_HI_MAXLEN), "v": deque(maxlen=PRESS_HI_MAXLEN)}
            for tag in TREND_PRESS_TAGS
        }
        self.trend_lines = {}

        # Deterministic colors: Matplotlib default cycle
        colors = mpl.rcParams["axes.prop_cycle"].by_key().get("color", [])
        if not colors:
            colors = [f"C{i}" for i in range(10)]
        color_cycle = cycle(colors)
        tag_colors = {tag: next(color_cycle) for tag in TREND_ALL_TAGS}

        # Create one line per tag on the appropriate axis
        for tag in TREND_TEMP_TAGS:
            line, = self.ax_temp.plot([], [], label=tag, color=tag_colors[tag])
            self.trend_lines[tag] = line
        for tag in TREND_PRESS_TAGS:
            line, = self.ax_press.plot([], [], label=tag, color=tag_colors[tag])
            self.trend_lines[tag] = line
        for tag in TREND_FLOW_TAGS:
            line, = self.ax_flow.plot([], [], label=tag, color=tag_colors[tag])
            self.trend_lines[tag] = line

        # External legends (will only show enabled lines)
        for ax, tags in ((self.ax_temp, TREND_TEMP_TAGS), (self.ax_press, TREND_PRESS_TAGS), (self.ax_flow, TREND_FLOW_TAGS)):
            ax.legend(loc="center left", bbox_to_anchor=(1.02, 0.5), fontsize="small")

        # Embed figure in Tk
        self.trend_canvas = FigureCanvasTkAgg(self.trend_figure, master=right_frame)
        self.trend_canvas.get_tk_widget().pack(fill="both", expand=True)

        # Label showing absolute time range at edges of the window
        self.trend_time_label = ttk.Label(right_frame, text="")
        self.trend_time_label.pack(anchor="e", pady=(2, 0))

        self.ax_temp.set_xlim(TREND_WINDOW_SECONDS, 0)
        self.trend_figure.tight_layout()
        # Space for external legends
        self.trend_figure.subplots_adjust(right=0.78)
        self.trend_canvas.draw_idle()

    def _update_trend_visibility(self):
        self._redraw_trend_plots()

    def _update_trend_values_column(self):
        """
        Update the 'Live Values' column on the Trends tab using snapshot_data,
        treating '0000' as no data.
        """
        snapshot = getattr(self, "snapshot_data", {})
        for tag, lbl in self.trend_value_labels.items():
            val = snapshot.get(tag, "0000")
            if val == "0000":
                lbl.config(text=f"{tag}: ----")
            else:
                lbl.config(text=f"{tag}: {val}")

    def _append_to_trend_buffers(self, snapshot, now_epoch):
        """
        Push snapshot values into the 60-second buffers (1 Hz), skipping invalid "0000".
        """
        if not snapshot:
            return

        for tag in TREND_ALL_TAGS:
            val = snapshot.get(tag, "0000")
            if val == "0000":
                continue

            if isinstance(val, str):
                try:
                    val_num = float(val)
                except ValueError:
                    continue
            else:
                val_num = float(val)

            buf = self.trend_buffers[tag]
            buf["t"].append(now_epoch)
            buf["v"].append(val_num)

    def _append_to_press_hi_buffers(self, snapshot, now_epoch):
        """Append pressure values into high-rate buffers (plotting only)."""
        if not snapshot:
            return
        # press_hi_buffers is created in _build_trends_tab
        if not hasattr(self, "press_hi_buffers"):
            return

        for tag in TREND_PRESS_TAGS:
            val = snapshot.get(tag, "0000")
            if val == "0000":
                continue
            try:
                val_num = float(val) if isinstance(val, str) else float(val)
            except Exception:
                continue
            buf = self.press_hi_buffers[tag]
            buf["t"].append(now_epoch)
            buf["v"].append(val_num)

    def _redraw_trend_plots(self):
        """
        Redraw the matplotlib plots based on current buffers and checkbox states.
        X-axis is 'seconds ago' (0 = now).
        """
        if not hasattr(self, "trend_canvas"):
            return

        now = time.time()

        for tag in TREND_ALL_TAGS:
            line = self.trend_lines[tag]
            # Pressure traces use high-rate buffers; other traces use the 1 Hz buffers.
            buf = self.press_hi_buffers[tag] if tag in TREND_PRESS_TAGS else self.trend_buffers[tag]
            times = list(buf["t"])
            values = list(buf["v"])

            if times and values and self.trend_check_vars[tag].get():
                x = [now - t for t in times]  # seconds ago
                x_vals, y_vals = [], []
                for xi, yi in zip(x, values):
                    if 0.0 <= xi <= TREND_WINDOW_SECONDS:
                        x_vals.append(xi)
                        y_vals.append(yi)
                line.set_data(x_vals, y_vals)
            else:
                line.set_data([], [])

        self.ax_temp.set_xlim(TREND_WINDOW_SECONDS, 0)

        if self.use_fixed_ranges.get():
            self.ax_temp.set_ylim(self.temp_ymin_var.get(), self.temp_ymax_var.get())
            self.ax_press.set_ylim(self.press_ymin_var.get(), self.press_ymax_var.get())
            self.ax_flow.set_ylim(self.flow_ymin_var.get(), self.flow_ymax_var.get())
        else:
            for ax in (self.ax_temp, self.ax_press, self.ax_flow):
                ax.relim()
                ax.autoscale_view()

        def set_external_legend(ax, tags):
            handles, labels = [], []
            for tag in tags:
                line = self.trend_lines[tag]
                if len(line.get_xdata()) > 0 and self.trend_check_vars[tag].get():
                    handles.append(line)
                    labels.append(tag)
            ax.legend(handles, labels, loc="center left", bbox_to_anchor=(1.02, 0.5), fontsize="small")

        set_external_legend(self.ax_temp, TREND_TEMP_TAGS)
        set_external_legend(self.ax_press, TREND_PRESS_TAGS)
        set_external_legend(self.ax_flow, TREND_FLOW_TAGS)

        start_epoch = now - TREND_WINDOW_SECONDS
        start_str = datetime.fromtimestamp(start_epoch).strftime("%H:%M:%S")
        end_str = datetime.fromtimestamp(now).strftime("%H:%M:%S")
        self.trend_time_label.config(text=f"Window: {start_str} – {end_str} (seconds ago on axis)")

        self.trend_canvas.draw_idle()

    # ----------------------------
    # Poll loop (background)
    # ----------------------------

    def _poll_loop(self):
        """
        Background thread:
          - Polls the PLC snapshot at ~10 Hz.
          - Updates Status + Trends live values via Tk 'after' callbacks.
          - Every ~1 s, appends to trends/logs and redraws the Trends plots.
        """
        while self.running:
            now = time.time()

            if self.online and self.fins:
                # Snapshot (CIO110 block)
                response = self.fins.read_snapshot()
                if response:
                    parsed = parse_snapshot(response)
                    if "error" not in parsed:
                        self.snapshot_data = parsed


                        # Feed high-rate pressure plot buffers (uses snapshot polling cadence)
                        self._append_to_press_hi_buffers(parsed, now)
                        # UI updates on the Tk thread
                        self.after(0, self._update_status_header)
                        self.after(0, self._update_trend_values_column)

                        # 1 Hz logging + trends
                        if (now - self._last_trend_log_time) >= 1.0:
                            self._last_trend_log_time = now

                            # File logging (thread-safe; no Tk calls here)
                            if getattr(self, "logger", None) is not None:
                                try:
                                    self.logger.log_snapshot(self.snapshot_data)
                                except Exception as e:
                                    print(f"Logger error: {e}")

                            # Trend buffers + redraw should run on the Tk thread (TkAgg safety)
                            def _trend_tick(snap=self.snapshot_data, tstamp=now):
                                self._append_to_trend_buffers(snap, tstamp)
                                self._redraw_trend_plots()

                            self.after(0, _trend_tick)

            # Poll a few CIO words for command/state reflection (lid + key valves)
            if self.online and self.fins and (now - self._last_cio_poll) >= 0.5:
                self._last_cio_poll = now
                try:
                    w105 = self.fins.read_cio_words(105, 1)
                    if w105 and len(w105) == 1:
                        self.cio_cache[105] = w105[0]
                    w12 = self.fins.read_cio_words(12, 1)
                    if w12 and len(w12) == 1:
                        self.cio_cache[12] = w12[0]
                    # Update UI representations of V6 + lid buttons
                    self.after(0, self._refresh_manual_command_states)
                except Exception:
                    pass

            # Poll heater limits / targets for the informational labels next to TE buttons (1 Hz)
            if self.online and self.fins and (now - self._last_setpoint_meta_poll) >= 1.0:
                self._last_setpoint_meta_poll = now
                try:
                    chamber_max = self.fins.read_word(16, area="DM")
                    chamber_sp = self.fins.read_word(122, area="CIO")
                    line2_sp = self.fins.read_word(145, area="DM")
                    line2_max = self.fins.read_word(10, area="DM")

                    def _meta_tick(
                        cmax=chamber_max,
                        csp=chamber_sp,
                        l2sp=line2_sp,
                        l2max=line2_max,
                    ):
                        self._set_manual_setpoint_meta_values(
                            te1_max_c=(cmax / 10.0) if cmax is not None else None,
                            te2_sp_c=(csp / 10.0) if csp is not None else None,
                            te210_sp_c=(l2sp / 10.0) if l2sp is not None else None,
                            te211_max_c=(l2max / 10.0) if l2max is not None else None,
                        )

                    self.after(0, _meta_tick)
                except Exception:
                    pass

            # If a run ended unexpectedly (error/cancel), make sure hi-res capture closes.
            try:
                cap = getattr(self, 'run_hires_capture', None)
                if cap is not None and cap.is_running() and self.run_state is None and not getattr(self, '_queue_run_list', None):
                    cap.stop()
            except Exception:
                pass

            time.sleep(0.05)  # ~20 Hz snapshot polling; adjust as needed for responsiveness vs CPU load

    

    # ----------------------------
    # Manual helpers
    # ----------------------------

    @staticmethod
    def _fmt_c(val_c):
        """Pretty-print a Celsius value for compact UI labels."""
        if val_c is None:
            return "----"
        try:
            f = float(val_c)
        except Exception:
            return "----"
        # If effectively an integer, drop the trailing .0
        if abs(f - round(f)) < 1e-9:
            return str(int(round(f)))
        return f"{f:.1f}"

    def _set_manual_setpoint_meta_values(
        self,
        *,
        te1_max_c=None,
        te2_sp_c=None,
        te210_sp_c=None,
        te211_max_c=None,
    ):
        """Update the '(Max = ...)' and '(Setpoint = ...)' labels in the Manual UI."""
        if not hasattr(self, "_manual_setpoint_meta_vars"):
            return

        v = self._manual_setpoint_meta_vars
        if "TE1" in v and te1_max_c is not None:
            v["TE1"].set(f"(Max = {self._fmt_c(te1_max_c)} °C)")
        if "TE2" in v and te2_sp_c is not None:
            v["TE2"].set(f"(Setpoint = {self._fmt_c(te2_sp_c)} °C)")
        if "TE210" in v and te210_sp_c is not None:
            v["TE210"].set(f"(Line 2 Setpoint = {self._fmt_c(te210_sp_c)} °C)")
        if "TE211" in v and te211_max_c is not None:
            v["TE211"].set(f"(Line 2 Max = {self._fmt_c(te211_max_c)} °C)")

    def _set_all_line_flows(self):
        """Set MFC100/200/300/400 to a single value (sccm)."""
        if not self.online or not self.fins:
            return
        try:
            val = int(float(self.all_line_flow_var.get()))
        except Exception:
            messagebox.showerror("Invalid flow", "Enter a numeric flow value (sccm).")
            return

        for tag in ("MFC100", "MFC200", "MFC300", "MFC400"):
            area, addr = SETPOINT_COMMANDS[tag]
            self.fins.write_word(addr, val, area=area)

    def _on_v6_toggle(self):
        """Checkbox means CLOSED; unchecked means OPEN."""
        is_closed = bool(self.v6_closed_var.get())
        # Convert to the semantic used by _toggle_valve (is_open)
        self._toggle_valve("V6", is_open=not is_closed)
        self._update_v6_indicator()

    def _update_v6_indicator(self):
        """Red=CLOSED, Green=OPEN, plus text indicator."""
        is_closed = bool(self.v6_closed_var.get())
        self.v6_indicator.delete("all")
        color = "red" if is_closed else "green"
        self.v6_indicator.create_oval(2, 2, 10, 10, fill=color, outline=color)
        self.v6_state_text.config(text="(CLOSED)" if is_closed else "(OPEN)")


    def _refresh_manual_command_states(self):
        """Reflect command-bit states into the Manual controls (best-effort).

        Since we don't have separate feedback bits, we treat the command bits as the current state.
        """
        w105 = self.cio_cache.get(105)
        if w105 is not None:
            # Standard valves
            if hasattr(self, "valve_states"):
                for name, widget in self.valve_states.items():
                    if name not in VALVE_COMMANDS:
                        continue
                    word, bit = VALVE_COMMANDS[name]
                    if word != 105:
                        continue
                    is_set = bool((w105 >> bit) & 0x1)
                    # V6 is handled separately
                    if name == "V6":
                        continue
                    if hasattr(widget, "set_state"):
                        widget.set_state(is_set)
                    elif hasattr(widget, "state"):
                        widget.state.set(is_set)
                        if hasattr(widget, "_update_light"):
                            widget._update_light()
                    else:
                        # Fallback for older widgets
                        try:
                            widget.var.set(is_set)
                        except Exception:
                            pass

            # V6 inverted: bit=1 means CLOSED, bit=0 means OPEN
            if hasattr(self, "v6_closed_var"):
                v6_word, v6_bit = VALVE_COMMANDS["V6"]
                if v6_word == 105:
                    v6_closed = bool((w105 >> v6_bit) & 0x1)
                    self.v6_closed_var.set(v6_closed)
                    self._update_v6_indicator()

        # Lid buttons depend on word 12 and PT1
        self._update_lid_buttons()

    def _open_lid(self):
        if not self.online or not self.fins:
            return

        # Only permit opening at (near) atmosphere
        pt1 = float(self.snapshot_data.get("PT1", 0.0)) if getattr(self, "snapshot_data", None) else 0.0
        if pt1 < 950.0:
            messagebox.showwarning("Cannot open lid", "PT1 must be > 950 hPa to open the lid.")
            return

        # Step 1: release the close command first
        ok1 = self.fins.write_bit(LID_CLOSE_CMD[0], LID_CLOSE_CMD[1], 0)

        def _assert_open():
            ok2 = self.fins.write_bit(LID_OPEN_CMD[0], LID_OPEN_CMD[1], 1)

            # Best-effort local cache update
            w12 = self.cio_cache.get(12, 0)
            w12 &= ~(1 << LID_CLOSE_CMD[1])   # close bit low
            w12 |=  (1 << LID_OPEN_CMD[1])    # open bit high
            self.cio_cache[12] = w12
            self._update_lid_buttons()

            print(f"OPEN LID: release_close={'OK' if ok1 else 'FAILED'}, assert_open={'OK' if ok2 else 'FAILED'}")

        # Small dwell so PLC can see the close-release before the open command
        self.after(150, _assert_open)

    def _close_lid(self):
        if not self.online or not self.fins:
            return
        # Close lid: 12.07=0 (PLC sets 12.08=1 automatically)
        self.fins.write_bit(LID_OPEN_CMD[0], LID_OPEN_CMD[1], 0)
        # best-effort: set close bit too (harmless if PLC overrides)
        self.fins.write_bit(LID_CLOSE_CMD[0], LID_CLOSE_CMD[1], 1)

        self.cio_cache[LID_OPEN_CMD[0]] = self.cio_cache.get(LID_OPEN_CMD[0], 0) & ~(1 << LID_OPEN_CMD[1])
        self.cio_cache[LID_CLOSE_CMD[0]] = self.cio_cache.get(LID_CLOSE_CMD[0], 0) | (1 << LID_CLOSE_CMD[1])
        self._update_lid_buttons()

    # ----------------------------
    # Vent / Evac helpers
    # ----------------------------
    def _set_standby_flows(self):
        """Apply standby flow setpoints (no valve changes)."""
        if not self.online or not self.fins:
            return
        standby = {
            "MFC1": 300,
            "MFC100": 40,
            "MFC200": 40,
            "MFC300": 40,
            "MFC400": 40,
        }
        for tag, val in standby.items():
            if tag not in SETPOINT_COMMANDS:
                continue
            area, addr = SETPOINT_COMMANDS[tag]
            try:
                self.fins.write_word(addr, int(val), area=area)
            except Exception:
                pass

    def _set_all_flows(self, mfc1: int, line_flow: int):
        """Set MFC1 and all line MFCs to the given values."""
        if not self.online or not self.fins:
            return
        targets = {
            "MFC1": mfc1,
            "MFC100": line_flow,
            "MFC200": line_flow,
            "MFC300": line_flow,
            "MFC400": line_flow,
        }
        for tag, val in targets.items():
            if tag not in SETPOINT_COMMANDS:
                continue
            area, addr = SETPOINT_COMMANDS[tag]
            try:
                self.fins.write_word(addr, int(val), area=area)
            except Exception:
                pass

    def _start_vent(self):
        """Vent chamber to atmosphere: close V1/V2, set flows to 2000, wait for PT2>=1050, then return standby flows."""
        if not self.online or not self.fins:
            return
        # stop any other action
        self._vent_active = True
        self._evac_active = False

        # Ensure V1/V2 closed
        self._toggle_valve("V1", False)
        self._toggle_valve("V2", False)

        # High flows to vent
        self._set_all_flows(mfc1=2000, line_flow=2000)

        try:
            self.vent_evac_status.config(text="Venting… waiting for PT1 ≥ 1050 hPa", foreground="gray")
        except Exception:
            pass

        self.after(250, self._vent_tick)

    def _vent_tick(self):
        if not getattr(self, "_vent_active", False):
            return
        try:
            pt1 = float(self.snapshot_data.get("PT1", 0.0))
        except Exception:
            pt1 = 0.0

        if pt1 >= 1050.0:
            # restore standby flows
            self._set_standby_flows()
            self._vent_active = False
            try:
                self.vent_evac_status.config(text="Vent complete (PT1 ≥ 1050 hPa) — standby flows restored", foreground="gray")
            except Exception:
                pass
            return

        self.after(250, self._vent_tick)

    def _cancel_vent(self):
        """Cancel vent: restore standby flows only (no valve changes)."""
        if not self.online or not self.fins:
            return
        self._vent_active = False
        self._set_standby_flows()
        try:
            self.vent_evac_status.config(text="Vent cancelled — standby flows restored", foreground="gray")
        except Exception:
            pass

    def _start_evac(self):
        """Evacuate chamber: set standby flows, then pump down using V2 throttle until PT1<90, then V1."""
        if not self.online or not self.fins:
            return
        self._evac_active = True
        self._vent_active = False

        # Standby flows first
        self._set_standby_flows()

        # Ensure V6 is open (vacuum path)
        if "V6" in VALVE_COMMANDS:
            self._toggle_valve("V6", True)

        try:
            self.vent_evac_status.config(text="Evacuating…", foreground="gray")
        except Exception:
            pass

        self.after(250, self._evac_tick)

    def _evac_tick(self):
        if not getattr(self, "_evac_active", False):
            return

        try:
            pt1 = float(self.snapshot_data.get("PT1", 9999.0))
        except Exception:
            pt1 = 9999.0

        # Chamber-pressure-based interlock:
        #  - If PT2 > 100 hPa: open V2 only (V1 closed)  (this was backwards previously)
        #  - Once PT2 <= 100 hPa: close V2 and open V1 (final evac state)
        if pt1 > 100.0:
            self._toggle_valve("V1", False)
            self._toggle_valve("V2", True)
        else:
            self._toggle_valve("V2", False)
            self._toggle_valve("V1", True)

        # Status (no stop condition; user cancels manually)
        try:
            self.vent_evac_status.config(text=f"Evac active — PT1 {pt1:.1f} hPa (Cancel to stop)", foreground="gray")
        except Exception:
            pass

        self.after(250, self._evac_tick)

    def _cancel_evac(self):
        """Cancel evac: close V1 and V2 only (no flow changes)."""
        if not self.online or not self.fins:
            return
        self._evac_active = False
        self._toggle_valve("V1", False)
        self._toggle_valve("V2", False)
        try:
            self.vent_evac_status.config(text="Evac cancelled — V1/V2 closed", foreground="gray")
        except Exception:
            pass


    def _lid_is_open_inferred(self) -> bool:
        """Infer lid state from the command bits 12.07 / 12.08."""
        w12 = getattr(self, "cio_cache", {}).get(12)
        if w12 is None:
            return False
        b_open = bool((w12 >> LID_OPEN_CMD[1]) & 0x1)
        b_close = bool((w12 >> LID_CLOSE_CMD[1]) & 0x1)
        # heuristic: open if open-bit set and close-bit not set
        return b_open and (not b_close)

    def _update_lid_buttons(self):
        """Grey out OPEN/CLOSE lid buttons based on inferred state and PT1."""
        try:
            pt1 = float(self.snapshot_data.get("PT1", 0.0))
        except Exception:
            pt1 = 0.0

        lid_open = self._lid_is_open_inferred()

        # OPEN only allowed at near atmosphere
        can_open = (not lid_open) and (pt1 >= 950.0) and self.online
        can_close = lid_open and self.online

        if hasattr(self, "open_lid_btn"):
            self.open_lid_btn.state(["!disabled"] if can_open else ["disabled"])
        if hasattr(self, "close_lid_btn"):
            self.close_lid_btn.state(["!disabled"] if can_close else ["disabled"])

    # ----------------------------
    # Recipe tab
    # ----------------------------

    def _build_recipe_tab(self):
        """Build the Recipe tab UI."""
        outer = ttk.Frame(self.recipe_tab)
        outer.pack(fill="both", expand=True, padx=10, pady=10)

        top = ttk.Frame(outer)
        top.pack(fill="x")

        ttk.Label(top, text="ALD Recipe", font=("Arial", 14, "bold")).pack(side="left")

        btns = ttk.Frame(top)
        btns.pack(side="right")

        ttk.Button(btns, text="Create Recipe", command=self._open_recipe_wizard).pack(side="left", padx=4)
        ttk.Button(btns, text="Edit Recipe", command=lambda: self._open_recipe_wizard(prefill=True)).pack(side="left", padx=4)
        ttk.Button(btns, text="Start Over", command=self._clear_current_recipe).pack(side="left", padx=4)

        ttk.Separator(outer, orient="horizontal").pack(fill="x", pady=8)

        mid = ttk.Frame(outer)
        mid.pack(fill="both", expand=True)

        # Left: setpoints summary
        left = ttk.LabelFrame(mid, text="Setpoints (Recipe)")
        left.pack(side="left", fill="y", padx=(0, 10))

        self.recipe_setpoint_vars = {
            "delay_min": tk.StringVar(value="0"),
            "TE1": tk.StringVar(value="200"),
            "TE2": tk.StringVar(value="150"),
            "TE2_tol": tk.StringVar(value="1.0"),
            "TE210": tk.StringVar(value="120"),
            "TE210_tol": tk.StringVar(value="1.0"),
            "TE211": tk.StringVar(value="140"),
            "post_TE2": tk.StringVar(value="20"),
            "post_TE2_tol": tk.StringVar(value="2.0"),
            "post_TE210": tk.StringVar(value="20"),
            "post_TE210_tol": tk.StringVar(value="2.0"),
            "MFC1": tk.StringVar(value="300"),
            "MFC100": tk.StringVar(value="40"),
            "MFC200": tk.StringVar(value="40"),
            "MFC300": tk.StringVar(value="40"),
            "MFC400": tk.StringVar(value="40"),
        }

        sp_rows = [
            ("Delay (min)", "delay_min", ""),
            ("TE1 max", "TE1", "°C"),
            ("TE2 target", "TE2", "°C"),
            ("TE2 tol", "TE2_tol", "°C"),
            ("TE211 max", "TE211", "°C"),
            ("TE210 target", "TE210", "°C"),
            ("TE210 tol", "TE210_tol", "°C"),
            ("Post TE2", "post_TE2", "°C"),
            ("Post tol", "post_TE2_tol", "°C"),
            ("Post TE210", "post_TE210", "°C"),
            ("Post TE210 tol", "post_TE210_tol", "°C"),
            ("MFC1", "MFC1", "sccm"),
            ("MFC100", "MFC100", "sccm"),
            ("MFC200", "MFC200", "sccm"),
            ("MFC300", "MFC300", "sccm"),
            ("MFC400", "MFC400", "sccm"),
        ]
        for r, (lbl, key, unit) in enumerate(sp_rows):
            ttk.Label(left, text=lbl).grid(row=r, column=0, sticky="w", padx=8, pady=2)
            e = ttk.Entry(left, textvariable=self.recipe_setpoint_vars[key], width=10)
            e.grid(row=r, column=1, sticky="w", padx=8, pady=2)
            ttk.Label(left, text=unit).grid(row=r, column=2, sticky="w")

        # Right: recipe steps tree + controls
        right = ttk.Frame(mid)
        right.pack(side="left", fill="both", expand=True)

        tree_frame = ttk.LabelFrame(right, text="Current Recipe")
        tree_frame.pack(fill="both", expand=True)

        cols = ("Block", "Cycles", "Step", "Precursor", "Line", "Pulse(s)", "Purge(s)", "SF_pulse(s)", "SF_hold(s)")
        self.recipe_tree = ttk.Treeview(tree_frame, columns=cols, show="headings", height=14)
        for c in cols:
            self.recipe_tree.heading(c, text=c)
            self.recipe_tree.column(c, width=90, anchor="center")
        self.recipe_tree.column("Precursor", width=140, anchor="w")
        self.recipe_tree.pack(side="left", fill="both", expand=True)

        ysb = ttk.Scrollbar(tree_frame, orient="vertical", command=self.recipe_tree.yview)
        self.recipe_tree.configure(yscroll=ysb.set)
        ysb.pack(side="right", fill="y")

        bottom = ttk.Frame(outer)
        bottom.pack(fill="x", pady=(10, 0))

        self.recipe_status = ttk.Label(bottom, text="Create a recipe to begin.")
        self.recipe_status.pack(side="left")

        # Bottom controls: file ops on the left, primary Run centered, queue/abort on the right
        bottom_controls = ttk.Frame(bottom)
        bottom_controls.pack(fill="x", expand=True)

        file_btns = ttk.Frame(bottom_controls)
        file_btns.pack(side="left")

        ttk.Button(file_btns, text="Save Recipe", command=self._save_current_recipe).pack(side="left", padx=4)
        ttk.Button(file_btns, text="Load Recipe", command=self._load_recipe_from_file).pack(side="left", padx=4)
        ttk.Button(file_btns, text="Queue Builder", command=self._open_queue_builder).pack(side="left", padx=4)

        center_btns = ttk.Frame(bottom_controls)
        center_btns.pack(side="left", fill="x", expand=True)
        # Make the primary action very visible
        ttk.Button(center_btns, text="Run Current Recipe", command=self._run_current_recipe).pack(anchor="center", pady=2)

        right_btns = ttk.Frame(bottom_controls)
        right_btns.pack(side="right")
        ttk.Button(right_btns, text="Run Queue", command=self._run_queue).pack(side="left", padx=4)
        ttk.Button(right_btns, text="Abort", command=self._abort_run).pack(side="left", padx=4)

        # Runner state
        self.current_recipe = None
        self.recipe_queue = []
        self._close_run_progress_popup()
        # If not running a queue, stop hi-res capture at end of this recipe.
        if not getattr(self, '_queue_run_list', None):
            self._stop_run_hires_capture()

        self.run_state = None
        self.run_abort_flag = False

        # Run monitor UI (delay clock + line indicators + progress)
        self.run_monitor = ttk.LabelFrame(self.recipe_tab, text="Run Monitor")
        self.run_monitor.pack(fill="x", padx=10, pady=(0, 10))

        self.delay_clock_var = tk.StringVar(value="--:--")
        self.delay_clock_lbl = ttk.Label(self.run_monitor, textvariable=self.delay_clock_var, font=("Courier New", 18, "bold"), foreground="red")
        self.delay_clock_lbl.pack(side="left", padx=10)

        self.run_timing_var = tk.StringVar(value="Start: --   End: --")
        ttk.Label(self.run_monitor, textvariable=self.run_timing_var).pack(side="left", padx=10)

        self.line_indicator_frame = ttk.Frame(self.run_monitor)
        self.line_indicator_frame.pack(side="right", padx=10)
        self.line_indicators = {}  # line -> (canvas, label)

    # ----------------------------
    # Recipe persistence / UI
    # ----------------------------

    def _clear_current_recipe(self):
        self.current_recipe = None
        if hasattr(self, "recipe_tree"):
            for i in self.recipe_tree.get_children():
                self.recipe_tree.delete(i)
        if hasattr(self, "recipe_status"):
            self.recipe_status.config(text="Create a recipe to begin.")
        self._rebuild_line_indicators([])

    def _rebuild_line_indicators(self, selected_lines):
        for w in self.line_indicator_frame.winfo_children():
            w.destroy()
        self.line_indicators = {}
        for line, name in selected_lines:
            f = ttk.Frame(self.line_indicator_frame)
            f.pack(side="left", padx=6)
            c = tk.Canvas(f, width=12, height=12, highlightthickness=0)
            c.pack(side="left")
            c.create_oval(2, 2, 10, 10, fill="red", outline="red")
            ttk.Label(f, text=f"Line {line}: {name}").pack(side="left", padx=(4, 0))
            self.line_indicators[line] = c

    def _set_line_indicator(self, line: int, active: bool):
        c = self.line_indicators.get(line)
        if not c:
            return
        c.delete("all")
        color = "green" if active else "red"
        c.create_oval(2, 2, 10, 10, fill=color, outline=color)

    def _save_current_recipe(self):
        if not self.current_recipe:
            messagebox.showinfo("No recipe", "Create a recipe first.")
            return
        path = filedialog.asksaveasfilename(defaultextension=".json", filetypes=[("Recipe JSON", "*.json")])
        if not path:
            return
        try:
            import json
            with open(path, "w", encoding="utf-8") as f:
                json.dump(self.current_recipe, f, indent=2)
                self.current_recipe_path = path
            self.recipe_status.config(text=f"Saved: {path}")
        except Exception as e:
            messagebox.showerror("Save failed", str(e))

    
    def _load_recipe_from_file(self):
        """Load a recipe JSON file and populate the Recipe tab."""
        path = filedialog.askopenfilename(filetypes=[("Recipe JSON", "*.json"), ("All files", "*.*")])
        if not path:
            return
        try:
            import json
            with open(path, "r", encoding="utf-8") as f:
                recipe = json.load(f)
            # Minimal schema validation
            if not isinstance(recipe, dict) or "blocks" not in recipe or "setpoints" not in recipe:
                messagebox.showerror("Invalid recipe", "That file does not look like a PicoSim recipe JSON.")
                return
            self.current_recipe = recipe
            self.current_recipe_path = path

            # Update setpoint widgets (best-effort)
            sp = recipe.get("setpoints", {})
            for k, var in getattr(self, "recipe_setpoint_vars", {}).items():
                if k in sp:
                    var.set(str(sp.get(k, "")))

            self._populate_recipe_tree()
            self.recipe_status.config(text=f"Loaded: {path}  Total time: {self._seconds_to_hms(self.current_recipe.get('total_time_s',0))}")
        except Exception as e:
            messagebox.showerror("Load failed", str(e))
    def _open_queue_builder(self):
        win = tk.Toplevel(self)
        win.title("Recipe Queue Builder")
        win.geometry("700x350")

        left = ttk.Frame(win)
        left.pack(side="left", fill="both", expand=True, padx=10, pady=10)

        lb = tk.Listbox(left)
        lb.pack(fill="both", expand=True)

        for p in self.recipe_queue:
            lb.insert("end", p)

        btns = ttk.Frame(win)
        btns.pack(side="right", fill="y", padx=10, pady=10)

        def add_files():
            paths = filedialog.askopenfilenames(filetypes=[("Recipe JSON", "*.json")])
            if not paths:
                return
            for p in paths:
                self.recipe_queue.append(p)
                lb.insert("end", p)

        def remove_sel():
            sel = list(lb.curselection())
            sel.reverse()
            for idx in sel:
                self.recipe_queue.pop(idx)
                lb.delete(idx)

        def move(delta):
            sel = lb.curselection()
            if not sel:
                return
            i = sel[0]
            j = i + delta
            if j < 0 or j >= lb.size():
                return
            self.recipe_queue[i], self.recipe_queue[j] = self.recipe_queue[j], self.recipe_queue[i]
            txt = lb.get(i)
            lb.delete(i)
            lb.insert(j, txt)
            lb.selection_set(j)

        ttk.Button(btns, text="Add...", command=add_files).pack(fill="x", pady=2)
        ttk.Button(btns, text="Remove", command=remove_sel).pack(fill="x", pady=2)
        ttk.Button(btns, text="Move Up", command=lambda: move(-1)).pack(fill="x", pady=2)
        ttk.Button(btns, text="Move Down", command=lambda: move(1)).pack(fill="x", pady=2)

    def _populate_recipe_tree(self):
        for i in self.recipe_tree.get_children():
            self.recipe_tree.delete(i)
        if not self.current_recipe:
            return
        blocks = self.current_recipe["blocks"]
        for b_idx, blk in enumerate(blocks, start=1):
            cycles = blk["cycles"]
            for s_idx, step in enumerate(blk["steps"], start=1):
                self.recipe_tree.insert(
                    "",
                    "end",
                    values=(
                        b_idx,
                        cycles,
                        s_idx,
                        step["name"],
                        step["line"],
                        step["pulse_s"],
                        step["purge_s"],
                        step.get("sf_pulse_s", 0.0),
                        step.get("sf_hold_s", 0.0),
                    ),
                )

        self.recipe_status.config(text=f"Loaded recipe: {self.current_recipe.get('title','(unnamed)')}  Total time: {self._seconds_to_hms(self.current_recipe.get('total_time_s',0))}")

        # build indicators
        selected_lines = [(ln, nm) for ln, nm in self.current_recipe.get("line_names", {}).items()]
        selected_lines = [(int(k), v) for k, v in selected_lines]
        selected_lines.sort(key=lambda x: x[0])
        self._rebuild_line_indicators(selected_lines)

    # ----------------------------
    # Recipe wizard
    # ----------------------------

    def _open_recipe_wizard(self, prefill: bool = False):
        RecipeWizard(self, prefill=prefill)

    # ----------------------------
    # Runner
    # ----------------------------

    def _run_current_recipe(self):
        if not self.current_recipe:
            messagebox.showinfo("No recipe", "Create a recipe first.")
            return
        if self.run_state is not None:
            messagebox.showwarning("Already running", "A recipe is already running.")
            return
        self.run_abort_flag = False
        self._start_run(self.current_recipe)

    def _run_queue(self):
        if not self.recipe_queue:
            messagebox.showinfo("No queue", "Add recipes to the queue first.")
            return
        if self.run_state is not None:
            messagebox.showwarning("Already running", "A recipe is already running.")
            return

        try:
            import json
            recipes = []
            for p in self.recipe_queue:
                with open(p, "r", encoding="utf-8") as f:
                    rcp = json.load(f)
                if not isinstance(rcp, dict) or "blocks" not in rcp or "setpoints" not in rcp:
                    raise ValueError(f"Invalid recipe JSON: {p}")
                rcp["_source_path"] = p
                recipes.append(rcp)
        except Exception as e:
            messagebox.showerror("Queue load failed", str(e))
            return

        self._queue_run_list = recipes
        self._queue_run_active = True
        self._queue_run_index = 0
        self.run_abort_flag = False
        self._start_run(self._queue_run_list[0])


    def _start_run(self, recipe: dict):
        self.run_state = {
            "phase": "prechecks",
            "recipe": recipe,
            "phase_started": time.time(),
            "delay_remaining_s": int(float(recipe["setpoints"]["delay_min"]) * 60),
            "current_block": 0,
            "current_cycle": 0,
            "current_step": 0,
            "cycle_left_in_block": None,
            "step_active_until": None,
        }
        self.recipe_status.config(text="Run: starting pre-checks...")
        self._show_run_progress_popup()
        self.delay_clock_var.set("--:--")
        self.run_timing_var.set("Start: --   End: --")

        # Start hi-res run capture
        self._start_run_hires_capture(recipe)

        # estimate start/end once stable+delay begins; updated later
        self.after(100, self._run_tick)

    # ----------------------------
    # Run progress popup
    # ----------------------------
    def _show_run_progress_popup(self):
        """Create (or raise) a small popup showing what the run is waiting on / doing."""
        try:
            if getattr(self, "_run_progress_win", None) and self._run_progress_win.winfo_exists():
                self._run_progress_win.deiconify()
                self._run_progress_win.lift()
                return
        except Exception:
            pass

        win = tk.Toplevel(self)
        win.title("Recipe Run Progress")
        win.geometry("520x260")
        win.resizable(False, False)

        # If user closes the window, just hide it (run continues)
        def _hide():
            try:
                win.withdraw()
            except Exception:
                pass
        win.protocol("WM_DELETE_WINDOW", _hide)

        self._run_progress_win = win
        self._run_progress_vars = {
            "pt": tk.StringVar(value="1. Waiting for PT1 to be < 20 hPa: --"),
            "te1": tk.StringVar(value="2. Waiting for TE2/TE1 to be in range: --"),
            "te210": tk.StringVar(value="3. Waiting for TE210/TE211 to be in range: --"),
            "delay": tk.StringVar(value="4. Delay before executing pulses: --"),
            "cycles": tk.StringVar(value="5. Cycles remaining: --"),
            "step": tk.StringVar(value="6. Current step: --"),
        }

        frm = ttk.Frame(win, padding=10)
        frm.pack(fill="both", expand=True)

        for k in ("pt","te1","te210","delay","cycles","step"):
            ttk.Label(frm, textvariable=self._run_progress_vars[k]).pack(anchor="w", pady=2)

        ttk.Separator(frm).pack(fill="x", pady=(8, 6))

        btns = ttk.Frame(frm)
        btns.pack(fill="x")
        ttk.Button(btns, text="Hide", command=_hide).pack(side="right")
        ttk.Label(btns, text="(Run continues even if hidden)", foreground="gray").pack(side="left")

    def _close_run_progress_popup(self):
        try:
            if getattr(self, "_run_progress_win", None) and self._run_progress_win.winfo_exists():
                self._run_progress_win.destroy()
        except Exception:
            pass
        self._run_progress_win = None
        self._run_progress_vars = None

    def _update_run_progress_popup(self, *, pt1=None, te2=None, te210=None, phase=None):
        """Update popup strings from current run state/snapshot."""
        if not getattr(self, "_run_progress_vars", None):
            return
        sp = {}
        try:
            sp = self.run_state.get("recipe", {}).get("setpoints", {}) if self.run_state else {}
        except Exception:
            sp = {}

        # 1) Pressure gate (Delay doesn't begin until PT1<20; sequencing until PT1<15 is handled in code)
        if pt1 is None:
            pt_txt = "--"
        else:
            pt_txt = f"{pt1:.1f} hPa"
        self._run_progress_vars["pt"].set(f"1. Waiting for PT1 to be < 20 hPa: {pt_txt}")

        # 2) TE2 (target) within tolerance
        te2_tol = float(sp.get("TE2_tol", 1.0)) if sp else 1.0
        te2_tgt = float(sp.get("TE2", 0.0)) if sp else 0.0
        if te2 is None:
            te2_txt = "--"
        else:
            te2_txt = f"{te2:.1f}°C (target {te2_tgt:.1f}±{te2_tol:.1f})"
        self._run_progress_vars["te1"].set(f"2. Waiting for TE2 to be in range: {te2_txt}")

        # 3) TE210 (target) within tolerance
        te210_tol = float(sp.get("TE210_tol", 1.0)) if sp else 1.0
        te210_tgt = float(sp.get("TE210", 0.0)) if sp else 0.0
        if te210 is None:
            te210_txt = "--"
        else:
            te210_txt = f"{te210:.1f}°C (target {te210_tgt:.1f}±{te210_tol:.1f})"
        self._run_progress_vars["te210"].set(f"3. Waiting for TE210 to be in range: {te210_txt}")

        # 4) Delay countdown
        if not self.run_state:
            self._run_progress_vars["delay"].set("4. Delay before executing pulses: --")
        else:
            if self.run_state.get("phase") == "delay":
                rem = int(self.run_state.get("delay_remaining_s", 0))
                mm = rem // 60
                ss = rem % 60
                self._run_progress_vars["delay"].set(f"4. Delay before executing pulses: {mm:02d}:{ss:02d}")
            elif self.run_state.get("phase") in ("sequencing","step_begin","step_wait"):
                self._run_progress_vars["delay"].set("4. Delay before executing pulses: done")
            else:
                self._run_progress_vars["delay"].set("4. Delay before executing pulses: (not started)")

        # 5) Cycles remaining
        if self.run_state and self.run_state.get("phase") in ("sequencing","step_begin","step_wait"):
            cyc = self.run_state.get("cycle_left_in_block", "--")
            blk = self.run_state.get("current_block", 0)
            self._run_progress_vars["cycles"].set(f"5. Cycles remaining (block {blk+1}): {cyc}")
        else:
            self._run_progress_vars["cycles"].set("5. Cycles remaining: --")

        # 6) Current step / action
        if self.run_state and self.run_state.get("phase") in ("sequencing","step_begin","step_wait"):
            line = self.run_state.get("current_line", None)
            action = self.run_state.get("current_action", None)
            blk = self.run_state.get("current_block", 0)
            si = self.run_state.get("current_step", 0)
            if line is None:
                self._run_progress_vars["step"].set(f"6. Current step: block {blk+1}, step {si+1}")
            else:
                self._run_progress_vars["step"].set(f"6. Current step: Line {line} — {action or '...'} (block {blk+1}, step {si+1})")
        else:
            self._run_progress_vars["step"].set("6. Current step: --")


    def _abort_run(self):
        self.run_abort_flag = True
        if self.run_state is None:
            return
        self.recipe_status.config(text="Run: aborting...")
        self._stop_run_hires_capture()
        # Immediate: close line valves; leave V2 and V6 open; set temps to 20°C
        self._safe_state_abort()
        self.run_state = None
        self.recipe_status.config(text="Aborted.")
        self._close_run_progress_popup()
        self.delay_clock_var.set("--:--")


    def _set_pump_valves_by_pressure(self, pt_chamber_hpa: float):
        """Set V1/V2 based on chamber pressure (PT1).

        Rule:
          - If PT1 > 100 hPa: V2 OPEN, V1 CLOSED
          - If PT1 <= 100 hPa: V2 CLOSED, V1 OPEN
        """
        if not self.online or not self.fins:
            return
        try:
            pt = float(pt_chamber_hpa)
        except Exception:
            pt = 9999.0

        if pt > 100.0:
            self._toggle_valve("V1", False)
            self._toggle_valve("V2", True)
        else:
            self._toggle_valve("V2", False)
            self._toggle_valve("V1", True)

    def _safe_state_abort(self):
        if not self.online or not self.fins:
            return
        # Close line valves
        for v in ("V110", "V210", "V211", "V310", "V420", "V422"):
            try:
                self._toggle_valve(v, False)
            except Exception:
                pass
        # Set V1/V2 according to chamber-pressure interlock; ensure V6 OPEN
        try:
            pt1 = float(self.snapshot_data.get("PT1", 9999.0))
        except Exception:
            pt1 = 9999.0
        try:
            self._set_pump_valves_by_pressure(pt1)
            self._toggle_valve("V6", True)
        except Exception:
            pass

        # Temps to 20 C
        for tag in ("TE2", "TE210"):
            area, addr = SETPOINT_COMMANDS[tag]
            self.fins.write_word(addr, 200, area=area)  # 20.0 °C -> 200 (0.1 °C units)
        # Note: TE1/TE211 are maxima; leave untouched.

    def _run_tick(self):
        if self.run_state is None:
            return

        if self.run_abort_flag:
            self._abort_run()
            return

        recipe = self.run_state["recipe"]
        phase = self.run_state["phase"]

        # convenience reads
        pt1 = float(self.snapshot_data.get("PT1", 9999.0)) if getattr(self, "snapshot_data", None) else 9999.0
        te2 = float(self.snapshot_data.get("TE2", -999.0)) if getattr(self, "snapshot_data", None) else -999.0
        te210 = float(self.snapshot_data.get("TE210", -999.0)) if getattr(self, "snapshot_data", None) else -999.0

        # Update run-progress popup
        try:
            self._update_run_progress_popup(pt1=pt1, te2=te2, te210=te210, phase=phase)
        except Exception:
            pass
        # PT safety during pulsing
        if phase in ("sequencing", "step_pulse", "step_purge", "step_sf") and pt1 > 20.0:
            self.recipe_status.config(text=f"ERROR: PT1 rose above 20 hPa (PT1={pt1:.1f}). Halting pulsing.")
            self._safe_state_abort()
            self.run_state = None
            return

        # Lid closed check (inferred)
        lid_open = self._lid_is_open_inferred()

        # --- State machine ---
        if phase == "prechecks":
            # PT1 must be < 100 before any temperature settings are applied
            if pt1 >= 100.0:
                self.recipe_status.config(text=f"Waiting: PT1 must be < 100 hPa before heating. (PT1={pt1:.1f})")
                # Ensure pumpdown: V6+V2 until PT1<100, then V1
                self._ensure_pumpdown_original(pt1)
                self.after(250, self._run_tick)
                return

            # Lid must be closed; if open, prompt
            if lid_open:
                if messagebox.askokcancel("Lid open", "Cannot execute recipe until lid is closed. Close lid now?"):
                    self._close_lid()
                    # Start pumpdown assist
                    self._ensure_pumpdown_original(pt1)
                    self.after(250, self._run_tick)
                    return
                else:
                    self.recipe_status.config(text="Run cancelled (lid open).")
                    self.run_state = None
                    return

            # OK: apply flows
            self.run_state["phase"] = "apply_flows"
            self.recipe_status.config(text="Applying MFC setpoints...")
            self.after(100, self._run_tick)
            return

        if phase == "apply_flows":
            self._apply_mfc_setpoints(recipe)
            self.run_state["phase"] = "pumpdown"
            self.recipe_status.config(text="Pumpdown: pulling vacuum...")
            self.after(250, self._run_tick)
            return

        if phase == "pumpdown":
            self._ensure_pumpdown_original(pt1)
            if pt1 < 90.0:
                self.run_state["phase"] = "apply_temps"
                self.recipe_status.config(text="Applying temperature setpoints...")
            self.after(250, self._run_tick)
            return

        if phase == "apply_temps":
            self._apply_temp_setpoints(recipe)
            self.run_state["phase"] = "wait_stable"
            self.recipe_status.config(text="Waiting for temps within tolerance and PT1 < 15...")
            self.after(500, self._run_tick)
            return

        if phase == "wait_stable":
            sp = recipe["setpoints"]
            te2_tol = float(sp.get("TE2_tol", 1.0))
            te210_tol = float(sp.get("TE210_tol", 1.0))
            te2_ok = abs(te2 - float(sp["TE2"])) <= te2_tol
            te210_ok = abs(te210 - float(sp["TE210"])) <= te210_tol

            if (pt1 < 15.0) and te2_ok and te210_ok:
                # Begin stabilization delay countdown (if any)
                self.run_state["phase"] = "delay"
                self.run_state["delay_started"] = time.time()
                self.recipe_status.config(text="Stabilization OK. Starting delay countdown...")
                # compute start/end estimates
                start_time = datetime.now()
                total = recipe.get("total_time_s", 0.0)
                delay_s = self.run_state["delay_remaining_s"]
                end_time = start_time + timedelta(seconds=(delay_s + total))
                self.run_timing_var.set(f"Start: {start_time.strftime('%H:%M:%S')}   End: {end_time.strftime('%H:%M:%S')}")
                self.after(200, self._run_tick)
                return

            self.recipe_status.config(text=f"Waiting: PT1={pt1:.1f} hPa; TE2={te2:.1f}°C; TE210={te210:.1f}°C")
            self.after(500, self._run_tick)
            return

        if phase == "delay":
            rem = int(self.run_state.get("delay_remaining_s", 0) or 0)
            self.run_state["delay_remaining_s"] = rem
            if rem <= 0:
                self.delay_clock_var.set("00:00")
                self.run_state["phase"] = "sequencing"
                self.recipe_status.config(text="Executing recipe...")
                self.run_state["current_block"] = 0
                self.run_state["current_cycle"] = 0
                self.run_state["current_step"] = 0
                self.run_state["cycle_left_in_block"] = None
                self.after(100, self._run_tick)
                return

            # tick down once per second
            mins = rem // 60
            secs = rem % 60
            self.delay_clock_var.set(f"{mins:02d}:{secs:02d}")
            self.run_state["delay_remaining_s"] = rem - 1
            self.after(1000, self._run_tick)
            return

        if phase == "sequencing":
            # Initialize first block/cycle/step
            self.run_state["phase"] = "step_begin"
            self.after(50, self._run_tick)
            return

        if phase == "step_begin":
            blocks = recipe["blocks"]
            b = self.run_state["current_block"]
            if b >= len(blocks):
                # Done
                self._finish_run_success(recipe)
                return

            blk = blocks[b]
            cycles = int(blk["cycles"])
            # Initialize cycle counters if starting block
            if self.run_state.get("cycle_left_in_block") is None:
                self.run_state["cycle_left_in_block"] = cycles

            # If cycles finished, advance block
            if self.run_state["cycle_left_in_block"] <= 0:
                self.run_state["current_block"] += 1
                self.run_state["current_step"] = 0
                self.run_state["cycle_left_in_block"] = None
                self.after(10, self._run_tick)
                return

            steps = blk["steps"]
            s = self.run_state["current_step"]
            if s >= len(steps):
                # end of cycle: decrement and restart steps
                self.run_state["cycle_left_in_block"] -= 1
                self.run_state["current_step"] = 0
                self.after(10, self._run_tick)
                return

            step = steps[s]
            # execute one step (pulse / stopflow / purge)
            self._execute_step(step)
            # schedule next step after duration
            dur = self._step_total_duration(step)
            self.run_state["step_active_until"] = time.time() + dur
            self.run_state["phase"] = "step_wait"
            self.after(50, self._run_tick)
            return

        if phase == "step_wait":
            if time.time() >= self.run_state["step_active_until"]:
                self._set_line_indicator(int(self.current_step_line), False)
                self.run_state["current_step"] += 1
                self.run_state["phase"] = "step_begin"
            self.after(50, self._run_tick)
            return

        # fallback
        self.after(200, self._run_tick)

    def _ensure_pumpdown_original(self, pt1: float):
        """Pumpdown / recipe vacuum state.

        We treat PT1 as chamber pressure.

        Behavior must match the Evac final state rule:
          - If PT1 > 100 hPa: V2 OPEN, V1 CLOSED (gentler initial pump path)
          - If PT1 <= 100 hPa: V2 CLOSED, V1 OPEN (final/steady evac state)

        V6 is forced OPEN in both cases.
        """
        if not self.online or not self.fins:
            return

        # Use the centralized interlock so behavior stays consistent across Run/Abort/Evac.
        try:
            self._set_pump_valves_by_pressure(pt1)
            self._toggle_valve("V6", True)
        except Exception:
            pass

    def _apply_mfc_setpoints(self, recipe: dict):
        sp = recipe["setpoints"]
        for tag in ("MFC1", "MFC100", "MFC200", "MFC300", "MFC400"):
            area, addr = SETPOINT_COMMANDS[tag]
            try:
                val = int(float(sp[tag]))
            except Exception:
                continue
            self.fins.write_word(addr, val, area=area)

    def _apply_temp_setpoints(self, recipe: dict):
        sp = recipe["setpoints"]
        # TE setpoints are in °C in the UI, but PLC expects 0.1 °C units (and snapshot parsing divides by 10).
        for tag in ("TE1", "TE2", "TE210", "TE211"):
            area, addr = SETPOINT_COMMANDS[tag]
            try:
                val_c = float(sp[tag])
                val_raw = int(round(val_c * 10.0))
            except Exception:
                continue
            self.fins.write_word(addr, val_raw, area=area)

    def _finish_run_success(self, recipe: dict):
        """At end: set chamber to post-deposition temperature, keep vacuum and flows."""
        self.recipe_status.config(text="Run complete. Setting post-deposition temperature...")
        # (hi-res capture stopped at end of run / end of queue)

        # --- NEW: ensure popup shows a clean terminal state ---
        if self.run_state is None:
            self.run_state = {}  # defensive; should not happen, but avoids key errors below

        # Clear any stale "current step" info so the popup doesn't keep showing "purge"
        self.run_state["current_line"] = None
        self.run_state["current_action"] = None
        self.run_state["phase"] = "complete"
        self.run_state["cycle_left_in_block"] = 0
        self.run_state["current_block"] = self.run_state.get("current_block", 0)
        self.run_state["current_step"] = self.run_state.get("current_step", 0)

        # If you have the popup open, push a final message into it
        try:
            self._update_run_progress_popup(
                f"Post-recipe setpoints engaged, recipe complete",
                f"Delay before executing pulses: done",
                f"Cycles remaining (block {self.run_state.get('current_block', 0)+1}): 0",
                f"Post-recipe setpoints engaged, recipe complete"
            )
        except Exception:
            pass
        # --- END NEW ---

        try:
            post_c = float(recipe["setpoints"]["post_TE2"])
            post_raw = int(round(post_c * 10.0))
            area, addr = SETPOINT_COMMANDS["TE2"]
            self.fins.write_word(addr, post_raw, area=area)
        except Exception:
            pass

        try:
            post_c210 = float(recipe["setpoints"].get("post_TE210", recipe["setpoints"]["TE210"]))
            post_raw210 = int(round(post_c210 * 10.0))
            area210, addr210 = SETPOINT_COMMANDS["TE210"]
            self.fins.write_word(addr210, post_raw210, area=area210)
        except Exception:
            pass



        # --- NEW: close out run state AFTER final popup update ---
        # Force the progress popup to show a clean terminal state (no more ticks after run_state=None)
        try:
            if hasattr(self, "_run_progress_vars") and self._run_progress_vars:
                # These keys should match what you set when creating the popup
                self._run_progress_vars["delay"].set("4. Delay before executing pulses: done")
                self._run_progress_vars["cycles"].set("5. Cycles remaining: 0")
                self._run_progress_vars["step"].set("6. Current step: Post-recipe setpoints engaged, recipe complete")
        except Exception:
            pass

        # Optional: make the main status match
        self.recipe_status.config(text="Post-recipe setpoints engaged, recipe complete")

        # Stop hi-res capture at end of a single-recipe run.
        if not (hasattr(self, "_queue_run_list") and self._queue_run_list):
            self._stop_run_hires_capture()

        self.run_state = None
        # Clear delay clock display
        self.delay_clock_var.set("--:--")

        # If this run was part of a queue, advance
        if hasattr(self, "_queue_run_list") and self._queue_run_list:
            self._queue_run_index += 1
            if self._queue_run_index < len(self._queue_run_list):
                self._start_run(self._queue_run_list[self._queue_run_index])
            else:
                self._queue_run_list = []
                self._stop_run_hires_capture()
                self._queue_run_active = False
                self.recipe_status.config(text="Queue complete.")
    def _close_all_line_valves(self):
        for v in ("V110", "V210", "V211", "V310", "V420", "V422"):
            self._toggle_valve(v, False)

    def _execute_step(self, step: dict):
        """Fire a step asynchronously using Tk timers."""
        self._close_all_line_valves()
        line = int(step["line"])
        self.current_step_line = line
        if self.run_state is not None:
            self.run_state["current_line"] = line
            self.run_state["current_action"] = "begin"

        # Determine durations
        pulse_s = float(step.get("pulse_s", 0.0))
        purge_s = float(step.get("purge_s", 0.0))
        sf_pulse_s = float(step.get("sf_pulse_s", 0.0))
        sf_hold_s = float(step.get("sf_hold_s", 0.0))

        has_stopflow = (sf_hold_s > 0.0) and (sf_pulse_s > 0.0)
        def open_line():
            if self.run_state is not None:
                # If this is the start of a step, mark action as 'pulse'.
                if self.run_state.get("current_action") in (None, "begin"):
                    self.run_state["current_action"] = "pulse"
            self._set_line_indicator(line, True)
            if line == 1:
                self._toggle_valve("V110", True)
            elif line == 2:
                # Line 2 pulse = open BOTH valves together, and close BOTH together.
                self._toggle_valve("V210", True)
                self._toggle_valve("V211", True)

            elif line == 3:
                self._toggle_valve("V310", True)
            elif line == 4:
                self._toggle_valve("V420", True)
                self._toggle_valve("V422", True)

        def close_line():
            if self.run_state is not None and self.run_state.get("current_action") == "pulse":
                self.run_state["current_action"] = "purge"
            self._close_all_line_valves()
            self._set_line_indicator(line, False)
        # Ensure recipe vacuum valves during sequencing (Evac-final behavior)
        try:
            pt1_now = float(self.snapshot_data.get("PT1", 9999.0))
        except Exception:
            pt1_now = 9999.0
        self._ensure_pumpdown_original(pt1_now)
        if has_stopflow:
            # Stop Flow sequence:
            if self.run_state is not None:
                self.run_state["current_action"] = "SF pulse"
            # 1) close V6
            self._toggle_valve("V6", False)
            # 2) pulse line for sf_pulse_s (line delivery happens while V6 closed)
            open_line()
            self.after(int(sf_pulse_s * 1000), close_line)
            # 3) keep V6 closed until sf_hold_s elapsed, then open V6 and purge
            def end_hold_then_purge():
                self._toggle_valve("V6", True)
                if self.run_state is not None:
                    self.run_state["current_action"] = "purge"
                # purge: all line valves closed; wait purge_s
                # nothing else required; flows already set
                # (We already have all lines closed)
                pass
            self.after(int(sf_hold_s * 1000), end_hold_then_purge)
            # purge will simply be time passage (runner waits total duration)
        else:
            # Normal pulse
            if pulse_s > 0:
                open_line()
                # close at end of pulse (for line2, V211 may have opened earlier; still close both)
                self.after(int(pulse_s * 1000), close_line)
            # purge is time passage

    def _step_total_duration(self, step: dict) -> float:
        pulse_s = float(step.get("pulse_s", 0.0))
        purge_s = float(step.get("purge_s", 0.0))
        sf_pulse_s = float(step.get("sf_pulse_s", 0.0))
        sf_hold_s = float(step.get("sf_hold_s", 0.0))

        if sf_hold_s > 0.0 and sf_pulse_s > 0.0:
            return sf_hold_s + purge_s
        # normal: pulse + purge
        return pulse_s + purge_s


class RecipeWizard(tk.Toplevel):
    """Wizard-style popup for creating/editing a recipe."""

    def __init__(self, app: PicoSimApp, prefill: bool = False):
        super().__init__(app)
        self.app = app
        self.title("Create Recipe")
        self.geometry("820x520")
        self.resizable(False, False)

        self.data = {
            "selected_lines": [],
            "line_names": {},
            "variants": {},        # variant_id -> dict(line, pulse_s, purge_s)
            "blocks": [],          # list of {steps:[variant_id...], cycles:int}
            "stopflow_enabled": False,
            "setpoints": {},
            "per_step_sf": [],     # expanded sf settings parallel to expanded steps
        }

        if prefill and app.current_recipe:
            try:
                self.data["selected_lines"] = [int(k) for k in app.current_recipe["line_names"].keys()]
                self.data["line_names"] = {str(k): v for k, v in app.current_recipe["line_names"].items()}
                self.data["setpoints"] = dict(app.current_recipe["setpoints"])
                # Variants and blocks are not perfectly reconstructable from expanded steps, so we start from scratch there.
            except Exception:
                pass

        # wizard navigation
        nav = ttk.Frame(self)
        nav.pack(fill="x", padx=10, pady=10)

        ttk.Button(nav, text="Start Over", command=self._start_over).pack(side="left")
        self.back_btn = ttk.Button(nav, text="Back", command=self._back)
        self.back_btn.pack(side="left", padx=(10, 0))
        self.next_btn = ttk.Button(nav, text="Next", command=self._next)
        self.next_btn.pack(side="left", padx=4)

        self.page_container = ttk.Frame(self)
        self.page_container.pack(fill="both", expand=True, padx=10, pady=10)

        self.pages = []
        self.page_index = 0

        self._build_pages()
        self._show_page(0)

    def _start_over(self):
        self.data = {
            "selected_lines": [],
            "line_names": {},
            "variants": {},
            "blocks": [],
            "stopflow_enabled": False,
            "setpoints": {},
        }
        self.page_index = 0
        self._build_pages()
        self._show_page(0)

    def _build_pages(self):
        for w in self.page_container.winfo_children():
            w.destroy()
        self.pages = []

        self.pages.append(self._page_delay())
        self.pages.append(self._page_choose_lines())
        self.pages.append(self._page_names())
        self.pages.append(self._page_define_variants())
        self.pages.append(self._page_build_blocks())
        self.pages.append(self._page_stopflow_times())
        self.pages.append(self._page_setpoints())
        self.pages.append(self._page_confirm())

    def _show_page(self, idx: int):
        # Hide current page
        for w in self.page_container.winfo_children():
            w.pack_forget()

        # Rebuild pages that depend on earlier choices (names/selected lines/blocks)
        if idx == 3:
            # Define precursor delivery settings depends on selected lines + names
            try:
                self.pages[3].destroy()
            except Exception:
                pass
            self.pages[3] = self._page_define_variants()

        if idx == 4:
            # Block builder depends on variants + any prior block edits
            try:
                self.pages[4].destroy()
            except Exception:
                pass
            self.pages[4] = self._page_build_blocks()

        if idx == 5:
            # Stop Flow timings depend on the expanded step list (blocks)
            try:
                self.pages[5].destroy()
            except Exception:
                pass
            self.pages[5] = self._page_stopflow_times()

        self.page_index = idx

        self.pages[idx].pack(fill="both", expand=True)

        # Populate confirmation summary on entry
        if idx == 7:
            try:
                recipe = self._build_final_recipe(only_build=True)
                self.summary_txt.delete("1.0", "end")
                self.summary_txt.insert("end", recipe.get("summary", ""))
            except Exception:
                pass

        self.back_btn.state(["!disabled"] if idx > 0 else ["disabled"])
        self.next_btn.config(text="Finish" if idx == (len(self.pages) - 1) else "Next")

    def _back(self):
        if self.page_index > 0:
            self._show_page(self.page_index - 1)

    def _next(self):
        # Validate current page
        if not self._validate_page(self.page_index):
            return

        if self.page_index == (len(self.pages) - 1):
            # Finish: commit to app
            recipe = self._build_final_recipe()
            self.app.current_recipe = recipe
            self.app._populate_recipe_tree()
            self.destroy()
            return

        self._show_page(self.page_index + 1)


    def _sanitize_name(self, name: str) -> str:
        """Make a stable identifier from a user-facing precursor name."""
        name = (name or "").strip()
        if not name:
            return "Line"
        # Replace whitespace with underscores and drop unsafe chars
        name = re.sub(r"\s+", "_", name)
        name = re.sub(r"[^A-Za-z0-9_\-]", "", name)
        return name or "Line"

    def _variant_prefix_for_line(self, line_no: int) -> str:
        # Prefer user-provided precursor name, else fallback to Line{n}
        nm = self.data.get("line_names", {}).get(str(line_no), f"Line{line_no}")
        return self._sanitize_name(nm)

    def _make_variant_id(self, line_no: int, suffix: int) -> str:
        return f"{self._variant_prefix_for_line(line_no)}.{suffix}"
    # ----- Pages -----

    def _page_delay(self):
        f = ttk.Frame(self.page_container)
        ttk.Label(f, text="Delay before executing the run (minutes, 0 for none):", font=("Arial", 12, "bold")).pack(anchor="w", pady=(0, 10))
        self.delay_var = tk.StringVar(value=str(self.app.recipe_setpoint_vars["delay_min"].get()))
        ttk.Entry(f, textvariable=self.delay_var, width=10).pack(anchor="w")
        return f

    def _page_choose_lines(self):
        f = ttk.Frame(self.page_container)
        ttk.Label(f, text="Which precursor lines do you want to use? (1-4)", font=("Arial", 12, "bold")).pack(anchor="w", pady=(0, 10))

        self.line_vars = {i: tk.BooleanVar(value=(i in self.data["selected_lines"])) for i in range(1, 5)}
        for i in range(1, 5):
            ttk.Checkbutton(f, text=f"Line {i}", variable=self.line_vars[i]).pack(anchor="w")
        return f

    def _page_names(self):
        f = ttk.Frame(self.page_container)
        ttk.Label(f, text="Enter precursor names for each selected line:", font=("Arial", 12, "bold")).pack(anchor="w", pady=(0, 10))

        self.name_vars = {}
        for i in range(1, 5):
            row = ttk.Frame(f)
            row.pack(fill="x", pady=4)
            ttk.Label(row, text=f"Line {i}").pack(side="left", padx=(0, 10))
            v = tk.StringVar(value=self.data["line_names"].get(str(i), ""))
            self.name_vars[i] = v
            e = ttk.Entry(row, textvariable=v, width=40)
            e.pack(side="left")
        return f

    def _page_stopflow_enable(self):
        f = ttk.Frame(self.page_container)
        ttk.Label(f, text="Do you want a Stop Flow step option?", font=("Arial", 12, "bold")).pack(anchor="w", pady=(0, 10))
        self.stopflow_enabled_var = tk.BooleanVar(value=bool(self.data.get("stopflow_enabled", False)))
        ttk.Radiobutton(f, text="Yes", variable=self.stopflow_enabled_var, value=True).pack(anchor="w")
        ttk.Radiobutton(f, text="No", variable=self.stopflow_enabled_var, value=False).pack(anchor="w")
        return f

    def _page_define_variants(self):
        f = ttk.Frame(self.page_container)
        ttk.Label(f, text="Define precursor delivery settings:", font=("Arial", 12, "bold")).pack(anchor="w", pady=(0, 10))

        self.variant_rows = []
        # Build a small table where user can add variants per line
        table = ttk.Frame(f)
        table.pack(fill="both", expand=True)

        headers = ["Variant", "Line", "Pulse (s)", "Purge (s)"]
        for c, h in enumerate(headers):
            ttk.Label(table, text=h, font=("Arial", 10, "bold")).grid(row=0, column=c, padx=6, pady=4, sticky="w")

        self.variants_container = table
        self._refresh_variant_table()
        ttk.Button(f, text="Add Variant", command=self._add_variant_row).pack(anchor="w", pady=(10, 0))
        return f


    def _refresh_variant_table(self):
        # Clear existing rows (but keep header row)
        for w in self.variants_container.grid_slaves():
            if int(w.grid_info()["row"]) != 0:
                w.destroy()

        self.variant_rows = []
        selected = [int(x) for x in self.data.get("selected_lines", [])]

        # Seed: one default setting per selected line if none exist yet
        if not self.data["variants"]:
            for ln in selected:
                vid = self._make_variant_id(ln, 1)
                self.data["variants"][vid] = {"line": ln, "pulse_s": 0.1, "purge_s": 0.1}

        r = 1
        for vid, vdat in sorted(self.data["variants"].items()):
            ln = int(vdat.get("line", 0))
            pulse_var = tk.StringVar(value=str(vdat.get("pulse_s", 0.1)))
            purge_var = tk.StringVar(value=str(vdat.get("purge_s", 0.1)))

            ttk.Label(self.variants_container, text=vid).grid(row=r, column=0, padx=6, pady=2, sticky="w")
            # Variant belongs to a specific line; show as read-only
            ttk.Label(self.variants_container, text=str(ln)).grid(row=r, column=1, padx=6, pady=2, sticky="w")

            e1 = ttk.Entry(self.variants_container, textvariable=pulse_var, width=10)
            e1.grid(row=r, column=2, padx=6, pady=2, sticky="w")
            e2 = ttk.Entry(self.variants_container, textvariable=purge_var, width=10)
            e2.grid(row=r, column=3, padx=6, pady=2, sticky="w")

            self.variant_rows.append((vid, ln, pulse_var, purge_var))
            r += 1


    def _add_variant_row(self):
        selected = [int(x) for x in self.data.get("selected_lines", [])]
        if not selected:
            messagebox.showwarning("No lines", "Select precursor lines first.")
            return

        # Prompt the user to choose which precursor line this variant belongs to.
        ln = selected[0]
        if len(selected) > 1:
            dlg = tk.Toplevel(self)
            dlg.title("Choose precursor line")
            dlg.resizable(False, False)
            dlg.transient(self)
            dlg.grab_set()

            ttk.Label(dlg, text="Add settings for which precursor line?", font=("Arial", 10, "bold")).pack(anchor="w", padx=10, pady=(10, 6))
            choice = tk.IntVar(value=ln)

            for n in selected:
                nm = self.data.get("line_names", {}).get(str(n), f"Line {n}")
                ttk.Radiobutton(dlg, text=f"Line {n}: {nm}", variable=choice, value=n).pack(anchor="w", padx=20)

            btns = ttk.Frame(dlg)
            btns.pack(fill="x", padx=10, pady=10)

            def ok():
                dlg.destroy()

            ttk.Button(btns, text="OK", command=ok).pack(side="left")
            ttk.Button(btns, text="Cancel", command=lambda: (choice.set(-1), dlg.destroy())).pack(side="right")

            self.wait_window(dlg)
            if int(choice.get()) not in selected:
                return
            ln = int(choice.get())

        # Find next suffix for this line, based on IDs of the form "<precursor>.<n>"
        prefix = self._variant_prefix_for_line(ln)
        max_suffix = 0
        for vid, vdat in self.data.get("variants", {}).items():
            if int(vdat.get("line", -1)) != ln:
                continue
            m = re.match(rf"^{re.escape(prefix)}\.(\d+)$", str(vid))
            if m:
                max_suffix = max(max_suffix, int(m.group(1)))

        suffix = max_suffix + 1
        vid = self._make_variant_id(ln, suffix)
        while vid in self.data["variants"]:
            suffix += 1
            vid = self._make_variant_id(ln, suffix)

        self.data["variants"][vid] = {"line": ln, "pulse_s": 0.1, "purge_s": 0.1}
        self._refresh_variant_table()

    def _page_build_blocks(self):
        f = ttk.Frame(self.page_container)
        ttk.Label(f, text="Build your recipe as blocks of steps with a cycle count:", font=("Arial", 12, "bold")).pack(anchor="w", pady=(0, 10))

        body = ttk.Frame(f)
        body.pack(fill="both", expand=True)

        self.block_tree = ttk.Treeview(body, columns=("Block", "Cycles", "Steps"), show="headings", height=10)
        for c in ("Block", "Cycles", "Steps"):
            self.block_tree.heading(c, text=c)
        self.block_tree.column("Block", width=60, anchor="center")
        self.block_tree.column("Cycles", width=80, anchor="center")
        self.block_tree.column("Steps", width=520, anchor="w")
        self.block_tree.pack(side="left", fill="both", expand=True)

        ysb = ttk.Scrollbar(body, orient="vertical", command=self.block_tree.yview)
        self.block_tree.configure(yscroll=ysb.set)
        ysb.pack(side="right", fill="y")

        btns = ttk.Frame(f)
        btns.pack(fill="x", pady=10)

        ttk.Button(btns, text="Add Step", command=self._add_step_to_current_block).pack(side="left", padx=4)
        ttk.Button(btns, text="Set Cycles", command=self._set_cycles_for_current_block).pack(side="left", padx=4)
        ttk.Button(btns, text="New Block", command=self._new_block).pack(side="left", padx=4)
        ttk.Button(btns, text="Remove Block", command=self._remove_block).pack(side="left", padx=4)

        self._refresh_block_tree()
        return f

    def _ensure_at_least_one_block(self):
        if not self.data["blocks"]:
            self.data["blocks"].append({"steps": [], "cycles": 0})

    def _refresh_block_tree(self):
        self._ensure_at_least_one_block()
        for i in self.block_tree.get_children():
            self.block_tree.delete(i)
        for idx, blk in enumerate(self.data["blocks"], start=1):
            steps_txt = ", ".join(blk["steps"]) if blk["steps"] else "(none)"
            self.block_tree.insert("", "end", values=(idx, blk["cycles"] if blk["cycles"] else "", steps_txt))

    
    def _add_step_to_current_block(self):
        self._sync_variants_from_table()
        vids = sorted(self.data["variants"].keys())
        if not vids:
            messagebox.showwarning("No settings", "Define at least one precursor delivery setting first.")
            return

        dlg = tk.Toplevel(self)
        dlg.title("Add Step")
        dlg.resizable(False, False)
        dlg.transient(self)
        dlg.grab_set()

        ttk.Label(dlg, text="Select a precursor delivery setting to add:", font=("Arial", 10, "bold")).pack(anchor="w", padx=10, pady=(10, 6))

        body = ttk.Frame(dlg)
        body.pack(fill="both", expand=True, padx=10)

        lb = tk.Listbox(body, height=10, width=60)
        sb = ttk.Scrollbar(body, orient="vertical", command=lb.yview)
        lb.configure(yscrollcommand=sb.set)
        lb.pack(side="left", fill="both", expand=True)
        sb.pack(side="left", fill="y")

        # Show friendly text: id + (Line n, pulse/purge)
        for vid in vids:
            v = self.data["variants"][vid]
            line = v["line"]
            pulse = v.get("pulse_s", 0)
            purge = v.get("purge_s", 0)
            lb.insert("end", f"{vid}   (Line {line}: pulse={pulse}s, purge={purge}s)")

        btns = ttk.Frame(dlg)
        btns.pack(fill="x", padx=10, pady=10)

        def add_selected():
            sel = lb.curselection()
            if not sel:
                return
            vid = vids[int(sel[0])]
            self.data["blocks"][-1]["steps"].append(vid)
            self._refresh_block_tree()
            dlg.destroy()

        ttk.Button(btns, text="Add", command=add_selected).pack(side="left")
        ttk.Button(btns, text="Cancel", command=dlg.destroy).pack(side="right")

        lb.bind("<Double-1>", lambda _e: add_selected())
    def _set_cycles_for_current_block(self):
        val = simpledialog.askinteger("Cycles", "Enter cycles for this block (>=1):", minvalue=1)
        if not val:
            return
        self.data["blocks"][-1]["cycles"] = int(val)
        self._refresh_block_tree()

    def _new_block(self):
        # require current block to have steps and cycles
        cur = self.data["blocks"][-1]
        if not cur["steps"] or int(cur.get("cycles", 0)) <= 0:
            messagebox.showwarning("Incomplete block", "Current block must have at least one step and a cycle count before creating a new block.")
            return
        self.data["blocks"].append({"steps": [], "cycles": 0})
        self._refresh_block_tree()

    def _remove_block(self):
        if len(self.data["blocks"]) <= 1:
            self.data["blocks"][0] = {"steps": [], "cycles": 0}
        else:
            self.data["blocks"].pop()
        self._refresh_block_tree()


    def _page_stopflow_times(self):
        f = ttk.Frame(self.page_container)
        ttk.Label(
            f,
            text="Stop Flow hold time per step (0 disables). Hold must be > pulse.",
            font=("Arial", 11, "bold")
        ).pack(anchor="w", pady=(0, 10))

        self.sf_table = ttk.Frame(f)
        self.sf_table.pack(fill="both", expand=True)

        headers = ["#", "Variant", "Line", "Pulse (s)", "SF Hold (s)"]
        for c, h in enumerate(headers):
            ttk.Label(self.sf_table, text=h, font=("Arial", 10, "bold")).grid(row=0, column=c, padx=6, pady=4, sticky="w")

        self.sf_rows = []
        self._refresh_sf_table()
        return f

    def _refresh_sf_table(self):
        # clear existing rows except header
        for w in self.sf_table.grid_slaves():
            if int(w.grid_info()["row"]) != 0:
                w.destroy()

        self.sf_rows = []
        steps = self._expanded_steps_for_sf()
        r = 1
        for idx, (vid, vdat) in enumerate(steps, start=1):
            line = int(vdat.get("line", 0))
            pulse_total = float(vdat.get("pulse_s", 0.0))

            ttk.Label(self.sf_table, text=str(idx)).grid(row=r, column=0, padx=6, pady=2, sticky="w")
            ttk.Label(self.sf_table, text=vid).grid(row=r, column=1, padx=6, pady=2, sticky="w")
            ttk.Label(self.sf_table, text=str(line)).grid(row=r, column=2, padx=6, pady=2, sticky="w")
            ttk.Label(self.sf_table, text=f"{pulse_total:.3g}").grid(row=r, column=3, padx=6, pady=2, sticky="w")

            hv = tk.StringVar(value="0")
            ttk.Entry(self.sf_table, textvariable=hv, width=12).grid(row=r, column=4, padx=6, pady=2, sticky="w")
            self.sf_rows.append((vid, vdat, hv))
            r += 1

    def _expanded_steps_for_sf(self):
        self._sync_variants_from_table()
        expanded = []
        for blk in self.data["blocks"]:
            for vid in blk["steps"]:
                expanded.append((vid, self.data["variants"][vid]))
        return expanded

    def _page_setpoints(self):
        f = ttk.Frame(self.page_container)
        ttk.Label(f, text="Enter temperature and flow setpoints:", font=("Arial", 12, "bold")).pack(anchor="w", pady=(0, 10))

        form = ttk.Frame(f)
        form.pack(anchor="w")

        def row(lbl, default):
            fr = ttk.Frame(form)
            fr.pack(anchor="w", pady=3)
            ttk.Label(fr, text=lbl, width=28).pack(side="left")
            v = tk.StringVar(value=str(default))
            ttk.Entry(fr, textvariable=v, width=10).pack(side="left")
            return v

        self.sp_TE1 = row("TE1 max (<=400°C)", self.app.recipe_setpoint_vars["TE1"].get())
        self.sp_TE2 = row("TE2 target", self.app.recipe_setpoint_vars["TE2"].get())
        self.sp_TE2_tol = row("TE2 tolerance", self.app.recipe_setpoint_vars["TE2_tol"].get())
        self.sp_TE211 = row("TE211 max (<=190°C)", self.app.recipe_setpoint_vars["TE211"].get())
        self.sp_TE210 = row("TE210 target", self.app.recipe_setpoint_vars["TE210"].get())
        self.sp_TE210_tol = row("TE210 tolerance", self.app.recipe_setpoint_vars["TE210_tol"].get())
        self.sp_post_TE2 = row("Post-deposition TE2", self.app.recipe_setpoint_vars["post_TE2"].get())
        self.sp_post_TE2_tol = row("Post-deposition tol", self.app.recipe_setpoint_vars["post_TE2_tol"].get())
        self.sp_post_TE210 = row("Post-deposition TE210", self.app.recipe_setpoint_vars["post_TE210"].get())
        self.sp_post_TE210_tol = row("Post-deposition TE210 tol", self.app.recipe_setpoint_vars["post_TE210_tol"].get())

        ttk.Separator(f, orient="horizontal").pack(fill="x", pady=10)

        self.sp_MFC1 = row("MFC1 (sccm)", self.app.recipe_setpoint_vars["MFC1"].get())
        self.sp_MFC100 = row("MFC100 (sccm)", self.app.recipe_setpoint_vars["MFC100"].get())
        self.sp_MFC200 = row("MFC200 (sccm)", self.app.recipe_setpoint_vars["MFC200"].get())
        self.sp_MFC300 = row("MFC300 (sccm)", self.app.recipe_setpoint_vars["MFC300"].get())
        self.sp_MFC400 = row("MFC400 (sccm)", self.app.recipe_setpoint_vars["MFC400"].get())

        return f

    def _page_confirm(self):
        f = ttk.Frame(self.page_container)
        ttk.Label(f, text="Confirm recipe summary:", font=("Arial", 12, "bold")).pack(anchor="w", pady=(0, 10))
        self.summary_txt = tk.Text(f, height=18, width=100)
        self.summary_txt.pack(fill="both", expand=True)
        return f


    def _seconds_to_hms(self, seconds: float) -> str:
        try:
            s = int(round(float(seconds)))
        except Exception:
            s = 0
        h = s // 3600
        m = (s % 3600) // 60
        ss = s % 60
        return f"{h:02d}:{m:02d}:{ss:02d}"
    # ----- Validation / build -----

    def _validate_page(self, idx: int) -> bool:
        if idx == 0:
            try:
                v = int(self.delay_var.get())
                if v < 0:
                    raise ValueError
                self.app.recipe_setpoint_vars["delay_min"].set(str(v))
                self.data["setpoints"]["delay_min"] = str(v)
                return True
            except Exception:
                messagebox.showerror("Invalid delay", "Enter a whole number >= 0.")
                return False

        if idx == 1:
            selected = [i for i, var in self.line_vars.items() if var.get()]
            if not selected:
                messagebox.showwarning("No lines", "Select at least one precursor line.")
                return False
            self.data["selected_lines"] = selected
            return True

        if idx == 2:
            selected = self.data["selected_lines"]
            names = {}
            for i in selected:
                nm = self.name_vars[i].get().strip()
                if not nm:
                    messagebox.showerror("Missing name", f"Enter a name for Line {i}.")
                    return False
                names[str(i)] = nm
            self.data["line_names"] = names

            # Rename any existing variants to use the precursor name as prefix: precursor.<n>
            if self.data.get("variants"):
                new_variants = {}
                per_line_count = {}
                for _old_vid, vdat in list(self.data["variants"].items()):
                    ln = int(vdat.get("line", 0))
                    per_line_count[ln] = per_line_count.get(ln, 0) + 1
                    new_vid = self._make_variant_id(ln, per_line_count[ln])
                    new_variants[new_vid] = vdat
                self.data["variants"] = new_variants

            return True

        if idx == 3:
            self._sync_variants_from_table()
            # Basic validation: times >=0, warn if both 0
            for vid, vdat in self.data["variants"].items():
                if vdat["pulse_s"] < 0 or vdat["purge_s"] < 0:
                    messagebox.showerror("Invalid time", f"{vid}: times must be >= 0.")
                    return False
                if vdat["pulse_s"] == 0 and vdat["purge_s"] == 0:
                    # allow, but warn
                    if not messagebox.askokcancel("Zero step", f"{vid} has pulse=0 and purge=0. Keep it anyway?"):
                        return False
            return True

        if idx == 4:
            # blocks must all have cycles and at least 1 step; last block too
            for bi, blk in enumerate(self.data["blocks"], start=1):
                if not blk["steps"]:
                    messagebox.showerror("Empty block", f"Block {bi} has no steps.")
                    return False
                if int(blk.get("cycles", 0)) <= 0:
                    messagebox.showerror("Missing cycles", f"Block {bi} needs a cycle count (>=1).")
                    return False
            return True

        if idx == 5:
            if not self.data.get("stopflow_enabled", False):
                self.data["per_step_sf"] = []
                return True

            # Store per-step Stop Flow HOLD only; pulse time is the step pulse.
            sf = []
            for vid, vdat, hv in self.sf_rows:
                try:
                    h = float(hv.get())
                    if h < 0:
                        raise ValueError
                except Exception:
                    messagebox.showerror("Invalid Stop Flow", "Stop Flow hold times must be numeric and >= 0.")
                    return False

                # 0 disables stop flow for that step
                if h == 0:
                    sf.append({"sf_hold_s": 0.0})
                    continue

                line = int(vdat.get("line", 0))
                pulse = float(vdat.get("pulse_s", 0.0))
                pulse_total = pulse

                if h <= pulse_total:
                    messagebox.showerror("Invalid Stop Flow", f"For {vid}, hold time must be greater than the step pulse time.")
                    return False

                sf.append({"sf_hold_s": h})

            self.data["per_step_sf"] = sf
            return True



        if idx == 6:
            # setpoints validation
            try:
                TE1 = float(self.sp_TE1.get()); TE2 = float(self.sp_TE2.get())
                TE211 = float(self.sp_TE211.get()); TE210 = float(self.sp_TE210.get())
                if TE1 > 400: raise ValueError("TE1>400")
                if TE211 > 190: raise ValueError("TE211>190")
            except Exception:
                messagebox.showerror("Invalid temperatures", "Check TE1<=400 and TE211<=190 and numeric entries.")
                return False

            if TE1 <= TE2:
                messagebox.showwarning("Note", MAX_HEAT_NOTE_MSG)
            if TE211 <= TE210:
                messagebox.showwarning("Note", MAX_HEAT_NOTE_MSG)

            sp = {
                "delay_min": self.data["setpoints"].get("delay_min", "0"),
                "TE1": self.sp_TE1.get(),
                "TE2": self.sp_TE2.get(),
                "TE2_tol": self.sp_TE2_tol.get(),
                "TE210": self.sp_TE210.get(),
                "TE210_tol": self.sp_TE210_tol.get(),
                "TE211": self.sp_TE211.get(),
                "post_TE2": self.sp_post_TE2.get(),
                "post_TE2_tol": self.sp_post_TE2_tol.get(),
                "post_TE210": self.sp_post_TE210.get(),
                "post_TE210_tol": self.sp_post_TE210_tol.get(),
                "MFC1": self.sp_MFC1.get(),
                "MFC100": self.sp_MFC100.get(),
                "MFC200": self.sp_MFC200.get(),
                "MFC300": self.sp_MFC300.get(),
                "MFC400": self.sp_MFC400.get(),
            }
            self.data["setpoints"] = sp
            return True

        if idx == 7:
            # Build and show summary
            recipe = self._build_final_recipe(only_build=True)
            self.summary_txt.delete("1.0", "end")
            self.summary_txt.insert("end", recipe.get("summary", ""))
            return True

        return True


    def _sync_variants_from_table(self):
        # Persist table edits back into self.data["variants"] while keeping each variant tied to its line.
        for vid, ln, pulse_var, purge_var in getattr(self, "variant_rows", []):
            line = int(ln)
            try:
                pulse = float(pulse_var.get())
                purge = float(purge_var.get())
            except Exception:
                # Leave existing values unchanged if parsing fails
                continue

            self.data["variants"][vid] = {"line": line, "pulse_s": pulse, "purge_s": purge}

    def _build_final_recipe(self, only_build: bool = False) -> dict:
        self._sync_variants_from_table()
        line_names = self.data["line_names"]
        sp = self.data["setpoints"]

        # Expand blocks into steps with names and per-step stopflow settings
        blocks_out = []
        sf_list = list(self.data.get("per_step_sf", []))
        sf_idx = 0
        total_time = 0.0

        for blk in self.data["blocks"]:
            steps_out = []
            for vid in blk["steps"]:
                v = self.data["variants"][vid]
                line = int(v["line"])
                name = line_names.get(str(line), f"Line {line}")
                step = {
                    "variant": vid,
                    "line": line,
                    "name": name,
                    "pulse_s": float(v["pulse_s"]),
                    "purge_s": float(v["purge_s"]),
                }
                if self.data.get("stopflow_enabled", False) and sf_idx < len(sf_list):
                    # sf_list stores HOLD only; pulse is derived from the step pulse.
                    step.update(sf_list[sf_idx])
                    if float(step.get("sf_hold_s", 0.0)) > 0.0:
                        step["sf_pulse_s"] = step["pulse_s"]
                    else:
                        step["sf_pulse_s"] = 0.0
                else:
                    step.update({"sf_pulse_s": 0.0, "sf_hold_s": 0.0})
                sf_idx += 1
                steps_out.append(step)

                # time
                if step["sf_hold_s"] > 0 and step["sf_pulse_s"] > 0:
                    total_time += step["sf_hold_s"] + step["purge_s"]
                else:
                    total_time += step["pulse_s"] + step["purge_s"]

            blocks_out.append({"cycles": int(blk["cycles"]), "steps": steps_out})
            total_time *= int(blk["cycles"])

        # Above multiplication is wrong (it multiplies cumulative), so compute properly:
        total_time = 0.0
        for blk in blocks_out:
            one_cycle = 0.0
            for step in blk["steps"]:
                if step["sf_hold_s"] > 0 and step["sf_pulse_s"] > 0:
                    one_cycle += step["sf_hold_s"] + step["purge_s"]
                else:
                    one_cycle += step["pulse_s"] + step["purge_s"]
            total_time += one_cycle * int(blk["cycles"])

        # summary text
        lines = []
        lines.append("Recipe Summary")
        lines.append("==============")
        lines.append("")
        lines.append(f"Delay (min): {sp.get('delay_min','0')}")
        lines.append("")
        lines.append("Setpoints:")
        for k in ("TE1","TE2","TE2_tol","TE211","TE210","TE210_tol","post_TE2","post_TE2_tol","post_TE210","post_TE210_tol","MFC1","MFC100","MFC200","MFC300","MFC400"):
            lines.append(f"  {k}: {sp.get(k,'')}")
        lines.append("")
        lines.append("Blocks:")
        for bi, blk in enumerate(blocks_out, start=1):
            lines.append(f"  Block {bi}  cycles={blk['cycles']}")
            for si, st in enumerate(blk["steps"], start=1):
                lines.append(f"    {si}. {st['name']} (Line {st['line']})  pulse={st['pulse_s']}s purge={st['purge_s']}s  SF(p={st['sf_pulse_s']},h={st['sf_hold_s']})")
        lines.append("")
        lines.append(f"Total runtime (excluding stabilization delay): {self._seconds_to_hms(total_time)}")

        recipe = {
            "title": f"Recipe {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}",
            "created": datetime.now().isoformat(timespec="seconds"),
            "line_names": line_names,
            "setpoints": sp,
            "blocks": blocks_out,
            "total_time_s": total_time,
            "summary": "\n".join(lines),
        }

        if only_build:
            return recipe
        return recipe

    # ----------------------------
    # Shutdown
    # ----------------------------

    def on_close(self):
        self.running = False
        if getattr(self, "logger", None) is not None:
            try:
                self.logger.close()
            except Exception:
                pass
        self.destroy()

if __name__ == "__main__":
    app = PicoSimApp()
    try:
        app.protocol("WM_DELETE_WINDOW", getattr(app, "on_close", app.destroy))
    except Exception:
        pass
    app.mainloop()
