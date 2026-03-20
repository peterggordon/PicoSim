# gui_components.py
# Reusable widgets for PicoSim GUI

import tkinter as tk
from tkinter import ttk, simpledialog

# --- Custom Pop-Up for Setpoints ---
def ask_setpoint(title, label, unit, minval=None, maxval=None, *, allow_float: bool = False):
    """Prompt the user for a numeric value.

    For temperature entries (TE*), PicoSim uses 0.1 °C units on the PLC,
    so allowing floats is useful (e.g., 75.0 °C).

    Returns None if cancelled.
    """
    prompt = f"Enter value for {label} ({unit}):"
    if allow_float:
        return simpledialog.askfloat(title, prompt, minvalue=minval, maxvalue=maxval)
    return simpledialog.askinteger(title, prompt, minvalue=minval, maxvalue=maxval)

# --- Toggle Button with Visual State ---
class ValveToggle(tk.Frame):
    def __init__(self, master, valve_name, command_fn, **kwargs):
        super().__init__(master, **kwargs)
        self.command_fn = command_fn
        self.state = tk.BooleanVar()
        self.name = valve_name

        self.button = ttk.Checkbutton(
            self,
            text=self.name,
            variable=self.state,
            command=self._on_toggle
        )
        self.button.pack(side="left")

        self.indicator = tk.Canvas(self, width=12, height=12)
        self.indicator.pack(side="right", padx=4)
        self._update_light()

    def _on_toggle(self):
        is_on = self.state.get()
        self._update_light()
        self.command_fn(self.name, is_on)

    def _update_light(self):
        self.indicator.delete("all")
        color = "green" if self.state.get() else "red"
        self.indicator.create_oval(2, 2, 10, 10, fill=color)

    def set_state(self, is_open):
        self.state.set(is_open)
        self._update_light()

    def get_state(self):
        return self.state.get()

# --- Labelled Live Readout ---
class LiveReadout(ttk.Frame):
    def __init__(self, master, label_text, unit, **kwargs):
        super().__init__(master, **kwargs)
        self.label = ttk.Label(self, text=label_text)
        self.value = ttk.Label(self, text="----")
        self.unit = ttk.Label(self, text=unit)

        self.label.grid(row=0, column=0, padx=2, sticky="w")
        self.value.grid(row=0, column=1, padx=2, sticky="e")
        self.unit.grid(row=0, column=2, padx=2, sticky="w")

    def update_value(self, val):
        self.value.config(text=str(val))

    def set_error(self):
        self.value.config(text="ERR")

    def clear(self):
        self.value.config(text="----")
