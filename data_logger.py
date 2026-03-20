# data_logger.py
# Handles logging of real-time data to file with rotation

import os
import time
from datetime import datetime
import tkinter as tk
from tkinter import filedialog

import threading
from typing import Callable, Dict, List, Optional

LOG_FOLDER = os.path.expanduser("~/PicoSimLogs")
MAX_LOG_FILES = 30

class DataLogger:
    def __init__(self, parent_frame=None):
        self.folder = LOG_FOLDER
        self.ensure_folder()
        self.filename = self._get_filename()
        self.file = open(self.filename, "a")

        if parent_frame:
            self.add_ui_buttons(parent_frame)

    def ensure_folder(self):
        os.makedirs(self.folder, exist_ok=True)

    def _get_filename(self):
        date_str = datetime.now().strftime("%B %d %Y")
        return os.path.join(self.folder, f"{date_str} DATA.txt")

    def log_snapshot(self, snapshot_dict):
        timestamp = datetime.now().strftime("%H:%M:%S")
        line = [timestamp] + [str(snapshot_dict.get(key, "0000")) for key in sorted(snapshot_dict.keys())]
        self.file.write(" ".join(line) + "\n")
        self.file.flush()

    def rotate_logs(self):
        files = sorted(
            [f for f in os.listdir(self.folder) if f.endswith("DATA.txt")],
            key=lambda f: os.path.getmtime(os.path.join(self.folder, f))
        )
        if len(files) > MAX_LOG_FILES:
            for old_file in files[:-MAX_LOG_FILES]:
                os.remove(os.path.join(self.folder, old_file))

    def close(self):
        if self.file:
            self.file.close()

    def set_folder_dialog(self):
        new_path = filedialog.askdirectory(title="Select Log Folder")
        if new_path:
            self.set_folder(new_path)

    def set_folder(self, new_path):
        self.folder = new_path
        self.ensure_folder()
        self.filename = self._get_filename()
        if self.file:
            self.file.close()
        self.file = open(self.filename, "a")

    def open_folder_in_explorer(self):
        os.startfile(self.folder)  # Windows only

    def add_ui_buttons(self, parent_frame):
        button_frame = tk.Frame(parent_frame)
        button_frame.pack(pady=5)

        set_btn = tk.Button(button_frame, text="Set Save Folder", command=self.set_folder_dialog)
        set_btn.grid(row=0, column=0, padx=10)

        open_btn = tk.Button(button_frame, text="Open Folder", command=self.open_folder_in_explorer)
        open_btn.grid(row=0, column=1, padx=10)


class HighResPressureCapture:
    """User-initiated, high-rate pressure capture into a separate CSV.

    Design constraints:
      - Add-on only: does not change normal (1 Hz) log cadence or format.
      - Reads from the same snapshot source PicoSim already uses (no new PLC mapping).
      - Pressure-only CSV with millisecond local time + fractional epoch seconds for alignment.

    Snapshot getter contract:
      - snapshot_getter() -> dict | None
      - Values should already be in engineering units as PicoSim displays them.
    """

    def __init__(
        self,
        log_folder_getter: Callable[[], str],
        snapshot_getter: Callable[[], Optional[Dict]],
        tags: List[str],
        target_hz: float = 50.0,
        flush_every: int = 50,
        filename_prefix: str = "PRESSURE_HIRES",
    ):
        self._log_folder_getter = log_folder_getter
        self._snapshot_getter = snapshot_getter

        self.tags = list(tags)
        self.target_hz = float(target_hz)
        self.period_s = 1.0 / self.target_hz if self.target_hz > 0 else 0.0
        self.flush_every = max(1, int(flush_every))

        self.filename_prefix = str(filename_prefix or "PRESSURE_HIRES").strip() or "PRESSURE_HIRES"

        self._stop_event = threading.Event()
        self._thread: Optional[threading.Thread] = None

        self._file = None
        self.filename: Optional[str] = None
        self._buffer: List[str] = []
        self._last_values: Dict[str, str] = {t: "" for t in self.tags}

        # Public telemetry
        self.samples_written: int = 0
        self.effective_hz: float = 0.0
        self._t0_perf: Optional[float] = None

    def is_running(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    def start(self) -> str:
        if self.is_running():
            return self.filename or ""

        folder = self._log_folder_getter()
        os.makedirs(folder, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.filename = os.path.join(folder, f"{self.filename_prefix}_{stamp}.csv")

        self._file = open(self.filename, "w", buffering=1)
        header = ["time_local", "epoch_s"] + self.tags
        self._file.write(",".join(header) + "\n")

        self._buffer.clear()
        self.samples_written = 0
        self.effective_hz = 0.0
        self._t0_perf = time.perf_counter()

        self._stop_event.clear()
        self._thread = threading.Thread(target=self._run, name="HighResPressureCapture", daemon=True)
        self._thread.start()
        return self.filename

    def stop(self) -> None:
        self._stop_event.set()
        if self._thread:
            self._thread.join(timeout=2.0)
        self._thread = None

        self._flush(force=True)
        if self._file:
            try:
                self._file.flush()
            finally:
                self._file.close()
        self._file = None

    def _flush(self, force: bool = False) -> None:
        if not self._file:
            self._buffer.clear()
            return
        if not force and len(self._buffer) < self.flush_every:
            return
        self._file.write("".join(self._buffer))
        self._file.flush()
        self._buffer.clear()

    def _update_rate(self) -> None:
        if self._t0_perf is None:
            return
        elapsed = time.perf_counter() - self._t0_perf
        if elapsed > 0:
            self.effective_hz = self.samples_written / elapsed

    def _run(self) -> None:
        if not self._file:
            return

        next_tick = time.perf_counter()

        while not self._stop_event.is_set():
            epoch = time.time()
            time_local = datetime.fromtimestamp(epoch).strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]

            snapshot = None
            try:
                snapshot = self._snapshot_getter()
            except Exception:
                snapshot = None

            # If snapshot missing, log timestamp + last-known values consistently.
            if isinstance(snapshot, dict):
                for t in self.tags:
                    if t in snapshot:
                        self._last_values[t] = str(snapshot[t])

            row = [time_local, f"{epoch:.6f}"] + [self._last_values[t] for t in self.tags]
            self._buffer.append(",".join(row) + "\n")

            self.samples_written += 1
            self._update_rate()
            self._flush(force=False)

            # Pace to target rate, but naturally degrades to "as fast as achieved".
            if self.period_s > 0:
                next_tick += self.period_s
                sleep_s = next_tick - time.perf_counter()
                if sleep_s > 0:
                    time.sleep(sleep_s)
                else:
                    next_tick = time.perf_counter()


# Hook for main.py
if __name__ == "__main__":
    import tkinter as tk
    root = tk.Tk()
    root.title("Logger Test")
    logger = DataLogger(parent_frame=root)
    root.mainloop()
