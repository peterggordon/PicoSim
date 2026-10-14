# CLAUDE.md

Guidance for working on PicoSim. Read this before changing anything.

## What this is

PicoSim is an open-source Python/tkinter desktop operator interface for legacy Picosun ALD (atomic layer deposition) tools whose vendor software is no longer supported. It talks to the tool's Omron PLC over FINS/UDP. It was built by reverse-engineering the PLC memory map, so the register addresses in the code are empirically verified ground truth, not documented values. Do not "correct" them from general Omron knowledge.

Features:
- Network tab: connect to the PLC (one snapshot read decides ONLINE/OFFLINE).
- Manual tab: setpoints, valve toggles, lid open/close, Vent and Evac buttons, live values, and a 60 s matplotlib trend view (temps, pressures, flows).
- Recipe tab: a wizard builds recipes (blocks of steps with cycle counts, per-line pulse/purge, optional stop-flow), saved as JSON. Recipes run through a Tk-timer state machine, singly or as a queue.
- Logging: 1 Hz text log (`DATA.txt`) always on once connected; a higher-rate CSV (`RUN_HIRES_*.csv` / `QUEUE_*.csv`) during recipe runs.

## Files

| File | Role |
|---|---|
| `main.py` | Everything UI and control: `PicoSimApp` (tabs, polling thread, manual controls, vent/evac, recipe runner state machine) and `RecipeWizard` (Toplevel wizard). Also holds `SETPOINT_COMMANDS`, `VALVE_COMMANDS`, lid bit constants, trend tag lists. |
| `network.py` | `FINSClient`: raw FINS/UDP over one socket. `read_snapshot()` (CIO110, 38 words), `read_words/read_word/read_cio_words`, `write_word(area=...)`, `write_bit()` (CIO bit area 0x30). A lock serializes all send/recv. Writes succeed only on FINS end code 0x0000. |
| `data_parser.py` | `FIELD_MAP` (byte offset to label, unit, scale) and `parse_snapshot()`, which turns the 90-byte snapshot response into a dict of scaled numbers, or `{"error": ...}`. |
| `data_logger.py` | `DataLogger` (daily `<Month DD YYYY> DATA.txt` in `~/PicoSimLogs`, plus Set Folder/Open Folder buttons) and `HighResPressureCapture` (threaded CSV writer; despite the name it logs all tags when given them). |
| `gui_components.py` | `ask_setpoint()` dialog, `ValveToggle` checkbox+LED, `LiveReadout`. |

## Design decisions and gotchas (settled, do not relitigate)

**Units and scaling**
- All temperatures are 0.1 degC integers on the PLC. UI is in degC; multiply by 10 on write, divide on read.
- Pressure scale is not uniform: PT1 and PT420 scale 1.0, the others 0.1 (see `FIELD_MAP`). Leave as is.
- TE1 and TE2 were swapped in the original GUI. The offsets in `FIELD_MAP` are already corrected (offset 20 = TE1, 22 = TE2). Do not swap back.
- `parse_snapshot` treats `FIELD_MAP` offsets under 76 as payload-relative (base 14). Keep offsets payload-relative.

**Memory areas (DM vs CIO)**
- TE1 button = chamber heater MAX, DM16. TE2 button = chamber TARGET, CIO122. TE210 = Line 2 target, DM145. TE211 = Line 2 heater MAX, DM10. These mix DM and CIO on purpose; always pass `area=` to `write_word/read_word`.
- Heater max below target means the target is never reached; `_warn_if_max_heating_low` and the wizard warn (`MAX_HEAT_NOTE_MSG`) but do not block.
- Wizard limits: TE1 max <= 400 degC, TE211 max <= 190 degC.

**Valves (all command bits, no feedback bits found)**
- Valve commands are bits in CIO105.xx (V1=.00, V2=.01, V6=.02, V110=.03, V210=.05, V211=.0C, V310=.0D). V420 hold-open is CIO102.09. V422 pulse is CIO107.10.
- **V6 is inverted**: bit 0 = open, bit 1 = closed. Handled in `_toggle_valve`, `_on_v6_toggle` (checkbox means CLOSED) and `_refresh_manual_command_states`. Any new V6 code must respect this.
- The UI "state" of a valve is just the command bit read back from CIO105 every 0.5 s. V420 (CIO102) and V422 (CIO107) are not read back.
- V422 is a momentary pulse (max 10 s, default 0.25 s) run in a worker thread in manual mode.

**Line delivery in recipes**
- Line 1 opens V110. Line 2 opens V210 and V211 together and closes both together. Line 3 opens V310. Line 4 opens V420 and V422 together.
- Every step starts with `_close_all_line_valves()`.
- Stop-flow step: close V6, pulse line for `sf_pulse_s`, keep V6 closed until `sf_hold_s`, reopen V6, then purge. Step duration is hold + purge (not pulse + purge).

**Pumping interlock (shared by Evac, recipe pumpdown, abort)**
- PT1 > 100 hPa: V2 open, V1 closed. PT1 <= 100 hPa: V2 closed, V1 open. Implemented in `_set_pump_valves_by_pressure`; V6 forced open. Use that helper rather than re-implementing.
- Vent: close V1/V2, all MFCs to 2000 sccm, wait PT1 >= 1050 hPa, restore standby flows (MFC1 300, lines 40).

**Lid**
- Lid state is inferred from command bits CIO12.07 (open cmd) and CIO12.08 (close cmd). Open allowed only if PT1 >= 950 hPa. Opening releases the close bit first, waits 150 ms, then asserts open. Treat the lid state as a heuristic, not a measurement.

**Recipe run sequence** (`_run_tick`, driven by `self.after`)
1. prechecks: PT1 < 100 and lid closed (prompts to close)
2. apply_flows (MFC setpoints)
3. pumpdown until PT1 < 90
4. apply_temps
5. wait_stable: PT1 < 15 and TE2/TE210 within tolerance
6. delay countdown (1 s ticks)
7. sequencing: step_begin / step_wait through blocks and cycles
8. finish: set post-deposition TE2 (TE210 post setpoint was added in v1.0.3; check `_finish_run_success`)
- Abort (`_safe_state_abort`): close all line valves, apply the pumping interlock, V6 open, TE2 and TE210 to 20 degC. Heater maxes and flows are left alone.
- `post_TE2_tol` (and the TE210 equivalent) are stored in the recipe but never read. Intentionally left alone to avoid scope creep.

**Threading**
- Tk widgets are touched only on the main thread: the poll thread uses `self.after(0, ...)`.
- The poll thread (`_poll_loop`) does the snapshot read, CIO12/CIO105 reads (0.5 s), and heater limit reads (1 s). It shares the UDP socket with manual writes via `FINSClient._io_lock`.
- Manual and runner writes happen on the Tk main thread and block for up to the 1 s socket timeout if the PLC does not answer.

**Logging**
- Normal log is 1 Hz and its format must not change (other tooling may read it). Columns are `HH:MM:SS` followed by values in sorted-tag order, no header.
- Hi-res capture is add-on only: it samples the same `snapshot_data` dict, so its real rate is bounded by the poll rate. Starts automatically with a recipe run, stays open across a queue.
- Log files are `*.txt` and `*.csv`, which are gitignored.

**Compatibility**
- Keep backward-compatibility fallbacks so old recipe JSON files load on new versions (use `.get()` with defaults for any new recipe field). The loader only requires `blocks` and `setpoints`.
- Recipe JSON: `title, created, line_names, setpoints (all strings), blocks[{cycles, steps[{variant, line, name, pulse_s, purge_s, sf_pulse_s, sf_hold_s}]}], total_time_s, summary`.

**Network setup**
- PC IP is hardcoded to 192.168.1.2 and PLC to 192.168.1.3:9600. FINS node numbers are the last octets. The PC must actually have that static IP or the socket bind fails and the app starts "OFFLINE". Making this configurable is a known TODO.

## Running

```
python main.py
```
Requires Python 3 with tkinter and matplotlib (`pip install -r requirements.txt`). `os.startfile` in the logger makes "Open Folder" Windows-only; everything else is portable. Without a PLC the app opens but stays OFFLINE; most controls do nothing offline.

## Building the Windows exe

On Windows (Store Python), PyInstaller must be invoked through `python -m`:

```
python -m PyInstaller --onefile --windowed ^
  --hidden-import main --hidden-import network --hidden-import data_parser ^
  --hidden-import gui_components --hidden-import data_logger ^
  main.py
```
The exe is distributed through GitHub Releases. Never commit `dist/`, `build/` or `*.spec` (gitignored). Keep the working folder outside OneDrive to avoid sync conflicts with Git.

## How to make changes (maintainer preferences)

The maintainer is a self-described novice coder. So:
- Give paste-ready edits with explicit placement: name the function, and include the surrounding context lines so the location is unambiguous. No speculative rewrites.
- Keep edits minimal and scoped. Do not silently expand into adjacent features or tidy unrelated code.
- Run `python -m py_compile` on every file you touch before handing it back.
- Bump `APP_VERSION` in `main.py` for user-visible changes.
- Commit messages put the version in the summary line, e.g. `v1.0.3: Add post-deposition TE210 setpoint for Line 2`.
- Never put em dashes in docs or release text meant for public readers.

## Known bugs and rough edges

Ordered roughly by how much they matter. None are fixed unless noted; fix only when asked.

1. **PT1 safety during pulsing is effectively inactive.** `_run_tick` checks `phase in ("sequencing", "step_pulse", "step_purge", "step_sf")`, but the real phases are `step_begin` and `step_wait`. Only `sequencing` (one tick) matches, so a PT1 > 20 hPa excursion mid-recipe does not halt the run.
2. **Aborting a queue leaves `_queue_run_list` populated.** `_abort_run` does not clear it, so the next "Run Current Recipe" is treated as part of the stale queue: filename prefix says QUEUE, and on completion `_finish_run_success` advances to the next stale queue entry. `_queue_run_list` is also never initialized in `__init__` (accessed via `getattr/hasattr`).
3. **Stop-flow is unreachable from the wizard.** `stopflow_enabled` is never set True (`_page_stopflow_enable` is not in the page list), so the per-step hold times on the Stop Flow page are ignored. The runner and recipe format support stop-flow; only the wizard path is dead. `_start_over` also omits `per_step_sf`.
4. **Final popup update raises and is swallowed.** `_finish_run_success` calls `_update_run_progress_popup` with four positional args, but the method is keyword-only. A `try/except` hides the TypeError; the popup text is set by the later direct `.set()` calls instead.
5. **No offline detection after connecting.** `self.online` is set once by "Connect to Tool" and never cleared on timeouts. If the PLC drops, polling just returns None, the display goes stale, and writes report FAILED in the console only.
6. **Blocking UDP on the UI thread.** Manual writes and the runner's valve toggles run on the Tk thread; a dead PLC freezes the UI for 1 s per call. Recipe valve timing also jitters with poll-thread socket contention.
7. **Pending step timers survive abort.** `after()` callbacks for `close_line` and `end_hold_then_purge` are not cancelled. `end_hold_then_purge` can reopen V6 after an abort (harmless in practice because abort also opens V6).
8. **Loggers.** `rotate_logs()` is never called, so old logs are never pruned. `DataLogger` picks its filename once and does not roll over at midnight. The "0000" no-data sentinel in the header and trend code is legacy: `parse_snapshot` never emits it.
9. **Post-deposition tolerances unused** (see above; intentional).
10. **Wizard prefill** ("Edit Recipe") restores lines, names and setpoints only; blocks and variants start empty.
11. **Misc.** `_build_recipe_tab` calls `_close_run_progress_popup()` and `_stop_run_hires_capture()` during construction (no-ops, but misplaced). Evac comments mention PT2 while the code uses PT1. Many `SETPOINT_COMMANDS` entries (TE110, TE310, PT*) have no UI and are unverified. `_start_run` reads `setpoints["delay_min"]` without a default, so a recipe lacking it will raise.

## Open items

- Make PC/PLC IPs configurable.
