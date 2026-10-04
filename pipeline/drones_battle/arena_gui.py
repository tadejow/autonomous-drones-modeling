"""Graphical front-end of the drone battle arena (tkinter, no extra dependencies).

    python -m pipeline.drones_battle.arena_gui [--dir path/to/student_teams]

* The team list shows every folder from ``student_teams/`` (ZIP files are
  unpacked automatically) with a validation status, plus optionally the
  example teams from ``teams/``.
* **Walka**: one match between the team chosen as attacker and the team chosen
  as defender, shown in the 3D window.
* **Turniej**: every checked team plays every other one in both roles; the
  matches are shown one after another in the same 3D window and the full
  results table opens after the last one.

Matches run in a worker thread; the GUI only exchanges messages with it
through a queue, so the window never freezes.
"""

from __future__ import annotations

import argparse
import csv
import os
import queue
import random
import subprocess
import sys
import threading
import tkinter as tk
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime
from pathlib import Path
from tkinter import filedialog, messagebox, ttk
from typing import Any, Callable, Optional

from pipeline.drones_battle.arena_orchestrator import MatchCancelled, run_match, window_layout_for
from pipeline.drones_battle.core.config import ArenaConfig, load_config
from pipeline.drones_battle.core.submissions import (
    CONFIG_5V5, DEFAULT_SUBMISSIONS_DIR, PACKAGE_DIR, SubmissionCheck, TeamEntry, check_submission,
    discover_submissions, example_teams, extract_zips,
)
from pipeline.drones_battle.core.types import MatchResult
from pipeline.drones_battle.tournament import MatchRecord, PairingStats, Standing, format_report, run_tournament

REPO_DIR = PACKAGE_DIR.parent.parent
RESULTS_DIR = PACKAGE_DIR / "matches" / "gui"
START_JITTER_M = 3.0
TEMPO_OPTIONS = {
    "1× (czas rzeczywisty)": (True, False, 1.0),
    "2×": (True, False, 2.0),
    "4×": (True, False, 4.0),
    "bez wizualizacji (najszybciej)": (False, True, 1.0),
}
CHECK_OK, CHECK_UNCHECKED = "☑", "☐"


class ArenaApp:
    def __init__(self, root: tk.Tk, submissions_dir: Path) -> None:
        self.root = root
        self.events: queue.Queue[tuple[str, Any]] = queue.Queue()
        self.entries: dict[str, TeamEntry] = {}
        self.checks: dict[str, Optional[SubmissionCheck]] = {}
        self.checked: set[str] = set()
        self.cancel = threading.Event()
        self.worker: Optional[threading.Thread] = None
        self.validator = ThreadPoolExecutor(max_workers=4, thread_name_prefix="validate")
        self.sitl_process: Optional[subprocess.Popen[str]] = None

        self.dir_var = tk.StringVar(value=str(submissions_dir))
        self.format_var = tk.StringVar(value="3v3")
        self.backend_var = tk.StringVar(value="kinematic")
        self.tempo_var = tk.StringVar(value=next(iter(TEMPO_OPTIONS)))
        self.rounds_var = tk.IntVar(value=1)
        self.hold_var = tk.IntVar(value=3)
        self.examples_var = tk.BooleanVar(value=True)
        self.attacker_var = tk.StringVar()
        self.defender_var = tk.StringVar()
        self.status_var = tk.StringVar(value="Gotowe.")
        self.summary_var = tk.StringVar()

        root.title("Walki Dronów – Arena")
        root.geometry("1180x760")
        root.minsize(980, 640)
        root.protocol("WM_DELETE_WINDOW", self._on_close)
        self._build()
        self.refresh()
        root.after(100, self._poll)

    # ------------------------------------------------------------------ layout
    def _build(self) -> None:
        style = ttk.Style(self.root)
        if "clam" in style.theme_names() and sys.platform != "win32":
            style.theme_use("clam")
        style.configure("Big.TButton", font=("TkDefaultFont", 11, "bold"), padding=8)

        top = ttk.Frame(self.root, padding=(10, 10, 10, 4))
        top.pack(fill="x")
        ttk.Label(top, text="Katalog drużyn:").pack(side="left")
        ttk.Entry(top, textvariable=self.dir_var, state="readonly").pack(side="left", fill="x", expand=True, padx=6)
        ttk.Button(top, text="Zmień…", command=self._choose_dir).pack(side="left")
        ttk.Button(top, text="Otwórz", command=lambda: _open_path(Path(self.dir_var.get()))).pack(side="left", padx=4)
        ttk.Button(top, text="Odśwież", command=self.refresh).pack(side="left")

        body = ttk.Panedwindow(self.root, orient="horizontal")
        body.pack(fill="both", expand=True, padx=10, pady=4)

        # Left: team list.
        left = ttk.Labelframe(body, text="Drużyny (kliknij ☐, aby zaznaczyć do turnieju)", padding=6)
        body.add(left, weight=3)
        columns = ("check", "team", "authors", "status")
        self.tree = ttk.Treeview(left, columns=columns, show="headings", selectmode="browse")
        for column, title, width, stretch in (
            ("check", "", 34, False), ("team", "Drużyna", 220, True), ("authors", "Autorzy", 200, True),
            ("status", "Status", 230, True),
        ):
            self.tree.heading(column, text=title)
            self.tree.column(column, width=width, stretch=stretch, anchor="center" if column == "check" else "w")
        self.tree.tag_configure("ok", foreground="#1b7f2a")
        self.tree.tag_configure("warn", foreground="#b26a00")
        self.tree.tag_configure("error", foreground="#c62828")
        self.tree.tag_configure("pending", foreground="#777777")
        scroll = ttk.Scrollbar(left, orient="vertical", command=self.tree.yview)
        self.tree.configure(yscrollcommand=scroll.set)
        self.tree.grid(row=0, column=0, sticky="nsew")
        scroll.grid(row=0, column=1, sticky="ns")
        left.rowconfigure(0, weight=1)
        left.columnconfigure(0, weight=1)
        self.tree.bind("<Button-1>", self._on_tree_click)
        self.tree.bind("<Double-1>", lambda _e: self._show_details())

        buttons = ttk.Frame(left)
        buttons.grid(row=1, column=0, columnspan=2, sticky="ew", pady=(6, 0))
        ttk.Button(buttons, text="Zaznacz poprawne", command=self._check_all_ok).pack(side="left")
        ttk.Button(buttons, text="Odznacz", command=self._uncheck_all).pack(side="left", padx=4)
        ttk.Button(buttons, text="Szczegóły", command=self._show_details).pack(side="left")
        ttk.Checkbutton(buttons, text="Pokaż drużyny przykładowe", variable=self.examples_var,
                        command=self._reload_list).pack(side="right")

        # Right: settings and actions.
        right = ttk.Frame(body, padding=(8, 0, 0, 0))
        body.add(right, weight=2)

        settings = ttk.Labelframe(right, text="Ustawienia", padding=8)
        settings.pack(fill="x")
        ttk.Label(settings, text="Format:").grid(row=0, column=0, sticky="w")
        ttk.Radiobutton(settings, text="3 vs 3", value="3v3", variable=self.format_var).grid(row=0, column=1, sticky="w")
        ttk.Radiobutton(settings, text="5 vs 5", value="5v5", variable=self.format_var).grid(row=0, column=2, sticky="w")
        ttk.Label(settings, text="Fizyka:").grid(row=1, column=0, sticky="w", pady=(4, 0))
        ttk.Radiobutton(settings, text="model kinematyczny", value="kinematic",
                        variable=self.backend_var).grid(row=1, column=1, sticky="w", pady=(4, 0))
        ttk.Radiobutton(settings, text="ArduPilot SITL", value="sitl",
                        variable=self.backend_var).grid(row=1, column=2, sticky="w", pady=(4, 0))
        ttk.Label(settings, text="Tempo:").grid(row=2, column=0, sticky="w", pady=(4, 0))
        ttk.Combobox(settings, textvariable=self.tempo_var, values=list(TEMPO_OPTIONS), state="readonly",
                     width=28).grid(row=2, column=1, columnspan=2, sticky="w", pady=(4, 0))
        ttk.Label(settings, text="Rundy na parę:").grid(row=3, column=0, sticky="w", pady=(4, 0))
        ttk.Spinbox(settings, from_=1, to=20, textvariable=self.rounds_var, width=5).grid(
            row=3, column=1, sticky="w", pady=(4, 0))
        ttk.Label(settings, text="Pauza po meczu [s]:").grid(row=4, column=0, sticky="w", pady=(4, 0))
        ttk.Spinbox(settings, from_=0, to=30, textvariable=self.hold_var, width=5).grid(
            row=4, column=1, sticky="w", pady=(4, 0))

        sitl = ttk.Frame(settings)
        sitl.grid(row=5, column=0, columnspan=3, sticky="w", pady=(8, 0))
        self.sitl_start = ttk.Button(sitl, text="Uruchom SITL", command=self._start_sitl)
        self.sitl_start.pack(side="left")
        self.sitl_stop = ttk.Button(sitl, text="Zatrzymaj SITL", command=self._stop_sitl)
        self.sitl_stop.pack(side="left", padx=4)
        if sys.platform == "win32":
            for button in (self.sitl_start, self.sitl_stop):
                button.state(["disabled"])
            ttk.Label(sitl, text="(SITL tylko na Linuksie / WSL)", foreground="#777777").pack(side="left")

        single = ttk.Labelframe(right, text="Pojedyncza walka", padding=8)
        single.pack(fill="x", pady=(10, 0))
        ttk.Label(single, text="Atakuje:").grid(row=0, column=0, sticky="w")
        self.attacker_box = ttk.Combobox(single, textvariable=self.attacker_var, state="readonly", width=34)
        self.attacker_box.grid(row=0, column=1, sticky="ew", padx=4)
        ttk.Label(single, text="Broni:").grid(row=1, column=0, sticky="w", pady=(4, 0))
        self.defender_box = ttk.Combobox(single, textvariable=self.defender_var, state="readonly", width=34)
        self.defender_box.grid(row=1, column=1, sticky="ew", padx=4, pady=(4, 0))
        ttk.Button(single, text="Zamień", command=self._swap).grid(row=0, column=2, rowspan=2, padx=(4, 0))
        single.columnconfigure(1, weight=1)
        self.fight_button = ttk.Button(single, text="Walka", style="Big.TButton", command=self.start_single)
        self.fight_button.grid(row=2, column=0, columnspan=3, sticky="ew", pady=(8, 0))

        tournament = ttk.Labelframe(right, text="Turniej (zaznaczone drużyny, każda para w obu rolach)", padding=8)
        tournament.pack(fill="x", pady=(10, 0))
        ttk.Label(tournament, textvariable=self.summary_var).pack(anchor="w")
        self.tournament_button = ttk.Button(tournament, text="Turniej", style="Big.TButton",
                                            command=self.start_tournament)
        self.tournament_button.pack(fill="x", pady=(6, 0))

        control = ttk.Frame(right)
        control.pack(fill="x", pady=(10, 0))
        self.stop_button = ttk.Button(control, text="Przerwij", command=self.stop)
        self.stop_button.pack(side="left")
        self.stop_button.state(["disabled"])
        ttk.Button(control, text="Wyniki…", command=self._open_results_dir).pack(side="right")
        self.progress = ttk.Progressbar(right, mode="determinate")
        self.progress.pack(fill="x", pady=(8, 0))
        ttk.Label(right, textvariable=self.status_var, wraplength=420).pack(anchor="w", pady=(4, 0))

        log_frame = ttk.Labelframe(self.root, text="Dziennik", padding=4)
        log_frame.pack(fill="both", padx=10, pady=(0, 10))
        self.log_text = tk.Text(log_frame, height=9, wrap="word", state="disabled", font=("TkFixedFont", 9))
        log_scroll = ttk.Scrollbar(log_frame, orient="vertical", command=self.log_text.yview)
        self.log_text.configure(yscrollcommand=log_scroll.set)
        self.log_text.pack(side="left", fill="both", expand=True)
        log_scroll.pack(side="right", fill="y")

        for variable in (self.rounds_var,):
            variable.trace_add("write", lambda *_: self._update_summary())

    # ------------------------------------------------------------------ team list
    def refresh(self) -> None:
        root = Path(self.dir_var.get())
        if root.is_dir():
            for message in extract_zips(root):
                self.log(message)
        else:
            self.log(f"Katalog {root} nie istnieje.")
        self.checks = {k: v for k, v in self.checks.items() if k.startswith("example:")}
        self._reload_list(validate=True)

    def _reload_list(self, validate: bool = False) -> None:
        students = discover_submissions(Path(self.dir_var.get()))
        examples = example_teams() if self.examples_var.get() else []
        self.entries = {e.key: e for e in students + examples}
        self.checked &= set(self.entries)
        for entry in examples:
            self.checks.setdefault(entry.key, SubmissionCheck(True, warnings=[]))
        if validate or any(e.key not in self.checks for e in students):
            for entry in students:
                if entry.key not in self.checks or validate:
                    self.checks[entry.key] = None
                    self.validator.submit(self._validate, entry)
        if not students:
            self.log(f"Brak katalogów drużyn w {self.dir_var.get()} (opis formatu: student_teams/README.md).")
        self._render_list()

    def _validate(self, entry: TeamEntry) -> None:
        self.events.put(("checked", (entry.key, check_submission(entry))))

    def _render_list(self) -> None:
        self.tree.delete(*self.tree.get_children())
        for key, entry in self.entries.items():
            check = self.checks.get(key)
            if check is None:
                status, tag = "sprawdzanie…", "pending"
            elif not check.ok:
                status, tag = check.summary(), "error"
            elif entry.kind == "example":
                status, tag = "OK (przykład)", "ok"
            else:
                status, tag = check.summary(), "warn" if check.warnings else "ok"
            mark = CHECK_OK if key in self.checked else CHECK_UNCHECKED
            name = entry.label
            self.tree.insert("", "end", iid=key, values=(mark, name, ", ".join(entry.authors), status), tags=(tag,))
        labels = [e.label for e in self.entries.values()]
        for box, variable, fallback in ((self.attacker_box, self.attacker_var, 0),
                                        (self.defender_box, self.defender_var, 1)):
            box["values"] = labels
            if variable.get() not in labels:
                variable.set(labels[fallback] if len(labels) > fallback else (labels[0] if labels else ""))
        self._update_summary()

    def _on_tree_click(self, event: tk.Event) -> Optional[str]:
        if self.tree.identify_region(event.x, event.y) != "cell" or self.tree.identify_column(event.x) != "#1":
            return None
        key = self.tree.identify_row(event.y)
        if key:
            self.checked.symmetric_difference_update({key})
            self.tree.set(key, "check", CHECK_OK if key in self.checked else CHECK_UNCHECKED)
            self._update_summary()
        return "break"

    def _check_all_ok(self) -> None:
        self.checked = {k for k, c in self.checks.items() if c is not None and c.ok and k in self.entries}
        self._render_list()

    def _uncheck_all(self) -> None:
        self.checked.clear()
        self._render_list()

    def _update_summary(self) -> None:
        count = len(self.checked)
        try:
            rounds = max(int(self.rounds_var.get()), 1)
        except (tk.TclError, ValueError):
            rounds = 1
        matches = count * (count - 1) * rounds
        self.summary_var.set(f"Zaznaczone drużyny: {count}   •   meczów: {matches}")

    def _show_details(self) -> None:
        key = self.tree.focus()
        if not key:
            return
        entry, check = self.entries[key], self.checks.get(key)
        lines = [entry.label, f"Katalog: {entry.folder}", f"Atak: {entry.attacker}", f"Obrona: {entry.defender}"]
        if entry.authors:
            lines.append("Autorzy: " + ", ".join(entry.authors))
        if check is None:
            lines.append("\nSprawdzanie w toku…")
        else:
            lines.append(f"\nŚredni czas wywołania: {check.mean_ms:.1f} ms, najdłuższy: {check.max_ms:.1f} ms")
            lines += ["\nBŁĘDY:"] + check.errors if check.errors else []
            lines += ["\nUwagi:"] + check.warnings if check.warnings else []
            if check.ok and not check.warnings:
                lines.append("Brak uwag.")
        _text_dialog(self.root, f"Drużyna {entry.label}", "\n".join(lines))

    def _choose_dir(self) -> None:
        chosen = filedialog.askdirectory(initialdir=self.dir_var.get(), title="Katalog z drużynami studentów")
        if chosen:
            self.dir_var.set(chosen)
            self.checked.clear()
            self.refresh()

    def _swap(self) -> None:
        attacker, defender = self.attacker_var.get(), self.defender_var.get()
        self.attacker_var.set(defender)
        self.defender_var.set(attacker)

    def _entry_by_label(self, label: str) -> Optional[TeamEntry]:
        return next((e for e in self.entries.values() if e.label == label), None)

    # ------------------------------------------------------------------ running
    def _config(self) -> ArenaConfig:
        config = load_config(CONFIG_5V5) if self.format_var.get() == "5v5" else load_config()
        return config.with_overrides(sandbox={"isolation": "process"})

    def _tempo(self) -> tuple[bool, bool, float]:
        visualize, fast, speedup = TEMPO_OPTIONS[self.tempo_var.get()]
        if self.backend_var.get() == "sitl" and speedup != 1.0:
            self.log("SITL działa w czasie rzeczywistym: tempo 2×/4× zmienione na 1×.")
            speedup = 1.0
        return visualize, fast, speedup

    def _ensure_valid(self, entries: list[TeamEntry]) -> Optional[list[TeamEntry]]:
        pending = [e.label for e in entries if self.checks.get(e.key) is None]
        if pending:
            messagebox.showinfo("Arena", "Poczekaj na zakończenie sprawdzania drużyn:\n" + "\n".join(pending))
            return None
        broken = [e for e in entries if not self.checks[e.key].ok]  # type: ignore[union-attr]
        if not broken:
            return entries
        names = "\n".join(f"• {e.label}: {self.checks[e.key].summary()}" for e in broken)  # type: ignore[union-attr]
        if messagebox.askyesno("Drużyny z błędami", f"Te drużyny mają błędy:\n{names}\n\nPominąć je?"):
            return [e for e in entries if e not in broken]
        return entries

    def _busy(self, busy: bool) -> None:
        for button in (self.fight_button, self.tournament_button):
            button.state(["disabled"] if busy else ["!disabled"])
        self.stop_button.state(["!disabled"] if busy else ["disabled"])

    def start_single(self) -> None:
        if self.worker is not None and self.worker.is_alive():
            return
        attacker = self._entry_by_label(self.attacker_var.get())
        defender = self._entry_by_label(self.defender_var.get())
        if attacker is None or defender is None:
            messagebox.showwarning("Arena", "Wybierz drużynę atakującą i broniącą.")
            return
        valid = self._ensure_valid([attacker] if attacker == defender else [attacker, defender])
        if valid is None or len(valid) < (1 if attacker == defender else 2):
            return
        self._launch(self._run_single, attacker, defender, self._config(), self._tempo(), self.backend_var.get())

    def start_tournament(self) -> None:
        if self.worker is not None and self.worker.is_alive():
            return
        teams = [self.entries[k] for k in self.entries if k in self.checked]
        if len(teams) < 2:
            messagebox.showwarning("Arena", "Zaznacz co najmniej dwie drużyny (kolumna ☐).")
            return
        valid = self._ensure_valid(teams)
        if valid is None:
            return
        if len(valid) < 2:
            messagebox.showwarning("Arena", "Zostało mniej niż dwie poprawne drużyny.")
            return
        rounds = max(int(self.rounds_var.get()), 1)
        self._launch(self._run_tournament, valid, rounds, self._config(), self._tempo(), self.backend_var.get(),
                     float(self.hold_var.get()))

    def _launch(self, target: Callable[..., None], *args: Any) -> None:
        self.cancel.clear()
        self._busy(True)
        self.worker = threading.Thread(target=self._guard, args=(target, *args), daemon=True)
        self.worker.start()

    def _guard(self, target: Callable[..., None], *args: Any) -> None:
        try:
            target(*args)
        except MatchCancelled:
            self.events.put(("log", "Przerwano."))
        except Exception as exc:  # noqa: BLE001 - report every failure in the GUI
            self.events.put(("error", f"{type(exc).__name__}: {exc}"))
        finally:
            self.events.put(("idle", None))

    def stop(self) -> None:
        self.cancel.set()
        self.status_var.set("Przerywanie…")

    def _run_single(self, attacker: TeamEntry, defender: TeamEntry, config: ArenaConfig,
                    tempo: tuple[bool, bool, float], backend_name: str) -> None:
        visualize, fast, speedup = tempo
        stamp = datetime.now().strftime("%Y-%m-%d_%H%M%S")
        record = RESULTS_DIR / f"{stamp}_{_safe(attacker.key)}_vs_{_safe(defender.key)}.jsonl"
        title = f"{attacker.label} (atak) vs {defender.label} (obrona)"
        self.events.put(("status", f"Walka: {title}" + ("  –  zamknij okno 3D, aby wrócić" if visualize else "")))
        self.events.put(("progress", (0, 1)))
        result = run_match(
            config, attacker.attacker, defender.defender, backend_name, fast=fast, visualize=visualize,
            record_path=record, seed=random.randrange(10_000),
            start_jitter_m=START_JITTER_M if backend_name == "kinematic" else 0.0, verbose=False,
            final_hold_s=None, speedup=speedup, cancel=self.cancel,
            on_event=lambda line: self.events.put(("log", line)), title=title,
        )
        self.events.put(("single_done", (attacker, defender, result, record)))

    def _run_tournament(self, teams: list[TeamEntry], rounds: int, config: ArenaConfig,
                        tempo: tuple[bool, bool, float], backend_name: str, hold_s: float) -> None:
        visualize, fast, speedup = tempo
        stamp = datetime.now().strftime("%Y-%m-%d_%H%M%S")
        record_dir = RESULTS_DIR / f"turniej_{stamp}"
        record_dir.mkdir(parents=True, exist_ok=True)
        records: list[MatchRecord] = []
        visualizer = None
        backend = None
        try:
            if visualize:
                from pipeline.drones_battle.arena_visualizer import VisualizerProcess

                visualizer = VisualizerProcess(config, window_layout_for(config, backend_name), "Turniej")
                visualizer.wait_ready()
            if backend_name == "sitl":
                from pipeline.drones_battle.backends import make_backend

                backend = make_backend("sitl", config)

            def on_start(att: TeamEntry, dfn: TeamEntry, index: int, total: int) -> None:
                self.events.put(("status", f"Mecz {index}/{total}: {att.label} (atak) vs {dfn.label} (obrona)"))
                self.events.put(("progress", (index - 1, total)))

            def on_match(record: MatchRecord, index: int, total: int) -> None:
                records.append(record)
                self.events.put(("match", record))
                self.events.put(("progress", (index, total)))

            standings, pairings = run_tournament(
                config, teams, rounds, START_JITTER_M if backend_name == "kinematic" else 0.0,
                base_seed=random.randrange(10_000), self_play=False, on_start=on_start, on_match=on_match,
                cancel=self.cancel, record_dir=record_dir,
                run_kwargs={
                    "backend_name": backend_name, "fast": fast, "visualize": False, "visualizer": visualizer,
                    "final_hold_s": hold_s if visualize else 0.0, "speedup": speedup, "backend": backend,
                    "on_event": lambda line: self.events.put(("log", "   " + line)),
                },
            )
        finally:
            if visualizer is not None:
                visualizer.close()
            if backend is not None:
                backend.shutdown()
        _save_results(record_dir, teams, standings, pairings, records)
        self.events.put(("tournament_done", (teams, standings, pairings, records, record_dir,
                                             self.cancel.is_set())))

    # ------------------------------------------------------------------ SITL
    def _start_sitl(self) -> None:
        if self.sitl_process is not None and self.sitl_process.poll() is None:
            self.log("SITL już działa.")
            return
        env = dict(os.environ)
        if self.format_var.get() == "5v5":
            env["ARENA_CONFIG"] = str(CONFIG_5V5)
        script = PACKAGE_DIR / "start_arena.sh"
        self.sitl_process = subprocess.Popen(
            ["bash", str(script)], cwd=REPO_DIR, env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
        )
        threading.Thread(target=self._pipe_output, args=(self.sitl_process,), daemon=True).start()
        self.log(f"Uruchamiam SITL ({self.format_var.get()}). Poczekaj, aż drony pojawią się na mapie.")

    def _stop_sitl(self) -> None:
        subprocess.Popen(["bash", str(PACKAGE_DIR / "stop_arena.sh")], cwd=REPO_DIR)
        self.log("Zatrzymuję SITL.")

    def _pipe_output(self, process: subprocess.Popen[str]) -> None:
        assert process.stdout is not None
        for line in process.stdout:
            self.events.put(("log", "[SITL] " + line.rstrip()))

    # ------------------------------------------------------------------ messages from workers
    def _poll(self) -> None:
        try:
            while True:
                kind, payload = self.events.get_nowait()
                self._handle(kind, payload)
        except queue.Empty:
            pass
        self.root.after(100, self._poll)

    def _handle(self, kind: str, payload: Any) -> None:
        if kind == "log":
            self.log(payload)
        elif kind == "status":
            self.status_var.set(payload)
            self.log(payload)
        elif kind == "progress":
            done, total = payload
            self.progress.configure(maximum=max(total, 1), value=done)
        elif kind == "checked":
            key, check = payload
            if key in self.entries:
                self.checks[key] = check
                self._render_list()
                if not check.ok:
                    self.log(f"{self.entries[key].label}: {check.summary()}")
        elif kind == "match":
            record: MatchRecord = payload
            if record.winner == "error":
                self.log(f"   błąd meczu: {record.reason}")
            else:
                attacked = record.winner == "attackers"
                winner = self._label(record.attacker if attacked else record.defender)
                side = "ATAK" if attacked else "OBRONA"
                self.log(f"   wygrywa {side} ({winner}), {record.reason}, t = {record.time:.1f} s")
        elif kind == "single_done":
            attacker, defender, result, record = payload
            self._report_single(attacker, defender, result, record)
        elif kind == "tournament_done":
            teams, standings, pairings, records, record_dir, cancelled = payload
            self.status_var.set("Turniej przerwany – wyniki częściowe." if cancelled else "Turniej zakończony.")
            ResultsWindow(self.root, teams, standings, pairings, records, record_dir, cancelled)
        elif kind == "error":
            self.status_var.set("Błąd: " + payload)
            self.log("BŁĄD: " + payload)
            messagebox.showerror("Arena", payload)
        elif kind == "idle":
            self._busy(False)
            if self.status_var.get().startswith(("Walka", "Mecz", "Przerywanie")):
                self.status_var.set("Gotowe.")

    def _report_single(self, attacker: TeamEntry, defender: TeamEntry, result: MatchResult, record: Path) -> None:
        winner = attacker if result.winner == "attackers" else defender
        role = "atak" if result.winner == "attackers" else "obrona"
        text = f"Wygrywa {winner.label} ({role}): {result.reason.value}, t = {result.time:.1f} s"
        self.status_var.set(text)
        self.log(text + f"   [nagranie: {record.name}]")

    def _label(self, key: str) -> str:
        entry = self.entries.get(key)
        return entry.label if entry else key

    def log(self, message: str) -> None:
        self.log_text.configure(state="normal")
        self.log_text.insert("end", f"{datetime.now():%H:%M:%S}  {message}\n")
        self.log_text.see("end")
        self.log_text.configure(state="disabled")

    def _open_results_dir(self) -> None:
        RESULTS_DIR.mkdir(parents=True, exist_ok=True)
        _open_path(RESULTS_DIR)

    def _on_close(self) -> None:
        if self.worker is not None and self.worker.is_alive():
            if not messagebox.askyesno("Arena", "Trwa walka. Przerwać i zamknąć?"):
                return
            self.cancel.set()
            self.worker.join(timeout=10.0)
        self.validator.shutdown(wait=False, cancel_futures=True)
        self.root.destroy()


class ResultsWindow:
    """Final standings, attacker win matrix and the list of matches (double-click = replay)."""

    def __init__(self, master: tk.Misc, teams: list[TeamEntry], standings: list[Standing],
                 pairings: dict[tuple[str, str], PairingStats], records: list[MatchRecord], record_dir: Path,
                 cancelled: bool) -> None:
        self.labels = {t.key: t.label for t in teams}
        self.records = records
        self.window = tk.Toplevel(master)
        self.window.title("Wyniki turnieju" + (" (przerwany)" if cancelled else ""))
        self.window.geometry("1000x720")
        self.record_dir = record_dir

        header = ttk.Frame(self.window, padding=(10, 10, 10, 0))
        header.pack(fill="x")
        played = sum(1 for r in records if r.winner != "error")
        ttk.Label(header, text=f"Mecze: {played}" + ("   •   TURNIEJ PRZERWANY" if cancelled else ""),
                  font=("TkDefaultFont", 11, "bold")).pack(side="left")
        ttk.Button(header, text="Otwórz katalog wyników", command=lambda: _open_path(record_dir)).pack(side="right")

        notebook = ttk.Notebook(self.window)
        notebook.pack(fill="both", expand=True, padx=10, pady=10)

        table = ttk.Frame(notebook, padding=6)
        notebook.add(table, text="Tabela")
        columns = ("place", "team", "points", "attacks", "defences", "attack_time")
        tree = ttk.Treeview(table, columns=columns, show="headings")
        for column, title, width in (("place", "#", 40), ("team", "Drużyna", 320), ("points", "Punkty", 80),
                                     ("attacks", "Wygrane ataki", 120), ("defences", "Wygrane obrony", 120),
                                     ("attack_time", "Śr. czas wygranego ataku", 180)):
            tree.heading(column, text=title)
            tree.column(column, width=width, anchor="w" if column == "team" else "center")
        tree.tag_configure("gold", background="#fff3c4")
        for place, s in enumerate(standings, start=1):
            mean_attack = f"{s.attack_time / s.attacks_won:.1f} s" if s.attacks_won else "–"
            tree.insert("", "end", values=(place, self.labels.get(s.team, s.team), s.points,
                                           f"{s.attacks_won}/{s.attacks_played}",
                                           f"{s.defences_won}/{s.defences_played}", mean_attack),
                        tags=("gold",) if place == 1 else ())
        tree.pack(fill="both", expand=True)
        ttk.Label(table, text="3 pkt za każdy wygrany mecz (w ataku lub w obronie). Remis punktowy rozstrzyga "
                              "krótszy łączny czas wygranych ataków, potem dłuższe przetrwanie przegranych obron.",
                  wraplength=900, foreground="#555555").pack(anchor="w", pady=(6, 0))

        matrix = ttk.Frame(notebook, padding=6)
        notebook.add(matrix, text="Macierz wyników")
        order = [s.team for s in standings]
        matrix_tree = ttk.Treeview(matrix, columns=["attacker"] + order, show="headings")
        matrix_tree.heading("attacker", text="Atak ↓  /  Obrona →")
        matrix_tree.column("attacker", width=220, anchor="w")
        for key in order:
            matrix_tree.heading(key, text=self.labels.get(key, key))
            matrix_tree.column(key, width=110, anchor="center")
        for attacker in order:
            cells = []
            for defender in order:
                stats = pairings.get((attacker, defender))
                cells.append("–" if stats is None else f"{stats.attacker_wins}/{stats.matches}")
            matrix_tree.insert("", "end", values=[self.labels.get(attacker, attacker)] + cells)
        matrix_tree.pack(fill="both", expand=True)
        ttk.Label(matrix, text="Komórka = ile meczów wygrał atak drużyny z wiersza przeciw obronie drużyny z kolumny.",
                  foreground="#555555").pack(anchor="w", pady=(6, 0))

        matches = ttk.Frame(notebook, padding=6)
        notebook.add(matches, text="Mecze (dwuklik = powtórka)")
        columns = ("n", "attacker", "defender", "round", "winner", "reason", "time")
        self.match_tree = ttk.Treeview(matches, columns=columns, show="headings")
        for column, title, width in (("n", "#", 40), ("attacker", "Atak", 220), ("defender", "Obrona", 220),
                                     ("round", "Runda", 60), ("winner", "Wygrywa", 160),
                                     ("reason", "Powód", 180), ("time", "Czas", 70)):
            self.match_tree.heading(column, text=title)
            self.match_tree.column(column, width=width, anchor="w" if column in ("attacker", "defender") else "center")
        for record in records:
            if record.winner == "error":
                winner = "błąd"
            else:
                winner_key = record.attacker if record.winner == "attackers" else record.defender
                role = "atak" if record.winner == "attackers" else "obrona"
                winner = f"{self.labels.get(winner_key, winner_key)} ({role})"
            self.match_tree.insert("", "end", iid=str(record.number), values=(
                record.number, self.labels.get(record.attacker, record.attacker),
                self.labels.get(record.defender, record.defender), record.round + 1, winner, record.reason,
                f"{record.time:.1f} s"))
        self.match_tree.bind("<Double-1>", self._replay)
        scroll = ttk.Scrollbar(matches, orient="vertical", command=self.match_tree.yview)
        self.match_tree.configure(yscrollcommand=scroll.set)
        self.match_tree.pack(side="left", fill="both", expand=True)
        scroll.pack(side="right", fill="y")
        self.window.lift()
        self.window.focus_force()

    def _replay(self, _event: tk.Event) -> None:
        selected = self.match_tree.focus()
        record = next((r for r in self.records if str(r.number) == selected), None)
        if record is None or not record.record_path or not Path(record.record_path).exists():
            return
        subprocess.Popen([sys.executable, "-m", "pipeline.drones_battle.replay", record.record_path], cwd=REPO_DIR)


# ---------------------------------------------------------------------- helpers
def _save_results(record_dir: Path, teams: list[TeamEntry], standings: list[Standing],
                  pairings: dict[tuple[str, str], PairingStats], records: list[MatchRecord]) -> None:
    labels = {t.key: t.label for t in teams}
    relabelled = [Standing(labels.get(s.team, s.team), s.points, s.attacks_won, s.attacks_played, s.defences_won,
                           s.defences_played, s.attack_time, s.defence_time) for s in standings]
    relabelled_pairs = {(labels.get(a, a), labels.get(d, d)): v for (a, d), v in pairings.items()}
    (record_dir / "wyniki.md").write_text(format_report(relabelled, relabelled_pairs) + "\n", encoding="utf-8")
    with (record_dir / "tabela.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(["miejsce", "druzyna", "punkty", "wygrane_ataki", "ataki", "wygrane_obrony", "obrony"])
        for place, s in enumerate(relabelled, start=1):
            writer.writerow([place, s.team, s.points, s.attacks_won, s.attacks_played, s.defences_won,
                             s.defences_played])
    with (record_dir / "mecze.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(["nr", "atak", "obrona", "runda", "zwyciezca", "powod", "czas_s", "nagranie"])
        for r in records:
            writer.writerow([r.number, labels.get(r.attacker, r.attacker), labels.get(r.defender, r.defender),
                             r.round + 1, r.winner, r.reason, f"{r.time:.1f}", r.record_path or ""])


def _text_dialog(master: tk.Misc, title: str, text: str) -> None:
    dialog = tk.Toplevel(master)
    dialog.title(title)
    dialog.geometry("760x420")
    widget = tk.Text(dialog, wrap="word", font=("TkFixedFont", 9))
    widget.insert("1.0", text)
    widget.configure(state="disabled")
    widget.pack(fill="both", expand=True, padx=8, pady=8)
    ttk.Button(dialog, text="Zamknij", command=dialog.destroy).pack(pady=(0, 8))


def _open_path(path: Path) -> None:
    path.mkdir(parents=True, exist_ok=True)
    if sys.platform == "win32":
        os.startfile(path)  # type: ignore[attr-defined]  # noqa: S606
    elif sys.platform == "darwin":
        subprocess.Popen(["open", str(path)])
    else:
        subprocess.Popen(["xdg-open", str(path)])


def _safe(text: str) -> str:
    return "".join(c if c.isalnum() or c in "-_" else "_" for c in text)


def main(argv: Optional[list[str]] = None) -> None:
    parser = argparse.ArgumentParser(description="Drone battle arena GUI.")
    parser.add_argument("--dir", default=str(DEFAULT_SUBMISSIONS_DIR), help="folder with student team folders")
    args = parser.parse_args(argv)
    root = tk.Tk()
    ArenaApp(root, Path(args.dir))
    root.mainloop()


if __name__ == "__main__":
    main()
