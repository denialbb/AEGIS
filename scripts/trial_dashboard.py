"""
trial_dashboard.py

A minimal curses dashboard for long-running, trial-based searches (Optuna or
otherwise):

  - one progress bar tracking total steps completed / total steps
  - a fixed-size, non-scrolling panel showing the most recent trial results
    (redrawn in place every update -- nothing ever scrolls into terminal
    scrollback, the rows just rotate as new trials complete)

Usage
-----
    from trial_dashboard import TrialDashboard

    with TrialDashboard(total_steps=_N_TOTAL, history_rows=10,
                         title="EKF Hyperparameter Search") as dash:
        ... call dash.advance(1) once per flight ...
        ... call dash.report_trial(...) once per finished trial ...

Thread safety
-------------
advance() and report_trial() may be called from worker threads (e.g.
study.optimize(objective, n_jobs=4)). A single re-entrant lock guards both
the shared state *and* the actual curses draw calls, because curses windows
are not safe to write to from more than one thread at a time.

Platform note
-------------
The standard `curses` module ships with Python on Linux/macOS. On Windows
you need `pip install windows-curses`. This also needs a real terminal
(a TTY) -- it will not render inside most notebook/IDE consoles.
"""

from __future__ import annotations

import curses
import threading
import time
from dataclasses import dataclass
from typing import Any, Optional


@dataclass
class TrialRecord:
    trial_num: int
    score: float
    rmse_pos: float
    rmse_vel: float
    nis: float
    best_so_far: float
    elapsed: float


def _format_bar(fraction: float, width: int) -> str:
    """Render a textual progress bar. Pure function (no curses dependency),
    kept separate so it can be unit-tested without a real terminal."""
    fraction = max(0.0, min(1.0, fraction))
    width = max(width, 1)
    filled = int(round(width * fraction))
    return "#" * filled + "-" * (width - filled)


class TrialDashboard:
    def __init__(self, total_steps: int, history_rows: int = 8, title: str = "Trial Progress"):
        self.total_steps = max(total_steps, 1)
        self.history_rows = history_rows
        self.title = title

        self._completed = 0
        self._history: list[TrialRecord] = []
        self._lock = threading.RLock()
        self._stdscr: Optional[Any] = None
        self._started = False
        self._last_render = 0.0
        self._min_interval = 0.05  # seconds; throttles redraw rate during fast loops
        self._status_msg: str = ""

    # ---------------------------------------------------------------- lifecycle
    def start(self) -> "TrialDashboard":
        if self._started:
            return self
        self._stdscr = curses.initscr()
        curses.noecho()
        curses.cbreak()
        curses.curs_set(0)
        self._stdscr.nodelay(True)
        self._stdscr.keypad(True)
        try:
            curses.start_color()
            curses.use_default_colors()
            curses.init_pair(1, curses.COLOR_GREEN, -1)    # progress bar
            curses.init_pair(2, curses.COLOR_YELLOW, -1)   # newest result row
            curses.init_pair(3, curses.COLOR_CYAN, -1)     # title
        except curses.error:
            pass  # terminal doesn't support color; fall back to defaults
        self._started = True
        self._render(force=True)
        return self

    def stop(self) -> None:
        if not self._started:
            return
        curses.nocbreak()
        if self._stdscr is not None:
            self._stdscr.keypad(False)
        curses.echo()
        curses.endwin()
        self._started = False

    def __enter__(self) -> "TrialDashboard":
        return self.start()

    def __exit__(self, exc_type, exc, tb) -> None:
        self.stop()

    # ----------------------------------------------------------------- updates
    def status(self, msg: str) -> None:
        """Set a status message displayed below the title."""
        with self._lock:
            self._status_msg = msg
            self._render(force=True)

    def advance(self, n: int = 1) -> None:
        """Call once per unit of work (e.g. once per flight)."""
        with self._lock:
            self._completed += n
            self._render()

    def report_trial(
        self,
        trial_num: int,
        score: float,
        rmse_pos: float,
        rmse_vel: float,
        nis: float,
        best_so_far: float,
        elapsed: float,
    ) -> None:
        """Call once per finished trial. Pushes a new row into the fixed,
        non-scrolling results panel (oldest row drops off once full)."""
        rec = TrialRecord(trial_num, score, rmse_pos, rmse_vel, nis, best_so_far, elapsed)
        with self._lock:
            self._history.append(rec)
            if len(self._history) > self.history_rows:
                self._history.pop(0)
            self._render(force=True)  # always show a finished trial immediately

    # ---------------------------------------------------------------- drawing
    def _render(self, force: bool = False) -> None:
        if self._stdscr is None:
            return
        now = time.monotonic()
        if not force and (now - self._last_render) < self._min_interval:
            return
        self._last_render = now

        with self._lock:
            scr = self._stdscr
            try:
                ch = scr.getch()  # non-blocking; drains keypresses / resize events
                if ch == curses.KEY_RESIZE:
                    curses.update_lines_cols()
            except curses.error:
                pass

            max_y, max_x = scr.getmaxyx()
            scr.erase()

            try:
                scr.addnstr(0, 0, self.title.center(max_x), max_x, curses.A_BOLD | curses.color_pair(3))
            except curses.error:
                pass

            if self._status_msg:
                try:
                    scr.addnstr(1, 0, self._status_msg.ljust(max_x)[:max_x], max_x)
                except curses.error:
                    pass

            pct = self._completed / self.total_steps
            bar_width = max(max_x - 22, 10)
            bar = _format_bar(pct, bar_width)
            progress_line = f" [{bar}] {min(pct, 1.0) * 100:5.1f}%  ({self._completed}/{self.total_steps}) "
            try:
                scr.addnstr(2, 0, progress_line, max_x, curses.color_pair(1))
            except curses.error:
                pass

            header = (
                f"{'trial':>6} {'score':>10} {'rmse_pos':>10} {'rmse_vel':>10} "
                f"{'nis':>10} {'best':>10} {'time(s)':>8}"
            )
            try:
                scr.addnstr(4, 0, header, max_x, curses.A_UNDERLINE)
            except curses.error:
                pass

            recent = list(reversed(self._history))  # newest trial on top
            for row_idx in range(self.history_rows):
                y = 5 + row_idx
                if y >= max_y:
                    break
                line = ""
                attr = curses.A_NORMAL
                if row_idx < len(recent):
                    rec = recent[row_idx]
                    line = (
                        f"{rec.trial_num:>6} {rec.score:>10.4f} {rec.rmse_pos:>10.4f} "
                        f"{rec.rmse_vel:>10.4f} {rec.nis:>10.4f} {rec.best_so_far:>10.4f} "
                        f"{rec.elapsed:>8.2f}"
                    )
                    if row_idx == 0:
                        attr = curses.color_pair(2) | curses.A_BOLD
                try:
                    scr.addnstr(y, 0, line.ljust(max_x), max_x, attr)
                except curses.error:
                    pass

            scr.refresh()


def _demo() -> None:
    """Standalone preview -- run `python3 trial_dashboard.py` in a real
    terminal to see it without needing Optuna or any EKF code at all."""
    import random

    n_trials, n_flights = 14, 6
    total_steps = n_trials * n_flights
    best = float("inf")

    with TrialDashboard(total_steps=total_steps, history_rows=8,
                         title="EKF Hyperparameter Search (demo)") as dash:
        for trial_num in range(n_trials):
            t0 = time.perf_counter()
            for _ in range(n_flights):
                time.sleep(0.05)
                dash.advance(1)
            score = random.uniform(0.5, 5.0)
            best = min(best, score)
            dash.report_trial(
                trial_num=trial_num,
                score=score,
                rmse_pos=random.uniform(0.1, 2.0),
                rmse_vel=random.uniform(0.1, 2.0),
                nis=random.uniform(0.5, 3.0),
                best_so_far=best,
                elapsed=time.perf_counter() - t0,
            )
        time.sleep(1.5)  # let the final state sit on screen before exit


if __name__ == "__main__":
    _demo()
