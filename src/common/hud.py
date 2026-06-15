import sys
import time
import numpy as np
from typing import Dict, Any, Optional, List

from rich.console import Console, Group
from rich.live import Live
from rich.panel import Panel
from rich.table import Table
from rich.text import Text
from rich.progress_bar import ProgressBar
from rich import box

import src.config as config


class HudDisplay:
    def __init__(self, num_engines: int) -> None:
        self.num_engines: int = num_engines
        self._console: Console = Console()
        self._is_tty: bool = self._console.is_terminal
        self._last_render: float = 0.0
        self._hud_interval: float = 1.0 / config.HUD_REFRESH_HZ
        self._mission_start: float = time.time()
        self._live: Optional[Live] = None
        self._data: Dict[str, Any] = {
            "state": "STANDBY",
            "altitude": 0.0,
            "est_alt": 0.0,
            "vertical_vel": 0.0,
            "lateral_vel": np.zeros(3),
            "position": np.zeros(3),
            "throttles": np.zeros(num_engines),
            "gimbals_deg": np.zeros((num_engines, 2)),
            "fuel_state": np.ones(num_engines),
            "fdi_deviation": 0.0,
            "fdi_threshold": config.FDI_THRESHOLD,
            "alloc_cond": 0.0,
            "saturated": set(),
            "kf_cov_pos": 0.0,
            "kf_cov_vel": 0.0,
            "dt_spike_count": 0,
            "skip_predict": False,
            "active_engine_count": 0,
            "total_engine_count": num_engines,
            "mass": 0.0,
            "a_avail": 0.0,
            "angular_velocity_mag": 0.0,
            "events": [],
        }

    def start(self) -> None:
        if not config.HUD_ENABLED or not self._is_tty:
            return
        self._live = Live(
            console=self._console,
            refresh_per_second=config.HUD_REFRESH_HZ,
            vertical_overflow="visible",
        )
        self._live.__enter__()

    def stop(self) -> None:
        if self._live is not None:
            self._live.__exit__(None, None, None)
            self._live = None

    def update(self, data: Dict[str, Any]) -> None:
        self._data.update(data)
        if self._live is None:
            return
        now = time.time()
        if (now - self._last_render) < self._hud_interval:
            return
        self._last_render = now
        self._live.update(self._build_display())

    def _build_display(self) -> Group:
        d = self._data
        elapsed = time.time() - self._mission_start
        t_str = f"T+{elapsed:.1f}s"

        state_color = self._state_color(d["state"])
        state_text = Text(d["state"], style=state_color)

        header = Table.grid(padding=(0, 1))
        header.add_column(justify="left")
        header.add_column(justify="center")
        header.add_column(justify="right")
        header.add_row(
            Text("AEGIS MISSION DIRECTOR", style="bold cyan"),
            Text("│ ", style="dim").append(state_text).append(" │"),
            Text(t_str, style="bold white"),
        )

        kinematics = Table.grid(padding=(0, 2))
        kinematics.add_column()
        kinematics.add_column()
        vel_color = "green" if abs(d["vertical_vel"]) < 50 else ("yellow" if abs(d["vertical_vel"]) < 200 else "red")
        vert_vel_str = f"{d['vertical_vel']:+.1f} m/s"
        alt_str = f"{d['est_alt']:.1f} m"
        mass_str = f"{d['mass']:.0f} kg"
        a_avail_str = f"{d['a_avail']:.1f} m/s²"
        lat_vel = d["lateral_vel"]
        lat_speed = float(np.linalg.norm(lat_vel))
        lat_color = "green" if lat_speed < 5 else ("yellow" if lat_speed < 20 else "red")
        kinematics.add_row(
            f"Alt: [{vel_color}]{alt_str}[/{vel_color}]   Vert Vel: [{vel_color}]{vert_vel_str}[/{vel_color}]   Mass: {mass_str}   a_avail: {a_avail_str}",
            f"Lat Speed: [{lat_color}]{lat_speed:.1f} m/s[/{lat_color}]   ω: {d['angular_velocity_mag']:.2f} rad/s",
        )

        pos = d["position"]
        kinematics.add_row(
            f"Est Pos: [{pos[0]:.1f}, {pos[1]:.1f}, {pos[2]:.1f}]",
            "",
        )

        throttle_table = Table(
            box=box.SIMPLE,
            show_header=True,
            header_style="bold",
            padding=(0, 1),
            expand=False,
        )
        throttle_table.add_column("Eng", justify="center", width=3)
        throttle_table.add_column("Thr%", justify="center", width=5)
        throttle_table.add_column("Gimbal", justify="center", width=10)
        throttle_table.add_column("Fuel", justify="left", width=12)
        throttle_table.add_column("Status", justify="center", width=7)

        gimbals_deg = d["gimbals_deg"]
        fuel_state = d["fuel_state"]
        throttles = d["throttles"]
        saturated = d["saturated"]

        num = self.num_engines
        for i in range(num):
            thr = float(throttles[i]) if i < len(throttles) else 0.0
            thr_pct = f"{thr * 100:.0f}"
            thr_style = "green" if thr < 0.8 else ("yellow" if thr < 1.0 else "red bold")

            if i < gimbals_deg.shape[0]:
                gx = float(gimbals_deg[i, 0])
                gy = float(gimbals_deg[i, 1])
                gim_str = f"{gx:+.1f}° {gy:+.1f}°"
            else:
                gim_str = "  N/A"

            fuel = float(fuel_state[i]) if i < len(fuel_state) else 0.0
            fuel_bar = self._fuel_bar(fuel)

            if i in saturated:
                status = Text("SAT", style="red bold")
            elif i >= len(throttles) or thr < 1e-6 and fuel < 0.5:
                status = Text("OFF", style="dim")
            else:
                status = Text("OK", style="green")

            throttle_table.add_row(
                str(i),
                Text(thr_pct, style=thr_style),
                gim_str,
                fuel_bar,
                status,
            )

        fdi_dev = d["fdi_deviation"]
        fdi_thresh = d["fdi_threshold"]
        fdi_style = "green" if fdi_dev < fdi_thresh * 0.5 else ("yellow" if fdi_dev < fdi_thresh else "red bold")
        fdi_str = f"dev={fdi_dev:.2f}/{fdi_thresh:.1f}"

        alloc_cond = d["alloc_cond"]
        alloc_style = "green" if alloc_cond < 1e3 else ("yellow" if alloc_cond < 1e4 else "red bold")
        alloc_str = f"cond={alloc_cond:.0f}"

        active_count = d["active_engine_count"]
        total_count = d["total_engine_count"]

        health = Table.grid(padding=(0, 2))
        health.add_column()
        health.add_column()
        health.add_column()
        health.add_row(
            Text(f"FDI: [{fdi_str}]", style=fdi_style),
            Text(f"Alloc: [{alloc_str}]", style=alloc_style),
            f"Engines: {active_count}/{total_count} active",
        )

        cov_pos = d["kf_cov_pos"]
        cov_vel = d["kf_cov_vel"]
        spikes = d["dt_spike_count"]
        skip = d["skip_predict"]
        health_row2 = Table.grid(padding=(0, 2))
        health_row2.add_column()
        health_row2.add_column()
        health_row2.add_column()
        health_row2.add_row(
            f"KF P₃₃={cov_pos:.1f}m  P₆₆={cov_vel:.1f}m/s",
            f"dt_spikes: {spikes}",
            Text(f"skip_pred: {skip}", style="yellow" if skip else ""),
        )

        body = Group(
            header,
            Text(""),
            kinematics,
            Text(""),
            throttle_table,
            Text(""),
            health,
            health_row2,
        )

        return Group(
            Panel(
                body,
                title="[bold cyan]AEGIS[/]",
                border_style="cyan",
                box=box.ROUNDED,
                padding=(0, 1),
            )
        )

    def _state_color(self, state: str) -> str:
        colors: Dict[str, str] = {
            "STANDBY": "dim white",
            "ASCENT_COAST": "blue",
            "DEORBIT_BURN": "magenta",
            "HYPERSONIC_COAST": "cyan",
            "POWERED_DESCENT": "yellow",
            "HOVER_TARGETING": "bright_yellow",
            "TERMINAL_DESCENT": "bright_green",
            "LANDED": "bold green",
            "HARD_ABORT": "bold red",
        }
        return colors.get(state, "white")

    def _fuel_bar(self, fuel: float, width: int = 8) -> str:
        filled = int(fuel * width)
        empty = width - filled
        if fuel > 0.5:
            style = "green"
        elif fuel > 0.2:
            style = "yellow"
        else:
            style = "red"
        return f"[{style}]{'█' * filled}{'░' * empty}[/{style}]"

    @property
    def is_active(self) -> bool:
        return self._live is not None
