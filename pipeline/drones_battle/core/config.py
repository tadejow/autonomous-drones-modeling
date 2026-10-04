"""Single source of truth for every number of the game.

All values have defaults matching the implementation plan; ``arena_config.toml``
overrides them section by section. ``start_arena.sh`` reads the same file
(through :mod:`pipeline.drones_battle.core.layout`), so SITL start positions
and the orchestrator can never disagree.
"""

from __future__ import annotations

import dataclasses
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Literal

from pipeline.math.geodesy import GeoOrigin

KillMode = Literal["land", "freefall"]
ConnectionMode = Literal["udp", "tcp"]

DEFAULT_CONFIG_PATH = Path(__file__).resolve().parent.parent / "arena_config.toml"


@dataclass(frozen=True)
class ArenaSection:
    """Geometry of the battlefield. The NED origin is the defenders' base."""

    origin_lat_deg: float = -35.36335
    origin_lon_deg: float = 149.16500
    ground_alt_amsl_m: float = 584.0
    attacker_start_north_m: float = 150.0
    start_spacing_m: float = 5.0
    takeoff_alt_m: float = 15.0
    north_min_m: float = -40.0
    north_max_m: float = 210.0
    east_min_m: float = -60.0
    east_max_m: float = 60.0
    alt_min_m: float = 3.0
    alt_max_m: float = 40.0

    @property
    def origin(self) -> GeoOrigin:
        return GeoOrigin(self.origin_lat_deg, self.origin_lon_deg, self.ground_alt_amsl_m)


@dataclass(frozen=True)
class GameSection:
    """Rules of the match."""

    attacker_ids: tuple[int, ...] = (1, 2, 3)
    defender_ids: tuple[int, ...] = (4, 5, 6)
    tick_hz: float = 10.0
    max_time_s: float = 120.0
    countdown_s: float = 3.0
    kill_radius_m: float = 2.0
    target_radius_m: float = 5.0
    target_alt_m: float = 10.0
    defender_exclusion_radius_m: float = 8.0
    kill_mode: KillMode = "land"
    mutual_kill: bool = False

    @property
    def dt(self) -> float:
        return 1.0 / self.tick_hz

    @property
    def target_ned(self) -> tuple[float, float, float]:
        return (0.0, 0.0, -self.target_alt_m)


@dataclass(frozen=True)
class SafetySection:
    """Limits applied to every command returned by student code."""

    attacker_max_speed_mps: float = 10.0
    defender_max_speed_mps: float = 10.0
    max_vertical_speed_mps: float = 3.0
    fence_margin_m: float = 5.0
    fence_gain: float = 1.0
    fence_return_speed_mps: float = 2.0

    def max_speed_for(self, team: str) -> float:
        return self.attacker_max_speed_mps if team == "attackers" else self.defender_max_speed_mps


@dataclass(frozen=True)
class SandboxSection:
    """How student code is executed."""

    isolation: Literal["process", "inline"] = "process"
    tick_budget_s: float = 0.03
    max_late_s: float = 3.0
    max_restarts: int = 3


@dataclass(frozen=True)
class SitlSection:
    """Connection details of the six ArduPilot SITL instances."""

    connection_mode: ConnectionMode = "udp"
    udp_base_port: int = 14550
    map_udp_base_port: int = 14650
    tcp_base_port: int = 5760
    port_step: int = 10
    connect_timeout_s: float = 60.0
    takeoff_timeout_s: float = 90.0
    stale_after_s: float = 1.0
    position_rate_hz: float = 10.0
    wind_speed_mps: float = 0.0
    wind_direction_deg: float = 0.0
    end_mode: Literal["LAND", "RTL"] = "LAND"


@dataclass(frozen=True)
class KinematicSection:
    """First-order velocity model used by the SITL-free backend."""

    tau_s: float = 0.6
    max_accel_mps2: float = 6.0
    position_noise_std_m: float = 0.0
    land_speed_mps: float = 1.5
    seed: int = 0


@dataclass(frozen=True)
class VisualizationSection:
    trail_length: int = 300
    pause_s: float = 0.05
    topdown: bool = False


@dataclass(frozen=True)
class ArenaConfig:
    arena: ArenaSection = field(default_factory=ArenaSection)
    game: GameSection = field(default_factory=GameSection)
    safety: SafetySection = field(default_factory=SafetySection)
    sandbox: SandboxSection = field(default_factory=SandboxSection)
    sitl: SitlSection = field(default_factory=SitlSection)
    kinematic: KinematicSection = field(default_factory=KinematicSection)
    visualization: VisualizationSection = field(default_factory=VisualizationSection)

    def team_of(self, drone_id: int) -> str:
        if drone_id in self.game.attacker_ids:
            return "attackers"
        if drone_id in self.game.defender_ids:
            return "defenders"
        raise KeyError(f"Unknown drone id: {drone_id}")

    @property
    def all_ids(self) -> tuple[int, ...]:
        return self.game.attacker_ids + self.game.defender_ids

    def with_overrides(self, **sections: dict[str, Any]) -> ArenaConfig:
        """Returns a copy with selected fields replaced, e.g. ``game={"mutual_kill": True}``."""
        return _apply(self, sections)

    def as_dict(self) -> dict[str, Any]:
        return dataclasses.asdict(self)


def _apply(config: ArenaConfig, raw: dict[str, Any]) -> ArenaConfig:
    replacements: dict[str, Any] = {}
    for section_name, values in raw.items():
        if not hasattr(config, section_name):
            raise KeyError(f"Unknown config section [{section_name}]")
        section = getattr(config, section_name)
        known = {f.name for f in dataclasses.fields(section)}
        unknown = set(values) - known
        if unknown:
            raise KeyError(f"Unknown keys in [{section_name}]: {sorted(unknown)}")
        coerced = {k: tuple(v) if isinstance(v, list) else v for k, v in values.items()}
        replacements[section_name] = dataclasses.replace(section, **coerced)
    return dataclasses.replace(config, **replacements)


def _load_toml(path: Path) -> dict[str, Any]:
    try:
        import tomllib  # Python >= 3.11
    except ModuleNotFoundError:
        try:
            import tomli as tomllib  # type: ignore[no-redef]
        except ModuleNotFoundError as exc:
            raise RuntimeError(
                "Reading TOML needs Python >= 3.11 or 'pip install tomli'."
            ) from exc
    with path.open("rb") as handle:
        return tomllib.load(handle)


def load_config(path: str | Path | None = None) -> ArenaConfig:
    """Loads ``arena_config.toml`` (or the given file) on top of the defaults."""
    config_path = Path(path) if path is not None else DEFAULT_CONFIG_PATH
    if path is None and not config_path.exists():
        return ArenaConfig()
    return _apply(ArenaConfig(), _load_toml(config_path))
