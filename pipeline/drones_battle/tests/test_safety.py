import math

import numpy as np
import pytest

from pipeline.drones_battle.core.config import ArenaConfig
from pipeline.drones_battle.core.safety import SafetyLimiter

MID_AIR = np.array([80.0, 0.0, -15.0])


@pytest.fixture()
def limiter() -> SafetyLimiter:
    return SafetyLimiter(ArenaConfig())


@pytest.mark.parametrize("raw", [None, "fast", (1.0, 2.0), (math.nan, 0.0, 0.0), (math.inf, 0, 0), {"a": 1}])
def test_invalid_commands_hover(limiter: SafetyLimiter, raw: object) -> None:
    assert limiter.apply(1, raw, MID_AIR) == (0.0, 0.0, 0.0)
    assert limiter.interventions[1]["invalid"] == 1


def test_speed_limit_keeps_direction(limiter: SafetyLimiter) -> None:
    v = limiter.apply(1, (30.0, 40.0, 0.0), MID_AIR)
    assert math.hypot(v[0], v[1]) == pytest.approx(10.0)
    assert v[1] / v[0] == pytest.approx(40.0 / 30.0)


def test_vertical_limit(limiter: SafetyLimiter) -> None:
    assert limiter.apply(1, (0.0, 0.0, 8.0), MID_AIR)[2] == pytest.approx(3.0)


def test_valid_command_passes_unchanged(limiter: SafetyLimiter) -> None:
    assert limiter.apply(2, [3, -4, 1], MID_AIR) == (3.0, -4.0, 1.0)
    assert not limiter.interventions[2]


def test_floor_forces_climb(limiter: SafetyLimiter) -> None:
    v = limiter.apply(1, (0.0, 0.0, 3.0), np.array([80.0, 0.0, -1.0]))  # 1 m above ground, diving
    assert v[2] < 0  # NED: negative = climbing


def test_wall_slows_down_and_pushes_back(limiter: SafetyLimiter) -> None:
    near = limiter.apply(1, (0.0, 10.0, 0.0), np.array([80.0, 58.0, -15.0]))
    assert near[1] == pytest.approx(2.0)  # gain 1.0 * 2 m to the wall
    outside = limiter.apply(1, (0.0, 10.0, 0.0), np.array([80.0, 65.0, -15.0]))
    assert outside[1] < 0


def test_defender_pushed_out_of_exclusion_sphere(limiter: SafetyLimiter) -> None:
    inside = np.array([0.0, 4.0, -10.0])  # 4 m from the target, radius 8 m
    v = limiter.apply(4, (0.0, -5.0, 0.0), inside)  # trying to move to the centre
    assert v[1] > 0
    # Attackers are not affected by the exclusion sphere.
    assert limiter.apply(1, (0.0, -5.0, 0.0), inside) == (0.0, -5.0, 0.0)


def test_team_speed_limits() -> None:
    limiter = SafetyLimiter(ArenaConfig().with_overrides(safety={"defender_max_speed_mps": 8.0}))
    assert np.linalg.norm(limiter.apply(4, (20.0, 0.0, 0.0), MID_AIR)) == pytest.approx(8.0)
    assert np.linalg.norm(limiter.apply(1, (20.0, 0.0, 0.0), MID_AIR)) == pytest.approx(10.0)
