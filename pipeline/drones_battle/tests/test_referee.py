import numpy as np
import pytest

from pipeline.drones_battle.core.config import ArenaConfig
from pipeline.drones_battle.core.referee import Referee, closest_approach
from pipeline.drones_battle.core.types import EventKind, RawState

A = np.array


def test_head_on_pass_between_samples_is_detected() -> None:
    # Relative speed 20 m/s at 10 Hz: the drones swap sides between two samples.
    d_min, s_star = closest_approach(A([0, 0, 0]), A([1, 0, 0]), A([1, 0, 0]), A([0, 0, 0]))
    assert d_min == pytest.approx(0.0)
    assert s_star == pytest.approx(0.5)
    # Point sampling would see 1 m at both ends; a 3 m swap would be missed entirely.
    d_min, _ = closest_approach(A([0, 0, 0]), A([3, 0, 0]), A([3, 0.5, 0]), A([0, 0.5, 0]))
    assert d_min == pytest.approx(0.5)


def test_parallel_flight_keeps_distance() -> None:
    d_min, _ = closest_approach(A([0, 0, 0]), A([1, 0, 0]), A([0, 2.1, 0]), A([1, 2.1, 0]))
    assert d_min == pytest.approx(2.1)


def test_static_drones() -> None:
    d_min, s_star = closest_approach(A([0, 0, 0]), A([0, 0, 0]), A([0, 3, 4]), A([0, 3, 4]))
    assert (d_min, s_star) == pytest.approx((5.0, 0.0))


def _states(positions: dict[int, tuple[float, float, float]]) -> dict[int, RawState]:
    return {i: RawState(np.array(p, dtype=float), np.zeros(3)) for i, p in positions.items()}


def _far_layout(**overrides: tuple[float, float, float]) -> dict[int, tuple[float, float, float]]:
    layout = {1: (150.0, -5.0, -15.0), 2: (150.0, 0.0, -15.0), 3: (150.0, 5.0, -15.0),
              4: (30.0, -5.0, -15.0), 5: (30.0, 0.0, -15.0), 6: (30.0, 5.0, -15.0)}
    layout.update({int(k[1:]): v for k, v in overrides.items()})
    return layout


def test_hit_kills_attacker_only() -> None:
    referee = Referee(ArenaConfig())
    before = _states(_far_layout(d1=(80.0, 0.0, -15.0), d5=(79.0, 0.0, -15.0)))
    after = _states(_far_layout(d1=(79.0, 0.0, -15.0), d5=(80.0, 0.0, -15.0)))
    outcome = referee.evaluate(before, after, 10.0)
    assert [(e.kind, e.actor, e.victim) for e in outcome.events] == [(EventKind.HIT, 5, 1)]
    assert referee.alive[1] is False and referee.alive[5] is True
    assert outcome.result is None


def test_mutual_kill_option() -> None:
    referee = Referee(ArenaConfig().with_overrides(game={"mutual_kill": True}))
    before = _states(_far_layout(d1=(80.0, 0.0, -15.0), d5=(80.5, 0.0, -15.0)))
    outcome = referee.evaluate(before, before, 1.0)
    assert {e.victim for e in outcome.hits} == {1, 5}


def test_defender_inside_exclusion_sphere_cannot_shoot() -> None:
    referee = Referee(ArenaConfig())
    layout = _states(_far_layout(d1=(3.0, 0.0, -10.0), d5=(4.0, 0.0, -10.0)))
    outcome = referee.evaluate(layout, layout, 5.0)
    assert not outcome.hits
    assert outcome.result is not None and outcome.result.winner == "attackers"


def test_simultaneous_hit_and_arrival_goes_to_defenders() -> None:
    referee = Referee(ArenaConfig().with_overrides(game={"defender_exclusion_radius_m": 0.0}))
    layout = _states(_far_layout(d1=(0.0, 0.0, -10.0), d5=(1.0, 0.0, -10.0)))
    outcome = referee.evaluate(layout, layout, 5.0)
    assert [e.victim for e in outcome.hits] == [1]
    assert outcome.result is None  # attackers 2 and 3 still fight
    assert not any(e.kind is EventKind.TARGET_REACHED for e in outcome.events)


def test_fast_attacker_cannot_tunnel_through_target() -> None:
    referee = Referee(ArenaConfig())
    before = _states(_far_layout(d1=(6.0, 0.0, -10.0)))
    after = _states(_far_layout(d1=(-6.0, 0.0, -10.0)))
    outcome = referee.evaluate(before, after, 3.0)
    assert outcome.result is not None and outcome.result.reason is EventKind.TARGET_REACHED


def test_all_down_and_timeout() -> None:
    config = ArenaConfig()
    referee = Referee(config)
    layout = _states(_far_layout())
    assert referee.evaluate(layout, layout, 1.0).result is None
    result = referee.evaluate(layout, layout, config.game.max_time_s).result
    assert result is not None and result.reason is EventKind.TIMEOUT and result.winner == "defenders"

    referee = Referee(config)
    for drone_id in config.game.attacker_ids:
        referee.alive[drone_id] = False
    result = referee.evaluate(layout, layout, 2.0).result
    assert result is not None and result.reason is EventKind.ALL_ATTACKERS_DOWN


def test_stale_drone_is_ignored() -> None:
    referee = Referee(ArenaConfig())
    layout = _states(_far_layout(d1=(80.0, 0.0, -15.0), d5=(80.5, 0.0, -15.0)))
    layout[5].stale = True
    assert not referee.evaluate(layout, layout, 1.0).hits
