import time
from pathlib import Path

import pytest

from pipeline.drones_battle.core.sandbox import InlineController, ProcessController, StrategyCall

STRATEGIES = Path(__file__).resolve().parent / "strategies"
MY_TEAM = {
    1: {"pos": (150.0, 0.0, -15.0), "vel": (0.0, 0.0, 0.0), "alive": True},
    2: {"pos": (150.0, 5.0, -15.0), "vel": (0.0, 0.0, 0.0), "alive": True},
}
ENEMY = {4: {"pos": (0.0, 0.0, -15.0), "vel": (0.0, 0.0, 0.0), "alive": True}}


def _call(t: float) -> StrategyCall:
    return StrategyCall(MY_TEAM, ENEMY, (0.0, 0.0, -10.0), t, {"max_speed": 10.0})


def _ask(controller, t: float, budget: float = 0.5):
    controller.submit(_call(t))
    return controller.collect(time.monotonic() + budget)


def test_inline_exception_means_hover() -> None:
    controller = InlineController("attackers", str(STRATEGIES / "crashing.py"), (1, 2))
    assert _ask(controller, 0.0) == {}
    assert "ZeroDivisionError" in controller.errors[0]


def test_inline_filters_foreign_drones() -> None:
    controller = InlineController("attackers", str(STRATEGIES / "garbage.py"), (1, 2))
    commands = _ask(controller, 0.0)
    assert set(commands) == {1, 2}
    assert any("99" in e for e in controller.errors)


def test_game_info_is_passed_when_requested() -> None:
    controller = InlineController("attackers", str(STRATEGIES / "with_game.py"), (1, 2))
    assert _ask(controller, 0.0) == {1: (0.0, 0.0, 0.0), 2: (0.0, 0.0, 0.0)}
    assert not controller.errors


def test_dotted_module_reference() -> None:
    controller = InlineController("attackers", "pipeline.drones_battle.team_attacker", (1, 2))
    commands = _ask(controller, 0.0)
    assert set(commands) == {1, 2}


def test_missing_function_is_reported() -> None:
    with pytest.raises(AttributeError):
        InlineController("attackers", "pipeline.drones_battle.strategy_utils", (1, 2))


def test_process_survives_infinite_loop_and_restarts() -> None:
    controller = ProcessController("attackers", str(STRATEGIES / "hanging.py"), (1, 2), max_late_s=0.3,
                                   max_restarts=1)
    try:
        assert _ask(controller, 0.0) == {1: (1.0, 0.0, 0.0), 2: (1.0, 0.0, 0.0)}
        # From t > 1 s the strategy hangs: the last valid commands are reused.
        started = time.monotonic()
        assert _ask(controller, 2.0, budget=0.05) == {1: (1.0, 0.0, 0.0), 2: (1.0, 0.0, 0.0)}
        assert time.monotonic() - started < 0.5
        deadline = time.monotonic() + 20.0
        while not controller.forfeited and time.monotonic() < deadline:
            _ask(controller, 2.0, budget=0.05)
            time.sleep(0.05)
        assert controller.forfeited
        assert controller.stats()["restarts"] == 2
    finally:
        controller.close()


def test_process_exception_is_contained() -> None:
    controller = ProcessController("attackers", str(STRATEGIES / "crashing.py"), (1, 2))
    try:
        assert _ask(controller, 0.0) == {}
        assert controller.stats()["exceptions"] == 1
    finally:
        controller.close()
