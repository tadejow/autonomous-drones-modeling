"""Placement of the 3D window on the right half of the screen (no real window needed)."""

import pytest

from pipeline.drones_battle import arena_visualizer as viz


def test_right_half_of_work_area() -> None:
    # 1874 px wide work area below a 20 px panel: the right half starts at x = 937.
    assert viz._right_half((0, 20, 1874, 820)) == (937, 20, 937, 820 - viz.TITLE_BAR_PX)


def test_wmctrl_unmaximizes_then_moves(monkeypatch: pytest.MonkeyPatch) -> None:
    calls = []
    monkeypatch.setattr(viz.sys, "platform", "linux")
    monkeypatch.setattr(viz.shutil, "which", lambda name: "/usr/bin/wmctrl")
    monkeypatch.setattr(viz.subprocess, "run", lambda args, **kwargs: calls.append(args))
    viz._wmctrl_place("A vs B", 937, 20, 937, 788)
    assert calls == [
        ["wmctrl", "-F", "-r", "A vs B", "-b", "remove,maximized_vert,maximized_horz"],
        ["wmctrl", "-F", "-r", "A vs B", "-e", "0,937,20,937,788"],
    ]


def test_wmctrl_skipped_without_wmctrl_or_off_linux(monkeypatch: pytest.MonkeyPatch) -> None:
    calls = []
    monkeypatch.setattr(viz.subprocess, "run", lambda args, **kwargs: calls.append(args))
    monkeypatch.setattr(viz.sys, "platform", "win32")
    viz._wmctrl_place("A vs B", 1, 2, 3, 4)
    monkeypatch.setattr(viz.sys, "platform", "linux")
    monkeypatch.setattr(viz.shutil, "which", lambda name: None)
    viz._wmctrl_place("A vs B", 1, 2, 3, 4)
    assert calls == []
