"""TUI screen composition and command-key parsing, no tty required."""

import re

from pico_bimanual_franka_teleop import tui
from pico_bimanual_franka_teleop.xr_input import KeyboardActivation

_ANSI = re.compile(r"\x1b\[[0-9;?]*[A-Za-z]")


def _rows(payload: bytes) -> list[str]:
    return [_ANSI.sub("", row) for row in payload.decode().split("\r\n")]


def test_compose_screen_paints_every_row_without_scrolling():
    payload = tui.compose_screen("status", ["op"], ["log"], width=40, height=12)
    assert len(_rows(payload)) == 12
    # Writing past the last cell would scroll the terminal and smear panes.
    assert not payload.decode().endswith("\r\n")


def test_compose_screen_clips_to_the_terminal_width():
    long = "x" * 300
    rows = _rows(tui.compose_screen(long, [long], [long], width=40, height=14))
    assert all(len(row) <= 40 for row in rows)


def test_compose_screen_shows_the_latest_output():
    lines = [f"line{index:03d}" for index in range(100)]
    payload = tui.compose_screen("", [], lines, width=60, height=16).decode()
    assert "line099" in payload
    assert "line000" not in payload


def test_compose_screen_colors_alerts_and_dims_process_noise():
    payload = tui.compose_screen(
        "STATE | ok",
        ["right cannot engage: missing tracker", "reset done"],
        ["vendor noise"],
        width=80,
        height=14,
    ).decode()
    # Exactly one red row: the fault line, not the routine feedback.
    assert payload.count(tui.ALERT_SGR) == 1
    assert tui.ALERT_SGR + "right cannot engage" in payload
    assert tui.STATUS_SGR + "STATE | ok" in payload
    assert tui.PROCESS_SGR in payload


def test_command_keys_pass_plain_letters_through():
    keys = list(KeyboardActivation._command_keys(b"xRq "))
    assert keys == ["x", "r", "q", " "]


def test_command_keys_swallow_escape_sequences():
    # Home sends ESC [ H and must not alias onto the reset command.
    assert list(KeyboardActivation._command_keys(b"\x1b[H")) == []
    # Arrows, F-keys (CSI ~), SS3 Home, and a bare ESC are all dropped.
    assert list(
        KeyboardActivation._command_keys(b"\x1b[A\x1b[15~\x1bOHx")
    ) == ["x"]
    assert list(KeyboardActivation._command_keys(b"\x1b")) == []
