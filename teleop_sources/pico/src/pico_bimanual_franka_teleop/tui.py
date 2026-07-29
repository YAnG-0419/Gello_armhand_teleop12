"""Single-terminal TUI for the operator process.

Three fixed regions replace the interleaved scrollback: a status header fed
by the control loop, an operator pane showing only the feedback for keys the
operator pressed, and a process pane that captures everything anything
writes to stdout or stderr. The capture is at the file-descriptor level
because the native SDKs (XRoboToolkit, MANUS, CAN vendors) print from C code
that never passes through sys.stdout.

Rendering is one full-screen repaint per write, throttled inside poll(), so
the 100 Hz control loop pays at most a few kilobytes of tty output every
refresh interval. No curses and no new dependencies: this is the same raw
tty handling KeyboardActivation already does, plus ANSI cursor addressing.
"""

from __future__ import annotations

import os
import sys
import threading
import time
from collections import deque

from .types import SIDES
from .xr_input import KeyboardActivation

STATUS_ROWS = 2
HELP = "space/l/r=engage  x=stop  o=open hands  h=HOME (moves arms)  q=quit"

# One accent each: status stands out, frames and process noise recede, and a
# fault line is the only red thing on screen.
STATUS_SGR = "\x1b[1;36m"
FRAME_SGR = "\x1b[90m"
PROCESS_SGR = "\x1b[90m"
ALERT_SGR = "\x1b[31m"
RESET_SGR = "\x1b[0m"
ALERT_WORDS = (
    "FAILED",
    "cannot engage",
    "disabled",
    "lost",
    "stale",
    "frozen",
    "missing",
    "jumped",
    "moved backwards",
)


def _clip(text: str, width: int) -> str:
    return text if len(text) <= width else text[: width - 1] + "…"


def _operator_sgr(line: str) -> str:
    return ALERT_SGR if any(word in line for word in ALERT_WORDS) else ""


def _tail(lines, rows: int, width: int) -> list[str]:
    kept = [_clip(line, width) for line in list(lines)[-rows:]]
    return [""] * (rows - len(kept)) + kept


def compose_screen(
    status: str,
    operator_lines,
    process_lines,
    width: int,
    height: int,
) -> bytes:
    """Build one full-screen repaint as a single escape-coded payload. Pure."""
    width = max(20, int(width))
    height = max(10, int(height))
    # Split the body half and half between the operator and process panes.
    body = height - STATUS_ROWS - 2
    operator_rows = body // 2
    process_rows = body - operator_rows

    rows: list[tuple[str, str]] = []
    chunks = [
        status[index : index + width] for index in range(0, len(status), width)
    ] or [""]
    for index in range(STATUS_ROWS):
        rows.append((chunks[index] if index < len(chunks) else "", STATUS_SGR))
    rows.append(
        (_clip("─ operator ── " + HELP + " " + "─" * width, width), FRAME_SGR)
    )
    rows.extend(
        (line, _operator_sgr(line))
        for line in _tail(operator_lines, operator_rows, width)
    )
    rows.append((("─ process output " + "─" * width)[:width], FRAME_SGR))
    rows.extend(
        (line, PROCESS_SGR)
        for line in _tail(process_lines, process_rows, width)
    )

    parts = ["\x1b[?25l\x1b[H"]
    for index, (text, sgr) in enumerate(rows):
        painted = sgr + text + RESET_SGR if sgr and text else text
        parts.append("\x1b[2K" + painted)
        if index < len(rows) - 1:
            parts.append("\r\n")
    parts.append("\x1b[0J")
    return "".join(parts).encode("utf-8", errors="replace")


class TeleopTui(KeyboardActivation):
    """KeyboardActivation whose terminal is paneled instead of scrolling.

    Drop-in for the keyboard object the inputs own: same keys, same
    activation semantics, same lifecycle. Additionally exposes set_status()
    for the control loop's once-per-second state line.
    """

    def __init__(
        self,
        device: str,
        sides: tuple[str, ...] = SIDES,
        refresh_interval: float = 0.1,
    ) -> None:
        self._feedback: deque[str] = deque(maxlen=50)
        self._process: deque[str] = deque(maxlen=400)
        self._status = "starting…"
        self._lock = threading.Lock()
        self._dirty = True
        self._next_render = 0.0
        self._refresh_interval = float(refresh_interval)
        self._write_fd: int | None = None
        self._saved_stdout: int | None = None
        self._saved_stderr: int | None = None
        self._pipe_read: int | None = None
        self._drain_thread: threading.Thread | None = None
        super().__init__(device, sides=sides)
        try:
            # A second, blocking fd for writes: the keyboard fd is
            # O_NONBLOCK, and a multi-kilobyte repaint may hit EAGAIN there.
            self._write_fd = os.open(device, os.O_WRONLY | os.O_NOCTTY)
            self._redirect_process_output()
        except BaseException:
            self.close()
            raise
        self._render(force=True)

    # ------------------------------------------------------------- feedback
    def _show(self, message: str) -> None:
        with self._lock:
            self._feedback.append(time.strftime("%H:%M:%S ") + message)
            self._dirty = True

    def set_status(self, text: str) -> None:
        with self._lock:
            self._status = str(text)
            self._dirty = True

    # ------------------------------------------------------------ rendering
    def poll(self) -> dict[str, bool]:
        active = super().poll()
        now = time.monotonic()
        if now >= self._next_render:
            self._next_render = now + self._refresh_interval
            self._render()
        return active

    def _render(self, force: bool = False) -> None:
        if self._write_fd is None:
            return
        with self._lock:
            if not (self._dirty or force):
                return
            self._dirty = False
            status = self._status
            operator = list(self._feedback)
            process = list(self._process)
        try:
            size = os.get_terminal_size(self._write_fd)
        except OSError:
            size = os.terminal_size((80, 24))
        try:
            os.write(
                self._write_fd,
                compose_screen(status, operator, process, size.columns, size.lines),
            )
        except OSError:
            pass

    # ------------------------------------------------------ process capture
    def _redirect_process_output(self) -> None:
        sys.stdout.flush()
        sys.stderr.flush()
        self._pipe_read, pipe_write = os.pipe()
        self._saved_stdout = os.dup(1)
        self._saved_stderr = os.dup(2)
        os.dup2(pipe_write, 1)
        os.dup2(pipe_write, 2)
        os.close(pipe_write)
        # fd 1 is now a pipe, where Python would block-buffer prints and hold
        # lines back for minutes; force per-line flushing into the pane.
        sys.stdout.reconfigure(line_buffering=True)
        self._drain_thread = threading.Thread(target=self._drain, daemon=True)
        self._drain_thread.start()

    def _drain(self) -> None:
        buffer = b""
        while True:
            try:
                chunk = os.read(self._pipe_read, 4096)
            except OSError:
                break
            if not chunk:
                break
            buffer += chunk
            *complete, buffer = buffer.split(b"\n")
            if complete:
                stamp = time.strftime("%H:%M:%S ")
                with self._lock:
                    for line in complete:
                        self._process.append(
                            stamp + line.decode("utf-8", errors="replace")
                        )
                    self._dirty = True

    # -------------------------------------------------------------- teardown
    def close(self) -> None:
        if self.fd is None:
            return
        try:
            sys.stdout.flush()
            sys.stderr.flush()
        except OSError:
            pass
        # Restore the process fds first so anything printed from here on,
        # including the interpreter's own traceback, reaches the terminal.
        if self._saved_stdout is not None:
            os.dup2(self._saved_stdout, 1)
            os.close(self._saved_stdout)
            self._saved_stdout = None
        if self._saved_stderr is not None:
            os.dup2(self._saved_stderr, 2)
            os.close(self._saved_stderr)
            self._saved_stderr = None
        # The dup2 restore released the pipe's write end; EOF stops the drain.
        if self._drain_thread is not None:
            self._drain_thread.join(timeout=1.0)
            self._drain_thread = None
        if self._pipe_read is not None:
            os.close(self._pipe_read)
            self._pipe_read = None
        if self._write_fd is not None:
            try:
                size = os.get_terminal_size(self._write_fd)
                os.write(
                    self._write_fd,
                    f"\x1b[?25h\x1b[{size.lines};1H\r\n".encode(),
                )
            except OSError:
                pass
            os.close(self._write_fd)
            self._write_fd = None
        super().close()
