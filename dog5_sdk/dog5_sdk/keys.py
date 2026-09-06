"""Non-blocking single-key input, for the operator keys a run needs.

A control loop must never block on input.  This polls: :meth:`KeyPoller.get`
returns the key that was pressed since the last call, or ``""``.

Three backends, picked automatically: POSIX raw-mode ``termios`` + ``select``
(the robot host), Windows ``msvcrt`` (a laptop running the simulator), and a
no-op when stdin is not a terminal (a pipe, a CI run, a notebook).  The no-op
is not a failure -- a run that cannot be stopped by hand simply runs to its
duration, and every safety trip still works.
"""
from __future__ import annotations

import sys

#: The keys the shipped runners use, so a controller can reuse the convention.
KEY_STOP = "x"      # stop the run now, torque off
KEY_LIMP = " "      # zero torque, keep the loop and the telemetry alive
KEY_PARK = "p"      # position-mode return to the crouch


class KeyPoller:
    """Context manager.  ``with KeyPoller() as keys: ... keys.get()``."""

    def __init__(self):
        self._mode = "none"
        self._fd = None
        self._saved = None
        self._select = None
        self._termios = None
        self._msvcrt = None
        self._open()

    def _open(self) -> None:
        try:
            if not sys.stdin.isatty():
                return
        except (AttributeError, ValueError):
            return
        try:
            import msvcrt
            self._msvcrt = msvcrt
            self._mode = "msvcrt"
            return
        except ImportError:
            pass
        try:
            import select
            import termios
            import tty
        except ImportError:
            return
        try:
            self._fd = sys.stdin.fileno()
            self._saved = termios.tcgetattr(self._fd)
            tty.setcbreak(self._fd)
        except Exception:
            self._fd = self._saved = None
            return
        self._select, self._termios = select, termios
        self._mode = "posix"

    @property
    def active(self) -> bool:
        """False when no key can ever arrive -- worth printing once."""
        return self._mode != "none"

    def get(self) -> str:
        """The next pending key, or "" if none.  Never blocks."""
        if self._mode == "msvcrt":
            if self._msvcrt.kbhit():
                try:
                    return self._msvcrt.getwch()
                except Exception:
                    return ""
            return ""
        if self._mode == "posix":
            ready, _, _ = self._select.select([sys.stdin], [], [], 0)
            if ready:
                return sys.stdin.read(1)
        return ""

    def close(self) -> None:
        if self._mode == "posix" and self._saved is not None:
            try:
                self._termios.tcsetattr(self._fd, self._termios.TCSADRAIN,
                                        self._saved)
            except Exception:
                pass
            self._saved = None
        self._mode = "none"

    def __enter__(self) -> "KeyPoller":
        return self

    def __exit__(self, *exc) -> bool:
        self.close()
        return False


def is_enter(key: str) -> bool:
    return key in ("\r", "\n")
