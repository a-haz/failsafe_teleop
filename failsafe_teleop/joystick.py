"""Gamepad teleop core without ROS.

Reads the Linux joystick API (/dev/input/jsN, struct js_event) directly, so
it needs no SDL and no joy node. Driving is hold-to-run: the deadman button
must be held. A lost or misbehaving pad stops the robot in the same cycle:

- Any read failure (I/O error, end of file, short read, bad descriptor,
  interrupted select or read) counts as PAD LOST: every button and axis is
  cleared, a zero command goes out in that cycle and in every cycle after,
  and the device is closed and reopened every reopen_s seconds.
- At most max_events events are drained per cycle. A pad that is still
  readable after that budget is treated as a faulty stream and also counts
  as lost, so the zero always goes out within the cycle.
- After every (re)open, motion stays inhibited until the deadman has been
  seen released and then pressed again. A press already held at open, which
  the kernel reports as an init event, never arms.

Teleop.step() is the whole control loop, so the publishing path is tested
offline with scripted devices.
"""
from __future__ import annotations

import errno
import math
import os
import select
import struct
from dataclasses import dataclass

EVENT = struct.Struct("IhBB")    # struct js_event: time (u32), value (s16), type (u8), number (u8)
JS_EVENT_BUTTON = 0x01
JS_EVENT_AXIS = 0x02
JS_EVENT_INIT = 0x80             # synthetic event the kernel sends on open (current state)


@dataclass
class PadConfig:
    max_linear: float = 0.3      # m/s at full stick
    max_angular: float = 0.6     # rad/s at full stick
    deadman_button: int = 2      # X on an Xbox 360 pad
    axis_linear: int = 1         # left stick vertical
    axis_angular: int = 0        # left stick horizontal
    invert_linear: bool = True   # stick up reads negative, and means forward
    invert_angular: bool = True  # stick left reads negative, and means counter-clockwise
    deadzone: float = 0.15
    reopen_s: float = 0.5        # retry interval for a lost pad
    max_events: int = 64         # events drained per cycle; a real pad sends far fewer

    def __post_init__(self):
        for name in ("max_linear", "max_angular", "reopen_s"):
            value = getattr(self, name)
            if not (math.isfinite(value) and value >= 0):
                raise ValueError("%s must be a finite number >= 0, got %r" % (name, value))
        if not 0.0 <= self.deadzone < 1.0:
            raise ValueError("deadzone must be in [0, 1), got %r" % (self.deadzone,))
        if self.max_events < 1:
            raise ValueError("max_events must be at least 1, got %r" % (self.max_events,))


class PadState:
    """Pad state and the arming rule. No ROS and no device."""

    def __init__(self, cfg: PadConfig | None = None):
        self.cfg = cfg or PadConfig()
        self._clear(False)

    def _clear(self, present):
        self.axes = {}
        self.buttons = {}
        self.present = present
        self.seen_release = False
        self.armed = False

    def connect(self):
        self._clear(True)

    def lose(self):
        self._clear(False)

    def event(self, data):
        _t, value, ev_type, number = EVENT.unpack(data)
        init = bool(ev_type & JS_EVENT_INIT)
        ev_type &= ~JS_EVENT_INIT
        if ev_type == JS_EVENT_BUTTON:
            self.buttons[number] = value
            if number == self.cfg.deadman_button:
                if not value:
                    self.seen_release = True
                    self.armed = False
                elif self.seen_release and not init:
                    self.armed = True
        elif ev_type == JS_EVENT_AXIS:
            self.axes[number] = max(-1.0, value / 32767.0)   # -32768 would exceed full stick

    def held(self):
        return self.present and self.armed and bool(self.buttons.get(self.cfg.deadman_button))

    def command(self):
        """(linear_x, angular_z): zero unless the pad is present, armed and the deadman held."""
        if not self.held():
            return 0.0, 0.0
        c = self.cfg
        lx = self.axes.get(c.axis_linear, 0.0) * (-1.0 if c.invert_linear else 1.0)
        az = self.axes.get(c.axis_angular, 0.0) * (-1.0 if c.invert_angular else 1.0)
        return (c.max_linear * lx if abs(lx) > c.deadzone else 0.0,
                c.max_angular * az if abs(az) > c.deadzone else 0.0)


def _errno_of(exc):
    if getattr(exc, "errno", None) is not None:
        return exc.errno
    return exc.args[0] if exc.args else None


def _ready(fd):
    return bool(select.select([fd], [], [], 0)[0])


def read_events(fd, state, ready=_ready, read=os.read, budget=None):
    """Drain at most `budget` pending events into state.

    Returns None while the pad is serviced, else why it is lost:
    'eintr', 'error N', 'eof', 'short' or 'flood'."""
    if budget is None:
        budget = state.cfg.max_events
    n = 0
    while True:
        try:
            more = ready(fd)
        except (OSError, ValueError) as exc:
            code = _errno_of(exc)
            return "eintr" if code == errno.EINTR else "error %s" % code
        if not more:
            return None
        if n >= budget:
            return "flood"
        try:
            data = read(fd, EVENT.size)
        except OSError as exc:
            code = _errno_of(exc)
            if code in (errno.EAGAIN, errno.EWOULDBLOCK):
                return None          # nothing to read after all; the pad state stands
            return "eintr" if code == errno.EINTR else "error %s" % code
        if not data:
            return "eof"             # device gone
        if len(data) != EVENT.size:
            return "short"           # broken stream
        state.event(data)
        n += 1


class Teleop:
    """The control loop without ROS: step(now) -> (publish, linear_x, angular_z, note).

    override=True publishes only while the deadman is held, plus one zero on
    release or pad loss, so another source (a planner) can drive while the
    pad is idle and holding the deadman with the stick centred is a hard stop."""

    def __init__(self, path="/dev/input/js0", cfg=None, override=False, opener=None, closer=os.close,
                 ready=_ready, read=os.read):
        self.path = path
        self.cfg = cfg or PadConfig()
        self.override = override
        self.opener = opener or (lambda p: os.open(p, os.O_RDONLY | os.O_NONBLOCK))
        self.closer = closer
        self.ready = ready
        self.read = read
        self.state = PadState(self.cfg)
        self.fd = None
        self.next_open = 0.0
        self.held_last = False

    def step(self, now):
        note = None
        if self.fd is None and now >= self.next_open:
            try:
                self.fd = self.opener(self.path)
                self.state.connect()
                note = "open"
            except OSError:
                self.fd = None
                self.next_open = now + self.cfg.reopen_s
        if self.fd is not None:
            lost = read_events(self.fd, self.state, self.ready, self.read)
            if lost:
                try:
                    self.closer(self.fd)
                except OSError:
                    pass
                self.fd = None
                self.state.lose()
                self.next_open = now + self.cfg.reopen_s
                note = "lost: " + lost
        lin, ang = self.state.command()
        held = self.state.held()
        publish = (not self.override) or held or self.held_last
        self.held_last = held
        return publish, lin, ang, note

    def close(self):
        if self.fd is not None:
            try:
                self.closer(self.fd)
            except OSError:
                pass
            self.fd = None
            self.state.lose()
