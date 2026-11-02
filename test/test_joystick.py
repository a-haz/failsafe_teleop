"""Offline tests for the gamepad core (no ROS, no pad, no robot).

The pipe tests drive read_events() through a real pipe (real select and
os.read). The loop tests drive Teleop.step() with a scripted device and check
what would be published in each 50 ms cycle."""
import errno
import fcntl
import os
import struct

import pytest

from failsafe_teleop import joystick as jt

EV = struct.Struct("IhBB")
BTN, AXIS, INIT = 0x01, 0x02, 0x80
DEADMAN = jt.PadConfig().deadman_button


def ev(value, ev_type, number):
    return EV.pack(0, value, ev_type, number)


INIT_RELEASED = [ev(0, BTN | INIT, DEADMAN), ev(0, AXIS | INIT, 0), ev(0, AXIS | INIT, 1)]
INIT_HELD = [ev(1, BTN | INIT, DEADMAN), ev(0, AXIS | INIT, 0), ev(0, AXIS | INIT, 1)]
PRESS, RELEASE = ev(1, BTN, DEADMAN), ev(0, BTN, DEADMAN)
FWD, CENTRE = ev(-32767, AXIS, 1), ev(0, AXIS, 1)
LEFT = ev(-32767, AXIS, 0)


def oserr(code):
    return OSError(code, os.strerror(code))


@pytest.fixture
def pipe():
    r, w = os.pipe()
    fcntl.fcntl(r, fcntl.F_SETFL, fcntl.fcntl(r, fcntl.F_GETFL) | os.O_NONBLOCK)
    fds = {"r": r, "w": w}
    yield fds
    for fd in fds.values():
        try:
            os.close(fd)
        except OSError:
            pass


def connected_state():
    s = jt.PadState()
    s.connect()
    return s


# ---------------------------------------------------------------- real pipe

def test_pipe_arm_drive_release_eof(pipe):
    r, w = pipe["r"], pipe["w"]
    s = connected_state()
    os.write(w, b"".join(INIT_RELEASED))
    assert jt.read_events(r, s) is None and s.present           # init burst read, serviced
    assert s.command() == (0.0, 0.0)                              # not armed after open
    os.write(w, PRESS + FWD)
    jt.read_events(r, s)
    assert s.command()[0] == pytest.approx(0.3)                   # press + forward
    os.write(w, RELEASE)
    jt.read_events(r, s)
    assert s.command() == (0.0, 0.0)                              # release -> zero
    os.write(w, PRESS)
    jt.read_events(r, s)
    os.close(w)
    assert jt.read_events(r, s) == "eof"                          # end of file -> lost


def test_pipe_short_read(pipe):
    s = connected_state()
    os.write(pipe["w"], b"".join(INIT_RELEASED) + PRESS + FWD + ev(0, AXIS, 0)[:5])
    assert jt.read_events(pipe["r"], s) == "short"


def test_closed_descriptor(pipe):
    r = pipe["r"]
    os.close(r)
    assert jt.read_events(r, jt.PadState()) == "error %d" % errno.EBADF


def test_pipe_flood(pipe):
    s = connected_state()
    os.write(pipe["w"], b"".join(INIT_RELEASED) + PRESS + FWD * 70)
    assert jt.read_events(pipe["r"], s) == "flood"                # 70 pending > budget 64


# ---------------------------------------------------------------- control loop

class Dev:
    """Scripted device: items are event bytes, b'' (EOF), short bytes, or ('select', errno) / ('read', errno)."""

    def __init__(self, open_errors=0):
        self.items = []
        self.opens = 0
        self.open_errors = open_errors
        self.closed = 0

    def opener(self, path):
        self.opens += 1
        if self.open_errors:
            self.open_errors -= 1
            raise oserr(errno.ENOENT)
        return 7

    def closer(self, fd):
        self.closed += 1

    def ready(self, fd):
        if not self.items:
            return False
        head = self.items[0]
        if isinstance(head, tuple) and head[0] == "select":
            self.items.pop(0)
            raise oserr(head[1])
        return True

    def read(self, fd, n):
        head = self.items.pop(0)
        if isinstance(head, tuple):
            raise oserr(head[1])
        return head


def loop(override=False, open_errors=0, cfg=None):
    d = Dev(open_errors)
    return d, jt.Teleop(path="/dev/fake", cfg=cfg, override=override, opener=d.opener, closer=d.closer,
                        ready=d.ready, read=d.read)


def moving(out):
    return out[0] and out[1] == pytest.approx(0.3)


def zero_published(out):
    return out[0] and out[1] == 0.0 and out[2] == 0.0


def test_loop_loss_reopen_and_rearm():
    d, t = loop()
    d.items += INIT_RELEASED
    out = t.step(0.0)
    assert zero_published(out) and out[3] == "open"
    d.items += [PRESS, FWD]
    assert moving(t.step(0.05))
    assert moving(t.step(0.10))                                   # held steady, no new events
    d.items += [b""]
    out = t.step(0.15)
    assert zero_published(out) and out[3] == "lost: eof"          # same cycle
    assert d.closed == 1 and t.fd is None
    opens = d.opens
    out = t.step(0.20)
    assert d.opens == opens and zero_published(out)               # no reopen before 0.5 s
    d.open_errors = 1
    out = t.step(0.66)
    assert d.opens == opens + 1 and zero_published(out) and t.fd is None   # reopen fails
    out = t.step(0.70)
    assert d.opens == opens + 1 and zero_published(out)           # no retry inside the interval
    d.items += INIT_HELD + [FWD]
    out = t.step(1.20)
    assert d.opens == opens + 2 and zero_published(out)           # held at open -> inhibited
    d.items += [PRESS]
    assert zero_published(t.step(1.25))                           # press without release -> zero
    d.items += [RELEASE, PRESS]
    assert moving(t.step(1.30))                                   # release then press -> armed


def test_loop_select_and_read_eintr():
    d, t = loop()
    d.items += INIT_RELEASED + [PRESS, FWD]
    t.step(0.0)
    d.items += [("select", errno.EINTR)]
    out = t.step(0.05)
    assert zero_published(out) and out[3] == "lost: eintr"
    d.items += INIT_RELEASED + [PRESS, FWD]
    t.step(1.0)
    assert moving(t.step(1.05))
    d.items += [("read", errno.EINTR)]
    out = t.step(1.10)
    assert zero_published(out) and out[3] == "lost: eintr"


def test_loop_flood_while_moving():
    d, t = loop()
    d.items += INIT_RELEASED + [PRESS, FWD]
    t.step(0.0)
    d.items += [ev(-32767 + i, AXIS, 1) for i in range(200)]
    out = t.step(0.05)
    assert zero_published(out) and out[3] == "lost: flood"


def test_loop_release_buried_in_flood():
    d, t = loop()
    d.items += INIT_RELEASED + [PRESS, FWD]
    t.step(0.0)
    d.items += [RELEASE] + [FWD] * 100
    assert zero_published(t.step(0.05))


def test_loop_read_eio_and_eagain():
    d, t = loop()
    d.items += INIT_RELEASED + [PRESS, FWD]
    t.step(0.0)
    d.items += [("read", errno.EIO)]
    out = t.step(0.05)
    assert zero_published(out) and out[3] == "lost: error %d" % errno.EIO
    d.items += INIT_RELEASED + [PRESS, FWD]
    t.step(1.0)
    d.items += [("read", errno.EAGAIN)]
    out = t.step(1.05)
    assert out[3] is None and t.fd is not None                    # nothing to read, no loss


def test_override_mode():
    d, t = loop(override=True)
    d.items += INIT_RELEASED
    assert t.step(0.0)[0] is False                                # idle pad publishes nothing
    d.items += [PRESS, FWD]
    assert moving(t.step(0.05))
    d.items += [RELEASE]
    assert zero_published(t.step(0.10))                           # exactly one zero on release
    assert t.step(0.15)[0] is False
    d.items += [PRESS]
    assert moving(t.step(0.20))
    d.items += [b""]
    out = t.step(0.25)
    assert zero_published(out) and out[3] == "lost: eof"          # one zero on loss
    assert t.step(0.30)[0] is False


def test_fresh_state_is_inert():
    assert jt.PadState().command() == (0.0, 0.0)


# ---------------------------------------------------------------- parameters

def test_parameters_scale_and_map_axes():
    cfg = jt.PadConfig(max_linear=1.2, max_angular=2.0, deadman_button=5, deadzone=0.1)
    s = jt.PadState(cfg)
    s.connect()
    for data in (ev(0, BTN | INIT, 5), ev(1, BTN, 5), ev(-32767, AXIS, 1), ev(-16384, AXIS, 0)):
        s.event(data)
    lin, ang = s.command()
    assert lin == pytest.approx(1.2)
    assert ang == pytest.approx(2.0 * 16384 / 32767)


@pytest.mark.parametrize("bad", [
    {"max_linear": float("nan")}, {"max_linear": float("inf")}, {"max_angular": -0.6},
    {"deadzone": 1.0}, {"deadzone": -0.1}, {"deadzone": float("nan")},
    {"reopen_s": float("nan")}, {"max_events": 0},
])
def test_unsafe_parameters_are_rejected(bad):
    with pytest.raises(ValueError):
        jt.PadConfig(**bad)


def test_deadzone_and_other_buttons_do_not_arm():
    s = connected_state()
    for data in (ev(0, BTN | INIT, DEADMAN), ev(1, BTN, DEADMAN + 1), ev(-32767, AXIS, 1)):
        s.event(data)
    assert s.command() == (0.0, 0.0)                              # another button is not the deadman
    s.event(PRESS)
    s.event(ev(-4000, AXIS, 1))                                   # 0.12 < deadzone 0.15
    s.event(LEFT)
    lin, ang = s.command()
    assert lin == 0.0 and ang == pytest.approx(0.6)


def test_close_releases_device():
    d, t = loop()
    d.items += INIT_RELEASED + [PRESS, FWD]
    t.step(0.0)
    t.step(0.05)
    t.close()
    assert d.closed == 1 and t.fd is None and t.state.command() == (0.0, 0.0)
