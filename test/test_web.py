"""Offline tests for the web joystick core: watchdog, e-stop latch and the HTTP API."""
import http.client
import json
import socket
import threading

import pytest

from failsafe_teleop.web import WebConfig, WebState, make_server, render_page


class Clock:
    def __init__(self, t=100.0):
        self.t = t

    def __call__(self):
        return self.t


@pytest.fixture
def st():
    clock = Clock()
    s = WebState(WebConfig(max_linear=0.3, max_angular=0.6, watchdog_s=0.4), clock=clock)
    s.clock_ = clock
    return s


def test_no_command_yet_is_zero(st):
    assert st.current() == (0.0, 0.0, False)
    assert st.snapshot()["age_ms"] is None


def test_command_is_scaled_and_clamped(st):
    st.set_cmd(0.5, -1.0)
    v, w, live = st.current()
    assert live and v == pytest.approx(0.15) and w == pytest.approx(-0.6)
    st.set_cmd(3.0, 9.0)
    assert st.current()[:2] == (pytest.approx(0.3), pytest.approx(0.6))


def test_watchdog_zeroes_after_0_4_s(st):
    st.set_cmd(1.0, 0.0)
    st.clock_.t += 0.39
    assert st.current()[0] == pytest.approx(0.3)
    st.clock_.t += 0.02                       # 0.41 s without a command
    assert st.current() == (0.0, 0.0, False)
    assert st.snapshot()["live"] is False


def test_fresh_command_after_watchdog_resumes(st):
    st.set_cmd(1.0, 0.0)
    st.clock_.t += 1.0
    assert st.current()[2] is False
    st.set_cmd(0.5, 0.0)
    assert st.current() == (pytest.approx(0.15), 0.0, True)


def test_estop_latches_until_reset(st):
    st.set_cmd(1.0, 0.5)
    st.set_estop(True)
    assert st.current() == (0.0, 0.0, False)
    for _ in range(5):                        # commands keep arriving: still stopped
        st.clock_.t += 0.05
        st.set_cmd(1.0, 0.5)
        assert st.current() == (0.0, 0.0, False)
    st.clock_.t += 5.0                        # time alone does not clear it
    st.set_cmd(1.0, 0.5)
    assert st.current() == (0.0, 0.0, False)
    assert st.snapshot()["estop"] is True


def test_reset_does_not_resume_old_command(st):
    st.set_cmd(1.0, 0.0)
    st.set_estop(True)
    st.set_estop(False)
    v, w, _ = st.current()
    assert (v, w) == (0.0, 0.0)               # needs a new command after the reset
    st.set_cmd(1.0, 0.0)
    assert st.current()[0] == pytest.approx(0.3)


def test_non_finite_values_stop(st):
    st.set_cmd(1.0, 0.0)
    st.set_cmd(float("nan"), 0.0)
    assert st.current()[:2] == (0.0, 0.0)
    st.set_cmd(1.0, float("inf"))
    assert st.current()[:2] == (0.0, 0.0)


def test_page_has_settings_and_no_placeholders():
    page = render_page(WebConfig(max_linear=0.25, max_angular=0.5, deadzone=0.1))
    assert "%" + "LIN%" not in page and "%" + "DEADZONE%" not in page
    assert "DEAD = 0.1" in page and "* 0.25" in page and "* 0.5" in page


# ---------------------------------------------------------------- HTTP

LOG = []


@pytest.fixture
def server():
    state = WebState(WebConfig(token="s3cret"))
    LOG.clear()
    srv = make_server(state, host="localhost", port=0, log=LOG.append)
    th = threading.Thread(target=srv.serve_forever, daemon=True)
    th.start()
    yield state, srv.server_address[1]
    srv.shutdown()
    srv.server_close()


def request(port, method, path, body=None, headers=None):
    c = http.client.HTTPConnection("localhost", port, timeout=5)
    data = None if body is None else (body if isinstance(body, bytes) else json.dumps(body).encode())
    c.request(method, path, body=data, headers=headers or {})
    r = c.getresponse()
    out = r.status, r.read()
    c.close()
    return out


def test_http_token_required(server):
    state, port = server
    assert request(port, "GET", "/")[0] == 401
    assert request(port, "POST", "/cmd?t=wrong", {"v": 1})[0] == 401
    assert state.current()[2] is False
    code, body = request(port, "GET", "/?t=s3cret")
    assert code == 200 and b"E-STOP" in body
    assert request(port, "GET", "/status", headers={"X-Token": "s3cret"})[0] == 200


def test_http_cmd_estop_reset(server):
    state, port = server
    code, body = request(port, "POST", "/cmd?t=s3cret", {"v": 1.0, "w": 0.0})
    assert code == 200 and json.loads(body)["live"] is True
    assert state.current()[0] == pytest.approx(0.3)
    code, body = request(port, "POST", "/estop?t=s3cret", {"on": True})
    assert json.loads(body)["estop"] is True
    request(port, "POST", "/cmd?t=s3cret", {"v": 1.0})
    assert state.current() == (0.0, 0.0, False)
    request(port, "POST", "/estop?t=s3cret", {"on": False})
    assert state.current()[:2] == (0.0, 0.0)
    request(port, "POST", "/cmd?t=s3cret", {"v": -0.5})
    assert state.current()[0] == pytest.approx(-0.15)
    request(port, "POST", "/stop?t=s3cret", {})
    assert state.current()[:2] == (0.0, 0.0)


def test_http_bad_input(server):
    state, port = server
    assert request(port, "POST", "/cmd?t=s3cret", b"{not json")[0] == 400
    assert request(port, "POST", "/cmd?t=s3cret", b"[1, 2]")[0] == 400
    assert request(port, "POST", "/cmd?t=s3cret", {"v": "fast"})[0] == 400
    assert request(port, "POST", "/nope?t=s3cret", {})[0] == 404
    assert request(port, "GET", "/nope?t=s3cret")[0] == 404
    assert state.current() == (0.0, 0.0, False)


def test_http_log_has_no_token(server):
    _, port = server
    request(port, "GET", "/status?t=s3cret")
    request(port, "GET", "/?x=1&t=wrong")
    request(port, "POST", "/cmd?t=s3cret", {"v": 0.1})
    assert len(LOG) == 2                                      # /cmd is not logged
    assert all("s3cret" not in line and "wrong" not in line for line in LOG)
    assert "t=***" in LOG[0]


def test_http_bad_request_line_gets_an_answer(server):
    _, port = server
    with socket.create_connection(("localhost", port), timeout=5) as s:
        s.sendall(b"\x16\x03\x01 not http\r\n\r\n")         # e.g. https:// typed for the page
        assert b" 400 " in s.recv(200)


def raw_exchange(port, data):
    """Send raw bytes on one connection and read until the server closes it."""
    with socket.create_connection(("localhost", port), timeout=5) as s:
        s.sendall(data)
        out = b""
        while True:
            chunk = s.recv(4096)
            if not chunk:
                return out
            out += chunk


def test_http_unread_body_closes_the_connection(server):
    state, port = server
    body = b'{"v": 1}'
    out = raw_exchange(port, b"POST /cmd?t=wrong HTTP/1.1\r\nHost: x\r\nContent-Length: %d\r\n\r\n%s"
                       % (len(body), body))
    assert out.count(b"HTTP/1.") == 1 and b" 401 " in out    # the body is not parsed as a request
    assert state.current()[2] is False


@pytest.mark.parametrize("length,code", [(b"-1", b" 400 "), (b"abc", b" 400 "), (b"1000000000", b" 413 ")])
def test_http_bad_content_length(server, length, code):
    state, port = server
    out = raw_exchange(port, b"POST /cmd?t=s3cret HTTP/1.1\r\nHost: x\r\nContent-Length: " + length +
                       b"\r\n\r\n{}")
    assert out.count(b"HTTP/1.") == 1 and code in out
    assert state.current()[2] is False
