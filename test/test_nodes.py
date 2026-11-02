"""Wiring tests for the two ROS 2 nodes against small stand-ins for rclpy and the
message packages, so they run without a ROS installation. They check parameter
handling and what each timer tick publishes, not ROS itself."""
import importlib
import signal
import sys
import types

import pytest


class Vec:
    def __init__(self):
        self.x = self.y = self.z = 0.0


class Twist:
    def __init__(self):
        self.linear = Vec()
        self.angular = Vec()


class BatteryState:
    def __init__(self, voltage=0.0):
        self.voltage = voltage


# State of the fake rclpy: whether the context is up, whether rclpy's own
# signal handlers were installed, and which signal the next spin() delivers.
ROS = types.SimpleNamespace(ok=False, handlers=False, signal=None)


class Pub:
    def __init__(self, topic):
        self.topic = topic
        self.msgs = []

    def publish(self, msg):
        if not ROS.ok:                       # rcl_publish after rcl_shutdown
            raise RuntimeError("Failed to publish: publisher's context is invalid")
        self.msgs.append(msg)


class Logger:
    def __init__(self):
        self.lines = []

    def info(self, m):
        self.lines.append(("info", m))

    def warn(self, m):
        self.lines.append(("warn", m))

    def error(self, m):
        self.lines.append(("error", m))


OVERRIDES = {}


class Node:
    def __init__(self, name):
        self.name = name
        self.pubs = []
        self.timers = []
        self.subs = []
        self.logger = Logger()

    def declare_parameter(self, name, default):
        return types.SimpleNamespace(value=OVERRIDES.get(name, default))

    def create_publisher(self, msg_type, topic, depth):
        pub = Pub(topic)
        self.pubs.append(pub)
        return pub

    def create_timer(self, period, cb):
        self.timers.append((period, cb))
        return cb

    def create_subscription(self, msg_type, topic, cb, depth):
        self.subs.append((topic, cb))

    def get_logger(self):
        return self.logger

    def destroy_node(self):
        self.destroyed = True


class SignalHandlerOptions:
    NO = 0
    ALL = 3


def fake_init(args=None, context=None, domain_id=None, signal_handler_options=None):
    ROS.ok = True
    ROS.handlers = signal_handler_options != SignalHandlerOptions.NO   # None means ALL in rclpy


def fake_spin(node):
    """Deliver ROS.signal the way rclpy does: its own handlers, when installed,
    shut the context down before control returns to the caller."""
    ROS.spun = node
    if ROS.handlers:
        ROS.ok = False
        if ROS.signal == signal.SIGINT:
            raise KeyboardInterrupt
        return
    if ROS.signal == signal.SIGINT:
        raise KeyboardInterrupt
    handler = signal.getsignal(ROS.signal)
    assert callable(handler), "the default action would kill the node without a stop"
    handler(ROS.signal, None)


def fake_try_shutdown():
    ROS.ok = False


@pytest.fixture
def fake_ros(monkeypatch):
    rclpy = types.ModuleType("rclpy")
    rclpy.init = fake_init
    rclpy.spin = fake_spin
    rclpy.try_shutdown = fake_try_shutdown
    rclpy_node = types.ModuleType("rclpy.node")
    rclpy_node.Node = Node
    rclpy.node = rclpy_node
    rclpy_signals = types.ModuleType("rclpy.signals")
    rclpy_signals.SignalHandlerOptions = SignalHandlerOptions
    rclpy.signals = rclpy_signals
    geometry_msgs = types.ModuleType("geometry_msgs")
    geometry_msgs_msg = types.ModuleType("geometry_msgs.msg")
    geometry_msgs_msg.Twist = Twist
    sensor_msgs = types.ModuleType("sensor_msgs")
    sensor_msgs_msg = types.ModuleType("sensor_msgs.msg")
    sensor_msgs_msg.BatteryState = BatteryState
    for name, mod in [("rclpy", rclpy), ("rclpy.node", rclpy_node), ("rclpy.signals", rclpy_signals),
                      ("geometry_msgs", geometry_msgs), ("geometry_msgs.msg", geometry_msgs_msg),
                      ("sensor_msgs", sensor_msgs), ("sensor_msgs.msg", sensor_msgs_msg)]:
        monkeypatch.setitem(sys.modules, name, mod)
    for name in ("failsafe_teleop.joy_node", "failsafe_teleop.web_node", "failsafe_teleop.spin"):
        monkeypatch.delitem(sys.modules, name, raising=False)
    OVERRIDES.clear()
    ROS.ok, ROS.handlers, ROS.signal = True, False, None
    sigterm = signal.getsignal(signal.SIGTERM)
    yield OVERRIDES
    signal.signal(signal.SIGTERM, sigterm)
    OVERRIDES.clear()


def test_joy_node_parameters_and_ticks(fake_ros):
    fake_ros.update({"cmd_vel_topic": "/rover_twist", "max_linear": 0.5, "deadman_button": 4, "rate_hz": 25.0})
    mod = importlib.import_module("failsafe_teleop.joy_node")
    node = mod.JoyTeleopNode()
    assert node.pubs[0].topic == "/rover_twist"
    assert node.timers[0][0] == pytest.approx(0.04)
    assert node.cfg.max_linear == 0.5 and node.cfg.deadman_button == 4
    assert node.cfg.max_angular == 0.6                     # default kept

    outputs = iter([(True, 0.5, -0.3, "open"), (True, 0.0, 0.0, "lost: eof"), (False, 0.0, 0.0, None)])
    node.loop = types.SimpleNamespace(step=lambda now: next(outputs), path="/dev/input/js0", close=lambda: None)
    node.tick()
    node.tick()
    node.tick()
    msgs = node.pubs[0].msgs
    assert len(msgs) == 2                                  # nothing published when publish is False
    assert (msgs[0].linear.x, msgs[0].angular.z) == (0.5, -0.3)
    assert (msgs[1].linear.x, msgs[1].angular.z) == (0.0, 0.0)
    levels = [lvl for lvl, _ in node.logger.lines]
    assert "warn" in levels and "error" in levels
    node.stop()
    assert msgs[-1].linear.x == 0.0


def test_web_node_publishes_watchdogged_command(fake_ros):
    fake_ros.update({"host": "localhost", "port": 0, "cmd_vel_topic": "/rover_twist",
                     "battery_topic": "/battery", "watchdog_s": 0.4})
    mod = importlib.import_module("failsafe_teleop.web_node")
    node = mod.WebTeleopNode()
    try:
        pub = node.pubs[0]
        assert pub.topic == "/rover_twist"
        node.tick()
        assert (pub.msgs[-1].linear.x, pub.msgs[-1].angular.z) == (0.0, 0.0)
        node.state.set_cmd(1.0, 0.5)
        node.tick()
        assert pub.msgs[-1].linear.x == pytest.approx(0.3)
        assert pub.msgs[-1].angular.z == pytest.approx(0.3)
        node.state.set_estop(True)
        node.tick()
        assert pub.msgs[-1].linear.x == 0.0
        topic, cb = node.subs[0]
        assert topic == "/battery"
        cb(BatteryState(voltage=24.6))
        assert node.state.snapshot()["battery_v"] == pytest.approx(24.6)
    finally:
        node.stop()
    assert pub.msgs[-1].linear.x == 0.0


def last_published(node):
    msg = node.pubs[0].msgs[-1]
    return msg.linear.x, msg.angular.z


@pytest.mark.parametrize("sig", [signal.SIGINT, signal.SIGTERM])
def test_joy_main_publishes_zero_on_signal(fake_ros, sig):
    mod = importlib.import_module("failsafe_teleop.joy_node")
    ROS.signal = sig
    mod.main([])
    node = ROS.spun
    assert last_published(node) == (0.0, 0.0)              # sent while the context was still up
    assert node.destroyed and ROS.ok is False


@pytest.mark.parametrize("sig", [signal.SIGINT, signal.SIGTERM])
def test_web_main_publishes_zero_on_signal(fake_ros, sig):
    fake_ros.update({"host": "localhost", "port": 0})
    mod = importlib.import_module("failsafe_teleop.web_node")
    ROS.signal = sig
    mod.main([])
    node = ROS.spun
    assert last_published(node) == (0.0, 0.0)
    assert node.state.estop is True and node.destroyed and ROS.ok is False


def test_web_node_never_serves_without_a_token(fake_ros):
    fake_ros.update({"host": "localhost", "port": 0})
    mod = importlib.import_module("failsafe_teleop.web_node")
    node = mod.WebTeleopNode()
    try:
        token = node.state.cfg.token
        assert len(token) >= 12
        assert any("?t=" + token in m for _, m in node.logger.lines)   # the operator needs the URL
    finally:
        node.stop()


def test_web_node_keeps_a_configured_token_out_of_the_log(fake_ros):
    fake_ros.update({"host": "localhost", "port": 0, "token": "pick-something"})
    mod = importlib.import_module("failsafe_teleop.web_node")
    node = mod.WebTeleopNode()
    try:
        assert node.state.cfg.token == "pick-something"
        assert not any("pick-something" in m for _, m in node.logger.lines)
    finally:
        node.stop()
