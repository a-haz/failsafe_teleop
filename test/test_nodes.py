"""Wiring tests for the two ROS 2 nodes against small stand-ins for rclpy and the
message packages, so they run without a ROS installation. They check parameter
handling and what each timer tick publishes, not ROS itself."""
import importlib
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


class Pub:
    def __init__(self, topic):
        self.topic = topic
        self.msgs = []

    def publish(self, msg):
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


@pytest.fixture
def fake_ros(monkeypatch):
    rclpy = types.ModuleType("rclpy")
    rclpy_node = types.ModuleType("rclpy.node")
    rclpy_node.Node = Node
    rclpy.node = rclpy_node
    geometry_msgs = types.ModuleType("geometry_msgs")
    geometry_msgs_msg = types.ModuleType("geometry_msgs.msg")
    geometry_msgs_msg.Twist = Twist
    sensor_msgs = types.ModuleType("sensor_msgs")
    sensor_msgs_msg = types.ModuleType("sensor_msgs.msg")
    sensor_msgs_msg.BatteryState = BatteryState
    for name, mod in [("rclpy", rclpy), ("rclpy.node", rclpy_node), ("geometry_msgs", geometry_msgs),
                      ("geometry_msgs.msg", geometry_msgs_msg), ("sensor_msgs", sensor_msgs),
                      ("sensor_msgs.msg", sensor_msgs_msg)]:
        monkeypatch.setitem(sys.modules, name, mod)
    for name in ("failsafe_teleop.joy_node", "failsafe_teleop.web_node"):
        monkeypatch.delitem(sys.modules, name, raising=False)
    OVERRIDES.clear()
    yield OVERRIDES
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
