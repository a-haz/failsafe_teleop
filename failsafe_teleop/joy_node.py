"""ROS 2 node: hold-to-drive gamepad teleop publishing geometry_msgs/Twist."""
import time

from geometry_msgs.msg import Twist
from rclpy.node import Node

from failsafe_teleop.joystick import PadConfig, Teleop
from failsafe_teleop.spin import run


class JoyTeleopNode(Node):
    def __init__(self):
        super().__init__("joy_teleop")
        d = PadConfig()
        p = self.declare_parameter
        device = p("device", "/dev/input/js0").value
        topic = p("cmd_vel_topic", "cmd_vel").value
        rate = float(p("rate_hz", 20.0).value)
        override = bool(p("override", False).value)
        self.cfg = PadConfig(
            max_linear=float(p("max_linear", d.max_linear).value),
            max_angular=float(p("max_angular", d.max_angular).value),
            deadman_button=int(p("deadman_button", d.deadman_button).value),
            axis_linear=int(p("axis_linear", d.axis_linear).value),
            axis_angular=int(p("axis_angular", d.axis_angular).value),
            invert_linear=bool(p("invert_linear", d.invert_linear).value),
            invert_angular=bool(p("invert_angular", d.invert_angular).value),
            deadzone=float(p("deadzone", d.deadzone).value),
            reopen_s=float(p("reopen_s", d.reopen_s).value),
            max_events=int(p("max_events", d.max_events).value),
        )
        self.loop = Teleop(path=device, cfg=self.cfg, override=override)
        self.pub = self.create_publisher(Twist, topic, 1)
        self.timer = self.create_timer(1.0 / rate, self.tick)
        self.get_logger().info(
            "joy_teleop on %s -> %s (%s): release then press button %d to arm, hold it and use the stick"
            % (device, topic, "override" if override else "teleop", self.cfg.deadman_button))

    def tick(self):
        publish, lin, ang, note = self.loop.step(time.monotonic())
        if note == "open":
            self.get_logger().warn("pad open (%s): motion inhibited until button %d is released and pressed"
                                   % (self.loop.path, self.cfg.deadman_button))
        elif note:
            self.get_logger().error("pad LOST (%s, %s): zero twist; inhibited until the pad is back and "
                                    "button %d is released and pressed"
                                    % (self.loop.path, note, self.cfg.deadman_button))
        if publish:
            msg = Twist()
            msg.linear.x = float(lin)
            msg.angular.z = float(ang)
            self.pub.publish(msg)

    def stop(self):
        try:
            self.pub.publish(Twist())
        finally:
            self.loop.close()


def main(args=None):
    run(JoyTeleopNode, args)


if __name__ == "__main__":
    main()
