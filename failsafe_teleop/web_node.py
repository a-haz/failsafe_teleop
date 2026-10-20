"""ROS 2 node: phone web joystick with a watchdog and a latched e-stop."""
import threading

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import BatteryState

from failsafe_teleop.web import WebConfig, WebState, make_server


class WebTeleopNode(Node):
    def __init__(self):
        super().__init__("web_teleop")
        d = WebConfig()
        p = self.declare_parameter
        host = p("host", "0.0.0.0").value
        port = int(p("port", 8080).value)
        topic = p("cmd_vel_topic", "cmd_vel").value
        rate = float(p("rate_hz", 20.0).value)
        battery_topic = p("battery_topic", "").value
        cfg = WebConfig(
            max_linear=float(p("max_linear", d.max_linear).value),
            max_angular=float(p("max_angular", d.max_angular).value),
            watchdog_s=float(p("watchdog_s", d.watchdog_s).value),
            deadzone=float(p("deadzone", d.deadzone).value),
            token=str(p("token", "").value),
        )
        self.state = WebState(cfg)
        self.pub = self.create_publisher(Twist, topic, 1)
        if battery_topic:
            self.create_subscription(BatteryState, battery_topic, self.on_battery, 1)
        self.server = make_server(self.state, host, port, log=lambda m: self.get_logger().info(m))
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()
        self.timer = self.create_timer(1.0 / rate, self.tick)
        self.get_logger().info(
            "web_teleop on http://%s:%d/%s -> %s (lin %.2f m/s, ang %.2f rad/s, watchdog %.2f s)"
            % (host, port, "?t=<token>" if cfg.token else "", topic, cfg.max_linear, cfg.max_angular,
               cfg.watchdog_s))

    def on_battery(self, msg):
        self.state.battery_v = float(msg.voltage)

    def tick(self):
        v, w, _ = self.state.current()
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        self.pub.publish(msg)

    def stop(self):
        self.state.set_estop(True)
        self.pub.publish(Twist())
        self.server.shutdown()
        self.server.server_close()


def main(args=None):
    rclpy.init(args=args)
    node = WebTeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
