"""Shared main() of the two nodes: spin, and always publish a stop on the way out.

rclpy's own SIGINT and SIGTERM handlers shut every context down before
control returns to main(), after which publishing the final zero fails with
"publisher's context is invalid". So rclpy is initialised without them:
Ctrl-C raises KeyboardInterrupt and SIGTERM is turned into one, the timer
wakes the executor within one period, and the node's stop() publishes its
zero while the context is still up. Only then is rclpy shut down.
"""
import signal

import rclpy
from rclpy.signals import SignalHandlerOptions


def _interrupt(signum, frame):
    raise KeyboardInterrupt


def run(node_class, args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGTERM, _interrupt)
    try:
        node = node_class()
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.stop()
            node.destroy_node()
    finally:
        rclpy.try_shutdown()
