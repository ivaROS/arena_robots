"""cmd_vel multiplexer for the hybrid local planner (Option B).

Two local planners run concurrently and publish to distinct pre-mux topics:
  * DynamicGap  -> (nav2 controller_server -> smoother -> collision_monitor) -> cmd_vel_dgap
  * SICNav      -> arena_planners bridge edge node                            -> cmd_vel_sicnav

This node forwards exactly ONE of them to the real robot `cmd_vel`, selected by the same
`controller_selector` (std_msgs/String) the `controller_switch` node already publishes
("DynamicGap" / "SICNav"). It replaces the ControllerSelector behavior-tree node used in the
single-controller_server variant, because here the two planners live on different substrates
(a nav2 controller vs. an async bridge node) and can only be arbitrated on their cmd_vel output.

Behavior:
  * republishes the ACTIVE input's latest Twist to the output at a fixed rate;
  * if the active input has gone stale (no message within input_timeout_s), publishes zero
    (fail-safe stop) rather than a frozen command;
  * before any selector message arrives, uses `default_name`.

`names[i]` maps to `topics[i]` (parallel lists, ROS params can't carry a dict). The selector
string must match one of `names`.
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import Twist
from std_msgs.msg import String


class CmdVelMux(Node):
    def __init__(self) -> None:
        super().__init__("cmd_vel_mux")

        self._names = list(self.declare_parameter("names", ["DynamicGap", "SICNav"]).value)
        self._topics = list(self.declare_parameter("topics", ["cmd_vel_dgap", "cmd_vel_sicnav"]).value)
        self._selector_topic = self.declare_parameter("selector_topic", "controller_selector").value
        self._output_topic = self.declare_parameter("output_topic", "cmd_vel").value
        self._rate = float(self.declare_parameter("publish_rate", 20.0).value)
        self._timeout = float(self.declare_parameter("input_timeout_s", 0.5).value)
        self._default = self.declare_parameter("default_name", self._names[0] if self._names else "").value

        if len(self._names) != len(self._topics) or not self._names:
            raise ValueError(f"names ({self._names}) and topics ({self._topics}) must be non-empty and equal length")

        self._active = self._default
        self._last_twist: dict[str, Twist] = {}
        self._last_time: dict[str, Time] = {}

        self._pub = self.create_publisher(Twist, self._output_topic, 10)
        # one subscription per input; bind the name so the callback knows which slot to fill
        self._subs = []
        for name, topic in zip(self._names, self._topics):
            self._subs.append(
                self.create_subscription(Twist, topic, self._make_cb(name), 10)
            )
        self._sel_sub = self.create_subscription(String, self._selector_topic, self._on_selector, 10)

        self._timer = self.create_timer(1.0 / self._rate, self._tick)
        self.get_logger().info(
            f"cmd_vel_mux up: {dict(zip(self._names, self._topics))} -> '{self._output_topic}' "
            f"(selector '{self._selector_topic}', default '{self._default}', timeout {self._timeout}s)"
        )

    def _make_cb(self, name: str):
        def _cb(msg: Twist) -> None:
            self._last_twist[name] = msg
            self._last_time[name] = self.get_clock().now()
        return _cb

    def _on_selector(self, msg: String) -> None:
        name = msg.data.strip()
        if name not in self._names:
            self.get_logger().warn(
                f"selector '{name}' not in {self._names}; ignoring", throttle_duration_sec=5.0
            )
            return
        if name != self._active:
            self.get_logger().info(f"mux: {self._active} -> {name}")
            self._active = name

    def _tick(self) -> None:
        twist = self._last_twist.get(self._active)
        t = self._last_time.get(self._active)
        fresh = twist is not None and t is not None and \
            (self.get_clock().now() - t).nanoseconds * 1e-9 <= self._timeout
        if fresh:
            self._pub.publish(twist)
        else:
            # Active planner silent/stale -> fail-safe stop (do NOT hold a stale command).
            self._pub.publish(Twist())


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CmdVelMux()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
