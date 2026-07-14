"""Heuristic controller-switch node for the hybrid local planner (Phase 1).

Chooses which nav2 controller runs, based on how close the nearest pedestrian is, and
publishes the choice to the ``controller_selector`` topic that the hybrid behavior tree's
``ControllerSelector`` node consumes (which in turn feeds ``FollowPath``'s ``controller_id``).

Rule (with hysteresis to avoid flapping):
  * start on ``default_controller`` (DynamicGap);
  * switch to ``crowded_controller`` (SICNav) when the nearest pedestrian is closer than
    ``enter_distance``;
  * switch back to ``default_controller`` once the nearest pedestrian is farther than
    ``exit_distance``.

This is the deliberately-simple Phase-1 arbiter. Phase 2 replaces this node's rule with a
trained RL policy that publishes to the same topic, reusing the exact same substrate (the
hybrid controller_config + behavior tree).

Pedestrians arrive on ``arena_peds`` (arena_people_msgs/Pedestrians) in a world frame; distance
is computed against the robot base frame via TF. ``rclpy`` does not resolve ``../`` in topic
names, so the ``../arena_peds`` convention (one namespace above the robot) is expanded manually
against the node namespace, mirroring dynamic_gap's Planner.cpp.
"""

from __future__ import annotations

import math

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time

import tf2_ros
from std_msgs.msg import String

from arena_people_msgs.msg import Pedestrians


def _quat_rotate(qx: float, qy: float, qz: float, qw: float,
                 vx: float, vy: float, vz: float) -> tuple[float, float, float]:
    """Rotate vector v by quaternion q (x, y, z, w). v' = q * v * q^-1."""
    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)
    rx = vx + qw * tx + (qy * tz - qz * ty)
    ry = vy + qw * ty + (qz * tx - qx * tz)
    rz = vz + qw * tz + (qx * ty - qy * tx)
    return rx, ry, rz


class ControllerSwitch(Node):
    def __init__(self) -> None:
        super().__init__("controller_switch")

        # --- Parameters ---
        self._peds_topic = self.declare_parameter("peds_topic", "../arena_peds").value
        self._selector_topic = self.declare_parameter("selector_topic", "controller_selector").value
        self._base_frame = self.declare_parameter("base_frame", "base_link").value
        self._default_controller = self.declare_parameter("default_controller", "DynamicGap").value
        self._crowded_controller = self.declare_parameter("crowded_controller", "SICNav").value
        self._enter_distance = float(self.declare_parameter("enter_distance", 2.0).value)
        self._exit_distance = float(self.declare_parameter("exit_distance", 3.0).value)
        self._publish_period = float(self.declare_parameter("publish_period_s", 0.2).value)
        self._peds_timeout = float(self.declare_parameter("peds_timeout_s", 1.0).value)

        if self._exit_distance < self._enter_distance:
            self.get_logger().warn(
                f"exit_distance ({self._exit_distance}) < enter_distance ({self._enter_distance}); "
                "hysteresis disabled. Expected exit_distance >= enter_distance."
            )

        # --- State ---
        self._current = self._default_controller
        self._last_peds: Pedestrians | None = None
        self._last_peds_time: Time | None = None

        # --- TF ---
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # --- Publisher: latched so a late-joining ControllerSelector still receives the choice ---
        selector_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )
        self._pub = self.create_publisher(String, self._selector_topic, selector_qos)

        # --- Subscriber: pedestrians (manual ../ expansion) ---
        peds_topic = self._expand_topic(self._peds_topic)
        self._sub = self.create_subscription(Pedestrians, peds_topic, self._on_peds, 10)

        self._timer = self.create_timer(self._publish_period, self._tick)

        # Publish the initial choice immediately (before any peds arrive -> default).
        self._publish(force=True)
        self.get_logger().info(
            f"controller_switch up: default={self._default_controller} crowded={self._crowded_controller} "
            f"enter<{self._enter_distance}m exit>{self._exit_distance}m | "
            f"peds='{peds_topic}' selector='{self._selector_topic}' base_frame='{self._base_frame}'"
        )

    def _expand_topic(self, topic: str) -> str:
        """Expand a leading ``../`` against the node namespace (rclpy does not do this).

        Mirrors dynamic_gap Planner.cpp: parent = namespace minus its last segment, then append
        the remainder. e.g. ns=/task_generator_node/jackal, ../arena_peds -> /task_generator_node/arena_peds.
        """
        if not topic.startswith("../"):
            return topic
        ns = self.get_namespace()
        slash = ns.rfind("/")
        parent = "" if slash <= 0 else ns[:slash]
        return f"{parent}/{topic[3:]}"

    def _on_peds(self, msg: Pedestrians) -> None:
        self._last_peds = msg
        self._last_peds_time = self.get_clock().now()

    def _nearest_ped_distance(self) -> float:
        """Nearest pedestrian distance (m) in the robot base frame, or +inf if none/unknown."""
        msg = self._last_peds
        if msg is None or not msg.pedestrians:
            return math.inf

        # Stale pedestrian data -> treat as "no peds" so we relax back to the default controller.
        if self._last_peds_time is not None:
            age = (self.get_clock().now() - self._last_peds_time).nanoseconds * 1e-9
            if age > self._peds_timeout:
                return math.inf

        ped_frame = msg.header.frame_id or "map"
        try:
            tf = self._tf_buffer.lookup_transform(
                self._base_frame, ped_frame, Time(), timeout=Duration(seconds=0.05)
            )
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException,
                tf2_ros.ConnectivityException, tf2_ros.TransformException) as exc:
            self.get_logger().warn(
                f"TF {ped_frame}->{self._base_frame} unavailable ({exc}); holding '{self._current}'",
                throttle_duration_sec=5.0,
            )
            return math.nan  # signal "unknown" -> caller holds current selection

        t = tf.transform.translation
        q = tf.transform.rotation
        nearest = math.inf
        for ped in msg.pedestrians:
            p = ped.pose.position
            rx, ry, _ = _quat_rotate(q.x, q.y, q.z, q.w, p.x, p.y, p.z)
            bx, by = rx + t.x, ry + t.y
            d = math.hypot(bx, by)
            if d < nearest:
                nearest = d
        return nearest

    def _tick(self) -> None:
        dist = self._nearest_ped_distance()
        if math.isnan(dist):
            # Unknown (TF not ready): hold current selection, keep republishing it.
            self._publish()
            return

        desired = self._current
        if self._current == self._default_controller:
            if dist < self._enter_distance:
                desired = self._crowded_controller
        else:  # currently crowded
            if dist > self._exit_distance:
                desired = self._default_controller

        if desired != self._current:
            self.get_logger().info(
                f"switch {self._current} -> {desired} (nearest ped {dist:.2f} m)"
            )
            self._current = desired
        self._publish()

    def _publish(self, force: bool = False) -> None:
        # Latched topic: republishing the same value each tick is cheap and guarantees delivery
        # to a ControllerSelector that subscribes after us. `force` is used for the startup sample.
        _ = force
        self._pub.publish(String(data=self._current))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ControllerSwitch()
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
