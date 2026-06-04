"""Drive `planner_server` from `<ns>/goal_pose`.

`planner_server` only publishes `/plan` as a side effect of the
`compute_path_to_pose` action. When we run nav2 in `planner_only` mode there is
no `bt_navigator` calling it, so this node bridges PoseStamped goals to the
action.
"""

from __future__ import annotations

import math

import rclpy
from arena_rclpy_mixins.spin import spin_node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

_DEDUPE_POS_TOL: float = 0.05
_DEDUPE_YAW_TOL: float = 0.05


def _yaw_from_quat(q: object) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class GoalToPlanBridge(Node):
    def __init__(self) -> None:
        super().__init__("goal_to_plan_bridge")

        goal_topic = self.declare_parameter("goal_topic", "goal_pose").value
        action_name = self.declare_parameter("action_name", "compute_path_to_pose").value
        self._planner_id = self.declare_parameter("planner_id", "").value

        # Periodic replanning of the active goal. /plan is published VOLATILE, so a
        # plan-once-per-goal model delivers nothing to subscribers that connect later
        # (e.g. a planner bridge starting up). Republishing keeps /plan reliably
        # available and fresh. Set to 0 to plan only once per goal.
        self._replan_period = float(self.declare_parameter("replan_period", 1.0).value)

        self._pending: PoseStamped | None = None   # goal received but not yet dispatched
        self._sent: PoseStamped | None = None       # last goal actually dispatched (dedupe key)
        self._client: ActionClient = ActionClient(self, ComputePathToPose, action_name)

        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(PoseStamped, goal_topic, self._on_goal, qos)

        # Dispatch loop: retry until the action server is ready. This fixes the
        # startup race where a goal arrived before compute_path_to_pose was up —
        # previously the goal was marked seen and (re)published identical goals
        # were deduped away, so /plan was never produced.
        self.create_timer(0.25, self._dispatch_pending)
        if self._replan_period > 0.0:
            self.create_timer(self._replan_period, self._replan)

    def _on_goal(self, msg: PoseStamped) -> None:
        # Already dispatched this goal, or already queued — nothing to do.
        if self._sent is not None and self._same_goal(self._sent, msg):
            return
        if self._pending is not None and self._same_goal(self._pending, msg):
            return
        self._pending = msg  # a newer/distinct goal supersedes any pending one

    def _dispatch_pending(self) -> None:
        if self._pending is None:
            return
        if not self._client.server_is_ready():
            return  # not ready yet; retry on the next tick
        self._send(self._pending)
        self._sent = self._pending
        self._pending = None

    def _replan(self) -> None:
        # Re-issue the active goal so the plan tracks the costmap; skip if a new
        # goal is already waiting to be dispatched.
        if self._pending is None and self._sent is not None and self._client.server_is_ready():
            self._send(self._sent)

    def _send(self, msg: PoseStamped) -> None:
        goal = ComputePathToPose.Goal()
        goal.goal = msg
        goal.use_start = False
        if self._planner_id:
            goal.planner_id = self._planner_id
        self._client.send_goal_async(goal)

    @staticmethod
    def _same_goal(a: PoseStamped, b: PoseStamped) -> bool:
        dx = a.pose.position.x - b.pose.position.x
        dy = a.pose.position.y - b.pose.position.y
        if math.hypot(dx, dy) > _DEDUPE_POS_TOL:
            return False
        return abs(_yaw_from_quat(a.pose.orientation) - _yaw_from_quat(b.pose.orientation)) <= _DEDUPE_YAW_TOL


def main() -> None:
    rclpy.init()
    node = GoalToPlanBridge()
    try:
        spin_node(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
