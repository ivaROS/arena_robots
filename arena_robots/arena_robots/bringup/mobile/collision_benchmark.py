from __future__ import annotations

from typing import ClassVar

from launch import Action
from launch.actions import ExecuteProcess

from arena_robots.bringup import Bringup, BringupMeta
from arena_robots.task_kinds import TaskKind


def _load_goto_pose_collision_benchmark() -> type:
    from arena_robots.task_server_handlers.goto_pose._passthrough import GotoPoseHandlerNone

    return GotoPoseHandlerNone


@BringupMeta.attach(requires={"mobile"}, cap="mobile")
class CollisionBenchmarkBringup(Bringup):
    """Constant-velocity drive for the collision-recorder benchmark.

    Publishes a fixed forward ``cmd_vel`` (no planner, no goal dispatch), like
    :class:`TestCollisionBringup`.  The episode window is bounded by
    ``CollisionBenchmarkAdapter`` via ``duration_s`` (sim time), so this bringup
    only needs to keep the robot moving; ``duration_s`` is accepted and ignored
    here.
    """

    kind = "collision-benchmark"
    task_handlers: ClassVar[dict] = {TaskKind.GOTO_POSE: _load_goto_pose_collision_benchmark}

    @property
    def goal_topic(self) -> str:
        return self.namespace("goal_pose")

    def _launch_actions(
        self,
        *,
        use_sim_time: bool = True,
        frame: str = "",
        linear_x: float = 1.0,
        rate_hz: float = 10.0,
        **_: object,
    ) -> list[Action]:
        cmd_vel_topic = str(self.namespace("cmd_vel"))
        twist = f"{{linear: {{x: {float(linear_x)}, y: 0.0, z: 0.0}}, angular: {{x: 0.0, y: 0.0, z: 0.0}}}}"
        return [
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "topic",
                    "pub",
                    "-r",
                    str(float(rate_hz)),
                    cmd_vel_topic,
                    "geometry_msgs/msg/Twist",
                    twist,
                ],
                output="log",
            ),
        ]
