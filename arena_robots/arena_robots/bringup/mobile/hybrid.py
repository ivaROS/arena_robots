"""Hybrid bringup (Option B): DynamicGap in nav2 + the async SICNav bridge + a cmd_vel mux.

Runs the same nav2 stack as `Nav2Bringup` but pins DynamicGap as the SOLE controller and turns on
the `hybrid_mux` launch path (routes DynamicGap's output to `cmd_vel_dgap` and launches the
`cmd_vel_mux` + `controller_switch` nodes). The SICNav planner itself is the smooth `arena_planners`
bridge, spawned as an async edge node by `HybridAdapter` (not a nav2 controller), publishing to
`cmd_vel_topic` (`cmd_vel_sicnav`) which the mux forwards to the real `cmd_vel`.
"""

from __future__ import annotations

from launch import Action
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

from arena_robots.bringup import BringupMeta
from arena_robots.bringup.mobile.nav2 import Nav2Bringup


@BringupMeta.attach(requires={"mobile"}, cap="mobile")
class HybridBringup(Nav2Bringup):
    kind = "hybrid"
    # task_handlers inherited from Nav2Bringup (GOTO_POSE -> navigate_to_pose action for DynamicGap).

    @property
    def cmd_vel_topic(self) -> str:
        # The SICNav bridge edge node's output topic; the cmd_vel_mux consumes it as `cmd_vel_sicnav`.
        return self.namespace("cmd_vel_sicnav")

    def _launch_actions(
        self,
        *,
        use_sim_time: bool = True,
        frame: str = "",
        global_planner: str = "navfn",
        task_generator_node: str = "",
        env_namespace: str = "",
        **_: object,
    ) -> list[Action]:
        # Same nav2 launch as Nav2Bringup, but forcing DynamicGap-only + the hybrid_mux substrate.
        launch_file = PathJoinSubstitution([
            FindPackageShare("arena_robots"), "launch", "adapters", "mobile", "nav2.launch.py",
        ])
        return [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(launch_file),
                launch_arguments={
                    "robot": self.robot.name,
                    "namespace": self.namespace,
                    "use_sim_time": str(use_sim_time).lower(),
                    "frame": frame,
                    "global_planner": global_planner,
                    "local_planner": "dynamicgap",   # DynamicGap is the sole nav2 controller
                    "inter_planner": "default",       # no ControllerSelector BT; the mux arbitrates
                    "hybrid_mux": "true",             # DGap out -> cmd_vel_dgap; launch mux + switch
                    "task_generator_node": task_generator_node,
                    "env_namespace": env_namespace,
                }.items(),
            )
        ]
