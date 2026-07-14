"""Nav2 adapter launch."""

from arena_bringup.future import PythonExpression
from arena_bringup.substitutions import (
    LaunchArgument,
    YAMLFileSubstitution,
    YAMLMergeSubstitution,
    YAMLReplaceSubstitution,
    YAMLRetrieveSubstitution,
)
from arena_robots.nav2 import (
    Nav2CollisionDerivedYAML,
    Nav2KinematicsDerivedYAML,
    Nav2SubBlockYAML,
    SensorsDerivedYAML,
)
from launch import LaunchDescription
from launch.actions import GroupAction, OpaqueFunction
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node, SetRemap
from launch_ros.descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    robots_root = FindPackageShare('arena_robots')

    ld_items = []
    LaunchArgument.auto_append(ld_items)

    robot = LaunchArgument('robot')
    task_generator_node = LaunchArgument('task_generator_node', default_value='')
    env_namespace = LaunchArgument('env_namespace', default_value='')
    namespace = LaunchArgument('namespace')
    frame = LaunchArgument('frame')
    use_sim_time = LaunchArgument('use_sim_time')
    global_planner = LaunchArgument('global_planner')
    local_planner = LaunchArgument('local_planner')
    inter_planner = LaunchArgument('inter_planner')
    train_mode = LaunchArgument('train_mode', default_value='false')
    planner_only = LaunchArgument('planner_only', default_value='false')
    # Option B (mobile:=hybrid): route DynamicGap's output to cmd_vel_dgap and launch the
    # controller_switch + cmd_vel_mux (which forwards the selected planner to the real cmd_vel).
    hybrid_mux = LaunchArgument('hybrid_mux', default_value='false')

    def nav2_cfg(*parts):
        return PathJoinSubstitution([robots_root, 'config', 'nav2', *parts])

    mobile_path = PathJoinSubstitution([
        robots_root, 'robots', robot.substitution, 'caps', 'mobile.yaml'
    ])
    model_params_path = PathJoinSubstitution([
        robots_root, 'robots', robot.substitution, 'model_params.yaml'
    ])
    # Robot base frame, used by the hybrid controller_switch node for its TF distance query.
    base_frame_sub = YAMLRetrieveSubstitution(
        YAMLFileSubstitution(model_params_path), 'base_frame'
    )
    interplanner_cfg = nav2_cfg('interplanners', inter_planner.substitution, 'interplanner_config.yaml')
    interplanner_yaml = YAMLFileSubstitution(interplanner_cfg)

    def retrieve(key):
        return YAMLRetrieveSubstitution(interplanner_yaml, key)

    substitutions = YAMLMergeSubstitution(
        YAMLFileSubstitution(nav2_cfg('defaults', 'model_params.yaml')),
        YAMLFileSubstitution(mobile_path),
        Nav2SubBlockYAML(mobile_path),
        Nav2CollisionDerivedYAML(mobile_path),
        Nav2KinematicsDerivedYAML(mobile_path),
        SensorsDerivedYAML(model_params_path, mobile_path),
        YAMLFileSubstitution(nav2_cfg('defaults', 'controller_config.yaml')),
        YAMLFileSubstitution(nav2_cfg('controllers', local_planner.substitution, 'controller_config.yaml')),
        YAMLFileSubstitution(nav2_cfg('defaults', 'planner_config.yaml')),
        YAMLFileSubstitution(nav2_cfg('planners', global_planner.substitution, 'planner_config.yaml')),
        YAMLFileSubstitution(nav2_cfg('defaults', 'interplanner_config.yaml')),
        YAMLFileSubstitution(interplanner_cfg),
        YAMLFileSubstitution.from_dict(
            {
                'frame': frame.substitution,
                'base_frame': YAMLRetrieveSubstitution(YAMLFileSubstitution(model_params_path), 'base_frame'),
                **task_generator_node.dict,
                **env_namespace.dict,
                'namespace': namespace.substitution,
                # collision_monitor output topic:
                #  - train_mode: dead topic (RL env publishes cmd_vel directly);
                #  - hybrid_mux: cmd_vel_dgap (the cmd_vel_mux forwards the selected planner to cmd_vel);
                #  - otherwise: the normal cmd_vel.
                'cmd_vel_out_topic': PythonExpression(
                    ['"cmd_vel_sink" if "', train_mode.substitution,
                     '" == "true" else ("cmd_vel_dgap" if "', hybrid_mux.substitution,
                     '" == "true" else "cmd_vel")']
                ),
                'default_nav_to_pose_bt_xml': retrieve('bt_navigator/ros__parameters/default_nav_to_pose_bt_xml'),
                'default_nav_through_poses_bt_xml': retrieve('bt_navigator/ros__parameters/default_nav_through_poses_bt_xml'),
                'plugin_lib_names': retrieve('bt_navigator/ros__parameters/plugin_lib_names'),
            },
            substitute=True
        ),
    )

    substituted_parameters = YAMLReplaceSubstitution(
        obj=YAMLFileSubstitution(nav2_cfg('nav2.yaml')),
        substitutions=YAMLFileSubstitution(substitutions)
    )

    nav2_configured_params = ParameterFile(
        RewrittenYaml(
            source_file=substituted_parameters,
            root_key=namespace.substitution,
            param_rewrites={
                'use_sim_time': use_sim_time.substitution,
            },
            convert_types=True
        ),
        allow_substs=True,
    )

    full_lifecycle_nodes = [
        'controller_server',
        'smoother_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'waypoint_follower',
        'velocity_smoother',
        'collision_monitor'
    ]

    def launch_setup(context, *args, **kwargs):
        tgn = task_generator_node.substitution.perform(context)
        is_planner_only = planner_only.substitution.perform(context).lower() == 'true'
        remappings = [
            ('map_server', '/map_server'),
            ('/tf', '/tf'),
            ('/tf_static', '/tf_static'),
        ]
        if tgn:
            remappings.append(('map', PathJoinSubstitution([tgn, 'map'])))

        planner_server_node = Node(
            package='nav2_planner', executable='planner_server', name='planner_server',
            output='screen', parameters=[nav2_configured_params]
        )

        if is_planner_only:
            lifecycle_nodes = ['planner_server']
            nav2_nodes = [planner_server_node]
        else:
            lifecycle_nodes = full_lifecycle_nodes
            # cmd_vel chain: controller -> cmd_vel_nav -> smoother -> cmd_vel_smoothed <- behaviors
            #                cmd_vel_smoothed -> collision_monitor -> cmd_vel (-> twist_stamper)
            nav2_nodes = [
                Node(
                    package='nav2_controller', executable='controller_server', name='controller_server',
                    output='screen', parameters=[nav2_configured_params],
                    remappings=[('cmd_vel', 'cmd_vel_nav')],
                ),
                Node(
                    package='nav2_smoother', executable='smoother_server', name='smoother_server',
                    output='screen', parameters=[nav2_configured_params]
                ),
                planner_server_node,
                Node(
                    package='nav2_behaviors', executable='behavior_server', name='behavior_server',
                    output='screen', parameters=[nav2_configured_params],
                    remappings=[('cmd_vel', 'cmd_vel_smoothed')],
                ),
                Node(
                    package='nav2_bt_navigator', executable='bt_navigator', name='bt_navigator',
                    output='screen', parameters=[nav2_configured_params]
                ),
                Node(
                    package='nav2_waypoint_follower', executable='waypoint_follower', name='waypoint_follower',
                    output='screen', parameters=[nav2_configured_params]
                ),
                Node(
                    package='nav2_velocity_smoother', executable='velocity_smoother', name='velocity_smoother',
                    output='screen', parameters=[nav2_configured_params],
                    remappings=[('cmd_vel', 'cmd_vel_nav'), ('smoothed_cmd_vel', 'cmd_vel_smoothed')],
                ),
                Node(
                    package='nav2_collision_monitor', executable='collision_monitor', name='collision_monitor',
                    output='screen', parameters=[nav2_configured_params]
                ),
            ]

        # Hybrid local planner: launch the heuristic controller-switch node alongside the nav2
        # stack. It publishes the active controller id (DynamicGap | SICNav) to the robot-namespaced
        # `controller_selector` topic that the hybrid behavior tree's ControllerSelector consumes.
        # Co-located here so it inherits the exact same namespace as bt_navigator/controller_server.
        if not is_planner_only and local_planner.substitution.perform(context) == 'hybrid':
            switch_use_sim_time = (
                use_sim_time.substitution.perform(context).strip().lower() in ('true', '1', 'yes')
            )
            nav2_nodes.append(
                Node(
                    package='arena_robots', executable='controller_switch_node',
                    name='controller_switch', output='screen',
                    parameters=[{
                        'use_sim_time': switch_use_sim_time,
                        'peds_topic': '../arena_peds',
                        'selector_topic': 'controller_selector',
                        'base_frame': base_frame_sub.perform(context),
                        'default_controller': 'DynamicGap',
                        'crowded_controller': 'SICNav',
                        'enter_distance': 2.0,
                        'exit_distance': 3.0,
                    }],
                )
            )

        # Option B (mobile:=hybrid, via HybridBringup passing hybrid_mux:=true):
        # DynamicGap is the sole nav2 controller and its output was routed to cmd_vel_dgap
        # (cmd_vel_out_topic above). The SICNav *bridge* runs as a separate async node (spawned by
        # HybridAdapter) publishing cmd_vel_sicnav. Launch the controller_switch + cmd_vel_mux here
        # so they share the robot namespace; the mux forwards the selected planner to cmd_vel.
        if not is_planner_only and hybrid_mux.substitution.perform(context).strip().lower() in ('true', '1', 'yes'):
            mux_use_sim_time = (
                use_sim_time.substitution.perform(context).strip().lower() in ('true', '1', 'yes')
            )
            nav2_nodes.append(
                Node(
                    package='arena_robots', executable='controller_switch_node',
                    name='controller_switch', output='screen',
                    parameters=[{
                        'use_sim_time': mux_use_sim_time,
                        'peds_topic': '../arena_peds',
                        'selector_topic': 'controller_selector',
                        'base_frame': base_frame_sub.perform(context),
                        'default_controller': 'DynamicGap',
                        'crowded_controller': 'SICNav',
                        'enter_distance': 2.0,
                        'exit_distance': 3.0,
                    }],
                )
            )
            nav2_nodes.append(
                Node(
                    package='arena_robots', executable='cmd_vel_mux_node',
                    name='cmd_vel_mux', output='screen',
                    parameters=[{'use_sim_time': mux_use_sim_time}],
                )
            )

        bringup_cmd_group = GroupAction([
            *(SetRemap(src=r[0], dst=r[1]) for r in remappings),
            *nav2_nodes,
            Node(
                package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_navigation',
                output='screen',
                parameters=[
                    {'autostart': True},
                    {'node_names': lifecycle_nodes},
                    nav2_configured_params
                ]
            ),
        ])
        return [bringup_cmd_group]

    return LaunchDescription([*ld_items, OpaqueFunction(function=launch_setup)])
