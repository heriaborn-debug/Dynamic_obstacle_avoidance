import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("m20_gazebo")
    v2_share = get_package_share_directory("m20_first_planner_v2")
    map_file = os.path.join(share, "config", "small_city_static.yaml")
    rviz_config = os.path.join(
        v2_share, "rviz", "m20_first_v2_navigation.rviz"
    )
    outer_dynamic_config = os.path.join(
        share, "config", "outer_dynamic_layer.yaml"
    )
    pcl_pass_grid_config = os.path.join(
        share, "config", "pcl_pass_grid.yaml"
    )
    airy_bridge_config = os.path.join(
        share, "config", "airy_pointcloud_bridge.yaml"
    )
    first_paths = os.path.join(share, "first_paths")
    first_v2_config = os.path.join(
        v2_share, "config", "first_planner_v2.yaml"
    )
    use_rviz = LaunchConfiguration("use_rviz")
    planner_executable = LaunchConfiguration("planner_executable")

    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "yaml_filename": map_file,
            }
        ],
    )
    map_lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_map",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "autostart": True,
                "node_names": ["map_server"],
            }
        ],
    )
    map_topic_alias = Node(
        package="m20_gazebo",
        executable="m20_map_topic_alias.py",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "input_topic": "/map",
                "grid_map_topic": "/GRID_MAP",
                "local_map_topic": "/local_map",
            }
        ],
    )
    map_to_odom = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="map_to_odom",
        arguments=[
            "--x",
            "0",
            "--y",
            "0",
            "--z",
            "0",
            "--roll",
            "0",
            "--pitch",
            "0",
            "--yaw",
            "0",
            "--frame-id",
            "map",
            "--child-frame-id",
            "odom",
        ],
        parameters=[{"use_sim_time": True}],
    )
    airy_pointcloud_bridge = Node(
        package="m20_gazebo",
        executable="m20_airy_pointcloud_bridge",
        output="screen",
        parameters=[
            airy_bridge_config,
            {"use_sim_time": True},
        ],
    )
    pcl_pass_grid = Node(
        package="m20_gazebo",
        executable="m20_pcl_pass_grid",
        output="screen",
        parameters=[
            pcl_pass_grid_config,
            {"use_sim_time": True},
        ],
    )
    outer_obstacle_cloud = Node(
        package="m20_gazebo",
        executable="m20_outer_obstacle_cloud",
        output="screen",
        parameters=[
            outer_dynamic_config,
            {
                "use_sim_time": True,
                "tracking_frame": "odom",
            },
        ],
    )
    astar_global = Node(
        package="m20_gazebo",
        executable="m20_astar_global_planner.py",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "map_topic": "/map",
                "odom_topic": "/m20/ground_truth",
                "goal_topics": ["/goal_pose", "/target_goal", "/move_base_simple/goal"],
                "path_topic": "/path_Astar",
                "global_path_topic": "/global_path",
                "local_goal_topic": "/local_goal",
                "lookahead_distance": 3.0,
                "inflation_radius": 0.45,
            }
        ],
    )
    first_v2 = Node(
        package="m20_first_planner_v2",
        executable=planner_executable,
        name="m20_first_planner_v2",
        output="screen",
        parameters=[
            first_v2_config,
            {
                "use_sim_time": True,
                "path_folder": first_paths,
                "localplanner_config": os.path.join(first_paths, "localplanner.yaml"),
            }
        ],
    )
    goal_diagnostics = Node(
        package="m20_first_planner_v2",
        executable="m20_v2_goal_diagnostics.py",
        name="m20_v2_goal_diagnostics",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "output_directory": "/home/ubuntu/social_nav_ws/logs/v2_goal_diagnostics",
                "sample_frequency": 10.0,
            }
        ],
    )
    gait_mode = Node(
        package="m20_gazebo",
        executable="m20_gait_mode_publisher.py",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "gait": 12,
                "topic": "/GAIT",
                "publish_frequency": 1.0,
            }
        ],
    )
    true_robot_topics = Node(
        package="m20_gazebo",
        executable="m20_true_robot_topic_bridge.py",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "local_goal_topic": "/local_goal",
                "way_point_topic": "/way_point",
                "speed_topic": "/speed",
                "stop_topic": "/stop",
                "check_obstacle_topic": "/check_obstacle",
                "autonomy_speed": 1.0,
                "publish_frequency": 10.0,
            }
        ],
    )
    reset_robot = Node(
        package="m20_gazebo",
        executable="m20_reset_robot.py",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "model_name": "m20",
                "world_name": "default",
                "x": 0.0,
                "y": 0.0,
                "z": 0.20,
                "yaw": 0.0,
                "stand_duration": 4.5,
                "standing_height_min": 0.45,
            }
        ],
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config],
        output="screen",
        condition=IfCondition(use_rviz),
        parameters=[{"use_sim_time": True}],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument(
                "planner_executable",
                default_value="m20_first_planner_v2_cpp",
            ),
            map_server,
            map_lifecycle_manager,
            map_topic_alias,
            map_to_odom,
            airy_pointcloud_bridge,
            pcl_pass_grid,
            outer_obstacle_cloud,
            gait_mode,
            true_robot_topics,
            reset_robot,
            astar_global,
            first_v2,
            goal_diagnostics,
            TimerAction(period=5.0, actions=[rviz]),
        ]
    )
