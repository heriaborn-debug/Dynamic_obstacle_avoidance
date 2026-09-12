import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    planner_share = get_package_share_directory("m20_first_planner_v2")
    prediction_share = get_package_share_directory("m20_dynamic_prediction_v2")

    odom_topic = LaunchConfiguration("odom_topic")
    tracked_objects_topic = LaunchConfiguration("tracked_objects_topic")
    cloud_topic = LaunchConfiguration("cloud_topic")
    goal_topic = LaunchConfiguration("goal_topic")
    map_topic = LaunchConfiguration("map_topic")
    cmd_topic = LaunchConfiguration("cmd_topic")
    paths_dir = LaunchConfiguration("paths_dir")
    world_frame = LaunchConfiguration("world_frame")
    base_frame = LaunchConfiguration("base_frame")

    predictor = Node(
        package="m20_dynamic_prediction_v2",
        executable="dynamic_trajectory_predictor_v2",
        name="dynamic_trajectory_predictor_v2",
        output="screen",
        condition=IfCondition(LaunchConfiguration("start_prediction")),
        parameters=[
            os.path.join(prediction_share, "config", "dynamic_prediction_v2.yaml"),
            {
                "use_sim_time": False,
                "input_topic": tracked_objects_topic,
                "odometry_topic": odom_topic,
                "static_map_topic": map_topic,
            },
        ],
    )

    scorer = Node(
        package="m20_dynamic_prediction_v2",
        executable="first_dynamic_scorer_v2",
        name="first_dynamic_scorer_v2",
        output="screen",
        condition=IfCondition(LaunchConfiguration("start_scorer")),
        parameters=[
            os.path.join(prediction_share, "config", "first_dynamic_scorer_v2.yaml"),
            {
                "use_sim_time": False,
                "path_folder": paths_dir,
                "odometry_topic": odom_topic,
                "world_frame": world_frame,
            },
        ],
    )

    planner = Node(
        package="m20_first_planner_v2",
        executable="m20_first_planner_v2_cpp",
        name="m20_first_planner_v2",
        output="screen",
        condition=IfCondition(LaunchConfiguration("start_planner")),
        parameters=[
            os.path.join(planner_share, "config", "first_planner_v2.yaml"),
            {
                "use_sim_time": False,
                "path_folder": paths_dir,
                "localplanner_config": PathJoinSubstitution(
                    [paths_dir, "localplanner.yaml"]
                ),
                "odom_topic": odom_topic,
                "goal_topic": goal_topic,
                "base_frame": base_frame,
                "cloud_topic": cloud_topic,
                "semantic_obstacle_cloud": True,
                "cmd_topic": cmd_topic,
                "require_dynamic_input": False,
                "enable_fallen_height_check": False,
            },
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "odom_topic", default_value="/m20/localization/odom"
            ),
            DeclareLaunchArgument(
                "tracked_objects_topic",
                default_value="/m20/autoware/tracked_objects",
            ),
            DeclareLaunchArgument(
                "cloud_topic",
                default_value="/m20/local_mapping/traversability/obstacle_cloud",
            ),
            DeclareLaunchArgument("goal_topic", default_value="/local_goal"),
            DeclareLaunchArgument("map_topic", default_value="/GRID_MAP"),
            DeclareLaunchArgument("world_frame", default_value="odom"),
            DeclareLaunchArgument("base_frame", default_value="base_link"),
            DeclareLaunchArgument(
                "cmd_topic", default_value="/m20/v2/cmd_vel_candidate"
            ),
            DeclareLaunchArgument(
                "paths_dir",
                default_value="/home/nvidia/m20_v2_nx/m20_first_paths",
            ),
            DeclareLaunchArgument("start_prediction", default_value="false"),
            DeclareLaunchArgument("start_scorer", default_value="false"),
            DeclareLaunchArgument("start_planner", default_value="true"),
            predictor,
            scorer,
            planner,
        ]
    )
