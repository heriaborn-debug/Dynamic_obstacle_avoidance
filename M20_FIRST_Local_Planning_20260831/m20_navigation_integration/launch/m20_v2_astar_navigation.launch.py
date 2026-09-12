from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    integration_share = get_package_share_directory("m20_navigation_integration")
    planner_share = get_package_share_directory("m20_first_planner_v2")
    config = integration_share + "/config/m20_astar.yaml"

    odom = LaunchConfiguration("odom_topic")
    tracked = LaunchConfiguration("tracked_objects_topic")
    cloud = LaunchConfiguration("cloud_topic")
    map_topic = LaunchConfiguration("map_topic")
    cmd = LaunchConfiguration("cmd_topic")
    global_path = LaunchConfiguration("global_path_topic")
    local_goal = LaunchConfiguration("local_goal_topic")
    world_frame = LaunchConfiguration("world_frame")

    astar = Node(
        package="m20_navigation_integration",
        executable="m20_astar_global_planner.py",
        name="m20_astar_global_planner",
        output="screen",
        parameters=[config, {
            "map_topic": map_topic,
            "odom_topic": odom,
            "global_path_topic": global_path,
        }],
    )
    lookahead = Node(
        package="m20_navigation_integration",
        executable="m20_path_lookahead.py",
        name="m20_path_lookahead",
        output="screen",
        parameters=[config, {
            "global_path_topic": global_path,
            "odom_topic": odom,
            "local_goal_topic": local_goal,
        }],
    )
    v2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            planner_share + "/launch/m20_first_v2_nx.launch.py"
        ),
        launch_arguments={
            "odom_topic": odom,
            "tracked_objects_topic": tracked,
            "cloud_topic": cloud,
            "goal_topic": local_goal,
            "map_topic": map_topic,
            "cmd_topic": cmd,
            "world_frame": world_frame,
            "base_frame": "base_link_dog",
        }.items(),
    )
    return LaunchDescription([
        DeclareLaunchArgument("odom_topic", default_value="/lio/robo/odom"),
        DeclareLaunchArgument("tracked_objects_topic", default_value="/m20/perception/tracked_objects"),
        DeclareLaunchArgument(
            "cloud_topic",
            default_value="/m20/local_mapping/traversability/obstacle_cloud",
        ),
        DeclareLaunchArgument("map_topic", default_value="/map"),
        DeclareLaunchArgument("global_path_topic", default_value="/m20/navigation/global_path"),
        DeclareLaunchArgument("local_goal_topic", default_value="/local_goal"),
        DeclareLaunchArgument("world_frame", default_value="world"),
        DeclareLaunchArgument("cmd_topic", default_value="/cmd_vel"),
        astar,
        lookahead,
        v2,
    ])
