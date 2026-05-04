from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_cpp_planner = LaunchConfiguration("use_cpp_planner")
    use_cpp_controller = LaunchConfiguration("use_cpp_controller")

    web_node = Node(
        package="indoor_route_nav",
        executable="web_app",
        name="indoor_route_nav_web_app",
        output="screen",
    )

    planner_node = Node(
        package="indoor_route_nav",
        executable="indoor_route_planner_node",
        name="indoor_route_planner_node",
        output="screen",
        condition=IfCondition(use_cpp_planner),
    )

    controller_node = Node(
        package="indoor_route_nav",
        executable="indoor_route_controller_node",
        name="indoor_route_controller_node",
        output="screen",
        condition=IfCondition(use_cpp_controller),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_cpp_planner",
            default_value="true",
            description="Start the C++ planner extension node.",
        ),
        DeclareLaunchArgument(
            "use_cpp_controller",
            default_value="true",
            description="Start the C++ path tracking controller node.",
        ),
        web_node,
        planner_node,
        controller_node,
    ])
