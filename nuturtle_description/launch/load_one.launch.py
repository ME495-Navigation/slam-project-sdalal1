"""

Starts all the nodes to visualize a robot in rviz.

Citation-https://github.com/m-elwin/nubot/blob/main/launch/nubot_rviz.launch.py.

"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import (
    DeclareLaunchArgument,
    Shutdown,
    SetLaunchConfiguration,
)
from launch.substitutions import (
    Command,
    PathJoinSubstitution,
    TextSubstitution,
    LaunchConfiguration,
    EqualsSubstitution,
)
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                name="use_jsp",
                default_value="true",
                description="true (default): use joint state publishes,\
                false: no joint states published",
            ),
            DeclareLaunchArgument(
                name="use_rviz",
                default_value="true",
                description="true (default): use rviz,\
                false: no rviz used",
            ),
            DeclareLaunchArgument(
                name="color",
                default_value="purple",
                choices=["purple", "red", "green", "blue"],
                description="purple(default):change turtlebot color to purple,\
                red:change turtlebot color to red,\
                green:change turtlebot color to green,\
                blue:change turtlebot color to blue",
            ),
            SetLaunchConfiguration(
                name="filename", value=["basic_", LaunchConfiguration("color"), ".rviz"]
            ),
            Node(
                package="joint_state_publisher",
                executable="joint_state_publisher",
                namespace=PathJoinSubstitution([LaunchConfiguration("color")]),
                condition=IfCondition(
                    EqualsSubstitution(LaunchConfiguration("use_jsp"), "true")
                ),
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                namespace=PathJoinSubstitution([LaunchConfiguration("color")]),
                condition=IfCondition(
                    EqualsSubstitution(LaunchConfiguration("use_rviz"), "true")
                ),
                arguments=[
                    "-d",
                    PathJoinSubstitution(
                        [
                            FindPackageShare("nuturtle_description"),
                            "config",
                            LaunchConfiguration("filename"),
                        ]
                    ),
                    "-f",
                    PathJoinSubstitution(
                        [
                            LaunchConfiguration("color"),
                            TextSubstitution(text="base_link"),
                        ]
                    ),
                ],
                on_exit=Shutdown(),
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                namespace=PathJoinSubstitution([LaunchConfiguration("color")]),
                parameters=[
                    {
                        "robot_description": Command(
                            [
                                TextSubstitution(text="xacro "),
                                PathJoinSubstitution(
                                    [
                                        FindPackageShare("nuturtle_description"),
                                        "urdf",
                                        "turtlebot3_burger.urdf.xacro",
                                    ]
                                ),
                                TextSubstitution(text=" color:="),
                                LaunchConfiguration("color"),
                            ]
                        ),
                        "frame_prefix": PathJoinSubstitution(
                            [LaunchConfiguration("color"),
                             TextSubstitution(text="")]
                        ),
                    },
                ],
            ),
        ]
    )
