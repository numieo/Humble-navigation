from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="nav2_amcl",
                executable="amcl",
                name="nav2_amcl",
                prefix=["gdbserver 127.0.0.1:3000"],
                emulate_tty=True
                # namespace="cpp_pubsub",
                # output="screen",
            ),
            # Node(
            #     package="cpp_pubsub",
            #     # namespace="turtlesim1",
            #     executable="talker",
            #     # name="sim",
            # )
        ]
    )
 