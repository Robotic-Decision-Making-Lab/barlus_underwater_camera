# Copyright 2024, Evan Palmer
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declare_ns = DeclareLaunchArgument("ns", default_value="barlus")
    declare_parameters_file = DeclareLaunchArgument(
        "parameters_file",
        default_value=PathJoinSubstitution(
            [
                FindPackageShare("barlus_gstreamer_proxy"),
                "config",
                "gstreamer_proxy.yaml",
            ]
        ),
    )

    gstreamer_proxy_node = ComposableNode(
        package="barlus_gstreamer_proxy",
        plugin="barlus::GStreamerProxy",
        namespace=LaunchConfiguration("ns"),
        name="gstreamer_proxy",
        parameters=[LaunchConfiguration("parameters_file")],
        extra_arguments=[{"use_intra_process_comms": True}],
    )

    rectify_node = ComposableNode(
        package="image_proc",
        plugin="image_proc::RectifyNode",
        name="rectify_node",
        namespace=LaunchConfiguration("ns"),
        remappings=[("image", "image_raw")],
    )

    debayer_node = ComposableNode(
        package="image_proc",
        plugin="image_proc::DebayerNode",
        namespace=LaunchConfiguration("ns"),
        name="debayer_node",
        remappings=[("image_raw", "image_raw")],
    )

    container = ComposableNodeContainer(
        name="barlus_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[gstreamer_proxy_node, rectify_node, debayer_node],
    )

    return LaunchDescription([declare_parameters_file, declare_ns, container])
