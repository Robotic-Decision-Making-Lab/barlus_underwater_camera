# Copyright 2025, Evan Palmer
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
            [FindPackageShare("barlus_image_pipeline"), "config", "track_markers.yaml"]
        ),
    )

    track_markers_node = ComposableNode(
        package="barlus_image_pipeline",
        plugin="barlus::TrackMarkersNode",
        namespace=LaunchConfiguration("ns"),
        name="track_markers_node",
        parameters=[LaunchConfiguration("parameters_file")],
        extra_arguments=[{"use_intra_process_comms": True}],
    )

    container = ComposableNodeContainer(
        name="barlus_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[track_markers_node],
    )

    return LaunchDescription([declare_parameters_file, declare_ns, container])
