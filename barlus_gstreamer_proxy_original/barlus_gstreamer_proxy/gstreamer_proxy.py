# Copyright 2024, Evan Palmer
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

from typing import Any

import cv2 as cv
import gi
import gi.repository
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_system_default,
)
from sensor_msgs.msg import CameraInfo, Image

gi.require_version("Gst", "1.0")
from gi.repository import Gst  # noqa: E402

from barlus_gstreamer_proxy.gstreamer_proxy_parameters import (  # noqa: E402
    gstreamer_proxy_parameters,
)


def gst_to_numpy(sample: Any) -> np.ndarray:
    """Convert a GStreamer sample to a numpy array."""
    buf = sample.get_buffer()
    caps = sample.get_caps()

    return np.ndarray(
        (
            caps.get_structure(0).get_value("height"),
            caps.get_structure(0).get_value("width"),
            3,
        ),
        buffer=buf.extract_dup(0, buf.get_size()),
        dtype=np.uint8,
    )


class GStreamerProxy(Node):
    def __init__(self) -> None:
        super().__init__("gstreamer_proxy")

        self.param_listener = gstreamer_proxy_parameters.ParamListener(self)
        self.params = self.param_listener.get_params()

        self.image_pub = self.create_publisher(
            Image, "/barlus/image_raw", qos_profile_system_default
        )
        self.camera_info_pub = self.create_publisher(
            CameraInfo,
            "/barlus/camera_info",
            QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
            ),
        )

        camera_info = CameraInfo()
        camera_info.header.stamp = self.get_clock().now().to_msg()
        camera_info.header.frame_id = "/barlus/camera_link"
        camera_info.height = self.params.frame.height
        camera_info.width = self.params.frame.width
        camera_info.distortion_model = self.params.distortion_model
        camera_info.d = self.params.distortion_coefficients
        camera_info.k = self.params.camera_matrix
        camera_info.p = self.params.projection_matrix

        # This uses transient local durability, so the message will persist for future
        # subscribers
        self.camera_info_pub.publish(camera_info)

        # Configure the GStreamer pipeline
        self.bridge = CvBridge()
        self.video_pipe, self.video_sink = self.configure_stream(
            self.params.stream_address
        )

    def configure_stream(self, address: str) -> tuple[Any, Any]:
        Gst.init(None)

        video_source = f"rtspsrc location={address} latency=0 protocols=tcp"
        video_codec = "! rtph264depay ! h264parse ! avdec_h264"
        video_decode = "! decodebin ! videoconvert ! video/x-raw,format=(string)BGR"
        video_sink_conf = (
            "! queue ! appsink emit-signals=true sync=false max-buffers=1 drop=true"
        )

        command = " ".join([video_source, video_codec, video_decode, video_sink_conf])

        video_pipe = Gst.parse_launch(command)
        video_pipe.set_state(Gst.State.PLAYING)
        video_sink = video_pipe.get_by_name("appsink0")

        def proxy_frame_cb(sink: Any) -> gi.repository.Gst.FlowReturn:
            frame = gst_to_numpy(sink.emit("pull-sample"))
            grey = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
            self.get_logger().info(f"Frame shape: {grey.shape}")
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(grey, encoding="mono8"))
            return gi.repository.Gst.FlowReturn.OK

        video_sink.connect("new-sample", proxy_frame_cb)

        return video_pipe, video_sink


def main(args=None):
    rclpy.init(args=args)
    proxy_node = GStreamerProxy()
    rclpy.spin(proxy_node)
    proxy_node.destroy_node()
    rclpy.shutdown()
