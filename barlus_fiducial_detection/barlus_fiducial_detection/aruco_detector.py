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

import time
from typing import Any

import cv2
import numpy as np
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_system_default,
)
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import CameraInfo, Image

from barlus_fiducial_detection.aruco_detector_parameters import (  # noqa: E402
    aruco_detector_parameters,
)

# TODO(evan-palmer): come back to this: https://chev.me/arucogen/


def to_transformation_matrix(
    x: float, y: float, z: float, rx: float, ry: float, rz: float
) -> np.ndarray:
    transformation_matrix = np.eye(4)
    transformation_matrix[:3, :3] = R.from_euler("xyz", [rx, ry, rz]).as_matrix()
    transformation_matrix[:3, 3] = [x, y, z]
    return transformation_matrix


class ArUcoDetector(Node):
    def __init__(self):
        super().__init__("aruco_detector")

        self.param_listener = aruco_detector_parameters.ParamListener(self)
        self.params = self.param_listener.get_params()

        self.camera_info: CameraInfo | None = None
        self.cv_bridge = CvBridge()

        # Track the state so that we can estimate the velocity using finite differences
        self.last_t = time.time()
        self.last_pose: np.ndarray | None = None

        # Initialize the ArUco detector
        dict_id = getattr(cv2.aruco, self.params.aruco_dictionary)
        aruco_dictionary = cv2.aruco.getPredefinedDictionary(dict_id)
        aruco_params = cv2.aruco.DetectorParameters()
        self.aruco_detector = cv2.aruco.ArucoDetector(aruco_dictionary, aruco_params)

        # Create a homogenous transformation matrix to apply to the marker pose
        self.marker_transform = to_transformation_matrix(
            *[
                getattr(self.params.marker_transform, attr)
                for attr in ["x", "y", "z", "rx", "ry", "rz"]
            ]
        )

        self.camera_sub = self.create_subscription(
            Image, "/barlus/image_raw", self.detect_marker, qos_profile_system_default
        )
        self.camera_info_sub = self.create_subscription(
            Image,
            "/barlus/camera_info",
            self.camera_info_callback,
            QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
            ),
        )

    def save_camera_info(self, camera_info: CameraInfo):
        self.camera_info = camera_info

    def detect_markers(self, frame: np.ndarray) -> tuple[Any, Any] | None:
        try:
            corners, ids, _ = self.detector.detectMarkers(frame)

            if len(ids) > 0:
                return corners, ids
        except cv2.error:
            self.get_logger().debug("Failed to detect markers in the frame")
            return

    def get_marker_pose(self, frame: np.ndarray) -> tuple[Any, Any] | None:
        if self.camera_info is None:
            self.get_logger().debug("Camera info not yet received")
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids = self.detect_markers(gray)

        if corners is None:
            return

        # If there are multiple markers, get the marker with the "longest" side, where
        # "longest" should be interpreted as the relative size in the image
        side_lengths = [
            abs(x1 - x2) + abs(y1 - y2)
            for ((x1, y1), *_, (x2, y2)) in (corner[0] for corner in corners)
        ]

        max_side_idx = side_lengths.index(max(side_lengths))
        marker_id = ids[max_side_idx]

        camera_matrix = np.array(self.camera_info.k, dtype=np.float64).reshape(3, 4)
        projection_matrix = np.array(self.camera_info.d, dtype=np.float64).reshape(1, 5)

        rot_vec, trans_vec, _ = cv2.aruco.estimatePoseSingleMarkers(
            corners[max_side_idx], marker_id, camera_matrix, projection_matrix
        )

        return rot_vec, trans_vec, marker_id

    def estimate_state_cb(self, image: Image):
        camera_pose = self.get_marker_pose(self.cv_bridge.imgmsg_to_cv2(image))

        if camera_pose is None or self.last_pose is None:
            return

        rot_vec, trans_vec, _ = camera_pose
        rot_mat, _ = cv2.Rodrigues(rot_vec)

        tf_camera_to_marker = np.eye(4)
        tf_camera_to_marker[:3, :3] = rot_mat
        tf_camera_to_marker[:3, 3] = trans_vec

        # Apply the marker transformation to the marker pose
        tf_camera_to_shifted_marker = tf_camera_to_marker @ self.marker_transform

        # Approximate the velocity using finite differences
        current_t = time.time()
        dt = current_t - self.last_t

        dT = self.last_pose.T @ tf_camera_to_shifted_marker
        lin_vel = (dT[:3, 3] - self.last_pose[:3, 3]) / dt

        # TODO(evan-palmer): finish implementing this

        # Publish the transform to the TF tree
        tf_msg = TransformStamped()
        tf_msg.header.stamp = self.get_clock().now().to_msg()
        tf_msg.header.frame_id = self.camera_info.header.frame_id
        tf_msg.child_frame_id = self.params.frame_id
        (
            tf_msg.transform.translation.x,
            tf_msg.transform.translation.y,
            tf_msg.transform.translation.z,
        ) = tf_camera_to_shifted_marker[:3, 3]
        (
            tf_msg.transform.rotation.x,
            tf_msg.transform.rotation.y,
            tf_msg.transform.rotation.z,
            tf_msg.transform.rotation.w,
        ) = R.from_matrix(tf_camera_to_shifted_marker[:3, :3]).as_quat()

        # Publish the state estimate
        odom = Odometry()
        # TODO(evan-palmer): Implement this
