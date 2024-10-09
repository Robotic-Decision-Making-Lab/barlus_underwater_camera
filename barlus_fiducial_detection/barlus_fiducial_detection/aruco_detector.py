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
from dataclasses import dataclass
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
from tf2_ros import TransformBroadcaster

from barlus_fiducial_detection.aruco_detector_parameters import (  # noqa: E402
    aruco_detector_parameters,
)


@dataclass
class State:
    tf: np.ndarray | None
    t: float


def to_transformation_matrix(
    x: float, y: float, z: float, rx: float, ry: float, rz: float
) -> np.ndarray:
    """Create a homogenous transformation matrix from the given translation and rotation.

    Args:
        x: The translation in the x direction.
        y: The translation in the y direction.
        z: The translation in the z direction.
        rx: The rotation about the x-axis.
        ry: The rotation about the y-axis.
        rz: The rotation about the z-axis.

    Returns:
        The transformation matrix.
    """
    transformation_matrix = np.eye(4)
    transformation_matrix[:3, :3] = R.from_euler("xyz", [rx, ry, rz]).as_matrix()
    transformation_matrix[:3, 3] = [x, y, z]
    return transformation_matrix


def transform_twist(twist: np.ndarray, tf: np.ndarray) -> np.ndarray:
    """Transform a twist from one frame to another.

    See Proposition 3.22 in "Modern Robotics: Mechanics, Planning, and Control" by Kevin
    M. Lynch and Frank C. Park.

    Args:
        twist: The twist to transform.
        tf: The transformation matrix from the desired frame to the current frame.

    Returns:
        The transformed twist.
    """
    rot = tf[:3, :3]
    trans = tf[3, :3]

    skew_trans = np.array(
        [
            [0, -trans[2], trans[1]],
            [trans[2], 0, -trans[0]],
            [-trans[1], trans[0], 0],
        ],
    )

    # Construct the adjoint transformation matrix
    ad_tf = np.zeros((6, 6))
    ad_tf[:3, :3] = rot
    ad_tf[3:, :3] = skew_trans @ rot
    ad_tf[3:, 3:] = rot

    return ad_tf @ twist


def estimate_body_velocity(current_state: State, previous_state: State) -> np.ndarray:
    """Estimate a velocity using finite differences.

    Args:
        current_state: The current state of the marker.
        previous_state: The previous state of the marker.

    Returns:
        The estimated velocity of the marker.
    """
    dt = current_state.t - previous_state.t

    twist_s = np.zeros(6)
    twist_s[:3] = (current_state.tf[:3, 3] - previous_state.tf[:3, 3]) / dt
    twist_s[3:] = (
        R.from_matrix(current_state.tf[:3, :3]).as_euler("xyz")
        - R.from_matrix(previous_state.tf[:3, :3]).as_euler("xyz")
    ) / dt

    return transform_twist(twist_s, current_state.tf.T)


class ArUcoDetector(Node):
    """Detect ArUco markers in the camera feed and estimate the state of the camera.

    Refer to the following link to generate the ArUco markers:
    https://chev.me/arucogen/
    """

    def __init__(self):
        super().__init__("aruco_detector")

        self.param_listener = aruco_detector_parameters.ParamListener(self)
        self.params = self.param_listener.get_params()

        self.camera_info: CameraInfo | None = None
        self.cv_bridge = CvBridge()

        # Track the state so that we can estimate the velocity using finite differences
        # The state information can be used by a Kalman filter or other estimator to
        # filter the state estimate
        self.last_state = State(None, time.time())

        # Initialize the ArUco detector
        dict_id = getattr(cv2.aruco, self.params.aruco_dictionary)
        aruco_dictionary = cv2.aruco.getPredefinedDictionary(dict_id)
        aruco_params = cv2.aruco.DetectorParameters()
        self.aruco_detector = cv2.aruco.ArucoDetector(aruco_dictionary, aruco_params)

        # Create a homogenous transformation matrix to apply to the marker pose
        # This is useful in case the marker is not placed at the COM
        self.marker_transform = to_transformation_matrix(
            *[
                getattr(self.params.marker_transform, attr)
                for attr in ["x", "y", "z", "rx", "ry", "rz"]
            ]
        )

        self.camera_sub = self.create_subscription(
            Image,
            "/barlus/image_raw",
            self.estimate_state_cb,
            qos_profile_system_default,
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

        self.tf_broadcaster = TransformBroadcaster(self)
        self.odom_pub = self.create_publisher(
            Odometry, "/barlus/aruco/odom", qos_profile_system_default
        )

    def save_camera_info(self, camera_info: CameraInfo):
        self.camera_info = camera_info

    def detect_markers(self, frame: np.ndarray) -> tuple[Any, Any] | None:
        """Detect ArUco markers in the given frame.

        Args:
            frame: The OpenCV image to detect markers in.

        Returns:
            A tuple containing the corners and IDs of the detected markers (if any).
        """
        try:
            corners, ids, _ = self.detector.detectMarkers(frame)

            if len(ids) > 0:
                return corners, ids
        except cv2.error:
            self.get_logger().debug("Failed to detect markers in the frame")
            return

    def get_marker_pose(
        self, frame: np.ndarray
    ) -> tuple[np.ndarray, np.ndarray, int] | None:
        """Estimate the pose of the marker in the given frame.

        This can be interpreted as the transformation from the camera frame to the marker,
        i.e., T_{camera}^{marker}.

        Args:
            frame: The OpenCV image to estimate the marker pose in.

        Returns:
            A tuple containing the rotation vector, translation vector, and ID of the
            detected marker (if any).
        """
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
        """Estimate the state of the object in the camera frame.

        This publishes both the TF to the TF tree and the state estimate, including
        position and body velocity.

        Args:
            image: The camera image retrieved from the Barlus camera.
        """
        camera_pose = self.get_marker_pose(self.cv_bridge.imgmsg_to_cv2(image))

        if camera_pose is None or self.last_state.tf is None:
            return

        rot_vec, trans_vec, _ = camera_pose
        rot_mat, _ = cv2.Rodrigues(rot_vec)

        # Apply the marker transformation to the marker pose
        tf_camera_to_marker = np.eye(4)
        tf_camera_to_marker[:3, :3] = rot_mat
        tf_camera_to_marker[:3, 3] = trans_vec
        tf_camera_to_shifted_marker = tf_camera_to_marker @ self.marker_transform

        # Approximate the body velocity using finite differences
        current_state = State(tf_camera_to_shifted_marker, time.time())
        twist_b = estimate_body_velocity(current_state, self.last_state)
        self.last_state = current_state

        # Publish the transform to the TF tree
        tf_msg = TransformStamped()
        tf_msg.header.stamp = self.get_clock().now().to_msg()
        tf_msg.header.frame_id = self.camera_info.header.frame_id
        tf_msg.child_frame_id = self.params.frame_id
        (
            tf_msg.transform.translation.x,
            tf_msg.transform.translation.y,
            tf_msg.transform.translation.z,
        ) = current_state.tf[:3, 3]
        (
            tf_msg.transform.rotation.x,
            tf_msg.transform.rotation.y,
            tf_msg.transform.rotation.z,
            tf_msg.transform.rotation.w,
        ) = R.from_matrix(current_state.tf[:3, :3]).as_quat()
        self.tf_broadcaster.sendTransform(tf_msg)

        # Publish the state estimate
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = self.camera_info.header.frame_id
        odom.child_frame_id = self.params.frame_id
        (
            odom.pose.pose.position.x,
            odom.pose.pose.position.y,
            odom.pose.pose.position.z,
        ) = current_state.tf[:3, 3]
        (
            odom.pose.pose.orientation.x,
            odom.pose.pose.orientation.y,
            odom.pose.pose.orientation.z,
            odom.pose.pose.orientation.w,
        ) = R.from_matrix(current_state.tf[:3, :3]).as_quat()
        odom.pose.covariance = self.params.detection_covariance.pose_covariance
        (
            odom.twist.twist.linear.x,
            odom.twist.twist.linear.y,
            odom.twist.twist.linear.z,
        ) = twist_b[:3]
        (
            odom.twist.twist.angular.x,
            odom.twist.twist.angular.y,
            odom.twist.twist.angular.z,
        ) = twist_b[3:]
        odom.twist.covariance = self.params.detection_covariance.twist_covariance
        self.odom_pub.publish(odom)
