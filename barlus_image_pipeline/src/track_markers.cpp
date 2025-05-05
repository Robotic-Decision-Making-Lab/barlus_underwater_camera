// Copyright 2025, Evan Palmer
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#include "barlus_image_pipeline/track_markers.hpp"

#include <gst/app/app.h>

#include <ranges>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "common.hpp"

namespace barlus
{

TrackMarkersNode::TrackMarkersNode(const rclcpp::NodeOptions & options)
: Node("track_markers_node", options)
{
  param_listener_ = std::make_shared<track_markers_node::ParamListener>(get_node_parameters_interface());
  params_ = param_listener_->get_params();

  // configure the aruco detector
  detector_params_ = cv::makePtr<cv::aruco::DetectorParameters>();
  dictionary_ = cv::makePtr<cv::aruco::Dictionary>(cv::aruco::getPredefinedDictionary(params_.dictionary));

  // set the marker coordinate system using the marker size
  const float size = static_cast<float>(params_.marker_size / 2.0);
  obj_points_ = cv::Mat(4, 1, CV_32FC3);
  obj_points_.ptr<cv::Vec3f>(0)[0] = cv::Vec3f(-size, size, 0);
  obj_points_.ptr<cv::Vec3f>(0)[1] = cv::Vec3f(size, size, 0);
  obj_points_.ptr<cv::Vec3f>(0)[2] = cv::Vec3f(size, -size, 0);
  obj_points_.ptr<cv::Vec3f>(0)[3] = cv::Vec3f(-size, -size, 0);

  // use the camera info manager to get the camera info from a configuration file
  camera_info_ = std::make_unique<sensor_msgs::msg::CameraInfo>();
  camera_info_manager_ =
    std::make_shared<camera_info_manager::CameraInfoManager>(this, params_.camera_name, params_.camera_info_url);

  if (!camera_info_manager_->isCalibrated()) {
    RCLCPP_ERROR(get_logger(), "%s is not calibrated", params_.camera_name.c_str());                 // NOLINT
    RCLCPP_ERROR(get_logger(), "%s requires a calibrated camera for marker detection", get_name());  // NOLINT
    rclcpp::shutdown();
    return;
  }

  // configure the tf broadcaster if enabled
  if (params_.publish_tf) {
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  }

  // configure a publisher for each marker id
  // allow the qos policies to be overridden
  rclcpp::PublisherOptions pub_options;
  pub_options.qos_overriding_options = rclcpp::QosOverridingOptions::with_default_policies();
  for (const auto & marker_id : params_.marker_ids) {
    publisher_map_[marker_id] = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      std::format("marker_{}/pose", marker_id), 10, pub_options);
  }

  if (!configure_stream()) {
    shutdown_stream();
    rclcpp::shutdown();
    return;
  }
}

TrackMarkersNode::~TrackMarkersNode() { shutdown_stream(); }

auto TrackMarkersNode::configure_stream() -> bool
{
  gst_init(nullptr, nullptr);

  RCLCPP_INFO(get_logger(), "Starting GStreamer pipeline with address: %s", params_.stream_address.c_str());  // NOLINT

  // create GStreamer pipeline
  GError * error = nullptr;
  pipeline_ = gst_parse_launch(make_gst_command(params_.stream_address).c_str(), &error);

  if (error != nullptr) {
    RCLCPP_ERROR(get_logger(), "Failed to create GStreamer pipeline. %s", error->message);  // NOLINT
    g_error_free(error);
    return false;
  }

  // configure appsink
  GstElement * sink = gst_bin_get_by_name(GST_BIN(pipeline_), "appsink0");  // NOLINT(bugprone-casting-through-void)
  if (sink == nullptr) {
    RCLCPP_ERROR(get_logger(), "Failed to get appsink from the pipeline");
    return false;
  }

  gst_app_sink_set_emit_signals(GST_APP_SINK_CAST(sink), static_cast<gboolean>(true));
  gst_app_sink_set_drop(GST_APP_SINK_CAST(sink), static_cast<gboolean>(true));
  gst_app_sink_set_max_buffers(GST_APP_SINK_CAST(sink), 1);

  // set the callbacks for the appsink
  GstAppSinkCallbacks callbacks = {
    nullptr,                                                                                     // eos
    [](GstAppSink * /*sink*/, gpointer /*user_data*/) -> GstFlowReturn { return GST_FLOW_OK; },  // new_preroll
    [](GstAppSink * sink, gpointer user_data) -> GstFlowReturn {                                 // new_sample
      auto * self = reinterpret_cast<TrackMarkersNode *>(user_data);
      *self->camera_info_ = self->camera_info_manager_->getCameraInfo();

      // convert the sample to a cv::Mat image
      GstSample * sample = gst_app_sink_pull_sample(GST_APP_SINK_CAST(sink));
      const cv::Mat image_raw = gst_to_cv_mat(sample);

      auto k = self->camera_info_->k;
      auto d = self->camera_info_->d;

      // convert the camera info to a cv::Mat
      cv::Mat intrinsics(3, 3, CV_64FC1, reinterpret_cast<void *>(k.data()));
      cv::Mat dist_coeffs(d.size(), 1, CV_64FC1, reinterpret_cast<void *>(d.data()));

      // undistort the image
      cv::Mat image_undistorted;
      cv::undistort(image_raw, image_undistorted, intrinsics, dist_coeffs);

      // convert the image to grayscale
      cv::Mat image_gray;
      cv::cvtColor(image_undistorted, image_gray, cv::COLOR_BGR2GRAY);

      // detect the markers from the processed image
      std::vector<int> ids;
      std::vector<std::vector<cv::Point2f>> corners, rejected;
      cv::aruco::detectMarkers(image_gray, self->dictionary_, corners, ids, self->detector_params_, rejected);

      // estimate the pose of each marker
      std::size_t n_markers = corners.size();
      std::vector<cv::Vec3d> rvecs(n_markers), tvecs(n_markers);

      if (!ids.empty()) {
        for (std::size_t i = 0; i < n_markers; i++) {
          cv::solvePnP(self->obj_points_, corners[i], intrinsics, dist_coeffs, rvecs[i], tvecs[i]);
        }
      } else {
        RCLCPP_DEBUG(self->get_logger(), "No markers detected");
        gst_sample_unref(sample);
        return GST_FLOW_OK;
      }

      // publish the pose & transform for each marker
      for (const auto & [i, marker_id] : std::views::enumerate(ids)) {
        if (self->publisher_map_.find(marker_id) == self->publisher_map_.end()) {
          continue;
        }

        auto pose = geometry_msgs::msg::PoseStamped();
        pose.header.frame_id = self->params_.frame_id;
        pose.header.stamp = self->now();

        pose.pose.position.x = tvecs[i][0];
        pose.pose.position.y = tvecs[i][1];
        pose.pose.position.z = tvecs[i][2];

        const tf2::Vector3 rvec(rvecs[i][0], rvecs[i][1], rvecs[i][2]);
        const tf2::Quaternion q(rvec.normalized(), rvec.length());
        tf2::convert(q, pose.pose.orientation);

        self->publisher_map_[marker_id]->publish(pose);

        if (self->params_.publish_tf) {
          geometry_msgs::msg::TransformStamped transform;
          transform.header = pose.header;
          transform.child_frame_id = std::format("marker_{}", marker_id);
          transform.transform.translation.x = pose.pose.position.x;
          transform.transform.translation.y = pose.pose.position.y;
          transform.transform.translation.z = pose.pose.position.z;
          transform.transform.rotation = pose.pose.orientation;

          self->tf_broadcaster_->sendTransform(transform);
        }
      }

      gst_sample_unref(sample);
      return GST_FLOW_OK;
    },
    nullptr,  // new_event
    nullptr,  // propose_allocation
    nullptr};
  gst_app_sink_set_callbacks(GST_APP_SINK_CAST(sink), &callbacks, this, nullptr);

  // set the callback for the bus - this lets us handle errors
  GstBus * bus;
  bus = gst_pipeline_get_bus(GST_PIPELINE(pipeline_));  // NOLINT(bugprone-casting-through-void)
  gst_bus_add_watch(
    bus,
    [](GstBus * /*bus*/, GstMessage * message, gpointer user_data) -> gboolean {
      if (GST_MESSAGE_TYPE(message) == GST_MESSAGE_ERROR) {
        auto * self = reinterpret_cast<TrackMarkersNode *>(user_data);
        GError * error;
        gchar * debug;
        gst_message_parse_error(message, &error, &debug);
        RCLCPP_ERROR(self->get_logger(), "Error: %s", error->message);  // NOLINT
        g_error_free(error);
        g_free(debug);
      }
      return static_cast<gboolean>(true);
    },
    this);
  gst_object_unref(bus);

  const GstStateChangeReturn ret = gst_element_set_state(pipeline_, GST_STATE_PLAYING);
  if (ret == GST_STATE_CHANGE_FAILURE) {
    RCLCPP_ERROR(get_logger(), "Failed to start GStreamer pipeline");
    return false;
  }

  return true;
}

auto TrackMarkersNode::shutdown_stream() -> void
{
  if (pipeline_ == nullptr) {
    return;
  }
  gst_element_set_state(pipeline_, GST_STATE_NULL);
  gst_object_unref(pipeline_);
}

}  // namespace barlus

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(barlus::TrackMarkersNode)
