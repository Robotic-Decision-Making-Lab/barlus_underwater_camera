#pragma once

#include <gst/gst.h>

#include <cv_bridge/cv_bridge.hpp>

namespace barlus
{

/// Configure the GStreamer pipeline command.
auto make_gst_command(const std::string & addr) -> std::string;

/// Convert a GStreamer sample to a cv::Mat image
auto gst_to_cv_mat(GstSample * sample) -> cv::Mat;

/// Convert a cv::Mat to a sensor_msgs::msg::Image
auto cv_mat_to_ros_image(const cv::Mat & image) -> sensor_msgs::msg::Image;

/// Resize an image by the given resize factor.
auto resize_image(const cv::Mat & image, double resize_factor) -> cv::Mat;

}  // namespace barlus
