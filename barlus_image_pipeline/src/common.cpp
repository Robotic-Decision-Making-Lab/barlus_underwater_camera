#include "common.hpp"

#include <format>

namespace barlus
{

auto make_gst_command(const std::string & addr) -> std::string
{
  // The following configurations are intended for low-latency transport
  const std::string video_source = std::format("rtspsrc location={} protocols=tcp", addr);
  const std::string video_codec = "! rtph264depay ! h264parse ! avdec_h264";
  const std::string video_decode = "! decodebin ! videoconvert ! video/x-raw,format=(string)BGR";
  const std::string video_sink_conf = "! queue ! appsink emit-signals=true sync=false max-buffers=1 drop=true";
  const std::string command = std::format("{} {} {} {}", video_source, video_codec, video_decode, video_sink_conf);
  return command;
}

auto gst_to_cv_mat(GstSample * sample) -> cv::Mat
{
  GstCaps * caps = gst_sample_get_caps(sample);
  GstBuffer * buffer = gst_sample_get_buffer(sample);
  GstStructure * structure = gst_caps_get_structure(caps, 0);

  int width, height;  // NOLINT
  gst_structure_get_int(structure, "width", &width);
  gst_structure_get_int(structure, "height", &height);

  GstMapInfo map;
  gst_buffer_map(buffer, &map, GST_MAP_READ);

  cv::Mat image(height, width, CV_8UC3, map.data);
  gst_buffer_unmap(buffer, &map);

  return image;
}

auto cv_mat_to_ros_image(const cv::Mat & image) -> sensor_msgs::msg::Image
{
  cv_bridge::CvImage cv_image;
  cv_image.image = image;
  cv_image.encoding = "bgr8";
  return *cv_image.toImageMsg();
}

auto resize_image(const cv::Mat & image, double resize_factor) -> cv::Mat
{
  const int new_width = static_cast<int>(image.cols * resize_factor);
  const int new_height = static_cast<int>(image.rows * resize_factor);

  cv::Mat resized_image;
  cv::resize(image, resized_image, cv::Size(new_width, new_height));

  return resized_image;
}

}  // namespace barlus
