#include "barlus_gstreamer_proxy/gstreamer_proxy.hpp"

namespace barlus
{

namespace
{

/// Convert a GStreamer sample to a cv::Mat image
auto gst_to_cv_mat(GstSample * sample) -> cv::Mat
{
  GstCaps * caps = gst_sample_get_caps(sample);
  GstBuffer * buffer = gst_sample_get_buffer(sample);
  GstStructure * structure = gst_caps_get_structure(caps, 0);

  int width;
  int height;
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

}  // namespace

GStreamerProxy::GStreamerProxy(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("gstreamer_proxy", options)
{
}

auto GStreamerProxy::on_configure(const rclcpp_lifecycle::State & /*state*/) -> CallbackReturn
{
  try {
    param_listener_ = std::make_shared<gstreamer_proxy::ParamListener>(get_node_parameters_interface());
    params_ = param_listener_->get_params();
  }
  catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Failed to get GStreamerProxy parameters: %s", e.what());  // NOLINT
    return CallbackReturn::ERROR;
  }

  image_pub_ = create_publisher<sensor_msgs::msg::Image>("/barlus/image_raw", rclcpp::SystemDefaultsQoS());
  camera_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>("/barlus/camera_info", rclcpp::SystemDefaultsQoS());

  gst_init(nullptr, nullptr);

  // The following configurations are intended for low-latency transport
  const std::string video_source = "rtspsrc location=" + address + " latency=0 protocols=tcp";
  const std::string video_codec = "! rtph264depay ! h264parse ! avdec_h264";
  const std::string video_decode = "! decodebin ! videoconvert ! video/x-raw,format=(string)BGR";
  const std::string video_sink_conf = "! queue ! appsink emit-signals=true sync=false max-buffers=1 drop=true";
  const std::string command = video_source + video_codec + video_decode + video_sink_conf;

  GError * error = nullptr;
  pipeline_ = gst_parse_launch(command.c_str(), &error);

  if (error != nullptr) {
    RCLCPP_ERROR(get_logger(), "Failed to create GStreamer pipeline: %s", error->message);  // NOLINT
    g_error_free(error);
    return CallbackReturn::ERROR;
  }

  GstElement * sink = gst_bin_get_by_name(GST_BIN(pipeline_), "appsink0");

  gst_app_sink_set_emit_signals(GST_APP_SINK(sink), true);
  gst_app_sink_set_drop(GST_APP_SINK(sink), true);
  gst_app_sink_set_max_buffers(GST_APP_SINK(sink), 1);
  GstAppSinkCallbacks callbacks = {
    nullptr,
    [](GstAppSink * /*sink*/, gpointer /*user_data*/) -> GstFlowReturn { return GST_FLOW_OK; },  // Pre-roll
    [](GstAppSink * sink, gpointer /*user_data*/) -> void {                                      // Image received
      GstSample * sample = gst_app_sink_pull_sample(sink);
      const cv::Mat image = gst_to_cv_mat(sample);
      image_pub_->publish(cv_mat_to_ros_image(image));
    }};

  gst_element_set_state(pipeline_, GST_STATE_PLAYING);

  return CallbackReturn::SUCCESS;
}

auto GStreamerProxy::on_shutdown(const rclcpp_lifecycle::State & /*state*/) -> CallbackReturn
{
  gst_element_set_state(pipeline_, GST_STATE_NULL);
  gst_object_unref(pipeline_);

  return CallbackReturn::SUCCESS;
}

}  // namespace barlus
