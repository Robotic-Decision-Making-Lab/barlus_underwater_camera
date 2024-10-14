#include "barlus_gstreamer_proxy/gstreamer_proxy.hpp"

namespace barlus
{

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

  // TODO(evan-palmer): Instantiate publishers

  gst_init(nullptr, nullptr);

  const std::string video_source = "rtspsrc location=" + address + " latency=0 protocols=tcp";
  const std::string video_codec = "! rtph264depay ! h264parse ! avdec_h264";
  const std::string video_decode = "! decodebin ! videoconvert ! video/x-raw,format=(string)BGR";
  const std::string video_sink_conf = "! queue ! appsink emit-signals=true sync=false max-buffers=1 drop=true";
  const std::string command = video_source + video_codec + video_decode + video_sink_conf;

  pipeline_ = gst_parse_launch(command.c_str(), nullptr);

  gst_element_set_state(pipe, GST_STATE_PLAYING);

  GstElement * sink = gst_bin_get_by_name(GST_BIN_CAST(pipeline_), "appsink0");

  g_signal_connect(
    sink,
    "new-sample",
    G_CALLBACK([](GstElement * sink, gpointer user_data) {
      GstSample * sample = gst_app_sink_pull_sample(GST_APP_SINK(sink));
      GstBuffer * buffer = gst_sample_get_buffer(sample);
      GstMapInfo map;
      gst_buffer_map(buffer, &map, GST_MAP_READ);

      cv::Mat image = cv::imdecode(cv::Mat(1, map.size, CV_8UC1, map.data), cv::IMREAD_COLOR);
      cv::imshow("GStreamerProxy", image);
      cv::waitKey(1);

      gst_buffer_unmap(buffer, &map);
      gst_sample_unref(sample);

      return GST_FLOW_OK;
    }),
    nullptr);

  return CallbackReturn::SUCCESS;
}

}  // namespace barlus
