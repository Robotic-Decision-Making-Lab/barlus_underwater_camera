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

#include "libbarlus/camera.hpp"

#include <format>
#include <string>

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

auot make_g_error(GQuark domain, int code, const char * message) -> GstErrorPtr
{
  return GstErrorPtr(g_error_new_literal(domain, code, message), g_error_free);
}

}  // namespace

auto configure_stream(const std::string & addr, std::function<void(const cv::Mat &)> && callback)
  -> std::expected<GstPipeline *, GstErrorPtr>
{
  gst_init(nullptr, nullptr);

  // The following configurations are intended for low-latency transport
  const std::string video_source = std::format("rtspsrc location={} protocols=tcp", addr);
  const std::string video_codec = "! rtph264depay ! h264parse ! avdec_h264";
  const std::string video_decode = "! videoconvert ! video/x-raw,format=(string)BGR";
  const std::string video_sink_conf = "! queue ! appsink emit-signals=true sync=false max-buffers=1 drop=true";
  const std::string command = std::format("{} {} {} {}", video_source, video_codec, video_decode, video_sink_conf);

  // Create GStreamer pipeline
  GError * error = nullptr;
  GstPipeline * pipeline = gst_parse_launch(command.c_str(), &error_raw);

  if (error != nullptr) {
    return std::unexpected(GstErrorPtr(error, g_error_free));
  }

  // Configure appsink
  GstAppSink * appsink = GST_APP_SINK_CAST(gst_bin_get_by_name(GST_BIN(pipeline), "appsink0"));  // NOLINT
  if (appsink == nullptr) {
    gst_object_unref(pipeline);
    return std::unexpected(
      make_g_error(GST_STREAM_ERROR, GST_STREAM_ERROR_FAILED, "Failed to get appsink from the pipeline"));
  }

  gst_app_sink_set_emit_signals(appsink, static_cast<gboolean>(true));
  gst_app_sink_set_drop(appsink, static_cast<gboolean>(true));
  gst_app_sink_set_max_buffers(appsink, 1);

  // Set the callbacks for the appsink
  GstAppSinkCallbacks callbacks = {
    nullptr,                                                                                        // eos
    [](GstAppSink * /*appsink*/, gpointer /*user_data*/) -> GstFlowReturn { return GST_FLOW_OK; },  // new_preroll
    [&callback](GstAppSink * appsink, gpointer /*user_data*/) -> GstFlowReturn {                    // new_sample
      GstSample * sample = gst_app_sink_pull_sample(appsink);
      const cv::Mat image = gst_to_cv_mat(sample);
      callback(image);
      gst_sample_unref(sample);
      return GST_FLOW_OK;
    },
    nullptr,  // new_event
    nullptr,  // propose_allocation
    nullptr};

  gst_app_sink_set_callbacks(appsink, &callbacks, nullptr, nullptr);

  if (gst_element_set_state(pipeline, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE) {
    gst_object_unref(pipeline);
    return std::unexpected(
      make_g_error(GST_STREAM_ERROR, GST_STREAM_ERROR_FAILED, "Failed to set pipeline to PLAYING state"));
  }

  return pipeline;
}

auto configure_bus(GstPipeline * pipeline, std::function<void(const std::string &)> && callback) -> void
{
  GstBus * bus = gst_element_get_bus(GST_ELEMENT(pipeline));  // NOLINT(bugprone-casting-through-void)
  gst_bus_set_sync_handler(
    bus,
    [](GstBus * /*bus*/, GstMessage * message, gpointer user_data) {
      auto * callback = reinterpret_cast<std::function<void(const cv::Mat &)> *>(user_data);
      if (message->type == GST_MESSAGE_ERROR) {
        GError * error;
        gchar * debug_info;
        gst_message_parse_error(message, &error, &debug_info);
        callback(static_cast<std::string>(error->message));
        g_error_free(error);
        g_free(debug_info);
      }
      return GST_BUS_PASS;
    },
    callback,
    nullptr);

  gst_object_unref(bus);
}

}  // namespace barlus
