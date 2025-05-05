# Barlus Image Pipeline

This package implements composable ROS 2 nodes that facilitate integration of
Barlus underwater cameras into a ROS 2 environment.

## Image Proxy Node

The image_proxy_node converts an RTSP video stream into `sensor_msgs/Image`
messages. The node can be composed with the [image_proc](https://github.com/ros-perception/image_pipeline/tree/rolling/image_proc)
nodes. After building the barlus_image_pipeline package, the image_proxy_node
can be launched using

```bash
ros2 launch barlus_image_pipeline image_proxy.launch.py
```

The raw frames will be published to the topic `/barlus/image_raw`. The camera
inrinsics are available on the topic `/barlus/camera_info`.

### Plugin

barlus::ImageProxyNode

## Track Markers Node

The track_markers_node detects ArUco markers in an RTSP video feed and publishes
the poses of the markers. This node is useful in scenarios where you only need
the detected poses and not the video feed in the ROS network. After building
the barlus_image_pipeline package, the track_markers_node can be launched using

```bash
ros2 launch barlus_image_pipeline track_markers.launch.py
```

Each configured marker will have its pose published on a `/barlus/marker_<marker-id>/pose`
topic.

> [!NOTE]
> The track_markers_node is not compatible with the image_proxy_node. If you
> need to perform pose detection with a ROS 2 video feed, then you should use
> the [image_proc::TrackMarkerNode](https://github.com/ros-perception/image_pipeline/blob/rolling/image_proc/include/image_proc/track_marker.hpp).

### Plugin

barlus::TrackMarkersNode
