# Barlus Underwater Camera

This package provides configurations and a ROS 2 interface for interacting with
the Barlus UW-S5-3PBX10 underwater camera.

## Network Configuration

The default network configurations for the Barlus UW-S5-3PBX10 are as follows:

```bash
Address: 192.168.1.10 # Verify this using the Barlus VCS software
Netmask: 255.255.255.0
Gateway: 192.168.1.1
```

In order to connect to the camera, add a new IPv4 network configuration with
the same Netmask and Gateway used by the camera. For example,

```bash
Address: 192.168.1.17
Netmask: 255.255.255.0
Gateway: 192.168.1.1
```

## Using MediaMTX proxy

MediaMTX can be used to connect to the RTSP server embedded into the camera to
multiplex the camera stream.

Prior to launching MediaMTX, first check the MediaMTX configurations set in
the `compose/mediamtx/mediamtx.yml` file. In particular, confirm that the
`barlus` RTSP path uses the correct IP address and authentication information
for your specific camera setup.

After verifying the MediaMTX configurations, launch MediaMTX with `docker compose`

```bash
cd /path/to/compose/mediamtx \
docker compose up
```

Once running, the stream can be played using FFmpeg

```bash
ffplay rtsp://<your-ip-address>:<mediamtx-port>/barlus
```

For low latency FFmpeg streaming, run the following

```bash
ffplay  -fflags nobuffer -flags low_delay -probesize 32 -rtsp_transport udp -i rtsp://<your-ip-address>:<mediamtx-port>/barlus
```

The stream can also be recorded using

```bash
ffmpeg -i rtsp://<your-ip-address>:<mediamtx-port>/barlus -c:v copy -an recording.mp4
```

## ROS 2 interface

`barlus_gstreamer_proxy`, a light wrapper around GStreamer has been implemented
to convert frames received from the camera into ROS 2 `sensor_msgs/Image`
messages. After building `barlus_gstreamer_proxy`, the node can be launched
using

```bash
ros2 launch barlus_gstreamer_proxy gstreamer_proxy.launch.yaml
```

The frames will be published to the topic `/barlus/image_raw`. The camera
intrinsics are available on the topic `/barlus/camera_info`.
