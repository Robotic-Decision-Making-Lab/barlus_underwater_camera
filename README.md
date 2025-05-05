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

After establishing a connection, verify the RTSP stream using FFmpeg

```bash
ffplay -fflags nobuffer -flags low_delay -probesize 32 -rtsp_transport tcp -i "rtsp://192.168.1.10:554/user=admin&password=&channel=1&stream=0.sdp?"
```

The stream can also be recorded using

```bash
ffmpeg -i "rtsp://192.168.1.10:554/user=admin&password=&channel=1&stream=0.sdp?" -c:v copy -an recording.mp4
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
ffplay  -fflags nobuffer -flags low_delay -probesize 32 -rtsp_transport udp -i rtsp://<your-ip-address>:<mediamtx-port>/barlus
```

## ROS 2 interface

`barlus_image_pipeline` has been implemented to support camera integration into
a ROS 2 environment. Please see the package documentation for further
information on the implemented nodes.

## Camera calibration

The camera intrinsics can be retrieved using the [camera_calibration](https://docs.ros.org/en/rolling/p/camera_calibration/index.html)
node. For example, to calibrate the camera using an [8x6 chessboard](https://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration?action=AttachFile&do=view&target=check-108.pdf),
run the following:

```bash
ros2 run camera_calibration cameracalibrator --size 8x6 --square 0.108 image:=/barlus/image_mono camera:=/barlus --no-service-check
```
