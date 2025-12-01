---
sidebar_position: 1
---

# 3.4.1 Reference Examples (Python)

## MIPI Camera
Please refer to [Algorithm Examples | MIPI Camera section](/rdk_s/Algorithm_Application/Python_Sample/mipi_camera_yolov5x)

<!--
This section introduces the usage of D-Robotics' Python `hobot_vio` image and multimedia library through example programs such as video stream decoding, covering operations like video streaming, scaling, encoding, and decoding.

## Video Stream Decoding

The example code is located under the directory `/app/pydev_demo/07_decode_rtsp_stream/` and implements the following functionalities:
1. Use OpenCV to open an RTSP stream and obtain the stream data.
2. Call the video decoding interface to decode the stream.
3. Output the decoded video via HDMI.

### Running Instructions

This example requires an RTSP stream. If users find it inconvenient to set up their own RTSP streaming service, they can use the preconfigured streaming service provided by the system. This service converts the video file `1080P_test.h264` into an RTSP stream accessible at the URL `rtsp://127.0.0.1/assets/1080P_test.h264`.

Users can start the streaming service with the following command:

```
cd /app/res
sunrise@ubuntu:/app/res# sudo ./live555MediaServer &
```

After successful startup, the log output appears as follows. Note the last line: `We use port 80`, indicating that the RTSP service runs on port 80. However, ports 8000 or 8080 might also be used; thus, the actual port number must be reflected when configuring the RTSP URL later:
```bash
sunrise@ubuntu:/app/res#
LIVE555 Media Server version 1.01 (LIVE555 Streaming Media library version 2020.07.09).
Play streams from this server using the URL
        rtsp://192.168.127.10/<filename>
where <filename> is a file present in the current directory.
Each file's type is inferred from its name suffix:
        ".264" => a H.264 Video Elementary Stream file
... omitted ...
(We use port 80 for optional RTSP-over-HTTP tunneling, or for HTTP live streaming (for indexed Transport Stream files only).)
```

Next, run the command `./decode_rtsp_stream.py` to launch the stream decoding program. The console will output information such as the URL, resolution, and frame rate, as shown in the following log:

```shell
sunrise@ubuntu:/app/pydev_demo/07_decode_rtsp_stream# ./decode_rtsp_stream.py
['rtsp://127.0.0.1/assets/1080P_test.h264']
RTSP stream frame_width:1920, frame_height:1080
Decoder(0, 1) return:0 frame count: 0
Camera vps return:0
Decode CHAN: 0 FPS: 30.34
Display FPS: 31.46
Decode CHAN: 0 FPS: 25.00
Display FPS: 24.98
RTSP stream frame_width:1920, frame_height:1080
```

Finally, the video stream will be output through the HDMI interface, allowing users to preview the video on a connected display.

:::info
Note that the `127.0.0.1` part in the URL above must be updated with the actual port number printed when `live555MediaServer` starts. For example:
```shell
# final output of live555MediaServer
...
(We use port 80 for optional RTSP-over-HTTP tunneling, or for HTTP live streaming (for indexed Transport Stream files only).)
...

# Actual command for decode_rtsp_stream.py
sunrise@ubuntu:/app/pydev_demo/07_decode_rtsp_stream# ./decode_rtsp_stream.py -u rtsp://127.0.0.1:80/assets/1080P_test.h264
```
:::

### Command-line Options

The example program `decode_rtsp_stream.py` supports various startup parameters to configure the RTSP address, enable/disable HDMI output, enable/disable AI inference, etc. The parameters are described below:

- **-u**: Specifies the RTSP network address. Multiple addresses can be provided, e.g.,  
  `-u "rtsp://127.0.0.1/assets/1080P_test.h264;rtsp://192.168.1.10:8000/assets/1080P_test.h264"`
- **-d**: Enables or disables HDMI display output. Display is enabled by default if this option is omitted. Use `-d 0` to disable display; when decoding multiple streams, only the first stream will be displayed.
- **-a**: Enables or disables AI inference. AI inference is disabled by default if this option is omitted. Use `-a` to enable AI inference (runs object detection algorithm).

**Common Usage Examples**

Decode the default stream and enable HDMI display:
```
sudo ./decode_rtsp_stream.py
```

Decode the default stream and disable HDMI display:
```
sudo ./decode_rtsp_stream.py -d 0
```

Decode a single RTSP stream:
```
sudo ./decode_rtsp_stream.py -u "rtsp://x.x.x.x/xxx"
```

Decode multiple RTSP streams:
```
sudo ./decode_rtsp_stream.py -u "rtsp://x.x.x.x/xxx;rtsp://x.x.x.x/xxx"
```

Decode the default stream and enable AI inference:
```
sudo ./decode_rtsp_stream.py -a
```

### Notes

- The RTSP stream pushed by the streaming server must include `PPS` and `SPS` parameter information; otherwise, decoding on the development board will fail, resulting in an error similar to the one shown below:
![image-20220728110439753](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/08_FAQ/image/multimedia/image-20220728110439753.png)

- When using `ffmpeg` to stream video files in formats such as `.mp4` or `.avi`, you must add the `-vbsf h264_mp4toannexb` option to embed `PPS` and `SPS` information into the stream. For example:

    ```
    ffmpeg -re -stream_loop -1 -i xxx.mp4 -vcodec copy -vbsf h264_mp4toannexb -f rtsp rtsp://192.168.1.195:8554/h264_stream
    ```

- Currently, RTSP video streams are only supported at 1080P resolution.

- RTSP streaming using VLC software is not supported because VLC does not include `PPS` and `SPS` information in its streams.
-->