---
sidebar_position: 1
---
# 3.4.1 Reference Examples (python)

In this chapter, we will introduce the usage of the `hobot_vio` image and multimedia library in D-Robotics Python language through examples such as video stream decoding. This includes operations such as video streaming, scaling, and encoding/decoding.

## Video Stream Decoding

The example code for this is located in the `/app/pydev_demo/08_decode_rtsp_stream/` directory. The implemented functionalities are:

1. Open the RTSP stream using OpenCV and get the stream data.
2. Decode the stream using the video decoding interface.
3. Display the decoded video using HDMI.

### Running Method

This example requires an RTSP stream to run. If the user cannot set up an RTSP streaming service, they can use the system's pre-installed streaming service. This service converts the `1080P_test.h264` video file into an RTSP stream with the URL `rtsp://127.0.0.1/1080P_test.h264`.

Users can start the streaming service using the following command:

```
cd /app/pydev_demo/08_decode_rtsp_stream/
root@ubuntu:/app/pydev_demo/08_decode_rtsp_stream# sudo ./live555MediaServer &
```

If the service starts successfully, the log will show `We use port 80` in the last line, indicating that the RTSP service is running on port 80. It might also have cases where it runs on ports 8000 and 8080. When setting the RTSP URL later, you need to modify it according to the actual port number used.

```bash
root@ubuntu:/app/pydev_demo/08_decode_rtsp_stream# 
LIVE555 Media Server version 1.01 (LIVE555 Streaming Media library version 2020.07.09).
Play streams from this server using the URL
        rtsp://192.168.1.10/<filename>
where <filename> is a file present in the current directory.
...
...
(We use port 80 for optional RTSP-over-HTTP tunneling, or for HTTP live streaming (for indexed Transport Stream files only).)
```

Then, execute the command `./decode_rtsp_stream.py` to start the streaming decoding program. The URL, resolution, frame rate, and other information will be displayed through the console output. The log will be as follows:

```shell
root@ubuntu:/app/pydev_demo/08_decode_rtsp_stream# ./decode_rtsp_stream.py 
['rtsp://127.0.0.1/1080P_test.h264']
RTSP stream frame_width:1920, frame_height:1080
Decoder(0, 1) return:0 frame count: 0
Camera vps return:0
Decode CHAN: 0 FPS: 30.34
Display FPS: 31.46
Decode CHAN: 0 FPS: 25.00
Display FPS: 24.98RTSP stream frame_width:1920, frame_height:1080
```

Finally, the video stream will be output via the HDMI interface, and users can preview the video image through a monitor.

### Option Parameter Description

The sample program `decode_rtsp_stream.py` can be modified by changing the startup parameters to set the RTSP address, switch HDMI output, switch AI inference, and other functions. The parameter descriptions are as follows:

- **-u**: Set the RTSP network address, supports multiple addresses input, such as: `-u "rtsp://127.0.0.1/1080P_test.h264;rtsp://192.168.1.10:8000/1080P_test.h264"`
- **-d**: Turn on or off the HDMI display output. If not set, the display is enabled by default. `-d 0` disables the display. When decoding multiple streams, only the video of the first stream is displayed.
- **-a**: Turn on or off the AI algorithm inference function. If not set, the algorithm is disabled by default. `-a` enables the algorithm inference and runs the target detection algorithm.

**Several common startup methods**

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

### Precautions

- The RTSP stream pushed by the streaming server needs to include the `PPS` and `SPS` parameter information, otherwise it will cause decoding abnormalities on the development board, and the error message is as follows:
![image-20220728110439753](./image/pydev_vio_demo/image-20220728110439753.png)

- When using `ffmpeg` to open video files in `.mp4 .avi` and other formats for streaming, the `-vbsf h264_mp4toannexb` option needs to be added to add the `PPS` and `SPS` information of the stream. For example:

    ```
    ffmpeg -re -stream_loop -1 -i xxx.mp4 -vcodec copy -vbsf h264_mp4toannexb -f rtsp rtsp://192.168.1.195:8554/h264_stream
    ```

- The RTSP video stream currently only supports 1080p resolution.

- Using VLC software for RTSP streaming is not supported because VLC software does not support adding `PPS` and `SPS` information.