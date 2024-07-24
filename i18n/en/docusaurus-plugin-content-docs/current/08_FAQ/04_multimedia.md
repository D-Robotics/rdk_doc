---
sidebar_position: 4
---
# 8.4 Multimedia Class

## Video Encoding/Decoding

<font color='Blue'>[Question]</font>

- When decoding the rtsp video stream on the development board, the following error occurs:  
![image-20220728110439753](./image/multimedia/image-20220728110439753.png)

<font color='Green'>[Answer]</font>

- The rtsp stream pushed by the streaming server needs to include `PPS` and `SPS` parameter information.

- When using `ffmpeg` to stream open video files in formats such as `.mp4` and `.avi`, the `-vbsf h264_mp4toannexb` option needs to be added to include the `PPS` and `SPS` information in the stream, for example:

    ```
    ffmpeg -re -stream_loop -1 -i xxx.mp4 -vcodec copy -vbsf h264_mp4toannexb -f rtsp rtsp://192.168.1.195:8554/h264_stream
    ```

- Currently, the rtsp video stream only supports 1080p resolution.

- Using VLC software for rtsp streaming is not supported because VLC software does not support adding `PPS` and `SPS` information.