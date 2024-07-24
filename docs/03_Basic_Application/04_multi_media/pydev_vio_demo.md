---
sidebar_position: 1
---

# 3.4.1 参考示例（python）

本章节将通过视频流解码等示例程序，介绍地平线Python语言的`hobot_vio`图像多媒体库的使用方法，包括视频拉流、缩放及编解码等操作。

## 视频流解码

本示例代码位于`/app/pydev_demo/08_decode_rtsp_stream/` 目录下，所实现的功能有：
1. 通过opencv打开rtsp码流，获取到码流数据
2. 调用视频解码接口对码流进行解码
3. 把解码后的视频通过HDMI显示

### 运行方法

本示例运行依赖rtsp流，如用户不方便搭建rtsp推流服务，可使用系统预置的推流服务。该服务会把`1080P_test.h264`视频文件处理成rtsp流，url地址为`rtsp://127.0.0.1/1080P_test.h264`。

用户可通过如下命令启动推流服务：

```
cd /app/pydev_demo/08_decode_rtsp_stream/
root@ubuntu:/app/pydev_demo/08_decode_rtsp_stream# sudo ./live555MediaServer &
```

服务正常启动后的log如下， 注意最后一行的 `We use port 80`, 说明rtsp服务运行在80端口，它有可能存在8000和8080的情况，在后面设置rtsp url的时候需要根据实际使用的端口号做修改：
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

然后调用`./decode_rtsp_stream.py `命令，启动拉流解码程序，并将url地址、分辨率、帧率等信息通过控制台输出，log如下：

```shell
root@ubuntu:/app/pydev_demo/08_decode_rtsp_stream# ./decode_rtsp_stream.py 
['rtsp://127.0.0.1/1080P_test.h264']
RTSP stream frame_width:1920, frame_height:1080
Decoder(0, 1) return:0 frame count: 0
Camera vps return:0
Decode CHAN: 0 FPS: 30.34
Display FPS: 31.46
Decode CHAN: 0 FPS: 25.00
Display FPS: 24.98
RTSP stream frame_width:1920, frame_height:1080
```

最后，视频流会通过HDMI接口输出，用户可以通过显示器预览视频画面。

### 选项参数说明

示例程序`decode_rtsp_stream.py`可通过修改启动参数，设置rtsp地址、开关HDMI输出、开关AI推理等功能。参数说明如下：

- **-u**  ： 设置rtsp网络地址，支持输入多个地址，如：`-u "rtsp://127.0.0.1/1080P_test.h264;rtsp://192.168.1.10:8000/1080P_test.h264"`
- **-d**  ： 开启、关闭HDMI的显示输出，不设置时默认开启显示，`-d 0 ` 关闭显示，多路解码时只显示第一路的视频
- **-a**  ： 开启、关闭AI算法推理功能，不设置时默认关闭算法，`-a`开启算法推理，运行目标检测算法

**几种常用的启动方式**

解码默认流并开启HDMI显示
```
sudo ./decode_rtsp_stream.py
```
解码默认流并关闭HDMI显示
```
sudo ./decode_rtsp_stream.py -d 0
```
解码单路rtsp流
```
sudo ./decode_rtsp_stream.py -u "rtsp://x.x.x.x/xxx"
```
解码多路rtsp流
```
sudo ./decode_rtsp_stream.py -u "rtsp://x.x.x.x/xxx;rtsp://x.x.x.x/xxx"
```
解码默认流并使能AI推理
```
sudo ./decode_rtsp_stream.py -a
```

### 注意事项

- 推流服务器推送的rtsp码流里面需要包含`PPS`和`SPS`参数信息，否则会导致开发板解码异常，错误信息如下：
![image-20220728110439753](./image/pydev_vio_demo/image-20220728110439753.png)

- 使用`ffmpeg`打开`.mp4 .avi`等格式的视频文件推流时，需要添加`-vbsf h264_mp4toannexb`选项，以添加码流的`PPS` 和`SPS`信息，例如：

    ```
    ffmpeg -re -stream_loop -1 -i xxx.mp4 -vcodec copy -vbsf h264_mp4toannexb -f rtsp rtsp://192.168.1.195:8554/h264_stream
    ```

- rtsp视频流目前仅支持1080p分辨率

- 不支持使用vlc软件进行rtsp推流，原因是vlc软件不支持添加`PPS`和`SPS`信息
