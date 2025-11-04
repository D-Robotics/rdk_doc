---
sidebar_position: 11
---

# 3.3.11 rtsp 推流解码示例介绍

## 示例简介
RTSP 流解码示例是一个位于 `/app/pydev_demo/08_decode_rtsp_stream/` 中的 **Python 接口** 开发代码示例，用于演示如何从 RTSP 视频流获取 H.264/H.265 码流，通过硬件解码、视频处理和 AI 推理，实现实时视频解码和目标检测功能。该示例展示了完整的视频处理流水线，包括 RTSP 流接收、硬件解码、视频处理、 AI 推理和结果显示。

## 效果展示
![output-img](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/pydev_08_runing.png)

## 硬件准备

### 硬件连接
1. 准备一个 RDK 开发板
2. 通过 HDMI 线连接显示器和开发板
3. 连接网线到开发板
4. 连接电源线
![connect-img](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/pydev_08_hw_connect.png)

## 快速开始

### 代码以及板端位置
进入到 `/app/pydev_demo/08_decode_rtsp_stream/` 位置，可以看到 RTSP 流解码示例包含以下文件：
```
root@ubuntu:/app/pydev_demo/08_decode_rtsp_stream# tree
.
├── 1080P_test.h264
├── decode_rtsp_stream.py
└── live555MediaServer
```

### 编译以及运行
我们先做好准备工作，如果要通过 HDMI 显示，通过 `systemctl stop lightdm` 命令关闭图形界面服务达到最佳显示效果。\
示例里面默认有 1080P_test.h264 文件，如果想要尝试 h265 格式的文件，可以从板端其他地方拷贝过来，比如 `/opt/tros/humble/lib/hobot_codec/config/1920x1080.h265` 目录下的文件。
首先需要启动 RTSP 流媒体服务器，然后运行 Python 脚本：

### 执行效果
```bash

# 关闭图形界面服务达到最佳显示效果
systemctl stop lightdm

# 启动 RTSP 流媒体服务器
./live555MediaServer &

# 运行 RTSP 流解码示例（h264）
python3 decode_rtsp_stream.py -u rtsp://127.0.0.1/1080P_test.h264 -d 1 -a 1

```
运行后，程序会连接 RTSP 流媒体服务器，解码视频流并进行目标检测，结果会通过 HDMI 显示：

```
root@ubuntu:/app/pydev_demo/08_decode_rtsp_stream#./live555MediaServer &
        version 1.01 (LIVE555 Streaming Media library version 2020.07.09).
Play streams from this server using the URL
        rtsp://192.168.127.10/<filename>
where <filename> is a file present in the current directory.
Each file's type is inferred from its name suffix:
        ".264" => a H.264 Video Elementary Stream file
        ".265" => a H.265 Video Elementary Stream file
        ".aac" => an AAC Audio (ADTS format) file
        ".ac3" => an AC-3 Audio file
        ".amr" => an AMR Audio file
        ".dv" => a DV Video file
        ".m4e" => a MPEG-4 Video Elementary Stream file
        ".mkv" => a Matroska audio+video+(optional)subtitles file
        ".mp3" => a MPEG-1 or 2 Audio file
        ".mpg" => a MPEG-1 or 2 Program Stream (audio+video) file
        ".ogg" or ".ogv" or ".opus" => an Ogg audio and/or video file
        ".ts" => a MPEG Transport Stream file
                (a ".tsx" index file - if present - provides server 'trick play' support)
        ".vob" => a VOB (MPEG-2 video with AC-3 audio) file
        ".wav" => a WAV Audio file
        ".webm" => a WebM audio(Vorbis)+video(VP8) file
See http://www.live555.com/mediaServer/ for additional documentation.
(We use port 80 for optional RTSP-over-HTTP tunneling, or for HTTP live streaming (for indexed Transport Stream files only).)


root@ubuntu:/app/pydev_demo/08_decode_rtsp_stream# ./decode_rtsp_stream.py 
['rtsp://127.0.0.1/1080P_test.h264']
Encoding detected via FourCC: h264, dec_type: 1
RTSP stream frame_width:1920, frame_height:1080
Decoder(0, 1) return:0 frame count: 0
Opened DRM device: /dev/dri/card0

.............
.............
.............

```
如果想尝试解码 h265 文件 ， 可以参考如下命令

```bash

# 关闭图形界面服务达到最佳显示效果
systemctl stop lightdm

# 拷贝 h265 文件到示例目录。
cp /opt/tros/humble/lib/hobot_codec/config/1920x1080.h265 /app/pydev_demo/08_decode_rtsp_stream/

# 运行 RTSP 流解码示例（h265）
python3 decode_rtsp_stream.py -u rtsp://127.0.0.1/1920x1080.h265 -d 1
```

```
root@ubuntu:/app/pydev_demo/08_decode_rtsp_stream# systemctl stop lightdm
root@ubuntu:/app/pydev_demo/08_decode_rtsp_stream# ./live555MediaServer &
[1] 4030
LIVE555 Media Server
        version 1.01 (LIVE555 Streaming Media library version 2020.07.09).
root@ubuntu:/app/pydev_demo/08_decode_rtsp_stream# Play streams from this server using the URL
        rtsp://10.0.0.32/<filename>
where <filename> is a file present in the current directory.
Each file's type is inferred from its name suffix:
        ".264" => a H.264 Video Elementary Stream file
        ".265" => a H.265 Video Elementary Stream file
        ".aac" => an AAC Audio (ADTS format) file
        ".ac3" => an AC-3 Audio file
        ".amr" => an AMR Audio file
        ".dv" => a DV Video file
        ".m4e" => a MPEG-4 Video Elementary Stream file
        ".mkv" => a Matroska audio+video+(optional)subtitles file
        ".mp3" => a MPEG-1 or 2 Audio file
        ".mpg" => a MPEG-1 or 2 Program Stream (audio+video) file
        ".ogg" or ".ogv" or ".opus" => an Ogg audio and/or video file
        ".ts" => a MPEG Transport Stream file
                (a ".tsx" index file - if present - provides server 'trick play' support)
        ".vob" => a VOB (MPEG-2 video with AC-3 audio) file
        ".wav" => a WAV Audio file
        ".webm" => a WebM audio(Vorbis)+video(VP8) file
See http://www.live555.com/mediaServer/ for additional documentation.
(We use port 80 for optional RTSP-over-HTTP tunneling, or for HTTP live streaming (for indexed Transport Stream files only).)

root@ubuntu:/app/pydev_demo/08_decode_rtsp_stream#
root@ubuntu:/app/pydev_demo/08_decode_rtsp_stream#
root@ubuntu:/app/pydev_demo/08_decode_rtsp_stream# python3 decode_rtsp_stream.py -u rtsp://127.0.0.1/1920x1080.h265 -d 1
['rtsp://127.0.0.1/1920x1080.h265']
Encoding detected via FourCC: hevc, dec_type: 2
RTSP stream frame_width:1920, frame_height:1080
Decoder(0, 2) return:0 frame count: 0
Opened DRM device: /dev/dri/card0
1920x1080
1280x800
1280x720
720x576
720x480
640x480
Resolution 1920x1080 exists in the list.
Opened DRM device: /dev/dri/card0
DRM is available, using libdrm for rendering.
------------------------------------------------------
Plane 0:
  Plane ID: 41
  Src W: 1920
  Src H: 1080
......
......
......


```

## 详细介绍

### 示例程序参数选项说明
RTSP 流解码示例支持以下命令行参数：
```
# 基本用法
python3 decode_rtsp_stream.py [-u <rtsp_url>] [-d] [-a]

# 多流示例
python3 decode_rtsp_stream.py [-u <rtsp_url1;rtsp_url2>] [-d] [-a]
```

参数说明：

-u, --rtsp_url：指定 RTSP 流地址，支持多个流地址用分号分隔

-d：启用显示功能（ 0- 禁用， 1- 启用）

-a：启用 AI 推理功能（启用时会自动启用显示功能）


### 软件架构说明
RTSP 流解码示例稍微有点复杂，需要配合 rtsp 服务器   并且代码内部采用多线程架构，所以这里以组件级的架构图来说明。

包含三个核心线程：
1. RTSP 流解码线程（ DecodeRtspStream）：

        连接 RTSP 流媒体服务器\
        自动检测流编码格式（ H.264/H.265/MJPEG）\
        使用硬件解码器解码视频流\
        管理解码帧队列

2. 视频显示线程（ VideoDisplay）：

        初始化 HDMI 显示\
        使用 VPS 进行视频处理（缩放、格式转换）\
        将处理后的视频帧发送到显示队列

3. AI 推理线程（ AiInference）：

        加载目标检测模型（ FCOS）\
        对视频帧进行推理\
        后处理并绘制检测结果

<center>
![software_arch](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/pydev_08_rtsp_sample_software_arch.png)
</center>

### API 流程说明

<center>
![API_Flow](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/pydev_08_rtsp_sample_api_flow.png)
</center>

### FAQ

Q: 运行示例时提示 "fail to open rtsp" 怎么办？\
A: 请确保 RTSP 流媒体服务器已正确启动，并且网络连接正常。可以使用 netstat -tlnp 检查服务器端口状态。

Q: 如何查看支持的编码格式？\
A: 程序会自动检测流编码格式，并在控制台输出检测结果，\
如 "Encoding detected via FourCC: h264, dec_type: 1"。

Q: 视频流延迟很高怎么办？\
A: 可以尝试降低视频流的分辨率或帧率，或者使用更轻量的目标检测模型。

Q: 如何同时处理多个 RTSP 流？\
A: 使用分号分隔多个 RTSP 地址，如 -u rtsp://url1;rtsp://url2 ，每个流会使用不同的解码通道。

Q: 如何修改目标检测模型？\
A: 在代码中修改模型加载路径，如 models = dnn.load('../models/your_model.bin')。

Q: 显示器不正常或没有输出怎么办？\
A: (1) 请检查 HDMI 连接，并确保显示服务已停止（如使用 systemctl stop lightdm）。\
(2) 检查显示器的分辨率是否和输出的视频分辨率匹配，如果不匹配，则寻找合适的源文件。


Q: 如何保存处理后的视频流？\
A: 可以在代码中添加视频保存逻辑，例如使用 OpenCV 的 VideoWriter 类保存视频文件。

Q: 如何调整检测阈值？\
A: 在代码中修改 fcos_postprocess_info.score_threshold 的值，例如改为 0.5 可以提高检测灵敏度。

Q: 支持哪些 RTSP 传输协议？\
A: 支持 TCP 和 UDP 传输协议，具体取决于 RTSP 服务器的配置。


