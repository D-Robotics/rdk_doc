---
sidebar_position: 2
---

# 3.2.2 decode2display 示例介绍

## 示例简介
decode2display 是一个位于 `/app/cdev_demo` 目录中的 **C 语言接口** 开发代码示例，用于演示如何使用 c 语言进行解码并显示到屏幕的程序。参考这个示例，用户可以理解并开发相关应用。

## 效果展示
下图展示的是程序执行期间， RDK 连接 HDMI 屏幕现象以及通过 ssh 连接到 RDK 的网络终端输出，\
其中 HDMI 屏幕会循环播放解码的文件 ， VScode 远程 RDK 的网络终端一直不断打印程序的执行状态。

![output-img](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/02_cdev_demo_sample/image/cdev_decode2dsiplay_output.png)

## 硬件准备
准备一个 RDK 开发板，通过 HDMI 登录桌面。

### 硬件连接
该示例不需要鼠标和键盘，所以这里连接了 HDMI 显示屏，网线接口，电源线

![connect-img](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/02_cdev_demo_sample/image/hardware-connect.png)

## 快速开始

### 代码以及板端位置
进入到 `/app/cdev_demo/decode2display 位置，可以看到 decode2display 示例里面包含了 2 个文件
```
root@ubuntu:/app/cdev_demo/decode2display# tree
.
├── decoder2display.c
└── Makefile

```

### 编译以及运行
我们直接在该目录下使用 make 命令即可编译出 decoder2display 可执行文件。
```
root@ubuntu:/app/cdev_demo/decode2display# tree
.
├── decoder2display
├── decoder2display.c
├── decoder2display.o
└── Makefile

```

### 执行效果
- **首先** 我先要准备好输入数据，这里我们可以将板端已有的数据，比如 `/opt/tros/humble/lib/hobot_codec/config/1920x1080.h264` 目录下的 `1920x1080.h264` 文件拷贝到当前目录，单独操作，不影响原始数据。
- **其次** 我们使用 `systemctl stop lightdm` 来关闭显示服务。
- **最后** 我们使用 `./decoder2display` 命令，默认的执行结果是将 `1920x1080.h264` 的文件，解码之后给到显示器进行显示。

```
root@ubuntu:/app/cdev_demo/decode2display# sudo ./decoder2display -w 1920 -h 1080 -i 1920x1080.h264
Opened DRM device: /dev/dri/card0
1920x1080
1280x800
1280x720
720x576
720x480
640x480
disp_w=1920, disp_h=1080
sp_start_decode success!
Opened DRM device: /dev/dri/card0
DRM is available, using libdrm for rendering.
------------------------------------------------------
Plane 0:
  Plane ID: 41
  Src W: 1920
  Src H: 1080
  CRTC X: 0
  CRTC Y: 0
  CRTC W: 1920
  CRTC H: 1080
  Format: NV12
  Z Pos: 0
  Alpha: 65535
  Pixel Blend Mode: 1
  Rotation: -1
  Color Encoding: -1
  Color Range: -1
------------------------------------------------------
Setting DRM client capabilities...
Setting up KMS...
CRTC ID: 31
CRTC ID: 63
Number of connectors: 1
Connector ID: 74
    Type: 11
    Type Name: HDMI-A
    Connection: Connected
    Modes: 30
    Subpixel: 1
    Mode 0: 1280x800 @ 59Hz
    Mode 1: 1920x1080 @ 60Hz
    Mode 2: 1920x1080 @ 60Hz
    Mode 3: 1920x1080 @ 60Hz
    Mode 4: 1920x1080i @ 60Hz
    Mode 5: 1920x1080i @ 60Hz
    Mode 6: 1920x1080i @ 60Hz
    Mode 7: 1920x1080 @ 50Hz
    Mode 8: 1920x1080i @ 50Hz
    Mode 9: 1920x1080 @ 24Hz
    Mode 10: 1920x1080 @ 24Hz
    Mode 11: 1280x720 @ 60Hz
    Mode 12: 1280x720 @ 60Hz
    Mode 13: 1280x720 @ 60Hz
    Mode 14: 1280x720 @ 50Hz
    Mode 15: 720x576 @ 50Hz
    Mode 16: 720x576 @ 50Hz
    Mode 17: 720x576i @ 50Hz
    Mode 18: 720x576i @ 50Hz
    Mode 19: 720x480 @ 60Hz
    Mode 20: 720x480 @ 60Hz
    Mode 21: 720x480 @ 60Hz
    Mode 22: 720x480 @ 60Hz
    Mode 23: 720x480 @ 60Hz
    Mode 24: 720x480i @ 60Hz
    Mode 25: 720x480i @ 60Hz
    Mode 26: 720x480i @ 60Hz
    Mode 27: 720x480i @ 60Hz
    Mode 28: 640x480 @ 60Hz
    Mode 29: 640x480 @ 60Hz
sp_start_display success!
2000/01/01 09:44:29.126 !INFO [SetImageFrame][0495]N2D init done!

Created new framebuffer: fb_id=77 for dma_buf_fd=56
add mapping dma_buf_fd:56 fb_id:77, mapping_count: 1
Created new framebuffer: fb_id=79 for dma_buf_fd=57
add mapping dma_buf_fd:57 fb_id:79, mapping_count: 2
2000/01/01 09:44:46.374 !INFO [vp_decode_work_func][0974]No more input data available, avpacket.size: 0. Re-cycling to send again.
2000/01/01 09:45:03.869 !INFO [vp_decode_work_func][0974]No more input data available, avpacket.size: 0. Re-cycling to send again.
2000/01/01 09:45:21.331 !INFO [vp_decode_work_func][0974]No more input data available, avpacket.size: 0. Re-cycling to send again.
2000/01/01 09:45:38.826 !INFO [vp_decode_work_func][0974]No more input data available, avpacket.size: 0. Re-cycling to send again.
2000/01/01 09:45:56.299 !INFO [vp_decode_work_func][0974]No more input data available, avpacket.size: 0. Re-cycling to send again.
2000/01/01 09:46:13.737 !INFO [vp_decode_work_func][0974]No more input data available, avpacket.size: 0. Re-cycling to send again.
2000/01/01 09:46:31.219 !INFO [vp_decode_work_func][0974]No more input data available, avpacket.size: 0. Re-cycling to send again.
2000/01/01 09:46:48.713 !INFO [vp_decode_work_func][0974]No more input data available, avpacket.size: 0. Re-cycling to send again.
2000/01/01 09:47:06.197 !INFO [vp_decode_work_func][0974]No more input data available, avpacket.size: 0. Re-cycling to send again.
2000/01/01 09:47:23.669 !INFO [vp_decode_work_func][0974]No more input data available, avpacket.size: 0. Re-cycling to send again.
2000/01/01 09:47:41.161 !INFO [vp_decode_work_func][0974]No more input data available, avpacket.size: 0. Re-cycling to send again.
2000/01/01 09:47:58.649 !INFO [vp_decode_work_func][0974]No more input data available, avpacket.size: 0. Re-cycling to send again.
2000/01/01 09:48:16.111 !INFO [vp_decode_work_func][0974]No more input data available, avpacket.size: 0. Re-cycling to send again.
2000/01/01 09:48:33.572 !INFO [vp_decode_work_func][0974]No more input data available, avpacket.size: 0. Re-cycling to send again.
```

在 HDMI 屏幕上显示的内容如下

![HDMI_IMG](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/02_cdev_demo_sample/image/cdev_decode2display_vlc.png)


## 详细介绍

### 示例程序参数选项说明
我们可以直接执行目标文件来确认参数选项说明
```
root@ubuntu:/app/cdev_demo/decode2display# sudo ./decoder2display 
Usage: decoder2display [OPTION...]
decode2display sample -- An example of streaming video decoding to the display

  -h, --height=height        height of input video
  -i, --input=path           input video path
  -w, --width=width          width of input video
  -?, --help                 Give this help list
      --usage                Give a short usage message
```
其中 \
-h  是必填选项，代表输入视频的像素高度 \
-w 是必填选项，代表输入视频的像素宽度 \
-i  是必填选项，代表输入视频的文件路径

### 软件架构说明
本 Sample 是基于 spcdev 接口实现的， 解析输入给 main 的参数，通过 libspcdev.so API，获取到显示屏的分辨率之后，初始化解码器和显示模块，根据适配的分辨率和显示器的分辨率，判断是否需要使用 VPS 进行

<center>
![software_arch](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/02_cdev_demo_sample/image/cdev_decode2display_software_arch.png)
</center>

### API 流程说明
<center>
![API_Flow](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/02_cdev_demo_sample/image/cdev_decode2display_api_flow.png)
</center>

### FAQ
__Q：__ API 有更详细的说明吗？\
__A：__ 可以到 [解码模块 API](../06_multi_media_sp_dev_api/multi_media_api/cdev_multimedia_api_x3/decoder_api.md) 位置进行查询。


__Q：__ 为什么要使用 systemctl stop lightdm\
__A：__ 我们也可以用窗口来显示，但资源占用过大，会导致不流畅或者卡顿，直接显示到显示器效果更直观。

