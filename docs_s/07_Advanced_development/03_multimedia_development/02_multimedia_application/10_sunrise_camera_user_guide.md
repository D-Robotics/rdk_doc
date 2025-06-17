# sunrise camera 用户使用说明

## 功能说明

sunrise camera 是官方开发的应用程序，方便用户对 Camera、 VIO、 Codec、 BPU 等模块的快速评测。用户通过 PC 上的 Web 浏览器，可以方便的对程序进行参数配置，并实时预览视频流、算法渲染结果等信息。程序主要功能如下：

- 支持智能摄像头功能模式，最大支持接入 2 路 Camera Sensor
- 支持智能多路解码分析盒功能模式
  - 编解码能力：最大支持 12 路 1080p@30 视频码流的编码或解码
  - 盒子模式是先解码视频文件再编码后传输的，所以盒子模式最大支持 6 路 1080p@30
- 支持通过 PC 浏览器对程序进行参数配置，例如 Camera sensor、编码码率、算法模型、视频源等参数
- 支持通过 PC 浏览器、 VLC 拉流、 HDMI 输出等多种方式预览图像效果
- 算法多种模型切换，例如 mobilenet_v2 、 yolov5s、 fcos 等


## 硬件环境准备

- 运行 sunrise camera 前，需要准备如下配件：
  - 开发板配套摄像头，如 SC230AI、 SC132GS、 F37 等
  - 网线一条，保证 PC 和开发板能够 ping 通
  - Micro USB 线一条 ( 无网口时 )，开发板可通过 USB 虚拟网口跟 PC 通讯

整体连接方式见下图：

![hardware_connection_diagram](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/images_to_upload/hardware_connection_diagram.png)

###  网络连接要求

为保证 `4K` 编码推流时的稳定性，建议开发板与 PC 机使用千兆网口直连。

## 编译与运行

### 编译

进入目录：`/app/multimedia_samples/sunrise_camera`
执行命令: `make`
生成的目标文件：`sunrise_camera`
```sh
root@ubuntu:/app/multimedia_samples/sunrise_camera# ls sunrise_camera/bin/
log  sunrise_camera  www
```

### 运行

sunrise camera 有如下两种运行方式：
1. 手动启动：适用于调试阶段
2. 上电自启动：适用于程序调试稳定后，部署到正式场景运行

**手动启动：**

编译 sunrise camera 后执行 `sh ./start_app.sh` 启动。

**上电自启动：**
1. 部署自启动文件（只需要第一次执行）
```sh
cp sunrise_camera.service /etc/systemd/system/sunrise_camera.service
```

2. 使能开机自启动
```sh
# 重新加载 systemd 配置
sudo systemctl daemon-reload

# 后台启动
sudo systemctl start sunrise_camera

# 查看状态: 证明sunrise_camera 后台启动成功了
sudo systemctl status sunrise_camera

#设置为开机自启动
sudo systemctl enable sunrise_camera

#重启
sync
reboot

```

3. 其他命令
```sh
# 禁止开机自启动
sudo systemctl disable sunrise_camera

# 停止后台运行的 sunrise_camera
sudo systemctl stop sunrise_camera

# 后台启动后，查看日志的命令
journalctl -u sunrise_camera.service -f --output=cat
```

## Web 客户端使用说明

### 主界面

sunrise camera 正常启动后，通过 chrome 浏览器在地址栏输入 IP 地址（开发板默认 IP 为 192.168.1.10 ）可以登录用户控制主界面，  例如： http://192.168.1.10

![login_method](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/images_to_upload/login_method.png)

登录成功后的界面显示如下：

![home_page](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/images_to_upload/home_page.png)

界面菜单详细说明如下：

| 功能编号 | 说明                                                         |
| -------- | ------------------------------------------------------------ |
| 1        | 配置应用方案，点击后打开应用方案选择和参数配置页面           |
| 2        | 显示当前运行的场景信息                                       |
| 3        | 显示当前应用方案的主要信息，例如使用的 sensor 型号，编解码参数，算法模型 |
| 4        | 应用方案框图，了解当前应用方案的数据流，点击后放大查看大图   |
| 5        | 视频显示主画面，根据应用方案运行的视频通路数自动调整显示画面数量 |
| 6        | 预览视频的实时帧率                                           |
| 7        | 算法运算的实时帧率                                           |

### 参数配置方式
1. sunrise camera 支持两种应用方案：`智能摄像机` 和 `智能分析盒`( 默认应用方案为 `单路视频分析盒`)
2. sunrise camera 支持通过 Web 端在线修改应用方案、选择 camera sensor 型号、设置解码、编码参数及选择算法模型等

#### 智能摄像机配置方式
![camera_page](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/images_to_upload/camera_page.png)

具体修改步骤如下：

1. 启动 sunrise_camera 程序，打开 chrome 浏览器输入设备的 IP 地址，例如： http://192.168.1.10
2. 点击 `配置应用方案` 按钮，上图 `1` 标号
3. 可以查看到当前的设备信息，包括芯片类型，软件版本， rtsp 码流链接（该链接支持在 vlc 软件上拉 rtsp 视频码流，可以支持录像、截图等操作），上图 `2` 标号
4. 选中应用方案，上图 `3` 标号，每种应用方案的参数设置和注意事项，请点击 `问号` 按钮了解。
5. 点击 `提交` 按钮，立刻实现应用方案的切换。（注：仅点击提交不会修改开发板的配置文件，可以用来临时修改参数进行调试）
6. 点击 `保存当前配置` 按钮会把设置好的配置写入开发板上的配置文件，下次重新启动 sunrise_camera 时会按照当前配置启动。
7. 点击 `恢复默认配置`，会重新用户配置，还原为 `单路视频分析盒` 的配置。

注意：
1. Camera 接口与电路板中的 CSI 接口一一对应，只有实际接入到硬件电路并且适配过的 Camera 时才会在使能 Camera 接口中显示
2. 如果点击了 `保存当前配置` 按钮，但是再次上电时更换了新的摄像头，那么新插入的摄像头默认是非使能状态


#### 智能分析盒配置方式
![box_page](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/images_to_upload/box_page.png)

具体修改步骤如下：

1. 启动 sunrise_camera 程序，打开 chrome 浏览器输入设备的 IP 地址，例如： http://192.168.1.10
2. 点击 `配置应用方案` 按钮，上图 `1` 标号
3. 可以查看到当前的设备信息，包括芯片类型，软件版本， rtsp 码流链接（该链接支持在 vlc 软件上拉 rtsp 视频码流，可以支持录像、截图等操作），上图 `2` 标号
4. 选中应用方案，上图 `3` 标号，每种应用方案的参数设置和注意事项，请点击 `问号` 按钮了解。
5. 点击 `提交` 按钮，立刻实现应用方案的切换。（注：仅点击提交不会修改开发板的配置文件，可以用来临时修改参数进行调试）
6. 点击 `保存当前配置` 按钮会把设置好的配置写入开发板上的配置文件，下次重新启动 sunrise_camera 时会按照当前配置启动。
7. 点击 `恢复默认配置`，会重新用户配置，还原为 `单路视频分析盒` 的配置。



## 智能摄像机配置

智能摄像机方案实现了 Camera sensor 图像的采集、处理、编码、 rtsp 推流及智能计算等功能，可以帮助用户快速体验多媒体图像和算法效果，方案功能框图如下：

![camera_solution](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/images_to_upload/camera_solution.png)

智能摄像机方案提供了以下功能：

- 实时视频监控
- 运行指定的算法
- 当有多颗 Camera Sensor 接入时，支持开启多路视频预览

### 参数说明

智能摄像机方案具有以下可调参数：

- 使能 Camera 接口：显示已经实际接入了摄像头的 CSI 接口的列表，并且可以根据需要使能不同的接口
- Sensor 型号：程序会探测接入到设备上的可以使用的 Camera Sensor 型号，根据需要进行配置
- 编码类型：控制视频编码格式，支持 H264\H265\Mjpeg, 根据硬件能力、软件支持情况会有所不同
- 编码码率：控制视频编码的码率，以下是不同分辨率视频的参考码率
  - 标清视频（ 480p : 256, 512, 768, 1024, 1536, 2048
  - 高清视频（ 720p ）: 512, 1024, 2048, 3072, 4096, 6144
  - 全高清视频（ 1080p ）: 1024, 2048, 4096, 6144, 8192, 12288
  - 2K 视频 : 2048, 4096, 8192, 12288, 16384, 24576
  - 4K 视频 : 4096, 8192, 16384, 24576, 32768, 49152
- 算法模型：选择运行算法模型

### 注意事项

在使用智能摄像机方案时，请注意以下事项：

- 确保 Camera Sensor 与开发板连接稳定
- 根据需要调整编码码率以平衡视频质量和网络带宽

## 智能分析盒配置

智能盒子方案实现了单路、四路 1080p 视频解码、拼接、编码、 rtsp 推流及智能计算等功能，用户可通过 Web 端、 HDMI 或者拉流方式预览效果。方案功能框图如下：

![box_solution](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/images_to_upload/box_solution.png)

智能分析盒方案提供了以下功能：

- 解码码流后运行算法并重新编码推流
  - 支持读取开发板上的本地视频码流文件
  - 支持使用 RTSP 码流
  - 支持对解码的视频文件进行缩放后再编码
  - 支持控制码率和帧率
- 运行指定的算法

### 参数说明

智能分析盒方案具有以下可调参数：

- 视频通道数：选择启用多少路视频分析通道
  - 启用的通道数的能力解码和编码合计不能超过 4K@90fps （硬件限制）
  - 设备默认分配给编解码、算法的 ION 内存为 1408MB ，如果启用路数过多，有可能会出现内存不足的问题需要调整 ION 内存大小
- 视频数据流（ stream ）： 支持读取开发板上的本地视频码流文件和 RTSP 码流
- 解码类型（ decode_type ）：控制视频解码格式，支持 H264\H265\Mjpeg
- 解码宽度（ decode_width ）：控制视频解码的宽度
- 解码高度（ decode_height ）：控制视频解码的高度
- 解码帧率（ decode_frame_rate ）：控制视频解码的帧率
- 编码类型（ encode_type ）：控制视频编码格式，支持 H264\H265\Mjpeg
- 编码宽度（ encode_width ）：控制视频编码的宽度
- 编码高度（ encode_height ）：控制视频编码的高度
- 编码帧率（ encode_frame_rate ）：控制视频编码的帧率
- 编码码率（ encode_bitrate ）：控制视频编码的码率，以下是不同分辨率视频的参考码率
  - 标清视频（ 480p ）： 256, 512, 768, 1024, 1536, 2048
  - 高清视频（ 720p ）： 512, 1024, 2048, 3072, 4096, 6144
  - 全高清视频（ 1080p ）： 1024, 2048, 4096, 6144, 8192, 12288
  - 2K 视频 : 2048, 4096, 8192, 12288, 16384, 24576
  - 4K 视频 : 4096, 8192, 16384, 24576, 32768, 49152
- 算法模型（ model ）：选择运行算法模型

### 注意事项

在使用智能分析盒方案时，请注意以下事项：

- 确保网络连接稳定，网络带宽满足数据传输量的要求
- 根据需要调整分析通道数和分析算法以满足应用需求
- 启用的通道数的能力解码和编码合计不能超过 4K@90fps （硬件限制）
- 设备默认分配给编解码、算法的 ION 内存为 1GB，如果启用路数过多，有可能会出现内存不足的问题，需要调大 ION

## 使用 VLC 播放器播放 RTSP 码流

sunrise camera 程序运行时会同步推送 rtsp 视频码流，用户可以通过 vlc 播放器播放 RTSP 码流，实现画面预览、录像、截图等功能。



### 播放方法

打开 vlc 播放器，选择 `媒体` 菜单，然后选择 `打开网络串流` 选项。

在 `打开媒体` 对话框中填入 url 地址，点击 `播放` 按钮即可开始播放，如下图：

![vlc_play_method](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/images_to_upload/vlc_play_method.png)

rtsp 的码流网络 URL 链接，可以在 web 的设备信息上查看，默认支持的码流链接为： rtsp://192.168.1.10/stream_chn0.h264

### 4K@30fps 拉流配置说明

1 、 4K@30fps 并且配置高码率（ 8192Kbps 以上）的情况下，为了保证视频播放流畅不丢帧，建议使用千兆网络，否则很容易出现视频马赛克和花屏等情况。

2 、 vlc buffer_size 设置， vlc 接收 buffer 修改：默认是 250000 ，建议改到 1200000 。

![vlc_rtsp_buffer_size](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/images_to_upload/vlc_rtsp_buffer_size.png)

3 、使用 http 模式，可以有效解决播放丢帧导致的花屏。

![vlc_http_mode](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/images_to_upload/vlc_http_mode.png)

4 、高级设置里面禁用时钟同步

![vlc_disable_clock_sync](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/images_to_upload/vlc_disable_clock_sync.png)
