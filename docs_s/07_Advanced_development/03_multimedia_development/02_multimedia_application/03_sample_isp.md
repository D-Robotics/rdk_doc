# sample_isp 使用说明

## 功能概述

`sample_isp` 完成 Camera Sensor 、 MIPI CSI 、 CIM 和 ISP 模块的初始化，实现从 ISP 模块获取视频帧数据的功能，支持从 ISP 模块获取 YUV 格式的图像。

### sample_isp 架构说明

`sample_isp` 包含多个示例，每个示例均以子目录形式存在 `/app/multimedia_samples/sample_isp` 下，每个子目录描述如下

| 目录      | 描述 |
| ----------- | ----------- |
| [get_isp_data](#get_isp_data)  | 单路 sensor 获取YUV视频帧示例  |
| [isp_feedback](#isp_feedback)  | ISP回灌视频帧示例  |

## get_isp_data

#### 代码位置及目录结构

`get_isp_data` 相关源码路径为 `/app/multimedia_samples/sample_isp/get_isp_data`，代码结构如下：

```shell
── get_isp_data
   ├── get_isp_data.c
   └── Makefile
```

- Makefile：用于编译程序的 Makefile 文件。
- get_isp_data.c：程序的主要源代码文件。


### 编译

在源码路径下执行 `make` 命令即可完成编译：

```Shell
cd /app/multimedia_samples/sample_isp/get_isp_data
make
```

### 运行
#### 程序运行方法
直接执行程序 `./get_isp_data -h` 可以获得帮助信息：

#### 程序参数选项说明


执行命令 `./get_isp_data -h` 可以获得帮助信息和支持的 Camera Sensor 列表。

```shell
root@ubuntu:/app/multimedia_samples/sample_isp/get_isp_data# ./get_isp_data -h
Usage: get_isp_data [OPTIONS]
Options:
  -s <sensor_index>      Specify sensor index
  -o <online>            Specify the connection method from VIN to ISP, 1: online 0: offline
  -l <link_port>         Specify the port for connecting serdes sensors, 0:A 1:B 2:C 3:D
  -h                     Show this help message
index: 0  sensor_name: imx219-30fps             config_file:linear_1920x1080_raw10_30fps_1lane.c
index: 1  sensor_name: sc1336_gmsl-30fps        config_file:linear_1280x720_raw10_30fps_2lane.c
index: 2  sensor_name: ar0820std-30fps          config_file:linear_3840x2160_30fps_1lane.c
index: 3  sensor_name: ar0820std-1080p30        config_file:linear_1920x1080_yuv_30fps_1lane.c
```

**命令参数说明：**

- `s <sensor_index>`: 该选项用于指定要使用的传感器索引。用户需要提供一个有效的索引值。
- `o <online>`: 该选项用于指定 VIN 到 ISP 的连接方式， 1: online 0: offline, 可选参数，默认是 offline 模式。
- `l <link_port>`: 该选项用于指定 Serdes Sensor 的连接的端口 , Serdes sensor 必须指定。
- `h`: 显示帮助信息。

#### 运行效果

以 imx219 sensor 为例，执行 `./get_isp_data -s 0` 。

```shell
root@ubuntu:/app/multimedia_samples/sample_isp/get_isp_data# ./get_isp_data -s 0
Using index:0  sensor_name:imx219-30fps  config_file:linear_1920x1080_raw10_30fps_1lane.c
mipi mclk is not configed.
Searching camera sensor on device: /proc/device-tree/soc/vcon@0 i2c bus: 1 mipi rx phy: 0
mipi rx used phy: 00000000
INFO: Found sensor_name:imx219-30fps on mipi rx csi 0, i2c addr 0x10, config_file:linear_1920x1080_raw10_30fps_1lane.c

INFO: ISP channel info:
        input info: [mipi_rx: 0] [is_online: 0]
        isp channel info: [hw_id: 0] [slot_id: 4]

***************  Command Lists  ***************
 g      -- get single frame
 l      -- get a set frames
 q      -- quit
 h      -- print help message

Command: g
handle 100197 isp dump yuv 1920x1080(stride:1920), buffer size: 2073600 + 1036800 frame id: 27, timestamp: 20832399744000

Command: q
quit
```


- g： 获取一帧图像，支持输入多个 g 来连续获取图像，例如输入 gggg ​

```shell
Command: g
handle 100197 isp dump yuv 1920x1080(stride:1920), buffer size: 2073600 + 1036800 frame id: 0, timestamp: 197145649052
```

- l： 连续获取 12 帧图像，相当于输入 12 个 g ​

```shell
Command: l
handle 100197 isp dump yuv 1920x1080(stride:1920), buffer size: 2073600 + 1036800 frame id: 1, timestamp: 197178981635
handle 100197 isp dump yuv 1920x1080(stride:1920), buffer size: 2073600 + 1036800 frame id: 355, timestamp: 208978985224
... ( 省略，总共 Dump 12 帧 ) ...
handle 100197 isp dump yuv 1920x1080(stride:1920), buffer size: 2073600 + 1036800 frame id: 394, timestamp: 210278982766
handle 100197 isp dump yuv 1920x1080(stride:1920), buffer size: 2073600 + 1036800 frame id: 395, timestamp: 210312316183
```

- q: 退出程序​

```shell
Command: q
quit
```

执行程序后会获取到如 `handle_100197_isp_chn0_1920x1080_stride_1920_frameid_27_ts_20832399744000.yuv` 命名格式的 YUV 图像。

## isp_feedback

#### 代码位置及目录结构

`isp_feedback` 相关源码路径为 `/app/multimedia_samples/sample_isp/isp_feedback`，代码结构如下：

```shell
── isp_feedback
   ├── isp_feedback.c
   └── Makefile
```

- Makefile：用于编译程序的 Makefile 文件。
- isp_feedback.c：程序的主要源代码文件。

### 编译

在源码路径下执行 `make` 命令即可完成编译：

```Shell
cd /app/multimedia_samples/sample_isp/isp_feedback
make
```

### 运行
#### 程序运行方法
直接执行程序 `./isp_feedback -h` 可以获得帮助信息：

#### 程序参数选项说明

执行命令 `./isp_feedback -h` 可以获得帮助信息。

```shell
root@ubuntu:/app/multimedia_samples/sample_isp/isp_feedback# ./isp_feedback -h
Usage: isp_feedback [OPTIONS]
Options:
  -f <file>              Specify Raw filename
  -F <format>            Specify Raw format eg: raw8 raw10 raw12
  -W <width>             Specify Raw width
  -H <height>            Specify Raw height
  -l <loop>              Specify feedback Raw loop
  -h                     Show this help message
```

**命令参数说明：**

- `f <file>`: 该选项用于指定要使用 Raw 图的文件名。
- `F <format>`: 该选项用于指定要使用 Raw 图的的格式，raw8、raw10、raw12。
- `W <width>`: 该选项用于指定要使用 Raw 图的width。
- `H <height>`: 该选项用于指定要使用 Raw 图的height。
- `l <loop>`: 该选项用于指定要使用 Raw 图的回灌的次数，默认10次。
- `h`: 显示帮助信息。

#### 运行效果

- 先使用 `get_vin_data -s 0` 获取一张 imx219 的 raw 图，`get_vin_data` 的使用详细参考 [sample_vin](sample_vin.html)。
- 接下来我们可以根据提示准备好的 RAW 图指定format、width、height等参数进行回灌，ISP 回灌时会使用对应的 dummy Sensor 的 ISP 效果库进行调校。

<div class="note">
<strong>注意：</strong> <br />
使用 dummy Sensor 进行回灌，无需实际接入硬件设备。程序会将raw图进行回灌isp，并使用对应的 ISP 效果库进行调校。<br />
dummy Sensor 的配置参数路径：/app/multimedia_samples/vp_sensors/dummy_sensor/dummy_sensor.c<br />
在使用 dummy sensor 进行图像调试或 ISP 回灌时，请根据实际采集到的raw图，正确配置 sensor_param 中的 bayer_start 和 bayer_pattern 字段。配置错误可能导致图像出现反色或颜色异常。<br />
</div>

以 imx219 sensor 为例，执行 `./isp_feedback  -f handle_34661_chn0_1920x1080_stride_2400_frameid_1_ts_5752227762025.raw -F raw10 -H 1080 -W 1920` 。

```shell
root@ubuntu:/app/multimedia_samples/sample_isp/isp_feedback# ./isp_feedback  -f handle_34661_chn0_1920x1080_stride_2400_frameid_1_ts_5752227762025.raw -F raw10 -H 1080 -W 1920
Using index:5  sensor_name:dummy  config_file:dummy_sensor.c
Creating camera with config: width=1920, height=1080, format=10
[INFO] Create isp node handle: 100197
isp process one frame cost:  2187075 ns
isp(100197) dump yuv 1920x1080(stride:1920), buffer size: 2073600 + 1036800 frame id: 0, timestamp: 0
isp process one frame cost:  2128900 ns
isp(100197) dump yuv 1920x1080(stride:1920), buffer size: 2073600 + 1036800 frame id: 0, timestamp: 0
isp process one frame cost:  2115925 ns
isp(100197) dump yuv 1920x1080(stride:1920), buffer size: 2073600 + 1036800 frame id: 0, timestamp: 0
isp process one frame cost:  2112825 ns
isp(100197) dump yuv 1920x1080(stride:1920), buffer size: 2073600 + 1036800 frame id: 0, timestamp: 0
isp process one frame cost:  2112700 ns
isp(100197) dump yuv 1920x1080(stride:1920), buffer size: 2073600 + 1036800 frame id: 0, timestamp: 0
isp process one frame cost:  2115200 ns
isp(100197) dump yuv 1920x1080(stride:1920), buffer size: 2073600 + 1036800 frame id: 0, timestamp: 0
isp process one frame cost:  2116475 ns
isp(100197) dump yuv 1920x1080(stride:1920), buffer size: 2073600 + 1036800 frame id: 0, timestamp: 0
isp process one frame cost:  2112700 ns
isp(100197) dump yuv 1920x1080(stride:1920), buffer size: 2073600 + 1036800 frame id: 0, timestamp: 0
isp process one frame cost:  2112775 ns
isp(100197) dump yuv 1920x1080(stride:1920), buffer size: 2073600 + 1036800 frame id: 0, timestamp: 0
isp process one frame cost:  2114200 ns
isp(100197) dump yuv 1920x1080(stride:1920), buffer size: 2073600 + 1036800 frame id: 0, timestamp: 0
```

程序运行启动，会在当前目录保存如下调校后的的 yuv 图像，默认回灌10次，计算每次的回灌耗时：

- Using index / sensor_name / config_file：表明当前使用 sensor index 为 5，sensor 名称为 dummy，对应的 sensor 配置文件为 dummy_sensor.c。
- 每处理一帧 RAW 数据，会输出以下ISP 处理完成耗时信息，isp process one frame cost:  2170275 ns。

### isp_feedback 常见问题

- 如果指定的图像的分辨率或格式参数与实际回灌图像不匹配，可能会导致图像异常。
