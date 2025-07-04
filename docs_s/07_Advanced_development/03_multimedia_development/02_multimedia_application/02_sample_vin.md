# sample_vin 使用说明

## 功能概述
`sample_vin`完成 Camera Sensor 、MIPI CSI 和 SIF 模块的初始化，实现从vin模块获取视频帧数据的功能，支持从 VIN 模块获取 Raw 或者 YUV 格式的图像。

### sample_vin 架构说明

`sample_vin` 包含多个示例，每个示例均以子目录形式存在 `/app/multimedia_samples/sample_vin` 下，每个子目录描述如下

| 目录      | 描述 |
| ----------- | ----------- |
| [get_vin_data](#get_vin_data)  | 单路 sensor 获取视频帧示例  |
| [get_multi_vin_data](#get_multi_vin_data)  | 多路 sensor 获取视频帧示例  |

## get_vin_data

### 编译

在源码路径下执行 `make` 命令即可完成编译：

```Shell
cd /app/multimedia_samples/sample_vin/get_vin_data
make
```

### 运行
#### 程序运行方法
直接执行程序 `./get_vin_data -h` 可以获得帮助信息：

#### 程序参数选项说明

执行命令 `./get_vin_data -h` 可以获得帮助信息和支持的 Camera Sensor 列表。

```shell
root@ubuntu:/app/multimedia_samples/sample_vin/get_vin_data# ./get_vin_data -h
Usage: get_vin_data [OPTIONS]
Options:
	-s <sensor_index>      Specify sensor index
	-l <link_port>         Specify the port for connecting serdes sensors, 0:A 1:B 2:C 3:D
	-h                     Show this help message
index: 0  sensor_name: imx219-30fps             config_file:linear_1920x1080_raw10_30fps_1lane.c
index: 1  sensor_name: sc1336_gmsl-30fps        config_file:linear_1280x720_raw10_30fps_2lane.c
index: 2  sensor_name: ar0820std-30fps          config_file:linear_3840x2160_30fps_1lane.c
index: 3  sensor_name: ar0820std-1080p30        config_file:linear_1920x1080_yuv_30fps_1lane.c
```

**命令参数说明：**

- `s <sensor_index>`: 该选项用于指定要使用的传感器索引。用户需要提供一个有效的索引值。
- `l <link_port>`: 该选项用于指定 Serdes Sensor 的连接的端口 , Serdes sensor 必须指定。
- `h`: 显示帮助信息。

#### 运行效果

以 imx219 sensor 为例，执行 `./get_vin_data -s 0` 。

```shell
root@ubuntu:/app/multimedia_samples/sample_vin/get_vin_data# ./get_vin_data -s 0
Using index:0  sensor_name:imx219-30fps  config_file:linear_1920x1080_raw10_30fps_1lane.c
mipi mclk is not configed.
Searching camera sensor on device: /proc/device-tree/soc/vcon@0 i2c bus: 1 mipi rx phy: 0
mipi rx used phy: 00000000
mipi mclk is not configed.
Searching camera sensor on device: /proc/device-tree/soc/vcon@1 i2c bus: 2 mipi rx phy: 1
mipi rx used phy: 00000000
INFO: Found sensor_name:imx219-30fps on mipi rx csi 1, i2c addr 0x10, config_file:linear_1920x1080_raw10_30fps_1lane.c

***************  Command Lists  ***************
 g      -- get single frame
 l      -- get a set frames
 q      -- quit
 h      -- print help message

Command: g
Dumping RAW data: handle 34661, resolution: 1920x1080 (stride: 2400), size: 2592000, frame id: 0, timestamp: 0
Dump image to file(handle_34661_chn0_1920x1080_stride_2400_frameid_0_ts_0.raw), size(2592000) succeeded
```

- g： 获取一帧图像，支持输入多个 g 来连续获取图像，例如输入 gggg ​

```shell
Command: g
Dumping RAW data: handle 34661, resolution: 1920x1080 (stride: 2400), size: 2592000, frame id: 138, timestamp: 1012736827400
Dump image to file(handle_34661_chn0_1920x1080_stride_2400_frameid_138_ts_1012736827400.raw), size(2592000) succeeded
```

- l： 连续获取 12 帧图像，相当于输入 12 个 g ​

```shell
Command: l
Dumping RAW data: handle 34661, resolution: 1920x1080 (stride: 2400), size: 2592000, frame id: 138, timestamp: 1012736827400
Dump image to file(handle_34661_chn0_1920x1080_stride_2400_frameid_138_ts_1012736827400.raw), size(2592000) succeeded
Dumping RAW data: handle 34661, resolution: 1920x1080 (stride: 2400), size: 2592000, frame id: 139, timestamp: 1012769731350
Dump image to file(handle_34661_chn0_1920x1080_stride_2400_frameid_139_ts_1012769731350.raw), size(2592000) succeeded
... ( 省略，总共 Dump 12 帧 ) ...
Dumping RAW data: handle 34661, resolution: 1920x1080 (stride: 2400), size: 2592000, frame id: 995, timestamp: 1040941500225
Dump image to file(handle_34661_chn0_1920x1080_stride_2400_frameid_995_ts_1040941500225.raw), size(2592000) succeeded
```

- q: 退出程序​

```shell
Command: q
quit
```

执行程序后会获取到如 `handle_34661_chn0_1920x1080_stride_2400_frameid_995_ts_1040941500225.raw` 命名格式的 raw 图像。

## get_multi_vin_data

### 编译

在源码路径下执行 `make` 命令即可完成编译：

```Shell
cd /app/multimedia_samples/sample_vin/get_multi_vin_data
make
```

### 运行
#### 程序运行方法
直接执行程序 `./get_multi_vin_data -h` 可以获得帮助信息：

#### 程序参数选项说明

执行命令 `./get_multi_vin_data -h` 可以获得帮助信息和支持的 Camera Sensor 列表。

```shell
root@ubuntu:/app/multimedia_samples/sample_vin/get_multi_vin_data# ./get_multi_vin_data -h
Usage: get_multi_vin_data [Options]
Options:
-c, --config="sensor=id"
	Configure parameters for each video pipeline, can be repeated up to 6 times.
	sensor   --  Sensor index,can have multiple parameters, reference sensor list.
	mode     --  Sensor mode of camera_config_t
	link     --  Sensor link port number, serdes sensor must be configured according to the hardware connection, can be set to [0-3] 0:A 1:B 2:C 3:D.
-h, --help      Show help message
Support sensor list:
index: 0  sensor_name: imx219-30fps             config_file:linear_1920x1080_raw10_30fps_1lane.c
index: 1  sensor_name: sc1336_gmsl-30fps        config_file:linear_1280x720_raw10_30fps_2lane.c
index: 2  sensor_name: ar0820std-30fps          config_file:linear_3840x2160_30fps_1lane.c
index: 3  sensor_name: ar0820std-1080p30        config_file:linear_1920x1080_yuv_30fps_1lane.c
index: 4  sensor_name: ovx3cstd-30fps           config_file:linear_1920x1280_yuv_30fps_1lane.c
```

**命令参数说明：**

- `c <sensor>`: 该选项用于指定要使用的传感器索引。用户需要提供一个有效的索引值。
- `l <link_port>`: 该选项用于指定 Serdes Sensor 的连接的端口 , Serdes sensor 必须指定。
- `h`: 显示帮助信息。

#### 运行效果

以 imx219 sensor 和 ar0820std 4K sensor为例，执行 `./get_multi_vin_data -c "sensor=0" -c "sensor=2 link=1"` 。

:::caution 注意
link 设定的值是根据 serdes sensor 连接到解串器上的端口而定的，请确保 serdes sensor 接到设定的端口。
:::

```shell
Using index:0  sensor_name:imx219-30fps  config_file:linear_1920x1080_raw10_30fps_1lane.c
mipi mclk is not configed.
Searching camera sensor on device: /proc/device-tree/soc/vcon@0 i2c bus: 1 mipi rx phy: 0
mipi rx used phy: 00000000
mipi mclk is not configed.
Searching camera sensor on device: /proc/device-tree/soc/vcon@1 i2c bus: 2 mipi rx phy: 1
mipi rx used phy: 00000000
INFO: Found sensor_name:imx219-30fps on mipi rx csi 1, i2c addr 0x10, config_file:linear_1920x1080_raw10_30fps_1lane.c
Using index:2  sensor_name:ar0820std-30fps  config_file:linear_3840x2160_30fps_1lane.c
Pipeline index 0:
	Sensor index: 0
	Sensor name: imx219-30fps
	Active mipi host: 1
Pipeline index 1:
	Sensor index: 0
	Sensor name: ar0820std-30fps
	Active mipi host: 4
Verbose: 1
vc_index:1
All deserial link info:
	[link_port:0] sc1336_gmsl:0@256
	[link_port:1] ar0820std:5@512
	[link_port:2] sc1336_gmsl:0@256
	[link_port:3] sc1336_gmsl:0@256
deserial_config:29_max96712, des_handle:148274
Dumping RAW data: handle 34661, resolution: 1920x1080 (stride: 2400), size: 2592000, frame id: 1, timestamp: 1317321489925
Dump image to file(handle_34661_chn-1_1920x1080_stride_2400_frameid_1_ts_1317321489925.raw), size(2592000) succeeded
Dumping YUV data: handle 100197, resolution: 3840x2160 (stride: 3840), size: 8294400 + 4147200, frame id: 1, timestamp: 1317379256975
Dump successful: handle_100197_chn1_3840x2160_stride_3840_frameid_1_ts_1317379256975.yuv (size: 256)
```

执行程序后会获取到 imx219 `handle_34661_chn-1_1920x1080_stride_2400_frameid_1_ts_1317321489925.raw` 命名格式的 RAW 图像 和 ar0820std 对应的`handle_100197_chn1_3840x2160_stride_3840_frameid_1_ts_1317379256975.yuv` 命名格式的 YUV 图像。
