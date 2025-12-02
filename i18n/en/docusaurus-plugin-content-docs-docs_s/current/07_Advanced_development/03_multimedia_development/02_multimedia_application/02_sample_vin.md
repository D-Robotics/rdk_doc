# sample_vin Usage Instructions

## Function Overview
`sample_vin` initializes the Camera Sensor, MIPI CSI, and SIF modules, enabling the acquisition of video frame data from the VIN module. It supports retrieving images in either Raw or YUV format from the VIN module.

### sample_vin Architecture Description

`sample_vin` contains multiple examples, each located as a subdirectory under `/app/multimedia_samples/sample_vin`. The subdirectories are described as follows:

| Directory      | Description |
| ----------- | ----------- |
| [get_vin_data](#get_vin_data)  | Example for acquiring video frames from a single sensor  |
| [get_multi_vin_data](#get_multi_vin_data)  | Example for acquiring video frames from multiple sensors  |

## get_vin_data

### Compilation

Run the `make` command in the source code directory to compile:

```Shell
cd /app/multimedia_samples/sample_vin/get_vin_data
make
```

### Execution
#### How to Run the Program
Execute the program directly with `./get_vin_data -h` to display help information:

#### Program Parameter Options Explanation

Running `./get_vin_data -h` displays help information and the list of supported Camera Sensors.

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

**Command Parameter Descriptions:**

- `s <sensor_index>`: This option specifies the sensor index to be used. The user must provide a valid index value.
- `l <link_port>`: This option specifies the port used to connect Serdes Sensors. It is mandatory for Serdes sensors.
- `h`: Displays help information.

#### Execution Results

Taking the imx219 sensor as an example, execute `./get_vin_data -s 0`.

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

- **g**: Acquire a single frame. Multiple 'g' inputs can be used to continuously capture images (e.g., inputting "gggg").

```shell
Command: g
Dumping RAW data: handle 34661, resolution: 1920x1080 (stride: 2400), size: 2592000, frame id: 138, timestamp: 1012736827400
Dump image to file(handle_34661_chn0_1920x1080_stride_2400_frameid_138_ts_1012736827400.raw), size(2592000) succeeded
```

- **l**: Continuously acquire 12 frames, equivalent to entering 'g' twelve times.

```shell
Command: l
Dumping RAW data: handle 34661, resolution: 1920x1080 (stride: 2400), size: 2592000, frame id: 138, timestamp: 1012736827400
Dump image to file(handle_34661_chn0_1920x1080_stride_2400_frameid_138_ts_1012736827400.raw), size(2592000) succeeded
Dumping RAW data: handle 34661, resolution: 1920x1080 (stride: 2400), size: 2592000, frame id: 139, timestamp: 1012769731350
Dump image to file(handle_34661_chn0_1920x1080_stride_2400_frameid_139_ts_1012769731350.raw), size(2592000) succeeded
... (omitted, total of 12 frames dumped) ...
Dumping RAW data: handle 34661, resolution: 1920x1080 (stride: 2400), size: 2592000, frame id: 995, timestamp: 1040941500225
Dump image to file(handle_34661_chn0_1920x1080_stride_2400_frameid_995_ts_1040941500225.raw), size(2592000) succeeded
```

- **q**: Quit the program.

```shell
Command: q
quit
```

After running the program, RAW images will be saved with filenames like `handle_34661_chn0_1920x1080_stride_2400_frameid_995_ts_1040941500225.raw`.

## get_multi_vin_data

### Compilation

Run the `make` command in the source code directory to compile:

```Shell
cd /app/multimedia_samples/sample_vin/get_multi_vin_data
make
```

### Execution
#### How to Run the Program
Execute the program directly with `./get_multi_vin_data -h` to display help information:

#### Program Parameter Options Explanation

Running `./get_multi_vin_data -h` displays help information and the list of supported Camera Sensors.

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

**Command Parameter Descriptions:**

- `c <sensor>`: This option specifies the sensor index to be used. The user must provide a valid index value.
- `l <link_port>`: This option specifies the port used to connect Serdes Sensors. It is mandatory for Serdes sensors.
- `h`: Displays help information.

#### Execution Results

Taking the imx219 sensor and the ar0820std 4K sensor as examples, execute `./get_multi_vin_data -c "sensor=0" -c "sensor=2 link=1"`.

:::caution Note
The value set for `link` depends on the port to which the Serdes sensor is connected on the deserializer. Ensure the Serdes sensor is connected to the specified port.
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

After running the program, you will obtain a RAW image from the imx219 sensor named like `handle_34661_chn-1_1920x1080_stride_2400_frameid_1_ts_1317321489925.raw` and a YUV image from the ar0820std sensor named like `handle_100197_chn1_3840x2160_stride_3840_frameid_1_ts_1317379256975.yuv`.