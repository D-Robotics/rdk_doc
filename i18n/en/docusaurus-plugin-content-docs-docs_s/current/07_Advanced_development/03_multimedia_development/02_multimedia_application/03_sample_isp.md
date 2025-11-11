# sample_isp Usage Guide

## Function Overview

`sample_isp` initializes the Camera Sensor, MIPI CSI, CIM, and ISP modules, enabling the retrieval of video frame data from the ISP module. It supports obtaining YUV-formatted images from the ISP module.

### sample_isp Architecture Description

`sample_isp` contains multiple examples, each located as a subdirectory under `/app/multimedia_samples/sample_isp`. Each subdirectory is described below:

| Directory      | Description |
| ----------- | ----------- |
| [get_isp_data](#get_isp_data)  | Example for single-sensor YUV video frame acquisition |
| [isp_feedback](#isp_feedback)  | Example for ISP video frame feedback |

## get_isp_data

#### Code Location and Directory Structure

The source code for `get_isp_data` is located at `/app/multimedia_samples/sample_isp/get_isp_data`, with the following structure:

```shell
── get_isp_data
   ├── get_isp_data.c
   └── Makefile
```

- Makefile: Makefile used to compile the program.
- get_isp_data.c: Main source code file of the program.

### Compilation

Run the `make` command in the source directory to compile:

```Shell
cd /app/multimedia_samples/sample_isp/get_isp_data
make
```

### Execution
#### How to Run the Program
Execute the program directly with `./get_isp_data -h` to display help information:

#### Program Parameter Options

Running `./get_isp_data -h` displays help information and a list of supported Camera Sensors.

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

**Command Parameter Descriptions:**

- `s <sensor_index>`: Specifies the sensor index to use. The user must provide a valid index value.
- `o <online>`: Specifies the connection method from VIN to ISP: 1 for online, 0 for offline. Optional; defaults to offline mode.
- `l <link_port>`: Specifies the port used to connect SerDes sensors. Required for SerDes sensors.
- `h`: Displays help information.

#### Execution Output

Taking the imx219 sensor as an example, run `./get_isp_data -s 0`.

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

- **g**: Acquires a single frame. Multiple 'g' inputs can be used to continuously acquire frames (e.g., input "gggg").

```shell
Command: g
handle 100197 isp dump yuv 1920x1080(stride:1920), buffer size: 2073600 + 1036800 frame id: 0, timestamp: 197145649052
```

- **l**: Continuously acquires 12 frames (equivalent to entering 12 'g' commands).

```shell
Command: l
handle 100197 isp dump yuv 1920x1080(stride:1920), buffer size: 2073600 + 1036800 frame id: 1, timestamp: 197178981635
handle 100197 isp dump yuv 1920x1080(stride:1920), buffer size: 2073600 + 1036800 frame id: 355, timestamp: 208978985224
... (omitted; total of 12 frames dumped) ...
handle 100197 isp dump yuv 1920x1080(stride:1920), buffer size: 2073600 + 1036800 frame id: 394, timestamp: 210278982766
handle 100197 isp dump yuv 1920x1080(stride:1920), buffer size: 2073600 + 1036800 frame id: 395, timestamp: 210312316183
```

- **q**: Quits the program.

```shell
Command: q
quit
```

After execution, YUV images will be saved with filenames following the format:  
`handle_100197_isp_chn0_1920x1080_stride_1920_frameid_27_ts_20832399744000.yuv`.

## isp_feedback

#### Code Location and Directory Structure

The source code for `isp_feedback` is located at `/app/multimedia_samples/sample_isp/isp_feedback`, with the following structure:

```shell
── isp_feedback
   ├── isp_feedback.c
   └── Makefile
```

- Makefile: Makefile used to compile the program.
- isp_feedback.c: Main source code file of the program.

### Compilation

Run the `make` command in the source directory to compile:

```Shell
cd /app/multimedia_samples/sample_isp/isp_feedback
make
```

### Execution
#### How to Run the Program
Execute the program directly with `./isp_feedback -h` to display help information:

#### Program Parameter Options

Running `./isp_feedback -h` displays help information.

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

**Command Parameter Descriptions:**

- `f <file>`: Specifies the filename of the RAW image to use.
- `F <format>`: Specifies the format of the RAW image (e.g., raw8, raw10, raw12).
- `W <width>`: Specifies the width of the RAW image.
- `H <height>`: Specifies the height of the RAW image.
- `l <loop>`: Specifies the number of times to loop the RAW feedback. Default is 10.
- `h`: Displays help information.

#### Execution Output

- First, use `get_vin_data -s 0` to capture a RAW image from the imx219 sensor. For detailed usage of `get_vin_data`, refer to [sample_vin](sample_vin.html).
- Next, perform ISP feedback by specifying parameters such as format, width, and height for the prepared RAW image. During ISP feedback, the corresponding dummy sensor's ISP tuning library will be applied.

<div class="note">
<strong>Note:</strong> <br />
When using a dummy sensor for feedback, no physical hardware connection is required. The program feeds the RAW image into the ISP and applies the corresponding ISP tuning library.<br />
Path to dummy sensor configuration parameters: `/app/multimedia_samples/vp_sensors/dummy_sensor/dummy_sensor.c`<br />
When debugging images or performing ISP feedback with a dummy sensor, correctly configure the `bayer_start` and `bayer_pattern` fields in `sensor_param` according to the actual captured RAW image. Incorrect configuration may cause color inversion or abnormal colors.<br />
</div>

Taking the imx219 sensor as an example, execute:  
`./isp_feedback -f handle_34661_chn0_1920x1080_stride_2400_frameid_1_ts_5752227762025.raw -F raw10 -H 1080 -W 1920`.

```shell
root@ubuntu:/app/multimedia_samples/sample_isp/isp_feedback# ./isp_feedback  -f handle_34661_chn0_1920x1080_stride_2400_frameid_1_ts_5752227762025.raw -F raw10 -H 1080 -W 1920
Using index:5  sensor_name:dummy  config_file:dummy_sensor.c
Creating camera with config: width=1920, height=1080, format=10
[INFO] Create isp node handle: 100197
isp process one frame cost:  2187075 ns
```isp(100197) dump yuv 1920x1080(stride:1920), buffer size: 2073600 + 1036800 frame id: 0, timestamp: 0  
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

The program starts running and saves the following calibrated YUV images in the current directory by default, looping back 10 times and calculating the processing time for each loop:

- Using index / sensor_name / config_file: Indicates that the current sensor index is 5, the sensor name is "dummy", and the corresponding sensor configuration file is dummy_sensor.c.  
- For each RAW frame processed, the following ISP processing time information is output: "isp process one frame cost: 2170275 ns."

### Common Issues with isp_feedback

- If the specified image resolution or format parameters do not match the actual loopback image, it may cause image anomalies.