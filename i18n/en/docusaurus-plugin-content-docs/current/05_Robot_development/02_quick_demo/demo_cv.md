---
sidebar_position: 4
---
# 5.2.4 Image Processing Acceleration

```mdx-code-block
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';
```

## Gaussian Blur

### Feature Introduction

Implements Gaussian blur functionality, with acceleration types including BPU acceleration and NEON acceleration. Currently, BPU acceleration only supports int16 format, while NEON acceleration supports both int16 and uint16 formats.

Code repository: [https://github.com/D-Robotics/hobot_cv](https://github.com/D-Robotics/hobot_cv)

### Supported Platforms

| Platform                | Runtime Environment                     | Example Functionality                    |
| ----------------------- | --------------------------------------- | ---------------------------------------- |
| RDK X3, RDK X3 Module   | Ubuntu 20.04 (Foxy), Ubuntu 22.04 (Humble) | Read ToF images and apply Gaussian blur |

### Prerequisites

#### RDK Platform

1. The RDK has been flashed with Ubuntu 20.04 or Ubuntu 22.04 system image.

2. TogetheROS.Bot has been successfully installed on the RDK.

### Usage Instructions

#### BPU Acceleration

The currently supported parameter ranges are as follows:

- Filter type: Gaussian blur  
- Supported data types: int16  
- Supported resolution: 320x240  
- Filter kernel: Gaussian 3x3  
- sigmaX: 0  
- sigmaY: 0  

#### NEON Acceleration

The currently supported parameter ranges are as follows:

- Filter type: Gaussian blur  
- Supported data types: int16, uint16  
- Filter kernel: Gaussian 3x3, 5x5  
- sigmaX: 0  
- sigmaY: 0  

The package provides a simple test program that takes a local ToF image as input and calls interfaces from hobot_cv to implement Gaussian blur functionality. For detailed interface documentation, please refer to the README.md file in the hobot_cv package.

#### RDK Platform

<Tabs groupId="tros-distro">
<TabItem value="foxy" label="Foxy">

```bash
# Configure tros.b environment
source /opt/tros/setup.bash
```

</TabItem>

<TabItem value="humble" label="Humble">

```bash
# Configure tros.b environment
source /opt/tros/humble/setup.bash
```

</TabItem>

</Tabs>

```shell
# Copy required models and configuration files for the example from tros.b installation path.
cp -r /opt/tros/${TROS_DISTRO}/lib/hobot_cv/config/ .

# Launch BPU-accelerated test program
ros2 launch hobot_cv hobot_cv_gaussian_blur.launch.py

# Launch NEON-accelerated test program
ros2 launch hobot_cv hobot_cv_neon_blur.launch.py
```

### Result Analysis

#### BPU Acceleration

```text
Output results:

===================
image name :images/frame1_4.png
infe cost time:1314
guss_time cost time:2685
hobotcv save rate:0.510615

analyse_result start 
---------GaussianBlur
out_filter type:2,cols:320,rows:240,channel:1
cls_filter type:2,cols:320,rows:240,channel:1
out_filter minvalue:96,max:2363
out_filter min,x:319,y:115
out_filter max,x:147,y:239
cls_filter minvalue:96,max:2364
cls_filter min,x:319,y:115
cls_filter max,x:147,y:239

diff diff diff
mat_diff minvalue:0,max:2
mat_diff min,x:2,y:0
mat_diff max,x:110,y:14

error sum:8.46524e+06,max:2,mean_error:0.439232
analyse_result,time_used_ms_end:2
analyse_result end 

------------------------- 
```

Explanation:

- `infe cost time:1314` // Indicates that hobotcv-accelerated Gaussian blur took 1314 microseconds.  
- `guss_time cost time:2685` // Indicates that OpenCV Gaussian blur took 2685 microseconds.  
- `hobotcv save rate = (guss_time cost time - infe cost time) / guss_time cost time = 0.510615`  

Based on the above comparison, performance improves by approximately 50% after hobot_cv acceleration.

- `error sum:8.46524e+06, max:2, mean_error:0.439232` // Total error for a single image: 8.46524e+06; maximum per-pixel error: 2; average error: 0.439232  
- Average error = sum / (width × height) = 8.46524e+06 / (320 × 240)

Performance comparison between hobot_cv BPU-accelerated Gaussian blur and OpenCV Gaussian blur:

| Interface Type       | Kernel Size   | Time (ms) | Single-core CPU Usage (%) |
| -------------------- | ------------- | --------- | --------------------------|
| Hobotcv gaussian     | Size(3,3)     | 1.10435   | 15.9                      |
| Opencv gaussian      | Size(3,3)     | 2.41861   | 49.7                      |

#### NEON Acceleration

```text
Output results:
[neon_example-1] ===================
[neon_example-1] image name :config/tof_images/frame1_4.png
[neon_example-1] hobotcv mean cost time:674
[neon_example-1] opencv mean cost time:1025
[neon_example-1] hobotcv mean save rate:0.342439
[neon_example-1]
[neon_example-1] analyse_result start
[neon_example-1] ---------Mean_Blur
[neon_example-1] error sum:8.43744e+06,max:1,mean_error:0.430833
[neon_example-1]
[neon_example-1] hobotcv gaussian cost time:603
[neon_example-1] opencv gaussian cost time:2545
[neon_example-1] hobotcv gaussian save rate:0.763065
[neon_example-1]
[neon_example-1] analyse_result start
[neon_example-1] ---------Gaussian_Blur
[neon_example-1] error sum:9.13206e+06,max:1,mean_error:0.466302
[neon_example-1]
[neon_example-1] -------------------------
```

- `hobotcv gaussian cost time:603` // hobotcv NEON-accelerated Gaussian blur took 603 microseconds.  
- `opencv gaussian cost time:2545` // OpenCV Gaussian blur took 2545 microseconds.  
- `hobotcv gaussian save rate = (opencv cost time - hobotcv cost time) / opencv cost time = 0.763065`  

Based on the above comparison, performance improves by approximately 76% after hobotcv acceleration.

Performance comparison between hobot_cv NEON-accelerated Gaussian blur and OpenCV Gaussian blur:

| Interface Type       | Kernel Size   | Time (ms) | Single-core CPU Usage (%) |
| -------------------- | ------------- | --------- | --------------------------|
| Hobotcv gaussian     | Size(3,3)     | 0.430284  | 27.1                       |
| Opencv gaussian      | Size(3,3)     | 2.42225   | 47                         |
| Hobotcv gaussian     | Size(5,5)     | 0.854871  | 39.1                       |
| Opencv gaussian      | Size(5,5)     | 3.15647   | 99.8                       |

## Mean Blur

### Feature Introduction
Implement mean filtering functionality with NEON acceleration. Currently, only int16 and uint16 formats are supported.

Code repository: [https://github.com/D-Robotics/hobot_cv](https://github.com/D-Robotics/hobot_cv)

### Supported Platforms

| Platform                | Runtime Environment                     | Example Functionality                    |
| ----------------------- | --------------------------------------- | ---------------------------------------- |
| RDK X3, RDK X3 Module   | Ubuntu 20.04 (Foxy), Ubuntu 22.04 (Humble) | Read ToF image and apply mean filtering |

### Prerequisites

#### RDK Platform

1. The RDK has been flashed with Ubuntu 20.04 or Ubuntu 22.04 system image.
2. TogetheROS.Bot has been successfully installed on the RDK.

### Usage Instructions

The current version supports the following parameter ranges:

- Filter type: Mean filter  
- Supported data types: int16, uint16  
- Filter kernel sizes: 3x3, 5x5  

The package provides a simple test program that reads a local ToF image and calls interfaces from hobot_cv to perform mean filtering. For detailed interface documentation, please refer to the README.md file in the hobot_cv package.

#### RDK Platform

<Tabs groupId="tros-distro">
<TabItem value="foxy" label="Foxy">

```bash
# Configure tros.b environment
source /opt/tros/setup.bash
```

</TabItem>

<TabItem value="humble" label="Humble">

```bash
# Configure tros.b environment
source /opt/tros/humble/setup.bash
```

</TabItem>

</Tabs>

```shell
# Copy required configuration files for the example from TogetheROS installation path.
cp -r /opt/tros/${TROS_DISTRO}/lib/hobot_cv/config/ .

# Launch the test program package
ros2 launch hobot_cv hobot_cv_neon_blur.launch.py
```

### Result Analysis

```text
Output:
[neon_example-1] ===================
[neon_example-1] image name :config/tof_images/frame1_4.png
[neon_example-1] hobotcv mean cost time:674
[neon_example-1] opencv mean cost time:1025
[neon_example-1] hobotcv mean save rate:0.342439
[neon_example-1]
[neon_example-1] analyse_result start
[neon_example-1] ---------Mean_Blur
[neon_example-1] error sum:8.43744e+06,max:1,mean_error:0.430833
[neon_example-1]
[neon_example-1] -------------------------
```

Explanation:

- `hobotcv mean cost time:674` // The NEON-accelerated mean filtering interface in hobot_cv took 674 microseconds.
- `opencv mean cost time:1025` // OpenCV’s mean filtering took 1025 microseconds.
- `hobotcv mean save rate = (opencv cost time - hobotcv cost time) / opencv cost time = 0.342439`

Based on the comparison above, performance of mean filtering improved by 34% after applying hobotcv acceleration.

- `error sum:8.43744e+06, max:1, mean_error:0.430833`  
  Total error for a single image after mean filtering: 8.43744e+06  
  Maximum pixel-wise error: 1  
  Mean error: 0.430833  
  Mean error = total error / (width × height) = 8.43744e+06 / (320 × 240)

#### Performance Comparison Between hobot_cv and OpenCV

| Interface Type    | Kernel Size | Time (ms) | Single-core CPU Usage (%) |
| ----------------- | ----------- | --------- | --------------------------|
| Hobotcv mean      | Size(3,3)   | 0.466397  | 31.8                      |
| Opencv mean       | Size(3,3)   | 0.676677  | 40.2                      |
| Hobotcv mean      | Size(5,5)   | 0.737171  | 47.7                      |
| Opencv mean       | Size(5,5)   | 0.798177  | 52.9                      |

## crop

### Feature Description

Implements image cropping functionality. Currently, only NV12 format is supported.

Code repository: [https://github.com/D-Robotics/hobot_cv](https://github.com/D-Robotics/hobot_cv)

### Supported Platforms

| Platform                | Runtime Environment                     | Example Functionality          |
| ----------------------- | --------------------------------------- | ------------------------------ |
| RDK X3, RDK X3 Module   | Ubuntu 20.04 (Foxy), Ubuntu 22.04 (Humble) | Read image and perform cropping |
| RDK X5, RDK X5 Module   | Ubuntu 22.04 (Humble)                   | Read image and perform cropping |

### Prerequisites

#### RDK Platform

1. The RDK has been flashed with Ubuntu 20.04 or Ubuntu 22.04 system image.
2. TogetheROS.Bot has been successfully installed on the RDK.

### Usage Instructions

#### RDK Platform

<Tabs groupId="tros-distro">
<TabItem value="foxy" label="Foxy">

```bash
# Configure tros.b environment
source /opt/tros/setup.bash
```

</TabItem>

<TabItem value="humble" label="Humble">

```bash
# Configure tros.b environment
source /opt/tros/humble/setup.bash
```

</TabItem>

</Tabs>

```shell
# Copy required models and configuration files for the example from tros.b installation path.
cp -r /opt/tros/${TROS_DISTRO}/lib/hobot_cv/config/ .

# Launch the launch file
ros2 launch hobot_cv hobot_cv_crop.launch.py
```

### Result Analysis

```text
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [crop_example-1]: process started with pid [3064]
[crop_example-1] [INFO] [1655951627.255477663] [example]: crop image to 960x540 pixels, time cost: 1 ms
[crop_example-1] [INFO] [1655951627.336889080] [example]: crop image to 960x540 pixels, time cost: 1 ms
[INFO] [crop_example-1]: process has finished cleanly [pid 3064]
```

According to the logs, the test program successfully cropped a local 1920×1080 resolution image. Processing time is as follows:

| Image Processing                        | Execution Time |
| --------------------------------------- | -------------- |
| Crop 1920×1080 to 960×540               | 1 ms           |

The original local image is 1920×1080; the top-left 960×540 region was cropped. The resulting image is shown below:

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/02_quick_demo/image/demo_cv/ori-crop.png)

## resize

### Feature Description

Implements image resizing functionality. Currently, only NV12 format is supported.

Code repository: [https://github.com/D-Robotics/hobot_cv](https://github.com/D-Robotics/hobot_cv)

### Supported Platforms

| Platform                              | Runtime Environment                     |
| ------------------------------------- | --------------------------------------- |
| RDK X3, RDK X3 Module                 | Ubuntu 20.04 (Foxy), Ubuntu 22.04 (Humble) |
| RDK X5, RDK X5 Module, RDK S100       | Ubuntu 22.04 (Humble)                   |
| RDK Ultra                             | Ubuntu 20.04 (Foxy)                     |
| X86                                   | Ubuntu 20.04 (Foxy)                     |

### Prerequisites

#### RDK Platform

1. The RDK has been flashed with Ubuntu 20.04 or Ubuntu 22.04 system image.
2. TogetheROS.Bot has been successfully installed on the RDK.
#### X86 Platform

1. Confirm that the X86 platform system is Ubuntu 20.04 and that TogetheROS.Bot has been successfully installed.

### Usage Guide

#### RDK/X86

<Tabs groupId="tros-distro">
<TabItem value="foxy" label="Foxy">

```bash
# Configure the tros.b environment
source /opt/tros/setup.bash
```

</TabItem>

<TabItem value="humble" label="Humble">

```bash
# Configure the tros.b environment
source /opt/tros/humble/setup.bash
```

</TabItem>

</Tabs>

```shell
# Copy the required models and configuration files for running the example from the TogetheROS installation path.
cp -r /opt/tros/${TROS_DISTRO}/lib/hobot_cv/config/ .

# Launch the launch file
ros2 launch hobot_cv hobot_cv_resize.launch.py
```

### Result Analysis

#### Resize on RDK X3 Platform

```shell
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [resize_example-1]: process started with pid [3083]
[resize_example-1] [INFO] [1655951649.930987924] [example]:
[resize_example-1] source image config/test.jpg is 1920x1080 pixels
[resize_example-1] [INFO] [1655951649.931155799] [example]: resize image to 960x540 pixels, time cost: 297 ms
[resize_example-1] [INFO] [1655951650.039223757] [example]: resize image to 960x540 pixels, time cost: 15 ms
[INFO] [resize_example-1]: process has finished cleanly [pid 3083]
```

According to the log, the test program successfully processed a local image with resolution 1920x1080 via resizing. The interface was called twice, with the following execution times:

| Image Processing                         | First Run Time | Second Run Time |
| ---------------------------------------- | -------------- | --------------- |
| Resize from 1920x1080 to 960x540         | 297 ms         | 15 ms           |

The first run took longer because it required hardware configuration of the VPS. Once the hardware configuration is finalized and unchanged, subsequent processing is handled directly by the hardware, significantly reducing execution time.

The original local image (1920x1080) and the resized image (960x540) are shown below:

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/02_quick_demo/image/demo_cv/ori-resize.png)

#### Performance Comparison on RDK X3 Platform

CPU usage was monitored using the `top` command, representing the CPU percentage consumed by the test process.  
Timing measurements are in milliseconds (ms), averaged over 1000 iterations.  
During testing, the CPU frequency was locked as follows:

```shell
sudo bash -c 'echo performance > /sys/devices/system/cpu/cpufreq/policy0/scaling_governor'
```

| src wxh   | dst wxh   | VPS Time | VPS Interface<br/>CPU Usage | BPU Time | BPU Interface<br/>CPU Usage | OpenCV Time | OpenCV Processing<br/>CPU Usage |
| --------- | --------- | -------- | --------------------------- | -------- | --------------------------- | ----------- | ------------------------------- |
| 512x512   | 128x128   | 1.53789  | 25.9                        | 1.11054  | 89                          | 1.71119     | 100.3                           |
| 640x640   | 320x320   | 2.48536  | 28.5                        | 1.82232  | 88                          | 1.82384     | 338.9                           |
| 896x896   | 384x384   | 4.54422  | 24.6                        | 2.81954  | 79.7                        | 7.84396     | 273.1                           |
| 1024x1024 | 512x512   | 6.01103  | 25.2                        | 3.89325  | 81.7                        | 2.55761     | 381.7                           |
| 1920x1088 | 512x512   | 11.0406  | 20.6                        | 5.8513   | 71.1                        | 8.19324     | 380.1                           |
| 1920x1080 | 960x544   | 11.1562  | 22.3                        | 7.09085  | 77.7                        | 15.2978     | 382.4                           |

## Rotate

### Feature Introduction

The rotate function implements image rotation. Currently, it only supports images in NV12 format, with supported rotation angles of 90°, 180°, and 270°.

Code repository: [https://github.com/D-Robotics/hobot_cv](https://github.com/D-Robotics/hobot_cv)

### Supported Platforms

| Platform                | Runtime Environment                     | Example Functionality              |
| ----------------------- | --------------------------------------- | ---------------------------------- |
| RDK X3, RDK X3 Module   | Ubuntu 20.04 (Foxy), Ubuntu 22.04 (Humble) | Read an image and perform rotation |

### Prerequisites

#### RDK Platform

1. The RDK has been flashed with an Ubuntu 20.04 or Ubuntu 22.04 system image.
2. TogetheROS.Bot has been successfully installed on the RDK.

### Usage Guide

#### RDK Platform

<Tabs groupId="tros-distro">
<TabItem value="foxy" label="Foxy">

```bash
# Configure the tros.b environment
source /opt/tros/setup.bash
```

</TabItem>

<TabItem value="humble" label="Humble">

```bash
# Configure the tros.b environment
source /opt/tros/humble/setup.bash
```

</TabItem>

</Tabs>

```shell
# Copy the required models and configuration files for running the example from the tros.b installation path.
cp -r /opt/tros/${TROS_DISTRO}/lib/hobot_cv/config/ .

# Launch the launch file
ros2 launch hobot_cv hobot_cv_rotate.launch.py
```

### Result Analysis

```shell
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [rotate_example-1]: process started with pid [3096]
[rotate_example-1] [INFO] [1655951661.173422471] [example]: rotate image 180 , time cost: 415 ms
[rotate_example-1]
[rotate_example-1] [INFO] [1655951661.416188013] [example]: second rotate image 180 , time cost: 40 ms
[rotate_example-1]
[INFO] [rotate_example-1]: process has finished cleanly [pid 3096]
```

According to the log, the test program successfully rotated a local image with resolution 1920x1080. The interface was called twice, with the following execution times:

| Image Processing                   | First Run Time | Second Run Time |
| ---------------------------------- | -------------- | --------------- |
| Rotate 1920x1080 by 180 degrees    | 415 ms         | 40 ms           |

The first run took longer because it required hardware configuration of the VPS. Once the hardware configuration is finalized and unchanged, subsequent processing is handled directly by the hardware, significantly reducing execution time.

The original local image (1920x1080) and the rotated image (1920x1080) are shown below:

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/03_boxs/segmentation/image/yolov8_seg/test.jpg)

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/02_quick_demo/image/demo_cv/rotate.jpg)

#### Performance Comparison Between hobot_cv and OpenCV

CPU usage was monitored using the `top` command, representing the CPU percentage consumed by the test process.  
Timing measurements are in milliseconds (ms), averaged over 1000 iterations.  
During testing, the CPU frequency was locked as follows:

```shell
sudo bash -c 'echo performance > /sys/devices/system/cpu/cpufreq/policy0/scaling_governor'
```

| src wxh   | Rotation Angle | hobot_cv Time | hobot_cv Interface CPU Usage | OpenCV Time | OpenCV Processing CPU Usage |
| --------- | -------------- | ------------- | ---------------------------- | ----------- | --------------------------- |
| 1920x1080 | 90             | 37.6568       | 61.6                         | 55.8886     | 100.0                       |
| 640x640   | 180            | 7.3133        | 66.8                         | 5.1806      | 100.0                       |
| 896x896   | 270            | 14.7723       | 62.5                         | 13.6497     | 100.0                       |

## Pyramid

### Feature Introduction

Implements image pyramid scaling functionality. Currently, only NV12 format is supported.

Code repository: [https://github.com/D-Robotics/hobot_cv](https://github.com/D-Robotics/hobot_cv)

### Supported Platforms

| Platform                | Runtime Environment                     | Example Functionality                    |
| ----------------------- | --------------------------------------- | ---------------------------------------- |
| RDK X3, RDK X3 Module   | Ubuntu 20.04 (Foxy), Ubuntu 22.04 (Humble) | Read an image and perform pyramid scaling |

### Prerequisites

#### RDK Platform

1. The RDK has been flashed with an Ubuntu 20.04 or Ubuntu 22.04 system image.
2. RDK has successfully installed TogetheROS.Bot.

### Usage Guide

#### RDK Platform


<Tabs groupId="tros-distro">
<TabItem value="foxy" label="Foxy">

```bash
# Configure the tros.b environment
source /opt/tros/setup.bash
```

</TabItem>

<TabItem value="humble" label="Humble">

```bash
# Configure the tros.b environment
source /opt/tros/humble/setup.bash
```

</TabItem>

</Tabs>

```shell
# Copy the models and configuration files required for running the example from the tros.b installation path.
cp -r /opt/tros/${TROS_DISTRO}/lib/hobot_cv/config/ .

# Launch the launch file
ros2 launch hobot_cv hobot_cv_pyramid.launch.py
```

### Result Analysis

```shell
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [pyramid_example-1]: process started with pid [3071]
[pyramid_example-1] [INFO] [1655951639.110992960] [example]: pyramid image , time cost: 299 ms
[pyramid_example-1]
[pyramid_example-1] [INFO] [1655951639.432398919] [example]: pyramid image , time cost: 19 ms
[pyramid_example-1]
[INFO] [pyramid_example-1]: process has finished cleanly [pid 3071]
```

According to the log, the test program successfully processed a local 1920x1080 resolution image using pyramid downsampling. The interface was called twice, with the following execution times:

| Image Processing                              | First Run Time | Second Run Time |
| -------------------------------------------- | -------------- | --------------- |
| Output of base layer (6-level pyramid) for 1920x1080 image | 299ms          | 19ms            |

The first run took longer because it required hardware configuration for VPS. Once the hardware configuration remains unchanged, subsequent processing is handled directly by the hardware, significantly reducing execution time.

The original local image (1920x1080) and the pyramid-downscaled image are shown below:

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/02_quick_demo/image/demo_cv/pym_ds.jpg)

The output includes six base layers, each layer's size being half that of the previous layer.

#### Performance Comparison

For an input image of 1920x1080 resolution, five upper pyramid layers were generated, resulting in images with resolutions of 960x540, 480x270, 240x134, 120x66, and 60x32 respectively. The efficiency of OpenCV and hobotcv was compared, with the following results:

CPU usage is reported as a percentage of a single core; time measurements are in milliseconds (ms).

| VPS Interface Time | VPS Interface CPU Usage (%) | OpenCV Time | OpenCV CPU Usage (%) |
| ------------------ | --------------------------- | ----------- | -------------------- |
| 19ms               | 42.5                        | 56          | 100                  |

## Color

### Feature Introduction

Implements conversion between NV12 and BGR24 image formats.

Code repository: [https://github.com/D-Robotics/hobot_cv](https://github.com/D-Robotics/hobot_cv)

### Supported Platforms

| Platform                              | Runtime Environment     | Example Functionality             |
| ------------------------------------- | ----------------------- | --------------------------------- |
| RDK X5, RDK X5 Module, RDK S100       | Ubuntu 22.04 (Humble)   | Conversion between NV12 and BGR24 |

### Prerequisites

#### RDK Platform

1. RDK has been flashed with the Ubuntu 22.04 system image.

2. RDK has successfully installed TogetheROS.Bot.

### Usage Guide

#### RDK Platform


<Tabs groupId="tros-distro">

<TabItem value="humble" label="Humble">

```bash
# Configure the tros.b environment
source /opt/tros/humble/setup.bash
```

</TabItem>

</Tabs>

```shell
# Copy the models and configuration files required for running the example from the tros.b installation path.
cp -r /opt/tros/${TROS_DISTRO}/lib/hobot_cv/config/ .

# Launch the launch file
ros2 launch hobot_cv hobot_cv_conversion.launch.py
```

### Result Analysis

```text
[INFO] [launch]: All log files can be found below /root/.ros/log/2025-03-25-14-50-55-535138-ubuntu-4139
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [test_conersion-1]: process started with pid [4140]
[test_conersion-1] [INFO] [1742885455.683144151] [hobot_cv]: bgr24_to_nv12 opencv time cost: 4 ms
[test_conersion-1] [INFO] [1742885455.685469463] [hobot_cv]: nv12_to_bgr24 neon 1 time cost: 2 ms
[test_conersion-1] [INFO] [1742885455.836798125] [hobot_cv]: nv12_to_bgr24 neon 2 time cost: 2 ms
[test_conersion-1] [INFO] [1742885455.992973665] [hobot_cv]: bgr24_to_nv12 neon 1 time cost: 1 ms
[test_conersion-1] [INFO] [1742885455.997803043] [hobot_cv]: nv12_to_bgr24 opencv time cost: 4 ms
[test_conersion-1] [INFO] [1742885456.156813423] [hobot_cv]: bgr24_to_nv12 neon 2 time cost: 1 ms
[test_conersion-1] [INFO] [1742885456.161413872] [hobot_cv]: nv12_to_bgr24 opencv time cost: 4 ms
[INFO] [test_conersion-1]: process has finished cleanly [pid 4140]
```