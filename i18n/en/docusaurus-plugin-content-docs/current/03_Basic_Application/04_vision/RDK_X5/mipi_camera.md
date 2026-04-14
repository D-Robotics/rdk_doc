---
sidebar_position: 1
---

# MIPI Camera Usage
For the use of the MIPI camera, you can directly refer to the "MIPI Camera Real-Time Detection" example in the Python samples.

## Example Overview
The MIPI camera real-time detection example is a **Python interface** development code example located in `/app/pydev_demo/08_mipi_camera_sample`. It demonstrates how to use the onboard MIPI camera for real-time object detection. This example uses the YOLOv5x object detection model to perform real-time inference on the video stream captured by the MIPI camera and displays the detection results via HDMI, outputting bounding box information.

Included examples:
```
root@ubuntu:/app/pydev_demo/08_mipi_camera_sample$ tree
.
├── 01_mipi_camera_yolov5s.py
├── 02_mipi_camera_dump.py
├── 03_mipi_camera_scale.py
├── 04_mipi_camera_crop_scale.py
├── 05_mipi_camera_streamer.py
└── coco_classes.names
```

## Effect Demonstration

### 01 Real-Time Object Detection Effect

:::info Visualizing Detection Results

To view real-time camera footage and visualized detection results on a display, you need to:

1. **Connect an external display**: Connect the development board to a monitor using an HDMI cable.
2. **Special handling for Desktop version**: If using the Desktop version of the system, first execute the following command to stop the desktop service:
   ```bash
   sudo systemctl stop lightdm
   ```
3. **Remote connection**: Connect to the board remotely via SSH.
4. **Run the code**: After executing the example program, you will see the real-time detection results on the connected display.

:::

![output-img](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/RDK_X5_Newly/pydev_08_mipi_camera_yolov5s_runing.jpg)

### 02 Image Capture and Save Effect

After running, multiple YUV format image files will be saved in the same directory as the script, with a default resolution of 1920x1080.

![output-img](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/RDK_X5_Newly/pydev_08_mipi_camera_dump.png)

### 03 Image Scaling Effect

After running, scaled YUV image files will be saved in the same directory as the script, with a default resolution of 640x360.

![output-img](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/RDK_X5_Newly/pydev_08_mipi_camera_scale.png)

### 04 Image Cropping and Scaling Effect

After running, cropped and scaled YUV image files (NV12 format) will be saved in the same directory as the script. By default, the center of the image is cropped and scaled. Adjusting the cropping position yields the following YUV image.

![output-img](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/RDK_X5_Newly/pydev_08_mipi_camera_crop_scale.png)

### 05 Real-Time Streaming Effect

After running, the camera feed is displayed in real-time on the HDMI screen (streaming test). Note that for the Desktop version, you must first execute `sudo systemctl stop lightdm` to stop the desktop service.

![output-img](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/RDK_X5_Newly/pydev_08_mipi_camera_stream.gif)

## Hardware Preparation

### Hardware Connection
1. Prepare an RDK development board.
2. Connect the officially adapted MIPI camera.
3. Connect the monitor and development board using an HDMI cable.
4. Connect the power cable and network cable.

![connect-img](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/RDK_X5_Newly/pydev_03_hw_connect.png)

## Quick Start

### Code and Board Location
The example files are located in `/app/pydev_demo/08_mipi_camera_sample`.

### Compilation and Execution
Python examples do not require compilation and can be run directly:

Running 01_mipi_camera_yolov5s.py:

```
cd /app/pydev_demo/08_mipi_camera_sample
python 01_mipi_camera_yolov5s.py
```

Running 02_mipi_camera_dump.py:

```
cd /app/pydev_demo/08_mipi_camera_sample
python 02_mipi_camera_dump.py -f 30 -c 10 -w 1920 -h 1080
```

Running 03_mipi_camera_scale.py:

```
cd /app/pydev_demo/08_mipi_camera_sample

# This example requires input.yuv as input. Here, we use output0.yuv from the previous example as input and execute the copy command.
cp output0.yuv input.yuv

# Then run the example
python 03_mipi_camera_scale.py -i input.yuv -o output_640x360.yuv -w 640 -h 360 --iwidth 1920 --iheight 1080
```

Running 04_mipi_camera_crop_scale.py:

```
cd /app/pydev_demo/08_mipi_camera_sample
python 04_mipi_camera_crop_scale.py -i input.yuv -o output_640x480.yuv -w 640 -h 480 --iwidth 1920 --iheight 1080 -x 304 -y 304 --crop_w 896 --crop_h 592
```

Running 05_mipi_camera_streamer.py:
```
cd /app/pydev_demo/08_mipi_camera_sample
python 05_mipi_camera_streamer.py -w 1920 -h 1080
```

<!-- ### Execution Effects
Execution effect of 01_mipi_camera_yolov5s.py:
After running, the program initializes the MIPI camera and HDMI display and begins real-time object detection. The detection results are displayed via HDMI.

Execution effect of 02_mipi_camera_dump.py:
After successful execution, multiple YUV files will be stored in the script's directory.

Execution effect of 03_mipi_camera_scale.py:
After successful execution, the scaled YUV file will be stored in the script's directory.

Execution effect of 04_mipi_camera_crop_scale.py:
After successful execution, the cropped and scaled YUV file will be stored in the script's directory. Note: The cropping width must be an integer multiple of 16 (i.e., aligned to 16 bytes).

Execution effect of 05_mipi_camera_streamer.py:
After successful execution, the screen will display the real-time feed. -->

[For more details, you can directly view the example](../../03_pydev_demo_sample/RDK_X5/08_mipi_camera_sample.md)