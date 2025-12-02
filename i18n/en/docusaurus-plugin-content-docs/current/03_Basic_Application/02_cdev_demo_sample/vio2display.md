---
sidebar_position: 5
---

# 3.2.5 vio2display Example Introduction

## Example Overview
vio2display is a **C language interface** development code example located in the /app/cdev_demo directory, used to demonstrate how to open a camera using C language and display the camera-captured data on a display. By referencing this example, users can understand and develop related applications.

## Effect Demonstration

As shown below, when executing the example, the display connected to the RDK's HDMI port captured the execution log displayed on the screen.

![connect-img](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/02_cdev_demo_sample/image/cdev_vio2display_example_effect.png)

## Hardware Preparation

### Hardware Connection

This example does not require a mouse and keyboard, so only the camera, HDMI display, Ethernet cable interface, and power cable are connected here.

![connect-img](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/02_cdev_demo_sample/image/vio_display_hardware_connect.png)

## Quick Start

### Code and Board Location
Navigate to the /app/cdev_demo/vio2display location, where you can see that the vio2display example contains 2 files:
```
root@ubuntu:/app/cdev_demo/vio2display# tree
.
├── Makefile
└── vio2display.c
```

### Compilation and Execution
We can directly use make in this directory to compile the vio2display executable file.
```
root@ubuntu:/app/cdev_demo/vio2display# tree
.
├── Makefile
├── vio2display
├── vio2display.c
└── vio2display.o
```

### Execution Effect

We first need to stop the lightdm service, then execute vio2display. Note that we need to pay attention to the camera resolution and fill in the -h and -w parameters according to the camera's resolution.

```
root@ubuntu:/app/cdev_demo/vio2display# systemctl stop lightdm
root@ubuntu:/app/cdev_demo/vio2display# ./vio2display -h 1080 -w 1920
Opened DRM device: /dev/dri/card0
1920x1080
1360x768
1280x1024
1280x720
1024x768
800x600
720x576
720x480
640x480
disp_w=1920, disp_h=1080
2000/01/01 12:45:58.112 !INFO [OpenCamera][0450]hbn module
set camera fps: -1,width: 1920,height: 1080
Camera 0:
        mipi_host: 0
Camera 1:
        mipi_host: 2
Camera 2:
        mipi_host: 0
Camera 3:
        mipi_host: 0
mipi mclk is not configed.
Searching camera sensor on device: /proc/device-tree/soc/cam/vcon@0 i2c bus: 6 mipi rx phy: 0
WARN: Sensor Name: sc1330t, Expected Chip ID: 0xCA18, Actual Chip ID Read: 0x00
WARN: Sensor Name: irs2875-tof, Expected Chip ID: 0x2875, Actual Chip ID Read: 0x00
WARN: Sensor Name: sc230ai-10fps, Expected Chip ID: 0xCB34, Actual Chip ID Read: 0x00
WARN: Sensor Name: sc230ai-30fps, Expected Chip ID: 0xCB34, Actual Chip ID Read: 0x00
WARN: Sensor Name: sc132gs-1280p, Expected Chip ID: 0x132, Actual Chip ID Read: 0x00
WARN: Sensor Name: sc035hgs, Expected Chip ID: 0x5A, Actual Chip ID Read: 0x00
WARN: Sensor Name: ov5640, Expected Chip ID: 0x5640, Actual Chip ID Read: 0x00
WARN: Sensor Name: f37, Expected Chip ID: 0xF37, Actual Chip ID Read: 0x00
WARN: Sensor Name: imx415-30fps-2lane, Expected Chip ID: 0x03, Actual Chip ID Read: 0x00
WARN: Sensor Name: imx415-30fps-4lane, Expected Chip ID: 0x03, Actual Chip ID Read: 0x00
WARN: Sensor Name: sc202cs-1600x1200, Expected Chip ID: 0xEB52, Actual Chip ID Read: 0x00
WARN: Sensor Name: irs2381c-tof, Expected Chip ID: 0x2381, Actual Chip ID Read: 0x00
[0] INFO: Found sensor name:imx219-1920x1080-30fps on mipi rx csi 0, i2c addr 0x10, config_file:linear_1920x1080_raw10_30fps_2lane.c
WARN: Sensor Name: ov5647-640x480-60fps, Expected Chip ID: 0x5647, Actual Chip ID Read: 0x00
WARN: Sensor Name: ov5647-1280x960-30fps, Expected Chip ID: 0x5647, Actual Chip ID Read: 0x00
WARN: Sensor Name: ov5647-1920x1080-30fps, Expected Chip ID: 0x5647, Actual Chip ID Read: 0x00
WARN: Sensor Name: ov5647-2592x1944-15fps, Expected Chip ID: 0x5647, Actual Chip ID Read: 0x00
WARN: Sensor Name: imx477-1280x960-120fps, Expected Chip ID: 0x477, Actual Chip ID Read: 0x00
WARN: Sensor Name: imx477-1920x1080-50fps, Expected Chip ID: 0x477, Actual Chip ID Read: 0x00
WARN: Sensor Name: imx477-2016x1520-40fps, Expected Chip ID: 0x477, Actual Chip ID Read: 0x00
WARN: Sensor Name: imx477-4000x3000-10fps, Expected Chip ID: 0x477, Actual Chip ID Read: 0x00
WARN: Sensor Name: sc035hgs-vc0, Expected Chip ID: 0x35, Actual Chip ID Read: 0x00
WARN: Sensor Name: sc035hgs-vc1, Expected Chip ID: 0x35, Actual Chip ID Read: 0x00
WARN: Sensor Name: sc231ai-30fps, Expected Chip ID: 0xCB6A, Actual Chip ID Read: 0x00
WARN: Sensor Name: imx586-30fps-4lane, Expected Chip ID: 0x586, Actual Chip ID Read: 0x00
WARN: Sensor Name: os08c10-30fps-2lane, Expected Chip ID: 0x53, Actual Chip ID Read: 0x00
2000/01/01 12:45:58.244 !INFO [CamInitParam][0326]Setting VSE channel-0: input_width:1920, input_height:1080, dst_w:1920, dst_h:1080
2000/01/01 12:45:58.244 !INFO [CamInitParam][0326]Setting VSE channel-1: input_width:1920, input_height:1080, dst_w:1920, dst_h:1080
2000/01/01 12:45:58.244 !INFO [vp_vin_init][0055]csi0 ignore mclk ex attr, because mclk is not configed at device tree.
================= VP Modules Status ====================
======================== VFLOW =========================
(active)[S0] vin0_C0*(dma)-m2m-isp0_C0-m2m-vse0_C0
========================= SIF ==========================
------------------- flow0 info -------------------
rx_index:0
vc_index:0
ipi_channels:1
width:1920
height:1080
format:0x2b
online_isp:1
ddr_en:0
bufnum:0
emb_en:0
embeded_dependence:0
embeded_width:0
embeded_height:0
size_err_cnt:1
========================= ISP ==========================
------------------- flow0 info -------------------
input_mode:2
sched_mode:0
tile_mode:0
af_mode:0
sensor_mode:0
input_width:1920
input_height:1080
input_format:1
input_bit_width:10
input_crop_x:0
input_crop_y:0
input_crop_w:1920
input_crop_h:1080
ddr_en:1
output_format:2
output_bit_width:8
========================= VSE ==========================
------------------- flow0 info -------------------
input_fps:0/0
input_width:1920
input_height:1080
input_format:2
input_bitwidth:8
dns0 channel: roi [0][0][1920][1080], target [1920][1080], fps [0/0]
dns1 channel: roi [0][0][1920][1080], target [1920][1080], fps [0/0]
========================= VENC =========================
========================= VDEC =========================
========================= JENC =========================
======================= Buffer =========================
----------------------------------------------
flowid module cid chn FREE  REQ  PRO  COM USED
----------------------------------------------
0      vin0   0   0     16    0    0    0    0
0      vin0   0   8      0    1    2    0    0

0      isp0   0   0     16    0    0    0    0
0      isp0   0   8      0    3    0    0    0

0      vse0   0   0     16    0    0    0    0
0      vse0   0   8      0    3    0    0    0
0      vse0   0   9      0    3    0    0    0

----------------------------------------------
flowid module cid chn FREE  REQ  PRO  COM USED
----------------------------------------------
0      vin0   0   0     16    0    0    0    0
0      vin0   0   8      0    1    2    0    0

0      isp0   0   0     16    0    0    0    0
0      isp0   0   8      0    3    0    0    0

0      vse0   0   0     16    0    0    0    0
0      vse0   0   8      0    3    0    0    0
0      vse0   0   9      0    3    0    0    0

========================= END ===========================
sp_open_camera success!
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
    Modes: 32
    Subpixel: 1
    Mode 0: 1920x1080 @ 60Hz
    Mode 1: 1920x1080 @ 60Hz
    Mode 2: 1920x1080 @ 60Hz
    Mode 3: 1920x1080i @ 60Hz
    Mode 4: 1920x1080i @ 60Hz
    Mode 5: 1920x1080 @ 50Hz
    Mode 6: 1920x1080i @ 50Hz
    Mode 7: 1280x1024 @ 60Hz
    Mode 8: 1440x900 @ 60Hz
    Mode 9: 1360x768 @ 60Hz
    Mode 10: 1280x720 @ 60Hz
    Mode 11: 1280x720 @ 60Hz
    Mode 12: 1280x720 @ 50Hz
    Mode 13: 1024x768 @ 60Hz
    Mode 14: 800x600 @ 60Hz
    Mode 15: 720x576 @ 50Hz
    Mode 16: 720x576 @ 50Hz
    Mode 17: 720x576 @ 50Hz
    Mode 18: 720x576i @ 50Hz
    Mode 19: 720x576i @ 50Hz
    Mode 20: 720x480 @ 60Hz
    Mode 21: 720x480 @ 60Hz
    Mode 22: 720x480 @ 60Hz
    Mode 23: 720x480 @ 60Hz
    Mode 24: 720x480 @ 60Hz
    Mode 25: 720x480i @ 60Hz
    Mode 26: 720x480i @ 60Hz
    Mode 27: 720x480i @ 60Hz
    Mode 28: 720x480i @ 60Hz
    Mode 29: 640x480 @ 60Hz
    Mode 30: 640x480 @ 60Hz
    Mode 31: 640x480 @ 60Hz
2000/01/01 12:45:59.074 !INFO [BindTo][0088]BindTo_CHN:-1

2000/01/01 12:45:59.074 !INFO [BindTo][0093]m_prev_module_chn:0


Press 'q' to Exit !
2000/01/01 12:45:59.088 !INFO [SetImageFrame][0495]N2D init done!

Created new framebuffer: fb_id=75 for dma_buf_fd=59
add mapping dma_buf_fd:59 fb_id:75, mapping_count: 1
Created new framebuffer: fb_id=79 for dma_buf_fd=60
add mapping dma_buf_fd:60 fb_id:79, mapping_count: 2
```

## Detailed Introduction

### Example Program Parameter Options Description
We can directly execute the target file to confirm the parameter options description:
```
root@ubuntu:/app/cdev_demo/vio2display# ./vio2display 
Usage: vio2display [OPTION...]
vio(sensors) display sample -- An example of display sensor's image

  -h, --height=height        sensor output height
  -w, --width=width          sensor output width
  -?, --help                 Give this help list
      --usage                Give a short usage message

Mandatory or optional arguments to long options are also mandatory or optional
for any corresponding short options.
```
Where: \
-h is a mandatory option, representing the pixel height of the input video \
-w is a mandatory option, representing the pixel width of the input video \
-i is a mandatory option, representing the file path of the input video

### Software Architecture Description

This example mainly captures YUV and RAW images from the camera. The logic is relatively simple: after opening the camera, call the interfaces provided by the libspcdev library, and then directly obtain YUV and RAW images for saving.

<center>
![software_arch](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/02_cdev_demo_sample/image/cdev_vio2display_software_arch.png)
</center>

### API Flow Description

<center>
![software_arch](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/02_cdev_demo_sample/image/cdev_vio2display_api_flow.png)
</center>

