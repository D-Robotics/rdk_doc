---
sidebar_position: 6
---

# 3.2.6 vio2encoder Sample Introduction

## Sample Overview
vio2encoder is a **C language interface** development code sample located in the `/app/cdev_demo` directory, demonstrating how to open the camera using C language and encode the data collected by the camera. By referring to this sample, users can understand and develop related applications.

## Effect Demonstration
vio2encoder is a sample that generates video streams. For example, after executing `./vio2encoder -w 1920 -h 1080 --iwidth 1920 --iheight 1080 -o stream.h264`, a stream.h264 file will be generated in the current directory.
```
root@ubuntu:/app/cdev_demo/vio2encoder# tree
.
├── Makefile
├── stream.h264
├── vio2encoder
├── vio2encoder.c
└── vio2encoder.o

```

## Hardware Preparation

### Hardware Connection
This sample requires a camera, here using the standard IMX219 for connection.

![connect-img](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/02_cdev_demo_sample/image/vio2capture_hardware_connect.png)


## Quick Start

### Code and Board Location
```
root@ubuntu:/app/cdev_demo/vio2encoder# tree
.
├── Makefile
└── vio2encoder.c
```

### Compilation and Execution
We can directly use make in this directory to compile the vio2encoder executable file.
```
root@ubuntu:/app/cdev_demo/vio2encoder# tree
.
├── Makefile
├── vio2encoder
├── vio2encoder.c
└── vio2encoder.o
```

### Execution Result
```
root@ubuntu:/app/cdev_demo/vio2encoder# ./vio2encoder -w 1920 -h 1080 --iwidth 1920 --iheight 1080 -o stream.h264
2000/01/01 13:07:01.434 !INFO [OpenCamera][0450]hbn module
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
2000/01/01 13:07:01.564 !INFO [CamInitParam][0326]Setting VSE channel-0: input_width:1920, input_height:1080, dst_w:1920, dst_h:1080
2000/01/01 13:07:01.565 !INFO [CamInitParam][0326]Setting VSE channel-1: input_width:1920, input_height:1080, dst_w:1920, dst_h:1080
2000/01/01 13:07:01.565 !INFO [vp_vin_init][0055]csi0 ignore mclk ex attr, because mclk is not configed at device tree.
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
2000/01/01 13:07:02.278 !INFO [OpenEncode][0072]pipe:0 type:0 1920x1080 bit_rate:8000 begin init

2000/01/01 13:07:02.278 !INFO [vp_encode_config_param][0418]codec type is h264: frame size:3110912  frame rate: 30
sp_start_encode success!
2000/01/01 13:07:02.315 !INFO [BindTo][0088]BindTo_CHN:-1

2000/01/01 13:07:02.315 !INFO [BindTo][0093]m_prev_module_chn:0

sp_module_bind(vio -> encoder) success!

```

## Detailed Introduction

### Sample Program Parameter Options Description

```
root@ubuntu:/app/cdev_demo/vio2encoder# ./vio2encoder 
Usage: vio2encoder [OPTION...]
vio2encode sample -- An example of using the camera to record and encode

  -h, --oheight=height       height of output video
      --iheight=height       sensor output height
      --iwidth=width         sensor output width
  -o, --output=path          output file path
  -w, --owidth=width         width of output video
  -?, --help                 Give this help list
      --usage                Give a short usage message

Mandatory or optional arguments to long options are also mandatory or optional
for any corresponding short options.
root@ubuntu:/app/cdev_demo/vio2encoder# 
```

-w: Encoded video width\
-h: Encoded video height\
--iwidth: Sensor output width\
--iheight: Sensor output height\
-o: Encoding output path

### Software Architecture Description
This sample encodes the data collected by the camera into H264 streams, so it requires resources such as the camera and encoder. When both the camera and encoder are initialized normally, it starts continuously collecting camera data and saving it to the stream.

<center>
![software_arch](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/02_cdev_demo_sample/image/cdev_vio2encoder_software_arch.png)
</center>

### API Flow Description
<center>
![software_arch](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/02_cdev_demo_sample/image/cdev_vio2encoder_api_flow.png)
</center>


