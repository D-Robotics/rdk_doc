---
sidebar_position: 4
---

# 3.2.4 vio_capture 示例介绍

## 示例简介
vio_capture 是一个位于 /app/cdev_demo 中的 C 语言开发代码示例 \
本示例用于从摄像头捕获 YUV 格式图像和 RAW 原始数据，并保存为本地文件。

## 效果展示
我们通过 vscode 执行本示例，观察摄像头捕获的图像
![connect-img](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/02_cdev_demo_sample/image/cdev_vio_capture_example_effect_1.png)

捕获的图片我们用 yuvplayer 检查，看是否符合我们的预期。

![connect-img](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/02_cdev_demo_sample/image/yuv0_img.png)


## 硬件准备

### 硬件连接

该示例不需要鼠标和键盘，所以这里连接了摄像头 HDMI 显示屏，网线接口，电源线

![connect-img](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/02_cdev_demo_sample/image/cdev_vio_capture_hardware_connect.png)

## 快速开始

### 代码以及板端位置
```
root@ubuntu:/app/cdev_demo/vio_capture# tree
.
├── capture.c
└── Makefile
```

### 编译以及运行
我们直接在该目录下使用 make 即可编译出 capture 可执行文件。
```
root@ubuntu:/app/cdev_demo/vio_capture# tree
.
├── capture
├── capture.c
├── capture.o
└── Makefile

```

### 执行效果

```

root@ubuntu:/app/cdev_demo/vio_capture# ./capture -b 16 -c 10 -h 1080 -w 1920
2000/01/01 08:32:50.675 !INFO [OpenCamera][0450]hbn module
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
2000/01/01 08:32:50.805 !INFO [CamInitParam][0326]Setting VSE channel-0: input_width:1920, input_height:1080, dst_w:1920, dst_h:1080
2000/01/01 08:32:50.805 !INFO [CamInitParam][0326]Setting VSE channel-1: input_width:1920, input_height:1080, dst_w:1920, dst_h:1080
2000/01/01 08:32:50.805 !INFO [vp_vin_init][0055]csi0 ignore mclk ex attr, because mclk is not configed at device tree.
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
capture time :0
temp_ptr.data_size[0]:4147200
capture time :1
temp_ptr.data_size[0]:4147200
capture time :2
temp_ptr.data_size[0]:4147200
capture time :3
temp_ptr.data_size[0]:4147200
capture time :4
temp_ptr.data_size[0]:4147200
capture time :5
temp_ptr.data_size[0]:4147200
capture time :6
temp_ptr.data_size[0]:4147200
capture time :7
temp_ptr.data_size[0]:4147200
capture time :8
temp_ptr.data_size[0]:4147200
capture time :9
temp_ptr.data_size[0]:4147200
root@ubuntu:/app/cdev_demo/vio_capture#
```

我们使用 ls -la 命令检查是否有抓到 raw 和 yuv 图像

```
root@ubuntu:/app/cdev_demo/vio_capture# ls -la
total 70964
drwxrwxr-x  2 root video    4096 Jan  1 08:07 .
drwxrwxr-x 10 root video    4096 Jul 31  2025 ..
-rwxr-xr-x  1 root video   14480 Jan  1  2000 capture
-rw-rw-r--  1 root video    4352 Jun  3  2025 capture.c
-rw-r--r--  1 root video    6296 Jan  1  2000 capture.o
-rw-rw-r--  1 root video     373 Jun  3  2025 Makefile
-rw-r--r--  1 root root  4147200 Jan  1 08:07 raw_0.raw
-rw-r--r--  1 root root  4147200 Jan  1 08:07 raw_1.raw
-rw-r--r--  1 root root  4147200 Jan  1 08:07 raw_2.raw
-rw-r--r--  1 root root  4147200 Jan  1 08:07 raw_3.raw
-rw-r--r--  1 root root  4147200 Jan  1 08:07 raw_4.raw
-rw-r--r--  1 root root  4147200 Jan  1 08:07 raw_5.raw
-rw-r--r--  1 root root  4147200 Jan  1 08:07 raw_6.raw
-rw-r--r--  1 root root  4147200 Jan  1 08:07 raw_7.raw
-rw-r--r--  1 root root  4147200 Jan  1 08:07 raw_8.raw
-rw-r--r--  1 root root  4147200 Jan  1 08:07 raw_9.raw
-rw-r--r--  1 root root  3110400 Jan  1 08:07 yuv_0.yuv
-rw-r--r--  1 root root  3110400 Jan  1 08:07 yuv_1.yuv
-rw-r--r--  1 root root  3110400 Jan  1 08:07 yuv_2.yuv
-rw-r--r--  1 root root  3110400 Jan  1 08:07 yuv_3.yuv
-rw-r--r--  1 root root  3110400 Jan  1 08:07 yuv_4.yuv
-rw-r--r--  1 root root  3110400 Jan  1 08:07 yuv_5.yuv
-rw-r--r--  1 root root  3110400 Jan  1 08:07 yuv_6.yuv
-rw-r--r--  1 root root  3110400 Jan  1 08:07 yuv_7.yuv
-rw-r--r--  1 root root  3110400 Jan  1 08:07 yuv_8.yuv
-rw-r--r--  1 root root  3110400 Jan  1 08:07 yuv_9.yuv
root@ubuntu:/app/cdev_demo/vio_capture# 
```
将产物下载到可以浏览 raw 和 yuv 的设备端，比如电脑主机上，然后通过软件预览 raw 和 yuv 图像。
raw 图一般比较暗， yuv 图像经过了 isp 处理，会更加还原现实效果。

## 详细是介绍

### 示例程序参数选项说明
```
root@ubuntu:/app/cdev_demo/vio_capture# ./capture 
Usage: capture [OPTION...]
capture sample -- An example of capture yuv/raw

  -b, --bit=bit              the depth of raw,mostly is 10,imx477 is 12
  -c, --count=number         capture number
  -h, --height=height        sensor output height
  -w, --width=width          sensor output width
  -?, --help                 Give this help list
      --usage                Give a short usage message

Mandatory or optional arguments to long options are also mandatory or optional
for any corresponding short options.
```

示例程序参数选项说明 \
--width	 -w	是摄像头 sensor 的输出宽度 \
--height -h	是摄像头 sensor 的输出高度 \
--bit	-b	是	RAW 位深（通常 8/10/16 ）\
--count	-c	是	捕获帧数

我们这里可以列举一些标配 sensor 的示例参数，方便参考使用。
| 型号   | 宽度 | 高度 | 位深 |
|--------|------|------|------|
| IMX219 | 1920 | 1080 | 16   |


### 软件架构说明
本示例主要是从摄像头捕获 YUV 和 RAW 图，逻辑相对简单，打开摄像头之后，调用 libspcdev 库提供的接口之后，可以直接获取到 YUV 和 RAW 图进行保存。

<center>
![software_arch](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/02_cdev_demo_sample/image/cdev_vio_capture_software_arch.png)
</center>


### API 流程说明

<center>
![software_arch](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/02_cdev_demo_sample/image/cdev_vio_capture_api_flow.png)
</center>



### FAQ
__Q：__ 使用其他摄像头为什么不能抓到图像 \
__A：__ 其他摄像头没有做过驱动适配， libspcdev 是调用了已经适配好的摄像头驱动去使用的，不同的摄像头参数不一样，所以其他摄像头不能抓到图像。

