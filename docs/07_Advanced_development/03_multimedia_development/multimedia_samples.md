---
sidebar_position: 2
---

# 7.3.2 示例程序
本章简述D-Robotics 多媒体应用示例，所涉及源码可以通过 sudo apt install hobot-multimedia-samples 获得，安装在 /app/multimedia_samples 目录下。
## get_sif_data 使用说明{#get_sif_data}

### 程序功能

下图所示为X3M的视频数据通路框图，其中的专业名词解释请查看 [多媒体开发概述-术语约定](./overview#terminology)。

![image-20220517184132422](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/03_multimedia_development/multimedia_samples/image-20220517184132422.png)

`get_sif_data` 完成 `sensor` 、`MIPI CSI` 和 `SIF` 模块的初始化，实现从`SIF`模块获取视频帧数据的功能，支持从`SIF`模块获取`Raw`、`YUV`两种格式的图像。

`get_sif_data` 可以有效帮助用户调试`sensor`和`X3M`的点亮调试，在打通`sensor -> SIF`的数据通路后，再调试其他模块的功能。

### 程序开发

#### 源码结构

源码位于 `/app/multimedia_samples/get_sif_data`

```
.
├── main.c                       # 主程序，完成sensor列表的加载，和命令控制
├── Makefile			 # 编译makefile
├── module.c
├── module.h
├── Readme.md
├── sensor_handle.c              # sensor 初始化、从sif中获取图像的接口
├── sensor_handle.h
├── sensors			 # sensor参数配置，每个新sensor在本目录新增一个文件
│   ├── sensor_f37.c
│   └── sensor_imx415.c
└── sensors.lds
```

#### 编译

当前代码通过一个Makefile文件配置编译

进入源码目录，执行以下命令进行编译生成`get_sif_data`程序


```bash
$ cd sample/get_sif_data
$ make clean # 清理源码，保持干净的代码环境
$ make
... ... # 一大段编译打印
$ ls
get_sif_data  main.c  main.o  Makefile  module.c  module.h  module.o  Readme.md  sensor_handle.c  sensor_handle.h  sensor_handle.o  sensors  sensors.lds
```

#### 添加新sensor

如果有新sensor需要调试，请参考 sensors 目录下的源码文件，对应添加一个新的sensor配置即可。

以F37为例说明关键代码：

```c
/* 
 * 添加sensor、mipi、sif dev、isp的参数配置
 * 各结构体中参数在代码中有已经有比较详细的注释说明
 * 其中isp部分参数在本程序中无需关注
 */
static int set_sensor_param(void)
{
        printf("set_sensor_param\n");
        /*定义 sensor   初始化的属性信息 */
        snsinfo = SENSOR_1LANE_F37_30FPS_10BIT_LINEAR_INFO;
        /*定义 mipi 初始化参数信息 */
        mipi_attr = MIPI_1LANE_SENSOR_F37_30FPS_10BIT_LINEAR_ATTR;
        /*定义 dev 初始化的属性信息 */
        devinfo = DEV_ATTR_F37_LINEAR_BASE;
        /*定义 pipe 属性信息 */
        pipeinfo = PIPE_ATTR_F37_LINEAR_BASE;

        return sensor_sif_dev_init();
        return 0;
}

/* 
 * 主程序遍历sensor模块时调用本函数完成sensor名和sensor参数配置接口的注册
 */
static int sensor_probe(void)
{
        int i = 0;

        /* 在sensor_lists里面找到一个空位置 */
        for (i = 0; i < ARRAY_SIZE(sensor_lists); i++) {
                if (0 == strlen(sensor_lists[i].sensor_tag)) break;
        }

        if (i >= ARRAY_SIZE(sensor_lists)) {
                printf("sensor lists is full\n");
                return -1;
        }

        strncpy(sensor_lists[i].sensor_tag, SENSOR_TAG, 31 > strlen(SENSOR_TAG) ? strlen(SENSOR_TAG) : 31);
        sensor_lists[i].func = set_sensor_param;
        return 0;
}

/* 注册sensor的模块入口，主程序在遍历sensor时会用到 */
SENSOR_MODULE_INSTALL(sensor_probe);
```


### 功能使用

#### 硬件连接

RDK X3 开发板通过`mipi host`接口用于连接`Sensor`模组，请根据当前要调试的`Sensor`模组型号正确连接。

#### 程序部署

按照上面的编译流程生成出`get_sif_data`后，执行该程序，根据提示选择当前连接在开发板上的sensor类别，比如当前连接的是 `F37 sensor`，则选择 1。 

如果初始化成功，会自动获取第一帧图像（pipe0_plane0_1920x1080_frame_001.raw）保存在程序运行的目录下（退出程序后执行 ls -l pipe0_plane0_1920x1080_frame_* 可以查看），并打印用户可以使用的命令，运行过程如下：

```bash
chmod +x get_sif_data
./get_sif_data

D-Robotics Sensor Test Tools V1.0

********************** Sensor Lists *************************
        0 -- IMX415
        1 -- F37
*************************************************************

Please select :1 # 选择 sensor
... ... # 一大段初始化日志
normal pipe_id (0)type(9)frame_id(1)buf_index(0)w x h(1920x1080) data_type 9 img_format 0
stride_size(2400) w x h1920 x 1080  size 2592000
pipe(0)dump normal raw frame id(1),plane(1)size(2592000) # 获取第一帧图像
filedump(pipe0_plane0_1920x1080_frame_001.raw, size(2592000) is successed
time cost 85 ms 
dumpToFile raw cost time 85 ms********************** Command Lists *************************
  q     -- quit
  g     -- get one frame
  l     -- get a set frames
  h     -- print help message

Command: 
```

**命令解释：**

- g： 获取一帧图像，支持输入多个`g`来连续获取图像，例如输入 `gggg`


```bash
Command: g
normal pipe_id (0)type(9)frame_id(4078)buf_index(5)w x h(1920x1080) data_type 9 img_format 0
stride_size(2400) w x h1920 x 1080  size 2592000
pipe(0)dump normal raw frame id(4078),plane(1)size(2592000)
filedump(pipe0_plane0_1920x1080_frame_4078.raw, size(2592000) is successed
time cost 67 ms 
dumpToFile raw cost time 67 ms
```

- l： 连续获取12帧图像，相当于输入12个 `g`


```bash
Command: l
normal pipe_id (0)type(9)frame_id(4588)buf_index(3)w x h(1920x1080) data_type 9 img_format 0
stride_size(2400) w x h1920 x 1080  size 2592000
pipe(0)dump normal raw frame id(4588),plane(1)size(2592000)
filedump(pipe0_plane0_1920x1080_frame_4588.raw, size(2592000) is successed
time cost 56 ms 
... ... # 连续的获取帧数据的打印
dumpToFile raw cost time 56 msnormal pipe_id (0)type(9)frame_id(4609)buf_index(7)w x h(1920x1080) data_type 9 img_format 0
stride_size(2400) w x h1920 x 1080  size 2592000
pipe(0)dump normal raw frame id(4609),plane(1)size(2592000)
filedump(pipe0_plane0_1920x1080_frame_4609.raw, size(2592000) is successed
time cost 57 ms 
dumpToFile raw cost time 57 ms
```

- q: 退出程序


```
Command: Command: q
quit
[  256.825912] [S0][V1]sif_video_streamoff
[  256.826439] SIF close node 1
[  256.853045] [S0][V0]sif_video_streamoff SIF last process stream off 
[  256.853922] [S0][V0]sif_video_streamoff
[  256.855476] hobot_dmcfreq_target: dmcfreq->rate:2666000000, target_rate:2666000000
[  256.856460] buf:performance
[  256.856460] , powersave_rate:2666000000, dmcfreq->pre_state:0
[  256.857610] [S0][V0]x3_sif_close SIF last process close 
[  256.858301] SIF close node 0
[  256.858807] [isp_drv]: camera_sys_stream_off: camera_sys_stream_off success line 1549 dev_name port_0
[  256.860006] [isp_drv:cam]: camera_fop_release: line 115 port 0 user_num 0  camera_cdev->start_num 0 
[  256.861229] vps mipi_host1: sensor1_mclk set(1) 0 as 24000000
[  256.861980] vps mipi_host1: sensor1_mclk set(0) 0 as 24000000
[  256.862741] vps mipi_host0: sensor0_mclk set(2) 0 as 24000000
[  256.863491] vps mipi_host0: sensor0_mclk set(1) 0 as 24000000
[  256.864241] vps mipi_host0: sensor0_mclk set(0) 0 as 24000000
```

#### 运行效果说明

执行程序后会获取到如 `pipe0_plane0_1920x1080_frame_4609.raw` 一样命名的`raw`图像，或者如`pipe0_1920x1080_frame_1024.yuv`一样命名的`yuv`图像。

请使用 [hobotplayer](https://archive.d-robotics.cc/downloads/hobotplayer/hobotplayerv.2.07.1.rar) 工具浏览图像，图像的参数配置说明如下：

- 浏览RAW图

按照如下图所示步骤配置选项，其中`file config`里面关注`pic_type`、`raw_type`、 `pix_length` 、`width`和`height`的配置，F37 配置为（PIC_RAW、MIPI_RAW、RAW_BIT_10， 1920，1080），IMX415配置为（PIC_RAW、MIPI_RAW、RAW_BIT_12， 3840，2160）

![image-20220517211101610](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/03_multimedia_development/multimedia_samples/image-20220517211101610.png)

- 浏览YUV图

按照如下图所示步骤配置选项，其中`file config`里面关注`pic_type`、`yuv_type`、`width`和`height`的配置，F37 配置为（PIC_YUV、YUV_NV12， 1920，1080），IMX415配置为（YUV_NV12， 3840，2160）

![image-20220517212105959](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/03_multimedia_development/multimedia_samples/image-20220517212105959.png)

## get_isp_data 使用说明{#get_isp_data}

### 程序功能

下图所示为X3M的视频数据通路框图，其中的专业名词解释请查看 [多媒体开发概述-术语约定](./overview#terminology)。

![image-20220517184132422](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/03_multimedia_development/multimedia_samples/image-20220517184132422.png)

`get_isp_data` 完成 `sensor` 、`MIPI CSI`  `SIF` 和 `ISP` 模块的初始化，实现从`ISP`模块获取视频帧数据的功能，支持从`ISP`模块获取`YUV`格式的图像。

`get_isp_data` 可以有效帮助用户调试`sensor`和`X3M`的ISP效果调试，在打通`sensor -> SIF -> ISP `的数据通路后，再调试其他模块的功能。

### 程序开发

#### 源码结构

源码位于 `/app/multimedia_samples/get_isp_data`

```
.
├── main.c                       # 主程序，完成sensor列表的加载，和命令控制
├── Makefile			 # 编译makefile
├── module.c
├── module.h
├── Readme.md
├── sensor_handle.c              # sensor 初始化、从isp中获取图像的接口
├── sensor_handle.h
├── sensors			 # sensor参数配置，每个新sensor在本目录新增一个文件
│   ├── sensor_f37.c
│   └── sensor_imx415.c
└── sensors.lds
```

#### 编译

当前代码通过一个Makefile文件配置编译，进入源码目录，执行以下命令进行编译生成`get_isp_data`程序：

```bash
$ cd /app/multimedia_samples/get_sif_data
$ make clean # 清理源码，保持干净的代码环境
$ make
... ... # 一大段编译打印
$ ls
get_isp_data  main.c  main.o  Makefile  module.c  module.h  module.o  Readme.md  sensor_handle.c  sensor_handle.h  sensor_handle.o  sensors  sensors.lds
```

#### 添加新sensor

如果有新sensor需要调试，请参考 sensors 目录下的源码文件，对应添加一个新的sensor配置即可。

以F37为例说明关键代码：

```c
/* 
 * 添加sensor、mipi、sif dev、isp的参数配置
 * 各结构体中参数在代码中有已经有比较详细的注释说明
 */
static int set_sensor_param(void)
{
        printf("set_sensor_param\n");
        /*定义 sensor   初始化的属性信息 */
        snsinfo = SENSOR_1LANE_F37_30FPS_10BIT_LINEAR_INFO;
        /*定义 mipi 初始化参数信息 */
        mipi_attr = MIPI_1LANE_SENSOR_F37_30FPS_10BIT_LINEAR_ATTR;
        /*定义 dev 初始化的属性信息 */
        devinfo = DEV_ATTR_F37_LINEAR_BASE;
        /*定义 pipe 属性信息 */
        pipeinfo = PIPE_ATTR_F37_LINEAR_BASE;
    	/*定义 dis 属性信息 */
    	disinfo = DIS_ATTR_F37_BASE;
    	/*定义 ldc 属性信息 */
    	ldcinfo = LDC_ATTR_F37_BASE;
        return sensor_sif_dev_init();
        return 0;
}

/* 
 * 主程序遍历sensor模块时调用本函数完成sensor名和sensor参数配置接口的注册
 */
static int sensor_probe(void)
{
        int i = 0;

        /* 在sensor_lists里面找到一个空位置 */
        for (i = 0; i < ARRAY_SIZE(sensor_lists); i++) {
                if (0 == strlen(sensor_lists[i].sensor_tag)) break;
        }

        if (i >= ARRAY_SIZE(sensor_lists)) {
                printf("sensor lists is full\n");
                return -1;
        }

        strncpy(sensor_lists[i].sensor_tag, SENSOR_TAG, 31 > strlen(SENSOR_TAG) ? strlen(SENSOR_TAG) : 31);
        sensor_lists[i].func = set_sensor_param;
        return 0;
}

/* 注册sensor的模块入口，主程序在遍历sensor时会用到 */
SENSOR_MODULE_INSTALL(sensor_probe);
```


### 功能使用

#### 硬件连接

RDK X3 开发板通过`mipi host`接口用于连接`Sensor`模组，请根据当前要调试的`Sensor`模组型号正确连接。

#### 程序部署

按照上面的编译流程生成出`get_isp_data`后，执行该程序，根据提示选择当前连接在开发板上的sensor类别，比如当前连接的是 `F37 sensor`，则选择 1。  

如果初始化成功，会自动获取第一帧图像（pipe0_1920x1080_frame_001.yuv）保存在程序运行的目录下（退出程序后执行 ls -l pipe0_1920x1080_frame_* 可以查看），并打印用户可以使用的命令，运行过程如下：

```bash
chmod +x get_isp_data
./get_isp_data

D-Robotics Sensor Test Tools V1.0

********************** Sensor Lists *************************
        0 -- IMX415
        1 -- F37
*************************************************************

Please select :1 # 选择 sensor
... ... # 一大段初始化日志
normal pipe_id (0)type(11)frame_id(1)buf_index(0)w x h(1920x1080) data_type 11 img_format 0
stride_size(2400) w x h1920 x 1080  size 2073600
pipe(0)dump normal yuv frame id(1),plane(1)size(2073600) # 获取第一帧图像
filedump(pipe0_1920x1080_frame_001.yuv, size(2073600) is successed
time cost 63 ms 
dumpToFile yuv cost time 63 ms********************** Command Lists *************************
  q     -- quit
  g     -- get one frame
  l     -- get a set frames
  h     -- print help message

Command: 
```

**命令解释：**

- g： 获取一帧图像，支持输入多个`g`来连续获取图像，例如输入 `gggg`


```bash
Command: g
normal pipe_id (0)type(11)frame_id(4078)buf_index(5)w x h(1920x1080) data_type 11 img_format 0
stride_size(2400) w x h1920 x 1080  size 2073600
pipe(0)dump normal yuv frame id(4078),plane(1)size(2073600)
filedump(pipe0_1920x1080_frame_4078.yuv, size(2073600) is successed
time cost 63 ms 
dumpToFile yuv cost time 63 ms
```

- l： 连续获取12帧图像，相当于输入12个 `g`


```bash
Command: l
normal pipe_id (0)type(11)frame_id(4588)buf_index(3)w x h(1920x1080) data_type 11 img_format 0
stride_size(2400) w x h1920 x 1080  size 2073600
pipe(0)dump normal yuv frame id(4588),plane(1)size(2073600)
filedump(pipe0_1920x1080_frame_4588.yuv, size(2073600) is successed
time cost 56 ms 
... ... # 连续的获取帧数据的打印
dumpToFile yuv cost time 56 msnormal pipe_id (0)type(11)frame_id(4609)buf_index(7)w x h(1920x1080) data_type 11 img_format 0
stride_size(2400) w x h1920 x 1080  size 2073600
pipe(0)dump normal yuv frame id(4609),plane(1)size(2073600)
filedump(pipe0_1920x1080_frame_4609.yuv, size(2073600) is successed
time cost 57 ms 
dumpToFile yuv cost time 57 ms
```

- q: 退出程序


```
Command: Command: q
quit
[  256.825912] [S0][V1]sif_video_streamoff
[  256.826439] SIF close node 1
[  256.853045] [S0][V0]sif_video_streamoff SIF last process stream off 
[  256.853922] [S0][V0]sif_video_streamoff
[  256.855476] hobot_dmcfreq_target: dmcfreq->rate:2666000000, target_rate:2666000000
[  256.856460] buf:performance
[  256.856460] , powersave_rate:2666000000, dmcfreq->pre_state:0
[  256.857610] [S0][V0]x3_sif_close SIF last process close 
[  256.858301] SIF close node 0
[  256.858807] [isp_drv]: camera_sys_stream_off: camera_sys_stream_off success line 1549 dev_name port_0
[  256.860006] [isp_drv:cam]: camera_fop_release: line 115 port 0 user_num 0  camera_cdev->start_num 0 
[  256.861229] vps mipi_host1: sensor1_mclk set(1) 0 as 24000000
[  256.861980] vps mipi_host1: sensor1_mclk set(0) 0 as 24000000
[  256.862741] vps mipi_host0: sensor0_mclk set(2) 0 as 24000000
[  256.863491] vps mipi_host0: sensor0_mclk set(1) 0 as 24000000
[  256.864241] vps mipi_host0: sensor0_mclk set(0) 0 as 24000000
```

#### 运行效果说明

执行程序后会获取到如 `pipe0_1920x1080_frame_4609.yuv` 一样命名的`yuv`图像

请使用 [hobotplayer](https://archive.d-robotics.cc/downloads/hobotplayer/hobotplayerv.2.07.1.rar) 工具浏览图像，图像的参数配置说明如下：

- 浏览YUV图

按照如下图所示步骤配置选项，其中`file config`里面关注`pic_type`、`yuv_type`、`width`和`height`的配置，F37 配置为（PIC_YUV、YUV_NV12， 1920，1080），IMX415配置为（YUV_NV12， 3840，2160）

![image-20220517212105959](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/03_multimedia_development/multimedia_samples/image-20220517212105959.png)

## sample_isp 使用说明{#sample_isp}

### 程序功能

`sample_isp`程序完成`isp`图像接口初始化，主要功能是调用每一个`isp`图像进行接口动态设置/获取参数，并返回测试结果

### 程序开发

#### 源码结构

源码位于`/app/multimedia_samples/sample_isp`
```
.
├── main.c			# 主程序
├── Makefile			# 编译makefile
└── Readme.md			# 程序说明
```

#### 编译

当前代码通过一个Makefile文件配置编译

进入源码目录，执行以下命令进行编译生成 `sample_isp`
```
$ cd /app/multimedia_samples/sample_isp
$ make clean # 清理源码，保持干净的代码环境
$ make
... ... # 一大段编译打印
$ ls
main.c  main.o  Makefile  sample_isp
```


### 功能使用

#### 程序部署

按照上面的编译流程生成出`sample_isp`后，运行该程序。

注意此程序运行前需要有当前的sensor程序在运行中，sensor的程序可以直接使用`Sunrise_camera`, `sample_isp`运行过程如下

```bash
chmod +x sample_isp
# ./sample_isp
============================================
APP: ./sample_isp
a: AE
b: AF
c: AWB
d: BL
e: DEMOSAIC
f: SHARPEN
g: GAMMA
h: IRIDIX
i: CNR
j: SINTER
k: TEMPER
l: SCENE_MODES
m: FIRMWARE STATE
n: MODULE CONTROL
o: REGISTER
p: LIBREG_AE
q: LIBREG_AWB
r: LIBREG_AF
s: METERING AE(read only)
t: METERING AWB(read only)
u: METERING AF(read only)
v: METERING AE_5BIN(read only)
w: METERING_DATA_TIME(read only)
x: SWITCH SCENCE
A: CSC
B: MESH SHADING
C: MESH SHADING LUT
D: RADIAL SHADING
E: RADIAL SHADING LUT
F: IRIDIX STRENGTH LEVEL
G: IDX_IRQ_SYNC
H: IDX_AWB_ZONE
I: IDX_AF_ZONE
L: IDX_AF_KERNEL
M: IDX_AEROI_INFO
N: IDX_LUMA_INFO
O: IDX_AEPARAM_INFO
J: IDX_AE5BIN_ZONE
K: IDX_AE_ZONE
P: IDX_AE_EX
y: Help
Q: Exit
============================================
ISP_TEST>
```
#### 命令解释：

- a： 获取/设置AE属性

- b： 示例中暂不支持

- c:  获取/设置AWB属性

- d:  获取/设置BlackLevel属性

- e:  获取/设置Demosaic属性

- f:  获取/设置SHARPEN属性

- g:  获取/设置GAMMA属性

- h:  获取/设置IRIDIX属性

- i:  获取/设置CNR属性

- j:  获取/设置SINTER属性

- k:  获取/设置TEMPER属性

- l:  获取/设置SCENE_MODES属性

- m:  获取/设置FWSTATE属性

- n:  获取/设置ModuleControl属性

- o:  获取/设置Register寄存器

- p:  注册AE回调接口

- q:  注册AWB回调接口

- r:  注册AF回调接口

- s:  获取AE统计信息

- t:  获取AWB统计信息

- u:  获取AF统计信息

- v:  获取AE_5BIN统计信息

- w:  获取最新的统计信息(代码里面默认获取AWB，可以仿照代码通过传参获取AE,AF统计信息)

- x:  切换isp效果库(so库需要跟isp_test文件同一目录)

- A:  获取/设置CSC属性

- B:  获取/设置MESH_SHADING属性

- C:  获取/设置MESH SHADING LUT属性

- D:  获取/设置RADIAL SHADING属性

- E:  获取/设置RADIAL SHADING LUT属性

- F:  获取/设置IRIDIX STRENGTH LEVEL属性

- G:  获取帧同步开始/结束时间

- H:  设置AWB_ZONE属性

- I:  设置AF_ZONE属性

- L:  获取/设置AF_KERNEL_INFO属性

- M:  获取/设置AEROI信息

- N:  获取LUMA信息

- O:  获取/设置AEParam信息

- J:  设置AE5BIN_ZONE属性

- K:  设置AEZONE属性

- P:  获取/设置AE额外属性

- y:  帮助信息

- Q:  退出程序


## sample_vps 使用说明{#sample_vps}

### 程序功能

`sample_vps` 程序使用一个 `vps grp` 的多个不同通道，对 `YUV` 图像进行裁剪，旋转，缩放等操作，展示 `vps` 的基本用法。更多丰富的 `vps` 图像处理使用请参考[视频处理](./video_processing)章节。

### 程序开发

#### 源码结构

源码位于：`/app/multimedia_samples/sample_vps`

```
.
|-- 19201080.yuv      # 回灌使用NV12格式文件 
|-- main.c            # 主程序
`-- Makefile          # 编译makefile
```

#### 编译

当前代码通过一个Makefile文件配置编译

进入源码目录，执行以下命令进行编译生成`sample_vps`

```shell
$ cd /app/multimedia_samples/sample_vps
$ make clean # 清理源码，保持干净的代码环境
$ make
... ... # 一大段编译打印
$ ls
19201080.yuv  main.c  main.o  Makefile  sample_vps
```
### 功能使用

#### 程序部署

按照上面的编译流程生成出 `sample_vps` ，确保当前目录下存在`19201080.yuv`，然后执行程序 `./sample_vps`

#### 运行效果说明
`YUV` 图片通过回灌方式，利用 `vps` 进行裁剪，旋转，缩放等功能，保存对应处理后的 `YUV` 图像。
* `grp_0_chn_1_out_1280_720.yuv` 为原图裁剪到 `1280x720` 分辨率；
* `grp_0_chn_2_out_1088_1920.yuv` 为原图旋转90度的图像；
* `grp_0_chn_3_out_960_540.yuv` 为原图缩小到 `960x540` 分辨率；
* `grp_0_chn_5_out_2880_1620.yuv` 为原图放大到 `2880x1620` 分辨率；

## sample_vps_zoom 使用说明{#sample_vps_zoom}

### 程序功能

`sample_vps_zoom` 程序使用 `vps` 的硬件模块 `ipu` 和 `pym` 对 `YUV` 图像中的部分区域做多倍放大处理，对处理后的 `YUV`图像编码成 `H264` 视频流，可以直接使用 `MPC-BE` 等工具进行预览，类似电子云台中的`zoom` 功能。整个程序的 `Pipeline` 如下图所示：

![Pipeline](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/03_multimedia_development/multimedia_samples/vps_zoom_pipeline.png)

如 `Pipeline` 所示，程序通过 `vps0` 读取 `YUV` 图像，`vps0 chn1` 和 `vps1` 绑定，通过 `ipu` 和 `pym` 做 `crop` 及放大后，将数据送给 `venc` 的 `chn1` 做 `H264` 编码，形成 `zoom` 放大的效果，同时 `vps0 chn2` 和 `venc chn0` 绑定做 `H264` 编码， `vps0 chn3` 和 `venc chn2` 绑定做 `H264` 编码。

### 程序开发

#### 源码结构

源码位于：`/app/multimedia_samples/sample_vps_zoom`

```
.
|-- 19201080.yuv      # 回灌使用NV12格式文件 
|-- main.c            # 主程序
`-- Makefile          # 编译makefile
```

#### 编译

当前代码通过一个Makefile文件配置编译

进入源码目录，执行以下命令进行编译生成`sample_vps_zoom`程序：

```shell
$ cd /app/multimedia_samples/sample_vps_zoom
$ make clean # 清理源码，保持干净的代码环境
$ make
... ... # 一大段编译打印
$ ls
19201080.yuv  main.c  main.o  Makefile  sample_vps_zoom
```

### 功能使用

#### 程序部署

按照上面的编译流程生成出 `sample_vps_zoom`，确保当前目录下存在 `19201080.yuv` 文件  

执行程序 `./sample_vps_zoom`

#### 运行效果说明

`YUV` 图片通过回灌方式，利用 `ipu`, `pym`, `venc` 模块编码成平滑放大的zoom `H264` 码流。效果如下所示。

![vps_1_chn_5_venc_0.h264_20230523_143448](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/03_multimedia_development/multimedia_samples/vps_1_chn_5_venc_0.h264_20230523_143448.gif)

## sample_osd 使用说明{#sample_osd}

### 程序功能

`sample_osd` 程序用于给 `vps` 通道输出的 `YUV` 数据叠加时间戳，汉语文字 `osd`。更多丰富的 `osd` 图像处理使用请参考[区域处理](./region_processing)章节。

### 程序开发

#### 源码结构

源码位于：`/app/multimedia_samples/sample_osd`

```
.
|-- 1280720.yuv       # 回灌使用NV12格式文件 
|-- main.c            # 主程序
|-- Makefile          # 编译makefile
```

#### 编译

当前代码通过一个Makefile文件配置编译

进入源码目录，执行以下命令进行编译生成`sample_osd`程序

```shell
$ cd sample/sample_osd
$ make clean # 清理源码，保持干净的代码环境
$ make
... ... # 一大段编译打印
$ ls
1280720.yuv  main.c  main.o  Makefile  sample_osd
```
### 功能使用

#### 程序部署

按照上面的编译流程生成出`sample_osd`后，确保当前目录下面存在`1280720.yuv`，执行`sample_osd`

#v## 运行效果说明

通过 `osd` 叠加后 `vps` 通道输出的的 `YUV` 图像如下图所示：

![Osd](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/03_multimedia_development/multimedia_samples/image-20220517151700.png)

## sample video codec 使用说明{#sample_video_codec}

### 程序功能

`sample_vdec_basic` 实现最基础解码功能，读取本地`H264`/`H265`/`JPEG`文件，进行解码保存`NV12`结果

`sample_venc_basic` 实现最基础编码功能，读取`NV12`图像，编码为`H264`（或`H265`或`JPEG`），并保存为本地文件

`sample_vdec_two_channel` 面向需要多通道同时解码的场景，在`sample_vdec_basic` 基础上增加一路解码通道，实现双通道解码功能。读取本地`H264`/`H265`/`JPEG`文件，两路同时进行解码分别保存`NV12`文件。

`sample_venc_two_channel` 面向需要多通道同时编码的场景，在`sample_venc_basic` 基础上增加一路编码通道，实现双通道编码功能。读取本地`NV12`文件，两路同时进行解码分别保存`H264`（或`H265`或`JPEG`）。

### 程序开发

#### 源码结构

源码位于：`/app/multimedia_samples/sample_video_codec`

```
.
├── example_vdec_basic
├── example_vdec_two_channel
├── example_venc_basic
├── example_venc_two_channel
├── Makefile
├── README.md
├── sample_vdec_basic.c
├── sample_vdec_two_channel.c
├── sample_venc_basic.c
└── sample_venc_two_channel.c
```

#### 编译

当前代码通过一个Makefile文件配置编译

进入源码目录，执行以下命令进行编译生成`sample_venc_basic`、`sample_vdec_basic`、`sample_vdec_two_channel`、`sample_venc_two_channel`程序

```shell
$ cd /app/multimedia_samples/sample_video_codec
$ make clean # 清理源码，保持干净的代码环境
$ make
... ... # 一大段编译打印
$ ls
example_vdec_basic        example_venc_basic        Makefile   sample_vdec_basic    sample_vdec_two_channel    sample_venc_basic    sample_venc_two_channel
example_vdec_two_channel  example_venc_two_channel  README.md  sample_vdec_basic.c  sample_vdec_two_channel.c  sample_venc_basic.c  sample_venc_two_channel.c
```

### 程序部署

#### sample_vdec_basic

按照上面的编译流程生成出`sample_vdec_basic`程序之后  

执行 `./sample_vdec_basic -w width -h height -t ecode_type -f file`

其中width为图像宽所包含像素个数

height为为图像高所包含的像素格式

encode_type可以为h264\h265\jpeg

file为要解码的文件名

#### sample_venc_basic

按照上面的编译流程生成出`sample_venc_basic`程序之后  

执行 `./sample_venc_basic -w width -h height -t ecode_type -f file0 -g file1`

其中width为图像宽所包含像素个数

height为为图像高所包含的像素格式

ecode_type可以为h264\h265\jpeg

file0为要编码的文件名需要为NV12格式

file1为要编码的文件名需要为NV12格式，其width和height需要和file0保持一样

#### sample_vdec_two_channel

按照上面的编译流程生成出`sample_vdec_two_channel`程序之后  


执行  `./sample_vdec_two_channel -w width -h height -t ecode_type -f file`

其中width为图像宽所包含像素个数

height为为图像高所包含的像素格式

encode_type可以为h264\h265\jpeg

file为要解码的文件名

#### sample_venc_two_channel

按照上面的编译流程生成出`sample_venc_two_channel`程序之后  

执行   `./sample_venc_two_channel -w width -h height -t ecode_type -f file0 -g file1`

其中width为图像宽所包含像素个数

height为为图像高所包含的像素格式

ecode_type可以为h264\h265\jpeg

file0为要编码的文件名需要为NV12格式

file1为要编码的文件名需要为NV12格式，其width和height需要和file0保持一样

### 运行效果说明

#### sample_vdec_basic

```bash
root@ubuntu:/app/multimedia_samples/sample_video_codec# ./sample_vdec_basic -w 1920 -h 1080 -t h264 -f 1080P.h264
mmzAlloc paddr = 0x1769a000, vaddr = 0x7fa49ac000 i = 0
mmzAlloc paddr = 0x17b8d000, vaddr = 0x7fa44b9000 i = 1
mmzAlloc paddr = 0x18080000, vaddr = 0x7fa42be000 i = 2
mmzAlloc paddr = 0x1827b000, vaddr = 0x7fa40c3000 i = 3
mmzAlloc paddr = 0x18634000, vaddr = 0x7f97790000 i = 4
try open
do while
[pstStream] pts:0, vir_ptr:548222451712, size:71225
feed raw data
do while
[pstStream] pts:1, vir_ptr:548217262080, size:29880
feed raw data
do while
vdec 1, 1920x1080
[pstStream] pts:2, vir_ptr:548215185408, size:20136
feed raw data

......
......
......


do while
[pstStream] pts:146, vir_ptr:548064448512, size:14653
feed raw data
do while
[pstStream] pts:147, vir_ptr:548062371840, size:14555
feed raw data
do while
[pstStream] pts:148, vir_ptr:548057182208, size:15637
feed raw data
do while
[pstStream] pts:149, vir_ptr:548051447808, size:15966
feed raw data
do while
There is no more input data, 0!
mmzFree paddr = 0x1769a000, vaddr = 0x7fa016d000 i = 0
mmzFree paddr = 0x17b8d000, vaddr = 0x7f9b2fd000 i = 1
mmzFree paddr = 0x18080000, vaddr = 0x7f9b102000 i = 2
mmzFree paddr = 0x1827b000, vaddr = 0x7f9ac0f000 i = 3
mmzFree paddr = 0x18634000, vaddr = 0x7f9a697000 i = 4
[ERROR]["vdec"][video/src/hb_vdec.c:743] [540.920523]HB_VDEC_GetFrame[743]: [HB_VDEC] HB_VDEC_GetFrame:743 Failed  VdChn = 0 s32Ret = -269024268

HB_VDEC_GetFrame failed:-269024268
vp exit ok!
Done
The program exited normally. If you encounter a Get_Frame error, it may be because the file has already been read. Please check.


```
输出 log 显示不断在将逐帧处理，最后的 LOG 打印也显示程序正常退出 ， 末尾附近出现的 `HB_VDEC_GetFrame Failed` 代表已经读不到输入的数据了，说明可能到了文件末尾。

该程序执行完成之后，在当前运行目录会生成 decode.nv12 文件，该文件内容随着解码内容更新。

#### sample_venc_basic

```bash
root@ubuntu:/app/multimedia_samples/sample_video_codec# ./sample_venc_basic -w 1920 -h 1080 -t h265 -f 1080P_file1.nv12  -g 1080P_file2.nv12
feed encode data thread running
 m_VencChnAttr.stRcAttr.enRcMode = 5 mmmmmmmmmmmmmmmmmm
 u32VbvBufferSize = 10 mmmmmmmmmmmmmmmmmm
buf:y: vaddr = 0x7f983e5000 paddr = 0x173da000; uv: vaddr = 0x7f982e7000, paddr = 0x178cd000
get encode data thread running
buf:y: vaddr = 0x7f915dc000 paddr = 0x18fdd000; uv: vaddr = 0x7f980e8000, paddr = 0x191d8000


^C[ERROR]["multimedia"][src/vdi/linux/vdi_osal.c:174] [ERROR][7838.68423][7015:7021][TASK] MCTaskDequeueOutputBufferLocked The component(stream_reader) has been terminated!
[ERROR]["venc"][video/src/hb_venc.c:982] [7838.689759]HB_VENC_GetStream[982]: [HB_VENC] HB_VENC_GetStream:982 Failed  VeChn = 0 s32Ret = -268958720

HB_VENC_GetStream failed. ret=-268958720
Done
root@ubuntu:/app/multimedia_samples/sample_video_codec#

```
上述效果我们可以看到使用命令 `./sample_venc_basic -w 1920 -h 1080 -t h265 -f 1080P_file1.nv12  -g 1080P_file2.nv12` 将 1080P_file1.nv12 和 1080P_file2.nv12 的图像编码成 h265 文件， 按下 `ctrl+c` 之后停止运行，中止循环编码过程，`HB_VENC_GetStream` 会获取不到数据，抛出应有的 LOG 打印。

程序结束之后，在当前运行目录下生成sample_venc.h264/sample_venc.h265/sample_venc.jpg。H264/H265文件内容为交替显示是file1和file2
#### sample_vdec_two_channel

```
root@ubuntu:/app/multimedia_samples/sample_video_codec# ./sample_vdec_two_channel -w 1920 -h 1080 -t h264 -f 1920x1080.h264
mmzAlloc paddr = 0x173a2000, vaddr = 0x7f935f5000 i = 0
mmzAlloc paddr = 0x17895000, vaddr = 0x7f93102000 i = 0
mmzAlloc paddr = 0x18080000, vaddr = 0x7f92c0f000 i = 1
mmzAlloc paddr = 0x18573000, vaddr = 0x7f92424000 i = 1
mmzAlloc paddr = 0x18d5e000, vaddr = 0x7f91c39000 i = 2
mmzAlloc paddr = 0x19117000, vaddr = 0x7f916c1000 i = 2
mmzAlloc paddr = 0x19312000, vaddr = 0x7f914c6000 i = 3
mmzAlloc paddr = 0x1950d000, vaddr = 0x7f912cb000 i = 3
mmzAlloc paddr = 0x19852000, vaddr = 0x7f910d0000 i = 4
mmzAlloc paddr = 0x19c0b000, vaddr = 0x7f90d17000 i = 4
try open
try open
[pstStream] pts:0, vir_ptr:547928154112, size:40
feed raw data
[pstStream] pts:0, vir_ptr:547933343744, size:40
feed raw data
[pstStream] pts:1, vir_ptr:547914661888, size:110529
feed raw data
[pstStream] pts:1, vir_ptr:547922964480, size:110529
feed raw data
[pstStream] pts:2, vir_ptr:547900624896, size:54178
feed raw data
[pstStream] pts:2, vir_ptr:547906359296, size:54178
feed raw data
[pstStream] pts:3, vir_ptr:547896471552, size:9596
feed raw data
[pstStream] pts:3, vir_ptr:547898548224, size:9596
......
......
......
feed raw data
[pstStream] pts:29, vir_ptr:547890491392, size:2444
feed raw data
[pstStream] pts:29, vir_ptr:547894394880, size:2444
feed raw data
[pstStream] pts:30, vir_ptr:547928154112, size:33999
feed raw data
[pstStream] pts:30, vir_ptr:547933343744, size:33999
feed raw data
[pstStream] pts:31, vir_ptr:547914661888, size:8696
feed raw data
[pstStream] pts:31, vir_ptr:547922964480, size:8696
feed raw data
[pstStream] pts:32, vir_ptr:547900624896, size:3833
feed raw data
[pstStream] pts:32, vir_ptr:547906359296, size:3833
feed raw data
[pstStream] pts:33, vir_ptr:547896471552, size:1214
feed raw data
[pstStream] pts:33, vir_ptr:547898548224, size:1214
feed raw data
^CmmzFree paddr = 0x17895000, vaddr = 0x7f93102000 i = 0
mmzFree paddr = 0x18573000, vaddr = 0x7f92424000 i = 1
mmzFree paddr = 0x173a2000, vaddr = 0x7f935f5000 i = 0
mmzFree paddr = 0x19117000, vaddr = 0x7f916c1000 i = 2
mmzFree paddr = 0x18080000, vaddr = 0x7f92c0f000 i = 1
mmzFree paddr = 0x1950d000, vaddr = 0x7f912cb000 i = 3
mmzFree paddr = 0x18d5e000, vaddr = 0x7f91c39000 i = 2
mmzFree paddr = 0x19c0b000, vaddr = 0x7f90d17000 i = 4
mmzFree paddr = 0x19312000, vaddr = 0x7f914c6000 i = 3
mmzFree paddr = 0x19852000, vaddr = 0x7f910d0000 i = 4
[ERROR]["vdec"][video/src/hb_vdec.c:743] [1636.753595]HB_VDEC_GetFrame[743]: [HB_VDEC] HB_VDEC_GetFrame:743 Failed  VdChn = 0 s32Ret = -269024268

HB_VDEC_GetFrame failed:-269024268
[ERROR]["vdec"][video/src/hb_vdec.c:743] [1636.756613]HB_VDEC_GetFrame[743]: [HB_VDEC] HB_VDEC_GetFrame:743 Failed  VdChn = 1 s32Ret = -269024268

HB_VDEC_GetFrame failed:-269024268
vp exit ok!
Done
root@ubuntu:/app/multimedia_samples/sample_video_codec#

```
在 h264 文件符合要求的情况下，我们执行类似 `./sample_vdec_two_channel -w 1920 -h 1080 -t h264 -f 1920x1080.h264` 这样的命令，可以看到 LOG 显示，一直在解码，程序会通过两个解码通道分别执行，在按下 `ctrl + c` 之后，程序结束，并且会在当前运行目录下生成sample_decode_ch0.nv12和sample_decode_ch1.nv12，该文件内容随着解码内容更新。

#### sample_venc_two_channel

```

root@ubuntu:/app/multimedia_samples/sample_video_codec# ./sample_venc_two_channel -w 1920 -h 1080 -t h264 -f 1080P_file1.nv12 -g 1080P_file2.nv12
feed encode data thread running
 m_VencChnAttr.stRcAttr.enRcMode = 5 mmmmmmmmmmmmmmmmmm
 u32VbvBufferSize = 10 mmmmmmmmmmmmmmmmmm
get encode data thread running
buf:y: vaddr = 0x7f90005000 paddr = 0x176d2000; uv: vaddr = 0x7f8a3da000, paddr = 0x17bc5000
feed encode data thread running
 m_VencChnAttr.stRcAttr.enRcMode = 5 mmmmmmmmmmmmmmmmmm
 u32VbvBufferSize = 10 mmmmmmmmmmmmmmmmmm
buf:y: vaddr = 0x7f88087000 paddr = 0x1830b000; uv: vaddr = 0x7f7b6f2000, paddr = 0x18af6000
get encode data thread running
buf:y: vaddr = 0x7f78758000 paddr = 0x1ac18000; uv: vaddr = 0x7f7845f000, paddr = 0x1b00e000
buf:y: vaddr = 0x7f7855d000 paddr = 0x1ae13000; uv: vaddr = 0x7f78361000, paddr = 0x1b10c000


^C[ERROR]["multimedia"][src/vdi/linux/vdi_osal.c:174] [ERROR][2770.31880][5508:5520][TASK] MCTaskDequeueOutputBufferLocked The component(stream_reader) has been terminated!
[ERROR]["multimedia"][src/vdi/linux/vdi_osal.c:174] [ERROR][2770.31897][5508:5514][TASK] MCTaskDequeueOutputBufferLocked The component(stream_reader) has been terminated!
[ERROR]["venc"][video/src/hb_venc.c:982] [2770.325412]HB_VENC_GetStream[982]: [HB_VENC] HB_VENC_GetStream:982 Failed  VeChn = 0 s32Ret = -268958720

HB_VENC_GetStream failed. ret=-268958720
[ERROR]["venc"][video/src/hb_venc.c:982] [2770.327741]HB_VENC_GetStream[982]: [HB_VENC] HB_VENC_GetStream:982 Failed  VeChn = 1 s32Ret = -268958720

HB_VENC_GetStream failed. ret=-268958720
Done
root@ubuntu:/app/multimedia_samples/sample_video_codec#

```
上述效果我们可以看到使用命令 `./sample_venc_two_channel -w 1920 -h 1080 -t h264 -f 1080P_file1.nv12 -g 1080P_file2.nv12` 将 1080P_file1.nv12 和 1080P_file2.nv12 编码成 h265 文件，按下 `ctrl + c` 会结束编码，结束期间，会出现 `HB_VENC_GetStream` 的打印。\
程序结束后会在当前运行目录下生成 sample_venc_ch0.h264（sample_venc_ch0.h265/sample_venc_ch0.jpg） 和 sample_venc_ch1.h264（sample_venc_ch1.h265/sample_venc_ch1.jpg）两个通道的文件。H264/H265文件内容为交替显示是file1和file2。

## sample_vot 使用说明{#sample_vot}

### 程序功能

`sample_vot程序`完成`VOT`模块的初始化，实现从当前目录读取一帧nv12的图片数据送到`VOT`的bt1120输出显示功能

### 程序开发

#### 源码结构

源码位于：`/app/multimedia_samples/sample_vot`

```
.
├── 1280_720yuv8.yuv			# 回灌使用720P的NV12格式文件
├── 1920_1080yuv8.yuv			# 回灌使用1080P的NV12格式文件
├── Makefile					# 编译makefile
├── Readme.md					# 程序说明
└── vot.c						# 主程序
```

#### 编译

当前代码通过一个Makefile文件配置编译

进入源码目录，执行以下命令进行编译生成`sample_vot`

```shell
cd /app/multimedia_samples/sample_vot
$ make clean # 清理源码，保持干净的代码环境
$ make
... ... # 一大段编译打印
$ ls
1280_720yuv8.yuv  1920_1080yuv8.yuv  Makefile  Readme.md  sample_vot  vot.c  vot.o
```


### 功能使用

#### 程序部署

按照上面的编译流程生成出`sample_vot`

执行程序 `./sample_vot 1080P30`。

```bash
chmod +x sample_vot
root@x3sdbx3-samsung2G-3200:/userdata# ./sample_vot 1080P60
[   26.051955] channel id is 0, enable is 0, reg value is 0x4ef00f.
[   26.052744] channel id is 1, enable is 0, reg value is 0x4ef00f.
[   26.053520] channel id is 2, enable is 0, reg value is 0x4ef00f.
[   26.054339] channel id is 3, enable is 0, reg value is 0x4ef00f.
stLayer width:1920[   26.055263] channel id is 0, enable is 1, reg value is 0x14ef00f.
stLayer height:1080
libiar: hb_disp_set_timing done!
stChnAttr priority :2
stChnAttr src width :1920
stChnAttr src height :1080
stChnAttr s32X :0
stChnAttr s32Y :0
stChnAttr u32DstWidth :1920
stChnAttr u32DstHeight :1080
[   26.056165] iar_output_stream.
stCrop width :1920
stCrop height :1080
[   26.059304] channel id is 0, enable is 1, reg value is 0x14ef00f.
framesize:3110400

（注意如果是X3 SDB生态板子，那么带的参数只支持1080P60/1080P30，如果是客户自己使用的sil902x的bt1120转换hdmi芯片，那么参数可以是如下：
	1080P60
	1080P59.94
	1080P50
	1080P30
	1080P29.97
	1080P25
	1080I60
	1080I59.94
	1080I50
	720P60
	720P59.94
	720P50
	720P29.97）


```

#### 运行效果说明

程序通过把`1920_1080yuv8.yuv`读到内存，并通过接口把数据送到`VOT`模块的`bt1120`接口,然后通过`hdmi`转换芯片输出`hdmi`效果到显示设备如下图

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/03_multimedia_development/multimedia_samples/20220520-163716.jpg)

## sample_lcd 使用说明{#sample_lcd}

### 程序功能

`sample_lcd程序`完成`VOT`模块的初始化，实现从当前目录读取一帧NV12的图片数据送到`VOT`的`midi-dsi`输出到lcd屏幕显示

### 程序开发

#### 源码结构

源码位于：`/app/multimedia_samples/sample_lcd`

```
.
├── 720x1280.yuv	# 回灌使用NV12格式文件
├── Makefile		# 编译makefile
├── Readme.md		# 程序说明
└── vot.c			# 主程序
```

#### 编译

当前代码通过一个Makefile文件配置编译

进入源码目录，执行以下命令进行编译生成`sample_lcd`程序

```shell
$ cd /app/multimedia_samples/sample_lcd
$ make clean # 清理源码，保持干净的代码环境
$ make
... ... # 一大段编译打印
$ ls
720x1280.yuv  vot.c  vot.o  Makefile  sample_lcd
```
### 功能使用

#### 程序部署

按照上面的编译流程生成出`sample_lcd`，确保当前目录下存在`720x1280.yuv`文件

执行程序 `./sample_lcd`。

```bash
chmod +x sample_lcd
# ./sample_lcd
root@x3sdbx3-samsung2G-3200:/userdata# ./sample_lcd 
libiar: hb_disp_set_timing done!
HB_VOT_SetChnAttr 0: 0
HB_VOT_EnableChn: 0
HB_VOT_EnableChn: 0
framesize:1382400
```

#### 运行效果说明

程序通过把`720x1280.yuv`读到内存，并通过接口把数据送到`VOT`模块的`midi-dsi`接口,然后显示到`lcd`屏幕设备如下图

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/03_multimedia_development/multimedia_samples/20220520-161120.jpg)
