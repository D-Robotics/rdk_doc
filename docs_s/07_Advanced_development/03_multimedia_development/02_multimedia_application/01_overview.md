# 示例代码介绍

本章主要介绍芯片上的多媒体模块的示例代码，主要包含多媒体硬件加速模块的使用示例。包含单模块功能示例、多模块组合串联示例和应用解决方案示例，用户可以使用这些示例快速完成功能评测，参考其中的实现代码可帮助用户快速上手并开发适合自身需求的应用。

通过使用本章节提供的示例，用户将能够：

- 理解每个硬件加速模块的基本功能和适用场景。
- 学习如何高效地将多个模块组合，解决实际问题。
- 快速掌握示例代码的使用方法，从而节省开发时间。

## 示例源码目录结构

所有示例代码的源码存放在设备的`/app/multimedia_samples` 目录下。

源码目录结构如下，按照功能和场景进行命名区分。

```
.
├── chip_base_test    # 驱动功能单元测试，请查阅 [ BSP 开发指南 - 驱动功能单元测试 ] 了解使用方法
├── Makefile.in       # Makefile 的参数配置，包括交叉编译工具链，头文件、库文件引用等
├── README.md
├── sample_codec      # 视频图像编、解码模块的示例代码，包括 H264\H265\JPEG\MJPEG 的编码和解码
├── sample_gdc        # GDC 模块支持的各种转换模式的示例代码
├── sample_isp        # ISP 模块的示例代码，包括如何初始化 ISP，获取 ISP 处理后的数据等
├── sample_pipeline   # 串联多功能模块的示例代码，例如： VIN->ISP->PYM->GDC->CODEC 编码的数据流通路功能测试
├── sample_vin        # 初始化 Camera Sensor，从 VIN 模块获取图像
├── sample_pym        # 初始化 PYM 模块，使用 PYM 对图像进行缩小操作
├── sample_gpu_3d     # 提供OpenCL 和 OpenGLES 两种接口使用 3DGPU的示例
├── sunrise_camera    # 集成大部分模块实现的应用方案示例代码，支持智能摄像头和智能分析盒，在 Web 上可以浏览视频图像和智能算法结果
├── utils             # 包含通用的函数及结构体
└── vp_sensors        # Camera Sensor 配置代码，本代码会被其他需要使用 Camera Sensor 的模块使用
```

注：`vp_sensors` 目录包含已支持的 `Camera Sensor` 和配置文件，不是单独的示例程序。`Camera Sensor` 添加方式请查阅参考 `vp_sensors/README.md`。

## 示例使用指南

所有示例程序、依赖的资源文件，默认安装在开发板的 `/app/multimedia_samples` 目录下，用户可以直接登录开发板使用这些示例。例如想要使用 sample_pym 示例，可以按照以下命令使用：

```
#1. 进入源码目录
cd /app/multimedia_samples/sample_pym

#2. 编译程序
make

#3. 执行 sample_pym 获取程序的帮助信息

./sample_pym
Usage: sample_pym [OPTIONS]
Options:
-i, --input_file FILE   Specify the input file
-w, --input_width WIDTH Specify the input width
-h, --input_height HEIGHT       Specify the input height
-f, --feedback                  Specify feedback mode
-V, --verbose           Enable verbose mode
```

其他示例都可以按照这种方式使用。

## 示例开发指南

### 编译方法

本模块的示例代码的依赖如下:
1. 依赖 BSP 源码编译生成的 hbre 头文件、库文件，以及根文件系统中的 ffmpeg 等头文件和动态库, 相关库和路径的引用在各个 Sample 目录下的Makfile中指定
```
LDFLAGS += -lvpf -lvio -lcam -lhbmem
LDFLAGS := -L/usr/hobot/lib
INCDIR := -I/usr/hobot/include -I/usr/include/cjson/
```

2. 依赖 `/app/multimedia_samples/vp_sensors` 目录中的 Sensor 配置文件，相关文件的引用在 `/app/multimedia_samples/Makefile.in` 中指定，配置如下：

```
INCS += -I . \
	-I ${PLATFORM_SAMPLES_DIR}/utils/ \
	-I ${PLATFORM_SAMPLES_DIR}/vp_sensors

SRC_PATH := ${PLATFORM_SAMPLES_DIR}/utils
SRC_PATH += $(shell find ${PLATFORM_SAMPLES_DIR}/vp_sensors -maxdepth 1 -type d)

```

- **整体编译所有示例**

	目前不支持整体编译

- **单个示例编译方法**

1. 登录到设备中
2. 进入 `/app/multimedia_samples/` 目录中各个各示例， 单独进行开发与编译。例如，单独编译 `sample_vin` 示例：

	```
	cd /app/multimedia_samples/sample_vin/get_vin_data
	make
	```
3. 编译生成的可执行程序，在当前目录下，可以直接执行, 如下：`get_vin_data` 是编译得到可执行程序
	```
	root@ubuntu:/app/multimedia_samples/sample_vin/get_vin_data# ls
	get_vin_data  get_vin_data.c  get_vin_data.o  Makefile
	```

## 常见问题

### /app 目录下空间不足导致示例运行失败

**问题原因：** /app 目录是挂载的 app 分区，我们有些示例程序在运行时需要保存大量的视频图像数据，可能把空间占满，让程序运行的结果不符合预期。

**解决方法：** 可以把程序拷贝到 其他目录执行，或清理不用的数据

### 串口终端登陆的情况下内核日志刷屏

在板端执行以下命令降低 Kernel 的日志输出级别，避免输出日志太多，影响查看程序的执行流程：

```
echo 4 > /proc/sys/kernel/printk
```
