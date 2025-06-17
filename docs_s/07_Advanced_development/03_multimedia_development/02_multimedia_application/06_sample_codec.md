# sample_codec 使用说明
## 功能概述
sample_codec 是一个用于编解码视频的示例程序。它可以根据配置文件（`codec_config.ini`）中定义的配置项进行视频编码和解码，帮助用户调试视频编解码器。

### 数据流说明
#### 编码数据流

![sample_codec_encode_data_flow](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/images_to_upload/sample_codec_encode_data_flow.png)

#### 解码数据流


![sample_codec_decode_data_flow](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/images_to_upload/sample_codec_decode_data_flow.png)

### 代码位置及目录结构

- 代码位置 `/app/multimedia_samples/sample_codec`
- 目录结构
```
sample_codec/
├── 1280x720_NV12.yuv
├── 1920x1080_NV12.yuv
├── 640x480_30fps.h264
├── codec_config.ini
├── Makefile
├── Readme.md
├── sample_codec.c
└── sample_codec.h
```

## 编译

- 进入 sample_codec 目录，执行 `make` 编译
- 输出成果物是 sample_codec 源码目录下的 `sample_codec`

**提示：** 在示例代码的目录下已经准备了 `1280x720_NV12.yuv`，`1920x1080_NV12.yuv` 和 `640x480_30fps.h264` 资源文件，您可以使用这几个文件快速运行 720P 、 1080P 分辨率的编码示例和 640 x 480 分辨率的 H264 解码示例。如果需要运行其他分辨率的编解码任务，请参考 `codec_config.ini` 的配置增加或者修改配置项，并且准备 input 文件。
## 运行

### 程序运行方法

执行程序 `./sample_codec -h` 可以获得帮助信息：

### 程序参数选项说明
```
./sample_codec -h
Usage: sample_codec -f config_file [-e encode_option] [-d decode_option] [-v] [-h]
Options:
  -f, --config_file FILE     Set the configuration file
  -e, --encode [OPTION]      Set the encoding option (optional), override encode_streams option
  -d, --decode [OPTION]      Set the decoding option (optional), override decode_streams option
  -v, --verbose              Enable verbose mode
  -h, --help                 Print this help message

Examples:
  Start codec video with codec_config.ini:
    sample_codec

  Start the specified encoding stream in codec_config.ini:
    sample_codec -e 0x1  -- Start the venc_stream1
    sample_codec -e 0x3  -- Start the venc_stream1 and venc_stream2

  Start the specified decoding stream in codec_config.ini:
    sample_codec -d 0x1  -- Start the vdec_stream1
    sample_codec -d 0x3  -- Start the vdec_stream1 and vdec_stream2

  Enable verbose mode for detailed logging:
    sample_codec -v

  Display this help message:
    sample_codec -h
```
**选项：**

- -f, --config_file FILE：指定配置文件的路径（可选），默认值为 `codec_config.ini`。
- -e, --encode [OPTION]：设置编码选项（可选）。如果使用此选项，则会覆盖配置文件中的 `encode_streams` 选项。
- -d, --decode [OPTION]：设置解码选项（可选）。如果使用此选项，则会覆盖配置文件中的 `decode_streams` 选项。
- -v, --verbose：启用详细模式，显示更多日志信息。
- -h, --help：显示帮助信息。

#### 使用示例

- 默认使用 `codec_config.ini` 中的配置项启动编解码（默认使能一路 H264 编码：`encode_streams = 0x1`），执行程序时不需要带任何参数：

```
./sample_codec
```

- 启动指定的编码流（在 `codec_config.ini` 中定义）：

```
sample_codec -e 0x1  # 启动 venc_stream1
sample_codec -e 0x3  # 启动 venc_stream1 和 venc_stream2
```

- 启动指定的解码流（在 `codec_config.ini` 中定义）：

```
sample_codec -e 0 -d 0x1 # 关闭编码，启动 vdec_stream1
sample_codec -e 0 -d 0x3  # 关闭编码，启动 vdec_stream1 和 vdec_stream2
```

- 启用详细模式以获取更多日志信息：

```
sample_codec -v
```

- 显示帮助信息：

```
sample_codec -h
```

### 配置文件说明

在 `codec_config.ini` 配置中定义了不同的视频编码和解码参数，设置默认启动的编解码通道数量。

其中编码参数选项说明如下：

```
[encode]
; 启用编码 , 按位运算
; 0x0 表示不启用编码
; 0x01 表示只启用 venc_stream1 编码流
; 0x02 表示只启用 venc_stream2 编码流
; 0x03 表示只启用前两路编码流（ venc_stream1 和 venc_stream2 ）, 0x07 表示启用前三路编码流 , 0x0f 表示启用前三路编码流，以此类推
encode_streams = 0x1

[venc_stream1]
; 编码类型（ 0 ： H264 ， 1 ： H265 ， 2 ： MJPEG， 3 ： JPEG）
codec_type = 0
width = 1920
height = 1080
frame_rate = 30
bit_rate = 8192
input = 1920x1080.yuv
output = 1920x1080_30fps.264
frame_num = 100
; profile, level, tier 配置
; h264 支持常见 baseline/main/high Profiles Level @ L5.2 ( 即最大 high@L5.2)
; h265 支持常见 main/main still picture profile @ L5.1 High Tier
; 详细可参考代码 ...
profile = h264_main@L4

[decode]
; 启用解码，按位运算
; 0x0 表示不启用解码
; 0x01 表示只启用 vdec_stream1 解码流
; 0x02 表示只启用 vdec_stream2 解码流
; 0x03 表示只启用前两路解码流（ vdec_stream1 和 vdec_stream2 ）, 0x07 表示启用前三路解码流 , 0x0f 表示启动前四路解码流，以此类推
decode_streams = 0x0

[vdec_stream1]
; 编码类型（ 0 ： H264 ， 1 ： H265 ， 2 ： MJPEG， 3 ： JPEG）
codec_type = 0
width = 1920
height = 1080
input = 1920x1080_30fps.264
output = 1920x1080.yuv
```

#### 编码配置

##### [encode]
- **encode_streams**：此选项用于指定要启用的编码流。采用按位运算的方式表示。例如，`0x1` 表示只启用 `venc_stream1` 编码流，`0x3` 表示启用前两路编码流（`venc_stream1` 和 `venc_stream2`）。**本选项的值会被命令行参数 -e 所覆盖**

##### [venc_stream]
- **codec_type**：指定编码的类型，可选值为 `0`（ H264 ）、`1`（ H265 ）、`2`（ MJPEG）和 `3`（ JPEG）。
- **width**：视频帧的宽度。
- **height**：视频帧的高度。
- **frame_rate**：视频的帧率。
- **bit_rate**：视频的比特率。
- **input**: 输入图像文件，仅支持 NV12 格式的 yuv 图像。一个文件中可以连续存放多帧图像，编码时会顺序、循环读取每一帧图像。
- **output**: 输出编码后的视频文件。
- **frame_num**: 要编码的视频帧数。如果输入图像文件中的图像帧数少于本参数的值，编码时会循环读取图像文件，直到达到或超过 frame_num 指定的帧数。
- **performance_test**: 指定是否运行性能测试的流程，与正常流程的区别是，性能测试流程会预先从文件中读取视频帧，存储到外部buffer中。
- **profile**： profile, level, tier 配置。
  - h264 支持 baseline/main/high Profiles Level @ L5.2 ( 即最大 high@L5.2)。
  - h265 支持 main/main still picture profile @ L5.1 High Tier。

#### 解码配置

##### [decode]
- **decode_streams**：此部分用于指定要启用的解码流。采用按位运算的方式表示。例如，`0x1` 表示只启用 `vdec_stream1` 解码流，`0x3` 表示启用前两路解码流（`vdec_stream1` 和 `vdec_stream2`）。

##### [vdec_stream]
- **codec_type**：指定解码的视频编码类型，可选值为 `0`（ H264 ）、`1`（ H265 ）、`2`（ MJPEG）和 `3`（ JPEG）。
- **width**：解码后视频帧的宽度。
- **height**：解码后视频帧的高度。
- **input**：输入待解码的视频文件路径，根据 `codec_type` 的设置，可以使用码流文件和 rtsp 码流。
- **output**：输出解码后的图像文件路径，仅支持输出 NV12 格式的 yuv 图像。解码后的图像会连续保存到一个文件中，因此请确保输出路径有足够的磁盘空间。请注意， YUV 图像通常占用较大的磁盘空间，特别是对于高分辨率和长时长的视频文件，可能会占用大量存储空间。在选择输出路径时，请确保目标存储设备有足够的可用空间。

### 运行效果

以配置文件中使能（ encode_streams = 0x1 ）的第一路 H264 编码为例。


```
./sample_codec
Config file: codec_config.ini
encode_streams: 0x1
decode_streams: 0x0
Encoding video...
Encode params...
 codec_type: 0, width: 1920, height: 1080, frame_rate: 30, bit_rate: 8192, input_file: 1920x1080_NV12.yuv, output_file: 1920x1080_30fps.h264, frame_num: 100
Encode idx: 0, init successful
Encode idx: 0, start successful
Encode idx: 0, frame= 1
Encode idx: 0, frame= 2
Encode idx: 0, frame= 3
... ...
Encode idx: 0, frame= 98
Encode idx: 0, frame= 99
Encode idx: 0, frame= 100
```

根据 `codec_config.ini` 中的配置 `frame_num = 100` ，编码 100 帧后程序自动退出。
