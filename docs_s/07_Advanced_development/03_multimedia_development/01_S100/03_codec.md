---
sidebar_position: 3
---
# 编解码

## 系统概述

### 概述

Codec（Coder-Decoder）是指编解码器，用于压缩或解压缩视频、图像、音频等媒体数据；S100
Soc中存在两种硬件编解码单元，分别是VPU（Video process unit）和JPU（Jpeg process
unit），可提供4K\@90fps的视频编解码能力和4K\@90fps的图像编解码能力。

#### JPU硬件特性

| **HW Feature** | **Feature Indicator**                        |
| -------------------- | -------------------------------------------------- |
| HW number            | 1                                                  |
| maximum input        | 8192x8192                                          |
| minimum input        | 32x32                                              |
| performance          | 4K\@90fps                                          |
| max instance         | 64                                                 |
| input image format   | 4:0:0, 4:2:0, 4:2:2, 4:4:0, and 4:4:4 color format |
| output image format  | 4:0:0, 4:2:0, 4:2:2, 4:4:0, and 4:4:4 color format |
| input crop           | Supports                                           |
| bitrate control      | FIXQP(MJPEG)                                       |
| rotation             | 90, 180, 270                                       |
| mirror               | Vertical, Horizontal, Vertical+Horizontal          |
| quantization table   | Supports Custom Settings                           |
| huffman table        | Supports Custom Settings                           |

#### VPU硬件特性

| **HW Feature**           | **Feature Indicator**                                                                                                                                                                                                                                                                                                                                                                                                     |
| ------------------------------ | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| HW number                      | 1                                                                                                                                                                                                                                                                                                                                                                                                                               |
| maximum input                  | 8192x4096                                                                                                                                                                                                                                                                                                                                                                                                                       |
| minimum input                  | 256x128                                                                                                                                                                                                                                                                                                                                                                                                                         |
| input alignment required       | width 32, height 8                                                                                                                                                                                                                                                                                                                                                                                                              |
| performance                    | 4K\@90fps                                                                                                                                                                                                                                                                                                                                                                                                                       |
| max instance                   | 32                                                                                                                                                                                                                                                                                                                                                                                                                              |
| input image format             | 4:2:0, 4:2:2 color format                                                                                                                                                                                                                                                                                                                                                                                                       |
| output image format            | 4:2:0, 4:2:2 color format                                                                                                                                                                                                                                                                                                                                                                                                       |
| input crop                     | Supports                                                                                                                                                                                                                                                                                                                                                                                                                        |
| bitrate control                | CBR, VBR, AVBR, FIXQP, QPMAP                                                                                                                                                                                                                                                                                                                                                                                                    |
| rotation                       | 90, 180, 270                                                                                                                                                                                                                                                                                                                                                                                                                    |
| mirror                         | Vertical, Horizontal, Vertical+Horizontal                                                                                                                                                                                                                                                                                                                                                                                       |
| long-term reference prediction | Supports Custom Settings                                                                                                                                                                                                                                                                                                                                                                                                        |
| intra refresh                  | Supports                                                                                                                                                                                                                                                                                                                                                                                                                        |
| deblocking filter              | Supports                                                                                                                                                                                                                                                                                                                                                                                                                        |
| request IDR                    | Supports                                                                                                                                                                                                                                                                                                                                                                                                                        |
| ROI mode                       | mode1: Users can set multiple zones’(up to 64) qp value(0-51), should not work with CBR or AVBR mode mode2: Users can set multiple zones’(up to 64) important level(0-8), should work with CBR or AVBR mode                                                                                                                                                                                                                   |
| GOP mode                       | 0: Custom GOP 1 : I-I-I-I,..I (all intra, gop_size=1) 2 : I-P-P-P,… P (consecutive P, gop_size=1) 3 : I-B-B-B,…B (consecutive B, gop_size=1) 4 : I-B-P-B-P,… (gop_size=2) 5 : I-B-B-B-P,… (gop_size=4) 6 : I-P-P-P-P,… (consecutive P, gop_size=4) 7 : I-B-B-B-B,… (consecutive B, gop_size=4) 8 : I-B-B-B-B-B-B-B-B,… (random access, gop_size=8) 9 : I-P-P-P,… P (consecutive P, gop_size = 1, with single reference) |

### 软件功能

#### 整体框架

MediaCodec子系统会提供音视频和图像的编解码组件，原始流封装和视频录像等功能。该系统主要会封装底层codec硬件资源和软件编解码库，为上层提供编解码能力。开发者可以基于提供的编解码接口实现H265和H264视频的编解码功能，也可以使用JPEG编码功能将摄像头数据存成JPEG图片，还可以使用视频录像功能实现摄像头数据的录制。

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/03_multimedia_development/02_S100/codec/2f8364ee5efbb8cb14136e0dc942248e.png)

#### 码率控制模式

MediaCodec支持对H264/H265和MJPEG协议的码率控制，分别支持H264/H265编码通道的CBR、VBR、AVBR、FixQp和QpMap五种码率控制方式，以及支持MJPGE编码通道的FixQp码率控制方式。

##### CBR说明

CBR表示恒定码率，能够保证整体的编码码率稳定。下面是CBR模式下各个参数含义：

| **数据项**    | **描述**                                                                                                                                                                                                                                                 | **取值范围** | **默认值** |
| ------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ------------------ | ---------------- |
| intra_period        | I帧间隔                                                                                                                                                                                                                                                        | [0,2047]           | 28               |
| intra_qp            | I帧的QP值                                                                                                                                                                                                                                                      | [0,51]             | 0                |
| bit_rate            | 目标平均比特率，单位是kbps                                                                                                                                                                                                                                     | [0,700000]         | 0                |
| frame_rate          | 目标帧率，单位是fps                                                                                                                                                                                                                                            | [1,240]            | 30               |
| initial_rc_qp       | 指定码率控制时的初始QP值，当该值不在[0,51]范围内,编码器内部会决定初始值                                                                                                                                                                                        | [0,63]             | 63               |
| vbv_buffer_size     | 指定VBV Buffer的大小，单位是ms；实际的VBV buffer的空间大小为bit_rate\*vbv_buffer_size/1000(kb)，该buffer的大小会影响编码图像质量和码率控制精度。当该buffer比较小时，码率控制精确度高，但图像编码质量较差；当该buffer比较大时，图像编码质量高，但是码率波动大。 | [10,3000]          | 10               |
| ctu_level_rc_enalbe | H264/H265的码率控制可以工作在CTU级别的控制，该模式可以达到更高精度的码率控制，但是会损失编码图像质量，该模式不可以和ROI编码一起工作，当使能ROI编码时，该功能自动失效。                                                                                         | [0,1]              | 0                |
| min_qp_I            | I帧的最小QP值                                                                                                                                                                                                                                                  | [0,51]             | 8                |
| max_qp_I            | I帧的最大QP值                                                                                                                                                                                                                                                  | [0,51]             | 51               |
| min_qp_P            | P帧的最小QP值                                                                                                                                                                                                                                                  | [0,51]             | 8                |
| max_qp_P            | P帧的最大QP值                                                                                                                                                                                                                                                  | [0,51]             | 51               |
| min_qp_B            | B帧的最小QP值                                                                                                                                                                                                                                                  | [0,51]             | 8                |
| max_qp_B            | B帧的最大QP值                                                                                                                                                                                                                                                  | [0,51]             | 51               |
| hvs_qp_enable       | H264/H265的码率控制可以工作在subCTU级别的控制，该模式会调整子宏块的QP值，进而提高主观图像质量。                                                                                                                                                                | [0,1]              | 1                |
| hvs_qp_scale        | 当hvs_qp_enable使能后有效，该值表示QP缩放因子。                                                                                                                                                                                                                | [0,4]              | 2                |
| max_delta_qp        | 当hvs_qp_enable使能后有效，指定HVS qp值的最大偏差范围。                                                                                                                                                                                                        | [0,51]             | 10               |
| qp_map_enable       | 使能ROI编码时的QP map                                                                                                                                                                                                                                          | [0,1]              | 0                |

##### VBR说明

VBR表示可变码率，简单场景分配比较大的qp，压缩率小，质量高。复杂场景分配较小qp，可以保证编码图像的质量稳定。下面是VBR模式下各个参数含义：

| **数据项** | **描述**        | **取值范围** | **默认值** |
| ---------------- | --------------------- | ------------------ | ---------------- |
| intra_period     | I帧间隔               | [0,2047]           | 28               |
| intra_qp         | I帧的QP值             | [0,51]             | 0                |
| frame_rate       | 目标帧率，单位是fps   | [1,240]            | 0                |
| qp_map_enable    | 使能ROI编码时的QP map | 0,1                | 0                |

##### AVBR说明

ABR表示恒定平均目标码率，简单场景分配较低码率，复杂场景分配足够码率，使得有限的码率能够在不同场景下合理分配，这类似VBR。同时一定时间内，平均码率又接近设置的目标码率，这样可以控制输出文件的大小，这又类似CBR。可以认为是CBR和VBR的折中方案，产生码率和图像质量相对稳定的码流。下面是AVBR模式下各个参数含义：

| **数据项**    | **描述**                                                                                                                                                                                                                                                 | **取值范围** | **默认值** |
| ------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ------------------ | ---------------- |
| intra_period        | I帧间隔                                                                                                                                                                                                                                                        | [0,2047]           | 28               |
| intra_qp            | I帧的QP值                                                                                                                                                                                                                                                      | [0,51]             | 0                |
| bit_rate            | 目标平均比特率，单位是kbps                                                                                                                                                                                                                                     | [0,700000]         | 0                |
| frame_rate          | 目标帧率，单位是fps                                                                                                                                                                                                                                            | [1,240]            | 30               |
| initial_rc_qp       | 指定码率控制时的初始QP值，当该值不在[0,51]范围内,编码器内部会决定初始值                                                                                                                                                                                        | [0,63]             | 63               |
| vbv_buffer_size     | 指定VBVBuffer的大小，单位是ms；实际的VBVbuffer的空间大小为bit_rate\*vbv_buffer_size/1000（kb），该buffer的大小会影响编码图像质量和码率控制精度。当该buffer比较小时，码率控制精确度高，但图像编码质量较差；当该buffer比较大时，图像编码质量高，但是码率波动大。 | [10,3000]          | 3000             |
| ctu_level_rc_enalbe | H264/H265的码率控制可以工作在CTU级别的控制，该模式可以达到更高精度的码率控制，但是会损失编码图像质量，该模式不可以和ROI编码一起工作，当使能ROI编码时，该功能自动失效。                                                                                         | [0,1]              | 0                |
| min_qp_I            | I帧的最小QP值                                                                                                                                                                                                                                                  | [0,51]             | 8                |
| max_qp_I            | I帧的最大QP值                                                                                                                                                                                                                                                  | [0,51]             | 51               |
| min_qp_P            | P帧的最小QP值                                                                                                                                                                                                                                                  | [0,51]             | 8                |
| max_qp_P            | P帧的最大QP值                                                                                                                                                                                                                                                  | [0,51]             | 51               |
| min_qp_B            | B帧的最小QP值                                                                                                                                                                                                                                                  | [0,51]             | 8                |
| max_qp_B            | B帧的最大QP值                                                                                                                                                                                                                                                  | [0,51]             | 51               |
| hvs_qp_enable       | H264/H265的码率控制可以工作在subCTU级别的控制，该模式会调整子宏块的QP值，进而提高主观图像质量。                                                                                                                                                                | [0,1]              | 1                |
| hvs_qp_scale        | 当hvs_qp_enable使能后有效，该值表示QP缩放因子。                                                                                                                                                                                                                | [0,4]              | 2                |
| max_delta_qp        | 当hvs_qp_enable使能后有效，指定HVSqp值的最大偏差范围。                                                                                                                                                                                                         | [0,51]             | 10               |
| qp_map_enable       | 使能ROI编码时的QPmap                                                                                                                                                                                                                                           | [0,1]              | 0                |

##### FixQp说明

FixQp表示固定每一个I帧、P帧的QP值，对于I/P帧可以分别设值。下面是FixQp模式下各个参数含义：

| **数据项** | **描述**      | **取值范围** | **默认值** |
| ---------------- | ------------------- | ------------------ | ---------------- |
| intra_period     | I帧间隔             | [0,2047]           | 28               |
| frame_rate       | 目标帧率，单位是fps | [1,240]            | 30               |
| force_qp_I       | 强制I帧的QP值       | [0,51]             | 0                |
| force_qp_P       | 强制P帧的QP值       | [0,51]             | 0                |
| force_qp_B       | 强制B帧的QP值       | [0,51]             | 0                |

##### QPMAP说明

QPMAP表示为一帧图像中的每一个块指定QP值，其中H265块大小为32x32,H264块大小为16x16。下面是QPMAP模式下各个参数含义：

| **数据项**   | **描述**                                                                                                       | **取值范围**                                                                 | **默认值** |
| ------------------ | -------------------------------------------------------------------------------------------------------------------- | ---------------------------------------------------------------------------------- | ---------------- |
| intra_period       | I帧间隔                                                                                                              | [0,2047]                                                                           | 28               |
| frame_rate         | 目标帧率，单位是fps                                                                                                  | [1,240]                                                                            | 30               |
| qp_map_array       | 指定QPmap表，H265的subCTU大小为32x32，需要为每一个subCTU指定一个QP值，每个QP值占一个字节，并且按照光栅扫描方向排序。 | 指针地址                                                                           | NULL             |
| qp_map_array_count | 指定QPmap表的大小。                                                                                                  | [0, MC_VIDEO_MAX_SUB_CTU_NUM]&&(ALIGN64(picWidth)\>\>5)\*(ALIGN64(picHeight)\>\>5) | 0                |

### 调试方法

#### 编码效果调优

根据当前客户使用codec进行视频编码的场景，多将码率模式设置为CBR，当编码的场景较为复杂时，为了保证视频质量，硬件会自动提高码率值，导致输出的视频较预期更大。因此为了兼顾视频质量和实际码率，需要统筹bit_rate和max_qp_I/P值的设置。下面给出了全I帧模式下，不同复杂场景下，码率设置为15000kbps时，不同max_qp_I下实际码率和qp的情况（不同场景复杂程度不同，下列数据仅供参考）：

- 只有I帧和B帧；
- B帧参考1个前向参考帧，一个后向参考帧；

GOP Preset 9

- 只有I帧和P帧；
- P帧参考1个前向参考帧；
- 低延时；

| 场景&参数                              | 室外白天复杂场景 bitrate(15000) max_qp_I(35) | 室外白天复杂场景 bitrate(15000) max_qp_I(38) | 室外白天复杂场景 bitrate(15000) max_qp_I(39) |
| -------------------------------------- | -------------------------------------------- | -------------------------------------------- | -------------------------------------------- |
| Bit alloction(bps)（越大图像质量越高） | 60300045                                     | 42186920                                     | 35898230                                     |
| Qp avg（越小图像质量越高）             | 35                                           | 38                                           | 39                                           |

#### GOP结构说明

H264/H265编码支持GOP结构的设置，用户可从预置的3种GOP结构种选择，也可自定义GOP结构。

GOP结构表可定义一组周期性的GOP结构，该GOP结构将用于整个编码过程。单个结构表中的元素如下表所示，其中可以指定该图像的参考帧，如果IDR帧后的其他帧指定的参考帧为IDR帧前的数据帧，编码器内部会自动处理这种情况使其不参考其他帧，用户无需关心这种情况。用户在自定义GOP结构时需要指明结构表的数量，最多可定义3个结构表，结构表的顺序需要按照解码顺序排列。
下面表示了结构表中各个元素的含义：

| 元素           | 描述                                                                                                                                                                                                 |
| -------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Type           | 帧类型(I、P、B)                                                                                                                                                                                      |
| POC            | GOP内帧的显示顺序，取值范围为[1,gop_size]。                                                                                                                                                          |
| QPoffset       | 自定义GOP中图片的量化参数                                                                                                                                                                            |
| NUM_REF_PIC_L0 | 标记为P帧使用多参考图片，仅在PIC_TYPE为P时有效。                                                                                                                                                     |
| temporal_id    | 帧的时间层，帧无法从具有较高时间 id（0\~6）的帧进行预测。                                                                                                                                            |
| 1st_ref_POC    | L0的第一张参考图片的POC                                                                                                                                                                              |
| 2nd_ref_POC    | Type为B时，第一张参考图片的POC是L1的； Type为P时，第二张参考图片的POC是L0的； 可以使reference_L1和B slice中的参考图片具有相同的POC， 但出于压缩效率的考虑，建议reference_L1和reference_L0的POC不同。 |

##### GOP预置结构

一共支持设置九种GOP预置结构

| 序号 | GOP结构   | 低延迟（编码顺序和显示顺序相同） | GOP大小 | 编码顺序                    | 最小源帧buffer数量 | 最小解码图片buffer数量 | 周期内（I 帧间隔）要求 |
| ---- | --------- | -------------------------------- | ------- | --------------------------- | ------------------ | ---------------------- | ---------------------- |
| 1    | I         | Yes                              | 1       | I0-I1-I2…                  | 1                  | 1                      |                        |
| 2    | P         | Yes                              | 1       | P0-P1-P2…                  | 1                  | 2                      |                        |
| 3    | B         | Yes                              | 1       | B0-B1-B2…                  | 1                  | 3                      |                        |
| 4    | BP        | NO                               | 2       | B1-P0-B3-P2…               | 1                  | 3                      |                        |
| 5    | BBBP      | Yes                              | 1       | B2-B1-B3-P0…               | 7                  | 4                      |                        |
| 6    | PPPP      | Yes                              | 4       | P0-P1-P2-P3…               | 1                  | 2                      |                        |
| 7    | BBBB      | Yes                              | 4       | B0-B1-B2-B3…               | 1                  | 3                      |                        |
| 8    | BBBB BBBB | Yes                              | 1       | B3-B2-B4- B1-B6-B5- B7-B0… | 12                 | 5                      |                        |
| 9    | P         | Yes                              | 1       | P0                          | 1                  | 2                      |                        |

GOP Preset 1

- 只有I帧，没有相互参考；
- 低延时；

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/03_multimedia_development/02_S100/codec/b02cc41ab083664ba3f8a3bef1543afa.png)

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/03_multimedia_development/02_S100/codec/fa1da95bc8801b2d6225b2abf1b2f2d3.png)

GOP Preset 2

- 只有I帧和P帧；
- P帧参考2个前向参考帧；
- 低延时；

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/03_multimedia_development/02_S100/codec/e3c2f773a89f6ee2fe2dab03200b6fd0.png)

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/03_multimedia_development/02_S100/codec/8fa5f892bd7282e82ac8ed96011c943d.png)

GOP Preset 3

- 只有I帧和B帧；
- B帧参考2个前向参考帧；
- 低延时；

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/03_multimedia_development/02_S100/codec/03bbdf35dc3e2a1b38f9e05d7038d064.png)

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/03_multimedia_development/02_S100/codec/e1b5707ea0c32b6a0c1658527a6186dd.png)

GOP Preset 4

- 只有I帧、P帧和B帧；
- P帧参考2个前向参考帧；
- B帧参考1个前向参考帧和一个后向参考帧；

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/03_multimedia_development/02_S100/codec/17e10e6a27db202fe9a0c2b5f3d5dd50.png)

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/03_multimedia_development/02_S100/codec/972bbe22d2e7364c1c0a3db03f57343e.png)

GOP Preset 5

- 只有I帧、P帧和B帧；
- P帧参考2个前向参考帧；
- B帧参考1个前向参考帧和一个后向参考帧，后向参考帧可为P帧或B帧；

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/03_multimedia_development/02_S100/codec/16ad2d15f0b22a91fda1450747a18422.png)

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/03_multimedia_development/02_S100/codec/8ff1393cdbb997c8768ea2f9f00c3c8b.png)

GOP Preset 6

- 只有I帧和P帧；
- P帧参考2个前向参考帧；
- 低延时；

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/03_multimedia_development/02_S100/codec/a5fbffa7c85a3423729f06d45f83a601.png)

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/03_multimedia_development/02_S100/codec/1e88c6cbacb8fff86f5d5fc301e01abd.png)

GOP Preset 7

- 只有I帧和B帧；
- B帧参考2个前向参考帧；
- 低延时；

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/03_multimedia_development/02_S100/codec/40cd6c4fa7cf66f9bf14c3675cb7ef20.png)

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/03_multimedia_development/02_S100/codec/be7fe30d2685e27e2b36f305ef745eb4.png)

GOP Preset 8

- 只有I帧和B帧；
- B帧参考1个前向参考帧，一个后向参考帧；

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/03_multimedia_development/02_S100/codec/9c46efaf2a9106bcee2468098e209b1f.png)

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/03_multimedia_development/02_S100/codec/d016b90fa0a06e183b6871bc430a8714.png)

GOP Preset 9

- 只有I帧和P帧；
- P帧参考1个前向参考帧；
- 低延时；

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/03_multimedia_development/02_S100/codec/0bc1b9d3e73b4037b64236650738b5cd.png)

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/03_multimedia_development/02_S100/codec/937b45950423ff5006e378cb510d695d.png)

#### VPU调试方式

VPU（视频处理单元）是一种专用的视觉处理单元，可以高效处理视频内容。VPU可以实现H265视频格式的编解码处理。用户通过Codec提供的接口即可获得输入的编码/解码流。

##### 编码状态

编码调试信息

```c
cat /sys/kernel/debug/vpu/venc
root@ubuntu:~# cat /sys/kernel/debug/vpu/venc
----encode enc param----
enc_idx  enc_id     profile       level width height pix_fmt fbuf_count extern_buf_flag bsbuf_count bsbuf_size mirror rotate
      0    h265        Main unspecified  4096   2160       0          5               1           5   13271040      0      0

----encode h265cbr param----
enc_idx rc_mode intra_period intra_qp bit_rate frame_rate initial_rc_qp vbv_buffer_size ctu_level_rc_enalbe min_qp_I max_qp_I min_qp_P max_qp_P min_qp_B max_qp_B hvs_qp_enable hvs_qp_scale qp_map_enable max_delta_qp
      0 h265cbr           20       30     5000         30            30            3000                   1        8       50        8       50        8       50             1            2             0           10
----encode gop param----
enc_idx  enc_id gop_preset_idx custom_gop_size decoding_refresh_type
      0    h265              2               0                     2
----encode intra refresh----
enc_idx  enc_id intra_refresh_mode intra_refresh_arg
      0    h265                  0                 0

----encode longterm ref----
enc_idx  enc_id use_longterm longterm_pic_period longterm_pic_using_period
      0    h265            0                   0                         0
----encode roi_params----
enc_idx  enc_id roi_enable roi_map_array_count
      0    h265          0                   0
----encode mode_decision 1----
enc_idx  enc_id mode_decision_enable pu04_delta_rate pu08_delta_rate pu16_delta_rate pu32_delta_rate pu04_intra_planar_delta_rate pu04_intra_dc_delta_rate pu04_intra_angle_delta_rate pu08_intra_planar_delta_rate pu08_intra_dc_delta_rate pu08_intra_angle_delta_rate pu16_intra_planar_delta_rate pu16_intra_dc_delta_rate pu16_intra_angle_delta_rate
      0    h265                    0               0               0               0               0                            0                        0                           0                            0                        0                           0                            0                        0                           0

----encode mode_decision 2----
enc_idx  enc_id pu32_intra_planar_delta_rate pu32_intra_dc_delta_rate pu32_intra_angle_delta_rate cu08_intra_delta_rate cu08_inter_delta_rate cu08_merge_delta_rate cu16_intra_delta_rate cu16_inter_delta_rate cu16_merge_delta_rate cu32_intra_delta_rate cu32_inter_delta_rate cu32_merge_delta_rate
      0    h265                            0                        0                           0                     0                     0                     0                     0                     0                     0                     0                     0                     0
----encode h265_transform----
enc_idx  enc_id chroma_cb_qp_offset chroma_cr_qp_offset user_scaling_list_enable
      0    h265                   0                   0                        0
----encode h265_pred_unit----
enc_idx  enc_id intra_nxn_enable constrained_intra_pred_flag strong_intra_smoothing_enabled_flag max_num_merge
      0    h265                1                           0                                   1             2
----encode h265 timing----
enc_idx  enc_id vui_num_units_in_tick vui_time_scale vui_num_ticks_poc_diff_one_minus1
      0    h265                  1000          30000                                 0
----encode h265 slice params----
enc_idx  enc_id h265_independent_slice_mode h265_independent_slice_arg h265_dependent_slice_mode h265_dependent_slice_arg
      0    h265                           0                          0                         0                        0
----encode h265 deblk filter----
enc_idx  enc_id slice_deblocking_filter_disabled_flag slice_beta_offset_div2 slice_tc_offset_div2 slice_loop_filter_across_slices_enabled_flag
      0    h265                                     0                      0                    0                                            1

----encode h265 sao param----
enc_idx  enc_id sample_adaptive_offset_enabled_flag

      0    h265                              1
----encode status----
enc_idx  enc_id cur_input_buf_cnt cur_output_buf_cnt left_recv_frame left_enc_frame total_input_buf_cnt total_output_buf_cnt     fps
      0    h265                 4                  1               0              0                1093                 1089      35
```

参数解释

| 调试信息分组             | 状态参数            | 说明                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                |
| ------------------------ | ------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| encode enc param         | 基础编码参数        | enc_idx：编码实例值 enc_id：编码类型 profile：profile类型 level：h265 level类型 width：编码宽度 height：编码高度 pix_fmt：输入帧像素类型 fbuf_count：输入的rameBuffer数 extern_buf_flag：是否使用外部输入buffer bsbuf_count：bitstreamBuffer数 bsbuf_size：bitstreamBuffer大小 mirror：是否设置镜像 rotate：是否设置旋转                                                                                                                                                                                            |
| encode h265cbr param     | CBR码率控制参数     | enc_idx：编码实例值 rc_mode：码率控制类型 intra_period：I帧间隔 intra_qp：I帧qp值 bit_rate：码率值 frame_rate：帧率 initial_rc_qp：初始QP值 vbv_buffer_size：VBV buffer的大小 ctu_level_rc_enalbe：码率控制是否工作在ctu级别 min_qp_I：I帧最小QP值 max_qp_I：I帧最大QP值 min_qp_P：P帧最小QP值 max_qp_P：P帧最大QP值 min_qp_B：B帧最小QP值 max_qp_B：B帧最大QP值 hvs_qp_enable：码率控制是否工作在subCTU级别 hvs_qp_scale：QP缩放因子 qp_map_enable：使能ROI编码时的QP map max_delta_qp：指定HVS QP值的最大偏差范围 |
| encode gop param         | GOP参数             | enc_idx：编码实例值 enc_id：编码类型 gop_preset_idx：选择预置的GOP结构 custom_gop_size：自定义时GOP的大小 decoding_refresh_type：设置IDR帧的具体类型                                                                                                                                                                                                                                                                                                                                                                |
| encode intra refresh     | 帧内刷新参数        | enc_idx：编码实例值 enc_id：编码类型 intra_refresh_mode：帧内刷新模式 intra_refresh_arg：帧内刷新参数                                                                                                                                                                                                                                                                                                                                                                                                               |
| encode longterm ref      | 长期参考帧参数      | enc_idx：编码实例值 enc_id：编码类型 use_longterm：使能长期参考帧 longterm_pic_period：长期参考帧周期 longterm_pic_using_period：参考长期参考帧的周期                                                                                                                                                                                                                                                                                                                                                               |
| encode roi_params        | ROI参数             | enc_idx：编码实例值 enc_id：编码类型 roi_enable：使能ROI编码 roi_map_array_count：ROI map中元素的个数                                                                                                                                                                                                                                                                                                                                                                                                               |
| encode mode_decision 1   | 块编码模式决策参数1 | 各种模式选择参数值，包括pu04_delta_rate，pu08_delta_rate等                                                                                                                                                                                                                                                                                                                                                                                                                                                          |
| encode mode_decision 2   | 块编码模式决策参数2 | 各种模式选择参数值，包括pu32_intra_planar_delta_rate， pu32_intra_dc_delta_rate等                                                                                                                                                                                                                                                                                                                                                                                                                                   |
| encode h265_transform    | Transform参数       | enc_idx：编码实例值 enc_id：编码类型 chroma_cb_qp_offset：指定cb分量的QP偏差 chroma_cr_qp_offset：指定cr分量的QP偏差 user_scaling_list_enable：使能用户指定的scaling list                                                                                                                                                                                                                                                                                                                                           |
| encode h265_pred_unit    | 预测单元参数        | enc_idx：编码实例值 enc_id：编码类型 intra_nxn_enable：使能intra NXN PUs constrained_intra_pred_flag：帧内预测是否受限 strong_intra_smoothing_enabled_flag：滤波过程是否使用双向线性插值 max_num_merge：指定merge候选的数量                                                                                                                                                                                                                                                                                         |
| encode h265 timing       | Timing参数          | enc_idx：编码实例值 enc_id：编码类型 vui_num_units_in_tick：指定时间单位数 vui_time_scale：一秒内的时间单位数 vui_num_ticks_poc_diff_one_minus1：指定与等于1的图片顺序计数值之差对应的时钟滴答数                                                                                                                                                                                                                                                                                                                    |
| encode h265 slice params | Slice参数           | enc_idx：编码实例值 enc_id：编码类型 h265_independent_slice_mode：独立slice编码模式 h265_independent_slice_arg：独立slice的大小 h265_dependent_slice_mode：非独立slice编码模式 h265_dependent_slice_arg：非独立slice的大小                                                                                                                                                                                                                                                                                          |
| encode h265 deblk filter | 去块滤波参数        | enc_idx：编码实例值 enc_id：编码类型 slice_deblocking_filter_disabled_flag：是否进行slice内部滤波 slice_beta_offset_div2：指定当前切片的β去块参数偏移量 slice_tc_offset_div2：指定当前切片的tC去块参数偏移量 slice_loop_filter_across_slices_enabled_flag：是否进行边界滤波                                                                                                                                                                                                                                        |
| encode h265 sao param    | SAO参数             | enc_idx：编码实例值 enc_id：编码类型 sample_adaptive_offset_enabled_flag：是否对经过去块滤波处理后的重构图像进行采样自适应偏移处理                                                                                                                                                                                                                                                                                                                                                                                  |
| encode status            | 当前编码状态参数    | enc_idx：编码实例值 enc_id：编码类型 cur_input_buf_cnt：当前使用的inputbuffer数量 cur_output_buf_cnt：当前使用的outputbuffer数量 left_recv_frame：剩余需要接收的帧数（设置receive_frame_number后有效） left_enc_frame：剩余需要编码的帧数（设置receive_frame_number后有效） total_input_buf_cnt：表示当前总使用的inputbuffer数 total_output_buf_cnt：表示当前总使用的outputbuffer数 fps：表示当前的帧率                                                                                                             |

##### 解码状态

解码调试信息

```c
cat /sys/kernel/debug/vpu/vdec
root@ubuntu:~# cat /sys/kernel/debug/vpu/vdec
----decode param----
dec_idx dec_id feed_mode pix_fmt bitstream_buf_size bitstream_buf_count frame_buf_count
   0   h265     1      0     13271040          6   6
----h265 decode param----
dec_idx dec_id reorder_enable skip_mode bandwidth_Opt cra_as_bla dec_temporal_id_mode target_dec_temporal_id_plus1
   0   h265        1          0          1            0        0                      0
----decode frameinfo----
dec_idx dec_id display_width display_height
    0   h265       4096       2160
----decode status----
dec_idx dec_id cur_input_buf_cnt cur_output_buf_cnt total_input_buf_cnt total_output_buf_cnt fps
   0   h265          5                1              458       453         53
```

参数解释

| 调试信息分组      | 状态参数         | 说明                                                                                                                                                                                                                                                                             |
| ----------------- | ---------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| decode param      | 基础解码参数     | dec_idx：解码实例值 dec_id：解码类型 feed_mode：数据填充类型 pix_fmt：输出像素类型 bitstream_buf_size：输入的bitstream缓存区大小 bitstream_buf_count：输入的bitstream缓存区个数 frame_buf_count：输出的Framebuffer缓存的个数                                                     |
| h265 decode param | H265解码基础参数 | dec_idx：解码实例值 dec_id：解码类型 reorder_enable：使能解码器按显示顺序输出帧序列 skip_mode：使能帧解码忽略模式 bandwidth_Opt：使能节省带宽模式 cra_as_bla：使能CRA作为BLA处理 dec_temporal_id_mode：指定temporal id的选择模式 target_dec_temporal_id_plus1：指定temporal id值 |
| decode frameinfo  | 解码输出帧信息   | dec_idx：解码实例值 dec_id：解码类型 display_width：显示的宽度 display_height：显示的高度                                                                                                                                                                                        |
| decode status     | 当前解码状态参数 | dec_idx：解码实例值 dec_id：解码类型 cur_input_buf_cnt：当前使用的inputbuffer数量 cur_output_buf_cnt：当前使用的outputbuffer数量 total_input_buf_cnt：当前总使用的inputbuffer数 total_output_buf_cnt：当前总使用的outputbuffer数 fps：当前帧率                                   |

#### JPU 调试方式

JPU（图片处理单元）主要用以完成JPEG/MJPEG的编解码功能。用户可以通过CODEC接口输入待编码的YUV数据或待解码的JPEG图片，通过JPU处理后获取编码完的JPEG图片或解码完的YUV数据。

##### 编码状态

编码调试信息

```c
cat /sys/kernel/debug/jpu/jenc
root@ubuntu:~# cat /sys/kernel/debug/jpu/jenc
----encode param----
enc_idx  enc_id width height pix_fmt fbuf_count extern_buf_flag bsbuf_count bsbuf_size mirror rotate
      0    jpeg  1920   1088       1          5               0           5    3137536      0      0

----encode rc param----
enc_idx   rc_mode frame_rate quality_factor
      0 noratecontrol          0              0
----encode status----
enc_idx  enc_id cur_input_buf_cnt cur_output_buf_cnt left_recv_frame left_enc_frame total_input_buf_cnt total_output_buf_cnt     fps
      0    jpeg                 4                  1               0              0                4344                 4340     287
```

参数解释

| 调试信息分组    | 状态参数          | 说明                                                                                                                                                                                                                                                                                                                                                                                                    |
| --------------- | ----------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| encode param    | 基础编码参数      | enc_idx：编码实例 enc_id：编码类型 width：图像宽度 height：图像高度 pix_fmt：像素类型 fbuf_count：输入的Framebuffer缓存的个数 extern_buf_flag：是否使用用户分配的输入buffer bsbuf_count：输出的bitstream缓存区个数 bsbuf_size：输出的bitstream的大小 mirror：是否设置镜像 rotate：是否设置旋转                                                                                                          |
| encode rc param | mjpeg码率控制参数 | enc_idx：编码实例 rc_mode：码率控制模式 frame_rate：目标帧率 quality_factor：量化因子                                                                                                                                                                                                                                                                                                                   |
| encode status   | 当前编码状态参数  | enc_idx：编码实例值 enc_id：编码类型 cur_input_buf_cnt：当前使用的inputbuffer数量 cur_output_buf_cnt：当前使用的outputbuffer数量 left_recv_frame：剩余需要接收的帧数（设置receive_frame_number后有效） left_enc_frame：剩余需要编码的帧数（设置receive_frame_number后有效） total_input_buf_cnt：表示当前总使用的inputbuffer数 total_output_buf_cnt：表示当前总使用的outputbuffer数 fps：表示当前的帧率 |

##### 解码状态

解码调试信息

```c
cat /sys/kernel/debug/jpu/jdec
root@ubuntu:~# cat /sys/kernel/debug/jpu/jdec

----decode param----
dec_idx  dec_id feed_mode pix_fmt bitstream_buf_size bitstream_buf_count frame_buf_count mirror rotate
      0    jpeg         1       1            3133440                   5               5      0      0

----decode frameinfo----
dec_idx  dec_id display_width display_height
      0    jpeg          1920           1088

----decode status----
dec_idx  dec_id cur_input_buf_cnt cur_output_buf_cnt total_input_buf_cnt total_output_buf_cnt     fps
      0    jpeg                 0                  1                3779                 3779     264
```

参数解释

| 调试信息分组     | 状态参数         | 说明                                                                                                                                                                                                                                                 |
| ---------------- | ---------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| decode param     | 解码基础参数     | dec_idx：解码实例 dec_id：解码类型 feed_mode： pix_fmt：图像像素 bitstream_buf_size：输入的bitstream缓存区大小 bitstream_buf_count：输入的bitstream缓存区个数 frame_buf_count：输出的Framebuffer缓存的个数 mirror：是否设置镜像 rotate：是否设置旋转 |
| decode frameinfo | 解码输出帧信息   | dec_idx：解码实例值 dec_id：解码类型 display_width：显示的宽度 display_height：显示的高度                                                                                                                                                            |
| decode status    | 当前编码状态参数 | dec_idx：解码实例值 dec_id：解码类型 cur_input_buf_cnt：当前使用的inputbuffer数量 cur_output_buf_cnt：当前使用的outputbuffer数量 total_input_buf_cnt：当前总使用的inputbuffer数 total_output_buf_cnt：当前总使用的outputbuffer数 fps：当前帧率       |

### 典型场景

#### 单路编码

单路编码场景如下图所示。Scenario0是简单场景，从EMMC中读取YUV视频/图像文件，经过VPU硬件编码输出的H26x码流或JPU硬件编码输出的Jpeg图像，最后保存为文件存储到EMMC。Scenario1是串联前后级模块的复杂场景，将摄像头采集的数据编码压缩后进行保存或通过网络和PCIE传输。

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/03_multimedia_development/02_S100/codec/788c1e3b839232111ccd53d35d25e278.png)

#### 单路解码

单路解码场景如下图所示。Scenario0是简单场景，从EMMC中读取H26x码流/Jpeg图像文件，经过VPU或JPU硬件解码输出的YUV数据，最后保存为文件存储到EMMC。Scenario1是串联前后级模块的复杂场景，通过网络或PCIE接收已编码的视频或图像数据，经过VPU或JPU硬件解码后使用IDE显示播放。

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/03_multimedia_development/02_S100/codec/e50f9bf3c4d1ecfbd36b354f9009e8bc.png)

#### 多路编码

多路编码场景如下图所示，Scenario0是文件输入的简单场景，Scenario1是串联前后级模块的复杂场景，需要注意的是在Scenario1场景要综合考虑链路中各个模块的能力限制。

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/03_multimedia_development/02_S100/codec/272e3467c640af379d1b4c0a1de27eae.png)

#### 多路解码

多路解码场景如下图所示，Scenario0是文件输入的简单场景，Scenario1是串联前后级模块的复杂场景，需要注意的是在Scenario1场景要综合考虑链路中各个模块的能力限制。

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/03_multimedia_development/02_S100/codec/04f0aba90a1d65017dfeb90f9afa43e2.png)

## Codec API

### MediaCodec接口说明

MediaCodec模块主要用于音频、视频和JPEG图像的编解码。本模块将会提供一系列的接口便于用户输入待处理的数据和获取处理完的数据。本模块支持多路编码或解码实例同时工作，其中视频和JPEG图像的编解码由硬件实现，用户需要调用libmultimedia.so，音频则基于FFMPEG的接口由软件实现，用户需要调用libffmedia.so，如下表所示为RDKS100所支持的视频编解码规范和音频编解码规范，需要注意的是AAC的编解码需要license授权，因此用户需要获得授权后才能使能相关代码。

H265 profile最高支持Main profile，Level最高支持L5.1，Tier支持Main-tier。H264 profile最高支持High profile，Level最高支持L5.2。MJPEG/JPEG 支持ISO/IEC 10918-1 Baseline sequential。音频支持：G.711 A-law/Mu-law，G.729 ADPCM，ADPCM IMA WAV，FLAC，AAC LC，AAC Main，AAC SSR，AAC LTP，AAC LD，AAC HE，AAC HEv2（AAC需要license授权）。

此外视频和图像的数据源包括VIO输入的图像和用户输入的YUV数据两类，用户输入的YUV数据则可能是从文件中加载或者通过网络传输获得；音频的数据源包括MIC输入的音频和用户输入的PCM数据两类，MIC输入的音频是由Audio Codec采集的数字信号，用户输入的PCM数据则可能是从文件或网络传输获得。

#### GOP

H264/H265编码支持GOP结构的设置，用户可从预置的9种GOP结构中选择，也可自定义GOP结构。

##### GOP结构表

GOP结构表可定义一组周期性的GOP结构，该GOP结构将用于整个编码过程。单个结构表中的元素如下表所示，其中可以指定该图像的参考帧，如果IDR帧后的其他帧指定的参考帧为IDR帧前的数据帧，编码器内部会自动处理这种情况使其不参考其他帧，用户无需关心这种情况。用户在自定义GOP结构时需要指明结构表的数量，最多可定义3个结构表，结构表的顺序需要按照解码顺序排列。

| Element        | Description                                                                                                                                                                                                                                                                                                                      |
| -------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Type           | Slice type（I，P）                                                                                                                                                                                                                                                                                                               |
| POC            | Display order of the frame within a GOP，ranging from 1 to GOP size                                                                                                                                                                                                                                                              |
| QPoffset       | A quantization parameter of the picture in the custom GOP                                                                                                                                                                                                                                                                        |
| NUM_REF_PIC_L0 | Flag to use multi reference picture for P picture. It is valid only if PIC_TYPE is P                                                                                                                                                                                                                                             |
| temporal_id    | Temporal layer of the frame. A frame cannot predict from a frame with a higher temporal id (0~6).                                                                                                                                                                                                                                |
| 1st_ref_POC    | The POC of the 1st reference picture of L0                                                                                                                                                                                                                                                                                       |
| 2nd_ref_POC    | The POC of 1st reference picture of L1 in case that Type is equal to B. The POC of 2nd reference picture of L0 in case that Type is equal to P. Note that reference_L1 can have the same POC as reference in B slice. But for compression efficiency it is recommended that reference_L1 have a different POC from reference_L0. |

##### GOP预置结构

| Index | GOP Stru cture | Low Delay (encoding order and display order are same) | GOP Size | Encoding Order              | Minimum Source Frame Buffer | Minimum Decoded Picture Buffer | Intra Period (I Frame Interval) Requirement |
| ----- | -------------- | ----------------------------------------------------- | -------- | --------------------------- | --------------------------- | ------------------------------ | ------------------------------------------- |
| 1     | I              | Yes                                                   | 1        | I0-I1-I2…                  | 1                           | 1                              |                                             |
| 2     | P              | Yes                                                   | 1        | P0-P1-P2…                  | 1                           | 2                              |                                             |
| 3     | B              | Yes                                                   | 1        | B0-B1-B2…                  | 1                           | 3                              |                                             |
| 4     | BP             | NO                                                    | 2        | B1-P0-B3-P2…               | 1                           | 3                              | Multiple of 2                               |
| 5     | BBBP           | Yes                                                   | 1        | B2-B1-B3-P0…               | 7                           | 4                              | Multiple of 4                               |
| 6     | PPPP           | Yes                                                   | 4        | P0-P1-P2-P3…               | 1                           | 2                              |                                             |
| 7     | BBBB           | Yes                                                   | 4        | B0-B1-B2-B3…               | 1                           | 3                              |                                             |
| 8     | BBBB BBBB      | Yes                                                   | 1        | B3-B2-B4- B1-B6-B5- B7-B0… | 12                          | 5                              | Multiple of 8                               |
| 9     | P              | Yes                                                   | 1        | P0                          | 1                           | 2                              |                                             |

以下会对9种预置的GOP结构进行说明。

GOP Preset 1

- 只有I帧，没有相互参考；
- 低延时；

![gop1](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/03_multimedia_development/02_S100/codec/gop1.png)

![gop2](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/03_multimedia_development/02_S100/codec/gop2.png)

GOP Preset 2

- 只有I帧和P帧；
- P帧参考2个前向参考帧；
- 低延时；

![gop3](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/03_multimedia_development/02_S100/codec/gop3.png)

![gop4](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/03_multimedia_development/02_S100/codec/gop4.png)

GOP Preset 3

- 只有I帧和B帧；
- B帧参考2个前向参考帧；
- 低延时；

![gop5](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/03_multimedia_development/02_S100/codec/gop5.png)

![gop6](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/03_multimedia_development/02_S100/codec/gop6.png)

GOP Preset 4

- 只有I帧、P帧和B帧；
- P帧参考2个前向参考帧；
- B帧参考1个前向参考帧和一个后向参考帧；

![gop7](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/03_multimedia_development/02_S100/codec/gop7.png)

![gop8](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/03_multimedia_development/02_S100/codec/gop8.png)

GOP Preset 5

- 只有I帧、P帧和B帧；
- P帧参考2个前向参考帧；
- B帧参考1个前向参考帧和一个后向参考帧，后向参考帧可为P帧或B帧；

![gop9](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/03_multimedia_development/02_S100/codec/gop9.png)

![gop10](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/03_multimedia_development/02_S100/codec/gop10.png)

GOP Preset 6

- 只有I帧和P帧；
- P帧参考2个前向参考帧；
- 低延时；

![gop11](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/03_multimedia_development/02_S100/codec/gop11.png)

![gop12](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/03_multimedia_development/02_S100/codec/gop12.png)

GOP Preset 7

- 只有I帧和B帧；
- B帧参考2个前向参考帧；
- 低延时；

![gop13](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/03_multimedia_development/02_S100/codec/gop13.png)

![gop14](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/03_multimedia_development/02_S100/codec/gop14.png)

GOP Preset 8

- 只有I帧和B帧；
- B帧参考1个前向参考帧，一个后向参考帧；

![gop15](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/03_multimedia_development/02_S100/codec/gop15.png)

![gop16](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/03_multimedia_development/02_S100/codec/gop16.png)

GOP Preset 9

- 只有I帧和P帧；
- P帧参考1个前向参考帧；
- 低延时；

![gop17](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/03_multimedia_development/02_S100/codec/gop17.png)

![gop18](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/03_multimedia_development/02_S100/codec/gop18.png)

#### 长期参考帧

用户可指定长期参考帧的周期和参考长期参考帧的周期，如下图所示：

![reference_frame](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/03_multimedia_development/02_S100/codec/reference_frame.png)

#### Intra Refresh

Intra
Refresh模式通过在非I帧内部周期性的插入帧内编码的MB/CTU来提高容错性。它能够为解码器提供更多的修复点来避免时域错误造成的图像损坏。用户可以指定MB/CTU的连续行数、列数或者步长来强制编码器插入帧内编码单元，用户还可指定帧内编码单元的大小由编码器内部决定哪一块需要帧内编码。

#### 码率控制

MediaCodec支持对H264、H265和MJPEG协议的码率控制，分别支持H264、H265编码通道的CBR、VBR、AVBR、FixQp和QpMap五种码率控制方式，以及支持MJPEG编码通道的FixQp码率控制方式。CBR能够保证整体的编码码率稳定；VBR则是保证编码图像的质量稳定；而AVBR会兼顾码率和图像质量，产生码率和图像质量相对稳定的码流；FixQp则是固定每一个I帧、P帧的QP值；QPMAP则是为一帧图像中的每一个块指定QP值，其中H265块大小为32x32，H264块大小为16x16。

对于CBR和AVBR来说，编码器内部会为每一帧图片找到合适的QP值，从而保证恒定码率。编码器内部支持三种级别的码率控制，分别为帧级别、CTU/MB级别和subCTU/subMB级别。其中帧级别的控制主要会根据目标码率为每一帧图片产生一个QP值，从而保证码率恒定；CTU/MB级别的控制则根据每一个64x64的CTU或16x16的MB的目标码率为每个block产生一个QP值，能够得到更好的码率控制，但是频繁的QP值调整会造成图像质量不稳定的问题；subCTU/subMB级别的控制则为每一个32x32的subCTU或8x8的subMB产生一个QP值，其中复杂的块会得到较高的QP值，静态的块则会得到较低的QP值，因为相比于复杂的区域人眼对于静态的区域更敏感，复杂和静态区域的检测主要依赖于内部硬件模块，这个级别控制主要是为了提高主观图像质量同时保证码率恒定，该模式控制下SSIM得分较高，但是PSNR得分会降低。

#### ROI

ROI编码的实现依赖于和QPMAP类似，需要用户按照光栅扫描的方向为每一个块设定QP值，如下图：

![roi_map](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/03_multimedia_development/02_S100/codec/roi_map.png)

对于H264编码来说，每一个块的大小为16x16，而H265中则为32x32。在ROI
map表中，每一个QP值占用一个字节，大小为0\~51。ROI编码可以和CBR和AVBR一起工作，当不使能CBR或AVBR时，每个块区域的实际QP值就为ROI
map中指定的值，当使能CBR或AVBR时，则每个块区域的实际值由以下公式得到：

QP(i) = MQP(i)+ RQP(i) - ROIAvgQP

其中MQP为ROI map中的值， RQP为编码器内部码率控制得到的值， ROIAvaQP为ROI
map中QP的平均值。

#### 输入输出buffer管理

MediaCodec的buffer包括输入和输出buffer两种，一般情况下，这些buffer会由MediaCodec通过ION接口统一分配，用户不需要关心buffer的分配，只需要在操作buffer前执行dequeue操作获取空闲的buffer，处理完后执行queue操作返还该buffer。但是为了减少某些情况下buffer的拷贝操作，比如PYM的输出buffer用来编码时，该buffer是由PYM内部通过ION分配，可直接作为MediaCodec的输入buffer，因此MediaCodec还支持编码时的输入buffer由用户分配，但是用户必须通过ION接口分配物理连续的buffer，还需要在MediaCodec配置前指定media\_codec\_context\_t中的external\_frame\_buf变量。需要注意的是，当用户指定输入buffer不需要MediaCodec分配之后，在buffer操作时，用户仍然需要执行dequeue操作获取队列信息，然后对队列中的信息进行赋值（主要是虚拟地址和物理地址），再执行queue操作。

![buffer](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/03_multimedia_development/02_S100/codec/buffer.png)

#### 帧率控制

MediaCodec内部目前不支持帧率控制，当用户不使用hb\_mm\_mc\_set\_camera接口使能VIO和MediaCodec的直接交互时，用户可自行控制buffer的输入帧率；当用户使能VIO和MediaCodec的交互后，用户不需要控制输入buffer，只需要操作输出buffer，此时MediaCodec内部也不会对输入buffer进行帧率控制，当编码出现卡顿或者输入buffer队列满等情况时，MediaCodec会等待队列控线后再次操作。

#### 帧Skip设置

用户可调用hb\_mm\_mc\_skip\_pic设置下一次queue操作输入的图像的编码模式为skip模式，该模式只对非I帧编码有效；skip模式下编码器内部会忽略输入帧，而是利用上一帧的重构帧生成该次输入的的重构帧，输入帧则被编码成P帧。

#### JPEG编解码限制

- JPEG/MJPEG编码时，当输入为yuv420和yuv422格式时，要求输入的宽16对齐，输入的高8对齐，当输入为yuv440、yuv444和yuv400格式时，要求输入的宽和高8对齐，如果同时crop，要求输入的x和y坐标8对齐；
- JPEG/MJPEG编码同时旋转90/270，当输入为yuv420格式时，要求输入的宽16对齐，高8对齐，当输入的格式为yuv422和yuv440时，要求输入的宽16对齐，输入的高16对齐，当输入的格式为yuv444和yuv400时，要求输入的宽和高8对齐，如果同时crop，当输入为yuv420格式时，要求crop的宽16对齐，

crop的高8对齐，当输入的格式为yuv422和yuv440时，要求crop的宽16对齐，crop的高16对齐，当输入的格式为yuv444和yuv400时，要求crop的宽和高8对齐；

- JPEG/MJPEG编码同时旋转90/270，当输入格式为yuv422格式时，旋转后的格式会变成yuv440，当输入格式为yuv440格式时，旋转后的格式变成yuv422；
- JPEG/MJPEG解码同时旋转或镜像时，要求输出的yuv格式需要和输入的图像格式一致，但是YUV422格式的JPEG/MJPEG旋转90/270解码时，要求输出的格式为YUV440p/YUYV/YVYU/UYVY/VYUY；
- JPEG/MJPEG解码时，旋转或镜像不能和Crop同时工作；
- JPEG/MJPEG解码时，输出buffer的宽高要求和输入格式的MCU

width和height对齐，如果使能了Crop，crop的参数（包括起始坐标和宽高）需要和输入格式的MCU
width和height对齐；（MCU size for 420: 16x16, 422: 16x8, 440: 8x16, 400: 8x8, 444: 8x8.）

- JPEG/MJPEG解码时，输出格式为Packed
  YUV444，要求输入格式为YUV444格式；
- JPEG/MJPEG解码只支持MC\_FEEDING\_MODE\_FRAME\_SIZE模式；
- JPEG编码时，如果用户指定bitstream buffer得size，需要额外分配4k大小。
- JPEG编码时，由于编码器内部处理以16x16为单元做编码处理，当待编码数据为非16x16对齐时，编码完的数据最后一部分填充的部分会存在差异，但是不会影响有效数据，这个是硬件限制；因此做md5比较时需要注意这点。

### API参考

#### hb\_mm\_mc\_get\_descriptor

【函数声明】

const
media\_codec\_descriptor\_t\*hb\_mm\_mc\_get\_descriptor(media\_codec\_id\_t
codec\_id);

【参数描述】

- \[IN\] media\_codec\_id\_t codec\_id：表示codec类型

【返回值】

- 非空：codec描述信息
- NULL：表示查询不到该codec id对应的描述符

【功能描述】

根据codec\_id获取MediaCodec支持的codec信息，信息包括codec名字，详细描述，MIME类型以及codec支持的profile类型等。

【示例代码】

```
#include "hb_media_codec.h"
#include "hb_media_error.h"
int main(int argc, char *argv[])
{
    const media_codec_descriptor_t *desc = NULL;
    desc = hb_mm_mc_get_descriptor(MEDIA_CODEC_ID_H265);
    return 0;
}
```

#### hb\_mm\_mc\_get\_default\_context

【函数声明】

hb\_s32 hb\_mm\_mc\_get\_default\_context(media\_codec\_id\_t codec\_id,
hb\_bool encoder, media\_codec\_context\_t \*context)

【参数描述】

- \[IN\] media\_codec\_id\_t codec\_id：表示codec类型
- \[IN\] hb\_bool encoder：指定codec是编码器还是解码器
- \[OUT\] media\_codec\_context\_t
  \*context：指定codec类型默认的context

【返回值】

- 0：操作成功
- HB\_MEDIA\_ERR\_INVALID\_PARAMS： 无效参数

【功能描述】

获取指定的codec的默认属性。

【示例代码】

```
#include "hb_media_codec.h"
#include "hb_media_error.h"
int main(int argc, char *argv[])
{
    int ret = 0;
    media_codec_context_t context;
    memset(&context, 0x00, sizeof(context));
    ret = hb_mm_mc_get_default_context(MEDIA_CODEC_ID_H265, 1, &context)
    return 0;
}
```

#### hb\_mm\_mc\_initialize

【函数声明】

hb\_s32 hb\_mm\_mc\_initialize(media\_codec\_context\_t \*context)

【参数描述】

- \[IN\] media\_codec\_context\_t \*context：指定codec类型的context

【返回值】

- 0：操作成功
- HB\_MEDIA\_ERR\_UNKNOWN： 未知错误
- HB\_MEDIA\_ERR\_INVALID\_PARAMS：无效参数
- HB\_MEDIA\_ERR\_OPERATION\_NOT\_ALLOWED：操作不允许
- HB\_MEDIA\_ERR\_INSUFFICIENT\_RES：内部内存资源不足
- HB\_MEDIA\_ERR\_NO\_FREE\_INSTANCE：没有可用的instance（Video最多32个，MJPEG/JPEG最多64个，Audio最多32个）

【功能描述】

初始化编码或解码器，调用成功后MediaCodec进入MEDIA\_CODEC\_STATE\_INITIALIZED状态。

【示例代码】

```
#include "hb_media_codec.h"
#include "hb_media_error.h"
static Uint64 osal_gettime(void)
{
    struct timespec tp;
    clock_gettime(CLOCK_MONOTONIC, &tp);
    return ((Uint64)tp.tv_sec*1000 + tp.tv_nsec/1000000);
}
typedef struct MediaCodecTestContext {
    media_codec_context_t *context;
    char *inputFileName;
    char *outputFileName;
} MediaCodecTestContext;
typedef struct AsyncMediaCtx {
    media_codec_context_t *ctx;
    FILE *inFile;
    FILE *outFile;
    int lastStream;
    Uint64 startTime;
    int32_t duration;
} AsyncMediaCtx;
static void on_encoder_input_buffer_available(hb_ptr userdata,
media_codec_buffer_t *inputBuffer) {
    AsyncMediaCtx *asyncCtx = (AsyncMediaCtx *)userdata;
    Int noMoreInput = 0;
    hb_s32 ret = 0;
    Uint64 curTime = 0;
    if (!noMoreInput) {
        curTime = osal_gettime();
        if ((curTime - asyncCtx->startTime)/1000 < (uint32_t)asyncCtx->duration) {
            ret = fread(inputBuffer->vframe_buf.vir_ptr[0], 1,
            inputBuffer->vframe_buf.size, asyncCtx->inFile);
            if (ret <= 0) {
                if(fseek(asyncCtx->inFile, 0, SEEK_SET)) {
                printf("Failed to rewind input filen");
            } else {
                ret = fread(inputBuffer->vframe_buf.vir_ptr[0], 1,
                inputBuffer->vframe_buf.size, asyncCtx->inFile);
                if (ret <= 0) {
                    printf("Failed to read input filen");
                }
            }
        }
    }
    if (!ret) {
        printf("%s There is no more input data!n", TAG);
        inputBuffer->vframe_buf.frame_end = TRUE;
        noMoreInput = 1;
    } else {
        inputBuffer->vframe_buf.frame_end = TRUE;
        inputBuffer->vframe_buf.size = 0;
    }
}
static void on_encoder_output_buffer_available(hb_ptr userdata,
media_codec_buffer_t *outputBuffer,
media_codec_output_buffer_info_t *extraInfo) {
    AsyncMediaCtx *asyncCtx = (AsyncMediaCtx *)userdata;
    mc_265_output_stream_info_t info = extraInfo->video_stream_info;
    fwrite(outputBuffer->vstream_buf.vir_ptr,
    outputBuffer->vstream_buf.size, 1, asyncCtx->outFile);
    if (outputBuffer->vstream_buf.stream_end) {
        printf("There is no more output data!n");
        asyncCtx->lastStream = 1;
    }
}
static void on_encoder_media_codec_message(hb_ptr userdata, hb_s32
       error) {
    AsyncMediaCtx *asyncCtx = (AsyncMediaCtx *)userdata;
    if (error) {
        asyncCtx->lastStream = 1;
        printf("ERROR happened!n");
    }
}
static void on_vlc_buffer_message(hb_ptr userdata, hb_s32 * vlc_buf)
{
    MediaCodecTestContext *ctx = (MediaCodecTestContext *)userdata;
    printf("%s %s VLC Buffer size = %d; Reset to %d.n", TAG,
    __FUNCTION__,
    *vlc_buf, ctx->vlc_buf_size);
    *vlc_buf = ctx->vlc_buf_size;
}
static void do_async_encoding(void *arg) {
    hb_s32 ret = 0;
    FILE *outFile;
    FILE *inFile;
    int step = 0;
    AsyncMediaCtx asyncCtx;
    MediaCodecTestContext *ctx = (MediaCodecTestContext *)arg;
    media_codec_context_t *context = ctx->context;
    char *inputFileName = ctx->inputFileName;
    char *outputFileName = ctx->outputFileName;
    media_codec_state_t state = MEDIA_CODEC_STATE_NONE;
    inFile = fopen(inputFileName, "rb");
    if (!inFile) {
        goto ERR;
    }
    outFile = fopen(outputFileName, "wb");
    if (!outFile) {
        goto ERR;
    }
    memset(&asyncCtx, 0x00, sizeof(AsyncMediaCtx));
    asyncCtx.ctx = context;
    asyncCtx.inFile = inFile;
    asyncCtx.outFile = outFile;
    asyncCtx.lastStream = 0;
    asyncCtx.duration = 5;
    asyncCtx.startTime = osal_gettime();
    ret = hb_mm_mc_initialize(context);
    if (ret) {
        goto ERR;
    }
    media_codec_callback_t callback;
    callback.on_input_buffer_available =
    on_encoder_input_buffer_available;
    callback.on_output_buffer_available =
    on_encoder_output_buffer_available;
    callback.on_media_codec_message = on_encoder_media_codec_message;
    ret = hb_mm_mc_set_callback(context, &callback, &asyncCtx);
    if (ret) {
        goto ERR;
    }
    media_codec_callback_t callback2;
    callback2.on_vlc_buffer_message = on_vlc_buffer_message;
    if (ctx->vlc_buf_size > 0) {
        ret = hb_mm_mc_set_vlc_buffer_listener(context, &callback2, ctx);
        if (ret) {
            goto ERR;
        }
    }
    ret = hb_mm_mc_configure(context);
    if (ret) {
        goto ERR;
    }
    mc_av_codec_startup_params_t startup_params;
    startup_params.video_enc_startup_params.receive_frame_number = 0;
    ret = hb_mm_mc_start(context, &startup_params);
    if (ret) {
        goto ERR;
    }
    while(!asyncCtx.lastStream) {
        sleep(1);
    }
    hb_mm_mc_stop(context);
    hb_mm_mc_release(context);
    context = NULL;
ERR:
    hb_mm_mc_get_state(context, &state);
    if (context && state !=
    MEDIA_CODEC_STATE_UNINITIALIZED) {
        hb_mm_mc_stop(context);
        hb_mm_mc_release(context);
    }
    if (inFile)
        fclose(inFile);
    if (outFile)
        fclose(outFile);
}
int main(int argc, char *argv[])
{
    int ret = 0;
    char outputFileName[MAX_FILE_PATH] = "./tmp.yuv";
    char inputFileName[MAX_FILE_PATH] = "./output.h265";
    mc_video_codec_enc_params_t *params = NULL;
    media_codec_context_t context;
    memset(&context, 0x00, sizeof(media_codec_context_t));
    context.codec_id = MEDIA_CODEC_ID_H265;
    context.encoder = 1;
    params = &context.video_enc_params;
    params->width = 640;
    params->height = 480;
    params->pix_fmt = MC_PIXEL_FORMAT_YUV420P;
    params->frame_buf_count = 5;
    params->external_frame_buf = 0;
    params->bitstream_buf_count = 5;
    params->rc_params.mode = MC_AV_RC_MODE_H265CBR;
    ret = hb_mm_mc_get_rate_control_config(&context, &params->rc_params);
    if (ret) {
        return -1;
    }
    params->rc_params.h265_cbr_params.bit_rate = 5000;
    params->rc_params.h265_cbr_params.frame_rate = 30;
    params->rc_params.h265_cbr_params.intra_period = 30;
    params->gop_params.decoding_refresh_type = 2;
    params->gop_params.gop_preset_idx = 2;
    params->rot_degree = MC_CCW_0;
    params->mir_direction = MC_DIRECTION_NONE;
    params->frame_cropping_flag = FALSE;
    MediaCodecTestContext ctx;
    memset(&ctx, 0x00, sizeof(ctx));
    ctx.context = &context;
    ctx.inputFileName = inputFileName;
    ctx.outputFileName = outputFileName;
    do_async_encoding(&ctx);
    return 0;
}
```

#### hb\_mm\_mc\_set\_callback

【函数声明】

hb\_s32 hb\_mm\_mc\_set\_callback(media\_codec\_context\_t \*context,
const media\_codec\_callback\_t \*callback, hb\_ptr userdata)

【参数描述】

- \[IN\] media\_codec\_context\_t \*context：指定codec类型的context
- \[IN\] const media\_codec\_callback\_t \*callback：用户回调函数
- \[IN\] hb\_ptr
  userdata：用户数据指针，该值会在回调函数被调用时作为入参传入

【返回值】

- 0：操作成功
- HB\_MEDIA\_ERR\_UNKNOWN： 未知错误
- HB\_MEDIA\_ERR\_INVALID\_PARAMS：无效参数
- HB\_MEDIA\_ERR\_OPERATION\_NOT\_ALLOWED：操作不允许

【功能描述】

设置回调函数指针，调用该函数后MediaCodec会进入异步工作模式。

【示例代码】

参考 [hb_mm_mc_initialize](#hb_mm_mc_initialize)

#### hb\_mm\_mc\_configure

【函数声明】

hb\_s32 hb\_mm\_mc\_configure(media\_codec\_context\_t \*context)

【参数描述】

- \[IN\] media\_codec\_context\_t \*context：指定codec类型的context

【返回值】

- 0：操作成功
- HB\_MEDIA\_ERR\_UNKNOWN： 未知错误
- HB\_MEDIA\_ERR\_INVALID\_PARAMS：无效参数
- HB\_MEDIA\_ERR\_OPERATION\_NOT\_ALLOWED：操作不允许
- HB\_MEDIA\_ERR\_INSUFFICIENT\_RES：内部内存资源不足
- HB\_MEDIA\_ERR\_INVALID\_INSTANCE：无效实例

【功能描述】

根据输入信息配置编码或解码器，调用成功后MediaCodec进入MEDIA\_CODEC\_STATE\_CONFIGURED状态。

【示例代码】

参考 [hb_mm_mc_initialize](#hb_mm_mc_initialize)

#### hb\_mm\_mc\_start

【函数声明】

hb\_s32 hb\_mm\_mc\_start(media\_codec\_context\_t \*context, const
mc\_av\_codec\_startup\_params\_t \*info)

【参数描述】

- \[IN\] media\_codec\_context\_t \*context：指定codec类型的context
- \[IN\] mc\_av\_codec\_startup\_params\_t
  \*info：指定音视频编解码时的启动参数

【返回值】

- 0：操作成功
- HB\_MEDIA\_ERR\_UNKNOWN： 未知错误
- HB\_MEDIA\_ERR\_INVALID\_PARAMS：无效参数
- HB\_MEDIA\_ERR\_OPERATION\_NOT\_ALLOWED：操作不允许
- HB\_MEDIA\_ERR\_INSUFFICIENT\_RES：内部内存资源不足
- HB\_MEDIA\_ERR\_INVALID\_INSTANCE：无效实例

【功能描述】

启动编码/解码流程，MediaCodec将创建编解码实例、设置序列或解析数据流、注册Framebuffer、编码头信息等，调用成功后MediaCodec进入MEDIA\_CODEC\_STATE\_STARTED状态。

【示例代码】

参考 [hb_mm_mc_initialize](#hb_mm_mc_initialize)

#### hb\_mm\_mc\_stop

【函数声明】

hb\_s32 hb\_mm\_mc\_stop(media\_codec\_context\_t \*context)

【参数描述】

- \[IN\] media\_codec\_context\_t \*context：指定codec类型的context

【返回值】

- 0：操作成功
- HB\_MEDIA\_ERR\_UNKNOWN： 未知错误
- HB\_MEDIA\_ERR\_OPERATION\_NOT\_ALLOWED：操作不允许
- HB\_MEDIA\_ERR\_INVALID\_INSTANCE：无效实例

【功能描述】

停止编码/解码流程，退出所有子线程并释放相关资源，调用成功后MediaCodec回到MEDIA\_CODEC\_STATE\_INITIALIZED状态。

【示例代码】

参考 [hb_mm_mc_initialize](#hb_mm_mc_initialize)

#### hb\_mm\_mc\_pause

【函数声明】

hb\_s32 hb\_mm\_mc\_pause(media\_codec\_context\_t \*context)

【参数描述】

- \[IN\] media\_codec\_context\_t \*context：指定codec类型的context

【返回值】

- 0：操作成功
- HB\_MEDIA\_ERR\_UNKNOWN： 未知错误
- HB\_MEDIA\_ERR\_OPERATION\_NOT\_ALLOWED：操作不允许
- HB\_MEDIA\_ERR\_INVALID\_INSTANCE：无效实例

【功能描述】

停止编码/解码流程，暂停所有子线程，调用成功后MediaCodec进入到MEDIA\_CODEC\_STATE\_PAUSED状态。

【示例代码】

参考 [hb_mm_mc_queue_input_buffer](#hb_mm_mc_queue_input_buffer)

#### hb\_mm\_mc\_flush

【函数声明】

hb\_s32 hb\_mm\_mc\_flush(media\_codec\_context\_t \*context)

【参数描述】

- \[IN\] media\_codec\_context\_t \*context：指定codec类型的context

【返回值】

- 0：操作成功
- HB\_MEDIA\_ERR\_UNKNOWN： 未知错误
- HB\_MEDIA\_ERR\_OPERATION\_NOT\_ALLOWED：操作不允许
- HB\_MEDIA\_ERR\_INVALID\_INSTANCE：无效实例

【功能描述】

刷新输入输出buffer缓冲区，强制编码器/解码器刷新未处理的输入输出buffer，函数调用后MediaCodec进入MEDIA\_CODEC\_STATE\_FLUSHING状态，操作成功后，MediaCodec会再次进入MEDIA\_CODEC\_STATE\_STARTED状态。

【示例代码】

参考 [hb_mm_mc_queue_input_buffer](#hb_mm_mc_queue_input_buffer)

#### hb\_mm\_mc\_release

【函数声明】

hb\_s32 hb\_mm\_mc\_release(media\_codec\_context\_t \*context)

【参数描述】

- \[IN\] media\_codec\_context\_t \*context：指定codec类型的context

【返回值】

- 0：操作成功
- HB\_MEDIA\_ERR\_UNKNOWN： 未知错误
- HB\_MEDIA\_ERR\_OPERATION\_NOT\_ALLOWED：操作不允许
- HB\_MEDIA\_ERR\_INVALID\_INSTANCE：无效实例

【功能描述】

释放MediaCodec内部所有资源，用户需要在调用该函数前调用hb\_mm\_mc\_stop来停止编解码，操作成功后MediaCodec进入MEDIA\_CODEC\_STATE\_UNINITIALIZED状态。

【示例代码】

参考 [hb_mm_mc_initialize](#hb_mm_mc_initialize)

#### hb\_mm\_mc\_get\_state

【函数声明】

hb\_s32 hb\_mm\_mc\_get\_state(media\_codec\_context\_t \*context,
media\_codec\_state\_t \*state)

【参数描述】

- \[IN\] media\_codec\_context\_t \*context：指定codec类型的context
- \[OUT\] media\_codec\_state\_t \*state：MediaCodec当前状态

【返回值】

- 0：操作成功
- HB\_MEDIA\_ERR\_UNKNOWN： 未知错误
- HB\_MEDIA\_ERR\_OPERATION\_NOT\_ALLOWED：操作不允许
- HB\_MEDIA\_ERR\_INVALID\_INSTANCE：无效实例
- HB\_MEDIA\_ERR\_INVALID\_PARAMS：无效参数

【功能描述】

获取MediaCodec当前的状态。

【示例代码】

参考 [hb_mm_mc_initialize](#hb_mm_mc_initialize)

#### hb\_mm\_mc\_get\_status

【函数声明】

hb\_s32 hb\_mm\_mc\_get\_status(media\_codec\_context\_t \*context,
mc\_inter\_status\_t \*status)

【参数描述】

- \[IN\] media\_codec\_context\_t \*context：指定codec类型的context
- \[OUT\] mc\_inter\_status\_t \*status：MediaCodec当前内部状态

【返回值】

- 0：操作成功
- HB\_MEDIA\_ERR\_UNKNOWN： 未知错误
- HB\_MEDIA\_ERR\_OPERATION\_NOT\_ALLOWED：操作不允许
- HB\_MEDIA\_ERR\_INVALID\_INSTANCE：无效实例
- HB\_MEDIA\_ERR\_INVALID\_PARAMS：无效参数

【功能描述】

获取MediaCodec当前内部的状态信息。

【示例代码】

参考 [hb_mm_mc_get_fd](#hb_mm_mc_get_fd)

#### hb\_mm\_mc\_queue\_input\_buffer

【函数声明】

hb\_s32 hb\_mm\_mc\_queue\_input\_buffer(media\_codec\_context\_t
\*context, media\_codec\_buffer\_t \*buffer, hb\_s32 timeout)

【参数描述】

- \[IN\] media\_codec\_context\_t \*context：指定codec类型的context
- \[IN\] media\_codec\_buffer\_t \*buffer：输入的buffer信息
- \[IN\] hb\_s32 timeout：超时时间

【返回值】

- 0：操作成功
- HB\_MEDIA\_ERR\_UNKNOWN： 未知错误
- HB\_MEDIA\_ERR\_OPERATION\_NOT\_ALLOWED：操作不允许
- HB\_MEDIA\_ERR\_INVALID\_INSTANCE：无效实例
- HB\_MEDIA\_ERR\_INVALID\_PARAMS：无效参数
- HB\_MEDIA\_ERR\_INVALID\_BUFFER：无效buffer
- HB\_MEDIA\_ERR\_WAIT\_TIMEOUT：等待超时

【功能描述】

填充需要处理的buffer到MediaCodec中。

【示例代码】

```
#include "hb_media_codec.h"

#include "hb_media_error.h"
typedef struct MediaCodecTestContext {
    media_codec_context_t *context;
    char *inputFileName;
    char *outputFileName;
    int32_t duration; // s
} MediaCodecTestContext;
Uint64 osal_gettime(void)
{
    struct timespec tp;
    clock_gettime(CLOCK_MONOTONIC, &tp);
    return ((Uint64)tp.tv_sec*1000 + tp.tv_nsec/1000000);
}
static void do_sync_encoding(void *arg) {
    hb_s32 ret = 0;
    FILE *inFile;
    FILE *outFile;
    int noMoreInput = 0;
    int lastStream = 0;
    Uint64 lastTime = 0;
    Uint64 curTime = 0;
    int needFlush = 1;
    MediaCodecTestContext *ctx = (MediaCodecTestContext *)arg;
    media_codec_context_t *context = ctx->context;
    char *inputFileName = ctx->inputFileName;
    char *outputFileName = ctx->outputFileName;
    media_codec_state_t state = MEDIA_CODEC_STATE_NONE;
    inFile = fopen(inputFileName, "rb");
    if (!inFile) {
        goto ERR;
    }
    outFile = fopen(outputFileName, "wb");
    if (!outFile) {
        goto ERR;
    }
    //get current time
    lastTime = osal_gettime();
    ret = hb_mm_mc_initialize(context);
    if (ret) {
        goto ERR;
    }
    ret = hb_mm_mc_configure(context);
    if (ret) {
        goto ERR;
    }
    mc_av_codec_startup_params_t startup_params;
    startup_params.video_enc_startup_params.receive_frame_number = 0;
    ret = hb_mm_mc_start(context, &startup_params);
    if (ret) {
        goto ERR;
    }
    ret = hb_mm_mc_pause(context);
    if (ret) {
        goto ERR;
    }
    do {
        if (!noMoreInput) {
            media_codec_buffer_t inputBuffer;
            memset(&inputBuffer, 0x00, sizeof(media_codec_buffer_t));
            ret = hb_mm_mc_dequeue_input_buffer(context, &inputBuffer, 100);
            if (!ret) {
                curTime = osal_gettime();
                if ((curTime - lastTime)/1000 < (uint32_t)ctx->duration) {
                    ret = fread(inputBuffer.vframe_buf.vir_ptr[0], 1,
                    inputBuffer.vframe_buf.size, inFile);
                    if (ret <= 0) {
                        if(fseek(inFile, 0, SEEK_SET)) {
                            printf("Failed to rewind input filen");
                        } else {
                        ret = fread(inputBuffer.vframe_buf.vir_ptr[0], 1,
                        inputBuffer.vframe_buf.size, inFile);
                        if (ret <= 0) {
                            printf("Failed to read input filen");
                        }
                }
            }
        } else {
            printf("Time up(%d)n",ctx->duration);
            ret = 0;
        }
        if (!ret) {
            printf("There is no more input data!n");
            inputBuffer.vframe_buf.frame_end = TRUE;
            noMoreInput = 1;
        }
        ret = hb_mm_mc_queue_input_buffer(context, &inputBuffer, 100);
        if (ret) {
            printf("Queue input buffer fail.n");
            break;
        } else {
            if (ret != (int32_t)HB_MEDIA_ERR_WAIT_TIMEOUT) {
                printf("Dequeue input buffer fail.n");
                break;
            }
        }

        if (!lastStream) {
            media_codec_buffer_t outputBuffer;
            media_codec_output_buffer_info_t info;
            memset(&outputBuffer, 0x00, sizeof(media_codec_buffer_t));
            memset(&info, 0x00, sizeof(media_codec_output_buffer_info_t));
            ret = hb_mm_mc_dequeue_output_buffer(context, &outputBuffer, &info,
            3000);
            if (!ret && outFile) {
                fwrite(outputBuffer.vstream_buf.vir_ptr,
                outputBuffer.vstream_buf.size, 1, outFile);
                ret = hb_mm_mc_queue_output_buffer(context, &outputBuffer, 100);
                if (ret) {
                    printf("Queue output buffer fail.n");
                    break;
                }
                if (outputBuffer.vstream_buf.stream_end) {
                    printf("There is no more output data!n");
                    lastStream = 1;
                    break;
                }
            } else {
                if (ret != (int32_t)HB_MEDIA_ERR_WAIT_TIMEOUT) {
                    printf("Dequeue output buffer fail.n");
                    break;
                }
            }
        }
        if (needFlush) {
            ret = hb_mm_mc_flush(context);
            needFlush = 0;
            if (ret) {
                break;
            }
        }
    }while(TRUE);
    hb_mm_mc_stop(context);
    hb_mm_mc_release(context);
    context = NULL;
ERR:
    hb_mm_mc_get_state(context, &state);
    if (context && state !=
        MEDIA_CODEC_STATE_UNINITIALIZED) {
        hb_mm_mc_stop(context);
        hb_mm_mc_release(context);
    }
    if (inFile)
        fclose(inFile);
    if (outFile)
    fclose(outFile);
}
int main(int argc, char *argv[])
{
    hb_s32 ret = 0;
    char outputFileName[MAX_FILE_PATH] = "./tmp.yuv";
    char inputFileName[MAX_FILE_PATH] = "./output.stream";
    mc_video_codec_enc_params_t *params;
    media_codec_context_t context;
    memset(&context, 0x00, sizeof(media_codec_context_t));
    context.codec_id = MEDIA_CODEC_ID_H265;
    context.encoder = TRUE;
    params = &context.video_enc_params;
    params->width = 640;
    params->height = 480;
    params->pix_fmt = MC_PIXEL_FORMAT_YUV420P;
    params->frame_buf_count = 5;
    params->external_frame_buf = FALSE;
    params->bitstream_buf_count = 5;
    params->rc_params.mode = MC_AV_RC_MODE_H265CBR;
    ret = hb_mm_mc_get_rate_control_config(&context, &params->rc_params);
    if (ret) {
        return -1;
    }
    params->rc_params.h265_cbr_params.bit_rate = 5000;
    params->rc_params.h265_cbr_params.frame_rate = 30;
    params->rc_params.h265_cbr_params.intra_period = 30;
    params->gop_params.decoding_refresh_type = 2;
    params->gop_params.gop_preset_idx = 2;
    params->rot_degree = MC_CCW_0;
    params->mir_direction = MC_DIRECTION_NONE;
    params->frame_cropping_flag = FALSE;
    MediaCodecTestContext ctx;
    memset(&ctx, 0x00, sizeof(ctx));
    ctx.context = &context;
    ctx.inputFileName = inputFileName;
    ctx.outputFileName = outputFileName;
    ctx.duration = 5;
    do_sync_encoding(&ctx);
}
```

#### hb\_mm\_mc\_dequeue\_input\_buffer

【函数声明】

hb\_s32 hb\_mm\_mc\_dequeue\_input\_buffer(media\_codec\_context\_t
\*context, media\_codec\_buffer\_t \*buffer, hb\_s32 timeout)

【参数描述】

- \[IN\] media\_codec\_context\_t \*context：指定codec类型的context
- \[IN\] hb\_s32 timeout：超时时间
- \[OUT\] media\_codec\_buffer\_t \*buffer：输入的buffer信息

【返回值】

- 0：操作成功
- HB\_MEDIA\_ERR\_UNKNOWN： 未知错误
- HB\_MEDIA\_ERR\_OPERATION\_NOT\_ALLOWED：操作不允许
- HB\_MEDIA\_ERR\_INVALID\_INSTANCE：无效实例
- HB\_MEDIA\_ERR\_INVALID\_PARAMS：无效参数
- HB\_MEDIA\_ERR\_INVALID\_BUFFER：无效buffer
- HB\_MEDIA\_ERR\_WAIT\_TIMEOUT：等待超时

【功能描述】

获取输入的buffer。

【示例代码】

参考 [hb_mm_mc_queue_input_buffer](#hb_mm_mc_queue_input_buffer)

#### hb\_mm\_mc\_queue\_output\_buffer

【函数声明】

hb\_s32 hb\_mm\_mc\_queue\_output\_buffer(media\_codec\_context\_t
\*context, media\_codec\_buffer\_t \*buffer, hb\_s32 timeout)

【参数描述】

- \[IN\] media\_codec\_context\_t \*context：指定codec类型的context
- \[IN\] media\_codec\_buffer\_t \*buffer：输出的buffer信息
- \[IN\] hb\_s32 timeout：超时时间

【返回值】

- 0：操作成功
- HB\_MEDIA\_ERR\_UNKNOWN： 未知错误
- HB\_MEDIA\_ERR\_OPERATION\_NOT\_ALLOWED：操作不允许
- HB\_MEDIA\_ERR\_INVALID\_INSTANCE：无效实例
- HB\_MEDIA\_ERR\_INVALID\_PARAMS：无效参数
- HB\_MEDIA\_ERR\_INVALID\_BUFFER：无效buffer
- HB\_MEDIA\_ERR\_WAIT\_TIMEOUT：等待超时

【功能描述】

返还处理完的output buffer到MediaCodec中。

【示例代码】

参考 [hb_mm_mc_queue_input_buffer](#hb_mm_mc_queue_input_buffer)

#### hb\_mm\_mc\_dequeue\_output\_buffer

【函数声明】

hb\_s32 hb\_mm\_mc\_dequeue\_output\_buffer(media\_codec\_context\_t
\*context, media\_codec\_buffer\_t \*buffer,
media\_codec\_output\_buffer\_info\_t \*info, hb\_s32 timeout)

【参数描述】

- \[IN\] media\_codec\_context\_t \*context：指定codec类型的context
- \[IN\] hb\_s32 timeout：超时时间
- \[OUT\] media\_codec\_buffer\_t \*buffer：输出的buffer信息
- \[IN\] media\_codec\_output\_buffer\_info\_t
  \*info：输出数据流的信息

【返回值】

- 0：操作成功
- HB\_MEDIA\_ERR\_UNKNOWN： 未知错误
- HB\_MEDIA\_ERR\_OPERATION\_NOT\_ALLOWED：操作不允许
- HB\_MEDIA\_ERR\_INVALID\_INSTANCE：无效实例
- HB\_MEDIA\_ERR\_INVALID\_PARAMS：无效参数
- HB\_MEDIA\_ERR\_INVALID\_BUFFER：无效buffer
- HB\_MEDIA\_ERR\_WAIT\_TIMEOUT：等待超时

【功能描述】

获取输出的buffer。

【示例代码】

参考 [hb_mm_mc_queue_input_buffer](#hb_mm_mc_queue_input_buffer)

#### hb\_mm\_mc\_get\_longterm\_ref\_mode

【函数声明】

hb\_s32 hb\_mm\_mc\_get\_longterm\_ref\_mode(media\_codec\_context\_t
\*context, mc\_video\_longterm\_ref\_mode\_t \*params)

【参数描述】

- \[IN\] media\_codec\_context\_t \*context：指定codec类型的context
- \[OUT\] mc\_video\_longterm\_ref\_mode\_t
  \*params：长期参考帧模式参数

【返回值】

- 0：操作成功
- HB\_MEDIA\_ERR\_UNKNOWN： 未知错误
- HB\_MEDIA\_ERR\_INVALID\_INSTANCE：无效实例
- HB\_MEDIA\_ERR\_INVALID\_PARAMS：无效参数

【功能描述】

获取长期参考帧模式的参数，适用于H264/H265。

【示例代码】

```
#include "hb_media_codec.h"
#include "hb_media_error.h"

typedef enum ENC_CONFIG_MESSAGE {
    ENC_CONFIG_NONE = (0 << 0),
    ENC_CONFIG_LONGTERM_REF = (1 << 0),
    ENC_CONFIG_INTRA_REFRESH = (1 << 1),
    ENC_CONFIG_RATE_CONTROL = (1 << 2),
    ENC_CONFIG_DEBLK_FILTER = (1 << 3),
    ENC_CONFIG_SAO = (1 << 4),
    ENC_CONFIG_ENTROPY = (1 << 5),
    ENC_CONFIG_VUI_TIMING = (1 << 6),
    ENC_CONFIG_SLICE = (1 << 7),
    ENC_CONFIG_REQUEST_IDR = (1 << 8),
    ENC_CONFIG_SKIP_PIC = (1 << 9),
    ENC_CONFIG_SMART_BG = (1 << 10),
    ENC_CONFIG_MONOCHROMA = (1 << 11),
    ENC_CONFIG_PRED_UNIT = (1 << 12),
    ENC_CONFIG_TRANSFORM = (1 << 13),
    ENC_CONFIG_ROI = (1 << 14),
    ENC_CONFIG_MODE_DECISION = (1 << 15),
    ENC_CONFIG_USER_DATA = (1 << 16),
    ENC_CONFIG_MJPEG = (1 << 17),
    ENC_CONFIG_JPEG = (1 << 18),
    ENC_CONFIG_CAMERA = (1 << 19),
    ENC_CONFIG_INSERT_USERDATA = (1 << 20),
    ENC_CONFIG_VUI = (1 << 21),
    ENC_CONFIG_3DNR = (1 << 22),
    ENC_CONFIG_REQUEST_IDR_HEADER = (1 << 23),
    ENC_CONFIG_ENABLE_IDR = (1 << 24),
    ENC_CONFIG_TOTAL = (1 << 25),
} ENC_CONFIG_MESSAGE;

typedef struct MediaCodecTestContext {
    media_codec_context_t *context;
    char *inputFileName;
    char *outputFileName;
    int32_t duration; // s
    ENC_CONFIG_MESSAGE message;
    mc_video_longterm_ref_mode_t ref_mode;
    mc_rate_control_params_t rc_params;
    mc_video_intra_refresh_params_t intra_refr;
    mc_video_deblk_filter_params_t deblk_filter;
    mc_h265_sao_params_t sao;
    mc_h264_entropy_params_t entropy;
    mc_video_vui_params_t vui;
    mc_video_vui_timing_params_t vui_timing;
    mc_video_slice_params_t slice;
    mc_video_3dnr_enc_params_t noise_reduction;
    mc_video_smart_bg_enc_params_t smart_bg;
    mc_video_pred_unit_params_t pred_unit;
    mc_video_transform_params_t transform;
    mc_video_roi_params_t roi;
    mc_video_mode_decision_params_t mode_decision;
} MediaCodecTestContext;

Uint64 osal_gettime(void)
{
    struct timespec tp;

    clock_gettime(CLOCK_MONOTONIC, &tp);

    return ((Uint64)tp.tv_sec*1000 + tp.tv_nsec/1000000);
}
uint8_t uuid[] =
    "dc45e9bd-e6d948b7-962cd820-d923eeef+HorizonAI";
static void set_message(MediaCodecTestContext *ctx) {
    int ret = 0;
    media_codec_context_t *context = ctx->context;

    mc_video_longterm_ref_mode_t *ref_mode = &ctx->ref_mode;
    hb_mm_mc_get_longterm_ref_mode(context, ref_mode);
    ref_mode->use_longterm = TRUE;
    ref_mode->longterm_pic_using_period = 20;
    ref_mode->longterm_pic_period = 30;
    //ctx->message = ENC_CONFIG_LONGTERM_REF;
    if (ctx->message & ENC_CONFIG_LONGTERM_REF) {
        ret = hb_mm_mc_set_longterm_ref_mode(context, &ctx->ref_mode);
    }
if (ctx->message & ENC_CONFIG_INTRA_REFRESH) {
hb_mm_mc_get_intra_refresh_config(context, &ctx->intra_refr)
        ret = hb_mm_mc_set_intra_refresh_config(context, &ctx->intra_refr);
}
if (ctx->message & ENC_CONFIG_SAO) {
hb_mm_mc_get_sao_config(context, &ctx->sao);
        ret = hb_mm_mc_set_sao_config(context, &ctx->sao);
    }

if (ctx->message & ENC_CONFIG_ENTROPY) {
hb_mm_mc_get_entropy_config(context, &ctx->entropy);
        ret = hb_mm_mc_set_entropy_config(context, &ctx->entropy);
    }

if (ctx->message & ENC_CONFIG_VUI) {
hb_mm_mc_get_vui_config(context, &ctx->vui);
    ret = hb_mm_mc_set_vui_config(context, &ctx->vui);
    }

if (ctx->message & ENC_CONFIG_VUI_TIMING) {
hb_mm_mc_get_vui_timing_config(context, &ctx->vui_timing);
        ret = hb_mm_mc_set_vui_timing_config(context, &ctx->vui_timing);
    }
    mc_rate_control_params_t *rc_params = &ctx->rc_params;
    rc_params->mode = context->video_enc_params.rc_params.mode;
    hb_mm_mc_get_rate_control_config(context, rc_params);
    switch (rc_params->mode) {
    case MC_AV_RC_MODE_H264CBR:
        rc_params->h264_cbr_params.bit_rate = 5000;
        rc_params->h264_cbr_params.intra_period = 60;
        break;
    case MC_AV_RC_MODE_H264VBR:
        rc_params->h264_vbr_params.intra_qp = 20;
        rc_params->h264_vbr_params.intra_period = 30;
        break;
    case MC_AV_RC_MODE_H264AVBR:
        rc_params->h264_avbr_params.intra_period = 15;
        rc_params->h264_avbr_params.intra_qp = 25;
        rc_params->h264_avbr_params.bit_rate = 2000;
        rc_params->h264_avbr_params.vbv_buffer_size = 3000;
        rc_params->h264_avbr_params.min_qp_I = 15;
        rc_params->h264_avbr_params.max_qp_I = 50;
        rc_params->h264_avbr_params.min_qp_P = 15;
        rc_params->h264_avbr_params.max_qp_P = 45;
        rc_params->h264_avbr_params.min_qp_B = 15;
        rc_params->h264_avbr_params.max_qp_B = 48;
        rc_params->h264_avbr_params.hvs_qp_enable = 0;
        rc_params->h264_avbr_params.hvs_qp_scale = 2;
        rc_params->h264_avbr_params.max_delta_qp = 5;
        rc_params->h264_avbr_params.qp_map_enable = 0;
        break;
    case MC_AV_RC_MODE_H264FIXQP:
        rc_params->h264_fixqp_params.force_qp_I = 23;
        rc_params->h264_fixqp_params.force_qp_P = 23;
        rc_params->h264_fixqp_params.force_qp_B = 23;
        rc_params->h264_fixqp_params.intra_period = 23;
        break;
    case MC_AV_RC_MODE_H264QPMAP:
        break;
    case MC_AV_RC_MODE_H265CBR:
        rc_params->h265_cbr_params.bit_rate = 5000;
        rc_params->h265_cbr_params.intra_period = 60;
        break;
    case MC_AV_RC_MODE_H265VBR:
        rc_params->h265_vbr_params.intra_qp = 20;
        rc_params->h265_vbr_params.intra_period = 30;
        break;
    case MC_AV_RC_MODE_H265AVBR:
        rc_params->h265_avbr_params.intra_period = 15;
        rc_params->h265_avbr_params.intra_qp = 25;
        rc_params->h265_avbr_params.bit_rate = 2000;
        rc_params->h265_avbr_params.vbv_buffer_size = 3000;
        rc_params->h265_avbr_params.min_qp_I = 15;
        rc_params->h265_avbr_params.max_qp_I = 50;
        rc_params->h265_avbr_params.min_qp_P = 15;
        rc_params->h265_avbr_params.max_qp_P = 45;
        rc_params->h265_avbr_params.min_qp_B = 15;
        rc_params->h265_avbr_params.max_qp_B = 48;
        rc_params->h265_avbr_params.hvs_qp_enable = 0;
        rc_params->h265_avbr_params.hvs_qp_scale = 2;
        rc_params->h265_avbr_params.max_delta_qp = 5;
        rc_params->h265_avbr_params.qp_map_enable = 0;
        break;
    case MC_AV_RC_MODE_H265FIXQP:
        rc_params->h265_fixqp_params.force_qp_I = 23;
        rc_params->h265_fixqp_params.force_qp_P = 23;
        rc_params->h265_fixqp_params.force_qp_B = 23;
        rc_params->h265_fixqp_params.intra_period = 23;
        break;
    case MC_AV_RC_MODE_H265QPMAP:
        break;
    default:
        break;
    }
    //ctx->message = ENC_CONFIG_RATE_CONTROL;
    if (ctx->message & ENC_CONFIG_RATE_CONTROL) {
        ret = hb_mm_mc_set_rate_control_config(context, &ctx->rc_params);
    }

    mc_video_deblk_filter_params_t *deblk_filter = &ctx->deblk_filter;
    hb_mm_mc_get_deblk_filter_config(context, deblk_filter);
    if (context->codec_id == MEDIA_CODEC_ID_H264) {
        deblk_filter->h264_deblk.disable_deblocking_filter_idc = 2;
        deblk_filter->h264_deblk.slice_alpha_c0_offset_div2 = 6;
        deblk_filter->h264_deblk.slice_beta_offset_div2 = 6;
    } else {
        deblk_filter->h265_deblk.slice_deblocking_filter_disabled_flag = 1;
        deblk_filter->h265_deblk.slice_beta_offset_div2 = 6;
        deblk_filter->h265_deblk.slice_tc_offset_div2 = 6;
        deblk_filter->h265_deblk.slice_loop_filter_across_slices_enabled_flag = 1;
    }
    //ctx->message = ENC_CONFIG_DEBLK_FILTER;
    if (ctx->message & ENC_CONFIG_DEBLK_FILTER) {
        ret = hb_mm_mc_set_deblk_filter_config(context, &ctx->deblk_filter);
    }

    if (context->codec_id == MEDIA_CODEC_ID_H264) {
        mc_h264_entropy_params_t *entropy = &ctx->entropy;
        hb_mm_mc_get_entropy_config(context, entropy);
        entropy->entropy_coding_mode = 0;
        ctx->message = ENC_CONFIG_ENTROPY;
        if (ctx->message & ENC_CONFIG_ENTROPY) {
            ret = hb_mm_mc_set_entropy_config(context, &ctx->entropy);
        }
    }

    //ctx->message = ENC_CONFIG_SKIP_PIC;
    if (ctx->message & ENC_CONFIG_SKIP_PIC) {
        ret = hb_mm_mc_skip_pic(context, 0), (int32_t)0);
    }

    //ctx->message = ENC_CONFIG_REQUEST_IDR;
    if (ctx->message & ENC_CONFIG_REQUEST_IDR) {
        ret = hb_mm_mc_request_idr_frame(context);
    }

    mc_video_slice_params_t *slice = &ctx->slice;
    hb_mm_mc_get_slice_config(context, slice);
    if (context->codec_id == MEDIA_CODEC_ID_H264) {
        slice->h264_slice.h264_slice_mode = 0;
        slice->h264_slice.h264_slice_arg = 60;
    } else {
        slice->h265_slice.h265_dependent_slice_mode = 0;
        slice->h265_slice.h265_dependent_slice_arg = 80;
        slice->h265_slice.h265_independent_slice_mode = 1;
        slice->h265_slice.h265_independent_slice_arg = 100;
    }
    //ctx->message = ENC_CONFIG_SLICE;
    if (ctx->message & ENC_CONFIG_SLICE) {
        ret = hb_mm_mc_set_slice_config(context, &ctx->slice);
    }

    mc_video_smart_bg_enc_params_t *smart_bg = &ctx->smart_bg;
    hb_mm_mc_get_smart_bg_enc_config(context, smart_bg);
    smart_bg->bg_detect_enable = 0;
    smart_bg->bg_threshold_diff = 8;
    smart_bg->bg_threshold_mean_diff = 1;
    smart_bg->bg_lambda_qp = 32;
    smart_bg->bg_delta_qp = 3;
    smart_bg->s2fme_disable = 0;
    //ctx->message = ENC_CONFIG_SMART_BG;
    if (ctx->message & ENC_CONFIG_SMART_BG) {
        ret = hb_mm_mc_set_smart_bg_enc_config(context, &ctx->smart_bg);
    }

    mc_video_pred_unit_params_t *pred_unit = &ctx->pred_unit;
    hb_mm_mc_get_pred_unit_config(context, pred_unit);
    if (context->codec_id == MEDIA_CODEC_ID_H264) {
        pred_unit->h264_intra_pred.constrained_intra_pred_flag = 1;
    } else {
        pred_unit->h265_pred_unit.intra_nxn_enable = 1;
        pred_unit->h265_pred_unit.constrained_intra_pred_flag = 1;
        pred_unit->h265_pred_unit.strong_intra_smoothing_enabled_flag = 0;
        pred_unit->h265_pred_unit.max_num_merge = 2;
    }
    //ctx->message = ENC_CONFIG_PRED_UNIT;
    if (ctx->message & ENC_CONFIG_PRED_UNIT) {
        ret = hb_mm_mc_set_pred_unit_config(context, &ctx->pred_unit);
    }

    mc_video_transform_params_t *transform = &ctx->transform;
    hb_mm_mc_get_transform_config(context, transform);
    if (context->codec_id == MEDIA_CODEC_ID_H264) {
        transform->h264_transform.transform_8x8_enable = 1;
        transform->h264_transform.chroma_cb_qp_offset = 4;
        transform->h264_transform.chroma_cr_qp_offset = 3;
        transform->h264_transform.user_scaling_list_enable = 0;
    } else {
        transform->h265_transform.chroma_cb_qp_offset = 6;
        transform->h265_transform.chroma_cr_qp_offset = 5;
        transform->h265_transform.user_scaling_list_enable = 0;
    }
    //ctx->message = ENC_CONFIG_TRANSFORM;
    if (ctx->message & ENC_CONFIG_TRANSFORM) {
        ret = hb_mm_mc_set_transform_config(context, &ctx->transform);
    }

    mc_video_roi_params_t *roi = &ctx->roi;
    hb_mm_mc_get_roi_config(context, roi);
    roi->roi_enable = 0;
    //ctx->message = ENC_CONFIG_ROI;
    if (ctx->message & ENC_CONFIG_ROI) {
        ret = hb_mm_mc_set_roi_config(context, &ctx->roi);
    }

    mc_video_mode_decision_params_t *mode_decision = &ctx->mode_decision;
    hb_mm_mc_get_mode_decision_config(context, mode_decision);
    mode_decision->mode_decision_enable = FALSE;
    mode_decision->pu04_delta_rate = 76;
    mode_decision->pu08_delta_rate = 80;
    mode_decision->pu16_delta_rate = 86;
    mode_decision->pu32_delta_rate = 87;
    mode_decision->pu04_intra_planar_delta_rate = 0;
    mode_decision->pu04_intra_dc_delta_rate = 0;
    mode_decision->pu04_intra_angle_delta_rate = 0;
    mode_decision->pu08_intra_planar_delta_rate = 0;
    mode_decision->pu08_intra_dc_delta_rate = 0;
    mode_decision->pu08_intra_angle_delta_rate = 0;
    mode_decision->pu16_intra_planar_delta_rate = 0;
    mode_decision->pu16_intra_dc_delta_rate = 0;
    mode_decision->pu16_intra_angle_delta_rate = 0;
    mode_decision->pu32_intra_planar_delta_rate = 0;
    mode_decision->pu32_intra_dc_delta_rate = 0;
    mode_decision->pu32_intra_angle_delta_rate = 0;
    mode_decision->cu08_intra_delta_rate = 0;
    mode_decision->cu08_inter_delta_rate = 0;
    mode_decision->cu08_merge_delta_rate = 0;
    mode_decision->cu16_intra_delta_rate = 0;
    mode_decision->cu16_inter_delta_rate = 0;
    mode_decision->cu16_merge_delta_rate = 0;
    mode_decision->cu32_intra_delta_rate = 0;
    mode_decision->cu32_inter_delta_rate = 0;
    mode_decision->cu32_merge_delta_rate = 0;
    //ctx->message = ENC_CONFIG_MODE_DECISION;
    if (ctx->message & ENC_CONFIG_MODE_DECISION) {
        ret = hb_mm_mc_set_mode_decision_config(context, &ctx->mode_decision);
}
    if (ctx->message & ENC_CONFIG_INSERT_USERDATA) {
        hb_u32 length = sizeof(uuid)/sizeof(uuid[0]);
        ret = hb_mm_mc_insert_user_data(context, uuid, length);
}
if (ctx->message & ENC_CONFIG_3DNR) {
hb_mm_mc_get_3dnr_enc_config(context, &ctx->noise_reduction);
        ret = hb_mm_mc_set_3dnr_enc_config(context, &ctx->noise_reduction);
}
    if (ctx->message & ENC_CONFIG_ENABLE_IDR) {
        // disable idr frame first
        if (ctx->enable_idr_num) {
            ret = hb_mm_mc_enable_idr_frame(context, 0);
        }
    }

    if (ctx->message & ENC_CONFIG_REQUEST_IDR_HEADER) {
        ret = hb_mm_mc_request_idr_header(context, ctx->force_idr_header);
    }
}

static void do_sync_encoding(void *arg) {
    hb_s32 ret = 0;
    FILE *inFile;
    FILE *outFile;
    int noMoreInput = 0;
    int lastStream = 0;
    Uint64 lastTime = 0;
    Uint64 curTime = 0;
    int needFlush = 1;
    MediaCodecTestContext *ctx = (MediaCodecTestContext *)arg;
    media_codec_context_t *context = ctx->context;
    char *inputFileName = ctx->inputFileName;
    char *outputFileName = ctx->outputFileName;
    media_codec_state_t state = MEDIA_CODEC_STATE_NONE;
    inFile = fopen(inputFileName, "rb");
    if (!inFile) {
        goto ERR;
    }
    outFile = fopen(outputFileName, "wb");
    if (!outFile) {
        goto ERR;
    }

    //get current time
    lastTime = osal_gettime();

    ret = hb_mm_mc_initialize(context);
    if (ret) {
        goto ERR;
    }

    ret = hb_mm_mc_configure(context);
    if (ret) {
        goto ERR;
    }

    mc_av_codec_startup_params_t startup_params;
    startup_params.video_enc_startup_params.receive_frame_number = 0;
    ret = hb_mm_mc_start(context, &startup_params);
    if (ret) {
        goto ERR;
    }

    ret = hb_mm_mc_pause(context);
    if (ret) {
        goto ERR;
    }

    do {
        set_message(ctx);
        if (!noMoreInput) {
            media_codec_buffer_t inputBuffer;
            memset(&inputBuffer, 0x00, sizeof(media_codec_buffer_t));
            ret = hb_mm_mc_dequeue_input_buffer(context, &inputBuffer, 100);
            if (!ret) {
                curTime = osal_gettime();
                if ((curTime - lastTime)/1000 < (uint32_t)ctx->duration) {
                    ret = fread(inputBuffer.vframe_buf.vir_ptr[0], 1,
                        inputBuffer.vframe_buf.size, inFile);
                    if (ret <= 0) {
                        if(fseek(inFile, 0, SEEK_SET)) {
                            printf("Failed to rewind input file\n");
                        } else {
                            ret = fread(inputBuffer.vframe_buf.vir_ptr[0], 1,
                                inputBuffer.vframe_buf.size, inFile);
                            if (ret <= 0) {
                                printf("Failed to read input file\n");
                            }
                        }
                    }
                } else {
                    printf("Time up(%d)\n",ctx->duration);
                    ret = 0;
                }
                if (!ret) {
                    printf("There is no more input data!\n");
                    inputBuffer.vframe_buf.frame_end = TRUE;
                    noMoreInput = 1;
                } 
                ret = hb_mm_mc_queue_input_buffer(context, &inputBuffer, 100);
                if (ret) {
                    printf("Queue input buffer fail.\n");
                    break;
                }
            } else {
                if (ret != (int32_t)HB_MEDIA_ERR_WAIT_TIMEOUT) {
                    printf("Dequeue input buffer fail.\n");
                    break;
                }
            }
        }

        if (!lastStream) {
            media_codec_buffer_t outputBuffer;
            media_codec_output_buffer_info_t info;
            memset(&outputBuffer, 0x00, sizeof(media_codec_buffer_t));
            memset(&info, 0x00, sizeof(media_codec_output_buffer_info_t));
            ret = hb_mm_mc_dequeue_output_buffer(context, &outputBuffer, &info, 3000);
            if (!ret && outFile) {
                fwrite(outputBuffer.vstream_buf.vir_ptr, outputBuffer.vstream_buf.size, 1, outFile);

                ret = hb_mm_mc_queue_output_buffer(context, &outputBuffer, 100);
                if (ret) {
                    printf("Queue output buffer fail.\n");
                    break;
                }
                if (outputBuffer.vstream_buf.stream_end) {
                    printf("There is no more output data!\n");
                    lastStream = 1;
                    break;
                }
            } else {
                if (ret != (int32_t)HB_MEDIA_ERR_WAIT_TIMEOUT) {
                    printf("Dequeue output buffer fail.\n");
                    break;
                }
            }
        }
        if (needFlush) {
            ret = hb_mm_mc_flush(context);
            needFlush = 0;
            if (ret) {
                break;
            }
        }
    }while(TRUE);

    hb_mm_mc_stop(context);

    hb_mm_mc_release(context);
    context = NULL;

ERR:
    hb_mm_mc_get_state(context, &state);
    if (context && state != MEDIA_CODEC_STATE_UNINITIALIZED) {
        hb_mm_mc_stop(context);
        hb_mm_mc_release(context);
    }

    if (inFile)
        fclose(inFile);

    if (outFile)
        fclose(outFile);
}

int main(int argc, char *argv[])
{
    hb_s32 ret = 0;
    char outputFileName[MAX_FILE_PATH] = "./tmp.yuv";
    char inputFileName[MAX_FILE_PATH] = "./output.stream";
    mc_video_codec_enc_params_t *params;
    media_codec_context_t context;

    memset(&context, 0x00, sizeof(media_codec_context_t));
    context.codec_id = MEDIA_CODEC_ID_H265;
    context.encoder = TRUE;
    params = &context.video_enc_params;
    params->width = 640;
    params->height = 480;
    params->pix_fmt = MC_PIXEL_FORMAT_YUV420P;
    params->frame_buf_count = 5;
    params->external_frame_buf = FALSE;
    params->bitstream_buf_count = 5;
    params->rc_params.mode = MC_AV_RC_MODE_H265CBR;
    ret = hb_mm_mc_get_rate_control_config(&context, &params->rc_params);
    if (ret) {
        return -1;
    }
    params->rc_params.h265_cbr_params.bit_rate = 5000;
    params->rc_params.h265_cbr_params.frame_rate = 30;
    params->rc_params.h265_cbr_params.intra_period = 30;
    params->gop_params.decoding_refresh_type = 2;
    params->gop_params.gop_preset_idx = 2;
    params->rot_degree = MC_CCW_0;
    params->mir_direction = MC_DIRECTION_NONE;
    params->frame_cropping_flag = FALSE;

    MediaCodecTestContext ctx;
    memset(&ctx, 0x00, sizeof(ctx));
    ctx.context = &context;
    ctx.inputFileName = inputFileName;
    ctx.outputFileName = outputFileName;
    ctx.duration = 5;
    do_sync_encoding(&ctx);
}
```

#### hb\_mm\_mc\_set\_longterm\_ref\_mode

【函数声明】

hb\_s32 hb\_mm\_mc\_set\_longterm\_ref\_mode(media\_codec\_context\_t
\*context, const mc\_video\_longterm\_ref\_mode\_t \*params)

【参数描述】

- \[IN\] media\_codec\_context\_t \*context：指定codec类型的context
- \[IN\] const mc\_video\_longterm\_ref\_mode\_t
  \*params：长期参考帧模式参数

【返回值】

- 0：操作成功
- HB\_MEDIA\_ERR\_UNKNOWN： 未知错误
- HB\_MEDIA\_ERR\_OPERATION\_NOT\_ALLOWED：操作不允许
- HB\_MEDIA\_ERR\_INVALID\_INSTANCE：无效实例
- HB\_MEDIA\_ERR\_INVALID\_PARAMS：无效参数

【功能描述】

设置长期参考帧模式的参数，该参数为动态参数，适用于H264/H265。

【示例代码】

参考 [hb_mm_mc_get_longterm_ref_mode](#hb_mm_mc_get_longterm_ref_mode)

#### hb\_mm\_mc\_get\_intra\_refresh\_config

【函数声明】

hb\_s32 hb\_mm\_mc\_get\_intra\_refresh\_config(media\_codec\_context\_t
\*context, mc\_video\_intra\_refresh\_params\_t \*params)

【参数描述】

- \[IN\] media\_codec\_context\_t \*context：指定codec类型的context
- \[OUT\] mc\_video\_intra\_refresh\_params\_t \*params：帧内刷新参数

【返回值】

- 0：操作成功
- HB\_MEDIA\_ERR\_UNKNOWN： 未知错误
- HB\_MEDIA\_ERR\_INVALID\_INSTANCE：无效实例
- HB\_MEDIA\_ERR\_INVALID\_PARAMS：无效参数

【功能描述】

获取帧内刷新参数，适用于H264/H265。

【示例代码】

参考 [hb_mm_mc_get_longterm_ref_mode](#hb_mm_mc_get_longterm_ref_mode)

#### hb\_mm\_mc\_set\_intra\_refresh\_config

【函数声明】

hb\_s32 hb\_mm\_mc\_set\_intra\_refresh\_config(media\_codec\_context\_t
\*context, const mc\_video\_intra\_refresh\_params\_t \*params)

【参数描述】

- \[IN\] media\_codec\_context\_t \*context：指定codec类型的context
- \[IN\] const mc\_video\_intra\_refresh\_params\_t
  \*params：帧内刷新参数

【返回值】

- 0：操作成功
- HB\_MEDIA\_ERR\_UNKNOWN： 未知错误
- HB\_MEDIA\_ERR\_OPERATION\_NOT\_ALLOWED：操作不允许
- HB\_MEDIA\_ERR\_INVALID\_INSTANCE：无效实例
- HB\_MEDIA\_ERR\_INVALID\_PARAMS：无效参数

【功能描述】

设置帧内刷新模式参数，该参数为静态参数，适用于H264/H265。

【示例代码】

参考 [hb_mm_mc_get_longterm_ref_mode](#hb_mm_mc_get_longterm_ref_mode)

#### hb\_mm\_mc\_get\_rate\_control\_config

【函数声明】

hb\_s32 hb\_mm\_mc\_get\_rate\_control\_config(media\_codec\_context\_t
\*context, mc\_rate\_control\_params\_t \*params)

【参数描述】

- \[IN\] media\_codec\_context\_t \*context：指定codec类型的context
- \[OUT\] mc\_rate\_control\_params\_t \*params：码率控制参数

【返回值】

- 0：操作成功
- HB\_MEDIA\_ERR\_UNKNOWN： 未知错误
- HB\_MEDIA\_ERR\_INVALID\_INSTANCE：无效实例
- HB\_MEDIA\_ERR\_INVALID\_PARAMS：无效参数

【功能描述】

获取码率控制参数，该参数为动态参数，适用于H264/H265/MJPEG。

【示例代码】

参考 [hb_mm_mc_get_longterm_ref_mode](#hb_mm_mc_get_longterm_ref_mode)

#### hb\_mm\_mc\_set\_rate\_control\_config

【函数声明】

hb\_s32 hb\_mm\_mc\_set\_rate\_control\_config(media\_codec\_context\_t
\*context, const mc\_rate\_control\_params\_t \*params)

【参数描述】

- \[IN\] media\_codec\_context\_t \*context：指定codec类型的context
- \[IN\] const mc\_rate\_control\_params\_t \*params：码率控制参数

【返回值】

- 0：操作成功
- HB\_MEDIA\_ERR\_UNKNOWN： 未知错误
- HB\_MEDIA\_ERR\_OPERATION\_NOT\_ALLOWED：操作不允许
- HB\_MEDIA\_ERR\_INVALID\_INSTANCE：无效实例
- HB\_MEDIA\_ERR\_INVALID\_PARAMS：无效参数

【功能描述】

设置码率控制参数，该参数为动态参数，适用于H264/H265/MJPEG。

【示例代码】

参考 [hb_mm_mc_get_longterm_ref_mode](#hb_mm_mc_get_longterm_ref_mode)

#### hb\_mm\_mc\_get\_deblk\_filter\_config

【函数声明】

hb\_s32 hb\_mm\_mc\_get\_deblk\_filter\_config(media\_codec\_context\_t
\*context, mc\_video\_deblk\_filter\_params\_t \*params)

【参数描述】

- \[IN\] media\_codec\_context\_t \*context：指定codec类型的context
- \[OUT\] mc\_video\_deblk\_filter\_params\_t \* params：去块滤波参数

【返回值】

- 0：操作成功
- HB\_MEDIA\_ERR\_UNKNOWN： 未知错误
- HB\_MEDIA\_ERR\_INVALID\_INSTANCE：无效实例
- HB\_MEDIA\_ERR\_INVALID\_PARAMS：无效参数

【功能描述】

获取去块滤波参数，适用于H264/H265。

【示例代码】

参考 [hb_mm_mc_get_longterm_ref_mode](#hb_mm_mc_get_longterm_ref_mode)

#### hb\_mm\_mc\_set\_deblk\_filter\_config

【函数声明】

hb\_s32 hb\_mm\_mc\_set\_deblk\_filter\_config(media\_codec\_context\_t
\*context, const mc\_video\_deblk\_filter\_params\_t \*params)

【参数描述】

- \[IN\] media\_codec\_context\_t \*context：指定codec类型的context
- \[IN\] const mc\_video\_deblk\_filter\_params\_t
  \*params：去块滤波参数

【返回值】

- 0：操作成功
- HB\_MEDIA\_ERR\_UNKNOWN： 未知错误
- HB\_MEDIA\_ERR\_OPERATION\_NOT\_ALLOWED：操作不允许
- HB\_MEDIA\_ERR\_INVALID\_INSTANCE：无效实例
- HB\_MEDIA\_ERR\_INVALID\_PARAMS：无效参数

【功能描述】

设置去块滤波参数，该参数为动态参数，适用于H264/H265。

【示例代码】

参考 [hb_mm_mc_get_longterm_ref_mode](#hb_mm_mc_get_longterm_ref_mode)

#### hb\_mm\_mc\_get\_sao\_config

【函数声明】

hb\_s32 hb\_mm\_mc\_get\_sao\_config(media\_codec\_context\_t \*context,
mc\_h265\_sao\_params\_t \*params)

【参数描述】

- \[IN\] media\_codec\_context\_t \*context：指定codec类型的context
- \[OUT\] mc\_h265\_sao\_params\_t \*params：SAO参数

【返回值】

- 0：操作成功
- HB\_MEDIA\_ERR\_UNKNOWN： 未知错误
- HB\_MEDIA\_ERR\_INVALID\_INSTANCE：无效实例
- HB\_MEDIA\_ERR\_INVALID\_PARAMS：无效参数

【功能描述】

获取SAO参数，适用于H265。

【示例代码】

参考 [hb_mm_mc_get_longterm_ref_mode](#hb_mm_mc_get_longterm_ref_mode)

#### hb\_mm\_mc\_set\_sao\_config

【函数声明】

hb\_s32 hb\_mm\_mc\_set\_sao\_config(media\_codec\_context\_t \*context,
const mc\_h265\_sao\_params\_t \*params)

【参数描述】

- \[IN\] media\_codec\_context\_t \*context：指定codec类型的context
- \[IN\] const mc\_h265\_sao\_params\_t \*params：SAO参数

【返回值】

- 0：操作成功
- HB\_MEDIA\_ERR\_UNKNOWN： 未知错误
- HB\_MEDIA\_ERR\_OPERATION\_NOT\_ALLOWED：操作不允许
- HB\_MEDIA\_ERR\_INVALID\_INSTANCE：无效实例
- HB\_MEDIA\_ERR\_INVALID\_PARAMS：无效参数

【功能描述】

设置SAO参数，该参数为静态参数，适用于H265。

【示例代码】

参考 [hb_mm_mc_get_longterm_ref_mode](#hb_mm_mc_get_longterm_ref_mode)

#### hb\_mm\_mc\_get\_entropy\_config

【函数声明】

hb\_s32 hb\_mm\_mc\_get\_entropy\_config(media\_codec\_context\_t
\*context, mc\_h264\_entropy\_params\_t \*params);

【参数描述】

- \[IN\] media\_codec\_context\_t \*context：指定codec类型的context
- \[IN\] mc\_h264\_entropy\_params\_t \*params：entropy参数

【返回值】

- 0：操作成功
- HB\_MEDIA\_ERR\_UNKNOWN： 未知错误
- HB\_MEDIA\_ERR\_OPERATION\_NOT\_ALLOWED：操作不允许
- HB\_MEDIA\_ERR\_INVALID\_INSTANCE：无效实例
- HB\_MEDIA\_ERR\_INVALID\_PARAMS：无效参数

【功能描述】

获取entropy参数，适用于H264。

【示例代码】

参考 [hb_mm_mc_get_longterm_ref_mode](#hb_mm_mc_get_longterm_ref_mode)

#### hb\_mm\_mc\_set\_entropy\_config

【函数声明】

extern hb\_s32 hb\_mm\_mc\_set\_entropy\_config(media\_codec\_context\_t
\*context, const mc\_h264\_entropy\_params\_t \*params);

【参数描述】

- \[IN\] media\_codec\_context\_t \*context：指定codec类型的context
- \[IN\] const mc\_h264\_entropy\_params\_t \*params：entropy数

【返回值】

- 0：操作成功
- HB\_MEDIA\_ERR\_UNKNOWN： 未知错误
- HB\_MEDIA\_ERR\_OPERATION\_NOT\_ALLOWED：操作不允许
- HB\_MEDIA\_ERR\_INVALID\_INSTANCE：无效实例
- HB\_MEDIA\_ERR\_INVALID\_PARAMS：无效参数

【功能描述】

设置entropy参数，适用于H264。

【示例代码】

参考 [hb_mm_mc_get_longterm_ref_mode](#hb_mm_mc_get_longterm_ref_mode)

#### hb\_mm\_mc\_get\_vui\_timing\_config

【函数声明】

hb\_s32 hb\_mm\_mc\_get\_vui\_timing\_config(media\_codec\_context\_t
\*context, mc\_video\_vui\_timing\_params\_t \*params)

【参数描述】

- \[IN\] media\_codec\_context\_t \*context：指定codec类型的context
- \[OUT\] mc\_video\_vui\_timing\_params\_t \*params：VUI Timing参数

【返回值】

- 0：操作成功
- HB\_MEDIA\_ERR\_UNKNOWN： 未知错误
- HB\_MEDIA\_ERR\_INVALID\_INSTANCE：无效实例
- HB\_MEDIA\_ERR\_INVALID\_PARAMS：无效参数

【功能描述】

获取VUI Timing参数，适用于H264/H265。

【示例代码】

参考 [hb_mm_mc_get_longterm_ref_mode](#hb_mm_mc_get_longterm_ref_mode)

#### hb\_mm\_mc\_set\_vui\_timing\_config

【函数声明】

hb\_s32 hb\_mm\_mc\_set\_vui\_timing\_config(media\_codec\_context\_t
\*context, const mc\_video\_vui\_timing\_params\_t \*params)

【参数描述】

- \[IN\] media\_codec\_context\_t \*context：指定codec类型的context
- \[IN\] const mc\_video\_vui\_timing\_params\_t \*params：VUI
  Timing参数

【返回值】

- 0：操作成功
- HB\_MEDIA\_ERR\_UNKNOWN： 未知错误
- HB\_MEDIA\_ERR\_OPERATION\_NOT\_ALLOWED：操作不允许
- HB\_MEDIA\_ERR\_INVALID\_INSTANCE：无效实例
- HB\_MEDIA\_ERR\_INVALID\_PARAMS：无效参数

【功能描述】

设置VUI Timing参数，该参数为静态参数，适用于H264/H265。

【示例代码】

参考 [hb_mm_mc_get_longterm_ref_mode](#hb_mm_mc_get_longterm_ref_mode)

#### hb\_mm\_mc\_get\_slice\_config

【函数声明】

hb\_s32 hb\_mm\_mc\_get\_slice\_config(media\_codec\_context\_t
\*context, mc\_video\_slice\_params\_t \*params)

【参数描述】

- \[IN\] media\_codec\_context\_t \*context：指定codec类型的context
- \[OUT\] mc\_video\_slice\_params\_t \*params：slice编码参数

【返回值】

- 0：操作成功
- HB\_MEDIA\_ERR\_UNKNOWN： 未知错误
- HB\_MEDIA\_ERR\_INVALID\_INSTANCE：无效实例
- HB\_MEDIA\_ERR\_INVALID\_PARAMS：无效参数

【功能描述】

获取slice编码参数，适用于H264/H265。

【示例代码】

参考 [hb_mm_mc_get_longterm_ref_mode](#hb_mm_mc_get_longterm_ref_mode)

#### hb\_mm\_mc\_set\_slice\_config

【函数声明】

hb\_s32 hb\_mm\_mc\_set\_slice\_config(media\_codec\_context\_t
\*context, const mc\_video\_slice\_params\_t \*params)

【参数描述】

- \[IN\] media\_codec\_context\_t \*context：指定codec类型的context
- \[IN\] const mc\_video\_slice\_params\_t \*params：slice编码参数

【返回值】

- 0：操作成功
- HB\_MEDIA\_ERR\_UNKNOWN： 未知错误
- HB\_MEDIA\_ERR\_OPERATION\_NOT\_ALLOWED：操作不允许
- HB\_MEDIA\_ERR\_INVALID\_INSTANCE：无效实例
- HB\_MEDIA\_ERR\_INVALID\_PARAMS：无效参数

【功能描述】

设置slice编码参数，该参数为动态参数，适用于H264/H265。限制每帧slice个数小于等于1500。

【示例代码】

参考 [hb_mm_mc_get_longterm_ref_mode](#hb_mm_mc_get_longterm_ref_mode)

#### hb\_mm\_mc\_insert\_user\_data

【函数声明】

hb\_s32 hb\_mm\_mc\_insert\_user\_data(media\_codec\_context\_t \*
context, hb\_u8 \*data, hb\_u32 length)

【参数描述】

- \[IN\] media\_codec\_context\_t \*context：指定codec类型的context
- \[IN\] hb\_u8 \*data：用户数据
- \[IN\] hb\_u32 length：用户数据长度

【返回值】

- 0：操作成功
- HB\_MEDIA\_ERR\_UNKNOWN： 未知错误
- HB\_MEDIA\_ERR\_INVALID\_INSTANCE：无效实例
- HB\_MEDIA\_ERR\_INVALID\_PARAMS：无效参数

【功能描述】

在编码流中插入用户数据，该参数为动态参数，适用于H264/H265/MJPG/JPG。

【示例代码】

参考 [hb_mm_mc_get_longterm_ref_mode](#hb_mm_mc_get_longterm_ref_mode)

#### hb\_mm\_mc\_request\_idr\_frame

【函数声明】

hb\_s32 hb\_mm\_mc\_request\_idr\_frame(media\_codec\_context\_t
\*context)

【参数描述】

- \[IN\] media\_codec\_context\_t \*context：指定codec类型的context

【返回值】

- 0：操作成功
- HB\_MEDIA\_ERR\_UNKNOWN： 未知错误
- HB\_MEDIA\_ERR\_OPERATION\_NOT\_ALLOWED：操作不允许
- HB\_MEDIA\_ERR\_INVALID\_INSTANCE：无效实例

【功能描述】

请求IDR帧，该接口可动态设置，适用于H264/H265。

【示例代码】

参考 [hb_mm_mc_get_longterm_ref_mode](#hb_mm_mc_get_longterm_ref_mode)

#### hb\_mm\_mc\_skip\_pic

【函数声明】

hb\_s32 hb\_mm\_mc\_skip\_pic(media\_codec\_context\_t
\*context，hb\_s32 src\_idx)

【参数描述】

- \[IN\] media\_codec\_context\_t \*context：指定codec类型的context
- \[IN\] hb\_s32 src\_idx：source buffer索引值

【返回值】

- 0：操作成功
- HB\_MEDIA\_ERR\_UNKNOWN： 未知错误
- HB\_MEDIA\_ERR\_OPERATION\_NOT\_ALLOWED：操作不允许
- HB\_MEDIA\_ERR\_INVALID\_INSTANCE：无效实例
- HB\_MEDIA\_ERR\_INVALID\_PARAMS：无效参数

【功能描述】

使能指定的图像的skip模式编码，该接口可动态设置，适用于H264/H265。

【示例代码】

参考 [hb_mm_mc_get_longterm_ref_mode](#hb_mm_mc_get_longterm_ref_mode)

#### hb\_mm\_mc\_get\_smart\_bg\_enc\_config

【函数声明】

extern hb\_s32
hb\_mm\_mc\_get\_smart\_bg\_enc\_config(media\_codec\_context\_t
\*context, mc\_video\_smart\_bg\_enc\_params\_t \*params);

【参数描述】

- \[IN\] media\_codec\_context\_t \*context：指定codec类型的context
- \[OUT\] mc\_video\_smart\_bg\_enc\_params\_t
  \*params：智能背景编码模式参数

【返回值】

- 0：操作成功
- HB\_MEDIA\_ERR\_UNKNOWN： 未知错误
- HB\_MEDIA\_ERR\_INVALID\_INSTANCE：无效实例
- HB\_MEDIA\_ERR\_INVALID\_PARAMS：无效参数

【功能描述】

获取智能背景编码参数，适用于H264/H265。

【示例代码】

参考 [hb_mm_mc_get_longterm_ref_mode](#hb_mm_mc_get_longterm_ref_mode)

#### hb\_mm\_mc\_set\_smart\_bg\_enc\_config

【函数声明】

hb\_s32 hb\_mm\_mc\_set\_smart\_bg\_enc\_config(media\_codec\_context\_t
\*context, const mc\_video\_smart\_bg\_enc\_params\_t \*params);

【参数描述】

- \[IN\] media\_codec\_context\_t \*context：指定codec类型的context
- \[IN\] const mc\_video\_smart\_bg\_enc\_params\_t
  \*params：智能背景编码模式参数

【返回值】

- 0：操作成功
- HB\_MEDIA\_ERR\_UNKNOWN： 未知错误
- HB\_MEDIA\_ERR\_OPERATION\_NOT\_ALLOWED：操作不允许
- HB\_MEDIA\_ERR\_INVALID\_INSTANCE：无效实例
- HB\_MEDIA\_ERR\_INVALID\_PARAMS：无效参数

【功能描述】

设置智能背景编码参数，该参数为动态参数，适用于H264/H265。

【示例代码】

参考 [hb_mm_mc_get_longterm_ref_mode](#hb_mm_mc_get_longterm_ref_mode)

#### hb\_mm\_mc\_get\_pred\_unit\_config

【函数声明】

hb\_s32 hb\_mm\_mc\_get\_pred\_unit\_config(media\_codec\_context\_t
\*context, mc\_video\_pred\_unit\_params\_t \*params)

【参数描述】

- \[IN\] media\_codec\_context\_t \*context：指定codec类型的context
- \[OUT\] mc\_video\_pred\_unit\_params\_t \*params：预测单元参数

【返回值】

- 0：操作成功
- HB\_MEDIA\_ERR\_UNKNOWN： 未知错误
- HB\_MEDIA\_ERR\_INVALID\_INSTANCE：无效实例
- HB\_MEDIA\_ERR\_INVALID\_PARAMS：无效参数

【功能描述】

获取预测单元参数，适用于H264/H265。

【示例代码】

参考 [hb_mm_mc_get_longterm_ref_mode](#hb_mm_mc_get_longterm_ref_mode)

#### hb\_mm\_mc\_set\_pred\_unit\_config

【函数声明】

hb\_s32 hb\_mm\_mc\_set\_pred\_unit\_config(media\_codec\_context\_t
\*context, const mc\_video\_pred\_unit\_params\_t \*params)

【参数描述】

- \[IN\] media\_codec\_context\_t \*context：指定codec类型的context
- \[IN\] const mc\_video\_pred\_unit\_params\_t \*params：预测单元参数

【返回值】

- 0：操作成功
- HB\_MEDIA\_ERR\_UNKNOWN： 未知错误
- HB\_MEDIA\_ERR\_OPERATION\_NOT\_ALLOWED：操作不允许
- HB\_MEDIA\_ERR\_INVALID\_INSTANCE：无效实例
- HB\_MEDIA\_ERR\_INVALID\_PARAMS：无效参数

【功能描述】

设置预测单元参数，该参数为动态参数，适用于H264/H265。

【示例代码】

参考 [hb_mm_mc_get_longterm_ref_mode](#hb_mm_mc_get_longterm_ref_mode)

#### hb\_mm\_mc\_get\_transform\_config

【函数声明】

hb\_s32 hb\_mm\_mc\_get\_transform\_config(media\_codec\_context\_t
\*context, mc\_video\_transform\_params\_t \*params)

【参数描述】

- \[IN\] media\_codec\_context\_t \*context：指定codec类型的context
- \[OUT\] mc\_video\_transform\_params\_t \*params：Transform参数

【返回值】

- 0：操作成功
- HB\_MEDIA\_ERR\_UNKNOWN： 未知错误
- HB\_MEDIA\_ERR\_INVALID\_INSTANCE：无效实例
- HB\_MEDIA\_ERR\_INVALID\_PARAMS：无效参数

【功能描述】

获取Transform参数，适用于H264/H265。

【示例代码】

参考 [hb_mm_mc_get_longterm_ref_mode](#hb_mm_mc_get_longterm_ref_mode)

#### hb\_mm\_mc\_set\_transform\_config

【函数声明】

hb\_s32 hb\_mm\_mc\_set\_transform\_config(media\_codec\_context\_t
\*context, const mc\_video\_transform\_params\_t \*params)

【参数描述】

- \[IN\] media\_codec\_context\_t \*context：指定codec类型的context
- \[IN\] const mc\_video\_transform\_params\_t \*params：
  Transform参数

【返回值】

- 0：操作成功
- HB\_MEDIA\_ERR\_UNKNOWN： 未知错误
- HB\_MEDIA\_ERR\_OPERATION\_NOT\_ALLOWED：操作不允许
- HB\_MEDIA\_ERR\_INVALID\_INSTANCE：无效实例
- HB\_MEDIA\_ERR\_INVALID\_PARAMS：无效参数

【功能描述】

设置Transform参数，该参数为动态参数，适用于H264/H265。

【示例代码】

参考 [hb_mm_mc_get_longterm_ref_mode](#hb_mm_mc_get_longterm_ref_mode)

#### hb\_mm\_mc\_get\_roi\_config

【函数声明】

hb\_s32 hb\_mm\_mc\_get\_roi\_config(media\_codec\_context\_t \*context,
mc\_video\_roi\_params\_t \*params)

【参数描述】

- \[IN\] media\_codec\_context\_t \*context：指定codec类型的context
- \[OUT\] mc\_video\_roi\_params\_t \*params：ROI编码参数

【返回值】

- 0：操作成功
- HB\_MEDIA\_ERR\_UNKNOWN： 未知错误
- HB\_MEDIA\_ERR\_INVALID\_INSTANCE：无效实例
- HB\_MEDIA\_ERR\_INVALID\_PARAMS：无效参数

【功能描述】

获取ROI编码参数，适用于H264/H265。

【示例代码】

参考 [hb_mm_mc_get_longterm_ref_mode](#hb_mm_mc_get_longterm_ref_mode)

#### hb\_mm\_mc\_set\_roi\_config

【函数声明】

hb\_s32 hb\_mm\_mc\_set\_roi\_config(media\_codec\_context\_t \*context,
const mc\_video\_roi\_params\_t \*params)

【参数描述】

- \[IN\] media\_codec\_context\_t \*context：指定codec类型的context
- \[IN\] const mc\_video\_roi\_params\_t \*params：ROI编码参数

【返回值】

- 0：操作成功
- HB\_MEDIA\_ERR\_UNKNOWN： 未知错误
- HB\_MEDIA\_ERR\_OPERATION\_NOT\_ALLOWED：操作不允许
- HB\_MEDIA\_ERR\_INVALID\_INSTANCE：无效实例
- HB\_MEDIA\_ERR\_INVALID\_PARAMS：无效参数

【功能描述】

设置ROI编码参数，该参数为动态参数，适用于H264/H265。

【示例代码】

参考 [hb_mm_mc_get_longterm_ref_mode](#hb_mm_mc_get_longterm_ref_mode)

#### hb\_mm\_mc\_get\_mode\_decision\_config

【函数声明】

hb\_s32 hb\_mm\_mc\_get\_mode\_decision\_config(media\_codec\_context\_t
\*context, mc\_video\_mode\_decision\_params\_t \*params);

【参数描述】

- \[IN\] media\_codec\_context\_t \*context：指定codec类型的context
- \[OUT\] mc\_video\_mode\_decision\_params\_t \*params：模式决策参数

【返回值】

- 0：操作成功
- HB\_MEDIA\_ERR\_UNKNOWN： 未知错误
- HB\_MEDIA\_ERR\_INVALID\_INSTANCE：无效实例
- HB\_MEDIA\_ERR\_INVALID\_PARAMS：无效参数

【功能描述】

获取mode decision参数，适用于H265。

【示例代码】

参考 [hb_mm_mc_get_longterm_ref_mode](#hb_mm_mc_get_longterm_ref_mode)

#### hb\_mm\_mc\_set\_mode\_decision\_config

【函数声明】

hb\_s32 hb\_mm\_mc\_set\_mode\_decision\_config(media\_codec\_context\_t
\*context, const mc\_video\_mode\_decision\_params\_t \*params);

【参数描述】

- \[IN\] media\_codec\_context\_t \*context：指定codec类型的context
- \[IN\] const mc\_video\_mode\_decision\_params\_t
  \*params：模式决策参数

【返回值】

- 0：操作成功
- HB\_MEDIA\_ERR\_UNKNOWN： 未知错误
- HB\_MEDIA\_ERR\_OPERATION\_NOT\_ALLOWED：操作不允许
- HB\_MEDIA\_ERR\_INVALID\_INSTANCE：无效实例
- HB\_MEDIA\_ERR\_INVALID\_PARAMS：无效参数

【功能描述】

设置模式决策参数，该参数为动态参数，适用于H265。

【示例代码】

参考 [hb_mm_mc_get_longterm_ref_mode](#hb_mm_mc_get_longterm_ref_mode)

#### hb\_mm\_mc\_get\_user\_data

【函数声明】

hb\_s32 hb\_mm\_mc\_get\_user\_data(media\_codec\_context\_t \*context,
mc\_user\_data\_buffer\_t \*params , hb\_s32 timeout)

【参数描述】

- \[IN\] media\_codec\_context\_t \*context：指定codec类型的context
- \[OUT\] mc\_user\_data\_buffer\_t \*params：用户数据
- \[IN\] timeout：超时时间

【返回值】

- 0：操作成功
- HB\_MEDIA\_ERR\_UNKNOWN： 未知错误
- HB\_MEDIA\_ERR\_OPERATION\_NOT\_ALLOWED：操作不允许
- HB\_MEDIA\_ERR\_INVALID\_INSTANCE：无效实例
- HB\_MEDIA\_ERR\_INVALID\_PARAMS：无效参数

【功能描述】

获取解码流中的用户数据，适用于H264/H265。

【示例代码】

```
#include "hb_media_codec.h"
#include "hb_media_error.h"
static int check_and_init_test(MediaCodecTestContext *ctx) {
    int32_t ret = 0;
    char *inputFileName, *outputFileName, *inputMd5FileName;
    EXPECT_NE(ctx, nullptr);
    EXPECT_NE(ctx->context, nullptr);
    if (ctx == NULL || ctx->context == NULL) {
        return -1;  
    }
    inputFileName = ctx->inputFileName;
    outputFileName = ctx->outputFileName;
    inputMd5FileName = ctx->inputMd5FileName;
    printf("%s[%d:%d] Thread work in %s mode\n", TAG, getpid(), gettid(),
    ctx->workMode == THREAD_WORK_MODE_SYNC ? "sync" :
        (ctx->workMode == THREAD_WORK_MODE_ASYNC ? "async" : "poll"));
    printf("%s[%d:%d] InputFileName = %s\n", TAG, getpid(), gettid(), inputFileName);
    printf("%s[%d:%d] OutputFileName = %s\n", TAG, getpid(), gettid(), outputFileName);
    printf("%s[%d:%d] InputMd5File = %s\n", TAG, getpid(), gettid(), inputMd5FileName);
    EXPECT_NE(inputFileName, nullptr);
    EXPECT_NE(outputFileName, nullptr);
    if (inputFileName == NULL || outputFileName == NULL) {
        return -1;
    }
    ctx->inFile = fopen(inputFileName, "rb");
    EXPECT_NE(ctx->inFile, nullptr);
    ctx->outFile = fopen(outputFileName, "wb+");
    EXPECT_NE(ctx->outFile, nullptr);
    if (ctx->inFile == NULL || ctx->outFile == NULL) {
        return -1;
    }
    if (ctx->md5Test == TRUE) {
        if (inputMd5FileName) {
            ctx->inMd5File = fopen(inputMd5FileName, "rb");
        }
        EXPECT_NE(ctx->inMd5File, nullptr);
        if (ctx->inMd5File == NULL) {
            return -1;
        }
    }
    // allocate ion buffers
    ctx->ionFd = ion_open();
    EXPECT_GT(ctx->ionFd, 0);
    if (ctx->ionFd <= 0) {
        return -1;
    }
    if (ctx->context->encoder == TRUE) {
        printf("%s[%d:%d] Thread use %s buffer mode, %d rc mode\n", TAG, getpid(), gettid(),
        ctx->context->video_enc_params.external_frame_buf ?
        "external" : "internal", 
        ctx->context->video_enc_params.rc_params.mode);
        if (ctx->context->video_enc_params.external_frame_buf) {
            ctx->exFb = (ExternalFrameBuffer *) malloc(
            ctx->context->video_enc_params.frame_buf_count * sizeof(ExternalFrameBuffer));
            EXPECT_NE(ctx->exFb, nullptr);
            if (ctx->exFb == NULL) {
                return -1;
            }
            for (Uint32 i=0; i<ctx->context->video_enc_params.frame_buf_count; i++) {
                ctx->exFb[i].buf.size = ctx->context->video_enc_params.width
                * ctx->context->video_enc_params.height * 3/2; // only for yuv420;
                ret = allocate_ion_mem(ctx->ionFd, &ctx->exFb[i].buf);
                EXPECT_EQ(ret, 0);
                if (ret != 0) {
                    return ret;
                }
                ctx->exFb[i].valid = 1;
                ctx->exFb[i].src_idx = i;
            }
        }
    } else {
        printf("%s[%d:%d] Thread use %s buffer mode, %d feed mode.\n", TAG, getpid(), gettid(),
        ctx->context->video_dec_params.external_bitstream_buf ?
        "external" : "internal",
        ctx->context->video_dec_params.feed_mode);
        if (ctx->context->video_dec_params.external_bitstream_buf) {
            ctx->exBs = (ExternalStreamBuffer *) malloc(
            ctx->context->video_dec_params.bitstream_buf_count * sizeof(ExternalStreamBuffer));
            EXPECT_NE(ctx->exBs, nullptr);
            if (ctx->exBs == NULL) {
                return -1;
            }
            for (Uint32 i=0; i<ctx->context->video_dec_params.bitstream_buf_count; i++) {
                ctx->exBs[i].buf.size = ctx->context->video_dec_params.bitstream_buf_size;
                ret = allocate_ion_mem(ctx->ionFd, &ctx->exBs[i].buf);
                EXPECT_EQ(ret, 0);
                if (ret != 0) {
                    return ret;
                }
                ctx->exBs[i].valid = 1;
                ctx->exBs[i].src_idx = i;
            }
        }
    }
// open decode files
    if (ctx->context->encoder != TRUE) {
    if (ctx->context->video_dec_params.feed_mode == MC_FEEDING_MODE_FRAME_SIZE) {
        ret = avformat_open_input(&ctx->avContext, ctx->inputFileName, 0, 0);
        EXPECT_GE(ret, 0);
        if (ret < 0) {
            return ret;
        }
        ret = avformat_find_stream_info(ctx->avContext, 0);
        EXPECT_GE(ret, 0);
        if (ret < 0) {
            return ret;
        }
        ctx->videoIndex = av_find_best_stream(ctx->avContext, AVMEDIA_TYPE_VIDEO, -1, -1, NULL, 0);
        EXPECT_GE(ctx->videoIndex, 0);
        if (ctx->videoIndex < 0) {
            return -1;
        }
        av_init_packet(&ctx->avpacket);
    } else {
        if (ctx->feedingSize == 0) {
            uint32_t KB = 1024;
            int32_t probability10;
            srand((uint32_t)time(NULL));
            ctx->feedingSize = rand() % MAX_FEEDING_SIZE;
            probability10 = (ctx->feedingSize % 100) < 10;
            if (ctx->feedingSize < KB) {
                if (probability10 == FALSE)
                    ctx->feedingSize *= 100;
                }
            }
            printf("%s[%d:%d] Feeding size = %d\n", TAG,
            getpid(), gettid(), ctx->feedingSize);
        }
        ctx->firstPacket = 1;
    }
    return 0;
}
static int check_and_release_test(MediaCodecTestContext *ctx) {
    int32_t ret = 0;
    int md5Match, wholeFileSize = 0;
    uint8_t *md5Buffer = NULL;
    EXPECT_NE(ctx, nullptr);
    EXPECT_NE(ctx->context, nullptr);
    EXPECT_NE(ctx->inFile, nullptr);
    EXPECT_NE(ctx->outFile, nullptr);
    if (ctx == NULL || ctx->context == NULL || ctx->inFile == NULL ||
        ctx->outFile == NULL) {
        return -1;
    }
    if (ctx->context->encoder != TRUE) {
        if (ctx->context->video_dec_params.feed_mode == MC_FEEDING_MODE_FRAME_SIZE) {
            if (ctx->avContext) {
                avformat_close_input(&ctx->avContext);
            }
        }
    }
    if (ctx->context->encoder == TRUE) {
        if (ctx->context->video_enc_params.external_frame_buf) {
            if (ctx->exFb) {
                for (Uint32 i=0; i<ctx->context->video_enc_params.frame_buf_count; i++) {
                    ret = release_ion_mem(ctx->ionFd, &ctx->exFb[i].buf);
                    EXPECT_EQ(ret, 0);
                }
                free(ctx->exFb);
            }
        }
    } else {
        if (ctx->context->video_dec_params.external_bitstream_buf) {
            if (ctx->exBs) {
                for (Uint32 i=0; i<ctx->context->video_dec_params.bitstream_buf_count; i++) {
                    ret = release_ion_mem(ctx->ionFd, &ctx->exBs[i].buf);
                    EXPECT_EQ(ret, 0);
                }
                free(ctx->exBs);
            }
        }
    }
    if (ctx->ionFd)
        ion_close(ctx->ionFd);
    if (ctx->md5Test && ctx->inMd5File) {
        fseek(ctx->outFile, 0, SEEK_END);
        wholeFileSize = ftell(ctx->outFile);
        fseek(ctx->outFile, 0, SEEK_SET);
        md5Buffer = (uint8_t *)malloc(wholeFileSize);
        EXPECT_NE(md5Buffer, nullptr);
        if (md5Buffer == NULL) {
            return -1;
        }
        fread(md5Buffer, wholeFileSize, 1, ctx->outFile);
        md5Match = compare_md5_value(MD5_SIZE, ctx->inMd5File,
        md5Buffer, wholeFileSize);
        free(md5Buffer);
        fclose(ctx->inMd5File);
        EXPECT_EQ(md5Match, 1);
        if (md5Match != 1) {
            return -1;
        }
    }
    if (ctx->outFile)
        fclose(ctx->outFile);
    if (ctx->inFile)
        fclose(ctx->inFile);
    return 0;
}
static void on_vlc_buffer_message(hb_ptr userdata, hb_s32 * vlc_buf) {
    MediaCodecTestContext *ctx = (MediaCodecTestContext *)userdata;
    ASSERT_NE(vlc_buf, nullptr);
    ASSERT_NE(ctx, nullptr);
    ASSERT_GE(ctx->vlc_buf_size, 0);
    if (ctx->testLog) {
        printf("%s %s VLC Buffer size = %d; Reset to %d.\n", TAG, __FUNCTION__,
        *vlc_buf, ctx->vlc_buf_size);
    }
    *vlc_buf = ctx->vlc_buf_size;
}
static int read_input_streams(MediaCodecTestContext *ctx,
    media_codec_buffer_t *inputBuffer) {
    Uint64 curTime = 0;
    int ret = 0, ret2 = 0;
    Uint32 bufIdx = 0, srcIdx = 0;
    Int32 doRead = TRUE, doRewind = FALSE;
    uint8_t *seqHeader = NULL;
    int seqHeaderSize = 0;
    void *bufPtr = NULL;
    int avalBufSize = 0;
    EXPECT_NE(ctx, nullptr);
    EXPECT_NE(ctx->context, nullptr);
    EXPECT_NE(ctx->inFile, nullptr);
    EXPECT_NE(ctx->outFile, nullptr);
    EXPECT_NE(inputBuffer, nullptr);
    if (ctx == NULL || ctx->context == NULL || ctx->inFile == NULL ||
        ctx->outFile == NULL || inputBuffer == NULL) {
        printf("%s[%d:%d] Invalid parameters(%s).\n",
        TAG, getpid(), gettid(), __FUNCTION__);
        return -1;
    }
    if (ctx->stabilityTest || ctx->pfTest) {
        doRewind = TRUE;
        curTime = osal_gettime();
        if ((curTime - ctx->testStartTime)/1000 < (uint32_t)ctx->duration) {
            doRead = TRUE;
        } else {
            printf("%s[%d:%d] Time up(%d)\n",
            TAG, getpid(), gettid(), ctx->duration);
            doRead = FALSE;
            ret = 0;
        }
    }
    if (ctx->context->video_dec_params.external_bitstream_buf) {
        // release input buffer and take it as the new input buffer
        for (bufIdx = 0;
            bufIdx < ctx->context->video_dec_params.bitstream_buf_count;
            bufIdx++) {
            if (ctx->exBs[bufIdx].valid &&
                ctx->exBs[bufIdx].src_idx == inputBuffer->vstream_buf.src_idx) {
                srcIdx = inputBuffer->vstream_buf.src_idx;
                break;
            }
        }
        EXPECT_NE(bufIdx, ctx->context->video_dec_params.bitstream_buf_count);
        if (bufIdx == ctx->context->video_dec_params.bitstream_buf_count) {
            return -1;
        }
        bufPtr = (void *)ctx->exBs[srcIdx].buf.virt_addr;
        if (ctx->context->video_dec_params.feed_mode == 
            MC_FEEDING_MODE_FRAME_SIZE) {
            avalBufSize = ctx->exBs[srcIdx].buf.size;
        } else {
            avalBufSize = (ctx->exBs[srcIdx].buf.size < (int)ctx->feedingSize) ?
            ctx->exBs[srcIdx].buf.size : ctx->feedingSize;
        }
        inputBuffer->vstream_buf.fd = ctx->exBs[srcIdx].buf.fd;
        inputBuffer->vstream_buf.phy_ptr =
        ctx->exBs[srcIdx].buf.phys_addr;
        inputBuffer->vstream_buf.vir_ptr =
        (hb_u8 *)ctx->exBs[srcIdx].buf.virt_addr;
    } else {
        bufPtr = (void *)inputBuffer->vstream_buf.vir_ptr;
        if (ctx->context->video_dec_params.feed_mode == 
            MC_FEEDING_MODE_FRAME_SIZE) {
            avalBufSize = inputBuffer->vstream_buf.size;
        } else {
            avalBufSize = (inputBuffer->vstream_buf.size < ctx->feedingSize) ?
            inputBuffer->vstream_buf.size : ctx->feedingSize;
        }
    }
    if (doRead == FALSE) {
        return ret;
    }
    // MC_FEEDING_MODE_FRAME_SIZE mode
    if (ctx->context->video_dec_params.feed_mode == MC_FEEDING_MODE_FRAME_SIZE) {
        do {
            if (ctx->avpacket.size == 0) {
                ret = av_read_frame(ctx->avContext, &ctx->avpacket);
                if (ret < 0 && doRewind == FALSE) {
                    printf("%s[%d:%d] Failed to read input file (error=0x%x)\n",
                        TAG, getpid(), gettid(), ret);
                }
                if (ret < 0 && doRewind == TRUE) {
                    avformat_close_input(&ctx->avContext);
                    ret2 = avformat_open_input(&ctx->avContext, ctx->inputFileName, 0, 0);
                    EXPECT_GE(ret2, 0);
                    if (ret2 < 0) {
                        ret = ret2;
                        break;
                    }
                    /*ret = avformat_find_stream_info(ctx->avContext, 0);
                    EXPECT_GE(ret, 0);
                    if (ret < 0) {
                        break;
                    }
                    ctx->videoIndex = av_find_best_stream(ctx->avContext, AVMEDIA_TYPE_VIDEO, -1, -1, NULL, 0);
                    EXPECT_GE(ctx->videoIndex, 0);
                    if (ctx->videoIndex < 0) {
                        ret = -1;
                        break;
                    }*/
                    av_init_packet(&ctx->avpacket);
                }
            } else {
                if (ctx->testLog) {
                    printf("%s[%d:%d] Reuse previous stream packet size %d\n",
                    TAG, getpid(), gettid(), ctx->avpacket.size);
                }
            }
        } while (ret < 0 && doRewind == TRUE);
        if (ret < 0) {
            if (ret == AVERROR_EOF || ctx->avContext->pb->eof_reached == TRUE) {
                printf("%s[%d:%d] End of file!\n", TAG, getpid(), gettid());
                ret = 0;
            } else {
                printf("%s[%d:%d] Failed to av_read_frame error(0x%08x)\n",
                TAG, getpid(), gettid(), ret);
            }
            return ret;
        }
        if (ctx->testLog) {
            printf("%s[%d:%d] Read packet size %d\n",
                TAG, getpid(), gettid(), ctx->avpacket.size);
        }
        seqHeaderSize = 0;
        if (ctx->firstPacket) {
            AVCodecParameters* codec;
            int retSize = 0;
            codec = ctx->avContext->streams[ctx->videoIndex]->codecpar;
            seqHeader = (uint8_t*)malloc(codec->extradata_size + 1024);
            if (seqHeader == NULL) {
                printf("%s[%d:%d] Failed to mallock seqHeader\n",
                TAG, getpid(), gettid());
                ret = -1;
                return ret;
            }
            memset((void*)seqHeader, 0x00, codec->extradata_size + 1024);
            seqHeaderSize = build_dec_seq_header(seqHeader,
            ctx->context->codec_id,
            ctx->avContext->streams[ctx->videoIndex], &retSize);
            if (seqHeaderSize < 0) {
                printf("%s[%d:%d] Failed to build seqHeader\n",
                TAG, getpid(), gettid());
                ret = -1;
                return ret;
            }
            ctx->firstPacket = 0;
        }
        if ((ctx->avpacket.size <= avalBufSize)
            && (seqHeaderSize <= avalBufSize)) {
            int bufSize = 0;
            if (seqHeaderSize) {
                memcpy(bufPtr, seqHeader, seqHeaderSize);
                bufSize = seqHeaderSize;
                /*memcpy((char *)bufPtr+bufSize,ctx->avpacket.data, ctx->avpacket.size);
                bufSize += ctx->avpacket.size;
                av_packet_unref(&ctx->avpacket);
                ctx->avpacket.size = 0;*/
            } else {
                memcpy(bufPtr,ctx->avpacket.data, ctx->avpacket.size);
                bufSize = ctx->avpacket.size;
                av_packet_unref(&ctx->avpacket);
                ctx->avpacket.size = 0;
            }
            inputBuffer->vstream_buf.size = bufSize;
        } else {
            printf("%s[%d:%d] The stream buffer is too "
            "small!\n", TAG, getpid(), gettid());
            return -1;
        }
        if (seqHeader) {
            free(seqHeader);
            seqHeader = NULL;
        }
        return 1;
    }
    // MC_FEEDING_MODE_STREAM_SIZE mode
    do {
        ret = fread(bufPtr, 1, avalBufSize, ctx->inFile);
        if (ret <= 0 && doRewind == FALSE) {
            printf("%s[%d:%d] Failed to read input file (error=0x%x)\n",
                TAG, getpid(), gettid(), ret);
        }
        if (ret <= 0 && doRewind == TRUE) {
            if(fseek(ctx->inFile, 0, SEEK_SET)) {
                printf("%s Failed to rewind input file (pid=%d, tid=%d)\n",
                    TAG, getpid(), gettid());
                break;
            }
        }
    } while (ret == 0 && doRewind == TRUE);
    inputBuffer->vstream_buf.size = ret > 0 ? ret : 0;
    return ret;
}
static int write_output_frames(MediaCodecTestContext *ctx,
    media_codec_buffer_t *outputBuffer) {
    int32_t ret = 0;
    EXPECT_NE(ctx, nullptr);
    EXPECT_NE(ctx->context, nullptr);
    EXPECT_NE(ctx->inFile, nullptr);
    EXPECT_NE(ctx->outFile, nullptr);
    EXPECT_NE(outputBuffer, nullptr);
    if (ctx == NULL || ctx->context == NULL || ctx->inFile == NULL ||
        ctx->outFile == NULL || outputBuffer == NULL) {
        printf("%s[%d:%d] Invalid parameters(%s).\n",
        TAG, getpid(), gettid(), __FUNCTION__);
        return -1;
    }
    if (!ctx->stabilityTest && !ctx->pfTest) {
        fwrite(outputBuffer->vframe_buf.vir_ptr[0], outputBuffer->vframe_buf.size,
            1, ctx->outFile);
    }
    return ret;
}
static int do_decode_params_checking(MediaCodecTestContext *ctx,
    media_codec_buffer_t *outputBuffer) {
    media_codec_context_t *context;
    int32_t ret = 0;
    EXPECT_NE(ctx, nullptr);
    EXPECT_NE(ctx->context, nullptr);
    if (ctx == NULL || ctx->context == NULL || ctx->inFile == NULL ||
        ctx->outFile == NULL || outputBuffer == NULL) {
        printf("%s[%d:%d] Invalid parameters(%s).\n",
        TAG, getpid(), gettid(), __FUNCTION__);
        return -1;
    }
    context = ctx->context;
    if (ctx->enable_get_userdata) {
        mc_user_data_buffer_t userdata = {0};
        ret = hb_mm_mc_get_user_data(context, &userdata, 0);
        if (!ret) {
            printf("%s[%d:%d] Get userdata %d:\n", TAG, getpid(), gettid(), userdata.size);
            for (uint32_t i = 0; i < userdata.size; i++) {
                if (i < 16) {
                    printf("%s[%d:%d] userdata[i]:%x\n", TAG, getpid(), gettid(), userdata.virt_addr[i]);
                } else {
                    printf("%s[%d:%d] userdata[i]:%c\n", TAG, getpid(), gettid(), userdata.virt_addr[i]);
                }
            }
            ret = hb_mm_mc_release_user_data(context, &userdata);
        } else {
            ret = 0;
        }
    }
    return ret;
}
static void do_sync_decoding(void *arg) {
    int ret = 0;
    int step = 0;
    MediaCodecTestContext *ctx = (MediaCodecTestContext *)arg;
    media_codec_context_t *context;
    media_codec_callback_t callback;
    media_codec_buffer_t inputBuffer;
    media_codec_buffer_t outputBuffer;
    media_codec_output_buffer_info_t info;
    int32_t decStartTime = 0, decFinishTime = 0;
    ctx->workMode = THREAD_WORK_MODE_SYNC;
    ASSERT_EQ(check_and_init_test(ctx), 0);
    context = ctx->context;
    //get current time
    ctx->testStartTime = osal_gettime();
    if (ctx->testLog) {
        printf("%s[%d:%d] Step %d initialize (outFile=%s, FileFd=%p)\n",
            TAG, getpid(), gettid(), step++, ctx->outputFileName, ctx->outFile);
    }
    ret = hb_mm_mc_initialize(context);
    ASSERT_EQ(ret, (int32_t)0);
    callback.on_vlc_buffer_message = on_vlc_buffer_message;
    if (ctx->vlc_buf_size > 0) {
        ret = hb_mm_mc_set_vlc_buffer_listener(context, &callback, ctx);
        ASSERT_EQ(ret, (int32_t)0);
    }
    if (ctx->testLog) {
        printf("%s[%d:%d] Step %d configure\n", TAG, getpid(), gettid(), step++);
    }
    ret = hb_mm_mc_configure(context);
    EXPECT_EQ(ret, (int32_t)0);
    if (ctx->testLog) {
        printf("%s[%d:%d] Step %d start\n", TAG, getpid(), gettid(), step++);
    }
    mc_av_codec_startup_params_t startup_params;
    memset(&startup_params, 0x00, sizeof(mc_av_codec_startup_params_t));
    ret = hb_mm_mc_start(context, &startup_params);
    EXPECT_EQ(ret, (int32_t)0);
    do {
        if (!ctx->lastStream) {
            if (ctx->testLog) {
                printf("%s[%d:%d] Step %d dequeue input\n", TAG, getpid(), gettid(), step++);
            }
            // process input buffers
            ret = hb_mm_mc_dequeue_input_buffer(context, &inputBuffer, 3000);
            //EXPECT_EQ(ret, (int32_t)0);
            if (!ret) {
                if (ctx->testLog) {
                    printf("%s[%d:%d] input buffer viraddr %p phy addr %x, size = %d\n",
                    TAG, getpid(), gettid(), inputBuffer.vstream_buf.vir_ptr,
                    inputBuffer.vstream_buf.phy_ptr,
                    inputBuffer.vstream_buf.size);
                }
                if (ctx->testLog) {
                    printf("%s[%d:%d] Step %d feed input (pid=%d, tid=%d)\n", TAG, getpid(), gettid(), step++);
                }
                ret = read_input_streams(ctx, &inputBuffer);
                if (ret <= 0) {
                    printf("%s[%d:%d] There is no more input data(ret=%d)!\n",
                    TAG, getpid(), gettid(), ret);
                    inputBuffer.vstream_buf.stream_end = TRUE;
                    inputBuffer.vstream_buf.size = 0;
                    ctx->lastStream = 1;
                }
                //EXPECT_EQ(ret, (int32_t)TRUE);
                if (ctx->testLog) {
                    printf("%s[%d:%d] Step %d queue input(size=%d)\n",
                    TAG, getpid(), gettid(), step++, inputBuffer.vstream_buf.size);
                }
                ret = hb_mm_mc_queue_input_buffer(context, &inputBuffer, 100);
                EXPECT_EQ(ret, (int32_t)0);
                if (ret != 0) {
                    break;
                }
                if (ctx->delaytest) {
                    decStartTime = osal_gettime();
                }
            } else {
                if (ret != (int32_t)HB_MEDIA_ERR_WAIT_TIMEOUT) {
                    EXPECT_EQ(ret, (int32_t)0);
                    char info[256];
                    hb_mm_strerror(ret, info, 256);
                    printf("%s[%d:%d] dequeue input buffer fail.(%s)\n", TAG, getpid(), gettid(), info);
                    break;
                }
            }
        }
        if (!ctx->lastFrame) {
            if (ctx->testLog) {
            printf("%s[%d:%d] Step %d dequeue output\n", TAG, getpid(), gettid(), step++);
            }
            // process output buffers
            memset(&outputBuffer, 0x00, sizeof(media_codec_buffer_t));
            memset(&info, 0x00, sizeof(media_codec_output_buffer_info_t));
            ret = hb_mm_mc_dequeue_output_buffer(context, &outputBuffer, &info, 100);
            //EXPECT_EQ(ret, (int32_t)0);
            if (!ret) {
                if (ctx->testLog) {
                    printf("%s[%d:%d] output bufferviraddr %p phy addr %x, size = %d, outFile = %p\n",
                    TAG, getpid(), gettid(), outputBuffer.vframe_buf.vir_ptr[0],
                    outputBuffer.vframe_buf.phy_ptr[0],
                    outputBuffer.vframe_buf.size, ctx->outFile);
                }
                if (ctx->testLog) {
                    printf("%s[%d:%d] Step %d write output file\n", TAG, getpid(), gettid(), step++);
                }
                if (ctx->delaytest) {
                    decFinishTime = osal_gettime();
                    if ((decFinishTime - decStartTime) >= ctx->delaytime) {
                        printf("%s[%d:%d] Decoding time is %d, more than %dms\n",
                        TAG, getpid(), gettid(), (decFinishTime - decStartTime), ctx->delaytime);
                        ASSERT_LE((decFinishTime - decStartTime), ctx->delaytime);
                    }
                }
                ASSERT_EQ(write_output_frames(ctx, &outputBuffer), 0);
                if (ctx->testLog) {
                    printf("%s[%d:%d] Step %d queue output\n", TAG, getpid(), gettid(), step++);
                }
                ASSERT_EQ(do_decode_params_checking(ctx, &outputBuffer), 0);
                ret = hb_mm_mc_queue_output_buffer(context, &outputBuffer, 100);
                EXPECT_EQ(ret, (int32_t)0);
                if (outputBuffer.vframe_buf.frame_end) {
                    printf("%s[%d:%d] There is no more output data!\n", TAG, getpid(), gettid());
                    ctx->lastFrame = 1;
                    break;
                }
                if (ret) {
                    break;
                }
            } else {
                char info[256];
                hb_mm_strerror(ret, info, 256);
                printf("%s[%d:%d] dequeue output buffer fail.(%s)\n", TAG, getpid(), gettid(), info);
                if (ret != (int32_t)HB_MEDIA_ERR_WAIT_TIMEOUT) {
                    EXPECT_EQ(ret, (int32_t)0);
                    break;
                }
                if (ctx->stabilityTest && ctx->lastStream ==1) {
                    break;
                }
            }
        }
    }while(TRUE);
    ret = hb_mm_mc_stop(context);
    EXPECT_EQ(ret, (int32_t)0);
    ret = hb_mm_mc_release(context);
    EXPECT_EQ(ret, (int32_t)0);
    ASSERT_EQ(check_and_release_test(ctx), 0);
}
int main(int argc, char *argv[])
{
    char outputFileName[MAX_FILE_PATH] = "input.h265";
    char inputFileName[MAX_FILE_PATH] = "output.yuv";
    mTestWidth = 640;
    mTestHeight = 480;
    mTestPixFmt = MC_PIXEL_FORMAT_YUV420P;
    mTestFeedMode = MC_FEEDING_MODE_FRAME_SIZE;
    mTestCodec = TEST_CODEC_ID_H265;
    mc_video_codec_dec_params_t *params;
    media_codec_context_t *context = (media_codec_context_t *)malloc(sizeof(media_codec_context_t ));
    ASSERT_NE(context, nullptr);
    memset(context, 0x00, sizeof(media_codec_context_t));
    context->codec_id = get_codec_id(mTestCodec);
    context->encoder = FALSE;
    params = &context->video_dec_params;
    params->feed_mode = mTestFeedMode;
    params->pix_fmt = mTestPixFmt;
    params->bitstream_buf_size = mTestWidth * mTestHeight * 3 / 2;
    params->bitstream_buf_count = 6;
    params->frame_buf_count = 8;
    if (context->codec_id == MEDIA_CODEC_ID_H265) {
        params->h265_dec_config.bandwidth_Opt = TRUE;
        params->h265_dec_config.reorder_enable = TRUE;
        params->h265_dec_config.skip_mode = 0;
        params->h265_dec_config.cra_as_bla = FALSE;
        params->h265_dec_config.dec_temporal_id_mode = 0;
        params->h265_dec_config.target_dec_temporal_id_plus1 = 0;
    }
    MediaCodecTestContext ctx;
    memset(&ctx, 0x00, sizeof(ctx));
    ctx.context = context;
    ctx.inputFileName = inputFileName;
    ctx.outputFileName = outputFileName;
    char inputMd5FileName[MAX_FILE_PATH];
    if (ctx.md5Test) {
        char inputMd5Suffix[MAX_FILE_PATH] = ".md5";
        snprintf(dedicatedSuffix, MAX_FILE_PATH, "%s", "dec");
        snprintf(inputMd5FileName, MAX_FILE_PATH, "%s%s_%s_%s%s",
        dedicatedInputPrefix, mGlobalPixFmtName[mTestPixFmt],
        dedicatedSuffix, mGlobalCodecName[mTestCodec], inputMd5Suffix);
        ctx.inputMd5FileName = inputMd5FileName;
    }
    do_sync_decoding(&ctx);
    if (context != NULL) {
        free(context);
    }
}
```

#### hb\_mm\_mc\_release\_user\_data

【函数声明】

hb\_s32 hb\_mm\_mc\_release\_user\_data(media\_codec\_context\_t \*
context, const mc\_user\_data\_buffer\_t \* params)

【参数描述】

- \[IN\] media\_codec\_context\_t \*context：指定codec类型的context
- \[IN\] const mc\_user\_data\_buffer\_t \* params： 用户数据

【返回值】

- 0：操作成功
- HB\_MEDIA\_ERR\_UNKNOWN： 未知错误
- HB\_MEDIA\_ERR\_OPERATION\_NOT\_ALLOWED：操作不允许
- HB\_MEDIA\_ERR\_INVALID\_INSTANCE：无效实例
- HB\_MEDIA\_ERR\_INVALID\_PARAMS：无效参数

【功能描述】

释放解码流中的用户数据，适用于H264/H265。

【示例代码】

参考 [hb_mm_mc_get_user_data](#hb_mm_mc_get_user_data)

#### hb\_mm\_mc\_get\_mjpeg\_config

【函数声明】

hb\_s32 hb\_mm\_mc\_get\_mjpeg\_config(media\_codec\_context\_t
\*context, mc\_mjpeg\_enc\_params\_t \*params)

【参数描述】

- \[IN\] media\_codec\_context\_t \*context：指定codec类型的context
- \[OUT\] mc\_mjpeg\_enc\_params\_t \*params：MJPEG编码参数

【返回值】

- 0：操作成功
- HB\_MEDIA\_ERR\_UNKNOWN： 未知错误
- HB\_MEDIA\_ERR\_INVALID\_INSTANCE：无效实例
- HB\_MEDIA\_ERR\_INVALID\_PARAMS：无效参数

【功能描述】

获取MJPEG编码参数，适用于MJPEG。

【示例代码】

```
#include "hb_media_codec.h"
#include "hb_media_error.h"
int main(int argc, char *argv[])
{
    int ret = 0;
    char outputFileName[MAX_FILE_PATH];
    char inputFileName[MAX_FILE_PATH];
    mTestCodec = TEST_CODEC_ID_MJPEG;
    mTestPixFmt = MC_PIXEL_FORMAT_NV12;
    char dedicatedSuffix[MAX_FILE_PATH] = "_test";
    char inputSuffix[MAX_FILE_PATH] = ".yuv";
    char outputSuffixJpeg[MAX_FILE_PATH] = ".jpg";
    char outputSuffixMjpg[MAX_FILE_PATH] = ".mjpg";
    snprintf(inputFileName, MAX_FILE_PATH, "%s%s%s",
        mInputSpecPrefix, mTest12Bit ? mGlobal12BPixFmtName[mTestPixFmt]
        : mGlobalPixFmtName[mTestPixFmt], inputSuffix);
    snprintf(outputFileName, MAX_FILE_PATH, "%s%s%s%s",
        mOutputSpecPrefix, mTest12Bit ? mGlobal12BPixFmtName[mTestPixFmt]
        : mGlobalPixFmtName[mTestPixFmt], dedicatedSuffix,
    mTestCodec == TEST_CODEC_ID_JPEG ? outputSuffixJpeg : outputSuffixMjpg);
    mc_video_codec_enc_params_t *params;
    media_codec_context_t *context = (media_codec_context_t *)malloc(sizeof(media_codec_context_t ));
    ASSERT_NE(context, nullptr);
    memset(context, 0x00, sizeof(media_codec_context_t));
    context->codec_id = mTestCodec == TEST_CODEC_ID_JPEG ?
        MEDIA_CODEC_ID_JPEG : MEDIA_CODEC_ID_MJPEG;
    context->encoder = TRUE;
    params = &context->video_enc_params;
    params->width = mTestWidth;
    params->height = mTestHeight;
    params->pix_fmt = mTestPixFmt;
    params->frame_buf_count = 5;
    params->bitstream_buf_count = 5;
    params->rot_degree = MC_CCW_0;
    params->mir_direction = MC_DIRECTION_NONE;
    params->frame_cropping_flag = FALSE;
    params->external_frame_buf = FALSE;
    if (context->codec_id == MEDIA_CODEC_ID_MJPEG) {
        params->rc_params.mode = MC_AV_RC_MODE_MJPEGFIXQP;
        ret = hb_mm_mc_get_rate_control_config(context, &params->rc_params);
        ASSERT_EQ(ret, (int32_t)0);
        params->mjpeg_enc_config.restart_interval = mTestWidth/16;
        params->mjpeg_enc_config.extended_sequential = mTest12Bit;
    } else {
        params->jpeg_enc_config.restart_interval = mTestWidth/16;
        params->jpeg_enc_config.extended_sequential = mTest12Bit;
    }
    mc_mjpeg_enc_params_t mjpeg_params;
    memset(&mjpeg_params, 0x00, sizeof(mjpeg_params));
    ret = hb_mm_mc_get_mjpeg_config(context, &mjpeg_params);
    ASSERT_EQ(ret, (int32_t)0);
    ret = hb_mm_mc_initialize(context);
    ASSERT_EQ(ret, (int32_t)0);
    ret = hb_mm_mc_set_mjpeg_config(context, &mjpeg_params);
    ASSERT_EQ(ret, (int32_t)0);
    ret = hb_mm_mc_stop(context);
    ASSERT_EQ(ret, (int32_t)0);
    ret = hb_mm_mc_release(context);
    ASSERT_EQ(ret, (int32_t)0);
    if (context != NULL) {
        free(context);
    }
}
```

#### hb\_mm\_mc\_set\_mjpeg\_config

【函数声明】

hb\_s32 hb\_mm\_mc\_set\_mjpeg\_config(media\_codec\_context\_t
\*context, const mc\_mjpeg\_enc\_params\_t \*params)

【参数描述】

- \[IN\] media\_codec\_context\_t \*context：指定codec类型的context
- \[IN\] const mc\_mjpeg\_enc\_params\_t \*params：MJPEG编码参数

【返回值】

- 0：操作成功
- HB\_MEDIA\_ERR\_UNKNOWN： 未知错误
- HB\_MEDIA\_ERR\_OPERATION\_NOT\_ALLOWED：操作不允许
- HB\_MEDIA\_ERR\_INVALID\_INSTANCE：无效实例
- HB\_MEDIA\_ERR\_INVALID\_PARAMS：无效参数

【功能描述】

设置MJPEG编码参数，该参数为动态参数，适用于MJPEG。

【示例代码】

参考 [hb_mm_mc_get_mjpeg_config](#hb_mm_mc_get_mjpeg_config)

#### hb\_mm\_mc\_get\_jpeg\_config

【函数声明】

hb\_s32 hb\_mm\_mc\_get\_jpeg\_config(media\_codec\_context\_t
\*context, mc\_jpeg\_enc\_params\_t \*params)

【参数描述】

- \[IN\] media\_codec\_context\_t \*context：指定codec类型的context
- \[OUT\] mc\_jpeg\_enc\_params\_t \*params：JPEG编码参数

【返回值】

- 0：操作成功
- HB\_MEDIA\_ERR\_UNKNOWN： 未知错误
- HB\_MEDIA\_ERR\_INVALID\_INSTANCE：无效实例
- HB\_MEDIA\_ERR\_INVALID\_PARAMS：无效参数

【功能描述】

获取JPEG编码参数，适用于JPEG。

【示例代码】

```
#include "hb_media_codec.h"
#include "hb_media_error.h"
int main(int argc, char *argv[])
{
    int ret = 0;
    char outputFileName[MAX_FILE_PATH];
    char inputFileName[MAX_FILE_PATH];
    mTestCodec = TEST_CODEC_ID_JPEG;
    mTestPixFmt = MC_PIXEL_FORMAT_NV12;
    char dedicatedSuffix[MAX_FILE_PATH] = "_test";
    char inputSuffix[MAX_FILE_PATH] = ".yuv";
    char outputSuffixJpeg[MAX_FILE_PATH] = ".jpg";
    char outputSuffixMjpg[MAX_FILE_PATH] = ".mjpg";
    snprintf(inputFileName, MAX_FILE_PATH, "%s%s%s",
        mInputSpecPrefix, mTest12Bit ? mGlobal12BPixFmtName[mTestPixFmt]
        : mGlobalPixFmtName[mTestPixFmt], inputSuffix);
    snprintf(outputFileName, MAX_FILE_PATH, "%s%s%s%s",
        mOutputSpecPrefix, mTest12Bit ? mGlobal12BPixFmtName[mTestPixFmt]
        : mGlobalPixFmtName[mTestPixFmt], dedicatedSuffix,
        mTestCodec == TEST_CODEC_ID_JPEG ? outputSuffixJpeg : outputSuffixMjpg);
    mc_video_codec_enc_params_t *params;
    media_codec_context_t *context = (media_codec_context_t *)malloc(sizeof(media_codec_context_t ));
    ASSERT_NE(context, nullptr);
    memset(context, 0x00, sizeof(media_codec_context_t));
    context->codec_id = mTestCodec == TEST_CODEC_ID_JPEG ?
    MEDIA_CODEC_ID_JPEG : MEDIA_CODEC_ID_MJPEG;
    context->encoder = TRUE;
    params = &context->video_enc_params;
    params->width = mTestWidth;
    params->height = mTestHeight;
    params->pix_fmt = mTestPixFmt;
    params->frame_buf_count = 5;
    params->bitstream_buf_count = 5;
    params->rot_degree = MC_CCW_0;
    params->mir_direction = MC_DIRECTION_NONE;
    params->frame_cropping_flag = FALSE;
    params->external_frame_buf = FALSE;
    if (context->codec_id == MEDIA_CODEC_ID_MJPEG) {
        params->rc_params.mode = MC_AV_RC_MODE_MJPEGFIXQP;
        ret = hb_mm_mc_get_rate_control_config(context, &params->rc_params);
        ASSERT_EQ(ret, (int32_t)0);
        params->mjpeg_enc_config.restart_interval = mTestWidth/16;
        params->mjpeg_enc_config.extended_sequential = mTest12Bit;
    } else {
        params->jpeg_enc_config.restart_interval = mTestWidth/16;
        params->jpeg_enc_config.extended_sequential = mTest12Bit;
    }
    mc_jpeg_enc_params_t jpeg_params;
    memset(&jpeg_params, 0x00, sizeof(jpeg_params));
    ret = hb_mm_mc_get_jpeg_config(context, &jpeg_params);
    ASSERT_EQ(ret, (int32_t)0);
    ret = hb_mm_mc_initialize(context);
    ASSERT_EQ(ret, (int32_t)0);
    jpeg_params.quality_factor = 30;
    jpeg_params.restart_interval = (((params->width+15)>>4) *
        ((params->height+15)>>4) * 2) + 1;
    jpeg_params.crop_en = FALSE;
    ret = hb_mm_mc_set_jpeg_config(context, &jpeg_params);
    ASSERT_EQ(ret, (int32_t)HB_MEDIA_ERR_INVALID_PARAMS);
    jpeg_params.restart_interval = 70;
    ret = hb_mm_mc_set_jpeg_config(context, &jpeg_params);
    ASSERT_EQ(ret, (int32_t)0);
    ret = hb_mm_mc_stop(context);
    ASSERT_EQ(ret, (int32_t)0);
    ret = hb_mm_mc_release(context);
    ASSERT_EQ(ret, (int32_t)0);
    if (context != NULL) {
        free(context);
    }
}
```

#### hb\_mm\_mc\_set\_jpeg\_config

【函数声明】

hb\_s32 hb\_mm\_mc\_set\_jpeg\_config(media\_codec\_context\_t
\*context, const mc\_jpeg\_enc\_params\_t \*params)

【参数描述】

- \[IN\] media\_codec\_context\_t \*context：指定codec类型的context
- \[IN\] const mc\_jpeg\_enc\_params\_t \*params：JPEG编码参数

【返回值】

- 0：操作成功
- HB\_MEDIA\_ERR\_UNKNOWN： 未知错误
- HB\_MEDIA\_ERR\_OPERATION\_NOT\_ALLOWED：操作不允许
- HB\_MEDIA\_ERR\_INVALID\_INSTANCE：无效实例
- HB\_MEDIA\_ERR\_INVALID\_PARAMS：无效参数

【功能描述】

设置JPEG编码参数，该参数为动态参数，适用于JPEG。

【示例代码】

参考 [hb_mm_mc_get_jpeg_config](#hb_mm_mc_get_jpeg_config)

#### hb\_mm\_mc\_get\_fd

【函数声明】

hb\_s32 hb\_mm\_mc\_get\_fd(media\_codec\_context\_t \* context, hb\_s32
\*fd)

【参数描述】

- \[IN\] media\_codec\_context\_t \*context：指定codec类型的context
- \[OUT\] hb\_s32 \*fd：设备节点fd

【返回值】

- 0：操作成功
- HB\_MEDIA\_ERR\_INVALID\_INSTANCE：无效实例
- HB\_MEDIA\_ERR\_INVALID\_PARAMS：无效参数

【功能描述】

获取设备节点fd，可用于select操作，监听编解码结果。

【示例代码】

```
#include "hb_media_codec.h"
#include "hb_media_error.h"

typedef struct MediaCodecTestContext {
    media_codec_context_t *context;
    char *inputFileName;
    char *outputFileName;
    int abnormal;
    int32_t duration; // s
} MediaCodecTestContext;

Uint64 osal_gettime(void)
{
    struct timespec tp;

    clock_gettime(CLOCK_MONOTONIC, &tp);

    return ((Uint64)tp.tv_sec*1000 + tp.tv_nsec/1000000);
}

static void do_poll_encoding_select(void *arg) {
    hb_s32 ret = 0;
    int pollFd = -1;
    FILE *outFile;
    int lastStream = 0;
    fd_set readFds;
    MediaCodecTestContext *ctx = (MediaCodecTestContext *)arg;
    media_codec_context_t *context = ctx->context;
    char *outputFileName = ctx->outputFileName;
    mc_inter_status_t status;

    outFile = fopen(outputFileName, "wb");
    if (!outFile) {
        goto ERR;
    }

    ret = hb_mm_mc_get_fd(context, &pollFd);
    if (ret) {
        goto ERR;
    }

    do {
        FD_ZERO(&readFds);
        FD_SET(pollFd, &readFds);
        ret = select(pollFd+1, &readFds, NULL, NULL, NULL);
        if (ret < 0) {
            printf("Failed to select fd = %d.(err %s)\n", pollFd, strerror(errno));
            ctx->abnormal = TRUE;
            break;
        } else if (ret == 0) {
            printf("Time out to select fd = %d.\n", pollFd);
            ctx->abnormal = TRUE;
            break;
        } else {
            if (FD_ISSET(pollFd, &readFds)) {
                ASSERT_EQ(hb_mm_mc_get_status(context, &status), (int32_t)0);
                if (ctx->testLog) {
                    printf("%s[%d:%d] output count %d input count %d\n", TAG, getpid(), gettid(),
                    status.cur_output_buf_cnt, status.cur_input_buf_cnt);
                }
                media_codec_buffer_t outputBuffer;
                media_codec_output_buffer_info_t info;
                memset(&outputBuffer, 0x00, sizeof(media_codec_buffer_t));
                memset(&info, 0x00, sizeof(media_codec_output_buffer_info_t));
                ret = hb_mm_mc_dequeue_output_buffer(context, &outputBuffer, &info, 100);
                if (!ret && outFile) {
                    fwrite(outputBuffer.vstream_buf.vir_ptr, outputBuffer.vstream_buf.size, 1, outFile);
                    ret = hb_mm_mc_queue_output_buffer(context, &outputBuffer, 100);

                    if (outputBuffer.vstream_buf.stream_end) {
                        printf("%There is no more output data!\n");
                        lastStream = 1;
                        break;
                    }
                    if (ret) {
                        ctx->abnormal = TRUE;
                        break;
                    }
                } else {
                    if (ret != (int32_t)HB_MEDIA_ERR_WAIT_TIMEOUT) {
                        printf("Dequeue output buffer fail.\n");
                        break;
                    }
                }
            }
        }
    } while (!lastStream && !ctx->abnormal);

ERR:
    if (pollFd) {
        hb_mm_mc_close_fd(context, pollFd)
    }
    if (outFile)
        fclose(outFile);
}

static void do_poll_encoding(void *arg) {
    pthread_t thread_id;
    void* retVal;
    hb_s32 ret = 0;
    FILE *inFile;
    int noMoreInput = 0;
    MediaCodecTestContext *ctx = (MediaCodecTestContext *)arg;
    media_codec_context_t *context = ctx->context;

    char *inputFileName = ctx->inputFileName;
    char *outputFileName = ctx->outputFileName;
    media_codec_state_t state = MEDIA_CODEC_STATE_NONE;
    inFile = fopen(inputFileName, "rb");
    if (!inFile) {
        goto ERR;
    }

    ret = hb_mm_mc_initialize(context);
    if (ret) {
        goto ERR;
    }

    ret = hb_mm_mc_configure(context);
    if (ret) {
        goto ERR;
    }

    mc_av_codec_startup_params_t startup_params;
    startup_params.video_enc_startup_params.receive_frame_number = 0;
    ret = hb_mm_mc_start(context, &startup_params);
    if (ret) {
        goto ERR;
    }

    pthread_create(&thread_id, NULL, (void* (*)(void*))do_poll_encoding_select, ctx);

    do {
        media_codec_buffer_t inputBuffer;
        memset(&inputBuffer, 0x00, sizeof(media_codec_buffer_t));
        ret = hb_mm_mc_dequeue_input_buffer(context, &inputBuffer, 100);
        if (!ret) {
            ret = fread(inputBuffer.vframe_buf.vir_ptr[0], 1,
                inputBuffer.vframe_buf.size, inFile);
            if (!ret) {
                printf("There is no more input data!\n");
                inputBuffer.vframe_buf.frame_end = TRUE;
                noMoreInput = 1;
            }
            ret = hb_mm_mc_queue_input_buffer(context, &inputBuffer, 100);
            if (ret) {
                printf("Queue input buffer fail.\n");
                break;
            }
        } else {
            if (ret != (int32_t)HB_MEDIA_ERR_WAIT_TIMEOUT) {
                printf("Dequeue input buffer fail.\n");
                break;
            }
        }
    }while(!noMoreInput && !ctx->abnormal);

    pthread_join(thread_id, &retVal);

    hb_mm_mc_stop(context);

    hb_mm_mc_release(context);
    context = NULL;

ERR:
    hb_mm_mc_get_state(context, &state);
    if (context && state != MEDIA_CODEC_STATE_UNINITIALIZED) {
        hb_mm_mc_stop(context);
        hb_mm_mc_release(context);
    }

    if (inFile)
        fclose(inFile);
}

int main(int argc, char *argv[])
{
    hb_s32 ret = 0;
    char outputFileName[MAX_FILE_PATH] = "./tmp.yuv";
    char inputFileName[MAX_FILE_PATH] = "./output.stream";
    mc_video_codec_enc_params_t *params;
    media_codec_context_t context;

    memset(&context, 0x00, sizeof(media_codec_context_t));
    context.codec_id = MEDIA_CODEC_ID_H265;
    context.encoder = TRUE;
    params = &context.video_enc_params;
    params->width = 640;
    params->height = 480;
    params->pix_fmt = MC_PIXEL_FORMAT_YUV420P;
    params->frame_buf_count = 5;
    params->external_frame_buf = FALSE;
    params->bitstream_buf_count = 5;
    params->rc_params.mode = MC_AV_RC_MODE_H265CBR;
    ret = hb_mm_mc_get_rate_control_config(&context, &params->rc_params);
    if (ret) {
        return -1;
    }
    params->rc_params.h265_cbr_params.bit_rate = 5000;
    params->rc_params.h265_cbr_params.frame_rate = 30;
    params->rc_params.h265_cbr_params.intra_period = 30;
    params->gop_params.decoding_refresh_type = 2;
    params->gop_params.gop_preset_idx = 2;
    params->rot_degree = MC_CCW_0;
    params->mir_direction = MC_DIRECTION_NONE;
    params->frame_cropping_flag = FALSE;

    MediaCodecTestContext ctx;
    memset(&ctx, 0x00, sizeof(ctx));
    ctx.context = &context;
    ctx.inputFileName = inputFileName;
    ctx.outputFileName = outputFileName;
    ctx.duration = 5;
    do_poll_encoding(&ctx);
}
```

#### hb\_mm\_mc\_close\_fd

【函数声明】

hb\_s32 hb\_mm\_mc\_close\_fd(media\_codec\_context\_t \* context,
hb\_s32 fd)

【参数描述】

- \[IN\] media\_codec\_context\_t \*context：指定codec类型的context
- \[IN\] hb\_s32 fd：设备节点fd

【返回值】

- 0：操作成功
- HB\_MEDIA\_ERR\_UNKNOWN： 未知错误
- HB\_MEDIA\_ERR\_OPERATION\_NOT\_ALLOWED：操作不允许
- HB\_MEDIA\_ERR\_INVALID\_INSTANCE：无效实例
- HB\_MEDIA\_ERR\_INVALID\_PARAMS：无效参数

【功能描述】

关闭设备节点。

【示例代码】

参考 [hb_mm_mc_get_fd](#hb_mm_mc_get_fd)

#### hb\_mm\_mc\_set\_camera

【函数声明】

hb\_s32 hb\_mm\_mc\_set\_camera(media\_codec\_context\_t \*context,
hb\_s32 pipeline, hb\_s32 channel\_port\_id)

【参数描述】

- \[IN\] media\_codec\_context\_t \*context：指定codec类型的context
- \[IN\] hb\_s32 pipeline：pipeline
- \[IN\] hb\_s32 channel\_port\_id：通道端口号

【返回值】

- 0：操作成功
- HB\_MEDIA\_ERR\_UNKNOWN： 未知错误
- HB\_MEDIA\_ERR\_OPERATION\_NOT\_ALLOWED：操作不允许
- HB\_MEDIA\_ERR\_INVALID\_INSTANCE：无效实例
- HB\_MEDIA\_ERR\_INVALID\_PARAMS：无效参数

【功能描述】

设置VIO的camera信息，该参数为静态参数，适用于H264/H265。

【示例代码】

```
#include "hb_media_codec.h"
#include "hb_media_error.h"
typedef struct _media_codec_context {
    media_codec_id_t codec_id;
    hb_bool encoder;
    hb_s32 instance_index;
    union {
        mc_video_codec_enc_params_t video_enc_params;
        mc_video_codec_dec_params_t video_dec_params;
        mc_audio_codec_enc_params_t audio_enc_params;
        mc_audio_codec_dec_params_t audio_dec_params;
    };
} media_codec_context_t;
void MediaCodecAPITest::init_params_H265() {
    memset(&context, 0x00, sizeof(media_codec_context_t));
    context.codec_id = MEDIA_CODEC_ID_H265;
    context.encoder = TRUE;
    params = &context.video_enc_params;
    params->width = mGlobalWidth;
    params->height = mGlobalHeight;
    params->pix_fmt = mGlobalPixFmt;
    params->frame_buf_count = 5;
    params->external_frame_buf = FALSE;
    params->bitstream_buf_count = 5;
    params->rc_params.mode = MC_AV_RC_MODE_H265CBR;
    EXPECT_EQ(hb_mm_mc_get_rate_control_config(&context, &params->rc_params), (int32_t)0);
    params->rc_params.h265_cbr_params.bit_rate = 5000;
    params->rc_params.h265_cbr_params.frame_rate = 30;
    params->rc_params.h265_cbr_params.intra_period = 6;
    params->gop_params.decoding_refresh_type = 2;
    params->gop_params.gop_preset_idx = 2;
    params->rot_degree = MC_CCW_0;
    params->mir_direction = MC_DIRECTION_NONE;
    params->frame_cropping_flag = FALSE;
}
int main(int argc, char *argv[])
{
    init_params_H265();
    EXPECT_EQ(hb_mm_mc_initialize(&context), (int32_t)0);
    EXPECT_EQ(hb_mm_mc_set_camera(&context, 1, 1), (int32_t)0);
    EXPECT_EQ(hb_mm_mc_release(&context), (int32_t)0);
}
```

#### hb\_mm\_mc\_get\_vui\_config

【函数声明】

hb\_s32 hb\_mm\_mc\_get\_vui\_config(media\_codec\_context\_t \*context,
mc\_video\_vui\_params\_t \*params)

【参数描述】

- \[IN\] media\_codec\_context\_t \*context：指定codec类型的context
- \[OUT\] mc\_video\_vui\_ params\_t \*params：VUI参数

【返回值】

- 0：操作成功
- HB\_MEDIA\_ERR\_UNKNOWN： 未知错误
- HB\_MEDIA\_ERR\_INVALID\_INSTANCE：无效实例
- HB\_MEDIA\_ERR\_INVALID\_PARAMS：无效参数

【功能描述】

获取VUI参数。

【其他说明】

【其它说明】

目前video编码时设置头信息中的默认color range为full
range模式，如果设置limit
range需要显式调用hb\_mm\_mc\_set\_vui\_config接口。

【示例代码】

```
#include "hb_media_codec.h"
#include "hb_media_error.h"
typedef enum ENC_CONFIG_MESSAGE {
    ENC_CONFIG_NONE = (0 << 0),
    ENC_CONFIG_LONGTERM_REF = (1 << 0),
    ENC_CONFIG_INTRA_REFRESH = (1 << 1),
    ENC_CONFIG_RATE_CONTROL = (1 << 2),
    ENC_CONFIG_DEBLK_FILTER = (1 << 3),
    ENC_CONFIG_SAO = (1 << 4),
    ENC_CONFIG_ENTROPY = (1 << 5),
    ENC_CONFIG_VUI_TIMING = (1 << 6),
    ENC_CONFIG_SLICE = (1 << 7),
    ENC_CONFIG_REQUEST_IDR = (1 << 8),
    ENC_CONFIG_SKIP_PIC = (1 << 9),
    ENC_CONFIG_SMART_BG = (1 << 10),
    ENC_CONFIG_MONOCHROMA = (1 << 11),
    ENC_CONFIG_PRED_UNIT = (1 << 12),
    ENC_CONFIG_TRANSFORM = (1 << 13),
    ENC_CONFIG_ROI = (1 << 14),
    ENC_CONFIG_MODE_DECISION = (1 << 15),
    ENC_CONFIG_USER_DATA = (1 << 16),
    ENC_CONFIG_MJPEG = (1 << 17),
    ENC_CONFIG_JPEG = (1 << 18),
    ENC_CONFIG_CAMERA = (1 << 19),
    ENC_CONFIG_INSERT_USERDATA = (1 << 20),
    ENC_CONFIG_VUI = (1 << 21),
    ENC_CONFIG_3DNR = (1 << 22),
    ENC_CONFIG_REQUEST_IDR_HEADER = (1 << 23),
    ENC_CONFIG_ENABLE_IDR = (1 << 24),
    ENC_CONFIG_TOTAL = (1 << 25),
} ENC_CONFIG_MESSAGE;
typedef struct MediaCodecTestContext {
    media_codec_context_t *context;
    char *inputFileName;
    char *outputFileName;
    int32_t duration; // s
    ENC_CONFIG_MESSAGE message;
    mc_video_longterm_ref_mode_t ref_mode;
    mc_rate_control_params_t rc_params;
    mc_video_intra_refresh_params_t intra_refr;
    mc_video_deblk_filter_params_t deblk_filter;
    mc_h265_sao_params_t sao;
    mc_h264_entropy_params_t entropy;
    mc_video_vui_params_t vui;
    mc_video_vui_timing_params_t vui_timing;
    mc_video_slice_params_t slice;
    mc_video_3dnr_enc_params_t noise_reduction;
    mc_video_smart_bg_enc_params_t smart_bg;
    mc_video_pred_unit_params_t pred_unit;
    mc_video_transform_params_t transform;
    mc_video_roi_params_t roi;
    mc_video_mode_decision_params_t mode_decision;
} MediaCodecTestContext;
Uint64 osal_gettime(void)
{
    struct timespec tp;
    clock_gettime(CLOCK_MONOTONIC, &tp);
    return ((Uint64)tp.tv_sec*1000 + tp.tv_nsec/1000000);
}
uint8_t uuid[] =
"dc45e9bd-e6d948b7-962cd820-d923eeef+HorizonAI";
static void set_message(MediaCodecTestContext *ctx) {
    int ret = 0;
    media_codec_context_t *context = ctx->context;
    if (ctx->message & ENC_CONFIG_VUI) {
        hb_mm_mc_get_vui_config(context, &ctx->vui);
        ret = hb_mm_mc_set_vui_config(context, &ctx->vui);
    }
}
static void do_sync_encoding(void *arg) {
    hb_s32 ret = 0;
    FILE *inFile;
    FILE *outFile;
    int noMoreInput = 0;
    int lastStream = 0;
    Uint64 lastTime = 0;
    Uint64 curTime = 0;
    int needFlush = 1;
    MediaCodecTestContext *ctx = (MediaCodecTestContext *)arg;
    media_codec_context_t *context = ctx->context;
    char *inputFileName = ctx->inputFileName;
    char *outputFileName = ctx->outputFileName;
    media_codec_state_t state = MEDIA_CODEC_STATE_NONE;
    inFile = fopen(inputFileName, "rb");
    if (!inFile) {
        goto ERR;
    }
    outFile = fopen(outputFileName, "wb");
    if (!outFile) {
        goto ERR;
    }
    //get current time
    lastTime = osal_gettime();
    ret = hb_mm_mc_initialize(context);
    if (ret) {
        goto ERR;
    }
    ret = hb_mm_mc_configure(context);
    if (ret) {
        goto ERR;
    }
    mc_av_codec_startup_params_t startup_params;
    startup_params.video_enc_startup_params.receive_frame_number = 0;
    ret = hb_mm_mc_start(context, &startup_params);
    if (ret) {
        goto ERR;
    }
    ret = hb_mm_mc_pause(context);
    if (ret) {
        goto ERR;
    }
    do {
    set_message(ctx);
    if (!noMoreInput) {
        media_codec_buffer_t inputBuffer;
        memset(&inputBuffer, 0x00, sizeof(media_codec_buffer_t));
        ret = hb_mm_mc_dequeue_input_buffer(context, &inputBuffer, 100);
        if (!ret) {
            curTime = osal_gettime();
            if ((curTime - lastTime)/1000 < (uint32_t)ctx->duration) {
                ret = fread(inputBuffer.vframe_buf.vir_ptr[0], 1,
                inputBuffer.vframe_buf.size, inFile);
                if (ret <= 0) {
                    if(fseek(inFile, 0, SEEK_SET)) {
                        printf("Failed to rewind input filen");
                    } else {
                        ret = fread(inputBuffer.vframe_buf.vir_ptr[0], 1,
                        inputBuffer.vframe_buf.size, inFile);
                        if (ret <= 0) {
                            printf("Failed to read input filen");
                        }
                    }
                }
            } else {
                printf("Time up(%d)n",ctx->duration);
                ret = 0;
            }
            if (!ret) {
                printf("There is no more input data!n");
                inputBuffer.vframe_buf.frame_end = TRUE;
                noMoreInput = 1;
            }
            ret = hb_mm_mc_queue_input_buffer(context, &inputBuffer, 100);
            if (ret) {
                printf("Queue input buffer fail.n");
                break;
            }
        } else {
            if (ret != (int32_t)HB_MEDIA_ERR_WAIT_TIMEOUT) {
                printf("Dequeue input buffer fail.n");
                break;
            }
        }
    }
    if (!lastStream) {
        media_codec_buffer_t outputBuffer;
        media_codec_output_buffer_info_t info;
        memset(&outputBuffer, 0x00, sizeof(media_codec_buffer_t));
        memset(&info, 0x00, sizeof(media_codec_output_buffer_info_t));
        ret = hb_mm_mc_dequeue_output_buffer(context, &outputBuffer, &info,
        3000);
        if (!ret && outFile) {
            fwrite(outputBuffer.vstream_buf.vir_ptr,
                outputBuffer.vstream_buf.size, 1, outFile);
            ret = hb_mm_mc_queue_output_buffer(context, &outputBuffer, 100);
            if (ret) {
                printf("Queue output buffer fail.n");
                break;
            }
            if (outputBuffer.vstream_buf.stream_end) {
                printf("There is no more output data!n");
                lastStream = 1;
                break;
            }
        } else {
            if (ret != (int32_t)HB_MEDIA_ERR_WAIT_TIMEOUT) {
                printf("Dequeue output buffer fail.n");
                break;
            }
        }
    }
    if (needFlush) {
        ret = hb_mm_mc_flush(context);
        needFlush = 0;
        if (ret) {
            break;
        }
    }
}while(TRUE);
hb_mm_mc_stop(context);
hb_mm_mc_release(context);
context = NULL;
ERR:
hb_mm_mc_get_state(context, &state);
if (context && state!=
    MEDIA_CODEC_STATE_UNINITIALIZED) {
    hb_mm_mc_stop(context);
    hb_mm_mc_release(context);
}
if (inFile)
    fclose(inFile);
if (outFile)
    fclose(outFile);
}
int main(int argc, char *argv[])
{
    hb_s32 ret = 0;
    char outputFileName[MAX_FILE_PATH] = "./tmp.yuv";
    char inputFileName[MAX_FILE_PATH] = "./output.stream";
    mc_video_codec_enc_params_t *params;
    media_codec_context_t context;
    memset(&context, 0x00, sizeof(media_codec_context_t));
    context.codec_id = MEDIA_CODEC_ID_H265;
    context.encoder = TRUE;
    params = &context.video_enc_params;
    params->width = 640;
    params->height = 480;
    params->pix_fmt = MC_PIXEL_FORMAT_YUV420P;
    params->frame_buf_count = 5;
    params->external_frame_buf = FALSE;
    params->bitstream_buf_count = 5;
    params->rc_params.mode = MC_AV_RC_MODE_H265CBR;
    ret = hb_mm_mc_get_rate_control_config(&context, &params->rc_params);
    if (ret) {
        return -1;
    }
    params->rc_params.h265_cbr_params.bit_rate = 5000;
    params->rc_params.h265_cbr_params.frame_rate = 30;
    params->rc_params.h265_cbr_params.intra_period = 30;
    params->gop_params.decoding_refresh_type = 2;
    params->gop_params.gop_preset_idx = 2;
    params->rot_degree = MC_CCW_0;
    params->mir_direction = MC_DIRECTION_NONE;
    params->frame_cropping_flag = FALSE;
    MediaCodecTestContext ctx;
    memset(&ctx, 0x00, sizeof(ctx));
    ctx.context = &context;
    ctx.inputFileName = inputFileName;
    ctx.outputFileName = outputFileName;
    mc_video_vui_params_t *vui = &ctx.vui;
    ret = hb_mm_mc_get_vui_config(context, vui);
    if (ret != 0) {
        return -1;
    }
    vui->h265_vui.video_signal_type_present_flag = 1;
    vui->h265_vui.video_format = 0;
    vui->h265_vui.video_full_range_flag = 0;
    ctx.message = ENC_CONFIG_VUI;
    do_sync_encoding(&ctx);
}
```

#### hb\_mm\_mc\_set\_vui\_config

【函数声明】

hb\_s32 hb\_mm\_mc\_set\_vui\_config(media\_codec\_context\_t \*context,
const mc\_video\_vui\_params\_t \*params)

【参数描述】

- \[IN\] media\_codec\_context\_t \*context：指定codec类型的context
- \[IN\] const mc\_video\_vui\_params\_t \*params：VUI参数

【返回值】

- 0：操作成功
- HB\_MEDIA\_ERR\_UNKNOWN： 未知错误
- HB\_MEDIA\_ERR\_OPERATION\_NOT\_ALLOWED：操作不允许
- HB\_MEDIA\_ERR\_INVALID\_INSTANCE：无效实例
- HB\_MEDIA\_ERR\_INVALID\_PARAMS：无效参数

【功能描述】

设置VUI 参数，该参数为静态参数。

【示例代码】

参考 [hb_mm_mc_get_vui_config](#hb_mm_mc_get_vui_config)

#### hb\_mm\_mc\_get\_3dnr\_enc\_config

【函数声明】

hb\_s32 hb\_mm\_mc\_get\_3dnr\_enc\_config(media\_codec\_context\_t
\*context, mc\_video\_3dnr\_enc\_params\_t \*params);

【参数描述】

- \[IN\] media\_codec\_context\_t \*context：指定codec类型的context
- \[IN\] mc\_video\_3dnr\_enc\_params\_t \*params：3DNR参数

【返回值】

- 0：操作成功
- HB\_MEDIA\_ERR\_UNKNOWN： 未知错误
- HB\_MEDIA\_ERR\_OPERATION\_NOT\_ALLOWED：操作不允许
- HB\_MEDIA\_ERR\_INVALID\_INSTANCE：无效实例
- HB\_MEDIA\_ERR\_INVALID\_PARAMS：无效参数

【功能描述】

获取3DNR参数，该参数为动态参数，适用于H265.

【示例代码】

```
#include "hb_media_codec.h"
#include "hb_media_error.h"
typedef enum ENC_CONFIG_MESSAGE {
    ENC_CONFIG_NONE = (0 << 0),
    ENC_CONFIG_LONGTERM_REF = (1 << 0),
    ENC_CONFIG_INTRA_REFRESH = (1 << 1),
    ENC_CONFIG_RATE_CONTROL = (1 << 2),
    ENC_CONFIG_DEBLK_FILTER = (1 << 3),
    ENC_CONFIG_SAO = (1 << 4),
    ENC_CONFIG_ENTROPY = (1 << 5),
    ENC_CONFIG_VUI_TIMING = (1 << 6),
    ENC_CONFIG_SLICE = (1 << 7),
    ENC_CONFIG_REQUEST_IDR = (1 << 8),
    ENC_CONFIG_SKIP_PIC = (1 << 9),
    ENC_CONFIG_SMART_BG = (1 << 10),
    ENC_CONFIG_MONOCHROMA = (1 << 11),
    ENC_CONFIG_PRED_UNIT = (1 << 12),
    ENC_CONFIG_TRANSFORM = (1 << 13),
    ENC_CONFIG_ROI = (1 << 14),
    ENC_CONFIG_MODE_DECISION = (1 << 15),
    ENC_CONFIG_USER_DATA = (1 << 16),
    ENC_CONFIG_MJPEG = (1 << 17),
    ENC_CONFIG_JPEG = (1 << 18),
    ENC_CONFIG_CAMERA = (1 << 19),
    ENC_CONFIG_INSERT_USERDATA = (1 << 20),
    ENC_CONFIG_VUI = (1 << 21),
    ENC_CONFIG_3DNR = (1 << 22),
    ENC_CONFIG_REQUEST_IDR_HEADER = (1 << 23),
    ENC_CONFIG_ENABLE_IDR = (1 << 24),
    ENC_CONFIG_TOTAL = (1 << 25),
} ENC_CONFIG_MESSAGE;
typedef struct MediaCodecTestContext {
    media_codec_context_t *context;
    char *inputFileName;
    char *outputFileName;
    int32_t duration; // s
    ENC_CONFIG_MESSAGE message;
    mc_video_longterm_ref_mode_t ref_mode;
    mc_rate_control_params_t rc_params;
    mc_video_intra_refresh_params_t intra_refr;
    mc_video_deblk_filter_params_t deblk_filter;
    mc_h265_sao_params_t sao;
    mc_h264_entropy_params_t entropy;
    mc_video_vui_params_t vui;
    mc_video_vui_timing_params_t vui_timing;
    mc_video_slice_params_t slice;
    mc_video_3dnr_enc_params_t noise_reduction;
    mc_video_smart_bg_enc_params_t smart_bg;
    mc_video_pred_unit_params_t pred_unit;
    mc_video_transform_params_t transform;
    mc_video_roi_params_t roi;
    mc_video_mode_decision_params_t mode_decision;
} MediaCodecTestContext;
Uint64 osal_gettime(void)
{
    struct timespec tp;
    clock_gettime(CLOCK_MONOTONIC, &tp);
    return ((Uint64)tp.tv_sec*1000 + tp.tv_nsec/1000000);
}
uint8_t uuid[] =
"dc45e9bd-e6d948b7-962cd820-d923eeef+HorizonAI";
static void set_message(MediaCodecTestContext *ctx) {
    int ret = 0;
    media_codec_context_t *context = ctx->context;
    if (ctx->message & ENC_CONFIG_VUI) {
        hb_mm_mc_get_vui_config(context, &ctx->vui);
        ret = hb_mm_mc_set_vui_config(context, &ctx->vui);
    }
}
static void do_sync_encoding(void *arg) {
    hb_s32 ret = 0;
    FILE *inFile;
    FILE *outFile;
    int noMoreInput = 0;
    int lastStream = 0;
    Uint64 lastTime = 0;
    Uint64 curTime = 0;
    int needFlush = 1;
    MediaCodecTestContext *ctx = (MediaCodecTestContext *)arg;
    media_codec_context_t *context = ctx->context;
    char *inputFileName = ctx->inputFileName;
    char *outputFileName = ctx->outputFileName;
    media_codec_state_t state = MEDIA_CODEC_STATE_NONE;
    inFile = fopen(inputFileName, "rb");
    if (!inFile) {
        goto ERR;
    }
    outFile = fopen(outputFileName, "wb");
    if (!outFile) {
        goto ERR;
    }
    //get current time
    lastTime = osal_gettime();
    ret = hb_mm_mc_initialize(context);
    if (ret) {
        goto ERR;
    }
    ret = hb_mm_mc_configure(context);
    if (ret) {
        goto ERR;
    }
    mc_av_codec_startup_params_t startup_params;
    startup_params.video_enc_startup_params.receive_frame_number = 0;
    ret = hb_mm_mc_start(context, &startup_params);
    if (ret) {
        goto ERR;
    }
    ret = hb_mm_mc_pause(context);
    if (ret) {
        goto ERR;
    }
    do {
    set_message(ctx);
    if (!noMoreInput) {
        media_codec_buffer_t inputBuffer;
        memset(&inputBuffer, 0x00, sizeof(media_codec_buffer_t));
        ret = hb_mm_mc_dequeue_input_buffer(context, &inputBuffer, 100);
        if (!ret) {
            curTime = osal_gettime();
            if ((curTime - lastTime)/1000 < (uint32_t)ctx->duration) {
                ret = fread(inputBuffer.vframe_buf.vir_ptr[0], 1,
                inputBuffer.vframe_buf.size, inFile);
                if (ret <= 0) {
                    if(fseek(inFile, 0, SEEK_SET)) {
                        printf("Failed to rewind input filen");
                    } else {
                        ret = fread(inputBuffer.vframe_buf.vir_ptr[0], 1,
                        inputBuffer.vframe_buf.size, inFile);
                        if (ret <= 0) {
                            printf("Failed to read input filen");
                        }
                    }
                }
            } else {
                printf("Time up(%d)n",ctx->duration);
                ret = 0;
            }
            if (!ret) {
                printf("There is no more input data!n");
                inputBuffer.vframe_buf.frame_end = TRUE;
                noMoreInput = 1;
            }
            ret = hb_mm_mc_queue_input_buffer(context, &inputBuffer, 100);
            if (ret) {
                printf("Queue input buffer fail.n");
                break;
            }
        } else {
            if (ret != (int32_t)HB_MEDIA_ERR_WAIT_TIMEOUT) {
                printf("Dequeue input buffer fail.n");
                break;
            }
        }
    }
    if (!lastStream) {
        media_codec_buffer_t outputBuffer;
        media_codec_output_buffer_info_t info;
        memset(&outputBuffer, 0x00, sizeof(media_codec_buffer_t));
        memset(&info, 0x00, sizeof(media_codec_output_buffer_info_t));
        ret = hb_mm_mc_dequeue_output_buffer(context, &outputBuffer, &info,
        3000);
        if (!ret && outFile) {
            fwrite(outputBuffer.vstream_buf.vir_ptr,
                outputBuffer.vstream_buf.size, 1, outFile);
            ret = hb_mm_mc_queue_output_buffer(context, &outputBuffer, 100);
            if (ret) {
                printf("Queue output buffer fail.n");
                break;
            }
            if (outputBuffer.vstream_buf.stream_end) {
                printf("There is no more output data!n");
                lastStream = 1;
                break;
            }
        } else {
            if (ret != (int32_t)HB_MEDIA_ERR_WAIT_TIMEOUT) {
                printf("Dequeue output buffer fail.n");
                break;
            }
        }
    }
    if (needFlush) {
        ret = hb_mm_mc_flush(context);
        needFlush = 0;
        if (ret) {
            break;
        }
    }
}while(TRUE);
hb_mm_mc_stop(context);
hb_mm_mc_release(context);
context = NULL;
ERR:
hb_mm_mc_get_state(context, &state);
if (context && state!=
    MEDIA_CODEC_STATE_UNINITIALIZED) {
    hb_mm_mc_stop(context);
    hb_mm_mc_release(context);
}
if (inFile)
    fclose(inFile);
if (outFile)
    fclose(outFile);
}
int main(int argc, char *argv[])
{
    hb_s32 ret = 0;
    char outputFileName[MAX_FILE_PATH] = "./tmp.yuv";
    char inputFileName[MAX_FILE_PATH] = "./output.stream";
    mc_video_codec_enc_params_t *params;
    media_codec_context_t context;
    memset(&context, 0x00, sizeof(media_codec_context_t));
    context.codec_id = MEDIA_CODEC_ID_H265;
    context.encoder = TRUE;
    params = &context.video_enc_params;
    params->width = 640;
    params->height = 480;
    params->pix_fmt = MC_PIXEL_FORMAT_YUV420P;
    params->frame_buf_count = 5;
    params->external_frame_buf = FALSE;
    params->bitstream_buf_count = 5;
    params->rc_params.mode = MC_AV_RC_MODE_H265CBR;
    ret = hb_mm_mc_get_rate_control_config(&context, &params->rc_params);
    if (ret) {
        return -1;
    }
    params->rc_params.h265_cbr_params.bit_rate = 5000;
    params->rc_params.h265_cbr_params.frame_rate = 30;
    params->rc_params.h265_cbr_params.intra_period = 30;
    params->gop_params.decoding_refresh_type = 2;
    params->gop_params.gop_preset_idx = 2;
    params->rot_degree = MC_CCW_0;
    params->mir_direction = MC_DIRECTION_NONE;
    params->frame_cropping_flag = FALSE;
    MediaCodecTestContext ctx;
    memset(&ctx, 0x00, sizeof(ctx));
    ctx.context = &context;
    ctx.inputFileName = inputFileName;
    ctx.outputFileName = outputFileName;
    mc_video_3dnr_enc_params_t *noise_rd = &ctx.noise_reduction;
    ret = hb_mm_mc_get_3dnr_enc_config(context, noise_rd);
    noise_rd->nr_y_enable = 0;
    noise_rd->nr_cb_enable = 0;
    noise_rd->nr_cr_enable = 0;
    noise_rd->nr_est_enable = 0;
    ctx.message = ENC_CONFIG_3DNR;
    do_sync_encoding(&ctx);
}
```

#### hb\_mm\_mc\_set\_3dnr\_enc\_config

【函数声明】

hb\_s32 hb\_mm\_mc\_set\_3dnr\_enc\_config(media\_codec\_context\_t
\*context, const mc\_video\_3dnr\_enc\_params\_t \*params);

【参数描述】

- \[IN\] media\_codec\_context\_t \*context：指定codec类型的context
- \[IN\] const mc\_video\_3dnr\_enc\_params\_t \*params：3DNR参数

【返回值】

- 0：操作成功
- HB\_MEDIA\_ERR\_UNKNOWN： 未知错误
- HB\_MEDIA\_ERR\_OPERATION\_NOT\_ALLOWED：操作不允许
- HB\_MEDIA\_ERR\_INVALID\_INSTANCE：无效实例
- HB\_MEDIA\_ERR\_INVALID\_PARAMS：无效参数

【功能描述】

设置3DNR参数，该参数为动态参数，适用于H265。

【示例代码】

参考 [hb_mm_mc_get_3dnr_enc_config](#hb_mm_mc_get_3dnr_enc_config)

#### hb\_mm\_mc\_request\_idr\_header

【函数声明】

hb\_s32 hb\_mm\_mc\_request\_idr\_header(media\_codec\_context\_t
\*context, hb\_u32 force\_header)

【参数描述】

- \[IN\] media\_codec\_context\_t \*context：指定codec类型的context
- \[IN\] hb\_u32 force\_header：

  > 0 : No froced header(VPS/SPS/PPS)
  >
  > 1 : Forced header before IDR frame
  >
  > 2 : Forced header before I frame for H264 or forced header before
  > CRA and IDR frame for H265
  >

【返回值】

- 0：操作成功
- HB\_MEDIA\_ERR\_UNKNOWN： 未知错误
- HB\_MEDIA\_ERR\_OPERATION\_NOT\_ALLOWED：操作不允许
- HB\_MEDIA\_ERR\_INVALID\_INSTANCE：无效实例
- HB\_MEDIA\_ERR\_INVALID\_PARAMS：无效参数

【功能描述】

请求帧头IDR帧头信息，适用于H264/H265。

【示例代码】

参考 [hb_mm_mc_get_longterm_ref_mode](#hb_mm_mc_get_longterm_ref_mode)

#### hb\_mm\_mc\_enable\_idr\_frame

【函数声明】

hb\_s32 hb\_mm\_mc\_enable\_idr\_frame(media\_codec\_context\_t
\*context, hb\_bool enable)

【参数描述】

- \[IN\] media\_codec\_context\_t \*context：指定codec类型的context
- \[IN\] hb\_bool enable：0：不使能；1：使能；

【返回值】

- 0：操作成功
- HB\_MEDIA\_ERR\_UNKNOWN： 未知错误
- HB\_MEDIA\_ERR\_OPERATION\_NOT\_ALLOWED：操作不允许
- HB\_MEDIA\_ERR\_INVALID\_INSTANCE：无效实例
- HB\_MEDIA\_ERR\_INVALID\_PARAMS：无效参数

【功能描述】

使能IDR帧，适用于H264/H265。

【示例代码】

参考 [hb_mm_mc_get_longterm_ref_mode](#hb_mm_mc_get_longterm_ref_mode)

#### hb\_mm\_mc\_register\_audio\_encoder

【函数声明】

hb\_s32 hb\_mm\_mc\_register\_audio\_encoder(hb\_s32 \*handle,
mc\_audio\_encode\_param\_t \*encoder)

【参数描述】

- \[IN\] hb\_s32 \*handle：编码器句柄
- \[IN\] mc\_audio\_encode\_param\_t \*encoder：audio编码器描述符；

【返回值】

- 0：操作成功
- HB\_MEDIA\_ERR\_UNKNOWN： 未知错误
- HB\_MEDIA\_ERR\_OPERATION\_NOT\_ALLOWED：操作不允许
- HB\_MEDIA\_ERR\_INVALID\_INSTANCE：无效实例
- HB\_MEDIA\_ERR\_INVALID\_PARAMS：无效参数

【功能描述】

注册audio编码器，适用于Audio。

【示例代码】

```
#include "hb_media_codec.h"
#include "hb_media_error.h"
#include "include/aac.h"
int main(int argc, char *argv[])
{
    mc_audio_codec_enc_params_t *params;
    media_codec_context_t context;
    memset(&context, 0x00, sizeof(media_codec_context_t));
    context.codec_id = MEDIA_CODEC_ID_AAC;
    context.encoder = TRUE;
    params = &context.audio_enc_params;
    params->bit_rate = 128000;
    params->frame_buf_count = 5;
    params->packet_count = 5;
    params->sample_fmt = MC_AV_SAMPLE_FMT_S16;
    params->sample_rate = MC_AV_SAMPLE_RATE_16000;
    params->channel_layout = MC_AV_CHANNEL_LAYOUT_STEREO;
    params->channels = 2;
    mc_aac_enc_config_t config;
    config.profile = MC_AAC_PROFILE_LOW;
    config.type = MC_AAC_DATA_TYPE_ADTS;
    params->enc_config = &config;
    int ret;
    int handle;
    mc_audio_encode_param_t encoder;
    encoder.ff_type = MEDIA_CODEC_ID_AAC;
    snprintf(encoder.ff_codec_name, sizeof(encoder.ff_codec_name), "aacenc");
    encoder.ff_audio_open_encoder = ff_audio_aac_open_encoder;
    encoder.ff_audio_encode_frame = ff_audio_aac_encode_frm;
    encoder.ff_audio_close_encoder = ff_audio_aac_close_encoder;
    ret = hb_mm_mc_register_audio_encoder(&handle, &encoder);
    printf("handle = %d\n", handle);
    ASSERT_EQ(ret, 0);
    ret = hb_mm_mc_unregister_audio_encoder(handle);
    ASSERT_EQ(ret, 0);
}
```

#### hb\_mm\_mc\_unregister\_audio\_encoder

【函数声明】

hb\_s32 hb\_mm\_mc\_unregister\_audio\_encoder(hb\_s32 handle)

【参数描述】

- \[IN\] hb\_s32 \*handle：编码器句柄

【返回值】

- 0：操作成功
- HB\_MEDIA\_ERR\_UNKNOWN： 未知错误
- HB\_MEDIA\_ERR\_OPERATION\_NOT\_ALLOWED：操作不允许
- HB\_MEDIA\_ERR\_INVALID\_INSTANCE：无效实例
- HB\_MEDIA\_ERR\_INVALID\_PARAMS：无效参数

【功能描述】

注销audio编码器，适用于Audio。

【示例代码】

参考 [hb_mm_mc_register_audio_encoder](#hb_mm_mc_register_audio_encoder)

#### hb\_mm\_mc\_register\_audio\_decoder

【函数声明】

hb\_s32 hb\_mm\_mc\_register\_audio\_decoder(hb\_s32 \*handle,
mc\_audio\_decode\_param\_t \*decoder)

【参数描述】

- \[IN\] hb\_s32 \*handle：解码器句柄
- \[IN\] mc\_audio\_decode\_param\_t \*decoder：audio解码器描述符；

【返回值】

- 0：操作成功
- HB\_MEDIA\_ERR\_UNKNOWN： 未知错误
- HB\_MEDIA\_ERR\_OPERATION\_NOT\_ALLOWED：操作不允许
- HB\_MEDIA\_ERR\_INVALID\_INSTANCE：无效实例
- HB\_MEDIA\_ERR\_INVALID\_PARAMS：无效参数

【功能描述】

注册audio解码器，适用于Audio。

【示例代码】

```
#include "hb_media_codec.h"
#include "hb_media_error.h"
#include "include/aac.h"
int main(int argc, char *argv[])
{
    mc_audio_codec_dec_params_t *params;
    media_codec_context_t context;
    memset(&context, 0x00, sizeof(media_codec_context_t));
    context.codec_id = MEDIA_CODEC_ID_AAC;
    context.encoder = FALSE;
    params = &context.audio_dec_params;
    params->feed_mode = MC_FEEDING_MODE_FRAME_SIZE;
    params->packet_buf_size = 1024;
    params->packet_count = 5;
    params->frame_cache_size = 5;
    params->frame_buf_count = 5;
    mc_aac_dec_config_t config;
    config.sample_rate = MC_AV_SAMPLE_RATE_8000;
    config.channels = 1;
    config.sample_fmt = MC_AV_SAMPLE_FMT_S16;
    params->dec_config = &config;
    mc_audio_decode_param_t decoder;
    decoder.ff_type = MEDIA_CODEC_ID_AAC;
    snprintf(decoder.ff_codec_name, sizeof(decoder.ff_codec_name), "aacdec");
    decoder.ff_audio_open_decoder = ff_audio_aac_open_decoder;
    decoder.ff_audio_decode_frame = ff_audio_aac_decode_frm;
    decoder.ff_audio_close_decoder = ff_audio_aac_close_decoder;
    ret = hb_mm_mc_register_audio_decoder(&handle, &decoder);
    ASSERT_EQ(ret, 0);
    ret = hb_mm_mc_unregister_audio_decoder(handle);
    ASSERT_EQ(ret, 0);
}
```

#### hb\_mm\_mc\_unregister\_audio\_decoder

【函数声明】

hb\_s32 hb\_mm\_mc\_unregister\_audio\_decoder(hb\_s32 handle)

【参数描述】

- \[IN\] hb\_s32 \*handle：解码器句柄；

【返回值】

- 0：操作成功
- HB\_MEDIA\_ERR\_UNKNOWN： 未知错误
- HB\_MEDIA\_ERR\_OPERATION\_NOT\_ALLOWED：操作不允许
- HB\_MEDIA\_ERR\_INVALID\_INSTANCE：无效实例
- HB\_MEDIA\_ERR\_INVALID\_PARAMS：无效参数

【功能描述】

注销audio解码器，适用于Audio。

【示例代码】

参考 [hb_mm_mc_register_audio_decoder](#hb_mm_mc_register_audio_decoder)

#### hb\_mm\_mc\_get\_explicit\_header\_config

【函数声明】

hb\_s32 hb\_mm\_mc\_get\_explicit\_header\_config
(media\_codec\_context\_t \*context, hb\_s32 \*status)

【参数描述】

- \[IN\] media\_codec\_context\_t \*context：指定codec类型的context
- \[OUT\] hb\_s32 \*status：使能/不使能头信息和IDR帧编码成一帧

【返回值】

- 0：操作成功
- HB\_MEDIA\_ERR\_UNKNOWN： 未知错误
- HB\_MEDIA\_ERR\_INVALID\_INSTANCE：无效实例
- HB\_MEDIA\_ERR\_INVALID\_PARAMS：无效参数

【功能描述】

获取头信息和IDR帧是否编码成一帧的配置，0：IDR和头信息独立，1：IDR和头信息合成一帧，适用于H264/H265。

【示例代码】

参考 [hb_mm_mc_get_longterm_ref_mode](#hb_mm_mc_get_longterm_ref_mode)

#### hb\_mm\_mc\_set\_explicit\_header\_config

【函数声明】

hb\_s32 hb\_mm\_mc\_set\_explicit\_header\_config
(media\_codec\_context\_t \*context, hb\_s32 status)

【参数描述】

- \[IN\] media\_codec\_context\_t \*context：指定codec类型的context
- \[IN\] hb\_s32 status：使能/不使能头信息和IDR帧编码成一帧

【返回值】

- 0：操作成功
- HB\_MEDIA\_ERR\_UNKNOWN： 未知错误
- HB\_MEDIA\_ERR\_OPERATION\_NOT\_ALLOWED：操作不允许
- HB\_MEDIA\_ERR\_INVALID\_INSTANCE：无效实例
- HB\_MEDIA\_ERR\_INVALID\_PARAMS：无效参数

【功能描述】

使能/不使能头信息和I帧编码成一帧，该参数为静态参数，0：IDR和头信息独立，1：IDR和头信息合成一帧，适用于H264/H265。

【示例代码】

参考 [hb_mm_mc_get_longterm_ref_mode](#hb_mm_mc_get_longterm_ref_mode)

#### hb\_mm\_mc\_get\_roi\_avg\_qp

【函数声明】

hb\_s32 hb\_mm\_mc\_get\_roi\_avg\_qp(media\_codec\_context\_t \*
context, hb\_u32 \* params)

【参数描述】

- \[IN\] media\_codec\_context\_t \*context：指定codec类型的context
- \[OUT\] hb\_u32 \*params：ROI 平均QP

【返回值】

- 0：操作成功
- HB\_MEDIA\_ERR\_UNKNOWN： 未知错误
- HB\_MEDIA\_ERR\_INVALID\_INSTANCE：无效实例
- HB\_MEDIA\_ERR\_INVALID\_PARAMS：无效参数

【功能描述】

获取ROI 平均QP值，0：代表该值由用户设定得QPMAP决定，适用于H264/H265。

【示例代码】

参考 [hb_mm_mc_get_longterm_ref_mode](#hb_mm_mc_get_longterm_ref_mode)

#### hb\_mm\_mc\_set\_roi\_avg\_qp

【函数声明】

hb\_s32 hb\_mm\_mc\_set\_roi\_avg\_qp(media\_codec\_context\_t \*
context, hb\_u32 params)

【参数描述】

- \[IN\] media\_codec\_context\_t \*context：指定codec类型的context
- \[IN\] hb\_u32 params：ROI平均QP值

【返回值】

- 0：操作成功
- HB\_MEDIA\_ERR\_UNKNOWN： 未知错误
- HB\_MEDIA\_ERR\_OPERATION\_NOT\_ALLOWED：操作不允许
- HB\_MEDIA\_ERR\_INVALID\_INSTANCE：无效实例
- HB\_MEDIA\_ERR\_INVALID\_PARAMS：无效参数

【功能描述】

设置ROI编码平均QP值，该参数为动态参数，0：表示使用设置的QP
Map中所有值得平均值，该值在RC模式为CBR或者AVBR时才能使编码效果生效，适用于H264/H265。

【示例代码】

参考 [hb_mm_mc_get_longterm_ref_mode](#hb_mm_mc_get_longterm_ref_mode)

### 主要参数说明

#### media\_codec\_state\_t

【描述】

定义Media codec的内部工作状态。

【定义】

```
typedef enum _media_codec_state {
    MEDIA_CODEC_STATE_NONE = -1,
    MEDIA_CODEC_STATE_UNINITIALIZED,
    MEDIA_CODEC_STATE_INITIALIZED,
    MEDIA_CODEC_STATE_CONFIGURED,
    MEDIA_CODEC_STATE_STARTED,
    MEDIA_CODEC_STATE_PAUSED,
    MEDIA_CODEC_STATE_FLUSHING,
    MEDIA_CODEC_STATE_ERROR,
    MEDIA_CODEC_STATE_TOTAL,
} media_codec_state_t;
```

#### media\_codec\_id\_t

【描述】

定义MediaCodec支持的codec id。

【定义】

```
typedef enum _media_codec_id {
    MEDIA_CODEC_ID_NONE = -1,
    /* Video Codecs */
    MEDIA_CODEC_ID_H264,
    MEDIA_CODEC_ID_H265,
    MEDIA_CODEC_ID_MJPEG,
    MEDIA_CODEC_ID_JPEG,
    /* Audio Codecs */
    MEDIA_CODEC_ID_FLAC,
    MEDIA_CODEC_ID_PCM_MULAW,
    MEDIA_CODEC_ID_PCM_ALAW,
    MEDIA_CODEC_ID_ADPCM_G726,
    MEDIA_CODEC_ID_ADPCM,
    MEDIA_CODEC_ID_AAC,
    MEDIA_CODEC_ID_MP3,
    MEDIA_CODEC_ID_MP2,
    MEDIA_CODEC_ID_TAK,
    MEDIA_CODEC_ID_AC3,
    MEDIA_CODEC_ID_WMA,
    MEDIA_CODEC_ID_AMR,
    MEDIA_CODEC_ID_APE,
    MEDIA_CODEC_ID_G729,
    MEDIA_CODEC_ID_G723,
    MEDIA_CODEC_ID_G722,
    MEDIA_CODEC_ID_IAC,
    MEDIA_CODEC_ID_RALF,
    MEDIA_CODEC_ID_QDMC,
    MEDIA_CODEC_ID_DTS,
    MEDIA_CODEC_ID_GSM,
    MEDIA_CODEC_ID_TTA,
    MEDIA_CODEC_ID_QCELP,
    MEDIA_CODEC_ID_MLP,
    MEDIA_CODEC_ID_ATRAC1,
    MEDIA_CODEC_ID_IMC,
    MEDIA_CODEC_ID_EAC,
    MEDIA_CODEC_ID_MP1,
    MEDIA_CODEC_ID_SIPR,
    MEDIA_CODEC_ID_OPUS,
    MEDIA_CODEC_ID_CELT,
    MEDIA_CODEC_ID_MOV_TEXT,
    MEDIA_CODEC_ID_TOTAL,
} media_codec_id_t;
```

#### mc\_video\_rate\_control\_mode\_t

【描述】

定义视频的码率控制方式，目前只支持H264/H265和MJPEG编码通道的码率控制。

【定义】

```
typedef enum _mc_video_rate_control_mode {
    MC_AV_RC_MODE_NONE = -1,
    MC_AV_RC_MODE_H264CBR,
    MC_AV_RC_MODE_H264VBR,
    MC_AV_RC_MODE_H264AVBR,
    MC_AV_RC_MODE_H264FIXQP,
    MC_AV_RC_MODE_H264QPMAP,
    MC_AV_RC_MODE_H265CBR,
    MC_AV_RC_MODE_H265VBR,
    MC_AV_RC_MODE_H265AVBR,
    MC_AV_RC_MODE_H265FIXQP,
    MC_AV_RC_MODE_H265QPMAP,
    MC_AV_RC_MODE_MJPEGFIXQP,
    MC_AV_RC_MODE_TOTAL,
} mc_video_rate_control_mode_t;
```

#### mc\_h264\_cbr\_params\_t

【描述】

定义H264的CBR控制方式下的可调节的参数集。

【定义】

```
typedef struct _mc_h264_cbr_params {
    hb_u32 intra_period;
    hb_u32 intra_qp;
    hb_u32 bit_rate;
    hb_u32 frame_rate;
    hb_u32 initial_rc_qp;
    hb_s32 vbv_buffer_size;
    hb_u32 mb_level_rc_enalbe;
    hb_u32 min_qp_I;
    hb_u32 max_qp_I;
    hb_u32 min_qp_P;
    hb_u32 max_qp_P;
    hb_u32 min_qp_B;
    hb_u32 max_qp_B;
    hb_u32 hvs_qp_enable;
    hb_s32 hvs_qp_scale;
    hb_u32 max_delta_qp;
    hb_bool qp_map_enable;
} mc_h264_cbr_params_t;
```

#### mc\_h264\_vbr\_params\_t

【描述】

定义H264的VBR控制方式下的可调节的参数集。

【定义】

```
typedef struct _mc_h264_vbr_params {
    hb_u32 intra_period;
    hb_u32 intra_qp;
    hb_u32 frame_rate;
    hb_bool qp_map_enable;
} mc_h264_vbr_params_t;
```

#### mc\_h264\_avbr\_params\_t

【描述】

定义H264的AVBR控制方式下的可调节的参数集。

【定义】

```
typedef struct _mc_h264_avbr_params {
    hb_u32 intra_period;
    hb_u32 intra_qp;
    hb_u32 bit_rate;
    hb_u32 frame_rate;
    hb_u32 initial_rc_qp;
    hb_s32 vbv_buffer_size;
    hb_u32 mb_level_rc_enalbe;
    hb_u32 min_qp_I;
    hb_u32 max_qp_I;
    hb_u32 min_qp_P;
    hb_u32 max_qp_P;
    hb_u32 min_qp_B;
    hb_u32 max_qp_B;
    hb_u32 hvs_qp_enable;
    hb_s32 hvs_qp_scale;
    hb_u32 max_delta_qp;
    hb_bool qp_map_enable;
} mc_h264_avbr_params_t;
```

#### mc\_h264\_fix\_qp\_params\_t

【描述】

定义H264的FixQP控制方式下的可调节的参数集。

【定义】

```
typedef struct _mc_h264_fix_qp_params {
    hb_u32 intra_period;
    hb_u32 frame_rate;
    hb_u32 force_qp_I;
    hb_u32 force_qp_P;
    hb_u32 force_qp_B;
} mc_h264_fix_qp_params_t;
```

#### mc_h264_qp_map_params_t

【描述】

定义H264的QPMAP控制方式下的可调节的参数集。

【定义】

```
typedef struct _mc_h264_qp_map_params {
    hb_u32 intra_period;
    hb_u32 frame_rate;
    hb_byte qp_map_array;
    hb_u32 qp_map_array_count;
} mc_h264_qp_map_params_t;
```

#### mc\_h265\_cbr\_params\_t

【描述】

定义H265的CBR控制方式下的可调节的参数集。

【定义】

```
typedef struct _mc_h265_cbr_params {
    hb_u32 intra_period;
    hb_u32 intra_qp;
    hb_u32 bit_rate;
    hb_u32 frame_rate;
    hb_u32 initial_rc_qp;
    hb_s32 vbv_buffer_size;
    hb_u32 ctu_level_rc_enalbe;
    hb_u32 min_qp_I;
    hb_u32 max_qp_I;
    hb_u32 min_qp_P;
    hb_u32 max_qp_P;
    hb_u32 min_qp_B;
    hb_u32 max_qp_B;
    hb_u32 hvs_qp_enable;
    hb_s32 hvs_qp_scale;
    hb_u32 max_delta_qp;
    hb_bool qp_map_enable;
} mc_h265_cbr_params_t;
```

#### mc\_h265\_vbr\_params\_t

【描述】

定义H265的VBR控制方式下的可调节的参数集。

【定义】

```
typedef struct _mc_h265_vbr_params {
    hb_u32 intra_period;
    hb_u32 intra_qp;
    hb_u32 frame_rate;
    hb_bool qp_map_enable;
} mc_h265_vbr_params_t;
```

#### mc\_h265\_avbr\_params\_t

【描述】

定义H265的AVBR控制方式下的可调节的参数集。

【定义】

```
typedef struct _mc_h265_avbr_params {
    hb_u32 intra_period;
    hb_u32 intra_qp;
    hb_u32 bit_rate;
    hb_u32 frame_rate;
    hb_u32 initial_rc_qp;
    hb_s32 vbv_buffer_size;
    hb_u32 ctu_level_rc_enalbe;
    hb_u32 min_qp_I;
    hb_u32 max_qp_I;
    hb_u32 min_qp_P;
    hb_u32 max_qp_P;
    hb_u32 min_qp_B;
    hb_u32 max_qp_B;
    hb_u32 hvs_qp_enable;
    hb_s32 hvs_qp_scale;
    hb_u32 max_delta_qp;
    hb_bool qp_map_enable;
} mc_h265_avbr_params_t;
```

#### mc\_h265\_fix\_qp\_params\_t

【描述】

定义H265的FixQP控制方式下的可调节的参数集。

【定义】

```
typedef struct _mc_h265_fix_qp_params {
    hb_u32 intra_period;
    hb_u32 frame_rate;
    hb_u32 force_qp_I;
    hb_u32 force_qp_P;
    hb_u32 force_qp_B;
} mc_h265_fix_qp_params_t;
```

#### mc\_h265\_qp\_map\_params\_t

【描述】

定义H265的QPMAP控制方式下的可调节的参数集。

【定义】

```
typedef struct _mc_h265_qp_map_params {
    hb_u32 intra_period;
    hb_u32 frame_rate;
    hb_byte qp_map_array;
    hb_u32 qp_map_array_count;
} mc_h265_qp_map_params_t;
```

#### mc\_mjpeg\_fix\_qp\_params\_t

【描述】

定义MJPEG的FixQP控制方式下的可调节的参数集。

【定义】

```
typedef struct _mc_mjpeg_fix_qp_params {
    hb_u32 frame_rate;
    hb_u32 quality_factor;
} mc_mjpeg_fix_qp_params_t;
```

#### mc\_video\_custom\_gop\_pic\_params\_t

【描述】

定义自定义的GOP结构表的数据结构。

【定义】

```
typedef struct _mc_video_custom_gop_pic_params {
    hb_u32 pic_type;
    hb_s32 poc_offset;
    hb_u32 pic_qp;
    hb_s32 num_ref_picL0;
    hb_s32 ref_pocL0;
    hb_s32 ref_pocL1;
    hb_u32 temporal_id;
} mc_video_custom_gop_pic_params_t;
```

#### mc\_inter\_status\_t

【描述】

定义media codec内部状态信息

【定义】

```
typedef struct _mc_inter_status {
    hb_u32 cur_input_buf_cnt;
    hb_u64 cur_input_buf_size;
    hb_u32 cur_output_buf_cnt;
    hb_u64 cur_output_buf_size;
    hb_u32 left_recv_frame;
    hb_u32 left_enc_frame;
    hb_u32 total_input_buf_cnt;
    hb_u32 total_output_buf_cnt;
    hb_s32 pipeline;
    hb_s32 channel_port_id;
} mc_inter_status_t;
```

#### media\_codec\_context\_t

【描述】

定义Media codec的上下文。

【定义】

```
typedef struct _media_codec_context {
    media_codec_id_t codec_id;
    hb_bool encoder;
    hb_s32 instance_index;
    union {
        mc_video_codec_enc_params_t video_enc_params;
        mc_video_codec_dec_params_t video_dec_params;
        mc_audio_codec_enc_params_t audio_enc_params;
        mc_audio_codec_dec_params_t audio_dec_params;
    };
    hb_ptr vpf_context;
    mc_video_cmd_prio_t priority;
} media_codec_context_t;
```

#### mc\_video\_codec\_enc\_params\_t

【描述】

定义视频编码器的编码参数，视频编码器类型包括H264，H265，MJPEG和JPEG。

【定义】

```
typedef struct _mc_video_codec_enc_params {
    hb_s32 width, height;
    mc_pixel_format_t pix_fmt;
    hb_u32 frame_buf_count;
    hb_bool external_frame_buf;
    hb_u32 bitstream_buf_count;
    hb_u32 bitstream_buf_size;
    mc_rate_control_params_t rc_params;
    mc_video_gop_params_t gop_params;
    mc_rotate_degree_t rot_degree;
    mc_mirror_direction_t mir_direction;
    hb_u32 frame_cropping_flag;
    mc_av_codec_rect_t crop_rect;
    hb_bool enable_user_pts;
    union {
        mc_h264_enc_config_t h264_enc_config;
        mc_h265_enc_config_t h265_enc_config;
        mc_mjpeg_enc_config_t mjpeg_enc_config;
        mc_jpeg_enc_config_t jpeg_enc_config;
    };
} mc_video_codec_enc_params_t;
```

#### mc\_video\_codec\_dec\_params\_t

【描述】

定义视频解码器的解码参数，视频解码器类型包括H264，H265，MJPEG和JPEG。

【定义】

```
typedef struct _mc_video_codec_dec_params {
    mc_av_stream_feeding_mode_t feed_mode;
    mc_pixel_format_t pix_fmt;
    hb_u32 bitstream_buf_size;
    hb_u32 bitstream_buf_count;
    hb_bool external_bitstream_buf;
    hb_u32 frame_buf_count;
    union {
        mc_h264_dec_config_t h264_dec_config;
        mc_h265_dec_config_t h265_dec_config;
        mc_mjpeg_dec_config_t mjpeg_dec_config;
        mc_jpeg_dec_config_t jpeg_dec_config;
    };
} mc_video_codec_dec_params_t;
```

#### mc\_audio\_codec\_enc\_params\_t

【描述】

定义audio codec的编码参数。

【定义】

```
typedef struct _mc_audio_codec_enc_params {
    hb_u32 bit_rate;
    hb_s32 frame_size;
    hb_s32 frame_buf_count;
    hb_s32 packet_count;
    mc_audio_sample_format_t sample_fmt;
    mc_audio_sample_rate_t sample_rate;
    mc_audio_channel_layout_t channel_layout;
    hb_s32 channels;
    hb_ptr enc_config;
} mc_audio_codec_enc_params_t;
```

#### mc\_audio\_codec\_dec\_params\_t

【描述】

定义audio codec的解码参数。

【定义】

```
typedef struct _mc_audio_codec_dec_params {
    mc_av_stream_feeding_mode_t feed_mode;
    hb_s32 packet_buf_size;
    hb_s32 packet_count;
    hb_s32 frame_cache_size;
    hb_s32 internal_frame_size;
    hb_s32 frame_buf_count;
    hb_ptr dec_config;
} mc_audio_codec_dec_params_t;
```

### 返回值说明

| 错误码 | 宏定义 | 描述 |
| --- | --- | --- |
| 0xF0000001 | HB_MEDIA_ERR_UNKNOWN | 未知的错误 |
| 0xF0000002 | HB_MEDIA_ERR_CODEC_NOT_FOUND | 找不到对应的codec |
| 0xF0000003 | HB_MEDIA_ERR_CODEC_OPEN_FAIL | 无法打开codec设备 |
| 0xF0000004 | HB_MEDIA_ERR_CODEC_RESPONSE_TIMEOUT | codec响应超时 |
| 0xF0000005 | HB_MEDIA_ERR_CODEC_INIT_FAIL | codec初始化失败 |
| 0xF0000006 | HB_MEDIA_ERR_OPERATION_NOT_ALLOWED | 操作不允许 |
| 0xF0000007 | HB_MEDIA_ERR_INSUFFICIENT_RES | 内部内存资源不足 |
| 0xF0000008 | HB_MEDIA_ERR_NO_FREE_INSTANCE | 没有可用的insta nce（VPU最多32个，JPU最 多64个，Audio最多32个） |
| 0xF0000009 | HB_MEDIA_ERR_INVALID_PARAMS | 无效的参数 |
| 0xF000000A | HB_MEDIA_ERR_INVALID_INSTANCE | 无效的实例 |
| 0xF000000B | HB_MEDIA_ERR_INVALID_BUFFER | 无效的buffer |
| 0xF000000C | HB_MEDIA_ERR_INVALID_COMMAND | 无效的指令 |
| 0xF000000D | HB_MEDIA_ERR_WAIT_TIMEOUT | 等待超时 |
| 0xF000000E | HB_MEDIA_ERR_FILE_OPERATION_FAILURE | 文件操作失败 |
| 0xF000000F | HB_MEDIA_ERR_PARAMS_SET_FAILURE | 参数设置失败 |
| 0xF0000010 | HB_MEDIA_ERR_PARAMS_GET_FAILURE | 参数获取失败 |
| 0xF0000011 | HB_MEDIA_ERR_CODING_FAILED | 编解码失败 |
| 0xF0000012 | HB_MEDIA_ERR_OUTPUT_BUF_FULL | 输出buffer满 |
| 0xF0000013 | HB_MEDIA_ERR_UNSUPPORTED_FEATURE | 不支持的功能 |
| 0xF0000014 | HB_MEDIA_ERR_INVALID_PRIORITY | 不支持的优先级 |

## Codec sample

### 编码示例

#### 功能概述

编码 yuv 图像, 生成 h264/h265 视频或 jpg 图片。

##### 软件架构说明

采用MediaCodec的poll模式来解耦输入和输出，可使编码帧率性能达到最优。
在主线程中灌YUV数据：取出一个空的input
buffer，配置YUV数据的地址信息（如phys addr），再queue input
buffer并通知编码器处理该帧数据；
另一个线程取输出码流：通过select接收硬件编码完成通知，取出一个硬件填满输出码流的output
buffer，将编码结果写到文件中后归还output buffer。

![image](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/03_multimedia_development/02_S100/codec/encoder2.png)

##### 硬件数据流说明

![image](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/03_multimedia_development/02_S100/codec/encoder1.png)

##### 代码位置及目录结构

sample代码位置在工程目录source/hobot-sp-samples/debian/app/multimedia\_demo/codec\_demo。

目录结构如下：

```
├── README.md
└── sample_venc_vdec
    ├── input_3840x2160_yuv420p.h264
    ├── input_3840x2160_yuv420p.yuv
    ├── Makefile
    ├── sample.c
    ├── sample_common.c
    ├── sample.h
    ├── sample_vdec.c
    └── sample_venc.c
```

根目录包含README.md，简要介绍编译命令，运行帮助信息及命令。

sample_venc_devc下的Makefile用于该目录下的编译。其中，sample.c是main入口的所在文件，sample\_common包含了一些共用的api，sample\_venc.c包含编码相关函数，sample\_vdec.c包含解码相关函数。

#### 编译

##### 编译环境

板端在安装hobot-sp-samples_*.deb包后，会包含codec\_demo源码内容。

##### 编译说明

本sample主要依赖libmm提供的API头文件：

```
#include "hb_media_codec.h"
#include "hb_media_error.h"
```

编译依赖的库有如下：

```
LIBS += -lpthread -ldl -lhbmem -lalog  -lmultimedia
LIBS += -lavformat -lavcodec -lavutil -lswresample
```

编译命令:

板端进入/app/multimedia\_demo/codec\_demo/sample\_venc\_vdec目录。

```
make
```

#### 运行

##### 支持平台

RDKS100。

##### 板端部署及配置

刷写系统软件镜像后，本sample的源码位于板端路径：`/app/multimedia_demo/codec_demo/sample_venc_vdec`；

可能需要用到的资源：

- 输入YUV图像默认包含4K格式的YUV裸流及H264文件，如果测试其他测试请用户自行上传；

##### 运行指南

###### 运行参数说明

`sample_codec` ： 应用程序名字

-m：编码或解码，默认编码， 0： encoder 1： decoder

-c：编解码器类型，默认H264， 0：H264 1：H265 2：mjpg 3：jpg

-w：图像宽度，默认3840

-h：图像高度，默认2160

-p：编解码图像格式，默认nv12， 0：yuv420p 1：nv12 2：nv21

-n：测试线程数量，默认1个

-i：输入文件的路径，默认`./input_${w}x${h}_${pixfmt}<_thread_idx>.yuv`

-o：输出文件的路径，默认`./output_${w}x${h}_${pixfmt}<_thread_idx>.{code_type}`

-H：打印帮助信息

###### 帮助菜单

```
Usage: ./sample_codec
        -m --samplemode sample mode, default encoder, {0-encoder, 1-decoder}
        -c --codecid codec id, default h264, {0-h264, 1-h265, 2-mjpeg, 3-jpeg}
        -w --width width, default 3840
        -h --height height, default 2160
        -p --pixfmt pix fmt, default nv12, {0-yuv420p, 1-nv12, ..}
        -n --threadnum test thread number, default 1
        -i --inputfile input file name, default ./input_${w}x${h}_${pixfmt}<_thread_idx>.yuv
        -o --outputfile output file name, default ./output_${w}x${h}_${pixfmt}<_thread_idx>.{code_type}
        -H --help print usage
```

###### 运行方法

输入源准备：将测试文件(如input\_3840x2160\_nv12.yuv)到当前目录 或
用-i指定文件路径；

编码一路 3840x2160 的YUV图像序列，生成 H264 视频码流

```
/app/multimedia_demo/codec_demo/sample_venc_vdec/sample_codec
```

编码一路 1920x1080 的YUV图像序列，生成 H265 视频码流

```
/app/multimedia_demo/codec_demo/sample_venc_vdec/sample_codec -c 1 -w 1920 -h 1080
```

编码一张 1920x1088 的YUV图像，生成 jpg 图片

```
/app/multimedia_demo/codec_demo/sample_venc_vdec/sample_codec -c 3 -w 1920 -h 1088
```

编码两路 3840x2160 的YUV图像序列，生成 H265 视频

```
/app/multimedia_demo/codec_demo/sample_venc_vdec/sample_codec -c 1 -n 2
```

编码四路 1920x1080 的YUV图像序列，生成 H265 视频

```
/app/multimedia_demo/codec_demo/sample_venc_vdec/sample_codec -c 1 -w 1920 -h 1080 -n 4
```

VPU CROP读入并编码：将1920x1300（图像尺寸不满足对齐要求）输入按`{x=200, y=300, w=1280, h=720}`大小读入数据并编码生成 H265 视频

```
/app/multimedia_demo/codec_demo/sample_venc_vdec/sample_codec -c 1 -w 1920 -h 1300
```

###### 运行结果说明

如下图所示为运行成功

![image](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/03_multimedia_development/02_S100/codec/encoder3.png)

查看生成的h264/h265/jpg是否正常

![image](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/03_multimedia_development/02_S100/codec/encoder4.png)

### 解码示例

#### 功能概述

解码h264/h265视频或jpg图片，生成yuv图像。

##### 软件架构

采用MediaCodec的poll模式来解耦输入和输出，可使解码帧率性能达到最优。
在主线程中灌码流数据：取出一个空的input
buffer，配置码流数据的地址信息（如phys addr），再queue input
buffer并通知解码器处理该帧数据；
另一个线程取输出YUV图像：通过select接收硬件解码完成通知，取出一个硬件填满输出图像的output
buffer，将解码结果写到文件中后归还output buffer。

![image](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/03_multimedia_development/02_S100/codec/decoder2.png)

##### 硬件数据流说明

![image](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/03_multimedia_development/02_S100/codec/decoder1.png)

##### 代码位置及目录结构

sample代码位置在工程目录source/hobot-sp-samples/debian/app/multimedia\_demo/codec\_demo。

目录结构如下：

```
├── README.md
└── sample_venc_vdec
    ├── input_3840x2160_yuv420p.h264
    ├── input_3840x2160_yuv420p.yuv
    ├── Makefile
    ├── sample.c
    ├── sample_common.c
    ├── sample.h
    ├── sample_vdec.c
    └── sample_venc.c
```

根目录包含README.md，简要介绍编译命令，运行帮助信息及命令。

sample_venc_devc下的Makefile用于该目录下的编译。其中，sample.c是main入口的所在文件，sample\_common包含了一些共用的api，sample\_venc.c包含编码相关函数，sample\_vdec.c包含解码相关函数。

#### 编译

##### 编译环境

板端在安装hobot-sp-samples_*.deb包后，会包含codec\_demo源码内容。

##### 编译说明

本sample主要依赖libmm提供的API头文件：

```
#include "hb_media_codec.h"
#include "hb_media_error.h"
```

编译依赖的库有如下：

```
LIBS += -lpthread -ldl -lhbmem -lalog  -lmultimedia
LIBS += -lavformat -lavcodec -lavutil -lswresample
```

编译命令:

板端进入/app/multimedia\_demo/codec\_demo/sample\_venc\_vdec目录。

```
make
```

#### 运行

##### 支持平台

RDKS100。

##### 板端部署及配置

刷写系统软件镜像后，本sample的源码位于板端路径：`/app/multimedia_demo/codec_demo/sample_venc_vdec`；

可能需要用到的资源：

- 输入YUV图像默认包含4K格式的YUV裸流及H264文件，如果测试其他测试请用户自行上传；

##### 运行指南

###### 运行参数说明

`sample_codec` ： 应用程序名字

-m：编码或解码，默认编码， 0： encoder 1： decoder

-c：编解码器类型，默认H264， 0：H264 1：H265 2：mjpg 3：jpg

-w：图像宽度，默认3840

-h：图像高度，默认2160

-p：编解码图像格式，默认nv12， 0：yuv420p 1：nv12 2：nv21

-n：测试线程数量，默认1个

-i：输入文件的路径，默认`./input_${w}x${h}_${pixfmt}<_thread_idx>.yuv`

-o：输出文件的路径，默认`./output_${w}x${h}_${pixfmt}<_thread_idx>.{code_type}`

-H：打印帮助信息

###### 帮助菜单

```
Usage: ./sample_codec
        -m --samplemode sample mode, default encoder, {0-encoder, 1-decoder}
        -c --codecid codec id, default h264, {0-h264, 1-h265, 2-mjpeg, 3-jpeg}
        -w --width width, default 3840
        -h --height height, default 2160
        -p --pixfmt pix fmt, default nv12, {0-yuv420p, 1-nv12, ..}
        -n --threadnum test thread number, default 1
        -i --inputfile input file name, default ./input_${w}x${h}_${pixfmt}<_thread_idx>.yuv
        -o --outputfile output file name, default ./output_${w}x${h}_${pixfmt}<_thread_idx>.{code_type}
        -H --help print usage
```

###### 运行方法

输入源准备：将测试文件(如input\_3840x2160\_nv12.h264)到当前目录 或
用-i指定文件路径；

解码一路 3840x2160 的 h264 视频, 生成 yuv 图像

```
/app/multimedia_demo/codec_demo/sample_venc_vdec/sample_codec -m 1
```

解码一路 1920x1080 的 h265 视频, 生成 yuv 图像

```
/app/multimedia_demo/codec_demo/sample_venc_vdec/sample_codec -m 1 -c 1 -w 1920 -h 1080
```

解码一张 1920x1088 的 jpg 图片, 生成 yuv 图像

```
/app/multimedia_demo/codec_demo/sample_venc_vdec/sample_codec -m 1 -c 3 -w 1920 -h 1088
```

解码两路 3840x2160 的 h264 视频, 生成 yuv 图像

```
/app/multimedia_demo/codec_demo/sample_venc_vdec/sample_codec -m 1 -n 2 -i
```

解码四路 1920x1080 的 h265 视频, 生成 yuv 图像

```
/app/multimedia_demo/codec_demo/sample_venc_vdec/sample_codec -m 1 -c 1 -n 4 -w 1920 -h 1080
```

###### 运行结果说明

如下图所示为运行成功

![image](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/03_multimedia_development/02_S100/codec/decoder3.png)

使用yuvplayer查看生成的yuv图像文件是否正常

![image](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/03_multimedia_development/02_S100/codec/decoder4.png)
