---
sidebar_position: 3
---
# Codec

## System Overview

### Overview

Codec (Coder-Decoder) refers to a codec used to compress or decompress media data such as video, images, and audio. The S100 SoC includes two hardware codec units: VPU (Video Processing Unit) and JPU (JPEG Processing Unit), providing 4K@90fps video codec capability and 4K@90fps image codec capability.

#### JPU Hardware Features

| **HW Feature** | **Feature Indicator**                        |
| -------------------- | -------------------------------------------------- |
| HW number            | 1                                                  |
| maximum input        | 8192x8192                                          |
| minimum input        | 32x32                                              |
| performance          | 4K@90fps                                          |
| max instance         | 64                                                 |
| input image format   | 4:0:0, 4:2:0, 4:2:2, 4:4:0, and 4:4:4 color format |
| output image format  | 4:0:0, 4:2:0, 4:2:2, 4:4:0, and 4:4:4 color format |
| input crop           | Supports                                           |
| bitrate control      | FIXQP(MJPEG)                                       |
| rotation             | 90, 180, 270                                       |
| mirror               | Vertical, Horizontal, Vertical+Horizontal          |
| quantization table   | Supports Custom Settings                           |
| huffman table        | Supports Custom Settings                           |

#### VPU Hardware Features

| **HW Feature**           | **Feature Indicator**                                                                                                                                                                                                                                                                                                                                                                                                     |
| ------------------------------ | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| HW number                      | 1                                                                                                                                                                                                                                                                                                                                                                                                                               |
| maximum input                  | 8192x4096                                                                                                                                                                                                                                                                                                                                                                                                                       |
| minimum input                  | 256x128                                                                                                                                                                                                                                                                                                                                                                                                                         |
| input alignment required       | width 32, height 8                                                                                                                                                                                                                                                                                                                                                                                                              |
| performance                    | 4K@90fps                                                                                                                                                                                                                                                                                                                                                                                                                       |
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

### Software Features

#### Overall Framework

The MediaCodec subsystem provides components for audio/video and image codec, raw stream packaging, and video recording. This system primarily encapsulates underlying codec hardware resources and software codec libraries to offer codec capabilities to upper layers. Developers can implement H.265 and H.264 video encoding/decoding functionalities using the provided codec APIs, use JPEG encoding to save camera data as JPEG images, or utilize the video recording feature to record camera data.

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/03_multimedia_development/02_S100/codec/2f8364ee5efbb8cb14136e0dc942248e.png)

#### Bitrate Control Modes

MediaCodec supports bitrate control for H.264/H.265 and MJPEG protocols, offering five bitrate control methods for H.264/H.265 encoding channels—CBR, VBR, AVBR, FixQp, and QpMap—and FixQp bitrate control for MJPEG encoding channels.

##### CBR Description

CBR stands for Constant Bitrate, ensuring stable overall encoding bitrate. Below are parameter descriptions for CBR mode:

| **Parameter**       | **Description**                                                                                                                                                                                                                                                 | **Range**          | **Default** |
| ------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ------------------ | ---------------- |
| intra_period        | I-frame interval                                                                                                                                                                                                                                                | [0,2047]           | 28               |
| intra_qp            | QP value for I-frames                                                                                                                                                                                                                                           | [0,51]             | 0                |
| bit_rate            | Target average bitrate, in kbps                                                                                                                                                                                                                                 | [0,700000]         | 0                |
| frame_rate          | Target frame rate, in fps                                                                                                                                                                                                                                       | [1,240]            | 30               |
| initial_rc_qp       | Initial QP value for rate control; if outside [0,51], the encoder internally determines the initial value                                                                                                                                                        | [0,63]             | 63               |
| vbv_buffer_size     | VBV buffer size in ms; actual VBV buffer size = bit_rate * vbv_buffer_size / 1000 (kb). Buffer size affects encoding quality and bitrate control accuracy: smaller buffers yield higher bitrate control precision but lower image quality; larger buffers improve image quality but increase bitrate fluctuation. | [10,3000]          | 10               |
| ctu_level_rc_enable | H.264/H.265 rate control can operate at CTU level for higher bitrate control precision at the cost of encoding quality. This mode cannot be used with ROI encoding; it is automatically disabled when ROI encoding is enabled. | [0,1]              | 0                |
| min_qp_I            | Minimum QP value for I-frames                                                                                                                                                                                                                                   | [0,51]             | 8                |
| max_qp_I            | Maximum QP value for I-frames                                                                                                                                                                                                                                   | [0,51]             | 51               |
| min_qp_P            | Minimum QP value for P-frames                                                                                                                                                                                                                                   | [0,51]             | 8                |
| max_qp_P            | Maximum QP value for P-frames                                                                                                                                                                                                                                   | [0,51]             | 51               |
| min_qp_B            | Minimum QP value for B-frames                                                                                                                                                                                                                                   | [0,51]             | 8                |
| max_qp_B            | Maximum QP value for B-frames                                                                                                                                                                                                                                   | [0,51]             | 51               |
| hvs_qp_enable       | H.264/H.265 rate control can operate at sub-CTU level, adjusting sub-macroblock QP values to improve subjective image quality.                                                                                                                                   | [0,1]              | 1                |
| hvs_qp_scale        | Effective when hvs_qp_enable is enabled; represents QP scaling factor.                                                                                                                                                                                           | [0,4]              | 2                |
| max_delta_qp        | Effective when hvs_qp_enable is enabled; specifies maximum allowable deviation for HVS QP values.                                                                                                                                                                | [0,51]             | 10               |
| qp_map_enable       | Enables QP map for ROI encoding.                                                                                                                                                                                                                                | [0,1]              | 0                |

##### VBR Description

VBR stands for Variable Bitrate, allocating higher QP (lower quality, higher compression) for simple scenes and lower QP (higher quality) for complex scenes to maintain consistent visual quality. Below are parameter descriptions for VBR mode:

| **Parameter**    | **Description**        | **Range** | **Default** |
| ---------------- | --------------------- | ------------------ | ---------------- |
| intra_period     | I-frame interval      | [0,2047]           | 28               |
| intra_qp         | QP value for I-frames | [0,51]             | 0                |
| frame_rate       | Target frame rate, in fps | [1,240]        | 0                |
| qp_map_enable    | Enables QP map for ROI encoding | [0,1]      | 0                |

##### AVBR Description

AVBR stands for Average Variable Bitrate, allocating lower bitrates for simple scenes and sufficient bitrates for complex scenes—similar to VBR—while maintaining an average bitrate close to the target over time, thus controlling output file size—similar to CBR. It is a compromise between CBR and VBR, producing relatively stable bitrate and image quality. Below are parameter descriptions for AVBR mode:

| **Parameter**       | **Description**                                                                                                                                                                                                                                                 | **Range**          | **Default** |
| ------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ------------------ | ---------------- |
| intra_period        | I-frame interval                                                                                                                                                                                                                                                | [0,2047]           | 28               |
| intra_qp            | QP value for I-frames                                                                                                                                                                                                                                           | [0,51]             | 0                |
| bit_rate            | Target average bitrate, in kbps                                                                                                                                                                                                                                 | [0,700000]         | 0                |
| frame_rate          | Target frame rate, in fps                                                                                                                                                                                                                                       | [1,240]            | 30               |
| initial_rc_qp       | Initial QP value for rate control; if outside [0,51], the encoder internally determines the initial value                                                                                                                                                        | [0,63]             | 63               |
| vbv_buffer_size     | VBV buffer size in ms; actual VBV buffer size = bit_rate * vbv_buffer_size / 1000 (kb). Buffer size affects encoding quality and bitrate control accuracy: smaller buffers yield higher bitrate control precision but lower image quality; larger buffers improve image quality but increase bitrate fluctuation. | [10,3000]          | 3000             |
| ctu_level_rc_enable | H.264/H.265 rate control can operate at CTU level for higher bitrate control precision at the cost of encoding quality. This mode cannot be used with ROI encoding; it is automatically disabled when ROI encoding is enabled. | [0,1]              | 0                |
| min_qp_I            | Minimum QP value for I-frames                                                                                                                                                                                                                                   | [0,51]             | 8                |
| max_qp_I            | Maximum QP value for I-frames                                                                                                                                                                                                                                   | [0,51]             | 51               |
| min_qp_P            | Minimum QP value for P-frames                                                                                                                                                                                                                                   | [0,51]             | 8                |
| max_qp_P            | Maximum QP value for P-frames                                                                                                                                                                                                                                   | [0,51]             | 51               |
| min_qp_B            | Minimum QP value for B-frames                                                                                                                                                                                                                                   | [0,51]             | 8                |
| max_qp_B            | Maximum QP value for B-frames                                                                                                                                                                                                                                   | [0,51]             | 51               |
| hvs_qp_enable       | H.264/H.265 rate control can operate at sub-CTU level, adjusting sub-macroblock QP values to improve subjective image quality.                                                                                                                                   | [0,1]              | 1                |
| hvs_qp_scale        | Effective when hvs_qp_enable is enabled; represents QP scaling factor.                                                                                                                                                                                           | [0,4]              | 2                |
| max_delta_qp        | Effective when hvs_qp_enable is enabled; specifies maximum allowable deviation for HVS QP values.                                                                                                                                                                | [0,51]             | 10               |
| qp_map_enable       | Enables QP map for ROI encoding.                                                                                                                                                                                                                                | [0,1]              | 0                |

##### FixQp Description

FixQp assigns fixed QP values to each I-frame and P-frame, with separate values allowed for I/P frames. Below are parameter descriptions for FixQp mode:

| **Parameter**    | **Description**      | **Range** | **Default** |
| ---------------- | ------------------- | ------------------ | ---------------- |
| intra_period     | I-frame interval    | [0,2047]           | 28               |
| frame_rate       | Target frame rate, in fps | [1,240]      | 30               |
| force_qp_I       | Forced QP value for I-frames | [0,51]    | 0                |
| force_qp_P       | Forced QP value for P-frames | [0,51]    | 0                |
| force_qp_B       | Forced QP value for B-frames | [0,51]    | 0                |

##### QPMAP Description

QPMAP allows specifying QP values for each block within a frame: 32x32 for H.265 and 16x16 for H.264. Below are parameter descriptions for QPMAP mode:

| **Parameter**      | **Description**                                                                                                       | **Range**                                                                 | **Default** |
| ------------------ | -------------------------------------------------------------------------------------------------------------------- | ---------------------------------------------------------------------------------- | ---------------- |
| intra_period       | I-frame interval                                                                                                     | [0,2047]                                                                           | 28               |
| frame_rate         | Target frame rate, in fps                                                                                            | [1,240]                                                                            | 30               |
| qp_map_array       | Specifies QP map table; for H.265, sub-CTU size is 32x32—each sub-CTU requires one QP value (1 byte), ordered in raster scan. | Pointer address                                                                    | NULL             |
| qp_map_array_count | Specifies size of QP map table.                                                                                      | [0, MC_VIDEO_MAX_SUB_CTU_NUM] && (ALIGN64(picWidth)>>5) * (ALIGN64(picHeight)>>5) | 0                |

### Debugging Methods

#### Encoding Quality Tuning

In current customer usage scenarios involving video encoding with the codec, CBR mode is commonly selected. In complex scenes, hardware automatically increases bitrate to maintain video quality, resulting in larger-than-expected output files. To balance video quality and actual bitrate, settings for bit_rate and max_qp_I/P must be coordinated. Below shows actual bitrate and QP under all-I-frame mode with a target bitrate of 15000 kbps across different scene complexities and max_qp_I values (data due to varying scene complexity):

| Scene & Parameters                         | Outdoor Daytime Complex Scene bitrate(15000) max_qp_I(35) | Outdoor Daytime Complex Scene bitrate(15000) max_qp_I(38) | Outdoor Daytime Complex Scene bitrate(15000) max_qp_I(39) |
| ------------------------------------------ | -------------------------------------------------------- | -------------------------------------------------------- | -------------------------------------------------------- |
| Bit allocation (bps) (higher = better quality) | 60300045                                                 | 42186920                                                 | 35898230                                                 |
| Avg QP (lower = better quality)            | 35                                                       | 38                                                       | 39                                                       |

#### GOP Structure Description

H.264/H.265 encoding supports configurable GOP structures, allowing users to select from three preset GOP structures or define custom GOP structures.

A GOP structure table defines a periodic GOP pattern applied throughout the encoding process. Elements within a single structure table are described below. Reference frames for each picture can be specified; if frames after an IDR reference frames before the IDR, the encoder internally handles this to prevent cross-IDR referencing—users need not manage this. When defining custom GOP structures, users must specify the number of structure tables (up to 3), ordered by decoding sequence.

Below describes elements within the structure table:

| Element            | Description                                                                                                                                                                                                 |
| ------------------ | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Type               | Frame type (I, P, B)                                                                                                                                                                                     |
| POC                | Display order within GOP, range [1, gop_size].                                                                                                                                                           |
| QPoffset           | Quantization parameter for pictures in custom GOP                                                                                                                                                        |
| NUM_REF_PIC_L0     | Indicates multi-reference pictures for P-frames; valid only when PIC_TYPE is P.                                                                                                                          |
| temporal_id        | Temporal layer of frame; frames cannot predict from frames with higher temporal_id (0~6).                                                                                                                |
| 1st_ref_POC        | POC of first reference picture in L0                                                                                                                                                                     |
| 2nd_ref_POC        | For B-frames, first reference picture POC is in L1; for P-frames, second reference picture POC is in L0. reference_L1 and reference_L0 may share the same POC in B-slices, but different POCs are recommended for compression efficiency. |

##### Preset GOP Structures

Nine preset GOP structures are supported:

| Index | GOP Structure | Low Latency (encoding order = display order) | GOP Size | Encoding Order               | Min Source Frame Buffer Count | Min Decoded Picture Buffer Count | Period Requirement (I-frame interval) |
| ----- | ------------- | -------------------------------------------- | -------- | ---------------------------- | ----------------------------- | -------------------------------- | ------------------------------------- |
| 1     | I             | Yes                                          | 1        | I0-I1-I2…                    | 1                             | 1                                |                                       |
| 2     | P             | Yes                                          | 1        | P0-P1-P2…                    | 1                             | 2                                |                                       |
| 3     | B             | Yes                                          | 1        | B0-B1-B2…                    | 1                             | 3                                |                                       |
| 4     | BP            | No                                           | 2        | B1-P0-B3-P2…                 | 1                             | 3                                |                                       |
| 5     | BBBP          | Yes                                          | 1        | B2-B1-B3-P0…                 | 7                             | 4                                |                                       |
| 6     | PPPP          | Yes                                          | 4        | P0-P1-P2-P3…                 | 1                             | 2                                |                                       |
| 7     | BBBB          | Yes                                          | 4        | B0-B1-B2-B3…                 | 1                             | 3                                |                                       |
| 8     | BBBB BBBB     | Yes                                          | 1        | B3-B2-B4-B1-B6-B5-B7-B0…     | 12                            | 5                                |                                       |
| 9     | P             | Yes                                          | 1        | P0                           | 1                             | 2                                |                                       |

GOP Preset 1

- Contains only I-frames with no inter-frame referencing;
- Low latency;

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/03_multimedia_development/02_S100/codec/b02cc41ab083664ba3f8a3bef1543afa.png)

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/03_multimedia_development/02_S100/codec/fa1da95bc8801b2d6225b2abf1b2f2d3.png)GOP Preset 2

- Contains only I-frames and P-frames;
- P-frames reference 2 forward reference frames;
- Low latency;

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/03_multimedia_development/02_S100/codec/e3c2f773a89f6ee2fe2dab03200b6fd0.png)

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/03_multimedia_development/02_S100/codec/8fa5f892bd7282e82ac8ed96011c943d.png)

GOP Preset 3

- Contains only I-frames and B-frames;
- B-frames reference 2 forward reference frames;
- Low latency;

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/03_multimedia_development/02_S100/codec/03bbdf35dc3e2a1b38f9e05d7038d064.png)

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/03_multimedia_development/02_S100/codec/e1b5707ea0c32b6a0c1658527a6186dd.png)

GOP Preset 4

- Contains only I-frames, P-frames, and B-frames;
- P-frames reference 2 forward reference frames;
- B-frames reference 1 forward reference frame and 1 backward reference frame;

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/03_multimedia_development/02_S100/codec/17e10e6a27db202fe9a0c2b5f3d5dd50.png)

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/03_multimedia_development/02_S100/codec/972bbe22d2e7364c1c0a3db03f57343e.png)

GOP Preset 5

- Contains only I-frames, P-frames, and B-frames;
- P-frames reference 2 forward reference frames;
- B-frames reference 1 forward reference frame and 1 backward reference frame, where the backward reference frame can be either a P-frame or a B-frame;

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/03_multimedia_development/02_S100/codec/16ad2d15f0b22a91fda1450747a18422.png)

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/03_multimedia_development/02_S100/codec/8ff1393cdbb997c8768ea2f9f00c3c8b.png)

GOP Preset 6

- Contains only I-frames and P-frames;
- P-frames reference 2 forward reference frames;
- Low latency;

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/03_multimedia_development/02_S100/codec/a5fbffa7c85a3423729f06d45f83a601.png)

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/03_multimedia_development/02_S100/codec/1e88c6cbacb8fff86f5d5fc301e01abd.png)

GOP Preset 7

- Contains only I-frames and B-frames;
- B-frames reference 2 forward reference frames;
- Low latency;

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/03_multimedia_development/02_S100/codec/40cd6c4fa7cf66f9bf14c3675cb7ef20.png)

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/03_multimedia_development/02_S100/codec/be7fe30d2685e27e2b36f305ef745eb4.png)

GOP Preset 8

- Contains only I-frames and B-frames;
- B-frames reference 1 forward reference frame and 1 backward reference frame;

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/03_multimedia_development/02_S100/codec/9c46efaf2a9106bcee2468098e209b1f.png)

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/03_multimedia_development/02_S100/codec/d016b90fa0a06e183b6871bc430a8714.png)

GOP Preset 9

- Contains only I-frames and P-frames;
- P-frames reference 1 forward reference frame;
- Low latency;

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/03_multimedia_development/02_S100/codec/0bc1b9d3e73b4037b64236650738b5cd.png)

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/03_multimedia_development/02_S100/codec/937b45950423ff5006e378cb510d695d.png)

#### VPU Debugging Method

The VPU (Video Processing Unit) is a dedicated visual processing unit capable of efficiently handling video content. The VPU can perform encoding and decoding of H.265 video formats. Users can obtain the encoded/decoded bitstream through the interfaces provided by Codec.

##### Encoding Status

Encoding Debug Information

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

Parameter Explanation

| Debug Info Group         | Status Parameter             | Description                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                |
| ------------------------ | ---------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| encode enc param         | Basic Encoding Parameters    | enc_idx: Encoding instance index<br/>enc_id: Encoding type<br/>profile: Profile type<br/>level: H.265 level type<br/>width: Encoding width<br/>height: Encoding height<br/>pix_fmt: Input frame pixel format<br/>fbuf_count: Number of input frame buffers<br/>extern_buf_flag: Whether external input buffers are used<br/>bsbuf_count: Number of bitstream buffers<br/>bsbuf_size: Bitstream buffer size<br/>mirror: Whether mirroring is enabled<br/>rotate: Whether rotation is enabled                                                                                                                                                                                            |
| encode h265cbr param     | CBR Rate Control Parameters  | enc_idx: Encoding instance index<br/>rc_mode: Rate control mode<br/>intra_period: I-frame interval<br/>intra_qp: QP value for I-frames<br/>bit_rate: Bitrate<br/>frame_rate: Frame rate<br/>initial_rc_qp: Initial QP value<br/>vbv_buffer_size: VBV buffer size<br/>ctu_level_rc_enalbe: Whether rate control operates at CTU level<br/>min_qp_I: Minimum QP for I-frames<br/>max_qp_I: Maximum QP for I-frames<br/>min_qp_P: Minimum QP for P-frames<br/>max_qp_P: Maximum QP for P-frames<br/>min_qp_B: Minimum QP for B-frames<br/>max_qp_B: Maximum QP for B-frames<br/>hvs_qp_enable: Whether rate control operates at sub-CTU level<br/>hvs_qp_scale: QP scaling factor<br/>qp_map_enable: Enable QP map for ROI encoding<br/>max_delta_qp: Maximum allowable deviation for HVS QP values |
| encode gop param         | GOP Parameters               | enc_idx: Encoding instance index<br/>enc_id: Encoding type<br/>gop_preset_idx: Selected preset GOP structure<br/>custom_gop_size: GOP size when using custom GOP<br/>decoding_refresh_type: Specific type of IDR frame                                                                                                                                                                                                                                                                                                                                                                |
| encode intra refresh     | Intra Refresh Parameters     | enc_idx: Encoding instance index<br/>enc_id: Encoding type<br/>intra_refresh_mode: Intra refresh mode<br/>intra_refresh_arg: Intra refresh argument                                                                                                                                                                                                                                                                                                                                                                                                               |
| encode longterm ref      | Long-Term Reference Parameters | enc_idx: Encoding instance index<br/>enc_id: Encoding type<br/>use_longterm: Enable long-term reference frames<br/>longterm_pic_period: Period for long-term reference frames<br/>longterm_pic_using_period: Period for referencing long-term reference frames                                                                                                                                                                                                                                                                                                                                                               |
| encode roi_params        | ROI Parameters               | enc_idx: Encoding instance index<br/>enc_id: Encoding type<br/>roi_enable: Enable ROI encoding<br/>roi_map_array_count: Number of elements in ROI map                                                                                                                                                                                                                                                                                                                                                                                                               |
| encode mode_decision 1   | Block Mode Decision Parameters 1 | Various mode selection parameter values, including pu04_delta_rate, pu08_delta_rate, etc.                                                                                                                                                                                                                                                                                                                                                                                                                                                          |
| encode mode_decision 2   | Block Mode Decision Parameters 2 | Various mode selection parameter values, including pu32_intra_planar_delta_rate, pu32_intra_dc_delta_rate, etc.                                                                                                                                                                                                                                                                                                                                                                                                                                   |
| encode h265_transform    | Transform Parameters         | enc_idx: Encoding instance index<br/>enc_id: Encoding type<br/>chroma_cb_qp_offset: QP offset for Cb component<br/>chroma_cr_qp_offset: QP offset for Cr component<br/>user_scaling_list_enable: Enable user-defined scaling list                                                                                                                                                                                                                                                                                                                                           |
| encode h265_pred_unit    | Prediction Unit Parameters   | enc_idx: Encoding instance index<br/>enc_id: Encoding type<br/>intra_nxn_enable: Enable intra NXN PUs<br/>constrained_intra_pred_flag: Whether intra prediction is constrained<br/>strong_intra_smoothing_enabled_flag: Whether bidirectional linear interpolation is used in filtering<br/>max_num_merge: Number of merge candidates                                                                                                                                                                                                                                                                                         |
| encode h265 timing       | Timing Parameters            | enc_idx: Encoding instance index<br/>enc_id: Encoding type<br/>vui_num_units_in_tick: Number of time units per tick<br/>vui_time_scale: Number of time units per second<br/>vui_num_ticks_poc_diff_one_minus1: Number of clock ticks corresponding to a picture order count difference of 1                                                                                                                                                                                                                                                                                                                    |
| encode h265 slice params | Slice Parameters             | enc_idx: Encoding instance index<br/>enc_id: Encoding type<br/>h265_independent_slice_mode: Independent slice encoding mode<br/>h265_independent_slice_arg: Size of independent slices<br/>h265_dependent_slice_mode: Dependent slice encoding mode<br/>h265_dependent_slice_arg: Size of dependent slices                                                                                                                                                                                                                                                                                          |
| encode h265 deblk filter | Deblocking Filter Parameters | enc_idx: Encoding instance index<br/>enc_id: Encoding type<br/>slice_deblocking_filter_disabled_flag: Whether in-slice deblocking filtering is disabled<br/>slice_beta_offset_div2: β deblocking parameter offset for current slice<br/>slice_tc_offset_div2: tC deblocking parameter offset for current slice<br/>slice_loop_filter_across_slices_enabled_flag: Whether cross-slice boundary filtering is enabled                                                                                                                                                                                                                                        |
| encode h265 sao param    | SAO Parameters               | enc_idx: Encoding instance index<br/>enc_id: Encoding type<br/>sample_adaptive_offset_enabled_flag: Whether sample adaptive offset is applied to reconstructed pictures after deblocking                                                                                                                                                                                                                                                                                                                                                                                  |
| encode status            | Current Encoding Status Parameters | enc_idx: Encoding instance index<br/>enc_id: Encoding type<br/>cur_input_buf_cnt: Number of currently used input buffers<br/>cur_output_buf_cnt: Number of currently used output buffers<br/>left_recv_frame: Remaining frames to receive (valid only when receive_frame_number is set)<br/>left_enc_frame: Remaining frames to encode (valid only when receive_frame_number is set)<br/>total_input_buf_cnt: Total number of input buffers used so far<br/>total_output_buf_cnt: Total number of output buffers used so far<br/>fps: Current frame rate                                                                                                             |

##### Decoding Status

Decoding Debug Information

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

Parameter Explanation

| Debug Info Group      | Status Parameter         | Description                                                                                                                                                                                                                                                                             |
| --------------------- | ------------------------ | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| decode param          | Basic Decoding Parameters | dec_idx: Decoding instance index<br/>dec_id: Decoding type<br/>feed_mode: Data feeding mode<br/>pix_fmt: Output pixel format<br/>bitstream_buf_size: Input bitstream buffer size<br/>bitstream_buf_count: Number of input bitstream buffers<br/>frame_buf_count: Number of output frame buffers                                                     |
| h265 decode param     | H.265 Decoding Basic Parameters | dec_idx: Decoding instance index<br/>dec_id: Decoding type<br/>reorder_enable: Enable decoder to output frames in display order<br/>skip_mode: Enable frame decode skip mode<br/>bandwidth_Opt: Enable bandwidth optimization mode<br/>cra_as_bla: Treat CRA as BLA<br/>dec_temporal_id_mode: Temporal ID selection mode<br/>target_dec_temporal_id_plus1: Specified temporal ID value |
| decode frameinfo      | Decoded Output Frame Info | dec_idx: Decoding instance index<br/>dec_id: Decoding type<br/>display_width: Display width<br/>display_height: Display height                                                                                                                                                                                        |
| decode status         | Current Decoding Status Parameters | dec_idx: Decoding instance index<br/>dec_id: Decoding type<br/>cur_input_buf_cnt: Number of currently used input buffers<br/>cur_output_buf_cnt: Number of currently used output buffers<br/>total_input_buf_cnt: Total number of input buffers used so far<br/>total_output_buf_cnt: Total number of output buffers used so far<br/>fps: Current frame rate                                   |

#### JPU Debugging Method

The JPU (JPEG Processing Unit) is primarily used to perform JPEG/MJPEG encoding and decoding. Users can input YUV data for encoding or JPEG images for decoding via the CODEC interface, and obtain the encoded JPEG images or decoded YUV data after JPU processing.

##### Encoding Status

Encoding Debug Information  

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

Parameter Explanation

| Debug Info Group | Status Parameter        | Description                                                                                                                                                                                                                                                                                                                                                                                                    |
| ---------------- | ----------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| encode param     | Basic Encoding Parameters | enc_idx: Encoding instance<br/>enc_id: Encoding type<br/>width: Image width<br/>height: Image height<br/>pix_fmt: Pixel format<br/>fbuf_count: Number of input framebuffer buffers<br/>extern_buf_flag: Whether user-allocated input buffers are used<br/>bsbuf_count: Number of output bitstream buffers<br/>bsbuf_size: Size of output bitstream buffer<br/>mirror: Whether mirroring is enabled<br/>rotate: Whether rotation is enabled |
| encode rc param  | MJPEG Rate Control Parameters | enc_idx: Encoding instance<br/>rc_mode: Rate control mode<br/>frame_rate: Target frame rate<br/>quality_factor: Quantization factor                                                                                                                                                                                                                                                                              |
| encode status    | Current Encoding Status Parameters | enc_idx: Encoding instance ID<br/>enc_id: Encoding type<br/>cur_input_buf_cnt: Number of currently used input buffers<br/>cur_output_buf_cnt: Number of currently used output buffers<br/>left_recv_frame: Remaining frames to receive (valid only if receive_frame_number is set)<br/>left_enc_frame: Remaining frames to encode (valid only if receive_frame_number is set)<br/>total_input_buf_cnt: Total number of input buffers used so far<br/>total_output_buf_cnt: Total number of output buffers used so far<br/>fps: Current frame rate |

##### Decoding Status

Decoding Debug Information

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

Parameter Explanation

| Debug Info Group    | Status Parameter       | Description                                                                                                                                                                                                                                                 |
| ------------------- | ---------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| decode param        | Basic Decoding Parameters | dec_idx: Decoding instance<br/>dec_id: Decoding type<br/>feed_mode: Feed mode<br/>pix_fmt: Pixel format<br/>bitstream_buf_size: Size of input bitstream buffer<br/>bitstream_buf_count: Number of input bitstream buffers<br/>frame_buf_count: Number of output framebuffer buffers<br/>mirror: Whether mirroring is enabled<br/>rotate: Whether rotation is enabled |
| decode frameinfo    | Decoded Frame Info     | dec_idx: Decoding instance ID<br/>dec_id: Decoding type<br/>display_width: Display width<br/>display_height: Display height                                                                                                                                   |
| decode status       | Current Decoding Status Parameters | dec_idx: Decoding instance ID<br/>dec_id: Decoding type<br/>cur_input_buf_cnt: Number of currently used input buffers<br/>cur_output_buf_cnt: Number of currently used output buffers<br/>total_input_buf_cnt: Total number of input buffers used so far<br/>total_output_buf_cnt: Total number of output buffers used so far<br/>fps: Current frame rate |

### Typical Scenarios

#### Single-Stream Encoding

The single-stream encoding scenario is illustrated below. Scenario 0 is a simple case: YUV video/image files are read from eMMC, encoded by VPU hardware into H.26x bitstreams or by JPU hardware into JPEG images, and finally saved back to eMMC as files. Scenario 1 is a more complex pipeline where camera-captured data is encoded, compressed, and either stored or transmitted over network/PCIe.

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/03_multimedia_development/02_S100/codec/788c1e3b839232111ccd53d35d25e278.png)

#### Single-Stream Decoding

The single-stream decoding scenario is illustrated below. Scenario 0 is a simple case: H.26x bitstreams or JPEG image files are read from eMMC, decoded by VPU or JPU hardware into YUV data, and saved back to eMMC as files. Scenario 1 is a complex pipeline where encoded video or image data is received via network or PCIe, decoded by VPU or JPU hardware, and displayed via IDE.

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/03_multimedia_development/02_S100/codec/e50f9bf3c4d1ecfbd36b354f9009e8bc.png)

#### Multi-Stream Encoding

The multi-stream encoding scenario is shown below. Scenario 0 is a simple file-input case. Scenario 1 is a complex pipeline involving multiple modules. Note that in Scenario 1, the capabilities and limitations of all modules in the pipeline must be carefully considered.

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/03_multimedia_development/02_S100/codec/272e3467c640af379d1b4c0a1de27eae.png)

#### Multi-Stream Decoding

The multi-stream decoding scenario is shown below. Scenario 0 is a simple file-input case. Scenario 1 is a complex pipeline involving multiple modules. Note that in Scenario 1, the capabilities and limitations of all modules in the pipeline must be carefully considered.

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/03_multimedia_development/02_S100/codec/04f0aba90a1d65017dfeb90f9afa43e2.png)

## Codec API

### MediaCodec Interface Description

The MediaCodec module is primarily used for audio, video, and JPEG image encoding/decoding. This module provides a set of interfaces to facilitate user input of data to be processed and retrieval of processed data. It supports multiple concurrent encoding or decoding instances. Video and JPEG image encoding/decoding are hardware-accelerated, requiring users to link against `libmultimedia.so`. Audio encoding/decoding is software-based using FFmpeg interfaces, requiring users to link against `libffmedia.so`. The table below lists the video and audio codecs supported on RDK S100. Note that AAC encoding/decoding requires a license; users must obtain proper authorization before enabling related features.

H.265 supports up to Main profile and Level 5.1, with Main-tier support. H.264 supports up to High profile and Level 5.2. MJPEG/JPEG supports ISO/IEC 10918-1 Baseline sequential. Supported audio codecs include: G.711 A-law/Mu-law, G.729 ADPCM, ADPCM IMA WAV, FLAC, AAC LC, AAC Main, AAC SSR, AAC LTP, AAC LD, AAC HE, and AAC HEv2 (AAC requires a license).

Additionally, video and image sources include two types: images from VIO input and user-provided YUV data (which may be loaded from files or received over a network). Audio sources include two types: audio from MIC input (digitized by the Audio Codec) and user-provided PCM data (which may be loaded from files or received over a network).

#### GOP

H.264/H.265 encoding supports GOP structure configuration. Users can select from nine predefined GOP structures or define custom GOP structures.

##### GOP Structure Table

A GOP structure table defines a periodic GOP pattern used throughout the encoding process. Elements in a single structure table are described below. Users can specify reference frames for each picture. If a frame after an IDR references a frame before the IDR, the encoder automatically handles this to avoid invalid references—users need not worry about such cases. When defining a custom GOP, users must specify the number of structure tables (up to 3), and tables must be ordered in decoding sequence.

| Element        | Description                                                                                                                                                                                                                                                                                                                      |
| -------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Type           | Slice type (I, P)                                                                                                                                                                                                                                                                                                               |
| POC            | Display order of the frame within a GOP, ranging from 1 to GOP size                                                                                                                                                                                                                                                              |
| QPoffset       | Quantization parameter offset for the picture in the custom GOP                                                                                                                                                                                                                                                                  |
| NUM_REF_PIC_L0 | Flag to enable multi-reference pictures for P frames. Valid only if PIC_TYPE is P                                                                                                                                                                                                                                                |
| temporal_id    | Temporal layer of the frame. A frame cannot reference another frame with a higher temporal_id (range: 0–6).                                                                                                                                                                                                                      |
| 1st_ref_POC    | POC of the first reference picture in L0                                                                                                                                                                                                                                                                                         |
| 2nd_ref_POC    | POC of the first reference picture in L1 if Type is B; POC of the second reference picture in L0 if Type is P. Note that reference_L1 can share the same POC as reference_L0 in B slices, but for better compression efficiency, it is recommended that reference_L1 and reference_L0 have different POCs. |

##### Predefined GOP Structures

| Index | GOP Structure | Low Delay (encoding order = display order) | GOP Size | Encoding Order              | Minimum Source Frame Buffer | Minimum Decoded Picture Buffer | Intra Period (I Frame Interval) Requirement |
| ----- | ------------- | ------------------------------------------ | -------- | --------------------------- | --------------------------- | ------------------------------ | ------------------------------------------- |
| 1     | I             | Yes                                        | 1        | I0-I1-I2…                  | 1                           | 1                              |                                             |
| 2     | P             | Yes                                        | 1        | P0-P1-P2…                  | 1                           | 2                              |                                             |
| 3     | B             | Yes                                        | 1        | B0-B1-B2…                  | 1                           | 3                              |                                             |
| 4     | BP            | No                                         | 2        | B1-P0-B3-P2…               | 1                           | 3                              | Multiple of 2                               |
| 5     | BBBP          | Yes                                        | 1        | B2-B1-B3-P0…               | 7                           | 4                              | Multiple of 4                               |
| 6     | PPPP          | Yes                                        | 4        | P0-P1-P2-P3…               | 1                           | 2                              |                                             |
| 7     | BBBB          | Yes                                        | 4        | B0-B1-B2-B3…               | 1                           | 3                              |                                             |
| 8     | BBBB BBBB     | Yes                                        | 1        | B3-B2-B4-B1-B6-B5-B7-B0…   | 12                          | 5                              | Multiple of 8                               |
| 9     | P             | Yes                                        | 1        | P0                          | 1                           | 2                              |                                             |

The following describes the nine predefined GOP structures.

GOP Preset 1

- Contains only I-frames with no inter-frame references;
- Low latency;

![gop1](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/03_multimedia_development/02_S100/codec/gop1.png)

![gop2](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/03_multimedia_development/02_S100/codec/gop2.png)

GOP Preset 2

- Contains only I-frames and P-frames;
- P-frames reference two previous frames;
- Low latency;

![gop3](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/03_multimedia_development/02_S100/codec/gop3.png)

![gop4](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/03_multimedia_development/02_S100/codec/gop4.png)

GOP Preset 3

- Contains only I-frames and B-frames;
- B-frames reference two previous frames;
- Low latency;

![gop5](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/03_multimedia_development/02_S100/codec/gop5.png)

![gop6](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/03_multimedia_development/02_S100/codec/gop6.png)

GOP Preset 4

- Contains I-, P-, and B-frames;
- P-frames reference two previous frames;
- B-frames reference one previous and one future frame;

![gop7](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/03_multimedia_development/02_S100/codec/gop7.png)

![gop8](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/03_multimedia_development/02_S100/codec/gop8.png)

GOP Preset 5

- Contains I-, P-, and B-frames;
- P-frames reference two previous frames;
- B-frames reference one previous and one future frame (which can be either P or B);

![gop9](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/03_multimedia_development/02_S100/codec/gop9.png)

![gop10](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/03_multimedia_development/02_S100/codec/gop10.png)

GOP Preset 6

- Contains only I-frames and P-frames;
- P-frames reference two previous frames;
- Low latency;

![gop11](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/03_multimedia_development/02_S100/codec/gop11.png)

![gop12](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/03_multimedia_development/02_S100/codec/gop12.png)

GOP Preset 7

- Contains only I-frames and B-frames;
- B-frames reference two previous frames;
- Low latency;

![gop13](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/03_multimedia_development/02_S100/codec/gop13.png)

![gop14](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/03_multimedia_development/02_S100/codec/gop14.png)

GOP Preset 8

- Contains only I-frames and B-frames;
- B-frames reference one previous and one future frame;

![gop15](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/03_multimedia_development/02_S100/codec/gop15.png)

![gop16](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/03_multimedia_development/02_S100/codec/gop16.png)
  
  GOP Preset 9

- Contains only I-frames and P-frames;
- Each P-frame references one forward reference frame;
- Low latency;

![gop17](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/03_multimedia_development/02_S100/codec/gop17.png)

![gop18](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/03_multimedia_development/02_S100/codec/gop18.png)



#### Long-term Reference Frames

Users can specify the interval for long-term reference frames and the interval at which frames reference long-term reference frames, as shown in the figure below:

![reference_frame](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/03_multimedia_development/02_S100/codec/reference_frame.png)



#### Intra Refresh

Intra Refresh mode enhances error resilience by periodically inserting intra-coded MBs/CTUs within non-I frames. It provides the decoder with more recovery points to prevent image corruption caused by temporal errors. Users can specify the number of consecutive rows, columns, or step size of MBs/CTUs to force the encoder to insert intra-coded units. Additionally, users may specify the size of intra-coded units, allowing the encoder to internally determine which blocks require intra coding.



#### Rate Control

MediaCodec supports bitrate control for H.264, H.265, and MJPEG codecs. For H.264 and H.265 encoding channels, it supports five rate control modes: CBR, VBR, AVBR, FixQp, and QpMap. For MJPEG encoding channels, it supports FixQp mode.  
- CBR ensures a stable overall encoded bitrate;  
- VBR maintains consistent visual quality;  
- AVBR balances both bitrate and quality, producing a relatively stable stream in terms of both bitrate and image quality;  
- FixQp fixes the QP value for every I-frame and P-frame;  
- QpMap assigns a specific QP value to each block within a frame (block size is 32×32 for H.265 and 16×16 for H.264).

For CBR and AVBR, the encoder internally determines an appropriate QP value for each frame to maintain constant bitrate. The encoder supports three levels of rate control: frame-level, CTU/MB-level, and subCTU/subMB-level.  
- Frame-level control calculates a single QP per frame based on the target bitrate to ensure bitrate stability.  
- CTU/MB-level control assigns a QP to each 64×64 CTU or 16×16 MB according to its target bitrate, achieving finer bitrate control, though frequent QP adjustments may cause visual quality instability.  
- subCTU/subMB-level control assigns a QP to each 32×32 subCTU or 8×8 subMB. Complex blocks receive higher QP values, while static blocks receive lower QP values—since the human eye is more sensitive to static regions than complex ones. Detection of complex vs. static regions relies on internal hardware modules. This level aims to improve subjective visual quality while maintaining bitrate stability, resulting in higher SSIM scores but potentially lower PSNR scores.



#### ROI

ROI encoding works similarly to QpMap: users must assign a QP value to each block in raster-scan order, as illustrated below:

![roi_map](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/03_multimedia_development/02_S100/codec/roi_map.png)

For H.264, each block is 16×16; for H.265, it is 32×32. In the ROI map, each QP value occupies one byte, ranging from 0 to 51. ROI encoding can operate alongside CBR or AVBR.  
- When CBR/AVBR is disabled, the actual QP for each block equals the value specified in the ROI map.  
- When CBR/AVBR is enabled, the actual QP for block *i* is calculated as:

QP(i) = MQP(i) + RQP(i) - ROIAvgQP

where:  
- MQP is the value from the ROI map,  
- RQP is the QP determined by the encoder’s internal rate control,  
- ROIAvgQP is the average QP value across the entire ROI map.



#### Input/Output Buffer Management

MediaCodec uses two types of buffers: input and output. Typically, these buffers are allocated uniformly by MediaCodec via the ION interface, so users do not need to manage allocation directly. Instead, users should call `dequeue` to obtain an available buffer before use and `queue` to return it after processing.  

However, to reduce unnecessary buffer copying in certain scenarios—e.g., when using PYM’s output buffer directly as MediaCodec input (since PYM allocates this buffer internally via ION)—MediaCodec also supports user-allocated input buffers. In such cases, users must allocate physically contiguous buffers via the ION interface and set the `external_frame_buf` field in `media_codec_context_t` before configuring MediaCodec.  

Note: Even when providing external input buffers, users must still perform `dequeue` to retrieve buffer metadata (e.g., virtual and physical addresses), populate this information, and then call `queue`.

![buffer](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/03_multimedia_development/02_S100/codec/buffer.png)



#### Frame Rate Control

MediaCodec does not currently support internal frame rate control.  
- If users do **not** enable direct VIO–MediaCodec interaction via `hb_mm_mc_set_camera`, they must manually control the input buffer frame rate.  
- If VIO–MediaCodec interaction **is** enabled, users only manage output buffers; input buffering is handled automatically. In this mode, MediaCodec performs no frame rate control on input buffers. If encoding stalls or the input buffer queue becomes full, MediaCodec will wait until space becomes available before proceeding.



#### Frame Skip Configuration

Users can call `hb_mm_mc_skip_pic` to set the next queued input frame to "skip" mode. This mode applies only to non-I frames. In skip mode, the encoder ignores the input frame and instead generates the reconstructed frame by reusing the previous frame’s reconstruction, encoding the current input as a P-frame.



#### JPEG Codec Limitations

- For JPEG/MJPEG **encoding**:  
  - With YUV420/YUV422 input: width must be 16-byte aligned, height 8-byte aligned.  
  - With YUV440/YUV444/YUV400 input: both width and height must be 8-byte aligned.  
  - If cropping is applied, the crop origin (x, y) must also be 8-byte aligned.

- For JPEG/MJPEG **encoding with 90°/270° rotation**:  
  - YUV420: width 16-aligned, height 8-aligned.  
  - YUV422/YUV440: width 16-aligned, height 16-aligned.  
  - YUV444/YUV400: width and height 8-aligned.  
  - With cropping:  
    - YUV420: crop width 16-aligned, crop height 8-aligned.  
    - YUV422/YUV440: crop width 16-aligned, crop height 16-aligned.  
    - YUV444/YUV400: crop width and height 8-aligned.

- For JPEG/MJPEG **encoding with 90°/270° rotation**:  
  - YUV422 input becomes YUV440 after rotation.  
  - YUV440 input becomes YUV422 after rotation.

- For JPEG/MJPEG **decoding with rotation or mirroring**:  
  - Output YUV format must match input format, **except**:  
    - When decoding YUV422 JPEG/MJPEG with 90°/270° rotation, output format must be YUV440p, YUYV, YVYU, UYVY, or VYUY.

- JPEG/MJPEG **decoding**: Rotation/mirroring cannot be used simultaneously with cropping.

- JPEG/MJPEG **decoding**: Output buffer dimensions must align with the input format’s MCU width and height. If cropping is enabled, all crop parameters (origin x/y and width/height) must also align with the MCU dimensions.  
  (MCU sizes: 420 → 16×16, 422 → 16×8, 440 → 8×16, 400 → 8×8, 444 → 8×8.)

- JPEG/MJPEG **decoding**: Output format Packed YUV444 requires input in YUV444 format.

- JPEG/MJPEG **decoding**: Only supports `MC_FEEDING_MODE_FRAME_SIZE` mode.

- JPEG **encoding**: If the user specifies the bitstream buffer size, an additional 4KB must be allocated.

- JPEG **encoding**: Since the encoder processes data in 16×16 units, non-aligned input may result in padding differences in the final encoded data. This does not affect valid pixel data but is a hardware limitation; caution is advised when performing MD5 comparisons.

### API Reference

#### hb_mm_mc_get_descriptor

【Function Declaration】

const media_codec_descriptor_t* hb_mm_mc_get_descriptor(media_codec_id_t codec_id);

【Parameters】

- [IN] media_codec_id_t codec_id: Specifies the codec type.

【Return Value】

- Non-null: Codec descriptor containing information.
- NULL: No descriptor found for the given codec ID.

【Description】

Retrieves codec information supported by MediaCodec based on `codec_id`, including codec name, detailed description, MIME type, and supported profiles.

【Example】

```c
#include "hb_media_codec.h"
#include "hb_media_error.h"
int main(int argc, char *argv[])
{
    const media_codec_descriptor_t *desc = NULL;
    desc = hb_mm_mc_get_descriptor(MEDIA_CODEC_ID_H265);
    return 0;
}
```

#### hb_mm_mc_get_default_context

【Function Declaration】

hb_s32 hb_mm_mc_get_default_context(media_codec_id_t codec_id, hb_bool encoder, media_codec_context_t *context);

【Parameters】

- [IN] media_codec_id_t codec_id: Specifies the codec type.
- [IN] hb_bool encoder: Indicates whether the codec is an encoder (`true`) or decoder (`false`).
- [OUT] media_codec_context_t *context: Receives the default context for the specified codec.

【Return Value】

- 0: Success.
- HB_MEDIA_ERR_INVALID_PARAMS: Invalid parameters.

【Description】

Obtains the default configuration attributes for a specified codec.

【Example】

```c
#include "hb_media_codec.h"
#include "hb_media_error.h"
int main(int argc, char *argv[])
{
    int ret = 0;
    media_codec_context_t context;
    memset(&context, 0x00, sizeof(context));
    ret = hb_mm_mc_get_default_context(MEDIA_CODEC_ID_H265, 1, &context);
    return 0;
}
```

#### hb_mm_mc_initialize

【Function Declaration】

hb_s32 hb_mm_mc_initialize(media_codec_context_t *context)

【Parameters】

- [IN] media_codec_context_t *context: Context configuration for the codec.

【Return Values】

- 0: Success.
- HB_MEDIA_ERR_UNKNOWN: Unknown error.
- HB_MEDIA_ERR_INVALID_PARAMS: Invalid parameters.
- HB_MEDIA_ERR_OPERATION_NOT_ALLOWED: Operation not permitted.
- HB_MEDIA_ERR_INSUFFICIENT_RES: Insufficient internal memory resources.
- HB_MEDIA_ERR_NO_FREE_INSTANCE: No free instance available (max: 32 for Video, 64 for MJPEG/JPEG, 32 for Audio).

【Description】

Initializes an encoder or decoder. Upon success, MediaCodec enters the `MEDIA_CODEC_STATE_INITIALIZED` state.

【Example】

```c
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
                printf("Failed to rewind input file\n");
            } else {
                ret = fread(inputBuffer->vframe_buf.vir_ptr[0], 1,
                inputBuffer->vframe_buf.size, asyncCtx->inFile);
                if (ret <= 0) {
                    printf("Failed to read input file\n");
                }
            }
        }
    }
    if (!ret) {
        printf("%s There is no more input data!\n", TAG);
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
        printf("There is no more output data!\n");
        asyncCtx->lastStream = 1;
    }
}
static void on_encoder_media_codec_message(hb_ptr userdata, hb_s32
       error) {
    AsyncMediaCtx *asyncCtx = (AsyncMediaCtx *)userdata;
    if (error) {
        asyncCtx->lastStream = 1;
        printf("ERROR happened!\n");
    }
}
static void on_vlc_buffer_message(hb_ptr userdata, hb_s32 * vlc_buf)
{
    MediaCodecTestContext *ctx = (MediaCodecTestContext *)userdata;
    printf("%s %s VLC Buffer size = %d; Reset to %d.\n", TAG,
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

#### hb_mm_mc_set_callback

【Function Declaration】

hb_s32 hb_mm_mc_set_callback(media_codec_context_t *context,
const media_codec_callback_t *callback, hb_ptr userdata)

【Parameter Description】

- [IN] media_codec_context_t *context: Context specifying the codec type
- [IN] const media_codec_callback_t *callback: User-defined callback functions
- [IN] hb_ptr userdata: Pointer to user data, which will be passed as an argument when the callback function is invoked

【Return Values】

- 0: Operation succeeded
- HB_MEDIA_ERR_UNKNOWN: Unknown error
- HB_MEDIA_ERR_INVALID_PARAMS: Invalid parameters
- HB_MEDIA_ERR_OPERATION_NOT_ALLOWED: Operation not allowed

【Function Description】

Sets the callback function pointers. After calling this function, MediaCodec enters asynchronous operation mode.  

【Example Code】

Refer to [hb_mm_mc_initialize](#hb_mm_mc_initialize)

#### hb\_mm\_mc\_configure

【Function Declaration】

hb\_s32 hb\_mm\_mc\_configure(media\_codec\_context\_t \*context)

【Parameter Description】

- [IN] media\_codec\_context\_t \*context: Context specifying the codec type

【Return Values】

- 0: Operation succeeded  
- HB\_MEDIA\_ERR\_UNKNOWN: Unknown error  
- HB\_MEDIA\_ERR\_INVALID\_PARAMS: Invalid parameters  
- HB\_MEDIA\_ERR\_OPERATION\_NOT\_ALLOWED: Operation not allowed  
- HB\_MEDIA\_ERR\_INSUFFICIENT\_RES: Insufficient internal memory resources  
- HB\_MEDIA\_ERR\_INVALID\_INSTANCE: Invalid instance  

【Function Description】

Configures the encoder or decoder based on the input information. Upon successful invocation, MediaCodec enters the MEDIA\_CODEC\_STATE\_CONFIGURED state.

【Example Code】

Refer to [hb_mm_mc_initialize](#hb_mm_mc_initialize)

#### hb\_mm\_mc\_start

【Function Declaration】

hb\_s32 hb\_mm\_mc\_start(media\_codec\_context\_t \*context, const  
mc\_av\_codec\_startup\_params\_t \*info)

【Parameter Description】

- [IN] media\_codec\_context\_t \*context: Context specifying the codec type  
- [IN] mc\_av\_codec\_startup\_params\_t *info: Startup parameters for audio/video encoding or decoding  

【Return Values】

- 0: Operation succeeded  
- HB\_MEDIA\_ERR\_UNKNOWN: Unknown error  
- HB\_MEDIA\_ERR\_INVALID\_PARAMS: Invalid parameters  
- HB\_MEDIA\_ERR\_OPERATION\_NOT\_ALLOWED: Operation not allowed  
- HB\_MEDIA\_ERR\_INSUFFICIENT\_RES: Insufficient internal memory resources  
- HB\_MEDIA\_ERR\_INVALID\_INSTANCE: Invalid instance  

【Function Description】

Starts the encoding/decoding process. MediaCodec will create the codec instance, set up sequences or parse the data stream, register Framebuffers, encode header information, etc. Upon successful invocation, MediaCodec enters the MEDIA\_CODEC\_STATE\_STARTED state.

【Example Code】

Refer to [hb_mm_mc_initialize](#hb_mm_mc_initialize)

#### hb\_mm\_mc\_stop

【Function Declaration】

hb\_s32 hb\_mm\_mc\_stop(media\_codec\_context\_t \*context)

【Parameter Description】

- [IN] media\_codec\_context\_t \*context: Context specifying the codec type

【Return Values】

- 0: Operation succeeded  
- HB\_MEDIA\_ERR\_UNKNOWN: Unknown error  
- HB\_MEDIA\_ERR\_OPERATION\_NOT\_ALLOWED: Operation not allowed  
- HB\_MEDIA\_ERR\_INVALID\_INSTANCE: Invalid instance  

【Function Description】

Stops the encoding/decoding process, terminates all child threads, and releases associated resources. Upon successful invocation, MediaCodec returns to the MEDIA\_CODEC\_STATE\_INITIALIZED state.

【Example Code】

Refer to [hb_mm_mc_initialize](#hb_mm_mc_initialize)

#### hb\_mm\_mc\_pause

【Function Declaration】

hb\_s32 hb\_mm\_mc\_pause(media\_codec\_context\_t \*context)

【Parameter Description】

- [IN] media\_codec\_context\_t \*context: Context specifying the codec type

【Return Values】

- 0: Operation succeeded  
- HB\_MEDIA\_ERR\_UNKNOWN: Unknown error  
- HB\_MEDIA\_ERR\_OPERATION\_NOT\_ALLOWED: Operation not allowed  
- HB\_MEDIA\_ERR\_INVALID\_INSTANCE: Invalid instance  

【Function Description】

Pauses the encoding/decoding process and suspends all child threads. Upon successful invocation, MediaCodec enters the MEDIA\_CODEC\_STATE\_PAUSED state.

【Example Code】

Refer to [hb_mm_mc_queue_input_buffer](#hb_mm_mc_queue_input_buffer)

#### hb\_mm\_mc\_flush

【Function Declaration】

hb\_s32 hb\_mm\_mc\_flush(media\_codec\_context\_t \*context)

【Parameter Description】

- \[IN\] media\_codec\_context\_t \*context: Context specifying the codec type

【Return Values】

- 0: Operation succeeded  
- HB\_MEDIA\_ERR\_UNKNOWN: Unknown error  
- HB\_MEDIA\_ERR\_OPERATION\_NOT\_ALLOWED: Operation not allowed  
- HB\_MEDIA\_ERR\_INVALID\_INSTANCE: Invalid instance  

【Function Description】

Flushes the input and output buffer queues, forcing the encoder/decoder to flush any unprocessed input or output buffers. After this function is called, MediaCodec enters the MEDIA\_CODEC\_STATE\_FLUSHING state. Upon successful completion, MediaCodec returns to the MEDIA\_CODEC\_STATE\_STARTED state.

【Example Code】

Refer to [hb_mm_mc_queue_input_buffer](#hb_mm_mc_queue_input_buffer)

#### hb\_mm\_mc\_release

【Function Declaration】

hb\_s32 hb\_mm\_mc\_release(media\_codec\_context\_t \*context)

【Parameter Description】

- [IN] media\_codec\_context\_t \*context: Context specifying the codec type

【Return Values】

- 0: Operation succeeded  
- HB\_MEDIA\_ERR\_UNKNOWN: Unknown error  
- HB\_MEDIA\_ERR\_OPERATION\_NOT\_ALLOWED: Operation not allowed  
- HB\_MEDIA\_ERR\_INVALID\_INSTANCE: Invalid instance  

【Function Description】

Releases all internal resources held by MediaCodec. The user must call hb\_mm\_mc\_stop to stop encoding/decoding before invoking this function. Upon successful completion, MediaCodec enters the MEDIA\_CODEC\_STATE\_UNINITIALIZED state.

【Example Code】

Refer to [hb_mm_mc_initialize](#hb_mm_mc_initialize)

#### hb\_mm\_mc\_get\_state

【Function Declaration】

hb\_s32 hb\_mm\_mc\_get\_state(media\_codec\_context\_t \*context,  
media\_codec\_state\_t \*state)

【Parameter Description】

- [IN] media\_codec\_context\_t \*context: Context specifying the codec type  
- [OUT] media\_codec\_state\_t \*state: Current state of MediaCodec  

【Return Values】

- 0: Operation succeeded  
- HB\_MEDIA\_ERR\_UNKNOWN: Unknown error  
- HB\_MEDIA\_ERR\_OPERATION\_NOT\_ALLOWED: Operation not allowed  
- HB\_MEDIA\_ERR\_INVALID\_INSTANCE: Invalid instance  
- HB\_MEDIA\_ERR\_INVALID\_PARAMS: Invalid parameters  

【Function Description】

Retrieves the current state of MediaCodec.

【Example Code】

Refer to [hb_mm_mc_initialize](#hb_mm_mc_initialize)

#### hb\_mm\_mc\_get\_status

【Function Declaration】

hb\_s32 hb\_mm\_mc\_get\_status(media\_codec\_context\_t \*context,  
mc\_inter\_status\_t \*status)

【Parameter Description】

- [IN] media\_codec\_context\_t \*context: Context specifying the codec type  
- [OUT] mc\_inter\_status\_t \*status: Current internal status of MediaCodec  
 
【Return Value】

- 0: Operation succeeded  
- HB\_MEDIA\_ERR\_UNKNOWN: Unknown error  
- HB\_MEDIA\_ERR\_OPERATION\_NOT\_ALLOWED: Operation not allowed  
- HB\_MEDIA\_ERR\_INVALID\_INSTANCE: Invalid instance  
- HB\_MEDIA\_ERR\_INVALID\_PARAMS: Invalid parameters  

【Function Description】

Obtain the current internal state information of MediaCodec.

【Example Code】

Refer to [hb_mm_mc_get_fd](#hb_mm_mc_get_fd)

#### hb\_mm\_mc\_queue\_input\_buffer

【Function Declaration】

hb\_s32 hb\_mm\_mc\_queue\_input\_buffer(media\_codec\_context\_t  
\*context, media\_codec\_buffer\_t \*buffer, hb\_s32 timeout)

【Parameter Description】

- \[IN\] media\_codec\_context\_t \*context: Context specifying the codec type  
- \[IN\] media\_codec\_buffer\_t \*buffer: Input buffer information  
- \[IN\] hb\_s32 timeout: Timeout duration  

【Return Value】

- 0: Operation succeeded  
- HB\_MEDIA\_ERR\_UNKNOWN: Unknown error  
- HB\_MEDIA\_ERR\_OPERATION\_NOT\_ALLOWED: Operation not allowed  
- HB\_MEDIA\_ERR\_INVALID\_INSTANCE: Invalid instance  
- HB\_MEDIA\_ERR\_INVALID\_PARAMS: Invalid parameters  
- HB\_MEDIA\_ERR\_INVALID\_BUFFER: Invalid buffer  
- HB\_MEDIA\_ERR\_WAIT\_TIMEOUT: Wait timeout  

【Function Description】

Submit a buffer requiring processing into MediaCodec.

【Example Code】

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
        } else {
            if (ret != (int32_t)HB_MEDIA_ERR_WAIT_TIMEOUT) {
                printf("Dequeue input buffer fail.\n");
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

【Function Declaration】

hb\_s32 hb\_mm\_mc\_dequeue\_input\_buffer(media\_codec\_context\_t
\*context, media\_codec\_buffer\_t \*buffer, hb\_s32 timeout)

【Parameter Description】

- \[IN\] media\_codec\_context\_t \*context: Context specifying the codec type  
- \[IN\] hb\_s32 timeout: Timeout duration  
- \[OUT\] media\_codec\_buffer\_t \*buffer: Input buffer information  

【Return Values】

- 0: Operation succeeded  
- HB\_MEDIA\_ERR\_UNKNOWN: Unknown error  
- HB\_MEDIA\_ERR\_OPERATION\_NOT\_ALLOWED: Operation not allowed  
- HB\_MEDIA\_ERR\_INVALID\_INSTANCE: Invalid instance  
- HB\_MEDIA\_ERR\_INVALID\_PARAMS: Invalid parameters  
- HB\_MEDIA\_ERR\_INVALID\_BUFFER: Invalid buffer  
- HB\_MEDIA\_ERR\_WAIT\_TIMEOUT: Wait timeout  

【Function Description】

Obtain an input buffer.

【Example Code】

Refer to [hb_mm_mc_queue_input_buffer](#hb_mm_mc_queue_input_buffer)

#### hb\_mm\_mc\_queue\_output\_buffer

【Function Declaration】

hb\_s32 hb\_mm\_mc\_queue\_output\_buffer(media\_codec\_context\_t
\*context, media\_codec\_buffer\_t \*buffer, hb\_s32 timeout)

【Parameter Description】

- \[IN\] media\_codec\_context\_t \*context: Context specifying the codec type  
- \[IN\] media\_codec\_buffer\_t \*buffer: Output buffer information  
- \[IN\] hb\_s32 timeout: Timeout duration  

【Return Values】

- 0: Operation succeeded  
- HB\_MEDIA\_ERR\_UNKNOWN: Unknown error  
- HB\_MEDIA\_ERR\_OPERATION\_NOT\_ALLOWED: Operation not allowed  
- HB\_MEDIA\_ERR\_INVALID\_INSTANCE: Invalid instance  
- HB\_MEDIA\_ERR\_INVALID\_PARAMS: Invalid parameters  
- HB\_MEDIA\_ERR\_INVALID\_BUFFER: Invalid buffer  
- HB\_MEDIA\_ERR\_WAIT\_TIMEOUT: Wait timeout  

【Function Description】

Return a processed output buffer back to MediaCodec.

【Example Code】

Refer to [hb_mm_mc_queue_input_buffer](#hb_mm_mc_queue_input_buffer)

#### hb\_mm\_mc\_dequeue\_output\_buffer

【Function Declaration】

hb\_s32 hb\_mm\_mc\_dequeue\_output\_buffer(media\_codec\_context\_t
\*context, media\_codec\_buffer\_t \*buffer,
media\_codec\_output\_buffer\_info\_t \*info, hb\_s32 timeout)

【Parameter Description】

- \[IN\] media\_codec\_context\_t \*context: Context specifying the codec type  
- \[IN\] hb\_s32 timeout: Timeout duration  
- \[OUT\] media\_codec\_buffer\_t \*buffer: Output buffer information  
- \[IN\] media\_codec\_output\_buffer\_info\_t
  \*info: Information of the output bitstream  

【Return Values】

- 0: Operation succeeded  
- HB\_MEDIA\_ERR\_UNKNOWN: Unknown error  
- HB\_MEDIA\_ERR\_OPERATION\_NOT\_ALLOWED: Operation not allowed  
- HB\_MEDIA\_ERR\_INVALID\_INSTANCE: Invalid instance  
- HB\_MEDIA\_ERR\_INVALID\_PARAMS: Invalid parameters  
- HB\_MEDIA\_ERR\_INVALID\_BUFFER: Invalid buffer  
- HB\_MEDIA\_ERR\_WAIT\_TIMEOUT: Wait timeout  

【Function Description】

Obtain an output buffer.

【Example Code】

Refer to [hb_mm_mc_queue_input_buffer](#hb_mm_mc_queue_input_buffer)

#### hb\_mm\_mc\_get\_longterm\_ref\_mode

【Function Declaration】

hb\_s32 hb\_mm\_mc\_get\_longterm\_ref\_mode(media\_codec\_context\_t
\*context, mc\_video\_longterm\_ref\_mode\_t \*params)

【Parameter Description】

- \[IN\] media\_codec\_context\_t \*context: Context specifying the codec type  
- \[OUT\] mc\_video\_longterm\_ref\_mode\_t
  \*params: Long-term reference frame mode parameters  

【Return Values】

- 0: Operation succeeded  
- HB\_MEDIA\_ERR\_UNKNOWN: Unknown error  
- HB\_MEDIA\_ERR\_INVALID\_INSTANCE: Invalid instance  
- HB\_MEDIA\_ERR\_INVALID\_PARAMS: Invalid parameters  

【Function Description】

Retrieve parameters of the long-term reference frame mode, applicable to H.264/H.265.

【Example Code】

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
    int32_t duration; // seconds
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
    }}

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
            }
        }
    } while (/* condition */);
}if (outputBuffer.vstream_buf.stream_end) {
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

【Function Declaration】

hb\_s32 hb\_mm\_mc\_set\_longterm\_ref\_mode(media\_codec\_context\_t
\*context, const mc\_video\_longterm\_ref\_mode\_t \*params)

【Parameter Description】

- \[IN\] media\_codec\_context\_t \*context: Context specifying the codec type
- \[IN\] const mc\_video\_longterm\_ref\_mode\_t
  \*params: Long-term reference frame mode parameters

【Return Values】

- 0: Operation succeeded
- HB\_MEDIA\_ERR\_UNKNOWN: Unknown error
- HB\_MEDIA\_ERR\_OPERATION\_NOT\_ALLOWED: Operation not allowed
- HB\_MEDIA\_ERR\_INVALID\_INSTANCE: Invalid instance
- HB\_MEDIA\_ERR\_INVALID\_PARAMS: Invalid parameters

【Function Description】

Sets long-term reference frame mode parameters. These parameters are dynamic and applicable to H.264/H.265.

【Example Code】

Refer to [hb_mm_mc_get_longterm_ref_mode](#hb_mm_mc_get_longterm_ref_mode)

#### hb\_mm\_mc\_get\_intra\_refresh\_config

【Function Declaration】

hb\_s32 hb\_mm\_mc\_get\_intra\_refresh\_config(media\_codec\_context\_t
\*context, mc\_video\_intra\_refresh\_params\_t \*params)

【Parameter Description】

- \[IN\] media\_codec\_context\_t \*context: Context specifying the codec type
- \[OUT\] mc\_video\_intra\_refresh\_params\_t \*params: Intra refresh parameters

【Return Values】

- 0: Operation succeeded
- HB\_MEDIA\_ERR\_UNKNOWN: Unknown error
- HB\_MEDIA\_ERR\_INVALID\_INSTANCE: Invalid instance
- HB\_MEDIA\_ERR\_INVALID\_PARAMS: Invalid parameters

【Function Description】

Retrieves intra refresh parameters, applicable to H.264/H.265.

【Example Code】

Refer to [hb_mm_mc_get_longterm_ref_mode](#hb_mm_mc_get_longterm_ref_mode)

#### hb\_mm\_mc\_set\_intra\_refresh\_config

【Function Declaration】

hb\_s32 hb\_mm\_mc\_set\_intra\_refresh\_config(media\_codec\_context\_t
\*context, const mc\_video\_intra\_refresh\_params\_t \*params)

【Parameter Description】

- \[IN\] media\_codec\_context\_t \*context: Context specifying the codec type
- \[IN\] const mc\_video\_intra\_refresh\_params\_t
  \*params: Intra refresh parameters

【Return Values】

- 0: Operation succeeded
- HB\_MEDIA\_ERR\_UNKNOWN: Unknown error
- HB\_MEDIA\_ERR\_OPERATION\_NOT\_ALLOWED: Operation not allowed
- HB\_MEDIA\_ERR\_INVALID\_INSTANCE: Invalid instance
- HB\_MEDIA\_ERR\_INVALID\_PARAMS: Invalid parameters

【Function Description】

Sets intra refresh mode parameters. These parameters are static and applicable to H.264/H.265.

【Example Code】

Refer to [hb_mm_mc_get_longterm_ref_mode](#hb_mm_mc_get_longterm_ref_mode)

#### hb\_mm\_mc\_get\_rate\_control\_config

【Function Declaration】

hb\_s32 hb\_mm\_mc\_get\_rate\_control\_config(media\_codec\_context\_t
\*context, mc\_rate\_control\_params\_t \*params)

【Parameter Description】

- \[IN\] media\_codec\_context\_t \*context: Context specifying the codec type
- \[OUT\] mc\_rate\_control\_params\_t \*params: Rate control parameters

【Return Values】

- 0: Operation succeeded
- HB\_MEDIA\_ERR\_UNKNOWN: Unknown error
- HB\_MEDIA\_ERR\_INVALID\_INSTANCE: Invalid instance
- HB\_MEDIA\_ERR\_INVALID\_PARAMS: Invalid parameters

【Function Description】

Retrieves rate control parameters. These parameters are dynamic and applicable to H.264/H.265/MJPEG.

【Example Code】

Refer to [hb_mm_mc_get_longterm_ref_mode](#hb_mm_mc_get_longterm_ref_mode)

#### hb\_mm\_mc\_set\_rate\_control\_config

【Function Declaration】

hb\_s32 hb\_mm\_mc\_set\_rate\_control\_config(media\_codec\_context\_t
\*context, const mc\_rate\_control\_params\_t \*params)【Parameter Description】

- \[IN\] media\_codec\_context\_t \*context: Context specifying the codec type  
- \[IN\] const mc\_rate\_control\_params\_t \*params: Rate control parameters  

【Return Values】

- 0: Operation succeeded  
- HB\_MEDIA\_ERR\_UNKNOWN: Unknown error  
- HB\_MEDIA\_ERR\_OPERATION\_NOT\_ALLOWED: Operation not allowed  
- HB\_MEDIA\_ERR\_INVALID\_INSTANCE: Invalid instance  
- HB\_MEDIA\_ERR\_INVALID\_PARAMS: Invalid parameters  

【Function Description】

Sets rate control parameters. These parameters are dynamic and applicable to H264/H265/MJPEG.

【Example Code】

Refer to [hb_mm_mc_get_longterm_ref_mode](#hb_mm_mc_get_longterm_ref_mode)

#### hb\_mm\_mc\_get\_deblk\_filter\_config

【Function Declaration】

hb\_s32 hb\_mm\_mc\_get\_deblk\_filter\_config(media\_codec\_context\_t  
\*context, mc\_video\_deblk\_filter\_params\_t \*params)

【Parameter Description】

- \[IN\] media\_codec\_context\_t \*context: Context specifying the codec type  
- \[OUT\] mc\_video\_deblk\_filter\_params\_t \*params: Deblocking filter parameters  

【Return Values】

- 0: Operation succeeded  
- HB\_MEDIA\_ERR\_UNKNOWN: Unknown error  
- HB\_MEDIA\_ERR\_INVALID\_INSTANCE: Invalid instance  
- HB\_MEDIA\_ERR\_INVALID\_PARAMS: Invalid parameters  

【Function Description】

Retrieves deblocking filter parameters, applicable to H264/H265.

【Example Code】

Refer to [hb_mm_mc_get_longterm_ref_mode](#hb_mm_mc_get_longterm_ref_mode)

#### hb\_mm\_mc\_set\_deblk\_filter\_config

【Function Declaration】

hb\_s32 hb\_mm\_mc\_set\_deblk\_filter\_config(media\_codec\_context\_t  
\*context, const mc\_video\_deblk\_filter\_params\_t \*params)

【Parameter Description】

- \[IN\] media\_codec\_context\_t \*context: Context specifying the codec type  
- \[IN\] const mc\_video\_deblk\_filter\_params\_t 
  \*params: Deblocking filter parameters  

【Return Values】

- 0: Operation succeeded  
- HB\_MEDIA\_ERR\_UNKNOWN: Unknown error  
- HB\_MEDIA\_ERR\_OPERATION\_NOT\_ALLOWED: Operation not allowed  
- HB\_MEDIA\_ERR\_INVALID\_INSTANCE: Invalid instance  
- HB\_MEDIA\_ERR\_INVALID\_PARAMS: Invalid parameters  

【Function Description】

Sets deblocking filter parameters. These parameters are dynamic and applicable to H264/H265.

【Example Code】

Refer to [hb_mm_mc_get_longterm_ref_mode](#hb_mm_mc_get_longterm_ref_mode)

#### hb\_mm\_mc\_get\_sao\_config

【Function Declaration】

hb\_s32 hb\_mm\_mc\_get\_sao\_config(media\_codec\_context\_t \*context,  
mc\_h265\_sao\_params\_t \*params)

【Parameter Description】

- \[IN\] media\_codec\_context\_t \*context: Context specifying the codec type  
- \[OUT\] mc\_h265\_sao\_params\_t \*params: SAO parameters  

【Return Values】

- 0: Operation succeeded  
- HB\_MEDIA\_ERR\_UNKNOWN: Unknown error  
- HB\_MEDIA\_ERR\_INVALID\_INSTANCE: Invalid instance  
- HB\_MEDIA\_ERR\_INVALID\_PARAMS: Invalid parameters  

【Function Description】

Retrieves SAO parameters, applicable to H265.

【Example Code】

Refer to [hb_mm_mc_get_longterm_ref_mode](#hb_mm_mc_get_longterm_ref_mode)

#### hb\_mm\_mc\_set\_sao\_config

【Function Declaration】

hb\_s32 hb\_mm\_mc\_set\_sao\_config(media\_codec\_context\_t \*context,  
const mc\_h265\_sao\_params\_t \*params)

【Parameter Description】

- \[IN\] media\_codec\_context\_t \*context: Context specifying the codec type  
- \[IN\] const mc\_h265\_sao\_params\_t \*params: SAO parameters  

【Return Values】

- 0: Operation succeeded  
- HB\_MEDIA\_ERR\_UNKNOWN: Unknown error  
- HB\_MEDIA\_ERR\_OPERATION\_NOT\_ALLOWED: Operation not allowed  
- HB\_MEDIA\_ERR\_INVALID\_INSTANCE: Invalid instance  
- HB\_MEDIA\_ERR\_INVALID\_PARAMS: Invalid parameters  

【Function Description】

Sets SAO parameters. These parameters are static and applicable to H265.

【Example Code】

Refer to [hb_mm_mc_get_longterm_ref_mode](#hb_mm_mc_get_longterm_ref_mode)

#### hb\_mm\_mc\_get\_entropy\_config

【Function Declaration】

hb\_s32 hb\_mm\_mc\_get\_entropy\_config(media\_codec\_context\_t \*context, mc\_h264\_entropy\_params\_t \*params);

【Parameter Description】

- \[IN\] media\_codec\_context\_t \*context: Context specifying the codec type  
- \[IN\] mc\_h264\_entropy\_params\_t \*params: Entropy parameters  

【Return Values】

- 0: Operation succeeded  
- HB\_MEDIA\_ERR\_UNKNOWN: Unknown error  
- HB\_MEDIA\_ERR\_OPERATION\_NOT\_ALLOWED: Operation not allowed  
- HB\_MEDIA\_ERR\_INVALID\_INSTANCE: Invalid instance  
- HB\_MEDIA\_ERR\_INVALID\_PARAMS: Invalid parameters  

【Function Description】

Retrieves entropy parameters, applicable to H264.

【Example Code】

Refer to [hb_mm_mc_get_longterm_ref_mode](#hb_mm_mc_get_longterm_ref_mode)

#### hb\_mm\_mc\_set\_entropy\_config

【Function Declaration】

extern hb\_s32 hb\_mm\_mc\_set\_entropy\_config(media\_codec\_context\_t \*context, const mc\_h264\_entropy\_params\_t \*params);

【Parameter Description】

- \[IN\] media\_codec\_context\_t \*context: Context specifying the codec type  
- \[IN\] const mc\_h264\_entropy\_params\_t \*params: Entropy parameters  

【Return Values】

- 0: Operation succeeded  
- HB\_MEDIA\_ERR\_UNKNOWN: Unknown error  
- HB\_MEDIA\_ERR\_OPERATION\_NOT\_ALLOWED: Operation not allowed  
- HB\_MEDIA\_ERR\_INVALID\_INSTANCE: Invalid instance  
- HB\_MEDIA\_ERR\_INVALID\_PARAMS: Invalid parameters  

【Function Description】

Sets entropy parameters, applicable to H264.

【Example Code】

Refer to [hb_mm_mc_get_longterm_ref_mode](#hb_mm_mc_get_longterm_ref_mode)

#### hb\_mm\_mc\_get\_vui\_timing\_config

【Function Declaration】

hb\_s32 hb\_mm\_mc\_get\_vui\_timing\_config(media\_codec\_context\_t  
\*context, mc\_video\_vui\_timing\_params\_t \*params)

【Parameter Description】

- \[IN\] media\_codec\_context\_t \*context: Context specifying the codec type  
- \[OUT\] mc\_video\_vui\_timing\_params\_t \*params: VUI Timing parameters  


【Return Value】

- 0: Operation succeeded  
- HB\_MEDIA\_ERR\_UNKNOWN: Unknown error  
- HB\_MEDIA\_ERR\_INVALID\_INSTANCE: Invalid instance  
- HB\_MEDIA\_ERR\_INVALID\_PARAMS: Invalid parameters  

【Function Description】

Gets VUI Timing parameters, applicable to H264/H265.

【Example Code】

Refer to [hb_mm_mc_get_longterm_ref_mode](#hb_mm_mc_get_longterm_ref_mode)

#### hb\_mm\_mc\_set\_vui\_timing\_config

【Function Declaration】

hb\_s32 hb\_mm\_mc\_set\_vui\_timing\_config(media\_codec\_context\_t \*context, const mc\_video\_vui\_timing\_params\_t \*params)

【Parameter Description】

- \[IN\] media\_codec\_context\_t \*context: Context specifying the codec type  
- \[IN\] const mc\_video\_vui\_timing\_params\_t \*params: VUI Timing parameters  

【Return Value】

- 0: Operation succeeded  
- HB\_MEDIA\_ERR\_UNKNOWN: Unknown error  
- HB\_MEDIA\_ERR\_OPERATION\_NOT\_ALLOWED: Operation not allowed  
- HB\_MEDIA\_ERR\_INVALID\_INSTANCE: Invalid instance  
- HB\_MEDIA\_ERR\_INVALID\_PARAMS: Invalid parameters  

【Function Description】

Sets VUI Timing parameters. These parameters are static and applicable to H264/H265.

【Example Code】

Refer to [hb_mm_mc_get_longterm_ref_mode](#hb_mm_mc_get_longterm_ref_mode)

#### hb\_mm\_mc\_get\_slice\_config

【Function Declaration】

hb\_s32 hb\_mm\_mc\_get\_slice\_config(media\_codec\_context\_t \*context, mc\_video\_slice\_params\_t \*params)

【Parameter Description】

- \[IN\] media\_codec\_context\_t \*context: Context specifying the codec type  
- \[OUT\] mc\_video\_slice\_params\_t \*params: Slice encoding parameters  

【Return Value】

- 0: Operation succeeded  
- HB\_MEDIA\_ERR\_UNKNOWN: Unknown error  
- HB\_MEDIA\_ERR\_INVALID\_INSTANCE: Invalid instance  
- HB\_MEDIA\_ERR\_INVALID\_PARAMS: Invalid parameters  

【Function Description】

Gets slice encoding parameters, applicable to H264/H265.

【Example Code】

Refer to [hb_mm_mc_get_longterm_ref_mode](#hb_mm_mc_get_longterm_ref_mode)

#### hb\_mm\_mc\_set\_slice\_config

【Function Declaration】

hb\_s32 hb\_mm\_mc\_set\_slice\_config(media\_codec\_context\_t \*context, const mc\_video\_slice\_params\_t \*params)

【Parameter Description】

- \[IN\] media\_codec\_context\_t \*context: Context specifying the codec type  
- \[IN\] const mc\_video\_slice\_params\_t \*params: Slice encoding parameters  

【Return Value】

- 0: Operation succeeded  
- HB\_MEDIA\_ERR\_UNKNOWN: Unknown error  
- HB\_MEDIA\_ERR\_OPERATION\_NOT\_ALLOWED: Operation not allowed  
- HB\_MEDIA\_ERR\_INVALID\_INSTANCE: Invalid instance  
- HB\_MEDIA\_ERR\_INVALID\_PARAMS: Invalid parameters  

【Function Description】

Sets slice encoding parameters. These parameters are dynamic and applicable to H264/H265. The number of slices per frame must not exceed 1500.

【Example Code】

Refer to [hb_mm_mc_get_longterm_ref_mode](#hb_mm_mc_get_longterm_ref_mode)

#### hb\_mm\_mc\_insert\_user\_data

【Function Declaration】

hb\_s32 hb\_mm\_mc\_insert\_user\_data(media\_codec\_context\_t \*context, hb\_u8 \*data, hb\_u32 length)

【Parameter Description】

- \[IN\] media\_codec\_context\_t \*context: Context specifying the codec type  
- \[IN\] hb\_u8 \*data: User data  
- \[IN\] hb\_u32 length: Length of user data  

【Return Value】

- 0: Operation succeeded  
- HB\_MEDIA\_ERR\_UNKNOWN: Unknown error  
- HB\_MEDIA\_ERR\_INVALID\_INSTANCE: Invalid instance  
- HB\_MEDIA\_ERR\_INVALID\_PARAMS: Invalid parameters  

【Function Description】

Inserts user data into the encoded bitstream. This parameter is dynamic and applicable to H264/H265/MJPG/JPG.

【Example Code】

Refer to [hb_mm_mc_get_longterm_ref_mode](#hb_mm_mc_get_longterm_ref_mode)

#### hb\_mm\_mc\_request\_idr\_frame

【Function Declaration】

hb\_s32 hb\_mm\_mc\_request\_idr\_frame(media\_codec\_context\_t \*context)

【Parameter Description】

- \[IN\] media\_codec\_context\_t \*context: Context specifying the codec type  

【Return Value】

- 0: Operation succeeded  
- HB\_MEDIA\_ERR\_UNKNOWN: Unknown error  
- HB\_MEDIA\_ERR\_OPERATION\_NOT\_ALLOWED: Operation not allowed  
- HB\_MEDIA\_ERR\_INVALID\_INSTANCE: Invalid instance  

【Function Description】

Requests an IDR frame. This interface supports dynamic configuration and is applicable to H264/H265.

【Example Code】

Refer to [hb_mm_mc_get_longterm_ref_mode](#hb_mm_mc_get_longterm_ref_mode)

#### hb\_mm\_mc\_skip\_pic

【Function Declaration】

hb\_s32 hb\_mm\_mc\_skip\_pic(media\_codec\_context\_t \*context, hb\_s32 src\_idx)

【Parameter Description】

- \[IN\] media\_codec\_context\_t \*context: Context specifying the codec type  
- \[IN\] hb\_s32 src\_idx: Source buffer index  

【Return Value】

- 0: Operation succeeded  
- HB\_MEDIA\_ERR\_UNKNOWN: Unknown error  
- HB\_MEDIA\_ERR\_OPERATION\_NOT\_ALLOWED: Operation not allowed  
- HB\_MEDIA\_ERR\_INVALID\_INSTANCE: Invalid instance  
- HB\_MEDIA\_ERR\_INVALID\_PARAMS: Invalid parameters  

【Function Description】

Enables skip-mode encoding for the specified picture. This interface supports dynamic configuration and is applicable to H264/H265.

【Example Code】

Refer to [hb_mm_mc_get_longterm_ref_mode](#hb_mm_mc_get_longterm_ref_mode)

#### hb\_mm\_mc\_get\_smart\_bg\_enc\_config

【Function Declaration】

extern hb\_s32  
hb\_mm\_mc\_get\_smart\_bg\_enc\_config(media\_codec\_context\_t  
\*context, mc\_video\_smart\_bg\_enc\_params\_t \*params);

【Parameter Description】

- \[IN\] media\_codec\_context\_t \*context: Context specifying the codec type  
- \[OUT\] mc\_video\_smart\_bg\_enc\_params\_t  
  \*params: Smart background encoding mode parameters  

【Return Value】

- 0: Operation succeeded  
- HB\_MEDIA\_ERR\_UNKNOWN: Unknown error
- HB\_MEDIA\_ERR\_INVALID\_INSTANCE: Invalid instance  
- HB\_MEDIA\_ERR\_INVALID\_PARAMS: Invalid parameters  

【Function Description】

Get smart background encoding parameters, applicable to H264/H265.

【Example Code】

Refer to [hb_mm_mc_get_longterm_ref_mode](#hb_mm_mc_get_longterm_ref_mode)

#### hb\_mm\_mc\_set\_smart\_bg\_enc\_config

【Function Declaration】

hb\_s32 hb\_mm\_mc\_set\_smart\_bg\_enc\_config(media\_codec\_context\_t  
\*context, const mc\_video\_smart\_bg\_enc\_params\_t \*params);

【Parameter Description】

- \[IN\] media\_codec\_context\_t \*context: Context specifying the codec type  
- \[IN\] const mc\_video\_smart\_bg\_enc\_params\_t  
  \*params: Smart background encoding mode parameters  

【Return Values】

- 0: Operation succeeded  
- HB\_MEDIA\_ERR\_UNKNOWN: Unknown error  
- HB\_MEDIA\_ERR\_OPERATION\_NOT\_ALLOWED: Operation not allowed  
- HB\_MEDIA\_ERR\_INVALID\_INSTANCE: Invalid instance  
- HB\_MEDIA\_ERR\_INVALID\_PARAMS: Invalid parameters  

【Function Description】

Set smart background encoding parameters. These parameters are dynamic and applicable to H264/H265.

【Example Code】

Refer to [hb_mm_mc_get_longterm_ref_mode](#hb_mm_mc_get_longterm_ref_mode)

#### hb\_mm\_mc\_get\_pred\_unit\_config

【Function Declaration】

hb\_s32 hb\_mm\_mc\_get\_pred\_unit\_config(media\_codec\_context\_t  
\*context, mc\_video\_pred\_unit\_params\_t \*params)

【Parameter Description】

- \[IN\] media\_codec\_context\_t \*context: Context specifying the codec type  
- \[OUT\] mc\_video\_pred\_unit\_params\_t \*params: Prediction unit parameters  

【Return Values】

- 0: Operation succeeded  
- HB\_MEDIA\_ERR\_UNKNOWN: Unknown error  
- HB\_MEDIA\_ERR\_INVALID\_INSTANCE: Invalid instance  
- HB\_MEDIA\_ERR\_INVALID\_PARAMS: Invalid parameters  

【Function Description】

Get prediction unit parameters, applicable to H264/H265.

【Example Code】

Refer to [hb_mm_mc_get_longterm_ref_mode](#hb_mm_mc_get_longterm_ref_mode)

#### hb\_mm\_mc\_set\_pred\_unit\_config

【Function Declaration】

hb\_s32 hb\_mm\_mc\_set\_pred\_unit\_config(media\_codec\_context\_t  
\*context, const mc\_video\_pred\_unit\_params\_t \*params)

【Parameter Description】

- \[IN\] media\_codec\_context\_t \*context: Context specifying the codec type  
- \[IN\] const mc\_video\_pred\_unit\_params\_t \*params: Prediction unit parameters  

【Return Values】

- 0: Operation succeeded  
- HB\_MEDIA\_ERR\_UNKNOWN: Unknown error  
- HB\_MEDIA\_ERR\_OPERATION\_NOT\_ALLOWED: Operation not allowed  
- HB\_MEDIA\_ERR\_INVALID\_INSTANCE: Invalid instance  
- HB\_MEDIA\_ERR\_INVALID\_PARAMS: Invalid parameters  

【Function Description】

Set prediction unit parameters. These parameters are dynamic and applicable to H264/H265.

【Example Code】

Refer to [hb_mm_mc_get_longterm_ref_mode](#hb_mm_mc_get_longterm_ref_mode)

#### hb\_mm\_mc\_get\_transform\_config

【Function Declaration】

hb\_s32 hb\_mm\_mc\_get\_transform\_config(media\_codec\_context\_t  
\*context, mc\_video\_transform\_params\_t \*params)

【Parameter Description】

- \[IN\] media\_codec\_context\_t \*context: Context specifying the codec type  
- \[OUT\] mc\_video\_transform\_params\_t \*params: Transform parameters  

【Return Values】

- 0: Operation succeeded  
- HB\_MEDIA\_ERR\_UNKNOWN: Unknown error  
- HB\_MEDIA\_ERR\_INVALID\_INSTANCE: Invalid instance  
- HB\_MEDIA\_ERR\_INVALID\_PARAMS: Invalid parameters  

【Function Description】

Get Transform parameters, applicable to H264/H265.

【Example Code】

Refer to [hb_mm_mc_get_longterm_ref_mode](#hb_mm_mc_get_longterm_ref_mode)

#### hb\_mm\_mc\_set\_transform\_config

【Function Declaration】

hb\_s32 hb\_mm\_mc\_set\_transform\_config(media\_codec\_context\_t  
\*context, const mc\_video\_transform\_params\_t \*params)

【Parameter Description】

- \[IN\] media\_codec\_context\_t \*context: Context specifying the codec type  
- \[IN\] const mc\_video\_transform\_params\_t \*params:  
  Transform parameters  

【Return Values】

- 0: Operation succeeded  
- HB\_MEDIA\_ERR\_UNKNOWN: Unknown error  
- HB\_MEDIA\_ERR\_OPERATION\_NOT\_ALLOWED: Operation not allowed  
- HB\_MEDIA\_ERR\_INVALID\_INSTANCE: Invalid instance  
- HB\_MEDIA\_ERR\_INVALID\_PARAMS: Invalid parameters  

【Function Description】

Set Transform parameters. These parameters are dynamic and applicable to H264/H265.

【Example Code】

Refer to [hb_mm_mc_get_longterm_ref_mode](#hb_mm_mc_get_longterm_ref_mode)

#### hb\_mm\_mc\_get\_roi\_config

【Function Declaration】

hb\_s32 hb\_mm\_mc\_get\_roi\_config(media\_codec\_context\_t \*context,  
mc\_video\_roi\_params\_t \*params)

【Parameter Description】

- \[IN\] media\_codec\_context\_t \*context: Context specifying the codec type  
- \[OUT\] mc\_video\_roi\_params\_t \*params: ROI encoding parameters  

【Return Values】

- 0: Operation succeeded  
- HB\_MEDIA\_ERR\_UNKNOWN: Unknown error  
- HB\_MEDIA\_ERR\_INVALID\_INSTANCE: Invalid instance  
- HB\_MEDIA\_ERR\_INVALID\_PARAMS: Invalid parameters  

【Function Description】

Get ROI encoding parameters, applicable to H264/H265.

【Example Code】

Refer to [hb_mm_mc_get_longterm_ref_mode](#hb_mm_mc_get_longterm_ref_mode)

#### hb\_mm\_mc\_set\_roi\_config

【Function Declaration】

hb\_s32 hb\_mm\_mc\_set\_roi\_config(media\_codec\_context\_t \*context,  
const mc\_video\_roi\_params\_t \*params)

【Parameter Description】

- \[IN\] media\_codec\_context\_t \*context: Context specifying the codec type  
- \[IN\] const mc\_video\_roi\_params\_t \*params: ROI encoding parameters  

【Return Values】

- 0: Operation succeeded  
- HB\_MEDIA\_ERR\_UNKNOWN: Unknown error  
- HB\_MEDIA\_ERR\_OPERATION\_NOT\_ALLOWED: Operation not allowed  
- HB\_MEDIA\_ERR\_INVALID\_INSTANCE: Invalid instance  
- HB\_MEDIA\_ERR\_INVALID\_PARAMS: Invalid parameters  

【Function Description】Set ROI encoding parameters. This parameter is a dynamic parameter and applies to H.264/H.265.

【Example Code】

Refer to [hb_mm_mc_get_longterm_ref_mode](#hb_mm_mc_get_longterm_ref_mode)

#### hb\_mm\_mc\_get\_mode\_decision\_config

【Function Declaration】

hb\_s32 hb\_mm\_mc\_get\_mode\_decision\_config(media\_codec\_context\_t  
\*context, mc\_video\_mode\_decision\_params\_t \*params);

【Parameter Description】

- \[IN\] media\_codec\_context\_t \*context: Context specifying the codec type  
- \[OUT\] mc\_video\_mode\_decision\_params\_t \*params: Mode decision parameters  

【Return Values】

- 0: Operation succeeded  
- HB\_MEDIA\_ERR\_UNKNOWN: Unknown error  
- HB\_MEDIA\_ERR\_INVALID\_INSTANCE: Invalid instance  
- HB\_MEDIA\_ERR\_INVALID\_PARAMS: Invalid parameters  

【Function Description】

Retrieve mode decision parameters. This function applies to H265.

【Example Code】

Refer to [hb_mm_mc_get_longterm_ref_mode](#hb_mm_mc_get_longterm_ref_mode)

#### hb\_mm\_mc\_set\_mode\_decision\_config

【Function Declaration】

hb\_s32 hb\_mm\_mc\_set\_mode\_decision\_config(media\_codec\_context\_t  
\*context, const mc\_video\_mode\_decision\_params\_t \*params);

【Parameter Description】

- \[IN\] media\_codec\_context\_t \*context: Context specifying the codec type  
- \[IN\] const mc\_video\_mode\_decision\_params\_t \*params: Mode decision parameters  

【Return Values】

- 0: Operation succeeded  
- HB\_MEDIA\_ERR\_UNKNOWN: Unknown error  
- HB\_MEDIA\_ERR\_OPERATION\_NOT\_ALLOWED: Operation not allowed  
- HB\_MEDIA\_ERR\_INVALID\_INSTANCE: Invalid instance  
- HB\_MEDIA\_ERR\_INVALID\_PARAMS: Invalid parameters  

【Function Description】

Set mode decision parameters. This parameter is a dynamic parameter and applies to H.265.

【Example Code】

Refer to [hb_mm_mc_get_longterm_ref_mode](#hb_mm_mc_get_longterm_ref_mode)

#### hb\_mm\_mc\_get\_user\_data

【Function Declaration】

hb\_s32 hb\_mm\_mc\_get\_user\_data(media\_codec\_context\_t \*context,  
mc\_user\_data\_buffer\_t \*params , hb\_s32 timeout)

【Parameter Description】

- \[IN\] media\_codec\_context\_t \*context: Context specifying the codec type  
- \[OUT\] mc\_user\_data\_buffer\_t \*params: User data  
- \[IN\] timeout: Timeout duration  

【Return Values】

- 0: Operation succeeded  
- HB\_MEDIA\_ERR\_UNKNOWN: Unknown error  
- HB\_MEDIA\_ERR\_OPERATION\_NOT\_ALLOWED: Operation not allowed  
- HB\_MEDIA\_ERR\_INVALID\_INSTANCE: Invalid instance  
- HB\_MEDIA\_ERR\_INVALID\_PARAMS: Invalid parameters  

【Function Description】

Retrieve user data from the decoded stream. This function applies to H.264/H.265.

【Example Code】

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
                        ret = -1;break;
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
                printf("%s[%d:%d] Failed to malloc seqHeader\n",
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
                }if (ctx->testLog) {
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

#### hb_mm_mc_release_user_data

【Function Declaration】

hb_s32 hb_mm_mc_release_user_data(media_codec_context_t *context, const mc_user_data_buffer_t * params)

【Parameter Description】

- [IN] media_codec_context_t *context: Context specifying the codec type
- [IN] const mc_user_data_buffer_t * params: User data

【Return Values】

- 0: Operation succeeded
- HB_MEDIA_ERR_UNKNOWN: Unknown error
- HB_MEDIA_ERR_OPERATION_NOT_ALLOWED: Operation not allowed
- HB_MEDIA_ERR_INVALID_INSTANCE: Invalid instance
- HB_MEDIA_ERR_INVALID_PARAMS: Invalid parameters

【Function Description】

Releases user data from the decoded stream. Applicable to H.264/H.265.

【Example Code】

Refer to [hb_mm_mc_get_user_data](#hb_mm_mc_get_user_data)

#### hb_mm_mc_get_mjpeg_config

【Function Declaration】

hb_s32 hb_mm_mc_get_mjpeg_config(media_codec_context_t *context, mc_mjpeg_enc_params_t *params)

【Parameter Description】

- [IN] media_codec_context_t *context: Context specifying the codec type
- [OUT] mc_mjpeg_enc_params_t *params: MJPEG encoding parameters

【Return Values】

- 0: Operation succeeded
- HB_MEDIA_ERR_UNKNOWN: Unknown error
- HB_MEDIA_ERR_INVALID_INSTANCE: Invalid instance
- HB_MEDIA_ERR_INVALID_PARAMS: Invalid parameters

【Function Description】

Retrieves MJPEG encoding parameters. Applicable to MJPEG.

【Example Code】

```# include "hb_media_codec.h"
# include "hb_media_error.h"
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

【Function Declaration】

hb\_s32 hb\_mm\_mc\_set\_mjpeg\_config(media\_codec\_context\_t
\*context, const mc\_mjpeg\_enc\_params\_t \*params)

【Parameter Description】

- \[IN\] media\_codec\_context\_t \*context: Context specifying the codec type
- \[IN\] const mc\_mjpeg\_enc\_params\_t \*params: MJPEG encoding parameters

【Return Values】

- 0: Operation succeeded
- HB\_MEDIA\_ERR\_UNKNOWN: Unknown error
- HB\_MEDIA\_ERR\_OPERATION\_NOT\_ALLOWED: Operation not allowed
- HB\_MEDIA\_ERR\_INVALID\_INSTANCE: Invalid instance
- HB\_MEDIA\_ERR\_INVALID\_PARAMS: Invalid parameters

【Function Description】

Sets MJPEG encoding parameters. These parameters are dynamic and applicable to MJPEG.

【Example Code】

Refer to [hb_mm_mc_get_mjpeg_config](#hb_mm_mc_get_mjpeg_config)

#### hb\_mm\_mc\_get\_jpeg\_config

【Function Declaration】

hb\_s32 hb\_mm\_mc\_get\_jpeg\_config(media\_codec\_context\_t
\*context, mc\_jpeg\_enc\_params\_t \*params)

【Parameter Description】

- \[IN\] media\_codec\_context\_t \*context: Context specifying the codec type
- \[OUT\] mc\_jpeg\_enc\_params\_t \*params: JPEG encoding parameters

【Return Values】

- 0: Operation succeeded
- HB\_MEDIA\_ERR\_UNKNOWN: Unknown error
- HB\_MEDIA\_ERR\_INVALID\_INSTANCE: Invalid instance
- HB\_MEDIA\_ERR\_INVALID\_PARAMS: Invalid parameters

【Function Description】

Retrieves JPEG encoding parameters, applicable to JPEG.

【Example Code】

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

【Function Declaration】

hb\_s32 hb\_mm\_mc\_set\_jpeg\_config(media\_codec\_context\_t
\*context, const mc\_jpeg\_enc\_params\_t \*params)

【Parameter Description】  

- \[IN\] media\_codec\_context\_t \*context: Context specifying the codec type  
- \[IN\] const mc\_jpeg\_enc\_params\_t \*params: JPEG encoding parameters  

【Return Values】

- 0: Operation succeeded  
- HB\_MEDIA\_ERR\_UNKNOWN: Unknown error  
- HB\_MEDIA\_ERR\_OPERATION\_NOT\_ALLOWED: Operation not allowed  
- HB\_MEDIA\_ERR\_INVALID\_INSTANCE: Invalid instance  
- HB\_MEDIA\_ERR\_INVALID\_PARAMS: Invalid parameters  

【Function Description】

Sets JPEG encoding parameters. These parameters are dynamic and applicable to JPEG encoding.

【Example Code】

Refer to [hb_mm_mc_get_jpeg_config](#hb_mm_mc_get_jpeg_config)

#### hb\_mm\_mc\_get\_fd

**Function Declaration**

hb\_s32 hb\_mm\_mc\_get\_fd(media\_codec\_context\_t \* context, hb\_s32 \*fd)

**Parameter Description**

- \[IN\] media\_codec\_context\_t \*context: Context specifying the codec type  
- \[OUT\] hb\_s32 \*fd: File descriptor of the device node  

**Return Values**

- 0: Operation succeeded  
- HB\_MEDIA\_ERR\_INVALID\_INSTANCE: Invalid instance  
- HB\_MEDIA\_ERR\_INVALID\_PARAMS: Invalid parameters  

**Function Description**

Obtains the file descriptor (fd) of the device node, which can be used with the select() operation to monitor codec results.

**Example Code**

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
                        printf("There is no more output data!\n");
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

【Function Declaration】

hb\_s32 hb\_mm\_mc\_close\_fd(media\_codec\_context\_t \* context,
hb\_s32 fd)

**【Parameter Description】

- \[IN\] media\_codec\_context\_t \*context: Context specifying the codec type
- \[IN\] hb\_s32 fd: File descriptor of the device node

【Return Values】

- 0: Operation succeeded
- HB\_MEDIA\_ERR\_UNKNOWN: Unknown error
- HB\_MEDIA\_ERR\_OPERATION\_NOT\_ALLOWED: Operation not allowed
- HB\_MEDIA\_ERR\_INVALID\_INSTANCE: Invalid instance
- HB\_MEDIA\_ERR\_INVALID\_PARAMS: Invalid parameters

【Function Description】

Closes the device node.

【Example Code】

Refer to [hb_mm_mc_get_fd](#hb_mm_mc_get_fd)

#### hb\_mm\_mc\_set\_camera

【Function Declaration】

hb\_s32 hb\_mm\_mc\_set\_camera(media\_codec\_context\_t \*context,
hb\_s32 pipeline, hb\_s32 channel\_port\_id)

【Parameter Description】

- \[IN\] media\_codec\_context\_t \*context: Context specifying the codec type
- \[IN\] hb\_s32 pipeline: Pipeline
- \[IN\] hb\_s32 channel\_port\_id: Channel port ID

【Return Values】

- 0: Operation succeeded
- HB\_MEDIA\_ERR\_UNKNOWN: Unknown error
- HB\_MEDIA\_ERR\_OPERATION\_NOT\_ALLOWED: Operation not allowed
- HB\_MEDIA\_ERR\_INVALID\_INSTANCE: Invalid instance
- HB\_MEDIA\_ERR\_INVALID\_PARAMS: Invalid parameters

【Function Description】

Sets VIO camera information. This parameter is static and applicable to H.264/H.265.

【Example Code】

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

【Function Declaration】

hb\_s32 hb\_mm\_mc\_get\_vui\_config(media\_codec\_context\_t \*context,
mc\_video\_vui\_params\_t \*params)

【Parameter Description】

- \[IN\] media\_codec\_context\_t \*context: Context specifying the codec type
- \[OUT\] mc\_video\_vui\_ params\_t \*params: VUI parameters

【Return Values】

- 0: Operation succeeded
- HB\_MEDIA\_ERR\_UNKNOWN: Unknown error
- HB\_MEDIA\_ERR\_INVALID\_INSTANCE: Invalid instance
- HB\_MEDIA\_ERR\_INVALID\_PARAMS: Invalid parameters

【Function Description】

Retrieves VUI parameters.

【Additional Notes】

Currently, during video encoding, the default color range in the header information is set to full range mode. To set it to limited range, you must explicitly call the hb\_mm\_mc\_set\_vui\_config interface.

【Example Code】

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
        ret = hb_mm_mc_dequeue_output_buffer(context, &outputBuffer, &info,
        3000);
        if (!ret && outFile) {
            fwrite(outputBuffer.vstream_buf.vir_ptr,
                outputBuffer.vstream_buf.size, 1, outFile);
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
int main(int argc, char *argv[]){
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

【Function Declaration】

hb\_s32 hb\_mm\_mc\_set\_vui\_config(media\_codec\_context\_t \*context,
const mc\_video\_vui\_params\_t \*params)

【Parameter Description】

- \[IN\] media\_codec\_context\_t \*context: Context specifying the codec type
- \[IN\] const mc\_video\_vui\_params\_t \*params: VUI parameters

【Return Values】

- 0: Operation succeeded
- HB\_MEDIA\_ERR\_UNKNOWN: Unknown error
- HB\_MEDIA\_ERR\_OPERATION\_NOT\_ALLOWED: Operation not allowed
- HB\_MEDIA\_ERR\_INVALID\_INSTANCE: Invalid instance
- HB\_MEDIA\_ERR\_INVALID\_PARAMS: Invalid parameters

【Function Description】

Sets VUI parameters, which are static parameters.

【Example Code】

Refer to [hb_mm_mc_get_vui_config](#hb_mm_mc_get_vui_config)

#### hb\_mm\_mc\_get\_3dnr\_enc\_config

【Function Declaration】

hb\_s32 hb\_mm\_mc\_get\_3dnr\_enc\_config(media\_codec\_context\_t
\*context, mc\_video\_3dnr\_enc\_params\_t \*params);

【Parameter Description】

- \[IN\] media\_codec\_context\_t \*context: Context specifying the codec type
- \[IN\] mc\_video\_3dnr\_enc\_params\_t \*params: 3DNR parameters

【Return Values】

- 0: Operation succeeded
- HB\_MEDIA\_ERR\_UNKNOWN: Unknown error
- HB\_MEDIA\_ERR\_OPERATION\_NOT\_ALLOWED: Operation not allowed
- HB\_MEDIA\_ERR\_INVALID\_INSTANCE: Invalid instance
- HB\_MEDIA\_ERR\_INVALID\_PARAMS: Invalid parameters

【Function Description】

Retrieves 3DNR parameters, which are dynamic parameters applicable to H.265.

【Example Code】

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
    ret = hb_mm_mc_configure(context);if (ret) {
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
        ret = hb_mm_mc_dequeue_output_buffer(context, &outputBuffer, &info,
        3000);
        if (!ret && outFile) {
            fwrite(outputBuffer.vstream_buf.vir_ptr,
                outputBuffer.vstream_buf.size, 1, outFile);
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

【Function Declaration】

hb\_s32 hb\_mm\_mc\_set\_3dnr\_enc\_config(media\_codec\_context\_t
\*context, const mc\_video\_3dnr\_enc\_params\_t \*params);

【Parameter Description】

- \[IN\] media\_codec\_context\_t \*context：Context specifying the codec type
- \[IN\] const mc\_video\_3dnr\_enc\_params\_t \*params：3DNR parameters

【Return Values】

- 0：Operation succeeded
- HB\_MEDIA\_ERR\_UNKNOWN： Unknown error
- HB\_MEDIA\_ERR\_OPERATION\_NOT\_ALLOWED：Operation not allowed
- HB\_MEDIA\_ERR\_INVALID\_INSTANCE：Invalid instance
- HB\_MEDIA\_ERR\_INVALID\_PARAMS：Invalid parameters


【Function Description】

Set the 3DNR parameters. This parameter is a dynamic one and is applicable to H265.

【Example Code】

Refer to  [hb_mm_mc_get_3dnr_enc_config](#hb_mm_mc_get_3dnr_enc_config)

#### hb\_mm\_mc\_request\_idr\_header

【Function Declaration】

hb\_s32 hb\_mm\_mc\_request\_idr\_header(media\_codec\_context\_t
\*context, hb\_u32 force\_header)

【Parameter Description】

- \[IN\] media\_codec\_context\_t \*context：Context specifying the codec type
- \[IN\] hb\_u32 force\_header：

  > 0 : No froced header(VPS/SPS/PPS)
  >
  > 1 : Forced header before IDR frame
  >
  > 2 : Forced header before I frame for H264 or forced header before
  > CRA and IDR frame for H265
  >

【Return Values】

- 0：Operation succeeded
- HB\_MEDIA\_ERR\_UNKNOWN： Unknown error
- HB\_MEDIA\_ERR\_OPERATION\_NOT\_ALLOWED：Operation not allowed
- HB\_MEDIA\_ERR\_INVALID\_INSTANCE：Invalid instance
- HB\_MEDIA\_ERR\_INVALID\_PARAMS：Invalid parameters

【Function Description】

Request the IDR frame header information of the frame header ID, applicable to H264/H265.

【Example Code】

参考 [hb_mm_mc_get_longterm_ref_mode](#hb_mm_mc_get_longterm_ref_mode)

#### hb\_mm\_mc\_enable\_idr\_frame

【Function Declaration】

hb\_s32 hb\_mm\_mc\_enable\_idr\_frame(media\_codec\_context\_t
\*context, hb\_bool enable)

【Parameter Description】

- \[IN\] media\_codec\_context\_t \*context：Context specifying the codec type
- \[IN\] hb\_bool enable：0: Disable；1: Enable；

【Return Values】

- 0：Operation succeeded
- HB\_MEDIA\_ERR\_UNKNOWN： Unknown error
- HB\_MEDIA\_ERR\_OPERATION\_NOT\_ALLOWED：Operation not allowed
- HB\_MEDIA\_ERR\_INVALID\_INSTANCE：Invalid instance
- HB\_MEDIA\_ERR\_INVALID\_PARAMS：Invalid parameters

【Function Description】

Enable IDR frames, applicable for H264/H265.

【Example Code】

Refer to  [hb_mm_mc_get_longterm_ref_mode](#hb_mm_mc_get_longterm_ref_mode)

#### hb\_mm\_mc\_register\_audio\_encoder

【Function Declaration】

hb\_s32 hb\_mm\_mc\_register\_audio\_encoder(hb\_s32 \*handle,
mc\_audio\_encode\_param\_t \*encoder)

【Parameter Description】

- \[IN\] hb\_s32 \*handle：Encoder handle
- \[IN\] mc\_audio\_encode\_param\_t \*encoder：audio编码器描述符；

【Return Values】

- 0：Operation succeeded
- HB\_MEDIA\_ERR\_UNKNOWN： Unknown error
- HB\_MEDIA\_ERR\_OPERATION\_NOT\_ALLOWED：Operation not allowed
- HB\_MEDIA\_ERR\_INVALID\_INSTANCE：Invalid instance
- HB\_MEDIA\_ERR\_INVALID\_PARAMS：Invalid parameters

【Function Description】

Register the audio encoder, applicable for Audio.

【Example Code】

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

【Function Declaration】

hb\_s32 hb\_mm\_mc\_unregister\_audio\_encoder(hb\_s32 handle)

【Parameter Description】

- \[IN\] hb\_s32 \*handle：encoder handle

【Return Values】

- 0：Operation succeeded
- HB\_MEDIA\_ERR\_UNKNOWN： Unknown error
- HB\_MEDIA\_ERR\_OPERATION\_NOT\_ALLOWED：Operation not allowed 
- HB\_MEDIA\_ERR\_INVALID\_INSTANCE：Invalid instance 
- HB\_MEDIA\_ERR\_INVALID\_PARAMS：Invalid parameters  


【Function Description】

Unregister the audio encoder, applicable for Audio.

【Example Code】

参考 [hb_mm_mc_register_audio_encoder](#hb_mm_mc_register_audio_encoder)

#### hb\_mm\_mc\_register\_audio\_decoder

【Function Declaration】

hb\_s32 hb\_mm\_mc\_register\_audio\_decoder(hb\_s32 \*handle,
mc\_audio\_decode\_param\_t \*decoder)

【Parameter Description】

- \[IN\] hb\_s32 \*handle：Decoder handle
- \[IN\] mc\_audio\_decode\_param\_t \*decoder：Audio decoder descriptor

【Return Values】

- 0：Operation succeeded
- HB\_MEDIA\_ERR\_UNKNOWN： Unknown error
- HB\_MEDIA\_ERR\_OPERATION\_NOT\_ALLOWED：Operation not allowed
- HB\_MEDIA\_ERR\_INVALID\_INSTANCE：Invalid instance
- HB\_MEDIA\_ERR\_INVALID\_PARAMS： Invalid parameters

【Function Description】

Register the audio decoder, applicable for Audio.

【Example Code】

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

#### hb_mm_mc_unregister_audio_decoder  

【Function Declaration】  

hb\_s32 hb\_mm\_mc\_unregister\_audio\_decoder(hb\_s32 handle)

**[Parameter Description]**

- \[IN\] hb\_s32 \*handle: Decoder handle;

**[Return Values]**

- 0: Operation succeeded  
- HB\_MEDIA\_ERR\_UNKNOWN: Unknown error  
- HB\_MEDIA\_ERR\_OPERATION\_NOT\_ALLOWED: Operation not allowed  
- HB\_MEDIA\_ERR\_INVALID\_INSTANCE: Invalid instance  
- HB\_MEDIA\_ERR\_INVALID\_PARAMS: Invalid parameters  

**[Function Description]**

Unregisters an audio decoder. Applicable to Audio.

**[Example Code]**

Refer to [hb_mm_mc_register_audio_decoder](#hb_mm_mc_register_audio_decoder)

#### hb\_mm\_mc\_get\_explicit\_header\_config

**[Function Declaration]**

hb\_s32 hb\_mm\_mc\_get\_explicit\_header\_config  
(media\_codec\_context\_t \*context, hb\_s32 \*status)

**[Parameter Description]**

- \[IN\] media\_codec\_context\_t \*context: Context specifying the codec type  
- \[OUT\] hb\_s32 \*status: Enable/disable encoding header information and IDR frame into a single frame  

**[Return Values]**

- 0: Operation succeeded  
- HB\_MEDIA\_ERR\_UNKNOWN: Unknown error  
- HB\_MEDIA\_ERR\_INVALID\_INSTANCE: Invalid instance  
- HB\_MEDIA\_ERR\_INVALID\_PARAMS: Invalid parameters  

**[Function Description]**

Gets the configuration indicating whether header information and the IDR frame are encoded into a single frame.  
0: IDR and header information are separate  
1: IDR and header information are combined into one frame  
Applicable to H264/H265.

**[Example Code]**

Refer to [hb_mm_mc_get_longterm_ref_mode](#hb_mm_mc_get_longterm_ref_mode)

#### hb\_mm\_mc\_set\_explicit\_header\_config

**[Function Declaration]**

hb\_s32 hb\_mm\_mc\_set\_explicit\_header\_config  
(media\_codec\_context\_t \*context, hb\_s32 status)

**[Parameter Description]**

- \[IN\] media\_codec\_context\_t \*context: Context specifying the codec type  
- \[IN\] hb\_s32 status: Enable/disable encoding header information and I-frame into a single frame  

**[Return Values]**

- 0: Operation succeeded  
- HB\_MEDIA\_ERR\_UNKNOWN: Unknown error  
- HB\_MEDIA\_ERR\_OPERATION\_NOT\_ALLOWED: Operation not allowed  
- HB\_MEDIA\_ERR\_INVALID\_INSTANCE: Invalid instance  
- HB\_MEDIA\_ERR\_INVALID\_PARAMS: Invalid parameters  

**[Function Description]**

Enables/disables encoding header information and I-frame into a single frame. This is a static parameter.  
0: IDR and header information are separate  
1: IDR and header information are combined into one frame  
Applicable to H264/H265.

**[Example Code]**

Refer to [hb_mm_mc_get_longterm_ref_mode](#hb_mm_mc_get_longterm_ref_mode)

#### hb\_mm\_mc\_get\_roi\_avg\_qp

**[Function Declaration]**

hb\_s32 hb\_mm\_mc\_get\_roi\_avg\_qp(media\_codec\_context\_t \*  
context, hb\_u32 \* params)

**[Parameter Description]**

- \[IN\] media\_codec\_context\_t \*context: Context specifying the codec type  
- \[OUT\] hb\_u32 \*params: ROI average QP  

**[Return Values]**

- 0: Operation succeeded  
- HB\_MEDIA\_ERR\_UNKNOWN: Unknown error  
- HB\_MEDIA\_ERR\_INVALID\_INSTANCE: Invalid instance  
- HB\_MEDIA\_ERR\_INVALID\_PARAMS: Invalid parameters  

**[Function Description]**

Gets the ROI average QP value. 0 indicates that the value is determined by the user-defined QP map. Applicable to H264/H265.

**[Example Code]**

Refer to [hb_mm_mc_get_longterm_ref_mode](#hb_mm_mc_get_longterm_ref_mode)

#### hb\_mm\_mc\_set\_roi\_avg\_qp

**[Function Declaration]**

hb\_s32 hb\_mm\_mc\_set\_roi\_avg\_qp(media\_codec\_context\_t \*  
context, hb\_u32 params)

**[Parameter Description]**

- \[IN\] media\_codec\_context\_t \*context: Context specifying the codec type  
- \[IN\] hb\_u32 params: ROI average QP value  

**[Return Values]**

- 0: Operation succeeded  
- HB\_MEDIA\_ERR\_UNKNOWN: Unknown error  
- HB\_MEDIA\_ERR\_OPERATION\_NOT\_ALLOWED: Operation not allowed  
- HB\_MEDIA\_ERR\_INVALID\_INSTANCE: Invalid instance  
- HB\_MEDIA\_ERR\_INVALID\_PARAMS: Invalid parameters  

**[Function Description]**

Sets the ROI average QP value for encoding. This is a dynamic parameter.  
0: Indicates using the average of all values in the configured QP map.  
This setting takes effect only when the rate control (RC) mode is CBR or AVBR.  
Applicable to H264/H265.

**[Example Code]**

Refer to [hb_mm_mc_get_longterm_ref_mode](#hb_mm_mc_get_longterm_ref_mode)

### Main Parameter Descriptions

#### media\_codec\_state\_t

**[Description]**

Defines the internal operational states of the Media codec.

**[Definition]**

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

**[Description]**

Defines the codec IDs supported by MediaCodec.

**[Definition]**

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

**Description**

Defines the bitrate control modes for video. Currently, only H.264/H.265 and MJPEG encoding channels support bitrate control.

**Definition**

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

**Description**

Defines the adjustable parameter set under H.264 CBR (Constant Bitrate) control mode.

**Definition**

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

**Description**

Defines the adjustable parameter set under H.264 VBR (Variable Bitrate) control mode.

**Definition**

```
typedef struct _mc_h264_vbr_params {
    hb_u32 intra_period;
    hb_u32 intra_qp;
    hb_u32 frame_rate;
    hb_bool qp_map_enable;
} mc_h264_vbr_params_t;
```

#### mc\_h264\_avbr\_params\_t

**Description**

Defines the adjustable parameter set under H.264 AVBR (Adaptive Variable Bitrate) control mode.

**Definition**

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

**Description**

Defines the adjustable parameter set under H.264 FixQP (Fixed Quantization Parameter) control mode.

**Definition**

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

**Description**

Defines the adjustable parameter set under H.264 QPMAP control mode.

**Definition**

```
typedef struct _mc_h264_qp_map_params {
    hb_u32 intra_period;
    hb_u32 frame_rate;
    hb_byte qp_map_array;
    hb_u32 qp_map_array_count;
} mc_h264_qp_map_params_t;
```

#### mc\_h265\_cbr\_params\_t

**Description**

Defines the adjustable parameter set under H.265 CBR (Constant Bitrate) control mode.

**Definition**

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

**Description**

Defines the adjustable parameter set under H.265 VBR (Variable Bitrate) control mode.

**Definition**

```
typedef struct _mc_h265_vbr_params {
    hb_u32 intra_period;
    hb_u32 intra_qp;
    hb_u32 frame_rate;
    hb_bool qp_map_enable;
} mc_h265_vbr_params_t;
```

#### mc\_h265\_avbr\_params\_t

**Description**

Defines the adjustable parameter set under H.265 AVBR (Adaptive Variable Bitrate) control mode.【Definition】

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

【Description】

Defines the adjustable parameter set for H.265 under FixQP rate control mode.

【Definition】

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

【Description】

Defines the adjustable parameter set for H.265 under QPMAP rate control mode.

【Definition】

```
typedef struct _mc_h265_qp_map_params {
    hb_u32 intra_period;
    hb_u32 frame_rate;
    hb_byte qp_map_array;
    hb_u32 qp_map_array_count;
} mc_h265_qp_map_params_t;
```

#### mc\_mjpeg\_fix\_qp\_params\_t

【Description】

Defines the adjustable parameter set for MJPEG under FixQP rate control mode.

【Definition】

```
typedef struct _mc_mjpeg_fix_qp_params {
    hb_u32 frame_rate;
    hb_u32 quality_factor;
} mc_mjpeg_fix_qp_params_t;
```

#### mc\_video\_custom\_gop\_pic\_params\_t

【Description】

Defines the data structure for a custom GOP structure table.

【Definition】

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

【Description】

Defines internal status information of the media codec.

【Definition】

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

【Description】

Defines the context of the Media codec.

【Definition】

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

【Description】

Defines encoding parameters for the video encoder. Supported video encoder types include H.264, H.265, MJPEG, and JPEG.

【Definition】

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

【Description】

Defines decoding parameters for the video decoder. Supported video decoder types include H.264, H.265, MJPEG, and JPEG.

【Definition】

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

【Description】

Defines the encoding parameters for an audio codec.

【Definition】

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

【Description】

Defines the decoding parameters for an audio codec.

【Definition】

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

### Return Value Description

| Error Code | Macro Definition | Description |
| --- | --- | --- |
| 0xF0000001 | HB_MEDIA_ERR_UNKNOWN | Unknown error |
| 0xF0000002 | HB_MEDIA_ERR_CODEC_NOT_FOUND | Corresponding codec not found |
| 0xF0000003 | HB_MEDIA_ERR_CODEC_OPEN_FAIL | Failed to open codec device |
| 0xF0000004 | HB_MEDIA_ERR_CODEC_RESPONSE_TIMEOUT | Codec response timeout |
| 0xF0000005 | HB_MEDIA_ERR_CODEC_INIT_FAIL | Codec initialization failed |
| 0xF0000006 | HB_MEDIA_ERR_OPERATION_NOT_ALLOWED | Operation not allowed |
| 0xF0000007 | HB_MEDIA_ERR_INSUFFICIENT_RES | Insufficient internal memory resources |
| 0xF0000008 | HB_MEDIA_ERR_NO_FREE_INSTANCE | No available instance (VPU supports up to 32, JPU up to 64, Audio up to 32) |
| 0xF0000009 | HB_MEDIA_ERR_INVALID_PARAMS | Invalid parameters |
| 0xF000000A | HB_MEDIA_ERR_INVALID_INSTANCE | Invalid instance |
| 0xF000000B | HB_MEDIA_ERR_INVALID_BUFFER | Invalid buffer |
| 0xF000000C | HB_MEDIA_ERR_INVALID_COMMAND | Invalid command |
| 0xF000000D | HB_MEDIA_ERR_WAIT_TIMEOUT | Wait timeout |
| 0xF000000E | HB_MEDIA_ERR_FILE_OPERATION_FAILURE | File operation failed |
| 0xF000000F | HB_MEDIA_ERR_PARAMS_SET_FAILURE | Parameter setting failed |
| 0xF0000010 | HB_MEDIA_ERR_PARAMS_GET_FAILURE | Parameter retrieval failed |
| 0xF0000011 | HB_MEDIA_ERR_CODING_FAILED | Encoding/decoding failed |
| 0xF0000012 | HB_MEDIA_ERR_OUTPUT_BUF_FULL | Output buffer full |
| 0xF0000013 | HB_MEDIA_ERR_UNSUPPORTED_FEATURE | Unsupported feature |
| 0xF0000014 | HB_MEDIA_ERR_INVALID_PRIORITY | Unsupported priority |

## Codec Sample

### Encoding Example

#### Function Overview

Encodes YUV images into H.264/H.265 video or JPG images.

##### Software Architecture Description

Uses MediaCodec's poll mode to decouple input and output, enabling optimal encoding frame rate performance.  
In the main thread, YUV data is fed into the encoder: an empty input buffer is acquired, the YUV data's address information (e.g., physical address) is configured, and then the input buffer is queued to notify the encoder to process this frame.  
In another thread, encoded output bitstreams are retrieved: upon receiving a hardware encoding completion notification via `select`, a filled output buffer is acquired, the encoded result is written to a file, and then the output buffer is returned.

![image](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/03_multimedia_development/02_S100/codec/encoder2.png)

##### Hardware Data Flow Description

![image](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/03_multimedia_development/02_S100/codec/encoder1.png)

##### Code Location and Directory Structure

The sample code is located at `source/hobot-sp-samples/debian/app/multimedia_demo/codec_demo` in the project directory.

Directory structure:

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

The root directory contains `README.md`, which briefly describes compilation commands, runtime help information, and usage instructions.

The `Makefile` under `sample_venc_vdec` is used to compile files in this directory. Specifically:
- `sample.c` contains the main entry point,
- `sample_common.c` contains shared APIs,
- `sample_venc.c` contains encoding-related functions,
- `sample_vdec.c` contains decoding-related functions.

#### Compilation

##### Compilation Environment

After installing the `hobot-sp-samples_*.deb` package on the board, the `codec_demo` source code will be included.

##### Compilation Instructions

This sample primarily depends on API header files provided by `libmm`:

```
#include "hb_media_codec.h"
#include "hb_media_error.h"
```

Compilation dependencies include the following libraries:

```
LIBS += -lpthread -ldl -lhbmem -lalog  -lmultimedia
LIBS += -lavformat -lavcodec -lavutil -lswresample
```

Compilation command:

On the board, navigate to `/app/multimedia_demo/codec_demo/sample_venc_vdec` and run:

```
make
```

#### Execution

##### Supported Platforms

RDKS100.

##### Board Deployment and Configuration

After flashing the system software image, the sample source code resides at `/app/multimedia_demo/codec_demo/sample_venc_vdec` on the board.

Required resources may include:
- Input YUV images: by default, raw 4K YUV streams and H.264 files are provided; for other tests, users must upload their own files.

##### Running Instructions

###### Command-line Argument Description

`sample_codec`: application name

`-m`: encoding or decoding mode; default is encoding (0: encoder, 1: decoder)

`-c`: codec type; default is H.264 (0: H.264, 1: H.265, 2: MJPEG, 3: JPEG)

`-w`: image width; default is 3840

`-h`: image height; default is 2160

`-p`: pixel format for encoding/decoding; default is NV12 (0: YUV420P, 1: NV12, 2: NV21)

`-n`: number of test threads; default is 1

`-i`: input file path; default is `./input_${w}x${h}_${pixfmt}<_thread_idx>.yuv`

`-o`: output file path; default is `./output_${w}x${h}_${pixfmt}<_thread_idx>.{code_type}`

`-H`: print help information

###### Help Menu

Run `./sample_codec --help` to display the help menu as follows:
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

###### Running Method

Prepare input source: place the test file (e.g., `input_3840x2160_nv12.yuv`) in the current directory, or specify the file path using `-i`.Encode a YUV image sequence with resolution 3840x2160 into an H264 video bitstream:

```
/app/multimedia_demo/codec_demo/sample_venc_vdec/sample_codec
```

Encode a YUV image sequence with resolution 1920x1080 into an H265 video bitstream:

```
/app/multimedia_demo/codec_demo/sample_venc_vdec/sample_codec -c 1 -w 1920 -h 1080
```

Encode a single YUV image with resolution 1920x1088 into a JPG image:

```
/app/multimedia_demo/codec_demo/sample_venc_vdec/sample_codec -c 3 -w 1920 -h 1088
```

Encode two YUV image sequences with resolution 3840x2160 into H265 videos:

```
/app/multimedia_demo/codec_demo/sample_venc_vdec/sample_codec -c 1 -n 2
```

Encode four YUV image sequences with resolution 1920x1080 into H265 videos:

```
/app/multimedia_demo/codec_demo/sample_venc_vdec/sample_codec -c 1 -w 1920 -h 1080 -n 4
```

VPU CROP read and encode: Read input data of size 1920x1300 (image dimensions not aligned as required) according to the crop region `{x=200, y=300, w=1280, h=720}` and encode it into an H265 video:

```
/app/multimedia_demo/codec_demo/sample_venc_vdec/sample_codec -c 1 -w 1920 -h 1300
```

###### Execution Result Description

The following figure shows a successful execution:

![image](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/03_multimedia_development/02_S100/codec/encoder3.png)

Check whether the generated H264/H265/JPG files are valid:

![image](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/03_multimedia_development/02_S100/codec/encoder4.png)

### Decoding Examples

#### Function Overview

Decode H264/H265 videos or JPG images into YUV images.

##### Software Architecture

MediaCodec's poll mode is adopted to decouple input and output, enabling optimal decoding frame rate performance.  
In the main thread, bitstream data is fed into the decoder: an empty input buffer is acquired, the physical address and other metadata of the bitstream data are configured, then the input buffer is queued to notify the decoder to process this frame.  
In another thread, decoded YUV images are retrieved: upon receiving a hardware decoding completion notification via `select`, a filled output buffer is acquired, the decoded result is written to a file, and the output buffer is released back.

![image](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/03_multimedia_development/02_S100/codec/decoder2.png)

##### Hardware Data Flow Description

![image](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/03_multimedia_development/02_S100/codec/decoder1.png)

##### Code Location and Directory Structure

Sample code is located under `source/hobot-sp-samples/debian/app/multimedia_demo/codec_demo` in the project directory.

Directory structure:

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

The root directory contains `README.md`, which briefly describes compilation commands, runtime help, and usage instructions.

The `Makefile` under `sample_venc_vdec` is used for compiling this directory. Specifically:
- `sample.c` contains the main entry point.
- `sample_common.c` provides common APIs.
- `sample_venc.c` implements encoding-related functions.
- `sample_vdec.c` implements decoding-related functions.

#### Compilation

##### Compilation Environment

After installing the `hobot-sp-samples_*.deb` package on the board, the `codec_demo` source code will be included.

##### Compilation Instructions

This sample primarily depends on API header files provided by `libmm`:

```
#include "hb_media_codec.h"
#include "hb_media_error.h"
```

Compilation requires the following libraries:

```
LIBS += -lpthread -ldl -lhbmem -lalog  -lmultimedia
LIBS += -lavformat -lavcodec -lavutil -lswresample
```

Compilation command:

On the board, navigate to `/app/multimedia_demo/codec_demo/sample_venc_vdec` and run:

```
make
```

#### Execution

##### Supported Platforms

RDKS100.

##### Board Deployment and Configuration

After flashing the system image, the sample source code resides at `/app/multimedia_demo/codec_demo/sample_venc_vdec` on the board.

Required resources may include:
- Default input YUV streams and H264 files in 4K format; users must upload other test files as needed.

##### Running Guide

###### Parameter Description

`sample_codec`: Application name

`-m`: Encoding or decoding mode; default is encoding (`0`: encoder, `1`: decoder)

`-c`: Codec type; default is H264 (`0`: H264, `1`: H265, `2`: mjpg, `3`: jpg)

`-w`: Image width; default is 3840

`-h`: Image height; default is 2160

`-p`: Pixel format; default is nv12 (`0`: yuv420p, `1`: nv12, `2`: nv21)

`-n`: Number of test threads; default is 1

`-i`: Input file path; default is `./input_${w}x${h}_${pixfmt}<_thread_idx>.yuv`

`-o`: Output file path; default is `./output_${w}x${h}_${pixfmt}<_thread_idx>.{code_type}`

`-H`: Print help information

###### Help Menu

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

###### How to Run

Prepare input sources: place test files (e.g., `input_3840x2160_nv12.h264`) in the current directory, or specify the file path using `-i`.

Decode one H264 video stream with resolution 3840x2160 into YUV images:

```
/app/multimedia_demo/codec_demo/sample_venc_vdec/sample_codec -m 1
```

Decode one H265 video stream with resolution 1920x1080 into YUV images:

```
/app/multimedia_demo/codec_demo/sample_venc_vdec/sample_codec -m 1 -c 1 -w 1920 -h 1080
```

Decode one JPG image with resolution 1920x1088 into a YUV image:

```
/app/multimedia_demo/codec_demo/sample_venc_vdec/sample_codec -m 1 -c 3 -w 1920 -h 1088
```

Decode two H264 video streams with resolution 3840x2160 into YUV images:

```/app/multimedia_demo/codec_demo/sample_venc_vdec/sample_codec -m 1 -n 2
```

Decode four 1920x1080 h265 video streams and generate YUV image files.

```
/app/multimedia_demo/codec_demo/sample_venc_vdec/sample_codec -m 1 -c 1 -n 4 -w 1920 -h 1080
```

###### Explanation of Execution Results

Successful execution is shown in the figure below:

![image](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/03_multimedia_development/02_S100/codec/decoder3.png)

Use YUVPlayer to verify whether the generated YUV image files are valid.

![image](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/03_multimedia_development/02_S100/codec/decoder4.png)