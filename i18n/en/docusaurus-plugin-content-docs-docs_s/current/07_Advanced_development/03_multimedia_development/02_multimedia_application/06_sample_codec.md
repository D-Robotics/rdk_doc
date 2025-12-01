# sample_codec Usage Instructions
## Function Overview
sample_codec is a sample program for video encoding and decoding. It can perform video encoding and decoding according to the configuration items defined in the configuration file (`codec_config.ini`), helping users debug video codecs.

### Data Flow Description
#### Encoding Data Flow

![sample_codec_encode_data_flow](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/images_to_upload/sample_codec_encode_data_flow.png)

#### Decoding Data Flow


![sample_codec_decode_data_flow](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/images_to_upload/sample_codec_decode_data_flow.png)

### Code Location and Directory Structure

- Code location: `/app/multimedia_samples/sample_codec`
- Directory structure:
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

## Compilation

- Enter the sample_codec directory and run `make` to compile.
- The output binary is `sample_codec` located in the sample_codec source directory.

**Note:** Sample resource files `1280x720_NV12.yuv`, `1920x1080_NV12.yuv`, and `640x480_30fps.h264` have already been prepared in the sample code directory. You can quickly run encoding examples at 720P and 1080P resolutions and an H264 decoding example at 640x480 resolution using these files. To run encoding/decoding tasks at other resolutions, please refer to the `codec_config.ini` configuration to add or modify configuration items and prepare your input files accordingly.

## Execution

### Program Execution Method

Run the program `./sample_codec -h` to get help information:

### Program Command-Line Options
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
**Options:**

- `-f, --config_file FILE`: Specify the path to the configuration file (optional). Default value is `codec_config.ini`.
- `-e, --encode [OPTION]`: Set the encoding option (optional). If this option is used, it overrides the `encode_streams` setting in the configuration file.
- `-d, --decode [OPTION]`: Set the decoding option (optional). If this option is used, it overrides the `decode_streams` setting in the configuration file.
- `-v, --verbose`: Enable verbose mode to display more log information.
- `-h, --help`: Display help information.

#### Usage Examples

- Start encoding/decoding using the default settings in `codec_config.ini` (by default, one H264 encoding stream is enabled: `encode_streams = 0x1`). No arguments are needed when running the program:

  ```
  ./sample_codec
  ```

- Start specified encoding streams (defined in `codec_config.ini`):

  ```
  ./sample_codec -e 0x1  # Start venc_stream1
  ./sample_codec -e 0x3  # Start venc_stream1 and venc_stream2
  ```

- Start specified decoding streams (defined in `codec_config.ini`):

  ```
  ./sample_codec -e 0 -d 0x1 # Disable encoding, start vdec_stream1
  ./sample_codec -e 0 -d 0x3  # Disable encoding, start vdec_stream1 and vdec_stream2
  ```

- Enable verbose mode for more detailed logging:

  ```
  ./sample_codec -v
  ```

- Display help information:

  ```
  ./sample_codec -h
  ```

### Configuration File Description

The `codec_config.ini` file defines various video encoding and decoding parameters and sets the default number of encoding/decoding channels to start.

Encoding parameter options are described as follows:

```
[encode]
; Enable encoding streams using bitwise operations
; 0x0 means no encoding streams enabled
; 0x01 means only venc_stream1 is enabled
; 0x02 means only venc_stream2 is enabled
; 0x03 means the first two encoding streams (venc_stream1 and venc_stream2) are enabled,
; 0x07 means the first three encoding streams are enabled,
; 0x0f means the first four encoding streams are enabled, and so on.
encode_streams = 0x1

[venc_stream1]
; Codec type (0: H264, 1: H265, 2: MJPEG, 3: JPEG)
codec_type = 0
width = 1920
height = 1080
frame_rate = 30
bit_rate = 8192
input = 1920x1080.yuv
output = 1920x1080_30fps.264
frame_num = 100
; profile, level, tier configuration
; H264 supports common baseline/main/high Profiles up to Level 5.2 (i.e., max high@L5.2)
; H265 supports main/main still picture profile @ L5.1 High Tier
; For details, please refer to the source code...
profile = h264_main@L4

[decode]
; Enable decoding streams using bitwise operations
; 0x0 means no decoding streams enabled
; 0x01 means only vdec_stream1 is enabled
; 0x02 means only vdec_stream2 is enabled
; 0x03 means the first two decoding streams (vdec_stream1 and vdec_stream2) are enabled,
; 0x07 means the first three decoding streams are enabled,
; 0x0f means the first four decoding streams are enabled, and so on.
decode_streams = 0x0

[vdec_stream1]
; Codec type (0: H264, 1: H265, 2: MJPEG, 3: JPEG)
codec_type = 0
width = 1920
height = 1080
input = 1920x1080_30fps.264
output = 1920x1080.yuv
```

#### Encoding Configuration

##### [encode]
- **encode_streams**: This option specifies which encoding streams to enable, using bitwise representation. For example, `0x1` enables only `venc_stream1`, and `0x3` enables the first two encoding streams (`venc_stream1` and `venc_stream2`). **This option can be overridden by the command-line argument `-e`.**

##### [venc_stream]
- **codec_type**: Specifies the encoding type. Valid values are `0` (H264), `1` (H265), `2` (MJPEG), and `3` (JPEG).
- **width**: Width of the video frame.
- **height**: Height of the video frame.
- **frame_rate**: Frame rate of the video.
- **bit_rate**: Bitrate of the video.
- **input**: Input image file, supporting only NV12 formatted YUV images. Multiple frames can be stored sequentially in one file; during encoding, frames are read sequentially and cyclically.
- **output**: Output encoded video file.
- **frame_num**: Number of video frames to encode. If the input YUV file contains fewer frames than this value, the file will be read cyclically until the specified number of frames is reached or exceeded.
- **performance_test**: Specifies whether to run a performance test. The difference from the normal flow is that the performance test preloads video frames from the file into an external buffer.
- **profile**: Configuration for profile, level, and tier.
  - H264 supports baseline/main/high Profiles up to Level 5.2 (i.e., maximum high@L5.2).
  - H265 supports main/main still picture profile @ L5.1 High Tier.

#### Decoding Configuration

##### [decode]
- **decode_streams**: This section specifies which decoding streams to enable, using bitwise representation. For example, `0x1` enables only `vdec_stream1`, and `0x3` enables the first two decoding streams (`vdec_stream1` and `vdec_stream2`).

##### [vdec_stream]
- **codec_type**: Specifies the video codec type to decode. Valid values are `0` (H264), `1` (H265), `2` (MJPEG), and `3` (JPEG).
- **width**: Width of the decoded video frame.
- **height**: Height of the decoded video frame.
- **input**: Path to the input video file to be decoded. Depending on the `codec_type` setting, this can be either a bitstream file or an RTSP stream.
- **output**: Path for the output decoded image file. Only NV12 formatted YUV images are supported. Decoded frames are saved sequentially into a single file, so ensure sufficient disk space at the output path. Note that YUV images typically consume significant disk space, especially for high-resolution or long-duration videos. Please ensure adequate free space on the target storage device when selecting the output path.

### Execution Results

Taking the first H264 encoding stream enabled in the configuration file (`encode_streams = 0x1`) as an example:


```
./sample_codec
Config file: codec_config.ini
```encode_streams: 0x1  
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

According to the configuration `frame_num = 100` in `codec_config.ini`, the program automatically exits after encoding 100 frames.