---
sidebar_position: 2
---

# 3.4.2 Reference Examples (C++)

This section introduces multiple functional examples for multimedia library development, including camera image capture, video encoding/decoding, video display, algorithm inference, and other features.

## Camera Image Capture and Display

The `vio2display` example implements image capture from a `MIPI` camera and outputs the video via an `HDMI` interface, allowing users to preview the live feed on a monitor.

Example flow diagram:

![image-vio_to_display](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/04_multi_media/image/cdev_demo/s100/image-vio_to_display.png)

- **Environment Setup:**

  - With the development board powered off, connect the `MIPI` camera to the board. Currently, this sample only supports MIPI sensors: IMX219, SC230AI.
  - Connect the development board to a monitor using an HDMI cable.
  - Power on the development board and log in via the command line.

- **Execution Method:**
  The example code is provided in source form and must be compiled using the `make` command before running. Follow these steps:

  ```bash
  sunrise@ubuntu:~$ cd /app/cdev_demo/vio2display
  sunrise@ubuntu:/app/cdev_demo/vio2display$ make
  sunrise@ubuntu:/app/cdev_demo/vio2display$ ./vio2display -w 1920 -h 1080
  ```

  Parameter descriptions:

  - `-w`: Sensor output width  
  - `-h`: Sensor output height  

- **Expected Result:**
  After successful execution, the development board outputs the real-time video captured by the `MIPI` camera to the connected monitor. Sample log output:

  ```bash
  sunrise@ubuntu:/app/cdev_demo/vio2display$ ./vio2display -w 1920 -h 1080
  [UCP]: log level = 3
  [UCP]: UCP version = 3.7.3
  [VP]: log level = 3
  [DNN]: log level = 3
  [HPL]: log level = 3
  [UCPT]: log level = 6
  disp_w=1920, disp_h=1080
  2025/06/16 10:12:21.574 !INFO [CamInitParam][0295]set camera fps: -1,width: 1920,height: 1080
  mipi mclk is not configed.
  Searching camera sensor on device: /proc/device-tree/soc/vcon@0 i2c bus: 1 mipi rx phy: 0
  WARN: Sensor Name: ar0820std-30fps, Expected Chip ID: 0xCB34, Actual Chip ID Read: 0x00
  [0] INFO: Found sensor name:imx219-30fps on mipi rx csi 0, i2c addr 0x10, config_file:linear_1920x1080_raw10_30fps_1lane.c
  2025/06/16 10:12:21.575 !INFO [CamInitPymParam][0258]Setting PYM channel:0: crop_x:0, crop_y:0, input_width:1920, input_height:1080, dst_w:1920, dst_h:1080
  sp_open_camera success!
  2025/06/16 10:12:21.727 !INFO [OpenDisplay][0111]Wayland is available, using Wayland for rendering.
  Using default socket path: /run/user/1000/wayland-0
  Press 'q' to Exit !
  ```

## Local Saving of Camera Images

The `vio_capture` example captures images from a MIPI camera and saves them locally in both RAW and YUV formats. The example flow diagram is as follows:

Example flow diagram:

![image-vio_capture.png](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/04_multi_media/image/cdev_demo/s100/image-vio_capture.png)

- **Environment Setup:**

  - With the development board powered off, connect the `MIPI` camera to the board. Currently, this sample only supports MIPI sensors: IMX219, SC230AI.
  - Power on the development board and log in via the command line.

- **Execution Method:**
  The example code is provided in source form and must be compiled using the `make` command before running. Follow these steps:

  ```bash
  sunrise@ubuntu:~$ cd /app/cdev_demo/vio_capture/
  sunrise@ubuntu:/app/cdev_demo/vio_capture$ make
  sunrise@ubuntu:/app/cdev_demo/vio_capture$ ./capture -b 10 -c 10 -w 1920 -h 1080
  ```

  Parameter descriptions:

  - `-b`: Bit depth of the RAW image. For example, IMX219 supports RAW10 format, so the bit depth is 10. Refer to the [Accessory List](http://sysgbj2.hobot.cc/rdk_doc/rdk_s/Advanced_development/hardware_development/accessory) for supported sensor formats.
  - `-c`: Number of images to save. The interval between capturing each image is typically 1/fps.
  - `-w`: Width of the saved images
  - `-h`: Height of the saved images

- **Expected Result:**
  After successful execution, the specified number of image files are saved in the current directory. RAW format files are named `raw_*.raw`, and YUV format files are named `yuv_*.yuv`. Sample log output:

  ```bash
  sunrise@ubuntu:/app/cdev_demo/vio_capture$ ./capture -b 10 -c 10 -w 1920 -h 1080
  [UCP]: log level = 3
  [UCP]: UCP version = 3.7.3
  [VP]: log level = 3
  [DNN]: log level = 3
  [HPL]: log level = 3
  [UCPT]: log level = 6
  2025/06/04 22:24:22.139 !INFO [CamInitParam][0296]set camera fps: -1,width: 1920,height: 1080

  mipi mclk is not configed.
  Searching camera sensor on device: /proc/device-tree/soc/vcon@0 i2c bus: 1 mipi rx phy: 0
  WARN: Sensor Name: ar0820std-30fps, Expected Chip ID: 0xCB34, Actual Chip ID Read: 0x00
  [0] INFO: Found sensor name:imx219-30fps on mipi rx csi 0, i2c addr 0x10, config_file:linear_1920x1080_raw10_30fps_1lane.c
  2025/06/04 22:24:22.140 !INFO [CamInitPymParam][0259]Setting PYM channel:0: crop_x:0, crop_y:0, input_width:1920, input_height:1080, dst_w:1920, dst_h:1080
  capture time :0
  temp_ptr.data_size[0]:2592000
  ... omitted ...
  capture time :9
  temp_ptr.data_size[0]:2592000
  ```

## Camera Image Capture and Encoding

The `vio2encoder` example captures images from a `MIPI` camera, encodes them, and saves the encoded video locally.

Example flow diagram:

![image-vio_to_encoder](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/04_multi_media/image/cdev_demo/s100/image-vio_to_encode.png)

- **Environment Setup:**

  - With the development board powered off, connect the `MIPI` camera to the board. Currently, this sample only supports MIPI sensors: IMX219, SC230AI.
  - Power on the development board and log in via the command line.

- **Execution Method:** Run the program using the following commands.  
  The example code is provided in source form and must be compiled using the `make` command before running. Follow these steps:

  ```bash
  sunrise@ubuntu:~$ cd /app/cdev_demo/vio2encoder
  sunrise@ubuntu:/app/cdev_demo/vio2encoder$ make
  sunrise@ubuntu:/app/cdev_demo/vio2encoder$ ./vio2encoder -w 1920 -h 1080 --iwidth 1920 --iheight 1080 -o stream.h264
  ```

  Parameter descriptions:

  - `-w`: Encoded video width  
  - `-h`: Encoded video height  
  - `--iwidth`: Sensor output width  
  - `--iheight`: Sensor output height  
  - `-o`: Output path for the encoded video  

- **Expected Result:**
  After successful execution, a video file named `stream.h264` is generated in the current directory. Sample log output:

  ```bash
  sunrise@ubuntu:/app/cdev_demo/vio2encoder$  ./vio2encoder -w 1920 -h 1080 --iwidth 1920 --iheight 1080 -o stream.h264
  [UCP]: log level = 3
  [UCP]: UCP version = 3.7.3
  [VP]: log level = 3
  [DNN]: log level = 3
  [HPL]: log level = 3
  [UCPT]: log level = 6
  2025/06/16 11:04:37.628 !INFO [CamInitParam][0295]set camera fps: -1,width: 1920,height: 1080

  mipi mclk is not configed.
  Searching camera sensor on device: /proc/device-tree/soc/vcon@0 i2c bus: 1 mipi rx phy: 0
  WARN: Sensor Name: ar0820std-30fps, Expected Chip ID: 0xCB34, Actual Chip ID Read: 0x00
  [0] INFO: Found sensor name:imx219-30fps on mipi rx csi 0, i2c addr 0x10, config_file:linear_1920x1080_raw10_30fps_1lane.c
  2025/06/16 11:04:37.629 !INFO [CamInitPymParam][0258]Setting PYM channel:0: crop_x:0, crop_y:0, input_width:1920, input_height:1080, dst_w:1920, dst_h:1080
  sp_open_camera success!
  2025/06/16 11:04:37.770 !INFO [vp_encode_config_param][0408]codec type is h264: frame size:3110912  frame rate: 30
  sp_start_encode success!
  sp_module_bind(vio -> encoder) success!
  ^C
  recv:2,Stoping...
  ```

## Video File Decoding and Display

The `decoder2display` example decodes a video file and outputs it via the `HDMI` interface, allowing users to preview the video on a monitor.

Example flow diagram:

![image-decoder_to_display](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/04_multi_media/image/cdev_demo/s100/image-decoder_to_display.png)

- **Environment Setup:**

  - Connect the development board to a monitor using an HDMI cable.
  - Power on the development board and log in via the command line.
  - Prepare an encoded video file `stream.h264` as input.

- **Execution Method:**
  The example code is provided in source form and must be compiled using the `make` command before running. Follow these steps:

  ```bash
  sunrise@ubuntu:~$ cd /app/cdev_demo/decode2display
  sunrise@ubuntu:/app/cdev_demo/decode2display$ make
  sunrise@ubuntu:/app/cdev_demo/decode2display$ ./decoder2display -w 1920 -h 1080 -i /app/res/assets/1080P_test.h264
  ```

  Parameter descriptions:

  - `-h`: Height of the video file  
  - `-w`: Width of the video file  
  - `-i`: Path to the video file  

- **Expected Result:**After the program runs correctly, the video画面 will be output through the development board's `HDMI` interface, and users can preview the video画面 on a monitor. The runtime log is as follows:
```bash
sunrise@ubuntu:/app/cdev_demo/decode2display$ ./decoder2display -w 1920 -h 1080 -i /app/res/assets/1080P_test.h264
[UCP]: log level = 3
[UCP]: UCP version = 3.7.3
[VP]: log level = 3
[DNN]: log level = 3
[HPL]: log level = 3
[UCPT]: log level = 6
disp_w=1920, disp_h=1080
sp_start_decode success!
2025/06/16 10:48:49.220 !WARN [sp_start_display][0049]Warning: Using vot_chn values 0-3 is deprecated. Defaulting to HDMI mode.
2025/06/16 10:48:49.221 !WARN [sp_start_display][0050]Please use the new method: pass 10 for DisplayPort (DP) or 11 for HDMI.
2025/06/16 10:48:49.221 !INFO [OpenDisplay][0111]Wayland is available, using Wayland for rendering.

Using default socket path: /run/user/1000/wayland-0
sp_start_display success!
```

## RTSP Stream Pulling and Decoding

This example, `rtsp2display`, implements the functionality of pulling an `rtsp` stream, decoding it, and outputting the video画面 via `HDMI`, allowing users to preview the画面 on a monitor.

Example workflow diagram:

![rtsp2display](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/04_multi_media/image/cdev_demo/s100/image-rtsp_to_display.png)

- **Environment Setup:**

  - Connect the development board to a monitor using an HDMI cable.
  - Power on the development board and log in via command line.
  - Prepare an `rtsp` stream as the input source using the pre-installed streaming service. This service converts the `1080P_test.h264` video file into an RTSP stream with the URL `rtsp://127.0.0.1/assets/1080P_test.h264`. Users can start the streaming service with the following commands:

    ```text
    cd /app/res
    sunrise@ubuntu:/app/res# sudo chmod +x live555MediaServer
    sunrise@ubuntu:/app/res# sudo ./live555MediaServer &
    ```

- **How to Run:**
  The example code is provided in source form and must be compiled using the `make` command before execution. Steps are as follows:

  ```bash
  sunrise@ubuntu:~$ cd /app/cdev_demo/rtsp2display
  sunrise@ubuntu:/app/cdev_demo/rtsp2display$ make # Some warning messages may appear; they can be ignored.
  sunrise@ubuntu:/app/cdev_demo/rtsp2display$ ./rtsp2display -i rtsp://127.0.0.1/assets/1080P_test.h264 -t tcp
  ```

  Parameter configuration:

  - `-i`: Stream URL
  - `-t`: Transport type, options are `tcp` / `udp`

- **Expected Result:**
  After the program runs correctly, the video画面 will be output through the development board's `HDMI` interface, and users can preview the video画面 on a monitor. The runtime log is as follows:

  ```
  sunrise@ubuntu:/app/cdev_demo/rtsp2display$ ./rtsp2display -i rtsp://127.0.0.1/assets/1080P_test.h264 -t tcp
  [UCP]: log level = 3
  [UCP]: UCP version = 3.7.3
  [VP]: log level = 3
  [DNN]: log level = 3
  [HPL]: log level = 3
  [UCPT]: log level = 6
  avformat_open_input ok!
  avformat_find_stream_info ok!
  Input #0, rtsp, from 'rtsp://127.0.0.1/assets/1080P_test.h264':
    Metadata:
      title           : H.264 Video, streamed by the LIVE555 Media Server
      comment         : 1080P_test.h264
    Duration: N/A, start: 0.040000, bitrate: N/A
    Stream #0:0: Video: h264 (High), yuv420p(progressive), 1920x1080 [SAR 1:1 DAR 16:9], 25 fps, 25 tbr, 90k tbn, 50 tbc
  av_dump_format ok!
  rtsp_w:1920,rtsp_h:1080
  display_w:1920,dispaly_h:1080
  2025/06/16 10:57:21.724 !WARN [sp_start_display][0049]Warning: Using vot_chn values 0-3 is deprecated. Defaulting to HDMI mode.
  2025/06/16 10:57:21.724 !WARN [sp_start_display][0050]Please use the new method: pass 10 for DisplayPort (DP) or 11 for HDMI.
  2025/06/16 10:57:21.724 !INFO [OpenDisplay][0111]Wayland is available, using Wayland for rendering.

  Using default socket path: /run/user/1000/wayland-0
  2025/06/16 10:57:21.793 !INFO [CamInitPymParam][0258]Setting PYM channel:0: crop_x:0, crop_y:0, input_width:1920, input_height:1080, dst_w:1920, dst_h:1080
  sp_open_vps success!
  Could not read frame ---(error 'End of file')
  ```

- **Notes:**
  - When using UDP for stream transmission, screen corruption may occur due to packet loss; switching to TCP transmission can resolve this issue.
  - If the above command results in a "Connection refused" error, the `127.0.0.1` part may need to include the actual port number printed when `live555MediaServer` starts. For example:
    ```shell
    # final output of live555MediaServer
    ...
    (We use port 8000 for optional RTSP-over-HTTP tunneling, or for HTTP live streaming (for indexed Transport Stream files only).)
    ...

    # rtsp2display actual command
    sunrise@ubuntu:/app/cdev_demo/rtsp2display$ ./rtsp2display -i rtsp://127.0.0.1:8000/assets/1080P_test.h264 -t tcp
    ```

## VPS Scaling Example

This example implements video scaling based on the Video Processing Subsystem (`VPS`) module, capable of scaling down a single frame extracted from a video file or scaling a specified image.

- **Environment Setup:**

  - Power on the development board and log in via command line.
  - Prepare an image (NV12) or video file (H264) as input.

- **How to Run:**
  The example code is provided in source form and must be compiled using the `make` command before execution. Steps are as follows:

  ```bash
  sunrise@ubuntu:~$ cd /app/cdev_demo/vps
  sunrise@ubuntu:/app/cdev_demo/vps$ make
  sunrise@ubuntu:/app/cdev_demo/vps$ ./vps -m 1 -i input_1080p.h264 -o output1.yuv --iheight 1080 --iwidth 1920 --oheight 720 --owidth 1280
  sunrise@ubuntu:/app/cdev_demo/vps$ ./vps -m 2 -i input_1080p.yuv -o output.yuv --iheight 1080 --iwidth 1920 --oheight 720 --owidth 1280
  ```

  **Parameter Configuration:**

  - `-i`: Path to the input file
  - `--iheight`: Input height
  - `--iwidth`: Input width
  - `-m`: Input mode: `1` for video stream; `2` for NV12 image
  - `-o`: Output path
  - `--oheight`: Output height
  - `--owidth`: Output width
  - `--skip`: (Optional) For video stream input, skip the specified number of initial frames.

- **Expected Result:**
  After the program runs correctly, the processed image file `output.yuv` will be saved in the current directory. The runtime log is as follows:

  ```shell
  sunrise@ubuntu:/app/cdev_demo/vps$ ./vps -m 1 -i input_1080p.h264 -o output1.yuv --iheight 1080 --iwidth 1920 --oheight 720 --owidth 1280
  [UCP]: log level = 3
  [UCP]: UCP version = 3.7.3
  [VP]: log level = 3
  [DNN]: log level = 3
  [HPL]: log level = 3
  [UCPT]: log level = 6
  2025/06/16 10:33:02.709 !INFO [CamInitPymParam][0258]Setting PYM channel:0: crop_x:0, crop_y:0, input_width:1920, input_height:1080, dst_w:1280, dst_h:720
  sunrise@ubuntu:/app/cdev_demo/vps$ ./vps -m 2 -i input_1080p.yuv -o output.yuv --iheight 1080 --iwidth 1920 --oheight 720 --owidth 1280
  [UCP]: log level = 3
  [UCP]: UCP version = 3.7.3
  [VP]: log level = 3
  [DNN]: log level = 3
  [HPL]: log level = 3
  [UCPT]: log level = 6
  2025/06/16 10:33:29.134 !INFO [CamInitPymParam][0258]Setting PYM channel:0: crop_x:0, crop_y:0, input_width:1920, input_height:1080, dst_w:1280, dst_h:720
  ... omitted ...
  ```

<!--
## Object Detection Algorithm — FCOS

This example implements an object detection algorithm for local video streams based on the `fcos` model, allowing users to preview detection results on a monitor.

- **Environment Setup:**

  - Connect the development board to a monitor using an HDMI cable.
  - Power on the development board and log in via command line.
  - Prepare a video file (H264) as input.

- **How to Run:**
  The example code is provided in source form and must be compiled using the `make` command before execution. Steps are as follows:

  ```bash
  sunrise@ubuntu:~$ cd /app/cdev_demo/bpu/src
  sunrise@ubuntu:/app/cdev_demo/bpu/src$ sudo make
  sunrise@ubuntu:/app/cdev_demo/bpu/src$ cd bin
  sunrise@ubuntu:/app/cdev_demo/bpu/src/bin$ sudo ./sample -f /app/model/basic/fcos_512x512_nv12.bin -m 1 -i 1080p_.h264 -w 1920 -h 1080
  ```

  **Parameter Configuration:**

  - `-f`: Path to the model file
  - `-h`: Height of the input video
  - `-w`: Width of the input video
  - `-i`: Path to the input video
  - `-m`: Model type, default is `1`

- **Expected Result:**
  After the program runs correctly, the video along with the algorithm-rendered detection results will be output via the `HDMI` interface, and users can preview it on a monitor. The runtime log is as follows:

  ```bash
  sunrise@ubuntu:/app/cdev_demo/bpu/src/bin$ sudo ./sample -f /app/model/basic/fcos_512x512_nv12.bin -m 1 -i 1080p_.h264 -w 1920 -h 1080
  [BPU_PLAT]BPU Platform Version(1.3.1)!
  [HBRT] set log level as 0. version = 3.14.5
  [DNN] Runtime version = 1.9.7_(3.14.5 HBRT)
  Model info:
  model_name: fcos_512x512_nv12Input count: 1input[0]: tensorLayout: 2 tensorType: 1 validShape:(1, 3, 512, 512, ), alignedShape:(1, 3, 512, 512, )
  Output count: 15Output[0]: tensorLayout: 0 tensorType: 13 validShape:(1, 64, 64, 80, ), alignedShape:(1, 64, 64, 80, )
  Output[1]: tensorLayout: 0 tensorType: 13 validShape:(1, 32, 32, 80, ), alignedShape:(1, 32, 32, 80, )
  Output[2]: tensorLayout: 0 tensorType: 13 validShape:(1, 16, 16, 80, ), alignedShape:(1, 16, 16, 80, )
  Output[3]: tensorLayout: 0 tensorType: 13 validShape:(1, 8, 8, 80, ), alignedShape:(1, 8, 8, 80, )
  Output[4]: tensorLayout: 0 tensorType: 13 validShape:(1, 4, 4, 80, ), alignedShape:(1, 4, 4, 80, )
  Output[5]: tensorLayout: 0 tensorType: 13 validShape:(1, 64, 64, 4, ), alignedShape:(1, 64, 64, 4, )
  Output[6]: tensorLayout: 0 tensorType: 13 validShape:(1, 32, 32, 4, ), alignedShape:(1, 32, 32, 4, )
  Output[7]: tensorLayout: 0 tensorType: 13 validShape:(1, 16, 16, 4, ), alignedShape:(1, 16, 16, 4, )
  Output[8]: tensorLayout: 0 tensorType: 13 validShape:(1, 8, 8, 4, ), alignedShape:(1, 8, 8, 4, )
  Output[9]: tensorLayout: 0 tensorType: 13 validShape:(1, 4, 4, 4, ), alignedShape:(1, 4, 4, 4, )
``````plaintext
  Output[10]: tensorLayout: 0 tensorType: 13 validShape:(1, 64, 64, 1, ), alignedShape:(1, 64, 64, 1, )
  Output[11]: tensorLayout: 0 tensorType: 13 validShape:(1, 32, 32, 1, ), alignedShape:(1, 32, 32, 1, )
  Output[12]: tensorLayout: 0 tensorType: 13 validShape:(1, 16, 16, 1, ), alignedShape:(1, 16, 16, 1, )
  Output[13]: tensorLayout: 0 tensorType: 13 validShape:(1, 8, 8, 1, ), alignedShape:(1, 8, 8, 1, )
  Output[14]: tensorLayout: 0 tensorType: 13 validShape:(1, 4, 4, 1, ), alignedShape:(1, 4, 4, 1, )
  libiar: hb_disp_set_timing done!
  dispaly init ret = 0
  vps open ret = 0
  module bind vps & display ret = 0
  display start ret = 0
  [x3_av_open_stream]:[380]:probesize: 5000000
  decode start ret = 0
  module bind decoder & vps ret = 0
  [ERROR]["vdec"][video/src/vdec_group.c:348] [8870.450264]vdec_channel_bump_thread[348]: VDEC_MODULE module try again

  [draw_rect]:[137]:========point is 0,return========
  fps:55.555556,processing time:18
  ```

## Object Detection Algorithm — YOLOv5s

This example implements a camera-based object detection algorithm using the `YOLOv5` model. Users can preview the detection results on a display.

- **Environment Setup:**

  - With the development board powered off, connect the `MIPI` camera to the board.
  - Connect the development board to a monitor using an HDMI cable.
  - Power on the development board and log in via the command line.

- **Execution Method:**
  The sample code is provided in source form and must be compiled using the `make` command before running. Follow these steps:

  ```bash
  sunrise@ubuntu:~$ cd /app/cdev_demo/bpu/src
  sunrise@ubuntu:/app/cdev_demo/bpu/src$ sudo make
  sunrise@ubuntu:/app/cdev_demo/bpu/src$ cd bin
  sunrise@ubuntu:/app/cdev_demo/bpu/src/bin$ sudo ./sample -f /app/model/basic/yolov5s_672x672_nv12.bin -m 0
  ```

  **Parameter Configuration:**

  - `-f`: Path to the model file  
  - `-m`: Model type (default is 0)

- **Expected Result:**
  After successful execution, the program outputs video frames along with algorithm-rendered detection results through the `HDMI` interface, which can be previewed on the connected display. Sample runtime logs are shown below:

  ```bash
  sunrise@ubuntu:/app/cdev_demo/bpu/src/bin$ sudo ./sample -f /app/model/basic/yolov5s_672x672_nv12.bin -m 0
  [BPU_PLAT]BPU Platform Version(1.3.6)!
  [HBRT] set log level as 0. version = 3.15.49.0
  [DNN] Runtime version = 1.23.8_(3.15.49 HBRT)
  [A][DNN][packed_model.cpp:247][Model](2000-01-01,19:06:39.821.214) [HorizonRT] The model builder version = 1.23.5
  [W][DNN]bpu_model_info.cpp:491][Version](2000-01-01,19:06:39.876.293) Model: yolov5s_v2_672x672_bayese_nv12. Inconsistency between the hbrt library version 3.15.49.0 and the model build version 3.15.47.0 detected, in order to ensure correct model results, it is recommended to use compilation tools and the BPU SDK from the same OpenExplorer package.
  Model info:
  model_name: yolov5s_v2_672x672_bayese_nv12Input count: 1input[0]: tensorLayout: 2 tensorType: 1 validShape:(1, 3, 672, 672, ), alignedShape:(1, 3, 672, 672, )
  Output count: 3Output[0]: tensorLayout: 0 tensorType: 13 validShape:(1, 84, 84, 255, ), alignedShape:(1, 84, 84, 255, )
  Output[1]: tensorLayout: 0 tensorType: 13 validShape:(1, 42, 42, 255, ), alignedShape:(1, 42, 42, 255, )
  Output[2]: tensorLayout: 0 tensorType: 13 validShape:(1, 21, 21, 255, ), alignedShape:(1, 21, 21, 255, )
  [INFO] board_id is 301, not need skip sci1.
  Searching camera sensor on device: /proc/device-tree/soc/cam/vcon@0 i2c bus: 6 mipi rx phy: 0
  INFO: Found sensor name:ov5647 on mipi rx csi 0, i2c addr 0x36, config_file:linear_1920x1080_raw10_30fps_2lane.c
  2000/01/01 19:06:40.012 !INFO [CamInitParam][0139]Setting VSE channel-0: input_width:1920, input_height:1080, dst_w:672, dst_h:672
  2000/01/01 19:06:40.013 !INFO [CamInitParam][0139]Setting VSE channel-1: input_width:1920, input_height:1080, dst_w:1920, dst_h:1080
  ... omitted ...
  ```-->