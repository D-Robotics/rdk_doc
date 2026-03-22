---
sidebar_position: 3
---

# 5.2.3 Image Codec

```mdx-code-block
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';
```

## Feature Overview

The image codec functionality is similar to the ROS `image_transport` package. RDK leverages hardware acceleration units to convert between MJPEG/H.264/H.265 and BGR8/RGB8/NV12 formats, significantly reducing CPU usage while improving conversion efficiency. On x86 platforms, only conversion between MJPEG and BGR8/RGB8/NV12 formats is supported.

Code repository: (https://github.com/D-Robotics/hobot_codec)

## Supported Platforms

| Platform | Runtime Environment | Example Functionality |
| ------- | ------------ | ------------------------------ |
| RDK X3, RDK X3 Module | Ubuntu 20.04 (Foxy), Ubuntu 22.04 (Humble) | Launch a MIPI camera to capture images, encode them, and display via web browser |
| RDK X5, RDK X5 Module, RDK S100 | Ubuntu 22.04 (Humble) | Launch a MIPI camera to capture images, encode them, and display via web browser |
| RDK Ultra | Ubuntu 20.04 (Foxy) | Launch a MIPI camera to capture images, encode them, and display via web browser |
| x86 | Ubuntu 20.04 (Foxy) | Publish YUV images using an image publisher tool, encode them, and display via web browser |

***RDK Ultra does not support H.264 video encoding format.***

## Prerequisites

### RDK Platform

1. RDK has been flashed with Ubuntu 20.04 or Ubuntu 22.04 system image.

2. TogetherROS.Bot has been successfully installed on RDK.

3. RDK is connected to an F37 camera or another MIPI camera.

### x86 Platform

1. The x86 environment has been set up with Ubuntu 20.04 system image.

2. The x86 version of tros.b has been installed in the x86 environment.

## Usage Instructions

Below, we use JPEG encoding as an example to demonstrate how to obtain NV12-format image data from either a camera or an image publisher tool, compress it into JPEG format, and preview it via a web browser on a PC.

### RDK Platform

1. Obtain YUV data and start JPEG encoding:

    Log in to the RDK via SSH, use `mipi_cam` as the data source, configure `hobot_codec` input as NV12 format and output as JPEG format. You may replace `mipi_cam` with your actual sensor model.

    a. Launch `mipi_cam`

    <Tabs groupId="tros-distro">
    <TabItem value="foxy" label="Foxy">

    ```bash
    # Set up tros.b environment
    source /opt/tros/setup.bash
    ```

    </TabItem>

    <TabItem value="humble" label="Humble">

    ```bash
    # Set up tros.b environment
    source /opt/tros/humble/setup.bash
    ```

    </TabItem>

    </Tabs>

    ```shell
    ros2 launch mipi_cam mipi_cam.launch.py mipi_video_device:=F37
    ```

    b. Launch `hobot_codec` for encoding

    <Tabs groupId="tros-distro">
    <TabItem value="foxy" label="Foxy">

    ```bash
    # Set up tros.b environment
    source /opt/tros/setup.bash
    ```

    </TabItem>

    <TabItem value="humble" label="Humble">

    ```bash
    # Set up tros.b environment
    source /opt/tros/humble/setup.bash
    ```

    </TabItem>

    </Tabs>

    ```shell
    ros2 launch hobot_codec hobot_codec.launch.py codec_in_mode:=shared_mem codec_in_format:=nv12 codec_out_mode:=ros codec_out_format:=jpeg codec_sub_topic:=/hbmem_img codec_pub_topic:=/image_jpeg
    ```

2. View the JPEG-encoded image in a web browser. Open another terminal:

    <Tabs groupId="tros-distro">
    <TabItem value="foxy" label="Foxy">

    ```bash
    # Set up tros.b environment
    source /opt/tros/setup.bash
    ```

    </TabItem>

    <TabItem value="humble" label="Humble">

    ```bash
    # Set up tros.b environment
    source /opt/tros/humble/setup.bash
    ```

    </TabItem>

    </Tabs>

    ```shell
    ros2 launch websocket websocket.launch.py websocket_image_topic:=/image_jpeg websocket_only_show_image:=true
    ```

3. On your PC, open a browser (Chrome/Firefox/Edge) and navigate to `http://IP:8000`, where IP is the RDK/x86 device’s IP address. Click the top-left button on the web page to view the real-time JPEG-encoded video stream.

    ![web-f37-codec](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/02_quick_demo/image/hobot_codec/web-f37-codec.png)

### x86 Platform

1. Obtain YUV data and start JPEG encoding:

    a. Launch the image publisher node

    <Tabs groupId="tros-distro">
    <TabItem value="foxy" label="Foxy">

    ```bash
    # Set up tros.b environment
    source /opt/tros/setup.bash
    ```

    </TabItem>

    <TabItem value="humble" label="Humble">

    ```bash
    # Set up tros.b environment
    source /opt/tros/humble/setup.bash
    ```

    </TabItem>

    </Tabs>

    ```shell
    // Copy required image files for the demo from tros.b installation path
    cp -r /opt/tros/${TROS_DISTRO}/lib/hobot_image_publisher/config/ .

    // Launch the image publisher node
    
    ros2 launch hobot_image_publisher hobot_image_publisher.launch.py publish_output_image_w:=960 publish_output_image_h:=544 publish_message_topic_name:=/hbmem_img publish_fps:=20 
    ```

    b. Launch the JPEG image encoding & publishing package

    <Tabs groupId="tros-distro">
    <TabItem value="foxy" label="Foxy">

    ```bash
    # Set up tros.b environment
    source /opt/tros/setup.bash
    ```

    </TabItem>

    <TabItem value="humble" label="Humble">

    ```bash
    # Set up tros.b environment
    source /opt/tros/humble/setup.bash
    ```

    </TabItem>

    </Tabs>

    ```shell
    ros2 launch hobot_codec hobot_codec.launch.py codec_in_mode:=shared_mem codec_in_format:=nv12 codec_out_mode:=ros codec_out_format:=jpeg codec_sub_topic:=/hbmem_img codec_pub_topic:=/image_jpeg
    ```

2. View JPEG-encoded images on the web client. Open another terminal:

        <Tabs groupId="tros-distro">
        <TabItem value="foxy" label="Foxy">

        ```bash
        # Set up the tros.b environment
        source /opt/tros/setup.bash
        ```

        </TabItem>

        <TabItem value="humble" label="Humble">

            ```bash
            # Set up the tros.b environment
            source /opt/tros/humble/setup.bash
            ```

        </TabItem>

        </Tabs>

        ```shell
        ros2 launch websocket websocket.launch.py websocket_image_topic:=/image_jpeg websocket_only_show_image:=true
        ```

3. On your PC, open a browser (Chrome/Firefox/Edge) and enter `http://IP:8000`, where IP is the IP address of your RDK/X86 device. Click the "Web Display" button in the upper-left corner to view the real-time JPEG-encoded video stream.

## Notes

If you encounter issues with the Hobot codec node failing to start, follow these steps for troubleshooting:

1. Verify that the tros.b environment has been properly set up.
2. Check whether the parameters are correct. For details, refer to the Hobot_codec [README.md](https://github.com/D-Robotics/hobot_codec).
```