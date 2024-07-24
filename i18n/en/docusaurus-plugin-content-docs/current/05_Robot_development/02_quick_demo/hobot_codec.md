---
sidebar_position: 3
---
# 5.2.3 Image Codec

```mdx-code-block
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';
```

## Introduction

The image codec functionality is similar to the ROS image_transport package. The Horizon RDK utilizes hardware acceleration to convert between the MJPEG/H264/H265 and BGR8/RGB8/NV12 formats, which significantly reduces CPU usage while improving conversion efficiency.

Code repository:  `https://github.com/HorizonRDK/hobot_codec>

## Supported Platforms

| Platform                       | System | Function                                |
| ------------------------------ | ---------------- | --------------------------------------------------- |
| RDK X3, RDK X3 Module | Ubuntu 20.04 (Foxy), Ubuntu 22.04 (Humble)     | Start MIPI camera to capture images, encode them, and display them via Web |

## Preparation

### Horizon RDK

1. The Horizon RDK has been flashed with the  Ubuntu 20.04/22.04 system image provided by Horizon.

2. TogetheROS.Bot has been successfully installed on the Horizon RDK.

3. The Horizon RDK has been connected to a camera, such as the F37 or other MIPI cameras.

## Usage

Taking JPEG encoding as an example, this section explains how to obtain NV12 format image data from a camera or image publishing tool, compress and encode it as JPEG, and preview the image on a PC via web.

1. Obtain YUV data and start JPEG encoding:

    **Horizon RDK**

    Log in to the Horizon RDK via SSH and use `mipi_cam` as the data source. Configure `hobot_codec` to input NV12 format and output JPEG format. Modify `mipi_cam` to the actual sensor model being used.

    a. Start `mipi_cam`

    <Tabs groupId="tros-distro">
    <TabItem value="foxy" label="Foxy">

    ```bash
    # Configure the tros.b environment
    source /opt/tros/setup.bash
    ```

    </TabItem>

    <TabItem value="humble" label="Humble">

    ```bash
    # Configure the tros.b environment
    source /opt/tros/humble/setup.bash
    ```

    </TabItem>

    </Tabs>

    ```shell
    ros2 launch mipi_cam mipi_cam.launch.py mipi_video_device:=F37
    ```

    b. Launch the hobot_codec encoder

    <Tabs groupId="tros-distro">
    <TabItem value="foxy" label="Foxy">

    ```bash
    # Configure the tros.b environment
    source /opt/tros/setup.bash
    ```

    </TabItem>

    <TabItem value="humble" label="Humble">

    ```bash
    # Configure the tros.b environment
    source /opt/tros/humble/setup.bash
    ```

    </TabItem>

    </Tabs>

    ```shell
    ros2 launch hobot_codec hobot_codec.launch.py codec_in_mode:=shared_mem codec_in_format:=nv12 codec_out_mode:=ros codec_out_format:=jpeg codec_sub_topic:=/hbmem_img codec_pub_topic:=/image_jpeg
    ```

2. To view the JPEG encoded images on the web interface, open another terminal:

```shell
ros2 launch websocket websocket.launch.py websocket_image_topic:=/image_jpeg websocket_only_show_image:=true
```

3. Open a web browser (Chrome/Firefox/Edge) on your PC and enter  `http://IP:8000`. Replace the IP address of the Horizon RDK. Click on the Web at the top left to view the real-time JPEG encoded image.

 ![web-f37-codec](./image/hobot_codec/web-f37-codec.png "Real-time image")

## Important notes:
If you encounter an abnormal startup of the Hobot codec node, you can troubleshoot the problem by following these steps:

1. Check if the tros.b environment is set.
2. Verify if the parameters are correct, please refer to the Hobot_codec README.md for details.