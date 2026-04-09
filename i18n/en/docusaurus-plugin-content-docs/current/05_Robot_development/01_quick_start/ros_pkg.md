---
sidebar_position: 5
---
# 5.1.5 Using ROS2 package

```mdx-code-block
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';
```

Prerequisite: TogetheROS.Bot has been successfully installed.

tros.b is fully compatible with ROS2 Foxy/Humble APIs and can reuse the rich set of ROS2 tools. Here, we demonstrate how to use ROS packages in tros.b by installing and using the ROS2 `image_transport` package as an example.

## Installing ROS2 Packages

### 1 Add ROS apt Repository

When installing tros.b, the ROS apt repository has already been added automatically—no manual addition is required.

Update the apt repository:

```shell
sudo apt update
```

:::caution **Note**
**If the `sudo apt update` command fails or returns an error, please refer to the FAQ section [Common Issues](../../08_FAQ/01_hardware_and_system.md), specifically `Q10: How to handle failures or errors when running apt update?` for solutions.**
:::

### 2 Install Packages

<Tabs groupId="tros-distro">
<TabItem value="foxy" label="Foxy">

```bash
sudo apt install ros-foxy-image-transport
sudo apt install ros-foxy-image-transport-plugins
```

</TabItem>
<TabItem value="humble" label="Humble">

```bash
sudo apt install ros-humble-image-transport
sudo apt install ros-humble-image-transport-plugins
```

</TabItem>
</Tabs>

## Using ROS2 Packages

Usage is identical to standard ROS2:

<Tabs groupId="tros-distro">
<TabItem value="foxy" label="Foxy">

```bash
source /opt/tros/setup.bash
ros2 run image_transport list_transports
```

</TabItem>
<TabItem value="humble" label="Humble">

```bash
source /opt/tros/humble/setup.bash
ros2 run image_transport list_transports
```

</TabItem>
</Tabs>

The output below shows the image formats supported by the `image_transport` package:

```shell
root@ubuntu:/opt/tros# ros2 run image_transport list_transports
Declared transports:
image_transport/compressed
image_transport/compressedDepth
image_transport/raw
image_transport/theora

Details:
----------
"image_transport/compressed"
 - Provided by package: compressed_image_transport
 - Publisher:
      This plugin publishes a CompressedImage using either JPEG or PNG compression.

 - Subscriber:
      This plugin decompresses a CompressedImage topic.

----------
"image_transport/compressedDepth"
 - Provided by package: compressed_depth_image_transport
 - Publisher:
      This plugin publishes a compressed depth images using PNG compression.

 - Subscriber:
      This plugin decodes a compressed depth images.

----------
"image_transport/raw"
 - Provided by package: image_transport
 - Publisher:
      This is the default publisher. It publishes the Image as-is on the base topic.

 - Subscriber:
      This is the default pass-through subscriber for topics of type sensor_msgs/Image.

----------
"image_transport/theora"
 - Provided by package: theora_image_transport
 - Publisher:
      This plugin publishes a video packet stream encoded using Theora.

 - Subscriber:
      This plugin decodes a video packet stream encoded using Theora.
```