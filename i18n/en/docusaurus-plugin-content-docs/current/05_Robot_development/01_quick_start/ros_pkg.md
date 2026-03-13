---
sidebar_position: 5
---
# 5.1.5 Using ROS2 package

```mdx-code-block
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';
```

Prerequisite: TogetheROS.Bot installed successfully

The interfaces of tros.b and the ROS2 Foxy/Humble version are fully compatible and can reuse the ROS2 rich tool package. Here we take the installation and use of the ROS2 foxy version ros-foxy-image-transport as an example to introduce how to use the ROS package in tros.b.

## Installing ROS2 package

### 1. Add ROS2 apt source

When installing tros.b, ROS2 apt source is automatically added, so there is no need to manually add it.

Update apt repository

```shell
sudo apt update
```

### 2. Install packages

```shell
sudo apt install ros-foxy-image-transport
sudo apt install ros-foxy-image-transport-plugins
```

## Using ROS2 package

Same as using ROS2

```shell
source /opt/tros/setup.bash
ros2 run image_transport list_transports
```

The running result is as follows, showing the supported image formats by image_transport package.

```shell
root@ubuntu:/opt/tros# ros2 run image_transport list_transports
Declared transports:
image_transport/compressed
image_transport/compressedDepth
image_transport/raw
image_transport/theora

Details:
----------
```"image_transport/compressed"
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