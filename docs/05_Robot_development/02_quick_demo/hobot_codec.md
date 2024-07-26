---
sidebar_position: 3
---

# 2.3 图像编解码

```mdx-code-block
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';
```

## 功能介绍

图像编解码功能与ROS image_transport package类似，地平线RDK采用硬件单元加速MJPEG/H264/H265与BGR8/RGB8/NV12格式之间转换，可以大幅降低CPU占用的同时提升格式转换效率，X86平台仅支持MJPEG与BGR8/RGB8/NV12格式之间的转换。

代码仓库：(https://github.com/D-Robotics/hobot_codec)

## 支持平台

| 平台    | 运行方式     | 示例功能                       |
| ------- | ------------ | ------------------------------ |
| RDK X3, RDK X3 Module | Ubuntu 20.04 (Foxy), Ubuntu 22.04 (Humble) | 启动MIPI摄像头获取图像，然后进行图像编码，最后通过Web展示 |
| RDK Ultra | Ubuntu 20.04 (Foxy) | 启动MIPI摄像头获取图像，然后进行图像编码，最后通过Web展示 |
| X86     | Ubuntu 20.04 (Foxy) | 使用图像发布工具发布YUV图像，然后进行图像编码，最后通过Web展示 |

***RDK Ultra不支持H.264视频编码格式。***

## 准备工作

### 地平线RDK平台

1. 地平线RDK已烧录好地平线提供的Ubuntu 20.04/Ubuntu 22.04系统镜像。

2. 地平线RDK已成功安装TogetheROS.Bot。

3. 地平线RDK已连接摄像头F37或其他MIPI摄像头。

### X86平台

1. X86环境已配置Ubuntu 20.04系统镜像。

2. X86环境已安装X86版本tros.b。

## 使用方式

下面以 JPEG 编码为例，介绍从摄像头或图像发布工具获取NV12格式图片数据，经过JPEG压缩编码后，实现在PC的Web端预览图片。

### 地平线RDK平台

1. 获取YUV数据，并启动JPGE编码：

    通过SSH登录地平线RDK，使用mipi_cam作为数据来源，配置hobot_codec输入为NV12格式，输出为JPEG格式，可修改mipi_cam为实际使用的sensor型号。

    a. 启动mipi_cam

    <Tabs groupId="tros-distro">
    <TabItem value="foxy" label="Foxy">

    ```bash
    # 配置tros.b环境
    source /opt/tros/setup.bash
    ```

    </TabItem>

    <TabItem value="humble" label="Humble">

    ```bash
    # 配置tros.b环境
    source /opt/tros/humble/setup.bash
    ```

    </TabItem>

    </Tabs>

    ```shell
    ros2 launch mipi_cam mipi_cam.launch.py mipi_video_device:=F37
    ```

    b. 启动hobot_codec编码

    <Tabs groupId="tros-distro">
    <TabItem value="foxy" label="Foxy">

    ```bash
    # 配置tros.b环境
    source /opt/tros/setup.bash
    ```

    </TabItem>

    <TabItem value="humble" label="Humble">

    ```bash
    # 配置tros.b环境
    source /opt/tros/humble/setup.bash
    ```

    </TabItem>

    </Tabs>

    ```shell
    ros2 launch hobot_codec hobot_codec.launch.py codec_in_mode:=shared_mem codec_in_format:=nv12 codec_out_mode:=ros codec_out_format:=jpeg codec_sub_topic:=/hbmem_img codec_pub_topic:=/image_jpeg
    ```

2. Web端查看JPEG编码图像，另起一个终端：

    ```shell
    source /opt/tros/setup.bash
    ros2 launch websocket websocket.launch.py websocket_image_topic:=/image_jpeg websocket_only_show_image:=true
    ```

3. PC打开浏览器（chrome/firefox/edge）输入 `http://IP:8000`，IP为地平线RDK/X86设备IP地址，点击左上方Web端展示即可查看JPEG编码的实时画面

    ![web-f37-codec](/../static/img/05_Robot_development/02_quick_demo/image/hobot_codec/web-f37-codec.png "实时图像")

### X86平台

1. 获取YUV数据，并启动JPGE编码：

    a. 启动图像发布节点

    <Tabs groupId="tros-distro">
    <TabItem value="foxy" label="Foxy">

    ```bash
    # 配置tros.b环境
    source /opt/tros/setup.bash
    ```

    </TabItem>

    <TabItem value="humble" label="Humble">

    ```bash
    # 配置tros.b环境
    source /opt/tros/humble/setup.bash
    ```

    </TabItem>

    </Tabs>

    ```shell
    //从tros.b的安装路径中拷贝出运行示例需要的图片文件
    cp -r /opt/tros/${TROS_DISTRO}/lib/hobot_image_publisher/config/ .

    // 启动图像发布节点
    
    ros2 launch hobot_image_publisher hobot_image_publisher.launch.py publish_output_image_w:=960 publish_output_image_h:=544 publish_message_topic_name:=/hbmem_img publish_fps:=20 
    ```

    b. 启动JPEG图片编码&发布pkg

    <Tabs groupId="tros-distro">
    <TabItem value="foxy" label="Foxy">

    ```bash
    # 配置tros.b环境
    source /opt/tros/setup.bash
    ```

    </TabItem>

    <TabItem value="humble" label="Humble">

    ```bash
    # 配置tros.b环境
    source /opt/tros/humble/setup.bash
    ```

    </TabItem>

    </Tabs>

    ```shell
    ros2 launch hobot_codec hobot_codec.launch.py codec_in_mode:=shared_mem codec_in_format:=nv12 codec_out_mode:=ros codec_out_format:=jpeg codec_sub_topic:=/hbmem_img codec_pub_topic:=/image_jpeg
    ```

2. Web端查看JPEG编码图像，另起一个终端：

    <Tabs groupId="tros-distro">
    <TabItem value="foxy" label="Foxy">

    ```bash
    # 配置tros.b环境
    source /opt/tros/setup.bash
    ```

    </TabItem>

    <TabItem value="humble" label="Humble">

    ```bash
    # 配置tros.b环境
    source /opt/tros/humble/setup.bash
    ```

    </TabItem>

    </Tabs>

    ```shell
    ros2 launch websocket websocket.launch.py websocket_image_topic:=/image_jpeg websocket_only_show_image:=true
    ```

3. PC打开浏览器（chrome/firefox/edge）输入 `http://IP:8000`，IP为地平线RDK/X86设备IP地址，点击左上方Web端展示即可查看JPEG编码的实时画面。

## 注意事项

如遇到 Hobot codec 节点启动异常，可通过下述步骤进行问题排查：

1. 是否设置 tros.b 环境
2. 参数是否正确，具体参考Hobot_codec README.md
