---
sidebar_position: 2
---

# 5.2.2 数据展示

```mdx-code-block
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';
```

## Web展示

### 功能介绍

Web展示用于预览摄像头图像（JPEG格式）和算法效果，通过网络将图像和算法结果传输到PC浏览器，然后进行渲染显示。该展示端还支持仅显示视频，而不渲染智能结果。

代码仓库：[https://github.com/D-Robotics/hobot_websocket](https://github.com/D-Robotics/hobot_websocket)

### 支持平台

| 平台    | 运行方式      | 示例功能                       |
| ------- | ------------- | ------------------------------ |
| RDK X3, RDK X3 Module | Ubuntu 20.04 (Foxy), Ubuntu 22.04 (Humble)  | 启动MIPI摄像头，并通过Web展示图像 |
| RDK X5, RDK X5 Module,RDK S100 | Ubuntu 22.04 (Humble)  | 启动MIPI摄像头，并通过Web展示图像 |
| RDK S600 | Ubuntu 24.04 (Jazzy) | 启动MIPI摄像头，并通过Web展示图像 |
| RDK Ultra | Ubuntu 20.04 (Foxy) | 启动MIPI摄像头，并通过Web展示图像 |
| X86     | Ubuntu 20.04 (Foxy) | 启动USB摄像头，并通过Web展示图像 |

### 准备工作

#### RDK平台

1. 确认摄像头F37正确接到RDK上

2. 确认PC可以通过网络访问RDK

3. 确认已成功安装TogetheROS.Bot

#### X86平台

1. 确认X86平台系统为Ubuntu 20.04，且已成功安装tros.b

2. 确认USB摄像头接入主机USB插口，并可正常识别

### 使用方式

#### RDK平台

1. 通过SSH登录RDK，启动板端相关程序

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
   <TabItem value="jazzy" label="Jazzy">

   ```bash
   # 配置tros.b环境
   source /opt/tros/jazzy/setup.bash
   ```

   </TabItem>

   </Tabs>

   ```bash
    ros2 launch mipi_cam mipi_cam.launch.py mipi_video_device:=F37
    ```

    b. 启动编码

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
   <TabItem value="jazzy" label="Jazzy">

   ```bash
   # 配置tros.b环境
   source /opt/tros/jazzy/setup.bash
   ```

   </TabItem>

   </Tabs>


   ```bash
   ros2 launch hobot_codec hobot_codec_encode.launch.py
   ```

    c. 启动websocket

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
   <TabItem value="jazzy" label="Jazzy">

   ```bash
   # 配置tros.b环境
   source /opt/tros/jazzy/setup.bash
   ```

   </TabItem>

   </Tabs>

   ```bash
   ros2 launch websocket websocket.launch.py websocket_image_topic:=/image_jpeg websocket_only_show_image:=true
   ```

2. PC浏览器（chrome/firefox/edge）输入 `http://IP:8000` ，即可查看图像，IP为RDK IP地址。

   ![websocket](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/02_quick_demo/image/demo_render/websocket.png)

#### X86平台

1. 启动hobot_usb_cam节点

   ```bash
   # 配置tros.b环境
   source /opt/tros/setup.bash
   # usb_video_device需要更改为实际usb摄像头video节点
   ros2 launch hobot_usb_cam hobot_usb_cam.launch.py usb_image_width:=1280 usb_image_height:=720 usb_video_device:=/dev/video0
   ```

2. 启动websocket节点

   ```bash
   # 配置tros.b环境
   source /opt/tros/setup.bash
   ros2 launch websocket websocket.launch.py websocket_image_topic:=/image websocket_only_show_image:=true
   ```

3. PC浏览器（chrome/firefox/edge）输入 `http://IP:8000` ，即可查看图像效果，IP为PC IP地址，若在本机访问，也可使用localhost。

### 注意事项

1. websocket需要使用8000端口，如果端口被占用，则会启动失败，解决方法如下：

   - 使用`lsof -i:8000`命令查看8000端口占用进程，使用`kill <PID>`关闭占用8000端口进程，然后重新启动websocket即可。

   - 若用户不想停止当前正在占用8000端口的服务，可以修改 `/opt/tros/${TROS_DISTRO}/lib/websocket/webservice/conf/nginx.conf` 配置文件中的`listen`端口号，改为大于1024且未使用的端口号。修改端口号后，浏览器端使用的URL也要同步修改。

## HDMI展示

### 功能介绍

本章节介绍通过HDMI展示camera nv12图像的使用，RDK通过HDMI接显示器即可显示实时图像效果，对应于hobot_hdmi package。

代码仓库：[https://github.com/D-Robotics/hobot_hdmi](https://github.com/D-Robotics/hobot_hdmi)

### 支持平台

| 平台     | 运行方式     | 示例功能                       |
| -------- | ------------ | ------------------------------ |
| RDK X3, RDK X3 Module | Ubuntu 20.04 (Foxy), Ubuntu 22.04 (Humble) | 启动MIPI摄像头，并通过HDMI展示图像 |
| RDK X5, RDK X5 Module | Ubuntu 22.04 (Humble) | 启动MIPI摄像头，并通过HDMI展示图像 |

:::caution **注意**
HDMI展示**EOL**说明：
- `RDK X3`和`RDK X3 Module`平台支持到`2.1.0`版本，对应TROS版本`2.2.0 (2024-04-11)`。
- `RDK X5`和`RDK X5 Module`平台支持到`2.4.2`版本，对应TROS版本`2.3.1 (2024-11-20)`。
:::

### 准备工作

#### RDK平台

1. RDK已烧录好Ubuntu系统镜像。

2. RDK已成功安装TogetheROS.Bot。

3. RDK已HDMI连接显示器。

### 使用介绍

#### RDK平台

通过SSH登录开发板，启动板端相关程序：

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
<TabItem value="jazzy" label="Jazzy">

```bash
# 配置tros.b环境
source /opt/tros/jazzy/setup.bash
```

</TabItem>

</Tabs>

使用RDK X5时, 需要额外使用下面命令:
```bash
# 关闭桌面显示
sudo systemctl stop lightdm
# 复制运行依赖
cp -r /opt/tros/${TROS_DISTRO}/lib/hobot_hdmi/config/ .
```

```shell
# HDMI图像渲染
ros2 launch hobot_hdmi hobot_hdmi.launch.py device:=F37
```

### 结果分析

在运行终端输出如下信息：

```text
[INFO] [launch]: All log files can be found below /root/.ros/log/2022-07-27-15-27-26-362299-ubuntu-13432
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [mipi_cam-1]: process started with pid [13434]
[INFO] [hobot_hdmi-2]: process started with pid [13436]
```

显示器显示图像如下：
![hdmi](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/02_quick_demo/image/demo_render/hdmi.png)

## RViz2展示

### 功能介绍

TogetheROS.Bot兼容ROS2，为了方便预览图像效果，可以通过RViz2获取图像。

### 支持平台

| 平台    | 运行方式      |
| ------- | ------------- |
| RDK X3, RDK X3 Module | Ubuntu 20.04 (Foxy), Ubuntu 22.04 (Humble) |
| RDK X5, RDK X5 Module, RDK S100 | Ubuntu 22.04 (Humble) |
| RDK S600 | Ubuntu 24.04 (Jazzy) |
| RDK Ultra | Ubuntu 20.04 (Foxy) |

### 准备工作

#### RDK平台

1. RDK已烧录好Ubuntu桌面版本系统镜像。

2. RDK已成功安装tros.b。

### 使用方式

#### RDK平台

1. 通过SSH登录RDK，启动板端相关程序

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
   <TabItem value="jazzy" label="Jazzy">

   ```bash
   # 配置tros.b环境
   source /opt/tros/jazzy/setup.bash
   ```

   </TabItem>

   </Tabs>

   ```shell
   # 启动mipi camera发布BGR8格式图像
   ros2 launch mipi_cam mipi_cam.launch.py mipi_out_format:=bgr8 mipi_image_width:=480 mipi_image_height:=272 mipi_io_method:=ros mipi_video_device:=F37
   ```

   注意: mipi_out_format请勿随意更改，RViz2只支持RGB8, RGBA8, BGR8, BGRA8等图像格式.

   如程序输出如下信息，说明节点已成功启动：

   ```shell
   [INFO] [launch]: All log files can be found below /root/.ros/log/2022-08-19-03-53-54-778203-ubuntu-2881662
   [INFO] [launch]: Default logging verbosity is set to INFO
   [INFO] [mipi_cam-1]: process started with pid [2881781]
   ```

2. RDK新建一个窗口，查询话题命令及返回结果如下：

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
   <TabItem value="jazzy" label="Jazzy">

   ```bash
   # 配置tros.b环境
   source /opt/tros/jazzy/setup.bash
   ```

   </TabItem>

   </Tabs>

   ```shell
   # 查询topic
   ros2 topic list
   ```

   输出：

   ```shell
   /camera_info
   /image_raw
   /parameter_events
   /rosout
   ```

3. RDK上启动RViz2订阅话题，并预览摄像头数据；

<Tabs groupId="tros-distro">
<TabItem value="foxy" label="Foxy">

   ```shell
   source /opt/tros/foxy/setup.bash
   ```

</TabItem>
<TabItem value="humble" label="Humble">

   ```shell
   source /opt/tros/humble/setup.bash
   ```

</TabItem>
<TabItem value="jazzy" label="Jazzy">

   ```bash
   source /opt/tros/jazzy/setup.bash
   ```

</TabItem>
</Tabs>

   ```shell
   # 安装RViz2
   sudo apt install ros-${TROS_DISTRO}-rviz-common ros-${TROS_DISTRO}-rviz-default-plugins ros-${TROS_DISTRO}-rviz2
   # 启动RViz2
   ros2 run rviz2 rviz2
   ```

   注意：RDK上运行rviz，需要使用mobaxterm等工具进行ssh连接，或者命令行ssh连接时加上“-Y”参数。

   在 RViz2 界面上首先点击 add 按钮，然后按照topic选择发布的图像，在该示例中topic名为/image_raw，然后点击image：

   ![rviz2-config](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/02_quick_demo/image/demo_render/rviz2-config.png)

   图像效果图如下：

   ![rviz2-result](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/02_quick_demo/image/demo_render/rviz2-result.png)


## RQt展示

### 功能介绍

TogetheROS.Bot兼容ROS2，支持通过RQt预览压缩格式图像，可以大幅度降低网络带宽消耗。本章节的示例将会在RDK上启动MIPI摄像头获取图像，然后在RDK上使用RQt预览。

### 支持平台

| 平台    | 运行方式      |
| ------- | ------------- |
| RDK X3, RDK X3 Module, RDK Ultra| Ubuntu 20.04 (Foxy), Ubuntu 22.04 (Humble)  |
| RDK X5, RDK X5 Module, RDK S100 | Ubuntu 22.04 (Humble) |
| RDK S600 | Ubuntu 24.04 (Jazzy) |

### 准备工作

#### RDK平台

1. RDK已烧录好Ubuntu桌面版本系统镜像。

2. RDK已成功安装tros.b。

### 使用方式

#### RDK平台

1. 通过SSH登录RDK开发板，启动mipi camera：

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
   <TabItem value="jazzy" label="Jazzy">

   ```bash
   # 配置tros.b环境
   source /opt/tros/jazzy/setup.bash
   ```

   </TabItem>

   </Tabs>

   ```shell
   ros2 launch mipi_cam mipi_cam.launch.py mipi_image_width:=640 mipi_image_height:=480 mipi_video_device:=F37
   ```

2. 在RDK上启动hobot_codec, 发布compressed格式图像：

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
   <TabItem value="jazzy" label="Jazzy">

   ```bash
   # 配置tros.b环境
   source /opt/tros/jazzy/setup.bash
   ```

   </TabItem>

   </Tabs>

   ```shell
   ros2 launch hobot_codec hobot_codec_encode.launch.py codec_out_format:=jpeg codec_pub_topic:=/image_raw/compressed
   ```

3. RDK上订阅话题，并预览摄像头数据；

<Tabs groupId="tros-distro">
<TabItem value="foxy" label="Foxy">

   ```shell
   source /opt/tros/foxy/setup.bash
   ```

</TabItem>
<TabItem value="humble" label="Humble">

   ```shell
   source /opt/tros/humble/setup.bash
   ```
</TabItem>
<TabItem value="jazzy" label="Jazzy">

   ```bash
   source /opt/tros/jazzy/setup.bash
   ```

</TabItem>
</Tabs>

   ```shell
   # 安装rqt
   sudo apt install ros-${TROS_DISTRO}-rqt-image-view ros-${TROS_DISTRO}-rqt ros-${TROS_DISTRO}-compressed-image-transport
   # 启动rqt
   ros2 run rqt_image_view rqt_image_view
   ```

   注意：RDK上运行rqt，需要使用mobaxterm等工具进行ssh连接，或者命令行ssh连接时加上“-Y”参数。

   选择话题`/image_raw/compressed`，图像效果图如下：

   ![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/02_quick_demo/image/demo_render/rqt-result.png)

## Foxglove展示

### 功能介绍

Foxglove是一个开源的工具包，包括线上和线下版。旨在简化机器人系统的开发和调试。它提供了一系列用于构建机器人应用程序的功能。

本章节主要用到Foxglove数据记录和回放功能：Foxglove允许将ROS2话题的数据记录到文件中，以便后续回放和分析。这对于系统故障诊断、性能优化和算法调试非常有用。

演示中，我们会利用TogetheROS开发的hobot_visualization功能包，将智能推理结果转换为ROS2渲染的话题信息。

代码仓库：[https://github.com/D-Robotics/hobot_visualization](https://github.com/D-Robotics/hobot_visualization)

### 支持平台

| 平台    | 运行方式      |
| ------- | ------------- |
| RDK X3, RDK X3 Module | Ubuntu 20.04 (Foxy), Ubuntu 22.04 (Humble) |
| RDK X5, RDK X5 Module, RDK S100 | Ubuntu 22.04 (Humble) |
| RDK S600 | Ubuntu 24.04 (Jazzy) |
| X86     | Ubuntu 20.04 (Foxy) |

### 准备工作

#### RDK平台

1. 确认摄像头F37正确接到RDK上

2. 确认PC可以通过网络访问RDK

3. 确认已成功安装TogetheROS.Bot

#### X86平台

1. 确认X86平台系统为Ubuntu 20.04，且已成功安装tros.b

### 使用方式

#### RDK平台 / X86平台

1. 通过SSH登录RDK平台，启动板端相关程序：

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
<TabItem value="jazzy" label="Jazzy">

```bash
# 配置tros.b环境
source /opt/tros/jazzy/setup.bash
```

</TabItem>

</Tabs>

```shell
export CAM_TYPE=fb

ros2 launch hobot_visualization hobot_vis_render.launch.py
```

同时，利用ssh登录另一个终端，在板端记录话题信息：

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
<TabItem value="jazzy" label="Jazzy">

```bash
# 配置tros.b环境
source /opt/tros/jazzy/setup.bash
```

</TabItem>

</Tabs>

```shell
# 记录rosbag数据，会生成在当前工作目录下
ros2 bag record -a
```

2. Foxglove在线页面播放rosbag数据

1）PC浏览器（chrome/firefox/edge）输入 (https://foxglove.dev/studio) , 进入foxglove官网

   ![foxglove](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/02_quick_demo/image/demo_render/foxglove_guide_1.png)

PS: 首次使用需要注册, 可使用谷歌账号或第三方邮箱进行注册。

   ![foxglove](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/02_quick_demo/image/demo_render/foxglove_guide_11.png)

2）进入可视化功能界面

   ![foxglove](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/02_quick_demo/image/demo_render/foxglove_guide_2.png)

3）点击选中本地rosbag文件

   ![foxglove](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/02_quick_demo/image/demo_render/foxglove_guide_3.png)

4）打开布局界面，在布局界面右上角，点击设置，选中图标，打开播放maker渲染消息功能

   ![foxglove](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/02_quick_demo/image/demo_render/foxglove_guide_4.png)

5）点击播放
   ![foxglove](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/02_quick_demo/image/demo_render/foxglove_guide_5.png)

6）观看数据
   ![foxglove](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/02_quick_demo/image/demo_render/foxglove_guide_6.png)

### 注意事项

1. Foxglove可视化图像数据，需采用ROS2官方的消息格式，使用foxglove支持的图像编码格式，详情请见 (https://foxglove.dev/docs/studio/panels/image)。

2. rosbag进行消息记录时，可能会录制其他设备的话题信息，因此为了保证rosbag数据的干净，可以通过设置'export ROS_DOMAIN_ID=xxx' ，如'export ROS_DOMAIN_ID=1'的方法。
