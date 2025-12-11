---
sidebar_position: 5
---

# 双目深度算法

```mdx-code-block
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';
```

## 功能介绍

地瓜双目深度估计算法输入为双目图像数据，输出为左视图对应的视差图和深度图。算法借鉴IGEV网络，采用了GRU架构，具有较好的数据泛化性和较高的推理效率。

双目算法代码仓库：https://github.com/D-Robotics/hobot_stereonet

mipi相机代码仓库：https://github.com/D-Robotics/hobot_mipi_cam

zed相机代码仓库：https://github.com/D-Robotics/hobot_zed_cam

双目算法讲解：[直播回放 | 基于RDK X5的AI双目算法部署实战](https://www.bilibili.com/video/BV1KdEjzREMz/?share_source=copy_web&vd_source=deb3551e36cc4b1c1020033ad17c564b)

## 支持平台

| 平台                  | 系统支持              | 示例功能                                    |
| --------------------- | --------------------- | ------------------------------------------- |
| RDK X5, RDK X5 Module | Ubuntu 22.04 (Humble) | 启动双目相机，推理出深度结果，并在Web端显示 |
| RDK S100, RDK S100P   | Ubuntu 22.04 (Humble) | 启动双目相机，推理出深度结果，并在Web端显示 |

## 算法版本信息

目前双目算法已有如下版本可供使用：

| 平台 | 算法版本 | 量化方式 | 输入尺寸 | 推理帧率(fps) | 算法特性                                                                  |
| ------------- | --- | --- | ---| --- | --- |
| X5 | V2.0 | int16 | 640x352x3x2 | 15 | 精度较高、帧率较低 |
| X5 | V2.1 | int16 | 640x352x3x2 | 15 | 有置信度输出 |
| X5 | V2.2 | int8 | 640x352x3x2 | 23 | 精度较低、帧率较高 |
| X5 | V2.3 | int8 | 640x352x3x2 | 27 | 帧率进一步提升 |
| X5 | V2.4 | int16 | 640x352x3x2 | 15 | 加入更多数据训练 |
| X5 | V2.4 | int8 | 640x352x3x2 | 23 | 加入更多数据训练 |
| S100/S600 | V2.1 | int16 | 640x352x3x2 | 53 | 有置信度输出 |
| S100/S600 | V2.4 | int16 | 640x352x3x2 | 53 | 有置信度输出，加入更多数据训练|

## 准备工作

### RDK平台

1. RDK已烧录好RDK OS系统
2. RDK已成功安装TogetheROS.Bot
3. 如果需要实时在线推理，请准备好双目相机，目前支持官方MIPI相机、ZED mini/2i USB相机
4. 如果需要离线推理，请准备好<strong style={{ color: 'red' }}>矫正后</strong>的双目图像数据
5. 确认PC机能够通过网络访问RDK

### 系统和功能包版本

|                                       | 版本            | 查询方法                                        |
| ------------------------------------- |---------------| ----------------------------------------------- |
| RDK X5系统镜像版本                    | 3.3.3及以上      | `cat /etc/version`                              |
| RDK S100系统镜像版本                  | 4.0.2-Beta及以上 | `cat /etc/version`                              |
| tros-humble-hobot-stereonet功能包版本 | 2.4.4及以上      | `apt list \| grep tros-humble-hobot-stereonet/` |
| tros-humble-mipi-cam功能包版本        | 2.3.13及以上     | `apt list \| grep tros-humble-mipi-cam/`        |
| tros-humble-hobot-zed-cam功能包版本   | 2.3.3及以上      | `apt list \| grep tros-humble-hobot-zed-cam/`   |

- 如果系统版本不符合要求，请参考文档对应章节进行镜像烧录
- 如果功能包版本不符合要求，请执行以下指令进行更新：

```bash
sudo apt update
sudo apt upgrade
```

:::caution **注意**
**如果`sudo apt update`命令执行失败或报错，请查看[常见问题](/docs/08_FAQ/01_hardware_and_system.md)章节的`Q10: apt update 命令执行失败或报错如何处理？`解决。**
:::

## 使用介绍

:::caution **注意**
**请用`root`用户执行文档中的命令，其它用户执行可能权限不够，造成一些不必要的错误。**
:::

![os_user](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/03_boxs/function/image/box_adv/os_user.png)

双目深度算法支持多款相机，启动命令有一些区别，具体启动命令如下：

### (1) 搭配 RDK Stereo Camera Module 启动

- RDK官方MIPI双目相机如图所示：

![RDK_Stereo_Cam_230ai](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/03_boxs/function/image/box_adv/RDK_Stereo_Cam_230ai.png)

<p style={{ color: 'red' }}> 注意：请检查相机背面丝印印有CDPxxx-V3，确认相机是V3版本 </p>

- 安装方式如图所示，接线请勿接反，会导致左右图对调，双目算法运行错误：

![RDK_X5_230ai](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/03_boxs/function/image/box_adv/RDK_X5_230ai.png)

![RDK_S100_230ai](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/03_boxs/function/image/box_adv/RDK_S100_230ai.png)

- 确认相机连接是否正常，通过ssh连接RDK，执行以下命令，如果输出0x30、0x32、0x50等地址，则代表相机连接正常：

```bash
# RDK X5
i2cdetect -r -y 4
i2cdetect -r -y 6

# RDK S100
i2cdetect -r -y 1
i2cdetect -r -y 2
```

![i2cdetect_230ai](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/03_boxs/function/image/box_adv/i2cdetect_230ai.png)

- 通过不同的launch文件，启动相应版本的双目算法，通过ssh连接RDK X5，执行以下命令：

<Tabs groupId="stereo-version">
<TabItem value="V2.0" label="V2.0">

```bash
# 只支持RDK X5

# 配置tros.b humble环境
source /opt/tros/humble/setup.bash

# 启动双目模型launch文件，其包含了算法和双目相机节点的启动
ros2 launch hobot_stereonet stereonet_model_web_visual_v2.0.launch.py \
mipi_image_width:=640 mipi_image_height:=352 mipi_lpwm_enable:=True mipi_image_framerate:=30.0 \
need_rectify:=False height_min:=-10.0 height_max:=10.0 pc_max_depth:=5.0
```

</TabItem>
<TabItem value="V2.1" label="V2.1">

```bash
# 支持RDK X5和RDK X100

# 配置tros.b humble环境
source /opt/tros/humble/setup.bash

# 启动双目模型launch文件，其包含了算法和双目相机节点的启动
ros2 launch hobot_stereonet stereonet_model_web_visual_v2.1.launch.py \
mipi_image_width:=640 mipi_image_height:=352 mipi_lpwm_enable:=True mipi_image_framerate:=30.0 \
need_rectify:=False height_min:=-10.0 height_max:=10.0 pc_max_depth:=5.0 \
uncertainty_th:=0.1
```

</TabItem>
<TabItem value="V2.2" label="V2.2">

```bash
# 只支持RDK X5

# 配置tros.b humble环境
source /opt/tros/humble/setup.bash

# 启动双目模型launch文件，其包含了算法和双目相机节点的启动
ros2 launch hobot_stereonet stereonet_model_web_visual_v2.2.launch.py \
mipi_image_width:=640 mipi_image_height:=352 mipi_lpwm_enable:=True mipi_image_framerate:=30.0 \
need_rectify:=False height_min:=-10.0 height_max:=10.0 pc_max_depth:=5.0
```

</TabItem>
<TabItem value="V2.3" label="V2.3">

```bash
# 只支持RDK X5

# 配置tros.b humble环境
source /opt/tros/humble/setup.bash

# 启动双目模型launch文件，其包含了算法和双目相机节点的启动
ros2 launch hobot_stereonet stereonet_model_web_visual_v2.3.launch.py \
mipi_image_width:=640 mipi_image_height:=352 mipi_lpwm_enable:=True mipi_image_framerate:=30.0 \
need_rectify:=False height_min:=-10.0 height_max:=10.0 pc_max_depth:=5.0
```

</TabItem>
<TabItem value="V2.4" label="V2.4">

```bash
# RDK X5运行如下指令：

# 配置tros.b humble环境
source /opt/tros/humble/setup.bash

# int16版本，启动双目模型launch文件，其包含了算法和双目相机节点的启动
ros2 launch hobot_stereonet stereonet_model_web_visual_v2.4_int16.launch.py \
mipi_image_width:=640 mipi_image_height:=352 mipi_lpwm_enable:=True mipi_image_framerate:=30.0 \
need_rectify:=False height_min:=-10.0 height_max:=10.0 pc_max_depth:=5.0

# int8版本，启动双目模型launch文件，其包含了算法和双目相机节点的启动
ros2 launch hobot_stereonet stereonet_model_web_visual_v2.4_int8.launch.py \
mipi_image_width:=640 mipi_image_height:=352 mipi_lpwm_enable:=True mipi_image_framerate:=30.0 \
need_rectify:=False height_min:=-10.0 height_max:=10.0 pc_max_depth:=5.0

# RDK S100运行如下指令：

# 配置tros.b humble环境
source /opt/tros/humble/setup.bash

# 启动双目模型launch文件，其包含了算法和双目相机节点的启动

ros2 launch hobot_stereonet stereonet_model_web_visual_v2.4.launch.py \
mipi_image_width:=640 mipi_image_height:=352 mipi_lpwm_enable:=True mipi_image_framerate:=30.0 \
need_rectify:=False height_min:=-10.0 height_max:=10.0 pc_max_depth:=5.0 \
uncertainty_th:=0.1
```

</TabItem>
</Tabs>

参数含义如下：

| 名称                 | 参数值      | 说明                                                                       |
| -------------------- | ----------- | -------------------------------------------------------------------------- |
| mipi_image_width     | 设置为640   | MIPI相机的输出分辨率是640*352                                              |
| mipi_image_height    | 设置为352   | MIPI相机的输出分辨率是640*352                                              |
| mipi_lpwm_enable     | 设置为True  | MIPI相机开启硬件同步                                                       |
| mipi_image_framerate | 设置为30.0  | MIPI相机输出帧率为30.0FPS                                                  |
| need_rectify         | 设置为False | 因为官方相机出厂自带标定参数，会自动矫正，不需要加载自定义标定文件进行矫正 |
| height_min           | 设置为-10.0 | 点云最小高度为-10.0m                                                       |
| height_max           | 设置为10.0  | 点云最大高度为10.0m                                                        |
| pc_max_depth         | 设置为5.0   | 点云最大距离为5.0m                                                         |
| uncertainty_th       | 设置为0.1   | 置信度设置，用于过滤噪声点，建议设置为0.1，置信度越小过滤越严格            |

- 出现如下日志表示双目算法启动成功，`fx/fy/cx/cy/base_line`是相机内参，如果深度图正常，但估计出来的距离有偏差，可能是相机内参存在问题：

![stereonet_run_success_log](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/03_boxs/function/image/box_adv/stereonet_run_success_log.png)

- 通过网页端查看深度图，在浏览器输入 http://ip:8000 (图中RDK ip是192.168.1.100)：

![web_depth_visual](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/03_boxs/function/image/box_adv/web_depth_visual.png)

- 通过rviz2查看点云，RDK可直接安装rviz2查看，注意rviz2中需要做如下配置：

```bash
# 安装rviz2
sudo apt install ros-humble-rviz2
# 启动rviz2
source /opt/tros/humble/setup.bash
rviz2
```

![stereonet_rviz](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/03_boxs/function/image/box_adv/stereonet_rviz.png)

- 如果用户想保存深度估计结果，可以添加如下参数实现，`save_image_all`打开保存开关，`save_freq`控制保存频率，`save_dir`控制保存的目录（如果目录不存在会自动创建），`save_total`控制保存的总数。程序运行将会保存**相机内参、左右图、视差图、深度图、可视化图**：

```bash
# 配置tros.b humble环境
source /opt/tros/humble/setup.bash

# 这里以V2.0版本的算法为例，其它版本的算法类似加入对应参数即可
ros2 launch hobot_stereonet stereonet_model_web_visual_v2.0.launch.py \
mipi_image_width:=640 mipi_image_height:=352 mipi_lpwm_enable:=True mipi_image_framerate:=30.0 \
need_rectify:=False height_min:=-10.0 height_max:=10.0 pc_max_depth:=5.0 \
save_image_all:=True save_freq:=4 save_dir:=./online_result save_total:=10
```

参数含义如下：

| 名称           | 参数值               | 说明                                       |
| -------------- | -------------------- | ------------------------------------------ |
| save_image_all | 设置为True           | 保存图像开关                               |
| save_freq      | 设置为4              | 每隔4帧保存一次，可修改为任意正数          |
| save_dir       | 设置为保存图像的目录 | 可根据需要设置保存位置                     |
| save_total     | 设置为10             | 总共保存10张图像，设置为-1则代表为一直保存 |

![stereonet_save_log](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/03_boxs/function/image/box_adv/stereonet_save_log.png)

![stereonet_save_files](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/03_boxs/function/image/box_adv/stereonet_save_files.png)

### (2) 搭配 RDK Stereo Camera GS130W 启动

- RDK官方MIPI双目相机如图所示：

![RDK_Stereo_Cam_132gs](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/03_boxs/function/image/box_adv/RDK_Stereo_Cam_132gs.png)

- 安装方式如图所示，接线请勿接反，会导致左右图对调，双目算法运行错误：

![RDK_X5_132gs](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/03_boxs/function/image/box_adv/RDK_X5_132gs.png)

- 确认相机连接是否正常，通过ssh连接RDK，执行以下命令，如果输出0x32、0x33、0x50等地址，则代表相机连接正常：

```bash
# RDK X5
i2cdetect -r -y 4
i2cdetect -r -y 6

# RDK S100
i2cdetect -r -y 1
i2cdetect -r -y 2
```

![i2cdetect_132gs](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/03_boxs/function/image/box_adv/i2cdetect_132gs.png)

- 通过不同的launch文件，启动相应版本的双目算法，通过ssh连接RDK X5，执行以下命令：

<Tabs groupId="stereo-version">
<TabItem value="V2.0" label="V2.0">

```bash
# 只支持RDK X5

# 配置tros.b humble环境
source /opt/tros/humble/setup.bash

# 启动双目模型launch文件，其包含了算法和双目相机节点的启动
ros2 launch hobot_stereonet stereonet_model_web_visual_v2.0.launch.py \
mipi_image_width:=640 mipi_image_height:=352 mipi_lpwm_enable:=True mipi_image_framerate:=30.0 mipi_rotation:=90.0 \
need_rectify:=False height_min:=-10.0 height_max:=10.0 pc_max_depth:=5.0
```

</TabItem>
<TabItem value="V2.1" label="V2.1">

```bash
# 支持RDK X5和RDK X100

# 配置tros.b humble环境
source /opt/tros/humble/setup.bash

# 启动双目模型launch文件，其包含了算法和双目相机节点的启动
ros2 launch hobot_stereonet stereonet_model_web_visual_v2.1.launch.py \
mipi_image_width:=640 mipi_image_height:=352 mipi_lpwm_enable:=True mipi_image_framerate:=30.0 mipi_rotation:=90.0 \
need_rectify:=False height_min:=-10.0 height_max:=10.0 pc_max_depth:=5.0 \
uncertainty_th:=0.1
```

</TabItem>
<TabItem value="V2.2" label="V2.2">

```bash
# 只支持RDK X5

# 配置tros.b humble环境
source /opt/tros/humble/setup.bash

# 启动双目模型launch文件，其包含了算法和双目相机节点的启动
ros2 launch hobot_stereonet stereonet_model_web_visual_v2.2.launch.py \
mipi_image_width:=640 mipi_image_height:=352 mipi_lpwm_enable:=True mipi_image_framerate:=30.0 mipi_rotation:=90.0 \
need_rectify:=False height_min:=-10.0 height_max:=10.0 pc_max_depth:=5.0
```

</TabItem>
<TabItem value="V2.3" label="V2.3">

```bash
# 只支持RDK X5

# 配置tros.b humble环境
source /opt/tros/humble/setup.bash

# 启动双目模型launch文件，其包含了算法和双目相机节点的启动
ros2 launch hobot_stereonet stereonet_model_web_visual_v2.3.launch.py \
mipi_image_width:=640 mipi_image_height:=352 mipi_lpwm_enable:=True mipi_image_framerate:=30.0 mipi_rotation:=90.0 \
need_rectify:=False height_min:=-10.0 height_max:=10.0 pc_max_depth:=5.0
```

</TabItem>
<TabItem value="V2.4" label="V2.4">

```bash
# RDK X5运行如下指令：

# 配置tros.b humble环境
source /opt/tros/humble/setup.bash

# int16版本，启动双目模型launch文件，其包含了算法和双目相机节点的启动
ros2 launch hobot_stereonet stereonet_model_web_visual_v2.4_int16.launch.py \
mipi_image_width:=640 mipi_image_height:=352 mipi_lpwm_enable:=True mipi_image_framerate:=30.0 mipi_rotation:=90.0 \
need_rectify:=False height_min:=-10.0 height_max:=10.0 pc_max_depth:=5.0

# int8版本，启动双目模型launch文件，其包含了算法和双目相机节点的启动
ros2 launch hobot_stereonet stereonet_model_web_visual_v2.4_int8.launch.py \
mipi_image_width:=640 mipi_image_height:=352 mipi_lpwm_enable:=True mipi_image_framerate:=30.0 mipi_rotation:=90.0 \
need_rectify:=False height_min:=-10.0 height_max:=10.0 pc_max_depth:=5.0

# RDK S100运行如下指令：

# 配置tros.b humble环境
source /opt/tros/humble/setup.bash

# 启动双目模型launch文件，其包含了算法和双目相机节点的启动

ros2 launch hobot_stereonet stereonet_model_web_visual_v2.4.launch.py \
mipi_image_width:=640 mipi_image_height:=352 mipi_lpwm_enable:=True mipi_image_framerate:=30.0 mipi_rotation:=90.0 \
need_rectify:=False height_min:=-10.0 height_max:=10.0 pc_max_depth:=5.0 \
uncertainty_th:=0.1
```

</TabItem>
</Tabs>

参数含义如下：

| 名称                 | 参数值      | 说明                                                                       |
| -------------------- | ----------- | -------------------------------------------------------------------------- |
| mipi_image_width     | 设置为640   | MIPI相机的输出分辨率是640*352                                              |
| mipi_image_height    | 设置为352   | MIPI相机的输出分辨率是640*352                                              |
| mipi_lpwm_enable     | 设置为True  | MIPI相机开启硬件同步                                                       |
| mipi_image_framerate | 设置为30.0  | MIPI相机输出帧率为30.0FPS                                                  |
| need_rectify         | 设置为False | 因为官方相机出厂自带标定参数，会自动矫正，不需要加载自定义标定文件进行矫正 |
| height_min           | 设置为-10.0 | 点云最小高度为-10.0m                                                       |
| height_max           | 设置为10.0  | 点云最大高度为10.0m                                                        |
| pc_max_depth         | 设置为5.0   | 点云最大距离为5.0m                                                         |
| uncertainty_th       | 设置为0.1   | 置信度设置，用于过滤噪声点，建议设置为0.1，置信度越小过滤越严格            |

- 出现如下日志表示双目算法启动成功，`fx/fy/cx/cy/base_line`是相机内参，如果深度图正常，但估计出来的距离有偏差，可能是相机内参存在问题：

![stereonet_run_success_log](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/03_boxs/function/image/box_adv/stereonet_run_success_log.png)

- 通过网页端查看深度图，在浏览器输入 http://ip:8000 (图中RDK ip是192.168.1.100)：

![web_depth_visual](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/03_boxs/function/image/box_adv/web_depth_visual.png)

- 通过rviz2查看点云，RDK可直接安装rviz2查看，注意rviz2中需要做如下配置：

```bash
# 安装rviz2
sudo apt install ros-humble-rviz2
# 启动rviz2
source /opt/tros/humble/setup.bash
rviz2
```

![stereonet_rviz](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/03_boxs/function/image/box_adv/stereonet_rviz.png)

- 如果用户想保存深度估计结果，可以添加如下参数实现，`save_image_all`打开保存开关，`save_freq`控制保存频率，`save_dir`控制保存的目录（如果目录不存在会自动创建），`save_total`控制保存的总数。程序运行将会保存**相机内参、左右图、视差图、深度图、可视化图**：

```bash
# 配置tros.b humble环境
source /opt/tros/humble/setup.bash

# 这里以V2.0版本的算法为例，其它版本的算法类似加入对应参数即可
ros2 launch hobot_stereonet stereonet_model_web_visual_v2.0.launch.py \
mipi_image_width:=640 mipi_image_height:=352 mipi_lpwm_enable:=True mipi_image_framerate:=30.0  mipi_rotation:=90.0 \
need_rectify:=False height_min:=-10.0 height_max:=10.0 pc_max_depth:=5.0 \
save_image_all:=True save_freq:=4 save_dir:=./online_result save_total:=10
```

参数含义如下：

| 名称           | 参数值               | 说明                                       |
| -------------- | -------------------- | ------------------------------------------ |
| save_image_all | 设置为True           | 保存图像开关                               |
| save_freq      | 设置为4              | 每隔4帧保存一次，可修改为任意正数          |
| save_dir       | 设置为保存图像的目录 | 可根据需要设置保存位置                     |
| save_total     | 设置为10             | 总共保存10张图像，设置为-1则代表为一直保存 |

![stereonet_save_log](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/03_boxs/function/image/box_adv/stereonet_save_log.png)

![stereonet_save_files](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/03_boxs/function/image/box_adv/stereonet_save_files.png)


### (3) 本地图片离线回灌

- 如果想利用本地图片评估算法效果，可以使用下列命令指定算法运行模式、图像数据地址以及相机内参，同时要保证图像数据经过**去畸变、极线对齐**。图片的格式如下图所示，第一张左目图像的命名为left000000.png，第二张左目图像的命名为left000001.png，以此类推。对应的第一张右目图像的命名为right000000.png，第二张右目图像的命名为right000001.png，以此类推。算法按序号遍历图像，直至图像全部计算完毕：

![stereonet_rdk](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/03_boxs/function/image/box_adv/image_format.png)

- 算法离线运行方式如下，**参考上文的在线启动命令**，删除在线启动命令中`mipi`相关的参数，添加`use_local_image=True`打开离线推理开关，`local_image_path`设置离线图像目录，`camera_fx/camera_fy/camera_cx/camera_cy/base_line`设置矫正后相机参数，`save_image_all:=True`打开保存结果开关，`save_dir`控制保存的目录（如果目录不存在会自动创建）。<span style={{ color: 'red' }}> 注意：回灌的图像一定要极线矫正，并且一定要设置正确的相机参数，否则回灌保存的结果可能是错误的 </span>

```shell
# 配置tros.b humble环境
source /opt/tros/humble/setup.bash

# 这里以V2.0版本的算法为例，其它版本的算法类似加入对应参数即可
ros2 launch hobot_stereonet stereonet_model_web_visual_v2.0.launch.py \
need_rectify:=False height_min:=-10.0 height_max:=10.0 pc_max_depth:=5.0 \
use_local_image:=True local_image_path:=./online_result \
camera_fx:=216.696533 camera_fy:=216.696533 camera_cx:=335.313477 camera_cy:=182.961578 base_line:=0.070943 \
save_image_all:=True save_dir:=./offline_result
```

![stereonet_offline_log](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/03_boxs/function/image/box_adv/stereonet_offline_log.png)

参数含义如下：

| 名称             | 参数值                 | 说明                                                               |
| ---------------- | ---------------------- | ------------------------------------------------------------------ |
| use_local_image  | 设置为True             | 图片回灌模式开关                                                   |
| local_image_path | 设置为离线数据目录     | 回灌图像的地址目录                                                 |
| need_rectify     | 设置为False            | 回灌图像要求经过极线矫正，不需要开启此开关，但要手动传入矫正后参数 |
| camera_fx        | 设置为相机矫正后内参fx | 相机内参                                                           |
| camera_fy        | 设置为相机矫正后内参fy | 相机内参                                                           |
| camera_cx        | 设置为相机矫正后内参cx | 相机内参                                                           |
| camera_cy        | 设置为相机矫正后内参cy | 相机内参                                                           |
| base_line        | 设置为相机矫正后基线   | 基线距离，单位为m                                                  |
| height_min       | 设置为-10.0            | 点云最小高度为-10.0m                                               |
| height_max       | 设置为10.0             | 点云最大高度为10.0m                                                |
| pc_max_depth     | 设置为5.0              | 是点云最大距离为5.0m                                               |
| save_image_all   | 设置为True             | 保存回灌结果                                                       |
| save_dir         | 设置为保存图像的目录   | 可根据需要设置保存位置                                             |

- 算法运行成功后，同样可以通过网页端和rviz2显示结果数据，参考上文对应的设置

### (4) 搭配ZED双目摄像头启动

- ZED双目摄像头如图所示：

![zed_cam](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/03_boxs/function/image/box_adv/zed_cam.png)

- 将ZED相机通过USB连接RDK，然后启动双目算法，通过ssh连接RDK，执行以下命令：

<p style={{ color: 'red' }}> 注意：运行ZED相机RDK一定要联网，因为ZED需要联网下载标定文件 </p>

<Tabs groupId="stereo-version">
<TabItem value="V2.0" label="V2.0">

```shell
# 只支持RDK X5

# 配置tros.b humble环境
source /opt/tros/humble/setup.bash

# 启动双目模型launch文件，其包含了算法和双目相机节点的启动
ros2 launch hobot_stereonet stereonet_model_web_visual_zed_v2.0.launch.py use_mipi_cam:=False \
height_min:=-10.0 height_max:=10.0 pc_max_depth:=5.0 uncertainty_th:=0.10
```

</TabItem>
<TabItem value="V2.1" label="V2.1">

```shell
# 只支持RDK S100

# 配置tros.b humble环境
source /opt/tros/humble/setup.bash

# 启动双目模型launch文件，其包含了算法和双目相机节点的启动
ros2 launch hobot_stereonet stereonet_model_web_visual_zed_v2.1.launch.py use_mipi_cam:=False \
height_min:=-10.0 height_max:=10.0 pc_max_depth:=5.0 uncertainty_th:=0.10
```

</TabItem>
<TabItem value="V2.2" label="V2.2">

```shell
# 只支持RDK X5

# 配置tros.b humble环境
source /opt/tros/humble/setup.bash

# 启动双目模型launch文件，其包含了算法和双目相机节点的启动
ros2 launch hobot_stereonet stereonet_model_web_visual_zed_v2.2.launch.py use_mipi_cam:=False \
height_min:=-10.0 height_max:=10.0 pc_max_depth:=5.0 uncertainty_th:=0.10
```

</TabItem>
<TabItem value="V2.4" label="V2.4">

```shell
# 只支持RDK S100

# 配置tros.b humble环境
source /opt/tros/humble/setup.bash

# 启动双目模型launch文件，其包含了算法和双目相机节点的启动
ros2 launch hobot_stereonet stereonet_model_web_visual_zed_v2.4.launch.py use_mipi_cam:=False \
height_min:=-10.0 height_max:=10.0 pc_max_depth:=5.0 uncertainty_th:=0.10
```

</TabItem>
</Tabs>

![stereonet_zed_run_success_log](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/03_boxs/function/image/box_adv/stereonet_zed_run_success_log.png)

联网的情况下程序会自动下载标定文件，如果RDK没有联网，可以手动下载标定文件然后上传到RDK：根据log信息，在PC端打开浏览器，输入(https://calib.stereolabs.com/?SN=38085162) ，即可下载标定文件SN38085162.conf，注意每台ZED的SN码是不一样的，使用请根据报错信息的提示下载对应的标定文件，将标定文件上传到`/root/zed/settings/`目录下，如果目录不存在创建一个即可。

- 通过网页端查看深度图，在浏览器输入 http://ip:8000 (ip为RDK对应的ip地址)，如需查看**点云**和**保存图像**请参考上文对应的设置

## hobot_stereonet功能包说明

### 订阅话题

| 名称               | 消息类型                     | 说明                                                         |
| ------------------ | ---------------------------- | ------------------------------------------------------------ |
| /image_combine_raw | sensor_msgs::msg::Image      | 双目相机节点发布的左右目拼接图像话题，用于模型推理深度       |
| /camera_info_topic | sensor_msgs::msg::CameraInfo | 双目相机节点发布的相机参数话题，用于视差图和深度图之间的转换 |

### 发布话题

| 名称                                 | 消息类型                      | 说明                                     |
| ------------------------------------ | ----------------------------- | ---------------------------------------- |
| /StereoNetNode/stereonet_depth       | sensor_msgs::msg::Image       | 发布的深度图像，像素值为深度，单位为毫米 |
| /StereoNetNode/stereonet_visual      | sensor_msgs::msg::Image       | 发布的比较直观的可视化渲染图像           |
| /StereoNetNode/stereonet_pointcloud2 | sensor_msgs::msg::PointCloud2 | 发布的点云深度话题                       |

### 其它重要参数

| 名称                   | 参数值                            | 说明                                                                                       |
| ---------------------- | --------------------------------- | ------------------------------------------------------------------------------------------ |
| stereo_image_topic     | 默认 /image_combine_raw           | 订阅双目图像消息的话题名                                                                   |
| camera_info_topic      | 默认 /image_right_raw/camera_info | 订阅相机矫正参数消息的话题名                                                               |
| need_rectify           | 默认 True                         | 是否指定自定义标定文件对图像进行矫正开关                                                   |
| stereo_calib_file_path | 默认 stereo.yaml                  | need_rectify=True的情况下，加载该路径下的标定文件进行标定                                  |
| stereo_combine_mode    | 默认 1                            | 左右目图像往往拼接在一张图上再发布出去，1为上下拼接，0为左右拼接，指示双目算法如何拆分图像 |
| KMean                  | 默认 10                           | 过滤稀疏离群点时每个点的临近点的数目，统计每个点与周围最近10个点的距离                     |
| stdv                   | 默认 0.01                         | 过滤稀疏离群点时判断是否为离群点的阈值，将标准差的倍数设置为0.01                           |
| leaf_size              | 默认 0.05                         | 设置点云的单位密度，表示半径0.05米的三维球内只有一个点                                     |
