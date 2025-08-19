---
sidebar_position: 6
---

# 双目OCC算法

```mdx-code-block
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';
```

## 功能介绍

**地瓜双目OCC算法**订阅双目图像，利用BPU进行算法推理，发布占用网格信息。

双目OCC算法代码仓库：https://github.com/D-Robotics/dstereo_occnet

zed相机代码仓库：https://github.com/D-Robotics/hobot_zed_cam

## 支持平台

| 平台                  | 系统支持              | 示例功能                                                   |
| --------------------- | --------------------- | ---------------------------------------------------------- |
| RDK X5, RDK X5 Module | Ubuntu 22.04 (Humble) | 启动双目相机，并通过Web展示双目图像，rviz2展示占用网格结果 |
| RDK S100, RDK S100P   | Ubuntu 22.04 (Humble) | 启动双目相机，并通过Web展示双目图像，rviz2展示占用网格结果 |

## 算法信息

| 模型          | 平台 | 输入尺寸    | 推理帧率(fps) |
| ------------- | ---- | ----------- | ------------- |
| DStereoOccNet | X5   | 2x3x352x640 | 6             |
| DStereoOccNet | S100 | 2x3x352x640 | 45            |

## 准备工作

### RDK平台

1. RDK已烧录好Ubuntu 22.04系统镜像

2. RDK已成功安装TogetheROS.Bot

3. 如果要实时在线推理，目前只支持搭配ZED-2i相机；如果要离线推理，请准备<strong style={{ color: 'red' }}>矫正后</strong>的双目图像数据

4. 确认PC机能够通过网络访问RDK

### 系统和功能包版本

|                          | 版本要求         | 查询方式                                       |
| ------------------------ | ---------------- | ---------------------------------------------- |
| RDK X5系统镜像版本       | 3.3.1及以上      | `cat /etc/version`                             |
| RDK S100系统镜像版本     | 4.0.2-Beta及以上 | `cat /etc/version`                             |
| dstereo_occnet功能包版本 | 1.0.1及以上      | `apt list \| grep tros-humble-dstereo-occnet/` |
| hobot_zed_cam            | 2.3.3及以上      | `apt list \| grep tros-humble-hobot-zed-cam/`  |

## 使用介绍

### 1. 搭配ZED-2i相机

- 需要配合hobot_zed_cam功能包一起使用

- 在RDK上执行以下命令（X5和S100均支持）:

```bash
# 配置tros.b humble环境
source /opt/tros/humble/setup.bash

# 启动ZED-2i相机和占用网络推理程序
ros2 launch dstereo_occnet zed2i_occ_node.launch.py
```

- 程序启动后可以通过网页查看ZED-2i发布的双目图像，在PC端浏览器输入http://ip:8000 即可查看双目图像，ip为RDK板端的ip，例子中为`192.168.128.10`，并且要保证PC和RDK能通过网络通讯

![ZED-2i-stereo-img](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/03_boxs/function/image/box_adv/ZED-2i-stereo-img.png)

- 程序启动后可以通过rviz2可查看占用网格，RDK可直接安装rviz2查看，注意rviz2中需要做如下配置：

```bash
# 安装rviz2
sudo apt install ros-humble-rviz2
# 启动rviz2
source /opt/tros/humble/setup.bash
rviz2
```

![rviz2-occ](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/03_boxs/function/image/box_adv/rviz2-occ.png)

- 如需保存结果请加入以下参数，`save_occ_flag`打开保存开关，`save_occ_dir`控制保存的目录（如果目录不存在会自动创建），`save_freq`控制保存频率，`save_total`控制保存的总数：

```bash
# 配置tros.b humble环境
source /opt/tros/humble/setup.bash

# 启动ZED-2i相机和占用网络推理程序
ros2 launch dstereo_occnet zed2i_occ_node.launch.py \
save_occ_flag:=True save_occ_dir:=./occ_result save_freq:=4 save_total:=10
```

### 2. 使用自定义数据离线推理

- 需要准备离线数据上传到RDK板端，离线数据的格式如下：
    - 离线目录需要包含左右图片，程序会进行判断，左图需要包含`left`字段，格式为png或者jpg，右图需要包含`right`字段、其它和左图一致
    - 图像分辨率为`640*352`，不支持其它分辨率
    - 左右图需要进行矫正，达到极线对齐的状态
    - 由于目前采用ZED-2i的数据训练模型，所以尽量使离线图像的内参接近ZED-2i，ZED-2i相机参数为: `fx=354.9999, fy=354.9999, cx=322.9469, cy=176.2076, baseline=0.12`

![stereonet_rdk](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/03_boxs/function/image/box_adv/image_format.png)

- 在RDK上执行以下命令（X5和S100均支持），`local_image_dir`控制离线数据的目录，`save_occ_flag`打开保存开关，`save_occ_dir`控制保存的目录（如果目录不存在会自动创建）:

```bash
ros2 launch dstereo_occnet offline_infer_web_visual.launch.py \
local_image_dir:=./offline_images save_occ_flag:=True save_occ_dir:=./offline_result
```

- 程序启动后同样可以通过网页查看ZED-2i发布的双目图像以及通过rviz2查看占用网格

## 功能包说明

### 参数

| 名称                | 参数值                                         | 说明                                                            |
| ------------------- | ---------------------------------------------- | --------------------------------------------------------------- |
| stereo_msg_topic    | 默认 /image_combine_raw                        | 订阅的双目图像话题名称                                          |
| camera_info_topic   | 默认 /image_combine_raw/camera_info            | 订阅的相机内参话题名称                                          |
| occ_model_file_path | 默认 X5-OCC-32x64x96x2_constinput_modified.bin | 双目占用网络模型的路径                                          |
| use_local_image     | 默认 False                                     | 是否使用离线推理                                                |
| local_image_dir     | 默认 config                                    | 离线推理存放图片的目录                                          |
| save_occ_flag       | 默认 False                                     | 是否保存推理结果                                                |
| save_occ_dir        | 默认 ./occ_results                             | 保存推理结果的目录                                              |
| save_freq           | 默认 1                                         | 保存频率，例如设置为4代表每隔4帧保存1次，默认每帧推理结果都保存 |
| save_total          | 默认 -1                                        | 保存总数，例如设置为10代表总共保存10帧结果，-1代表一直保存      |
| voxel_size          | 默认 0.02                                      | 每个占用网格的大小，单位m，0.02代表每个占用网格为2\*2\*2cm      |
| log_level           | 默认 INFO                                      | 日志等级，默认INFO                                              |

### 订阅话题

| 话题名称                       | 消息类型                     | 说明                                                                                                     |
| ------------------------------ | ---------------------------- | -------------------------------------------------------------------------------------------------------- |
| /image_combine_raw             | sensor_msgs::msg::Image      | 订阅双目图像，图像格式为NV12，图像要求上下排列，上面是左图，下面是右图，可以通过stereo_msg_topic参数修改 |
| /image_combine_raw/camera_info | sensor_msgs::msg::CameraInfo | 订阅相机内参，可选，不一定要有该话题，有该话题可以在保存结果时把相机参数一起保存                         |

### 发布话题

| 名称                       | 消息类型                      | 说明                                  |
| -------------------------- | ----------------------------- | ------------------------------------- |
| /dstereo_occnet_node/voxel | sensor_msgs::msg::PointCloud2 | 发布的占用网格数据，可以使用rviz2显示 |
