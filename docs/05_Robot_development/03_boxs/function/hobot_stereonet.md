---
sidebar_position: 13
---

# 双目深度算法

## 功能介绍

双目深度估计算法是使用地平线[OpenExplorer](https://developer.horizon.ai/api/v1/fileData/horizon_j5_open_explorer_cn_doc/hat/source/examples/stereonet.html)
在[SceneFlow](https://lmb.informatik.uni-freiburg.de/resources/datasets/SceneFlowDatasets.en.html)
数据集上训练出来的`StereoNet`模型。

算法输入为双目图像数据，分别是左右视图。算法输出为左视图的深度。

此示例使用mipi双目相机作为图像数据输入源，利用BPU进行算法推理，发布包含双目图像左图和感知结果的话题消息，在PC端rviz2上渲染算法结果。

## 支持平台

| 平台   | 运行方式              | 示例功能                                      |
| ------ | --------------------- | --------------------------------------------- |
| RDK X5 | Ubuntu 22.04 (Humble) | · 启动双目相机、推理出深度结果，并在Web端显示 |

## 物料清单

- 双目相机，目前支持多款mipi相机和usb相机
- 没有双目相机的情况下，用户也可以采用离线数据进行回灌获取深度估计结果

## 使用方法

### 功能安装和更新

- 在运行双目深度算法之前，需要确保系统镜像版本在`3.1.1`版本之上，查询系统版本的命令如下：

```bash
cat /etc/version
```

![os_version](/../static/img/05_Robot_development/03_boxs/function/image/box_adv/os_version.png)

- 如果系统版本不符合要求，请参考文档`1.2`章节进行镜像烧录

- 此外，还需要确保`tros-humble-mipi-cam`功能包在2.3.7版本（及以上）、`tros-humble-hobot-stereonet`
  功能包在2.4.0版本（及以上），查询功能包版本的命令如下：

```bash
apt list | grep tros-humble-mipi-cam
apt list | grep tros-humble-hobot-stereonet
```

![stereonet_version](/../static/img/05_Robot_development/03_boxs/function/image/box_adv/stereonet_version.png)

- 如果功能包版本不符合要求，需要进行升级，在RDK系统的终端中运行如下指令，即可更新：

```bash
apt update
apt upgrade
```

- **注意**：在一些旧镜像版本中，可能存在`tros-humble-stereonet-model`功能包，需要对其进行卸载后，才能对功能包进行更新，在RDK系统的终端中运行如下指令：

```bash
# 确认系统存在tros-humble-stereonet-model功能包
apt list | grep tros-humble-hobot-stereonet-model
# 卸载
sudo apt-get remove tros-humble-stereonet-model
# 如果卸载失败，则执行强制删除命令
# sudo dpkg --remove --force-all tros-humble-stereonet-model
# 安装新的包
sudo apt install -y tros-humble-hobot-stereonet
```

### 双目算法的版本

目前双目算法更新到V3版本，相比于之前的版本推理耗时更低

### 启动双目图像发布、算法推理和图像可视化

双目深度算法支持多款相机，mipi相机和usb相机都可以支持，启动命令有一些区别，具体启动命令如下：

<span style={{ color: 'red' }}> 注意：通过ssh连接RDK X5请用`root`用户登录，或者切换到`root`用户，不然某些命令执行可能权限不够 </span>

![os_user](/../static/img/05_Robot_development/03_boxs/function/image/box_adv/os_user.png)

#### (1) 搭配RDK X5官方双目摄像头启动

- RDK X5官方双目摄像头如图所示：

![RDK_Stereo_Cam_230ai](/../static/img/05_Robot_development/03_boxs/function/image/box_adv/RDK_Stereo_Cam_230ai.png)

- 安装方式如图所示，接线请勿接反，会导致左右图对调，双目算法运行错误：

![RDK_X5_230ai](/../static/img/05_Robot_development/03_boxs/function/image/box_adv/RDK_X5_230ai.png)

- 确认相机连接是否正常，通过ssh连接RDK X5，执行以下命令，如果输出如图所示结果，则代表相机连接正常：

```bash
i2cdetect -r -y 4
i2cdetect -r -y 6
```

![i2cdetect_230ai](/../static/img/05_Robot_development/03_boxs/function/image/box_adv/i2cdetect_230ai.png)

- 启动双目算法，通过ssh连接RDK X5，执行以下命令：

```bash
# 配置tros.b humble环境
source /opt/tros/humble/setup.bash

# 启动双目模型launch文件，其包含了算法和双目相机节点的启动
ros2 launch hobot_stereonet stereonet_model_web_visual_v3.launch.py \
need_rectify:=False mipi_image_width:=640 mipi_image_height:=352 \
height_min:=-10.0 height_max:=10.0 pc_max_depth:=5.0
```

- 出现如下日志表示双目算法启动成功，`fx/fy/cx/cy/base_line`是相机内参，如果深度图正常，但估计出来的距离有偏差，可能是相机内参存在问题：

![stereonet_run_success](/../static/img/05_Robot_development/03_boxs/function/image/box_adv/stereonet_run_success.png)

- 通过网页端查看深度图，在浏览器输入 http://ip:8000 (图中RDK X5 ip是192.168.1.100)：

![web_depth_visual](/../static/img/05_Robot_development/03_boxs/function/image/box_adv/web_depth_visual.png)

- 通过rviz查看点云，需要用户具备一定的ROS2基础，将PC和RDK
  X5配置到同一个网段，能够相互ping通，订阅双目模型节点发布的相关话题，才可以在rviz中显示点云，注意rviz中需要做如下配置：

![stereonet_rviz](/../static/img/05_Robot_development/03_boxs/function/image/box_adv/stereonet_rviz.png)

- 如果用户想保存深度估计结果，可以添加如下参数实现，`save_image_all`打开保存开关，`save_freq`控制保存频率，`save_dir`控制保存的目录（如果目录不存在会自动创建），`save_total`控制保存的总数。程序运行将会保存**相机内参、左右图、视差图、深度图、可视化图**：

```bash
# 配置tros.b humble环境
source /opt/tros/humble/setup.bash

# 启动双目模型launch文件，其包含了算法和双目相机节点的启动
ros2 launch hobot_stereonet stereonet_model_web_visual_v3.launch.py \
need_rectify:=False mipi_image_width:=640 mipi_image_height:=352 \
height_min:=-10.0 height_max:=10.0 pc_max_depth:=5.0 \
save_image_all:=True save_freq:=4 save_dir:=./stereonet_result save_total:=10
```

![stereonet_save](/../static/img/05_Robot_development/03_boxs/function/image/box_adv/stereonet_save.png)

![stereonet_save_files](/../static/img/05_Robot_development/03_boxs/function/image/box_adv/stereonet_save_files.png)

#### (2) 本地图片离线回灌

- 如果想利用本地图片评估算法效果，可以使用下列命令指定算法运行模式、图像数据地址以及相机内参，同时要保证图像数据经过去畸变、极线对齐。图片的格式如下图所示，第一张左目图像的命名为left000000.png，第二张左目图像的命名为left000001.png，以此类推。对应的第一张右目图像的命名为right000000.png，第二张右目图像的命名为right000001.png，以此类推。算法按序号遍历图像，直至图像全部计算完毕：

![stereonet_rdk](/../static/img/05_Robot_development/03_boxs/function/image/box_adv/image_format.png)

- 算法离线运行方式如下，通过ssh连接RDK X5，执行以下命令：

```shell
# 配置tros.b humble环境
source /opt/tros/humble/setup.bash

# 启动双目模型launch文件，注意相机参数的设置，需要手动输入矫正后参数
ros2 launch hobot_stereonet stereonet_model_web_visual.launch.py \
stereonet_model_file_path:=/opt/tros/humble/share/hobot_stereonet/config/DStereoV2.2.bin postprocess:=v3 \
use_local_image:=True local_image_path:=./stereonet_result \
need_rectify:=False camera_fx:=216.696533 camera_fy:=216.696533 camera_cx:=335.313477 camera_cy:=182.961578 base_line:=0.070943 \
height_min:=-10.0 height_max:=10.0 pc_max_depth:=5.0 save_image_all:=True
```

参数含义如下：

| 名称             | 参数值                                 | 说明                 |
| ---------------- | -------------------------------------- | -------------------- |
| use_local_image  | 设置为True                             | 是否启用图片回灌模式 |
| local_image_path | 设置为离线数据目录                     | 回灌图像的地址目录   |
| camera_fx        | 设置为相机矫正后内参fx                 | 相机内参             |
| camera_fy        | 设置为相机矫正后内参fy                 | 相机内参             |
| camera_cx        | 设置为相机矫正后内参cx                 | 相机内参             |
| camera_cy        | 设置为相机矫正后内参cy                 | 相机内参             |
| base_line        | 设置为相机矫正后基线                   | 基线距离             |
| height_min       | 一般设置为-10.0m                       | 点云最小高度         |
| height_max       | 一般设置为+10.0m                       | 点云最大高度         |
| pc_max_depth     | 一般设置为+5.0m                        | 点云最大深度         |
| save_image_all   | 如果只是web看一下结果，可以设置为False | 是否保存结果         |

- 算法运行成功后，同样可以通过网页端和rviz显示实时渲染数据，参考上文，离线运行的结果将会保存在`离线数据目录下的result子目录中`，同样会保存**相机内参、左右图、视差图、深度图、可视化图**

#### (3) 搭配ZED双目摄像头启动

- ZED双目摄像头如图所示：

![zed_cam](/../static/img/05_Robot_development/03_boxs/function/image/box_adv/zed_cam.png)

- ZED的启动依赖于`hobot_zed_cam`功能包，确认功能包已经安装，版本在2.3.1及以上，通过ssh连接RDK X5，执行以下命令：

```shell
apt list | grep tros-humble-hobot-zed-cam
```

![zed_cam_version](/../static/img/05_Robot_development/03_boxs/function/image/box_adv/zed_cam_version.png)

- 将ZED相机通过USB连接RDK X5，然后启动双目算法，通过ssh连接RDK X5，执行以下命令：

```shell
ros2 launch hobot_zed_cam test_stereo_zed_rectify.launch.py \
resolution:=720p dst_width:=640 dst_height:=352 \
stereonet_model_file_path:=/opt/tros/humble/share/hobot_stereonet/config/DStereoV2.2.bin postprocess:=v3 \
render_type:=1 height_min:=-10.0 height_max:=10.0 pc_max_depth:=5.0 
```

- 通过网页端查看深度图，在浏览器输入 http://ip:8000 ，更多可视化相关的内容请参考前文

## 接口说明

### 订阅话题

| 名称               | 消息类型                | 说明                                                   |
| ------------------ | ----------------------- | ------------------------------------------------------ |
| /image_combine_raw | sensor_msgs::msg::Image | 双目相机节点发布的左右目拼接图像话题，用于模型推理深度 |

### 发布话题

| 名称                                 | 消息类型                      | 说明                                     |
| ------------------------------------ | ----------------------------- | ---------------------------------------- |
| /StereoNetNode/stereonet_pointcloud2 | sensor_msgs::msg::PointCloud2 | 发布的点云深度话题                       |
| /StereoNetNode/stereonet_depth       | sensor_msgs::msg::Image       | 发布的深度图像，像素值为深度，单位为毫米 |
| /StereoNetNode/stereonet_visual      | sensor_msgs::msg::Image       | 发布的比较直观的可视化渲染图像           |

### 参数

| 名称                | 参数值                  | 说明                                                                                       |
| ------------------- | ----------------------- | ------------------------------------------------------------------------------------------ |
| stereo_image_topic  | 默认 /image_combine_raw | 订阅双目图像消息的话题名                                                                   |
| need_rectify        | 默认 True               | 是否对双目数据做基线对齐和去畸变，相机内外参在config/stereo.yaml文件内指定                 |
| stereo_combine_mode | 默认 1                  | 左右目图像往往拼接在一张图上再发布出去，1为上下拼接，0为左右拼接，指示双目算法如何拆分图像 |
| height_min          | 默认 -0.2               | 过滤掉相机垂直方向上高度小于height_min的点，单位为米                                       |
| height_max          | 默认 999.9              | 过滤掉相机垂直方向上高度大于height_max的点，单位为米                                       |
| KMean               | 默认 10                 | 过滤稀疏离群点时每个点的临近点的数目，统计每个点与周围最近10个点的距离                     |
| stdv                | 默认 0.01               | 过滤稀疏离群点时判断是否为离群点的阈值，将标准差的倍数设置为0.01                           |
| leaf_size           | 默认 0.05               | 设置点云的单位密度，表示半径0.05米的三维球内只有一个点                                     |

## 算法耗时

当log等级设置为debug时，程序会打印出算法各阶段耗时情况，供用户debug算法性能瓶颈。

```shell
ros2 launch hobot_stereonet stereonet_model.launch.py \
stereo_image_topic:=/image_combine_raw stereo_combine_mode:=1 need_rectify:=True log_level:=debug
```

![stereonet_rdk](/../static/img/05_Robot_development/03_boxs/function/image/box_adv/consume.png)

## 注意事项

1. 模型的输入尺寸为宽：640，高352，相机发布的图像分辨率应为640x352
2. 如果双目相机发布图像的格式为NV12，那么双目图像的拼接方式必须为上下拼接
