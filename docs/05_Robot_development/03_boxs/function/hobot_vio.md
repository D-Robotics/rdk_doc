---
sidebar_position: 12
---
# 视觉惯性里程计算法

```mdx-code-block
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';
```

## 功能介绍

视觉惯性里程计（Visual Inertial Odometry，VIO）是融合相机与惯性测量单元（Inertial Measurement Unit，IMU）数据实现机器人定位的算法。VIO定位算法具有成本低、适用环境广等优点，在室外环境下能够有效弥补卫星定位中遮挡、多路径干扰等失效场景。优秀、鲁棒的VIO算法是实现室外高精度导航定位的关键。
![](/../static/img/05_Robot_development/03_boxs/function/image/box_adv/hobot_vio_rviz.jpeg)
代码仓库： (https://github.com/D-Robotics/hobot_vio.git)

## 支持平台

| 平台   | 运行方式     | 示例功能                                                     |
| ------ | ------------ | ------------------------------------------------------------ |
| RDK X3, RDK X3 Module | Ubuntu 20.04 (Foxy), Ubuntu 22.04 (Humble) | 使用realsense的图像和IMU数据作为算法输入，算法输出机器人运动轨迹，轨迹可在PC的rviz2上可视化 |
| RDK Ultra | Ubuntu 20.04 (Foxy) | 使用realsense的图像和IMU数据作为算法输入，算法输出机器人运动轨迹，轨迹可在PC的rviz2上可视化 |

## 准备工作

1. RDK已烧录好Ubuntu 20.04/Ubuntu 22.04系统镜像。

2. RDK已成功安装TogetheROS.Bot和Realsense的ROS2 Package。

3. realsense相机，连接到RDK的USB 3.0接口。

4. 确认PC机能够通过网络访问RDK。

## 使用介绍

算法订阅realsense相机的图像和IMU数据作为算法的输入，经过计算得到相机的轨迹信息，并通过ROS2的话题机制发布相机的运动轨迹，轨迹结果可在PC的rviz2软件查看。算法的输入和输出topic如下表所示：

### 输入topic

| 参数名      | 类型        | 解释                                                         | 是否必须 | 默认值                                             |
| ----------- | ----------- | ------------------------------------------------------------ | -------- | -------------------------------------------------- |
| path_config | std::string | vio算法配置文件路径                                          | 是       | `/opt/tros/${TROS_DISTRO}/lib/hobot_vio/config/realsenseD435i.yaml` |
| image_topic | std::string | vio算法订阅的图像数据话题名                                  | 是       | /camera/infra1/image_rect_raw                      |
| imu_topic   | std::string | vio算法订阅的IMU数据话题名                                   | 是       | /camera/imu                                        |
| sample_gap  | std::string | vio算法处理频率，1表示每帧图像都会参与轨迹计算，2表示每两帧图像计算一次，依此类推 | 是       | 2                                                  |

### 输出topic

| topic名                      | 类型                | 解释                        |
| ---------------------------- | ------------------- | --------------------------- |
| horizon_vio/horizon_vio_path | nav_msgs::msg::Path | vio算法输出的机器人运动轨迹 |

启动命令：


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
ros2 launch hobot_vio hobot_vio.launch.py 
```

## 结果分析

在X3上启动算法示例后在运行终端输出如下信息，首先启动realsense节点发布图像和IMU数据，随后算法进入初始化流程，此时等待用户平移相机完成初始化，初始化完成后算法开始输出定位坐标：

```text
[INFO] [launch]: All log files can be found below /root/.ros/log/2023-07-07-19-48-31-464088-ubuntu-562910
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [hobot_vio-1]: process started with pid [563077]
[INFO] [ros2 launch realsense2_camera rs_launch.py  depth_module.profile:=640x480x30 enable_depth:=false enable_color:=false enable_gyro:=true enable_accel:=true enable_sync:=true gyro_fps:=200 accel_fps:=200 unite_imu_method:=2 enable_infra1:=true-2]: process started with pid [563081]
[hobot_vio-1] T_CtoI:
[hobot_vio-1]    0.999934   0.0103587   0.0049969   0.0270761
[hobot_vio-1]  -0.0104067    0.999899  0.00967935 -0.00272628
[hobot_vio-1] -0.00489613 -0.00973072    0.999941  -0.0518149
[hobot_vio-1]           0           0           0           1
[hobot_vio-1] system use_rtk_: 0
[hobot_vio-1] [static initializer] not enough imu readings
[hobot_vio-1] [static initializer] not enough imu readings
[hobot_vio-1] [static initializer] IMU belows th 0.011508, 0.00274453 < 0.5, 0
[hobot_vio-1] [static initializer] IMU belows th 0.0105996, 0.00273085 < 0.5, 0
[hobot_vio-1] [static initializer] IMU belows th 0.00964632, 0.00280866 < 0.5, 0
[hobot_vio-1] [static initializer] IMU belows th 0.00892132, 0.00279346 < 0.5, 0
[hobot_vio-1] [static initializer] IMU belows th 0.00816016, 0.00281761 < 0.5, 0
[hobot_vio-1] [static initializer] IMU belows th 0.00776753, 0.00277049 < 0.5, 0
[hobot_vio-1] [static initializer] IMU belows th 0.00744219, 0.00274874 < 0.5, 0
[hobot_vio-1] [static initializer] IMU belows th 0.420251, 0.36058 < 0.5, 0
[hobot_vio-1] HorizonVIO Successfully initialized!
[hobot_vio-1] [WARN] [1688730518.534178615] [horizon_vio_node]: Localization position[x, y, z]: [0.0225533, -0.0504654, 0.00943574]
[hobot_vio-1] [WARN] [1688730518.534634139] [horizon_vio_node]: Image time 1688730518.314490318
[hobot_vio-1] [WARN] [1688730518.621440869] [horizon_vio_node]: Localization position[x, y, z]: [0.0231779, -0.0533648, 0.00787081]
[hobot_vio-1] [WARN] [1688730518.621558739] [horizon_vio_node]: Image time 1688730518.380982161
[hobot_vio-1] [WARN] [1688730518.743525086] [horizon_vio_node]: Localization position[x, y, z]: [0.0290396, -0.0610474, 0.0106718]
[hobot_vio-1] [WARN] [1688730518.743637249] [horizon_vio_node]: Image time 1688730518.447472572
[hobot_vio-1] [WARN] [1688730518.866076119] [horizon_vio_node]: Localization position[x, y, z]: [0.0381324, -0.0737757, 0.0164843]
[hobot_vio-1] [WARN] [1688730518.866186156] [horizon_vio_node]: Image time 1688730518.513962030
[hobot_vio-1] SLAM feats: 0
[hobot_vio-1] KF feats: 338
[hobot_vio-1] 132.853 ms all consumed
[hobot_vio-1] travel(m): 0.000
[hobot_vio-1] [WARN] [1688730519.002002975] [horizon_vio_node]: Localization position[x, y, z]: [0.05018, -0.088422, 0.0240244]
[hobot_vio-1] [WARN] [1688730519.002130095] [horizon_vio_node]: Image time 1688730518.580449104
[hobot_vio-1] SLAM feats: 0
[hobot_vio-1] KF feats: 31
[hobot_vio-1] 142.996 ms all consumed
[hobot_vio-1] travel(m): 0.014
[hobot_vio-1] [WARN] [1688730519.146149433] [horizon_vio_node]: Localization position[x, y, z]: [0.0167176, -0.0189649, 0.0588413]
[hobot_vio-1] [WARN] [1688730519.146279428] [horizon_vio_node]: Image time 1688730518.646935701
[hobot_vio-1] SLAM feats: 0
[hobot_vio-1] KF feats: 26
[hobot_vio-1] 96.911 ms all consumed
[hobot_vio-1] travel(m): 0.025
[hobot_vio-1] [WARN] [1688730519.244168068] [horizon_vio_node]: Localization position[x, y, z]: [0.000805884, 0.0134815, 0.0730707]
[hobot_vio-1] [WARN] [1688730519.244270439] [horizon_vio_node]: Image time 1688730518.713421583
[hobot_vio-1] SLAM feats: 0
[hobot_vio-1] KF feats: 23
[hobot_vio-1] 52.470 ms all consumed
[hobot_vio-1] travel(m): 0.034
[hobot_vio-1] [WARN] [1688730519.297642444] [horizon_vio_node]: Localization position[x, y, z]: [0.00226324, 0.0120054, 0.0796328]
[hobot_vio-1] [WARN] [1688730519.297738190] [horizon_vio_node]: Image time 1688730518.779906034
[hobot_vio-1] SLAM feats: 0
[hobot_vio-1] KF feats: 33
[hobot_vio-1] 47.407 ms all consumed
[hobot_vio-1] travel(m): 0.042
```

