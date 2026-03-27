---
sidebar_position: 7
---

# 双目IMU相机

```mdx-code-block
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';
```

## 1. 功能介绍

地瓜双目IMU相机自带完整的标定参数，包括双目标定参数、IMU内参、双目和IMU之间的外参，用户无需额外标定即可直接使用。
利用这些参数，可以通过双目立体匹配算法计算高精度的深度图，实现实时的三维环境感知。
同时，相机的数据能应用于开源视觉惯性里程计（VIO）算法，如 OpenVINS 等，可计算相机位姿与轨迹信息。
适用于机器人导航、避障等应用，为开发者提供即插即用的深度感知与视觉惯性融合能力。

mipi相机代码仓库：https://github.com/D-Robotics/hobot_mipi_cam

## 2. 支持平台

| 平台                  | 系统支持              | 示例功能                            |
| --------------------- | --------------------- | ----------------------------------- |
| RDK X5, RDK X5 Module | Ubuntu 22.04 (Humble) | 启动双目相机，输出双目图像和IMU数据 |

## 3. 准备工作

### 3.1. RDK平台

1. RDK已烧录好RDK OS系统
2. RDK已成功安装TogetheROS.Bot
3. 确认PC机能够通过网络访问RDK

### 3.2. 系统和功能包版本

|                                       | 版本        | 查询方法                                        |
| ------------------------------------- | ----------- | ----------------------------------------------- |
| RDK X5系统镜像版本                    | 3.4.1及以上 | `cat /etc/version`                              |
| tros-humble-hobot-stereonet功能包版本 | 2.5.0及以上 | `apt list \| grep tros-humble-hobot-stereonet/` |
| tros-humble-mipi-cam功能包版本        | 2.5.0及以上 | `apt list \| grep tros-humble-mipi-cam/`        |

- 如果系统镜像版本不符合要求，请参考文档对应章节进行镜像烧录
- 如果功能包版本不符合要求，请执行以下指令进行更新：

```bash
sudo apt update
sudo apt install --only-upgrade tros-humble-hobot-stereonet
sudo apt install --only-upgrade tros-humble-mipi-cam
```

:::caution **注意**
**如果`sudo apt update`命令执行失败或报错，请查看[常见问题](/docs/08_FAQ/01_hardware_and_system.md)章节的`Q10: apt update 命令执行失败或报错如何处理？`解决。**
:::

## 4. 启动双目相机

### 4.1. 双目IMU相机

- 注意相机有黑色金属外壳和亚克力外壳两个版本，功能都是一样的

![LH_IMU_cam](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/03_boxs/function/image/box_adv/LH_IMU_cam.jpg)

### 4.2. 硬件连接

1. 模组背后开关设置为`EXT`模式，不要在`LPWM`模式
2. 相机的黑色杜邦线要连接RDK X5的`37`管脚，需要外部触发控制相机和IMU的时间同步，管脚定义可以查看[3.1.1 管脚定义与应用](../../../03_Basic_Application/01_40pin_user_sample/40pin_define.md)

![RDK_X5_LH_IMU_cam](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/03_boxs/function/image/box_adv/RDK_X5_LH_IMU_cam.png)

### 4.3. RDK X5配置

1. RDK X5上要进行相应的设置才能读取IMU数据，首先检查一下系统版本是否符合要求，需要`3.4.1`版本以上：

```bash
cat /etc/version
```

2. 在RDK X5上运行以下指令进行配置：

```bash
srpi-config
```

![LH_IMU_cam_config1](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/03_boxs/function/image/box_adv/LH_IMU_cam_config1.png)
![LH_IMU_cam_config2](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/03_boxs/function/image/box_adv/LH_IMU_cam_config2.png)
![LH_IMU_cam_config3](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/03_boxs/function/image/box_adv/LH_IMU_cam_config3.png)
![LH_IMU_cam_config4](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/03_boxs/function/image/box_adv/LH_IMU_cam_config4.png)

3. 重启后，确认目录`/sys/bus/iio/devices/`有`iio:device1`和`iio:device2`则代表配置成功

```bash
ll /sys/bus/iio/devices/
```

![LH_IMU_cam_config5](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/03_boxs/function/image/box_adv/LH_IMU_cam_config5.png)

### 4.4. 相机启动指令

1. 执行如下指令启动相机：

```bash
source /opt/tros/humble/setup.bash

ros2 launch mipi_cam mipi_cam_dual_channel.launch.py \
mipi_channel:=2 mipi_channel2:=0 \
mipi_lpwm_enable:=True mipi_frame_ts_type:=realtime \
mipi_image_width:=816 mipi_image_height:=960 \
mipi_image_framerate:=10.0 mipi_gdc_enable:=True \
mipi_out_format:=nv12 \
log_level:=info
```

参数解释：

- mipi_channel:=2 mipi_channel2:=0 调整左右目拼接顺序
- mipi_lpwm_enable:=True 开启LPWM硬件同步
- mipi_frame_ts_type:=realtime 时间戳采用系统时间
- mipi_image_width:=816 mipi_image_height:=960 调整图像分辨率，最高可设置为1088*1280
- mipi_image_framerate:=10.0 调整相机帧率，最高可调整为30.0
- mipi_gdc_enable:=True 开启GDC矫正会发布矫正后的双目图像，否则会发布带畸变的图像
- mipi_out_format:=nv12 设置图像格式，支持nv12/bgr8
- log_level:=info 日志等级，info等级可以打印标定参数，如果不需要显示那么多信息，则设置为warn

2. 程序运行成功会打印如下日志，包括相机全部的标定参数，目前双目标定采用鱼眼模式，可以参考[OpenCV fisheye](https://docs.opencv.org/4.x/db/d58/group__calib3d__fisheye.html)的介绍：

![LH_IMU_cam_run_success_log](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/03_boxs/function/image/box_adv/LH_IMU_cam_run_success_log.png)

3. 发布的话题如下：

```bash
ros2 topic list -v
```

![LH_IMU_cam_topic](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/03_boxs/function/image/box_adv/LH_IMU_cam_topic.png)

- /image_combine_raw 为上下拼接的双目图像，mipi_channel:=2 mipi_channel2:=0 参数控制拼接顺序
- /image_left_raw//image_right_raw 为左右目数据话题，mipi_channel:=2 mipi_channel2:=0 参数控制顺序
- /imu_data 为IMU数据话题，发布陀螺仪和加速度计数据

:::caution **注意**
**imu数据话题中`angular_velocity` 单位是 rad/s，`linear_acceleration`单位是 m/s²，重力加速度值是`9.81`**
:::

## 5. 启动双目深度算法

### 5.1. 启动指令

- 参考[双目深度算法](/docs/05_Robot_development/03_boxs/spatial/hobot_stereonet.md)，里面有双目算法的相关介绍和启动指令介绍
- 本相机启动指令为：

```bash
bash run_stereo.sh --mipi_rotation 0.0

# 参考对应文档查看具体的参数设置
```

### 5.2. 结果展示

- 启动后，可以在web端查看RGB图和深度图，在浏览器输入 http://ip:8000 (图中RDK IP是192.168.128.10)：

![LH_IMU_cam_DStereo](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/03_boxs/function/image/box_adv/LH_IMU_cam_DStereo.png)

## 6. 双目VIO算法（以OpenVINS为例）

### 6.1. 录制rosbag

目前还不支持在线运行VIO算法，需要录制rosbag后在PC上进行计算

- 运行指令

```bash
source /opt/tros/humble/setup.bash

ros2 pkg prefix mipi_cam

ros2 launch mipi_cam mipi_cam_dual_channel.launch.py \
mipi_channel:=2 mipi_channel2:=0 \
mipi_lpwm_enable:=True mipi_frame_ts_type:=realtime \
mipi_image_width:=816 mipi_image_height:=960 \
mipi_image_framerate:=10.0 mipi_gdc_enable:=False \
mipi_out_format:=nv12 \
log_level:=warn
```

- 主要录制imu数据话题和双目图像话题，由于RDK X5写入数据的性能有限，不建议录制太长时间

```bash
ros2 bag record /imu_data /image_combine_raw --max-cache-size 1073741824
```

- 将ros2 bag转为ros1格式，可以自行写程序实现

### 6.2. 准备VIO参数

配置OpenVINS的参数，在OpenVINS的config目录新建文件夹`drobotics_stereo_imu_cam`，创建以下3个文件，注意相机的标定文件需要根据上文的命令读取

- estimator_config.yaml

```yaml
%YAML:1.0 # need to specify the file type at the top!

verbosity: "INFO" # ALL, DEBUG, INFO, WARNING, ERROR, SILENT

use_fej: true # if first-estimate Jacobians should be used (enable for good consistency)
integration: "rk4" # discrete, rk4, analytical (if rk4 or analytical used then analytical covariance propagation is used)
use_stereo: true # if we have more than 1 camera, if we should try to track stereo constraints between pairs
max_cameras: 2 # how many cameras we have 1 = mono, 2 = stereo, >2 = binocular (all mono tracking)

calib_cam_extrinsics: true # if the transform between camera and IMU should be optimized R_ItoC, p_CinI
calib_cam_intrinsics: true # if camera intrinsics should be optimized (focal, center, distortion)
calib_cam_timeoffset: true # if timeoffset between camera and IMU should be optimized
calib_imu_intrinsics: false # if imu intrinsics should be calibrated (rotation and skew-scale matrix)
calib_imu_g_sensitivity: false # if gyroscope gravity sensitivity (Tg) should be calibrated

max_clones: 11 # how many clones in the sliding window
max_slam: 50 # number of features in our state vector
max_slam_in_update: 25 # update can be split into sequential updates of batches, how many in a batch
max_msckf_in_update: 40 # how many MSCKF features to use in the update
dt_slam_delay: 1 # delay before initializing (helps with stability from bad initialization...)

gravity_mag: 9.7887 # magnitude of gravity in this location

feat_rep_msckf: "GLOBAL_3D"
feat_rep_slam: "ANCHORED_MSCKF_INVERSE_DEPTH"
feat_rep_aruco: "ANCHORED_MSCKF_INVERSE_DEPTH"

# zero velocity update parameters we can use
# we support either IMU-based or disparity detection.
try_zupt: false
zupt_chi2_multipler: 0 # set to 0 for only disp-based
zupt_max_velocity: 0.1
zupt_noise_multiplier: 10
zupt_max_disparity: 0.5 # set to 0 for only imu-based
zupt_only_at_beginning: false

# ==================================================================
# ==================================================================

init_window_time: 2.0 # how many seconds to collect initialization information
init_imu_thresh: 1.5 # threshold for variance of the accelerometer to detect a "jerk" in motion
init_max_disparity: 10.0 # max disparity to consider the platform stationary (dependent on resolution)
init_max_features: 50 # how many features to track during initialization (saves on computation)

init_dyn_use: false # if dynamic initialization should be used
init_dyn_mle_opt_calib: false # if we should optimize calibration during intialization (not recommended)
init_dyn_mle_max_iter: 50 # how many iterations the MLE refinement should use (zero to skip the MLE)
init_dyn_mle_max_time: 0.05 # how many seconds the MLE should be completed in
init_dyn_mle_max_threads: 6 # how many threads the MLE should use
init_dyn_num_pose: 6 # number of poses to use within our window time (evenly spaced)
init_dyn_min_deg: 10.0 # orientation change needed to try to init

init_dyn_inflation_ori: 10 # what to inflate the recovered q_GtoI covariance by
init_dyn_inflation_vel: 100 # what to inflate the recovered v_IinG covariance by
init_dyn_inflation_bg: 10 # what to inflate the recovered bias_g covariance by
init_dyn_inflation_ba: 100 # what to inflate the recovered bias_a covariance by
init_dyn_min_rec_cond: 1e-12 # reciprocal condition number thresh for info inversion

init_dyn_bias_g: [ 0.0, 0.0, 0.0 ] # initial gyroscope bias guess
init_dyn_bias_a: [ 0.0, 0.0, 0.0 ] # initial accelerometer bias guess

# ==================================================================
# ==================================================================

record_timing_information: false # if we want to record timing information of the method
record_timing_filepath: "/tmp/traj_timing.txt" # https://docs.openvins.com/eval-timing.html#eval-ov-timing-flame

# if we want to save the simulation state and its diagional covariance
# use this with rosrun ov_eval error_simulation
save_total_state: false
filepath_est: "/tmp/ov_estimate.txt"
filepath_std: "/tmp/ov_estimate_std.txt"
filepath_gt: "/tmp/ov_groundtruth.txt"

# ==================================================================
# ==================================================================

# our front-end feature tracking parameters
# we have a KLT and descriptor based (KLT is better implemented...)
use_klt: true # if true we will use KLT, otherwise use a ORB descriptor + robust matching
num_pts: 200 # number of points (per camera) we will extract and try to track
fast_threshold: 20 # threshold for fast extraction (warning: lower threshs can be expensive)
grid_x: 5 # extraction sub-grid count for horizontal direction (uniform tracking)
grid_y: 5 # extraction sub-grid count for vertical direction (uniform tracking)
min_px_dist: 10 # distance between features (features near each other provide less information)
knn_ratio: 0.70 # descriptor knn threshold for the top two descriptor matches
track_frequency: 21.0 # frequency we will perform feature tracking at (in frames per second / hertz)
downsample_cameras: false # will downsample image in half if true
num_opencv_threads: 4 # -1: auto, 0-1: serial, >1: number of threads
histogram_method: "HISTOGRAM" # NONE, HISTOGRAM, CLAHE

# aruco tag tracker for the system
# DICT_6X6_1000 from https://chev.me/arucogen/
use_aruco: false
num_aruco: 1024
downsize_aruco: true

# ==================================================================
# ==================================================================

# camera noises and chi-squared threshold multipliers
up_msckf_sigma_px: 1
up_msckf_chi2_multipler: 1
up_slam_sigma_px: 1
up_slam_chi2_multipler: 1
up_aruco_sigma_px: 1
up_aruco_chi2_multipler: 1

# masks for our images
use_mask: false

# imu and camera spacial-temporal
# imu config should also have the correct noise values
relative_config_imu: "kalibr_imu_chain.yaml"
relative_config_imucam: "kalibr_imucam_chain.yaml"
```

- kalibr_imu_chain.yaml

```yaml
      
%YAML:1.0

imu0:
  T_i_b:
    - [1.0, 0.0, 0.0, 0.0]
    - [0.0, 1.0, 0.0, 0.0]
    - [0.0, 0.0, 1.0, 0.0]
    - [0.0, 0.0, 0.0, 1.0]
  accelerometer_noise_density: 0.02229489595390929  # [ m / s^2 / sqrt(Hz) ]   ( accel "white noise" )
  accelerometer_random_walk: 0.0001785433950802699  # [ m / s^3 / sqrt(Hz) ].  ( accel bias diffusion )
  gyroscope_noise_density: 0.001145986736669183     # [ rad / s / sqrt(Hz) ]   ( gyro "white noise" )
  gyroscope_random_walk: 1.2431490829218913e-05     # [ rad / s^2 / sqrt(Hz) ] ( gyro bias diffusion ) 
  rostopic: /imu/data
  time_offset: 0.0
  update_rate: 344.0
  # three different modes supported:
  # "calibrated" (same as "kalibr"), "kalibr", "rpng"
  model: "kalibr"
  # how to get from Kalibr imu.yaml result file:
  #   - Tw is imu0:gyroscopes:M:
  #   - R_IMUtoGYRO: is imu0:gyroscopes:C_gyro_i:
  #   - Ta is imu0:accelerometers:M:
  #   - R_IMUtoACC not used by Kalibr
  #   - Tg is imu0:gyroscopes:A:
  Tw:
    - [ 1.0, 0.0, 0.0 ]
    - [ 0.0, 1.0, 0.0 ]
    - [ 0.0, 0.0, 1.0 ]
  R_IMUtoGYRO:
    - [ 1.0, 0.0, 0.0 ]
    - [ 0.0, 1.0, 0.0 ]
    - [ 0.0, 0.0, 1.0 ]
  Ta:
    - [ 1.0, 0.0, 0.0 ]
    - [ 0.0, 1.0, 0.0 ]
    - [ 0.0, 0.0, 1.0 ]
  R_IMUtoACC:
    - [ 1.0, 0.0, 0.0 ]
    - [ 0.0, 1.0, 0.0 ]
    - [ 0.0, 0.0, 1.0 ]
  Tg:
    - [ 0.0, 0.0, 0.0 ]
    - [ 0.0, 0.0, 0.0 ]
    - [ 0.0, 0.0, 0.0 ]
```

- kalibr_imucam_chain.yaml

```yaml
%YAML:1.0
cam0:
  T_cam_imu:
    - [-0.9999730211612913, 0.0011654864078604154, 0.007252488606936615, 0.05855576022314668]
    - [0.0011859186262073965, 0.9999953385940187, 0.002813607514727948, 0.002858928245212033]
    - [-0.00724917557882737, 0.0028221324681899397, -0.9999697420531085, -0.005490310834812385]
    - [0.0, 0.0, 0.0, 1.0]
  cam_overlaps: [1]
  camera_model: pinhole
  distortion_coeffs: [-0.029225109913193572, 0.02082403492287568, -0.03194158971070967, 0.0165934134408496]
  distortion_model: equidistant
  intrinsics: [497.81111262347383, 497.8055103598601, 396.48820025924294, 463.4451903675188]
  resolution: [816, 960]
  rostopic: /left_camera/image_raw
  timeshift_cam_imu: 0.005901386399303203
cam1:
  T_cam_imu:
    - [-0.9999845146989352, 0.004550487081340542, 0.003203658792316898, -0.010601870574768642]
    - [0.0045684145407885015, 0.9999738226044781, 0.005611033271290525, 0.002251517696825047]
    - [-0.0031780419944595945, 0.00562558202416158, -0.9999791262201281, -0.005008993420470941]
    - [0.0, 0.0, 0.0, 1.0]
  T_cn_cnm1:
    - [0.9999860743077356, 0.0033735794462250975, 0.004058343544320085, -0.06914417862245166]
    - [-0.003362141436934531, 0.9999903663337931, -0.0028219221399547337, -0.00042600348830910853]
    - [-0.0040678244261232255, 0.0028082381177397444, 0.9999877832269312, 0.0007114163409117424]
    - [0.0, 0.0, 0.0, 1.0]
  cam_overlaps: [0]
  camera_model: pinhole
  distortion_coeffs: [-0.028463396839828604, 0.012205852066252196, -0.01306188175421103, 0.001536051968099967]
  distortion_model: equidistant
  intrinsics: [494.7591785888601, 494.9156071869387, 384.67971500453064, 488.34269230328925]
  resolution: [816, 960]
  rostopic: /right_camera/image_raw
  timeshift_cam_imu: 0.00591541095880247
```

### 6.3. 启动OpenVINS

- 开启3个终端，分别执行

```bash
# 启动openvins
roslaunch ov_msckf subscribe.launch config:=drobotics_stereo_imu_cam verbosity:=DEBUG \
dosave:=true path_est:=~/openvins_traj.txt
```

```bash
# 启动rviz
rosrun rviz rviz -d <OpenVINS目录>/open_vins/ov_msckf/launch/display.rviz
```

```bash
# 播放采集的rosbag数据
rosbag play xxx_ros1.bag
```

![LH_IMU_cam_OpneVINS](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/03_boxs/function/image/box_adv/LH_IMU_cam_OpneVINS.gif)

