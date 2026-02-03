---
sidebar_position: 5
---

# 双目深度算法

```mdx-code-block
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';
```

## 1. 功能介绍

地瓜双目深度估计算法输入为双目图像数据，输出为左视图对应的视差图和深度图。算法借鉴IGEV网络，采用了GRU架构，具有较好的数据泛化性和较高的推理效率。

双目算法代码仓库：https://github.com/D-Robotics/hobot_stereonet

mipi相机代码仓库：https://github.com/D-Robotics/hobot_mipi_cam

zed相机代码仓库：https://github.com/D-Robotics/hobot_zed_cam

双目算法讲解：[直播回放 | 基于RDK X5的AI双目算法部署实战](https://www.bilibili.com/video/BV1KdEjzREMz/?share_source=copy_web&vd_source=deb3551e36cc4b1c1020033ad17c564b)

## 2. 支持平台

| 平台                  | 系统支持              | 示例功能                                    |
| --------------------- | --------------------- | ------------------------------------------- |
| RDK X5, RDK X5 Module | Ubuntu 22.04 (Humble) | 启动双目相机，推理出深度结果，并在Web端显示 |
| RDK S100, RDK S100P   | Ubuntu 22.04 (Humble) | 启动双目相机，推理出深度结果，并在Web端显示 |
| RDK S600              | Ubuntu 24.04 (Jazzy)  | 启动双目相机，推理出深度结果，并在Web端显示 |

## 3. 模型版本

| 平台 | 算法版本 | 量化方式 | 输入尺寸    | 推理帧率(fps) | 算法特性                       |
| ---- | -------- | -------- | ----------- | ------------- | ------------------------------ |
| X5   | V2.0     | int16    | 640x352x3x2 | 15            | 精度较高、帧率较低             |
| X5   | V2.1     | int16    | 640x352x3x2 | 15            | 有置信度输出                   |
| X5   | V2.2     | int8     | 640x352x3x2 | 23            | 精度较低、帧率较高             |
| X5   | V2.3     | int8     | 640x352x3x2 | 27            | 帧率进一步提升                 |
| X5   | V2.4     | int16    | 640x352x3x2 | 15            | 加入更多数据训练               |
| X5   | V2.4     | int8     | 640x352x3x2 | 23            | 加入更多数据训练               |
| S100 | V2.1     | int16    | 640x352x3x2 | 53            | 有置信度输出                   |
| S100 | V2.4     | int16    | 640x352x3x2 | 53            | 有置信度输出，加入更多数据训练 |

## 4. 准备工作

### 4.1. RDK平台

1. RDK已烧录好RDK OS系统
2. RDK已成功安装TogetheROS.Bot
3. 如果需要在线推理，请准备好双目相机，目前支持多款MIPI相机、ZED mini/2i USB相机
4. 如果需要离线推理，请准备好双目图像数据
5. 确认PC机能够通过网络访问RDK

### 4.2. 系统和功能包版本

|                                       | 版本             | 查询方法                                        |
| ------------------------------------- | ---------------- | ----------------------------------------------- |
| RDK X5系统镜像版本                    | 3.3.3及以上      | `cat /etc/version`                              |
| RDK S100系统镜像版本                  | 4.0.2-Beta及以上 | `cat /etc/version`                              |
| tros-humble-hobot-stereonet功能包版本 | 2.5.0及以上      | `apt list \| grep tros-humble-hobot-stereonet/` |
| tros-humble-mipi-cam功能包版本        | 2.3.13及以上     | `apt list \| grep tros-humble-mipi-cam/`        |
| tros-humble-hobot-zed-cam功能包版本   | 2.3.3及以上      | `apt list \| grep tros-humble-hobot-zed-cam/`   |

- 如果系统镜像版本不符合要求，请参考文档对应章节进行镜像烧录
- 如果功能包版本不符合要求，请执行以下指令进行更新：

```bash
sudo apt update
sudo apt install --only-upgrade tros-humble-hobot-stereonet
sudo apt install --only-upgrade tros-humble-mipi-cam
sudo apt install --only-upgrade tros-humble-hobot-zed-cam
```

:::caution **注意**
**如果`sudo apt update`命令执行失败或报错，请查看[常见问题](/docs/08_FAQ/01_hardware_and_system.md)章节的`Q10: apt update 命令执行失败或报错如何处理？`解决。**
:::

## 5. 算法启动

### 5.1. 注意事项（必看！！！）

:::caution **注意**
**请用`root`用户执行文档中的命令，其它用户执行可能权限不够，造成一些不必要的错误。**
:::

![os_user](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/03_boxs/function/image/box_adv/os_user.png)

### 5.2. MIPI双目相机安装

#### (1) 230AI MIPI双目相机

- RDK官方230AI MIPI双目相机如图所示：

![RDK_Stereo_Cam_230ai](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/03_boxs/function/image/box_adv/RDK_Stereo_Cam_230ai.png)

<p style={{ color: 'red' }}> 注意：请检查相机背面丝印印有CDPxxx-V3，确认相机是V3版本 </p>

- 安装方式如图所示：

![RDK_X5_230ai](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/03_boxs/function/image/box_adv/RDK_X5_230ai.png)

![RDK_S100_230ai](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/03_boxs/function/image/box_adv/RDK_S100_230ai.png)

#### (2) 132GSMIPI双目相机

- RDK官方132GS MIPI双目相机如图所示：

![RDK_Stereo_Cam_132gs](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/03_boxs/function/image/box_adv/RDK_Stereo_Cam_132gs.png)

- 安装方式如图所示：

![RDK_X5_132gs](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/03_boxs/function/image/box_adv/RDK_X5_132gs.png)


### 5.3. 在线启动指令

#### (1) 确认双目相机I2C信号正常

- 确认230AI双目相机I2C信号是否正常，通过ssh连接RDK，执行以下命令，如果输出0x30、0x32、0x50等地址，则代表相机连接正常：

```bash
# RDK X5
i2cdetect -r -y 4
i2cdetect -r -y 6

# RDK S100
i2cdetect -r -y 1
i2cdetect -r -y 2
```

![i2cdetect_230ai](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/03_boxs/function/image/box_adv/i2cdetect_230ai.png)


- 确认132GS双目相机I2C信号是否正常，通过ssh连接RDK，执行以下命令，如果输出0x32、0x33、0x50等地址，则代表相机连接正常：

```bash
# RDK X5
i2cdetect -r -y 4
i2cdetect -r -y 6

# RDK S100
i2cdetect -r -y 1
i2cdetect -r -y 2
```

![i2cdetect_132gs](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/03_boxs/function/image/box_adv/i2cdetect_132gs.png)

:::caution **注意**
**如果I2C信号检测不到，相机无法正常工作**
:::


#### (2) 创建启动脚本

- 方法1：如果已经安装tros-humble-hobot-stereonet功能包，则可以直接复制

```bash
cp -rv /opt/tros/humble/share/hobot_stereonet/script/run_stereo.sh ./
```

- 方法2：手动创建启动脚本`run_stereo.sh`，写入以下内容

```bash
#!/bin/bash
source /opt/tros/humble/setup.bash

ros2 pkg prefix mipi_cam
ros2 pkg prefix hobot_stereonet

rm -rfv performance_*.txt

# stereonet version
stereonet_version=v2.4_int16

# node name
stereo_node_name=StereoNetNode

# uncertainty
uncertainty_th=-0.10

# topic
stereo_image_topic=/image_combine_raw
camera_info_topic=/image_right_raw/camera_info
depth_image_topic="~/stereonet_depth"
depth_camera_info_topic="~/stereonet_depth/camera_info"
pointcloud2_topic="~/stereonet_pointcloud2"
rectify_left_image_topic="~/rectify_left_image"
rectify_right_image_topic="~/rectify_right_image"
publish_rectify_bgr=False
origin_left_image_topic="~/origin_left_image"
origin_right_image_topic="~/origin_right_image"
publish_origin_enable=True
visual_image_topic="~/stereonet_visual"

# mipi cam
use_mipi_cam=True
mipi_image_width=640
mipi_image_height=352
mipi_image_framerate=30.0
mipi_frame_ts_type=realtime
mipi_gdc_enable=True
mipi_lpwm_enable=True
mipi_rotation=90.0
mipi_channel=2
mipi_channel2=0
mipi_cal_rotation=0.0

# calib
calib_method=none
stereo_calib_file_path=calib.yaml

# render
render_type=distance
render_perf=True
render_max_disp=80
render_z_near=-1.0
render_z_range=3.0

# speckle filter
speckle_filter_enable=False
max_speckle_size=100
max_disp_diff=1.0

# pointcloud
pointcloud_height_min=-5.0
pointcloud_height_max=5.0
pointcloud_depth_max=5.0

# pcl filter
pcl_filter_enable=False
grid_size=0.1
grid_min_point_count=5

# thread
infer_thread_num=2
save_thread_num=4
max_save_task=50

# save
save_result_flag=False
save_dir=./result
save_freq=1
save_total=-1
save_stereo_flag=True
save_origin_flag=False
save_disp_flag=True
save_uncert_flag=False
save_depth_flag=True
save_visual_flag=True
save_pcd_flag=False

# local image
use_local_image_flag=False
local_image_dir=./offline
image_sleep=0

# camera intrinsic
camera_cx=0.0
camera_cy=0.0
camera_fx=0.0
camera_fy=0.0
baseline=0.0
doffs=0.0

# mask
left_img_mask_enable=False

# epipolar
epipolar_mode=False
epipolar_img=origin
chessboard_per_rows=20
chessboard_per_cols=11
chessboard_square_size=0.06

# web
stereonet_pub_web=True
codec_sub_topic=/$stereo_node_name/stereonet_visual
codec_in_format=bgr8
codec_pub_topic=/image_jpeg
websocket_image_topic=/image_jpeg
websocket_channel=0

while [[ $# -gt 0 ]]; do
  case $1 in
    # stereonet version
    --stereonet_version) stereonet_version=$2; shift 2 ;;

    # node name
    --stereo_node_name) stereo_node_name=$2; shift 2 ;;

    # uncertainty
    --uncertainty_th) uncertainty_th=$2; shift 2 ;;

    # topic
    --stereo_image_topic) stereo_image_topic=$2; shift 2 ;;
    --camera_info_topic) camera_info_topic=$2; shift 2 ;;
    --depth_image_topic) depth_image_topic=$2; shift 2 ;;
    --depth_camera_info_topic) depth_camera_info_topic=$2; shift 2 ;;
    --pointcloud2_topic) pointcloud2_topic=$2; shift 2 ;;
    --rectify_left_image_topic) rectify_left_image_topic=$2; shift 2 ;;
    --rectify_right_image_topic) rectify_right_image_topic=$2; shift 2 ;;
    --publish_rectify_bgr) publish_rectify_bgr=$2; shift 2 ;;
    --origin_left_image_topic) origin_left_image_topic=$2; shift 2 ;;
    --origin_right_image_topic) origin_right_image_topic=$2; shift 2 ;;
    --publish_origin_enable) publish_origin_enable=$2; shift 2 ;;
    --visual_image_topic) visual_image_topic=$2; shift 2 ;;

    # mipi cam
    --use_mipi_cam) use_mipi_cam=$2; shift 2 ;;
    --mipi_image_width) mipi_image_width=$2; shift 2 ;;
    --mipi_image_height) mipi_image_height=$2; shift 2 ;;
    --mipi_image_framerate) mipi_image_framerate=$2; shift 2 ;;
    --mipi_frame_ts_type) mipi_frame_ts_type=$2; shift 2 ;;
    --mipi_gdc_enable) mipi_gdc_enable=$2; shift 2 ;;
    --mipi_lpwm_enable) mipi_lpwm_enable=$2; shift 2 ;;
    --mipi_rotation) mipi_rotation=$2; shift 2 ;;
    --mipi_channel) mipi_channel=$2; shift 2 ;;
    --mipi_channel2) mipi_channel2=$2; shift 2 ;;
    --mipi_cal_rotation) mipi_cal_rotation=$2; shift 2 ;;

    # calib
    --calib_method) calib_method=$2; shift 2 ;;
    --stereo_calib_file_path) stereo_calib_file_path=$2; shift 2 ;;

    # render
    --render_type) render_type=$2; shift 2 ;;
    --render_perf) render_perf=$2; shift 2 ;;
    --render_max_disp) render_max_disp=$2; shift 2 ;;
    --render_z_near) render_z_near=$2; shift 2 ;;
    --render_z_range) render_z_range=$2; shift 2 ;;

    # speckle filter
    --speckle_filter_enable) speckle_filter_enable=$2; shift 2 ;;
    --max_speckle_size) max_speckle_size=$2; shift 2 ;;
    --max_disp_diff) max_disp_diff=$2; shift 2 ;;

    # pointcloud
    --pointcloud_height_min) pointcloud_height_min=$2; shift 2 ;;
    --pointcloud_height_max) pointcloud_height_max=$2; shift 2 ;;
    --pointcloud_depth_max) pointcloud_depth_max=$2; shift 2 ;;

    # pcl filter
    --pcl_filter_enable) pcl_filter_enable=$2; shift 2 ;;
    --grid_size) grid_size=$2; shift 2 ;;
    --grid_min_point_count) grid_min_point_count=$2; shift 2 ;;

    # thread
    --infer_thread_num) infer_thread_num=$2; shift 2 ;;
    --save_thread_num) save_thread_num=$2; shift 2 ;;
    --max_save_task) max_save_task=$2; shift 2 ;;

    # save
    --save_result_flag) save_result_flag=$2; shift 2 ;;
    --save_dir) save_dir=$2; shift 2 ;;
    --save_freq) save_freq=$2; shift 2 ;;
    --save_total) save_total=$2; shift 2 ;;
    --save_stereo_flag) save_stereo_flag=$2; shift 2 ;;
    --save_origin_flag) save_origin_flag=$2; shift 2 ;;
    --save_disp_flag) save_disp_flag=$2; shift 2 ;;
    --save_uncert_flag) save_uncert_flag=$2; shift 2 ;;
    --save_depth_flag) save_depth_flag=$2; shift 2 ;;
    --save_visual_flag) save_visual_flag=$2; shift 2 ;;
    --save_pcd_flag) save_pcd_flag=$2; shift 2 ;;

    # local image
    --use_local_image_flag) use_local_image_flag=$2; shift 2 ;;
    --local_image_dir) local_image_dir=$2; shift 2 ;;
    --image_sleep) image_sleep=$2; shift 2 ;;

    # camera intrinsic
    --camera_cx) camera_cx=$2; shift 2 ;;
    --camera_cy) camera_cy=$2; shift 2 ;;
    --camera_fx) camera_fx=$2; shift 2 ;;
    --camera_fy) camera_fy=$2; shift 2 ;;
    --baseline) baseline=$2; shift 2 ;;
    --doffs) doffs=$2; shift 2 ;;

    # mask
    --left_img_mask_enable) left_img_mask_enable=$2; shift 2 ;;

    # epipolar
    --epipolar_mode) epipolar_mode=$2; shift 2 ;;
    --epipolar_img) epipolar_img=$2; shift 2 ;;
    --chessboard_per_rows) chessboard_per_rows=$2; shift 2 ;;
    --chessboard_per_cols) chessboard_per_cols=$2; shift 2 ;;
    --chessboard_square_size) chessboard_square_size=$2; shift 2 ;;

    # web
    --stereonet_pub_web) stereonet_pub_web=$2; shift 2 ;;
    --codec_sub_topic) codec_sub_topic=$2; shift 2 ;;
    --codec_in_format) codec_in_format=$2; shift 2 ;;
    --codec_pub_topic) codec_pub_topic=$2; shift 2 ;;
    --websocket_image_topic) websocket_image_topic=$2; shift 2 ;;
    --websocket_channel) websocket_channel=$2; shift 2 ;;

    *) echo "unknown param: $1"; exit 1 ;;
  esac
done

ros2 launch hobot_stereonet stereonet_model_web_visual_$stereonet_version.launch.py \
stereo_node_name:=$stereo_node_name \
uncertainty_th:=$uncertainty_th \
stereo_image_topic:=$stereo_image_topic camera_info_topic:=$camera_info_topic \
depth_image_topic:=$depth_image_topic depth_camera_info_topic:=$depth_camera_info_topic \
pointcloud2_topic:=$pointcloud2_topic rectify_left_image_topic:=$rectify_left_image_topic \
rectify_right_image_topic:=$rectify_right_image_topic publish_rectify_bgr:=$publish_rectify_bgr \
origin_left_image_topic:=$origin_left_image_topic origin_right_image_topic:=$origin_right_image_topic \
publish_origin_enable:=$publish_origin_enable visual_image_topic:=$visual_image_topic \
use_mipi_cam:=$use_mipi_cam mipi_image_width:=$mipi_image_width mipi_image_height:=$mipi_image_height \
mipi_image_framerate:=$mipi_image_framerate mipi_frame_ts_type:=$mipi_frame_ts_type \
mipi_gdc_enable:=$mipi_gdc_enable mipi_lpwm_enable:=$mipi_lpwm_enable mipi_rotation:=$mipi_rotation \
mipi_channel:=$mipi_channel mipi_channel2:=$mipi_channel2 mipi_cal_rotation:=$mipi_cal_rotation \
calib_method:=$calib_method stereo_calib_file_path:=$stereo_calib_file_path \
render_type:=$render_type render_perf:=$render_perf render_max_disp:=$render_max_disp render_z_near:=$render_z_near render_z_range:=$render_z_range \
speckle_filter_enable:=$speckle_filter_enable max_speckle_size:=$max_speckle_size max_disp_diff:=$max_disp_diff \
pointcloud_height_min:=$pointcloud_height_min pointcloud_height_max:=$pointcloud_height_max pointcloud_depth_max:=$pointcloud_depth_max \
pcl_filter_enable:=$pcl_filter_enable grid_size:=$grid_size grid_min_point_count:=$grid_min_point_count \
infer_thread_num:=$infer_thread_num save_thread_num:=$save_thread_num max_save_task:=$max_save_task \
use_local_image_flag:=$use_local_image_flag local_image_dir:=$local_image_dir image_sleep:=$image_sleep \
save_result_flag:=$save_result_flag save_dir:=$save_dir save_freq:=$save_freq save_total:=$save_total save_stereo_flag:=$save_stereo_flag \
save_origin_flag:=$save_origin_flag save_disp_flag:=$save_disp_flag save_uncert_flag:=$save_uncert_flag save_depth_flag:=$save_depth_flag \
save_visual_flag:=$save_visual_flag save_pcd_flag:=$save_pcd_flag \
use_local_image_flag:=$use_local_image_flag local_image_dir:=$local_image_dir image_sleep:=$image_sleep \
camera_cx:=$camera_cx camera_cy:=$camera_cy camera_fx:=$camera_fx camera_fy:=$camera_fy baseline:=$baseline doffs:=$doffs \
left_img_mask_enable:=$left_img_mask_enable \
epipolar_mode:=$epipolar_mode epipolar_img:=$epipolar_img \
chessboard_per_rows:=$chessboard_per_rows chessboard_per_cols:=$chessboard_per_cols chessboard_square_size:=$chessboard_square_size \
stereonet_pub_web:=$stereonet_pub_web codec_sub_topic:=$codec_sub_topic codec_in_format:=$codec_in_format \
codec_pub_topic:=$codec_pub_topic websocket_image_topic:=$websocket_image_topic websocket_channel:=$websocket_channel
```

#### (3) 执行启动指令

- 通过ssh连接RDK，执行以下命令，则可以启动算法：

<Tabs groupId="RDK">
<TabItem value="RDK X5" label="RDK X5">

```bash
# 搭配230AI相机
bash run_stereo.sh --mipi_rotation 0.0

# 搭配132GS相机
bash run_stereo.sh

# 注意：
# 需要观察网页端图像RGB图是否是左目相机采集的图像，可以用镜头盖遮挡一下左目相机确认
# 如果左右目相机顺序不正确，有两个方法调整：
# 方法1：交换MIPI线
# 方法2：在上面的运行指令上，加入参数：--channel 0 --channel2 2
```

</TabItem>
<TabItem value="RDK S100" label="RDK S100">

```bash
# 搭配230AI相机
bash run_stereo.sh --stereonet_version v2.4 --mipi_rotation 0.0

# 搭配132GS相机
bash run_stereo.sh --stereonet_version v2.4

# 注意：
# 需要观察网页端图像RGB图是否是左目相机采集的图像，可以用镜头盖遮挡一下左目相机确认
# 如果左右目相机顺序不正确，有两个方法调整：
# 方法1：交换MIPI线
# 方法2：在上面的运行指令上，加入参数：--channel 0 --channel2 2
```

</TabItem>
</Tabs>

- 左右目相机定义，<span style={{ color: 'red' }}> 需要确认下文网页端显示的RGB图像是否是左相机拍摄的图像 </span>：

![230ai_left_right_cam](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/03_boxs/function/image/box_adv/230ai_left_right_cam.png)

- 双目算法启动成功后会打印如下日志，`fx/fy/cx/cy/baseline`是相机内参，`fps`是算法运行的帧率：

![stereonet_run_success_log](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/03_boxs/function/image/box_adv/stereonet_run_success_log.png)

- 通过网页端查看RGB图和深度图，在浏览器输入 http://ip:8000 (图中RDK ip是192.168.1.100)：

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


#### (4) 参数定义

`run_stereo.sh`脚本有很多可设置参数：

- stereonet_version控制启动不同版本的算法
  - RDK X5可以设置为`v2.0`、`v2.1`、`v2.2`、`v2.3`、`v2.4_int16`、`v2.4_int8`
  - RDK S100可以设置为`v2.1`、`v2.4`
- stereo_node_name控制ros节点的名称
- uncertainty_th为置信度阈值，只有带置信度的模型并且设置为正数时才会生效，如果需要开启，建议设置为`0.10`
- stereo_image_topic/camera_info_topic/depth_image_topic等控制ros节点发布的话题名称

- mipi_image_width、mipi_image_height、mipi_image_framerate控制相机的分辨率和帧率
- mipi_gdc_enable控制相机开启GDC矫正，相机会读取EEPROM存储的参数进行图像的畸变矫正，目前生成的相机都带有出厂标定参数
- mipi_lpwm_enable控制相机开启硬件同步，使得双目相机获取的左右图像时间戳是一致的，如果设置为False，则使用软同步，同步误差较大
- mipi_rotation控制图像是否进行旋转，目前132GS相机CMOS安装时有90°旋转，需要设置该参数为`90.0`
- mipi_channel、mipi_channel2用于调换左右图输出顺序

- calib_method控制矫正方式
  - 当`mipi_gdc_enable:=True`时，代表`hobot_mipi_cam`功能包已经对图像经过矫正，`hobot_stereonet`功能包不需要再进行矫正，calib_method设置为`none`即可
  - 当`mipi_gdc_enable:=False`时，或者相机无法对图像进行矫正时，需要将calib_method设置为`custom`，并且需要指定`stereo_calib_file_path`
- stereo_calib_file_path控制自定义标定参数的路径

- render_type控制渲染方式，默认是`distance`，会自动根据深度图距离自动渲染伪彩色图像用于网页端显示
- render_perf控制渲染图像上是否展示CPU、BPU占用率、Latency、FPS信息，可以设置为`True`、`False`

- speckle_filter_enable控制是否开启speckle filter滤波，可以设置为`True`、`False`
- max_speckle_size控制speckle的大小，小于该大小的speckle将会被滤除，设置越大，滤波效果更强
- max_disp_diff控制speckle中视差的差异阈值，邻域小于该阈值的像素点将划分为同一个speckle，设置越小，滤波效果更强

- pointcloud_height_min/pointcloud_height_max/pointcloud_depth_max控制点云的显示范围，单位是m

- pcl_filter_enable控制是否开启点云滤波，可以设置为`True`、`False`
- grid_size控制点云滤波时的网格大小，单位m
- grid_min_point_count控制点云滤波时的网格最小点数，小于该数量的点会被滤除

- save_result_flag控制是否保存结果，如果开启保存则会保存**相机参数、原始左右图、矫正后左右图、视差图、深度图、点云**
- save_dir控制保存的目录，目录不存在会自动创建，请确保该目录下有足够空间，否则会保存失败
- save_freq控制保存的频率，例如设置为4代表每隔4帧保存一次
- save_total控制保存的总数，设置为-1代表一直保存，设置为100代表保存100帧则不再保存

#### (5) 保存一帧图像

- 程序运行成功后，可以开启另一个终端，执行如下指令保存一帧数据：

```bash
source /opt/tros/humble/setup.bash

# 首先查看一下节点是否正常运行，注意一下是否设置了ROS_DOMAIN_ID或者改变了节点名称
ros2 node list

# 如果/StereoNetNode节点正常运行，运行如下指令可以保存一帧数据
# 设置保存目录，建议设置绝对路径，如果保存目录不存在会自动创建
ros2 param set /StereoNetNode save_dir /root/online_once
# 保存一帧数据，可重复执行
ros2 param set /StereoNetNode save_result_once true
```

#### (7) 保存批量数据

- 方法1：在启动时指定参数保存

```bash
# 搭配230AI相机
bash run_stereo.sh --mipi_rotation 0.0 \
--save_result_flag True --save_dir /root/online_batch \
--save_freq 1 --save_total -1 \
--save_stereo_flag True --save_origin_flag False \
--save_disp_flag True --save_uncert_flag False \
--save_depth_flag True --save_visual_flag True \
--save_pcd_flag False

# 搭配132GS相机
bash run_stereo.sh \
--save_result_flag True --save_dir /root/online_batch \
--save_freq 1 --save_total -1 \
--save_stereo_flag True --save_origin_flag False \
--save_disp_flag True --save_uncert_flag False \
--save_depth_flag True --save_visual_flag True \
--save_pcd_flag False
```

![stereonet_save_log](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/03_boxs/function/image/box_adv/stereonet_save_log.png)

![stereonet_save_files](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/03_boxs/function/image/box_adv/stereonet_save_files.png)

- 方法2：程序运行成功后，可以开启另一个终端，执行如下指令保存数据

```bash
source /opt/tros/humble/setup.bash

# 首先查看一下节点是否正常运行，注意一下是否设置了ROS_DOMAIN_ID或者改变了节点名称
ros2 node list

# 如果/StereoNetNode节点正常运行，运行如下指令可以保存数据
# 设置保存目录，建议设置绝对路径，如果保存目录不存在会自动创建
ros2 param set /StereoNetNode save_dir /root/online_batch
# 设置保存总数
ros2 param set /StereoNetNode save_total 10
# 设置保存频率
ros2 param set /StereoNetNode save_freq 1

# 设置保存内容，按需要设置
ros2 param set /StereoNetNode save_stereo_flag true
ros2 param set /StereoNetNode save_origin_flag true
ros2 param set /StereoNetNode save_disp_flag true
ros2 param set /StereoNetNode save_uncert_flag true
ros2 param set /StereoNetNode save_depth_flag true
ros2 param set /StereoNetNode save_visual_flag true
ros2 param set /StereoNetNode save_pcd_flag true

# 执行保存命令
ros2 param set /StereoNetNode save_result_flag true

# 如果保存完毕后，还需要继续保存，需要再执行一下下面两条指令
# 重新设置保存总数
ros2 param set /StereoNetNode save_total 10
# 执行保存命令
ros2 param set /StereoNetNode save_result_flag true
```

### 5.4. 离线启动指令

#### (1) 准备离线图像

- 如果想利用本地图像评估算法效果，需要准备如下数据并上传到RDK：

1. **去畸变、极线对齐**的左右目图像，png或者jpg格式，图片需要按照规则命名，左目图像需要带有`left`字段，右目图像需要带有`right`字段，算法按序号遍历图像，直至图像全部计算完毕：

![stereonet_rdk](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/03_boxs/function/image/box_adv/image_format.png)

2. 相机内参文件，保存在图像目录下，命名为`camera_intrinsic.txt`，参考内容如下：
```bash
# fx fy cx cy baseline(m)
215.762581 215.762581 325.490113 173.881556 0.079957
```

#### (2) 执行启动指令

- 通过ssh连接RDK，执行以下命令：

```bash
bash run_stereo.sh \
--use_local_image_flag True --local_image_dir /root/imgdir \
--save_result_flag True --save_dir /root/offline \
--save_stereo_flag True --save_origin_flag False \
--save_disp_flag True --save_uncert_flag False \
--save_depth_flag True --save_visual_flag True \
--save_pcd_flag True

# 如果网页端显示太快，可以加入参数控制一下停顿时间：--image_sleep 2000
```

- 运行成功后，会打印如下日志

![stereonet_offline_log](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/03_boxs/function/image/box_adv/stereonet_offline_log.png)

- 通过网页端查看RGB图和深度图，在浏览器输入 http://ip:8000 (图中RDK ip是192.168.128.10)：

![web_depth_visual_offline](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/03_boxs/function/image/box_adv/web_depth_visual_offline.png)

### 5.5. 搭配ZED相机运行

#### (1) ZED相机安装

- ZED双目摄像头如图所示：

![zed_cam](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/03_boxs/function/image/box_adv/zed_cam.png)

- 将ZED相机通过USB连接到RDK即可

#### (2) 启动指令

- 首先，启动ZED相机，通过ssh连接RDK，X5和S100执行相同指令：

```bash
source /opt/tros/humble/setup.bash

ros2 launch hobot_zed_cam zed_cam_node.launch.py \
resolution:=720p \
need_rectify:=true dst_width:=640 dst_height:=352
```

参数解释：

| 参数         | 定义                                                               |
| ------------ | ------------------------------------------------------------------ |
| resolution   | zed原始输出分辨率，带畸变，720p表示1280*720的分辨率，可设置为1080p |
| need_rectify | 表示最终输出的图像是否需要矫正                                     |
| dst_width    | 最终输出的矫正后图像分辨率为640*352                                |
| dst_height   | 最终输出的矫正后图像分辨率为640*352                                |

<p style={{ color: 'red' }}> 注意：运行ZED相机RDK一定要联网，因为ZED需要联网下载标定文件 </p>


![stereonet_zed_run_success_log](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/03_boxs/function/image/box_adv/stereonet_zed_run_success_log.png)

联网的情况下程序会自动下载标定文件，如果RDK没有联网，可以手动下载标定文件然后上传到RDK。
根据log信息，在PC端打开浏览器，输入(https://calib.stereolabs.com/?SN=38085162) ，即可下载标定文件SN38085162.conf。
注意每台ZED的SN码是不一样的，使用请根据报错信息下载对应的标定文件，将标定文件上传到`/root/zed/settings/`目录下，如果目录不存在则手动创建。

- 然后，启动双目算法，开启另一个终端执行：

```bash
bash run_stereo.sh --use_mipi_cam False
```

- 通过网页端查看深度图，在浏览器输入 http://ip:8000 (ip为RDK对应的ip地址)，如需查看**点云**和**保存图像**请参考上文对应的设置

## 6. hobot_stereonet功能包说明

### 6.1. 订阅话题

| 默认名称（参数可调）                 | 消息类型                     | 说明                                       |
| ------------------------------------ | ---------------------------- | ------------------------------------------ |
| /image_combine_raw                   | sensor_msgs::msg::Image      | 左右目上下拼接的图像，用于模型推理         |
| /image_right_raw/camera_info（可选） | sensor_msgs::msg::CameraInfo | 相机标定参数，用于视差图和深度图之间的转换 |

### 6.2. 发布话题

| 默认名称（参数可调）                 | 消息类型                      | 说明                 |
| ------------------------------------ | ----------------------------- | -------------------- |
| /StereoNetNode/stereonet_depth       | sensor_msgs::msg::Image       | 深度图像，单位为毫米 |
| /StereoNetNode/stereonet_visual      | sensor_msgs::msg::Image       | 可视化渲染图像       |
| /StereoNetNode/stereonet_pointcloud2 | sensor_msgs::msg::PointCloud2 | 点云，单位为m        |
| /StereoNetNode/rectify_left_image    | sensor_msgs::msg::Image       | 矫正后左图，输入算法 |
| /StereoNetNode/rectify_right_image   | sensor_msgs::msg::Image       | 矫正后右图，输入算法 |
| /StereoNetNode/origin_left_image     | sensor_msgs::msg::Image       | 原始左图，不输入算法 |
| /StereoNetNode/origin_right_image    | sensor_msgs::msg::Image       | 原始右图，不输入算法 |









