---
sidebar_position: 11
---
# 双目深度估计算法

## 功能介绍

双目深度估计算法是使用地平线[OpenExplorer](https://developer.horizon.cc/api/v1/fileData/horizon_j5_open_explorer_cn_doc/hat/source/examples/stereonet.html)在[SceneFlow](https://lmb.informatik.uni-freiburg.de/resources/datasets/SceneFlowDatasets.en.html)数据集上训练出来的`StereoNet`模型。

算法输入为双目图像数据，分别是左右视图。算法输出为左视图的视差。

此示例使用ZED 2i双目相机作为图像数据输入源，利用BPU进行算法推理，发布包含双目图像左图和感知结果的话题消息，在PC端浏览器上渲染显示算法结果。

代码仓库： (https://github.com/D-Robotics/hobot_stereonet.git)

## 支持平台

| 平台      | 运行方式     | 示例功能                                |
| --------- | ------------ | --------------------------------------- |
| RDK Ultra | Ubuntu 20.04 (Foxy) | 使用本地回灌，并通过web展示推理渲染结果 |

## 准备工作

1. RDK已烧录好地平线提供的Ubuntu 20.04系统镜像。

2. RDK已成功安装TogetheROS.Bot。

3. ZED 2i双目相机，连接到RDK的USB 3.0接口。

4. 确认PC机能够通过网络访问RDK。

## 使用介绍

订阅从ZED 2i双目相机采集到的图像数据作为输入，经过推理后发布包含双目图像左图和感知结果的话题消息，通过websocket package实现在PC端浏览器上渲染显示发布的图片和对应的算法结果。

启动命令：


```bash
# 配置tros.b环境
source /opt/tros/setup.bash

# 启动websocket服务
ros2 launch websocket websocket_service.launch.py

ros2 launch hobot_stereonet hobot_stereonet_demo.launch.py 
```

## 结果分析

在运行终端输出如下信息：

```text
[INFO] [launch]: All log files can be found below /root/.ros/log/2023-07-05-18-23-51-350999-hobot-2628272
[INFO] [launch]: Default logging verbosity is set to INFO
webserver has launch
[INFO] [hobot_stereo_usb_cam-1]: process started with pid [2628275]
[INFO] [talker-2]: process started with pid [2628277]
[INFO] [websocket-3]: process started with pid [2628279]
[INFO] [hobot_stereonet-4]: process started with pid [2628281]
[hobot_stereo_usb_cam-1] [WARN] [1688581432.042569331] [stereo_usb_cam_node]: Get params complete.
[hobot_stereo_usb_cam-1]  camera_name: default_cam
[hobot_stereo_usb_cam-1]  video_device index: 0
[hobot_stereo_usb_cam-1]  image_width: 1280
[hobot_stereo_usb_cam-1]  image_height: 720
[hobot_stereo_usb_cam-1]  io_method_name: shared_mem
[hobot_stereo_usb_cam-1]  pub_topic_name: hbmem_stereo_img
[hobot_stereo_usb_cam-1]  out_format: nv12
[hobot_stereo_usb_cam-1]  enable_fb: 0
[hobot_stereo_usb_cam-1]  enable_dump: 0
[hobot_stereonet-4] [WARN] [1688581432.071555206] [stereonet_node]:
[hobot_stereonet-4]  sub_hbmem_topic_name: hbmem_stereo_img
[hobot_stereonet-4]  ros_img_topic_name: /stereonet_node_output
[hobot_stereo_usb_cam-1] [sl_oc::video::VideoCapture] INFO: ZED Open Capture - Camera module - Version: 0.6.0
[hobot_stereo_usb_cam-1] [sl_oc::video::VideoCapture] INFO: Camera resolution: 2560x720@30Hz
[hobot_stereo_usb_cam-1] [sl_oc::video::VideoCapture] INFO: Trying to open the device '/dev/video0'
[hobot_stereonet-4] [BPU_PLAT]BPU Platform Version(1.3.3)!
[hobot_stereonet-4] [HBRT] set log level as 0. version = 3.14.25.0
[hobot_stereonet-4] [DNN] Runtime version = 1.12.3_(3.14.25 HBRT)
[hobot_stereo_usb_cam-1] [sl_oc::video::VideoCapture] INFO: Opened camera with SN: 38085162
[hobot_stereo_usb_cam-1] [sl_oc::video::VideoCapture] INFO: Device '/dev/video0' opened
[hobot_stereonet-4] [WARN] [1688581432.344738873] [dnn]: Run default SetOutputParser.
[hobot_stereonet-4] [WARN] [1688581432.344880957] [dnn]: Set output parser with default dnn node parser, you will get all output tensors and should parse output_tensors in PostProcess.
[hobot_stereonet-4] [WARN] [1688581432.347218373] [stereonet_node]: model_input_count: 1, model_input_width: 1280, model_input_height: 720
[hobot_stereo_usb_cam-1] [WARN] [1688581432.412578248] [stereo_usb_cam_node]: Open video device 0 success.
[hobot_stereo_usb_cam-1] camera sn: 38085162[/dev/video0]
[hobot_stereonet-4] [WARN] [1688581434.992634291] [stereonet_node]: input fps: 1.60, out fps: 1.60, preprocess time ms: 1191, infer time ms: 48, msg preparation for pub time cost ms: 8
[hobot_stereonet-4] [WARN] [1688581436.203778417] [stereonet_node]: input fps: 0.82, out fps: 0.82, preprocess time ms: 1157, infer time ms: 47, msg preparation for pub time cost ms: 2
```

在PC端的浏览器输入http://IP:8000 即可查看图像和算法渲染效果（IP为RDK的IP地址）：

![](/../static/img/05_Robot_development/03_boxs/function/image/box_adv/stereonet_rdk.png)

相同场景下ZED的深度估计可视化效果如下：

![](/../static/img/05_Robot_development/03_boxs/function/image/box_adv/stereonet_zed.png)

可以看到对于有光线变化区域，深度学习方法的深度估计准确率更高。
