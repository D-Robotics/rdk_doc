---
sidebar_position: 11
---

# web 显示摄像头

## 示例简介
Web 显示摄像头示例是一个位于 `/app/pydev_demo/09_web_display_camera_sample/` 中的 **Python 接口** 开发代码示例，用于演示如何使用板载 MIPI 摄像头进行实时目标检测，并通过 WebSocket 将检测结果推送到 Web 浏览器端进行实时显示。该示例使用 YOLOv5x 目标检测模型对 MIPI 摄像头采集的视频流进行实时推理，并通过 WebSocket 协议将图像和检测框信息推送到 Web 客户端。

## 效果展示
运行后通过浏览器访问开发板 IP 地址，可在 Web 页面实时查看摄像头画面和检测结果。

![output-img](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/RDK_X5_Newly/pydev_11_running.png)

## 硬件准备

### 硬件连接
1. 准备一个 RDK 开发板
2. 连接官方适配的 MIPI 摄像头
3. 连接网线到开发板
4. 连接电源线
![connect-img](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/RDK_X5_Newly/pydev_05_hw_connect.png)


## 快速开始

### 代码以及板端位置
进入到 `/app/pydev_demo/09_web_display_camera_sample/` 位置，可以看到 Web 显示摄像头示例包含以下文件：
```
root@ubuntu:/app/pydev_demo/09_web_display_camera_sample# tree
.
├── coco_classes.names
├── mipi_camera_web_yolov5x.py
├── start_nginx.sh
├── webservice
│   ├── client_body_temp
│   ├── conf
│   │   ├── nginx.conf
│   │   ├── ...
│   ├── html
│   │   ├── assets
│   │   ├── index.html
│   │   └── ...
│   ├── logs
│   ├── proxy_temp
│   └── sbin
└── x3_pb2.py
```

### 编译以及运行
首先需要启动 Nginx 服务器，然后运行 Python 脚本：

:::info 提示

采用 IMX477 相机时需要设置 mipi_camera_web.py 文件中（第 39 行左右）的 fps=50。

:::

```bash
# 进入到示例目录
cd /app/pydev_demo/09_web_display_camera_sample/

# 启动 Nginx 服务器
cd webservice/
./sbin/nginx -p .

#（如果 ./sbin/nginx 没有执行权限，需要添加，参考命令如下）
# chmod +x ./sbin/nginx

# 运行 Web 摄像头示例
# 返回示例目录
cd ..
python mipi_camera_web_yolov5x.py
```


### 执行效果

运行后，程序会启动 Web 服务，可以通过浏览器访问开发板的 IP 地址查看实时视频流和目标检测结果。

在浏览器中访问 http:// 开发板 IP ，默认的是 http://192.168.127.10。
![pydev_05_wb_disp_web_img](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/RDK_X5_Newly/pydev_05_wb_disp_web_img.png)

点击浏览器中显示的 `web Display` ，即可看到实时视频流和目标检测结果，效果可以查看文章开头的 [效果展示小节](#效果展示)

## 详细介绍

### 示例程序参数选项说明

该示例支持命令行参数配置，如果不指定参数，程序会使用默认值自动加载同目录下的测试图像进行推理。

| 参数 | 说明 | 类型 | 默认值 |
|------|------|------|--------|
| `--model-path` | BPU 量化模型路径（`.bin`） | str | `/app/model/basic/yolov5x_672x672_nv12.bin` |
| `--priority` | 推理优先级（0~255，255为最高） | int | `0` |
| `--bpu-cores` | BPU 核心索引列表（如 `0 1`） | int list | `[0]` |
| `--label-file` | 类别标签文件路径 | str | `coco_classes.names` |
| `--nms-thres` | 非极大值抑制的 IoU 阈值 | float | `0.45` |
| `--score-thres` | 检测置信度阈值 | float | `0.25` |

### 软件架构说明

本部分介绍 Web 显示摄像头示例的软件架构和工作流程，说明程序从初始化到 Web 推送的完整执行过程，帮助理解代码的整体结构和数据流向。

<center>
![software_arch](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/RDK_X5_Newly/pydev_05_wb_disp_cam_software_arch.png)
</center>


<!-- 1. **模型加载** - 加载 YOLOv5x 目标检测模型
2. **摄像头初始化** - 使用 `srcampy.Camera()` 初始化 MIPI 摄像头，设置分辨率为 1920x1072（16 字节对齐）
3. **编码器初始化** - 使用 `srcampy.Encoder()` 初始化 JPEG 编码器
4. **WebSocket 服务器启动** - 启动异步 WebSocket 服务器，监听端口 8080
5. **图像采集** - 从 MIPI 摄像头获取视频帧（NV12 格式）
6. **图像预处理** - 分离 NV12 的 Y 和 UV 分量，缩放图像至模型输入尺寸
7. **模型推理** - 在 BPU 上执行 YOLOv5x 前向推理
8. **结果后处理** - 解码输出、过滤低置信度结果、NMS 去重、坐标映射回原图
9. **图像编码** - 将原始图像编码为 JPEG 格式
10. **结果序列化** - 将检测结果和图像封装为 Protocol Buffer 消息
11. **WebSocket 推送** - 通过 WebSocket 将消息推送到所有连接的 Web 客户端
12. **Web 端显示** - 浏览器接收消息并实时渲染检测结果 -->

Web 显示摄像头示例涉及的不仅仅是 mipi_camera_web.py 源码 ， 还有 nginx 服务器和 web 接收端 ，所以软件架构相对前几个示例更加多一点，包含以下几个核心部分：

1. **MIPI 摄像头采集** - 使用 `srcampy.Camera()` 初始化 MIPI 摄像头并采集视频流

2. **目标检测模型** - 加载 YOLOv5x 目标检测模型对视频帧进行实时推理

3. **视频编码** - 使用 `srcampy.Encoder()` 对视频帧进行 JPEG 编码

4. **WebSocket 服务** - 创建 WebSocket 服务器，实时推送视频帧和检测结果

5. **Web 前端** - 提供 HTML 页面和 JavaScript 代码，用于在浏览器中显示视频和检测结果

6. **Nginx 服务器** - 提供静态文件服务和 HTTP 代理功能

### API 流程说明
本部分列出示例程序中使用的主要 API 接口，说明各接口的功能、输入参数和返回值，帮助开发者快速了解代码实现细节和接口调用方式。

<center>
![API_Flow](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/RDK_X5_Newly/pydev_11_01_flow.png)
</center>

1. `load_class_names(class_file)`

   加载类别名称，输入：类别文件路径，返回：类别名称列表

2. `srcampy.Camera()`

   创建 MIPI 摄像头对象

3. `cam.open_cam(pipe_id, video_index, fps, width_list, height_list)`

   打开摄像头，输入：通道号、视频索引、帧率、宽度列表 [512, 1920]、高度列表 [512, 1072]

4. `hbm_runtime.HB_HBMRuntime(model_path)`

   加载 YOLOv5x 模型，输入：模型文件路径

5. `WebSocketServer(host, port, model_handler, camera_manager, detection_classes)`

   创建 WebSocket 服务器实例，输入：主机地址、端口号、模型处理器、摄像头管理器、类别列表

6. `websockets.serve(handler, host, port)`

   启动 WebSocket 服务器，输入：处理函数、主机地址（0.0.0.0）、端口号（8080）

7. `cam.get_img(chn, width, height)`

   获取摄像头原始图像，输入：通道号（2）、宽（1920）、高（1072），返回：NV12 格式图像数据

8. `split_nv12_bytes(img, width, height)`

   分离 NV12 图像的 Y 和 UV 分量，输入：NV12 图像数据、宽（1920）、高（1072），返回：Y 分量、UV 分量

9. `resize_nv12_yuv(y, uv, target_w, target_h)`

   缩放 NV12 图像，输入：Y 分量、UV 分量、目标宽（672）、目标高（672），返回：缩放后的 Y、UV 分量

10. `model.run(input_tensor)`

   执行模型推理，输入：预处理后的输入张量字典，返回：模型输出字典

11. `dequantize_outputs(outputs, output_quants)`

   结果反量化，输入：模型输出字典、输出量化参数，返回：float32 类型数据

12. `decode_outputs(output_names, fp32_outputs, strides, anchors, classes_num)`

   解码 YOLO 模型输出，输入：输出名称列表、反量化后的输出、步长、锚点、类别数（80），返回：预测结果

13. `filter_predictions(predictions, score_threshold)`

   过滤预测结果，输入：预测结果、置信度阈值，返回：检测框、置信度、类别

14. `NMS(boxes, scores, classes, iou_threshold)`

   非极大值抑制，输入：检测框、置信度、类别、IoU 阈值，返回：保留的索引

15. `scale_coords_back(boxes, orig_w, orig_h, model_w, model_h, resize_type)`

   调整检测框至原图尺寸，输入：检测框、原图宽（1920）、原图高（1072）、模型输入宽（672）、模型输入高（672）、缩放类型，返回：缩放后的检测框

16. `serialize_message(frame_message, data, detection_classes)`

   序列化检测结果和图像为 Protocol Buffer 消息，输入：帧消息对象、检测结果列表、类别名称列表，返回：序列化后的字节流

17. `websocket.send(serialized_buf)`

   发送消息到客户端，输入：序列化后的数据（字节流）

18. `cam.close_cam()`

   关闭摄像头



### FAQ
Q: 运行示例时提示端口被占用怎么办？\
A: 请检查是否有其他程序占用了 8080 端口，可以使用 netstat -tlnp 命令查看端口占用情况。

Q: 浏览器无法访问视频流怎么办？\
A: 请检查开发板的网络连接，并确保防火墙没有阻止相关端口的访问。

Q: 视频流延迟很高怎么办？\
A: 可以尝试降低视频分辨率或帧率，或者使用更轻量的目标检测模型。

Q: 如何修改 Web 前端的界面？\
A: 可以修改 webservice/html/ 目录下的 HTML、 CSS 和 JavaScript 文件来自定义界面。 -->

Q: 如何添加新的检测类别？\
A: 需要修改 get_classes() 函数，添加新的类别名称，并重新训练模型或使用支持新类别的模型。

Q: 如何保存视频流？\
A: 可以在代码中添加视频保存逻辑，例如使用 OpenCV 的 VideoWriter 类保存视频文件。



