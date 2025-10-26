---
sidebar_position: 10
---

# 3.3.10 web 显示摄像头示例介绍

## 示例简介
Web 显示摄像头示例是一个位于 `/app/pydev_demo/05_web_display_camera_sample/` 中的 **Python 接口** 开发代码示例，用于演示如何将 MIPI 摄像头的视频流和目标检测结果通过 Web 服务实时推送到浏览器端显示。该示例结合了 MIPI 摄像头采集、 AI 目标检测、视频编码和 WebSocket 实时传输等技术，提供了一个完整的 Web 视频监控解决方案。

## 效果展示
![output-img](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/pydev_05_runing.png)

## 硬件准备

### 硬件连接
1. 准备一个 RDK 开发板
2. 连接官方适配的 MIPI 摄像头
3. 连接网线到开发板
4. 连接电源线
![connect-img](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/pydev_05_hw_connect.png)


## 快速开始

### 代码以及板端位置
进入到 `/app/pydev_demo/05_web_display_camera_sample/` 位置，可以看到 Web 显示摄像头示例包含以下文件：
```
root@ubuntu:/app/pydev_demo/05_web_display_camera_sample# tree
.
├── mipi_camera_web.py
├── start_nginx.sh
├── webservice
│ ├── conf
│ │ ├── nginx.conf
│ │ └── ...
│ ├── html
│ │ ├── index.html
│ │ ├── assets
│ │ └── ...
│ ├── logs
│ └── sbin
│ └── nginx
└── x3_pb2.py
```


### 编译以及运行
首先需要启动 Nginx 服务器，然后运行 Python 脚本：
```bash
# 启动 Nginx 服务器
./start_nginx.sh

# 运行 Web 摄像头示例
python3 mipi_camera_web.py
```


### 执行效果

运行后，程序会启动 Web 服务，可以通过浏览器访问开发板的 IP 地址查看实时视频流和目标检测结果：

```
```

在浏览器中访问 http:// 开发板 IP 即可看到实时视频流和目标检测结果。默认的是 http://192.168.127.10

## 详细介绍

### 示例程序参数选项说明
Web 显示摄像头示例不需要命令行参数，直接运行即可。程序会自动初始化 MIPI 摄像头和 WebSocket 服务。


### 软件架构说明
Web 显示摄像头示例涉及的不仅仅是 mipi_camera_web.py 源码 ， 还有 nginx 服务器和 web 接收端 ，所以软件架构相对前几个示例更加多一点，包含以下几个核心部分：

1. MIPI 摄像头采集：使用 srcampy.Camera() 初始化 MIPI 摄像头并采集视频流

2. 目标检测模型：加载 FCOS 目标检测模型对视频帧进行实时推理

3. 视频编码：使用 srcampy.Encoder() 对视频帧进行 JPEG 编码

4. WebSocket 服务：创建 WebSocket 服务器，实时推送视频帧和检测结果

5. Web 前端：提供 HTML 页面和 JavaScript 代码，用于在浏览器中显示视频和检测结果

6. Nginx 服务器：提供静态文件服务和 HTTP 代理功能

<center>
![software_arch](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/pydev_05_wb_disp_cam_software_arch.png)
</center>

### API 流程说明

1. 摄像头初始化： cam.open_cam()

2. 编码器初始化： enc.encode()

3. 模型加载： models = pyeasy_dnn.load('../models/fcos_512x512_nv12.bin')

4. WebSocket 服务启动： websockets.serve(web_service, "0.0.0.0", 8080)

5. 实时处理循环：

- 从摄像头获取图像帧

- 使用模型进行目标检测

- 编码图像帧为 JPEG 格式

- 通过 WebSocket 推送图像和检测结果

<center>
![API_Flow](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/pydev_05_wb_disp_cam_api_flow.png)
</center>

### FAQ
Q: 运行示例时提示端口被占用怎么办？\
A: 请检查是否有其他程序占用了 8080 端口，可以使用 netstat -tlnp 命令查看端口占用情况。

Q: 浏览器无法访问视频流怎么办？\
A: 请检查开发板的网络连接，并确保防火墙没有阻止相关端口的访问。

Q: 视频流延迟很高怎么办？\
A: 可以尝试降低视频分辨率或帧率，或者使用更轻量的目标检测模型。

Q: 如何修改 Web 前端的界面？\
A: 可以修改 webservice/html/ 目录下的 HTML、 CSS 和 JavaScript 文件来自定义界面。

Q: 如何添加新的检测类别？\
A: 需要修改 get_classes() 函数，添加新的类别名称，并重新训练模型或使用支持新类别的模型。

Q: 如何保存视频流？\
A: 可以在代码中添加视频保存逻辑，例如使用 OpenCV 的 VideoWriter 类保存视频文件。



