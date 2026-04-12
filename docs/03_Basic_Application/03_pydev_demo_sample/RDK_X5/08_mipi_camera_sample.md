---
sidebar_position: 10
---

# MIPI 摄像头实时检测

## 示例简介
MIPI 摄像头实时检测示例是一个位于 `/app/pydev_demo/08_mipi_camera_sample` 中的 **Python 接口** 开发代码示例，用于演示如何使用板载 MIPI 摄像头进行实时目标检测。该示例使用 YOLOv5x 目标检测模型对 MIPI 摄像头采集的视频流进行实时推理，并将检测结果通过 HDMI 显示，输出检测框信息。

包含的示例：
```
root@ubuntu:/app/pydev_demo/08_mipi_camera_sample$ tree
.
├── 01_mipi_camera_yolov5s.py
├── 02_mipi_camera_dump.py
├── 03_mipi_camera_scale.py
├── 04_mipi_camera_crop_scale.py
├── 05_mipi_camera_streamer.py
└── coco_classes.names
```

## 效果展示

### 01实时目标检测效果

:::info 可视化检测效果

如果需要通过显示屏查看实时的摄像头画面和检测结果可视化效果，需要：

1. **外接显示屏**：通过 HDMI 线缆将开发板连接到显示器
2. **Desktop 版特殊处理**：如果使用的是 Desktop 版本系统，需要先执行以下命令关闭桌面服务：
   ```bash
   sudo systemctl stop lightdm
   ```
3. **远程连接**：通过 SSH 远程连接到板端
4. **运行代码**：执行示例程序后，即可在连接的显示屏上看到实时的检测结果

:::

![output-img](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/RDK_X5_Newly/pydev_08_mipi_camera_yolov5s_runing.jpg)

### 02图像采集保存效果

运行后会在脚本同级目录下保存多个 YUV 格式的图像文件，默认是 1920x1080 的分辨率。

![output-img](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/RDK_X5_Newly/pydev_08_mipi_camera_dump.png)

### 03图像缩放处理效果

运行后会在脚本同级目录下保存缩放后的 YUV 图像文件，默认是 640x360 的分辨率。

![output-img](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/RDK_X5_Newly/pydev_08_mipi_camera_scale.png)

### 04图像裁剪缩放效果

运行后会在脚本同级目录下保存裁剪并缩放后的 YUV 图像文件（NV12 格式），默认是裁剪缩放画面中心，我们调整一下裁剪位置，可以得到如下 YUV 图像。

![output-img](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/RDK_X5_Newly/pydev_08_mipi_camera_crop_scale.png)

### 05实时推流显示效果

运行后通过 HDMI 屏幕实时显示摄像头画面（推流测试）, 注意 Desktop 版本需要先执行 `sudo systemctl stop lightdm` 关闭桌面服务。

![output-img](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/RDK_X5_Newly/pydev_08_mipi_camera_stream.gif)



## 硬件准备

### 硬件连接
1. 准备一个 RDK 开发板
2. 连接官方适配的 MIPI 摄像头
3. 通过 HDMI 线连接显示器和开发板
4. 连接电源线和网线

![connect-img](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/RDK_X5_Newly/pydev_03_hw_connect.png)

## 快速开始

### 代码以及板端位置
示例文件位于 `/app/pydev_demo/08_mipi_camera_sample`

### 编译以及运行
Python 示例无需编译，直接运行即可：

01_mipi_camera_yolov5s.py 运行:

```
cd /app/pydev_demo/08_mipi_camera_sample
python 01_mipi_camera_yolov5s.py
```

02_mipi_camera_dump.py 运行:

```
cd /app/pydev_demo/08_mipi_camera_sample
python 02_mipi_camera_dump.py -f 30 -c 10 -w 1920 -h 1080
```

03_mipi_camera_scale.py 运行:

```
cd /app/pydev_demo/08_mipi_camera_sample

# 该示例需要 input.yuv 作为输入，这里我们以上个运行示例中的 output0.yuv 作为输入，执行拷贝命令
cp output0.yuv input.yuv

# 然后运行示例
python 03_mipi_camera_scale.py -i input.yuv -o output_640x360.yuv -w 640 -h 360 --iwidth 1920 --iheight 1080
```

04_mipi_camera_crop_scale.py 运行:

```
cd /app/pydev_demo/08_mipi_camera_sample
python 04_mipi_camera_crop_scale.py -i input.yuv -o output_640x480.yuv -w 640 -h 480 --iwidth 1920 --iheight 1080 -x 304 -y 304 --crop_w 896 --crop_h 592
```

05_mipi_camera_streamer.py 运行:
```
cd /app/pydev_demo/08_mipi_camera_sample
python 05_mipi_camera_streamer.py -w 1920 -h 1080
```

<!-- ### 执行效果
01_mipi_camera_yolov5s.py 执行效果:
运行后，程序会初始化 MIPI 摄像头和 HDMI 显示，并开始实时目标检测。检测结果会通过 HDMI 显示。

02_mipi_camera_dump.py 执行效果:
运行成功后，脚本所在目录会存放多个yuv文件。

03_mipi_camera_scale.py 执行效果:
运行成功后，脚本所在目录会存放缩放后的yuv文件。

04_mipi_camera_crop_scale.py 执行效果:
运行成功后，脚本所在目录会存放缩放后的yuv文件。注意:裁剪宽度必须是 16 的整数倍（即对齐到 16 字节）。

05_mipi_camera_streamer.py 执行效果:
运行成功后，屏幕会显示实时画面。 -->

## 详细介绍

### 示例程序参数选项说明
#### 01_mipi_camera_yolov5s.py 示例参数说明

MIPI 摄像头实时检测示例不需要命令行参数，直接运行即可。程序会自动检测并使用板载 MIPI 摄像头。

#### 02_mipi_camera_dump.py 示例参数说明

| 参数 | 说明 | 类型 | 示例 |
|------|------|------|------|
| `-f` | 帧率（FPS） | int | `30` |
| `-c` | 采集帧数（count） | int | `10` |
| `-w` | 图像宽度 | int | `1920` |
| `-h` | 图像高度 | int | `1080` |

#### 03_mipi_camera_scale 示例参数说明
| 参数 | 说明 | 类型 | 示例 |
|------|------|------|--------|
| `-i` | 输入YUV 文件路径 | str | `input.yuv` |
| `-o` | 输出文件路径 | str | `output_scale.yuv` |
| `-w` | 输出图像宽度 | int | `640` |
| `-h` | 输出图像高度 | int | `360` |
| `--iwidth` | 输入图像宽度 | int | `1920` |
| `--iheight` | 输入图像高度 | int | `1080` |

#### 04_mipi_camera_crop_scale 示例参数说明

| 参数 | 说明 | 类型 | 示例 |
|------|------|------|--------|
| `-i` | 输入 YUV 文件路径 | str | `input.yuv` |
| `-o` | 输出文件路径 | str | `output_crop_scale.yuv` |
| `-w` | 输出图像宽度 | int | `640` |
| `-h` | 输出图像高度 | int | `480` |
| `--iwidth` | 输入图像原始宽度 | int | `1920` |
| `--iheight` | 输入图像原始高度 | int | `1080` |
| `-x` | 裁剪区域左上角 X 坐标 | int | `304` |
| `-y` | 裁剪区域左上角 Y 坐标 | int | `304` |
| `--crop_w` | 裁剪区域宽度 | int | `896` |
| `--crop_h` | 裁剪区域高度 | int | `592` |

#### 05_mipi_camera_streamer 示例参数说明

| 参数 | 说明 | 类型 | 示例 |
|------|------|------|------|
| `-w` | 输出图像宽度 | int | `1920` |
| `-h` | 输出图像高度 | int | `1080` |

### 软件架构说明

本部分介绍 MIPI 摄像头实时检测示例的软件架构和工作流程，说明各示例程序从初始化到运行完成的完整执行过程，帮助理解代码的整体结构和数据流向。

#### 实时目标检测示例软件架构

<center>
![software_arch](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/RDK_X5_Newly/pydev_10_mipi_sample_software_arch1.png)
</center>

1. **模型加载** - 使用 `hbm_runtime` 模块加载模型文件
2. **摄像头初始化** - 初始化 MIPI 摄像头
3. **显示初始化** - 初始化 HDMI 显示
4. **设备绑定** - 将摄像头输出绑定到显示
5. **图像采集** - 从 MIPI 摄像头获取视频帧
6. **图像预处理** - 缩放图像至模型输入尺寸，格式转换
7. **模型推理** - 在 BPU 上执行 YOLOv5x 前向推理
8. **结果后处理** - 解码输出、过滤低置信度结果、NMS 去重、坐标映射
9. **结果可视化** - 在原图上绘制检测框和标签
10. **显示输出** - 通过 HDMI 显示结果，控制台输出 FPS 和检测信息

#### 图像采集保存示例软件架构

<center>
![software_arch](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/RDK_X5_Newly/pydev_10_mipi_sample_software_arch2.png)
</center>

1. **摄像头初始化** - 初始化 MIPI 摄像头
2. **参数配置** - 设置采集帧率、分辨率、采集帧数
3. **图像采集** - 持续抓拍指定数量的图像帧
4. **文件保存** - 将采集的图像保存为 YUV 格式文件

#### 图像缩放处理示例软件架构

<center>
![software_arch](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/RDK_X5_Newly/pydev_10_mipi_sample_software_arch3.png)
</center>

1. **参数获取** - 解析命令行参数，获取输入输出文件路径和图像尺寸
2. **VPS 初始化** - 创建 VPS 对象，打开硬件缩放通道
3. **文件读取** - 读取输入 YUV 文件
4. **硬件缩放** - 通过硬件 VPS 完成图像缩放
5. **结果保存** - 将缩放后的图像保存为新的 YUV 文件
6. **资源清理** - 关闭 VPS，释放硬件资源

#### 图像裁剪缩放示例软件架构

<center>
![software_arch](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/RDK_X5_Newly/pydev_10_mipi_sample_software_arch4.png)
</center>

1. **参数获取** - 解析命令行参数，获取输入输出文件路径、图像尺寸及裁剪区域坐标
2. **VPS 初始化** - 创建 VPS 对象，打开硬件裁剪缩放通道
3. **文件读取** - 读取输入 YUV 文件
4. **硬件处理** - 通过硬件 VPS 完成图像裁剪
5. **结果保存** - 将处理后的图像保存为新的 YUV 文件
6. **资源清理** - 关闭 VPS，释放硬件资源

#### 实时推流显示示例软件架构

<center>
![software_arch](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/RDK_X5_Newly/pydev_10_mipi_sample_software_arch5.png)
</center>

1. **参数获取** - 解析命令行参数，获取显示分辨率
2. **显示器初始化** - 创建显示对象，初始化 HDMI 显示层
3. **摄像头初始化** - 创建摄像头对象，打开 MIPI 摄像头
4. **设备绑定** - 使用硬件绑定将摄像头数据流直接连接到显示器
5. **实时推流** - 摄像头画面持续通过硬件通路输出到 HDMI 显示器
6. **设备解绑** - 解除摄像头与显示器的绑定
7. **资源清理** - 关闭显示器和摄像头，释放硬件资源


### API 流程说明

本部分列出示例程序中使用的主要 API 接口，说明各接口的功能、输入参数和返回值，帮助开发者快速了解代码实现细节和接口调用方式。

#### 实时目标检测主要接口：

<center>
![API_Flow](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/RDK_X5_Newly/pydev_10_01_flow1.png)
</center>

1. `srcampy.Camera()`

   创建 MIPI 摄像头对象

2. `get_display_res()`

   获取 HDMI 显示分辨率，返回：宽、高

3. `cam.open_cam(pipe_id, video_index, fps, width, height, raw_height, rae_width)`
   打开摄像头，输入：camera 对应的 pipeline 通道号、camera 对应的 host 编号、帧率、输出宽度、输出高度、原始宽度、原始高度

4. `srcampy.Display()`

   创建 HDMI 显示对象

5. `disp.display(layer, width, height)`

   初始化显示层，输入：显示层号、宽、高

6. `srcampy.bind(camera, display)`

   绑定摄像头和显示，输入：摄像头对象、显示对象

7. `hbm_runtime.HB_HBMRuntime(model_path)`

   加载模型，输入：模型文件路径

8. `model.set_scheduling_params(priority, bpu_cores)`

   设置模型调度参数，输入：优先级(0-255)、BPU核心列表
<!-- 9. `common.print_model_info(model)` - 打印模型信息（输入输出维度、量化参数等） -->

9. `load_class_names(class_file)`

   加载类别名称，输入：类别文件路径，返回：类别名称列表

10. `cam.get_img(chn, width, height)`

      获取摄像头图像帧，输入：通道号（默认为2）、宽、高，返回：NV12 格式图像数据

11. `split_nv12_bytes(img, width, height)`

      分离 NV12 图像的 Y 和 UV 分量，输入：NV12 图像数据、宽、高，返回：Y 分量、UV 分量

12. `resize_nv12_yuv(y, uv, target_w, target_h)`

      缩放 NV12 图像，输入：Y 分量、UV 分量、目标宽、目标高，返回：缩放后的 Y、UV 分量

13. `model.run(input_tensor)`

      执行模型推理，输入：预处理后的输入张量字典，返回：模型输出字典

14. `dequantize_outputs(outputs, output_quants)`

      结果反量化，输入：模型输出字典、输出量化参数，返回：float32 类型数据

15. `decode_outputs(output_names, fp32_outputs, strides, anchors, classes_num)`

      解码 YOLO 模型输出，输入：输出名称列表、反量化后的输出、步长、锚点、类别数，返回：预测结果

16. `filter_predictions(predictions, score_threshold)`

      过滤预测结果，输入：预测结果、置信度阈值，返回：检测框、置信度、类别

17. `NMS(boxes, scores, classes, iou_threshold)`

      非极大值抑制，输入：检测框、置信度、类别、IoU 阈值，返回：保留的索引

18. `scale_coords_back(boxes, orig_w, orig_h, model_w, model_h, resize_type)`

      调整检测框至原图尺寸，输入：检测框、原图宽高、模型输入宽高、缩放类型，返回：缩放后的检测框

19. `draw_detections_on_disp(display, boxes, cls_ids, scores, class_names, color_map, chn)`

      在显示层绘制检测结果，输入：显示对象、检测框、类别ID、置信度、类别列表、颜色映射、通道号

20. `srcampy.unbind(camera, display)`
   
      解绑摄像头和显示

21. `cam.close_cam()`
   
      关闭摄像头

22. `disp.close()`

      关闭显示

#### 图像采集保存主要接口：

<center>
![API_Flow](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/RDK_X5_Newly/pydev_10_01_flow2.png)
</center>

1. `libsrcampy.Camera()`

   创建 MIPI 摄像头对象

2. `cam.open_cam(pipe_id, video_index, fps, width, height,)`

   打开摄像头，输入：camera 对应的 pipeline 通道号、camera 对应的 host 编号、帧率、宽度、高度

3. `cam.get_img(chn)`

   获图像，输入：获取图像的模块，返回：YUV 格式图像数据

4. `file.write(data)`

   写入文件数据，输入：图像数据

5. `cam.close_cam()`

   关闭摄像头

#### 图像缩放处理主要接口：

<center>
![API_Flow](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/RDK_X5_Newly/pydev_10_01_flow3.png)
</center>

1. `libsrcampy.Camera()`

   创建 VPS（视频处理系统）对象

2. `vps.open_vps(grp_id, chn_id, input_w, input_h, output_w, output_h)`

   打开 VPS 通道，输入：组ID、通道ID、输入宽高、输出宽高

3. `file.read()`

   读取文件数据，返回：图像数据（字节流）

4. `vps.set_img(img_data)`

   设置输入图像数据，输入：YUV 图像数据（字节流）

5. `vps.get_img(chn, width, height)`

   获取处理后的图像，输入：通道号、宽、高，返回：处理后的 YUV 图像数据（字节流）

6. `file.write(data)`

   写入处理后的图像数据，输入：图像数据（字节流）

7. `vps.close_cam()`

   关闭 VPS

#### 图像裁剪缩放主要接口：

<center>
![API_Flow](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/RDK_X5_Newly/pydev_10_01_flow4.png)
</center>

1. `libsrcampy.Camera()`

   创建 VPS（视频处理系统）对象

2. `vps.open_vps(grp_id, chn_id, input_w, input_h, output_w, output_h, crop_rect)`

   打开 VPS 通道并设置裁剪区域，输入：组ID、通道ID、输入宽高、输出宽高、裁剪区域[x, y, w, h]

3. `file.read()`

   读取文件数据，返回：图像数据（字节流）

4. `vps.set_img(img_data)`

   设置输入图像数据，输入：YUV 图像数据（字节流）

5. `vps.get_img(chn, width, height)`

   获取处理后的图像，输入：通道号、宽、高，返回：处理后的 YUV 图像数据（NV12 格式，字节流）

6. `file.write(data)`

   写入处理后的图像数据，输入：图像数据（字节流）

7. `vps.close_cam()`

   关闭 VPS

#### 实时推流显示主要接口：

<center>
![API_Flow](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/RDK_X5_Newly/pydev_10_01_flow5.png)
</center>

1. `libsrcampy.Display()`

   创建 HDMI 显示对象

2. `disp.display(layer, width, height)`

   初始化显示层，输入：显示层号、宽、高

3. `libsrcampy.Camera()`

   创建 MIPI 摄像头对象

4. `cam.open_cam(pipe_id, video_index, fps, width, height,)`

   打开摄像头，输入：camera 对应的 pipeline 通道号、camera 对应的 host 编号、帧率、宽度、高度

5. `libsrcampy.bind(camera, display)`

   绑定摄像头和显示，输入：摄像头对象、显示对象，返回：绑定结果

6. `libsrcampy.unbind(camera, display)`

   解绑摄像头和显示

7. `disp.close()`

   关闭显示

8. `cam.close_cam()`

   关闭摄像头

### FAQ

Q: 运行示例时提示摄像头初始化失败怎么办？\
A: 请检查 MIPI 摄像头是否正确连接，并确保摄像头驱动已正确加载。可以尝试重启设备。

Q: HDMI 显示不正常或没有输出怎么办？\
A: 请检查 HDMI 连接，并确保显示服务已停止（如使用 systemctl stop lightdm）。

Q: 如何调整检测阈值？\
A: 在代码中修改 `--score-thres` 的值，例如改为 0.5 可以提高检测灵敏度。

Q: 如何修改显示的分辨率？\
A: 修改代码中的 `sensor_width` 和 `sensor_height` 变量，但需要注意显示设备是否支持该分辨率。

Q: 运行示例时帧率很低怎么办？\
A: 可以尝试使用更轻量的模型，或者调整摄像头的采集分辨率。

Q: 如何保存检测结果图像？\
A: 可以在代码中添加图像保存逻辑，例如使用 `cv2.imwrite()` 保存处理后的图像。

<!-- Q: 如何添加新的检测类别？\
A: 需要修改 `get_classes()` 函数，添加新的类别名称，并重新训练模型或使用支持新类别的模型。 -->


