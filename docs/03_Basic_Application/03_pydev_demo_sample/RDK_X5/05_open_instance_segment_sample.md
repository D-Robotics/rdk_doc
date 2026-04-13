---
sidebar_position: 5
---

# 开放域实例

## 示例简介
开放域实例分割示例是一组位于 `/app/pydev_demo/05_open_instance_seg_sample/` 中的 **Python 接口** 开发代码示例，用于演示如何使用 `hbm_runtime` 模块进行开放域目标检测任务。该示例使用 YOLO-World 模型，支持通过文本提示词（prompts）进行灵活的目标检测，无需预定义类别列表。

包含的模型示例：
```
root@ubuntu:/app/pydev_demo/05_open_instance_seg_sample$ tree -L 1
.
└── 01_yolo_world
```

## 效果展示
示例会检测图像中与文本提示词匹配的目标，并绘制检测框、类别名称和置信度。

![output-img](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/RDK_X5_Newly/pydev_05_running_yoloworld.png)

## 硬件准备

### 硬件连接
该示例只需要 RDK 开发板本身，无需额外的外设连接。确保开发板正常供电并启动系统。

![connect-img](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/RDK_X5_Newly/pydev_01_hw_connect.png)

## 快速开始

### 代码以及板端位置

进入到 `/app/pydev_demo/05_open_instance_seg_sample/` 位置，可以看到包含了开放域实例分割示例文件夹：

```
root@ubuntu:/app/pydev_demo/05_open_instance_seg_sample# tree
.
└── 01_yolo_world
    ├── dog.jpeg
    ├── offline_vocabulary_embeddings.json
    ├── result.jpg
    ├── yolo_world.bin
    └── yoloworld.py
```

### 编译以及运行
Python 示例无需编译，直接运行即可。

```
cd /app/pydev_demo/05_open_instance_seg_sample/01_yolo_world
python yoloworld.py
```

### 执行效果

```
root@ubuntu:/app/pydev_demo/05_open_instance_seg_sample/01_yolo_world# python yoloworld.py 
[BPU_PLAT]BPU Platform Version(1.3.6)!
[HBRT] set log level as 0. version = 3.15.55.0
[DNN] Runtime version = 1.24.5_(3.15.55 HBRT)
[A][DNN][packed_model.cpp:247][Model](2000-01-01,10:59:17.916.291) [HorizonRT] The model builder version = 1.23.6
[W][DNN]bpu_model_info.cpp:491][Version](2000-01-01,10:59:18.311.878) Model: yolo_world. Inconsistency between the hbrt library version 3.15.55.0 and the model build version 3.15.49.0 detected, in order to ensure correct model results, it is recommended to use compilation tools and the BPU SDK from the same OpenExplorer package.
...
[Saved] Result saved to: result.jpg
```

## 详细介绍

### 示例程序参数选项说明
开放域实例分割示例支持命令行参数配置，如果不指定参数，程序会使用默认值自动加载同目录下的测试图像进行推理。

| 参数 | 说明 | 类型 | 默认值 |
|------|------|------|--------|
| `--model-path` | BPU 量化模型路径（`.bin`） | str | `/app/model/basic/yolo_world.bin` |
| `--test-img` | 输入测试图像路径 | str | `dog.jpeg` |
| `--label-file` | 词汇表嵌入文件路径（JSON 格式） | str | `offline_vocabulary_embeddings.json` |
| `--prompts` | 文本提示词，逗号分隔（如 `dog,cat`） | str | `dog` |
| `--img-save-path` | 检测结果图像保存路径 | str | `result.jpg` |
| `--priority` | 推理优先级（0~255，255为最高） | int | `0` |
| `--bpu-cores` | BPU 核心索引列表（如 `0 1`） | int list | `[0]` |
| `--nms-thres` | 非极大值抑制（NMS）阈值 | float | `0.45` |
| `--score-thres` | 置信度阈值 | float | `0.05` |

### 软件架构说明

本部分介绍开放域实例分割示例的软件架构和工作流程，说明程序从模型加载到结果输出的完整执行过程，帮助理解代码的整体结构和数据流向。

<center>
![software_arch](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/RDK_X5_Newly/pydev_05_yoloworld_sample_software_arch.png)
</center>

1. **词汇表加载** - 从 JSON 文件加载词汇表嵌入向量
2. **提示词处理** - 解析用户提供的文本提示词，从词汇表中查找对应的嵌入向量
3. **模型加载** - 使用 `hbm_runtime` 模块加载预编译的模型文件
4. **图像加载** - 读取输入测试图像
5. **图像预处理** - 将图像缩放至模型输入尺寸，BGR 转 RGB，转换为 NCHW 格式
6. **模型推理** - 在 BPU 上执行模型前向计算，输入图像张量和文本嵌入向量
7. **结果后处理** - 使用 post_utils 库解析推理结果，进行解码、置信度过滤和 NMS 非极大值抑制
8. **结果输出** - 在图像上绘制检测框并保存结果图像

### API 流程说明

本部分列出示例程序中使用的主要 API 接口，说明各接口的功能、输入参数和返回值，帮助开发者快速了解代码实现细节和接口调用方式。其中， `cv2.` 开头的是 OPENCV 的通用接口，`np.` 开头的是 Numpy 的通用接口，其余为 hbm_runtime 及其相关接口。

<center>
![API_Flow](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/RDK_X5_Newly/pydev_05_yoloworld_sample_api_flow.png)
</center>

1. `json.load(file)`

   加载词汇表嵌入文件，输入：文件路径，返回：词汇表字典

2. `hbm_runtime.HB_HBMRuntime(model_path)`

   加载预编译模型，输入：模型文件路径

3. `load_image(image_path)`

   加载测试图像，输入：图像文件路径

4. `cv2.resize(img, (0,0), fx,fy)`

   图像缩放，输入：图像、目标尺寸、水平方向缩放因子、垂直方向缩放因子,返回:缩放后的图像

5. `np.transpose`

   数组转置，将图像从 HWC 转为 NCHW 格式

6. `model.run(input_dict)`

   执行模型推理，输入：包含图像张量和文本嵌入，返回：模型输出

7. `cv2.minMaxLoc(array)`

   查找数组中的最大值和位置，输入：模型输出，返回：置信度最小值、置信度最大值、最小值位置、最大值位置

8. `NMS(boxes，scores_np，nms_thres)`

   非极大值抑制，输入：检测框、置信度、类别、IoU 阈值, 返回：保留的索引

9. `draw_boxes(img, boxes, cls_ids, scores, classes, rdk_colors)`

    绘制检测结果，输入：图像、检测框、类别ID、置信度、类别列表、颜色映射
   
10. `cv2.imwrite(img_path,img)`

    保存最终结果，输入：保存路径，图像

### FAQ

Q: 运行示例时出现 "ModuleNotFoundError: No module named 'hbm_runtime'" 错误怎么办？\
A: 请确保已正确安装 RDK 的 Python 环境， hbm_runtime 模块等官方提供的专用推理库。

Q: 如何更换测试图片？\
A: 在运行命令时使用 `--test-img` 参数指定图片路径，例如：`python yoloworld.py --test-img your_image.jpg`

Q: 提示词在词汇表中找不到怎么办？\
A: 请检查 `--label-file` 指定的词汇表文件是否包含该提示词。可以打开 JSON 文件查看可用的词汇列表。

Q: 如何更换检测目标？\
A: 修改 `--prompts` 参数, 例如： `--prompts cat`

Q: 检测结果不准确怎么办？\
A: 可以尝试调整 `--score-thres` 和 `--nms-thres` 参数，或者使用更具体的提示词。同时确保输入图像质量良好。

Q: 如何调整检测阈值？\
A: 在代码中修改 `--score-thres` 的值，例如改为 0.1 可以提高检测灵敏度。

Q: 如何查看词汇表中可用的提示词？\
A: 打开 `offline_vocabulary_embeddings.json` 文件，查看其中的键（keys）列表，这些就是可用的提示词。

Q: 如何获取其他预训练的 YOLO-World 模型？\
A: 可以参考 [model_zoo 仓库 ](https://github.com/D-Robotics/rdk_model_zoo) 或者 [ 工具链的基础模型仓库 ](https://github.com/D-Robotics/hobot_model)
