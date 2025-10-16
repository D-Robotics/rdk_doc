---
sidebar_position: 12
---

# 文本检测与识别-PaddleOCR

本示例基于BPU推理引擎运行 PaddleOCR 模型进行文本检测与识别，支持中文场景的 OCR 识别与可视化，本示例代码位于`/app/cdev_demo/bpu/08_OCR_sample/01_paddleOCR/`目录下。


## 模型说明
- 简介：

    本示例实现了基于 PaddleOCR v3 的中文文字检测与识别（两阶段 OCR）任务。整体流程包括检测文字区域（检测模型）与逐区域识别文字内容（识别模型）。

- HBM 模型名称：

- 检测模型（Detection）：cn_PP-OCRv3_det_infer-deploy_640x640_nv12.hbm

- 识别模型（Recognition）：cn_PP-OCRv3_rec_infer-deploy_48x320_rgb.hbm

- 输入格式：

    - 检测模型：BGR 图像 → resize 到 640×640，转换为 NV12 格式（Y、UV 分离）

    - 识别模型：旋转裁剪后的 BGR 文本块图像 → resize 到 48×320，归一化、转为 RGB 格式

- 输出：

    - 检测模型：分割概率图（1×1×H×W），后处理得到文本框坐标

    - 识别模型：字符 token 的 logits，CTC 解码后得到识别文本字符串

## 功能说明
- 模型加载

    加载文字检测与识别模型，并解析输入输出的相关信息。

- 输入预处理

    - 检测模型：将原图 resize 到 640×640，并转换为 NV12 格式（用于 BPU 推理）。

    - 识别模型：将每个旋转裁剪后的文本块 resize 到 48×320，转为 RGB 格式并归一化，最终转为 NCHW 结构。

- 推理执行

    调用 .infer() 方法进行前向推理，输出包括概率图（检测）与 logits（识别）。

- 结果后处理

    - 检测模型：

        - 对概率图二值化（使用设定阈值）

        - 查找文本区域轮廓并扩张

        - 提取旋转框并裁剪图像区域

    - 识别模型：

        - 使用 CTCLabelDecode 对 logits 解码，映射为文字字符串

    最终将识别结果以红色文字标注在空白图中，与原图拼接可视化。

## 环境依赖
在编译运行前，请确保安装以下依赖：
```bash
sudo apt update
sudo apt install -y libgflags-dev libpolyclipping-dev
```

## 目录结构
```text
.
|-- CMakeLists.txt                 # CMake 构建脚本：目标/依赖/包含路径/链接库
|-- FangSong.ttf                   # 中文字体（用于在可视化画布上渲染识别文字）
|-- README.md                      # 使用说明（当前文件）
|-- inc
|   `-- paddleOCR.hpp              # OCR 封装头文件：检测/识别类接口（加载/预处理/推理/后处理）
`-- src
    |-- main.cc                    # 程序入口：解析参数→检测→裁剪→识别→可视化→保存
    `-- paddleOCR.cc               # 具体实现：多边形框生成、裁剪、CTC 解码、文本渲染
```

## 编译工程
- 配置与编译
    ```bash
    mkdir build && cd build
    cmake ..
    make -j$(nproc)
    ```

## 模型下载
若在程序运行时未找到模型，可通过下列命令下载
```bash
# 检测模型
wget https://archive.d-robotics.cc/downloads/rdk_model_zoo/rdk_s100/paddle_ocr/cn_PP-OCRv3_det_infer-deploy_640x640_nv12.hbm
# 识别模型
wget https://archive.d-robotics.cc/downloads/rdk_model_zoo/rdk_s100/paddle_ocr/cn_PP-OCRv3_rec_infer-deploy_48x320_rgb.hbm
```

## 参数说明
| 参数名                | 说明                                  | 默认值                                                                         |
| ------------------ | ----------------------------------- | --------------------------------------------------------------------------- |
| `--det_model_path` | 文本检测模型（`.hbm`）                    | `/opt/hobot/model/s100/basic/cn_PP-OCRv3_det_infer-deploy_640x640_nv12.hbm` |
| `--rec_model_path` | 文本识别模型（`.hbm`）                    | `/opt/hobot/model/s100/basic/cn_PP-OCRv3_rec_infer-deploy_48x320_rgb.hbm`  |
| `--test_image`     | 输入测试图像路径                            | `/app/res/assets/gt_2322.jpg`                                        |
| `--label_file`     | 识别标签文件                                | `/app/res/labels/ppocr_keys_v1.txt`                       |
| `--threshold`      | 文本区域二值化阈值（检测后处理用）                   | `0.5`                                                        |
| `--ratio_prime`    | 文本框膨胀系数（检测后处理用，影响多边形外扩）        | `2.7`                                                        |



## 快速运行
- 运行模型
    - 确保在`build`目录中
    - 使用默认参数
        ```bash
        ./paddleOCR
        ```
    - 指定参数运行
        ```bash
        ./paddleOCR \
            --det_model_path /opt/hobot/model/s100/basic/cn_PP-OCRv3_det_infer-deploy_640x640_nv12.hbm \
            --rec_model_path /opt/hobot/model/s100/basic/cn_PP-OCRv3_rec_infer-deploy_48x320_rgb.hbm \
            --test_image     /app/res/assets/gt_2322.jpg \
            --label_file     /app/res/labels/ppocr_keys_v1.txt \
            --threshold 0.5 \
            --ratio_prime 2.7
        ```
- 查看结果

    运行成功后，会将结果绘制在原图上，并保存到build/result.jpg
    ```bash
    [Saved] Result saved to: result.jpg
    ```

## 注意事项
- 输出结果存储为result.jpg，用户可自行查看。

- 如需了解更多部署方式或模型支持情况，请参考官方文档或联系平台技术支持。
