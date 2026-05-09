---
sidebar_position: 12
---

# 文本检测与识别-PaddleOCR

本示例基于 `hbm_runtime` 推理引擎运行 PaddleOCR 模型进行文本检测与识别，支持中文场景的 OCR 识别与可视化，本示例代码位于`/app/pydev_demo/08_OCR_sample/01_paddleOCR/`目录下，本示例代码位于/app/pydev_demo/02_detection_sample/02_ultralytics_yolo11/ 目录下。


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

- 模型下载地址（程序自动下载）：

    ```bash
    # 检测模型
    https://archive.d-robotics.cc/downloads/rdk_model_zoo/rdk_s100/paddle_ocr/cn_PP-OCRv3_det_infer-deploy_640x640_nv12.hbm
    # 识别模型
    https://archive.d-robotics.cc/downloads/rdk_model_zoo/rdk_s100/paddle_ocr/cn_PP-OCRv3_rec_infer-deploy_48x320_rgb.hbm
    ```
## 功能说明
- 模型加载

    使用 `hbm_runtime` 分别加载文字检测与识别模型，并解析输入输出名称和形状信息，支持设置推理优先级与 BPU 核绑定。

- 输入预处理

    - 检测模型：将原图 resize 到 640×640，并转换为 NV12 格式（用于 BPU 推理）。

    - 识别模型：将每个旋转裁剪后的文本块 resize 到 48×320，转为 RGB 格式并归一化，最终转为 NCHW 结构。

- 推理执行

    调用 .run() 方法进行前向推理，输出包括概率图（检测）与 logits（识别）。

- 结果后处理

    - 检测模型：

        - 对概率图二值化（使用设定阈值）

        - 查找文本区域轮廓并扩张

        - 提取旋转框并裁剪图像区域

    - 识别模型：

        - 使用 CTCLabelDecode 对 logits 解码，映射为文字字符串

    最终将识别结果以红色文字标注在空白图中，与原图拼接可视化。

## 环境依赖
- 确保安装了pydev中的环境依赖
    ```bash
    pip install -r ../../requirements.txt
    ```
- 安装OCR处理所需的包
    ```bash
    pip install pyclipper==1.3.0.post6 Pillow==9.0.1 paddlepaddle
    ```

## 目录结构
```text
.
├── FangSong.ttf                # 中文显示用字体
├── paddle_ocr.py               # 主程序，完成文本检测与识别
├── postprocess/                # 后处理逻辑（排序、合并、解码等）
└── README.md                   # 使用说明
```

## 参数说明
| 参数名                | 默认值                                        | 说明                             |
| ------------------ | ----------------------------------------------- | -------------------------------- |
| `--det-model-path` | `/opt/hobot/model/s100/basic/cn_PP-OCRv3_det_infer-deploy_640x640_nv12.hbm` | 文本检测模型路径                  |
| `--rec-model-path` | `/opt/hobot/model/s100/basic/cn_PP-OCRv3_rec_infer-deploy_48x320_rgb.hbm`   | 文本识别模型路径                  |
| `--priority`       | `0`                                             | 模型推理优先级，数值越大优先级越高  |
| `--bpu-cores`      | `[0]`                                           | 指定运行推理的 BPU 核心索引        |
| `--test-img`       | `/app/res/assets/gt_2322.jpg`                      | 输入图像路径                      |
| `--label-file`     | `/app/res/labels/ppocr_keys_v1.txt`                | 文本识别所需标签文件路径           |
| `--img-save-path`  | `result.jpg`                                    | 推理结果图像的保存路径             |
| `--threshold`      | `0.5`                                           | 文本区域二值化的阈值               |
| `--ratio-prime`    | `2.7`                                           | 文本框膨胀系数，用于检测框的形态调整 |

## 快速运行
- 运行模型
    - 使用默认参数
        ```bash
        python paddle_ocr.py
        ```
    - 指定参数运行
        ```bash
        python paddle_ocr.py \
        --det-model-path /opt/hobot/model/s100/basic/cn_PP-OCRv3_det_infer-deploy_640x640_nv12.hbm \
        --rec-model-path /opt/hobot/model/s100/basic/cn_PP-OCRv3_rec_infer-deploy_48x320_rgb.hbm \
        --test-img /app/res/assets/gt_2322.jpg \
        --label-file /app/res/labels/ppocr_keys_v1.txt \
        --img-save-path result.jpg \
        --priority 0 \
        --bpu-cores 0 \
        --threshold 0.5 \
        --ratio-prime 2.7
        ```
- 查看结果

    运行成功后，会将结果绘制在原图上，并保存到 --img-save-path 指定路径
    ```bash
    [Saved] Result saved to: result.jpg
    ```

## 注意事项
- 若指定模型路径不存在，程序将尝试自动下载模型。
