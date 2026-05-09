---
sidebar_position: 11
---

# 自动语音识别-ASR

本示例基于 `hbm_runtime` 推理引擎运行语音识别模型，实现对 .wav 格式语音文件的自动转写，输出对应的文字内容，本示例代码位于`/app/pydev_demo/07_speech_sample/01_asr/` 目录下。


## 模型说明
- 简介：

    ASR（Automatic Speech Recognition）自动语音识别模型用于将音频信号转换为文本。输入为单通道语音波形（经过采样率转换和标准化处理），输出为字符级别的 token 序列。配合字典（vocab）文件使用，可实现中文语音转写。本示例使用量化后的 .hbm 模型。

- HBM模型名称：asr.hbm

- 输入格式：音频波形，单通道，采样率为 16kHz，最大长度为 30000（样本点）

- 输出：字符 token 的概率分布（logits），通过 argmax 解码后映射为识别文本

- 模型下载地址（程序自动下载）：

    ```bash
    https://archive.d-robotics.cc/downloads/rdk_model_zoo/rdk_s100/asr/asr.hbm
    ```
## 功能说明
- 模型加载

    使用 `hbm_runtime` 加载 ASR 模型，并自动解析模型输入输出形状和量化信息。

- 输入预处理

    使用 SoundFile 读取音频（支持 .wav），将音频：

    - 转为单通道
    - 重采样至目标采样率（默认 16kHz）
    - 标准化为零均值单位方差（z-score）
    - 补零或截断至固定长度（如 30000）
    - 支持生成器方式处理长音频，实现流式识别。

- 推理执行

    采用 .run() 方法完成推理，输出 logits 张量。

- 结果后处理

    使用 np.argmax() 从输出 logits 中获取 token 索引，结合 vocab 字典文件（JSON 格式）映射为字符，输出最终识别文本。


## 环境依赖
- 确保安装了pydev中的环境依赖
    ```bash
    pip install -r ../../requirements.txt
    ```
- 安装soundfile包
    ```bash
    pip install soundfile==0.13.1
    ```

## 目录结构
```text
01_asr/
├── asr.py                      # 主推理脚本
```

## 参数说明
| 参数名            | 说明                                          | 默认值                  |
| ----------------- | -------------------------------------------- | ----------------------------|
| `--model-path`    | 模型路径（`.hbm` 格式）                        | `/opt/hobot/model/s100/basic/asr.hbm`                    |
| `--audio-file`    | 输入音频文件（支持 `.wav` 或 `.flac`）         | `/app/res/assets/chi_sound.wav` |
| `--vocab-file`    | 词表文件，映射 token → id                     | `/app/res/labels/vocab.json`    |
| `--priority`      | 推理优先级，0\~255，数值越大越优先             | `0`                           |
| `--bpu-cores`   ` | 指定使用哪些 BPU 核心（如：`--bpu-cores 0 1`） | `[0]`                         |
| `--audio_maxlen`  | 音频裁剪/填充后的固定长度（单位：采样点数）     | `30000`                         |
| `--new_rate`      | 目标采样率，音频会自动重采样为该采样率          | `16000`                         |


## 快速运行
- 运行模型
    - 使用默认参数
        ```bash
        python asr.py
        ```
    - 指定参数运行
        ```bash
        python asr.py \
        --model-path /opt/hobot/model/s100/basic/asr.hbm \
        --audio-file /app/res/assets/chi_sound.wav \
        --vocab-file /app/res/labels/vocab.json \
        --priority 0 \
        --bpu-cores 0 \
        --audio_maxlen 30000 \
        --new_rate 16000
        ```
- 查看结果

    运行成功后，会将结果打印出来
    ```bash
    我是来自阿里云的大规模语言磨型过叫通意千问||
    ```

## 注意事项
- 若指定模型路径不存在，程序将尝试自动下载模型。
