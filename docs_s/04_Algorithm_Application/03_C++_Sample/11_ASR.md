---
sidebar_position: 11
---

# 自动语音识别-ASR

本示例基于BPU推理引擎运行语音识别模型，实现对 .wav 格式语音文件的自动转写，输出对应的文字内容，本示例代码位于`/app/cdev_demo/bpu/07_speech_sample/01_asr/`目录下。


## 模型说明
- 简介：

    ASR（Automatic Speech Recognition）自动语音识别模型用于将音频信号转换为文本。输入为单通道语音波形（经过采样率转换和标准化处理），输出为字符级别的 token 序列。配合字典（vocab）文件使用，可实现中文语音转写。本示例使用量化后的 .hbm 模型。

- HBM模型名称：asr.hbm

- 输入格式：音频波形，单通道，采样率为 16kHz，最大长度为 30000（样本点）

- 输出：字符 token 的概率分布（logits），通过 argmax 解码后映射为识别文本

## 功能说明
- 模型加载

   加载 ASR 模型，并自动解析模型输入输出形状和量化信息。

- 输入预处理

    使用 SoundFile 读取音频（支持 .wav），将音频：

    - 转为单通道
    - 重采样至目标采样率（默认 16kHz）
    - 标准化为零均值单位方差（z-score）
    - 补零或截断至固定长度（如 30000）
    - 支持生成器方式处理长音频，实现流式识别。

- 推理执行

    采用 .infer() 方法完成推理。

- 结果后处理

    从输出 logits 中获取 token 索引，结合 vocab 字典文件（JSON 格式）映射为字符，输出最终识别文本。


## 环境依赖
在编译运行前，请确保安装以下依赖：
```bash
sudo apt update
sudo apt install -y libgflags-dev libsndfile1-dev libsamplerate0-dev
```

## 目录结构
```text
.
|-- CMakeLists.txt                  # CMake 构建脚本：目标/依赖/包含与链接配置
|-- README.md                       # 使用说明（当前文件）
|-- inc
|   |-- asr.hpp                     # ASR 推理封装头文件（加载/预处理/推理/后处理接口）
|   `-- audio_chunk_reader.hpp      # 音频切片读取器：读文件→重采样→分片输出
`-- src
    |-- asr.cc                      # ASR 推理实现：输入写入、前向计算、CTC 解码等
    |-- audio_chunk_reader.cc       # 切片读取实现：libsndfile + libsamplerate 流式分块
    `-- main.cc                     # 程序入口：参数解析→循环分片→推理→拼接转写文本
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
wget https://archive.d-robotics.cc/downloads/rdk_model_zoo/rdk_s100/asr/asr.hbm
```

## 参数说明
| 参数名           | 说明                               | 默认值                                   |
| -------------- | -------------------------------- | ------------------------------------- |
| `--model_path` | 模型文件路径（`.hbm`）                   | `/opt/hobot/model/s100/basic/asr.hbm` |
| `--test_sound` | 输入音频文件路径（`.wav`）                 | `/app/res/assets/chi_sound.wav`       |
| `--vocab_file` | 词表（JSON），映射 **class id → token** | `/app/res/labels/vocab.json`          |

## 快速运行
- 运行模型
    - 确保在`build`目录中
    - 使用默认参数
        ```bash
        ./asr
        ```
    - 指定参数运行
        ```bash
        ./asr \
            --model_path /opt/hobot/model/s100/basic/asr.hbm \
            --test_sound /app/res/assets/chi_sound.wav \
            --vocab_file /app/res/labels/vocab.json
        ```
- 查看结果

    运行成功后，会将结果打印出来
    ```bash
    我是来自阿里云的大规模语言磨型过叫通意千问||
    ```

## 注意事项
- 如需了解更多部署方式或模型支持情况，请参考官方文档或联系平台技术支持。
