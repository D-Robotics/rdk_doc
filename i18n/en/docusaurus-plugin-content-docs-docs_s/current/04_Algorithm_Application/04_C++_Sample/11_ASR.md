---
sidebar_position: 11
---

# Automatic Speech Recognition - ASR

This example runs a speech recognition model based on the BPU inference engine to automatically transcribe `.wav` audio files into corresponding text. The example code is located in the `/app/cdev_demo/bpu/07_speech_sample/01_asr/` directory.

## Model Description
- **Introduction**:

    The ASR (Automatic Speech Recognition) model converts audio signals into text. The input is a single-channel audio waveform (after sample rate conversion and normalization), and the output is a character-level token sequence. When used together with a vocabulary (`vocab`) file, it supports Chinese speech transcription. This example uses a quantized `.hbm` model.

- **HBM Model Name**: asr.hbm

- **Input Format**: Audio waveform, single-channel, sampled at 16kHz, with a maximum length of 30,000 samples.

- **Output**: Probability distribution (logits) over character tokens; after argmax decoding, mapped to recognized text.

## Functionality Overview
- **Model Loading**

   Loads the ASR model and automatically parses its input/output shapes and quantization information.

- **Input Preprocessing**

    Reads audio using SoundFile (supports `.wav`) and performs the following steps:

    - Converts to single-channel
    - Resamples to target sample rate (default: 16kHz)
    - Normalizes to zero-mean and unit-variance (z-score)
    - Pads or truncates to a fixed length (e.g., 30,000 samples)
    - Supports generator-based processing for long audio, enabling streaming recognition.

- **Inference Execution**

    Performs inference using the `.infer()` method.

- **Post-processing**

    Extracts token indices from output logits and maps them to characters using the `vocab` dictionary file (in JSON format), producing the final transcribed text.

## Environment Dependencies
Before compiling and running, ensure the following dependencies are installed:
```bash
sudo apt update
sudo apt install -y libgflags-dev libsndfile1-dev libsamplerate0-dev
```

## Directory Structure
```text
.
|-- CMakeLists.txt                  # CMake build script: target/dependency/include/link configuration
|-- README.md                       # Usage instructions (this file)
|-- inc
|   |-- asr.hpp                     # ASR inference wrapper header (interfaces for loading/preprocessing/inference/post-processing)
|   `-- audio_chunk_reader.hpp      # Audio chunk reader: reads file → resamples → outputs chunks
`-- src
    |-- asr.cc                      # ASR inference implementation: input writing, forward computation, CTC decoding, etc.
    |-- audio_chunk_reader.cc       # Chunk reading implementation: libsndfile + libsamplerate for streaming chunking
    `-- main.cc                     # Program entry point: argument parsing → loop over chunks → inference → concatenate transcribed text
```

## Build Instructions
- **Configuration and Compilation**
    ```bash
    mkdir build && cd build
    cmake ..
    make -j$(nproc)
    ```

## Model Download
If the model is not found during runtime, download it using the following command:
```bash
wget https://archive.d-robotics.cc/downloads/rdk_model_zoo/rdk_s100/asr/asr.hbm
```

## Parameter Description
| Parameter        | Description                                      | Default Value                                |
| ---------------- | ------------------------------------------------ | -------------------------------------------- |
| `--model_path`   | Path to the model file (`.hbm`)                  | `/opt/hobot/model/s100/basic/asr.hbm`        |
| `--test_sound`   | Path to the input audio file (`.wav`)            | `/app/res/assets/chi_sound.wav`              |
| `--vocab_file`   | Vocabulary file (JSON), mapping **class id → token** | `/app/res/labels/vocab.json`             |

## Quick Start
- **Run the Model**
    - Ensure you are in the `build` directory.
    - Run with default parameters:
        ```bash
        ./asr
        ```
    - Run with specified parameters:
        ```bash
        ./asr \
            --model_path /opt/hobot/model/s100/basic/asr.hbm \
            --test_sound /app/res/assets/chi_sound.wav \
            --vocab_file /app/res/labels/vocab.json
        ```
- **View Results**

    Upon successful execution, the result will be printed:
    ```bash
    I am Qwen, a large-scale language model developed by Alibaba Cloud.||
    ```

## Notes
- For more information about deployment options or model support, please refer to the official documentation or contact platform technical support.