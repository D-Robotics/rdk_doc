---
sidebar_position: 11
---

# Automatic Speech Recognition - ASR

This example runs a speech recognition model based on the `hbm_runtime` inference engine to automatically transcribe `.wav` audio files into corresponding text. The example code is located in the `/app/pydev_demo/07_speech_sample/01_asr/` directory.

## Model Description
- **Introduction**:

    The ASR (Automatic Speech Recognition) model converts audio signals into text. The input is a single-channel audio waveform (after sample rate conversion and normalization), and the output is a character-level token sequence. When used together with a vocabulary (`vocab`) file, it supports Chinese speech transcription. This example uses a quantized `.hbm` model.

- **HBM Model Name**: `asr.hbm`

- **Input Format**: Audio waveform, single-channel, sampled at 16kHz, with a maximum length of 30,000 samples.

- **Output**: Probability distribution (logits) over character tokens. After decoding via argmax, the logits are mapped to the recognized text.

- **Model Download URL** (automatically downloaded by the program):

    ```bash
    https://archive.d-robotics.cc/downloads/rdk_model_zoo/rdk_s100/asr/asr.hbm
    ```

## Functionality Description
- **Model Loading**

    Load the ASR model using `hbm_runtime`, which automatically parses the model's input/output shapes and quantization information.

- **Input Preprocessing**

    Use SoundFile to read audio files (supports `.wav`), and process the audio as follows:

    - Convert to single-channel
    - Resample to the target sample rate (default: 16kHz)
    - Normalize to zero-mean and unit-variance (z-score)
    - Pad or truncate to a fixed length (e.g., 30,000 samples)
    - Support generator-based processing for long audio inputs, enabling streaming recognition.

- **Inference Execution**

    Perform inference using the `.run()` method, producing a logits tensor as output.

- **Post-processing**

    Use `np.argmax()` to obtain token indices from the output logits, then map them to characters using the vocab dictionary file (in JSON format) to produce the final transcribed text.

## Environment Dependencies
- Ensure that the dependencies in `pydev` are installed:
    ```bash
    pip install -r ../../requirements.txt
    ```
- Install the `soundfile` package:
    ```bash
    pip install soundfile==0.13.1
    ```

## Directory Structure
```text
01_asr/
├── asr.py                      # Main inference script
```

## Parameter Description
| Parameter         | Description                                               | Default Value                                     |
| ----------------- | --------------------------------------------------------- | -------------------------------------------------|
| `--model-path`    | Path to the model file (`.hbm` format)                    | `/opt/hobot/model/s100/basic/asr.hbm`            |
| `--audio-file`    | Input audio file (supports `.wav` or `.flac`)             | `/app/res/assets/chi_sound.wav`                  |
| `--vocab-file`    | Vocabulary file mapping tokens to IDs                     | `/app/res/labels/vocab.json`                     |
| `--priority`      | Inference priority (0–255; higher values indicate higher priority) | `0`                                              |
| `--bpu-cores`     | Specify which BPU cores to use (e.g., `--bpu-cores 0 1`)   | `[0]`                                            |
| `--audio_maxlen`  | Fixed length after audio padding/truncation (in samples)  | `30000`                                          |
| `--new_rate`      | Target sample rate; audio will be automatically resampled | `16000`                                          |

## Quick Start
- **Run the model**
    - Using default parameters:
        ```bash
        python asr.py
        ```
    - Running with specified parameters:
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
- **View Results**

    Upon successful execution, the result will be printed:
    ```bash
    I am Qwen, a large-scale language model developed by Alibaba Cloud.
    ```

## Notes
- If the specified model path does not exist, the program will attempt to download the model automatically.