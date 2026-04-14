---
sidebar_position: 1
---

# Image Classification

## Introduction

The image classification samples are **Python API** examples under `/app/pydev_demo/01_classification_sample/` that show how to use the `hbm_runtime` module for classification. Different models implement the same task.

Included layouts:

```
root@ubuntu:/app/pydev_demo/01_classification_sample$ tree -L 1
.
├── 01_resnet18
└── 02_mobilenetv2
```

## Demo

Both samples behave similarly; only the model differs. Below is ResNet18; MobileNetV2 looks similar.

![output-img](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/RDK_X5_Newly/pydev_01_running.png)

## Hardware setup

### Connections
Only the RDK board is required; power it and boot.

![connect-img](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/RDK_X5_Newly/pydev_01_hw_connect.png)

## Quick start

### Code location on device

```
root@ubuntu:/app/pydev_demo/01_classification_sample# tree
.
├── 01_resnet18
│   ├── imagenet1000_clsidx_to_labels.txt
│   ├── resnet18.py
│   └── zebra_cls.jpg
└── 02_mobilenetv2
    ├── imagenet1000_clsidx_to_labels.txt
    ├── mobilenetv2.py
    └── zebra_cls.jpg
```

### Build and run
No compilation; enter each folder and run:

```
cd /app/pydev_demo/01_classification_sample/01_resnet18
python resnet18.py 

cd /app/pydev_demo/01_classification_sample/02_mobilenetv2
python mobilenetv2.py
```

### Sample output

#### ResNet18

```
root@ubuntu:/app/pydev_demo/01_classification_sample/01_resnet18# python resnet18.py 
[BPU_PLAT]BPU Platform Version(1.3.6)!
[HBRT] set log level as 0. version = 3.15.55.0
[DNN] Runtime version = 1.24.5_(3.15.55 HBRT)
[A][DNN][packed_model.cpp:247][Model](2026-01-20,18:14:57.93.241) [HorizonRT] The model builder version = 1.23.5
[W][DNN]bpu_model_info.cpp:491][Version](2026-01-20,18:14:57.223.307) Model: resnet18_224x224_nv12. Inconsistency between the hbrt library version 3.15.55.0 and the model build version 3.15.47.0 detected, in order to ensure correct model results, it is recommended to use compilation tools and the BPU SDK from the same OpenExplorer package.

...

Top-5 Predictions:
zebra: 0.9853
tiger, Panthera tigris: 0.0009
prairie chicken, prairie grouse, prairie fowl: 0.0008
gazelle: 0.0006
warthog: 0.0004
root@ubuntu:/app/pydev_demo/01_classification_sample/01_resnet18# 
```

#### MobileNetV2

```
root@ubuntu:/app/pydev_demo/01_classification_sample/02_mobilenetv2# python mobilenetv2.py 
[BPU_PLAT]BPU Platform Version(1.3.6)!
[HBRT] set log level as 0. version = 3.15.55.0
[DNN] Runtime version = 1.24.5_(3.15.55 HBRT)
[A][DNN][packed_model.cpp:247][Model](2026-01-21,17:24:56.88.461) [HorizonRT] The model builder version = 1.23.5
[W][DNN]bpu_model_info.cpp:491][Version](2026-01-21,17:24:56.172.366) Model: mobilenetv2_224x224_nv12. Inconsistency between the hbrt library version 3.15.55.0 and the model build version 3.15.47.0 detected, in order to ensure correct model results, it is recommended to use compilation tools and the BPU SDK from the same OpenExplorer package.

...

Top-5 Predictions:
zebra: 0.9936
tiger, Panthera tigris: 0.0039
hartebeest: 0.0007
tiger cat: 0.0007
impala, Aepyceros melampus: 0.0003
```

## Details

### Command-line options

Both samples share the same CLI shape; default model paths differ. With no arguments, defaults load the bundled test image.

| Option | Description | Type | Default |
|--------|-------------|------|---------|
| `--model-path` | BPU `.bin` model path | str | Per-sample (see below) |
| `--test-img` | Input image path | str | `zebra_cls.jpg` |
| `--label-file` | Class label mapping (dict format) | str | `imagenet1000_clsidx_to_labels.txt` |
| `--priority` | Inference priority (0–255, 255 highest) | int | `0` |
| `--bpu-cores` | BPU core indices (e.g. `0 1`) | int list | `[0]` |

**Default model paths**

| Sample | Default model path |
|--------|-------------------|
| ResNet18 | `/opt/hobot/model/x5/basic/resnet18_224x224_nv12.bin` |
| MobileNetV2 | `/opt/hobot/model/x5/basic/mobilenetv2_224x224_nv12.bin` |

### Software architecture

<center>
![software_arch](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/RDK_X5_Newly/pydev_01_basic_software_arch1.png)
</center>

1. **Load model** — `hbm_runtime` loads the compiled model  
2. **Load image**  
3. **Preprocess** — resize to input size, BGR → NV12  
4. **Inference** — BPU forward  
5. **Post-process** — `post_utils` for probabilities  
6. **Output** — Top-5 classes and scores  

### API flow

<center>
![API_Flow](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/RDK_X5_Newly/pydev_01_basic_api_flow1.png)
</center>

1. `hbm_runtime.HB_HBMRuntime(model_path)` — load model  
2. `load_image(image_path)` — load image  
3. `resized_image(img, input_W, input_H, resize_type)` — resize  
4. `bgr_to_nv12_planes(image)` — BGR → NV12 planes  
5. `model.run(input_data)` — inference  
6. `print_topk_predictions(outputs, idx2label)` — Top-K output  

### FAQ

**Q:** `ModuleNotFoundError: No module named 'hbm_runtime'`.\
**A:** Install the RDK Python environment and `hbm_runtime`.

**Q:** Change test image?\
**A:** Use `--test-img`, e.g. `python resnet18.py --test-img your_image.jpg`.

**Q:** Model differences?\
**A:** ResNet18 is more accurate but slower; MobileNetV2 is faster with slightly lower accuracy.

**Q:** More pretrained models?\
**A:** [model_zoo](https://github.com/D-Robotics/rdk_model_zoo) or [hobot_model](https://github.com/D-Robotics/hobot_model).

