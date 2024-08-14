---
sidebar_position: 2
---

# 7.4.2 Getting Started Guide

## Overview


This guide is intended to introduce the usage process of the PTQ method after training in the D-Robotics algorithm toolchain. If you are a first-time user of the D-Robotics algorithm toolchain, it is recommended that you follow the steps in this guide to learn. If you have completed the learning of the content in the Getting Started Guide section, you can refer to the steps in the [**Quick Experiments**](#quick_experiments) section of this guide to convert and run private models on the board.
If you need to learn more about the D-Robotics algorithm toolchain, please refer to the [**Intermediate Guide**](./intermediate) section.


## Environment Installation {#env_install}


This section mainly introduces the necessary environmental preparation work before using the D-Robotics algorithm toolchain.


**Hardware Environment**

In order to use the D-Robotics algorithm toolchain smoothly, D-Robotics recommends that the development machine you choose should meet the following requirements:



  | Hardware/Operating System | Requirements                                 |
  |---------------------------|----------------------------------------------|
  | CPU                       | CPU I3 or above, or equivalent E3/E5 processor|
  | Memory                    | 16GB or above                                 |
  | GPU (optional)            | CUDA 11.6, driver version Linux:>= 510.39.01*<br/>Compatible graphics cards include but are not limited to:<br/>1)GeForce RTX 3090<br/>2)GeForce RTX 2080 Ti<br/>3)NVIDIA TITAN V<br/>4)Tesla V100S-PCIE-32GB              
  | System                    | Ubuntu 20.04                                 |


**Deployment on Development Machine**

:::caution Note

  Before converting the model, make sure that the Ubuntu or Centos system in the development machine has installed the ``Anaconda3`` environment that supports Python 3.8.x version.
:::

- 1.Execute the following command on the development machine to obtain the model conversion data package:
```bash
    wget -c ftp://xj3ftp@vrftp.horizon.ai/ai_toolchain/ai_toolchain.tar.gz --ftp-password=xj3ftp@123$%

    wget -c ftp://xj3ftp@vrftp.horizon.ai/model_convert_sample/yolov5s_v2.0.tar.gz --ftp-password=xj3ftp@123$%
```

:::tip Tip
  1. If you need more examples of public model conversion, you can execute the command: ``wget -c ftp://xj3ftp@vrftp.horizon.ai/model_convert_sample/horizon_model_convert_sample.tar.gz --ftp-password=xj3ftp@123$%`` to obtain them.
  2. D-Robotics also provides a Docker image that supports model conversion. If you need to use the Docker environment, please read the [**Intermediate Guide-Using Docker Environment**](./intermediate/environment_config#using-docker-environment) section.- 
:::

  2. Create the model conversion environment:
```bash
    // horizon_bpu is the name of the environment, you can set it yourself

    conda create -n horizon_bpu python=3.8 -y   
```

- 3. Enter the model conversion environment:
```bash
    // horizon_bpu is the name of the python environment created in the previous step. The conda environment command may vary depending on the operating system. Please choose one of the following two commands that can enter the conda model conversion environment.

    source activate horizon_bpu or conda activate horizon_bpu  
```

- 4. Unzip the environment for model conversion and the installation package of the sample model, and install the relevant dependencies:
```bash
    tar -xvf yolov5s_v2.0.tar.gz

    tar -xvf ai_toolchain.tar.gz

    pip install ai_toolchain/h* -i https://mirrors.aliyun.com/pypi/simple

    pip install pycocotools -i https://mirrors.aliyun.com/pypi/simple
```


After successful installation, you can type the command hb_mapper --help to verify if you can get the help information properly. If the following information is printed, it means that the environment has been installed successfully:
```bash
    hb_mapper --help
    Usage: hb_mapper [OPTIONS] COMMAND [ARGS]...

      hb_mapper is an offline model transform tool provided by horizon.

    Options:
      --version   Show the version and exit.
      -h, --help  Show this message and exit.

    Commands:
      checker    check whether the model meet the requirements.
      infer      inference and dump output feature as float vector.
      makertbin  transform caffe model to quantization model, generate runtime...
```

:::tip Tips

  When performing model conversion later, please first use the command "source activate horizon_bpu" or "conda activate horizon_bpu" to enter the model conversion environment!

  The overall size of the D-Robotics AI Toolchain installation package is about 200M. The download of the installation package and the installation of dependencies are affected by network speed. The entire installation process takes about 20 minutes. Please be patient until the installation is complete.
:::
  
## Quick Experience {#quick_experiments}

In this chapter, we introduce the basic usage process of the D-Robotics algorithm toolchain PTQ solution, so that you can quickly get started. Here we take yolov5s model running on the RDK X3 development board as an example to demonstrate the usage for you. For more detailed content of the D-Robotics algorithm toolchain PTQ solution, please read the [**Advanced Guide - PTQ Principles and Steps**](./intermediate/ptq_process) chapter.

:::tip Tips
  To convert models supported by RDK Ultra, replace the ``0x_xx_X3.sh`` script command in the steps of the following chapters with the ``0x_xx_Ultra.sh`` script command for model conversion.
:::

### Development Environment Preparation

If the development environment is not prepared, please refer to the [**Environment Installation**](#env_install) chapter for environment installation.

### Model Preparation

If the development environment is ready, please use the command: ``source activate horizon_bpu`` or ``conda activate horizon_bpu`` to enter the model conversion environment on the development machine.

Run the following command to check if the yolov5s floating-point model exists:

```bash
    ls -l yolov5s_v2.0/04_detection/03_yolov5s/mapper
```

After the command is executed, if the following log appears, it means that the model is ready:

```bash
    -rwxr-xr-x 1 10488 10501      640 Jul 31 18:35 01_check_Ultra.sh
    -rwxr-xr-x 1 10488 10501      645 Jul 31 18:35 01_check_X3.sh
    -rwxr-xr-x 1 10488 10501      661 Jul 31 18:24 02_preprocess.sh
    -rwxr-xr-x 1 10488 10501      609 Jul 31 18:34 03_build_Ultra.sh
    -rwxr-xr-x 1 10488 10501      606 Aug 14 16:49 03_build_X3.sh
    -rwxr-xr-x 1 10488 10501     2752 Mar  9 11:34 README.cn.md
    -rwxr-xr-x 1 10488 10501     1422 Jul 31 18:24 README.md
    -rwxr-xr-x 1 10488 10501 29999538 Mar  9 14:01 YOLOv5s.onnx
    -rwxr-xr-x 1 10488 10501    13039 Jul 31 18:24 postprocess.py
    -rwxr-xr-x 1 10488 10501     3133 Jul 31 18:24 preprocess.py
    -rwxr-xr-x 1 10488 10501    11304 Jul 31 18:34 yolov5s_config_Ultra.yaml
    -rwxr-xr-x 1 10488 10501    11275 Jul 31 18:25 yolov5s_config_X3.yaml

```

If the above log does not appear after executing the command, please read the [**Environment Installation**](#env_install) chapter to download the model example package.

### Model Verification

If the floating-point model sample is ready, follow the steps below to verify the model to ensure that it complies with the support constraints of the D-Robotics RDK X3 processor.

-   Enter the yolov5s model directory of the floating-point model conversion example

```bash
cd yolov5s_v2.0/04_detection/03_yolov5s/mapper
```

- Model Validation

```bash
    # Check the model structure and operator support, and provide hardware allocation information for each operator (BPU/CPU). RDK X3 execution script: 01_check_X3.sh; RDK Ultra execution script: 01_check_Ultra.sh
    bash 01_check_X3.sh
```

After the command execution is completed, if the following log appears, it indicates that the model validation is successful

```bash
    2022-12-21 22:29:51,153 INFO [Wed Dec 21 22:29:51 2022] End to D-Robotics NN Model Convert.
    2022-12-21 22:29:51,181 INFO ONNX model output num : 3
    2022-12-21 22:29:51,219 INFO End model checking....
```

### Model Conversion

After the model validation is successful, perform model conversion according to the following steps.

- Preprocess calibration data

```bash
    bash 02_preprocess.sh
```

After the command execution is completed, if the following log appears and there are no errors, it indicates that the data preprocessing is successful

```bash
    write:./calibration_data_rgb_f32/COCO_val2014_000000181677.rgb
    write:./calibration_data_rgb_f32/COCO_val2014_000000181714.rgb
    write:./calibration_data_rgb_f32/COCO_val2014_000000181739.rgb
```

- Model conversion

```bash
    # The configuration file yolov5s_config_X3.yaml required for conversion is already placed in the same folder as the script 03_build_X3.sh. RDK X3 execution script: 03_build_X3.sh; RDK Ultra execution script: 03_build_Ultra.sh
    bash 03_build_X3.sh
```

After the command execution is completed, if the following log appears and there are no errors, it indicates that the model conversion is successful

```bash
    2022-12-21 22:36:48,087 INFO Convert to runtime bin file sucessfully!
    2022-12-21 22:36:48,087 INFO End Model Convert
```

After the model conversion is completed, the model files and static performance evaluation files will be saved in the `model_output` folder.

-   torch-jit-export_subgraph_0.html        # Static performance evaluation file (better readability)
-   torch-jit-export_subgraph_0.json        # Static performance evaluation file
-   hb_model_modifier.log                   # Log information generated during the model conversion steps
-   cache.json                              # Cache file (automatically generated when RDK Ultra optimization level is configured as O3)
-   yolov5s_672x672_nv12.bin     # Model file for loading and running on the D-Robotics processor
-   yolov5s_672x672_nv12_calibrated_model.onnx      # Intermediate model file for subsequent model accuracy verification
-   yolov5s_672x672_nv12_optimized_float_model.onnx # Intermediate model file for subsequent model accuracy verification
-   yolov5s_672x672_nv12_original_float_model.onnx # Intermediate model file for subsequent model accuracy verification
-   yolov5s_672x672_nv12_quantized_model.onnx # Intermediate model file for subsequent model accuracy verification


### Model Deployment

**Note**: Before deploying the model, make sure to complete the environment setup on the development board according to the steps in the [**System Installation**](..../../../01_Quick_start/install_os.md) section. 
Copy the fixed-point model `yolov5s_672x672_nv12.bin` to the `/app/pydev_demo/models` directory on the development board, and run the following command:

```bash
    cd /app/pydev_demo/07_yolov5_sample/
    sudo python3 ./test_yolov5.py
```

After successful execution, the segmentation results of the image will be outputted, and the segmentation effect image "result.jpg" will be dumped:

```bash
    ......
    detected item num:  15
    person is in the picture with confidence:0.8555
    person is in the picture with confidence:0.7774
    person is in the picture with confidence:0.6599
    person is in the picture with confidence:0.6310
    person is in the picture with confidence:0.6091
    person is in the picture with confidence:0.5242
    person is in the picture with confidence:0.5182
    person is in the picture with confidence:0.4737
    person is in the picture with confidence:0.4037
    person is in the picture with confidence:0.4023
    kite is in the picture with confidence:0.8651
    kite is in the picture with confidence:0.8428
    kite is in the picture with confidence:0.7063
    kite is in the picture with confidence:0.6806
    kite is in the picture with confidence:0.5446
    ......
```

![yolov5s-result](../../../../../../static/img/07_Advanced_development/04_toolchain_development/image/beginner/yolov5s-result.png)

For examples of common APIs, please refer to the [**yolov5 Object Detection Algorithm**](../../04_Algorithm_Application/01_pydev_dnn_demo/static_image.md) section.For more API instructions on model inference, please refer to the chapter [**Python Development Guide - Model Inference API Usage Instructions**](../../04_Algorithm_Application/01_pydev_dnn_demo/pydev_dnn_api.md) and [**C/C++ Development Guide - Model Inference API Usage Instructions**](../../04_Algorithm_Application/02_cdev_dnn_api/cdev_dnn_api.md).

### Generic Model Performance Accuracy Metrics

The table below provides performance and accuracy metrics of typical deep neural network models on the "RDK X3" development board by Horizon.

![model_accuracy](../../../../../../static/img/07_Advanced_development/04_toolchain_development/image/beginner/model_accuracy.png)

**Note:**

1. The data in the table are measured results on the D-Robotics RDK X3 development board, and the test models are from the [horizon_model_convert_sample](./beginner) model example package.

2. For the BPU/CPU hybrid heterogeneous models in the model example package, the time consumption of a single frame mainly consists of the following modules: input quantization CPU node, model BPU operator, model CPU operator, output dequantization CPU node, and CPU post-processing. The details are explained as follows:

   a. Input quantization CPU node: It performs the input quantization operation from float32 to int8, and it only exists in models with featuremap input. The quantization time consumption is proportional to the input shape.

   b. Model CPU operator:

      ⅰ. Detection models do not include CPU operators.
      
      ⅱ. Softmax and Reshape at the end of classification models are CPU operators.
      
      ⅲ. Argmax at the end of segmentation model DeepLabV3+ is a CPU operator.

   c. Output dequantization CPU node: It performs the output dequantization operation from int8 to float32. The quantization time consumption is proportional to the output shape.

   d. Currently, D-Robotics supports manually removing the quantization/dequantization nodes of the model and integrating them into the pre- and post-processing code by users to reduce the overhead of redundant data traversal. Taking the EfficientDet model as an example, by removing the dequantization node and integrating it into the post-processing, the inference performance has increased from 66 FPS to 100 FPS.

   e. Currently, the post-processing of D-Robotics example models has not been specifically optimized for performance. You can use optimization methods such as approximate efficient implementation according to your actual requirements for code-level acceleration.

3. In practical applications, BPU and CPU can run concurrently to improve overall inference speed.