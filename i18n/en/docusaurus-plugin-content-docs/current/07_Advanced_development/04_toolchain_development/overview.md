---
sidebar_position: 1
---

# 7.4.1 Introduction

The D-Robotics Algorithm Toolchain is an algorithm solution developed based on the D-Robotics processor. It can help you quantize floating-point models into fixed-point models and quickly deploy self-developed algorithm models on the D-Robotics processor.

Currently, most models trained on GPUs are floating-point models, which means that the parameters are stored using the float data type. The D-Robotics BPU architecture processor uses INT8 as the computing precision (the general precision of processors in the industry) and can only run quantized fixed-point models. The process of converting a trained floating-point model to a fixed-point model is called quantization. Depending on whether the quantized parameters need to be adjusted, we can divide the quantization methods into Quantization Aware Training (QAT) and Post-Training Quantization (PTQ).

The D-Robotics Algorithm Toolchain mainly uses the Post-Training Quantization (PTQ) method, which only requires using a batch of calibration data to calibrate the trained floating-point model, converting the trained FP32 network directly into a network that performs fixed-point calculations. During this process, there is no need to train the original floating-point model, only a few hyperparameters need to be adjusted to complete the quantization process. The whole process is simple and fast and has been widely used in edge computing and cloud scenarios.

## Instructions for Use

This section is applicable to developers using the D-Robotics processor and is used to introduce some instructions for using the D-Robotics Algorithm Toolchain.

### Instructions for Floating-Point Models (FP32)

-   Support quantizing caffe floating-point models of caffe version 1.0 and onnx floating-point models with ir_version≤7, opset10, and opset11 into fixed-point models supported by Horizon.

-   Floating-point models trained with other frameworks need to be exported as onnx floating-point models that meet the requirements of the first point before quantization can be performed.

-   The model input dimensions only support fixed 4 dimensions in the format of NCHW or NHWC (the N dimension can only be 1), for example, 1x3x224x224 or 1x224x224x3. Dynamic dimensions and non-4-dimensional inputs are not supported.

-   Do not include post-processing operators, such as NMS calculations, in the floating-point models.

### Explanation of Model Operators List

-   Currently, all Caffe and ONNX operators supported by the D-Robotics processor are provided. Other operators that are not listed are not supported due to hardware limitations of the D-Robotics processor. For the specific supported operator list, please refer to the chapter [**Supported Operator List**](/toolchain_development/intermediate/supported_op_list).