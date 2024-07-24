---
sidebar_position: 1
---

# 7.4.1 简介

地平线算法工具链是基于地平线处理器研发的算法解决方案，可以帮助您把浮点模型量化为定点模型， 并在地平线处理器上快速部署自研算法模型。

目前在GPU上训练的模型大部分都是浮点模型，即参数使用的是float类型存储；地平线BPU架构的处理器使用的是  INT8   的计算精度（业内处理器的通用精度），只能运行定点量化模型。从训练出的浮点模型转为定点模型的过程，我们叫做量化，依据是否要对量化后的参数进行调整，我们可以将量化方法分为QAT（Quantification Aware Training）量化感知训练和PTQ（Post-Training Quantization）训练后量化。

地平线算法工具链主要使用的是<font color='Red'>训练后量化PTQ</font>方法，只需使用一批校准数据对训练好的浮点模型进行校准, 将训练过的FP32网络直接转换为定点计算的网络，此过程中无需对原始浮点模型进行任何训练，只对几个超参数调整就可完成量化过程, 整个过程简单快速, 目前在端侧和云侧场景已得到广泛应用。 


## 使用注意事项

本章节适用于使用地平线处理器的开发者，用于介绍地平线算法工具链的一些使用注意事项。

### 浮点模型(FP32)注意事项

-   支持<font color='Red'>caffe 1.0</font> 版本的caffe浮点模型和<font color='Red'>ir_version≤7</font> 、<font color='Red'>opset10</font> 、<font color='Red'>opset11</font> 版本的onnx浮点模型量化成地平线支持的定点模型；

-   其他框架训练的浮点模型需要先导出第1点要求符合版本的onnx浮点模型后，才能进行量化；

-   模型输入维度只支持<font color='Red'>固定4维</font> 输入NCHW或NHWC（N维度只能为1），例如：1x3x224x224或1x224x224x3， 不支持动态维度及非4维输入；

-   浮点模型中不要包含有<font color='Red'>后处理算子</font> ,例如：nms计算。

### 模型算子列表说明

-   目前提供了地平线处理器可支持的所有Caffe和ONNX算子情况，其他未列出的算子因<font color='Red'>地平线处理器 bpu硬件限制</font> ，<font color='Red'>暂不支持</font> 。具体算子支持列表，请参考 [**模型算子支持列表**](/toolchain_development/intermediate/supported_op_list) 章节内容。