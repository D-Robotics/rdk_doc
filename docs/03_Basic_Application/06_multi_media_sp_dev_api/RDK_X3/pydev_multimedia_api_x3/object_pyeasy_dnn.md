---
sidebar_position: 1
---

# BPU 算法推理


## 模块描述

pyeasy_dnn 是底层 DNN 推理能力的 C++ 封装实现，对外暴露轻量的 Python 接口，核心包含模块级模型加载方法、Model 对象推理方法，以及TensorProperties 张量属性获取三类能力。该接口适配 RDK X3 以及 RDK X5 （软件版本3.5.0 之前）系列硬件平台的神经网络推理场景。

## 基础规格
- 硬件兼容：RDK 系列开发板（支持 NPU 硬件加速）
- 模型格式支持：适配 RDK 平台的 bin 等模型格式（具体依赖底层推理库）
- 数据交互：推理输入 / 输出以张量（Tensor）形式交互，支持通过属性获取张量形状、数据类型等信息
- 接口依赖：依赖 RDK 平台 libdnn.so 底层推理库，Python 层仅封装核心推理流程
- 核心能力：模型加载、推理执行、张量属性查询

## 参考示例
可以参考 01_basic_sample/test_efficientnasnet_m.py 示例介绍


## API 参考

### 一、模块级方法
| API 接口 | 接口功能 |
| ---- | ----- |
| load | **加载模型文件，返回 Model 推理对象** |

### 二、Model 对象方法
| API 接口 | 接口功能 |
| ---- | ----- |
| forward | **执行模型推理，输入张量数据，返回输出张量对象** |

### 三、TensorProperties 张量属性（只读）
| API 接口 | 接口功能 |
| ---- | ----- |
| tensor_type | **获取张量类型（如输入 / 输出张量、中间张量等）** |
| dtype | **获取张量的数据类型（如 float32/uint8/int32 等）** |
| layout | **获取张量的维度布局（如 NCHW/NHWC/CHW 等）** |
| shape | **获取张量的逻辑形状（如 (1, 3, 640, 640)）** |
| alignedShape | **获取张量的对齐形状（硬件层对齐后的实际形状，适配 RDK 硬件内存对齐要求）** |
| validShape | **获取张量的有效形状（实际参与计算的有效维度，区别于对齐形状）** |
| scale_data | **获取张量的量化缩放系数（量化模型专用，浮点模型通常为 1.0）** |



### 一、模块级方法

#### load

<font color='Blue'>【函数声明】</font>  

```python
model = dnn.load(model_path, **kwargs)
```

<font color='Blue'>【功能描述】</font>  

加载指定路径的深度学习模型文件，完成模型解析、硬件适配和内存分配，返回可执行推理的 Model 对象。



<font color='Blue'>【参数描述】</font>  

| 参数名称      | 定义描述                  | 取值范围    |
| ----------- | ------------------------ | --------  |
| model_path     | 模型文件的绝对 / 相对路径  | 有效文件路径，支持 RKNN/ONNX 等（依底层库而定）  |
| **kwargs | 可选参数（依底层实现扩展） | 如 core_num（推理核心数）、precision（推理精度）等，无则传空 |


<font color='Blue'>【返回值】</font>  

| 返回值 | 描述 |
| ------ | ----- |
| model  | 初始化完成的 Model 对象，包含推理能力和张量属性  |


<font color='Blue'>【使用方法】</font> 

```python
from hobot_dnn import pyeasy_dnn as dnn

models = dnn.load('../models/efficientnasnet_m_300x300_nv12.bin')
```


<font color='Blue'>【注意事项】</font> 

- 模型路径需指向实际存在的模型文件，且模型需与当前 RDK 硬件平台兼容；
- 加载失败会抛出 RuntimeError 异常，需捕获处理；
- 可选参数（kwargs）需匹配底层 Dnnpy_load 函数的关键字参数定义，无特殊需求可省略。

### 二、Model 对象方法
#### forward

<font color='Blue'>【功能描述】</font>

基于已加载的模型，接收输入张量数据，调用底层推理接口完成前向计算，返回包含输出张量数据和属性的 TensorProperties 对象列表。
<font color='Blue'>【函数声明】</font>  

```python
output_tensors = model.forward(inputs, **kwargs)
```

<font color='Blue'>【参数描述】</font>  


| 参数名称      | 定义描述                  | 取值范围    |
| ----------- | ------------------------ | --------  |
| inputs    | 模型推理输入数据列表  | 列表中每个元素为 numpy 数组 / 字节流，形状、数据类型需匹配模型输入张量定义  |
| **kwargs  | 推理可选参数（依底层实现扩展）   | 如 batch_size（批次大小）等，无则传空 |


<font color='Blue'>【返回值】</font>  

| 返回值 | 定义描述 |                 
| ------ | ----- |
| output_tensors  | TensorProperties 对象列表，每个对象包含输出张量的数值和属性（shape/dtype 等）  |


:::info 注意！
输入数据的形状、数据类型（dtype）必须与模型输入张量的 shape/dtype 属性严格匹配；
多输入模型需按输入张量顺序传入 inputs 列表；
推理耗时与模型复杂度、硬件核心数相关，建议异步调用避免阻塞主线程。
:::


### 三、TensorProperties 张量属性（只读）
TensorProperties 是封装张量元信息的对象，通过 getter 函数暴露以下只读属性，无独立方法，直接通过属性名访问


| 属性名      | 功能描述                  | 数据类型    |
| ----------- | ------------------------ | --------  |
| tensor_type | 获取张量类型（如输入 / 输出张量、中间张量等） | 字符串 / 整型 |
| dtype | 获取张量的数据类型（如 float32/uint8/int32 等） | 字符串 / 枚举值 |
| layout | 获取张量的维度布局（如 NCHW/NHWC/CHW 等） | 字符串 |
| shape | 获取张量的逻辑形状（如 (1, 3, 640, 640)  ） | 元组（整型） |
| alignedShape | 	获取张量的对齐形状（硬件层对齐后的实际形状，适配 RDK 硬件内存对齐要求） | 元组（整型） |
| validShape | 获取张量的有效形状（实际参与计算的有效维度，区别于对齐形状） | 元组（整型） |
| scale_data | 获取张量的量化缩放系数（量化模型专用，浮点模型通常为 1.0） | 浮点型 / 数组 |


#### 属性访问示例
```
# 假设 model 是已加载的 Model 对象，inputs 为输入张量属性列表
input_tensor = model.inputs[0]

# 访问张量属性
print(f"张量类型：{input_tensor.tensor_type}")
print(f"数据类型：{input_tensor.dtype}")
print(f"维度布局：{input_tensor.layout}")
print(f"逻辑形状：{input_tensor.shape}")
print(f"对齐形状：{input_tensor.alignedShape}")
print(f"有效形状：{input_tensor.validShape}")
print(f"量化缩放系数：{input_tensor.scale_data}")

# 输出张量属性访问同理
output_tensor = model.forward(inputs=[input_data])[0]
print(f"输出张量数据类型：{output_tensor.dtype}")
```

<font color='Blue'>【注意事项】</font> 

- 所有属性均为只读，无法直接修改，需通过输入数据预处理匹配张量属性；
- alignedShape 是 RDK 硬件层的内存对齐形状（如 X3 芯片要求宽度 32 对齐），实际数据存储尺寸以该属性为准；
- validShape 表示张量中有效数据的维度范围，部分场景下与 shape 一致；
- scale_data 仅在量化模型（如 int8 量化 RKNN）中有实际意义，浮点模型该值通常为 1.0。

## 补充说明
1、 资源释放：dnn_python.cpp 未显式定义 release 方法，模型资源通常在 Model 对象被 Python 垃圾回收时自动释放，若需手动释放可参考底层库接口扩展；
2、 异常处理：模型加载、推理执行失败时会抛出 Python 异常（如 RuntimeError），建议通过 try-except 捕获；
3、 硬件适配：张量的 alignedShape 需适配 RDK 芯片的内存对齐规则（如 X3 宽度 32 对齐），否则可能导致推理异常或数据错误；
4、 多输入 / 输出：forward 方法的 inputs 参数为列表，对应多输入模型的多个输入张量；返回的 output_tensors 列表对应多输出模型的多个输出张量，顺序与模型定义一致。