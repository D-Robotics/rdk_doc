---
sidebar_position: 3
---

# 4.1.3 ModelZoo推理接口

## 概述

ModelZoo使用的板端推理接口为**bpu_infer_lib**。在上一小节中，我们讲述了**bpu_infer_lib**的安装方式、快速上手方法、和使用指南。

本小节，我们将具体介绍bpu_infer_lib中，各个api接口的具体细节和使用方式。

## Infer对象

Infer对象是串联整个推理流程的对象。构建一个Infer对象，它将负责**模型加载、读取输入、模型推理、获取输出**的全链路。

### Infer

<font color='Blue'>【功能描述】</font>

该函数为Infer对象的构造函数，一般用于bpu_infer_lib的导入语句之后。

<font color='Blue'>【参数描述】</font>  

| 参数名称      | 参数类型      | 定义描述                  |
| ----------- | ----------- | ------------------------ |
| debug | bool | 是否为后续推理链路提供debug打印 |

<font color='Blue'>【使用方法】</font> 

```Python
import bpu_infer_lib
inf = bpu_infer_lib.Infer(True)
```

<font color='Blue'>【返回值】</font>  

| 参数类型      | 描述                  |
| ----------- | ------------------------ |
| Infer | 返回一个Infer类的对象，并提供后续可使用的接口 |

### load_model

<font color='Blue'>【功能描述】</font>

load_model函数为Infer对象的一个成员函数，用于加载地瓜bpu异构模型。

<font color='Blue'>【参数描述】</font>  

| 参数名称      | 参数类型      | 定义描述                  |
| ----------- | ----------- | ------------------------ |
| model_path | string | 地瓜bpu异构模型.bin文件的储存路径 |

<font color='Blue'>【使用方法】</font> 

```Python
inf.load_model("yolo_world.bin")
```

完成模型加载后，我们可以查看该模型要求的输入、输出的相关属性。举个例子，我们可以通过以下方式查看以下信息：

- 模型的输入个数
- 模型每个输入要求的数据排布（layout）
- 模型每个输入要求的数据类型（type）

```Python
print("Number of model's inputs:", len(inf.inputs))
print("Input[0]'s tensor layout:", inf.inputs[0].properties.tensorLayout)
print("Input[0]'s tensor type:", inf.inputs[0].properties.tensorType)
print("Input[1]'s tensor layout:", inf.inputs[1].properties.tensorLayout)
print("Input[1]'s tensor type:", inf.inputs[1].properties.tensorType)
```

结果如下：

```
Number of model's inputs: 2
Input[0]'s tensor layout: HB_DNN_LAYOUT_NCHW
Input[0]'s tensor type: HB_DNN_TENSOR_TYPE_F32
Input[1]'s tensor layout: HB_DNN_LAYOUT_NCHW
Input[1]'s tensor type: HB_DNN_TENSOR_TYPE_F32
```

同理，我们也可以查看：

- 模型的输出个数
- 模型每个输出要求的数据排布（layout）
- 模型每个输出要求的数据类型（type）

```Python
print("Number of model's outputs:", len(inf.outputs))
print("Output[0]'s tensor layout:", inf.outputs[0].properties.tensorLayout)
print("Output[0]'s tensor type:", inf.outputs[0].properties.tensorType)
print("Output[1]'s tensor layout:", inf.outputs[1].properties.tensorLayout)
print("Output[1]'s tensor type:", inf.outputs[1].properties.tensorType)
```

结果如下：

```
Number of model's outputs: 2
Output[0]'s tensor layout: HB_DNN_LAYOUT_NCHW
Output[0]'s tensor type: HB_DNN_TENSOR_TYPE_F32
Output[1]'s tensor layout: HB_DNN_LAYOUT_NCHW
Output[1]'s tensor type: HB_DNN_TENSOR_TYPE_F32
```

<font color='Blue'>【返回值】</font>  

| 参数类型      | 描述                  |
| ----------- | ------------------------ |
| bool | 返回模型是否加载成功 |


### read_input

<font color='Blue'>【功能描述】</font>

read_input函数为Infer对象的一个成员函数，用于读取一个预处理后的numpy array输入。

<font color='Blue'>【参数描述】</font>  

| 参数名称      | 参数类型      | 定义描述                  |
| ----------- | ----------- | ------------------------ |
| input | np.array | 对应输入的numpy array |
| index | int | 对应输入numpy array的索引。在load_model后，可以打开debug查看多个inputs的顺序。对于单输入模型，输入index=0即可 |

<font color='Blue'>【使用方法】</font> 

```Python
inf.read_input(input_image, 0)
inf.read_input(text_embeddings, 1)
```

:::tip 你知道吗

inf.read_input会自动校验开发者输入numpy array的数据类型和尺寸，不合适的数据类型和尺寸都会被校验。从而避免不合理的输入被模型读取。
:::

<font color='Blue'>【返回值】</font>  

| 参数类型      | 描述                  |
| ----------- | ------------------------ |
| bool | 返回对应索引的numpy array是否正确被Infer对象所读入 |


### forward

<font color='Blue'>【功能描述】</font>

forward函数为Infer对象的一个成员函数，一般用于读取模型输入后进行地瓜bpu异构模型的推理环节。

<font color='Blue'>【参数描述】</font>  

| 参数名称      | 参数类型      | 定义描述                  |
| ----------- | ----------- | ------------------------ |
| more | bool | （可选参数，默认为False）在该次推理完成后，后续仍需使用该接口进行多次推理时，可将该参数设置为True。 |

<font color='Blue'>【使用方法】</font> 

```Python
inf.forward()
```

### get_output

<font color='Blue'>【功能描述】</font>

get_output函数为Infer对象的一个成员函数，一般用于Infer对象在完成forward函数后，获取推理结果时使用。

<font color='Blue'>【使用方法】</font> 

```Python
inf.get_output()
```

推理结果为numpy array，可以用以下方式，分别获得两个输出：

```Python
classes_scores = inf.outputs[0].data
bboxes = inf.outputs[1].data
```

<font color='Blue'>【返回值】</font>  

| 参数类型      | 描述                  |
| ----------- | ------------------------ |
| bool | 返回Infer对象是否能成功获取模型的推理结果 |


