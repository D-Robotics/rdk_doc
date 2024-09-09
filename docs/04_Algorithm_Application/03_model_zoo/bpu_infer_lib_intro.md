---
sidebar_position: 2
---

# 4.3.2 ModelZoo快速上手

## 概述

ModelZoo使用的板端推理接口为**bpu_infer_lib**，bpu_infer_lib是对板端推理C++接口libdnn的Python封装。做到**安装简易、好学好用、快速上手**。

## 安装方式

bpu_infer_lib可使用

```
pip install bpu_infer_lib
```

的方式在RDK系列开发板端进行安装。

## 快速上手

本次快速上手以ModelZoo的检测大模型yoloworld为例，向开发者展示bpu_infer_lib推理库的基础用法。

:::tip Tip

Yoloworld仓库链接：https://github.com/D-Robotics/rdk_model_zoo/tree/main/demos/llm/yoloworld
:::

该仓库已为开发者提供了可在RDK系列开发板上直接部署的地瓜异构模型`yolo_world.bin`。

假设开发者已将该模型下载，此时我们可以导入bpu_infer_lib推理库，并使用该库最重要的一个类 - **Infer**类来创建一个对象：

```Python
import bpu_infer_lib
inf = bpu_infer_lib.Infer(False)
```

inf对象将负责**模型加载、读取输入、模型推理、获取输出**的全链路。

该对象的构造函数接收一个参数，参数名为debug，类型为bool。即是否开启debug模式，如开启debug模式，则会打印更多输出。

### 模型加载

模型加载我们将调用inf对象的load_model成员函数，该函数接收一个地瓜异构.bin模型的路径，并完成模型加载：

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

### 读取输入

读取输入的接口我们使用`inf.read_input`成员函数，该函数接收两个参数。

- 第一个参数为与模型输入类型匹配的numpy array
- 第二个参数为输入的index，即第几个输入（从0开始，对于单输入模型，这里填入0即可）

我们将预处理好的，yoloworld模型要求的两个输入：`input_image`和`text_embeddings`使用该接口分别读入模型：

```Python
inf.read_input(input_image, 0)
inf.read_input(text_embeddings, 1)
```

:::tip 你知道吗

inf.read_input会自动校验开发者输入numpy array的数据类型和尺寸，不合适的数据类型和尺寸都会被校验。从而避免不合理的输入被模型读取。
:::

### 模型推理

在完成模型加载、输入读取后，下一步，我们即可进行模型推理。

模型推理使用`inf.forward`函数，该函数不接收任何参数，完成模型加载、输入读取后即可执行：

```Python
inf.forward()
```

### 获取输出

当异构模型在BPU上完成推理后，我们可以使用`inf.get_output`函数获取推理结果。`inf.get_output`函数不接收任何参数，可以直接使用：

```Python
inf.get_output()
```

推理结果为numpy array，可以用以下方式，分别获得两个输出：

```Python
classes_scores = inf.outputs[0].data
bboxes = inf.outputs[1].data
```