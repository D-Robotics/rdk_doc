---
sidebar_position: 6
---

# 4.1.6 模型推理接口说明

## 概要介绍

开发板Ubuntu系统预装了Python版本的`pyeasy_dnn`模型推理模块，通过加载模型并创建`Model`对象，完成模型推理、数据解析等功能。

模块推理过程可分为加载模型、图像推理、数据解析三个步骤，代码示例如下：

```python
from hobot_dnn import pyeasy_dnn as dnn

#create model object
models = model.load('./model.bin')

#do inference with image
outputs = models[0].forward(image)

for item in outputs:
    output_array.append(item.buffer)
post_process(output_array)
```

## Model对象{#model}

Model对象在加载模型时创建，包含了`inputs`、`outputs`、`forward`等成员和方法，详细说明如下：

### inputs

<font color='Blue'>【功能描述】</font>

返回模型的tensor输入信息，通过索引指定具体输入，例如：inputs[0]表示第0组输入。

<font color='Blue'>【函数声明】</font>  

```python
Model.inputs(tuple(pyDNNTensor))
```

<font color='Blue'>【参数描述】</font>  

| 参数名称      | 定义描述                  |
| ----------- | ------------------------ |
| index | 表示输入tensor的索引 |

<font color='Blue'>【使用方法】</font> 

```python
def print_properties(pro):
    print("tensor type:", pro.tensor_type)
    print("data type:", pro.dtype)
    print("layout:", pro.layout)
    print("shape:", pro.shape)

models = dnn.load('../models/fcos_512x512_nv12.bin')
input = models[0].inputs[0]

print_properties(input.properties)
```

<font color='Blue'>【返回值】</font>  

返回`pyDNNTensor`类型的对象，说明如下：

| 参数名称 | 描述 |
| ------ | ----- |
| properties  | 表示tensor的属性  |
| buffer    | 表示tensor中的数据, numpy格式 |
| name    | 表示tensor中的名称 |

<font color='Blue'>【注意事项】</font>  

无

### outputs

<font color='Blue'>【功能描述】</font>  

返回模型的tensor输出信息，通过索引指定具体输出，例如：outputs[0]表示第0组输出。

<font color='Blue'>【函数声明】</font>  

```python
Model.outputs(tuple(pyDNNTensor))
```

<font color='Blue'>【参数描述】</font>  

| 参数名称      | 定义描述                  |
| ----------- | ------------------------ |
| index | 表示输出tensor的索引 |

<font color='Blue'>【使用方法】</font>  

```python
def print_properties(pro):
    print("tensor type:", pro.tensor_type)
    print("data type:", pro.dtype)
    print("layout:", pro.layout)
    print("shape:", pro.shape)

models = dnn.load('../models/fcos_512x512_nv12.bin')
output = models[0].outputs[0]

print_properties(output.properties)
```

<font color='Blue'>【返回值】</font>  

返回`pyDNNTensor`类型的对象，说明如下：

| 参数名称 | 描述 |
| ------ | ----- |
| properties  | 表示tensor的属性  |
| buffer    | 表示tensor中的数据, numpy格式 |
| name    | 表示tensor中的名称 |

<font color='Blue'>【注意事项】</font>  

无


### forward

<font color='Blue'>【功能描述】</font>  

根据指定输入进行模型推理。

<font color='Blue'>【函数声明】</font>  

```python
Model.forward(args &args, kwargs &kwargs)
```

<font color='Blue'>【参数描述】</font>  

| 参数名称      | 定义描述                  | 取值范围 |
| ----------- | ------------------------ | ------- |
| args | 需要推理输入数据 | numpy：单模型输入，list[numpy, numpy, ...]: 多模型输入 |
| kwargs | core_id，表示模型推理的core id | 0：自动分配，1：core0,2：core1 |
| kwargs | priority，表示当前模型推理任务的优先级 | 取值范围0~255，数值越大，优先级越高 |

<font color='Blue'>【使用方法】</font>  

```python
img = cam.get_img(2, 512, 512)

img = np.frombuffer(img, dtype=np.uint8)
outputs = models[0].forward(img)
```

<font color='Blue'>【返回值】</font>  

返回`outputs`对象。


<font color='Blue'>【注意事项】</font>  

无

## 示例代码
可以查看 [模型推理示例](./pydev_dnn_demo) 章节详细了解。
