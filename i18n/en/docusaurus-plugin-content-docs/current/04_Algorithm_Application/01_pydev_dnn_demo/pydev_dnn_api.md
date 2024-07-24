---
sidebar_position: 6
---

# 4.1.6 Model Inference Interface Description

## Overview

The development board Ubuntu system comes pre-installed with the Python version of the pyeasy_dnn model inference module. By loading the model and creating a Model object, functions such as model inference and data parsing can be completed.

The module inference process can be divided into three steps: loading the model, image inference, and data parsing. The code example is as follows:

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

## Model Object {#model}
The Model object is created when the model is loaded. It contains members and methods such as inputs, outputs, and forward, detailed as follows:

### inputs

<font color='Blue'>【Function Description】</font>

Returns the tensor input information of the model. Specific input can be specified by index, for example: inputs[0] represents the 0th input.

<font color='Blue'>【Function Declaration】</font>

```python
Model.inputs(tuple(pyDNNTensor))
```

<font color='Blue'>【Parameter Description】</font>


| Parameter Name	      | Definition Description                  |
| ----------- | ------------------------ |
| index | Represents the index of the input tensor |

<font color='Blue'>【Usage Method】</font>


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

<font color='Blue'>【Return Value】</font>

Returns an object of type pyDNNTensor, detailed as follows:

| Parameter Name	 | Description |
| ------ | ----- |
| properties  | Represents the properties of the tensor  |
| buffer    | Represents the data in the tensor, in numpy format |
| name    | Represents the name in the tensor |

<font color='Blue'>【Notes】</font>

None

outputs
<font color='Blue'>【Function Description】</font>

Returns the tensor output information of the model. Specific output can be specified by index, for example: outputs[0] represents the 0th output.

<font color='Blue'>【Function Declaration】</font> 

```python
Model.outputs(tuple(pyDNNTensor))
```

<font color='Blue'>【Parameter Description】</font>


|Parameter Name	| Definition Description                 |
| ----------- | ------------------------ |
| index	| Represents the index of the output tensor |

<font color='Blue'>【Usage Method】</font>


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

<font color='Blue'>【Return Value】</font>

Returns an object of type pyDNNTensor, detailed as follows:

| Parameter Name	| Description |
| ------ | ----- |
| properties  | Represents the properties of the tensor  |
| buffer    | Represents the data in the tensor, in numpy format |
| name    | Represents the name in the tensor |

<font color='Blue'>【Notes】</font>

None

forward
<font color='Blue'>【Function Description】</font>

Performs model inference based on the specified input.

<font color='Blue'>【Function Declaration】</font>

```python
Model.forward(args &args, kwargs &kwargs)
```

<font color='Blue'>【Parameter Description】</font>


| Parameter Name	| Definition Description	| Value Range |
| ----------- | ------------------------ | ------- |
| args | Input data for inference	| numpy: single model input, list[numpy, numpy, ...]: multiple model inputs |
| kwargs | core_id, represents the core id for model inference	| 0: automatic allocation, 1: core0, 2: core1 |
| kwargs | priority, represents the priority of the current model inference task	| Value range 0~255, the larger the number, the higher the priority |



args	
kwargs	
kwargs	

<font color='Blue'>【Usage Method】</font>  

```python
img = cam.get_img(2, 512, 512)

img = np.frombuffer(img, dtype=np.uint8)
outputs = models[0].forward(img)
```
<font color='Blue'>【Return Value】</font>

Returns an outputs object.

<font color='Blue'>【Notes】</font>

None

## Example Code

You can view the [Model Inference Example ](./pydev_dnn_demo)  section for more details.
