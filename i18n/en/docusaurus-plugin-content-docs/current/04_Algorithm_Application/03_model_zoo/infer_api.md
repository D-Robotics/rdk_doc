---
sidebar_position: 3
---

# 4.3.3 ModelZoo Inference Interface

## Overview

The board-side inference interface used by ModelZoo is **bpu_infer_lib**. In the previous section, we discussed the installation method, quick start method, and usage guide of bpu_infer_lib.

In this section, we will specifically introduce the details and usage methods of each API interface in bpu_infer_lib.

## Infer Object

The Infer object is the one that connects the entire inference process. Constructing an Infer object, it will be responsible for the whole process of **model loading, input reading, model inference, and output acquisition**.

### Infer

<font color='Blue'>【Function Description】</font>

This function is the constructor of the Infer object, which is generally used after the import statement of bpu_infer_lib.

<font color='Blue'>【Parameter Description】</font>  

| Parameter Name | Parameter Type | Definition Description                                       |
| ----------- | ----------- | ------------------------ |
| debug          | bool           | Whether to provide debug printing for subsequent inference chains |

<font color='Blue'>【Usage】</font> 

```Python
import bpu_infer_lib
inf = bpu_infer_lib.Infer(True)
```

<font color='Blue'>【Return Value】</font>  

| Parameter Type | Description                                                  |
| ----------- | ------------------------ |
| Infer          | Returns an Infer class object, providing subsequent usable interfaces |

### load_model

<font color='Blue'>【Function Description】</font>

The load_model function is a member function of the Infer object, used to load the DiGua BPU heterogeneous model.

<font color='Blue'>【Parameter Description】</font>  

| Parameter Name | Parameter Type | Definition Description                                       |
| ----------- | ----------- | ------------------------ |
| model_path     | string         | The storage path of the DiGua BPU heterogeneous model .bin file |

<font color='Blue'>【Usage】</font> 

```Python
inf.load_model("yolo_world.bin")
```

- After the model is loaded, we can view the attributes required for the model's inputs and outputs. For example, we can view the following information:
  - The number of model inputs
  - The data layout (layout) required for each model input
  - The data type (type) required for each model input

```Python
print("Number of model's inputs:", len(inf.inputs))
print("Input[0]'s tensor layout:", inf.inputs[0].properties.tensorLayout)
print("Input[0]'s tensor type:", inf.inputs[0].properties.tensorType)
print("Input[1]'s tensor layout:", inf.inputs[1].properties.tensorLayout)
print("Input[1]'s tensor type:", inf.inputs[1].properties.tensorType)
```

The results are as follows:

```
Number of model's inputs: 2
Input[0]'s tensor layout: HB_DNN_LAYOUT_NCHW
Input[0]'s tensor type: HB_DNN_TENSOR_TYPE_F32
Input[1]'s tensor layout: HB_DNN_LAYOUT_NCHW
Input[1]'s tensor type: HB_DNN_TENSOR_TYPE_F32
```

- Similarly, we can also view:
  - The number of model outputs
  - The data layout (layout) required for each model output
  - The data type (type) required for each model output

```Python
print("Number of model's outputs:", len(inf.outputs))
print("Output[0]'s tensor layout:", inf.outputs[0].properties.tensorLayout)
print("Output[0]'s tensor type:", inf.outputs[0].properties.tensorType)
print("Output[1]'s tensor layout:", inf.outputs[1].properties.tensorLayout)
print("Output[1]'s tensor type:", inf.outputs[1].properties.tensorType)
```

The results are as follows:

```
Number of model's outputs: 2
Output[0]'s tensor layout: HB_DNN_LAYOUT_NCHW
Output[0]'s tensor type: HB_DNN_TENSOR_TYPE_F32
Output[1]'s tensor layout: HB_DNN_LAYOUT_NCHW
Output[1]'s tensor type: HB_DNN_TENSOR_TYPE_F32
```

<font color='Blue'>【Return Value】</font>  

| Parameter Type | Description                                      |
| ----------- | ------------------------ |
| bool           | Returns whether the model is loaded successfully |


### read_input

<font color='Blue'>【Function Description】</font>

The read_input function is a member function of the Infer object, used to read a preprocessed numpy array input.

<font color='Blue'>【Parameter Description】</font>  

| Parameter Name | Parameter Type | Definition Description                                       |
| ----------- | ----------- | ------------------------ |
| input          | np.array       | The corresponding input numpy array                          |
| index          | int            | The index of the corresponding input numpy array. After load_model, debug can be turned on to view the order of multiple inputs. For single-input models, input index=0 will suffice |

<font color='Blue'>【Usage】</font> 

```Python
inf.read_input(input_image, 0)
inf.read_input(text_embeddings, 1)
```

:::tip Did you know

inf.read_input will automatically check the data type and size of the numpy array input by the developer. Inappropriate data types and sizes will be checked. This prevents unreasonable inputs from being read by the model.

:::

<font color='Blue'>【Return Value】</font>  

| Parameter Type | Description                                                  |
| ----------- | ------------------------ |
| bool           | Returns whether the numpy array at the corresponding index is correctly read into the Infer object |


### forward

<font color='Blue'>【Function Description】</font>

The forward function is a member function of the Infer object, generally used after reading the model input to perform the inference of the DiGua BPU heterogeneous model.

<font color='Blue'>【Parameter Description】</font>  

| Parameter Name | Parameter Type | Definition Description                                       |
| ----------- | ----------- | ------------------------ |
| more           | bool           | (Optional parameter, default is False) If you need to use this interface for multiple inferences after this inference is completed, you can set this parameter to True. |

<font color='Blue'>【Usage】</font> 

```Python
inf.forward()
```

### get_output

<font color='Blue'>【Function Description】</font>

The get_output function is a member function of the Infer object, generally used after the Infer object completes the forward function to obtain the inference results.

<font color='Blue'>【Usage】</font> 

```Python
inf.get_output()
```

The inference results are numpy arrays, and the two outputs can be obtained as follows:

```Python
classes_scores = inf.outputs[0].data
bboxes = inf.outputs[1].data
```

<font color='Blue'>【Return Value】</font>  

| Parameter Type | Description                                                  |
| ----------- | ------------------------ |
| bool           | Returns whether the Infer object can successfully obtain the model's inference results |


