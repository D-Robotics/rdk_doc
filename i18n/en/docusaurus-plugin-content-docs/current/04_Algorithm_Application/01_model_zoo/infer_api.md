---
sidebar_position: 3
---

# 4.1.3 ModelZoo Inference API

## Overview

The board-side inference interface used by ModelZoo is **bpu_infer_lib**. In the previous section, we described how to install bpu_infer_lib, get started quickly, and use its basic features.

In this section, we will introduce the details and usage of each API provided by bpu_infer_lib.

## Infer Object

The Infer object orchestrates the entire inference process. By constructing an Infer object, you can handle the **model loading, input reading, inference, and output retrieval** in a complete workflow.

### Infer

<font color='Blue'>[Function Description]</font>

This function is the constructor for the Infer object, typically used after importing bpu_infer_lib.

<font color='Blue'>[Parameter Description]</font>  

| Parameter Name | Type | Description |
| -------------- | ---- | ----------- |
| debug | bool | Whether to enable debug printing for subsequent inference steps |

<font color='Blue'>[Usage]</font> 

```Python
import bpu_infer_lib
inf = bpu_infer_lib.Infer(True)
```

<font color='Blue'>[Return Value]</font>  

| Type | Description |
| ---- | ----------- |
| Infer | Returns an Infer object with available interfaces for further use |

### load_model

<font color='Blue'>[Function Description]</font>

The load_model function is a member of the Infer object, used to load a DiGua BPU heterogeneous model.

<font color='Blue'>[Parameter Description]</font>  

| Parameter Name | Type | Description |
| -------------- | ---- | ----------- |
| model_path | string | Path to the DiGua BPU heterogeneous model .bin file |

<font color='Blue'>[Usage]</font> 

```Python
inf.load_model("yolo_world.bin")
```

After loading the model, you can check the required input and output properties. For example, you can view:

- Number of model inputs
- Data layout required for each input
- Data type required for each input

```Python
print("Number of model's inputs:", len(inf.inputs))
print("Input[0]'s tensor layout:", inf.inputs[0].properties.tensorLayout)
print("Input[0]'s tensor type:", inf.inputs[0].properties.tensorType)
print("Input[1]'s tensor layout:", inf.inputs[1].properties.tensorLayout)
print("Input[1]'s tensor type:", inf.inputs[1].properties.tensorType)
```

Example output:

```
Number of model's inputs: 2
Input[0]'s tensor layout: HB_DNN_LAYOUT_NCHW
Input[0]'s tensor type: HB_DNN_TENSOR_TYPE_F32
Input[1]'s tensor layout: HB_DNN_LAYOUT_NCHW
Input[1]'s tensor type: HB_DNN_TENSOR_TYPE_F32
```

Similarly, you can check:

- Number of model outputs
- Data layout required for each output
- Data type required for each output

```Python
print("Number of model's outputs:", len(inf.outputs))
print("Output[0]'s tensor layout:", inf.outputs[0].properties.tensorLayout)
print("Output[0]'s tensor type:", inf.outputs[0].properties.tensorType)
print("Output[1]'s tensor layout:", inf.outputs[1].properties.tensorLayout)
print("Output[1]'s tensor type:", inf.outputs[1].properties.tensorType)
```

Example output:

```
Number of model's outputs: 2
Output[0]'s tensor layout: HB_DNN_LAYOUT_NCHW
Output[0]'s tensor type: HB_DNN_TENSOR_TYPE_F32
Output[1]'s tensor layout: HB_DNN_LAYOUT_NCHW
Output[1]'s tensor type: HB_DNN_TENSOR_TYPE_F32
```

<font color='Blue'>[Return Value]</font>  

| Type | Description |
| ---- | ----------- |
| bool | Returns whether the model was loaded successfully |


### read_input

<font color='Blue'>[Function Description]</font>

The read_input function is a member of the Infer object, used to read a preprocessed numpy array as input.

<font color='Blue'>[Parameter Description]</font>  

| Parameter Name | Type | Description |
| -------------- | ---- | ----------- |
| input | np.array | The input numpy array |
| index | int | The index of the input numpy array. After loading the model, you can enable debug to check the order of multiple inputs. For single-input models, use index=0. |

<font color='Blue'>[Usage]</font> 

```Python
inf.read_input(input_image, 0)
inf.read_input(text_embeddings, 1)
```

:::tip Did you know?

inf.read_input automatically checks the data type and shape of the input numpy array. Any mismatched data type or shape will be validated, preventing invalid inputs from being read by the model.
:::

<font color='Blue'>[Return Value]</font>  

| Type | Description |
| ---- | ----------- |
| bool | Returns whether the numpy array at the specified index was successfully read by the Infer object |


### forward

<font color='Blue'>[Function Description]</font>

The forward function is a member of the Infer object, typically used after reading model inputs to perform inference with the DiGua BPU heterogeneous model.

<font color='Blue'>[Parameter Description]</font>  

| Parameter Name | Type | Description |
| -------------- | ---- | ----------- |
| more | bool | (Optional, default is False) If you need to perform multiple inferences using this interface, set this parameter to True. |

<font color='Blue'>[Usage]</font> 

```Python
inf.forward()
```

### get_output

<font color='Blue'>[Function Description]</font>

The get_output function is a member of the Infer object, typically used after calling the forward function to retrieve inference results.

<font color='Blue'>[Usage]</font> 

```Python
inf.get_output()
```

The inference results are numpy arrays. You can obtain the two outputs as follows:

```Python
classes_scores = inf.outputs[0].data
bboxes = inf.outputs[1].data
```

<font color='Blue'>[Return Value]</font>  

| Type | Description |
| ---- | ----------- |
| bool | Returns whether the Infer object successfully retrieved the inference results |


