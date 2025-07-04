---
sidebar_position: 2
---

# 4.1.2 Quick Start with ModelZoo

## Overview

The board-side inference interface used by ModelZoo is **bpu_infer_lib**, which is a Python wrapper for the C++ inference interface libdnn. It is designed to be **easy to install, user-friendly, and quick to get started**.

## Installation

You can install bpu_infer_lib on RDK series development boards using the following commands:

```
# For RDK X5, use:
pip install bpu_infer_lib_x5

# For RDK X3, use:
pip install bpu_infer_lib_x3
```

## Quick Start

This quick start guide uses the large detection model yoloworld from ModelZoo as an example to demonstrate the basic usage of the bpu_infer_lib inference library.

:::tip Tip

Yoloworld repository: https://github.com/D-Robotics/rdk_model_zoo/tree/main/demos/llm/yoloworld
:::

The repository provides a ready-to-deploy heterogeneous model `yolo_world.bin` for RDK series development boards.

Assuming you have downloaded the model, you can import the bpu_infer_lib inference library and use its most important class, **Infer**, to create an object:

```python
import bpu_infer_lib
inf = bpu_infer_lib.Infer(False)
```

The `inf` object is responsible for the **entire pipeline: model loading, input reading, inference, and output retrieval**.

The constructor of this object takes a parameter named `debug` (type: bool), which enables debug mode if set to True, providing more verbose output.

### Model Loading

To load a model, call the `load_model` member function of the `inf` object. This function takes the path to a heterogeneous `.bin` model and loads it:

```python
inf.load_model("yolo_world.bin")
```

After loading the model, you can check the required input and output properties. For example, you can view:

- The number of model inputs
- The layout of each input tensor
- The data type of each input tensor

```python
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

- The number of model outputs
- The layout of each output tensor
- The data type of each output tensor

```python
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

### Reading Inputs

To read inputs, use the `inf.read_input` member function, which takes two parameters:

- The first parameter is a numpy array matching the model's input type.
- The second parameter is the input index (starting from 0; for single-input models, use 0).

For the yoloworld model, use this interface to read the two preprocessed inputs: `input_image` and `text_embeddings`:

```python
inf.read_input(input_image, 0)
inf.read_input(text_embeddings, 1)
```

:::tip Did you know?

`inf.read_input` automatically checks the data type and shape of the input numpy array. Incompatible types or shapes will be validated and rejected, preventing invalid inputs from being read by the model.
:::

### Model Inference

After loading the model and reading the inputs, you can perform inference.

Use the `inf.forward` function, which takes no parameters and can be called directly after model loading and input reading:

```python
inf.forward()
```

### Retrieving Outputs

Once inference is complete on the BPU, use the `inf.get_output` function to retrieve the results. This function takes no parameters and can be called directly:

```python
inf.get_output()
```

The inference results are numpy arrays. You can access the two outputs as follows:

```python
classes_scores = inf.outputs[0].data
bboxes = inf.outputs[1].data
```
