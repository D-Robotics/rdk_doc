---
sidebar_position: 2
---

# 4.3.2 ModelZoo Quick Start

## Overview

The board-side inference interface used by ModelZoo is **bpu_infer_lib**, which is a Python wrapper of the board-side inference C++ interface libdnn. It is designed to be **easy to install, easy to learn, and quick to get started**.

## Installation Method

bpu_infer_lib can be installed on the RDK series development boards using the following commands:

```
# For RDK X5, use the following command
pip install bpu_infer_lib_x5

# For RDK X3, use the following command
pip install bpu_infer_lib_x3
```

## Quick Start

This quick start guide uses the detection model yoloworld from ModelZoo as an example to show developers the basic usage of the bpu_infer_lib inference library.

:::tip Tip

Yoloworld repository link：https://github.com/D-Robotics/rdk_model_zoo/tree/main/demos/llm/yoloworld
:::

The repository provides the geeva heterogeneous model `yolo_world.bin` that can be directly deployed on the RDK series development boards.

Assuming the developer has downloaded the model, we can import the bpu_infer_lib inference library and use the most important class of this library - the **Infer** class to create an object:

```Python
import bpu_infer_lib
inf = bpu_infer_lib.Infer(False)
```

The inf object will be responsible for the entire process of **model loading, input reading, model inference, and output acquisition**.

The constructor of this object accepts one parameter, named debug, which is of type bool. If debug mode is enabled, more output will be printed.

### Model Loading

For model loading, we will call the load_model member function of the inf object, which accepts the path to a geeva heterogeneous .bin model and completes the model loading:

```Python
inf.load_model("yolo_world.bin")
```

After the model is loaded, we can view the attributes required for the model's inputs and outputs. For example, we can view the following information:

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

Similarly, we can also view:

- The number of model inputs
- The data layout (layout) required for each model input
- The data type (type) required for each model input

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

### Reading Inputs

For reading inputs, we use the `inf.read_input` member function, which accepts two parameters.

- The first parameter is a numpy array that matches the model input type
- The second parameter is the input index, i.e., which input (starting from 0, for single-input models, fill in 0 here)

We will use this interface to read the two inputs required by the yoloworld model: `input_image` and `text_embeddings` into the model:

```Python
inf.read_input(input_image, 0)
inf.read_input(text_embeddings, 1)
```

:::tip Did you know

inf.read_input will automatically check the data type and size of the numpy array input by the developer. Inappropriate data types and sizes will be checked. This prevents unreasonable inputs from being read by the model.
:::

### Model Inference

After completing model loading and input reading, the next step is to perform model inference.

Model inference uses the `inf.forward` function, which does not accept any parameters and can be executed after model loading and input reading:

```Python
inf.forward()
```

### Obtaining Outputs

When the heterogeneous model completes inference on the BPU, we can use the `inf.get_output` function to obtain the inference results. The `inf.get_output` function does not accept any parameters and can be used directly:

```Python
inf.get_output()
```

The inference results are numpy arrays, and the two outputs can be obtained as follows:

```Python
classes_scores = inf.outputs[0].data
bboxes = inf.outputs[1].data
```