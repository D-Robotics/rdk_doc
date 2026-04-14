---
sidebar_position: 1
---

# BPU algorithm inference (`pyeasy_dnn`)


## Module description

`pyeasy_dnn` is a lightweight Python wrapper around the underlying DNN inference stack. It exposes module-level model loading, `Model` inference, and read-only `TensorProperties` metadata. It targets neural-network inference on RDK X3 and RDK X5 (**software versions before 3.5.0**).

## Basics

- **Hardware**: RDK boards with NPU acceleration
- **Model formats**: Platform `.bin` models (subject to the underlying runtime)
- **Data**: Inputs and outputs are tensors; shape, dtype, and related attributes are queryable
- **Dependencies**: Requires `libdnn.so` on the device; Python only wraps the core flow
- **Capabilities**: Load models, run inference, query tensor properties

## Example

See `01_basic_sample/test_efficientnasnet_m.py`.


## API reference

### Module-level APIs

| API | Purpose |
| ---- | ----- |
| load | **Load a model file and return a `Model` object** |

### `Model` methods

| API | Purpose |
| ---- | ----- |
| forward | **Run inference with input tensors; returns output tensor objects** |

### `TensorProperties` (read-only)

| Attribute | Purpose |
| ---- | ----- |
| tensor_type | **Tensor role (input / output / intermediate, etc.)** |
| dtype | **Element dtype (e.g. float32, uint8, int32)** |
| layout | **Layout (e.g. NCHW, NHWC, CHW)** |
| shape | **Logical shape (e.g. (1, 3, 640, 640))** |
| alignedShape | **Hardware-aligned shape (memory layout required by RDK)** |
| validShape | **Valid shape for actual computation (may differ from aligned shape)** |
| scale_data | **Quantization scale (quantized models; often 1.0 for float models)** |


### Module-level API

#### load

<font color='Blue'>【Declaration】</font>

```python
model = dnn.load(model_path, **kwargs)
```

<font color='Blue'>【Description】</font>

Loads the model at `model_path`, parses it, adapts it to hardware, allocates memory, and returns an executable `Model` object.


<font color='Blue'>【Parameters】</font>

| Name | Description | Notes |
| ----------- | ------------------------ | --------  |
| model_path | Absolute or relative path to the model | Valid path; formats depend on the runtime (e.g. RKNN/ONNX) |
| **kwargs | Optional arguments | e.g. `core_num`, `precision`; omit if unused |


<font color='Blue'>【Returns】</font>

| Value | Description |
| ------ | ----- |
| model | Initialized `Model` with inference capability and tensor metadata |


<font color='Blue'>【Example】</font>

```python
from hobot_dnn import pyeasy_dnn as dnn

models = dnn.load('../models/efficientnasnet_m_300x300_nv12.bin')
```


<font color='Blue'>【Notes】</font>

- The path must exist and the model must match the current RDK platform.
- Load failures raise `RuntimeError`; handle with try/except.
- `kwargs` must match the underlying `Dnnpy_load` keyword arguments when used.

### `Model` methods

#### forward

<font color='Blue'>【Description】</font>

Runs forward inference with the loaded model, consuming input tensors and returning a list of `TensorProperties` objects for outputs.

<font color='Blue'>【Declaration】</font>

```python
output_tensors = model.forward(inputs, **kwargs)
```

<font color='Blue'>【Parameters】</font>


| Name | Description | Notes |
| ----------- | ------------------------ | --------  |
| inputs | List of inputs | Each element is a NumPy array or bytes; must match model inputs |
| **kwargs | Optional inference args | e.g. `batch_size`; omit if unused |


<font color='Blue'>【Returns】</font>

| Value | Description |
| ------ | ----- |
| output_tensors | List of `TensorProperties` objects with values and metadata (shape/dtype, etc.) |


:::info Note
Input shape and dtype must match the model’s input tensors exactly.  
Multi-input models require `inputs` in the defined order.  
Latency depends on model size and core count; consider async execution to avoid blocking the main thread.
:::


### `TensorProperties` (read-only)

`TensorProperties` holds tensor metadata. Access attributes directly; there are no mutating methods.


| Name | Description | Type |
| ----------- | ------------------------ | --------  |
| tensor_type | Tensor role (input/output/intermediate, etc.) | str / int |
| dtype | Element dtype | str / enum |
| layout | Layout (e.g. NCHW/NHWC/CHW) | str |
| shape | Logical shape | tuple of int |
| alignedShape | Hardware-aligned shape | tuple of int |
| validShape | Valid computational shape | tuple of int |
| scale_data | Quantization scale | float or array |


#### Example

```
# Suppose `model` is loaded and `inputs` are prepared
input_tensor = model.inputs[0]

print(f"Tensor type: {input_tensor.tensor_type}")
print(f"Dtype: {input_tensor.dtype}")
print(f"Layout: {input_tensor.layout}")
print(f"Logical shape: {input_tensor.shape}")
print(f"Aligned shape: {input_tensor.alignedShape}")
print(f"Valid shape: {input_tensor.validShape}")
print(f"Scale: {input_tensor.scale_data}")

output_tensor = model.forward(inputs=[input_data])[0]
print(f"Output dtype: {output_tensor.dtype}")
```

<font color='Blue'>【Notes】</font>

- All attributes are read-only; adjust preprocessing instead of mutating metadata.
- `alignedShape` reflects hardware alignment (e.g. width aligned to 32 on X3); storage size follows this attribute.
- `validShape` is the range of valid dimensions; often matches `shape`.
- `scale_data` matters for quantized models (e.g. int8 RKNN); float models often use 1.0.

## Additional notes

1. **Release**: `dnn_python.cpp` does not expose an explicit `release`; resources are usually freed when the `Model` is garbage-collected. For manual release, use lower-level APIs if available.
2. **Errors**: Load/inference failures raise Python exceptions (e.g. `RuntimeError`); use try/except.
3. **Alignment**: `alignedShape` must follow RDK memory rules (e.g. X3 width alignment); mismatch can cause wrong results or errors.
4. **Multi I/O**: `forward` accepts a list for multi-input models; returned outputs follow the model’s output order.
