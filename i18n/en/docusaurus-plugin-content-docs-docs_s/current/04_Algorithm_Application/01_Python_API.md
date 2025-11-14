---
sidebar_position: 1
id: python-api
title: Python API Reference Manual
sidebar_label: 4.1 Python API
---
## Overview
`hbm_runtime` is a Python binding interface based on pybind11, designed to access and operate the C++ libraries `libhbucp`/`libdnn`, providing high-performance neural network model loading and inference capabilities.

This interface encapsulates low-level model runtime details, enabling Python users to conveniently load single or multiple neural network models, manage input/output metadata, and flexibly execute inference operations. The interface supports multiple input data formats and ensures input data is stored contiguously through intelligent conversion, thereby enhancing runtime efficiency.

### Applicable Scenarios
- Rapid integration and invocation of `hbm_runtime` runtime functionality within a Python environment.
- Applications requiring high inference efficiency and flexibility, such as robotic vision and intelligent edge computing.
- Scenarios requiring simultaneous loading and management of multiple models, with dynamic configuration of inference priorities and hardware resource allocation.

### Key Features
- **Multi-model support**  
  - Supports loading either a single model or a group of multiple models. Each model can independently retrieve input/output metadata and perform inference.
- **Flexible input formats**  
  - Supports single input (`numpy.ndarray`);
  - Supports input dictionaries mapped by model name (`Dict[str, np.ndarray]`);
  - Supports multi-model, multi-input structures (`Dict[str, Dict[str, np.ndarray]]`). All inputs are automatically checked for C-contiguous memory layout and copied if necessary to ensure efficient and correct low-level access.
- **Specifiable inference priority**  
  - Allows explicit specification of scheduling priority for model tasks via the `priority: Dict[str, int]` parameter, enabling the scheduler to reasonably allocate inference tasks under limited hardware resources.
- **Specifiable BPU cores for inference**  
  - Supports explicitly assigning BPU compute cores for model inference via `bpu_cores: Dict[str, List[int]]`, enabling strategies such as heterogeneous core binding.
- **Parallel multi-model inference**  
  - For multi-model input scenarios, the underlying system automatically employs a multi-threaded mechanism to execute each model’s inference task in parallel (`multi-threaded launch`). This achieves higher throughput on multi-core BPU systems (note: single-core BPU execution remains sequential).
- **Metadata access interface**  
  - Number, names, and data types (`hbDNNDataType` enum) of inputs/outputs;
  - Tensor shapes, memory strides, and quantization parameters (including scale, zero point, and quantization type) for inputs/outputs;
  - Model description information, HBM file metadata, etc.
- **Fully bound type system**  
  - Supports quantization parameter structure `QuantParams`, data type enum `hbDNNDataType`, model scheduling parameter object `SchedParam`, and quantization type enum `hbDNNQuantiType`, providing type-safe attribute access.

## Installation
The `hbm_runtime` module is a high-performance Python interface for C++-based inference runtime, relying on pybind11 and Horizon Robotics’ underlying inference libraries (e.g., `libdnn`, `libhbucp`). It supports installation via system DEB packages (`.deb`) and is compatible with Python 3.10 and above.

### System Dependencies
| Dependency          | Minimum Version | Description                                               |
|---------------------|-----------------|-----------------------------------------------------------|
| Python              | ≥ 3.10          | Python 3.10 is recommended                                |
| pip                 | ≥ 22.0          | Required for installing wheel packages                    |
| pybind11            | Any             | Used during build; not required when installing package   |
| scikit-build-core   | ≥ 0.7           | Used when building wheel packages (source builds only)    |
| Horizon Base Libraries | Platform-dependent | e.g., `libdnn.so`, `libucp.so`, typically provided by BSP |

### Building the Wheel Package
There are three methods to build the wheel package, as described below.

#### Building During DEB Installation
The `hbm_runtime` wheel is automatically built during installation of the `hobot-dnn` package. After installing the `.deb` package, the `hbm_runtime` `.whl` file will be generated.

```bash
# Install from repository
sudo apt-get install hobot-dnn

# Install from local .deb package (note: package name may vary by build timestamp)
dpkg -i hobot-dnn_4.0.4-20250909195426_arm64.deb

# After installation, the wheel package can be found in /tmp on the device
ls /tmp

# Note: wheel filename varies by version; xxx represents the version number
# hbm_runtime-x.x.x-cp310-cp310-manylinux_2_34_aarch64.whl
```

#### Building During System Software Compilation
When compiling the system software image, the `hobot-dnn` `.deb` package is installed, which triggers the build of the `hbm_runtime` wheel package. The resulting `.whl` file is saved to `out/product/deb_packages`.

```bash
sudo ./pack_image.sh

ls out/product/deb_packages

# Note: wheel filename varies by version; xxx represents the version number
# hbm_runtime-x.x.x-cp310-cp310-manylinux_2_34_aarch64.whl
```

#### Building On-Device
```bash
# Navigate to the hbm_runtime source directory
cd /usr/hobot/lib/hbm_runtime

# Run the build script
./build.sh

# View the built wheel package
ls dist/

# Note: wheel filename varies by version; xxx represents the version number
# hbm_runtime-x.x.x-cp310-cp310-manylinux_2_34_aarch64.whl
```

### Installation Methods

#### Using a Wheel Package
Choose one of the following two methods:

- **Install from local wheel file**  
  Locate the `.whl` file built in the "Building the Wheel Package" section.

  ```bash
  # Example: install local .whl file using pip (note: filename varies by version)
  pip install hbm_runtime-x.x.x-cp310-cp310-manylinux_2_34_aarch64.whl
  ```

- **Install from pypi**
  ```bash
  pip install hbm_runtime
  ```

#### Using a .deb Package
Choose one of the following two methods:

- **Install from local .deb package**
  ```bash
  # Example: install .deb package (note: package name may vary by build timestamp)
  sudo dpkg -i hobot-dnn_4.0.2-20250714201215_arm64.deb
  ```

- **Install via apt repository**
  ```bash
  sudo apt-get install hobot-dnn
  ```

- **Common Issues**
  - If files are not updated after `.deb` installation, check whether other dependencies (e.g., an older version of `hobot-spdev`) are preventing overwrites.
  - Use `dpkg -L hobot-dnn` to verify whether files were deployed successfully.

### Uninstallation Instructions
- Uninstall pip-installed package:
  ```bash
  pip uninstall hbmruntime
  ```

- Uninstall .deb-installed package:
  ```bash
  sudo apt remove hobot-dnn
  ```

## Quick Start
This section demonstrates how to use `hbm_runtime` for model loading and inference. With just a few lines of code, you can run a model and obtain output results.

### Environment Setup
Ensure that `HBMRuntime` is properly installed (see [Installation](#installation)) and that you have an `.hbm` model file ready.

### Examples

#### Single-Model, Single-Input Inference
Applicable when the model has only one input tensor.

```python
import numpy as np
from hbm_runtime import HB_HBMRuntime

# Load model
model = HB_HBMRuntime("/opt/hobot/model/s100/basic/lanenet256x512.hbm")

# Get model name and input name
model_name = model.model_names[0]
input_name = model.input_names[model_name][0]  # assuming the model has only one input

# Get the corresponding input shape
input_shape = model.input_shapes[model_name][input_name]

# Construct numpy input
input_tensor = np.ones(input_shape, dtype=np.float32)

# Run inference
outputs = model.run(input_tensor)

# Retrieve output results
output_array = outputs[model_name]
print("Output:", output_array)
```

#### Single-Model, Multi-Input Inference
Applicable when the model has multiple input tensors.

```python
import numpy as np
from hbm_runtime import HB_HBMRuntime

hb_dtype_map = {
    "U8": np.uint8,
    "S8": np.int8,
    "F32": np.float32,
    "F16": np.float16,
    "U16": np.uint16,
    "S16": np.int16,
    "S32": np.int32,
    "U32": np.uint32,
    "BOOL8": np.bool_,
}

# Load model
model = HB_HBMRuntime("/opt/hobot/model/s100/basic/yolov5x_672x672_nv12.hbm")

# Get model name (assuming only one model is loaded)
model_name = model.model_names[0]

# Prepare input names, shapes, and data types
input_names = model.input_names[model_name]
input_shapes = model.input_shapes[model_name]
input_dtypes = model.input_dtypes[model_name]

# Construct input dictionary

input_tensors = {}
for name in input_names:
    shape = input_shapes[name]
    np_dtype = hb_dtype_map.get(input_dtypes[name].name, np.float32)  # fallback
    input_tensors[name] = np.ones(shape, dtype=np_dtype)

# Optional: Specify inference priority and BPU device
priority = {model_name: 5}
bpu_cores = {model_name: [0]}

model.set_scheduling_params(
    priority=priority,
    bpu_cores=bpu_cores
)

# Perform inference, optionally specifying priority and BPU cores
results = model.run(input_tensors)

# Output results
for output_name, output_data in results[model_name].items():
    print(f"Output: {output_name}, shape={output_data.shape}")

```

### Common Issues
| Issue                              | Description                                                                 |
|-----------------------------------|-----------------------------------------------------------------------------|
| How to obtain model names?        | Use `model.model_names` to view the list of loaded model names.             |
| How to confirm input dimensions/types? | Use `model.input_shapes` and `model.input_dtypes`.                          |
| How to confirm BPU core allocation? | Specify BPU cores using the `bpu_cores` parameter (e.g., [0, 1, 2, 3]); actual availability depends on hardware support. |

For more advanced usage (e.g., multi-input models, reading quantization parameters), please refer to the [API Reference](#module-class-function-reference-api-reference).

## Module/Class/Function Reference (API Reference)
The Python module `hbm_runtime` is a PyBind11-wrapped interface for Horizon's HBM model inference, implemented on top of the underlying `libdnn` and `libhbucp` libraries. It provides unified APIs for model loading, querying input/output information, performing inference, and supports multi-model loading, multi-input inference, specifying inference models, BPU cores, and inference task priorities.

### Enumerations
#### hbDNNDataType
##### Tensor data type enumeration:
- S4: 4-bit signed
- U4: 4-bit unsigned
- S8: 8-bit signed
- U8: 8-bit unsigned
- F16: 16-bit float
- S16: 16-bit signed
- U16: 16-bit unsigned
- F32: 32-bit float
- S32: 32-bit signed
- U32: 32-bit unsigned
- F64: 64-bit float
- S64: 64-bit signed
- U64: 64-bit unsigned
- BOOL8: 8-bit bool type
- MAX: Maximum value (reserved field)

##### Example
```python
from hbm_runtime import hbDNNDataType
print(hbDNNDataType.F32)  # Output: hbDNNDataType.F32
```
#### hbDNNQuantiType
##### Tensor quantization type enumeration:
- NONE: Non-quantized type
- SCALE: Linear scale quantization (scale + zero_point)
##### Example
```python
from hbm_runtime import hbDNNQuantiType
print(hbDNNQuantiType.SCALE)  # Output: hbDNNQuantiType.SCALE
```

### Class Reference
#### HB_HBMRuntime
Model runtime class that loads one or multiple HBM model files and provides inference execution interfaces.
##### Constructor
- Function signature
    ```python
    HB_HBMRuntime(model_file: str)
    HB_HBMRuntime(model_files: List[str])
    ```
- Parameters

    | Parameter    | Type         | Description                                 |
    |--------------|--------------|---------------------------------------------|
    | model_file   | str          | Path to the HBM model file                  |
    | model_files  | List[str]    | Paths to multiple HBM model files (for multi-model loading) |
- Return value

  Class instance
- Example
    ```python
    from hbm_runtime import HB_HBMRuntime

    model = HB_HBMRuntime("model.hbm")
    # Or load multiple models:
    model = HB_HBMRuntime(["model1.hbm", "model2.hbm"])
    ```

##### Attributes
All attributes listed below are read-only.
- version: str
  - Description:
    - Retrieves the library version number.
  - Structure:
    - str: Version number string.
    - Example:
        ```python
        print("Version:", HB_HBMRuntime.version)
        ```
- model_names: List[str]
  - Description:
    - List of loaded model names.
  - Structure:
    - List[str]: List of model names.
  - Example:
    ```python
    print(model.model_names)
    # Output: ['model_1', 'model_2']
    ```
- model_count: int
  - Description:
    - Number of loaded models.
  - Structure:
    - int: Number of loaded models.
  - Example:
    ```python
    print(model.model_count)
    # Output: 2
    ```
- model_descs: Dict[str, str]
  - Description:
    - Description information for each model (embedded notes from the model).
  - Structure:
    - Dict[str, str]: Keys are model names; values are overall model descriptions, typically provided by the compiler.
  - Example:
    ```python
    # Print descriptions for all models
    print(model.model_descs)
    # Output: {'yolov5x_672x672_nv12': 'Image classification model based on ResNet-18.'}
    ```

- hbm_descs: Dict[str, str]
  - Description:
    - Annotation or metadata information from each HBM file.
  - Structure:
    - Dict[str, str]: Keys are HBM filenames (e.g., "resnet18"); values are annotation or metadata strings from the HBM file.
  - Example:
    ```python
    # Print description information for all model files
    print(model.hbm_descs)
    # Output: {'/opt/hobot/model/s100/basic/yolov5x_672x672_nv12.hbm': 'xxx'}
    ```
- input_counts: Dict[str, int]
  - Description:
    - Number of input tensors for each model.
  - Structure:
    - Dict[str, int]: Keys are model names; values are the number of input tensors for the corresponding model.
  - Example:
    ```python
    # Print description information for all model files
    print(model.input_counts)
    # Output: {'yolov5x_672x672_nv12': 2}
    ```
- input_names: Dict[str, List[str]]
  - Description:
    - List of input tensor names for each model.
  - Structure:
    - Outer Dict[str, ...]: Keys are model names.
    - Inner List[str]: List of input tensor names for the model.
  - Example:
    ```python
    print(model.input_names)
    # Output: {'yolov5x_672x672_nv12': ['data_y', 'data_uv']}
    ```
- input_descs: Dict[str, Dict[str, str]]
  - Description:
    - Description for each input tensor.
  - Structure:
    - Outer Dict[str, ...]: Model names.
    - Inner Dict[str, str]: Keys are input tensor names; values are their descriptions.
  - Example:
    ```python
    # Print description information for all model files
    print(model.input_descs)
    # Output: {'yolov5x_672x672_nv12': {'data_uv': 'xxx', 'data_y': 'xxx'}}
    ```
- input_shapes: Dict[str, Dict[str, List[int]]]
  - Description:
    - Shape of each input tensor.
  - Structure:
    - Outer Dict[str, ...]: Model names.
    - Inner Dict[str, List[int]]: Keys are input names; values are tensor dimensions (shapes).
  - Example:
    ```python
    model.input_shapes
    # Output: {'yolov5x_672x672_nv12': {'data_uv': [1, 336, 336, 2], 'data_y': [1, 672, 672, 1]}}
    ```
- input_dtypes: Dict[str, Dict[str, hbDNNDataType]]
  - Description:
    - Data type of each input tensor.
  - Structure:
    - Outer Dict[str, ...]: Model names.
    - Inner Dict[str, hbDNNDataType]: Keys are input tensor names; values are data types (e.g., F32, U8).
  - Example:
   ```python
    print(model.input_dtypes)
    # Output: {'yolov5x_672x672_nv12': {'data_uv': <hbDNNDataType.U8: 3>, 'data_y': <hbDNNDataType.U8: 3>}}
    
    ```

- input_quants: Dict[str, Dict[str, QuantParams]]
  - Functionality:
    - Provides quantization parameter information for all input tensors of each model. Used to support pre-processing computations for quantized models or to understand the quantization scheme of tensors.
  - Structure:
    - Outer Dict[str, ...]: Keys are model names (e.g., "resnet50").
    - Inner Dict[str, QuantParams]: Keys are input tensor names; values are QuantParams instances.
    - QuantParams class attributes:
      - scale: np.ndarray — Quantization scale factor, typically a floating-point array.
      - zero_point: np.ndarray — Zero point used for symmetric/asymmetric quantization offset.
      - quant_type: hbDNNQuantiType — Enumeration value indicating quantization type (e.g., SCALE, NONE).
      - axis: int — Indicates the axis along which quantization is applied, in the case of per-channel quantization.
  - Example:
    ```python
    quanti_info = model.input_quants
    for model, inputs in quanti_info.items():
        print(f"{model}:")
        for name, info in inputs.items():
            print(f"  {name}:")
            print(f"    quant_type: {info.quant_type.name}")
            print(f"    quantize_axis: {info.axis}")
            print(f"    scale_data: {info.scale.tolist()}")
            print(f"    zero_point_data: {info.zero_point.tolist()}")
    ```

- input_strides: Dict[str, Dict[str, List[int]]]
  - Functionality:
    - Stride information for each input tensor.
  - Structure:
    - Outer Dict[str, ...]: Model names.
    - Inner Dict[str, List[int]]: Keys are input names; values are stride information for the corresponding input tensors.
  - Example:
    ```python
    print(model.input_strides)
    # Output: {'yolov5x_672x672_nv12': {'data_uv': [-1, -1, 2, 1], 'data_y': [-1, -1, 1, 1]}}
    ```
    Note: For detailed explanation of strides, refer to the description in the libdnn library within the [OE documentation](http://j6.doc.oe.hobot.cc/3.0.31/guide/ucp/runtime/bpu_sdk_api/data_structure/hbDNNTensorProperties.html).

- output_counts: Dict[str, int]
  - Functionality:
    - Number of output tensors for each model.
  - Structure:
    - Dict[str, int]: Keys are model names; values are the number of output tensors for the corresponding model.
  - Example:
    ```python
    print(model.output_counts)
    # Output: {'yolov5x_672x672_nv12': 3}
    ```

- output_names: Dict[str, List[str]]
  - Functionality:
    - List of output tensor names for each model.
  - Structure:
    - Outer Dict[str, ...]: Keys are model names.
    - Inner List[str]: List of output tensor names for the corresponding model.
  - Example:
    ```python
    print(model.output_names)
    # Output: {'yolov5x_672x672_nv12': ['output', '1310', '1312']}
    ```

- output_descs: Dict[str, Dict[str, str]]
  - Functionality:
    - Descriptions for each output tensor.
  - Structure:
    - Outer Dict[str, ...]: Model names.
    - Inner Dict[str, str]: Keys are output tensor names; values are their corresponding descriptions.
  - Example:
    ```python
    print(model.output_descs)
    # Output: {'yolov5x_672x672_nv12': {'1310': 'xxx', '1312': 'xxx', 'output': 'xxx'}}
    ```

- output_shapes: Dict[str, Dict[str, List[int]]]
  - Functionality:
    - Shape of each output tensor.
  - Structure:
    - Outer Dict[str, ...]: Model names.
    - Inner Dict[str, List[int]]: Keys are output names; values are the dimensions (shape) of the corresponding output tensors.
  - Example:
    ```python
    print(model.output_shapes)
    # Output: {'yolov5x_672x672_nv12': {'1310': [1, 42, 42, 255], '1312': [1, 21, 21, 255], 'output': [1, 84, 84, 255]}}
    ```

- output_dtypes: Dict[str, Dict[str, hbDNNDataType]]
  - Functionality:
    - Data type of each output tensor.
  - Structure:
    - Outer Dict[str, ...]: Model names.
    - Inner Dict[str, hbDNNDataType]: Keys are output tensor names; values are data types (e.g., F32, U8).
  - Example:
    ```python
    print(model.output_dtypes)
    # Output: {'yolov5x_672x672_nv12': {'1310': <hbDNNDataType.S32: 8>, '1312': <hbDNNDataType.S32: 8>, 'output': <hbDNNDataType.S32: 8>}}
    ```

- output_quants: Dict[str, Dict[str, QuantParams]]
  - Functionality:
    - Provides quantization parameter information for all output tensors of each model. Used to support post-processing computations for quantized models (e.g., converting int8 data back to float32) or to understand the quantization scheme (e.g., scale-based).
  - Structure:
    - Outer Dict[str, ...]: Keys are model names (e.g., "resnet50").
    - Inner Dict[str, QuantParams]: Keys are output tensor names; values are QuantParams instances.
    - QuantParams class attributes:
      - scale: np.ndarray — Quantization scale factor, typically a floating-point array.
      - zero_point: np.ndarray — Zero point used for symmetric/asymmetric quantization offset.
      - quant_type: hbDNNQuantiType — Enumeration value indicating quantization type (e.g., SCALE, NONE).
      - axis: int — Indicates the axis along which quantization is applied, in the case of per-channel quantization.
  - Example:
    ```python
    output_quanti = model.output_quants
    for model, outputs in output_quanti.items():
        print(f"{model}:")
        for name, info in outputs.items():
            print(f"  {name}:")
            print(f"    quant_type: {info.quant_type.name}")
            print(f"    quantize_axis: {info.axis}")
            print(f"    scale_data: {info.scale}")
            print(f"    zero_point_data: {info.zero_point}")
    ```

- output_strides: Dict[str, Dict[str, List[int]]]
  - Functionality:
    - Stride information for each output tensor.
  - Structure:
    - Outer Dict[str, ...]: Model names.
    - Inner Dict[str, List[int]]: Keys are output names; values are stride information for the corresponding output tensors.
  - Example:
    ```python
    print(model.output_strides)
    # Output: {'yolov5x_672x672_nv12': {'1310': [1806336, 43008, 1024, 4], '1312': [451584, 21504, 1024, 4], 'output': [7225344, 86016, 1024, 4]}}
    ```
    Note: For detailed explanation of strides, refer to the description in the libdnn library within the [OE documentation](http://j6.doc.oe.hobot.cc/3.0.31/guide/ucp/runtime/bpu_sdk_api/data_structure/hbDNNTensorProperties.html).

- sched_params: Dict[str, SchedParam]
  - Functionality:
    - sched_params provides scheduling parameters for all currently loaded models, including:
      - Priority (`priority`)
      - Custom ID (`customId`)
      - Assigned BPU cores (`bpu_cores`)
      - Device ID (`deviceId`)
    These scheduling parameters influence how models execute on hardware, especially critical in multi-model deployments or multi-core devices.
  - Structure:
    - Outer Dict[str, ...]: Model names.
    - Inner SchedParam: Instance of the SchedParam class containing scheduling parameters `priority`, `customId`, `bpu_cores`, and `deviceId` for the model:
      ```python
      {
          "model_name": SchedParam(
              priority: int,
              customId: int,
              bpu_cores: List[int],
              deviceId: int
          )
      }
      ```
  - Example:
    ```python
    params = model.sched_params
    for name, sched in params.items():
        print(f"Model: {name}")
        print(f"  priority: {sched.priority}")
        print(f"  customId: {sched.customId}")
        print(f"  bpu_cores: {sched.bpu_cores}")
        print(f"  deviceId: {sched.deviceId}")
    # Output:
    # Model: yolo12s_detect_nashe_640x640_nv12
    #   priority: 10
    #   customId: 0
    #   bpu_cores: [0]
    #   deviceId: 0
    # Model: yolov5nu_detect_nashe_640x640_nv12
    #   priority: 66
    #   customId: 0
    #   bpu_cores: [-1]
    #   deviceId: 0
    ```
    Note: A `bpu_cores` value of -1 indicates automatic core assignment by the scheduler.

##### Inference Execution Functions

- run(input_tensor, **kwargs)
  - Function Signature:
    ```python
    run(input_tensor: np.ndarray, **kwargs) -> Dict[str, Dict[str, np.ndarray]]
    ```
  - Functionality:
    - Designed for inference with a single model and a single input. The input is a NumPy array corresponding to the model's sole input tensor. When only one model is loaded, specifying the model name is optional.
  - Parameters:

    | Parameter       | Type          | Description                                                                                             |
    |-----------------|---------------|---------------------------------------------------------------------------------------------------------|
    | input_tensor    | np.ndarray    | Single input tensor, used only in single-model, single-input inference scenarios. The tensor shape must match the expected input shape of the model. |
    | kwargs          | Variable keyword arguments | `model_name` (`str`): Specifies the model name (optional if only one model is loaded; required otherwise). |

  - Return Value:
    - Type: Dict[str, Dict[str, np.ndarray]]
    - Outer dictionary keys: Model names.
    - Inner dictionary keys: Output tensor names.
    - Values: Corresponding NumPy output arrays.
  - Example: Refer to the "Quick Start" section, specifically the part on single-model, single-input inference.

- run(input_tensors: Dict[str, np.ndarray], **kwargs)
  - Function Signature:
    ```python
    run(input_tensors: Dict[str, np.ndarray], **kwargs) -> Dict[str, Dict[str, np.ndarray]]
    ```
  - Functionality:Applicable to single-model, multi-input inference. Each input tensor is specified by its input name, consistent with the model definition. When only one model is loaded, the model name in the input can be omitted.
  - Parameter Description

    | Parameter Name   | Type                      | Description                                                                                           |
    |------------------|---------------------------|--------------------------------------------------------------------------------------------------------|
    | input_tensors    | Dict[str, np.ndarray]     | Multiple input tensors, used only in single-model multi-input inference scenarios. Keys are input tensor names, and values are corresponding NumPy arrays. |
    | kwargs           | Variable keyword arguments| `model_name` (`str`): Specifies the model name (can be omitted if only one model is loaded; otherwise, it must be specified). |

  - Return Value  
    Same as above.
  - Example: Refer to the "Quick Start" section, specifically the part on single-model multi-input inference.

- run(multi_input_tensors: Dict[str, Dict[str, np.ndarray]], **kwargs)
  - Function Signature
    ```python
    run(multi_input_tensors: Dict[str, Dict[str, np.ndarray]], **kwargs) -> Dict[str, Dict[str, np.ndarray]]
    ```
  - Function Description  
    Applicable to scenarios involving simultaneous inference of multiple models. Each model provides its own set of multiple input tensors. There is no need to specify `model_name`, as model names are already indicated in the dictionary keys. If a model name is explicitly specified, only that model will be executed.
  - Parameter Description

        | Parameter Name        | Type                              | Description |
        |-----------------------|-----------------------------------|-------------|
        | multi_input_tensors   | Dict[str, Dict[str, np.ndarray]]  | Multi-model inference input. The outer dictionary keys are model names; the inner dictionaries map input names to tensors. Supports running multiple models concurrently (each model may have multiple inputs). |
        | kwargs                | Variable keyword arguments        | `model_name` (`str`): Specifies the model name (can be omitted if only one model is loaded; otherwise, it must be specified). |

  - Return Value  
    Same as above.
  - Example
    ```python
    import numpy as np
    from hbm_runtime import HB_HBMRuntime

    # Map hbDNNDataType to NumPy types
    hb_dtype_map = {
        "F32": np.float32,
        "F16": np.float16,
        "S8": np.int8,
        "U8": np.uint8,
        "S16": np.int16,
        "U16": np.uint16,
        "S32": np.int32,
        "U32": np.uint32,
        "S64": np.int64,
        "U64": np.uint64,
        "BOOL8": np.bool_,
    }

    # Load multiple models
    model_files = ["/opt/hobot/model/s100/basic/lanenet256x512.hbm",
        "/opt/hobot/model/s100/basic/yolov5x_672x672_nv12.hbm"]

    model = HB_HBMRuntime(model_files)

    # Print loaded model names
    print("Loaded models:", model.model_names)

    # Construct multiple input tensors for each model
    multi_input_tensors = {}

    for model_name in model.model_names:
        model_inputs = {}

        for input_name in model.input_names[model_name]:
            shape = model.input_shapes[model_name][input_name]
            dtype_enum = model.input_dtypes[model_name][input_name]

            np_dtype = hb_dtype_map.get(dtype_enum.name, np.float32)

            model_inputs[input_name] = np.ones(shape, dtype=np_dtype)

        multi_input_tensors[model_name] = model_inputs

    # Optional: Specify inference priority and BPU devices
    priority = {name: 5 for name in model.model_names}
    bpu_cores = {name: [0] for name in model.model_names}

    model.set_scheduling_params(
        priority=priority,
        bpu_cores=bpu_cores
    )

    # Perform inference
    results = model.run(multi_input_tensors)

    # Output results
    for model_name, outputs in results.items():
        print(f"\nModel: {model_name}")
        for output_name, output_tensor in outputs.items():
            print(f"  Output: {output_name}, shape={output_tensor.shape}, dtype={output_tensor.dtype}")
    ```
- Detailed Explanation of `kwargs` Parameters
  - model_name
    - Type: `str` (model name string)
    - Description: Specifies the model to be used for inference, which must be one of the currently loaded models. In the first two `run` methods, this parameter can be omitted if only one model is loaded; if multiple models are loaded, this parameter must be specified. In the third `run` method, it can still be omitted—in which case all models provided in `multi_input_tensors` will be inferred; if specified, only the named model will be executed.
    - Example:
        ```python
        outputs = model.run(input_tensor, model_name="resnet18")
        ```
##### Configuration Functions
- set_scheduling_params
  - Function Signature
    ```python
    def set_scheduling_params(
        priority: Optional[Dict[str, int]] = None,
        bpu_cores: Optional[Dict[str, List[int]]] = None,
        custom_id: Optional[Dict[str, int]] = None,
        device_id: Optional[Dict[str, int]] = None
    ) -> None
    ```
  - Parameter Description

    | Parameter Name | Type                             | Description                                                                 |
    |----------------|----------------------------------|-----------------------------------------------------------------------------|
    | priority       | Optional dict (model name → int) | Sets scheduling priority for each model. Range is typically 0–255; higher values indicate higher priority. |
    | bpu_cores      | Optional dict (model name → List[int]) | Specifies the list of BPU core indices to bind the model to. Default indicates automatic allocation; actual behavior depends on hardware support. |
    | custom_id      | Optional dict (model name → int) | Custom priority identifier (e.g., timestamp, frame ID). Lower values indicate higher priority. Priority order: `priority` > `custom_id`. |
    | device_id      | Optional dict (model name → int) | Specifies the device ID on which the model should run.                      |

  - Return Value  
    None
  - Example:
    ```python
    # Set scheduling parameters
    model.set_scheduling_params(
        priority={"model1": 200, "model2": 100},
        bpu_cores={"model1": [0, 1], "model2": [0]}
    )

    # Verify that settings took effect
    params = runtime.sched_params
    print(params["model1"].priority)   # Output: 200
    print(params["model1"].bpu_cores)  # Output: [0, 1]
    ```
- Exception Handling
  - A `ValueError` is raised if input tensor dimensions or data types do not match the model expectations.
  - If input tensors are non-contiguous (not C-style), a contiguous copy is automatically created internally.
  - Before inference, ensure that input tensor shapes exactly match those in `input_shapes`.

#### QuantParams Class  
  Tensor quantization parameter object.

##### Attributes
- scale: `numpy.ndarray`, array of quantization scale factors  
- zero_point: `numpy.ndarray`, array of zero points  
- quant_type: `hbDNNQuantiType`, indicating the quantization mode  
- axis: `int`, quantization axis (used in per-channel quantization)

##### Example:
  ```python
    # Retrieve quantization parameters for a specific model output
    tensor_qparams = model.output_quants[model_name][output_name]
    print("scale:", tensor_qparams.scale)
    print("zero_point:", tensor_qparams.zero_point)
    print("type:", tensor_qparams.quant_type)
    print("axis:", tensor_qparams.axis)
  ```

#### SchedParam Class  
  Model scheduling parameter object, used to configure hardware-level scheduling policies (e.g., priority, core binding).

##### Attributes
- priority: `Dict[str, int]`  
  Priority settings for each model. Keys are model names; values are integer priorities (higher values = higher priority, range: 0–255).
- customId: `Dict[str, int]`  
  Custom priority identifiers (e.g., timestamp, frame ID). Lower values = higher priority. Priority order: `priority` > `customId`.
- bpu_cores: `Dict[str, List[int]]`  
  List of BPU core IDs to bind each model to. Keys are model names; values are integer lists, e.g., `[0]` or `[0, 1]` for binding to one or multiple cores.
- deviceId: `Dict[str, int]`  
  Device ID on which each model is deployed. Keys are model names; values are device IDs.

##### Example:
  ```python
  from hbm_runtime import HB_HBMRuntime, SchedParam

  # Create a scheduling parameter object
  sched = SchedParam()
  sched.priority = {"modelA": 8}
  sched.customId = {"modelA": 1001}
  sched.bpu_cores = {"modelA": [0, 1]}
  sched.deviceId = {"modelA": 0}

  # Apply scheduling parameters to the runtime
  model.set_scheduling_params(priority=sched.priority,
                              custom_id=sched.customId,
                              bpu_cores=sched.bpu_cores,
                              device_id=sched.deviceId)
  ```