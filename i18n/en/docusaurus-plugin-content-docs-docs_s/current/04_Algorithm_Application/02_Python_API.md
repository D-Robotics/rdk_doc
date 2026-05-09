---
sidebar_position: 2
id: python-api
title: Python API Manual
sidebar_label: 4.2 Python API
---
## Overview
hbm_runtime is a Python binding interface based on pybind11, used to access and operate libhbucp/libdnn C++ libraries, providing high-performance neural network model loading and inference functions.

This interface encapsulates underlying model runtime details, enabling Python users to easily load single or multiple neural network models, manage model input and output information, and flexibly execute inference operations. The interface supports multiple input data formats and ensures continuous memory storage through intelligent conversion, improving operational efficiency.

### Applicable Scenarios
- Quickly integrate and call hbm_runtime runtime functions in a Python environment.
- Applications with high requirements for inference efficiency and flexibility, such as robot vision and intelligent edge computing.
- Scenarios that need to load and manage multiple models simultaneously and dynamically configure inference priority and hardware resource allocation.

### Key Features
- Multi-Model Support
  - Supports loading a single model or a model group consisting of multiple models, with each model independently obtaining input/output metadata and performing inference.
- Flexible Input Formats
  - Supports single input (numpy.ndarray);
  - Supports model name mapped input dictionary (Dict[str, np.ndarray]);
  - Supports multi-model multi-input structure (Dict[str, Dict[str, np.ndarray]]); all inputs are automatically checked for C-contiguous memory format, with copies made if necessary to ensure efficient and correct access by the underlying layers.
- Specified Inference Priority (priority)
  - Allows explicit specification of scheduling priority for model tasks via the priority: Dict[str, int] parameter, supporting the scheduler in reasonably scheduling inference tasks under limited hardware resources.
- Specified Inference BPU Core (bpu_cores)
  - Supports explicit specification of BPU calculation cores used during model inference via bpu_cores: Dict[str, List[int]], enabling strategies such as heterogeneous core resource binding.
- Parallel Multi-Model Inference
  - For multi-model input scenarios, the underlying layer automatically uses a multi-threading mechanism to execute each model's inference task in parallel (multi-threaded launch), achieving higher throughput on multi-core BPU systems (execution remains serial on single-core BPU).
- Metadata Access Interface
  - Number, name, and data type (hbDNNDataType enum) of inputs and outputs;
  - Shape, memory stride, and quantization parameters (including scale, zero point, quantization type) of input/output tensors;
  - Model description information, HBM file description information, etc.
- Fully Bound Type System
  - Supports QuantParams quantization parameter structure, hbDNNDataType data type enum, SchedParam model scheduling parameter object, and hbDNNQuantiType quantization type enum, providing type-safe attribute access.

## Installation
The hbm_runtime module is a high-performance inference runtime Python interface implemented in C++, depending on pybind11 and underlying inference libraries provided by Horizon (such as libdnn, libhbucp, etc.). Installation is supported via system DEB packages (.deb) and is suitable for Python 3.10 and above.

### System Dependencies
| Dependency | Minimum Version | Description |
| :--- | :--- | :--- |
| Python | ≥ 3.10 | Python 3.10 is recommended |
| pip | ≥ 22.0 | Required for installing wheel packages |
| pybind11 | Any | Used during build, not required as a runtime dependency |
| scikit-build-core | ≥ 0.7 | Used for building wheel packages (source build only) |
| Horizon Base Libraries | Platform-dependent | Such as libdnn.so, libucp.so, etc., usually provided by the BSP |

### Building Wheel Packages
There are three ways to build wheel packages, introduced below.

#### Build during DEB installation
The wheel build for hbm_runtime is added during the installation process of the hobot-dnn package. After the DEB package is installed, the whl package for hbm-runtime will be generated.
  ```bash
  # Install via repository
  sudo apt-get install hobot-dnn

  # Install via local DEB package (note the actual package name based on the build time)
  dpkg -i hobot-dnn_4.0.4-20250909195426_arm64.deb

  # After installation, the wheel package can be found in the /tmp directory on the board
  ls /tmp

  # Note: The whl package name varies by version, where xxx represents the version
  # hbm_runtime-x.x.x-cp310-cp310-manylinux_2_34_aarch64.whl
  ```

#### Build during system software compilation
During the image compilation of system software, the hobot-dnn DEB is installed. In this process, the hbm-runtime whl package is built and transferred to the `out/product/deb_packages` directory.
  ```bash
  sudo ./pack_image.sh

  ls out/product/deb_packages

  # Note: The whl package name varies by version, where xxx represents the version
  # hbm_runtime-x.x.x-cp310-cp310-manylinux_2_34_aarch64.whl
  ```

#### Build on the edge device
  ```bash
  # Enter the hbm_runtime source code library
  cd /usr/hobot/lib/hbm_runtime

  # Run the build command
  ./build.sh

  # Check the built wheel package
  ls dist/

  # Note: The whl package name varies by version, where xxx represents the version
  # hbm_runtime-x.x.x-cp310-cp310-manylinux_2_34_aarch64.whl
  ```

### Installation Methods

#### Using Wheel Packages
There are two ways to install via wheel; choose one.
- Install via local wheel package
  - Find the whl file built in the "Building Wheel Packages" section.

  ```bash
  # Example: Install a local whl package using pip (note the version in the name)
  pip install hbm_runtime-x.x.x-cp310-cp310-manylinux_2_34_aarch64.whl
  ```

- Install from pypi source
  ```bash
  pip install hbm_runtime
  ```

#### Using .deb Packages
There are two ways to install via DEB; choose one.
- Install via local DEB package
  ```bash
  # Example: Install a DEB package (note the actual package name)
  sudo dpkg -i hobot-dnn_4.0.2-20250714201215_arm64.deb
  ```

- Install via apt repository

  ```bash
  sudo apt-get install hobot-dnn
  ```

- FAQ
  - If the files do not take effect after .deb installation, check if other dependencies are preventing overwriting (e.g., an existing old version of hobot-spdev).
  - Use dpkg -L hobot-dnn to check if files were successfully deployed.



### Uninstallation
- Uninstall pip-installed packages:
  ```bash
  pip uninstall hbmruntime
  ```

- Uninstall .deb-installed packages:
  ```bash
  sudo apt remove hobot-dnn
  ```

## Quick Start
This section describes how to use hbm_runtime for model loading and inference. With just a few lines of code, you can run a model and obtain output results.

### Environment Preparation
Please ensure that HBMRuntime is correctly installed (see [Installation](#installation)) and that the hbm model file is available.

### Examples

#### Single-Model Single-Input Inference
Suitable for cases where the model has only one input tensor.
```python
import numpy as np
from hbm_runtime import HB_HBMRuntime

# Load model
model = HB_HBMRuntime("/opt/hobot/model/s100/basic/lanenet256x512.hbm")

# Get model and input names
model_name = model.model_names[0]
input_name = model.input_names[model_name][0]  # Assuming single input

# Get the corresponding shape for this input
input_shape = model.input_shapes[model_name][input_name]

# Construct numpy input
input_tensor = np.ones(input_shape, dtype=np.float32)

# Run inference
outputs = model.run(input_tensor)

# Get output results
output_array = outputs[model_name]
print("Output:", output_array)
```

#### Single-Model Multi-Input Inference
Suitable for cases where the model has multiple input tensors.
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

# Get model name (assuming only one model loaded)
model_name = model.model_names[0]

# Prepare input names and shapes
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

# Run inference, optionally specifying priority and BPU cores
results = model.run(input_tensors)

# Output results
for output_name, output_data in results[model_name].items():
    print(f"Output: {output_name}, shape={output_data.shape}")

```

### FAQ
| Question | Description |
| :--- | :--- |
| How to get the model name? | Use `model.model_names` to see the list of loaded model names. |
| How to confirm input dimensions/types? | Use `model.input_shapes` and `model.input_dtypes`. |
| How to confirm BPU core allocation? | Use the `bpu_cores` parameter to specify [0, 1, 2, 3], depending on hardware support. |

For more complex usage (multi-input models, reading quantization parameters, etc.), please refer to the [API Section](#api-reference).

## API Reference
The Python module hbm_runtime is a Horizon HBM model inference interface encapsulated via PyBind11, based on the underlying libdnn and libhbucp. It provides unified model loading, input/output information query, and inference execution functions, supporting multi-model loading, multi-input inference, specified inference models, BPU cores, inference task priority, etc.

### Enums
#### hbDNNDataType
##### Tensor Data Type Enum:
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
##### Tensor Quantization Type Enum:
- NONE: Non-quantized type
- SCALE: Linear scale quantization (scale + zero_point)
##### Example
```python
from hbm_runtime import hbDNNQuantiType
print(hbDNNQuantiType.SCALE)  # Output: hbDNNQuantiType.SCALE
```

### Class Documentation
#### HB_HBMRuntime
Model runtime class, loads one or more HBM model files, and provides an inference execution interface.
##### Constructor
- Signature
    ```python
    HB_HBMRuntime(model_file: str)
    HB_HBMRuntime(model_files: List[str])
    ```
- Parameters

    | Parameter Name | Type | Description |
    | :--- | :--- | :--- |
    | model_file | str | Path to HBM model file |
    | model_files | List[str] | Paths to multiple HBM model files (for multi-model) |
- Return Value
  Class object
- Example
    ```python
    from hbm_runtime import HB_HBMRuntime

    model = HB_HBMRuntime("model.hbm")
    # Or load multiple models:
    model = HB_HBMRuntime(["model1.hbm", "model2.hbm"])
    ```

##### Attributes
All attributes below are read-only.
- version: str
  - Description:
    - Get the library version number.
  - Structure:
    - str: Version number string.
    - Example:
print("Version:", HB_HBMRuntime.version)
- model_names: List[str]
  - Description:
    - List of loaded model names.
  - Structure:
    - List[str]: List of model names
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
    - Description of each model (from model-embedded notes).
  - Structure:
    - Dict[str, str]: Key is the model name, value is the overall model description, usually from the compiler.
  - Example:
    ```python
    # Print all model descriptions
    print(model.model_descs)
    # Output: {'yolov5x_672x672_nv12': 'Image classification model based on ResNet-18.'}
    ```

- hbm_descs: Dict[str, str]
  - Description:
    - Note information in each HBM file.
  - Structure:
    - Dict[str, int]: Key is the .hbm file name (e.g., "resnet18"), value is the comment or metadata string in the HBM file.
  - Example:
    ```python
    # Print all HBM file descriptions
    print(model.hbm_descs)
    # Output: {'/opt/hobot/model/s100/basic/yolov5x_672x672_nv12.hbm': 'xxx'}
    ```
- input_counts: Dict[str, int]
  - Description:
    - Number of input tensors for each model.
  - Structure:
    - Dict[str, int]: Key is the model name, value is the number of input tensors for that model.
  - Example:
    ```python
    # Print input counts for all models
    print(model.input_counts)
    # Output: {'yolov5x_672x672_nv12': 2}
    ```
- input_names: Dict[str, List[str]]
  - Description:
    - List of input tensor names for each model.
  - Structure:
    - Outer Dict[str, ...]: Key is model name.
    - Inner List[str]: List of names for all input tensors of that model.
  - Example:
    ```python
    print(model.input_names)
    # Output: {'yolov5x_672x672_nv12': ['data_y', 'data_uv']}
    ```
- input_descs: Dict[str, Dict[str, str]]
  - Description:
    - Description for each input tensor.
  - Structure:
    - Outer Dict[str, ...]: Model name.
    - Inner Dict[str, str]: Key is input tensor name, value is description.
  - Example:
    ```python
    # Print descriptions for all input tensors
    print(model.input_descs)
    # Output: {'yolov5x_672x672_nv12': {'data_uv': 'xxx', 'data_y': 'xxx'}}
    ```
- input_shapes: Dict[str, Dict[str, List[int]]]
  - Description:
    - Shape for each input tensor.
  - Structure:
    - Outer Dict[str, ...]: Model name.
    - Inner Dict[str, List[int]]: Key is input name, value is input tensor dimensions (shape).
  - Example:
    ```python
    model.input_shapes
    # Output: {'yolov5x_672x672_nv12': {'data_uv': [1, 336, 336, 2], 'data_y': [1, 672, 672, 1]}}
    ```
- input_dtypes: Dict[str, Dict[str, hbDNNDataType]]
  - Description:
    - Data type for each input tensor.
  - Structure:
    - Outer Dict[str, ...]: Model name.
    - Inner Dict[str, hbDNNDataType]: Key is input tensor name, value is data type (e.g., F32, U8).
  - Example:
    ```python
    print(model.input_dtypes)
    # Output: {'yolov5x_672x672_nv12': {'data_uv': <hbDNNDataType.U8: 3>, 'data_y': <hbDNNDataType.U8: 3>}}
    ```

- input_quants: Dict[str, Dict[str, QuantParams]]
  - Description:
    - Provides quantization parameter information for all input tensors of each model. Used to support preprocessing calculations for quantized models or to understand tensor quantization methods.
  - Structure:
    - Outer Dict[str, ...]: Key is model name, e.g., "resnet50";
    - Inner Dict[str, QuantParams]: Key is input tensor name, value is QuantParams instance;
    - QuantParams attributes:
      - scale: np.ndarray — Quantization scale factor, usually a float array;
      - zero_point: np.ndarray — Zero point, for symmetric/asymmetric quantization offset;
      - quant_type: hbDNNQuantiType — Quantization type enum value (e.g., SCALE, NONE);
      - axis: int — If channel-quantized, this field indicates on which axis.
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
  - Description:
    - Stride information for each input tensor.
  - Structure:
    - Outer Dict[str, ...]: Model name.
    - Inner Dict[str, List[int]]: Key is input name, value is input tensor stride.
  - Example:
    ```python
    print(model.input_strides)
    # Output: {'yolov5x_672x672_nv12': {'data_uv': [-1, -1, 2, 1], 'data_y': [-1, -1, 1, 1]}}
    ```
    Note: For detailed meaning of stride, refer to the description of the libdnn library in the [OE Documentation](http://j6.doc.oe.hobot.cc/3.0.31/guide/ucp/runtime/bpu_sdk_api/data_structure/hbDNNTensorProperties.html).
- output_counts: Dict[str, int]
  - Description:
    - Number of output tensors for each model.
  - Structure:
    - Dict[str, int]: Key is the model name, value is the number of output tensors for that model.
  - Example:
    ```python
    print(model.output_counts)
    # Output: {'yolov5x_672x672_nv12': 3}
    ```
- output_names: Dict[str, List[str]]
  - Description:
    - List of output tensor names for each model.
  - Structure:
    - Outer Dict[str, ...]: Key is model name.
    - Inner List[str]: List of names for all output tensors of that model.
  - Example:
    ```python
    print(model.output_names)
    # Output: {'yolov5x_672x672_nv12': ['output', '1310', '1312']}
    ```
- output_descs: Dict[str, Dict[str, str]]
  - Description:
    - Description for each output tensor.
  - Structure:
    - Outer Dict[str, ...]: Model name.
    - Inner Dict[str, str]: Key is output tensor name, value is description.
  - Example:
    ```python
    print(model.output_descs)
    # Output: {'yolov5x_672x672_nv12': {'1310': 'xxx', '1312': 'xxx', 'output': 'xxx'}}
    ```
- output_shapes: Dict[str, Dict[str, List[int]]]
  - Description:
    - Shape for each output tensor.
  - Structure:
    - Outer Dict[str, ...]: Model name.
    - Inner Dict[str, List[int]]: Key is output name, value is output tensor dimensions (shape).
  - Example:
    ```python
    print(model.output_shapes)
    # Output: {'yolov5x_672x672_nv12': {'1310': [1, 42, 42, 255], '1312': [1, 21, 21, 255], 'output': [1, 84, 84, 255]}}
    ```
- output_dtypes: Dict[str, Dict[str, List[int]]]
  - Description:
    - Data type for each output tensor.
  - Structure:
    - Outer Dict[str, ...]: Model name.
    - Inner Dict[str, hbDNNDataType]: Key is output tensor name, value is data type (e.g., F32, U8).
  - Example:
    ```python
    print(model.output_dtypes)
    # Output: {'yolov5x_672x672_nv12': {'1310': <hbDNNDataType.S32: 8>, '1312': <hbDNNDataType.S32: 8>, 'output': <hbDNNDataType.S32: 8>}}
    ```
- output_quants: Dict[str, Dict[str, QuantParams]]
  - Description:
    - Provides quantization parameter information for all output tensors of each model. Used to support post-processing calculations for quantized models (such as converting int8 data back to float32) or to understand tensor quantization methods (scale-based, etc.).
  - Structure:
    - Outer Dict[str, ...]: Key is model name, e.g., "resnet50";
    - Inner Dict[str, QuantParams]: Key is output tensor name, value is QuantParams instance;
    - QuantParams attributes:
      - scale: np.ndarray — Quantization scale factor, usually a float array;
      - zero_point: np.ndarray — Zero point, for symmetric/asymmetric quantization offset;
      - quant_type: hbDNNQuantiType — Quantization type enum value (e.g., SCALE, NONE);
      - axis: int — If channel-quantized, this field indicates on which axis.
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
  - Description:
    - Stride information for each output tensor.
  - Structure:
    - Outer Dict[str, ...]: Model name.
    - Inner Dict[str, List[int]]: Key is output name, value is output tensor stride.
  - Example:
    ```python
    print(model.output_strides)
    # Output: {'yolov5x_672x672_nv12': {'1310': [1806336, 43008, 1024, 4], '1312': [451584, 21504, 1024, 4], 'output': [7225344, 86016, 1024, 4]}}
    ```
  Note: For detailed meaning of stride, refer to the description of the libdnn library in the [OE Documentation](http://j6.doc.oe.hobot.cc/3.0.31/guide/ucp/runtime/bpu_sdk_api/data_structure/hbDNNTensorProperties.html).

- sched_params: Dict[str, SchedParam]
  - Description:
    sched_params is used to obtain the current scheduling parameters (Scheduling Parameters) for all models, including each model's:
    - Priority (priority)
    - Custom ID (customId)
    - Allocated BPU cores (bpu_cores)
    - Device ID (deviceId)
    These scheduling parameters affect how the model runs on hardware, which is especially important in multi-model deployment or on multi-core devices.
  - Structure:
    - Outer Dict[str, ...]: Model name.
    - Inner SchedParam: Instance of SchedParam class, containing scheduling parameters priority, customId, bpu_cores, and deviceId for that model;
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
    Note: bpu_cores returning -1 indicates automatic allocation by the scheduler.

##### Inference Functions
- run(input_tensor, **kwargs)
  - Signature
    ```python
    run(input_tensor: np.ndarray, **kwargs) -> Dict[str, Dict[str, np.ndarray]]
    ```
  - Description
    Suitable for single-model, single-input inference. The input is a NumPy array corresponding to the model's only input tensor. If only one model is loaded, the model name can be omitted.
  - Parameters

    | Parameter Name | Type | Description |
    | :--- | :--- | :--- |
    | input_tensor | np.ndarray | Single input tensor, only for single-model and single-input inference scenarios. Tensor shape must match the model's corresponding input. |
    | kwargs | Variable keyword arguments | `model_name` (`str`): Specifies the model name (can be omitted if there is only one model, otherwise must be specified). |

  - Return Value
    - Type: Dict[str, Dict[str, np.ndarray]]
    - Outer dictionary key: Model name
    - Inner dictionary key: Output tensor name
    - Value: Corresponding NumPy output array
  - Example: Refer to the Single-Model Single-Input Inference part of the Quick Start section.
- run(input_tensors: Dict[str, np.ndarray], **kwargs)
  - Signature
    ```python
    run(input_tensors: Dict[str, np.ndarray], **kwargs) -> Dict[str, Dict[str, np.ndarray]]
    ```
  - Description
    Suitable for single-model, multi-input inference. Each input tensor is specified by an input name, consistent with the model definition. If only one model is loaded, the model name can be omitted.
  - Parameters

    | Parameter Name | Type | Description |
    | :--- | :--- | :--- |
    | input_tensors | Dict[str, np.ndarray] | Multi-input tensors, only for single-model multi-input inference scenarios. Keys are input tensor names, values are corresponding NumPy arrays. |
    | kwargs | Variable keyword arguments | `model_name` (`str`): Specifies the model name (can be omitted if there is only one model, otherwise must be specified). |

  - Return Value
    Same as above.
  - Example: Refer to the Single-Model Multi-Input Inference part of the Quick Start section.
- run(multi_input_tensors: Dict[str, Dict[str, np.ndarray]], **kwargs)
  - Signature
    ```python
    run(multi_input_tensors: Dict[str, Dict[str, np.ndarray]], **kwargs) -> Dict[str, Dict[str, np.ndarray]]
    ```
  - Description
    Suitable for simultaneous inference of multiple models, where each model provides its own multiple input tensors. No need to specify model_name as it's reflected in the keys. If model_name is specified, only the specified model will be inferred.
  - Parameters

        | Parameter Name | Type | Description |
        | :--- | :--- | :--- |
        | multi_input_tensors | Dict[str, Dict[str, np.ndarray]] | Multi-model inference, outer keys are model names, inner keys are input name → tensor mappings. Supports running multiple models (each can have multiple inputs) simultaneously. |
        | kwargs | Variable keyword arguments | `model_name (str)`: Specifies the model name (can be omitted if there is only one model, otherwise must be specified). |

  - Return Value
    Same as above.
  - Example
    ```python
    import numpy as np
    from hbm_runtime import HB_HBMRuntime

    # Map hbDNNDataType to numpy types
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

    # Print names of loaded models
    print("Loaded models:", model.model_names)

    # Construct multi-input tensors for each model
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

    # Run inference
    results = model.run(multi_input_tensors)

    # Output results
    for model_name, outputs in results.items():
        print(f"\nModel: {model_name}")
        for output_name, output_tensor in outputs.items():
            print(f"  Output: {output_name}, shape={output_tensor.shape}, dtype={output_tensor.dtype}")
    ```
- Detailed Description of kwargs Parameters
  - model_name
    - Type: str model string
    - Description: Specifies the model used for current inference, which must be one of the currently loaded models. In the first two run methods, this parameter can be omitted if only one model is loaded; if multiple models are loaded, this parameter must be specified. In the third run method, it can still be omitted; if omitted, all models provided in multi_input_tensors will be inferred; if specified, only the specified model will be inferred.
    - Example:
        ```python
        outputs = model.run(input_tensor, model_name="resnet18")
        ```

##### Configuration Functions
- set_scheduling_params
  - Signature
    ```python
    def set_scheduling_params(
        priority: Optional[Dict[str, int]] = None,
        bpu_cores: Optional[Dict[str, List[int]]] = None,
        custom_id: Optional[Dict[str, int]] = None,
        device_id: Optional[Dict[str, int]] = None
    ) -> None
    ```
  - Parameters

    | Parameter Name | Type | Description |
    | :--- | :--- | :--- |
    | priority | Optional dictionary (model name -> int) | Set scheduling priority for each model, typically in the range 0-255, with higher values representing higher priority. |
    | bpu_cores | Optional dictionary (model name -> List[int]) | Specify the list of BPU core indices bound to the model. Default means automatic allocation, depending on hardware support. |
    | custom_id | Optional dictionary (model name -> int) | Custom priority (e.g., timestamp, frame id), where smaller values represent higher priority. Priority order: priority > customId. |
    | device_id | Optional dictionary (model name -> int) | Specify which device the model runs on. |

  - Return Value
    None
  - Example:
    ```python
    # Set scheduling parameters
    model.set_scheduling_params(
        priority={"model1": 200, "model2": 100},
        bpu_cores={"model1": [0, 1], "model2": [0]}
    )

    # Verify if settings took effect
    params = runtime.sched_params
    print(params["model1"].priority)   # Output: 200
    print(params["model1"].bpu_cores)  # Output: [0, 1]
    ```
- Exception Description
  - A ValueError is raised if the input tensor dimensions or types do not match the model.
  - If the input tensor is not contiguous (non-C-style), a contiguous copy is automatically created internally.
  - Ensure the input tensor shape exactly matches input_shapes before inference.

#### QuantParams Class
Tensor quantization parameter object.
##### Attributes
- scale: numpy.ndarray, quantization scale factor array
- zero_point: numpy.ndarray, zero point array
- quant_type: hbDNNQuantiType, quantization mode
- axis: int, quantization axis (for per-channel quantization)
##### Example:
    ```python
    # Get quantization parameters for a certain model output
    tensor_qparams = model.output_quants[model_name][output_name]
    print("scale:", tensor_qparams.scale)
    print("zero_point:", tensor_qparams.zero_point)
    print("type:", tensor_qparams.quant_type)
    print("axis:", tensor_qparams.axis)
    ```

#### SchedParam Class
Model scheduling parameter object, used to configure model execution scheduling policies on hardware (priority, core binding, etc.).
##### Attributes
- priority: Dict[str, int]
  Priority settings for each model. Key is model name, value is priority integer (larger values indicate higher priority, range 0-255).
- customId: Dict[str, int]
  Custom priority (e.g., timestamp, frame id), where smaller values indicate higher priority. Priority order: priority > customId.
- bpu_cores: Dict[str, List[int]]
  List of BPU core indices bound to the model. Key is model name, value is a list of integers, e.g., [0], [0, 1].
- deviceId: Dict[str, int]
  Specifies the device ID for model deployment. Key is model name, value is device ID.
##### Example:
  ```python
  from hbm_runtime import HB_HBMRuntime, SchedParam

  # Create a scheduling parameter object
  sched = SchedParam()
  sched.priority = {"modelA": 8}
  sched.customId = {"modelA": 1001}
  sched.bpu_cores = {"modelA": [0, 1]}
  sched.deviceId = {"modelA": 0}

  # Apply scheduling parameters to runtime
  model.set_scheduling_params(priority=sched.priority,
                              custom_id=sched.customId,
                              bpu_cores=sched.bpu_cores,
                              device_id=sched.deviceId)
  ```
