---
sidebar_position: 1
id: python-api
title: Python 接口手册
sidebar_label: 4.1 Python 接口
---
## 简介（Overview）
hbm_runtime是基于pybind11的Python绑定接口，用于访问和操作libhbucp/libdnn C++ 库，提供高性能的神经网络模型加载和推理功能。

该接口封装了底层模型运行时细节，使 Python 用户能够方便地加载单个或多个神经网络模型，管理模型的输入输出信息，并灵活执行推理操作。接口支持多种输入数据格式，且通过智能转换保证输入数据连续存储，提升运行效率。

### 适用场景
- 在 Python 环境中快速集成和调用 hbm_runtime运行时功能。
- 机器人视觉、智能边缘计算等对推理效率和灵活性有较高要求的应用。
- 需要同时加载和管理多个模型，动态配置推理优先级及硬件资源分配的场景。

### 关键特性
- 多模型支持
  - 支持加载单个模型或多个模型组成的模型组，每个模型均可独立获取输入输出元信息并进行推理。
- 灵活输入格式
  - 支持单输入（numpy.ndarray）；
  - 支持模型名映射的输入字典（Dict[str, np.ndarray]）；
  - 支持多模型多输入结构（Dict[str, Dict[str, np.ndarray]]）；所有输入均自动检查是否为 C 连续内存格式，必要时进行拷贝，保证底层高效正确访问。
- 指定推理优先级（priority）
  - 可通过 priority: Dict[str, int] 参数显式指定模型任务的调度优先级，支持调度器在有限硬件资源下合理调度推理任务。
- 指定推理 BPU 核心（bpu_cores）
  - 支持通过 bpu_cores: Dict[str, List[int]] 显式指定模型推理时使用的 BPU 计算核心，实现异构核资源绑定等策略。
- 多模型并行推理
  - 对于多模型输入场景，底层自动采用多线程机制并行执行每个模型的推理任务（multi-threaded launch），在多核 BPU 系统上可获得更高吞吐（单核BPU底层还是串行执行）。
- 元信息访问接口
  - 输入输出数量、名称、数据类型（hbDNNDataType 枚举）；
  - 输入输出张量形状、内存 stride、量化参数（包括 scale、zero point、量化类型）；
  - 模型描述信息、HBM文件描述信息等。
- 完整绑定的类型系统
  - 支持量化参数结构 QuantParams，数据类型枚举 hbDNNDataType，模型调度参数对象SchedParam  和量化类型枚举 hbDNNQuantiType，提供类型安全的属性访问。

## 安装说明（Installation）
本模块 hbm_runtime 是基于 C++ 实现的高性能推理运行时 Python 接口，依赖 pybind11 和地平线提供的底层推理库（如 libdnn, libhbucp 等）。支持通过系统 DEB 包（.deb） 的方式进行安装，适用于 Python 3.10 及以上版本。
### 系统依赖
| 依赖项       | 最低版本  | 说明                                                   |
|------------|-----------|--------------------------------------------------------|
| Python     | ≥ 3.10    | 推荐使用 Python 3.10                                   |
| pip        | ≥ 22.0    | 安装 wheel 包所需                                      |
| pybind11   | 任意      | 构建时使用，安装包时不需要依赖                         |
| scikit-build-core | ≥ 0.7 | 构建 wheel 包时使用（仅源码构建）                    |
| 地平线基础库 | 根据平台 | 如 libdnn.so、libucp.so 等，通常由 BSP 提供           |

### 安装方式
- 安装 hobot-dnn 包
    ```python
    sudo apt update
    sudo apt install hobot-dnn
    ```

### 卸载说明
- 卸载安装的包：
    ```python
    sudo apt remove hobot-dnn
    ```

## 快速开始（Quick Start）
  本节介绍如何使用hbm_runtime进行模型加载和推理。只需几行代码，即可运行模型并获取输出结果。
### 环境准备
  请确保已正确安装 HBMRuntime（详见[安装说明](#安装说明installation)），并已具备模型文件 hbm 模型。
### 示例
#### 单模型单输入推理
  适用于模型只有一个输入张量的情况。
```python
import numpy as np
from hbm_runtime import HB_HBMRuntime

# 加载模型
model = HB_HBMRuntime("/opt/hobot/model/s100/basic/lanenet256x512.hbm")

# 获取模型名与输入名
model_name = model.model_names[0]
input_name = model.input_names[model_name][0]  # 假设模型只有一个输入

# 获取该输入对应的 shape
input_shape = model.input_shapes[model_name][input_name]

# 构造 numpy 输入
input_tensor = np.ones(input_shape, dtype=np.float32)

# 执行推理
outputs = model.run(input_tensor)

# 获取输出结果
output_array = outputs[model_name]
print("Output:", output_array)
```
#### 单模型多输入推理
    适用于模型有多个输入张量的情况。
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

# 加载模型
model = HB_HBMRuntime("/opt/hobot/model/s100/basic/yolov5x_672x672_nv12.hbm")

# 获取模型名（假设只加载了一个模型）
model_name = model.model_names[0]

# 准备输入名和 shape
input_names = model.input_names[model_name]
input_shapes = model.input_shapes[model_name]
input_dtypes = model.input_dtypes[model_name]

# 构造输入字典
input_tensors = {}
for name in input_names:
    shape = input_shapes[name]
    np_dtype = hb_dtype_map.get(input_dtypes[name].name, np.float32)  # fallback
    input_tensors[name] = np.ones(shape, dtype=np_dtype)

# 可选：指定推理优先级和 BPU 设备
priority = {model_name: 5}
bpu_cores = {model_name: [0]}

model.set_scheduling_params(
    priority=priority,
    bpu_cores=bpu_cores
)

# 执行推理，可选指定优先级和 BPU 核
results = model.run(input_tensors)

# 输出结果
for output_name, output_data in results[model_name].items():
    print(f"Output: {output_name}, shape={output_data.shape}")

```
### 常见问题
| 问题                     | 说明                                                       |
|------------------------|------------------------------------------------------------|
| 模型名称如何获取？       | 可通过 `model.model_names` 查看加载的模型名列表。          |
| 输入维度/类型如何确认？  | 使用 `model.input_shapes`、`model.input_dtypes`。           |
| 如何确认 BPU 核心分配？  | 使用 bpu_cores 参数指定 [0, 1, 2, 3]，具体需看硬件的支持情况。|

  如需更复杂用法（多输入模型、量化参数读取等），请参考[API部分](#模块类函数说明api-reference)。

## 模块/类/函数说明（API Reference）
Python 模块 hbm_runtime 是通过 PyBind11 封装的地平线 HBM 模型推理接口，基于底层 libdnn 和 libhbucp 实现。提供统一封装的模型加载、输入输出信息查询、推理执行等功能，支持多模型加载、多输入推理、指定推理模型，BPU Core、推理任务优先级等。

### 枚举类型
#### hbDNNDataType
##### 张量数据类型枚举：
- S4：4-bit signed
- U4：4-bit unsigned
- S8：8-bit signed
- U8：8-bit unsigned
- F16：16-bit float
- S16：16-bit signed
- U16：16-bit unsigned
- F32：32-bit float
- S32：32-bit signed
- U32：32-bit unsigned
- F64：64-bit float
- S64：64-bit signed
- U64：64-bit unsigned
- BOOL8：8-bit bool 类型
- MAX：最大值（保留字段）

##### 示例
```python
from hbm_runtime import hbDNNDataType
print(hbDNNDataType.F32)  # 输出: hbDNNDataType.F32
```
#### hbDNNQuantiType
##### 张量量化类型枚举：
- NONE：非量化类型
- SCALE：线性缩放量化（scale + zero_point）
##### 示例
```python
from hbm_runtime import hbDNNQuantiType
print(hbDNNQuantiType.SCALE)  # 输出: hbDNNQuantiType.SCALE
```

### 类说明
#### HB_HBMRuntime
模型运行时类，加载一个或多个 HBM 模型文件，并提供推理执行接口。
##### 构造函数
- 函数签名
    ```python
    HB_HBMRuntime(model_file: str)
    HB_HBMRuntime(model_files: List[str])
    ```
- 参数说明

    | 参数名       | 类型         | 说明                                 |
    |------------|--------------|--------------------------------------|
    | model_file | str          | HBM 模型文件路径                     |
    | model_files| List[str]    | 多个 HBM 模型文件路径（用于多模型） |
- 返回值

  类对象
- 示例
    ```python
    from hbm_runtime import HB_HBMRuntime

    model = HB_HBMRuntime("model.hbm")
    # 或加载多个模型：
    model = HB_HBMRuntime(["model1.hbm", "model2.hbm"])
    ```

##### 属性说明
以下所有属性均为只读。
- version: str
  - 功能说明：
    - 获取库版本号。
  - 结构说明：
    - str：版本号字符串。
    - 示例：
print("版本:", HB_HBMRuntime.version)
- model_names: List[str]
  - 功能说明：
    - 加载的模型名称列表。
  - 结构说明：
    - List[str]: 模型名称列表
  - 示例：
    ```python
    print(model.model_names)
    # 输出：['model_1', 'model_2']
    ```
- model_count: int
  - 功能说明：
    - 加载的模型数量。
  - 结构说明：
    - int :加载的模型数量。
  - 示例：
    ```python
    print(model.model_count)
    # 输出：2
    ```
- model_descs: Dict[str, str]
  - 功能说明：
    - 每个模型的描述信息（来自模型内嵌的备注）。
  - 结构说明：
    - Dict[str, str]: 键为模型名称，值为模型的整体描述信息，通常来自编译器。
  - 示例：
    ```python
    #打印所有模型的描述信息
    print(model.model_descs)
    # 输出：{'yolov5x_672x672_nv12': 'Image classification model based on ResNet-18.'}
    ```

- hbm_descs: Dict[str, str]
  - 功能说明：
    - 每个 HBM 文件中的备注信息。
  - 结构说明：
    - Dict[str, int]：键为.hbm 文件名（例如 "resnet18"），值为HBM 文件中的注释或元信息字符串。
  - 示例：
    ```python
    #打印所有模型文件的描述信息
    print(model.hbm_descs)
    # 输出：{'/opt/hobot/model/s100/basic/yolov5x_672x672_nv12.hbm': 'xxx'}
    ```
- input_counts: Dict[str, int]
  - 功能说明：
    - 每个模型输入张量数量。
  - 结构说明：
    - Dict[str, int]：键为模型名称，值为该模型的输入张量个数。
  - 示例：
    ```python
    #打印所有模型文件的描述信息
    print(model.input_counts)
    # 输出：{'yolov5x_672x672_nv12': 2}
    ```
- input_names: Dict[str, List[str]]
  - 功能说明：
    - 每个模型的输入张量名称列表。
  - 结构说明：
    - 外层 Dict[str, ...]：键为模型名称。
    - 内层 List[str]：为该模型所有输入张量的名称列表。
  - 示例：
    ```python
    print(model.input_names)
    # 输出：{'yolov5x_672x672_nv12': ['data_y', 'data_uv']}
    ```
- input_descs: Dict[str, Dict[str, str]]
  - 功能说明：
    - 每个输入张量的描述
  - 结构说明：
    - 外层 Dict[str, ...]：模型名称。
    - 内层 Dict[str, str]：键为输入张量名称，值为描述信息。
  - 示例：
    ```python
    #打印所有模型文件的描述信息
    print(model.input_descs)
    # 输出：{'yolov5x_672x672_nv12': {'data_uv': 'xxx', 'data_y': 'xxx'}}
    ```
- input_shapes: Dict[str, Dict[str, List[int]]]
  - 功能说明：
    -  每个输入张量的 shape
  - 结构说明：
    - 外层 Dict[str, ...]：模型名称。
    - 内层 Dict[str, List[int]]：键为输入名，值为输入张量的维度（形状）。
  - 示例：
    ```python
    model.input_shapes
    # 输出：{'yolov5x_672x672_nv12': {'data_uv': [1, 336, 336, 2], 'data_y': [1, 672, 672, 1]}}
    ```
- input_dtypes: Dict[str, Dict[str, hbDNNDataType]]
  - 功能说明：
    -  每个输入张量的数据类型
  - 结构说明：
    - 外层 Dict[str, ...]：模型名称。
    - 内层 Dict[str, hbDNNDataType]：键为输入张量名，值为数据类型（例如 F32、U8）。
  - 示例：
    ```python
    print(model.input_dtypes)
    # 输出：{'yolov5x_672x672_nv12': {'data_uv': <hbDNNDataType.U8: 3>, 'data_y': <hbDNNDataType.U8: 3>}}
    ```

- input_quants: Dict[str, Dict[str, QuantParams]]
  - 功能说明：
    - 提供每个模型所有输入张量的量化参数信息。用于支持量化模型的前处理计算，或者了解张量的量化方式。
  - 结构说明：
    - 外层 Dict[str, ...]：键是模型名称（model name），例如 "resnet50";
    - 内层 Dict[str, QuantParams]：键是输入张量名，值为 QuantParams 实例；
    - QuantParams 类属性：
      - scale: np.ndarray — 量化比例因子，通常为浮点数数组；
      - zero_point: np.ndarray — 零点，用于对称/非对称量化偏移；
      - quant_type: hbDNNQuantiType — 量化类型枚举值（如 SCALE、NONE）；
      - axis: int — 如果是通道量化，则该字段表示在哪个轴上量化。
  - 示例：
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
  - 功能说明：
    - 每个输入张量的 stride 步长信息
  - 结构说明：
    - 外层 Dict[str, ...]：模型名称。
    - 内层Dict[str, List[int]]：键为输入名，值为输入张量的stride信息。
  - 示例：
    ```python
    print(model.input_strides)
    # 输出：{'yolov5x_672x672_nv12': {'data_uv': [-1, -1, 2, 1], 'data_y': [-1, -1, 1, 1]}}
    ```
    注意：stride的详细含义可参考[OE文档](http://j6.doc.oe.hobot.cc/3.0.31/guide/ucp/runtime/bpu_sdk_api/data_structure/hbDNNTensorProperties.html)中libdnn库中的描述；
- output_counts: Dict[str, int]
  - 功能说明：
    - 每个模型输出张量数量。
  - 结构说明：
    - Dict[str, int]：键为模型名称，值为该模型的输出张量个数。
  - 示例：
    ```python
    print(model.output_counts)
    # 输出：{'yolov5x_672x672_nv12': 3}
    ```
- output_names: Dict[str, List[str]]
  - 功能说明：
    - 每个模型的输出张量名称列表。
  - 结构说明：
    - 外层 Dict[str, ...]：键为模型名称。
    - 内层 List[str]：为该模型所有输出张量的名称列表。
  - 示例：
    ```python
    print(model.output_names)
    # 输出：{'yolov5x_672x672_nv12': ['output', '1310', '1312']}
    ```
- output_descs: Dict[str, Dict[str, str]]
  - 功能说明：
    - 每个输出张量的描述。
  - 结构说明：
    - 外层 Dict[str, ...]：模型名称。
    - 内层 Dict[str, str]：键为输出张量名称，值为描述信息。
  - 示例：
    ```python
    print(model.output_descs)
    # 输出：{'yolov5x_672x672_nv12': {'1310': 'xxx', '1312': 'xxx', 'output': 'xxx'}}
    ```
- output_shapes: Dict[str, Dict[str, List[int]]]
  - 功能说明：
    -  每个输出张量的 shape。
  - 结构说明：
    - 外层 Dict[str, ...]：模型名称。
    - 内层 Dict[str, List[int]]：键为输出名，值为输出张量的维度（形状）。
  - 示例：
    ```python
    print(model.output_shapes)
    # 输出：{'yolov5x_672x672_nv12': {'1310': [1, 42, 42, 255], '1312': [1, 21, 21, 255], 'output': [1, 84, 84, 255]}}
    ```
- output_dtypes: Dict[str, Dict[str, List[int]]]
  - 功能说明：
    -  每个输出张量的数据类型。
  - 结构说明：
    - 外层 Dict[str, ...]：模型名称。
    - 内层 Dict[str, hbDNNDataType]：键为输出张量名，值为数据类型（例如 F32、U8）。
  - 示例：
    ```python
    print(model.output_dtypes)
    # 输出：{'yolov5x_672x672_nv12': {'1310': <hbDNNDataType.S32: 8>, '1312': <hbDNNDataType.S32: 8>, 'output': <hbDNNDataType.S32: 8>}}
    ```
- output_quants: Dict[str, Dict[str, QuantParams]]
  - 功能说明：
    - 提供每个模型所有输出张量的量化参数信息。用于支持量化模型的后处理计算（如将 int8 数据还原为 float32），或者了解张量的量化方式（scale-based 等）。
  - 结构说明：
    - 外层 Dict[str, ...]：键是模型名称（model name），例如 "resnet50";
    - 内层 Dict[str, QuantParams]：键是输出张量名，值为 QuantParams 实例；
    - QuantParams 类属性：
      - scale: np.ndarray — 量化比例因子，通常为浮点数数组；
      - zero_point: np.ndarray — 零点，用于对称/非对称量化偏移；
      - quant_type: hbDNNQuantiType — 量化类型枚举值（如 SCALE、NONE）；
      - axis: int — 如果是通道量化，则该字段表示在哪个轴上量化。
  - 示例：
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
  - 功能说明：
    -  每个输出张量的 stride 步长信息
  - 结构说明：
    - 外层 Dict[str, ...]：模型名称。
    - 内层Dict[str, List[int]]：键为输出名，值为输出张量的stride信息。
  - 示例：
    ```python
    print(model.output_strides)
    # 输出：{'yolov5x_672x672_nv12': {'1310': [1806336, 43008, 1024, 4], '1312': [451584, 21504, 1024, 4], 'output': [7225344, 86016, 1024, 4]}}
    ```
  注意：stride的详细含义可参考[OE文档](http://j6.doc.oe.hobot.cc/3.0.31/guide/ucp/runtime/bpu_sdk_api/data_structure/hbDNNTensorProperties.html)中libdnn库中的描述；

- sched_params: Dict[str, SchedParam]
  - 功能说明：
    sched_params是用于获取当前所有模型的调度参数（Scheduling Parameters），包括每个模型的：
    - 优先级（priority）
    - 自定义 ID（customId）
    - 分配的 BPU 核心（bpu_cores）
    - 所属设备 ID（deviceId）
    这些调度参数用于影响模型在硬件上运行的方式，尤其在多模型部署或多核设备上尤为重要。
  - 结构说明：
    - 外层 Dict[str, ...]：模型名称。
    - 内层SchedParam ：为SchedParam 类的实例，包含该模型的调度参数priority、customId、bpu_cores和deviceId；
      ```python
      {
          "模型名": SchedParam(
              priority: int,
              customId: int,
              bpu_cores: List[int],
              deviceId: int
          )
      }
      ```
  - 示例：
    ```python
    params = model.sched_params
    for name, sched in params.items():
        print(f"Model: {name}")
        print(f"  priority: {sched.priority}")
        print(f"  customId: {sched.customId}")
        print(f"  bpu_cores: {sched.bpu_cores}")
        print(f"  deviceId: {sched.deviceId}")
    # 输出：
    # Model: yolo12s_detect_nashe_640x640_nv12
    #   priority: 10
    #   customId: 0
    #   bpu_cores: [0]
    #  deviceId: 0
    # Model: yolov5nu_detect_nashe_640x640_nv12
    #   priority: 66
    #   customId: 0
    #   bpu_cores: [-1]
    #   deviceId: 0
    ```
    注意：bpu_cores返回-1表示由调度器自动分配；
##### 推理执行函数
- run(input_tensor, **kwargs)
  - 函数签名
    ```python
    run(input_tensor: np.ndarray, **kwargs) -> Dict[str, Dict[str, np.ndarray]]
    ```
  - 功能说明
    适用于单模型、单输入的推理。输入为一个 NumPy 数组，对应模型的唯一输入张量，当只有一个模型被加载时输入的模型名称可省略。
  - 参数说明

    | 参数名       | 类型          | 说明                                                                                             |
    |--------------|---------------|--------------------------------------------------------------------------------------------------|
    | input_tensor | np.ndarray    | 单输入张量，仅用于单模型且单输入的推理场景。张量 shape 必须与模型对应输入一致。                    |
    | kwargs       | 可变关键字参数 |`model_name` (`str`): 指定模型名称（若为单个模型可省略，否则需指定）|

  - 返回值
    - 类型：Dict[str, Dict[str, np.ndarray]]
    - 外层字典键：模型名称
    - 内层字典键：输出张量名称
    - 值：对应的 NumPy 输出数组
  - 示例：参考快速开始章节，单模型单输入推理部分。
- run(input_tensors: Dict[str, np.ndarray], **kwargs)
  - 函数签名
    ```python
    run(input_tensors: Dict[str, np.ndarray], **kwargs) -> Dict[str, Dict[str, np.ndarray]]
    ```
  - 功能说明
    适用于单模型、多输入的推理。每个输入张量通过输入名指定，与模型定义保持一致，当只有一个模型被加载时输入的模型名称可省略。
  - 参数说明

    | 参数名        | 类型                      | 说明                                                                                           |
    |---------------|---------------------------|------------------------------------------------------------------------------------------------|
    | input_tensors | Dict[str, np.ndarray]     | 多输入张量，仅用于单模型多输入的推理场景。键为输入张量名称，值为对应的 NumPy 数组。                |
    | kwargs        | 可变关键字参数            |`model_name` (`str`): 指定模型名称（若为单个模型可省略，否则需指定）|

  - 返回值
    同上。
  - 示例：参考快速开始章节，单模型多输入推理部分。
- run(multi_input_tensors: Dict[str, Dict[str, np.ndarray]], **kwargs)
  - 函数签名
    ```python
    run(multi_input_tensors: Dict[str, Dict[str, np.ndarray]], **kwargs) -> Dict[str, Dict[str, np.ndarray]]
    ```
  - 功能说明
    适用于多模型同时推理的场景，每个模型提供各自的多个输入张量，不需要指定 model_name，模型名已在 key 中体现，若指定模型名称则只会推理指定的模型。
  - 参数说明

        | 参数名              | 类型                              | 说明 |
        |-------------------|-----------------------------------|------|
        | multi_input_tensors | Dict[str, Dict[str, np.ndarray]] | 多模型推理，外层键为模型名称，内层是输入名 → 张量的映射。支持同时运行多个模型（每个模型可多输入）。 |
        | kwargs             | 可变关键字参数                    | `model_name (str)`: 指定模型名称（若为单个模型可省略，否则需指定） |

  - 返回值

    同上
  - 示例
    ```python
    import numpy as np
    from hbm_runtime import HB_HBMRuntime

    # 映射 hbDNNDataType 到 numpy 类型
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

    # 加载多个模型
    model_files = ["/opt/hobot/model/s100/basic/lanenet256x512.hbm",
        "/opt/hobot/model/s100/basic/yolov5x_672x672_nv12.hbm"]

    model = HB_HBMRuntime(model_files)

    # 打印加载的模型名称
    print("Loaded models:", model.model_names)

    # 为每个模型构造多个输入张量
    multi_input_tensors = {}

    for model_name in model.model_names:
        model_inputs = {}

        for input_name in model.input_names[model_name]:
            shape = model.input_shapes[model_name][input_name]
            dtype_enum = model.input_dtypes[model_name][input_name]

            np_dtype = hb_dtype_map.get(dtype_enum.name, np.float32)

            model_inputs[input_name] = np.ones(shape, dtype=np_dtype)

        multi_input_tensors[model_name] = model_inputs

    # 可选：指定推理优先级和 BPU 设备
    priority = {name: 5 for name in model.model_names}
    bpu_cores = {name: [0] for name in model.model_names}

    model.set_scheduling_params(
        priority=priority,
        bpu_cores=bpu_cores
    )

    # 执行推理
    results = model.run(multi_input_tensors)

    # 输出结果
    for model_name, outputs in results.items():
        print(f"\nModel: {model_name}")
        for output_name, output_tensor in outputs.items():
            print(f"  Output: {output_name}, shape={output_tensor.shape}, dtype={output_tensor.dtype}")
    ```
- kwargs 参数详细说明
  - model_name
    - 类型：str模型字符串
    - 说明：指定当前推理使用的模型，必须是当前加载的模型之一。在前两个run方法中，若加的模型只有一个，此参数可省略，若有多个模型，必须指定该参数。第三个run方法中还是可省略，省略后会推理multi_input_tensors中给出的所有模型，若指定，则只会推理指定的模型。
    - 示例：
        ```python
        outputs = model.run(input_tensor，model_name="resnet18")
        ```
##### 配置函数
- set_scheduling_params
  - 函数签名
    ```python
    def set_scheduling_params(
        priority: Optional[Dict[str, int]] = None,
        bpu_cores: Optional[Dict[str, List[int]]] = None,
        custom_id: Optional[Dict[str, int]] = None,
        device_id: Optional[Dict[str, int]] = None
    ) -> None
    ```
  - 参数说明

    | 参数名      | 类型                             | 说明                                                                 |
    |-------------|----------------------------------|----------------------------------------------------------------------|
    | priority    | 可选字典（模型名 -> int）        | 设置每个模型的调度优先级，范围通常为 0～255，数值越高优先级越高       |
    | bpu_cores   | 可选字典（模型名 -> List[int]）  | 指定模型绑定的 BPU 核心索引列表，默认表示自动分配，具体需看硬件支持情况 |
    | custom_id   | 可选字典（模型名 -> int）        | 自定义优先级，例如：时间戳、frame id等，数值越小优先级越高。优先级：priority > customId。|
    | device_id   | 可选字典（模型名 -> int）        | 指定模型运行在哪个设备上                                             |

  - 返回值
    无（None）
  - 示例：
    ```python
    # 设置调度参数
    model.set_scheduling_params(
        priority={"model1": 200, "model2": 100},
        bpu_cores={"model1": [0, 1], "model2": [0]}
    )

    # 验证设置是否生效
    params = runtime.sched_params
    print(params["model1"].priority)   # 输出: 200
    print(params["model1"].bpu_cores)  # 输出: [0, 1]
    ```
- 异常说明
  - 若输入张量维度、类型与模型不匹配，会抛出 ValueError。
  - 若输入张量非连续（非 C-style），内部会自动 copy 一份连续张量。
  - 推理前需确保输入张量 shape 与 input_shapes 完全一致。
#### QuantParams 类
  张量量化参数对象。
##### 属性
- scale: numpy.ndarray，量化比例因子数组
- zero_point: numpy.ndarray，零点数组
- quant_type: hbDNNQuantiType 类型，表示量化模式
- axis: int，量化轴（若为 per-channel 量化）
##### 示例：
    ```python
    # 获取模型某个输出的量化参数
    tensor_qparams = model.output_quants[model_name][output_name]
    print("scale:", tensor_qparams.scale)
    print("zero_point:", tensor_qparams.zero_point)
    print("type:", tensor_qparams.quant_type)
    print("axis:", tensor_qparams.axis)
    ```

#### SchedParam 类
  模型调度参数对象，用于配置模型在硬件上的运行调度策略（优先级、核心绑定等）。
##### 属性
- priority: Dict[str, int]
 每个模型的优先级设置。键为模型名，值为优先级整数（数值越大，优先级越高，取值范围为 0~255）。
- customId: Dict[str, int]
 自定义优先级，例如：时间戳、frame id等，数值越小优先级越高。优先级：priority > customId。
- bpu_cores: Dict[str, List[int]]
 模型绑定的 BPU 核心编号列表。键为模型名，值为一个整数列表，例如 [0], [0, 1] 表示绑定一个或多个核心。
- deviceId: Dict[str, int]
 指定模型部署的设备 ID。键为模型名，值为设备编号。
##### 示例：
  ```python
  from hbm_runtime import HB_HBMRuntime, SchedParam

  #创建一个调度参数对象
  sched = SchedParam()
  sched.priority = {"modelA": 8}
  sched.customId = {"modelA": 1001}
  sched.bpu_cores = {"modelA": [0, 1]}
  sched.deviceId = {"modelA": 0}

  # 应用调度参数到运行时
  model.set_scheduling_params(priority=sched.priority,
                              custom_id=sched.customId,
                              bpu_cores=sched.bpu_cores,
                              device_id=sched.deviceId)
  ```
