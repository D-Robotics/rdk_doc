---
sidebar_position: 7
---
# 4.2.7 Data Types and Data Structures

## Version Information Class

``HB_DNN_VERSION_MAJOR``


    #define HB_DNN_VERSION_MAJOR 1U

DNN major version number.

``HB_DNN_VERSION_MINOR``


    #define HB_DNN_VERSION_MINOR 1U

DNN minor version number.

``HB_DNN_VERSION_PATCH``



    #define HB_DNN_VERSION_PATCH 0U

DNN patch version number.

:::info Note

  Please note that the version numbers of the version information types in this section may vary with each release. The version numbers provided here are for reference only. Please refer to the release materials you have obtained for the actual version.
:::

## Model Class

``HB_DNN_TENSOR_MAX_DIMENSIONS``



    #define HB_DNN_TENSOR_MAX_DIMENSIONS 8

The maximum dimensions of a tensor are set to ``8``.

``HB_DNN_INITIALIZE_INFER_CTRL_PARAM``

```
#define HB_DNN_INITIALIZE_INFER_CTRL_PARAM(param) \
{                                                 \
    (param)->bpuCoreId = HB_BPU_CORE_ANY;         \
    (param)->dspCoreId = HB_DSP_CORE_ANY;         \
    (param)->priority = HB_DNN_PRIORITY_LOWEST;   \
    (param)->more = false;                        \
    (param)->customId = 0;                        \
    (param)->reserved1 = 0;                       \
    (param)->reserved2 = 0;                       \
}
```

Initialize control parameters.

``hbPackedDNNHandle_t``

```
typedef void *hbPackedDNNHandle_t;
```

DNN handle, pointing to packed multiple models.

``hbDNNHandle_t``

```
typedef void *hbDNNHandle_t;
```

DNN handle, pointing to a single model.

``hbDNNTaskHandle_t``

```
typedef void *hbDNNTaskHandle_t;
```

Task handle, pointing to a task.

``hbDNNTensorLayout``

```
typedef enum {
  HB_DNN_LAYOUT_NHWC = 0,
  HB_DNN_LAYOUT_NCHW = 2,
  HB_DNN_LAYOUT_NONE = 255,
} hbDNNTensorLayout;
```
The arrangement format of tensors. ``NHWC`` stands for Number, Height, Width, and Channel respectively.

+ Members

    | Member Name           | Description               |
    |------------------------|---------------------------|
    | ``HB_DNN_LAYOUT_NONE`` | No layout defined.         |
    | ``HB_DNN_LAYOUT_NHWC`` | Layout is ``NHWC``.       |
    | ``HB_DNN_LAYOUT_NCHW`` | Layout is ``NCHW``.       |

``hbDNNDataType``
```c
    typedef enum {
      HB_DNN_IMG_TYPE_Y,
      HB_DNN_IMG_TYPE_NV12,
      HB_DNN_IMG_TYPE_NV12_SEPARATE,
      HB_DNN_IMG_TYPE_YUV444,
      HB_DNN_IMG_TYPE_RGB,
      HB_DNN_IMG_TYPE_BGR,
      HB_DNN_TENSOR_TYPE_S4,
      HB_DNN_TENSOR_TYPE_U4,
      HB_DNN_TENSOR_TYPE_S8,
      HB_DNN_TENSOR_TYPE_U8,
      HB_DNN_TENSOR_TYPE_F16,
      HB_DNN_TENSOR_TYPE_S16,
      HB_DNN_TENSOR_TYPE_U16,
      HB_DNN_TENSOR_TYPE_F32,
      HB_DNN_TENSOR_TYPE_S32,
      HB_DNN_TENSOR_TYPE_U32,
      HB_DNN_TENSOR_TYPE_F64,
      HB_DNN_TENSOR_TYPE_S64,
      HB_DNN_TENSOR_TYPE_U64,
      HB_DNN_TENSOR_TYPE_MAX
    } hbDNNDataType;
```
Tensor types. ``S`` represents signed, ``U`` represents unsigned, and ``F`` represents floating-point; the number following these letters indicates the number of bits.

``HB_DNN_IMG_TYPE_NV12`` and ``HB_DNN_IMG_TYPE_NV12_SEPARATE`` both represent NV12 data but differ in storage.

When inferring a model with NV12 input, users can change the ``tensorType`` attribute according to their actual situation to either ``HB_DNN_IMG_TYPE_NV12`` or ``HB_DNN_IMG_TYPE_NV12_SEPARATE``.

+ Members

    | Member Name                      | Description                              |
    |-----------------------------------|------------------------------------------|
    | ``HB_DNN_IMG_TYPE_Y``             | Tensor type is an image with only Y channel. |
    | ``HB_DNN_IMG_TYPE_NV12``       | Tensor type is an NV12 image.                |
    | ``HB_DNN_IMG_TYPE_NV12_SEPARATE`` | Tensor type is an image with separate Y and UV channels as input. |
    | ``HB_DNN_IMG_TYPE_YUV444``        | Tensor type is an image with YUV444 input.       |
    | ``HB_DNN_IMG_TYPE_RGB``           | Tensor type is an RGB image.                  |
    | ``HB_DNN_IMG_TYPE_BGR``           | Tensor type is a BGR image.                    |
    | ``HB_DNN_TENSOR_TYPE_S4``         | Tensor type is signed 4-bit.                     |
    | ``HB_DNN_TENSOR_TYPE_U4``         | Tensor type is unsigned 4-bit.                    |
    | ``HB_DNN_TENSOR_TYPE_S8``         | Tensor type is signed 8-bit.                     |
    | ``HB_DNN_TENSOR_TYPE_U8``         | Tensor type is unsigned 8-bit.                    |
    | ``HB_DNN_TENSOR_TYPE_F16``        | Tensor type is floating-point 16-bit.            |
    | ``HB_DNN_TENSOR_TYPE_S16``        | Tensor type is signed 16-bit.                    |
    | ``HB_DNN_TENSOR_TYPE_U16``        | Tensor type is unsigned 16-bit.                   |
    | ``HB_DNN_TENSOR_TYPE_F32``        | Tensor type is floating-point 32-bit.            |
    | ``HB_DNN_TENSOR_TYPE_S32``        | Tensor type is signed 32-bit.                    |
    | ``HB_DNN_TENSOR_TYPE_U32``        | Tensor type is unsigned 32-bit.                   |
    | ``HB_DNN_TENSOR_TYPE_F64``        | Tensor type is floating-point 64-bit.            |
    | ``HB_DNN_TENSOR_TYPE_S64``        | Tensor type is signed 64-bit.                    |
    | ``HB_DNN_TENSOR_TYPE_U64``        | Tensor type is unsigned 64-bit.                   |
    | ``HB_DNN_TENSOR_TYPE_MAX``        | Represents the maximum tensor type ID.          |

``hbDNNTensorShape``
```C
    typedef struct {
      int32_t dimensionSize[HB_DNN_TENSOR_MAX_DIMENSIONS];
      int32_t numDimensions;
    } hbDNNTensorShape;
```
Tensor shape. For example, a 224x224 BGR color space image has ``numDimensions=4``, if the layout is NHWC,
the ``dimensionSize`` array would sequentially store the image's ``Number=1``, ``Height=224``, ``Width=224``, and ``Channel=3``.

+ Members

    |    Member Name   |       Description       |
    |-------------------|-------------------------|
    | ``dimensionSize`` | Size of each tensor dimension. |
    | ``numDimensions`` | The number of dimensions in the tensor. |

``hbDNNQuantiShift``
```C
    typedef struct {
      int32_t shiftLen;
      uint8_t *shiftData;
    } hbDNNQuantiShift;
```
Shift/De-Shift Data for Quantization/Dequantization.

**For Input:**
If a floating-point data point ``data[i]`` is collected, with its corresponding shift data being ``shift[i]``, the inference data fed into the model would be: `data[i] * (1 << shift[i])` rounded to the nearest integer;

**For Output:**
If the inference result is ``data[i]`` and its corresponding shift data is ``shift[i]``, then the final inference result would be: `data[i] / (1 << shift[i])`.

:::caution Note
Here, ``shiftLen`` is determined by the data ``data`` based on either the `per-axis` or `per-tensor` quantization/dequantization method.
When the data ``data`` is quantized/dequantized using the "per-tensor" method, ``shiftLen`` equals ``1``, and in this case, the value of ``quantizeAxis`` need not be considered; otherwise, it equals the size of the ``quantizeAxis`` dimension in the data ``data``.
:::

+ **Members**

    | Member Name | Description |
    |---------------|--------------------|
    | ``shiftLen``  | Length of the shift data. |
    | ``shiftData`` | Start address of the shift data. |

``hbDNNQuantiScale``
```C
    typedef struct {
      int32_t scaleLen;
      float *scaleData;
      int32_t zeroPointLen;
      int8_t *zeroPointData;
    } hbDNNQuantiScale;
```
Scaling/De-scaling Data for Quantization/Dequantization.

**For Input:**
If a floating-point data point ``data[i]`` is collected, with its corresponding scaling data as ``scale[i]`` and zero-point offset data as ``zeroPoint[i]``, the inference data fed into the model would be: `g((data[i] / scale[i]) + zeroPoint[i])`, where `g(x) = clip(nearbyint(x))` using fesetround(FE_TONEAREST) rounding method, truncated to: U8: `g(x) ∈ [0, 255]`, S8: `g(x) ∈ [-128, 127]`;

**For Output:**
If the inference result is ``data[i]`` with its corresponding scaling data as ``scale[i]`` and zero-point offset data as ``zeroPoint[i]``, the final inference result would be: `(data[i] - zeroPoint[i]) * scale[i]`.

:::caution Note
Here, ``scaleLen`` is determined by the data ``data`` based on either the `per-axis` or `per-tensor` quantization/dequantization method.
When the data ``data`` is quantized/dequantized using the "per-tensor" method, ``scaleLen`` equals ``1``, and in this case, the value of ``quantizeAxis`` need not be considered; otherwise, it equals the size of the ``quantizeAxis`` dimension in the data ``data``. The length of ``zeroPointLen`` remains consistent with that of ``scaleLen``.
:::

+ **Members**

    | Member Name       | Description        |
    |-------------------|--------------------|
    | ``scaleLen``       | Length of the scaling data.   |
    | ``scaleData``       | Start address of the scaling data. |
    | ``zeropointLen``   | Length of the zero-point offset data. |
    | ``zeropointData`` | Start address of the zero-point offset data. |

``hbDNNQuantiType``
```C
    typedef enum {
      NONE, 
      SHIFT,
      SCALE,
    } hbDNNQuantiType;
```
Quantization/Dequantization Types for Fixed-Point Floating-Point Conversion.

``NONE`` represents no processing required for the data; the parameters for quantization/dequantization of type ``SHIFT`` are stored in the ``hbDNNQuantiShift`` structure, while those for type ``SCALE`` are stored in the ``hbDNNQuantiScale`` structure.

+ **Members**

    | Member Name     | Description       |
    |----------------|---------------------|
    | ``NONE``        | No quantization.    |
    | ``SHIFT``       | Quantization type is ``SHIFT``. |
    | ``SCALE``       | Quantization type is ``SCALE``. |

``hbDNNTensorProperties``
```C
    typedef struct {
      hbDNNTensorShape validShape;
      hbDNNTensorShape alignedShape;
      int32_t tensorLayout;
      int32_t tensorType;
      hbDNNQuantiShift shift;
      hbDNNQuantiScale scale;
      hbDNNQuantiType quantiType;
      int32_t quantizeAxis;
      int32_t alignedByteSize;
      int32_t stride[HB_DNN_TENSOR_MAX_DIMENSIONS];
    } hbDNNTensorProperties;
```
Information about Tensors.

In this context, when obtained from the model information, ``alignedShape`` refers to the tensor's shape after alignment; after preparing input data, ``alignedShape`` must be consistent with the actual input shape of the tensor.

+ **Members**

    | Member Name       | Description         |
    |--------------------|-----------------------|
    | ``validShape``      | Shape of the tensor's valid content. |
    | ``alignedShape``    | Aligned shape of the tensor's content. |
    | ``tensorLayout``    | Tensor layout format. |
    | ``tensorType``      | Type of the tensor. |
    | ``shift``           | Quantization shift. |
    | ``scale``           | Quantization scale. |
    | ``quantiType``      | Quantization type. |
    | ``quantizeAxis``    | Quantization axis, effective only for per-axis quantization. |
    | ``alignedByteSize`` | Memory size of the tensor's aligned content. |
    | ``stride``          | Stride for each dimension in the tensor's validShape. |

:::info Remark
The tensor information obtained through the interface is as required by the model, but you can modify certain tensor information according to your actual input. Currently, only the information of ``alignedShape`` and ``tensorType`` can be modified, and they must meet specific requirements.

  **``alignedShape``:**

  1. If you prepare input based on ``alignedShape``, there is no need to change ``alignedShape``.

  2. If you prepare input based on ``validShape``, you should change ``alignedShape`` to ``validShape``; internally, the inference library will perform padding operations on the data.

  **``tensorType``:**

  When inferring models with NV12 input, you can change the ``tensorType`` attribute of the tensor according to your situation to either ``HB_DNN_IMG_TYPE_NV12`` or ``HB_DNN_IMG_TYPE_NV12_SEPARATE``.
:::

``hbDNNTaskPriority``
```C
    typedef enum {
      HB_DNN_PRIORITY_LOWEST = 0,
      HB_DNN_PRIORITY_HIGHEST = 255,
      HB_DNN_PRIORITY_PREEMP = HB_DNN_PRIORITY_HIGHEST,
    } hbDNNTaskPriority;
```
Configuration of Task Priority, providing default parameters.

``hbDNNTensor``

```C
typedef struct {
  hbSysMem sysMem[4];
  hbDNNTensorProperties properties;
} hbDNNTensor;
```

This structure defines a tensor used for storing input and output information. For tensors of type ``NV12_SEPARATE``, two ``hbSysMem`` elements are required, while all other types require one.

+ Members

    | Member Name  | Description                                   |
    |--------------|------------------------------------------------|
    | ``sysMem``     | Memory where the tensor data is stored.         |
    | ``properties`` | Information about the properties of the tensor. |

``hbDNNRoi``

```C
typedef struct {
  int32_t left;
  int32_t top;
  int32_t right;
  int32_t bottom;
} hbDNNRoi;
```

This structure represents a rectangular region of interest (ROI). The ROI spans from `W∈[left, right]` to `H∈[top, bottom]` in terms of pixel coordinates.

+ Members

    | Member Name | Description                                       |
    |-------------|--------------------------------------------------|
    | ``left``   | Pixel width coordinate of the top-left point.      |
    | ``top``    | Pixel height coordinate of the top-left point.     |
    | ``right``  | Pixel width coordinate of the bottom-right point.  |
    | ``bottom`` | Pixel height coordinate of the bottom-right point. |

``hbDNNInferCtrlParam``

```c
typedef struct {
  int32_t bpuCoreId;
  int32_t dspCoreId;
  int32_t priority;
  int32_t more;
  int64_t customId;
  int32_t reserved1;
  int32_t reserved2;
} hbDNNInferCtrlParam;
```

This structure encapsulates control parameters for model inference. In this context, ``bpuCoreId`` and ``dspCoreId`` are used to control which BPU and DSP cores are utilized for model inference; however, X3 does not support DSP inference, so ``dspCoreId`` is just a placeholder. The ``more`` parameter is designed for scenarios involving batch processing of small models. If the user wants all tasks to finish before getting outputs, they should set ``more`` to ``0`` for the last task and ``1`` for preceding tasks, supporting up to 255 small model inferences, where a small model could be a resizer model with each ROI potentially treated as a separate small model. The ``customId`` parameter allows users to define their own priority level, such as timestamp or frame id, where a smaller value indicates higher priority. Priority order: priority > customId.

+ Members

    | Member Name   | Description                                         |
    |---------------|-----------------------------------------------------|
    | ``bpuCoreId``  | ID of the BPU core.                                 |
    | ``dspCoreId``  | ID of the DSP core (placeholder in X3).             |
    | ``priority``   | Task priority level.                                |
    | ``more``        | Whether there are subsequent tasks following this one.|
    | ``customId``   | Custom priority value defined by the user.           |
    | ``reserved1``  | Reserved field 1.                                    |
    | ``reserved2``  | Reserved field 2.                                    |

## System Classes

``hbBPUCore``

```c
typedef enum {
  HB_BPU_CORE_ANY = 0,
  HB_BPU_CORE_0 = (1 << 0),
  HB_BPU_CORE_1 = (1 << 1)
} hbBPUCore;
```

This enumeration specifies BPU core options.

+ Members

    | Member Name     | Description                               |
    |----------------|-------------------------------------------|
    | ``HB_BPU_CORE_ANY`` | Any available BPU core.                 |
    | ``HB_BPU_CORE_0``   | BPU core 0.                              |
    | ``HB_BPU_CORE_1``   | BPU core 1.                              |

``hbDSPCore``

```c
typedef enum {
  HB_DSP_CORE_ANY = 0,
  HB_DSP_CORE_0 = (1 << 0),
  HB_DSP_CORE_1 = (1 << 1)
} hbDSPCore;
```

This enumeration defines DSP core options.

+ Members

    | Member Name     | Description                               |
    |----------------|-------------------------------------------|
    | ``HB_DSP_CORE_ANY`` | Any available DSP core.                  |
    | ``HB_DSP_CORE_0``   | DSP core 0.                               |
    | ``HB_DSP_CORE_1``   | DSP core 1.                               |

``hbSysMem``

```c
typedef struct {
  uint64_t phyAddr;
  void *virAddr;
  uint32_t memSize;
} hbSysMem;
```

This system memory structure is used for allocating system memory.

+ Members

    | Member Name     | Description                             |
    |----------------|-----------------------------------------|
    | ``phyAddr``    | Physical address.                        |
    | ``virAddr``    | Virtual address.                         |
    | ``memSize``    | Size of the allocated memory.            |

``hbSysMemFlushFlag``

```c
typedef enum {
  HB_SYS_MEM_CACHE_INVALIDATE = 1,
  HB_SYS_MEM_CACHE_CLEAN = 2
} hbSysMemFlushFlag;
```

This enumeration defines system memory and cache synchronization flags. There exists a cache between the CPU and memory, which can cause data discrepancies. To ensure that the most up-to-date data is always retrieved, we need to synchronize data between the cache and memory before and after CPU reads or writes. Before a CPU read, data from memory is updated into the cache. After a CPU write, data from the cache is updated back into memory.

![hbSysMemFlushFlag](./image/cdev_dnn_api/hbSysMemFlushFlag.png)

+ Members

    | Member Name               | Description                                                |
    |--------------------------|------------------------------------------------------------|
    | ``HB_SYS_MEM_CACHE_INVALIDATE`` | Synchronize memory to cache, used before a CPU read operation. |
    | ``HB_SYS_MEM_CACHE_CLEAN``       | Synchronize cache to memory, used after a CPU write operation. |


## Pre-processing Class


``HB_DNN_INITIALIZE_RESIZE_CTRL_PARAM``
```C
    #define HB_DNN_INITIALIZE_RESIZE_CTRL_PARAM(param)     \
      {                                                     \
        (param)->bpuCoreId = HB_BPU_CORE_ANY;              \
        (param)->resizeType = HB_DNN_RESIZE_TYPE_BILINEAR; \
        (param)->priority = HB_DNN_PRIORITY_LOWEST;        \
        (param)->reserved1 = 0;                             \
        (param)->reserved2 = 0;                             \
        (param)->reserved3 = 0;                             \
        (param)->reserved4 = 0;                             \
      }
```
Initialize control parameters.

``hbDNNResizeType``
```C
    typedef enum {
      HB_DNN_RESIZE_TYPE_BILINEAR = 0,
    } hbDNNResizeType;
```
``Resize`` type.

+ Members

    |Member Name     |Description       |
    |-----------|----------------|
    |``HB_DNN_RESIZE_TYPE_BILINEAR`` |  Resize type is bilinear interpolation.|

``hbDNNResizeCtrlParam``

```c
typedef struct {
  int32_t bpuCoreId;
  int32_t priority;
  hbDNNResizeType resizeType;
  int32_t reserved1;
  int32_t reserved2;
  int32_t reserved3;
  int32_t reserved4;
} hbDNNResizeCtrlParam;
```

Controls parameters of `Resize`.

+ Members

    |Member Name     |Description       |
    |-----------|----------------|
    |`bpuCoreId`  | BPU core ID.|
    |`priority`   | Task priority.|
    |`resizeType` | Resize type.|
    |`reserved1`  | Reserved field 1.|
    |`reserved2`  | Reserved field 2.|
    |`reserved3`  | Reserved field 3.|
    |`reserved4`  | Reserved field 4.|