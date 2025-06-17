---
sidebar_position: 6
---
# 4.2.6 Model Pre-processing API

## hbDNNResize()

**【Function Prototype】**

``int32_t hbDNNResize(hbDNNTaskHandle_t *taskHandle, hbDNNTensor *output, const hbDNNTensor *input, const hbDNNRoi *roi, hbDNNResizeCtrlParam *resizeCtrlParam)``

**【Description】**

Resize tasks according to the input parameters.

:::info Note
  This interface is compatible with older versions and will not be maintained in the future. If you need to resize the model input, please refer to the ``hbDNNRoiInfer()`` function for model inference.
:::

**【Parameters】**

- [out] ``taskHandle``: Handle pointer to the task.
- [in/out] ``output``: Output of the resize task.
- [in] ``input``: Input of the resize task.
- [in] ``roi``: Roi information of the input.
- [in] ``resizeCtrlParam``: Parameters to control the resize task.

**【Return Type】**

- Returns ``0`` if the API is executed successfully, otherwise it fails.

:::info Note

  1. Currently, only resizing with the same ``hbDNNDataType`` is supported, and it must be of the ``IMG`` type.
  2. For inputs of type ``HB_DNN_IMG_TYPE_NV12`` or ``HB_DNN_IMG_TYPE_NV12_SEPARATE``, the width and height must be multiples of 2.
  3. The scaling range is `dst / src ∈ [1/185, 256)`.
  4. The original image size should satisfy `1 <= W <= 4080`, `16 <= stride <= 4080`, and the ``stride`` must be a multiple of 16.
  5. The output size should satisfy `Wout <= 4080`, `Hout <= 4080`.
  6. The ``roi`` must be inside the image.
  7. Up to 32 resize tasks can exist simultaneously.
:::