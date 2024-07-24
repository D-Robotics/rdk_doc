---
sidebar_position: 6
---

# 模型前处理 API


## hbDNNResize()


**【函数原型】**  

``int32_t hbDNNResize(hbDNNTaskHandle_t *taskHandle, hbDNNTensor *output, const hbDNNTensor *input, const hbDNNRoi *roi, hbDNNResizeCtrlParam *resizeCtrlParam)``

**【功能描述】** 

根据输入参数进行resize任务。

:::info 备注
  此接口为兼容老版本，后续不在维护，若需对模型输入进行Resize，请参考使用 ``hbDNNRoiInfer()`` 函数进行模型推理
:::

**【参数】**

- [out]  ``taskHandle``           任务句柄指针。
- [in/out] ``output``             resize任务的输出。
- [in]   ``input``                resize任务的输入。
- [in]   ``roi``                  输入的roi信息。
- [in]   ``resizeCtrlParam``      控制resize任务的参数。

**【返回类型】**

- 返回 ``0`` 则表示API成功执行，否则执行失败。

:::info 备注

  1. 目前只支持相同 ``hbDNNDataType`` 的resize，并且必须为 ``IMG`` 类型；
  2. 对于 ``HB_DNN_IMG_TYPE_NV12``， ``HB_DNN_IMG_TYPE_NV12_SEPARATE`` 类型的输入，宽和高必须为2的倍数；
  3. 缩放范围是 :math:`dst / src ∈ [1/185, 256)`；
  4. 原图尺寸要求是 :math:`1 <= W <= 4080`, :math:`16 <= stride <= 4080`， ``stride`` 必须是16的倍数；
  5. 输出尺寸要求是 :math:`Wout <= 4080`, :math:`Hout <= 4080`；
  6. ``roi`` 必须在图像的内部。
  7. 最多支持同时存在32个reisze任务。
:::
