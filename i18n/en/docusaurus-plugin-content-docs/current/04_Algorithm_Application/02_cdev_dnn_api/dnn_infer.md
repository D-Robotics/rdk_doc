---
sidebar_position: 4
---
# 4.2.4 Model Inference API


## hbDNNInfer()


**【Function Prototype】**  

``int32_t hbDNNInfer(hbDNNTaskHandle_t *taskHandle, hbDNNTensor **output, const hbDNNTensor *input, hbDNNHandle_t dnnHandle, hbDNNInferCtrlParam *inferCtrlParam)``

**【Description】** 

Execute an inference task based on the input parameters. The caller can use the returned ``taskHandle`` across functions and threads.

**【Parameters】**

- [out]     ``taskHandle``          Pointer to the task handle.
- [in/out]  ``output``              Output of the inference task.
- [in]      ``input``               Input of the inference task.
- [in]      ``dnnHandle``           Pointer to the DNN handle.
- [in]      ``inferCtrlParam``      Parameters to control the inference task.

**【Return Type】** 

- Returns ``0`` if the API is executed successfully, otherwise it fails.

:::info Note

  When using this interface to submit a task, the ``taskHandle`` should be set to ``nullptr`` in advance, unless it is to append a task to the specified ``taskHandle`` (i.e., using the ``inferCtrlParam::more`` function).

  Up to 32 model tasks are supported at the same time.

  For batch models, it is allowed to set the memory address of input tensors separately. For example, if the input validShape/alignedShape of the model is [4, 3, 224, 224], you can allocate four hbDNNTensors, and set the validShape/alignedShape of each hbDNNTensor as [1, 3, 224, 224] to store the data of each batch. When the model has multiple inputs, the order of ``input`` should be input0[batch0], input0[batch1], ..., inputn[batch0], inputn[batch1], ....

:::

## hbDNNRoiInfer()


**【Function Prototype】**  

``int32_t hbDNNRoiInfer(hbDNNTaskHandle_t *taskHandle, hbDNNTensor **output, const hbDNNTensor *input, hbDNNRoi *rois, int32_t roiCount, hbDNNHandle_t dnnHandle, hbDNNInferCtrlParam *inferCtrlParam)``

**【Description】** 

Execute an ROI inference task based on the input parameters. The caller can use the returned ``taskHandle`` across functions and threads.

**[Parameters]**

- [out]     ``taskHandle``       Pointer to the task handle.
- [in/out]  ``output``           Output of the inference task.
- [in]      ``input``            Input of the inference task.
- [in]      ``rois``             Roi box information.
- [in]      ``roiCount``         Number of roi boxes.
- [in]      ``dnnHandle``        Pointer to the dnn handle.
- [in]      ``inferCtrlParam``   Parameters to control the inference task.

**[Return Type]**

- Returns ``0`` if the API is executed successfully, otherwise it fails.

:::info Note

  If using **RDK X3**, follow these rules:
  | This interface supports batch processing, assuming the number of data batches to be inferred is ``batch``, and the number of model inputs is ``input_count``, where the number of input sources for the resizer is ``resizer_count``.
  | Prepare input parameter ``input``: The range of the array index of ``input`` for the i-th ``batch`` is `[i * input\_count`, `(i + 1) * input\_count)，i=[0,batch)`.
  | Prepare input parameter ``rois``: Each input source for the resizer should match a roi. The range of the array index of ``rois`` for the i-th ``batch`` is `[i * resizer\_count`, `(i + 1) * resizer\_count)，i=[0,batch)`. The order of rois in each batch should be consistent with the order of the inputs.
  | Limit on the number of ``batch``: The range should be [1, 255].

  Model Limitations: The compilation parameter ``input_source`` of the model needs to be set to ``resizer`` during compilation, and the h*w of the model should be less than 18432.

  When submitting a task using this interface, the ``taskHandle`` should be set to ``nullptr`` in advance, unless it is appending a task to the specified ``taskHandle`` (i.e. using the ``inferCtrlParam::more`` function).

  The ``left`` and ``top`` values of the ``roi`` must be even, while the ``right`` and ``bottom`` values must be odd.

  The size of the ``roi`` must satisfy the following requirements: `16 <= width < 256`, `16 <= height < 256`.

  The scaling range is `0.5 < roi / src <= 8`.

  Currently, up to 32 model tasks can exist simultaneously.

  API Example: Please refer to the ``roi_infer.sh`` instructions in [Model Inference DNN API Usage Example Documentation](/toolchain_development/intermediate/runtime_sample#model-inference-dnn-api-usage-examples) for more details.

  Model Limitations: During the model conversion, set the compilation parameter input_source to `{'input_name': 'resizer'}` to generate a resizer model. For specific parameter configuration details, refer to the introduction in [PTQ Quantization Principles and Steps for Model Conversion](/toolchain_development/intermediate/ptq_process#model-conversion).

  ![resizer](./image/cdev_dnn_api/resizer.png)

  Currently, NV12 data with multiple inputs is also supported. The commonly used output sizes (HxW) of the resizer are: 128x128, 128x64, 64x128, 160x96.
:::

:::info Note

  If using **RDK Ultra**, follow these rules:

- ``input_count``: Number of model input branches
- ``output_count``: Number of model output branches
- ``resizer_count``: Number of resizer input-source branches in the model (≤input_count). When the model processes a batch of data, one resizer input-source branch handles one roi.
- ``roiCount``: Total number of rois, its value is ``batch * resizer_count``
- ``data_batch``: The number of data batches that the model needs to infer, its value is ``roiCount / resizer_count``
- ``model_batch``: The batch size used internally by the model, i.e., the batch_size input to the model during inference. D-Robotics Toolchain supports compiling the model as a batch model.

Input/Output Example Explanation:

Taking a more complex multi-input model as an example, let's assume the model has 3 input branches (2 resizer inputs, 1 ddr input) and 1 output branch, and is compiled with ``batch=2``. The model needs to process a total of 3 batches of data, with 6 rois in total (i.e., 2 rois per data batch). Given the following information:

- ``input_count`` = 3
- ``output_count`` = 1
- ``resizer_count`` = 2
- ``roiCount`` = 6
- ``data_batch`` = 3
- ``model_batch`` = 2

So for model inference, these three batches of data need to prepare ``input_count * data_batch = 9`` input-tensors with independent addresses.

Assuming the static information of the model input/output is as follows:

- Model Input (model_info):

  - tensor_0_resizer: [2, 3, 128, 128]
  - tensor_1_resizer: [2, 3, 256, 256]
  - tensor_2_ddr: [2, 80, 1, 100]

- Model Output (model_info):

  - tensor_out: [2, 100, 1, 56]

Then the dynamic information of the model during inference is:

- Model Input (input_tensors):

  - [1x3x128x128, 1x3x256x256, 1x80x1x100, 1x3x128x128, 1x3x256x256, 1x80x1x100, 1x3x128x128, 1x3x256x256, 1x80x1x100]

- Model Output (output_tensors):

  - [4x100x1x56]

Here, because ``model_batch = 2``, the underlying BPU can handle 2 batches of data in a single execution. And because ``data_batch = 3``, the highest dimension of the output_tensor is calculated using the formula ``ceil[(data_batch) / model_batch] * model_batch``, which is always a multiple of ``model_batch``. This is also a hardware instruction requirement of the BPU, and any missing inputs will be automatically ignored during computation.

Interface Limitations Explanation:

- Regarding the ``batch`` number limitation: its range should be [1, 255].
- When using this interface to submit a task, ``taskHandle`` should be set to ``nullptr`` in advance, unless it is appending a task to a specific ``taskHandle`` (i.e., using the ``inferCtrlParam::more`` functionality).
- The roi size requirement is: `2 <= width <= 4096`, `2 <= height <= 4096`.
- The original image size requirement is: `1 <= W <= 4096`, `16 <= stride <= 131072`, and ``stride`` must be a multiple of 16.
- The output size requirement is: `2 <= Wout`, `2 <= Hout`.
- The roi scaling factor limit is: `0 <= step <= 262143`, where the step is calculated using the formula: `step = ((src_len - 1)*65536 + (dst_len - 1)/2)/(dst_len - 1)`, where src_len is the W or H of the roi and dst_len is the required W or H of the model.
- Up to 32 model tasks can exist simultaneously.
:::

## hbDNNWaitTaskDone()

**【Function prototype】**

``int32_t hbDNNWaitTaskDone(hbDNNTaskHandle_t taskHandle, int32_t timeout)``

**【Description】**

Wait for task completion or timeout.

**【Parameters】**

- [in]  ``taskHandle``         Pointer to the task handle.
- [in]  ``timeout``            Timeout configuration (in milliseconds).

**【Return type】**

- Returns ``0`` if the API is executed successfully, otherwise it fails.

:::info Note

  1. ``timeout > 0`` indicates the waiting time;
  2. ``timeout <= 0`` indicates waiting until the task is completed.
:::

## hbDNNReleaseTask()

**【Function prototype】**

``int32_t hbDNNReleaseTask(hbDNNTaskHandle_t taskHandle)``

**【Description】**

Release the task. If the task is not executed, it will be canceled and released directly. If it has been executed, it will be canceled and released after running to certain nodes.

**【Parameters】**

- [in]  ``taskHandle``         Pointer to the task handle.

**【Return type】**

- Returns ``0`` if the API is executed successfully, otherwise it fails.