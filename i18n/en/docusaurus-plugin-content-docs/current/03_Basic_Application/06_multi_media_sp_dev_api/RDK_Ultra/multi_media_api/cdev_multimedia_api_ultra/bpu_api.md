---
sidebar_position: 5
---
# BPU (Algorithm Inference Module) API

The `BPU` API provides interfaces for performing algorithm inference tasks, such as AI model execution. It allows users to initialize and manage inference tasks, allocate and release tensor memory, and get prediction results from a specified model.

## Functions

| Function                       | Description                                                       |
|---------------------------------|-------------------------------------------------------------------|
| sp_init_bpu_module              | **Initialize the algorithm inference module object and create an inference task** |
| sp_bpu_start_predict            | **Perform AI algorithm inference and obtain the inference result** |
| sp_release_bpu_module           | **Close the algorithm inference task**                           |
| sp_init_bpu_tensors             | **Allocate memory for tensors**                                  |
| sp_deinit_bpu_tensor            | **Destroy tensor memory**                                        |

## sp_init_bpu_module

**Function Declaration**  
`bpu_module *sp_init_bpu_module(const char *model_file_name)`

**Description**  
Initializes an algorithm inference task by opening the algorithm model file `model_file_name`. This file must be a model that has been converted by the D-Robotics AI algorithm toolchain or a fixed-point model obtained from training.

**Parameters**  
- `model_file_name`: The name of the algorithm model file (must be a valid, pre-processed model).

**Return Type**  
- Returns an AI algorithm inference task object on success.
- Returns `NULL` on failure.

## sp_bpu_start_predict

**Function Declaration**  
`int32_t sp_bpu_start_predict(bpu_module *bpu_handle, char *addr)`

**Description**  
Performs AI algorithm inference by passing image data to the model, and retrieves the inference results.

**Parameters**  
- `bpu_handle`: The AI algorithm inference task object (created by `sp_init_bpu_module`).
- `addr`: The input image data (e.g., pre-processed image or raw data depending on model requirements).

**Return Type**  
- Returns `0` on success.
- Returns `-1` on failure.

## sp_init_bpu_tensors

**Function Declaration**  
`int32_t sp_init_bpu_tensors(bpu_module *bpu_handle, hbDNNTensor *output_tensors)`

**Description**  
Allocates memory for the tensors to store the output of the AI inference process. This is needed to store the results from the inference model.

**Parameters**  
- `bpu_handle`: The AI algorithm inference task object.
- `output_tensors`: The tensor where the output of the inference will be stored.

**Return Type**  
- Returns `0` on success.
- Returns `-1` on failure.

## sp_deinit_bpu_tensor

**Function Declaration**  
`int32_t sp_deinit_bpu_tensor(hbDNNTensor *tensor, int32_t len)`

**Description**  
Frees the memory allocated for a tensor and performs memory cleanup. This function is used to release the tensor data when it is no longer needed.

**Parameters**  
- `tensor`: The tensor to be deinitialized (released).
- `len`: The length of the tensor data.

**Return Type**  
- Returns `0` on success.
- Returns `-1` on failure.

## sp_release_bpu_module

**Function Declaration**  
`int32_t sp_release_bpu_module(bpu_module *bpu_handle)`

**Description**  
Closes the AI algorithm inference task, releasing all related resources.

**Parameters**  
- `bpu_handle`: The AI algorithm inference task object.

**Return Type**  
- Returns `0` on success.
- Returns `-1` on failure.
