---
sidebar_position: 3
---
# 4.2.3 Model Information Retrieval API

## hbDNNGetModelNameList()


**[Function Prototype]**

``int32_t hbDNNGetModelNameList(const char ***modelNameList, int32_t *modelNameCount, hbPackedDNNHandle_t packedDNNHandle)``

**[Description]**

Retrieve the list of names and the count of models pointed to by ``packedDNNHandle``.

**[Parameters]**

- [out] ``modelNameList`` : List of model names.
- [out] ``modelNameCount`` : Number of model names.
- [in]  ``packedDNNHandle``   : D-Robotics DNN handle that points to multiple models.

**[Return Type]**

- Returns ``0`` for a successful execution of the API, otherwise, it fails.

## hbDNNGetModelHandle()


**[Function Prototype]**

``int32_t hbDNNGetModelHandle(hbDNNHandle_t *dnnHandle, hbPackedDNNHandle_t packedDNNHandle, const char *modelName)``

**[Description]**

Get a handle to a model from the list of models pointed to by ``packedDNNHandle``. The returned ``dnnHandle`` can be used across functions and threads by the caller.

**[Parameters]**

- [out] ``dnnHandle`` : DNN handle that points to a model.
- [in]  ``packedDNNHandle`` : DNN handle that points to multiple models.
- [in]  ``modelName`` : Model name.

**[Return Type]**

- Returns ``0`` for a successful execution of the API, otherwise, it fails.

## hbDNNGetInputCount()**[Function Prototype]**

`int32_t hbDNNGetInputCount(int32_t *inputCount, hbDNNHandle_t dnnHandle)`

**[Description]**

Get the number of input tensors for the model pointed by `dnnHandle`.

**[Parameters]**

- [out] `inputCount`: The number of input tensors for the model.
- [in] `dnnHandle`: DNN handle pointing to a model.

**[Return Type]**

- Returns `0` if the API is executed successfully, otherwise it fails.


## hbDNNGetInputName()


**[Function Prototype]**

`int32_t hbDNNGetInputName(const char **name, hbDNNHandle_t dnnHandle, int32_t inputIndex)`

**[Description]**

Get the name of the input tensor for the model pointed by `dnnHandle` at the specified index.

**[Parameters]**

- [out] `name`: The name of the input tensor.
- [in] `dnnHandle`: DNN handle pointing to a model.
- [in] `inputIndex`: The index of the input tensor.

**[Return Type]**

- Returns `0` if the API is executed successfully, otherwise it fails.


## hbDNNGetInputTensorProperties()


**[Function Prototype]**

`int32_t hbDNNGetInputTensorProperties(hbDNNTensorProperties *properties, hbDNNHandle_t dnnHandle, int32_t inputIndex)`

**[Description]**

Get the properties of the specific input tensor for the model pointed by `dnnHandle`.

**[Parameters]**

- [out] `properties`: The properties of the input tensor.
- [in] `dnnHandle`: DNN handle pointing to a model.
- [in] `inputIndex`: The index of the input tensor.

**[Return Type]**  

- Return ``0`` indicates successful execution of the API, otherwise it fails.

## hbDNNGetOutputCount()


**[FunctionPrototype]**

``int32_t hbDNNGetOutputCount(int32_t *outputCount, hbDNNHandle_t dnnHandle)``

**[Description]** 

Get the number of output tensors in the model pointed by ``dnnHandle``.

**[Parameters]**

- [out] ``outputCount``  Number of output tensors in the model.
- [in]  ``dnnHandle``    DNN handle pointing to a model.

**[Return Type]**  

- Return ``0`` indicates successful execution of the API, otherwise it fails.

## hbDNNGetOutputName()


**[Function Prototype]**

``int32_t hbDNNGetOutputName(const char **name, hbDNNHandle_t dnnHandle, int32_t outputIndex)``

**[Description]** 

Get the name of the output tensor in the model pointed by ``dnnHandle``.

**[Parameters]**

- [out] ``name``        Name of the output tensor.
- [in]  ``dnnHandle``   DNN handle pointing to a model.
- [in]  ``outputIndex``  Index of the output tensor in the model.

**[Return Type]**  

- Return ``0`` indicates successful execution of the API, otherwise it fails.

## hbDNNGetOutputTensorProperties()


**【Function Prototype】**  

``int32_t hbDNNGetOutputTensorProperties(hbDNNTensorProperties *properties, hbDNNHandle_t dnnHandle, int32_t outputIndex)``

**【Function Description】** 

Get the properties of the output tensor specified by ``dnnHandle`` in the model.

**【Parameters】**

- [out] ``properties``    Information of the output tensor.
- [in]  ``dnnHandle``     DNN handle pointing to a model.
- [in]  ``outputIndex``   Index of the model's output tensor.

**【Return Type】** 

Return ``0`` if the API is executed successfully, otherwise it fails.