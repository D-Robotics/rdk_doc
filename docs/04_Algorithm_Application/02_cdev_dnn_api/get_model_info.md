---
sidebar_position: 3
---

# 模型信息获取 API


## hbDNNGetModelNameList()


**【函数原型】**  

``int32_t hbDNNGetModelNameList(const char ***modelNameList, int32_t *modelNameCount, hbPackedDNNHandle_t packedDNNHandle)``

**【功能描述】**  

获取 ``packedDNNHandle`` 所指向模型的名称列表和个数。

**【参数】**

- [out] ``modelNameList``    模型名称列表。
- [out] ``modelNameCount``   模型名称个数。
- [in]  ``packedDNNHandle``  Horizon DNN句柄，指向多个模型。

**【返回类型】**  

- 返回 ``0`` 则表示API成功执行，否则执行失败。

## hbDNNGetModelHandle()


**【函数原型】**  

``int32_t hbDNNGetModelHandle(hbDNNHandle_t *dnnHandle, hbPackedDNNHandle_t packedDNNHandle, const char *modelName)``

**【功能描述】** 

从 ``packedDNNHandle`` 所指向模型列表中获取一个模型的句柄。调用方可以跨函数、跨线程使用返回的 ``dnnHandle``。

**【参数】**

- [out] ``dnnHandle``         DNN句柄，指向一个模型。
- [in]  ``packedDNNHandle``   DNN句柄，指向多个模型。
- [in]  ``modelName``         模型名称。

**【返回类型】**  

- 返回 ``0`` 则表示API成功执行，否则执行失败。

## hbDNNGetInputCount()


**【函数原型】**  

``int32_t hbDNNGetInputCount(int32_t *inputCount, hbDNNHandle_t dnnHandle)``

**【功能描述】** 

获取 ``dnnHandle`` 所指向模型输入张量的个数。

**【参数】**

- [out] ``inputCount``  模型输入张量的个数。
- [in]  ``dnnHandle``   DNN句柄，指向一个模型。

**【返回类型】**  

- 返回 ``0`` 则表示API成功执行，否则执行失败。

## hbDNNGetInputName()


**【函数原型】**  

``int32_t hbDNNGetInputName(const char **name, hbDNNHandle_t dnnHandle, int32_t inputIndex)``

**【功能描述】** 

获取 ``dnnHandle`` 所指向模型输入张量的名称。

**【参数】**

- [out] ``name``        模型输入张量的名称。
- [in]  ``dnnHandle``   DNN句柄，指向一个模型。
- [in]  ``inputIndex``  模型输入张量的编号。

**【返回类型】**  

- 返回 ``0`` 则表示API成功执行，否则执行失败。

## hbDNNGetInputTensorProperties()


**【函数原型】**  

``int32_t hbDNNGetInputTensorProperties(hbDNNTensorProperties *properties, hbDNNHandle_t dnnHandle, int32_t inputIndex)``

**【功能描述】** 

获取 ``dnnHandle`` 所指向模型特定输入张量的属性。

**【参数】**

- [out] ``properties``   输入张量的信息。
- [in]  ``dnnHandle``    DNN句柄，指向一个模型。
- [in]  ``inputIndex``   模型输入张量的编号。

**【返回类型】**  

- 返回 ``0`` 则表示API成功执行，否则执行失败。

## hbDNNGetOutputCount()


**【函数原型】**  

``int32_t hbDNNGetOutputCount(int32_t *outputCount, hbDNNHandle_t dnnHandle)``

**【功能描述】** 

获取 ``dnnHandle`` 所指向模型输出张量的个数。

**【参数】**

- [out] ``outputCount``  模型输出张量的个数。
- [in]  ``dnnHandle``    DNN句柄，指向一个模型。

**【返回类型】**  

- 返回 ``0`` 则表示API成功执行，否则执行失败。

## hbDNNGetOutputName()


**【函数原型】**  

``int32_t hbDNNGetOutputName(const char **name, hbDNNHandle_t dnnHandle, int32_t outputIndex)``

**【功能描述】** 

获取 ``dnnHandle`` 所指向模型输出张量的名称。

**【参数】**

- [out] ``name``        模型输出张量的名称。
- [in]  ``dnnHandle``   DNN句柄，指向一个模型。
- [in]  ``outputIndex``  模型输出张量的编号。

**【返回类型】**  

- 返回 ``0`` 则表示API成功执行，否则执行失败。

## hbDNNGetOutputTensorProperties()


**【函数原型】**  

``int32_t hbDNNGetOutputTensorProperties(hbDNNTensorProperties *properties, hbDNNHandle_t dnnHandle, int32_t outputIndex)``

**【功能描述】** 

获取 ``dnnHandle`` 所指向模型特定输出张量的属性。

**【参数】**

- [out] ``properties``    输出张量的信息。
- [in]  ``dnnHandle``     DNN句柄，指向一个模型。
- [in]  ``outputIndex``   模型输出张量的编号。

**【返回类型】** 

- 返回 ``0`` 则表示API成功执行，否则执行失败。
