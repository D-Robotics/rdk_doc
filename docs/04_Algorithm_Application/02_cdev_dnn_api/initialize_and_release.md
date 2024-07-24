---
sidebar_position: 2
---

# 模型加载/释放 API


## hbDNNInitializeFromFiles()

**【函数原型】**  

``int32_t hbDNNInitializeFromFiles(hbPackedDNNHandle_t *packedDNNHandle, const char **modelFileNames, int32_t modelFileCount)``

**【功能描述】**  

从文件完成对 ``packedDNNHandle`` 的创建和初始化。调用方可以跨函数、跨线程使用返回的 ``packedDNNHandle``。

**【参数】**

- [out] ``packedDNNHandle``  Horizon DNN句柄，指向多个模型。
- [in]  ``modelFileNames``   模型文件的路径。
- [in]  ``modelFileCount``   模型文件的个数。

**【返回类型】**  

- 返回 ``0`` 则表示API成功执行，否则执行失败。

## hbDNNInitializeFromDDR()


**【函数原型】**  

``int32_t hbDNNInitializeFromDDR(hbPackedDNNHandle_t *packedDNNHandle, const void **modelData, int32_t *modelDataLengths, int32_t modelDataCount)``

**【功能描述】**  

从文件完成对 ``packedDNNHandle`` 的创建和初始化。调用方可以跨函数、跨线程使用返回的 ``packedDNNHandle``。

**【参数】**

- [out] ``packedDNNHandle``  Horizon DNN句柄，指向多个模型。
- [in]  ``modelData``        模型文件的指针。
- [in]  ``modelDataLengths`` 模型数据的长度。
- [in]  ``modelDataCount``   模型数据的个数。

**【返回类型】**  

- 返回 ``0`` 则表示API成功执行，否则执行失败。

## hbDNNRelease()


**【函数原型】**  

``int32_t hbDNNRelease(hbPackedDNNHandle_t packedDNNHandle)``

**【功能描述】**  

将 ``packedDNNHandle`` 所指向的模型释放。

**【参数】**

- [in] ``packedDNNHandle``  Horizon DNN句柄，指向多个模型。

**【返回类型】**  

- 返回 ``0`` 则表示API成功执行，否则执行失败。
