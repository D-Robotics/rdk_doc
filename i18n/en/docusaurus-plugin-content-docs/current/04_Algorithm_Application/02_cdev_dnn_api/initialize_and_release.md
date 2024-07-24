---
sidebar_position: 2
---
# 4.2.2 Model Load/Unload API


## hbDNNInitializeFromFiles()

**【Function Prototype】**

``int32_t hbDNNInitializeFromFiles(hbPackedDNNHandle_t *packedDNNHandle, const char **modelFileNames, int32_t modelFileCount)``

**【Function Description】**

Create and initialize ``packedDNNHandle`` from files. The returned ``packedDNNHandle`` can be used across functions and threads.

**【Parameters】**

- [out] ``packedDNNHandle``: D-Robotics DNN handle that points to multiple models.
- [in]  ``modelFileNames``: Path of model files.
- [in]  ``modelFileCount``: Number of model files.

**【Return Type】**

- Return ``0`` if the API is executed successfully, otherwise execution fails.

## hbDNNInitializeFromDDR()


**【Function Prototype】**

``int32_t hbDNNInitializeFromDDR(hbPackedDNNHandle_t *packedDNNHandle, const void **modelData, int32_t *modelDataLengths, int32_t modelDataCount)``

**【Function Description】**

Create and initialize ``packedDNNHandle`` from DDR. The returned ``packedDNNHandle`` can be used across functions and threads.

**【Parameters】**

- [out] ``packedDNNHandle``: D-Robotics DNN handle that points to multiple models.
- [in]  ``modelData``: Pointer to model data.
- [in]  ``modelDataLengths``: Length of model data.
- [in]  ``modelDataCount``: Number of model data.

**【Return Type】**

- Return ``0`` if the API is executed successfully, otherwise execution fails.

## hbDNNRelease()

**【Function Prototype】**

``int32_t hbDNNRelease(hbPackedDNNHandle_t packedDNNHandle)``

**【Description】**

Release the model pointed by ``packedDNNHandle``.

**【Parameters】**

- [in] ``packedDNNHandle``: D-Robotics DNN handle pointing to multiple models.

**【Return Type】**

- Return ``0`` if the API is executed successfully, otherwise indicate failure.