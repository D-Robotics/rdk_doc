---
sidebar_position: 3
---
# 7.3.3 System Control
## Overview
System control is used to initialize and deinitialize the entire media system, and establish relationships between modules through binding interfaces. It provides the VP (Video Pool) module for managing large blocks of physical memory allocation.

## Functional Description

### Video Pool

The VP (Video Pool) video pool provides large blocks of physical memory and management functions, and is responsible for memory allocation and recovery.
The video pool is composed of a group of physically contiguous and equally sized buffer blocks. It needs to be configured and initialized before use. The number of buffer pools and the size of buffer blocks can be configured according to the needs.

### Binding Relationship

![image-20220329183230983](../../../../../../static/img/07_Advanced_development/03_multimedia_development/system_control/system_control_bind_relationship.png)

Note: The binding relationship between modules can be established using the HB_SYS_Bind interface. Once bound, the data processed by the data source will be automatically sent to the data sink.

### Operation Mode

**Online Mode:** The data between modules is directly transferred from the previous module to the next module through the internal bus without the need to read/write DDR. This can reduce latency and save DDR bandwidth.

**Offline Mode:** The data from the previous module is first written to DDR, and then the next module reads the data from DDR. When multiple sensors are connected, all connected sensors are processed offline.

| Mode  |      VIN_SIF and VIN_ISP      |        VIN_ISP and VPS         |       VIN_SIF and VPS        |
| :---: | :---------------------------: | :----------------------------: | :--------------------------: |
| Online |    SIF(RAW) --> ISP    |      ISP(YUV) --> VPS      |      SIF(YUV) --> VPS     |
| Offline | SIF(RAW) --> DDR --> ISP | ISP(YUV) --> DDR --> VPS | SIF(YUV) --> DDR --> ISP |

Note: The HB_SYS_SetVINVPSMode interface is used to set the operation mode between VIN and VPS.

## API Reference

- HB_SYS_Init: Initialize the media system (reserved).
- HB_SYS_Exit: Exclude the media system (reserved).
- HB_SYS_Bind: Bind the data source to the data receiver.
- HB_SYS_UnBind: Unbind the data source from the data receiver.
- HB_SYS_SetVINVPSMode: Set the operation mode between VIN and VPS modules. 
- HB_SYS_GetVINVPSMode: Get the operation mode between VIN and VPS modules for a specified pipeid.
- HB_VP_SetConfig: Set the properties of the Video Pool video pool.
- HB_VP_GetConfig: Get the properties of the Video Pool video pool.
- HB_VP_Init: Initialize the Video Pool video pool.
- HB_VP_Exit: Deinitialize the Video Pool video pool.
- HB_VP_CreatePool: Create a video buffer pool.
- HB_VP_DestroyPool: Destroy a video buffer pool.
- HB_VP_GetBlock: Get a buffer block.
- HB_VP_ReleaseBlock: Release an acquired buffer block.- HB_VP_PhysAddr2Block: Get the buffer block ID from the physical address of the buffer block.
- HB_VP_Block2PhysAddr: Get the physical address of a buffer block.
- HB_VP_Block2PoolId: Get the ID of the cache pool where a cache block is located.
- HB_VP_MmapPool: Map the user space virtual address for a video cache pool.
- HB_VP_MunmapPool: Unmap the user space virtual address for a video cache pool.
- HB_VP_GetBlockVirAddr: Get the user space virtual address of a cache block in a video cache pool.
- HB_VP_InquireUserCnt: Check if a buffer block is in use.
- HB_VP_SetAuxiliaryConfig: Set additional information for a video buffer pool.
- HB_SYS_Alloc: Allocate memory in user space.
- HB_SYS_Free: Free a memory block.
- HB_SYS_AllocCached: Allocate cached memory in user space.
- HB_SYS_CacheInvalidate: Invalidate the cache of the cached memory.
- HB_SYS_CacheFlush: Flush the cache of the cached memory.
- HB_VP_DmaCopy: Copy physical memory through DMA.

### HB_SYS_Init
【Function Declaration】
```c
int HB_SYS_Init(void);
```
【Function Description】
> Reserved interface, currently has no effect.

【Parameter Description】
> None

【Return Value】

| Return Value | Description |
|:------------:|:-----------:|
|      0       |   Success   |
|     Non-zero |   Failure   |

【Notes】
> None

【Reference Code】
> None

### HB_SYS_Exit
【Function Declaration】
```c
int HB_SYS_Exit(void);
```
【Function Description】
> Reserved interface, currently has no effect.

【Parameter Description】
> None【Return Value】

| Returns | Description |
|:------:|:----:|
|    0   | Success |
|   Non-zero  | Failure |

【Notes】
> None

【Reference Code】
> None

### HB_SYS_Bind
【Function Declaration】
```c
int HB_SYS_Bind(const SYS_MOD_S *pstSrcMod, const SYS_MOD_S *pstDstMod);
```
【Function Description】
> Establishes a binding relationship between VIN pipelines, channels, VPS groups/channels, VO channels, and VENC channels.

【Parameter Description】

| Parameter Name | Description | Input/Output |
| :-------: | :----------: | :-------: |
| pstSrcMod | Source module pointer |   Input    |
| pstDstMod | Destination module pointer |   Input    |

【Return Value】

| Returns | Description |
|:------:|:----:|
|    0   | Success |
|   Non-zero  | Failure |

【Notes】
> None

【Reference Code】
> None

### HB_SYS_UnBind
【Function Declaration】
```c
int HB_SYS_UnBind(const SYS_MOD_S *pstSrcMod, const SYS_MOD_S *pstDstMod);
```
【Function Description】
> Unbinds the binding relationship between VIN pipelines, channels, VPS groups/channels, VO channels, and VENC channels.

【Function Description】
> Get the working mode of the VIN, VPS module of the specified pipe ID.

【Parameter Description】

| Parameter Name | Description | Input/Output |
| :------------: | :---------: | :----------: |
|    pipeId      | Pipe number |    Input     |

【Return Value】

| Return Value |       Description       |
| :----------: | :---------------------: |
|     `>=0`      | SYS_VIN_VPS_MODE_E enum |
|      `<0`      |        Failed           |

【Notice】
> None

【Reference Code】
> None

### HB_SYS_SetVINVPSMode
**Function Declaration**
```c
int HB_SYS_SetVINVPSMode(int pipeId, const SYS_VIN_VPS_MODE_E mode);
```
**Function Description**
> Sets the working mode between the VIN and VPS modules.

**Parameter Descriptions**

| Parameter Name |       Description       | Input/Output |
| :-------------: | :---------------------: | :---------: |
|     pipeId      |          Pipe ID         |    Input    |
|       mode      | VIN-VPS operating mode |    Input    |

**Return Values**

| Return Value | Description |
| :---------: | :----------: |
|      0      |     Success    |
| Non-zero   |      Failure   |

**Caution**
> None

**Reference Code**
> None

### HB_SYS_GetVINVPSMode
**Function Declaration**
```c
int HB_SYS_GetVINVPSMode(int pipeId);
```
**Function Description**
> Retrieves the working mode of the VIN and VPS modules for the specified pipe ID.

**Parameter Descriptions**

| Parameter Name |       Description       | Input/Output |
| :-------------: | :---------------------: | :---------: |
|     pipeId      |          Pipe ID         |    Input    |

**Return Values**

| Return Value |                Meaning                 |
| :---------: | :----------------------------------: |
|    `>=0`     | A value of type SYS_VIN_VPS_MODE_E    |
|    `<0`      |                            Failure        |

**Caution**
> None

**Reference Code**
> None

Video Cache Pool

### HB_VP_SetConfig
【Function Declaration】
```c
int HB_VP_SetConfig(VP_CONFIG_S *VpConfig);
```
【Function Description】
> Set the attributes of the Video Pool video buffer pool.

【Parameter Description】

| Parameter Name |          Description          | Input/Output |
| :------------: | :---------------------------: | :----------: |
|    vpConfig    | Pointer to video buffer pool properties |    Input     |

【Return Value】

| Return Value | Description |
|:------------:|:-----------:|
|      0       |    Success  |
|   Non-zero   |    Failed   |

【Notice】
> None【Reference Code】
> VideoPool Reference Code

### HB_VP_GetConfig
【Function Declaration】
```c
int HB_VP_GetConfig(VP_CONFIG_S *VpConfig);
```
【Description】
> Get the properties of the Video Pool video buffer pool.

【Parameter Description】

| Parameter Name |     Description     | Input/Output |
| :------------: | :-----------------: | :----------: |
|    vpConfig    | Video buffer pool property pointer |    Output    |

【Return Value】

| Return Value | Description |
| :----------: | :---------: |
|      0       |   Success   |
|    Non-zero  |   Failure   |

【Note】
> None

【Reference Code】
> None

### HB_VP_Init
【Function Declaration】
```c
int HB_VP_Init(void);
```
【Description】
> Initialize the video buffer pool.

【Parameter Description】
> None

【Return Value】

| Return Value | Description |
| :----------: | :---------: |
|      0       |   Success   |
|    Non-zero  |   Failure   |

【Note】
> It is necessary to call HB_VP_SetConfig to configure the cache pool properties before initializing the cache pool, otherwise it will fail.【Reference code】
> Reference code for VideoPool

### HB_VP_Exit
【Function Declaration】
```c
int HB_VP_Exit(void);
```
【Function Description】
> Uninitialize the video pool

【Parameter Description】
> None

【Return Value】

| Return Value | Description |
|:------:|:----:|
|    0   | Success |
|   Non-zero  | Failure |

【Notes】
> None

【Reference code】
> Reference code for VideoPool

### HB_VP_CreatePool
【Function Declaration】
```c
uint32_t HB_VP_CreatePool(VP_POOL_CONFIG_S *VpPoolCfg);
```
【Function Description】
> Create a video pool

【Parameter Description】

| Parameter | Description | Input/Output |
| :-------: | :--------------------: | :-------: |
| VpPoolCfg | Pointer to cache pool configuration attributes  |   Input    |

【Return Value】

|       Return Value        |       Description       |
| :-----------------: | :--------------: |
| Non-VP_INVALID_POOLID | Valid pool ID number |
|  VP_INVALID_POOLID  |  Failed to create pool  |

【Notes】【Reference code】
> Reference code for VideoPool

### HB_VP_DestroyPool
【Function Declaration】
```c
int HB_VP_DestroyPool(uint32_t Pool);
```
【Function Description】
> Destroy a video buffer pool

【Parameter Description】

| Parameter Name |     Description     | Input/Output |
| :------------: | :-----------------: | :----------: |
|      Pool      | ID of the pool     |    Input     |

【Return Value】

| Return Value | Description |
| :----------: | :---------: |
|       0      |   Success   |
|    Non-zero  |   Failure   |

【Note】
> N/A

【Reference code】
> Reference code for VideoPool

### HB_VP_GetBlock
【Function Declaration】
```c
uint32_t HB_VP_GetBlock(uint32_t Pool, uint64_t u64BlkSize);
```
【Function Description】
> Obtain a buffer block

【Parameter Description】

|  Parameter Name   |    Description     | Input/Output |
| :---------------: | :----------------: | :----------: |
|       Pool        |     ID of pool     |    Input     |
|    u64BlkSize     |    Size of block   |    Input     |

【Return Value】

|    Return Value   |     Description    |### Translation
| :-----------------: | :------------: |
| Non-VP_INVALID_HANDLE | Valid buffer block id |
| VP_INVALID_HANDLE | Failed to get buffer block |

【Notes】
> u64BlkSize must be less than or equal to the buffer block size specified when creating the cache pool

【Reference code】
> VideoPool reference code

### HB_VP_ReleaseBlock
【Function declaration】
```c
int HB_VP_ReleaseBlock(uint32_t Block);
```
【Function description】
> Release a buffer block that has been acquired

【Parameter description】

| Parameter name | Description | Input/Output |
| :------: | :------: | :-------: |
|  Block   | Buffer block id |   Input    |

【Return value】

| Return value | Description |
|:------:|:----:|
|    0   | Success |
|   Non-zero  | Failed |

【Notes】
> None

【Reference code】
> VideoPool reference code

### HB_VP_PhysAddr2Block
【Function declaration】
```c
uint32_t HB_VP_PhysAddr2Block(uint64_t u64PhyAddr);
```
【Function description】
> Get the buffer block id through the physical address of the buffer block

【Parameter description】

|  Parameter name  |      Description      | Input/Output |
| :--------: | :------------: | :-------: |
| u64PhyAddr | Physical address of the buffer block |   Input    |【返回值】

| Return Value | Description |
|:------------:|:-----------:|
|      0       |   Success   |
|     Non-zero      |   Failure   |

【Notes】
> None

【Reference code】
> VideoPool reference code

### HB_VP_Block2PhysAddr
【Function description】
```c
uint64_t HB_VP_Block2PhysAddr(uint32_t Block);
```
【Function description】
> Get the physical address of a buffer block

【Parameter description】

| Parameter Name | Description | Input/Output |
| :------------: | :---------: | :----------: |
|     Block      | Buffer block id |    Input     |

【返回值】

| Return Value |        Description        |
| :----------: | :-----------------------: |
|      0       |   Invalid return value    |
|   Non-zero    |    Valid physical address   |

【Notes】
> None

【Reference code】
> VideoPool reference code

### HB_VP_Block2PoolId
【Function description】
```c
uint32_t HB_VP_Block2PoolId(uint32_t Block);
```
【Function description】
> Get the buffer pool id through the buffer block id

【Parameter description】| Parameter Name | Description | Input/Output |
| :------------: | :---------: | :----------: |
|     Block      | Buffer block ID |    Input     |

【Return Value】

| Return Value |     Description      |
| :----------: | :------------------: |
|  Non-negative integer |  Valid buffer pool ID  |
|    Negative integer   |  Invalid buffer pool ID  |

【Note】
> None

【Reference Code】
> Reference code for VideoPool

### HB_VP_MmapPool
【Function Declaration】
```c
int HB_VP_MmapPool(uint32_t Pool);
```
【Function Description】
> Map user-space virtual address for a buffer pool

【Parameter Description】

| Parameter Name | Description | Input/Output |
| :------------: | :---------: | :----------: |
|     Pool       | Buffer pool ID |    Input     |

【Return Value】

| Return Value |     Description      |
| :----------: | :------------------: |
|       0      |         Success          |
|    Non-zero  |         Failure          |

【Note】
> None

【Reference Code】
> Reference code for VideoPool

### HB_VP_MunmapPool
【Function Declaration】
```c
int HB_VP_MunmapPool(uint32_t Pool);
```

【Function Description】
> Remove user-mode mapping for a buffer pool

【Parameter Description】

| Parameter Name |      Description      | Input/Output |
| :------------: | :------------------: | :----------: |
|      Pool      | Buffer pool ID number |    Input     |

【Return Value】

| Return Value | Description |
| :----------: | :---------: |
|       0      |   Success   |
|     Non-zero     |   Failure   |

【Notes】
> None

【Reference Code】
> Reference code for VideoPool

### HB_VP_GetBlockVirAddr
【Function Declaration】
```c
int HB_VP_GetBlockVirAddr(uint32_t Pool, uint64_t u64PhyAddr, void **ppVirAddr);
```
【Function Description】
> Get the user-mode virtual address of a cache block in a video buffer pool

【Parameter Description】

|  Parameter Name  |      Description       | Input/Output |
| :--------------: | :-------------------: | :----------: |
|       Pool       |   Buffer pool ID number  |    Input     |
|    u64PhyAddr    |   Buffer pool physical address |    Input     |
|    ppVirAddr    |   Buffer pool virtual address |    Output     |

【Return Value】

| Return Value | Description |
| :----------: | :---------: |
|       0      |   Success   |
|     Non-zero     |   Failure   |

【Notes】
> None

【Reference Code】
> Reference code for VideoPool

### HB_VP_InquireUserCnt
【Function Declaration】
```c
int HB_VP_InquireUserCnt(uint32_t Block);
```
【Function Description】
> Check if the buffer block is being used.

【Parameter Description】

| Parameter Name |  Description | Input/Output |
| :------: | :------: | :-------: |
|  Block   | Buffer block ID |   Input    |

【Return Value】

|    Return Value     |   Description   |
| :-----------------: | :------: |
|  VP_INVALID_HANDLE  | Query failed |
| Non-VP_INVALID_HANDLE | Reference count |

【Note】
> None

【Reference Code】
> None

### HB_VP_SetAuxiliaryConfig
【Function Declaration】
```c
int HB_VP_SetAuxiliaryConfig (const VP_AUXILIARY_CONFIG_S *pstAuxiliaryConfig);
```
【Function Description】
> Set the auxiliary configuration of the video buffer pool.

【Parameter Description】

| Parameter Name |             Description             | Input/Output |
| :----------------: | :--------------------------: | :-------: |
| pstAuxiliaryConfig | Configuration structure for auxiliary information of video buffer pool |   Input    |

【Return Value】

| Return Value | Description |
|:------:|:----:|
|    0   | Success |
|   Non-zero  | Failure |

【Note】

### HB_SYS_Alloc
【Function Declaration】
```c
int HB_SYS_Alloc(uint64_t *pu64PhyAddr, void **ppVirAddr, uint32_t u32Len);
```
【Description】
> Allocate memory in user mode.

【Parameter Description】

| Parameter Name | Description | Input/Output |
| :------------: | :---------: | :----------: |
|  pu64PhyAddr  |  Physical address pointer  |    Output    |
|   ppVirAddr   |    Pointer to virtual address    |    Output    |
|    u32Len    |    Size of memory allocation    |    Input    |

【Return Value】

| Return Value | Description |
|:------:|:----:|
|    0   | Success |
|  Non-zero  | Failure |

【Notes】
> Need to call HB_VP_Init to initialize video buffer pool.

【Sample Code】
```c
    ret = HB_SYS_Alloc(&paddr, &vaddr, 0x1000);
    if (ret == 0) {
        printf("Alloc paddr = 0x%x, vaddr = 0x%x\n", paddr, vaddr);
    }
    ret = HB_SYS_Free(paddr, vaddr);
    if (ret == 0) {
        printf("Free ok\n");
    }
```

### HB_SYS_AllocCached
【Function Declaration】
```c
int HB_SYS_AllocCached(uint64_t *pu64PhyAddr, void **ppVirAddr, uint32_t u32Len);
```
【Description】
> Allocate cached memory in user mode.【Parameter Description】

| Parameter Name |       Description        | Input/Output |
| :------------: | :----------------------: | :----------: |
|  pu64PhyAddr   |      Physical Address    |    Output    |
|   ppVirAddr    | Pointer to Virtual Address |    Output    |
|    u32Len      |    Size of Memory Block   |    Input     |

【Return Value】

| Return Value |  Description |
| :----------: | :----------: |
|       0      |     Success  |
|    Non-zero  |    Failure   |

【Note】
> Need to call HB_VP_Init to initialize the video buffer pool

【Reference Code】
> None

### HB_SYS_Free
【Function Declaration】
```c
int HB_SYS_Free(uint64_t u64PhyAddr, void *pVirAddr);
```
【Function Description】
> Free Memory Block

【Parameter Description】

| Parameter Name |    Description     | Input/Output |
| :------------: | :----------------: | :----------: |
|  u64PhyAddr    |    Physical Address  |    Input     |
|   pVirAddr     | Pointer to Virtual Address  |    Input     |

【Return Value】

| Return Value |  Description |
| :----------: | :----------: |
|       0      |     Success  |
|    Non-zero  |    Failure   |

【Note】
> None

【Reference Code】
> Refer to HB_SYS_Alloc

### HB_SYS_CacheInvalidate
**Function Declaration**
```c
int HB_SYS_CacheInvalidate(uint64_t pu64PhyAddr, void *pVirAddr, uint32_t u32Len);
```
**Function Description**
> Invalidates the cache for the given memory region with cache.

**Parameter Descriptions**

| Parameter Name | Description | Input/Output |
| :------------: | :---------: | :--------: |
| pu64PhyAddr | Physical Address | Input |
| pVirAddr | Virtual Address Pointer | Input |
| u32Len | Length | Input |

**Return Values**

| Return Value | Description |
|:-------------:|:-----------:|
| 0 | Success |
| Non-zero | Failure |

**Notes**
> None

**Reference Code**
> N/A

### HB_SYS_CacheFlush
**Function Declaration**
```c
int HB_SYS_CacheFlush(uint64_t pu64PhyAddr, void *pVirAddr, uint32_t u32Len);
```
**Function Description**
> Flushes the cache for the given memory region with cache.

**Parameter Descriptions**

| Parameter Name | Description | Input/Output |
| :------------: | :---------: | :--------: |
| pu64PhyAddr | Physical Address | Input |
| pVirAddr | Virtual Address Pointer | Input |
| u32Len | Length | Input |

**Return Values**

| Return Value | Description |
|:-------------:|:-----------:|
| 0 | Success |
| Non-zero | Failure |

**Notes**
> None

**Reference Code**
> N/A

### HB_VP_DmaCopy
【Function Declaration】
```c
int HB_VP_DmaCopy(void *dstPaddr, void *srcPaddr, uint32_t len);
```
【Function Description】
> Copy physical memory through DMA.

【Parameter Description】

| Parameter Name |     Description     | Input/Output |
| :------------: | :-----------------: | :----------: |
|   dstPaddr     | Destination address |    Input     |
|   srcPaddr     |    Source address   |    Input     |
|      len       |       Length        |    Input     |

【Return Value】

| Return Value | Description |
|:------------:|:-----------:|
|      0       |   Success   |
|   Non-zero   |   Failed    |

【Notes】
> dstPaddr and srcPaddr should be continuous physical addresses.

【Reference Code】
> None

## Data Types
### HB_SYS_MOD_ID_E
**Struct Definition**
```c
typedef enum HB_SYS_MOD_ID_E {
    HB_ID_SYS = 0,
    HB_ID_VIN,
    HB_ID_VOT,
    HB_ID_VPS,
    HB_ID_RGN,
    HB_ID_AIN,
    HB_ID_AOT,
    HB_ID_VENC,
    HB_ID_VDEC,
    HB_ID_AENC,
    HB_ID_ADEC,
    HB_ID_MAX,
} SYS_MOD_ID_E;
```
**Function Description**
> Module identifier.

**Member Descriptions**
> None.

### HB_SYS_MOD_S
**Struct Definition**
```c
typedef struct HB_SYS_MOD_S {
    SYS_MOD_ID_E enModId;
    uint8_t s32DevId;
    uint8_t s32ChnId;
} SYS_MOD_S;
```
**Function Description**
> This structure serves as an abstract index for each module.

**Member Descriptions**

|   Member   | Meaning                                                                                   |
| :--------: | :--------------------------------------------------------------------------------------- |
| enModId    | Module ID number                                                                       |
| s32DevId   | An abstract for the pipeline in multi-channel scenarios, e.g., the nth pipe in VIN, nth group in VPS |
| s32ChnId   | Channel index number                                                                   |

### HB_SYS_VIN_VPS_MODE_E
**Struct Definition**
```c
typedef enum HB_SYS_VIN_VPS_MODE_E {
    VIN_ONLINE_VPS_ONLINE,
    VIN_ONLINE_VPS_OFFLINE,
    VIN_OFFLINE_VPS_ONLINE,
    VIN_OFFLINE_VPS_OFFINE,
    VIN_SIF_VPS_ONLINE,
    VIN_SIF_OFFLINE_ISP_OFFLINE_VPS_ONLINE,
    VIN_SIF_ONLINE_DDR_ISP_DDR_VPS_ONLINE,
    VIN_SIF_ONLINE_DDR_ISP_ONLINE_VPS_ONLINE,
    VIN_FEEDBACK_ISP_ONLINE_VPS_ONLINE,
    VIN_SIF_OFFLINE_VPS_OFFLINE,
    VIN_SIF_OFFLINE,
} SYS_VIN_VPS_MODE_E;
```
**Function Description**
> Represents the online/offline mode of VIN and VPS, as well as the internal working mode of VIN.

**Member Descriptions**

| Member                              | Meaning                                                                                   |
| :--------------------------------- | :--------------------------------------------------------------------------------------- |
| VIN_ONLINE_VPS_ONLINE               | VIN_SIF and VIN_ISP are online, both VIN_ISP and VPS are online                                |
| VIN_ONLINE_VPS_OFFLINE              | VIN_SIF and VIN_ISP are online, while VIN_ISP and VPS are offline                             |
| VIN_OFFLINE_VPS_ONLINE              | VIN_SIF and VIN_ISP are offline, but both VIN_ISP and VPS are online                        |
| VIN_OFFLINE_VPS_OFFLINE             | VIN_SIF and VIN_ISP are offline, both VIN_ISP and VPS are offline                             |
| VIN_SIF_VPS_ONLINE                  | VIN_SIF directly sends data to VPS                                                            |
| VIN_SIF_OFFLINE_ISP_OFFLINE_VPS_ONLINE | VIN_SIF and VIN_ISP are offline, VIN_ISP and VPS are online, with VIN_ISP dumping to DDR        |
| VIN_SIF_ONLINE_DDR_ISP_DDR_VPS_ONLINE | VIN_SIF and VIN_ISP are online, VIN_SIF connected to DDR, while VIN_ISP is offline and VPS is online |
| VIN_SIF_ONLINE_DDR_ISP_ONLINE_VPS_ONLINE | Same as above, but for dumping data from VIN_SIF                                              |
| VIN_FEEDBACK_ISP_ONLINE_VPS_ONLINE   | VIN_SIF operates in feedback raw mode                                                            |
| VIN_SIF_OFFLINE_VPS_OFFLINE          | VIN_SIF and VPS are offline, typically for YUV to IPU conversion                            |
| VIN_SIF_OFFLINE                     | VIN_SIF directly connects to DDR                                                               |


### HB_VP_POOL_CONFIG_S
[Structure Definition]
```c
typedef struct HB_VP_POOL_CONFIG_S {
    uint64_t u64BlkSize;
    uint32_t u32BlkCnt;
    uint32_t cacheEnable;
} VP_POOL_CONFIG_S;
```
[Function Description]
> Video buffer pool configuration structure

[Member Explanation]

|   Member    | Meaning               |
| :---------: | :--------------------- |
| u64BlkSize  | Size of buffer blocks  |
|  u32BlkCnt  | Number of blocks per pool |
| cacheEnable | Whether cache is enabled for the pool |

### HB_VP_CONFIG_S
[Structure Definition]
```c
struct HB_VP_CONFIG_S {
    uint32_t u32MaxPoolCnt;
    VP_POOL_CONFIG_S pubPool[VP_MAX_PUB_POOLS];
} VP_CONFIG_S;
```
[Function Description]
> Video buffer pool attribute structure

[Member Explanation]|    Member   | Meaning                  |
| :---------: | :----------------------- |
| u32MaxPoolCnt | Number of buffer pools that can be accommodated in the entire system |
|   pubPool   | Structure defining the properties of a public buffer pool           |

### HB_VP_AUXILIARY_CONFIG_S
【Structure Definition】
```c
typedef struct HB_VP_AUXILIARY_CONFIG_S {
    int u32AuxiliaryConfig;
} VP_AUXILIARY_CONFIG_S;
```
【Functional Description】
> Structure for configuring additional information for video buffer pools

【Member Description】

|     Member      | Meaning       |
| :-------------: | :------------ |
| AuxiliaryConfig | Additional information type |

### hb_vio_buffer_t
【Structure Definition】
```c
typedef struct hb_vio_buffer_s {
    image_info_t img_info;
    address_info_t img_addr;
} hb_vio_buffer_t;
```

【Functional Description】
> Structure for regular buffer information, representing one frame of image

【Member Description】

|    Member   | Meaning           |
| :---------: | :---------------- |
| img_info | Image data information |
| img_addr | Image address information |
            |
### pym_buffer_t
**Structure Definition:**
```c
typedef struct pym_buffer_s {
    image_info_t pym_img_info;
    address_info_t pym[6];
    address_info_t pym_roi[6][3];
    address_info_t us[6];
    char *addr_whole[HB_VIO_BUFFER_MAX_PLANES];
    uint64_t paddr_whole[HB_VIO_BUFFER_MAX_PLANES];
    uint32_t layer_size[30][HB_VIO_BUFFER_MAX_PLANES];
} pym_buffer_t;
```
**Function Description:**
> Pyramid buffer structure

**Member Descriptions:**

| Member           | Meaning                                                                                     |
| :---------------: | :--------------------------------------------------------------------------------------------- |
| pym_img_info      | Pyramid data information                                                                          |
| pym               | Base-level pyramid data addresses, corresponding to s0, s4, s8, s16, s20, and s24                  |
| pym_roi           | ROI data addresses for the base-level pyramid, e.g., pym_roi[0][0] corresponds to s0's s1, etc. |
| us                | Addresses for the 6 US channel output data                                                           |
| addr_whole        | Virtual address of the entire pyramid buffer's first address                                      |
| paddr_whole       | Physical address of the entire pyramid buffer's first address                                      |
| layer_size        | Data size for each layer of the pyramid                                                                |

### image_info_t
**Structure Definition:**
```c
typedef struct image_info_s {
    uint16_t sensor_id;
    uint32_t pipeline_id;
    uint32_t frame_id;
    uint64_t time_stamp;
    struct timeval tv;
    int buf_index;
    int img_format;
    int fd[HB_VIO_BUFFER_MAX_PLANES];
    uint32_t size[HB_VIO_BUFFER_MAX_PLANES];
    uint32_t planeCount;
    uint32_t dynamic_flag;
    uint32_t water_mark_line;
    VIO_DATA_TYPE_E data_type;
    buffer_state_e state;
} image_info_t;
```
**Function Description:**
> Image information structure

**Member Descriptions:**

| Member          | Meaning                                                                                     |
| :--------------: | :--------------------------------------------------------------------------------------------- |
| sensor_id        | Sensor ID                                                                                        |
| pipeline_id      | Channel number for the corresponding data                                                                  |
| frame_id         | Data frame number                                                                                 |
| time_stamp       | HW timestamp (SIF internal hardware time, updated during each FS, not related to system time)   |
| tv               | System time of HAL getting the buffer, the system time when SIF records at the start of a frame |
| buf_index        | Index of the acquired buffer                                                                          |
| img_format       | Image format                                                                                        |
| fd               | ION buffer file descriptor                                                                            |
| size             | Size of each plane                                                                                  |
| planeCount       | Number of planes in the image                                                                         |
| dynamic_flag     | Flag for dynamically changing size                                                                     |
| water_mark_line | Advanced watermark line, unsupported on XJ3                                                               |
| data_type        | Image data type                                                                                     |
| state            | Buffer state, user state in the user layer                                                            |

### address_info_t
**Structure Definition:**
```c
typedef struct address_info_s {
    uint16_t width;
    uint16_t height;
    uint16_t stride_size;
    char *addr[HB_VIO_BUFFER_MAX_PLANES];
    uint64_t paddr[HB_VIO_BUFFER_MAX_PLANES];
} address_info_t;
```
**Function Description:**
> Image address information structure

**Member Descriptions:**

| Member     | Meaning                                       |
| :---------: | :----------------------------------------- |
| width      | Image data width                               |
| height     | Image data height                               |
| stride_size | Image data memory stride (width of one row in memory) |
| addr       | Virtual addresses, stored per YUV plane         |
| paddr      | Physical addresses, stored per YUV plane         |

**Error Codes:**

| Error Code | Macro Definition | Description                                                                                   |
| :--------: | :-------------: | :----------------------------------------------------------------------------------------------- |
| -268500032 | VP_INVALID_BLOCKID | Invalid buffer block ID                                                                       |
| -268500033 | VP_INVALID_POOLID  | Invalid buffer pool ID                                                                        |
| -268500034 | HB_ERR_VP_NOT_PERM | Operation not allowed                                                                           |
| -268500035 | HB_ERR_VP_UNEXIST | Video buffer pool does not exist                                                                |
| -268500036 | HB_ERR_VP_BUSY    | Buffer pool is busy                                                                             |
| -268500037 | HB_ERR_SYS_BUSY   | System is busy                                                                                  |
| -268500038 | HB_ERR_SYS_ILLEGAL_PARAM | System interface illegal parameter                                                            |
| -268500039 | HB_ERR_SYS_NOMEM | System interface failed to allocate memory                                                      |
| -268500040 | HB_ERR_VP_ILLEGAL_PARAM | Invalid parameter set for buffer pool interface                                                 |