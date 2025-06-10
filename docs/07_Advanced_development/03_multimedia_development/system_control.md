---
sidebar_position: 3
---

# 7.3.3 系统控制
## 概述
系统控制用来初始化和去初始化整个媒体系统，通过绑定接口建立各模块间的关系。提供大块物理内存分配管理的VP（Video Pool）模块。

## 功能描述

### 视频缓冲池

VP（Video Pool）视频缓冲池提供大块物理内存及管理功能，负责内存的分配和回收。
视频缓冲池由一组物理地址连续，大小相同的缓冲块组成，在使用前需要配置及初始化，可根据使用需要，配置不同数量的缓冲池和调整缓冲块的大小。

### 绑定关系

![image-20220329183230983](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/03_multimedia_development/system_control/system_control_bind_relationship.png)

注：通过HB_SYS_Bind接口可以在模块间建立绑定关系，绑定后数据源处理完成的数据会自动发送给数据端。

### 工作模式

**在线模式：** 模块间的数据通过内部总线直接从上一模块传输给下一模块，不需要读写DDR，可以降低延时，节省DDR带宽

**离线模式：** 上一模块的数据先写入DDR，下一模块再从DDR中读取数据，多于一路sensor接入时，所有接入sensor都按离线处理。

| 模式 |     VIN_SIF和VIN_ISP     |       VIN_ISP和VPS        |       VIN_SIF和VPS       |
| :--: | :----------------------: | :-----------------------: | :----------------------: |
| 在线 |     SIF(RAW) --> ISP     |     ISP(YUV) --> VPS      |     SIF(YUV) --> VPS     |
| 离线 | SIF(RAW) --> DDR --> ISP | ISP(YUV) --> DDR  --> VPS | SIF(YUV) --> DDR --> ISP |

注：HB_SYS_SetVINVPSMode接口用来设定VIN和VPS间的工作模式。

## API参考

- HB_SYS_Init : 初始化媒体系统（预留）。
- HB_SYS_Exit : 初始化媒体系统（预留）。
- HB_SYS_Bind : 数据源到数据接收者绑定。
- HB_SYS_UnBind : 数据源到数据接收者解绑定。
- HB_SYS_SetVINVPSMode : 设置VIN，VPS模块间的工作模式。 
- HB_SYS_GetVINVPSMode : 获取指定pipeid的VIN，VPS模块间工作模式。
- HB_VP_SetConfig :  设置Video Pool视频缓冲池属性。
- HB_VP_GetConfig  :  获取Video Pool视频缓冲池属性。
- HB_VP_Init :  初始化Video Pool视频缓冲池。
- HB_VP_Exit :  去初始化Video Pool视频缓冲池。
- HB_VP_CreatePool :  创建一个视频缓冲池。
- HB_VP_DestroyPool :  销毁一个视频缓存池。
- HB_VP_GetBlock :  获取一个缓存块。
- HB_VP_ReleaseBlock :  释放一个已经获取的缓存块。
- HB_VP_PhysAddr2Block :  通过缓冲块物理地址获取缓冲块id
- HB_VP_Block2PhysAddr :  获取一个缓冲块的物理地址
- HB_VP_Block2PoolId :  获取一个缓存块所在缓存池的 ID 。
- HB_VP_MmapPool :  为一个视频缓存池映射用户态虚拟地址。
- HB_VP_MunmapPool :  为一个视频缓存池解除用户态映射。
- HB_VP_GetBlockVirAddr :  获取一个视频缓存池中的缓存块的用户态虚拟地址。
- HB_VP_InquireUserCnt :  查询缓冲块是否使用。
- HB_VP_SetAuxiliaryConfig  :  设置视频缓冲池的附加信息。
- HB_SYS_Alloc :  在用户态分配内存。
- HB_SYS_Free :  释放内存块。
- HB_SYS_AllocCached :  在用户态分配带cache内存。
- HB_SYS_CacheInvalidate :  设置该带cache的内存cache无效。
- HB_SYS_CacheFlush :  刷新该带cache的内存cache。
- HB_VP_DmaCopy :  通过DMA拷贝物理内存。

### HB_SYS_Init
【函数声明】
```c
int HB_SYS_Init(void);
```
【功能描述】
> 预留接口，目前无作用。

【参数描述】
> 无

【返回值】

| 返回值 | 描述 |
|:------:|:----:|
|    0   | 成功 |
|   非0  | 失败 |

【注意事项】
> 无

【参考代码】
> 无

### HB_SYS_Exit
【函数声明】
```c
int HB_SYS_Exit(void);
```
【功能描述】
> 预留接口，目前无作用。

【参数描述】
> 无

【返回值】

| 返回值 | 描述 |
|:------:|:----:|
|    0   | 成功 |
|   非0  | 失败 |

【注意事项】
> 无

【参考代码】
> 无

### HB_SYS_Bind
【函数声明】
```c
int HB_SYS_Bind(const SYS_MOD_S *pstSrcMod, const SYS_MOD_S *pstDstMod);
```
【功能描述】
> 在VIN管道，通道，VPS组/通道，VO通道，VENC通道之间建立绑定关系。

【参数描述】

| 参数名称  |     描述     | 输入/输出 |
| :-------: | :----------: | :-------: |
| pstSrcMod |  源模块指针  |   输入    |
| pstDstMod | 目的模块指针 |   输入    |

【返回值】

| 返回值 | 描述 |
|:------:|:----:|
|    0   | 成功 |
|   非0  | 失败 |

【注意事项】
> 无

【参考代码】
> 无

### HB_SYS_UnBind
【函数声明】
```c
int HB_SYS_UnBind(const SYS_MOD_S *pstSrcMod, const SYS_MOD_S *pstDstMod);
```
【功能描述】
> 在VIN管道，通道，VPS组/通道，VO通道，VENC通道之间解除绑定关系。

【参数描述】

| 参数名称  |     描述     | 输入/输出 |
| :-------: | :----------: | :-------: |
| pstSrcMod |  源模块指针  |   输入    |
| pstDstMod | 目的模块指针 |   输入    |

【返回值】

| 返回值 | 描述 |
|:------:|:----:|
|    0   | 成功 |
|   非0  | 失败 |

【注意事项】
> 无

【参考代码】
> 无

### HB_SYS_SetVINVPSMode
【函数声明】
```c
int HB_SYS_SetVINVPSMode(int pipeId, const SYS_VIN_VPS_MODE_E mode);
```
【功能描述】
> 设置VIN，VPS模块间的工作模式。

【参数描述】

| 参数名称 |       描述       | 输入/输出 |
| :------: | :--------------: | :-------: |
|  pipeId  |      Pipe号      |   输入    |
|   mode   | VIN，VPS工作模式 |   输入    |

【返回值】

| 返回值 | 描述 |
|:------:|:----:|
|    0   | 成功 |
|   非0  | 失败 |

【注意事项】
> 无

【参考代码】
> 无

### HB_SYS_GetVINVPSMode
【函数声明】
```c
int HB_SYS_GetVINVPSMode(int pipeId);
```
【功能描述】
> 获取指定pipeid的VIN，VPS模块间工作模式。

【参数描述】

| 参数名称 |  描述  | 输入/输出 |
| :------: | :----: | :-------: |
|  pipeId  | Pipe号 |   输入    |

【返回值】

| 返回值 |        描述        |
| :----: | :----------------: |
| `>= 0` | `SYS_VIN_VPS_MODE_E` |
| `< 0`  |        失败        |

【注意事项】
> 无

【参考代码】
> 无


视频缓存池

### HB_VP_SetConfig
【函数声明】
```c
int HB_VP_SetConfig(VP_CONFIG_S *VpConfig);
```
【功能描述】
> 设置Video Pool视频缓冲池属性

【参数描述】

| 参数名称 |        描述        | 输入/输出 |
| :------: | :----------------: | :-------: |
| vpConfig | 视频缓冲池属性指针 |   输入    |

【返回值】

| 返回值 | 描述 |
|:------:|:----:|
|    0   | 成功 |
|   非0  | 失败 |

【注意事项】
> 无

【参考代码】
> VideoPool参考代码

### HB_VP_GetConfig
【函数声明】
```c
int HB_VP_GetConfig (VP_CONFIG_S *VpConfig);
```
【功能描述】
> 获取Video Pool视频缓冲池属性

【参数描述】

| 参数名称 |        描述        | 输入/输出 |
| :------: | :----------------: | :-------: |
| vpConfig | 视频缓冲池属性指针 |   输出    |

【返回值】

| 返回值 | 描述 |
|:------:|:----:|
|    0   | 成功 |
|   非0  | 失败 |

【注意事项】
> 无

【参考代码】
> 无

### HB_VP_Init
【函数声明】
```c
int HB_VP_Init(void);
```
【功能描述】
> 初始化视频缓冲池

【参数描述】
> 无

【返回值】

| 返回值 | 描述 |
|:------:|:----:|
|    0   | 成功 |
|   非0  | 失败 |

【注意事项】
> 必须先调用 HB_VP_SetConfig 配置缓存池属性，再初始化缓存池，否则会失败。

【参考代码】
> VideoPool参考代码

### HB_VP_Exit
【函数声明】
```c
int HB_VP_Exit(void);
```
【功能描述】
> 去初始化视频缓冲池

【参数描述】
> 无

【返回值】

| 返回值 | 描述 |
|:------:|:----:|
|    0   | 成功 |
|   非0  | 失败 |

【注意事项】
> 无

【参考代码】
> VideoPool参考代码

### HB_VP_CreatePool
【函数声明】
```c
uint32_t HB_VP_CreatePool(VP_POOL_CONFIG_S *VpPoolCfg);
```
【功能描述】
> 创建一个视频缓冲池

【参数描述】

| 参数名称  |          描述          | 输入/输出 |
| :-------: | :--------------------: | :-------: |
| VpPoolCfg | 缓冲池配置属性参数指针 |   输入    |

【返回值】

|       返回值        |       描述       |
| :-----------------: | :--------------: |
| 非VP_INVALID_POOLID | 有效的缓冲池ID号 |
|  VP_INVALID_POOLID  |  创建缓冲池失败  |

【注意事项】
> 无

【参考代码】
> VideoPool参考代码

### HB_VP_DestroyPool
【函数声明】
```c
int HB_VP_DestroyPool(uint32_t Pool);
```
【功能描述】
> 销毁一个视频缓冲池

【参数描述】

| 参数名称 |     描述     | 输入/输出 |
| :------: | :----------: | :-------: |
|   Pool   | 缓冲池的id号 |   输入    |

【返回值】

| 返回值 | 描述 |
|:------:|:----:|
|    0   | 成功 |
|   非0  | 失败 |

【注意事项】
> 无

【参考代码】
> VideoPool参考代码

### HB_VP_GetBlock
【函数声明】
```c
uint32_t HB_VP_GetBlock(uint32_t Pool, uint64_t u64BlkSize);
```
【功能描述】
> 获取一个缓冲块

【参数描述】

|  参数名称  |    描述    | 输入/输出 |
| :--------: | :--------: | :-------: |
|    Pool    | 缓冲池id号 |   输入    |
| u64BlkSize | 缓冲块大小 |   输入    |

【返回值】

|       返回值        |      描述      |
| :-----------------: | :------------: |
| 非VP_INVALID_HANDLE | 有效的缓冲块id |
|  VP_INVALID_HANDLE  | 获取缓冲块失败 |

【注意事项】
> u64BlkSize 须小于或等于创建该缓存池时指定的缓存块大小

【参考代码】
> VideoPool参考代码

### HB_VP_ReleaseBlock
【函数声明】
```c
int HB_VP_ReleaseBlock(uint32_t Block);
```
【功能描述】
> 释放一个已经获取的缓冲块

【参数描述】

| 参数名称 |   描述   | 输入/输出 |
| :------: | :------: | :-------: |
|  Block   | 缓冲块id |   输入    |

【返回值】

| 返回值 | 描述 |
|:------:|:----:|
|    0   | 成功 |
|   非0  | 失败 |

【注意事项】
> 无

【参考代码】
> VideoPool参考代码

### HB_VP_PhysAddr2Block
【函数声明】
```c
uint32_t HB_VP_PhysAddr2Block(uint64_t u64PhyAddr);
```
【功能描述】
> 通过缓冲块物理地址获取缓冲块id

【参数描述】

|  参数名称  |      描述      | 输入/输出 |
| :--------: | :------------: | :-------: |
| u64PhyAddr | 缓冲块物理地址 |   输入    |

【返回值】

| 返回值 | 描述 |
|:------:|:----:|
|    0   | 成功 |
|   非0  | 失败 |

【注意事项】
> 无

【参考代码】
> VideoPool参考代码

### HB_VP_Block2PhysAddr
【函数声明】
```c
uint64_t HB_VP_Block2PhysAddr(uint32_t Block);
```
【功能描述】
> 获取一个缓冲块的物理地址

【参数描述】

| 参数名称 |   描述   | 输入/输出 |
| :------: | :------: | :-------: |
|  Block   | 缓冲块id |   输入    |

【返回值】

| 返回值 |     描述     |
| :----: | :----------: |
|   0    |  无效返回值  |
|  非0   | 有效物理地址 |

【注意事项】
> 无

【参考代码】
> VideoPool参考代码

### HB_VP_Block2PoolId
【函数声明】
```c
uint32_t HB_VP_Block2PoolId(uint32_t Block);
```
【功能描述】
> 通过缓冲块id获取缓冲池id

【参数描述】

| 参数名称 |   描述   | 输入/输出 |
| :------: | :------: | :-------: |
|  Block   | 缓冲块id |   输入    |

【返回值】

| 返回值 |      描述      |
| :----: | :------------: |
| 非负数 | 有效的缓冲池id |
|  负数  | 无效的缓冲池id |

【注意事项】
> 无

【参考代码】
> VideoPool参考代码

### HB_VP_MmapPool
【函数声明】
```c
int HB_VP_MmapPool(uint32_t Pool);
```
【功能描述】
> 为一个缓冲池映射用户态虚拟地址

【参数描述】

| 参数名称 |    描述    | 输入/输出 |
| :------: | :--------: | :-------: |
|   Pool   | 缓冲池id号 |   输入    |

【返回值】

| 返回值 | 描述 |
|:------:|:----:|
|    0   | 成功 |
|   非0  | 失败 |

【注意事项】
> 无

【参考代码】
> VideoPool参考代码

### HB_VP_MunmapPool
【函数声明】
```c
int HB_VP_MunmapPool(uint32_t Pool);
```
【功能描述】
> 为一个缓冲池去除用户态映射

【参数描述】

| 参数名称 |    描述    | 输入/输出 |
| :------: | :--------: | :-------: |
|   Pool   | 缓冲池id号 |   输入    |

【返回值】

| 返回值 | 描述 |
|:------:|:----:|
|    0   | 成功 |
|   非0  | 失败 |

【注意事项】
> 无

【参考代码】
> VideoPool参考代码

### HB_VP_GetBlockVirAddr
【函数声明】
```c
int HB_VP_GetBlockVirAddr(uint32_t Pool, uint64_t u64PhyAddr, void **ppVirAddr);
```
【功能描述】
> 获取一个视频缓存池中的缓存块的用户态虚拟地址

【参数描述】

|  参数名称  |      描述      | 输入/输出 |
| :--------: | :------------: | :-------: |
|    Pool    |   缓冲池id号   |   输入    |
| u64PhyAddr | 缓冲池物理地址 |   输入    |
| ppVirAddr  | 缓冲池虚拟地址 |   输出    |

【返回值】

| 返回值 | 描述 |
|:------:|:----:|
|    0   | 成功 |
|   非0  | 失败 |

【注意事项】
> 无

【参考代码】
> VideoPool参考代码

### HB_VP_InquireUserCnt
【函数声明】
```c
int HB_VP_InquireUserCnt(uint32_t Block);
```
【功能描述】
> 查询缓冲块是否使用

【参数描述】

| 参数名称 |   描述   | 输入/输出 |
| :------: | :------: | :-------: |
|  Block   | 缓冲块id |   输入    |

【返回值】

|       返回值        |   描述   |
| :-----------------: | :------: |
|  VP_INVALID_HANDLE  | 查询失败 |
| 非VP_INVALID_HANDLE | 引用计数 |

【注意事项】
> 无

【参考代码】
> 无

### HB_VP_SetAuxiliaryConfig
【函数声明】
```c
int HB_VP_SetAuxiliaryConfig (const VP_AUXILIARY_CONFIG_S *pstAuxiliaryConfig);
```
【功能描述】
> 设置视频缓冲池的附加信息

【参数描述】

|      参数名称      |             描述             | 输入/输出 |
| :----------------: | :--------------------------: | :-------: |
| pstAuxiliaryConfig | 视频缓冲池附加信息配置结构体 |   输入    |

【返回值】

| 返回值 | 描述 |
|:------:|:----:|
|    0   | 成功 |
|   非0  | 失败 |

【注意事项】
> 无

【参考代码】
> 无

### HB_SYS_Alloc
【函数声明】
```c
int HB_SYS_Alloc(uint64_t *pu64PhyAddr, void **ppVirAddr, uint32_t u32Len);
```
【功能描述】
> 在用户态分配内存

【参数描述】

|  参数名称   |        描述        | 输入/输出 |
| :---------: | :----------------: | :-------: |
| pu64PhyAddr |    物理地址指针    |   输出    |
|  ppVirAddr  | 指向虚拟地址的指针 |   输出    |
|   u32Len    |   申请内存的大小   |   输入    |

【返回值】

| 返回值 | 描述 |
|:------:|:----:|
|    0   | 成功 |
|   非0  | 失败 |

【注意事项】
> 需要调用HB_VP_Init初始化视频缓冲池

【参考代码】
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
【函数声明】
```c
int HB_SYS_AllocCached(uint64_t *pu64PhyAddr, void **ppVirAddr, uint32_t u32Len);
```
【功能描述】
> 在用户态分配带cache内存

【参数描述】

|  参数名称   |        描述        | 输入/输出 |
| :---------: | :----------------: | :-------: |
| pu64PhyAddr |    物理地址指针    |   输出    |
|  ppVirAddr  | 指向虚拟地址的指针 |   输出    |
|   u32Len    |   申请内存的大小   |   输入    |

【返回值】

| 返回值 | 描述 |
|:------:|:----:|
|    0   | 成功 |
|   非0  | 失败 |

【注意事项】
> 需要调用HB_VP_Init初始化视频缓冲池

【参考代码】
> 无

### HB_SYS_Free
【函数声明】
```c
int HB_SYS_Free(uint64_t u64PhyAddr, void *pVirAddr);
```
【功能描述】
> 释放内存块

【参数描述】

|  参数名称  |     描述     | 输入/输出 |
| :--------: | :----------: | :-------: |
| u64PhyAddr |   物理地址   |   输入    |
|  pVirAddr  | 虚拟地址指针 |   输入    |

【返回值】

| 返回值 | 描述 |
|:------:|:----:|
|    0   | 成功 |
|   非0  | 失败 |

【注意事项】
> 无

【参考代码】
> 参考HB_SYS_Alloc

### HB_SYS_CacheInvalidate
【函数声明】
```c
int HB_SYS_CacheInvalidate(uint64_t pu64PhyAddr, void *pVirAddr, uint32_t u32Len);
```
【功能描述】
> 设置该带cache的内存cache无效。

【参数描述】

|  参数名称   |     描述     | 输入/输出 |
| :---------: | :----------: | :-------: |
| pu64PhyAddr |   物理地址   |   输入    |
|  pVirAddr   | 虚拟地址指针 |   输入    |
|   u32Len    |     长度     |   输入    |

【返回值】

| 返回值 | 描述 |
|:------:|:----:|
|    0   | 成功 |
|   非0  | 失败 |

【注意事项】
> 无

【参考代码】
> 无

### HB_SYS_CacheFlush
【函数声明】
```c
int HB_SYS_CacheFlush(uint64_t pu64PhyAddr, void *pVirAddr, uint32_t u32Len);
```
【功能描述】
> 刷新该带cache的内存cache。

【参数描述】

| 参数名称 |     描述     | 输入/输出 |
| :------: | :----------: | :-------: |
|    p     |  u64PhyAddr  | 物理地址  | 输入 |
| pVirAddr | 虚拟地址指针 |   输入    |
|  u32Len  |     长度     |   输入    |

【返回值】

| 返回值 | 描述 |
|:------:|:----:|
|    0   | 成功 |
|   非0  | 失败 |

【注意事项】
> 无

【参考代码】
> 无

### HB_VP_DmaCopy
【函数声明】
```c
int HB_VP_DmaCopy(void *dstPaddr, void *srcPaddr, uint32_t len);
```
【功能描述】
> 通过DMA拷贝物理内存。

【参数描述】

| 参数名称 |     描述     | 输入/输出 |
| :------: | :----------: | :-------: |
| dstPaddr | 目的物理地址 |   输入    |
| srcPaddr |  源物理地址  |   输入    |
|   len    |     长度     |   输入    |

【返回值】

| 返回值 | 描述 |
|:------:|:----:|
|    0   | 成功 |
|   非0  | 失败 |

【注意事项】
> dstPaddr、srcPaddr需要是连续的物理地址

【参考代码】
> 无

## 数据类型
### HB_SYS_MOD_ID_E
【结构定义】
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
【功能描述】
> 模块的ID号。

【成员说明】
> 无。

### HB_SYS_MOD_S
【结构定义】
```c
typedef struct HB_SYS_MOD_S {
    SYS_MOD_ID_E enModId;
    uint8_t s32DevId;
    uint8_t s32ChnId;
} SYS_MOD_S;
```
【功能描述】
> 该结构体是各模块索引的一种抽象。

【成员说明】

|   成员   | 含义                                                                        |
| :------: | :-------------------------------------------------------------------------- |
| enModId  | 模块ID号                                                                    |
| s32DevId | 多路时各模块pipeline的抽象，如在VIN中表示第几个pipe，在VPS中表示第几个group |
| s32ChnId | 通道索引号                                                                  |

### HB_SYS_VIN_VPS_MODE_E
【结构定义】
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
【功能描述】
> 表示VIN和VPS的在线与离线模式和VIN内部的工作模式。

【成员说明】

| 成员                                     | 含义                                                                                   |
| :--------------------------------------- | :------------------------------------------------------------------------------------- |
| VIN_ONLINE_VPS_ONLINE                    | VIN_SIF和VIN_ISP在线，VIN_ISP和VPS在线                                                 |
| VIN_ONLINE_VPS_OFFLINE                   | VIN_SIF和VIN_ISP在线，VIN_ISP和VPS离线                                                 |
| VIN_OFFLINE_VPS_ONLINE                   | VIN_SIF和VIN_ISP离线，VIN_ISP和VPS在线                                                 |
| VIN_OFFLINE_VPS_OFFINE                   | VIN_SIF和VIN_ISP离线，VIN_ISP和VPS离线                                                 |
| VIN_SIF_VPS_ONLINE                       | VIN_SIF直接在线发送数据到VPS                                                           |
| VIN_SIF_OFFLINE_ISP_OFFLINE _VPS_ONLINE   | VIN_SIF和VIN_ISP离线，VIN_ISP和VPS在线，同时VIN_ISP到DDR,一般用来dump VIN_ISP的图      |
| VIN_SIF_ONLINE_DDR_ISP_DDR _VPS_ONLINE    | VIN_SIF和VIN_ISP在线，同时VIN_SIF到DDR，VIN_ISP和VPS离线                               |
| VIN_SIF_ONLINE_DDR_ISP_ONL INE_VPS_ONLINE | VIN_SIF和VIN_ISP在线，VIN_ISP和VPS在线，同时VIN_SIF到DDR，一般用来dump VIN_SIF出来的图 |
| VIN_FEEDBACK_ISP_ONLINE _VPS_ONLINE       | VIN_SIF回灌raw模式                                                                     |
| VIN_SIF_OFFLINE_VPS_OFFLINE              | VIN_SIF和VPS离线，一般是YUV到IPU                                                       |
| VIN_SIF_OFFLINE                          | VIN_SIF直接到DDR                                                                       |

### HB_VP_POOL_CONFIG_S
【结构定义】
```c
typedef struct HB_VP_POOL_CONFIG_S {
    uint64_t u64BlkSize;
    uint32_t u32BlkCnt;
    uint32_t cacheEnable;
} VP_POOL_CONFIG_S;
```
【功能描述】
> 视频缓冲池配置结构体

【成员说明】

|    成员     | 含义                   |
| :---------: | :--------------------- |
| u64BlkSize  | 缓冲块大小             |
|  u32BlkCnt  | 每个缓冲池的缓冲块个数 |
| cacheEnable | 缓冲池是否使能cache    |

### HB_VP_CONFIG_S
【结构定义】
```c
struct HB_VP_CONFIG_S {
    uint32_t u32MaxPoolCnt;
    VP_POOL_CONFIG_S pubPool[VP_MAX_PUB_POOLS];
} VP_CONFIG_S;
```
【功能描述】
> 视频缓冲池属性结构体

【成员说明】

|     成员      | 含义                           |
| :-----------: | :----------------------------- |
| u32MaxPoolCnt | 整个系统中可以容纳缓冲池的个数 |
|    pubPool    | 公共缓冲池属性结构体           |

### HB_VP_AUXILIARY_CONFIG_S
【结构定义】
```c
typedef struct HB_VP_AUXILIARY_CONFIG_S {
    int u32AuxiliaryConfig;
} VP_AUXILIARY_CONFIG_S;
```
【功能描述】
> 视频缓冲池附加信息的配置结构体

【成员说明】

|      成员       | 含义         |
| :-------------: | :----------- |
| AuxiliaryConfig | 附加信息类型 |

### hb_vio_buffer_t
【结构定义】
```c
typedef struct hb_vio_buffer_s {
    image_info_t img_info;
    address_info_t img_addr;
} hb_vio_buffer_t;
```

【功能描述】
> 普通buf信息结构体，一个结构体表示一帧图像

【成员说明】

|   成员   | 含义         |
| :------: | :----------- |
| img_info | 图像数据信息 |
| img_addr | 图像地址信息 |

### pym_buffer_t
【结构定义】
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
【功能描述】
> 金字塔buf结构体

【成员说明】

|     成员     | 含义                                                                                                              |
| :----------: | :---------------------------------------------------------------------------------------------------------------- |
| pym_img _info | 金字塔数据信息                                                                                                    |
|     pym      | 金字塔基础层数据地址，对应s0 s4 s8 s16 s20 s24                                                                    |
|   pym_roi    | 金字塔基础层关联的roi层数据地址信息pym_roi[0][0]对应s0下的s1 pym_roi[0][1]对应s0下的s2, pym_roi[0][2]对应s0下的s3 |
|      us      | 金字塔6个us通道输出数据地址信息                                                                                   |
|  addr_whole  | 金字塔整块buf虚拟地址首地址                                                                                       |
| paddr_whole  | 金字塔整块buf物理地址首地址                                                                                       |
|  layer_size  | 每一层的数据大小                                                                                                  |

### image_info_t
【结构定义】
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
【功能描述】
> 图像信息结构体

【成员说明】

|      成员       | 含义                                                         |
| :-------------: | :----------------------------------------------------------- |
|    sensor_id    | Sensor id                                                    |
|   pipeline_id   | 对应数据通道号                                               |
|    frame_id     | 数据帧号                                                     |
|   time_stamp    | HW time stamp，sif内部硬件时间，每次FS的时候更新，和系统时间没关系，一般用来同步 |
|       tv        | system time of hal get buf，sif在framestart打的系统时间      |
|    buf_index    | 获取到的Buffer索引                                           |
|   img_format    | 图像格式                                                     |
|       fd        | ion buf fd                                                   |
|      size       | 对应plane的size                                              |
|   planeCount    | image 含有的plane 数目                                       |
|  dynamic_flag   | 动态改变size标记                                             |
| water_mark_line | 提前水位信息，XJ3不支持                                      |
|    data_type    | image的数据类型                                              |
|      state      | buf的状态，在用户层则是user状态                              |

### address_info_t
【结构定义】
```c
typedef struct address_info_s {
    uint16_t width;
    uint16_t height;
    uint16_t stride_size;
    char *addr[HB_VIO_BUFFER_MAX_PLANES];
    uint64_t paddr[HB_VIO_BUFFER_MAX_PLANES];
} address_info_t;
```
【功能描述】
> 图像地址信息结构体

【成员说明】

|    成员     | 含义                                       |
| :---------: | :----------------------------------------- |
|    width    | 图像数据宽度                               |
|   height    | 图像数据高度                               |
| stride_size | 图像数据内存stride(实际内存存储的一行宽度) |
|    addr     | 虚拟地址，按照yuv plane数存放              |
|    paddr    | 物理地址，按照yuv plane数存放              |

## 错误码

|   错误码   |          宏定义          |          描述          |
| :--------: | :----------------------: | :--------------------: |
| -268500032 |    VP_INVALID_BLOCKID    |       无效缓冲块       |
| -268500033 |    VP_INVALID_POOLID     |       无效缓冲池       |
| -268500034 |    HB_ERR_VP_NOT_PERM    |       操作不允许       |
| -268500035 |    HB_ERR_VP_UNEXIST     |    视频缓冲池不存在    |
| -268500036 |      HB_ERR_VP_BUSY      |        缓冲池忙        |
| -268500037 |     HB_ERR_SYS_BUSY      |         系统忙         |
| -268500038 | HB_ERR_SYS_ILLEGAL_PARAM |    系统接口参数非法    |
| -268500039 |     HB_ERR_SYS_NOMEM     |  系统接口分配内存失败  |
| -268500040 | HB_ERR_VP_ILLEGAL_PARAM  | 缓冲池接口参数设置无效 |
