---
sidebar_position: 5
---

# 模型内存操作 API


## hbSysAllocMem()


**【函数原型】**  

``int32_t hbSysAllocMem(hbSysMem *mem, uint32_t size)``

**【功能描述】** 

申请BPU内存。

**【参数】**

- [in]  ``size``                申请内存的大小。
- [out] ``mem``                 内存指针。

**【返回类型】** 

- 返回 ``0`` 则表示API成功执行，否则执行失败。

## hbSysAllocCachedMem()


**【函数原型】**  

``int32_t hbSysAllocCachedMem(hbSysMem *mem, uint32_t size)``

**【功能描述】** 

申请缓存的BPU内存。

**【参数】**

- [in]  ``size``              申请内存的大小。
- [out] ``mem``               内存指针。

**【返回类型】**

- 返回 ``0`` 则表示API成功执行，否则执行失败。

## hbSysFlushMem()


**【函数原型】**  

``int32_t hbSysFlushMem(hbSysMem *mem, int32_t flag)``

**【功能描述】** 

对缓存的BPU内存进行刷新。

**【参数】**

- [in]  ``mem``               内存指针。
- [in]  ``flag``              刷新标志符。

**【返回类型】**

- 返回 ``0`` 则表示API成功执行，否则执行失败。

## hbSysFreeMem()


**【函数原型】**  

``int32_t hbSysFreeMem(hbSysMem *mem)``

**【功能描述】** 

释放BPU内存。

**【参数】**

- [in]  ``mem``               内存指针。

**【返回类型】**

- 返回 ``0`` 则表示API成功执行，否则执行失败。

## hbSysWriteMem()


**【函数原型】**  

``int32_t hbSysWriteMem(hbSysMem *dest, char *src, uint32_t size)``

**【功能描述】** 

写入BPU内存。

**【参数】**

- [out] ``dest``                内存指针。
- [in]  ``src``                 数据指针。
- [in]  ``size``                数据大小。

**【返回类型】**

- 返回 ``0`` 则表示API成功执行，否则执行失败。

## hbSysReadMem()


**【函数原型】**  

``int32_t hbSysReadMem(char *dest, hbSysMem *src, uint32_t size)``

**【功能描述】** 

读取BPU内存。

**【参数】**

- [out] ``dest``               数据指针。
- [in]  ``src``                内存指针。
- [in]  ``size``               数据大小。

**【返回类型】**

- 返回 ``0`` 则表示API成功执行，否则执行失败。

## hbSysRegisterMem()


**【函数原型】**  

``int32_t hbSysRegisterMem(hbSysMem *mem)``

**【功能描述】** 

将已知物理地址的内存区间注册成可被BPU使用的内存标识，得到的内存是cacheable的。

**【参数】**

- [in/out] ``mem``               内存指针。

**【返回类型】**

- 返回 ``0`` 则表示API成功执行，否则执行失败。

## hbSysUnregisterMem()


**【函数原型】**  

``int32_t hbSysUnregisterMem(hbSysMem *mem)``

**【功能描述】** 

注销由 ``hbSysRegisterMem`` 注册的内存标识。

**【参数】**

- [in] ``mem``               内存指针。

**【返回类型】**

- 返回 ``0`` 则表示API成功执行，否则执行失败。
