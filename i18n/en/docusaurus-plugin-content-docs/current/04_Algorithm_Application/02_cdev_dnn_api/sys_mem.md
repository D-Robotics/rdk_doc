---
sidebar_position: 5
---
# 4.2.5 Model Memory Operation API

## hbSysAllocMem()

**【Function prototype】** 

``int32_t hbSysAllocMem(hbSysMem *mem, uint32_t size)``

**【Description】** 

Allocate BPU memory.

**【Parameters】**

- [in] ``size``  The size of the memory to be allocated.
- [out] ``mem``  Memory pointer.

**【Return type】** 

- Returns ``0`` if the API is executed successfully, otherwise the execution fails.

## hbSysAllocCachedMem()

**【Function prototype】** 

``int32_t hbSysAllocCachedMem(hbSysMem *mem, uint32_t size)``

**【Description】** 

Allocate cached BPU memory.

**【Parameters】**

- [in] ``size``  The size of the memory to be allocated.
- [out] ``mem``  Memory pointer.

**【Return type】**

- Returns ``0`` if the API is executed successfully, otherwise the execution fails.

## hbSysFlushMem()

**[Function Prototype]**

``int32_t hbSysFlushMem(hbSysMem *mem, int32_t flag)``

**[Description]**

Flushes the cache BPU memory.

**[Parameters]**

- [in]  ``mem``               Memory pointer.
- [in]  ``flag``              Flush flag.

**[Return Type]**

- Returns ``0`` if the API is executed successfully, otherwise it fails.

## hbSysFreeMem()


**[Function Prototype]**

``int32_t hbSysFreeMem(hbSysMem *mem)``

**[Description]**

Frees BPU memory.

**[Parameters]**

- [in]  ``mem``               Memory pointer.

**[Return Type]**

- Returns ``0`` if the API is executed successfully, otherwise it fails.

## hbSysWriteMem()


**[Function Prototype]**

``int32_t hbSysWriteMem(hbSysMem *dest, char *src, uint32_t size)``

**[Description]**

Writes to BPU memory.

**[Parameters]**

- [out] `dest`  Memory pointer.
- [in] `src`    Pointer to data.
- [in] `size`   Size of data.

**[Return Type]**

- Returns 0 if the API is executed successfully, otherwise it fails.

## hbSysReadMem()


**[Function Prototype]**  

``int32_t hbSysReadMem(char *dest, hbSysMem *src, uint32_t size)``

**[Function Description]** 

Reads BPU memory.

**[Parameters]**

- [out] `dest`Pointer to data.
- [in] `src`Pointer to memory.
- [in] `size`Size of data.

**[Return Type]**

- Returns 0 if the API is executed successfully, otherwise it fails.

## hbSysRegisterMem()


**[Function Prototype]**  

``int32_t hbSysRegisterMem(hbSysMem *mem)``

**[Function Description]** 

Registers a memory range with a known physical address as a memory identifier that can be used by BPU, and the memory obtained is cacheable.

**[Parameters]**

- [in/out] `mem`Pointer to memory.

**[Return Type]**

- Returns 0 if the API is executed successfully, otherwise it fails.

## hbSysUnregisterMem()


**[Function Prototype]**

``int32_t hbSysUnregisterMem(hbSysMem *mem)``

**[Description]**

This function is used to unregister the memory identifier registered by ``hbSysRegisterMem``.

**[Parameters]**

- [in] ``mem``: Memory pointer.

**[Return Type]**

If the API is executed successfully, it returns ``0``; otherwise, it returns a non-zero value indicating failure.