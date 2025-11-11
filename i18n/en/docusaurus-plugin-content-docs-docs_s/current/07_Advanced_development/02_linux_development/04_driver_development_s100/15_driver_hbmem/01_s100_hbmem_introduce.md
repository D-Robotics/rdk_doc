# Function Description

The Hbmem module primarily implements the following functionalities: **memory allocation**, **memory sharing**, **memory queue management**, and **memory pools**, specifically designed for managing **system-reserved memory**.

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/02_linux_development/driver_development_s100/hbmem/01_func_overview.png)

## Memory Allocation

The memory allocation interfaces mainly implement functionalities such as allocation, deallocation, and cache flushing for physically contiguous memory. This module provides two types of memory allocation:

- **com_buf**: Used for general-purpose allocation of large contiguous memory blocks, suitable for scenarios like audio data or plain feature maps.
- **graph_buf**: Used for allocating memory for RGB, RAW, and YUV images. For RGB and RAW formats, a single buffer is used. For planar YUV formats, physical addresses of individual components are non-contiguous, making it suitable for scenarios such as Pyramid output buffers.

Additionally, if multiple graphic buffers need to be allocated, users can utilize the graphic buffer group-related interfaces. These interfaces support allocating multiple graphic buffers at once and returning them as a group.

This module also supports setting memory attributes, including cache attributes and memory heap attributes. After successful memory allocation, **users obtain a corresponding file descriptor**. Based on this file descriptor, users can perform operations such as releasing the physical memory, invalidating/flushing caches, and retrieving buffer information. To facilitate memory operations, this module also supports using virtual addresses for cache invalidation/flushing and retrieving buffer information.

It is **not recommended** for users to directly perform mmap or pass physical addresses, as these operations **do not increment the memory's reference count**, potentially leading to situations where users continue accessing memory after it has already been freed.

## Memory Sharing

The memory sharing module provides interfaces enabling safe memory sharing across multiple threads or processes. Users can directly pass buffers of type com_buf or graph_buf to another thread or process and then import the buffer using the relevant import interfaces. This allows users to retrieve the buffer's information and increment its reference count, preventing premature deallocation by other modules and thus enabling safe buffer sharing. This sharing mechanism is primarily implemented through memory queue management, facilitating buffer exchange between producers and consumers.

Similarly, graphic buffer groups also support safe memory sharing across multiple threads or processes, using a method analogous to that of individual graphic buffers.

## Memory Queue Management

The memory queue management module provides a generic queue management mechanism supporting four operational states for queue elements: **FREE**, **DEQUEUE**, **QUEUE**, and **REQUEST**:

- **FREE**: Indicates that the producer can acquire this queue slot and fill in element information.
- **DEQUEUE**: Indicates that the element has been acquired by the producer and has not yet been returned.
- **QUEUE**: Indicates that the producer has filled the element and it is ready for the consumer to acquire.
- **REQUEST**: Indicates that the element has been acquired by the consumer and has not yet been released.

Producers and consumers use their respective interfaces to ensure proper cycling of buffer elements between them.

**Note**: Since the memory queue is implemented as a circular queue, once it becomes full, writing new events will overwrite the oldest (first) event. Additionally, **this queue only supports operations within a single process**.

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/02_linux_development/driver_development_s100/hbmem/01_memory_queue.png)

## Memory Pool

The memory pool module provides interfaces allowing users to create a local memory pool via the memory allocation functionality and then allocate or deallocate small memory blocks from this pool.

**Using the memory pool module enables users to allocate memory quickly without entering kernel mode**, thereby achieving more efficient memory management. Currently, user programs (e.g., hbrt, dnn, etc.) often implement their own memory management modules to accelerate memory allocation, resulting in code duplication and increased development and maintenance overhead. Therefore, users are encouraged to adopt the memory pool module for memory management to improve code reusability.

During system boot-up, a large memory block is pre-allocated for users. Consequently, when users request memory later, they can directly use memory from this pre-allocated pool, bypassing the need to enter kernel mode for allocation—effectively pre-completing this step. **Note that this memory pool currently only supports operations within a single process, and memory allocated from the pool cannot be shared across multiple processes.**

## Shared Memory Pool

The shared memory pool provides the same functionality as described above, with the key difference being that memory allocated from a shared memory pool supports sharing across multiple processes. However, all buffers allocated from a shared memory pool must be of the same size, which can only be specified when the shared memory pool is created. Due to multi-process sharing support, operations such as importing or freeing buffers from a shared memory pool are slightly less efficient compared to those from a regular (non-shared) memory pool.