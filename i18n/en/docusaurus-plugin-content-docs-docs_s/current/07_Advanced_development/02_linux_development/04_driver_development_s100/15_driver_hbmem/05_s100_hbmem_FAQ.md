# FAQ

## FAQ001: Memory Concepts

- Q: What are the purposes of the other two types of reserved ION memory (cma_reserved and carveout)?
    - A: Both types of memory are essentially carveout. The reason they are distinguished is that one carveout is primarily used for BPU and codec memory allocation in the system, while modules in VIO allocate memory from the other carveout.
- Q: Is Hbmem required because certain IPs need physically contiguous memory?
    - A: Yes.

## FAQ002: Structure Descriptions

- Q: What is the semantic difference between com_buf and graph_buf?
    - A: com_buf represents a single, contiguous buffer with a simpler structure. In contrast, graph_buf is more complex and consists of multiple planes.
- Q: Does com_buf have width and height attributes, or only a size?
    - A: It only has a size. Width and height attributes must be retrieved from individual modules; however, graph_buf does include width and height attributes.
- Q: Are com_buf and graph_buf structurally just different encapsulations, with graph_buf carrying richer semantics, but fundamentally equivalent?
    - A: Yes.
- Q: Does offset only exist if we explicitly set it? Is this offset the base address when allocating memory?
    - A: Yes, it must be set manually. For example, if you obtain a buffer and want to pass an offset address to another process, you can set the offset here. This offset is then passed along with the buffer and transparently transmitted to the other process. Newly allocated buffers always have an offset of zero. The offset is used during inter-process transmission to carry transparent information indicating an offset within the buffer.
- Q: Why was graph_buf introduced in addition to com_buf? What's the difference?
    - A: When allocating com_buf, you can only specify size and standard allocation attributes. In contrast, graph_buf allows specifying width, height, format, stride, contiguity, and other standard allocation attributes during allocation. Compared to com_buf, graph_buf carries richer buffer metadata, making it easier for users to directly allocate buffers in RGB/YUV/RAW formats.

## FAQ003: Operational Notes

- Q: Does each process need to call module_open? Is this an extra step for users? Can it be opened repeatedly? Is there internal protection?
    - A: Yes, each process must call module_open, as different engineers typically develop each process, necessitating this operation per process. Within a single process, multiple opens are not recommended and unnecessary; each open must have a corresponding close. Normally, if allocated memory isn't freed explicitly, it will be freed during module_close. However, missing the corresponding close operation will prevent memory from being freed. That said, when a process terminates, all ION-allocated memory will be released automatically.
- Q: Can the file descriptor (fd) limit be increased using Linux ulimit?
    - A: Yes, ulimit can be used to raise the fd limit. In some scenarios, the issue isn't actually insufficient fds but rather fd leaks caused by incorrect API usage.

## FAQ004: Function Interfaces

- Q: Are these interface functions merely wrappers at the driver level?
    - A: They are more than just wrappers. Buffers allocated via Hbmem are universally usable across modules, and all relevant information can be retrieved via virtual addresses. For instance, after obtaining a buffer from VIO, you can convert it to a com_buf and pass it to another module like BPU.
- Q: Can buffer information be obtained via the get_buffer_info interface?
    - A: Yes. Additionally, you can use hb_mem_get_com_buffer_info_with_vaddr to retrieve com_buf information using a virtual address.
- Q: Is invalid a blocking interface? Could calling invalid from two processes cause a deadlock? What happens after a timeout?
    - A: It is a blocking interface; deadlocks cannot occur as the function handles this internally; there is no timeout—the function only returns after the operation completes.

## FAQ005: Memory Allocation

- Q: Is memory allocated via the interface physically contiguous?
    - A: First, it depends on which heap is selected—memory allocated from CMA or carveout heaps is physically contiguous. Second, you can choose the heap via parameters.
- Q: Are the provided APIs equivalent to generic interfaces but with added inter-process sharing capability? If inter-process sharing isn't needed, can malloc be used instead?
    - A: If you use standard malloc, the allocated memory will not be physically contiguous.
- Q: Does the allocated buffer support automatic format conversion? Does the S100X support hardware-based format conversion?
    - A: Automatic format conversion is not supported. For hardware-based format conversion, consider using CIM and Pyramid.
- Q: How do I specify which heap to use for memory allocation?
    - A: Include one of the following flags in the allocation flags:
        ``` shell
        HB_MEM_USAGE_PRIV_HEAP_DMA;
        HB_MEM_USAGE_PRIV_HEAP_RESERVED;
        HB_MEM_USAGE_PRIV_HEAP_2_RESERVED;
        ```
- Q: Assuming the allocated memory is accessed only by CPUs in processes A and B, should cacheable memory be allocated for performance? If cacheable, does process B need to perform invalidate when only reading?
    - A: If used exclusively between CPUs, cacheable memory is recommended. Enabling cacheable significantly accelerates memory access—by over 1000x as expected. For CPU-only read/write access, no invalidate or flush operations are needed; cache coherency is guaranteed by the CPU's MESI protocol.
- Q: If the memory is intended for an IP (e.g., GDC/codec) and won't be accessed by process B's CPU, can we directly pass the physical address, or must we still use a buffer?
    - A: Directly using a physical address without allocating via ION cannot guarantee exclusive usage during the address's lifetime—it might be allocated by another user. Thus, when hardware or software uses the buffer, its lifetime must be properly managed.
- Q: Why is the virtual address converted to uint8_t when allocating YUV memory?
    - A: Storing the virtual address in a uint8_t* pointer primarily addresses compatibility—some systems are 32-bit while others are 64-bit.
- Q: What is the purpose of the offset field in com_buf and graphic_buf?
    - A: The offset field indicates the data's offset relative to the buffer's virtual or physical address. Combined with the address, it allows direct access to the desired data.
- Q: What does the warning "No need to invalidate for uncached buffer" mean during invalid/flush operations?
    - A: This warning indicates that invalidate/flush operations are unnecessary for uncached memory. You can check the memory's cacheability via its flags.
- Q: Are there memory allocation requirements for using hbmem_dmacpy—e.g., must memory be allocated via hb_mem_alloc_com_buf or hbmem_alloc?
    - A: Memory allocated from Hbmem can be used directly via its virtual address. For non-Hbmem memory, use memcopy for copying.
- Q: How can I check memory attributes?
    - A: If you have a common buffer or graphic buffer structure, check its flags. If you only have a virtual address, use hb_mem_get_buf_info to retrieve the corresponding flags.
- Q: If one heap fails to allocate, the system automatically tries another heap—why does allocation failure still occur?
    - A: Fallback to another heap doesn't combine heaps; it attempts allocation in the other heap's space, which is still limited by that heap's total size. Each heap is managed independently, so allocatable size is constrained by each heap's available space.

## FAQ006: Memory Sharing

- Q: What information is transferred between processes during inter-process memory sharing?
    - A: A complete com_buf or graph_buf structure is transferred.
- Q: When passing com_buf from one process to another, must it be converted via an import interface before use?
    - A: Yes.
- Q: Does import have a corresponding release operation that decrements the reference count?
    - A: Yes. Every import must be paired with a free operation. Continuous imports without frees will cause fd counts to exceed 1024.
- Q: Do different hardware components have distinct memory management APIs? Can their memories be converted and accessed mutually?
    - A: Yes.
- Q: In multi-process memory sharing, how can process A know if process B has released the imported buffer? Can hb_mem_get_share_info be used?
    - A: Yes. Use hb_mem_wait_share_status to wait until the client count returns to its pre-import value. Before importing, use hb_mem_get_share_info to check the current client count.
- Q: Why are both in_buf and out_buf needed during memory sharing import?
    - A: During sharing, a new dma_buf must be created for the buffer with a new fd. Additionally, the shared memory might not occupy the entire buffer. Import also increments the buffer's underlying reference count.
- Q: Why does dma copy use memcopy instead of dmacopy for memory smaller than PAGE_SIZE?
    - A: Previously, dma copy had size restrictions, so memcopy was used for copies under 4K. Now, dma copy supports any size down to 1 byte.
- Q: Why must an imported buffer be released when no longer needed?
    - A: Import increments the buffer's reference count, while hb_mem_free_buf decrements it.
- Q: How can a thread or process verify the validity of a received com_buf?
    - A: Users may need to check buffer validity—if invalid, they should allocate a new one.  
         For synchronized threads within the same process, use hb_mem_get_com_buf/hb_mem_get_graph_buf to retrieve buffer info from libhbmem and verify it matches the allocated buffer.  
         For cross-process sharing, since libhbmem info is process-isolated, buffer validity cannot be confirmed via libhbmem interfaces.
- Q: How to share only part of a buffer between processes?
    - A: If process A allocates an 8K buffer but wants to share only the last 4K with process B, it should set the physical address to PA+4KB and size to 4K when passing the buffer. After import, process B will get a virtual address starting at the mmap base plus a 4KB offset.  
Note: Setting offset won't achieve this—after import, the virtual address will always be the mmap base address.
- Q: If the buffer type (common/graphic) is unknown, how to get its ID? Must import always be called for inter-process sharing? What if the type is uncertain?
    - A: Use hb_mem_get_buf_type_and_buf_with_vaddr to get the buffer type and structure from the virtual address, then extract the share ID.  
Inter-process sharing always requires import—otherwise, buffer safety during its lifetime isn't guaranteed, and virtual addresses from other processes are unusable without import (which generates a new virtual address).
- Q: With multiple addresses of uncertain allocation origin (common/graphic buffer or separate allocations), how should they be handled?
    - A: Use hb_mem_get_buf_type_and_buf_with_vaddr to determine the buffer type and structure from the virtual address, then import. The same buffer supports multiple imports.
- Q: In special scenarios where process A allocates memory, stops using it, and transfers management to process B—can B perform the actual release?
    - A: Yes. Once process A confirms process B has imported the buffer, A can release it. Then, when B releases the buffer, the memory is freed immediately—effectively transferring buffer lifetime control to B.

## FAQ007: Memory Queue Management

- Q: Earlier it was mentioned that memory queues are single-process, yet shared memory can be used for inter-process communication—what exactly does memory queue management mean?
    - A: Memory sharing and memory queue management are separate features. Memory sharing allows an IP to pass a buffer directly to another process, which can then use import to access it. Memory queue management handles buffers within a single process—for example, a server can create a buffer queue, add buffers to it, and clients can dequeue them, similar to a message queue. The queue isn't restricted to buffer info—it can hold other data too.

## FAQ008: Memory Pools

- Q: How to understand pre-allocating a large memory block for use, then managing it with a custom memory management queue?
    - A: Memory pool allocation first requests a large block from a heap. Subsequent allocations happen within this block without kernel involvement, making them much faster than direct API calls. Free operations return memory to the pool, which can be destroyed after use. For managing multiple allocated buffers, you can either implement your own queue or use our provided management queue.
- Q: When using memory pools, must we pre-estimate total memory needs and reserve it at the application layer?
    - A: Yes.
- Q: Can memory pools be used for inter-process operations?
    - A: Not recommended—it's essentially a single buffer.
- Q: Are there size requirements for pool allocation?
    - A: No specific requirements—allocate as needed as long as space is available. Allocations are page-size aligned.

## FAQ009: Software Performance

- Q: How is Hbmem stability ensured?
    - A: Hbmem is based on ION. We conduct extensive testing—including exception cases, long-term stability, and stress tests—on both ION and Hbmem interfaces. Additionally, we validate allocation functionality in real-world scenarios like VIO, BPU, and codec.
- Q: How are un-released allocations handled?
    - A: First, if a process exits without releasing memory, the system automatically recovers it. Second, for running processes with leaks, you can check sys nodes to monitor the user's memory allocation status.

## FAQ010: Memory Leaks

- Q: Won't process termination cause memory leaks?
    - A: Normally, all memory is reclaimed upon process termination with no leaks. However, if process A allocates memory and shares it with process B (incrementing the usage count), terminating A won't free the memory—it remains with B and is released only when B terminates.

## FAQ011: Software Debugging

- Q: Are there diagnostic tools for these interfaces?
    - A: Yes, use sys nodes for diagnostics:
        ```shell
        # Corresponds to HB_MEM_USAGE_PRIV_HEAP_DMA
        cat /sys/kernel/debug/ion/heaps/ion_cma
        ```

        ```shell
        # Corresponds to HB_MEM_USAGE_PRIV_HEAP_RESERVED
        cat /sys/kernel/debug/ion/heaps/carveout
        ```

        ```shell
        # Corresponds to HB_MEM_USAGE_PRIV_HEAP_2_RESERVED
        cat /sys/kernel/debug/ion/heaps/cma_reserved
        ```

        ```shell
        # For all heap info
        cat /sys/kernel/debug/ion/heaps/all_heap_info
        ```

        ```shell
        cat /sys/kernel/debug/ion/clients/"client name"
        ```

## FAQ012: Module Usage

- Q: Do modules like camera and pyramid output com_buf?
    - A: Yes—all current system modules use com_buf with contiguous memory.
- Q: Is the hbmem_alloc interface in hbmem.h discouraged?
    - A: Yes—it's the legacy interface kept for compatibility. If you have existing Hbmem-based code, you can use it; otherwise, use the new interfaces.