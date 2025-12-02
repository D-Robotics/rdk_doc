# Software Description

## API

Non-root users cannot use the Hbmem API.

| Source File                              | Exposed Interface Functions                  | Description                                    |
|------------------------------------------|----------------------------------------------|------------------------------------------------|
| Basic Interface: hb_mem_manager.c        | hb_mem_get_version                           | Get version information                        |
|                                          | hb_mem_module_open                           | Open module                                    |
|                                          | hb_mem_module_close                          | Close module                                   |
| Memory Allocation Interface: hb_mem_allocator.c | hb_mem_alloc_com_buf                    | Allocate com_buf memory                        |
|                                          | hb_mem_get_com_buf                           | Get corresponding com_buf info via fd          |
|                                          | hb_mem_alloc_graph_buf                       | Allocate graph_buf memory                      |
|                                          | hb_mem_get_graph_buf                         | Get graph_buf info via fd                      |
|                                          | hb_mem_free_buf                              | Free corresponding buffer                      |
|                                          | hb_mem_invalidate_buf                        | Invalidate cache corresponding to buffer       |
|                                          | hb_mem_flush_buf                             | Flush buffer to memory                         |
|                                          | hb_mem_is_valid_buf                          | Check if buffer info is valid                  |
|                                          | hb_mem_get_phys_addr                         | Get physical address of buffer                 |
|                                          | hb_mem_get_buf_info                          | Get buffer-related info                        |
|                                          | hb_mem_free_buf_with_vaddr                   | Free buffer memory via virtual address         |
|                                          | hb_mem_invalidate_buf_with_vaddr             | Invalidate cache corresponding to buffer via virtual address |
|                                          | hb_mem_flush_buf_with_vaddr                  | Flush buffer to memory via virtual address     |
|                                          | hb_mem_get_com_buf_with_vaddr                | Get com_buf via virtual address                |
|                                          | hb_mem_get_graph_buf_with_vaddr              | Get graph_buf via virtual address              |
|                                          | hb_mem_get_share_info                        | Get shared memory info                         |
|                                          | hb_mem_get_share_info_with_vaddr             | Get shared info via virtual address            |
|                                          | hb_mem_wait_share_status                     | Wait to acquire shared memory info             |
|                                          | hb_mem_wait_share_status_with_vaddr          | Wait to acquire shared info via virtual address|
|                                          | hb_mem_get_buffer_process_info               | Get PID of the process holding the buffer via virtual address |
|                                          | hb_mem_get_buffer_process_info_with_share_id | Get PID of the process holding the buffer via share_id |
|                                          | hb_mem_get_consume_info                      | Get buffer's consume count                     |
|                                          | hb_mem_get_consume_info_with_vaddr           | Get buffer's consume count via virtual address |
|                                          | hb_mem_wait_consume_status                   | Wait until consume count reaches a certain value |
|                                          | hb_mem_wait_consume_status_with_vaddr        | Wait until consume count reaches a certain value via virtual address |
|                                          | hb_mem_dma_copy                              | DMA copy                                       |
|                                          | hb_mem_alloc_graph_buf_group                 | Allocate graphic buffer group                  |
|                                          | hb_mem_get_graph_buf_group                   | Get graphic buffer group via fd                |
|                                          | hb_mem_get_graph_buf_group_with_vaddr        | Get graphic buffer group via virtual address   |
|                                          | hb_mem_get_buf_and_type_with_vaddr           | Get buffer and buffer type via virtual address |
|                                          | hb_mem_get_buf_type_with_vaddr               | Get buffer type via virtual address            |
|                                          | hb_mem_get_buf_type_and_buf_with_vaddr       | Get buffer and type, supporting type conversion|
|                                          | hb_mem_inc_user_consume_cnt                  | Increase user-space reference count of buffer via fd |
|                                          | hb_mem_dec_user_consume_cnt                  | Decrease user-space reference count of buffer via fd |
|                                          | hb_mem_inc_user_consume_cnt_with_vaddr       | Increase user-space reference count of buffer via virtual address |
|                                          | hb_mem_dec_user_consume_cnt_with_vaddr       | Decrease user-space reference count of buffer via virtual address |
| Memory Pool Interface: hb_mem_pool.c     | hb_mem_pool_create                           | Create memory pool                             |
|                                          | hb_mem_pool_destroy                          | Destroy memory pool                            |
|                                          | hb_mem_pool_alloc_buf                        | Allocate buffer from memory pool               |
|                                          | hb_mem_pool_free_buf                         | Free memory allocated from memory pool         |
|                                          | hb_mem_pool_get_info                         | Get memory pool info                           |
| Shared Memory Pool Interface: hb_mem_share_pool.c | hb_mem_share_pool_create             | Create shared memory pool                      |
|                                          | hb_mem_share_pool_destroy                    | Destroy shared memory pool                     |
|                                          | hb_mem_share_pool_alloc_buf                  | Allocate buffer from shared memory pool        |
|                                          | hb_mem_share_pool_free_buf                   | Free memory allocated from shared memory pool  |
|                                          | hb_mem_share_pool_get_info                   | Get shared memory pool info                    |
| Memory Queue Interface: hb_mem_queue.c   | hb_mem_create_buf_queue                      | Create queue                                   |
|                                          | hb_mem_destroy_buf_queue                     | Destroy queue                                  |
|                                          | hb_mem_dequeue_buf                           | Dequeue an available free slot from queue      |
|                                          | hb_mem_queue_buf                             | Insert buffer into queue                       |
|                                          | hb_mem_request_buf                           | Request buffer from queue                      |
|                                          | hb_mem_release_buf                           | Release requested buffer from queue            |
|                                          | hb_mem_cancel_buf                            | Cancel operation on queue                      |
| Memory Sharing Interface: hb_mem_share.c | hb_mem_import_com_buf                        | Import com_buf                                 |
|                                          | hb_mem_import_graph_buf                      | Import graph buffer                            |
|                                          | hb_mem_inc_com_buf_consume_cnt               | Increase com buffer consume count              |
|                                          | hb_mem_dec_consume_cnt                       | Decrease consume count                         |
|                                          | hb_mem_dec_consume_cnt_with_vaddr            | Decrease consume count via virtual address     |
|                                          | hb_mem_inc_graph_buf_consume_cnt             | Increase graph buffer consume count            |
|                                          | hb_mem_import_com_buf_with_paddr             | Import via physical address                    |
|                                          | hb_mem_import_graph_buf_group                | Import graphic buffer group                    |
|                                          | hb_mem_inc_graph_buf_group_consume_cnt       | Increase graphic buffer group consume count    |

## Memory Allocation Attribute Description

| Data Item                                | Description                                                                                                                         |
|------------------------------------------|-------------------------------------------------------------------------------------------------------------------------------------|
| HB_MEM_USAGE_CPU_READ_NEVER              | CPU will never read this memory; memory will not be assigned read permission                                                       |
| HB_MEM_USAGE_CPU_READ_OFTEN              | CPU frequently reads this memory; memory will be assigned read permission                                                           |
| HB_MEM_USAGE_CPU_READ_MASK               | Mask used to retrieve read-related attributes; HB_MEM_USAGE_CPU_READ_OFTEN has higher priority than HB_MEM_USAGE_CPU_READ_NEVER    |
| HB_MEM_USAGE_CPU_WRITE_NEVER             | CPU will never write this memory; memory will not be assigned write permission                                                     |
| HB_MEM_USAGE_CPU_WRITE_OFTEN             | CPU frequently writes this memory; memory will be assigned write permission (read permission is automatically added)                |
| HB_MEM_USAGE_CPU_WRITE_MASK              | Mask used to retrieve write-related attributes; HB_MEM_USAGE_CPU_WRITE_OFTEN has higher priority than HB_MEM_USAGE_CPU_WRITE_NEVER |
| HB_MEM_USAGE_HW_CIM                      | Indicates memory is used by camera interface module; does not affect allocation, used for debug info                               |
| HB_MEM_USAGE_HW_PYRAMID                  | Indicates memory is used by pyramid-related modules; does not affect allocation, used for debug info                                |
| HB_MEM_USAGE_HW_GDC                      | Indicates memory is used as input buffer for geometric distortion correction modules; does not affect allocation, used for debug info |
| HB_MEM_USAGE_HW_GDC_OUT                  | Indicates memory is used as output buffer for geometric distortion correction modules; does not affect allocation, used for debug info |
| HB_MEM_USAGE_HW_STITCH                   | Indicates memory is used by stitch-related modules; does not affect allocation, used for debug info                                |
| HB_MEM_USAGE_HW_OPTICAL_FLOW             | Indicates memory is used by optical flow-related modules; does not affect allocation, used for debug info                          |
| HB_MEM_USAGE_HW_BPU                      | Indicates memory is used by BPU module; does not affect allocation, used for debug info                                            |
| HB_MEM_USAGE_HW_ISP                      | Indicates memory is used by ISP module; does not affect allocation, used for debug info                                            |
| HB_MEM_USAGE_HW_DISPLAY                  | Indicates memory is used by Display-related modules; does not affect allocation, used for debug info                               |
| HB_MEM_USAGE_HW_VIDEO_CODEC              | Indicates memory is used by video codec-related modules; does not affect allocation, used for debug info                           |
| HB_MEM_USAGE_HW_JPEG_CODEC               | Indicates memory is used by jpeg codec-related modules; does not affect allocation, used for debug info                            |
| HB_MEM_USAGE_HW_VDSP                     | Indicates memory is used by VDSP-related modules; does not affect allocation, used for debug info                                  |
| HB_MEM_USAGE_HW_IPC                      | Indicates memory is used by IPC-related modules; does not affect allocation, used for debug info                                   |
| HB_MEM_USAGE_HW_PCIE                     | Indicates memory is used by PCIe-related modules; does not affect allocation, used for debug info                                  |
| HB_MEM_USAGE_HW_YNR                      | Indicates memory is used by YNR-related modules; does not affect allocation, used for debug info                                   |
| HB_MEM_USAGE_HW_MASK                     | Mask used to retrieve hardware-related attributes; attributes under this mask are mutually exclusive, with priority decreasing from top to bottom. If multiple attributes are specified or a non-listed value is used, defaults to "other" |
| HB_MEM_USAGE_MAP_INITIALIZED             | Memory needs initialization; allocated memory is initialized to 0. When neither MAP_INITIALIZED nor MAP_UNINITIALIZED is specified, DMA heap is initialized by default while RESERVED heap is not. This attribute is mutually exclusive with HB_MEM_USAGE_MAP_UNINITIALIZED and has higher priority. |
| HB_MEM_USAGE_MAP_UNINITIALIZED           | Memory does not need initialization; allocated memory is not initialized to 0. This attribute is mutually exclusive with HB_MEM_USAGE_MAP_INITIALIZED and has lower priority. |
| HB_MEM_USAGE_CACHED                      | Indicates this buffer has cache attributes                                                                                         |
| HB_MEM_USAGE_GRAPHIC_CONTIGUOUS_BUF      | Specifies graph_buf should be allocated with physically contiguous memory                                                          |
| HB_MEM_USAGE_MEM_POOL                    | Indicates this buffer is for memory pool; users do not need to specify this parameter when allocating buffers. Even if specified, it is ignored internally. |
| HB_MEM_USAGE_MEM_SHAREPOOL               | Indicates this buffer is for memory share pool; users do not need to specify this parameter when allocating buffers. Even if specified, it is ignored internally. |
| HB_MEM_USAGE_TRIVIAL_MASK                | Mask used to retrieve miscellaneous attributes. Except for the mutual exclusivity between HB_MEM_USAGE_MAP_INITIALIZED and HB_MEM_USAGE_MAP_UNINITIALIZED, other attributes under this mask can coexist. |
| HB_MEM_USAGE_PRIV_HEAP_DMA               | Specifies memory allocation from DMA heap                                                                                          |
| HB_MEM_USAGE_PRIV_HEAP_RESERVERD         | Specifies memory allocation from Carveout heap (original definition for backward compatibility; not recommended)                   |
| HB_MEM_USAGE_PRIV_HEAP_RESERVED          | Specifies memory allocation from Carveout heap (latest definition; recommended)                                                    |
| HB_MEM_USAGE_PRIV_HEAP_2_RESERVERD       | Specifies memory allocation from Carveout heap2 (original definition for backward compatibility; not recommended)                  |
| HB_MEM_USAGE_PRIV_HEAP_2_RESERVED        | Specifies memory allocation from Carveout heap2 (latest definition; recommended)                                                   |
| HB_MEM_USAGE_PRIV_MASK                   | Mask used to retrieve private attributes; attributes under this mask are mutually exclusive, with priority decreasing from top to bottom. When multiple attributes are specified, the highest priority one is selected. If a non-listed value is specified, buffer is allocated from DMA heap by default. |

## Buffer Type Description

| Type                             | Description          |
|----------------------------------|----------------------|
| HB_MEM_BUFFER_TYPE_COMMON        | Common buffer        |
| HB_MEM_BUFFER_TYPE_GRAPHIC       | Graphic buffer       |
| HB_MEM_BUFFER_TYPE_GRAPHIC_GROUP | Graphic buffer group |

## Heap Introduction

| dts_name     | dts_compatible | heap_mask                       | heap_id                    | flag_mask                                                            | debug_node     |
|--------------|----------------|---------------------------------|----------------------------|----------------------------------------------------------------------|----------------|
| ion_carveout  | ion-carveout   | ION_HEAP_CARVEOUT_MASK          | ION_HEAP_TYPE_CARVEOUT     | HB_MEM_USAGE_PRIV_HEAP_RESERVERD/HB_MEM_USAGE_PRIV_HEAP_RESERVED     | carveout       |
| ion_reserved  | ion-pool       | ION_HEAP_TYPE_CMA_RESERVED_MASK | ION_HEAP_TYPE_CMA_RESERVED | HB_MEM_USAGE_PRIV_HEAP_2_RESERVERD/HB_MEM_USAGE_PRIV_HEAP_2_RESERVED | cma_reserved   |
| ion_cma       | ion-cma        | ION_HEAP_TYPE_DMA_MASK          | ION_HEAP_TYPE_DMA          | HB_MEM_USAGE_PRIV_HEAP_DMA                                           | ion_cma        |

## Memory Allocation Principles

When Hbmem allocates memory in the ION driver, it first checks whether sufficient memory remains in the intended heap. If not, allocation proceeds from other heaps. The specific allocation logic is as follows:  
(cma_reserved ⇒ carveout ⇒ cma or carveout ⇒ cma_reserved ⇒ cma or cma ⇒ cma_reserved)

## Header File Description

The public header files of libhbmem are hb_mem_mgr.h, hbmem.h, and hb_mem_err.h.  
Among them, hb_mem_mgr.h is the main public interface file of libhbmem, defining all related operations; hbmem.h contains some compatibility interfaces—new applications are recommended to use hb_mem_mgr.h;  
hb_mem_err.h defines the error codes returned by libhbmem.

## Static and Dynamic Library Description

libhbmem compiles into one static library file libhbmem.a and three dynamic library files: libhbmem.so.1.0.0, libhbmem.so.1, and libhbmem.so.  
Among these, libhbmem.so.1 and libhbmem.so are symbolic links; libhbmem.so.1.0.0 is the actual library file, with the numeric suffix indicating the version number.