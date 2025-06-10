# 软件说明

## API

非root用户无法使用Hbmem的api。

| 源文件                              | 提供对外接口函数                             | 说明                                    |
|-------------------------------------|----------------------------------------------|-----------------------------------------|
| 基础接口:hb_mem_manager.c           | hb_mem_get_version                           | 获取版本信息                            |
|                                     | hb_mem_module_open                           | 打开模块                                |
|                                     | hb_mem_module_close                          | 关闭模块                                |
| 内存分配功能接口:hb_mem_allocator.c | hb_mem_alloc_com_buf                         | 申请com_buf内存                         |
|                                     | hb_mem_get_com_buf                           | 通过fd获取对应的com_buf信息             |
|                                     | hb_mem_alloc_graph_buf                       | 申请graph_buf内存                       |
|                                     | hb_mem_get_graph_buf                         | 通过fd获取graph_buf信息                 |
|                                     | hb_mem_free_buf                              | 释放对应的buffer                        |
|                                     | hb_mem_invalidate_buf                        | 使buffer对应cache无效                   |
|                                     | hb_mem_flush_buf                             | 将buffer刷新到内存中                    |
|                                     | hb_mem_is_valid_buf                          | 判断buffer信息是否有效                  |
|                                     | hb_mem_get_phys_addr                         | 获取buffer的物理地址                    |
|                                     | hb_mem_get_buf_info                          | 获取buffer相关信息                      |
|                                     | hb_mem_free_buf_with_vaddr                   | 通过虚拟地址释放buffer内存              |
|                                     | hb_mem_invalidate_buf_with_vaddr             | 通过虚拟地址使buffer对应cache无效       |
|                                     | hb_mem_flush_buf_with_vaddr                  | 通过虚拟地址将buffer刷新到内存中        |
|                                     | hb_mem_get_com_buf_with_vaddr                | 通过虚拟地址获取com_buf                 |
|                                     | hb_mem_get_graph_buf_with_vaddr              | 通过虚拟地址获取graph_buf               |
|                                     | hb_mem_get_share_info                        | 获取共享内存信息                        |
|                                     | hb_mem_get_share_info_with_vaddr             | 通过虚拟地址获取共享信息                |
|                                     | hb_mem_wait_share_status                     | 等待获取共享内存信息                    |
|                                     | hb_mem_wait_share_status_with_vaddr          | 通过虚拟地址等待获取共享信息            |
|                                     | hb_mem_get_buffer_process_info               | 通过虚拟地址获取持有该buffer的进程pid   |
|                                     | hb_mem_get_buffer_process_info_with_share_id | 通过share_id获取持有该buffer的进程pid   |
|                                     | hb_mem_get_consume_info                      | 获取buffer的consume count               |
|                                     | hb_mem_get_consume_info_with_vaddr           | 通过虚拟地址获取buffer的consume count   |
|                                     | hb_mem_wait_consume_status                   | 等待consume count到达某个值             |
|                                     | hb_mem_wait_consume_status_with_vaddr        | 通过虚拟地址等待consume count到达某个值 |
|                                     | hb_mem_dma_copy                              | dma copy                                |
|                                     | hb_mem_alloc_graph_buf_group                 | 申请graphic buffer group                |
|                                     | hb_mem_get_graph_buf_group                   | 通过fd获取graphic buffer group          |
|                                     | hb_mem_get_graph_buf_group_with_vaddr        | 通过虚拟地址获取graphic buffer group    |
|                                     | hb_mem_get_buf_and_type_with_vaddr           | 通过虚拟地址获取buffer和buffer类型      |
|                                     | hb_mem_get_buf_type_with_vaddr               | 通过虚拟地址获取buffer类型              |
|                                     | hb_mem_get_buf_type_and_buf_with_vaddr       | 获取buffer和类型，支持类型转换          |
|                                     | hb_mem_inc_user_consume_cnt                  | 通过fd增加buffer用户态引用计数          |
|                                     | hb_mem_dec_user_consume_cnt                  | 通过fd减少buffer用户态引用计数          |
|                                     | hb_mem_inc_user_consume_cnt_with_vaddr       | 通过虚拟地址增加buffer用户态引用计数    |
|                                     | hb_mem_dec_user_consume_cnt_with_vaddr       | 通过虚拟地址减少buffer用户态引用计数    |
| 内存池功能接口:hb_mem_pool.c        | hb_mem_pool_create                           | 创建内存池                              |
|                                     | hb_mem_pool_destroy                          | 销毁内存池                              |
|                                     | hb_mem_pool_alloc_buf                        | 在内存池上分配buffer                    |
|                                     | hb_mem_pool_free_buf                         | 释放在内存池上分配的内存                |
|                                     | hb_mem_pool_get_info                         | 获取内存池信息                          |
| 共享内存池功能接口:hb_mem_share_pool.c | hb_mem_share_pool_create                  | 创建共享内存池                          |
|                                     | hb_mem_share_pool_destroy                   | 销毁共享内存池                          |
|                                     | hb_mem_share_pool_alloc_buf                 | 在共享内存池上分配buffer                |
|                                     | hb_mem_share_pool_free_buf                  | 释放在共享内存池上分配的内存            |
|                                     | hb_mem_share_pool_get_info                  | 获取共享内存池信息                      |
| 内存队列功能接口:hb_mem_queue.c     | hb_mem_create_buf_queue                     | 创建queue                               |
|                                     | hb_mem_destroy_buf_queue                    | 销毁queue                               |
|                                     | hb_mem_dequeue_buf                          | 取出queue中的空闲可用slot               |
|                                     | hb_mem_queue_buf                            | 在queue中插入buffer                     |
|                                     | hb_mem_request_buf                          | 在queue中请求buffer                     |
|                                     | hb_mem_release_buf                          | 释放queue中请求的buffer                 |
|                                     | hb_mem_cancel_buf                           | 取消对queue的操作                       |
| 内存共享功能接口:hb_mem_share.c     | hb_mem_import_com_buf                       | import com_buf                          |
|                                     | hb_mem_import_graph_buf                     | import graph buffer                     |
|                                     | hb_mem_inc_com_buf_consume_cnt              | 增加com buffer consume count            |
|                                     | hb_mem_dec_consume_cnt                      | 减少consume count                       |
|                                     | hb_mem_dec_consume_cnt_with_vaddr           | 通过虚拟地址减少consume count           |
|                                     | hb_mem_inc_graph_buf_consume_cnt            | 增加graph buffer consume count          |
|                                     | hb_mem_import_com_buf_with_paddr            | 通过物理地址进行import操作              |
|                                     | hb_mem_import_graph_buf_group               | import graphic buffer group             |
|                                     | hb_mem_inc_graph_buf_group_consume_cnt      | 增加graphic buffer group consume count  |

## 内存分配属性说明

| 数据项                                     | 描述                                                                                                                         |
|-------------------------------------------|-----------------------------------------------------------------------------------------------------------------------------|
| HB_MEM_USAGE_CPU_READ_NEVER               | CPU不会读该内存，内存将不会被分配读属性                                                                                     |
| HB_MEM_USAGE_CPU_READ_OFTEN               | CPU经常读该内存，内存将被分配读属性                                                                                         |
| HB_MEM_USAGE_CPU_READ_MASK                | 用于获取read相关属性的掩码，HB_MEM_USAGE_CPU_READ_OFTEN属性优先级高于HB_MEM_USAGE_CPU_READ_NEVER                            |
| HB_MEM_USAGE_CPU_WRITE_NEVER              | CPU不会写该内存，内存将不会被分配写属性                                                                                     |
| HB_MEM_USAGE_CPU_WRITE_OFTEN              | CPU经常写该内存，内存将被分配写属性，该属性会自动添加读属性                                                                 |
| HB_MEM_USAGE_CPU_WRITE_MASK               | 用于获取write相关属性的掩码，HB_MEM_USAGE_CPU_WRITE_OFTEN属性优先级高于HB_MEM_USAGE_CPU_WRITE_NEVER                         |
| HB_MEM_USAGE_HW_CIM                       | 表明该内存用于 camera interface module相关模块，不影响内存分配，用于debug信息                                              |
| HB_MEM_USAGE_HW_PYRAMID                   | 表明该内存用于pyramid相关模块，不影响内存分配，用于debug信息                                                               |
| HB_MEM_USAGE_HW_GDC                       | 表明该内存用于geometric distortion correction相关模块输入buffer，不影响内存分配，用于debug信息                             |
| HB_MEM_USAGE_HW_GDC_OUT                   | 表明该内存用于geometric distortion correction相关模块输出buffer，不影响内存分配，用于debug信息                             |
| HB_MEM_USAGE_HW_STITCH                    | 表明该内存用于stitch相关模块，不影响内存分配，用于debug信息                                                                |
| HB_MEM_USAGE_HW_OPTICAL_FLOW              | 表明该内存用于optical flow相关模块，不影响内存分配，用于debug信息                                                          |
| HB_MEM_USAGE_HW_BPU                       | 表明该内存用于BPU模块，不影响内存分配，用于debug信息                                                                        |
| HB_MEM_USAGE_HW_ISP                       | 表明该内存用于ISP模块，不影响内存分配，用于debug信息                                                                        |
| HB_MEM_USAGE_HW_DISPLAY                   | 表明该内存用于Display相关模块，不影响内存分配，用于debug信息                                                               |
| HB_MEM_USAGE_HW_VIDEO_CODEC               | 表明该内存用于video codec相关模块，不影响内存分配，用于debug信息                                                           |
| HB_MEM_USAGE_HW_JPEG_CODEC                | 表明该内存用于jpeg codec相关模块，不影响内存分配，用于debug信息                                                            |
| HB_MEM_USAGE_HW_VDSP                      | 表明该内存用于vdsp相关模块，不影响内存分配，用于debug信息                                                                  |
| HB_MEM_USAGE_HW_IPC                       | 表明该内存用于ipc相关模块，不影响内存分配，用于debug信息                                                                   |
| HB_MEM_USAGE_HW_PCIE                      | 表明该内存用于PCIe相关模块，不影响内存分配，用于debug信息                                                                  |
| HB_MEM_USAGE_HW_YNR                       | 表明该内存用于YNR相关模块，不影响内存分配，用于debug信息                                                                   |
| HB_MEM_USAGE_HW_MASK                      | 用于获取硬件相关属性的掩码，该掩码下的属性互斥，从上到下优先级依次降低，当指定多个属性或指定为非上述值时，默认指定成"other" |
| HB_MEM_USAGE_MAP_INITIALIZED              | 需要初始化该内存，内存分配后被初始化为0，当未指定MAP_INITIALIZED和MAP_UNINITIALIZED时， DMA heap会被默认初始化，RESERVED heap不会被初始化，该属性和HB_MEM_USAGE_MAP_UNINITIALIZED互斥，且优先级较高 |
| HB_MEM_USAGE_MAP_UNINITIALIZED            | 不需要初始化该内存，内存分配后不被初始化为0，该属性和HB_MEM_USAGE_MAP_INITIALIZED互斥，且优先级较低                        |
| HB_MEM_USAGE_CACHED                       | 表明该块buffer具有cache属性                                                                                                |
| HB_MEM_USAGE_GRAPHIC_CONTIGUOUS_BUF       | 指定graph_buf分配连续的物理内存                                                                                            |
| HB_MEM_USAGE_MEM_POOL                     | 用于表明该buffer用于memory pool，用户分配buffer时无需指定该参数，即使指定该参数，内部默认忽略                              |
| HB_MEM_USAGE_MEM_SHAREPOOL                | 用于表明该buffer用于memory share pool，用户分配buffer时无需指定该参数，即使指定该参数，内部默认忽略                        |
| HB_MEM_USAGE_TRIVIAL_MASK                 | 用于获取杂项相关属性的掩码，该掩码下的属性，除了HB_MEM_USAGE_MAP_INITIALIZED 和HB_MEM_USAGE_MAP_UNINITIALIZED互斥，其他可以并存 |
| HB_MEM_USAGE_PRIV_HEAP_DMA                | 指定从DMA heap分配内存                                                                                                      |
| HB_MEM_USAGE_PRIV_HEAP_RESERVERD          | 指定从Carveout heap分配内存，原始定义(兼容客户已使用的枚举)，不建议使用                                                     |
| HB_MEM_USAGE_PRIV_HEAP_RESERVED           | 指定从Carveout heap分配内存，最新定义，建议使用                                                                             |
| HB_MEM_USAGE_PRIV_HEAP_2_RESERVERD        | 指定从Carveout heap2分配内存，是原始定义(兼容客户已使用的枚举)，不建议使用                                                  |
| HB_MEM_USAGE_PRIV_HEAP_2_RESERVED         | 指定从Carveout heap2分配内存，最新定义，建议使用                                                                            |
| HB_MEM_USAGE_PRIV_MASK                    | 用于获取私有属性的掩码，该掩码下的属性互斥，从上到下优先级依次降低，当指定多个属性时，默认选用高优属性，当指定为非上述值时，默认从DMA heap分配buffer |

## buffer类型说明

| 类型                          | 描述                 |
|-------------------------------|----------------------|
| HB_MEM_BUFFER_TYPE_COMMON      | common buffer       |
| HB_MEM_BUFFER_TYPE_GRAPHIC     | graphic buffer      |
| HB_MEM_BUFFER_TYPE_GRAPHIC_GROUP | graphic buffer group |

## heap介绍

| dts_name   | dts_compatible | heap_mask                       | heap_id                    | flag_mask                                                            | debug_node   |
|------------|----------------|---------------------------------|----------------------------|----------------------------------------------------------------------|--------------|
| ion_carveout | ion-carveout | ION_HEAP_CARVEOUT_MASK          | ION_HEAP_TYPE_CARVEOUT     | HB_MEM_USAGE_PRIV_HEAP_RESERVERD/HB_MEM_USAGE_PRIV_HEAP_RESERVED     |  carveout   |
| ion_reserved | ion-pool   | ION_HEAP_TYPE_CMA_RESERVED_MASK | ION_HEAP_TYPE_CMA_RESERVED | HB_MEM_USAGE_PRIV_HEAP_2_RESERVERD/HB_MEM_USAGE_PRIV_HEAP_2_RESERVED | cma_reserved  |
| ion_cma    | ion-cma        | ION_HEAP_TYPE_DMA_MASK          | ION_HEAP_TYPE_DMA          | HB_MEM_USAGE_PRIV_HEAP_DMA                                           | ion_cma      |

## 内存分配原则

Hbmem在ION驱动中分配内存时，首先会去预期的heap上查询是否剩余的足够内存。如果没有，则需要从其他heap进行分配。具体分配逻辑如下：
（cma_reserved=>carveout=>cma or carveout=>cma_reserved =>cma or cma=>cma_reserved）

## 头文件说明

libhbmem的对外头文件为hb_mem_mgr.h，hbmem.h和hb_mem_err.h。
其中hb_mem_mgr.h为libhbmem主要的对外接口文件，其中定义了libhbmem相关操作的所有接口；hbmem.h中为一些兼容接口，如果新开发应用推荐使用对外头文件hb_mem_mgr.h；
hb_mem_err.h中定义了libhbmem返回的错误码。

## 静态库和动态库说明

libhbmem编译输出一个静态库文件libhbmem.a，和三个动态库文件libhbmem.so.1.0.0, libhbmem.so.1, libhbmem.so。 \
其中libhbmem.so.1, libhbmem.so为动态软链接；libhbmem.so.1.0.0为本体，数字后缀为版本号。
