# FAQ

## FAQ001: 内存概念

- 问：三种ion预留的内存，其他两种内存（cma_reserved和carveout）的用途是什么？
    - 答：这两种内存本质上都属于carveout，区分开来的原因是：一个carveout主要用于系统中bpu、codec内存分配，vio中的模块从另外一个carveout分配内存。
- 问：是否因为部分IP需要连续的物理内存，所以需要使用Hbmem？
    - 答：对。

## FAQ002: 结构体说明

- 问：com_buf和graph_buf含义上有什么区别？
    - 答：com_buf是单块的连续的buffer，他的结构比较简单。graph_buf的话比较复杂，分多个plane。
- 问：com_buf是否有长和宽的属性，还是说只有一个size？
    - 答：只有一个size，长和宽的属性需要去各个模块取；但是graph_buf是有长和宽的属性的。
- 问：com_buf和graph_buf这两个结构体只是封装形式不一样，graph_buf的结构体的语义更多，本质上这两个是没有什么区别的是吗？
    - 答：对。
- 问：offset首先需要我们设置偏移才会有吗？这个offset是申请内存时候的基址吗？
    - 答：对，需要自行设置。比如说拿到一个buffer，但是你传给另外一个进程，希望传送一个偏移地址过去，就可以在这里面设置一个offset。然后这个offset就是通过buffer传递，透传到另一个进程。申请出来的buffer，所有的offset都是零。offset用于进程之间传输时，附带透传信息，用于表示buffer中的偏移地址。
- 问：com_buf和 graph_buf的区别，为什么增加一个graph_buf？
    - 答：com_buf在分配时只可指定size和常规分配属性；graph_buf在分配时可指定宽、高、格式、跨距、是否连续以及其他常规分配属性；graph_buf相较于com_buf带有更多的buffer信息，便于用户直接分配RGB/YUV/RAW格式的buffer。

## FAQ003: 操作注意点

- 问：是不是每个进程都需要进行module_open？是相当于用户多一次操作吗？可以重复打开吗？里面带有保护吗？
    - 答：对，每个进程都需要进行module_open，由于每个进程都是由不同的工程师开发的，所以需要每个进程都进行一次module_open的操作。在一个进程内，不建议做多次open，且没有必要，同时open需要有相应的close。因为，正常情况下，如果申请的内存没有free，会在module_close的时候将内存free，但是如果少了对应的close操作，就会导致内存无法free。但是，在进程销毁的时候，会将所有ION申请的内存进行释放。
- 问：fd的数量限制可以使用Linux ulimit进行修改扩充吗？
    - 答：可以通过ulimit的方式去提高fd的数量。有些场景中，fd并不是真正不够用，而是一些接口的错误操作，导致fd出现泄漏。

## FAQ004: 函数接口

- 问：这些接口函数是不是相当于在驱动层进行了一个封装？
    - 答：不仅仅是进行了封装。Hbmem申请的buffer，在各个模块之间都是通用的，而且可以通过虚拟地址拿到所有的相关信息。比如说从vio那边拿到了一块buffer之后，就可以把它转成com_buf，然后再给到另外一个模块去使用，比如说BPU。
- 问：是否可以通过get_buffer_info这个接口获得一些buffer的信息？
    - 答：可以。此外可以通过hb_mem_get_com_buffer_info_with_vaddr这个接口，利用虚拟地址得到相关的com_buf的信息。
- 问：invalid是一个阻塞接口吗？在两个进程中invalid是否会导致死锁？超时之后会怎么样？
    - 答：是一个阻塞接口；不会存在死锁的情况，函数内部会处理；不会存在超时的情况，该函数直到操作完成才会返回。

## FAQ005: 内存分配

- 问：接口申请得到的内存是否物理连续？
    - 答：首先看，选择的heap是哪一个，cma和carveout对应的heap申请到的内存都是物理连续的。其次，可以通过参数来选择heap。
- 问：提供的API接口是不是等同于比通用接口多了一个进程间共享的这样一个接口，如果不需要进程间共享，是否可以使用malloc？
    - 答：如果使用本身的malloc，申请得到的内存物理不连续。
- 问：内存分配得到的buffer支持自动格式转换吗？S100X上有支持硬件转换的功能吗？
    - 答：不支持自动格式转换，硬件格式转换可以尝试使用CIM和Pyramid。
- 问：如何指定heap申请内存？
    - 答：在申请时使用的flags里面，带上以下属性的一种:
        ``` shell
        HB_MEM_USAGE_PRIV_HEAP_DMA;
        HB_MEM_USAGE_PRIV_HEAP_RESERVED;
        HB_MEM_USAGE_PRIV_HEAP_2_RESERVED;
        ```
- 问：假设申请的地址仅限于AB进程cpu访问，从性能来看，需要申请cacheable的吗？如果cacheable的话，b进程只读的时候需要做invalidate吗？
    - 答：如果只是在CPU之间使用，那推荐申请cacheable的内存。开启cacheable之后，内存的访问速率会加快很多，预期1000倍以上。只是在CPU之间进行读写访问，不需要进行invalid和flush等操作，cache一致性由CPU的MESI协议保证。
- 问：假设内存是要跑某个ip，比如gdc/codec这种，b进程的cpu不会访问，那可以直接传物理地址吗，还是也必须走buffer？
    - 答：如果直接使用物理地址，而不使用ION申请buffer，无法保证生命周期中这段地址只有一个用户在使用。因为该内存可能会被其他用户申请走，所以在硬件或者软件使用buffer时，需要保证在buffer的生命周期中。
- 问：在进行yuv内存申请的时候，对应的虚拟地址为什么要转换成uint8_t？
    - 答：将虚拟地址存放在一个uint8_t *的指针中，主要是为了考虑兼容性的问题。因为有些系统可能是32位的，有些系统为64位。
- 问：com_buf和graphic_buf中的offset的作用是什么？
    - 答：offset字段可以表明数据相对于当前buffer中虚拟地址或者物理地址的偏移量，利用offset和虚拟地址或者物理地址可以直接操作期望的数据。
- 问：invalid和flush时出现"No need toinvalidate for uncached buffer"警告，是什么意思？
    - 答：该警告表示针对uncacheable的内存不需要使用invalid和flush等操作。可以通过flags查看对应的内存属性确认该内存是否是cacheable的。
- 问：如果想用hbmem_dmacpy 对内存申请有要求么 比如 hb_mem_alloc_com_buf 还是 hbmem_alloc 上？
    - 答：针对从Hbmem分配的内存可以直接使用相应的虚拟地址；对于非Hbmem分配的内存，通过接口memcopy实现内存拷贝。
- 问：如何查看内存的属性？
    - 答：如果用户当前可以获得common buffer或者graphic buffer结构体，可以根据其中的flags进行确认内存属性；如果用户当前只有虚拟地址，可以通过接口hb_mem_get_buf_info获取对应flags。
- 问：目前支持某个heap无法分配时自动从另一个heap分配内存，为什么还存在无法分配内存的情况？
    - 答：所谓的某一个heap分配不出来去另一个heap分配，不是说把两个heap拼凑起来再次分配，而是去另一个heap空间分配，仍然受到这个heap的总大小限制，每个heap是单独管理的。因此可分配的大小受到每个heap可用大小的限制。

## FAQ006: 内存共享

- 问：内存在进程间共享时，进程间传递的是哪些信息？
    - 答：传送一个完整的com_buf或graph_buf结构体信息。
- 问：com_buf从一个进程传到另一个进程需要通过import接口转一下，才能使用吗？
    - 答：对。
- 问：improt是不是对应有个释放的，对应计数减一？
    - 答：对。所有的import都要做free操作。如果一直import，而没有free，就会导致fd一直增加，超过1024。
- 问：不同的硬件是有不同的内存管理API吗？他们之间的内存可以相互转换？且其他硬件模块都能访问？
    - 答：是的。
- 问：多进程内存共享，A进程如何知道B进程import后有没有释放？通过hb_mem_get_share_info查询吗？
    - 答：可以使用接口hb_mem_wait_share_status等待client count变为import之前，可以在import之前使用接口hb_mem_get_share_info查询当前的client count。
- 问：在内存共享import的时候，为什么需要in_buf与out_buf两个buffer？
    - 答：首先是，内存共享时，需要重新对该buffer进行dma_buf创建，并关联新的fd。同时共享的内存并不一定占用整个buffer的大小。同时import的时候会增加底层该buffer的引用计数。
- 问：为什么dma copy中小于PAGE_SIZE的内存使用memcopy而不用dmacopy？
    - 答：由于之前dma copy对于内存的大小有限制，所以在拷贝内存小于4K的时候使用memcopy。现在dma copy对于内存大小没有限制，最小支持1B
- 问：import之后buffer不再使用为什么需要释放？
    - 答：import本质上是增加对应buffer的引用计数，而hb_mem_free_buf本质上是减少对应buffer的引用计数。
- 问：一个线程或者一个进程获得一个com_buf如何确认buffer的有效性？
    - 答：用户使用时，存在一种情况当前线程或者进程需要检查获取的buffer是否有效，如果无效则自己去申请buffer。 \
         针对同一个进程中的同步线程的场景下，可以通过hb_mem_get_com_buf/hb_mem_get_graph_buf接口获取libhbmem中保存的该buffer的信息，并通过匹配相应的信息判断buffer是否是申请得到的。 \
         针对不同进程之间的共享的场景，由于不同进程libhbmem的信息是独立的，所以无法通过libhbmem的接口进行信息确认。
- 问：进程间共享buffer时，如何只共享buffer的一部分？
    - 答：当进程A申请了8K大小的buffer时，但是只想将其中后4K共享给进程B。进程A只需在传递buffer时，在要传递的buffer中，将物理地址设置为PA+4KB，将size设置为4K。这样在进程B import该buffer之后，获取到的虚拟地址就是buffer映射后起始虚拟地址加上4KB的偏移量。
需要注意，设置offset不能起到上述的作用，设置offset之后，在buffer import后返回的buffer中，虚拟地址是mmap之后的起始虚拟地址。
- 问：地址不确定是commonbuffer或者graphicbuffer怎么获取id，同时进程间必须调用import函数吗，不确定是那种类型怎么办？
    - 答：可以通过调用接口hb_mem_get_buf_type_and_buf_with_vaddr获取虚拟地址对应的buffer类型和buffer结构体数据，并从中获取share id。
同进程之间共享buffer，必须先进行import操作。不然无法保证在正常生命周期中安全使用该buffer，同时不同进程中的虚拟地址无法直接使用，需要import之后生成新的虚拟地址。
- 问：多个地址的情况下，不确定是否地址统一通过commonbuffer或者graphicbuffer分配，还是都单独分配的情况应该怎么使用？
    - 答：通过调用hb_mem_get_buf_type_and_buf_with_vaddr接口虚拟地址对应的buffer类型和buffer结构体数据import之后使用，同一个buffer支持多次import。
- 问：某些特殊场景，a进程申请的地址，a不会再用，传给b进程，b会管理，这种场景下，可以b来真正的释放这块内存吗？
    - 答：A进程在确保B进程import之后就可以释放内存，这样B进程释放内存之后，该内存就会直接释放。换句话说，该buffer的生命周期转由B进程控制。

## FAQ007: 内存队列管理

- 问：前面提到内存队列只是单进程的，但是后面提到共享内存是可以用作进程间通信的，内存队列的管理具体是怎么样一个含义？
    - 答：内存共享和内存队列管理其实是两个功能。内存共享主要是指，当某个IP拿到一块buffer的时候，可以直接把它传给另外一个进程。然后另外一个进程可以利用import的接口直接拿到它对应的一个信息，实现内存共享。内存队列管理，就是在进程内进行内存的管理。例如可以在server端创建一个buffer queue，然后将buffer添加到队列中，对应的client端可以去dequeue buffer。类似于消息队列。同时这个队列并没有强制要求放buffer信息，放其他信息都是可以的。

## FAQ008: 内存池

- 问：如何理解先申请一大块内存再去使用，然后利用自己的内存管理队列管理？
    - 答：内存池分配函数，首先会从某个heap中分配得到一大块内存。后续可以在这块内存中去反复分配，它分配时不需要进入内核态处理，所以分配效率是比直接调用对应的API接口要高很多。free的操作相当于将内存还给内存池，最后使用完之后可以将内存池销毁。对于分配得到的多块buffer，如果需要进行队列管理，可以自己管理，或者利用我们提供的管理队列去管理。
- 问：使用内存池，是否需要提前预判一下，总共需要多少内存，在应用层预留出来？
    - 答：是这样的。
- 问：Memory pool是否能进行进程间操作？
    - 答：这个不是很推荐，本质来说，它是一块buf。
- 问：pool申请大小是否有要求？
    - 答：没有要求，需要多少分配多少，只要剩余空间足够。同时分配以pagesize对齐。

## FAQ009: 软件性能

- 问：如何保证Hbmem的稳定性？
    - 答：Hbmem本身是基于ION实现，我们会针对ion接口和Hbmem的接口做相关的异常case、长稳和压力测试来保证其功能，此外还会在vio、bpu、codec等场景中开展分配功能的验证。
- 问：对于用户分配了未释放的场景如何保证？
    - 答：首先对于单个进程退出时不释放的话，我们系统是会自动内存回收；其次如果单个进程运行时不释放的话，可以查看相关sys节点检查该用户的内存分配情况。

## FAQ010: 内存泄漏

- 问：进程销毁的时候不会导致内存泄漏吗？
    - 答：正常情况下，进程销毁时所有的内存都会被回收，不会存在任何的泄漏。但是有一种情况，比如说在A进程中分配了内存，然后共享给B进程，这导致内存使用计数加一。如果这时候销毁A进程，就不会去释放对应的内存。内存会留在B进程中，会在B进程销毁的时候被释放。

## FAQ011: 软件调试

- 问：这些接口在使用过程中，是否有诊断工具？
    - 答：有相应的诊断工具，可以利用sys节点查看。
        ```shell
        #对应HB_MEM_USAGE_PRIV_HEAP_DMA
        cat /sys/kernel/debug/ion/heaps/ion_cma
        ```

        ```shell
        #对应HB_MEM_USAGE_PRIV_HEAP_RESERVED
        cat /sys/kernel/debug/ion/heaps/carveout
        ```

        ```shell
        #对应HB_MEM_USAGE_PRIV_HEAP_2_RESERVED
        cat /sys/kernel/debug/ion/heaps/cma_reserved
        ```

        ```shell
        #对应获取所有heap信息
        cat /sys/kernel/debug/ion/heaps/all_heap_info
        ```

        ```shell
        cat /sys/kernel/debug/ion/clients/"client名称"
        ```

## FAQ012: 模块使用

- 问：camera、金字塔等模块输出的都是com_buf？
    - 答：对，现在系统上的模块使用的都是com_buf，内存都是连续的。
- 问：hbmem.h文件中的hbmem_alloc接口不建议使用吗？
    - 答：不建议使用，这是原来的接口，留下来是为了兼容性。如果以前有一套程序基于Hbmem，就可以使用这个，如果没有，使用新的接口就行。
