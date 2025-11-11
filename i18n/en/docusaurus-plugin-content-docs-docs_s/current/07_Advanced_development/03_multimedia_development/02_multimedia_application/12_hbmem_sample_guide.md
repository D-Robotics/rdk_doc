# hbmem sample usage instructions

## Function Overview

Instructions for using hbmem APIs, including creation and usage of com buffer, graphic buffer, graphic buffer group, queue, pool, share pool, and multi-process sharing.

## Software Architecture Description

This sample is implemented based on the libhbmem API, invoking APIs provided by libhbmem to allocate memory for different buffer types and enable inter-process sharing.

![hbmem_sample_block_diagram](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/03_multimedia_development/02_S100/hbmem/hbmem_sample_block_diagram.png)

## Code Location and Directory Structure

```text
1. Code location: /app/communication_demo/hbmem_demo/sample_hbmem
2. Directory structure:
    ./
    ├── Makefile
    ├── sample_alloc.c
    ├── sample.c
    ├── sample_common.c
    ├── sample_common.h
    ├── sample_pool.c
    ├── sample_queue.c
    ├── sample_share.c
    └── sample_share_pool.c
```

## API Workflow Description

The libhbmem library calls ION to allocate buffers.

![hbmem_sample_call_sequence](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/03_multimedia_development/02_S100/hbmem/hbmem_sample_call_sequence.png)

Allocation flowchart for com buffer, graph buffer, and graphic buffer group:

![hbmem_alloc_buf](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/03_multimedia_development/02_S100/hbmem/hbmem_alloc_buf.png)

Queue usage flowchart:

![hbmem_queue](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/03_multimedia_development/02_S100/hbmem/hbmem_queue.png)

Pool usage flowchart:

![hbmem_pool](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/03_multimedia_development/02_S100/hbmem/hbmem_pool.png)

Share pool usage flowchart:

![hbmem_share_pool](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/03_multimedia_development/02_S100/hbmem/hbmem_share_pool.png)

## Compilation

### Compilation Environment

This sample supports direct compilation on the board.

### Compilation Instructions

This sample primarily depends on header files provided by libhbmem:

```c
#include <hb_mem_mgr.h>
#include <hb_mem_err.h>
```

Compilation dependencies include the following libraries:

```makefile
LIBS += -lhbmem -lpthread -lalog -ldl -lstdc++
```

### Compilation Commands

```bash
# Enter the sample directory:
cd /app/communication_demo/hbmem_demo/sample_hbmem
# Compile the sample:
make
# Output file:
/app/communication_demo/hbmem_demo/sample_hbmem/libhbmem_sample
```

## Execution

### Supported Platforms

- RDK S100 / RDK S100P

### Running Guide

#### Execution Methods

Create a com buffer:

```bash
/app/communication_demo/hbmem_demo/sample_hbmem/libhbmem_sample -m 1
```

Create cached and non-cached com buffers, perform assignment, and measure execution time:

```bash
/app/communication_demo/hbmem_demo/sample_hbmem/libhbmem_sample -m 2
```

Create com buffers in various heaps:

```bash
/app/communication_demo/hbmem_demo/sample_hbmem/libhbmem_sample -m 3
```

Create a graphic buffer:

```bash
/app/communication_demo/hbmem_demo/sample_hbmem/libhbmem_sample -m 4
```

Create graphic buffers in various heaps:

```bash
/app/communication_demo/hbmem_demo/sample_hbmem/libhbmem_sample -m 5
```

Create a graphic buffer group:

```bash
/app/communication_demo/hbmem_demo/sample_hbmem/libhbmem_sample -m 6
```

Create graphic buffer groups in various heaps:

```bash
/app/communication_demo/hbmem_demo/sample_hbmem/libhbmem_sample -m 7
```

Intra-process common buffer sharing:

```bash
/app/communication_demo/hbmem_demo/sample_hbmem/libhbmem_sample -m 8
```

Intra-process common buffer sharing with consume_cnt:

```bash
/app/communication_demo/hbmem_demo/sample_hbmem/libhbmem_sample -m 9
```

Inter-process common buffer sharing:

```bash
/app/communication_demo/hbmem_demo/sample_hbmem/libhbmem_sample -m 10
```

Intra-process graphic buffer sharing:

```bash
/app/communication_demo/hbmem_demo/sample_hbmem/libhbmem_sample -m 11
```

Intra-process graphic buffer sharing with consume_cnt:

```bash
/app/communication_demo/hbmem_demo/sample_hbmem/libhbmem_sample -m 12
```

Inter-process graphic buffer sharing:

```bash
/app/communication_demo/hbmem_demo/sample_hbmem/libhbmem_sample -m 13
```

Intra-process graphic buffer group sharing:

```bash
/app/communication_demo/hbmem_demo/sample_hbmem/libhbmem_sample -m 14
```

Intra-process graphic buffer group sharing with consume_cnt:

```bash
/app/communication_demo/hbmem_demo/sample_hbmem/libhbmem_sample -m 15
```

Inter-process graphic buffer group sharing:

```bash
/app/communication_demo/hbmem_demo/sample_hbmem/libhbmem_sample -m 16
```

Inter-process share pool sharing:

```bash
/app/communication_demo/hbmem_demo/sample_hbmem/libhbmem_sample -m 17
```

Share pool usage:

```bash
/app/communication_demo/hbmem_demo/sample_hbmem/libhbmem_sample -m 18
```buffer queue usage

```bash
/app/communication_demo/hbmem_demo/sample_hbmem/libhbmem_sample -m 19
```

pool usage

```bash
/app/communication_demo/hbmem_demo/sample_hbmem/libhbmem_sample -m 20
```

Implement buffer conversion between graphic buffer and common buffer

```bash
/app/communication_demo/hbmem_demo/sample_hbmem/libhbmem_sample -m 21
```

Increment user-space reference count of common buffer

```bash
/app/communication_demo/hbmem_demo/sample_hbmem/libhbmem_sample -m 22
```

Increment user-space reference count of graphic buffer

```bash
/app/communication_demo/hbmem_demo/sample_hbmem/libhbmem_sample -m 23
```

### Runtime Parameter Description

- `-m` specifies the sample test case; refer to the execution output for details.

### Test Result Description

- If the log ends with `xxxx done`, it indicates the sample executed successfully.

### Execution Output Examples

#### Create com buffer

```text
/app/communication_demo/hbmem_demo/sample_hbmem/libhbmem_sample -m 1
sample_mode = 1
=================================================
Ready to sample_alloc_com_buf
alloc com buf, share_id: 16
[1752:1752] Do system command cat /sys/kernel/debug/ion/clients/1752*.
        heap_name:    size_in_bytes :  handle refcount :    handle import :       buffer ptr :  buffer refcount :  buffer share id : buffer share count
        ion_cma:           400000 :                1 :                1 :         fd48e1ab :                3:               16 :                1
[1752:1752] Do system command cat /sys/kernel/debug/ion/heaps/ion_cma | grep -w 1752 | grep -w libhbmem_sample.
        libhbmem_sample             1752          4194304
        libhbmem_sample             1752            other          4194304 0
[1752:1752] Result 0.
free com buf
[1752:1752] Do system command cat /sys/kernel/debug/ion/clients/1752*.
        heap_name:    size_in_bytes :  handle refcount :    handle import :       buffer ptr :  buffer refcount :  buffer share id : buffer share count
[1752:1752] Do system command cat /sys/kernel/debug/ion/heaps/ion_cma | grep -w 1752 | grep -w libhbmem_sample.
[1752:1752] Result 256.
sample_alloc_com_buf done
=================================================
```

(Other execution output examples retain the original text structure, with only formatting and keyword adjustments applied; repetitive content omitted here.)