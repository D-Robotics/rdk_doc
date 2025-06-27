# hbmem sample

## 功能概述

hbmem api使用说明，包括com buffer、graphic buffer、graphic buffer group、queue、pool、share pool的创建使用，多进程共享等。

## 软件架构说明

本sample基于libhbmem API实现，调用libhbmem提供的API，实现不同buffer类型的内存申请和进程间共享。

![hbmem_sample_block_diagram](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/03_multimedia_development/02_S100/hbmem/hbmem_sample_block_diagram.png)

## 代码位置及目录结构

```text
1. 代码位置：/app/communication_demo/hbmem_demo/sample_hbmem
2. 目录结构：
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

## API流程说明

libhbmem库调用ion进行buffer分配

![hbmem_sample_call_sequence](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/03_multimedia_development/02_S100/hbmem/hbmem_sample_call_sequence.png)

com buffer、graph buffer、graphic buffer group 分配流程图

![hbmem_alloc_buf](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/03_multimedia_development/02_S100/hbmem/hbmem_alloc_buf.png)

queue使用流程图

![hbmem_queue](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/03_multimedia_development/02_S100/hbmem/hbmem_queue.png)

pool使用流程图

![hbmem_pool](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/03_multimedia_development/02_S100/hbmem/hbmem_pool.png)

share pool 使用流程图

![hbmem_share_pool](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/03_multimedia_development/02_S100/hbmem/hbmem_share_pool.png)

## 编译

### 编译环境

本sample支持在板端直接编译。

### 编译说明

本sample主要依赖libhbmem提供的头文件：

```c
#include <hb_mem_mgr.h>
#include <hb_mem_err.h>
```

编译依赖的库有如下：

```makefile
LIBS += -lhbmem -lpthread -lalog -ldl -lstdc++
```

### 编译命令

```bash
# 进入sample所在目录：
cd /app/communication_demo/hbmem_demo/sample_hbmem
# 编译本sample:
make
# 输出文件:
/app/communication_demo/hbmem_demo/sample_hbmem/libhbmem_sample
```

## 运行

### 支持平台

- RDK S100/RDK S100P

### 运行指南

#### 运行方法

创建com buffer

```bash
/app/communication_demo/hbmem_demo/sample_hbmem/libhbmem_sample -m 1
```

创建cache，noncached com buffer，赋值，比较耗时

```bash
/app/communication_demo/hbmem_demo/sample_hbmem/libhbmem_sample -m 2
```

在各个heaps里，创建com buffer

```bash
/app/communication_demo/hbmem_demo/sample_hbmem/libhbmem_sample -m 3
```

创建graph buffer

```bash
/app/communication_demo/hbmem_demo/sample_hbmem/libhbmem_sample -m 4
```

在各个heaps里，创建graph buffer

```bash
/app/communication_demo/hbmem_demo/sample_hbmem/libhbmem_sample -m 5
```

创建graph buffer group

```bash
/app/communication_demo/hbmem_demo/sample_hbmem/libhbmem_sample -m 6
```

在各个heaps里，创建graph buffer group

```bash
/app/communication_demo/hbmem_demo/sample_hbmem/libhbmem_sample -m 7
```

进程内common buffer共享

```bash
/app/communication_demo/hbmem_demo/sample_hbmem/libhbmem_sample -m 8
```

进程内common buffer共享，consume_cnt

```bash
/app/communication_demo/hbmem_demo/sample_hbmem/libhbmem_sample -m 9
```

进程间common buffer共享

```bash
/app/communication_demo/hbmem_demo/sample_hbmem/libhbmem_sample -m 10
```

进程内graphic buffer共享

```bash
/app/communication_demo/hbmem_demo/sample_hbmem/libhbmem_sample -m 11
```

进程内graphic buffer共享，consume_cnt

```bash
/app/communication_demo/hbmem_demo/sample_hbmem/libhbmem_sample -m 12
```

进程间graphic buffer共享

```bash
/app/communication_demo/hbmem_demo/sample_hbmem/libhbmem_sample -m 13
```

进程内graphic buffer group共享

```bash
/app/communication_demo/hbmem_demo/sample_hbmem/libhbmem_sample -m 14
```

进程内graphic buffer group共享，consume_cnt

```bash
/app/communication_demo/hbmem_demo/sample_hbmem/libhbmem_sample -m 15
```

进程间graphic buffer group共享

```bash
/app/communication_demo/hbmem_demo/sample_hbmem/libhbmem_sample -m 16
```

进程间share pool 共享

```bash
/app/communication_demo/hbmem_demo/sample_hbmem/libhbmem_sample -m 17
```

share pool使用

```bash
/app/communication_demo/hbmem_demo/sample_hbmem/libhbmem_sample -m 18
```

buffer queue 使用

```bash
/app/communication_demo/hbmem_demo/sample_hbmem/libhbmem_sample -m 19
```

pool使用

```bash
/app/communication_demo/hbmem_demo/sample_hbmem/libhbmem_sample -m 20
```

在graphic buffer与common buffer之间实现buffer转换

```bash
/app/communication_demo/hbmem_demo/sample_hbmem/libhbmem_sample -m 21
```

增加common buffer用户态引用计数

```bash
/app/communication_demo/hbmem_demo/sample_hbmem/libhbmem_sample -m 22
```

增加graphic buffer用户态引用计数

```bash
/app/communication_demo/hbmem_demo/sample_hbmem/libhbmem_sample -m 23
```

### 运行参数说明

- `-m` 指定sample用例，具体见运行结果展示

### 测试结果说明

- 如果log最后以`xxxx done`结尾，则表示sample执行成功

### 运行结果展示

#### 创建com buffer

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

（其他运行结果展示部分保持原有文本结构，仅作格式转换和关键词替换，此处略去重复内容）
