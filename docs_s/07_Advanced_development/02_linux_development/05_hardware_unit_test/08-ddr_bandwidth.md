---
sidebar_position: 8
---

# DDR 带宽测试

## 测试原理

DDR（ Double Data Rate）内存带宽是衡量内存系统数据传输能力的关键性能指标，它反映了内存可以在单位时间内传输的数据量，通常以 MB/s 或 GB/s 为单位，通过执行特定的内存操作并测量其执行时间，可以估算内存的带宽性能，主要的测试操作有： Copy、 Scale、 Add、 Triad。

**1. Copy（拷贝）操作：**

```c
for (j=0; j<STREAM_ARRAY_SIZE; j++)
    c[j] = a[j];
```

- 这个操作将数组 a[] 的数据复制到数组 c[]。它只涉及内存的读取和写入，因此它测量的是单纯的内存带宽。

**2. Scale（缩放）操作：**

```c
for (j=0; j<STREAM_ARRAY_SIZE; j++)
    b[j] = scalar * c[j];
```

- 这个操作将数组 c[] 的每个元素乘以一个常数 scalar，然后将结果存储到数组 b[] 中。这一操作通过读取 c[] 和写入 b[] 来测量带宽。

**3. Add（加法）操作：**

```c
for (j=0; j<STREAM_ARRAY_SIZE; j++)
    c[j] = a[j] + b[j];
```

- 该操作将数组 a[] 和 b[] 中的对应元素相加，并将结果存储到数组 c[] 中。这是一种读取两个数组并写入到另一个数组的操作。

**4. Triad（三元组）操作：**

```c
for (j=0; j<STREAM_ARRAY_SIZE; j++)
    a[j] = b[j] + scalar * c[j];
```

- 这是一个更复杂的操作，它将数组 b[] 中的元素与数组 c[] 中的元素按照常数 scalar 进行三元组加法，并将结果存储到数组 a[] 中。

## 准备工作

**1.** 确认 DDR 类型和频率：不同类型的 DDR 内存（如 LPDDR4 ）和其频率会影响带宽的测试结果，使用命令 `cat /sys/class/boardinfo/ddr_type` 查看 DDR 状态信息。

```bash
root@buildroot:/# cat /sys/class/boardinfo/ddr_type
LPDDR5
```

目前 RDKS100 DDR 频率为 6400MHz。

**2.** 确认在 /app/multimedia_samples/chip_base_test/08_ddr_bandwidth/ 路径下存在 stream 测试文件，如不存在，可在路径下重新编译生成：

```shell
root@buildroot:/# cd /app/multimedia_samples/chip_base_test/08_ddr_bandwidth
root@buildroot:/# gcc -O3 -fopenmp -DNTIMES=100 stream.c -lgomp -o stream
```

## 测试方法

确保已完成准备工作后，运行测试命令：

```shell
root@buildroot:/# ./stream
```

等待 10 秒左右后得到以下结果：

```yaml
-------------------------------------------------------------
STREAM version $Revision: 5.10 $
-------------------------------------------------------------
This system uses 8 bytes per array element.
-------------------------------------------------------------
Array size = 10000000 (elements), Offset = 0 (elements)
Memory per array = 76.3 MiB (= 0.1 GiB).
Total memory required = 228.9 MiB (= 0.2 GiB).
Each kernel will be executed 100 times.
 The *best* time for each kernel (excluding the first iteration)
 will be used to compute the reported bandwidth.
-------------------------------------------------------------
Number of Threads requested = 6
Number of Threads counted = 6
-------------------------------------------------------------
Your clock granularity/precision appears to be 1 microseconds.
Each test below will take on the order of 3674 microseconds.
   (= 3674 clock ticks)
Increase the size of the arrays if this shows that
you are not getting at least 20 clock ticks per test.
-------------------------------------------------------------
WARNING -- The above is only a rough guideline.
For best results, please be sure you know the
precision of your system timer.
-------------------------------------------------------------
-------------------------------------------------------------
Function    Best Rate MB/s  Avg time     Min time     Max time
Copy:           47239.8     0.003658     0.003387     0.004648
Scale:          48721.4     0.003657     0.003284     0.005366
Add:            46592.6     0.006095     0.005151     0.008138
Triad:          46431.4     0.006107     0.005169     0.008374
-------------------------------------------------------------
Solution Validates: avg error less than 1.000000e-13 on all three arrays
-------------------------------------------------------------
```

**关键信息说明：**

测试结果中的 `Copy`，`Scale`，`Add`，`Triad` 四项即带宽结果，该四项数据的测试原理如下图所示：

![DDR_test_principle](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/images_to_upload/DDR_test_principle.png)

- **Copy (复制)：** 它先访问一个内存单元读出其中的值，再将值写入到另一个内存单元。
  - 测试描述：在 Copy 测试中，系统将一个数组的内容复制到另一个数组，这是内存带宽的最基本测试，主要考察的是系统在执行内存到内存的简单数据复制时的性能。
  - 带宽结果： 47578.1 MB/s
- **Scale (乘法)：** 先从内存单元读出其中的值，作一个乘法运算，再将结果写入到另一个内存单元。
  - 测试描述： Scale 不仅涉及内存带宽，还需要 CPU 执行计算任务，因此更能体现处理器和内存之间的协作性能。
  - 带宽结果： 47904.1 MB/s
- **Add (加法)：** 先从内存单元读出两个值，做加法运算，再将结果写入到另一个内存单元。
  - 测试描述： Add 测试模拟了两个数组相加并将结果存储到第三个数组中，这测试了 CPU 和内存在并行操作时的带宽需求。
  - 带宽结果： 44642.0 MB/
- **Triad (组合操作)：** 将 Copy、 Scale、 Add 三种操作组合起来进行测。具体操作方式是：先从内存单元中读两个值 a、 b ，对其进行乘加混合运算（ a + 因子 * b ） ，将运算结果写入到另一个内存单元
  - 测试描述： Triad 不仅涉及两个数组相加，还将结果与另一个数组进行缩放，是一个同时进行计算、加法和内存访问的复合操作。
  - 带宽结果： 42987.3 MB/s

- **输出结果中的数值的含义：**

  - **Best Rate MB/s（最佳速率）：** 在操作中达到的最高内存传输速率，以兆字节 / 秒（ MB/s）为单位。表示峰值性能。

  - **Avg time（平均时间）：** 每次操作的平均时间，以秒为单位，表示性能的平均延迟时间。

  - **Min time（最小时间）：** 操作的最短时间，以秒为单位，表示在某一次操作中的最佳性能。

  - **Max time（最大时间）：** 操作的最长时间，以秒为单位，表示某一次复制操作中的最差性能。

## 测试指标

DDR 带宽通常基于内存的频率和总线宽度来计算，公式如下：

```shell
带宽 (MB/s) =  内存时钟频率 (MHz) * 2 * 总线宽度 (bit) / 8
```

S100 平台下， DDR 内存频率为 4266 MHz（ 2133x2 ），内存总线宽度为 96 位（即 12 字节），则理论带宽为：

```shell
带宽 (MB/s) = 6400 MHz × 12 Byte = 76800 MB/s

```

### 分数标准

实际 DDR 带宽的测试结果通常会低于理论带宽，实际带宽的标准应为理论带宽的 60%-70%，以 S100 平台 为例，理论带宽为 76800 MB/s，因此标准带宽为：

```shell
分数标准 =  ddr_score(76800) * 0.6 = 46080
```

### 测试结果

以接近实际 DDR 使用方式的 `Triad` 操作为例，其分数结果超过了标准带宽 46080 MB/s，因此测试结果符合 DDR 带宽标准且达到预期的性能要求。
