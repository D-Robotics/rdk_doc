---
sidebar_position: 1
---

# CPU-BPU-DDR 压力测试

## 测试原理

CPU-BPU-DDR 压力测试的测试原理主要涉及对 CPU、 BPU 和 DDR 在高负荷条件下的性能和稳定性进行评估，以下是这些组件压力测试的一些基本原理：

**1. CPU DDR 压力测试**

- **测试目标**：使用 `stressapptest` 工具模拟高负载环境，进行大量计算、数据处理和内存操作，测试系统在多线程并发任务下的性能。
- **测试目的**：验证 CPU 在长时间高负载下的稳定性与性能，确保 CPU 和 DDR 能在高负荷运行时维持正常工作，避免崩溃、过热或性能下降等问题。

- **stressapptest CPU 压测原理：** CPU 压测主要通过多线程并发执行一些计算密集型的任务来实现，这些任务包括：计算密集型任务、多线程并行执行等。
	- 线程创建： `stressapptest` 通过 `pthread_create()` 来创建多个线程，每个线程执行计算任务，线程的数量通常由用户通过 -C 参数指定。

	```C
	for (int i = 0; i < num_threads; i++) {
		pthread_create(&threads[i], NULL, cpu_stress_function, (void*)i);
	}
	```

	- 计算密集型任务：每个线程会执行一些计算密集型的操作，例如浮点数计算、内存读写等，这会消耗 CPU 资源。

	```C
	void* cpu_stress_function(void* arg) {
		while (true) {
				// Perform some CPU-intensive calculations
				double a = 3.14159265358979;
				for (int i = 0; i < 1000000; i++) {
						a = a * a * 3.14159;  // Simulate a calculation
				}
		}
		return NULL;
	}
	```

	- 线程同步和管理：通过 `pthread_join()` 等函数确保线程的正确执行和同步。

- **stressapptest DDR 压测原理：** DDR 内存 的压力测试主涉及大量的内存分配、访问和数据交换，通过高内存使用、频繁的内存读写操作、以及内存带宽的消耗来进行。
	- 内存分配： 在内存压力测试中， stressapptest 会根据 -M 参数（例如 -M 8192 ）来分配指定大小的内存。

	```C
	void* allocate_memory(size_t size) {
		void* ptr = malloc(size);  // 分配指定大小的内存
		if (ptr == NULL) {
				perror("Memory allocation failed");
				exit(1);
		}
		return ptr;
	}
	```

	- 内存读写操作 : stressapptest 会在分配的内存区域中进行大量的读取和写入，以下是一个简单的内存读写操作的例子：

	```C
	void stress_memory(void* ptr, size_t size) {
		volatile char* data = (volatile char*)ptr;
				for (size_t i = 0; i < size; i++) {
								data[i] = (char)(i % 256);  // 写入数据
								char temp = data[i];  // 读取数据
				}
	}
	```

	- 线程访问内存： stressapptest 使用多线程来增加内存压力。多个线程并发地对内存进行访问，模拟高负载下内存的使用情况。

- 命令分析：运行压测脚本后，运行如下命令： `stressapptest -s "$stime" -M "$memory_size" -f /tmp/sat.io1 -f /tmp/sat.io2 -i "$io_threads" -m 8 -C 2 -W`
	- `-s "$stime"`: 这个参数指定了 测试的持续时间。
	- `-M "$memory_size`": 这个参数指定了内存使用量，在源码中，-M 参数用于控制内存的分配大小。
	- `-f /tmp/sat.io1 -f /tmp/sat.io2`: -f 参数用于指定文件用于 I/O 测试。
	- `-i "$io_threads"`: 该参数指定了用于 I/O 操作的线程数。
	- `-m 8`: 这个参数控制 内存压力的强度。
	- `-C 2`: 这个参数指定了进行压力测试时使用的 CPU 核心数。
	- `-W`: 这个参数启用了 写操作，意味着不仅进行内存的读取，还会进行写入操作。



**2. BPU 压力测试**

- **测试目标：** 通过运行 `run.sh` 脚本，并利用 `bpu_os_test` 工具调整 -b （ BPU 核心 ）和 -r（ BPU 负载比例 ）参数，模拟不同的 BPU 负载场景，确保 BPU 在高负载下依然能够稳定工作并达到预期性能。
- **测试目的**：确保 BPU 在执行计算任务时，能够输出与预期相符的正确结果，检查 BPU 长时间高负载运行下仍能稳定工作，不出现崩溃或错误。

- **tc_hbdk3 BPU 压测原理：** 通过加载包含大量计算任务的模型，利用 BPU 进行加速计算，从而实现对 BPU 的压力测试。
- **命令分析：** 运行压测脚本后，运行如下命令：`tc_hbdk3 -t $1 -b $2 -f $HBM_FILE -i $SRC_FILE -n $MODEL_NAME -o $OUTPUT_0_0,$OUTPUT_0_1,$OUTPUT_1_0,$OUTPUT_1_1 -g 0 -c 0`
	- -t $1 ： 传递 $portion 参数的值，可以灵活地控制压力测试的强度，并且可以根据不同的测试需求来选择合适的负载水平。
	- -b $2 ： 选择使用的 BPU 核心数。
	- -f $HBM_FILE：-f 参数指定了 HBM 文件。
	- -i $SRC_FILE： 该文件包含模型推理或训练时使用的输入数据，程序会从中读取数据并传递给 BPU 进行计算。
	- -n $MODEL_NAME： -n 参数指定了要加载和运行的 模型名称
	- -o $OUTPUT_0_0 ： 这些输出文件用于保存计算结果或日志数据。

## 准备工作

**1.** 开始压力测试前，需要在芯片上添加散热片，否则芯片可能会进入过温保护影响测试结果。

**2.** 确认在 /app/multimedia_samples/chip_base_test/01_cpu_bpu_ddr 路径下存在的文件是否完整：

```shell
01_cpu_bpu_ddr/
`-- scripts
	|-- Readme.md
	|-- bpu_os_test
	|-- lib
	|   |-- libhbrt4.so
	|   `-- libhbtl.so
	|-- module
	|   |-- input_1.bin
	|   |-- input_2.bin
	|   `-- yolov3.hbm
	|-- run.sh
	|-- stop_test.sh
	|-- stress_test.sh
	`-- stressapptest
```

## 测试方法

压测脚本支持输入后缀 -h 查看命令参数的说明 :

```shell
./stress_test.sh -h

Usage: ./stress_test.sh [options]

Options:
	-t <time>        Set the test duration (e.g., 2h for hours, 30m for minutes; default: 48h).
	-m <size>        Set the memory size for stress test in MB (default: 100).
	-i <threads>     Set the I/O threads for stress test (default: 4).
	-b <bpu_core>    Specify the BPU core to use (default: 0).
	-r <portion>     Set the BPU portion value (default: 100).
	-o <directory>   Set the output directory for logs (default: ../../log).
	-h, --help       Show this help message and exit.

Example:
	./stress_test.sh -t 24h -m 200 -i 8 -b 80
```

各参数解析如下：

- `t <time>`: 用于设置压力测试的持续时间 , 时间格式可以是小时（如 2h）或分钟（如 30m），默认值为 48 小时。
- `m <size>`: 用于设置压力测试中内存的大小（单位： MB），默认值为 100MB。
- `i <threads>`: 用于设置 I/O 线程的数量，默认值为 4 。
- `b <bpu_core>`: 用于指定 BPU（基于处理单元）核心的编号，默认值为 0 。
- `r <portion>`: 用于设置 BPU 负载的比例，默认值为 100 （即满负载）。
- `o <directory>`: 用于设置日志输出的目录，默认值为 ../../log。
- `h, --help`: 显示帮助信息并退出脚本。

**示例：**
例如，使用命令 `./stress_test.sh -t 24h -m 200 -i 8 -b 80` 来运行一个 24 小时的压力测试，使用 200MB 内存， 8 个 I/O 线程， 80% 的 BPU 负载。

确保已完成准备工作后，运行测试命令：

```shell
cd /app/multimedia_samples/chip_base_test/01_cpu_bpu_ddr/scripts

./stress_test.sh
```

压测脚本通过 `hrut_somstatus` 命令来监控 **stressapptest** 的运行情况，运行脚本启动时的日志如下：

```shell
#hrut_somstatus
temperature-->
	pvt_cmn_pvtc1_t1 : 66.322 (C)
	pvt_cmn_pvtc1_t2 : 65.771 (C)
	pvt_mcu_pvtc1_t1 : 67.976 (C)
	pvt_mcu_pvtc1_t2 : 68.344 (C)
	pvt_bpu_pvtc1_t1 : 68.712 (C)
voltage-->
	FAKE     : 100.0 (mV)
	VDDQ_DDR2 : 503.0 (mV)
	VDD_MCU  : 748.0 (mV)
	VDDIO_PVT_MCU : 1802.0 (mV)
	VDDIO_TOP4_1V8 : 1802.0 (mV)
	VDDQ_DDR0 : 503.0 (mV)
	VDD2H_DDR0 : 1052.0 (mV)
	VAA_DDR0 : 1792.0 (mV)
	VDD_DDR0 : 742.0 (mV)
	VDDQ_DDR1 : 497.0 (mV)
	VDD2H_DDR1 : 1040.0 (mV)
	VAA_DDR1 : 1792.0 (mV)
	VDD_DDR1 : 742.0 (mV)
	CMN_PVTC1_V13 : 0.0 (mV)
	CMN_PVTC1_V14 : 0.0 (mV)
	CMN_PVTC1_V15 : 0.0 (mV)
	CMN_PVTC1_V16 : 0.0 (mV)
	VDDIO_SD_SDIO_T : 1788.0 (mV)
	VDD_CPU  : 776.0 (mV)
	CMN_PVTC1_V19 : 0.0 (mV)
	VAA_DDR2 : 1790.0 (mV)
	VDD2H_DDR2 : 1030.0 (mV)
	VDD_DDR2 : 737.0 (mV)
	VDDIO_TOP2_1V8 : 1780.0 (mV)
	VDD_BPU  : 731.0 (mV)
	VDDIO_TOP0_1V8 : 1790.0 (mV)
	PLL_TOP0_VDDHV : 1768.0 (mV)
	CMN_PVTC1_V27 : 0.0 (mV)
	CMN_PVTC1_V28 : 0.0 (mV)
	CMN_PVTC1_V29 : 0.0 (mV)
	CMN_PVTC1_V30 : 0.0 (mV)
	CMN_PVTC1_V31 : 0.0 (mV)
	CMN_PVTC1_V32 : 0.0 (mV)
	VDDIO_SD_33_MCU : 3308.0 (mV)
	VDDIO_ADC_MCU : 1802.0 (mV)
	VDDIO_MCU_1V8 : 1802.0 (mV)
	PLL_MCU_VDDHV : 1802.0 (mV)
	PLL_MCU_VDDPOST : 748.0 (mV)
	PLL_MCU_VDDREF : 748.0 (mV)
	VDD_TOP  : 753.0 (mV)
	VDD_AON  : 753.0 (mV)
	VDDIO_SD_AON_CA : 1644.0 (mV)
	VDDIO_PVT_1V8 : 1792.0 (mV)
	MCU_PVTC1_V11 : 0.0 (mV)
	MCU_PVTC1_V12 : 0.0 (mV)
	MCU_PVTC1_V13 : 0.0 (mV)
	MCU_PVTC1_V14 : 0.0 (mV)
	MCU_PVTC1_V15 : 0.0 (mV)
	MCU_PVTC1_V16 : 0.0 (mV)
cpu frequency-->
						min   cur     max
	policy0: 1125000        1500000 1500000
	policy4: 1125000        1500000 1500000
bpu status information---->
								ratio
	bpu0:   99
```

`hrut_somstatus` 命令结果解析如下：

- `temperature` ：代表当前 板载、 MCU、 BPU 的温度。
- `cpu frequency` ：分别代表 CPU 的最小运行频率、当前运行频率和最大运行频率（ MHZ）。
- `bpu status information` ：代表 BPU 的最小、当前和最大运行频率，其中 ratio 代表 BPU 当前占用率。

执行 `htop` 命令查看 `CPU` 的占用率

	![Htop](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/images_to_upload/Htop.png)

其中每一行显示了一个 CPU 核心的状态，格式为：

```shell
CpuX  [ 进度条 ]
```

- `CpuX`：0~5：表示 CPU 核心编号（本设备为 6 核心）。
- `[########...]`：为每个核心的实时负载进度条。
- `#`：表示用户态（user）占比。
- `*`：表示内核态（system）占比。
- `Load average: 7.10 9.56 9.57`：表示系统在 1 分钟、5 分钟、15 分钟内的平均负载。

## 测试指标
测试程序启动后，会在 `/app/multimedia_samples/chip_base_test/log` 目录下产生 `bpu-stressX.log` 和 `cpu-stressX.log` 两份日志文件用来记录压测时的状态，确保能够在压测中保持如下内容：

- 能稳定运行 48 小时，不出现重启或挂死的情况。
- 使用以下命令检查日志文件中是否存在 `fail`、`error`、`timeout` 等异常打印。

```shell
cd "/app/multimedia_samples/chip_base_test/log/" && grep -iE 'error|fail|timeout' bpu-stress*.log cpu-stress*.log
```

- 执行 `TOP` 命令查看 CPU、 BPU 占比情况，正常状态应稳定保持在 98-100% 区间。

### 测试结果

输入命令：

```shell
cd "/app/multimedia_samples/chip_base_test/log/" && grep -iE 'error|fail|timeout' bpu-stress*.log cpu-stress*.log
```

显示如下：

```shell
cpu-stress1.log:2025/05/19-22:01:32(CST) Stats: Completed: 206320.00M in 9.46s 21819.44MB/s, with 0 hardware incidents, 0 errors
cpu-stress1.log:2025/05/19-22:01:32(CST) Status: PASS - please verify no corrected errors
cpu-stress2.log:2025/05/19-22:20:22(CST) Stats: Completed: 220626.00M in 9.00s 24511.95MB/s, with 0 hardware incidents, 0 errors
cpu-stress2.log:2025/05/19-22:20:22(CST) Status: PASS - please verify no corrected errors
cpu-stress3.log:2025/05/19-22:21:20(CST) Stats: Completed: 196480.00M in 9.00s 21829.11MB/s, with 0 hardware incidents, 0 errors
```

打印信息中并未出现其他异常信息，且 errors 数量为 0 ，代表压测结果正常。
