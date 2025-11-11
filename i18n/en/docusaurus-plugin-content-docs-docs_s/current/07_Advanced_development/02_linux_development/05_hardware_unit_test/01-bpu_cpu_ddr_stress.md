---
sidebar_position: 1
---

# CPU-BPU-DDR Stress Test

## Test Principles

The CPU-BPU-DDR stress test primarily evaluates the performance and stability of the CPU, BPU, and DDR under high-load conditions. Below are the fundamental principles of stress testing for these components:

**1. CPU-DDR Stress Test**

- **Test Objective**: Use the `stressapptest` tool to simulate a high-load environment involving intensive computation, data processing, and memory operations, thereby testing system performance under multi-threaded concurrent workloads.
- **Test Purpose**: Verify CPU stability and performance under prolonged high loads, ensuring that both the CPU and DDR can operate normally under heavy workloads without crashing, overheating, or suffering performance degradation.

- **stressapptest CPU Stress Test Principle**: CPU stress testing is primarily achieved by concurrently executing compute-intensive tasks across multiple threads. These tasks include compute-heavy operations and parallel multi-threaded execution.
	- Thread Creation: `stressapptest` creates multiple threads via `pthread_create()`, with each thread performing computational tasks. The number of threads is typically specified by the user via the `-C` parameter.

	```C
	for (int i = 0; i < num_threads; i++) {
		pthread_create(&threads[i], NULL, cpu_stress_function, (void*)i);
	}
	```

	- Compute-Intensive Tasks: Each thread executes compute-intensive operations such as floating-point calculations and memory reads/writes, thereby consuming CPU resources.

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

	- Thread Synchronization and Management: Functions like `pthread_join()` ensure proper thread execution and synchronization.

- **stressapptest DDR Stress Test Principle**: DDR memory stress testing primarily involves extensive memory allocation, access, and data exchange through high memory usage, frequent read/write operations, and memory bandwidth consumption.
	- Memory Allocation: During memory stress testing, `stressapptest` allocates a specified amount of memory according to the `-M` parameter (e.g., `-M 8192`).

	```C
	void* allocate_memory(size_t size) {
		void* ptr = malloc(size);  // Allocate memory of specified size
		if (ptr == NULL) {
				perror("Memory allocation failed");
				exit(1);
		}
		return ptr;
	}
	```

	- Memory Read/Write Operations: `stressapptest` performs extensive read and write operations on the allocated memory region. Below is a simple example:

	```C
	void stress_memory(void* ptr, size_t size) {
		volatile char* data = (volatile char*)ptr;
				for (size_t i = 0; i < size; i++) {
								data[i] = (char)(i % 256);  // Write data
								char temp = data[i];  // Read data
				}
	}
	```

	- Multi-threaded Memory Access: `stressapptest` uses multiple threads to increase memory pressure. Multiple threads concurrently access memory, simulating high-load memory usage scenarios.

- Command Analysis: After running the stress test script, execute the following command: `stressapptest -s "$stime" -M "$memory_size" -f /tmp/sat.io1 -f /tmp/sat.io2 -i "$io_threads" -m 8 -C 2 -W`
	- `-s "$stime"`: Specifies the test duration.
	- `-M "$memory_size"`: Specifies the amount of memory to use. In the source code, the `-M` parameter controls the size of allocated memory.
	- `-f /tmp/sat.io1 -f /tmp/sat.io2`: The `-f` parameter specifies files for I/O testing.
	- `-i "$io_threads"`: Specifies the number of threads for I/O operations.
	- `-m 8`: Controls the intensity of memory stress.
	- `-C 2`: Specifies the number of CPU cores used for stress testing.
	- `-W`: Enables write operations, meaning both memory reads and writes are performed.

**2. BPU Stress Test**

- **Test Objective**: Run the `run.sh` script and use the `bpu_os_test` tool to adjust the `-b` (BPU cores) and `-r` (BPU load ratio) parameters, simulating various BPU load scenarios to ensure the BPU remains stable and delivers expected performance under high loads.
- **Test Purpose**: Ensure the BPU produces correct results consistent with expectations during computational tasks, and verify that it remains stable under prolonged high-load operation without crashing or producing errors.

- **tc_hbdk3 BPU Stress Test Principle**: Load models containing extensive computational tasks and leverage the BPU for accelerated computation, thereby conducting BPU stress testing.
- **Command Analysis**: After running the stress test script, execute the following command: `tc_hbdk3 -t $1 -b $2 -f $HBM_FILE -i $SRC_FILE -n $MODEL_NAME -o $OUTPUT_0_0,$OUTPUT_0_1,$OUTPUT_1_0,$OUTPUT_1_1 -g 0 -c 0`
	- `-t $1`: Passes the value of the `$portion` parameter, allowing flexible control over stress test intensity and enabling selection of appropriate load levels based on different test requirements.
	- `-b $2`: Specifies the number of BPU cores to use.
	- `-f $HBM_FILE`: The `-f` parameter specifies the HBM file.
	- `-i $SRC_FILE`: This file contains input data used for model inference or training; the program reads data from it and passes it to the BPU for computation.
	- `-n $MODEL_NAME`: The `-n` parameter specifies the model name to load and run.
	- `-o $OUTPUT_0_0`: These output files store computation results or log data.

## Preparation

**1.** Before starting the stress test, attach a heatsink to the chip; otherwise, the chip may trigger thermal protection, affecting test results.

**2.** Confirm that all files under the path `/app/chip_base_test/01_cpu_bpu_ddr` are present and complete:
:::warning
The dynamic libraries under the `/lib/` directory and the model/input files under the `module/` directory listed below are **exclusively intended for BPU stress testing** and **must not be used for any other purpose**.
:::
```shell
01_cpu_bpu_ddr/
└── scripts
    ├── Readme.md
    ├── bpu_os_test
    ├── lib
    │   ├── libhbrt4.so
    │   └── libhbtl.so
    ├── module
    │   ├── input_1.bin
    │   ├── input_2.bin
    │   └── yolov3.hbm
    ├── run.sh
    ├── stop_test.sh
    ├── stress_test.sh
    └── stressapptest

3 directories, 11 files
```

## Test Procedure

The stress test script supports the `-h` suffix to display command parameter descriptions:

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

Parameter explanations:

- `t <time>`: Sets the stress test duration. Time format can be hours (e.g., 2h) or minutes (e.g., 30m), with a default value of 48 hours.
- `m <size>`: Sets the memory size (in MB) for the stress test, with a default value of 100 MB.
- `i <threads>`: Sets the number of I/O threads, with a default value of 4.
- `b <bpu_core>`: Specifies the BPU (Brain Processing Unit) core ID, with a default value of 0.
- `r <portion>`: Sets the BPU load ratio, with a default value of 100 (i.e., full load).
- `o <directory>`: Sets the log output directory, with a default value of `../../log`.
- `h, --help`: Displays help information and exits the script.

**Example:**
For instance, run the command `sudo ./stress_test.sh -t 24h -m 200 -i 8 -b 80` to perform a 24-hour stress test using 200 MB of memory, 8 I/O threads, and 80% BPU load.

After completing the preparation steps, execute the test command:

```shell
cd /app/chip_base_test/01_cpu_bpu_ddr/scripts

sudo ./stress_test.sh
```

The stress test script monitors the **stressapptest** process using the `hrut_somstatus` command. Sample log output upon script startup is as follows:

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
```VDD_BPU  : 731.0 (mV)
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

The output of the `hrut_somstatus` command is explained as follows:

- `temperature`: Represents the current temperatures of the board, MCU, and BPU.
- `cpu frequency`: Represents the CPU's minimum, current, and maximum operating frequencies (in MHz).
- `bpu status information`: Represents the BPU's minimum, current, and maximum operating frequencies, where `ratio` indicates the current BPU utilization.

Execute the `htop` command to check CPU utilization:

	![Htop](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/images_to_upload/Htop.png)

Each line displays the status of a CPU core in the following format:

```shell
CpuX  [ progress bar ]
```

- `CpuX` (0~5): Indicates the CPU core number (this device has 6 cores).
- `[########...]`: Real-time load progress bar for each core.
- `#`: Represents the user-space (user) utilization percentage.
- `*`: Represents the kernel-space (system) utilization percentage.
- `Load average: 7.10 9.56 9.57`: Represents the system’s average load over the past 1, 5, and 15 minutes.

## Test Metrics

After the test program starts, it generates two log files—`bpu-stressX.log` and `cpu-stressX.log`—in the `/app/chip_base_test/log` directory to record system status during stress testing. Ensure the following conditions are met during the stress test:

- Stable operation for 48 hours without rebooting or hanging.
- Use the following command to check whether the log files contain any abnormal messages such as `fail`, `error`, or `timeout`:

```shell
cd "/app/chip_base_test/log/" && grep -iE 'error|fail|timeout' bpu-stress*.log cpu-stress*.log
```

- Run the `TOP` command to monitor CPU and BPU utilization. Under normal conditions, utilization should remain steadily within the 98–100% range.

### Test Results

Enter the command:

```shell
cd "/app/chip_base_test/log/" && grep -iE 'error|fail|timeout' bpu-stress*.log cpu-stress*.log
```

The output is as follows:

```shell
cpu-stress1.log:2025/05/19-22:01:32(CST) Stats: Completed: 206320.00M in 9.46s 21819.44MB/s, with 0 hardware incidents, 0 errors
cpu-stress1.log:2025/05/19-22:01:32(CST) Status: PASS - please verify no corrected errors
cpu-stress2.log:2025/05/19-22:20:22(CST) Stats: Completed: 220626.00M in 9.00s 24511.95MB/s, with 0 hardware incidents, 0 errors
cpu-stress2.log:2025/05/19-22:20:22(CST) Status: PASS - please verify no corrected errors
cpu-stress3.log:2025/05/19-22:21:20(CST) Stats: Completed: 196480.00M in 9.00s 21829.11MB/s, with 0 hardware incidents, 0 errors
```

No abnormal messages appear in the output, and the number of errors is 0, indicating that the stress test results are normal.