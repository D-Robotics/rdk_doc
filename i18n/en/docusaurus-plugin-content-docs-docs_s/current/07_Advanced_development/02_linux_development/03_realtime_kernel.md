---
sidebar_position: 3
---

# 7.2.3 Applying the Real-Time Kernel

The `RDKS100` enables the real-time kernel by default. A real-time kernel is an operating system kernel capable of providing more precise and reliable response times, commonly used in applications with high time-sensitivity requirements, such as robotic control and industrial automation. Below are commands for enabling and disabling the real-time kernel, along with common use cases and testing methods.

## Real-Time Performance Testing

When testing the performance of a real-time kernel, a commonly used tool is `rt-tests`, which includes multiple test programs for evaluating real-time kernel performance. The following is an example of using `rt-tests` for performance testing:

### Installing the rt-tests Tool

If `rt-tests` is not yet installed, use the following commands to install it:

```
sudo apt update
sudo apt install rt-tests
```

### Running the cyclictest

`cyclictest` is a commonly used test within `rt-tests` for evaluating a system's timing behavior and response latency. Execute the following command to run the `cyclictest`:

```
sudo cyclictest -l50000000 -m -S -p90 -i200 -h400
```

This command runs a real-time performance test with the following parameters:

- `-l50000000`: Sets the number of test iterations.
- `-m`: Instructs `cyclictest` to lock current and subsequent memory pages using `mlock` during the test, preventing swapping from affecting results.
- `-S`: Specifies a standard SMP architecture test where all threads use the same `-a`, `-t`, `-n`, and priority settings.
- `-p90`: Specifies the real-time thread priority. `-p90` means the test runs under real-time scheduling class with priority 90.
- `-i200`: Sets the sleep interval (in microseconds) for test threads.
- `-h400`: After completion, outputs a histogram showing the count of latencies below the specified threshold (400 microseconds).

The test results will display the system’s minimum, maximum, and average latency, along with other performance statistics.

### Analyzing Test Results

Analyze the `cyclictest` results to evaluate the real-time kernel's performance. Pay close attention to the minimum and maximum latencies to ensure they fall within acceptable ranges. A lower maximum latency and more consistent latency values indicate better real-time kernel performance. The figures below show test results obtained while using the real-time kernel and running the `/app/pydev_demo/03_mipi_camera_sample` example.

![image-20230914145619064](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/02_linux_development/image/realtime_kernel/image-20230914145619064.png)

![image-20230914145234528](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/02_linux_development/image/realtime_kernel/image-20230914145234528.png)

The fields in the `cyclictest` output have the following meanings:

- `T`: Thread number and thread ID used in the test.
- `P`: Priority used during the test. In your output, the value is 90, indicating the test ran under real-time scheduling class with priority 90.
- `I`: Expected wake-up interval (in microseconds) for the latency measurement thread.
- `C`: Number of latency measurements taken (i.e., loop count).
- `Min`: Minimum latency (in microseconds) observed in the current test cycle.
- `Act`: Actual latency (in microseconds) observed in the current test cycle.
- `Avg`: Average latency (in microseconds) observed in the current test cycle.
- `Max`: Maximum latency (in microseconds) observed in the current test cycle.