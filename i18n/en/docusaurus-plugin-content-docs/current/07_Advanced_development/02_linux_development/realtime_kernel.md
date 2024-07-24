---
sidebar_position: 5
---
# 7.2.5 Applying Real-Time Kernel

This chapter aims to describe how to enable real-time kernel (Preempt-RT kernel) on `RDK X3` and `RDK X3 Module`, and provide the corresponding commands for users to use. Real-time kernel is an operating system kernel that can provide more precise and reliable response time, and is commonly used in applications with high time sensitivity requirements, such as robot control and industrial automation. The following are the related commands for enabling and disabling real-time kernel, as well as some common use cases and testing methods.

## Command to Enable Real-Time Kernel

To enable real-time kernel on RDK X3, please execute the following commands:

```bash
sudo apt update
sudo apt install hobot-kernel-headers=2.0.0-01~rt hobot-boot=2.0.2-01~rt hobot-bpu-drivers=2.0.0-01~rt
sudo reboot
```

These commands will install kernel headers, kernel files, drivers, and BPU drivers that are compatible with real-time kernel. After completion, restart the system to make the changes effective. You can use the `uname -a` command to see the kernel version information with `PREEMPT RT` indication.

![image-20230914142401210](image/realtime_kernel/image-20230914142401210.png)

## Command to Restore to Standard Kernel

If you need to restore to the standard kernel, you can execute the following commands:

```bash
sudo apt install hobot-kernel-headers hobot-boot hobot-bpu-drivers
sudo reboot
```

These commands will uninstall the components related to the real-time kernel and install the components corresponding to the standard kernel. After completion, restart the system to switch back to the standard kernel.

## Real-Time Performance Testing

When you need to test the performance of the real-time kernel, a commonly used performance testing tool is `rt-tests`, which includes multiple test programs that can be used to test the performance of the real-time kernel. Here is an example of performing performance testing with `rt-tests`:

### Installing the rt-tests Tool

If the `rt-tests` tool has not been installed yet, you can use the following command to install it:

```
sudo apt install rt-tests
```

### Running the cyclictest Test

The `cyclictest` test is a commonly used test in `rt-tests`, which is used to evaluate the timing behavior and response time of the system. Execute the following command to run the `cyclictest` test:

```
sudo cyclictest -l50000000 -m -S -p90 -i200 -h400
```

This command will run a real-time performance test with the following options:

- `-l50000000`: Sets the number of iterations for the test to run.
- `-m`: Specifies that `cyclictest` will lock the current and upcoming memory through mlock during the test to prevent swap interference.
- `-S`: Specifies the standard SMP architecture test, where all threads will use the same -a -t -n and priorities.
- `-p90`: Specifies the priority of the real-time thread. `-p90` means the test will run in the real-time scheduling class with a priority of 90.
- `-i200`: Sets the sleep time for the test threads.
- `-h400`: After the test is completed, output a histogram and the count of times the latency was less than the specified value (400) in microseconds.

The test results will display the minimum, maximum, and average latency of the system, as well as some other performance statistics.

### Analyzing Test Results

Analyze the `cyclictest` test results to assess the performance of the real-time kernel. Pay attention to the minimum and maximum latencies to ensure they are within an acceptable range. Smaller maximum latency and more consistent latency indicate better performance of the real-time kernel. The following graphs show the results of the test when using a real-time kernel and running the `/app/pydev_demo/03_mipi_camera_sample` example.

![image-20230914145619064](image/realtime_kernel/image-20230914145619064.png)

![image-20230914145234528](image/realtime_kernel/image-20230914145234528.png)

The meaning of each field in the `cyclictest` output is as follows:

- `T`: Thread identifier and thread number for the test run.
- `P`: Priority used during the test run. In your output, it has a value of 90, indicating the test ran in the real-time scheduling class with a priority of 90.
- `I`: Expected wake-up period for the latency measurement thread in microseconds.
- `C`: Number of measurements taken for the latency, i.e., the loop count.
- `Min`: Minimum latency for the current test cycle in microseconds.
- `Act`: Actual latency for the current test cycle in microseconds.
- `Avg`: Average latency for the current test cycle in microseconds.
- `Max`: Maximum latency for the current test cycle in microseconds.