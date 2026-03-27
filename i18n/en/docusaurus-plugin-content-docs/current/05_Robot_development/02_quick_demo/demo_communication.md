---
sidebar_position: 5
---

# 5.2.5 Data Communication

```mdx-code-block
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';
```

## Zero-Copy

### Feature Introduction

TogetheROS.Bot provides a flexible and efficient zero-copy feature that can significantly reduce communication latency and CPU usage for large-sized data. By integrating the performance_test tool, tros.b enables convenient evaluation of performance differences before and after enabling zero-copy. The performance_test tool supports configuration of parameters such as the number of subscribers, message size, QoS settings, etc., facilitating performance assessment under various scenarios. Key performance metrics include:

- **Latency**: Transmission time of a message from publisher to subscriber.
- **CPU Usage**: Percentage of CPU utilization caused by communication activities.
- **Resident Memory**: Includes heap-allocated memory, shared memory, and stack memory used internally by the system.
- **Sample Statistics**: Includes the number of messages sent, received, and lost in each experiment.

Code repositories:
  - [https://github.com/D-Robotics/rclcpp](https://github.com/D-Robotics/rclcpp)
  - [https://github.com/D-Robotics/rcl_interfaces](https://github.com/D-Robotics/rcl_interfaces)
  - [https://github.com/D-Robotics/benchmark](https://github.com/D-Robotics/benchmark)

:::info
- The tros.b Foxy version introduces the "zero-copy" feature based on ROS2 Foxy.
- The tros.b Humble version utilizes the native "zero-copy" feature provided by ROS2 Humble.
:::

### Supported Platforms

| Platform                          | Operating System                     |
| --------------------------------- | ------------------------------------ |
| RDK X3, RDK X3 Module             | Ubuntu 20.04 (Foxy), Ubuntu 22.04 (Humble) |
| RDK X5, RDK X5 Module, RDK S100   | Ubuntu 22.04 (Humble)                |

:::caution
***The RDK Ultra platform supports zero-copy data communication, but the zero-copy performance benchmarking package is not yet available.***
:::

### Preparation

#### RDK

1. Before testing, switch the RDK to performance mode to ensure accurate test results. Use the following command:

   ```bash
   echo performance > /sys/devices/system/cpu/cpufreq/policy0/scaling_governor 
   ```

   For more configuration details, refer to the [System Configuration](/System_configuration/) section.

2. Ensure the performance_test tool package is installed on the RDK. Installation commands:

   <Tabs groupId="tros-distro">
   <TabItem value="foxy" label="Foxy">

   ```bash
   sudo apt update
   sudo apt install tros-performance-test
   ```

   </TabItem>
   <TabItem value="humble" label="Humble">

   ```bash
   sudo apt update
   sudo apt install tros-humble-performance-test
   ```

   </TabItem>
   </Tabs>

:::caution **Note**
**If the `sudo apt update` command fails or returns an error, refer to the FAQ section [Common Issues](/docs/08_FAQ/01_hardware_and_system.md), specifically `Q10: How to handle failures or errors when running apt update?` for solutions.**
:::

### Usage Guide

#### RDK Platform

1. Run a 4MB data transmission test without zero-copy enabled, using the following command:

 <Tabs groupId="tros-distro">
 <TabItem value="foxy" label="Foxy">

    ```bash
    source /opt/tros/setup.bash
    ros2 run performance_test perf_test --reliable --keep-last --history-depth 10 -s 1 -m Array4m -r 100 --max-runtime 30
    ```

 </TabItem>
 <TabItem value="humble" label="Humble">

    ```bash
    source /opt/tros/humble/setup.bash
    ros2 run performance_test perf_test --reliable --keep-last --history-depth 10 -s 1 -m Array4m -r 100 --max-runtime 30
    ```

 </TabItem>
 </Tabs>

    **Test results are as follows**:

    ```dotnetcli
    run time

    +--------------+-----------+--------+----------+
    | T_experiment | 30.982817 | T_loop | 1.000126 |
    +--------------+-----------+--------+----------+

    samples                                              latency

    +------+------+------+-----------+---------------+   +----------+----------+----------+----------+
    | recv | sent | lost | data_recv | relative_loss |   | min      | max      | mean     | variance |
    +------+------+------+-----------+---------------+   +----------+----------+----------+----------+
    | 99   | 100  | 0    | 418505326 | 0.000000      |   | 0.004327 | 0.005605 | 0.004546 | 0.000000 |
    +------+------+------+-----------+---------------+   +----------+----------+----------+----------+

    publisher loop                                       subscriber loop

    +----------+----------+----------+----------+        +----------+----------+----------+----------+
    | min      | max      | mean     | variance |        | min      | max      | mean     | variance |
    +----------+----------+----------+----------+        +----------+----------+----------+----------+
    | 0.007260 | 0.008229 | 0.008057 | 0.000000 |        | 0.000000 | 0.000000 | 0.000000 | 0.000000 |
    +----------+----------+----------+----------+        +----------+----------+----------+----------+

    system usage

    +-------------+-----------+---------+--------+--------+----------+--------+--------+
    | utime       | stime     | maxrss  | ixrss  | idrss  | isrss    | minflt | majflt |
    +-------------+-----------+---------+--------+--------+----------+--------+--------+
    | 23120954000 | 121597000 | 65092   | 0      | 0      | 0        | 11578  | 2      |
    +-------------+-----------+---------+--------+--------+----------+--------+--------+
    | nswap       | inblock   | oublock | msgsnd | msgrcv | nsignals | nvcsw  | nivcsw |
    +-------------+-----------+---------+--------+--------+----------+--------+--------+
    | 0           | 0         | 0       | 0      | 0      | 0        | 9885   | 7193   |
    +-------------+-----------+---------+--------+--------+----------+--------+--------+

    Maximum runtime reached. Exiting.
    ```

2. Run a 4MB data transmission test with zero-copy enabled (using the `--zero-copy` argument), using the following command:

 <Tabs groupId="tros-distro">
 <TabItem value="foxy" label="Foxy">

    ```bash
    source /opt/tros/setup.bash
    ros2 run performance_test perf_test --zero-copy --reliable --keep-last --history-depth 10 -s 1 -m Array4m -r 100 --max-runtime 30
    ```

 </TabItem>
 <TabItem value="humble" label="Humble">

    ```bash
    source /opt/tros/humble/setup.bash
    export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    export FASTRTPS_DEFAULT_PROFILES_FILE=/opt/tros/humble/lib/hobot_shm/config/shm_fastdds.xml
    export RMW_FASTRTPS_USE_QOS_FROM_XML=1
    export ROS_DISABLE_LOANED_MESSAGES=0
    ros2 run performance_test perf_test --zero-copy --reliable --keep-last --history-depth 10 -s 1 -m Array4m -r 100 --max-runtime 30
    ```

 </TabItem>
 </Tabs>

    **Test results are as follows**:

    ```dotnetcli
    run time

    +--------------+-----------+--------+----------+
    | T_experiment | 30.554773 | T_loop | 1.000084 |
    +--------------+-----------+--------+----------+

    samples                                              latency

    +------+------+------+-----------+---------------+   +----------+----------+----------+----------+
    | recv | sent | lost | data_recv | relative_loss |   | min      | max      | mean     | variance |
    +------+------+------+-----------+---------------+   +----------+----------+----------+----------+
    | 99   | 99   | 0    | 418701472 | 0.000000      |   | 0.000146 | 0.000381 | 0.000195 | 0.000000 |
    +------+------+------+-----------+---------------+   +----------+----------+----------+----------+

    publisher loop                                       subscriber loop

    +----------+----------+----------+----------+        +----------+----------+----------+----------+
    | min      | max      | mean     | variance |        | min      | max      | mean     | variance |
    +----------+----------+----------+----------+        +----------+----------+----------+----------+
    | 0.009812 | 0.009895 | 0.009877 | 0.000000 |        | 0.000000 | 0.000000 | 0.000000 | 0.000000 |
    +----------+----------+----------+----------+        +----------+----------+----------+----------+

    system usage

    +------------+-----------+---------+--------+--------+----------+--------+--------+
    | utime      | stime     | maxrss  | ixrss  | idrss  | isrss    | minflt | majflt |
    +------------+-----------+---------+--------+--------+----------+--------+--------+
| 8727113000 | 307920000 | 46224   | 0      | 0      | 0        | 6440   | 0      |
    +------------+-----------+---------+--------+--------+----------+--------+--------+
    | nswap      | inblock   | oublock | msgsnd | msgrcv | nsignals | nvcsw  | nivcsw |
    +------------+-----------+---------+--------+--------+----------+--------+--------+
    | 0          | 0         | 0       | 0      | 0      | 0        | 9734   | 2544   |
    +------------+-----------+---------+--------+--------+----------+--------+--------+

    Maximum runtime reached. Exiting.
    ```

### Result Analysis

The `performance_test` tool can output various types of statistical results. Below, we primarily compare differences in latency and system resource usage:

**Latency**  
The average communication latencies with "zero-copy" disabled and enabled are 4.546 ms and 0.195 ms, respectively. This clearly shows that the "zero-copy" feature significantly reduces communication latency.

**System Usage**

```dotnetcli
  +------------------+---------------+-------------------+--------+--------+----------+------------------+---------------------+
  | utime            | stime         | maxrss            | ixrss  | idrss  | isrss    | minflt           | majflt              |
  +------------------+---------------+-------------------+--------+--------+----------+------------------+---------------------+
  | Userspace time (Hz) | System time (Hz) | Resident memory size (Bytes) | 0      | 0      | 0        | Minor page faults    | Major page faults       |
  +------------------+---------------+-------------------+--------+--------+----------+------------------+---------------------+
  | nswap            | inblock       | oublock           | msgsnd | msgrcv | nsignals | nvcsw            | nivcsw              |
  +------------------+---------------+-------------------+--------+--------+----------+------------------+---------------------+
  | 0                | 0             | 0                 | 0      | 0      | 0        | Voluntary context switches | Involuntary context switches |
  +------------------+---------------+-------------------+--------+--------+----------+------------------+---------------------+
```

| Communication Method | Latency  | utime+stime | maxrss | minflt | majflt | nvcsw | nivcsw |
| ---------------------| ---------| ------------|--------|--------|--------|-------|--------|
| Non-"zero-copy"      | 0.004546 | 23242551000 | 65092  | 11578  |   2    | 9885  |  7193  |
| "Zero-copy"          | 0.000381 | 9035033000  | 46224  | 6440   |   0    | 9734  |  2544  |

Comparison reveals the following:

- The sum of `utime` and `stime` for "zero-copy" is significantly lower than that of non-"zero-copy", indicating that "zero-copy" consumes less CPU resources.
- The `maxrss` value for "zero-copy" is lower than that of non-"zero-copy", indicating that "zero-copy" uses less memory.
- Both `minflt` and `majflt` counts for "zero-copy" are significantly lower than those of non-"zero-copy", suggesting that "zero-copy" exhibits less communication jitter.
- Both `nvcsw` and `nivcsw` counts for "zero-copy" are significantly lower than those of non-"zero-copy", further indicating reduced communication jitter with "zero-copy".

In summary, for large-data communication, "zero-copy" demonstrates clear advantages over non-"zero-copy" in terms of CPU consumption, memory usage, and communication latency jitter.