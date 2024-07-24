---
sidebar_position: 5
---
# 5.2.5 Communication

## Zero-copy

```mdx-code-block
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';
```

### Introduction

TogetheROS.Bot provides a flexible and efficient zero-copy function that can significantly reduce the communication latency and CPU usage of large-sized data. By integrating the performance_test tool, tros.b can easily evaluate the performance difference before and after enabling zero-copy. The performance_test tool can configure parameters such as sub count, message size, and QoS to facilitate the evaluation of communication performance in different scenarios. The main performance indicators are as follows:

- Latency: The time it takes for a message to be transmitted from pub to sub.
- CPU usage: The percentage of CPU usage by communication activities.
- Resident memory: Includes heap allocated memory, shared memory, and stack memory used for system internals.
- Sample statistics: Includes the number of messages sent, received, and lost.

Code repositories: [https://github.com/HorizonRDK/rclcpp](https://github.com/HorizonRDK/rclcpp), [https://github.com/HorizonRDK/rcl_interfaces](https://github.com/HorizonRDK/rcl_interfaces)

### Supported Platforms

| Platform    | System      | Function                       |
| ------- | ------------ | ------------------------------ |
| RDK X3, RDK X3 Module | Ubuntu 20.04 (Foxy), Ubuntu 22.04 (Humble) | Show zero-copy performance test results |

### Preparation

#### Horizon RDK

1. Before starting the test, adjust the Horizon RDK to performance mode to ensure the accuracy of the test results. Use the following commands:

   ```bash
   echo performance > /sys/class/devfreq/devfreq0/governor
   echo performance > /sys/devices/system/cpu/cpufreq/policy0/scaling_governor 
   ```

2. The performance_test package is already installed in Horizon RDK. Install it using the command:

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

### Usage Guide

#### Horizon RDK Platform

1. Test the transmission of 4M data without enabling zero-copy. Use the following command:

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

    **Test results are shown below**:

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

2. Start the 4M data transfer test with zero-copy (use --zero-copy parameter), the command is as follows:

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

    **Test results are shown below**:
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

The performance_test tool can output various types of statistical results. We mainly compare the differences in latency and system usage:

**latency**

The mean latency for communication with "zero-copy" enabled and disabled are 4.546ms and 0.195ms, respectively. It can be seen that "zero-copy" significantly reduces communication latency.

**system usage**

| Communication Method     | Latency     | utime+stime | maxrss | minflt | majflt | nvcsw | nivcsw |
| ------------------------ | ----------- | ----------- | ------ | ------ | ------ | ------| ------ |
| Non-"zero-copy"           | 0.004546    | 23242551000 | 65092  | 11578  | 2      | 9885  | 7193   |
| "zero-copy"               | 0.000381    | 9035033000  | 46224  | 6440   | 0      | 9734  | 2544   |

Comparing the two methods, we can see that:

- The sum of utime and stime in the "zero-copy" method is significantly lower than in the non-"zero-copy" method, indicating that "zero-copy" consumes less CPU resources.
- The maxrss in the "zero-copy" method is lower than in the non-"zero-copy" method, indicating that "zero-copy" occupies less memory.
- The minflt and majflt in the "zero-copy" method are significantly lower than in the non-"zero-copy" method, indicating that "zero-copy" communication has less jitter.
- The nvcsw and nivcsw in the "zero-copy" method are significantly lower than in the non-"zero-copy" method, indicating that "zero-copy" communication has less jitter.

Overall, for large data communication, the "zero-copy" method is clearly superior to the non-"zero-copy" method in terms of CPU consumption, memory usage, and communication latency jitter.