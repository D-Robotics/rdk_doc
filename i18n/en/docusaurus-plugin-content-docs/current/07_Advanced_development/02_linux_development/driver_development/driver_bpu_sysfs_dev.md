---
sidebar_position: 9
---
# BPU Driver sysfs debugging interface

## BPU sysfs node description

```bash
/sys/devices/system/bpu
```

![image](./image/driver_develop_guide/15b0a3742a6721d475db1a6e21b1809e.png)

The information of each file node can be obtained using the "cat" command or set using the "echo" command. The specific contents are as follows:

-   bpu\*: directory, bpu0 and bpu1 correspond to the two cores of the bpu respectively, and the nodes in each directory are set as follows:

![image](./image/driver_develop_guide/550eb60c5eb1a66ca126be088c08d9b5.png)

- burst_len: read/write file, burst_len corresponding to the core

- hotplug: read/write file, whether hotplugging is enabled for the core:

  -   0: disable;
  -   1: enable;

- power_enable: read/write file, power switch of the core, used to turn on/off the corresponding BPU core hardware power;

- devfreq: read/write file, used to read/set the frequency of the core:

  - Taking BPU0 as an example, set the frequency scaling policy to "userspace"

    ```
    echo userspace > /sys/devices/system/bpu/bpu0/devfreq/devfreq*/governor
    ```

  - View the supported frequencies of BPU:

    ```
    cat /sys/devices/system/bpu/bpu0/devfreq/devfreq*/available_frequencies
    ```

  - Set BPU to the target frequency, the target frequency must be supported by BPU:

    ```
    echo 200000000 > /sys/devices/system/bpu/bpu0/devfreq/devfreq*/userspace/set_freq
    ```

  - Confirm the set frequency:

    ```
    cat /sys/devices/system/bpu/bpu0/devfreq/devfreq*/available_frequencies
    ```
- limit: Read/write file used to set the number of buffers related to the Core hardware. The default value is 0, any value greater than 0 is the actual number of buffers. It is related to priority, where smaller positive values have higher priority and the task is scheduled earlier, but the efficiency of task switching would be reduced. Users should set this based on the actual situation.

- power_level: Read/write file used to set the power level at which the Core operates (including power supply and frequency):

  - 1: Linux dvf dynamic adjustment
  - 0: Performance priority, highest power consumption
  - < 0: Within range, smaller values correspond to lower power consumption levels

- users: Read-only file used to obtain user information using the Core. Refer to the users section below for detailed information.

- queue: Read-only file used to obtain the number of FunctionCall that the driver can currently set.

![](./image/driver_develop_guide/4edf614de291d4c7005d01be0cc10041.png)

- ratio: Read-only file used to obtain utilization information of the Core.

![](./image/driver_develop_guide/2724e46795d0798ea433eb990be22149.png)

- fc_time: Obtain information about fc tasks processed on the Core. Each task has the following sub-items:

![](./image/driver_develop_guide/6cad811f2afb4fa4984ad3c44ae0f5aa.png)

- index: The position of the task in the BPU hardware FIFO
- id: User-defined interrupt id
- hwid: Interrupt id maintained by the underlying driver
- group: User-defined group id, user process id
- prio: Task priority
- s_time: Timestamp of the start of task processing
- e_time: Timestamp of the end of task processing
- r_time: Total execution time of the task
- core_num: Read-only file, the number of cores in the BPU hardware
- group: Read-only file, information about task groups running on the BPU. Can be obtained by running "cat group":

![](./image/driver_develop_guide/f90196d698265775d78717a9ab9967ce.png)

- group: User-defined group id and process id;
    - prop: User-defined proportion value
    - ratio: Current actual utilization ratio
- ratio: Read-only file, current utilization of the BPU
- users: Read-only file, current users of the BPU. Users are classified as those who set tasks through the BPU framework and those who specify tasks for each Core. Can be obtained by running "cat users":

![](./image/driver_develop_guide/b898a1588bd4a94e332a5dbd9bd8f46a.png)
-   user: User process ID
-   ratio: The corresponding occupancy rate of the user on the Core

## Usage Example

All cases in the following text target BPU0. All commands can be executed after running the model application.

### Disable BPU Core

Execute the following command:

```bash
echo 0 > /sys/devices/system/bpu/bpu0/power_enable
```

### Hot-Swap BPU Core

Hot-Swapping does not affect the running of single-core model applications and is not supported for dual-core model applications. Once Hot-Swapping is enabled, it will not be disabled by default. If you need to disable Hot-Swapping, please set it manually (echo 0 to the corresponding sysfs node). Execute the following commands in order:

```bash
echo 1 > /sys/devices/system/bpu/bpu0/hotplug
echo 0 > /sys/devices/system/bpu/bpu0/power_enable
```

### Reduce BPU Core Power Consumption

This command does not shut down the BPU core, but only lowers the corresponding core frequency/power consumption. For specific values and corresponding meanings, please refer to the above text:

```bash
echo -2 > /sys/devices/system/bpu/bpu0/power_level
```

### Priority Model Usage

Compile and use priority models according to the relevant instructions of HBDK compiler. Set tasks using the hb_bpu_core_set_fc_prio interface or the hb_bpu_core_set_fc_group interface with a specific group_id:

```bash
echo 2 > /sys/devices/system/bpu/bpu0/limit
```

The limit interface can be used for debugging. Users can also use the following command to set environment variables when executing applications:

```bash
export BPLAT_CORELIMIT=2
```