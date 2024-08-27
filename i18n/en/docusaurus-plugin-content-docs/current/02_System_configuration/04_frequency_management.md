---
sidebar_position: 4
---
# 2.4 Thermal Control and CPU Frequency Management

:::info Note
The following information applies to the `RDK X3` and `RDK X3 Module` development boards and does not apply to the `RDK Ultra` development board.
:::

## Thermal Control

To avoid chip overheating under heavy load, power management is implemented at the operating system level. The SoC has an internal temperature sensor, and the Thermal subsystem monitors this temperature.

### Main Temperature Points

- **Boot Temperature:** This is the maximum temperature during system startup. If the temperature exceeds this threshold, the system will immediately throttle the CPU and BPU frequencies during boot. The current configuration value can be obtained using the command `cat /sys/devices/virtual/thermal/thermal_zone0/trip_point_0_temp`, with a default value of 80000 (80 degrees Celsius).
- **Throttling Temperature:** This is the temperature at which the CPU and BPU frequencies are throttled. When the temperature exceeds this threshold, the CPU and BPU frequencies are reduced to decrease the SoC power consumption. The CPU frequency can be throttled down to a minimum of 240 MHz, and the BPU frequency can be throttled down to a minimum of 400 MHz. The current configuration value can be obtained using the command `cat /sys/devices/virtual/thermal/thermal_zone0/trip_point_1_temp`, with a default value of 95000 (95 degrees Celsius).
- **Shutdown Temperature:** This is the temperature at which the system shuts down to protect the chip and hardware. It is recommended to ensure proper cooling of the device to prevent shutdowns. After a shutdown, the device does not automatically restart and requires a manual power cycle of the development board to restart. The current configuration value can be obtained using the command `cat /sys/devices/virtual/thermal/thermal_zone0/trip_point_2_temp`, with a default value of 105000 (105 degrees Celsius).

The current chip operating frequency, temperature, and other status can be viewed using the `sudo hrut_somstatus` command:

![image-20220714113732289](../../../../../static/img/02_System_configuration/image/cpu_frequency/image-20220714113732289.png)

### Configuring Temperature Thresholds

The throttling and shutdown temperature thresholds can be temporarily configured using the following commands. The throttling temperature cannot exceed the shutdown temperature, and the shutdown temperature cannot be set above 105 degrees Celsius.

For example, to set the throttling temperature to 85 degrees Celsius:

```text
echo 85000 > /sys/devices/virtual/thermal/thermal_zone0/trip_point_1_temp
```

For example, to set the shutdown temperature to 105 degrees Celsius:

```text
echo 105000 > /sys/devices/virtual/thermal/thermal_zone0/trip_point_2_temp
```

Please note that the above configuration will be reset to the default values after a system restart. To ensure persistent configuration, these commands can be added to the startup scripts for automatic configuration.

## CPU Frequency Management

The Linux kernel includes the cpufreq subsystem for controlling CPU frequencies and frequency control policies.

Navigate to the `/sys/devices/system/cpu/cpufreq/policy0` directory and execute the `ls` command. You will see the following files:

```shell
affected_cpus    // CPUs currently affected by the frequency control (offline CPUs are not displayed)
cpuinfo_cur_freq                    // Current CPU frequency (unit: KHz)
cpuinfo_max_freq                    // The highest frequency available for the CPU under the current scaling strategy (unit: KHz)
cpuinfo_min_freq                    // The lowest frequency available for the CPU under the current scaling strategy (unit: KHz)
cpuinfo_transition_latency          // The time required to switch the processor frequency (unit: ns)
related_cpus                        // The CPU cores affected by this control strategy (including all online+offline CPUs)
scaling_available_frequencies       // List of CPU supported frequencies (unit: KHz)
scaling_available_governors         // List of all available governors (scaling) types in the current kernel
scaling_boost_frequencies           // List of supported frequencies for CPU in boost mode (unit: KHz)
scaling_cur_freq                    // The currently cached CPU frequency in the cpufreq module, without checking the CPU hardware registers.
scaling_disable_freq                // The CPU frequency that is disabled and can only be set to one value
scaling_driver                      // The currently used scaling driver
scaling_governor                    // The current governor (scaling) strategy
scaling_max_freq                    // The highest frequency available for the CPU under the current scaling strategy (read from the cpufreq module cache)
scaling_min_freq                    // The lowest frequency available for the CPU under the current scaling strategy (read from the cpufreq module cache)
scaling_setspeed                    // A file that should be used to switch the governor to 'userspace' before use. Echo a value to this file to switch the frequency.
```

The Linux kernel used by the RDK system supports the following types of scaling strategies:

- Performance: It always keeps the CPU in the highest power consumption and highest performance state, which is the maximum frequency supported by the hardware.
- Powersave: It always keeps the CPU in the lowest power consumption and lowest performance state, which is the minimum frequency supported by the hardware.
- Ondemand: It periodically checks the load and adjusts the frequency accordingly. When the load is low, it adjusts to the minimum frequency that can meet the current load requirements. When the load is high, it immediately boosts to the highest performance state.
- Conservative: It is similar to the ondemand strategy. It periodically checks the load and adjusts the frequency accordingly. When the load is low, it adjusts to the minimum frequency that can meet the current load requirements. However, when the load is high, it gradually increases the frequency instead of immediately setting it to the highest performance state.
- Userspace: It exposes the control interface through sysfs to allow users to customize their own strategies. Users can manually adjust the frequency in the user space.
- Schedutil: This is a strategy introduced in Linux-4.7 that adjusts the frequency based on the CPU utilization information provided by the scheduler. It has similar effects to the ondemand strategy but is more accurate and natural (as the scheduler has the best understanding of CPU usage).

Users can control the CPU scaling strategy by modifying the corresponding settings under the directory `/sys/devices/system/cpu/cpufreq/policy0`.

For example, to set the CPU to performance mode:

```shell
sudo bash -c "echo performance > /sys/devices/system/cpu/cpufreq/policy0/scaling_governor"
```

Or to set the CPU to a fixed frequency (1GHz):

```shell
sudo bash -c "echo userspace > /sys/devices/system/cpu/cpufreq/policy0/scaling_governor"
sudo bash -c "echo 1000000 > /sys/devices/system/cpu/cpufreq/policy0/scaling_setspeed"
```

### CPU Overclocking

:::info Note
The following content is applicable to `RDK X3` and `RDK X3 Module` development boards and not applicable to the `RDK Ultra` development board.
:::

<iframe width="560" height="315" src="https://www.youtube.com/embed/WqLxbN2qw-k?si=UM_e7W97TfRGDGDp" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

The development board uses the CPU Freq driver to manage the CPU operating state. The default mode is the 'ondemand' mode, where the CPU frequency is dynamically adjusted based on the load to save power. User can change to the 'performance' mode to make the CPU always operate at the highest frequency. The command is as follows:

```bash
sudo bash -c 'echo performance > /sys/devices/system/cpu/cpufreq/policy0/scaling_governor'
```

The development board provides overclocking function in the system, which can increase the CPU maximum frequency from 1.2GHz to 1.5GHz. The configuration command is as follows:

```bash
sudo bash -c 'echo 1 > /sys/devices/system/cpu/cpufreq/boost'
```

The CPU frequency configured by the above command only takes effect during current operation. If the device is restarted, it will return to the default configuration.

:::info Note
Overclocking the CPU will increase the power consumption and heat dissipation of the chip. If stability issues occur, you can disable the overclocking function with the following command:

```bash
sudo bash -c 'echo 0 > /sys/devices/system/cpu/cpufreq/boost'
```

:::

You can use the `sudo hrut_somstatus` command to check the current chip operating frequency, temperature, and other status:

![image-20220714113732289](../../../../../static/img/02_System_configuration/image/cpu_frequency/image-20220714113732289.png)