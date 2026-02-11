---
sidebar_position: 4
---
# 2.4 Thermal Control and CPU Frequency Management

```mdx-code-block
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';
```



:::info Note
The following information does not apply to the `RDK Ultra` development board.
:::

## X3 Thermal Control

:::info Note
The following information applies to the `RDK X3` and `RDK X3 Module` development boards.
:::

To avoid chip overheating under heavy load, power management is implemented at the operating system level. The SoC has an internal temperature sensor, and the Thermal subsystem monitors this temperature.

### Main Temperature Points

- **Boot Temperature:** This is the maximum temperature during system startup. If the temperature exceeds this threshold, the system will immediately throttle the CPU and BPU frequencies during boot. The current configuration value can be obtained using the command `cat /sys/devices/virtual/thermal/thermal_zone0/trip_point_0_temp`, with a default value of 80000 (80 degrees Celsius/176 degrees Fahrenheit).
- **Throttling Temperature:** This is the temperature at which the CPU and BPU frequencies are throttled. When the temperature exceeds this threshold, the CPU and BPU frequencies are reduced to decrease the SoC power consumption. The CPU frequency can be throttled down to a minimum of 240 MHz, and the BPU frequency can be throttled down to a minimum of 400 MHz. The current configuration value can be obtained using the command `cat /sys/devices/virtual/thermal/thermal_zone0/trip_point_1_temp`, with a default value of 95000 (95 degrees Celsius/203 degrees Fahrenheit).
- **Shutdown Temperature:** This is the temperature at which the system shuts down to protect the chip and hardware. It is recommended to ensure proper cooling of the device to prevent shutdowns. After a shutdown, the device does not automatically restart and requires a manual power cycle of the development board to restart. The current configuration value can be obtained using the command `cat /sys/devices/virtual/thermal/thermal_zone0/trip_point_2_temp`, with a default value of 105000 (105 degrees Celsius/221 degrees Fahrenheit).

The current chip operating frequency, temperature, and other status can be viewed using the `sudo hrut_somstatus` command:

![image-20220714113732289](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/02_System_configuration/image/cpu_frequency/image-20220714113732289.png)

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

### CPU Frequency Management  

The Linux kernel comes with a built-in cpufreq subsystem to control CPU frequency and frequency control policies.  

Navigate to the directory `/sys/devices/system/cpu/cpufreq/policy0` and run `ls`. You will see the following files in the directory:

```shell
affected_cpus						// CPU cores affected by the current policy (does not show offline CPUs)
cpuinfo_cur_freq					// Current CPU frequency (unit: KHz)
cpuinfo_max_freq					// Maximum available CPU frequency under the current policy (unit: KHz)
cpuinfo_min_freq					// Minimum available CPU frequency under the current policy (unit: KHz)
cpuinfo_transition_latency			// Time required for the processor to switch frequencies (unit: ns)
related_cpus						// List of CPU cores affected by this policy (includes all CPUs, online and offline)
scaling_available_frequencies		// List of supported CPU hardware frequencies (unit: KHz)
scaling_available_governors			// All available governor (frequency scaling) types in the current kernel
scaling_boost_frequencies			// List of supported CPU hardware frequencies in boost (overclocking) mode (unit: KHz)
scaling_cur_freq					// Stores the current CPU frequency cached by the cpufreq module; does not check CPU hardware registers.
scaling_disable_freq				// Disabled CPU frequency; only one frequency can be set.
scaling_driver						// The currently used frequency scaling driver
scaling_governor					// The current governor (frequency scaling) policy
scaling_max_freq					// Maximum available CPU frequency under the current policy (read from cpufreq module cache)
scaling_min_freq					// Minimum available CPU frequency under the current policy (read from cpufreq module cache)
scaling_setspeed					// Used only when the governor is set to 'userspace'. Writing a value to this file changes the frequency.
```

The Linux kernel used in the RDK system supports the following types of frequency scaling policies:

- Performance: Always keeps the CPU at the highest performance (and highest power consumption) state, i.e., the maximum frequency supported by the hardware.

- Powersave: Always keeps the CPU at the lowest power consumption (and lowest performance) state, i.e., the minimum frequency supported by the hardware.

- Ondemand: Periodically checks the system load and adjusts the frequency accordingly. When the load is low, it scales down to the minimum frequency sufficient for the current load. When the load is high, it immediately scales up to the maximum performance state.

- Conservative: Similar to the ondemand policy, it periodically checks the load and adjusts the frequency. For low load, it scales down to the minimum sufficient frequency. For high load, instead of immediately jumping to the maximum frequency, it increases the frequency step by step.

- Userspace: Exposes the control interface to users via sysfs, allowing custom policies and manual frequency adjustment in user space.

- Schedutil: Introduced starting from Linux kernel version 4.7. This policy utilizes CPU utilization information provided by the scheduler to adjust the frequency. It is similar in effect to the ondemand policy but is more precise and natural because the scheduler has the best knowledge of CPU usage.

Users can set the frequency scaling policy in the following two ways:

<Tabs groupId="tool-type">
<TabItem value="login" label="Login to the RDK">

Users can control the CPU frequency scaling policy by modifying the corresponding settings in the directory `/sys/devices/system/cpu/cpufreq/policy0`.

For example, to set the CPU to run in performance mode:  

```shell
sudo bash -c "echo performance > /sys/devices/system/cpu/cpufreq/policy0/scaling_governor"
```

Or to set the CPU to run at a fixed frequency (1 GHz):  

```shell
sudo bash -c "echo userspace > /sys/devices/system/cpu/cpufreq/policy0/scaling_governor"
sudo bash -c "echo 1000000 > /sys/devices/system/cpu/cpufreq/policy0/scaling_setspeed"
```

#### CPU Overclocking

:::info NOTE
The following content applies to the `RDK X3` and `RDK X3 Module` development boards, but not to the `RDK Ultra` board.
:::

<!-- Video: https://www.bilibili.com/video/BV1rm4y1E73q/?p=14 -->

The development board uses the CPU Freq driver to manage the CPU operating state. The default mode is `ondemand`, where the CPU frequency dynamically adjusts based on the load to save power. Users can switch to `performance` mode to make the CPU always run at the maximum frequency using the following command:

```bash
sudo bash -c 'echo performance > /sys/devices/system/cpu/cpufreq/policy0/scaling_governor'
```

The development board provides an overclocking feature in the system, which can increase the maximum CPU frequency from 1.2 GHz to 1.5 GHz. Use the following command to configure it:  

```bash
sudo bash -c 'echo 1 > /sys/devices/system/cpu/cpufreq/boost'
```

The CPU frequency configured using the above command only takes effect during the current session. It will revert to the default configuration after a device reboot.

:::info NOTE
CPU overclocking increases chip power consumption and heat generation. If stability issues occur, you can disable the overclocking feature using the following command:  

```bash
sudo bash -c 'echo 0 > /sys/devices/system/cpu/cpufreq/boost'
```

:::

You can use the `sudo hrut_somstatus` command to view the current chip operating frequency, temperature, and other status information:  

![image-20220714113732289](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/02_System_configuration/image/cpu_frequency/image-20220714113732289.png)  

</TabItem>

<TabItem value="rdk-studio" label="Log in with RDK Studio">

The Performance Node tool in RDK Studio provides system hardware monitoring and real-time performance control capabilities. It combines detailed data monitoring with dynamic performance tuning to help users fully understand and manage their computer's operational status.

:::info Note

- RDK Studio download link: [Click to download](https://developer.d-robotics.cc/en/rdkstudio)
- RDK Studio user guide: [Click to view](../../01_Quick_start/09_RDK_Studio/01_rdk_studio.md)

:::

1. Use RDK Studio to add devices. Refer to [Add RDK Device](../01_Quick_start/09_RDK_Studio/05_Device_management/01_hardware_resource.md).
2. Click the application space icon to view more applications.

    ![Application Space Interface](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/en/rdk_studio_left_menu_device_manage_hr_add_device_application_space_download.png)

3. Click to install Performance Node onto the development board.

    ![Download Interface](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/02_System_configuration/image/cpu_frequency/rdk_studio/en/performance-node-install.png)

4. Performance Node monitoring display:
   
    - **CPU Performance Interface:**

        ![CPU Performance Interface](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/02_System_configuration/image/cpu_frequency/rdk_studio/performance-node-CPU.png)

        - Click on the CPU label above to enable/disable the corresponding CPU performance display.
        - Click the button in the upper right corner for area zoom and restore.
        - Click the download button to save the current performance interface as an image.

    - **BPU/GPU Performance Interface:**

        ![BPU/GPU Performance Interface](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/02_System_configuration/image/cpu_frequency/rdk_studio/performance-node-BPU-GPUpng.png)
    
        - Click on the BPU/GPU label above to enable/disable the corresponding BPU/GPU performance display.
        - Click the button in the upper right corner for area zoom and restore.
        - Click the download button to save the current performance interface as an image.
    
    - **Memory Performance Interface: Displays used and available memory.**

        ![Memory Performance Interface](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/02_System_configuration/image/cpu_frequency/rdk_studio/performance-node-memory.png)
    
          - Click the button in the upper right corner for area zoom and restore.
          - Click the download button to save the current performance interface as an image.

    - **Disk Performance Interface: Displays used and available disk space.**

        ![Disk Performance Interface](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/02_System_configuration/image/cpu_frequency/rdk_studio/performance-node-disk.png)
    
        - Click the button in the upper right corner for area zoom and restore.
        - Click the download button to save the current performance interface as an image.

5. Performance Monitoring Settings:
   
   ![Settings Interface](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/02_System_configuration/image/cpu_frequency/rdk_studio/performance-node-operation.png)

   - Set Sampling Interval: The default sampling interval is 1000ms. After entering the interval time, click `Current sampling interval 1000ms, click to modify` to complete the interval time setting.
   - CPU Frequency Scaling Mode: Click `performance (CPU frequency scaling mode, click to switch)` to switch the CPU frequency scaling mode.
   - Interface Switching: The default is vertical screen interface. Click `Click to go to horizontal screen interface` to switch to horizontal display.
  
      ![Horizontal Screen Interface](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/02_System_configuration/image/cpu_frequency/rdk_studio/performance-node-row.png)
  
</TabItem>
</Tabs>

## X5 Thermal Control
:::info Note
The following information applies to the `RDK X5` development boards.
:::

### temperature sensor
There are three temperature sensors on X5, which are used to display the temperature of DDR/BPU/CPU. In/sys/class/hwmon/, there is a hwmon0 directory containing the relevant parameters of temperature sensors. temp1_input is the temperature of DDR, temp2input is the temperature of BPU, and temp3_input is the temperature of CPU. The accuracy of temperature is 0.001 degrees Celsius

```bash
cat /sys/class/hwmon/hwmon0/temp1_input
46643
```
> The temperature sensor of BPU is located in the BPU subsystem, and the BPU subsystem is only powered on when the BPU is running, so the temperature of the BPU can only be viewed when it is running.

### Thermal
Linux Thermal is a temperature control module in the Linux system, mainly used to control the heat generated by the chip during system operation, so as to maintain the temperature of the chip and the device casing within a safe and comfortable range.

To achieve reasonable control of equipment temperature, we need to understand the following three modules:

Device for obtaining temperature: abstracted as Thermal Zone Device in the Thermal framework, X5 has two thermal zones, namely thermalzone0 and thermalzone1;

Devices that require cooling: abstracted as Thermal Cooling Devices in the Thermal framework, including CPU, BPU, GPU, and DDR;

Temperature control strategy: abstracted as Thermal Governor in the Thermal framework;

The information and controls of the above modules can be obtained in the/sys/class/male directory.

There are a total of four cooling devices in X5:

- cooling_device0: cpu

- cooling_device1: bpu

- cooling_device2: gpu

- cooling_device3: ddr

Among them, the cooling device DDR is associated with thermalzone0, and the cooling device CPU/BPU/GPU is associated with thermalzone1. The current default strategy is known to be using stepw_ise through the following command.

```bash
cat /sys/class/thermal/thermal_zone0/policy
```
The supported strategies can be seen through the following command: user_Space, stepw_ise, a total of two types.

```bash
cat /sys/class/thermal/thermal_zone0/available_policies
```
User_Space reports the current temperature of the temperature zone, temperature control trigger points, and other information to the user space through uevent, and the user space software formulates the temperature control strategy.

Stepw_ise is a relatively mild temperature control strategy that gradually increases the cooling state during each polling cycle

The specific strategy to choose is based on the needs of the product. It can be specified during compilation or dynamically switched through sysfs. For example, the strategy for dynamically switching thermalzone0 to user_stpace mode

```bash
echo user_space > /sys/class/thermal/thermal_zone0/policy
```
There is one trip_point in thermalzone0, which is used to control the frequency modulation temperature of the cooling device DDR

The frequency modulation temperature of DDR can be viewed through sysfs, and the current configuration is 95 degrees

```bash
cat /sys/devices/virtual/thermal/thermal_zone0/trip_point_0_temp
```
If you want to adjust the frequency modulation temperature of DDR, such as 85 degrees, you can use the following command:

```bash
echo 85000 > /sys/devices/virtual/thermal/thermal_zone0/trip_point_0_temp
```
There are three triplepoints in thermalzone1, where triple_point_0_temp is reserved; Trip_point_1_temp is the frequency modulation temperature of this thermal zone, which can control the frequency of CPU/BPU/GPU. It is currently set to 95 degrees. Trip_point_2_temp is the shutdown temperature, currently set to 105 degrees. For example, if you want the junction temperature to reach 85 degrees Celsius, the CPU/BPU/GPU will start frequency modulation:

```bash
echo 85000 > /sys/devices/virtual/thermal/thermal_zone1/trip_point_1_temp
```
If you want to adjust the shutdown temperature to 105 degrees Celsius:

```bash
echo 105000 > /sys/devices/virtual/thermal/thermal_zone1/trip_point_2_temp
```
> Please note that the above configuration will be reset to the default values after a system restart. To ensure persistent configuration, these commands can be added to the startup scripts for automatic configuration.

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

Currently supported frequencies include:  
```shell
cat /sys/devices/system/cpu/cpufreq/policy0/scaling_available_frequencies
300000 600000 1200000 1500000
```  


The Linux kernel used by the RDK system supports the following types of scaling strategies:

- Performance: It always keeps the CPU in the highest power consumption and highest performance state, which is the maximum frequency supported by the hardware.
- Powersave: It always keeps the CPU in the lowest power consumption and lowest performance state, which is the minimum frequency supported by the hardware.
- Ondemand: It periodically checks the load and adjusts the frequency accordingly. When the load is low, it adjusts to the minimum frequency that can meet the current load requirements. When the load is high, it immediately boosts to the highest performance state.
- Conservative: It is similar to the ondemand strategy. It periodically checks the load and adjusts the frequency accordingly. When the load is low, it adjusts to the minimum frequency that can meet the current load requirements. However, when the load is high, it gradually increases the frequency instead of immediately setting it to the highest performance state.
- Userspace: It exposes the control interface through sysfs to allow users to customize their own strategies. Users can manually adjust the frequency in the user space.
- Schedutil: This is a strategy introduced in Linux-4.7 that adjusts the frequency based on the CPU utilization information provided by the scheduler. It has similar effects to the ondemand strategy but is more accurate and natural (as the scheduler has the best understanding of CPU usage).

Users can set the frequency scaling policy in the following two ways:

<Tabs groupId="tool-type">
<TabItem value="login" label="Login to the RDK">

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

The development board uses the CPU Freq driver to manage the CPU operating state. The default mode is the `schedutil` mode, where the CPU frequency is dynamically adjusted based on the load to save power. Users can switch to the `performance` mode to make the CPU always run at the highest frequency using the following command:  

```bash
echo performance >/sys/devices/system/cpu/cpu0/cpufreq/scaling_governor
```

The development board provides an overclocking feature in the system, which can increase the maximum CPU frequency from 1.5GHz to 1.8GHz. The configuration commands are as follows:  

```bash
echo 1 >/sys/devices/system/cpu/cpufreq/boost
echo performance >/sys/devices/system/cpu/cpu0/cpufreq/scaling_governor
```

The CPU frequency configured using the above commands only takes effect during the current session. If the device is rebooted, the configuration will revert to the default settings.  

:::info Note
CPU overclocking increases chip power consumption and heat. If stability issues occur, the overclocking feature can be disabled using the following command:  

```bash
echo 0 >/sys/devices/system/cpu/cpufreq/boost
```

:::

:::info Note
Overclocking is only supported on X5H; it is not available on X5M. The chip type can be checked using the following command. For example, the following query shows the chip is X5M:  

```bash
cat /sys/class/socinfo/soc_name
X5M
```

If the chip type is identified as `X5U` (i.e., `X5 UNKNOWN`), it usually indicates that the chip is an early production version, and the corresponding identifier has not been burned into its eFUSE. Therefore, the accurate chip model cannot be read through software and requires manual identification based on the chip's silk print information.  

![image-20251029-114153](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/02_System_configuration/image/cpu_frequency/image-20251029-114153.png)

For such chips, overclocking restrictions are not enabled, meaning overclocking configuration can theoretically be executed. However, please note:

- Power consumption will increase significantly after overclocking.

- System stability cannot be guaranteed; there may be risks of abnormal operation or reduced lifespan.

It is recommended to use this feature only during the research and development phase for testing and not to rely on overclocking for mass production.  

:::

You can use the `sudo hrut_somstatus` command to view the current chip operating frequency, temperature, and other status information:  

![image-20240829171934000](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/02_System_configuration/image/cpu_frequency/image-20240829171934000.png)

</TabItem>

<TabItem value="rdk-studio" label="Log in with RDK Studio">

The Performance Node tool in RDK Studio provides system hardware monitoring and real-time performance control capabilities. It combines detailed data monitoring with dynamic performance tuning to help users fully understand and manage their computer's operational status.

:::info Note

- RDK Studio download link: [Click to download](https://developer.d-robotics.cc/en/rdkstudio)
- RDK Studio user guide: [Click to view](../../01_Quick_start/09_RDK_Studio/01_rdk_studio.md)

:::

1. Use RDK Studio to add devices. Refer to [Add RDK Device](../01_Quick_start/09_RDK_Studio/05_Device_management/01_hardware_resource.md).
2. Click the application space icon to view more applications.

    ![Application Space Interface](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/en/rdk_studio_left_menu_device_manage_hr_add_device_application_space_download.png)

3. Click to install Performance Node onto the development board.

    ![Download Interface](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/02_System_configuration/image/cpu_frequency/rdk_studio/en/performance-node-install.png)

4. Performance Node monitoring display:
   
    - **CPU Performance Interface:**

        ![CPU Performance Interface](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/02_System_configuration/image/cpu_frequency/rdk_studio/performance-node-CPU.png)

        - Click on the CPU label above to enable/disable the corresponding CPU performance display.
        - Click the button in the upper right corner for area zoom and restore.
        - Click the download button to save the current performance interface as an image.

    - **BPU/GPU Performance Interface:**

        ![BPU/GPU Performance Interface](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/02_System_configuration/image/cpu_frequency/rdk_studio/performance-node-BPU-GPUpng.png)
    
        - Click on the BPU/GPU label above to enable/disable the corresponding BPU/GPU performance display.
        - Click the button in the upper right corner for area zoom and restore.
        - Click the download button to save the current performance interface as an image.
    
    - **Memory Performance Interface: Displays used and available memory.**

        ![Memory Performance Interface](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/02_System_configuration/image/cpu_frequency/rdk_studio/performance-node-memory.png)
    
          - Click the button in the upper right corner for area zoom and restore.
          - Click the download button to save the current performance interface as an image.

    - **Disk Performance Interface: Displays used and available disk space.**

        ![Disk Performance Interface](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/02_System_configuration/image/cpu_frequency/rdk_studio/performance-node-disk.png)
    
        - Click the button in the upper right corner for area zoom and restore.
        - Click the download button to save the current performance interface as an image.

5. Performance Monitoring Settings:
   
   ![Settings Interface](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/02_System_configuration/image/cpu_frequency/rdk_studio/performance-node-operation.png)

   - Set Sampling Interval: The default sampling interval is 1000ms. After entering the interval time, click `Current sampling interval 1000ms, click to modify` to complete the interval time setting.
   - CPU Frequency Scaling Mode: Click `performance (CPU frequency scaling mode, click to switch)` to switch the CPU frequency scaling mode.
   - Interface Switching: The default is vertical screen interface. Click `Click to go to horizontal screen interface` to switch to horizontal display.
  
      ![Horizontal Screen Interface](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/02_System_configuration/image/cpu_frequency/rdk_studio/performance-node-row.png)
  
</TabItem>
</Tabs>