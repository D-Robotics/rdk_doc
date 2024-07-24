---
sidebar_position: 12
---

# DDR Stress Testing Plan

## Purpose

This article mainly introduces the testing method for long-term high-pressure operation of DDR particles in high and low temperature environments.

## Glossary

Definition of terms:
PASS: After the test is completed, all functions and performance of the DUT meet expectations, and no anomalies occur.
FAIL: After the test is completed, the DUT's functions are damaged or malfunctioning, or do not meet the specified requirements.

## Testing Categories

| **Testing Category** | **Environmental Temperature** | **Testing Duration** |
| -------------------- | ---------------------------- | -------------------- |
| DDR Normal Temperature Stress Test | 25 degrees | 48 hours |
| DDR High Temperature Stress Test | 60 degrees | 48 hours |
| DDR Low Temperature Stress Test | -25 degrees | 48 hours |

## Testing Environment

### DUT Stress Program

| **ITEM** | **Version Number/Device Number** | **Applicable Testing Categories** |
| -------- | ------------------------------- | --------------------------------- |
| Test Script | xj3_ddr_stress.tar.gz | General Stress Testing |
| Number of DUTs | 5 or more | General Stress Testing |

Download `xj3_ddr_stress_gcc9.3.tar.gz` from the corresponding SDK version directory of [unittest](http://sunrise.horizon.cc/downloads/unittest/)

### How to Use the Stress Program

1. Transfer `xj3_ddr_stress_gcc9.3.tar.gz` to the userdata directory of XJ3 via Ethernet or other tools.

2. Enter the userdata directory on the XJ3 side: cd /userdata

3. Uncompress the test file

```
tar -zxvf xj3_ddr_stress_gcc9.3.tar.gz
```

4. Grant permission to the xj3_ddr_stress folder

```
chmod 777 xj3_ddr_stress
```

5. Enter the directory "cd xj3_ddr_stress"

```
cd xj3_ddr_stress 
```

6. Set CPU mode and frequency reduction temperature

```
# If the device restarts, these two commands need to be configured again
echo performance > /sys/devices/system/cpu/cpufreq/policy0/scaling_governor 
echo 105000 > /sys/devices/virtual/thermal/thermal_zone0/trip_point_1_temp
```

7. Execute the stress test script, run cpu test and bpu test in the background

```
sh ./scripts/xj3-stress.sh
```

8. Check the running status with top

![image-20220324192755274](./image/driver_develop_guide/image-20220324192755274.png)

9. The cpu test log is stored in /userdata/cpu-stress.log, check the current running status

```
# It will be printed every 10 seconds
tail /userdata/cpu-stress.log
```

![image-20220324192849234](./image/driver_develop_guide/image-20220324192849234.png)

Note: The program runs continuously for 48 hours by default (as shown in the figure below, the value after the -s parameter is 172800, in seconds)

10. The bpu test log is stored in /userdata/bpu-stress.log, check the current running status

```
# As long as the log is being updated, it is still testing, it runs continuously by default
tail /userdata/bpu-stress.log
```

11. Check the cpu test result, if the last Status in /userdata/cpu-stress.log shows PASS and there is no "error" or "miscompare" keyword in the log, it indicates a PASS result.

![image-20220324193228651](./image/driver_develop_guide/image-20220324193228651.png)

12. If the last Status in /userdata/cpu-stress.log shows FAIL or if there is an "error" or "miscompare" keyword in the log, it indicates a FAIL result.13. Check the bpu test result. If keywords such as "error" or "system hung" are present in the bpu_stress.log, it means the test has failed.

![image-20220324193250187](./image/driver_develop_guide/image-20220324193250187.png)

13、check `bpu test result`, if  `error`, `system hung` ,etc, appeared in `bpu_stress.log`, it means the test has failed.