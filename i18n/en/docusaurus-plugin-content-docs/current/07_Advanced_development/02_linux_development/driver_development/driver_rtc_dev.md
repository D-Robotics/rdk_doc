---
sidebar_position: 13
---

# RTC Debugging Guide

Note: Using pcf8563 chip as an example

## Driver Code

```
drivers/rtc/rtc-pcf8563.c
```

## Kernel Configuration

CONFIG_RTC_DRV_PCF8563

![image-20220323100439451](./image/driver_develop_guide/image-20220323100439451.png)

## Using the RTC

After the driver is successfully loaded, the /dev/rtc1 device node will appear, which corresponds to pcf-8563. You can use the following commands for testing.

```bash
# Create a soft link from /dev/rtc1 to /dev/rtc

ln -sf /dev/rtc1 /dev/rtc

# Test commands
date -s 2022.01.21-21:24:00   	# Set system time
hwclock -w       				# Write system time to RTC
hwclock -r       				# Read RTC time to confirm if it was successfully written
hwclock -s       				# If the RTC battery is present, after power loss and power on again, update system time to RTC time
date             				# Read system time
```

:::info Note
If you need to ensure that the RTC can still record time after power loss, you need to provide separate power supply to the RTC. The power supply interface is RTC Battery Con.
:::