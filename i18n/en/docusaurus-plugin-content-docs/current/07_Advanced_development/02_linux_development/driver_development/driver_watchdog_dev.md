---
sidebar_position: 14
---
# Watchdog Driver Debugging Guide

## Code Path

```
drivers/watchdog/hobot_wdt.c # watchdog driver source file
include/linux/watchdog.h # watchdog driver header file
```

## DTS Configuration

```
/* arch/arm64/boot/dts/hobot/hobot-xj3.dtsi */
watchdog: watchdog@0xA1002000 {
    compatible = "hobot,hobot-wdt";
    reg = <0 0xA1002000 0 0x1000>;
    clocks = <&timer0_mclk>;
    clock-names = "watchdog_mclk";
    interrupt-parent = <&gic>;
    interrupts = <0 15 4>;
    pet-time = <6>;
    bark-time = <11>;
    bite-time = <15>;
    status = "disabled";
};

/* arch/arm64/boot/dts/hobot/hobot-x3-sdb.dts */
&watchdog {
	status = "okay";
};

/* arch/arm64/boot/dts/hobot/hobot-xj3-xvb.dtsi */
&watchdog {
	status = "okay";
};

```

## Kernel Configuration

```
/* arch/arm64/configs/xj3_debug_defconfig */
CONFIG_WATCHDOG=y
CONFIG_WATCHDOG_CORE=y
# CONFIG_WATCHDOG_NOWAYOUT is not set
CONFIG_WATCHDOG_HANDLE_BOOT_ENABLED=y
# CONFIG_WATCHDOG_SYSFS is not set
#
# Watchdog Device Drivers
#
# CONFIG_SOFT_WATCHDOG is not set
# CONFIG_GPIO_WATCHDOG is not set
# CONFIG_XILINX_WATCHDOG is not set
# CONFIG_ZIIRAVE_WATCHDOG is not set
# CONFIG_ARM_SP805_WATCHDOG is not set
# CONFIG_ARM_SBSA_WATCHDOG is not set
# CONFIG_CADENCE_WATCHDOG is not set
# CONFIG_DW_WATCHDOG is not set
# CONFIG_MAX63XX_WATCHDOG is not set
CONFIG_HOBOT_WATCHDOG=y
# CONFIG_HOBOT_WATCHDOG_ENABLE is not set /*Enable this option for the system to automatically feed the dog*/
CONFIG_HOBOT_WATCHDOG_TEST=y
# CONFIG_MEN_A21_WDT is not set

/* arch/arm64/configs/xj3_debug_defconfig */
CONFIG_WATCHDOG=y
CONFIG_WATCHDOG_CORE=y
# CONFIG_WATCHDOG_NOWAYOUT is not set
CONFIG_WATCHDOG_HANDLE_BOOT_ENABLED=y
# CONFIG_WATCHDOG_SYSFS is not set

#
# Watchdog Device Drivers
#
# CONFIG_SOFT_WATCHDOG is not set
# CONFIG_GPIO_WATCHDOG is not set
# CONFIG_XILINX_WATCHDOG is not set
# CONFIG_ZIIRAVE_WATCHDOG is not set
# CONFIG_ARM_SP805_WATCHDOG is not set
# CONFIG_ARM_SBSA_WATCHDOG is not set
# CONFIG_CADENCE_WATCHDOG is not set
# CONFIG_DW_WATCHDOG is not set
# CONFIG_MAX63XX_WATCHDOG is not set
CONFIG_HOBOT_WATCHDOG=y
CONFIG_HOBOT_WATCHDOG_ENABLE=y/*Enable this option for the system to automatically feed the dog*/
# CONFIG_HOBOT_WATCHDOG_TEST is not set
# CONFIG_MEN_A21_WDT is not set
```

## Usage Example

```
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h> //UNIX standard function definitions
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h> // file control definition
#include <termios.h> // PPSIX terminal control definition
#include <errno.h> // error number definition
#include <pthread.h>
#include <linux/watchdog.h>
#include <string.h>
#include <sys/ioctl.h>

int watchdogfd;
int feeddog = 1;

void *feeddogthread()
{
    int feeddogvalue;
    int returnval;

    feeddogvalue = 65535;

    while (feeddog)
    {
        // reload the watchdog count register every 10 seconds
        printf("feed dog\n");
        returnval = write(watchdogfd, &feeddogvalue, sizeof(int));
        sleep(10);
    }
}

int main()
{
    pthread_t watchdogThd;
    // int watchdogfd;
    int returnval;
    char readline[32], *p;

    // open the watchdog device
    if ((watchdogfd = open("/dev/watchdog", O_RDWR | O_NONBLOCK)) < 0)
    {
        printf("cannot open the watchdog device\n");
        exit(0);
    }

    int timeout = 15;
    int timeleft;
    ioctl(watchdogfd, WDIOC_SETTIMEOUT, &timeout);
    printf("The timeout was set to %d seconds\n", timeout);

    // create feeddog thread
    returnval = pthread_create(&watchdogThd, NULL, feeddogthread, NULL);
    if (returnval < 0)
        printf("cannot create feeddog thread\n");
}while (1) {
        printf("Command (e quit): ");
        memset(readline, '\0', sizeof(readline));
        fgets(readline, sizeof(readline), stdin);

        /* Remove leading spaces */
        p = readline;
        while (*p == ' ' || *p == '\t')
            p++;

        switch (*p) {
        case 'g':
            ioctl(watchdogfd, WDIOC_GETTIMEOUT, &timeout);
            printf("The timeout was is %d seconds\n", timeout);
            break;
        case 'e':
            printf("Close watchdog and exit safely!\n");
            int disable_dog = WDIOS_DISABLECARD;
            ioctl(watchdogfd, WDIOC_SETOPTIONS, &disable_dog);
            close(watchdogfd);
            break;
        case 's':
            printf("stop feed dog\n");
            feeddog = 0;
            break;
        case 't':
            ioctl(watchdogfd, WDIOC_GETTIMELEFT, &timeleft);
            printf("The timeout was is %d seconds\n", timeleft);
            break;
        case 'r':
            printf("we don't close watchdog. The machine will reboot in a few seconds!\n");
            printf("wait......\n");
            break;
        default:
            printf("get error char: %c\n", *p);
        }

    }

    return 0;
}