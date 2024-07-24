---
sidebar_position: 14
---

# 看门狗驱动调试指南

## 代码路径

```
drivers/watchdog/hobot_wdt.c # watchdog 驱动代码源文件
include/linux/watchdog.h # watchdog 驱动代码头文件
```

## dts配置

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

## 内核配置

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
# CONFIG_HOBOT_WATCHDOG_ENABLE is not set/*打开这个选项系统会自动喂狗*/
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
CONFIG_HOBOT_WATCHDOG_ENABLE=y/*打开这个选项系统会自动喂狗*/
# CONFIG_HOBOT_WATCHDOG_TEST is not set
# CONFIG_MEN_A21_WDT is not set
```

## 使用示例

```
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h> //UNIX标准函数定义
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>  //文件控制定义
#include <termios.h>    //PPSIX终端控制定义
#include <errno.h>  //错误号定义
#include <pthread.h>
#include <linux/watchdog.h>
#include <string.h>
#include <sys/ioctl.h>

int watchdogfd;
int feeddog = 1;

void* feeddogthread()
{
	int feeddogvalue;
	int returnval;

	feeddogvalue = 65535;

	while (feeddog) {
		//每隔10秒，将重载看门狗计数寄存器的值
		printf("feed dog\n");
		returnval = write(watchdogfd, &feeddogvalue, sizeof(int));
		sleep(10);
	}
}

int main()
{
	pthread_t watchdogThd;
	//int watchdogfd;
	int returnval;
	char readline[32], *p;

	//打开看门狗设备
	if ((watchdogfd = open("/dev/watchdog", O_RDWR|O_NONBLOCK)) < 0) {
		printf("cannot open the watchdog device\n");
		exit(0);
	}

	int timeout = 15;
	int timeleft;
    ioctl(watchdogfd, WDIOC_SETTIMEOUT, &timeout);
    printf("The timeout was set to %d seconds\n", timeout);

	//创建喂狗线程
	returnval = pthread_create(&watchdogThd, NULL, feeddogthread, NULL);
	if (returnval < 0)
		printf("cannot create feeddog thread\n");

	while (1) {
		printf("Command (e quit): ");
		memset(readline, '\0', sizeof(readline));
        fgets(readline, sizeof(readline), stdin);
    
        /* 去字符串前部空字符 */
        p = readline;
        while(*p == ' ' || *p == '\t')
                p++;

        switch(*p) {
        case 'g':
        	ioctl(watchdogfd, WDIOC_GETTIMEOUT, &timeout);
    		printf("The timeout was is %d seconds\n", timeout);
    		break;
    	case 'e':
			printf("Close watchdog an exit safety!\n");
			//write(watchdogfd, "V", 1);
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

```
