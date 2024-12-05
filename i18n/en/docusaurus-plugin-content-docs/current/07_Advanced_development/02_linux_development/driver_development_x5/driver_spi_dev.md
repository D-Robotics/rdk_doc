---
sidebar_position: 7
---

# SPI Debugging Guide

## SPI Hardware Support

X5 supports a total of 7 SPI controllers, with 6 controllers (spi0-spi5) located in the LSIO subsystem and 1 controller (spi6) located in the DSP subsystem. All SPI controllers support both master and slave modes.

## Linux SPI Driver Framework Overview

- **spi driver layer**: This layer mainly implements operations for the SPI hardware IP, and also implements the interfaces defined by the SPI framework.
- **spi framework layer**: This layer can be considered as an adaptation layer for the SPI driver. It defines a set of interfaces that the lower layer (driver) needs to implement, while providing a common interface that abstracts hardware details to the upper layers.
- **spi char device layer**: Provides nodes for the user space, making it easier for data exchange between user space and kernel space.

### Code Path

X5 uses the NewThink (新思) SSI controller, and the driver code is located in the `drivers/spi` directory. The main files are as follows:


```bash
drivers/spi/spi-dw-core.c
drivers/spi/spi-dw-mmio.c
drivers/spi/spi-dw-dma.c
```
### Controller Hardware Description

All SPI controllers on the X5 can operate in Master/Slave modes. The operational limitations for Master and Slave modes are as follows:

- **SPI-Master**: Maximum frequency of 50MHz
- **SPI-Slave**: Maximum frequency of 32MHz

All SPI controllers on the X5 can operate in either interrupt or DMA mode. The limitations for interrupt mode are as follows:

- **SPI-Slave**:
    - **Rx**: With the CPU running at a fixed frequency of 1.5GHz, it can achieve up to 32MHz.
    - **Tx**: With the CPU running at a fixed frequency of 1.5GHz, it can achieve up to 16MHz.
- **SPI-Master**: Normal communication is supported.

### DTS Configuration Description

The device tree definitions for the X5 SPI controllers are located in the `x5.dtsi` file under the `arch/arm64/boot/dts/hobot/` folder in the SDK package kernel directory.

<font color="red">Note:</font> The nodes in `x5.dtsi` mainly declare SoC-level features and are independent of specific circuit boards. In most cases, these do not need to be modified.

The X5 SPI controller is disabled by default. Please enable the corresponding SPI controller in the appropriate DTS file based on your actual hardware configuration.

For example, to enable SPI2:


```c
&spi2 {
	status = "okay";
	pinctrl-names = "default";
	pinctrl-0 = <&pinctrl_spi2>;
};
```

### DTS Configuration for Using DMA

To use DMA, you need to bind the corresponding DMA handshake in the relevant DTS file. For example, to enable DMA for SPI2:


```c
&spi2 {
	status = "okay";
	pinctrl-names = "default";
	pinctrl-0 = <&pinctrl_spi2>;
	dma-names = "tx", "rx";
	dmas = <&axi_dmac 25>, <&axi_dmac 24>;
};
```

When binding SPI6, you need to specify `dsp_axi_dma`, as shown below:


```c
&spi6 {
	status = "okay";
	pinctrl-names = "default";
	pinctrl-0 = <&pinctrl_dsp_spi>;
	dma-names = "tx", "rx";
	dmas = <&dsp_axi_dma 21>, <&dsp_axi_dma 20>;
};
```

SPI DMA Handshake List:

| SPI  | DMA TX | DMA RX |
| ---- | ------ | ------ |
| SPI0 | 20     | 21     |
| SPI1 | 22     | 23     |
| SPI2 | 24     | 25     |
| SPI3 | 26     | 27     |
| SPI4 | 28     | 29     |
| SPI5 | 30     | 31     |
| SPI6 | 21     | 22     |

## SPI Function Verification

This section introduces the function verification of X5 SPI, including environment setup, test command execution, and test code.

### Test Environment Preparation

Ensure that the kernel's `CONFIG_SPI_SPIDEV` is enabled:


```c
/* arch/arm64/configs/hobot_x5_soc_defconfig */
...
CONFIG_SPI_SPIDEV=m
...
```

Ensure that a dummy slave device has been created under the SPI controller node to be tested in the current hardware's DTS.


```c
&spi2 {
	spidev@2 {
		compatible = "dr,x5-spidev";
		spi-max-frequency = <32000000>;
		reg = <0>;
	};
}
```

### SPI Internal Loopback Test

The SPI internal loopback test is only supported by SPI Master. The principle is that the SPI hardware's TX FIFO sends data to the RX FIFO, forming a loopback.

The test command and expected results are as follows:


```c
# modprobe spidev
# ./spidev_tc -D /dev/spidev2.0 -v -s 1000000 -m 3 -l -e 10 -t 1
   spi mode: 0x20
   bits per word: 8
   max speed: 1000000 Hz (1000 KHz)
   userspace spi read and write test, len=10 times=1
   test, times=0
   TX | 67 C6 69 73 51 FF 4A EC 29 CD __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __

   RX | 67 C6 69 73 51 FF 4A EC 29 CD __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __
```

### SPI External Loopback Test

The SPI external loopback test involves setting up one SPI Slave and one SPI Master, with corresponding wires connected for the test. 


```c
&spi2 {
	status = "okay";
	pinctrl-names = "default";
	pinctrl-0 = <&pinctrl_spi2>;
	spi-slave;

	slave@0 {
		compatible = "dr,x5-spidev";
		spi-max-frequency = <32000000>;
		reg = <0>;
	};
}
```

Modify the SPI4 DTS to support Master functionality as follows:


```c
&spi4 {
	status = "okay";
	pinctrl-names = "default";
	pinctrl-0 = <&pinctrl_spi4>;

	spidev@0 {
		compatible = "dr,x5-spidev";
		spi-max-frequency = <32000000>;
		reg = <0>;
	};
}
```

The test command and expected results are as follows (using SPI2 as the Slave and SPI4 as the Master):


```c
# ./spidev_tc -D /dev/spidev2.0 -v -s 1000000 -m 2 -e 10 -t 1&
	bits per word: 8
	max speed: 1000000 Hz (1000 KHz)
	userspace spi write test, len=10 times=1
	test, times=0
	TX | 67 C6 69 73 51 FF 4A EC 29 CD __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __

# ./spidev_tc -D /dev/spidev4.0 -v -s 1000000 -m 1 -e 10 -t 1&
	spi mode: 0x0
	bits per word: 8
	max speed: 1000000 Hz (1000 KHz)
	userspace spi read test, len=10 times=1
	test, times=0
	RX | 67 C6 69 73 51 FF 4A EC 29 CD __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __

[2]+  Done                       ./spidev_tc -D /dev/spidev2.0 -v -s 1000000 -m 1 -e 10 -t 1
[1]+  Done                       ./spidev_tc -D /dev/spidev4.0 -v -s 1000000 -m 2 -e 10 -t 1
```

:::info Note
When performing the external loopback test, the SPI Slave program should be executed first, followed by the SPI Master program. If the SPI Master program is executed first, followed by the SPI Slave program, data loss may occur due to desynchronization between the Master and Slave. Similarly, the `-t` parameter of `spidev_tc` should only be specified once. If multiple tests are needed, it is recommended to write a script to execute the test program multiple times to ensure synchronization between the Master and Slave.
:::

## Appendix

**Appendix 1: Test Case Source Code: spidev_tc.c**


```c
/*
 * SPI testing utility (using spidev driver)
 *
 * Copyright (c) 2007  MontaVista Software, Inc.
 * Copyright (c) 2007  Anton Vorontsov <avorontsov@ru.mvista.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License.
 *
 * Cross-compile with cross-gcc -I/path/to/cross-kernel/include
 */

#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <getopt.h>
#include <fcntl.h>
#include <time.h>
#include <sys/ioctl.h>
#include <linux/ioctl.h>
#include <sys/stat.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))

static void pabort(const char *s)
{
	perror(s);
	abort();
}

static const char *device = "/dev/spidev1.1";
static uint32_t mode;
static uint8_t bits = 8;
static char *input_file;
static char *output_file;
static uint32_t speed = 500000;
static uint16_t delay;
static int verbose;
static int transfer_size;
static int iterations;
static int interval = 5; /* interval in seconds for showing transfer rate */
static int rw_mode = 0;	//1: read, 2: write, 3: write and read
static int rw_len = 4;
static int rw_times = 5;
static int frameid_flag = 0;
static int crc_flag = 0;
static int heart_flag = 0;
static uint32_t tx_frameid = 0, rx_frameid = 0;
static uint32_t heart_interval = 5;	// 5s

uint8_t default_tx[] = {
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0x40, 0x00, 0x00, 0x00, 0x00, 0x95,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xF0, 0x0D,
};

uint8_t heart_pack[] = {
	0xAA, 0x55, 0xAA, 0x55,
	0xAA, 0x55, 0xAA, 0x55,
	0xAA, 0x55, 0xAA, 0x55,
	0xAA, 0x55, 0xAA, 0x55,
};

uint8_t default_rx[ARRAY_SIZE(default_tx)] = {0, };
char *input_tx;

/** CRC table for the CRC-16. The poly is 0x8005 (x^16 + x^15 + x^2 + 1) */
static uint16_t const crc16_table[256] = {
	0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
	0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
	0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
	0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
	0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
	0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
	0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
	0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
	0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
	0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
	0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
	0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
	0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
	0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
	0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
	0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
	0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
	0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
	0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
	0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
	0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
	0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
	0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
	0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
	0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
	0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
	0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
	0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
	0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
	0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
	0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
	0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040
};

static inline uint16_t crc16_byte(uint16_t crc, const uint8_t data)
{
	return (crc >> 8) ^ crc16_table[(crc ^ data) & 0xff];
}

static uint16_t crc16(uint16_t crc, uint8_t const *buffer, size_t len)
{
	while (len--)
		crc = crc16_byte(crc, *buffer++);
	return crc;
}

static void hex_dump(const void *src, size_t length, size_t line_size,
		     char *prefix)
{
	int i = 0;
	const unsigned char *address = src;
	const unsigned char *line = address;
	unsigned char c;

	printf("%s | ", prefix);
	while (length-- > 0) {
		printf("%02X ", *address++);
		if (!(++i % line_size) || (length == 0 && i % line_size)) {
			if (length == 0) {
				while (i++ % line_size)
					printf("__ ");
			}
			printf(" | ");  /* right close */
			while (line < address) {
				c = *line++;
				printf("%c", (c < 33 || c == 255) ? 0x2E : c);
			}
			printf("\n");
			if (length > 0)
				printf("%s | ", prefix);
		}
	}
}

static void hex_dump2(const void *src, size_t length, size_t line_size,
		  char *prefix)
{
     int i = 0;
     const unsigned char *address = src;

     printf("%s | ", prefix);
     while (length-- > 0) {
	     printf("%02X ", *address++);
	     if (!(++i % line_size) || (length == 0 && i % line_size)) {
		     if (length == 0) {
			     while (i++ % line_size)
				     printf("__ ");
		     }
		     printf("\n");
		     if (length > 0)
			     printf("%s | ", prefix);
	     }
     }
     printf("\n");
}

/*
 *  Unescape - process hexadecimal escape character
 *      converts shell input "\x23" -> 0x23
 */
static int unescape(char *_dst, char *_src, size_t len)
{
	int ret = 0;
	int match;
	char *src = _src;
	char *dst = _dst;
	unsigned int ch;

	while (*src) {
		if (*src == '\\' && *(src+1) == 'x') {
			match = sscanf(src + 2, "%2x", &ch);
			if (!match)
				pabort("malformed input string");

			src += 4;
			*dst++ = (unsigned char)ch;
		} else {
			*dst++ = *src++;
		}
		ret++;
	}
	return ret;
}

static void transfer(int fd, uint8_t const *tx, uint8_t const *rx, size_t len)
{
	int ret;
	int out_fd;
	struct spi_ioc_transfer tr = {
		.tx_buf = (unsigned long)tx,
		.rx_buf = (unsigned long)rx,
		.len = len,
		.delay_usecs = delay,
		.speed_hz = speed,
		.bits_per_word = bits,
	};

	struct timespec currentTime;
	struct timespec currentTime1;

	if (mode & SPI_TX_QUAD)
		tr.tx_nbits = 4;
	else if (mode & SPI_TX_DUAL)
		tr.tx_nbits = 2;
	if (mode & SPI_RX_QUAD)
		tr.rx_nbits = 4;
	else if (mode & SPI_RX_DUAL)
		tr.rx_nbits = 2;
	if (!(mode & SPI_LOOP)) {
		if (mode & (SPI_TX_QUAD | SPI_TX_DUAL))
			tr.rx_buf = 0;
		else if (mode & (SPI_RX_QUAD | SPI_RX_DUAL))
			tr.tx_buf = 0;
	}

	clock_gettime(CLOCK_REALTIME, &currentTime);
	long milliseconds = currentTime.tv_nsec / 1000000;
	printf("before trans time: %ld ms\n", milliseconds);
	ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
	if (ret < 1)
		pabort("can't send spi message");
	clock_gettime(CLOCK_REALTIME, &currentTime1);
	long milliseconds1 = currentTime1.tv_nsec / 1000000;
	printf("after trans time: %ld ms\n", milliseconds1);

	printf("diff: %ld ms\n", (milliseconds1 - milliseconds));

	if (verbose)
		hex_dump(tx, len, 32, "TX");

	if (output_file) {
		out_fd = open(output_file, O_WRONLY | O_CREAT | O_TRUNC, 0666);
		if (out_fd < 0)
			pabort("could not open output file");

		ret = write(out_fd, rx, len);
		if (ret != len)
			pabort("not all bytes written to output file");

		close(out_fd);
	}

	if (verbose)
		hex_dump(rx, len, 32, "RX");
}

static void transfer2(int fd, uint8_t const *tx, uint8_t const *rx, size_t len)
{
	int ret;
	int out_fd;
	struct spi_ioc_transfer tr = {
		.tx_buf = (unsigned long)tx,
		.rx_buf = (unsigned long)rx,
		.len = len,
		.delay_usecs = delay,
		.speed_hz = speed,
		.bits_per_word = bits,
	};

	if (mode & SPI_TX_QUAD)
		tr.tx_nbits = 4;
	else if (mode & SPI_TX_DUAL)
		tr.tx_nbits = 2;
	if (mode & SPI_RX_QUAD)
		tr.rx_nbits = 4;
	else if (mode & SPI_RX_DUAL)
		tr.rx_nbits = 2;
	if (!(mode & SPI_LOOP)) {
		if (mode & (SPI_TX_QUAD | SPI_TX_DUAL))
			tr.rx_buf = 0;
		else if (mode & (SPI_RX_QUAD | SPI_RX_DUAL))
			tr.tx_buf = 0;
	}

	if (verbose && rw_mode >> 1)
		hex_dump2(tx, len, 32, "TX");

	ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
	if (ret < 1) {
		//pabort("can't send spi message");
		printf("can't send spi message");
	} else {
		if (output_file) {
			out_fd = open(output_file, O_WRONLY | O_CREAT | O_TRUNC, 0666);
			if (out_fd < 0)
				pabort("could not open output file");

			ret = write(out_fd, rx, len);
			if (ret != len)
				pabort("not all bytes written to output file");

			close(out_fd);
		}

		if (verbose && rw_mode&0x01)
			hex_dump2(rx, len, 32, "RX");
	}
	if (strcmp(tx, rx) != 0) {
		printf("SPI transfer failed\n");
		exit(-1);
	}

}

static void transfer3(int fd, uint8_t const *tx, uint8_t const *rx, size_t len)
{
	int ret;
	uint8_t const *p = rx;
	struct spi_ioc_transfer tr = {
		.tx_buf = (unsigned long)tx,
		.rx_buf = (unsigned long)rx,
		.len = len,
		.delay_usecs = delay,
		.speed_hz = speed,
		.bits_per_word = bits,
	};

	if (mode & SPI_TX_QUAD)
		tr.tx_nbits = 4;
	else if (mode & SPI_TX_DUAL)
		tr.tx_nbits = 2;
	if (mode & SPI_RX_QUAD)
		tr.rx_nbits = 4;
	else if (mode & SPI_RX_DUAL)
		tr.rx_nbits = 2;
	if (!(mode & SPI_LOOP)) {
		if (mode & (SPI_TX_QUAD | SPI_TX_DUAL))
			tr.rx_buf = 0;
		else if (mode & (SPI_RX_QUAD | SPI_RX_DUAL))
			tr.tx_buf = 0;
	}

	hex_dump2(tx, len, 32, "TX");

	ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
	if (ret < 1) {
		printf("can't send spi message");
	} else {
		hex_dump2(rx, len, 32, "RX");

		if (frameid_flag) {
			uint32_t *pframeid = (uint32_t *)p;
			rx_frameid = *pframeid;
			p += sizeof(uint32_t);

			printf("device = %s, crc_val = %d", device, rx_frameid);
		}

		if (crc_flag) {
			uint16_t crc_val_check = crc16(0, p, sizeof(heart_pack));
			p += sizeof(heart_pack);
			uint16_t *pcrc_val = (uint16_t *)p;

			printf("device = %s, crc_val = %d, crc_val_check = %d", device, *pcrc_val, crc_val_check);
		}
	}
}

static void print_usage(const char *prog)
{
	printf("Usage: %s [-DsbdiolHOLC3vpNR24SImetfch1]\n", prog);
	puts("  -D --device   device to use (default /dev/spidev1.1)\n"
	     "  -s --speed    max speed (Hz)\n"
	     "  -d --delay    delay (usec)\n"
	     "  -b --bpw      bits per word\n"
	     "  -i --input    input data from a file (e.g. \"test.bin\")\n"
	     "  -o --output   output data to a file (e.g. \"results.bin\")\n"
	     "  -l --loop     loopback\n"
	     "  -H --cpha     clock phase\n"
	     "  -O --cpol     clock polarity\n"
	     "  -L --lsb      least significant bit first\n"
	     "  -C --cs-high  chip select active high\n"
	     "  -3 --3wire    SI/SO signals shared\n"
	     "  -v --verbose  Verbose (show tx buffer)\n"
	     "  -p            Send data (e.g. \"1234\\xde\\xad\")\n"
	     "  -N --no-cs    no chip select\n"
	     "  -R --ready    slave pulls low to pause\n"
	     "  -2 --dual     dual transfer\n"
	     "  -4 --quad     quad transfer\n"
	     "  -S --size     transfer size\n"
	     "  -I --iter     iterations\n"
	     "  -m --rw-mode  1 read, 2 write, 3 write and read\n"
	     "  -e --rw-len   read or write len\n"
	     "  -t --rw-times read or write times\n"
		 "  -f --frame-id support frame id\n"
		 "  -c --crc support crc\n"
		 "  -h --heart pack for test\n"
		 "  -1 --heart pack interval(default: 5s)\n");
	exit(1);
}

static void parse_opts(int argc, char *argv[])
{
	while (1) {
		static const struct option lopts[] = {
			{ "device",  1, 0, 'D' },
			{ "speed",   1, 0, 's' },
			{ "delay",   1, 0, 'd' },
			{ "bpw",     1, 0, 'b' },
			{ "input",   1, 0, 'i' },
			{ "output",  1, 0, 'o' },
			{ "loop",    0, 0, 'l' },
			{ "cpha",    0, 0, 'H' },
			{ "cpol",    0, 0, 'O' },
			{ "lsb",     0, 0, 'L' },
			{ "cs-high", 0, 0, 'C' },
			{ "3wire",   0, 0, '3' },
			{ "no-cs",   0, 0, 'N' },
			{ "ready",   0, 0, 'R' },
			{ "dual",    0, 0, '2' },
			{ "verbose", 0, 0, 'v' },
			{ "quad",    0, 0, '4' },
			{ "size",    1, 0, 'S' },
			{ "iter",    1, 0, 'I' },
			{ "rw-mode",    1, 0, 'm' },
			{ "rw-len",    1, 0, 'e' },
			{ "rw-times",    1, 0, 't' },
			{ "frame-id",	0, 0, 'f' },
			{ "crc", 0, 0, 'c' },
			{ "heart", 0, 0, 'h' },
			{ "interval", 0, 0, '1' },
			{ NULL, 0, 0, 0 },
		};
		int c;

		c = getopt_long(argc, argv, "D:s:d:b:i:o:lHOLC3NR24p:vS:I:m:e:t:fch1:",
				lopts, NULL);
		//printf("optind: %d\n", optind);
		//printf("optarg: %s\n", optarg);
		//printf("option: %c\n", c);

		if (c == -1)
			break;

		switch (c) {
		case 'D':
			device = optarg;
			break;
		case 's':
			speed = atoi(optarg);
			break;
		case 'd':
			delay = atoi(optarg);
			break;
		case 'b':
			bits = atoi(optarg);
			break;
		case 'i':
			input_file = optarg;
			break;
		case 'o':
			output_file = optarg;
			break;
		case 'l':
			mode |= SPI_LOOP;
			break;
		case 'H':
			mode |= SPI_CPHA;
			break;
		case 'O':
			mode |= SPI_CPOL;
			break;
		case 'L':
			mode |= SPI_LSB_FIRST;
			break;
		case 'C':
			mode |= SPI_CS_HIGH;
			break;
		case '3':
			mode |= SPI_3WIRE;
			break;
		case 'N':
			mode |= SPI_NO_CS;
			break;
		case 'v':
			verbose = 1;
			break;
		case 'R':
			mode |= SPI_READY;
			break;
		case 'p':
			input_tx = optarg;
			break;
		case '2':
			mode |= SPI_TX_DUAL;
			break;
		case '4':
			mode |= SPI_TX_QUAD;
			break;
		case 'S':
			transfer_size = atoi(optarg);
			break;
		case 'I':
			iterations = atoi(optarg);
			break;
		case 'm':
			rw_mode = atoi(optarg);
			break;
		case 'e':
			rw_len = atoi(optarg);
			break;
		case 't':
			rw_times = atoi(optarg);
			break;
		case 'f':
			frameid_flag = 1;
			break;
		case 'c':
			crc_flag = 1;
			break;
		case 'h':
			heart_flag = 1;
			break;
		case '1':
			heart_interval = atoi(optarg);
			break;
		default:
			print_usage(argv[0]);
			break;
		}
	}
	if (mode & SPI_LOOP) {
		if (mode & SPI_TX_DUAL)
			mode |= SPI_RX_DUAL;
		if (mode & SPI_TX_QUAD)
			mode |= SPI_RX_QUAD;
	}
}

static void transfer_escaped_string(int fd, char *str)
{
	size_t size = strlen(str);
	uint8_t *tx;
	uint8_t *rx;

	tx = malloc(size);
	if (!tx)
		pabort("can't allocate tx buffer");

	rx = malloc(size);
	if (!rx)
		pabort("can't allocate rx buffer");

	size = unescape((char *)tx, str, size);
	transfer(fd, tx, rx, size);
	free(rx);
	free(tx);
}

static void transfer_file(int fd, char *filename)
{
	ssize_t bytes;
	struct stat sb;
	int tx_fd;
	uint8_t *tx;
	uint8_t *rx;

	if (stat(filename, &sb) == -1)
		pabort("can't stat input file");

	tx_fd = open(filename, O_RDONLY);
	if (tx_fd < 0)
		pabort("can't open input file");

	tx = malloc(sb.st_size);
	if (!tx)
		pabort("can't allocate tx buffer");

	rx = malloc(sb.st_size);
	if (!rx)
		pabort("can't allocate rx buffer");

	bytes = read(tx_fd, tx, sb.st_size);
	if (bytes != sb.st_size)
		pabort("failed to read input file");

	transfer(fd, tx, rx, sb.st_size);
	free(rx);
	free(tx);
	close(tx_fd);
}

static uint64_t _read_count;
static uint64_t _write_count;

static void show_transfer_rate(void)
{
	static uint64_t prev_read_count, prev_write_count;
	double rx_rate, tx_rate;

	rx_rate = ((_read_count - prev_read_count) * 8) / (interval*1000.0);
	tx_rate = ((_write_count - prev_write_count) * 8) / (interval*1000.0);

	printf("rate: tx %.1fkbps, rx %.1fkbps\n", rx_rate, tx_rate);

	prev_read_count = _read_count;
	prev_write_count = _write_count;
}

static void transfer_buf(int fd, int len)
{
	uint8_t *tx;
	uint8_t *rx;
	int i;

	tx = malloc(len);
	if (!tx)
		pabort("can't allocate tx buffer");
	for (i = 0; i < len; i++)
		tx[i] = random();

	rx = malloc(len);
	if (!rx)
		pabort("can't allocate rx buffer");

	transfer(fd, tx, rx, len);

	_write_count += len;
	_read_count += len;

	if (mode & SPI_LOOP) {
		if (memcmp(tx, rx, len)) {
			fprintf(stderr, "transfer error !\n");
			hex_dump(tx, len, 32, "TX");
			hex_dump(rx, len, 32, "RX");
			exit(1);
		}
	}

	free(rx);
	free(tx);
}

static void transfer_read_write(int fd)
{
	uint8_t *tx;
	uint8_t *rx;
	int i, j;
	int len, times;
	char str[64] = {0};

	len = rw_len > 0 ? rw_len : 4;
	times = rw_times > 0 ? rw_times : 4;
	if (rw_mode == 2)
		sprintf(str, "write");
	else if (rw_mode == 3)
		sprintf(str, "read and write");
	else {
		rw_mode = 1;
		sprintf(str, "read");
	}

	printf("userspace spi %s test, len=%d times=%d\n", str, len, times);

	tx = malloc(len + 4);
	if (!tx)
		pabort("can't allocate tx buffer");
	rx = malloc(len + 4);
	if (!rx)
		pabort("can't allocate rx buffer");

	for (j = 0; j < rw_times; j++) {
		memset(tx, 0, len);
		memset(rx, 0, len);

		if (rw_mode >> 1) {
			for (i = 0; i < len; i++)
				tx[i] = random();
		} else {
			for (i = 0; i < len; i++)
				tx[i] = i;
		}
		printf("test, times=%d\n", j);
		transfer2(fd, tx, rx, len);
		//sleep(2);
	}
}

static void transfer_heart_pack(int fd)
{
	uint8_t *p;
	uint8_t *tx;
	uint8_t *rx;
	int len;

	len = sizeof(heart_pack);
	if (frameid_flag)
		len += sizeof(uint32_t);
	if (crc_flag)
		len += sizeof(uint16_t);

	tx = malloc(len);
	if (!tx)
		pabort("can't allocate tx buffer");
	rx = malloc(len);
	if (!rx)
		pabort("can't allocate rx buffer");

	while (1) {
		memset(tx, 0 ,len);
		memset(rx, 0, len);

		p = tx;
		if (frameid_flag) {
			uint32_t *pframeid = (uint32_t *)p;
			*pframeid = tx_frameid;
			tx_frameid++;
			p += sizeof(uint32_t);

			printf("device = %s, crc_val = %d", device, *pframeid);
		}

		memcpy(p, heart_pack, sizeof(heart_pack));
		p += sizeof(heart_pack);

		if (crc_flag) {
			uint16_t crc_val = crc16(0, heart_pack, sizeof(heart_pack));
			uint16_t *pcrc_val = (uint16_t *)p;
			*pcrc_val = crc_val;

			printf("device = %s, crc_val = %d", device, crc_val);
		}

		transfer3(fd, tx, rx, len);

		sleep(heart_interval);
	}
}

int main(int argc, char *argv[])
{
	int ret = 0;
	int fd;

	parse_opts(argc, argv);

	fd = open(device, O_RDWR);
	if (fd < 0)
		pabort("can't open device");

	/*
	 * spi mode
	 */
	ret = ioctl(fd, SPI_IOC_WR_MODE32, &mode);
	if (ret == -1)
		pabort("can't set spi mode");

	ret = ioctl(fd, SPI_IOC_RD_MODE32, &mode);
	if (ret == -1)
		pabort("can't get spi mode");

	/*
	 * bits per word
	 */
	ret = ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
	if (ret == -1)
		pabort("can't set bits per word");

	ret = ioctl(fd, SPI_IOC_RD_BITS_PER_WORD, &bits);
	if (ret == -1)
		pabort("can't get bits per word");

	/*
	 * max speed hz
	 */
	ret = ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
	if (ret == -1)
		pabort("can't set max speed hz");

	ret = ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed);
	if (ret == -1)
		pabort("can't get max speed hz");

	printf("spi mode: 0x%x\n", mode);
	printf("bits per word: %d\n", bits);
	printf("max speed: %d Hz (%d KHz)\n", speed, speed/1000);

	if (input_tx && input_file)
		pabort("only one of -p and --input may be selected");

	if (input_tx)
		transfer_escaped_string(fd, input_tx);
	else if (input_file)
		transfer_file(fd, input_file);
	else if (transfer_size) {
		struct timespec last_stat;

		clock_gettime(CLOCK_MONOTONIC, &last_stat);

		while (iterations-- > 0) {
			struct timespec current;

			transfer_buf(fd, transfer_size);

			clock_gettime(CLOCK_MONOTONIC, &current);
			if (current.tv_sec - last_stat.tv_sec > interval) {
				show_transfer_rate();
				last_stat = current;
			}
		}
		printf("total: tx %.1fKB, rx %.1fKB\n",
		       _write_count/1024.0, _read_count/1024.0);
	} else if (rw_mode) {
		transfer_read_write(fd);
	} else if (heart_flag) {
		transfer_heart_pack(fd);
	} else
		transfer(fd, default_tx, default_rx, sizeof(default_tx));

	close(fd);

	return ret;
}
```

**附录2 测试用例源码：Makefile**

```shell
OUT_DIR = $(shell pwd)/_build/

.PHONY: build install clean

BIN_NAME = spidev_tc
SRCS = $(wildcard ./*.c)
OBJS = $(addprefix ${OUT_DIR}/, $(patsubst %.c, %.o, ${SRCS}))

BIN_TEST1 = ${OUT_DIR}/${BIN_NAME}
OBJECT = ${BIN_TEST1}

build: ${OBJECT}

Q:=

$(OBJECT): $(OBJS)
	$(Q)mkdir -p $(abspath $(dir $@))
	$(Q)echo CC $@
	$(Q)${CC} ${CFLAGS} ${INCS} $^ ${LDFLAGS} $(LIBS) -o $@

${OUT_DIR}/%.o: %.c
	$(Q)mkdir -p $(abspath $(dir $@))
	$(Q)echo CC $@
	$(Q)${CC} $(INCS) -c $< -o $@

clean :
	rm -rf $(OBJS) $(OBJECT)
```
