---
sidebar_position: 3
---

# I2C Debugging Guide

## Preface

The S100 chip provides a standard I2C bus. The I2C bus controller transfers information between devices connected to the bus via a Serial Data Line (SDA) and a Serial Clock Line (SCL). Each device has a unique address (whether it's a microcontroller—MCU, LCD controller, memory, or keyboard interface) and can act as both a transmitter and a receiver (determined by the device's functionality).

The I2C controller supports the following features:

- Supports four speed modes:
    - Standard mode (0–100 Kb/s)
    - Fast mode (100–400 Kb/s)
    - Fast mode plus (400–1000 Kb/s)
    - High-speed mode (1000 Kb/s–3.4 Mb/s)
- Supports master/slave mode configuration
- Supports both 7-bit and 10-bit addressing modes

## Driver Code

```bash
drivers/i2c/i2c-dev.c # I2C character device interface code
drivers/i2c/i2c-core-base.c  # I2C framework code
drivers/i2c/busses/i2c-designware-platdrv.c   # I2C driver source file
```

### Kernel Configuration Location

```bash
/* hobot-drivers/configs/drobot_s100_defconfig */
CONFIG_I2C_CHARDEV=y 			# I2C driver userspace configuration macro
CONFIG_I2C_DESIGNWARE_PLATFORM=y 	# DesignWare I2C driver configuration macro
```

### Kernel DTS Node Configuration

``` {.text}
/*kernel/arch/arm64/boot/dts/hobot/drobot-s100-soc.dtsi*/
i2c0: i2c@39420000 {
        power-domains = <&scmi_smc_pd PD_IDX_LSPERI_TOP>;
        #address-cells = <1>;
        #size-cells = <0>;
        compatible = "snps,designware-i2c";
        reg = <0x0 0x39420000 0x0 0x10000>;
        clocks = <&scmi_smc_clk CLK_IDX_TOP_PERI_I2C0>;
        clock-names = "apb_pclk";
        interrupts = <GIC_SPI PERISYS_I2C0_INTR PERISYS_I2C0_INTR_TRIG_TYPE>;
        clock-frequency = <400000>;
        i2c-sda-hold-time-ns = <50>;
        pinctrl-names = "default", "gpio";
        pinctrl-0 = <&cam_i2c0>;
        pinctrl-1 = <&cam_i2c0_gpio>;
        scl-gpios = <&cam_port0 8 GPIO_ACTIVE_HIGH>;
        sda-gpios = <&cam_port0 9 GPIO_ACTIVE_HIGH>;
        status = "okay";
};
```

## I2C Usage

Detailed instructions on I2C usage can be found in `kernel/Documentation/i2c/`. This document mainly highlights S100-specific aspects of the I2C driver interface.

### Kernel Space

The S100 I2C driver provides an interface in kernel space to configure the I2C transfer frequency. Usage is as follows:

#### I2C Speed Configuration

The default I2C speed is 400K. The driver supports four speeds: 100K, 400K, 1M, and 3.4M. The speed can be modified by adjusting the `clock-frequency` property of the corresponding I2C node in the device tree (DTS). The relevant code for actual speed selection is as follows:

``` {.text}
kernel/drivers/i2c/busses/i2c-designware-common.c
Supported I2C speeds are defined as:
static const u32 supported_speeds[] = {
	I2C_MAX_HIGH_SPEED_MODE_FREQ,
	I2C_MAX_FAST_MODE_PLUS_FREQ,
	I2C_MAX_FAST_MODE_FREQ,
	I2C_MAX_STANDARD_MODE_FREQ,
};
...
kernel/drivers/i2c/busses/i2c-designware-pcidrv.c
The function interface to retrieve clock-frequency from the device tree is:
i2c_parse_fw_timings(&pdev->dev, t, false);
Expanded as:
void i2c_parse_fw_timings(struct device *dev, struct i2c_timings *t, bool use_defaults)
{
	bool u = use_defaults;
	u32 d;

	i2c_parse_timing(dev, "clock-frequency", &t->bus_freq_hz,
			 I2C_MAX_STANDARD_MODE_FREQ, u);

	d = t->bus_freq_hz <= I2C_MAX_STANDARD_MODE_FREQ ? 1000 :
	    t->bus_freq_hz <= I2C_MAX_FAST_MODE_FREQ ? 300 : 120;
	i2c_parse_timing(dev, "i2c-scl-rising-time-ns", &t->scl_rise_ns, d, u);

	d = t->bus_freq_hz <= I2C_MAX_FAST_MODE_FREQ ? 300 : 120;
	i2c_parse_timing(dev, "i2c-scl-falling-time-ns", &t->scl_fall_ns, d, u);

	i2c_parse_timing(dev, "i2c-scl-internal-delay-ns",
			 &t->scl_int_delay_ns, 0, u);
	i2c_parse_timing(dev, "i2c-sda-falling-time-ns", &t->sda_fall_ns,
			 t->scl_fall_ns, u);
	i2c_parse_timing(dev, "i2c-sda-hold-time-ns", &t->sda_hold_ns, 0, u);
	i2c_parse_timing(dev, "i2c-digital-filter-width-ns",
			 &t->digital_filter_width_ns, 0, u);
	i2c_parse_timing(dev, "i2c-analog-filter-cutoff-frequency",
			 &t->analog_filter_cutoff_freq_hz, 0, u);
}
The function interface to validate I2C speed is:
i2c_dw_validate_speed(dev);
Expanded as:
int i2c_dw_validate_speed(struct dw_i2c_dev *dev)
{
	struct i2c_timings *t = &dev->timings;
	unsigned int i;

	/*
	 * Only standard mode at 100kHz, fast mode at 400kHz,
	 * fast mode plus at 1MHz and high speed mode at 3.4MHz are supported.
	 */
	for (i = 0; i < ARRAY_SIZE(supported_speeds); i++) {
		if (t->bus_freq_hz == supported_speeds[i])
			return 0;
	}

	dev_err(dev->dev,
		"%d Hz is unsupported, only 100kHz, 400kHz, 1MHz and 3.4MHz are supported\n",
		t->bus_freq_hz);

	return -EINVAL;
}
```

### User Space

Typically, I2C devices are controlled by kernel drivers, but all devices on the bus can also be accessed from userspace via the `/dev/i2c-%d` interface. Detailed documentation is available in `Documentation/i2c/dev-interface.rst` under the kernel source tree.

#### Debug Interfaces

**Register Information Retrieval**

To view I2C register information (using i2c-0 as an example):

``` {.text}
root@ubuntu:/# cat /sys/kernel/debug/dw_i2c0/registers
39420000.i2c registers:
=================================
CON:            0x00000065
SAR:            0x00000055
DATA_CMD:       0x00000800
INTR_STAT:      0x00000000
INTR_MASK:      0x00000000
RX_TL:          0x00000000
TX_TL:          0x00000002
STATUS:         0x00000006
TXFLR:          0x00000000
RXFLR:          0x00000000
SDA_HOLD:       0x0001000c
TX_ABRT:        0x00000000
EN_STATUS:      0x00000000
CLR_RESTA:      0x00000000
PARAM:          0x000303ee
VERSION:        0x3230322a
TYPE:           0x44570140
=================================
```

**reldump_en Interface**

Real-time dump enable interface. When enabled, real-time I2C transfer data can be viewed via `dmesg`.

``` {.text}
# enable
echo 1 > /sys/kernel/debug/dw_i2c0/reldump_en

# disable
echo 0 > /sys/kernel/debug/dw_i2c0/reldump_en
```

**fifodump_en Interface**

FIFO dump enable interface. When enabled, it dumps the most recent I2C read/write transactions. Each channel is configured separately and only supports master mode with 7-bit addressing.

``` {.text}
# enable
echo 1 > /sys/kernel/debug/dw_i2c0/fifodump_en

# disable
echo 0 > /sys/kernel/debug/dw_i2c0/fifodump_en
```

**fifodump Interface**

Temporarily stores I2C data. When `fifodump_en` is enabled, the stored data can be printed using `cat`.

Transmission normal:

``` {.text}
root@ubuntu:~# i2cdump -f -y 0 0x28
     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f    0123456789abcdef
00: 00 1f 21 00 00 00 00 00 00 00 XX XX XX XX XX XX    .?!.......XXXXXX
10: XX XX XX XX XX XX XX XX XX XX XX XX XX XX XX XX    XXXXXXXXXXXXXXXX
20: XX XX XX XX XX XX XX XX XX XX XX XX XX XX XX XX    XXXXXXXXXXXXXXXX
30: XX XX XX XX XX XX XX XX XX XX XX XX XX XX XX XX    XXXXXXXXXXXXXXXX
40: XX XX XX XX XX XX XX XX XX XX XX XX XX XX XX XX    XXXXXXXXXXXXXXXX
50: XX XX XX XX XX XX XX XX XX XX XX XX XX XX XX XX    XXXXXXXXXXXXXXXX
60: XX XX XX XX XX XX XX XX XX XX XX XX XX XX XX XX    XXXXXXXXXXXXXXXX
70: XX XX XX XX XX XX XX XX XX XX XX XX XX XX XX XX    XXXXXXXXXXXXXXXX
80: XX XX XX XX XX XX XX XX XX XX XX XX XX XX XX XX    XXXXXXXXXXXXXXXX
90: XX XX XX XX XX XX XX XX XX XX XX XX XX XX XX XX    XXXXXXXXXXXXXXXX
a0: XX XX XX XX XX XX XX XX XX XX XX XX XX XX XX XX    XXXXXXXXXXXXXXXX
b0: XX XX XX XX XX XX XX XX XX XX XX XX XX XX XX XX    XXXXXXXXXXXXXXXX
c0: XX XX XX XX XX XX XX XX XX XX XX XX XX XX XX XX    XXXXXXXXXXXXXXXX
d0: XX XX XX XX XX XX XX XX XX XX XX XX XX XX XX XX    XXXXXXXXXXXXXXXX
e0: XX XX XX XX XX XX XX XX XX XX XX XX XX XX XX XX    XXXXXXXXXXXXXXXX
f0: XX XX XX XX XX XX XX XX XX XX XX XX XX XX XX XX    XXXXXXXXXXXXXXXX

root@ubuntu:~# cat /sys/kernel/debug/dw_i2c0/fifodump
=b[0]-t[1229.799811]-n[2]-me[0]-ce[1]-ae[0x800008]=
m[0]-a[0x28]-f[0x0]:0x84
m[1]-a[0x28]-f[0x1]:0x00
=b[0]-t[1229.799881]-n[2]-me[0]-ce[1]-ae[0x800008]=
m[0]-a[0x28]-f[0x0]:0x85
m[1]-a[0x28]-f[0x1]:0x00
=b[0]-t[1229.799953]-n[2]-me[0]-ce[1]-ae[0x800008]=
m[0]-a[0x28]-f[0x0]:0x86
m[1]-a[0x28]-f[0x1]:0x00
=b[0]-t[1229.800046]-n[2]-me[0]-ce[1]-ae[0x800008]=
m[0]-a[0x28]-f[0x0]:0x87
m[1]-a[0x28]-f[0x1]:0x00
=b[0]-t[1229.800152]-n[2]-me[0]-ce[1]-ae[0x800008]=
m[0]-a[0x28]-f[0x0]:0x88
m[1]-a[0x28]-f[0x1]:0x00
=b[0]-t[1229.800233]-n[2]-me[0]-ce[1]-ae[0x800008]=
m[0]-a[0x28]-f[0x0]:0x89
m[1]-a[0x28]-f[0x1]:0x00
=b[0]-t[1229.800308]-n[2]-me[0]-ce[1]-ae[0x800008]=
m[0]-a[0x28]-f[0x0]:0x8a
m[1]-a[0x28]-f[0x1]:0x00
=b[0]-t[1229.800384]-n[2]-me[0]-ce[1]-ae[0x800008]=
m[0]-a[0x28]-f[0x0]:0x8b
m[1]-a[0x28]-f[0x1]:0x00

 b: bus number.
 t: timestamp.
 n: msg numbers.
 m: msg label.
 a: slave addr.
 f: flags, 0 is write, 1 is read.
```

Transmission abnormal:

``` {.text}
=b[1]-t[32991.050148]-n[2]-me[0]-ce[1]-ae[0x800001]=
m[0]-a[0x10]-f[0x0]:0xfa
m[1]-a[0x10]-f[0x1]:0x13
=b[1]-t[32991.050257]-n[2]-me[0]-ce[1]-ae[0x800001]=
m[0]-a[0x10]-f[0x0]:0xfb
m[1]-a[0x10]-f[0x1]:0x13

me: msg_err. Normal transmission is equal to "0", address/length mismatch will equal "-EINVAL".
ce: cmd_err. Normal transmission is equal to "0", the error was "DW_IC_ERR_TX_ABRT (0x1u)" transmit termination error.
ae: abort_source.
```

**Whitelist interface**

``` {.text}
root@ubuntu:~# echo 01 02 > /sys/kernel/debug/dw_i2c0/whitelist
root@ubuntu:~# cat /sys/kernel/debug/dw_i2c0/whitelist
whitelist: 01 02
```

1.  The whitelist can be configured using the echo command and displayed using the cat command.
2.  The whitelist supports address filtering for both reldump and fifodump.
3.  Input is interpreted as hexadecimal by default; invalid input or addresses exceeding 128 will trigger an error. Writing "0" disables the whitelist.

### i2c-tools

i2c-tools is an open-source utility suite. It has already been cross-compiled and included in the S100 system software's rootfs, allowing customers to use it directly:

-   i2cdetect --- enumerates I2C buses and all devices on each bus
-   i2cdump --- displays all register values of an I2C device
-   i2cget --- reads the value of a specific register from an I2C device
-   i2cset --- writes a value to a specific register of an I2C device

## I2C Testing

### Checking i2cdev nodes

Verify whether i2c-dev nodes have been created. All devices listed below are configured as master devices:

``` {.text}
root@ubuntu:~# ls /sys/class/i2c-dev/
i2c-0  i2c-1  i2c-2  i2c-3  i2c-4  i2c-5

root@ubuntu:~# cat /sys/class/i2c-dev/i2c-0/name
Synopsys DesignWare I2C adapter
```

Check whether I2C device nodes have been created:

``` {.text}
root@ubuntu:~# ls /sys/class/i2c-dev/i2c-0/device/
delete_device  device  i2c-dev  name  new_device  of_node  power  subsystem  uevent
```

### Test procedure

- Test command: i2cdetect -y -r 3
- Test example as follows:

``` {.text}
root@ubuntu:~# i2cdetect -r -y 3
     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
00:                         -- -- -- -- -- -- -- -- 
10: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
20: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
30: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
40: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
50: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
60: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
70: -- -- -- -- -- -- -- --  
```