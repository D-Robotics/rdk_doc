---
sidebar_position: 2
---

# UART Driver Debugging Guide

The S100 chip has a total of four UART interfaces, namely UART0 to UART3. Among them, UART0 is used as the debug console. By default, DMA is disabled for UART0, and its baud rate (either 115200 or 921600) is determined by the Bootstrip pin configuration.

The other UART interfaces are used for data transmission. DMA is enabled by default in the device tree, and users can configure various baud rates via software. The commonly used baud rate is 921600. UART0 and UART1 support hardware flow control, while the other UART interfaces do not support this feature.


## UART Usage Instructions

### Code Paths

```shell
drivers/tty/serial/8250/8250_dw.c   # UART driver file
drivers/tty/serial/8250/8250_port.c    # UART port operation file
drivers/tty/serial/8250/8250_core.c    # Core 8250 UART driver
hobot-drivers/serial/8250_pdma.c    # UART PDMA operation implementation file
```

### Kernel Configuration

``` {.text}
/*hobot-drivers/configs/drobot_s100_defconfig*/
CONFIG_SERIAL_8250=y   # 8250 driver configuration
CONFIG_SERIAL_8250_CONSOLE=y   # 8250 console driver configuration
CONFIG_SERIAL_8250_DW=y   # Enable Designware-specific features
```

### DTS Device Node Configuration

``` {.text}
/*kernel/arch/arm64/boot/dts/hobot/drobot-s100-soc.dtsi*/
uart1: uart@394A0000 {
      power-domains = <&scmi_smc_pd PD_IDX_LSPERI_TOP>;
      compatible = "snps,dw-apb-uart";
      reg = <0x0 0x394A0000 0x0 0x10000>;
      reg-shift = <2>;
      reg-io-width = <4>;
      interrupts = <GIC_SPI PERISYS_UART0_INTR PERISYS_UART0_INTR_TRIG_TYPE>;
      clock-frequency = <200000000>;
      pinctrl-names = "default";
      pinctrl-0 = <&peri_uart0>;
      dmas = <&pdma0 0>, <&pdma0 1>;
      dma-names = "rx", "tx";
      status = "okay";
 };
```

## Usage Examples

### Check UART Nodes

``` {.text}
ls /dev/ttyS*
/dev/ttyS0  /dev/ttyS1  /dev/ttyS2  /dev/ttyS3
```

### UART-related Operations

-   View UART configuration such as baud rate:

    ``` {.text}
    stty -F /dev/ttyS1 -a
    ```

-   Configure baud rate, etc.:

    ``` {.text}
    stty -F /dev/ttyS1 speed 921600 cs8 -cstopb parenb -parodd
    ```

-   Read data from UART:

    ``` {.text}
    cat /dev/ttyS1
    ```

-   Send test data via UART:

    ``` {.text}
    echo 123456789 > /dev/ttyS1
    ```

## Notes

- In the RDK S100 hardware design, the 40-pin GPIO uses TI's TXS-series level-shifting ICs to convert 1.8V I/O signals to 3.3V I/O signals. To ensure signal quality and reliability, avoid using additional level-shifting ICs on the communication peer side. If multiple stages of level shifting are used, pay close attention to the actual hardware signal integrity.