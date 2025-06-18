---
sidebar_position: 2
---

# UART驱动调试指南

S100芯片共有4路uart，即uart0~uart3。其中uart0作为调试控制台使用，默认不开启DMA，且需要通过控制Bootstrip pin决定波特率为115200或921600;

其他几路uart用作数据传输功能，设备树中默认开启DMA，支持用户通过软件配置为各种波特率，常用波特率为921600。uart0和1支持硬件流控功能，其他几路uart不支持该功能。


## UART使用说明

### 代码路径

```shell
drivers/tty/serial/8250/8250_dw.c   #uart驱动文件
drivers/tty/serial/8250/8250_port.c    #uart端口操作文件
drivers/tty/serial/8250/8250_core.c    #8250 uart驱动核心
hobot-drivers/serial/8250_pdma.c    #uart PDMA操作实现文件
```

### 内核配置

``` {.text}
/*hobot-drivers/configs/drobot_s100_defconfig*/
CONFIG_SERIAL_8250=y   # 8250驱动配置
CONFIG_SERIAL_8250_CONSOLE=y   #8250 console驱动配置
CONFIG_SERIAL_8250_DW=y   #使能Designware独有的feature
```

### DTS设备节点配置


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

## 使用示例

### 检查uart节点

``` {.text}
ls /dev/ttyS*
/dev/ttyS0  /dev/ttyS1  /dev/ttyS2  /dev/ttyS3
```

### 串口相关操作

-   查看串口波特率等配置：

    ``` {.text}
    stty -F /dev/ttyS1 -a
    ```

-   配置波特率等：

    ``` {.text}
    stty -F /dev/ttyS1 speed 921600 cs8 -cstopb parenb -parodd
    ```

-   读取串口数据：

    ``` {.text}
    cat /dev/ttyS1
    ```

-   向串口输出数据测试：

    ``` {.text}
    echo 123456789 > /dev/ttyS1
    ```

## 注意事项

- RDK S100硬件设计上40PIN GPIO使用了TI的TXS系列的电平转换芯片将1.8V IO转成3.3V IO，为了信号的质量和可靠性，通信对端尽量不要使用电平转换芯片对其再次进行转换，若使用多级请关注实际硬件信号质量。
