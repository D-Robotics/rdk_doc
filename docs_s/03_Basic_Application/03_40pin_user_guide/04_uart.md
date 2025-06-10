---
sidebar_position: 4
---

# 3.3.4 串口应用

RDK S100 在 40PIN 支持 UART2，没有使能，物理管脚号 8 和 10，IO 电压 3.3V。

:::info

40pin 上需要波动拨码开关来选择使用 UART2 还是 I2C5, 具体细节可以查看下图：

![image-rdk_100_funcreuse_40pin](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/hardware_interface/image-rdk_100_funcreuse_40pin.png)

波动拨码开关之后还需要修改设备树文件，修改路径及方式如下：

```{.text}
/*kernel/arch/arm64/boot/dts/hobot/drobot-s100-soc.dtsi*/
uart2: uart@394C0000 {
        power-domains = <&scmi_smc_pd PD_IDX_LSPERI_TOP>;
        compatible = "snps,dw-apb-uart";
        reg = <0x0 0x394C0000 0x0 0x10000>;
        reg-shift = <2>;
        reg-io-width = <4>;
        interrupts = <GIC_SPI PERISYS_UART2_INTR PERISYS_UART2_INTR_TRIG_TYPE>;
        clock-frequency = <200000000>;
        pinctrl-names = "default";
        pinctrl-0 = <&peri_uart2>;
        status = "okay";
};
```

管脚定义请参考 [管脚配置与定义](./01_40pin_define.md#40pin_define)

:::

请参阅 `/app/40pin_samples/test_serial.py`了解如何使用串口的详细信息。

:::tip
以下所提及的管脚仅作示例说明，不同平台的端口值存在差异，实际情况应以实际为准。亦可直接使用`/app/40pin_samples/`目录下的代码，该代码已在板子上经过实际验证。
:::

## 回环测试

把 TXD 和 RXD 在硬件上进行连接，然后运行测试程序，进行写和读操作，预期结果是读出的数据要完全等于写入的数据

### 硬件连接

把 TXD 和 RXD 通过跳线帽直接硬件上连接在一起：

![image-20220512101820743](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_40pin_user_guide/image/40pin_user_guide/image-rdk_s100_uart.png)

### 测试过程

- 运行 `python3 /app/40pin_samples/test_serial.py`
- 从打印的串口设备（其中 /dev/ttyS0 是系统调试口，不建议对它进行测试，除非你完全明白它的作用）中选择总线号和片选号作为输入选项，例如 RDK X3 选择测试 `/dev/ttyS3`，RDK X5 选择测试 `/dev/ttyS1`，RDK Ultra 选择测试 `/dev/ttyS2` ， RDK s100 选择测试 `/dev/ttyS2`按回车键确认，并输入波特率参数：

```
root@ubuntu:/app/40pin_samples# ./test_serial.py
List of enabled UART:
/dev/ttyS0  /dev/ttyS1  /dev/ttyS2  /dev/ttyS3

请输出需要测试的串口设备名:/dev/ttyS2
请输入波特率(9600,19200,38400,57600,115200,921600):921600
Serial<id=0x7f819dcac0, open=True>(port='/dev/ttyS3', baudrate=921600, bytesize=8, parity='N', stopbits=1, timeout=1, xonxoff=False, rtscts=False, dsrdtr=False)
```

- 程序正确运行起来后会持续打印 `Send: AA55` 和 `Recv:  AA55`：

```
Starting demo now! Press CTRL+C to exit
Send:  AA55
Recv:  AA55
```

## 测试代码

```python
#!/usr/bin/env python3

import sys
import signal
import os
import time

# 导入python串口库
import serial
import serial.tools.list_ports

def signal_handler(signal, frame):
    sys.exit(0)

def serialTest():
    print("List of enabled UART:")
    os.system('ls /dev/tty[a-zA-Z]*')
    uart_dev= input("请输出需要测试的串口设备名:")

    baudrate = input("请输入波特率(9600,19200,38400,57600,115200,921600):")
    try:
        ser = serial.Serial(uart_dev, int(baudrate), timeout=1) # 1s timeout
    except Exception as e:
        print("open serial failed!\n")

    print(ser)

    print("Starting demo now! Press CTRL+C to exit")

    while True:
        test_data = "AA55"
        write_num = ser.write(test_data.encode('UTF-8'))
        print("Send: ", test_data)

        received_data = ser.read(write_num).decode('UTF-8')
        print("Recv: ", received_data)

        time.sleep(1)

    ser.close()
    return 0


if __name__ == '__main__':
    signal.signal(signal.SIGINT, signal_handler)
    if serialTest() != 0:
        print("Serial test failed!")
    else:
        print("Serial test success!")

```
