---
sidebar_position: 6
---

# 3.3.6 SPI 应用

RDK S100 在 40PIN 上物理管脚 `19, 21, 23, 24, 26`引出了的 `SPI0` 总线，支持两个片选，IO 电压 3.3V；

请参阅 `/app/40pin_samples/test_spi.py`了解如何使用 SPI 的详细信息。

:::tip
以下所提及的管脚仅作示例说明，不同平台的端口值存在差异，实际情况应以实际为准。亦可直接使用`/app/40pin_samples/`目录下的代码，该代码已在板子上经过实际验证。
:::

## 回环测试

把 MISO 和 MOSI 在硬件上进行连接，然后运行 SPI 测试程序，进行写和读操作，预期结果是读出的数据要完全等于写入的数据

### 硬件连接

把 MISO 和 MOSI 通过跳线帽直接硬件上连接在一起：

![image-20220512101915524](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_40pin_user_guide/image/40pin_user_guide/image-rdk_s100_spi.png)

### 测试过程

- 运行 `python3 /app/40pin_samples/test_spi.py`
- 从打印的 spi 控制器中选择总线号和片选号作为输入选项，例如选择测试 `spidev0.0`，则`bus num` 和 `cs num`都是`0`，按回车键确认：

```
List of enabled spi controllers:
/dev/spidev0.0  /dev/spidev0.1
Please input SPI bus num:0
Please input SPI cs num:0
```

- 程序正确运行起来后会持续打印 `0x55 0xAA`，如果打印的是 `0x00 0x00`，那么就说明 spi 的回环测试失败。

```
Starting demo now! Press CTRL+C to exit
0x55 0xAA
0x55 0xAA
```

## 测试代码

```python
#!/usr/bin/env python3

import sys
import signal
import os
import time

# 导入spidev模块
import spidev

def signal_handler(signal, frame):
    sys.exit(0)

def BytesToHex(Bytes):
    return ''.join(["0x%02X " % x for x in Bytes]).strip()

def spidevTest():
    # 设置spi的bus号（0, 1, 2）和片选(0, 1)
    spi_bus = input("Please input SPI bus num:")
    spi_device = input("Please input SPI cs num:")
    # 创建spidev类的对象以访问基于spidev的Python函数。
    spi=spidev.SpiDev()
    # 打开spi总线句柄
    spi.open(int(spi_bus), int(spi_device))

    # 设置 spi 频率为 12MHz
    spi.max_speed_hz = 12000000

    print("Starting demo now! Press CTRL+C to exit")

    # 发送 [0x55, 0xAA], 接收的数据应该也是 [0x55, 0xAA]
    try:
        while True:
            resp = spi.xfer2([0x55, 0xAA])
            print(BytesToHex(resp))
            time.sleep(1)

    except KeyboardInterrupt:
        spi.close()

if __name__ == '__main__':
    signal.signal(signal.SIGINT, signal_handler)
    print("List of enabled spi controllers:")
    os.system('ls /dev/spidev*')

    spidevTest()

```
