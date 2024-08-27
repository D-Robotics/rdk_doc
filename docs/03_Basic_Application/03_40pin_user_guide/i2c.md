---
sidebar_position: 5
---

# 3.3.5 I2C应用

RDK X3 在 40PIN 上默认使能 I2C0，物理管脚号 3 和 5，IO电压3.3V。

RDK X5 在 40PIN 上默认使能 I2C5（物理管脚号 3 和 5）和 I2C0（物理管脚号 27 和 28），IO电压3.3V。

RDX Ultra 在 40PIN 上默认使能 I2C4（物理管脚号 3 和 5） 和 I2C6（物理管脚号 27 和 28），IO电压3.3V。

请参阅 `/app/40pin_samples/test_i2c.py`了解如何使用I2C的详细信息。

## 测试方法

- 运行测试程序 `python3 /app/40pin_samples/test_i2c.py`

- 首先列出当前系统使能的 I2C 总线
- 通过输入总线号扫描得到当前总线上连接了哪些外设
- 输入外设地址（16进制数），测试程序会从该外设上读取一个字节的数据

## 运行效果

```bash
Starting demo now! Press CTRL+C to exit
List of enabled I2C controllers:
/dev/i2c-0  /dev/i2c-1
Please input I2C BUS num:1
     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
00:          -- -- -- -- -- -- -- -- -- -- -- -- --
10: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
20: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
30: -- -- -- -- -- -- -- -- -- -- -- UU -- -- -- --
40: 40 -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
50: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
60: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
70: -- -- -- -- -- -- -- --
Please input I2C device num(Hex):40
Read data from device 40 on I2C bus 1
read value= b'`'
```

## 测试代码

```python
#!/usr/bin/env python3

import sys
import signal
import os
import time

# 导入i2cdev
from i2cdev import I2C

def signal_handler(signal, frame):
    sys.exit(0)

def i2cdevTest():
    # device, bus = 0x51, 0
    bus = input("Please input I2C BUS num:")
    os.system('i2cdetect -y -r ' + bus)
    device = input("Please input I2C device num(Hex):")
    print("Read data from device %s on I2C bus %s" % (device, bus))
    i2c = I2C(eval("0x" + device), int(bus))
    value = i2c.read(1)
    i2c.write(value)
    print("read value=", value)
    i2c.close()

if __name__ == '__main__':
    signal.signal(signal.SIGINT, signal_handler)
    print("Starting demo now! Press CTRL+C to exit")
    print("List of enabled I2C controllers:")
    os.system('ls /dev/i2c*')
    while True:
        i2cdevTest()
```
