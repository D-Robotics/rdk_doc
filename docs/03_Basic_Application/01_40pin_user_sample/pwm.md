---
sidebar_position: 3
---

# 3.1.3 PWM应用

Hobot.GPIO 库仅在带有附加硬件 PWM 控制器的引脚上支持 PWM。与 RPi.GPIO 库不同，Hobot.GPIO 库不实现软件模拟 PWM。

RDK X3 和 RDK Ultra 都支持 2 个 PWM 通道，对应40pin引脚为33 32。

RDK X5 支持 4组PWM，每组2路输出，共8个PWM输出, 如下表：

| PMW组 | PWM 通道 | 40PIN 引脚 |
| --- | ---- | ---- |
| PWM0 | LSIO_PWM_OUT0 | 29 |
| PWM0 | LSIO_PWM_OUT1 | 31 |
| PWM1 | LSIO_PWM_OUT2 | 37 |
| PWM1 | LSIO_PWM_OUT3 | 24 |
| PWM2 | LSIO_PWM_OUT4 | 28 |
| PWM2 | LSIO_PWM_OUT5 | 27 |
| PWM3 | LSIO_PWM_OUT6 | 32 |
| PWM3 | LSIO_PWM_OUT7 | 33 |

RDK X5 默认使能PWM3，可以通过`srpi-config`系统配置工具，使能其他的PWM组，这会将对应的引脚复用为PWM输出，重启后生效。

选择`3 Interface Options` -> `I3 Peripheral bus config`, 然后选择对应的`pwm`组,切换到`okay`。

请参阅 `/app/40pin_samples/simple_pwm.py`了解如何使用 PWM 通道的详细信息。

### 测试代码
打开 `output_pin` 指定的PWM通道，初始占空比 25%， 先每0.25秒增加5%占空比，达到100%之后再每0.25秒减少5%占空比，在正常输出波形时，可以通过示波器或者逻辑分析仪测量输出信号，观察波形。

```python
#!/usr/bin/env python3
import sys
import signal
import Hobot.GPIO as GPIO
import time

def signal_handler(signal, frame):
    sys.exit(0)

output_pin = 33

GPIO.setwarnings(False)

def main():
    # Pin Setup:
    # Board pin-numbering scheme
    GPIO.setmode(GPIO.BOARD)
    # 支持的频率范围： X3: 48KHz ~ 192MHz X5: 0.05HZ ~ 100MHZ
    p = GPIO.PWM(output_pin, 48000)
    # 初始占空比 25%， 先每0.25秒增加5%占空比，达到100%之后再每0.25秒减少5%占空比
    val = 25
    incr = 5
    p.ChangeDutyCycle(val)
    p.start(val)

    print("PWM running. Press CTRL+C to exit.")
    try:
        while True:
            time.sleep(0.25)
            if val >= 100:
                incr = -incr
            if val <= 0:
                incr = -incr
            val += incr
            p.ChangeDutyCycle(val)
    finally:
        p.stop()
        GPIO.cleanup()

if __name__ == '__main__':
    signal.signal(signal.SIGINT, signal_handler)
    main()

```
