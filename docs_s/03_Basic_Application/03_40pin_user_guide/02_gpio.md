---
sidebar_position: 2
---

# 3.3.2 GPIO应用

开发板预置了 GPIO Python 库 `Hobot.GPIO`，用户可以通过如下命令导入GPIO库。

```shell
root@ubuntu:~# sudo python3
Python 3.10.12 (main, Feb  4 2025, 14:57:36) [GCC 11.4.0] on linux
Type "help", "copyright", "credits" or "license" for more information.
>>> import Hobot.GPIO as GPIO
>>> GPIO.VERSION
'0.0.2'
>>> GPIO.model
'RDK_S100'
```

:::tip
以下所提及的管脚仅作示例说明，不同平台的端口值存在差异，实际情况应以实际为准。亦可直接使用`/app/40pin_samples/`目录下的代码，该代码已在板子上经过实际验证。
:::


## 设置引脚编码方式

开发板的引脚编码有 4 种模式：

- BOARD：物理引脚序号，与开发板的丝印序号一一对应。
- BCM：根据博通SoC制定的GPIO命名规则。
- CVM： 使用字符串代替数字，对应于CVM / CVB连接器的信号名称。
- SOC： 对应的编号是芯片内部的 GPIO 管脚序号。

本文推荐用户使用`BOARD`编码模式，设置编码的方式如下：
注意：编码每次只能设置一次，如果想要重新设置，需要`GPIO.cleanup()`后重新设置
```python
GPIO.setmode(GPIO.BOARD)
# or
GPIO.setmode(GPIO.BCM)
# or
GPIO.setmode(GPIO.CVM)
# or
GPIO.setmode(GPIO.SOC)
```

查询编码方式：

```python
GPIO.getmode()
```

程序会输出 `BOARD, BCM, CVM, SOC or None` 其中的一种结果。

## 警告信息

以下几种情况下运行代码，会有警告日志输出，但并不会影响正常功能：

 - 用户尝试使用的GPIO，已在其他应用程序中使用；
 - 在设置模式和通道之前，尝试调用 `GPIO.cleanup` 清理管脚；

如要屏蔽警告信息，可通过如下命令实现：

```python
GPIO.setwarnings(False)
```

## 管脚配置

:::info

在`RDK S100`平台上，支持`40-pin` GPIO拓展， `40-pin`在使用过程中有如下的限制:

- `40-pin`上有一组引脚涉及到二选一（UART2, I2C5）。
- `40-pin`上PCM相关引脚如果要使用需要波动拨码开关。

上述描述细节可以查看下图：

![image-rdk_100_funcreuse_40pin](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/hardware_interface/image-rdk_100_funcreuse_40pin.png)

管脚定义请参考 [管脚配置与定义](./01_40pin_define.md#40pin_define)

:::

GPIO管脚在使用之前，需要进行相应的配置，具体如下：

设置为输入的方式如下：
```python
GPIO.setup(channel, GPIO.IN)
```

设置为输出的方式如下：

```python
GPIO.setup(channel, GPIO.OUT)
```

也可以为输出通道指定一个初始值，例如：

```python
GPIO.setup(channel, GPIO.OUT, initial=GPIO.HIGH)
```

另外，工具支持同时设置多个输出通道，例如：

```python
# set gpio(18,12,13) to output
channels = [18, 12, 13]
GPIO.setup(channels, GPIO.OUT)
```

## 输入操作

要读取通道的值，请使用：

```python
GPIO.input(channel)
```

命令返回值为 0 或者 1。 0 代表 GPIO.LOW， 1 代表 GPIO.HIGH。

## 输出操作

要设置通道的输出值，请使用：

```python
GPIO.output(channel, state)
```

其中 state 可以是 GPIO.LOW 或 GPIO.HIGH。

## 清理管脚占用

在程序推出前，推荐进行通道清理动作，请使用：

```python
GPIO.cleanup()
```

如果只想清理特定通道，请使用：

```python
# 清除单个通道
GPIO.cleanup(channel)
# 清除一组通道
GPIO.cleanup( (channel1, channel2) )
GPIO.cleanup( [channel1, channel2] )
```

## 查看管脚状态

此功能允许您检查对应 GPIO 通道的功能：

```python
GPIO.gpio_function(channel)
```

该函数返回 IN 或 OUT。

## 边沿检测与中断

边沿是电信号`从低到高`（上升沿）或`从高到低`（下降沿）的变化，这种改变可以看作是一种事件的发生，这种事件可以用来触发CPU中断信号。

:::info

在`RDK S100`平台上，`40-pin`上功能名为PERI_GPIO的管脚不支持中断使用，它们在`BOARD`编码模式下的编号为：**11**、**13**、**15**、**16**、**18**、**22**、**29**、**31**、**36**、**37**；

管脚定义请参考 [管脚配置与定义](./01_40pin_define.md#40pin_define)

:::

GPIO库提供了三种方法来检测输入事件：

### wait_for_edge() 函数

此函数阻塞调用线程，直到检测到对应的边缘变化。函数调用如下：

```python
GPIO.wait_for_edge(channel, GPIO.RISING)
```

其中，第二个参数指定要检测的边沿，取值范围为`GPIO.RISING、GPIO.FALLING 或 GPIO.BOTH`。如果要指定等待时间，可以选择设置超时：

```python
# 超时以毫秒为单位
GPIO.wait_for_edge(channel, GPIO.RISING, timeout=500)
```

在超时时间内外部信号发生变化，函数返回检测的通道号；如果发生超时，函数返回None。

### event_detected() 函数

此函数可用于定期检查自上次调用以来是否发生了事件。该函数可以按如下方式设置和调用：

```python
# 在通道GPIO上设置上升沿检测
GPIO.add_event_detect(channel, GPIO.RISING)
if GPIO.event_detected(channel):
    print("Rising edge event detected")
```

您可以检测 GPIO.RISING、GPIO.FALLING 或 GPIO.BOTH 的事件。

### 检测到边沿事件时运行回调函数

此功能可用于注册回调函数，回调函数运行在独立的处理线程中，使用说明如下：

```python
# define callback function
def callback_fn(channel):
    print("Callback called from channel %s" % channel)

# enable rising detection
GPIO.add_event_detect(channel, GPIO.RISING, callback=callback_fn)
```

如有需要，也可以添加多个回调，方法如下：

```python
def callback_one(channel):
    print("First Callback")

def callback_two(channel):
    print("Second Callback")

GPIO.add_event_detect(channel, GPIO.RISING)
GPIO.add_event_callback(channel, callback_one)
GPIO.add_event_callback(channel, callback_two)
```

由于所有回调函数运行在同一线程上，因此不同的回调是按顺序运行的，而不是同时运行。

为了通过将多个事件合并为一个事件来防止多次调用回调函数，可以选择设置去抖动时间：

```python
# bouncetime unit is ms
GPIO.add_event_detect(channel, GPIO.RISING, callback=callback_fn, bouncetime=200)
```

### 关闭中断

如果不再需要边沿检测，可以将其删除，如下所示：

```python
GPIO.remove_event_detect(channel)
```

## 测试例程

在 `/app/40pin_samples/`目录下提供主要的测试例程：

| 测试例程名             | 说明                                          |
| ---------------------- | --------------------------------------------- |
| simple_out.py          | 单个管脚`输出`测试                            |
| simple_input.py        | 单个管脚`输入`测试                            |
| button_led.py          | 一个管脚作为按键输入，一个管脚作为输出控制LED |
| button_event.py        | 捕获管脚的上升沿、下降沿事件                  |
| button_interrupt.py    | 中断方式处理管脚的上升沿、下降沿事件          |

- GPIO 设置为`输出模式`，1秒钟切换输出电平，可以用于控制LED灯的循环亮灭， 测试代码 `simple_out.py`：

```python
#!/usr/bin/env python3
import sys
import signal
import Hobot.GPIO as GPIO
import time


def signal_handler(signal, frame):
    sys.exit(0)


# 定义使用的GPIO通道为output_pin


def determine_pins():
    board_id = GPIO.gpio_pin_data.parse_boardid()
    if GPIO.gpio_pin_data.if_s100_40pin(board_id):
        return 37
    else:
        return 26


def main():
    output_pin = determine_pins()
    # 设置管脚编码模式为硬件编号 BOARD
    GPIO.setmode(GPIO.BOARD)
    # 设置为输出模式，并且初始化为高电平
    GPIO.setup(output_pin, GPIO.OUT, initial=GPIO.HIGH)
    # 记录当前管脚状态
    curr_value = GPIO.HIGH
    print("Starting demo now! Press CTRL+C to exit")
    try:
        # 间隔1秒时间，循环控制LED灯亮灭
        while True:
            time.sleep(1)
            GPIO.output(output_pin, curr_value)
            curr_value ^= GPIO.HIGH
    finally:
        GPIO.cleanup()


if __name__ == '__main__':
    signal.signal(signal.SIGINT, signal_handler)
    main()
```

- GPIO 设置为`输入模式`，通过忙轮询方式读取管脚电平值，测试代码 `simple_input.py`：

```python
#!/usr/bin/env python3
import sys
import signal
import Hobot.GPIO as GPIO
import time


def signal_handler(signal, frame):
    sys.exit(0)


# 定义使用的GPIO通道为input_pin


GPIO.setwarnings(False)


def determine_pins():
    board_id = GPIO.gpio_pin_data.parse_boardid()
    if GPIO.gpio_pin_data.if_s100_40pin(board_id):
        return 37
    else:
        return 26


def main():
    prev_value = None
    input_pin = determine_pins()
    # 设置管脚编码模式为硬件编号 BOARD
    GPIO.setmode(GPIO.BOARD)
    # 设置为输入模式
    GPIO.setup(input_pin, GPIO.IN)

    print("Starting demo now! Press CTRL+C to exit")
    try:
        while True:
            # 读取管脚电平
            value = GPIO.input(input_pin)
            if value != prev_value:
                if value == GPIO.HIGH:
                    value_str = "HIGH"
                else:
                    value_str = "LOW"
                print("Value read from pin {} : {}".format(input_pin, value_str))
                prev_value = value
            time.sleep(1)
    finally:
        GPIO.cleanup()


if __name__=='__main__':
    signal.signal(signal.SIGINT, signal_handler)
    main()

```

- GPIO 设置为输入模式，捕获管脚的上升沿、下降沿事件，测试代码 `button_event.py`, 实现检测24号管脚的下降沿，然后控制23号管脚的输出：

```python
#!/usr/bin/env python3
import sys
import signal
import Hobot.GPIO as GPIO
import time


def signal_handler(signal, frame):
    sys.exit(0)


# 定义使用的GPIO通道：
# led_pin作为输出，可以点亮一个LED
# but_pin作为输入，可以接一个按钮
BOARD_ID_PATH = "/sys/class/boardinfo/adc_boardid"


# 禁用警告信息
GPIO.setwarnings(False)


def determine_pins():
    board_id = GPIO.gpio_pin_data.parse_boardid()
    if GPIO.gpio_pin_data.if_s100_40pin(board_id):
        return 23, 24
    else:
        return 26, 27


def main():

    led_pin, but_pin = determine_pins()
    # 设置管脚编码模式为硬件编号 BOARD
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(led_pin, GPIO.OUT)  # LED pin set as output
    GPIO.setup(but_pin, GPIO.IN)  # button pin set as input

    # Initial state for LEDs:
    GPIO.output(led_pin, GPIO.LOW)

    print("Starting demo now! Press CTRL+C to exit")
    try:
        while True:
            print("Waiting for button event")
            GPIO.wait_for_edge(but_pin, GPIO.FALLING)

            # event received when button pressed
            print("Button Pressed!")
            GPIO.output(led_pin, GPIO.HIGH)
            time.sleep(1)
            GPIO.output(led_pin, GPIO.LOW)
    finally:
        GPIO.cleanup()  # cleanup all GPIOs


if __name__ == '__main__':
    signal.signal(signal.SIGINT, signal_handler)
    main()

```

- GPIO 设置为输入模式，启动gpio中断功能，响应管脚的上升沿、下降沿事件，测试代码 `button_interrupt.py`, 实现检测 24 号管脚的下降沿，然后控制15号管脚快速切换高低电平 5 次：

```python
#!/usr/bin/env python3
import sys
import signal
import Hobot.GPIO as GPIO
import time


def signal_handler(signal, frame):
    sys.exit(0)


# 定义使用的GPIO通道：
# 15号作为输出，可以点亮一个LED
# 16号作为输出，可以点亮一个LED
# but_pin作为输入，可以接一个按钮
led_pin_1 = 15  # BOARD 编码 15
led_pin_2 = 16  # BOARD 编码 16

# 禁用警告信息
GPIO.setwarnings(False)


def determine_pins():
    board_id = GPIO.gpio_pin_data.parse_boardid()
    if GPIO.gpio_pin_data.if_s100_40pin(board_id):
        return 24
    else:
        return 27


# 按下按钮时 LED 2 快速闪烁 5 次
def blink(channel):
    print("Blink LED 2")
    for i in range(5):
        GPIO.output(led_pin_2, GPIO.HIGH)
        time.sleep(0.5)
        GPIO.output(led_pin_2, GPIO.LOW)
        time.sleep(0.5)


def main():
    but_pin = determine_pins()
    # Pin Setup:
    GPIO.setmode(GPIO.BOARD)  # BOARD pin-numbering scheme
    GPIO.setup([led_pin_1, led_pin_2], GPIO.OUT)  # LED pins set as output
    GPIO.setup(but_pin, GPIO.IN)  # button pin set as input

    # Initial state for LEDs:
    GPIO.output(led_pin_1, GPIO.LOW)
    GPIO.output(led_pin_2, GPIO.LOW)

    # 把blink函数注册为按钮下降沿事件的中断处理函数
    GPIO.add_event_detect(but_pin, GPIO.FALLING, callback=blink, bouncetime=10)
    # 开始测试， Led1 缓慢闪烁
    print("Starting demo now! Press CTRL+C to exit")
    try:
        while True:
            # blink LED 1 slowly
            GPIO.output(led_pin_1, GPIO.HIGH)
            time.sleep(2)
            GPIO.output(led_pin_1, GPIO.LOW)
            time.sleep(2)
    finally:
        GPIO.cleanup()  # cleanup all GPIOs


if __name__ == '__main__':
    signal.signal(signal.SIGINT, signal_handler)
    main()

```
## hb_gpioinfo工具介绍

hb_gpioinfo 是适配RDK S100的一个gpio帮助工具，可以查看当前开发板的的PinName和PinNum的对应关系，命令示例输出如下：
```shell
sunrise@ubuntu:/root$ sudo hb_gpioinfo
|--- ---------------- --------------------|
|Idx|        Pin Name|            Pin Func|
|--- ---------------- --------------------|
|439|        GNSS_INT|                gpio|
|--- ---------------- --------------------|
|440|       PERI_RSTO|      Not Configured|
|--- ---------------- --------------------|
|441|        CAM_PINT|      Not Configured|
|--- ---------------- --------------------|
|442|          SD_1V8|        video_sd_1v8|
|--- ---------------- --------------------|
|443|      SD_BUS_POW|      Not Configured|
|--- ---------------- --------------------|
|444|     SENSOR0_ERR|      Not Configured|
|--- ---------------- --------------------|
|445|     SENSOR1_ERR|      Not Configured|
|--- ---------------- --------------------|
|446|     SENSOR2_ERR|      Not Configured|
|--- ---------------- --------------------|
|447|     SENSOR3_ERR|      Not Configured|
|--- ---------------- --------------------|
|448|     SENSOR4_ERR|      Not Configured|
|--- ---------------- --------------------|
|449|     SENSOR5_ERR|      Not Configured|
|--- ---------------- --------------------|
|450|     SENSOR6_ERR|      Not Configured|
|--- ---------------- --------------------|
|451|     SENSOR7_ERR|      Not Configured|
|--- ---------------- --------------------|
|452|     SENSOR8_ERR|      Not Configured|
|--- ---------------- --------------------|
|453|     SENSOR9_ERR|                gpio|
|--- ---------------- --------------------|
|454|    SENSOR10_ERR|      Not Configured|
|--- ---------------- --------------------|
|455|    SENSOR11_ERR|      Not Configured|
|--- ---------------- --------------------|
|456|     LPWM0_DOUT0|     cam_lpwm0_dout0|
|--- ---------------- --------------------|
|457|     LPWM0_DOUT1|     cam_lpwm0_dout1|
|--- ---------------- --------------------|
|458|     LPWM0_DOUT2|     cam_lpwm0_dout2|
|--- ---------------- --------------------|
|459|     LPWM0_DOUT3|     cam_lpwm0_dout3|
|--- ---------------- --------------------|
|460|     LPWM1_DOUT0|     cam_lpwm1_dout0|
|--- ---------------- --------------------|
|461|     LPWM1_DOUT1|     cam_lpwm1_dout1|
|--- ---------------- --------------------|
|462|     LPWM1_DOUT2|     cam_lpwm1_dout2|
|--- ---------------- --------------------|
|463|     LPWM1_DOUT3|     cam_lpwm1_dout3|
|--- ---------------- --------------------|
|464|        I2C0_SCL|        cam_i2c0_scl|
|--- ---------------- --------------------|
|465|        I2C0_SDA|        cam_i2c0_sda|
|--- ---------------- --------------------|
|466|        I2C1_SCL|        cam_i2c1_scl|
|--- ---------------- --------------------|
|467|        I2C1_SDA|        cam_i2c1_sda|
|--- ---------------- --------------------|
|468|        I2C2_SCL|        cam_i2c2_scl|
|--- ---------------- --------------------|
|469|        I2C2_SDA|        cam_i2c2_sda|
|--- ---------------- --------------------|
|470|        I2C3_SCL|        cam_i2c3_scl|
|--- ---------------- --------------------|
|471|        I2C3_SDA|        cam_i2c3_sda|
|--- ---------------- --------------------|
|472|        I2C4_SCL|        cam_i2c4_scl|
|--- ---------------- --------------------|
|473|        I2C4_SDA|        cam_i2c4_sda|
|--- ---------------- --------------------|
|474|       UART1_RXD|      peri_uart1_rxd|
|--- ---------------- --------------------|
|475|       PCM0_MCLK|      Not Configured|
|--- ---------------- --------------------|
|476|       PCM0_BCLK|      Not Configured|
|--- ---------------- --------------------|
|477|      PCM0_FSYNC|      Not Configured|
|--- ---------------- --------------------|
|478|      PCM0_DATA0|      Not Configured|
|--- ---------------- --------------------|
|479|      PCM0_DATA1|      Not Configured|
|--- ---------------- --------------------|
|480|   EMAC_MDC_HSI0|  peri_emac_mdc_hsi0|
|--- ---------------- --------------------|
|481|  EMAC_MDIO_HSI0| peri_emac_mdio_hsi0|
|--- ---------------- --------------------|
|482|          SD_CLK|      Not Configured|
|--- ---------------- --------------------|
|483|          SD_CMD|      Not Configured|
|--- ---------------- --------------------|
|484|        SD_DATA0|      Not Configured|
|--- ---------------- --------------------|
|485|        SD_DATA1|      Not Configured|
|--- ---------------- --------------------|
|486|        SD_DATA2|      Not Configured|
|--- ---------------- --------------------|
|487|        SD_DATA3|      Not Configured|
|--- ---------------- --------------------|
|488|        SD_DATA4|      Not Configured|
|--- ---------------- --------------------|
|489|        SD_DATA5|      Not Configured|
|--- ---------------- --------------------|
|490|        SD_DATA6|      Not Configured|
|--- ---------------- --------------------|
|491|        SD_DATA7|      Not Configured|
|--- ---------------- --------------------|
|492|    SD_DATA_STRB|      Not Configured|
|--- ---------------- --------------------|
|493|        SD_DET_N|      Not Configured|
|--- ---------------- --------------------|
|494|        SD_WPROT|      Not Configured|
|--- ---------------- --------------------|
|495|        I2C5_SCL|       peri_i2c5_scl|
|--- ---------------- --------------------|
|496|        I2C5_SDA|       peri_i2c5_sda|
|--- ---------------- --------------------|
|497|       SPI0_CSN0|      peri_spi0_csn0|
|--- ---------------- --------------------|
|498|       SPI0_CSN1|      peri_spi0_csn1|
|--- ---------------- --------------------|
|499|       SPI0_MOSI|      peri_spi0_mosi|
|--- ---------------- --------------------|
|500|       SPI0_MISO|      peri_spi0_miso|
|--- ---------------- --------------------|
|501|       SPI0_SCLK|      peri_spi0_sclk|
|--- ---------------- --------------------|
|502|       SPI1_CSN0|      peri_spi1_csn0|
|--- ---------------- --------------------|
|503|       SPI1_CSN1|      peri_spi1_csn1|
|--- ---------------- --------------------|
|504|       SPI1_MOSI|      peri_spi1_mosi|
|--- ---------------- --------------------|
|505|       SPI1_MISO|      peri_spi1_miso|
|--- ---------------- --------------------|
|506|       SPI1_SCLK|      peri_spi1_sclk|
|--- ---------------- --------------------|
|507|       UART0_TXD|      peri_uart0_txd|
|--- ---------------- --------------------|
|508|       UART0_RXD|      peri_uart0_rxd|
|--- ---------------- --------------------|
|509|      UART0_RTSN|     peri_uart0_rtsn|
|--- ---------------- --------------------|
|510|      UART0_CTSN|     peri_uart0_ctsn|
|--- ---------------- --------------------|
|511|       UART1_TXD|      peri_uart1_txd|
|--- ---------------- --------------------|
|--- ---------------- --------------------|
```
