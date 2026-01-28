---
sidebar_position: 3
---

# 3.1.3 PWM应用

```mdx-code-block
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';
```


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
    # 支持的频率范围： X3: 48KHz ~ 192MHz X5: 0.05HZ ~ 1MHZ
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
## 运行方式

<Tabs groupId="tool-type">
<TabItem value="login" label="登录开发板">

执行 `simple_pwm.py` 程序，以启动 GPIO 读写程序

  ```bash
  sunrise@ubuntu:~$ cd /app/40pin_samples/
  sunrise@ubuntu:/app/40pin_samples$ sudo python3 ./simple_pwm.py
  ```
</TabItem>

<TabItem value="rdk-studio" label="RDK Studio">

## 方式一：使用 VS Code APP

1. 使用 RDK Studio 添加设备，参见[添加 RDK 设备](../../01_Quick_start/09_RDK_Studio/05_Device_management/01_hardware_resource.md)。
   
2. 点击应用空间的 Visual Studio Code 应用图标打开应用。
   
3. 输入添加设备时所选账号的密码，按 “Enter” 键确认。
   
   :::info 提示

   - 用户名：root-密码：root
   - 用户名：sunrise-密码：sunrise

   :::
   
   ![VScode界面](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/rdk_studio/open_vscode_password_input.png)

4. 进入 VS Code APP 程序界面，点击上方导航栏的 “Terminal” 新建终端。

    ![VScode界面](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/rdk_studio/open_vscode_terminal.png)

5.  执行 `./simple_pwm.py` 程序。

  ```bash
  sunrise@ubuntu:~$ cd /app/40pin_samples/
  sunrise@ubuntu:/app/40pin_samples$ sudo python3 ./simple_pwm.py
  ```

## 方式二：使用板端 VS Code Web

        1. 点击应用空间图标，查看更多应用。

            ![应用空间界面](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/rdk_studio_left_menu_device_manage_hr_add_device_application_space_download.png)
            
        2. 点击安装 VS Code Web 到开发板上，用于运行开发板预置的功能测试代码，无需本地安装 VS Code。

            ![下载界面](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/rdk_studio/web_vscode-download.png)

        3. 点击 VS Code Web 图标，打开 VS Code Web。
            
            ![VS Code Web](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/rdk_studio/web_vscode.png)

        4. 点击 `Open Folder` 后填写代码程序所在路径 `/app/`，点击 `OK` 键确认。
            
            ![VScode Open Folder 界面](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/rdk_studio/open_app_path_ok.png)

        5. 进入 VS Code APP 程序界面，点击左侧导航栏的列表图标，选择 `Terminal` ——> `New Terminal`,新建终端。

            ![VScode 新建终端界面](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/rdk_studio/vscode_open_terminal.png)

        6.  执行 `./simple_pwm.py` 程序，以启动 GPIO 读写程序。

          ```bash
          sunrise@ubuntu:~$ cd /app/40pin_samples/
          sunrise@ubuntu:/app/40pin_samples$ sudo python3 ././simple_pwm.py
          ```
## 方式三：使用 Node-RED

1. 使用 RDK Studio 添加设备，参见[添加 RDK 设备](../../01_Quick_start/09_RDK_Studio/05_Device_management/01_hardware_resource.md)。
   
2. 点击应用空间的 Node-RED 图标打开应用。
3. 在 Node-RED 中根据需要配置相应的工作流。



</TabItem>
</Tabs> 


