---
sidebar_position: 3
---
# 3.1.3 Using PWM

```mdx-code-block
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';
```

The Hobot.GPIO library only supports PWM on pins with additional hardware PWM controllers. Unlike the RPi.GPIO library, the Hobot.GPIO library does not implement software simulated PWM. 

Both RDK X3 and RDK Ultra support 2 PWM channels, corresponding to pins 33 and 32 on the 40-pin header.

RDK X5 supports four PWM groups, with two output channels per group, providing a total of eight PWM outputs, as shown in the table below:

| PMW Group | PWM Channel | 40PIN Pin |
| --- | ---- | ---- |
| PWM0 | LSIO_PWM_OUT0 | 29 |
| PWM0 | LSIO_PWM_OUT1 | 31 |
| PWM1 | LSIO_PWM_OUT2 | 37 |
| PWM1 | LSIO_PWM_OUT3 | 24 |
| PWM2 | LSIO_PWM_OUT4 | 28 |
| PWM2 | LSIO_PWM_OUT5 | 27 |
| PWM3 | LSIO_PWM_OUT6 | 32 |
| PWM3 | LSIO_PWM_OUT7 | 33 |

By default, PWM3 is enabled on the RDK X5. Additional PWM groups can be enabled via the `srpi-config` system configuration tool, which will remap the corresponding pins for PWM output. The configuration takes effect after reboot.

Select `3 Interface Options` → `I3 Peripheral Bus Config`, then choose the corresponding PWM group and set it to okay.

Refer to `/app/40pin_samples/simple_pwm.py` for detailed information on how to use PWM channels.

## Test Code

Open the PWM channel specified by `output_pin`, with an initial duty cycle of 25%. Increase the duty cycle by 5% every 0.25 seconds until 100% is reached, and then decrease the duty cycle by 5% every 0.25 seconds. When the normal output waveform is present, you can measure the output signal and observe the waveform using an oscilloscope or logic analyzer.

```python
#!/usr/bin/env python3

import Hobot.GPIO as GPIO
import time

# PWM-supporting pins: 32 and 33
output_pin = 33

def main():
    # Pin Setup:
    # Board pin-numbering scheme
    GPIO.setmode(GPIO.BOARD)
    # Supported frequency range for RDK X3: 48KHz ~ 192MHz
    # Supported frequency range for RDK Ultra: 1Hz ~ 12MHz
    # Supported frequency range for RDK X5: 1Hz ~ 12MHz
    p = GPIO.PWM(output_pin, 48000)
    # Initial duty cycle of 25%. Increase by 5% every 0.25 seconds until 100% is reached, then decrease by 5% every 0.25 seconds
    val = 25
    incr = 5
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

## Execution Steps

<Tabs groupId="tool-type">
<TabItem value="login" label="Login to the RDK">

Execute the `simple_pwm.py` program to start the GPIO read/write program.

  ```bash
  sunrise@ubuntu:~$ cd /app/40pin_samples/
  sunrise@ubuntu:/app/40pin_samples$ sudo python3 ./simple_pwm.py
  ```
</TabItem>

<TabItem value="rdk-studio" label="RDK Studio">


**Method 1: Install VS Code APP Locally**

1. Use RDK Studio to add devices. Refer to [Add RDK Device](../../01_Quick_start/09_RDK_Studio/05_Device_management/01_hardware_resource.md).
    
2. Use VS Code to open the sample project in the RDK device.
    
    :::warning Note
    
    Requires local installation of VS Code software. Click the VS Code APP icon on the device card in RDK Studio to automatically open the local VS Code, then use the SSH Remote plugin to open the sample project in the RDK device (the SSH Remote plugin will be automatically installed, no manual installation needed).
    
    :::
        
        
    ![VScode APP icon](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/rdk_studio/en/app_vscode.png)

3. Click the VS Code APP icon to open VS Code, enter the password for the account selected when adding the device, and press "Enter" to confirm.

    :::info Note

    - Username: root——Password: root
    - Username: sunrise——Password: sunrise

    :::
        
    ![VScode interface](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/rdk_studio/open_vscode_password_input.png)
    
4. Enter the VS Code APP interface, click `Terminal` in the top navigation bar to create a new terminal.
    
    ![VScode interface](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/rdk_studio/open_vscode_terminal.png)

5.  Execute the `./simple_pwm.py` program.

  ```bash
  sunrise@ubuntu:~$ cd /app/40pin_samples/
  sunrise@ubuntu:/app/40pin_samples$ sudo python3 ./simple_pwm.py
  ```

**Method 2: Using On-board VS Code Web**

1. Click the application space icon to view more applications.

    ![Application Space Interface](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/en/rdk_studio_left_menu_device_manage_hr_add_device_application_space_download.png)
    
2. Click to install VS Code Web onto the development board. This is used to run preloaded functional test code on the development board without needing to install VS Code locally.

    ![Download Interface](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/rdk_studio/en/web_vscode-download.png)

3. Click the VS Code Web icon to open VS Code Web.
    
    ![VS Code Web](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/rdk_studio/en/web_vscode.png)

4. Click `Open Folder`, then enter the path where the code program is located: `/app/`, and click `OK` to confirm.
    
    ![VS Code Open Folder Interface](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/rdk_studio/open_app_path_ok.png)

5. Enter the VS Code APP interface, click the list icon on the left navigation bar, select `Terminal` -> `New Terminal` to create a new terminal.

    ![VS Code New Terminal Interface](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/rdk_studio/vscode_open_terminal.png)

6.  Execute the `./simple_pwm.py` program to start the GPIO read/write program.

  ```bash
  sunrise@ubuntu:~$ cd /app/40pin_samples/
  sunrise@ubuntu:/app/40pin_samples$ sudo python3 ././simple_pwm.py
  ```



</TabItem>
</Tabs>