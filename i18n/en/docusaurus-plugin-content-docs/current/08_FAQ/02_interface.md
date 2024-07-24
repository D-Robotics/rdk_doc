---
sidebar_position: 2
---

# 8.2 Interface Class

### 40PIN Interface

<font color='Blue'>[Question]</font> 

- Does the development board support VDD_5V as a power input in the 40PIN interface?

<font color='Green'>[Answer]</font> 

- Version 1.2 and above of the development board can support it. The version number can be confirmed through the silk screen information on the development board.

<font color='Blue'>[Question]</font> 

- Does the development board support the C/C++ interface functionality in the 40PIN interface?

<font color='Green'>[Answer]</font>

- Please refer to the forum article, [X3Lite WiringPi](https://developer.horizon.ai/forumDetail/109609560406362634)

### Serial Interface

<font color='Blue'>[Question]</font> 

- After power on, there is no log display in the serial interface.

<font color='Green'>[Answer]</font> 

- Check whether the red power indicator on the development board is on.
- Check the connection between the development board and the serial interface board, refer to the [Debug UART](../installation/hardware_interface#debug_uart) chapter for more details.
- Check the serial port parameter configuration in the terminal software, the correct configuration is as follows:  

![image-20221124200013163](./image/interface/image-20221124200013163.png)

### Network Interface

<font color='Blue'>[Question]</font> 

- The development board cannot connect to the internet.

<font color='Green'>[Answer]</font> 

- Check the network configuration, please refer to the [Network Configuration](../configuration/network) chapter.

<font color='Blue'>[Question]</font> 

- The development board cannot be connected via SSH.<font color='Green'>【Answer】</font>

- The prompt "Connection timed out" indicates a network communication error. Please refer to the [SSH login](../installation/remote_login#ssh) chapter for network configuration confirmation.
- The prompt "Authentication failed" indicates incorrect user or password for login. Please confirm and try again.

![image-20221124201544978](./image/interface/image-20221124201544978.png)

<font color='Blue'>【Question】</font>

- When using wireless network on the development board, the connection is unstable and the transmission speed is slow.

<font color='Green'>【Answer】</font>

- The wireless network on the development board is easily affected by factors such as metal casing and heat sink, leading to weakened signal strength. You can install an external antenna to enhance the signal.

<font color='Blue'>【Question】</font>

- The wireless network on the development board cannot be used, and the `wlan0` device cannot be found when using the `ifconfig` command.

<font color='Green'>【Answer】</font>

- Execute the `rfkill unblock wlan` command to re-enable the wireless network.

### USB Interface

<font color='Blue'>【Question】</font>

- What is the default device node for the USB camera when connected to the development board?

<font color='Green'>【Answer】</font>

- The device node for the USB camera is `/dev/video8`.

<font color='Blue'>【Question】</font>

- After plugging in a USB camera to the development board, the `/dev/video8` device node is not generated.

<font color='Green'>【Answer】</font>

- Confirm if the USB camera is working properly by connecting it to a computer to see if it can be recognized.
- Make sure the camera is properly plugged in and try re-plugging it.
- Check if the development board's Micro USB interface is not connected to a data cable.

<font color='Blue'>【Question】</font>

When connecting a remote control joystick to the development board, the `/dev/input/js0` device node is not generated.

<font color='Green'>【Answer】</font>

- Update the system on the development board and install the drivers and toolkits according to the following steps:- Load driver: `sudo modprobe -a joydev`
- Install test tool: `sudo apt install joystick`
- Test command: `jstest /dev/input/js0`

### MIPI CSI interface

<font color='Blue'>【Question】</font>

- After connecting the MIPI camera to the development board, the `i2cdetect` command cannot detect the I2C address.

<font color='Green'>【Answer】</font>

- Please refer to the [MIPI camera](../installation/hardware_interface#mipi_port) section to confirm the camera's connection method.
- Do not plug or unplug the camera while the development board is powered on, as it may damage the camera due to short circuit.

<font color='Blue'>【Question】</font>

- When running the MIPI camera example, an error is reported:
    ```bash
    sunrise@ubuntu:/app/pydev_demo/03_mipi_camera_sample$ python3 mipi_camera.py
    Traceback (most recent call last):
    File "mipi_camera.py", line 29, in <module>
        disp_w, disp_h = get_display_res()
    File "mipi_camera.py", line 27, in get_display_res
        return int(res[1]), int(res[0])
    ValueError: invalid literal for int() with base 10: b'open device /dev/lt8618_ioctl failed\ndevice not open\n1080'
    ```

<font color='Green'>【Answer】</font>

- When running the example program with the `sunrise` account, `sudo` permission is required. For example, `sudo python3 mipi_camera.py`.

### Display interface

<font color='Blue'>【Question】</font>

- What resolutions does the development board's HDMI interface support?

<font color='Green'>【Answer】</font>

- Supported resolution types can be found in the [HDMI interface](../installation/hardware_interface#hdmi_interface) section.