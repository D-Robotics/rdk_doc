---
sidebar_position: 2
---

# Waveshare WM8960 Audio HAT

## Product Overview
The WM8960 Audio HAT, produced by Waveshare, is an audio adapter board based on the WM8960 Codec. It supports dual-channel microphone recording and audio playback. The appearance of the adapter board is shown below:

![image-audio-wm8960](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/02_audio/image/image-audio-wm8960.jpg)

For detailed information about the audio adapter board, please refer to the [WM8960 Audio HAT](https://www.waveshare.net/wiki/WM8960_Audio_HAT) documentation.

## Installation

- ### Hardware Setup

1. Connect the adapter board to the 40-pin header of the RDK X3 as shown below:  
![image-wm8960-audio-hat-setup](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/02_audio/image/image-wm8960-audio-hat-setup.jpg)

- ### Software Configuration

2. Use `srpi-config` to configure the audio board.  
Navigate to `3 Interface Options` -> `I5 Audio`  
Select `WM8960 Audio HAT`:
![image-audio-driver-hat-config00](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/02_audio/image/image-audio-driver-hat-config01.png)  

3. Run the command `sync && reboot` to restart the development board. If the following device nodes appear under `/dev/snd`, the adapter board has been installed successfully:
    ```shell
    root@ubuntu:~# ls /dev/snd/
    by-path  controlC0  pcmC0D0c  pcmC0D0p  pcmC0D1c  pcmC0D1p  timer
    ```
On the `RDK X3`, the playback node for this audio board is `pcmC0D0p`, and the recording node is `pcmC0D1c`.

- ### Uninstallation
1. Use `srpi-config` to configure the audio board.  
Navigate to `3 Interface Options` -> `I5 Audio`  
Select `UNSET` to uninstall the audio driver and related configurations.

2. Remove the adapter board.

## Usage

Check if the sound card exists and verify the device number.

To confirm if the sound card is registered, run:
```
cat /proc/asound/cards 
```

To check the location of the functional devices, run:
```
cat /proc/asound/devices
```

- ### Recording
Dual-channel microphone recording:

```
tinycap ./2chn_test.wav -D 0 -d 1 -c 2 -b 16 -r 48000 -p 512 -n 4 -t 5
```

- ### Playback
Dual-channel audio playback:

```
tinyplay ./2chn_test.wav -D 0 -d 0
```

## FAQ
[See this link for common audio issues](../../../08_FAQ/04_multimedia.md)
