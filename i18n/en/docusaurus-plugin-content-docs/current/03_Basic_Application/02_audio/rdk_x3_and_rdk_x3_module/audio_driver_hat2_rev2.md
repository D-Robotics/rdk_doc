---
sidebar_position: 1
---

# Waveshare Audio Driver HAT REV2

## Product Overview

The Audio Driver HAT REV2, produced by Waveshare Electronics, is an audio adapter board featuring a dual Codec solution with ES7210+ES8156. It supports 4-channel circular microphone recording, dual-channel audio playback, audio loopback, and more. The board is shown below:

![image-audio-driver-hat](../../../../../../../static/img/03_Basic_Application/02_audio/image/image-audio-driver-hat.jpg)

For detailed information about the audio sub-board, please refer to the [Audio Driver HAT](https://www.waveshare.net/shop/Audio-Driver-HAT.htm).

## Installation

- ### Hardware Setup

1. Connect the adapter board to the 40-pin header of the RDK X3 as shown below.  
![image-audio-driver-hat-setup](../../../../../../../static/img/03_Basic_Application/02_audio/image/image-audio-driver-hat-setup.jpg)

2. Run the command `cat /sys/class/socinfo/som_name` to check the board type, and set the DIP switches on the audio sub-board according to the returned value:
    - If the value is 5 or 6, set all three DIP switches to the `ON` position.
    - If the value is 8, set all three DIP switches to the `OFF` position.

- ### Software Configuration

1. Use `srpi-config` to configure the audio board.  
Go to `3 Interface Options` -> `I5 Audio`  
Select `Audio Driver HAT V2`:
![image-audio-driver-hat-config00](../../../../../../../static/img/03_Basic_Application/02_audio/image/image-audio-driver-hat-config00.png)  

2. Run `sync && reboot` to restart the board. If the following device nodes appear under /dev/snd, the installation was successful:
     ```shell
     root@ubuntu:/userdata# ls /dev/snd
     by-path  controlC0  pcmC0D0c  pcmC0D1p  timer
     ```

- ### Uninstallation

1. Use `srpi-config` to configure the audio board.  
Go to `3 Interface Options` -> `I5 Audio`  
Select `UNSET` to uninstall the audio driver and related configurations.

2. Remove the adapter board.

## Usage

Check if the sound card exists and verify the device number.

To confirm the sound card is registered, run:
```
cat /proc/asound/cards 
```

To check the location of functional devices, run:
```
cat /proc/asound/devices
```

- ### Recording

2-channel microphone recording:

```
tinycap ./2chn_test.wav -D 0 -d 0 -c 2 -b 16 -r 48000 -p 512 -n 4 -t 5
```

4-channel microphone recording:

```
tinycap ./4chn_test.wav -D 0 -d 0 -c 4 -b 16 -r 48000 -p 512 -n 4 -t 5
```

- ### Playback

Dual-channel audio playback:

```
tinyplay ./2chn_test.wav -D 0 -d 1
```

- ### Audio Loopback Test

The playback loopback signal of this audio board uses recording channels 7 & 8, so you need to use the 8-channel recording command for collection.

Start 8-channel microphone recording:
```shell
tinycap ./8chn_test.wav -D 0 -d 0 -c 8 -b 16 -r 48000 -p 512 -n 4 -t 5
```

Start dual-channel audio playback:
```
tinyplay ./2chn_test.wav -D 0 -d 1
```

After recording, you can use audio software to check the spectrum information of channels 7 & 8 in the `2chn_test.wav` file.

## FAQ

1. The RDK X3 Module does not support the old Audio Driver HAT REV1. Please use the REV2 version.

[For more questions, see this link](../../../08_FAQ/04_multimedia.md)