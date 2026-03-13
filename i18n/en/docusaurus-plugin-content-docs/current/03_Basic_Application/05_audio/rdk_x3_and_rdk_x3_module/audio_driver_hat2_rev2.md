---
sidebar_position: 1
---

```mdx-code-block
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';
```

# Waveshare Audio Driver HAT REV2

## Product Overview

The Audio Driver HAT REV2, produced by Waveshare Electronics, is an audio adapter board featuring a dual Codec solution with ES7210+ES8156. It supports 4-channel circular microphone recording, dual-channel audio playback, audio loopback, and more. The board is shown below:

![image-audio-driver-hat](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/02_audio/image/image-audio-driver-hat.jpg)

For detailed information about the audio sub-board, please refer to the [Audio Driver HAT](https://www.waveshare.net/shop/Audio-Driver-HAT.htm).

## Installation

- ### Hardware Setup

<Tabs groupId="rdk-type">
<TabItem value="rdk-x3-pi" label="RDK-X3-PI">

1. Connect the adapter board to the 40-pin header of the RDK X3 as shown below.  
![image-audio-driver-hat-setup](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/02_audio/image/image-audio-driver-hat-setup.jpg)

</TabItem>

<TabItem value="rdk-x3-md" label="RDK-X3-Module">

1. Connect the adapter board to the 40-pin header of the RDK X3 as shown below. 
![image-x3md-v2](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/02_audio/image/image-x3md-v2.png)

</TabItem>
</Tabs>

2. Run the command `cat /sys/class/socinfo/som_name` to check the board type, and set the DIP switches on the audio sub-board according to the returned value:
    - If the value is 5 or 6, set all three DIP switches to the `ON` position.
    - If the value is 8, set all three DIP switches to the `OFF` position.

- ### Software Configuration

1. Use `srpi-config` to configure the audio board.  
Go to `3 Interface Options` -> `I5 Audio`  
Select `Audio Driver HAT V2`:
![image-audio-driver-hat-config00](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/02_audio/image/image-audio-driver-hat-config00.png)  

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

Output example:
 ```
 root@ubuntu:~# cat /proc/asound/cards
     0 [hobotsnd5      ]: hobotsnd5 - hobotsnd5
                          hobotsnd5
```
If you can see an entry similar to "hobotsnd5," it indicates that the sound card has been recognized.

To check the location of functional devices, run:
```
cat /proc/asound/devices
```

Output example:
```
root@ubuntu:~# cat /proc/asound/devices
    2: [ 0]   : control
    3: [ 0- 0]: digital audio playback
    4: [ 0- 1]: digital audio capture
    33:        : timer
```

### Recording

<Tabs groupId="rdk-type">
<TabItem value="rdk-x3-pi" label="RDK-X3-PI">

**2-channel microphone recording**  

Use tinycap to record 2-channel audio:
```
tinycap ./2chn_test.wav -D 0 -d 1 -c 2 -b 16 -r 48000 -p 512 -n 4 -t 5
```

**4-channel microphone recording**


```
tinycap ./4chn_test.wav -D 0 -d 1 -c 4 -b 16 -r 48000 -p 512 -n 4 -t 5
```

### Playback

**Dual-channel audio playback (4-channel playback is not supported)**

Use tinyplay to play the recorded audio file:
```
tinyplay ./2chn_test.wav -D 0 -d 0
```

### Audio Loopback Test

The audio loopback feature can be used to capture signals from the playback channel for subsequent analysis.

**Start 8-channel microphone recording(Including Loopback)**

The playback loopback signal of this audio board uses recording channels 7 & 8, so you need to use the 8-channel recording command for collection.

```shell
tinycap ./8chn_test.wav -D 0 -d 1 -c 8 -b 16 -r 48000 -p 512 -n 4 -t 5
```

**Start dual-channel audio playback**
```
tinyplay ./2chn_test.wav -D 0 -d 0
```

**After recording,** you can use audio software to check the spectrum information of channels 7 & 8 in the `2chn_test.wav` file.

</TabItem>

<TabItem value="rdk-x3-md" label="RDK-X3-Module">

- **2-channel microphone recording:**  
  Record 2-channel audio using tinycap:

  ```shell
  tinycap ./2chn_test.wav -D 0 -d 0 -c 2 -b 16 -r 48000 -p 512 -n 4 -t 5
  ```

- **4-Channel Microphone Recording**

  ```shell
  tinycap ./4chn_test.wav -D 0 -d 0 -c 4 -b 16 -r 48000 -p 512 -n 4 -t 5
  ```

###  Playback Operations

- **Dual-Channel Audio Playback (4-channel playback is not supported)**  
  Play recorded audio files using tinyplay:  

  ```shell
  tinyplay ./2chn_test.wav -D 0 -d 1
  ```

### Audio Loopback Test

The audio loopback function can be used to capture signals from the playback channel for subsequent analysis.

- **8-Channel Microphone Recording (including loopback)**
  The loopback signals of this audio board are mapped to recording channels 7 and 8. Use the 8-channel recording command:

  ```shell
  tinycap ./8chn_test.wav -D 0 -d 0 -c 8 -b 16 -r 48000 -p 512 -n 4 -t 5
  ```

- **Start Dual-Channel Audio Playback Simultaneously**

  ```shell
  tinyplay ./2chn_test.wav -D 0 -d 1
  ```

- **Analyze Loopback Signals**
  After recording, use audio analysis software such as Audacity to open 8chn_test.wav and examine the waveform or spectrum of channels 7 and 8 to verify the loopback functionality.
</TabItem>
</Tabs>
## FAQ

- If no sound card is detected, please check whether the hardware connections and the settings of the DIP switches are correct.

- If there is no sound during recording or playback, please verify that the audio file format and channel count are consistent with the command parameters.

- If there is no signal in the recording channel, please confirm that the 8-channel recording command has been used correctly.

[For more questions, see this link](../../../08_FAQ/04_multimedia.md)