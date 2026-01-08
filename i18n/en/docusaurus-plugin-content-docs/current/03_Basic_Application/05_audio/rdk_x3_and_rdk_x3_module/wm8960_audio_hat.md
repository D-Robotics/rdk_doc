---
sidebar_position: 2
---

```mdx-code-block
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';
```

# Waveshare WM8960 Audio HAT

## Product Overview
The WM8960 Audio HAT, produced by Waveshare, is an audio adapter board based on the WM8960 Codec. It supports dual-channel microphone recording and audio playback. The appearance of the adapter board is shown below:

![image-audio-wm8960](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/02_audio/image/image-audio-wm8960.jpg)

For detailed information about the audio adapter board, please refer to the [WM8960 Audio HAT](https://www.waveshare.net/wiki/WM8960_Audio_HAT) documentation.

## Installation

### Hardware Setup

<Tabs groupId="rdk-type">
<TabItem value="rdk-x3-pi" label="RDK-X3-PI">

1. Connect the adapter board to the 40-pin header of the RDK X3 as shown below:  
![image-wm8960-audio-hat-setup](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/02_audio/image/image-wm8960-audio-hat-setup.jpg)

</TabItem>

<TabItem value="rdk-x3-md" label="RDK-X3-Module">

1. Connect the adapter board to the 40-pin header of the RDK X3 as shown below:  
![image-x3md-wm8960](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/02_audio/image/image-x3md-wm8960.png)

</TabItem>
</Tabs>

### Software Configuration

1. Use `srpi-config` to configure the audio board.  
Navigate to `3 Interface Options` -> `I5 Audio`  
Select `WM8960 Audio HAT`:
![image-audio-driver-hat-config00](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/02_audio/image/image-audio-driver-hat-config01.png)  

1. Run the command `sync && reboot` to restart the development board. If the following device nodes appear under `/dev/snd`, the adapter board has been installed successfully:
    ```shell
    root@ubuntu:~# ls /dev/snd/
    by-path  controlC0  pcmC0D0c  pcmC0D0p  pcmC0D1c  pcmC0D1p  timer
    ```
On the `RDK X3`, the playback node for this audio board is `pcmC0D0p`, and the recording node is `pcmC0D1c`.

### Uninstallation
1. Use `srpi-config` to configure the audio board.  
Navigate to `3 Interface Options` -> `I5 Audio`  
Select `UNSET` to uninstall the audio driver and related configurations.

1. Remove the adapter board.

## Usage

### 1. Check the Sound Card Device

Check if the sound card exists and verify the device number.

- To confirm if the sound card is registered, run:
```
cat /proc/asound/cards 
```

Example output:

```
 root@ubuntu:~# cat /proc/asound/cards
 0 [hobotsnd6      ]: hobotsnd6 - hobotsnd6
                      hobotsnd6
```

If an entry similar to "hobotsnd6" is visible, it indicates that the sound card has been recognized.

- To check the location of the functional devices, run:
```
cat /proc/asound/devices
```

Example output:

```
root@ubuntu:~# cat /proc/asound/devices
    2: [ 0]   : control
    3: [ 0- 0]: digital audio playback
    4: [ 0- 0]: digital audio capture
    5: [ 0- 1]: digital audio playback
    6: [ 0- 1]: digital audio capture
    33:        : timer
```

### Recording
<Tabs groupId="rdk-type">
<TabItem value="rdk-x3-pi" label="RDK-X3-PI">

**Dual-channel microphone recording**

Use tinycap to record 2-channel audio:

```
tinycap ./2chn_test.wav -D 0 -d 0 -c 2 -b 16 -r 48000 -p 512 -n 4 -t 5
```

### Playback
**Dual-channel audio playback**

Using tinyplay to play recorded audio files, the commonly used parameters are as follows:

```
tinyplay ./2chn_test.wav -D 0 -d 1
```

</TabItem>

<TabItem value="rdk-x3-md" label="RDK-X3-Module">

### Recording

Dual-channel microphone recording:

```shell
 tinycap ./2chn_test.wav -D 0 -d 1 -c 2 -b 16 -r 48000 -p 512 -n 4 -t 5
```

### Playback

Dual-channel audio playback:
    Use tinyplay to play the recorded audio file. Common parameters are as follows:

```shell
tinyplay ./2chn_test.wav -D 0 -d 0
```
</TabItem>
</Tabs>

## FAQ
[See this link for common audio issues](../../../08_FAQ/04_multimedia.md)
