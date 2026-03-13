---
sidebar_position: 3
---

# Waveshare Audio Driver HAT REV2

## Product Overview
The Audio Driver HAT REV2, produced by Waveshare Electronics, is an audio expansion board featuring the ES7210+ES8156 dual Codec solution. It supports 4-channel circular microphone recording, dual-channel audio playback, audio loopback, and more. The board is shown below:

![image-audio-driver-hat](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/02_audio/image/image-audio-driver-hat.jpg)

For detailed information about the audio sub-board, please refer to the [Audio Driver HAT](https://www.waveshare.net/shop/Audio-Driver-HAT.htm).

## Installation

- ### Hardware Setup
1. Connect the expansion board to the 40-pin header of the RDK X3 as shown below.  
![image-audio-driver-hat-setup](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/02_audio/image/image-audio-driver-hat-setup.jpg)

2. Run the command `cat /sys/class/socinfo/som_name` to check the board type, and set the DIP switches on the audio sub-board accordingly:
   - If the returned value is 5 or 6, set all three DIP switches to the `ON` position.
   - If the returned value is 8, set all three DIP switches to the `OFF` position.

- ### Software Configuration
1. Use `srpi-config` to configure the audio board.  
Go to `3 Interface Options` -> `I5 Audio`  
Select `Audio Driver HAT V2`:
![image-audio-codec-select-hat-v2-rdkx5.png](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/02_audio/image/image-audio-codec-select-hat-v2-rdkx5.png)

2. Follow the prompts to execute the `reboot` command, or run `sync && reboot` to restart the board. If `cat /proc/asound/cards` shows a sound card other than `duplexaudio`, the expansion board is installed successfully.
```shell
root@ubuntu:~# cat /proc/asound/cards 
 0 [duplexaudioi2s1]: simple-card - duplex-audio-i2s1
                      duplex-audio-i2s1
 1 [duplexaudio    ]: simple-card - duplex-audio
                      duplex-audio
```

- ### Uninstallation
1. Use `srpi-config` to configure the audio board.  
Go to `3 Interface Options` -> `I5 Audio`  
Select `UNSET` to uninstall the audio driver and related configurations.

2. Remove the expansion board.

## Usage

Check if the sound card exists and verify the device number.

To confirm the sound card is registered (as mentioned above):
```shell
root@ubuntu:~# cat /proc/asound/cards 
 0 [duplexaudioi2s1]: simple-card - duplex-audio-i2s1
                      duplex-audio-i2s1
 1 [duplexaudio    ]: simple-card - duplex-audio
                      duplex-audio
```

To check logical devices:
```shell
root@ubuntu:~# cat /proc/asound/devices
  2: [ 0- 0]: digital audio playback
  3: [ 0- 1]: digital audio capture
  4: [ 0]   : control
  5: [ 1- 0]: digital audio playback
  6: [ 1- 0]: digital audio capture
  7: [ 1]   : control
 33:        : timer
```

To check the actual device files in user space:
```shell
root@ubuntu:~# ls /dev/snd/
by-path  controlC0  controlC1  pcmC0D0p  pcmC0D1c  pcmC1D0c  pcmC1D0p  timer
```
From the above, and with reference to the [Onboard Earphone Audio Port](in_board_es8326.md#运行), you can confirm that sound card 0 corresponds to the `Audio Driver HAT REV2` node; the devices exist, and the device numbers are `0-0` and `0-1`. The actual devices to operate are `pcmC0D0p` and `pcmC0D1c`.

The onboard sound card corresponds to 1, with device number `1-0`, which is not used here.

- ### Recording

- 2-channel microphone recording:

```
tinycap ./2chn_test.wav -D 0 -d 1 -c 2 -b 16 -r 48000 -p 512 -n 4 -t 5
```

- 4-channel microphone recording:

```
tinycap ./4chn_test.wav -D 0 -d 1 -c 4 -b 16 -r 48000 -p 512 -n 4 -t 5
```

- ### Playback

- Dual-channel audio playback (4-channel playback is not supported):

```
tinyplay ./2chn_test.wav -D 0 -d 0
```

- ### Audio Loopback Test

The playback loopback signal of this audio board uses recording channels 7 & 8, so you need to use the 8-channel recording command to capture it.

- Start 8-channel microphone recording
```shell
tinycap ./8chn_test.wav -D 0 -d 1 -c 8 -r 16000 -b 16 -t 3 -p 256
```

- Start dual-channel audio playback
```
tinyplay ./2chn_test.wav -D 0 -d 0
```

After recording, you can use audio software to check the spectrum information of channels 7 & 8 in the `8chn_test.wav` file.

## FAQ
[For more questions, see this link](../../../08_FAQ/04_multimedia.md)
