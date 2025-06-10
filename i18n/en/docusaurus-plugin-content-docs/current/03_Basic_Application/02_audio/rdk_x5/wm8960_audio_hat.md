---
sidebar_position: 4
---

# Waveshare WM8960 Audio HAT

## Product Overview
The WM8960 Audio HAT is an audio adapter board produced by Waveshare Electronics, featuring the WM8960 Codec. It supports dual-channel microphone recording and audio playback. The board is shown below:

![image-audio-wm8960](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/02_audio/image/image-audio-wm8960.jpg)

For more details, refer to the [WM8960 Audio HAT Wiki](https://www.waveshare.net/wiki/WM8960_Audio_HAT).

## Installation

- ### Hardware Setup

Connect the audio HAT to the 40-pin header of the RDK X5 as shown below:  
![image-wm8960-audio-hat-rkd-x5-setup-2](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/02_audio/image/image-wm8960-audio-hat-rkd-x5-setup-2.png)

- ### Software Configuration

1. Use `srpi-config` to configure the audio board.  
Go to `3 Interface Options` -> `I5 Audio`  
Select `WM8960 Audio Driver HAT`:
![image-audio-codec-select-wm8960-hat-rdkx5](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/02_audio/image/image-audio-codec-select-wm8960-hat-rdkx5.png)  

2. Follow the prompt to reboot, or run `sync && reboot` to restart the board. If `cat /proc/asound/cards` shows a `duplexaudioi2s1` sound card, the installation was successful.

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

2. Remove the HAT from the board.

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

To check user-space device files:
```shell
root@ubuntu:~# ls /dev/snd/
by-path  controlC0  controlC1  pcmC0D0p  pcmC0D1c  pcmC1D0c  pcmC1D0p  timer
```
From the above, and with reference to the [Onboard Earphone Audio Port](in_board_es8326.md#运行), you can confirm that sound card 0 corresponds to the `WM8960 Audio Driver HAT` node. The devices exist, with device numbers `0-0` and `0-1`. The actual devices to operate are `pcmC0D0p` and `pcmC0D1c`.

The onboard sound card is card 1, device number `1-0`, which is not used here.

This audio HAT requires audio routing configuration before each function. The following commands must be run before each scenario.

- ### Recording
- Dual-channel microphone recording:

```
tinymix -D 0 set 'Left Input Boost Mixer LINPUT1 Volume' 3
tinymix -D 0 set 'Right Input Boost Mixer RINPUT1 Volume' 3

tinymix -D 0 set 'Left Input Boost Mixer LINPUT1 Volume' 1
tinymix -D 0 set 'Right Input Boost Mixer RINPUT1 Volume' 1

tinymix -D 0 set 'Capture Volume' 40,40
tinymix -D 0 set 'ADC PCM Capture Volume' 200,200

tinymix -D 0 set 'Left Boost Mixer LINPUT1 Switch' 1
tinymix -D 0 set 'Right Boost Mixer RINPUT1 Switch' 1

tinymix -D 0 set 'Left Input Mixer Boost Switch' 1
tinymix -D 0 set 'Right Input Mixer Boost Switch' 1

tinymix -D 0 set 'Capture Switch' 1,1
tinycap ./2chn_test.wav -D 0 -d 0 -c 2 -b 16 -r 48000 -p 512 -n 4 -t 5
```

- ### Playback

- Dual-channel speaker playback

```
tinymix -D  0 set 'Left Output Mixer PCM Playback Switch' 1
tinymix -D  0 set 'Right Output Mixer PCM Playback Switch' 1
tinymix -D  0 set 'Speaker DC Volume' 3
tinymix -D  0 set 'Speaker AC Volume' 3
tinymix -D  0 set 'Speaker Playback Volume' 127,127
tinymix -D  0 set 'Playback Volume' 255,255
tinymix -D  0 set 'Left Output Mixer PCM Playback Switch' 1
tinymix -D  0 set 'Right Output Mixer PCM Playback Switch' 1
tinyplay ./2chn_test.wav -D 0 -d 0
```

- Simultaneous headphone and speaker playback
```
tinymix -D  0 set 'Headphone Playback Volume' 80,80
tinymix -D  0 set 'Playback Volume' 220,220
tinymix -D  0 set 'Speaker DC Volume' 4
tinymix -D  0 set 'Left Output Mixer PCM Playback Switch' 1
tinymix -D  0 set 'Right Output Mixer PCM Playback Switch' 1
tinyplay ./2chn_test.wav -D 0 -d 0
```

- Headphone playback only (speaker muted)
```
tinymix -D  0 set 'Headphone Playback Volume' 115,115
tinymix -D  0 set 'Speaker Playback Volume' 0,0
tinymix -D  0 set 'Playback Volume' 244,244
tinymix -D  0 set 'Speaker DC Volume' 4
tinymix -D  0 set 'Left Output Mixer PCM Playback Switch' 1
tinymix -D  0 set 'Right Output Mixer PCM Playback Switch' 1
tinyplay ./2chn_test.wav -D 0 -d 0
```

## FAQ
Q1: Hardware and software are working, audio path is normal, speakers have sound, but headphones are silent.

A1: The headphone volume may be too low. Increase the headphone volume, for example:  
`tinymix -D  0 set 'Headphone Playback Volume' 115,115`

[See more questions here](../../../08_FAQ/04_multimedia.md)
