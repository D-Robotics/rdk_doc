---
sidebar_position: 2
---
# On-board Earphone Audio Port

## Product Overview
Comes with a built-in 3.5mm audio jack, compatible with 4-pole headsets or other devices that fit the interface.

## Installation

Connect a 3.5mm headset to the audio jack. Note that the headset must be 4-pole; 3-pole connectors usually do not have a MIC (microphone).

![image-audio-earphone](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/02_audio/image/image-audio-earphone.png)

Ensure the headset is fully inserted, as shown below:

![image-audio-earphone-correct](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/02_audio/image/image-audio-earphone-correct.png)

Incorrect connection examples:

![image-audio-earphone-error-connection1](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/02_audio/image/image-audio-earphone-error-connection1.png)
![image-audio-earphone-error-connection2](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/02_audio/image/image-audio-earphone-error-connection2.png)

In the incorrect examples above, part of the metal connector is exposed, resulting in an incomplete connection. Even if the program runs normally, you may not be able to hear or record sound.

## Operation

Check if the sound card exists and verify the device number.

To confirm if the sound card is registered, use:
```shell
 0 [duplexaudio    ]: simple-card - duplex-audio
            duplex-audio
```

To check logical devices:
```shell
root@ubuntu:~# cat /proc/asound/devices
  2: [ 0- 0]: digital audio playback
  3: [ 0- 0]: digital audio capture
  4: [ 0]   : control
 33:        : timer

```
To check the actual device files in user space:
```shell
root@ubuntu:~# ls /dev/snd/
by-path/   controlC0  pcmC0D0c   pcmC0D0p   timer    
```

From the above, you can confirm that sound card 0 is the on-board sound card; the devices exist, and the device number is `0-0`. The actual devices to operate are `pcmC0D0p` and `pcmC0D0c`.

- ### Recording

```
# arecord -Dhw:0,0 -c 2 -r 48000 -f S24_LE -t wav -d 10 /userdata/record1.wav
Recording WAVE '/userdata/record1.wav' : Signed 24 bit Little Endian, Rate 48000 Hz, Stereo
```
If you see normal recording logs, wait about 10 seconds (`-d 10` means 10 seconds). After recording, you can play back the recording with the following command.

- ### Playback
```
# aplay -D hw:0,0 /userdata/record1.wav
Playing WAVE '/userdata/record1.wav' : Signed 24 bit Little Endian, Rate 48000 Hz, Stereo
```
Normally, you should hear the recorded sound through the headset.

## FAQ
1. [How to distinguish between USB sound card and on-board sound card](../../../08_FAQ/04_multimedia.md)

[For more questions, see this link](../../../08_FAQ/04_multimedia.md)
