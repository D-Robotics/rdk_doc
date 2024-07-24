---
sidebar_position: 1
---
# 3.2.1 Audio Adapter Board On RDK X3

The RDK X3 currently supports **WM8960 Audio HAT**. This board are designed to meet the functional requirements of different voice scenarios. The following will provide detailed instructions on how to use this audio board.

:::note Note

If you are prompted that the Miniboot version is not the latest after installing the driver, please go to `1 System Options` -> `S7 Update Miniboot` to update Miniboot.

:::


## WM8960 Audio HAT

### Product Introduction

WM8960 Audio HAT is an audio adapter board produced by Waveshare, using the WM8960 Codec solution, and can achieve dual channel microphone recording and audio playback functions. The appearance of the adapter board is as shown in the figure below:

![image-audio-wm8960](./image/image-audio-wm8960.jpg)

This audio sub-board is only compatible with RDK X3 version 2.0, and the version number can be queried as follows. The return value 8 indicates that the device is 2.0 version.
```shell
root@ubuntu:/userdata# cat /sys/class/socinfo/som_name
8
```

For a detailed introduction to the audio adapter board, please refer to [WM8960 Audio HAT](https://www.waveshare.net/wiki/WM8960_Audio_HAT).

### Installation Method

1. Connect the adapter board to the 40-pin header of RDK X3 as shown in the following picture  
![image-wm8960-audio-hat-setup](./image/image-wm8960-audio-hat-setup.jpg)

2. Configure the audio board using `srpi-config`  
Enter `3 Interface Options` -> `I5 Audio`  
Select `WM8960 Audio HAT`:
![image-audio-driver-hat-config00](./image/image-audio-driver-hat-config01.png) 

3. Run the command `sync && reboot` to restart the development board. If the device nodes like the following appear under /dev/snd, it indicates that the adapter board is installed successfully.
    ```shell
    root@ubuntu:~# ls /dev/snd/
    Playback   controlC0  pcmC0D0c  pcmC0D0p  pcmC0D1c  pcmC0D1p  pcmC1D0c  pcmC1D0p  timer
    ```by-path  controlC0  pcmC0D0c  pcmC0D0p  pcmC0D1c  pcmC0D1p  timer
    ```

### Uninstalling Method
1. Configure the audio board using `srpi-config`   
Go to `3 Interface Options`->`I5 Audio`  
Select `UNSET` to uninstall the audio driver and related configurations

### Audio Nodes
The playback node for this audio board on the `RDK X3` is `pcmC0D1p`, and the recording node is `pcmC0D0c`

### Recording and Playback Test

The test uses the toolset of `tinyalsa` library: use `tinycap` for recording and `tinyplay` for playback

Usage of `tinycap`:
```shell
tinycap
Usage: tinycap {file.wav | --} [-D card] [-d device] [-c channels] [-r rate] [-b bits] [-p period_size] [-n n_periods] [-t time_in_seconds]

Use -- for filename to send raw PCM to stdout
```
Usage of `tinyplay`:
```shell
tinyplay
usage: tinyplay file.wav [options]
options:
-D | --card   <card number>    The device to receive the audio
-d | --device <device number>  The card to receive the audio
-p | --period-size <size>      The size of the PCM's period
-n | --period-count <count>    The number of PCM periods
-i | --file-type <file-type >  The type of file to read (raw or wav)
-c | --channels <count>        The amount of channels per frame
-r | --rate <rate>             The amount of frames per second
-b | --bits <bit-count>        The number of bits in one sample
-M | --mmap                    Use memory mapped IO to play audio
```
If you want to learn more about the `tinyalsa` library, please refer to their [repository](https://github.com/tinyalsa/tinyalsa)



- Recording with a 2-channel microphone:

```
tinycap ./2chn_test.wav -D 0 -d 0 -c 2 -b 16 -r 48000 -p 512 -n 4 -t 5
```

- Playback of stereo audio:

```
tinyplay ./2chn_test.wav -D 0 -d 1
```


## Coexistence of Audio Subboard and USB Sound Card

If you have a USB sound card and want it to coexist with the above-mentioned audio subboard, please follow the steps below:

1. Make sure the audio subboard is functional according to the tutorial above.

2. Connect the USB sound card and observe the newly added nodes under `/dev/snd` after the driver is loaded. Taking `WM8960` + USB **full-duplex** sound card as an example:
```bash
    root@ubuntu:~# ls /dev/snd/
    by-path  controlC0  pcmC0D0c  pcmC0D0p  pcmC0D1c  pcmC0D1p  pcmC1D0c  pcmC1D0p  timer
```
The `pcmC1D0c pcmC1D0p` nodes are for the USB sound card, and they share a full-duplex node.

3. Modify `/etc/pulse/default.pa` and add the corresponding node information below `load-module module-alsa-source`:
```apacheconf
...

.ifexists module-udev-detect.so
load-module module-alsa-sink device=hw:0,1 mmap=false tsched=0 fragments=2 fragment_size=960 rate=48000 channels=2 rewind_safeguard=960
load-module module-alsa-source device=hw:0,0 mmap=false tsched=0 fragments=2 fragment_size=960 rate=48000 channels=2
load-module alsa device=hw:1,0 #corresponding to the above nodes
load-module module-alsa-source device=hw:1,0 #corresponding to the above nodes
.else

...
```

4. Save the configuration and reboot the development board.