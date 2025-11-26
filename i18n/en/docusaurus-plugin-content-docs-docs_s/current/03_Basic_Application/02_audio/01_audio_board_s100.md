---
sidebar_position: 1
---

# 3.2.1 RDK S100 Series Audio Usage Guide

S100 audio is developed based on the standard ALSA framework. In user space, the open-source alsa-lib provides libraries and binary tools for functional testing. This section describes the basic methods for audio functionality testing on the S100.

## Introduction to alsa-lib

### Parameter Description for arecord/aplay

- Common parameter explanations

| Parameter         | Description                                                  | Additional Notes                        |
|-------------------|--------------------------------------------------------------|-----------------------------------------|
| -D                | Specifies the device node. Set via hw:x,x                    | x,x correspond to the card number and device number respectively |
| -c                | Channel: number of audio channels, including mono, stereo, or multi-channel configurations such as 8/16 channels |                                         |
| -r                | Rate: also known as sample rate, i.e., the number of samples per second |                                         |
| -f                | Sample format (bit depth)                                    | Common formats: U8/S16_LE/S32_LE        |
| -t                | File type                                                    | Includes wav, voc, raw, etc.            |
| -d                | Recording duration (in seconds). Recording stops after the specified time |                                         |
| --period-size     | Number of frames processed per hardware interrupt            |                                         |
| --buffer-size     | Size of the data buffer                                      |                                         |
| -I                | Non-interleaved mode                                         |                                         |

- Common commands

  - List sound cards and digital audio devices

  ```
  arecord -l
  aplay -l
  ```

  - Query hardware capabilities

  ```
  arecord --dump-hw-params -Dhw:0,0
  aplay --dump-hw-params -Dhw:0,1 /dev/zero
  ```

  - Loopback test command

  ```
  arecord -Dhw:0,0 -c 2 -r 48000 -f S16_LE -t wav -d 10 | aplay -Dhw:0,1
  ```

  - Non-interleaved data storage. Note: Non-interleaved data is raw PCM.

  ```
  arecord -Dhw:0,0 -c 2 -r 48000 -f S16_LE -t wav -d 5 -I 0
  aplay -Dhw:0,1 -c 2 -r 48000 -f S16_LE -t wav -I 0
  ```

  - Interleaved data storage

  ```
  arecord -Dhw:0,0 -c 2 -r 16000 -f S16_LE -t wav -d 5 test.wav
  aplay -Dhw:0,1 test.wav
  ```

### Control Commands

- Query all control parameters and their current values for the current codec

```
amixer scontrols
amixer scontents
```

- Parameter adjustment

For example, adjusting playback volume:

```
amixer sset 'DAC' 120 // Set
amixer sget 'DAC' // Get
```

:::info Note

The default sound card/device number is 0,0. If multiple sound cards are connected in your actual setup, you must specify the correct card and device numbers using -D and -c.
:::

How to confirm the current sound card/device number:

- arecord -l

```
**** List of CAPTURE Hardware Devices ****
card 0: s100snd2 [s100snd2], device 0: s100dailink0 ES7210 4CH ADC 0-0 []
  Subdevices: 1/1
  Subdevice #0: subdevice #0
```

- /proc/asound/cards node

```
root@ubuntu:/userdata# cat /proc/asound/cards
 0 [s100snd2       ]: s100snd2 - s100snd2
                      s100snd2
```

### Application Space Interface

For API descriptions and usage, refer to the official documentation: https://www.alsa-project.org/alsa-doc/alsa-lib/pcm_2pcm_8c.html

## Audio Driver HAT REV2

### Preparation

The S100 is compatible with the audio HAT produced by Waveshare, connecting to the S100 development board via the 40-pin header. For details about the HAT and connection instructions, refer to: [Audio HAT Usage Guide](../../07_Advanced_development/02_linux_development/04_driver_development_s100/11_driver_audio.md#Audio-HAT-Usage-Guide)

### Device Nodes

After the audio HAT is connected to the S100 and the driver is loaded, the following device nodes are created:

- pcmC0D0c: capture (recording)

- pcmC0D1p: playback

- controlC0: control

### Functional Testing

- Recording

```
arecord -Dhw:0,0 -c 2 -r 48000 -f S16_LE -t wav -d 5 test.wav
```

This command records a 5-second WAV audio file with 48 kHz sampling rate, 2 channels, and 16-bit depth. Users can adjust these parameters according to their specific requirements.

- Playback

```
aplay -Dhw:0,1 test.wav
```

This command plays a WAV file. The audio format is parsed from the WAV file header and passed to the driver.