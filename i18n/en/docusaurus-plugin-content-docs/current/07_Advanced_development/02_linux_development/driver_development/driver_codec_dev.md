---
sidebar_position: 15
---

# Audio Codec Adaptation Guide

### Overview

This chapter primarily explains the concept of audio and provides instructions for debugging and adding sound card configurations.

Related concepts:
- `DAI`: **Digital Audio Interface** refers to a digital audio interface.
- `CPU DAI`: The digital audio interface on the CPU side, which can be understood as the `X3`'s `I2S` interface.
- `CODEC DAI`: Refers to the `Codec` itself. It controls the workflow of the `Codec` and can be simply understood as the driver for the `Codec`.
- `DAI LINK`: Binds the `CPU DAI` and the `CODEC DAI`.
- `PLATFORM`: Specifies the platform driver on the CPU side, typically a `DMA` driver.

### Audio Development Instructions

A complete sound card configuration consists of `CPU DAI`, `CODEC DAI`, `PLATFORM`, and `DAI LINK`. These correspond respectively to the `i2s` driver, the `codec` driver, the `dma` driver, and the sound card driver (e.g., `source/kernel/sound/soc/hobot/hobot-snd-wm8960.c`). This section uses the addition of a new **full-duplex stereo** `Codec` – `WM8960` as an example to demonstrate how to add a sound card.

### I2S Parameters

The I2S on the X3 chip has the following characteristics:
- Channel support: Supports `1/2/4/8/16` channel inputs for audio input and `1/2` channel outputs for audio output.
  
- Sampling rate support: `8k/16k/32k/48k/44.1k/64k`
  
- Sampling precision support: `8bit/16bit`
  
- Transmission protocol support: `i2s/dsp(TDM)A`
  
- Default clock in I2S master mode: mclk is 12.288M bclk is 2.048M. Under the condition that mclk remains unchanged, bclk supports frequencies of 6.144M, 4.096M, 3.072M, 2.048M, and 1.536M, dynamically adjusted based on the parameters transmitted by the application layer. The frequency adjustment strategy is located in the hobot_i2s_sample_rate_set function within `sound/soc/hobot/hobot-cpudai.c`. For supporting the 44.1k sampling rate, the closest achievable frequency without adjusting PLL is 44.11764kHz.
  
- **When I2S operates in slave mode, before any read or write operations to I2S registers are performed, it is essential to have the bclk clock signal inputted; otherwise, accessing I2S module registers will result in exceptions, causing the system to malfunction.**

For the board-level `RDM X3 Module`, there are additional restrictions:

- Only clock signals (`BCLK`, `LRCK`) for `I2S0` are exposed, and the clock signals of `I2S1` are shorted to the clock signals of `I2S0` on the core board, meaning both share the same clock.

:::danger Warning

It is crucial to remember that when I2S operates as a **Slave**, **it is mandatory** to ensure the bclk signal is present before attempting to access any I2S-related registers on the **X3** side!

:::

### Adding a New Codec

#### Adding codec driver

After obtaining the `Codec` driver code, copy it to the `source/kernel/sound/soc/codecs/` directory.

#### Adding build options

Modify the `Kconfig` and `Makefile` files in the `source/kernel/sound/soc/codecs/` directory to include the `WM8960` driver in the driver compilation process.

Add the following to `Kconfig`:

```makefile
config SND_SOC_WM8960
	tristate "Wolfson Microelectronics WM8960 CODEC"
	depends on I2C
```

Add the following to `Makefile`:

```makefile
snd-soc-wm8960-objs := wm8960.o
```

#### Enabling the driver in Kernel

```bash
Device Drivers --->
    <*> Sound card support --->
        <*> Advanced Linux Sound Architecture --->
            <*> ALSA for SoC audio support --->
                CODEC drivers -->
                    [*] Wolfson Microelectronics WM8960 CODEC
```

Save the kernel configuration.

### Modifying dts File

In the corresponding `i2c` section, add `codec` information. For instance, if `WM8960` is connected to the `i2c0` bus, add the following configuration under `i2c0`:

```c
&i2c0 {   
        status = "okay";
        #address-cells = <1>;
        #size-cells = <0>;
           
        wm8960:wm8960@0x1a{
            compatible = "wlf,wm8960";
            reg = <0x1a>;
            #sound-dai-cells = <0>;
        }; 
}
```

In the `dts` file, configure the sound card information required for the corresponding `Codec`.

`WM8960` is used in a setup with one master (playback) and one slave (record) on the `RDK X3 Module`, with both `I2S` instances sharing the same clock.

```c
&i2s0 {
    status = "okay";
    #sound-dai-cells = <0>;
    clocks = <&i2s0_mclk>, <&i2s0_bclk>;
    clock-names = "i2s-mclk", "i2s-bclk";
    ms = <1>; // This attribute determines whether this i2s acts as master or slave, where 1 means master
    share_clk = <1>;  // This attribute is required for the shared clock to take effect
    bclk_set = <1536000>; // Duplex mode
};

&i2s1 {
    status = "okay";
    #sound-dai-cells = <0>;
    clocks = <&i2s1_mclk>, <&i2s1_bclk>;
    clock-names = "i2s-mclk", "i2s-bclk";
    ms = <4>; // This attribute determines whether this i2s acts as master or slave, where 4 means slave
    share_clk = <1>; // This attribute is required for the shared clock to take effect
    bclk_set = <1536000>;
};



&snd6 {
    status = "okay";
    model = "hobotsnd6"; // Sound card name
    work_mode = <0>; /*0: simple mode; 1: duplex mode*/
    dai-link@0 {
        dai-format = "i2s"; // Operation mode
        // bitclock-master; // Slave mode, does not send bclk
        // frame-master; // Slave mode, does not send lrck
        //frame-inversion; // Clock polarity
        link-name = "hobotdailink0";
        cpu {
            sound-dai = <&i2s1>; // The i2s bound to this path, determined based on your hardware configuration
        };
        codec {
            sound-dai = <&wm8960>; // Corresponding to the name set earlier for i2c0
        };
        platform {
            sound-dai = <&i2sidma1>; // Corresponding to sound-dai, using i2sidma1 when using i2s1
        };
    };
    dai-link@1 {
        dai-format = "i2s"; // Operation mode
        bitclock-master; // Master mode, sends bclk
        frame-master; // Master mode, sends lrck
        //frame-inversion;
        link-name = "hobotdailink1";
        cpu {
            sound-dai = <&i2s0>; // The i2s bound to this path, determined based on your hardware configuration
        };
        codec {
            sound-dai = <&wm8960>; // Corresponding to the name set earlier for i2c0
        };
        platform {
            sound-dai = <&i2sidma0>; // Corresponding to sound-dai, using i2sidma0 when using i2s0
        };
    };
};
```

### Writing DAI LINK Driver

This part of the driver is quite generic, and you can refer to `source/kernel/sound/soc/hobot/hobot-snd-wm8960.c` for guidance when writing it. Be sure to match your implementation with the device tree configuration.

After completing the driver, add it to both the `Makefile` and `Kconfig`, and then include this driver in the kernel build process to generate the complete driver module.

## Debugging Instructions

### Sound Card Debugging

#### Loading Driver Modules

```bash
modprobe snd-soc-wm8960
modprobe hobot-i2s-dma
modprobe hobot-cpudai
modprobe hobot-snd-wm8960
```

#### Confirming Successful Registration of Sound Card Driver

```bash
root@ubuntu:~# cat /proc/asound/cards
0 [hobotsnd6      ]: hobotsnd6 - hobotsnd6
                     hobotsnd6
root@ubuntu:~# ls -l /dev/snd/
total 0
drwxr-xr-x  2 root root       60 Mar 28 01:54 by-path
crw-rw----+ 1 root audio 116,  2 Mar 28 01:54 controlC0
crw-rw----+ 1 root audio 116,  4 Mar 28 01:54 pcmC0D0c
crw-rw----+ 1 root audio 116,  3 Mar 28 01:54 pcmC0D0p
crw-rw----+ 1 root audio 116,  6 Mar 28 01:54 pcmC0D1c
crw-rw----+ 1 root audio 116,  5 Mar 28 01:54 pcmC0D1p
crw-rw----+ 1 root audio 116, 33 Mar 28 01:54 timer
```
#### Using the Sound Card

Please refer to [Audio Adapter Board Usage](../../hardware_development/rdk_x3_module/audio_board.md) for instructions.

### Common Debugging Methods

- Check `i2s` registers. When recording or playback fails, it's usually necessary to first examine whether the register configuration is correct to identify the problem.

View `i2s0` register configuration:
```bash
cat /sys/devices/platform/soc/a5007000.i2s/reg_dump
```

View `i2s1` register configuration:
```bash
cat /sys/devices/platform/soc/a5008000.i2s/reg_dump
```

- Check `codec` register configurations. Please consult your supplier for the relevant method.

- Use an oscilloscope to measure audio signals including `mclk/bclk/lrck/data`. Confirm that the clock signal frequencies `bclk/lrck` match the configured sample rate. If they do not match, audio may play back too fast or slow.
