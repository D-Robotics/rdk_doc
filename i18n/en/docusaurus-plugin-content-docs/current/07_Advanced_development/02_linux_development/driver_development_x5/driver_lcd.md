---
sidebar_position: 3
---

# LCD Driver Debugging Guide

## 1. Specifications

The MIPI-DSI interface specifications for X5 are as follows:

- Supports up to four data lanes
- The maximum rate for each lane is <font color="red">2.5Gbps</font>
- Supports <font color="red">Non-Burst-Sync-Pulse</font> synchronous pulse mode, <font color="red">Non-Burst-Sync-Even</font> synchronous event mode, and Burst mode
- Supports <font color="red">MIPI_DSI_CLOCK_NON_CONTINUOUS</font> non-continuous clock mode and <font color="red">MIPI_DSI_CLOCK_CONTINUOUS</font> continuous clock mode

## 2. Debugging Process

Before adding a new screen driver, the following information needs to be determined:

- The screen's timing, including hbp (horizontal back porch), hfp (horizontal front porch), hsa (horizontal sync active), vbp (vertical back porch), vfp (vertical front porch), and vsa (vertical sync active)
- The screen's initialization sequence
- The supported transmission mode of the screen, whether it is burst or non-burst
- The supported clock mode of the screen, whether it is continuous or non-continuous

Taking the `JC050HD134` screen as an example, the following information can be derived from the configuration provided by the manufacturer:

```
#define Width 720
#define Height 1280

#define VFP 20
#define VBP 20
#define VSA 4

#define HFP 32
#define HBP 20
#define HSA 20
```
### Kernel Driver Layer

In the kernel, there is a pre-debugged reference driver: `panel-atk-md0550.c`. Future panel drivers can be derived from this driver.

Copy the `panel-atk-md0550.c` file from `kernel/drivers/gpu/drm/panel/` and rename it as `panel-jc-050hd134.c`.

The following modifications are based on `panel-jc-050hd134.c`.

Modify the `name` attribute in `panel_simple_dsi_driver` to `panel-jc-050hd134`:


```
static struct mipi_dsi_driver panel_simple_dsi_driver = {
	.driver =
		{
			.name		= "panel-jc-050hd134",
			.of_match_table = dsi_of_match,
		},
	.probe	  = panel_simple_dsi_probe,
	.remove	  = panel_simple_dsi_remove,
	.shutdown = panel_simple_dsi_shutdown,
};
```

Modify the `drm_display_mode` structure:

```
static const struct drm_display_mode jc_050hd134_mode = { // Change structure name to jc_050hd134_mode
    .clock       = 65000, // Pixel clock in kHz, calculated by: fps * (htotal + vtotal)
    .hdisplay    = 720, // Horizontal display area width
    .hsync_start = 720 + 32, // hdisplay + hfp (Horizontal Front Porch)
    .hsync_end   = 720 + 32 + 20, // hsync_start + hsa (Horizontal Sync Active)
    .htotal      = 720 + 32 + 20 + 20, // hsync_end + hbp (Horizontal Back Porch)
    .vdisplay    = 1280, // Vertical display area height
    .vsync_start = 1280 + 20, // vdisplay + vfp (Vertical Front Porch)
    .vsync_end   = 1280 + 20 + 4, // vsync_start + vsa (Vertical Sync Active)
    .vtotal      = 1280 + 20 + 4 + 20, // vsync_end + vbp (Vertical Back Porch)
    .flags       = DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_NVSYNC, // Polarity flags
};
```

Modify the `panel_desc_dsi` structure as follows:
```
static const struct panel_desc_dsi jc_050hd134 = { // Rename structure to jc_050hd134
    .desc =
    {
        .modes      = &jc_050hd134_mode, // Points to the timing structure
        .num_modes  = 1, // Number of timing modes for this panel, typically 1
        .bpc        = 8, // Bits per color, RGB888 means 8 bits per color, RGB666 means 6 bits per color
        .size = // Physical size of the visible area in mm
        {
            .width  = 62,
            .height = 110,
        },
        .connector_type = DRM_MODE_CONNECTOR_DSI, // DSI connector type
    },
    .flags   = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_SYNC_PULSE, // DSI mode flags, indicating video mode and non-burst mode
    // For more flags, refer to kernel/include/drm/drm_mipi_dsi.h
    .format  = MIPI_DSI_FMT_RGB888, // Panel color format
    .lanes   = 4, // Number of data lanes
};
```

Modify the `dsi_of_match` to prepare for binding the device tree:

```
static const struct of_device_id dsi_of_match[] = {
    {.compatible = "jc-050hd134", .data = &jc_050hd134}, // Bind the device tree compatible string to the panel description
    {
        /* sentinel */
    }
};
MODULE_DEVICE_TABLE(of, dsi_of_match); // Register the device table for device tree matching
```

Modify the `panel_simple_dsi_init` function. This function essentially calls the `dsi_dcs_write_seq` function to write the MIPI initialization sequence to the panel.

If you receive the initialization parameters from the screen manufacturer in the following format:

```
DSI_CMD(0x04);DSI_PA(0xB9);
DSI_PA(0xF1);
DSI_PA(0x12);
DSI_PA(0x83);

DSI_CMD(0x1C);DSI_PA(0xBA);
DSI_PA(0x33); // 1
DSI_PA(0x81); // 2
DSI_PA(0x05); // 3
DSI_PA(0xF9); // 4
DSI_PA(0x0E); // 5
DSI_PA(0x0E); // 6
DSI_PA(0x20); // 7
DSI_PA(0x00); // 8
DSI_PA(0x00); // 9
DSI_PA(0x00); //10
DSI_PA(0x00); //11
DSI_PA(0x00); //12
DSI_PA(0x00); //13
DSI_PA(0x00); //14
DSI_PA(0x44); //15
DSI_PA(0x25); //16
DSI_PA(0x00); //17
DSI_PA(0x91); //18
DSI_PA(0x0A); //19
DSI_PA(0x00); //20
DSI_PA(0x00); //21
DSI_PA(0x02); //22
DSI_PA(0x4F); //23
DSI_PA(0xD1); //24
DSI_PA(0x00); //25
DSI_PA(0x00); //26
DSI_PA(0x37); //27

......
```

Convert to Driver Code
```
dsi_dcs_write_seq(dsi, 0xb9, 0xF1, 0x12, 0x83);
dsi_dcs_write_seq(dsi, 0xBA, 0x33, 0x81, 0x05, 0xF9, 0x0E, 0x0E, 0x20, 0x00, 0x00, 0x00,
			  0x00, 0x00, 0x00, 0x00, 0x44, 0x25, 0x00, 0x91, 0x0A, 0x00, 0x00, 0x02,
			  0x4F, 0xD1, 0x00, 0x00, 0x37);
......
```

You can see that in the two sets of initialization sequences provided by the manufacturer, the first value written is the length of the initialization sequence itself. When converting this into driver code, you don't need to include it manually as it will be calculated automatically during the writing process.

If the initialization sequence provided by the screen manufacturer looks like the following:

```
panel-init-sequence-zero = [
 39 00 04 B9 FF 83 94
 15 00 02 36 01
 39 00 07 BA 63 03 68 6B B2 C0
 39 00 0B B1 48 12 72 09 32 54 71 71 57 47
......
]
```

Converting to Driver Code:
```
	dsi_dcs_write_seq(dsi, 0xb9, 0xff, 0x83, 0x94);
	dsi_dcs_write_seq(dsi, 0x36, 0x01);
	dsi_dcs_write_seq(dsi, 0xba, 0x63, 0x03, 0x68, 0x6b, 0xb2, 0xc0);
	dsi_dcs_write_seq(dsi, 0xb1, 0x48, 0x12, 0x72, 0x09, 0x32, 0x54, 0x71, 0x71, 0x57, 0x47);
```
Here, we will explain the conversion using the first sequence `39 00 04 B9 FF 83 94` as an example:

- `39` indicates that this sequence is written using the MIPI_DSI_DCS_LONG_WRITE (0x39) data type. When converting to driver code, you do not need to specify this manually; it will be automatically generated based on the length of the sequence.
- `00` indicates that after this write operation, the system will sleep for 0ms, which can be implemented using the `msleep(ms)` function.
- `04` marks the length of the write sequence, which will be automatically generated, so you do not need to fill it in.
- The remaining data represents the initialization sequence itself.

For displays that require specific power-on-reset timing, you can modify the GPIO behavior in the `panel_simple_prepare` and `panel_simple_unprepare` functions, which correspond to the initialization and de-initialization states respectively.

To apply the changes, modify the following files:
- `kernel/drivers/gpu/drm/panel/Kconfig`
- `kernel/drivers/gpu/drm/panel/Makefile`

Kconfig:
```
config DRM_PANEL_JC_050HD134
	tristate "JC 050HD134 panel"
	depends on OF
	depends on DRM_MIPI_DSI
	depends on BACKLIGHT_CLASS_DEVICE
	select VIDEOMODE_HELPERS
	help
	  Say Y here if you want to enable support for the JC050HD134
	  panel with 720x1280 resolution. This panel support
	  MIPI DSI interface.
```

Makefile:
```
obj-$(CONFIG_DRM_PANEL_JC_050HD134) += panel-jc-050hd134.o
```

### Kernel Device Tree Level

At the board-level device tree, here we take `kernel/arch/arm64/boot/dts/hobot/x5-evb.dts` as an example.

Add the following properties to the `mipi_dsi0` node:

```
&mipi_dsi0 {

    status = "okay";
    dsi_panel0@0 {
        compatible = "jc-050hd134"; // This should match dsi_of_match
        reg = <0>;

        pinctrl-names = "default";
        pinctrl-0 = <&lsio_gpio0_14>; // Related to the reset pin
        reset-gpios = <&ls_gpio0_porta 14 GPIO_ACTIVE_HIGH>; // Related to the reset pin. The active level should be decided based on your screen's specifications.
        backlight = <&dsi_backlight>; // Backlight related

        port {
            panel_in: endpoint {
                remote-endpoint =
                    <&mipi_dsi_out>;
            };
        };
    };

};

```

添加dsi_backlight节点：
```
&dsi_backlight {
    status = "okay";
    pwms = <&lpwm1 1 1000000>; // Related to hardware PCB, check where the backlight PWM signal is sourced from. This indicates the backlight signal comes from LPWM1_1.
    // For other properties and details, please refer to kernel/Documentation/devicetree/bindings/leds/backlight/pwm-backlight.yaml
};

```

## 3. Compile

Please refer to section 7.2.1 to set up the build environment and successfully compile the image before proceeding with the following steps!

Execute `./mk_kernel.sh menuconfig` to enter the kernel configuration menu, and follow the path below to enter the Panels compilation options:

```
Device Drivers  --->
    Graphics support  --->
        Display Panels  --->
```
Find the JC 050HD134 panel, press the spacebar to compile it as a module. Then, save the configuration and exit.

Execute `./mk_kernel.sh` to compile the kernel. After compilation, you can find the corresponding driver file `panel-jc-050hd134.ko` in the `deploy/kernel/modules/lib/modules/6.1.83/kernel/drivers/gpu/drm/panel` directory.

Execute the following commands to generate the required `.deb` packages:
- `./mk_deb.sh hobot-dtb`
- `./mk_deb.sh hobot-boot`
- `./mk_deb.sh hobot-kernel-headers`

The resulting `.deb` packages, such as `hobot-dtb*.dtb`, `hobot-boot*.dtb`, and `hobot-kernel-headers*.dtb`, will be located in the `deploy/deb_pkgs/` directory. 

These three `.deb` packages can be directly transferred to the device to update the device tree, kernel, driver files, and kernel headers. Alternatively, you can execute `./pack_image.sh` to include them in the filesystem.

## 4. Testing

After powering on the RDK X5, the display-related drivers will be loaded, and the Ubuntu desktop will appear. To enable the JC-050HD134 panel, you need to edit the `/etc/init.d/S70loadko` script. Add the `modprobe panel-jc-050hd134` command before `modprobe vio_n2d`.

Since the RDK X5 system defaults to HDMI output, you need to switch to the MIPI DSI display mode by executing the following command:

```bash
mv /etc/X11/xorg.conf.d/xorg_dsi_ignore.conf /etc/X11/xorg.conf.d/xorg_dsi_ignore.conf.disable
mv /etc/X11/xorg.conf.d/xorg_hdmi_ignore.conf.disable /etc/X11/xorg.conf.d/xorg_hdmi_ignore.conf
```
You can also choose the display output method using `raspi-config`. For details, refer to the [Display Choose DSI or HDMI](../../../System_configuration/srpi-config#display-options) section.

After rebooting the device, you should see the system desktop on the LCD screen.

If the LCD does not display anything, please verify the following steps:

1. Check if the screen has backlight or a control chip and whether the corresponding driver has been loaded successfully.

2. Confirm whether the DRM and MIPI drivers have loaded correctly by running the command:

```
root@ubuntu:~# dmesg | grep drm
[    6.717478] systemd[1]: Starting Load Kernel Module drm...
[   13.676055] vs-drm 3e000000.disp_apb:display-subsystem: bound 3e080000.vs-sif (ops sif_component_ops [vs_drm])
[   13.676374] vs-drm 3e000000.disp_apb:display-subsystem: bound 3e000000.dc8000Nano (ops dc_component_ops [vs_drm])
[   13.676514] vs-drm 3e000000.disp_apb:display-subsystem: bound 3e010000.bt1120 (ops bt1120_component_ops [vs_drm])
[   13.676553] vs-drm 3e000000.disp_apb:display-subsystem: bound 3e000000.disp_apb:bt1120_bridge (ops bt1120_bridge_component_ops [vs_drm])
[   13.676595] vs-drm 3e000000.disp_apb:display-subsystem: bound 3e000000.disp_apb:bt1120_bridge_wb (ops bt1120_bridge_component_ops [vs_drm])
[   13.676710] vs-drm 3e000000.disp_apb:display-subsystem: bound 3e060000.mipi_dsi0 (ops dsi_component_ops [vs_drm])
[   13.676834] vs-drm 3e000000.disp_apb:display-subsystem: bound 3e000000.disp_apb:dsi-encoder (ops encoder_component_ops [vs_drm])
[   13.676940] vs-drm 3e000000.disp_apb:display-subsystem: bound 3e000000.disp_apb:hdmi-encoder (ops encoder_component_ops [vs_drm])
[   13.678810] [drm] Initialized vs-drm 1.0.0 20191101 for 3e000000.disp_apb:display-subsystem on minor 0
```
1.The message `vs-drm 3e000000.disp_apb:display-subsystem: bound 3e060000.mipi_dsi0 (ops dsi_component_ops [vs_drm])` indicates that the MIPI DSI interface has been successfully loaded and initialized.

2.The message `[drm] Initialized vs-drm 1.0.0 20191101 for 3e000000.disp_apb:display-subsystem on minor 0` indicates that the display driver has been successfully loaded.

3.To check the status of the connectors, run the following command:`modetest -M vs-drm -c`
```
root@ubuntu:~# modetest -M vs-drm -c
Connectors:
id      encoder status          name            size (mm)       modes   encoders
73      72      connected       DSI-1           800x480         1       72
  modes:
        index name refresh (Hz) hdisp hss hse htot vdisp vss vse vtot
  #0 800x480 60.00 800 890 892 950 480 487 489 510 29070 flags: nhsync, nvsync; type: preferred, driver
  props:
        1 EDID:
                flags: immutable blob
                blobs:

                value:
        2 DPMS:
                flags: enum
                enums: On=0 Standby=1 Suspend=2 Off=3
                value: 0
        5 link-status:
                flags: enum
                enums: Good=0 Bad=1
                value: 0
        6 non-desktop:
                flags: immutable range
                values: 0 1
                value: 0
        4 TILE:
                flags: immutable blob
                blobs:

                value:
```

4. **Test with `modetest` command**

Use the following command to stop the `lightdm` desktop:`sudo systemctl stop lightdm`

Then use the modetest command to perform the test:`modetest -M vs-drm -a -s 73@31:800x480 -P 33@31:800x480@NV12`

If everything is successful, the connected screen will light up and display the following pattern:

![image-20220518111319607](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/02_linux_development/driver_development_x5/screenshot-20241125-162524.png)

## 5. Remarks

Currently, the RDK X5 supports 7 models of Waveshare LCD screens. For usage instructions, refer to the [Display Screen Usage](../../hardware_development/rdk_x5/display) section.

The drivers have been integrated into the kernel:

```
kernel/drivers/gpu/drm/panel/panel-wh-cm480.c
kernel/drivers/gpu/drm/panel/panel-waveshare-dsi.c
```

Device tree modifications are implemented through `dtoverlay`. For more details, refer to `source/hobot-display`.

You can also follow this approach to support additional LCD screens.

