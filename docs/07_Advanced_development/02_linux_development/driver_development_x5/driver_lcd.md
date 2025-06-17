---
sidebar_position: 11
---

# LCD驱动调试指南

## 1.规格参数

X5的MIPI-DSI接口规格如下：

- 最大支持四路数据通路
- 每一路最高速率为<font color="red">2.5Gbps</font>
- 支持<font color="red">Non-Burst-Sync-Pulse</font>同步脉冲模式和<font color="red">Non-Burst-Sync-Even</font>同步事件模式以及Burst突发模式
- 支持<font color="red">MIPI_DSI_CLOCK_NON_CONTINUOUS</font>非连续时钟模式和<font color="red">MIPI_DSI_CLOCK_CONTINUOUS</font>连续时钟模式

## 2.调试流程
在添加新屏幕驱动之前，需要确定以下信息：

屏幕的时序，即hbp、hfp、hsa、vbp、vfp、vsa

屏幕的初始化序列

屏幕支持的传输模式，是burst还是non-burst

屏幕支持的时钟模式，是continuous还是non-continuous

以`JC050HD134`这款屏幕为例，从厂家提供的配置中得知以下信息：

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
### Kernel 驱动层面
kernel里面有一份已经调试好的参考代码：panel-atk-md0550.c，后续的屏幕驱动可以从这份驱动上面派生出来。

将kernel/drivers/gpu/drm/panel/panel-atk-md0550.c拷贝一份，并重命名为panel-jc-050hd134.c。

以下修改都是基于panel-jc-050hd134.c。

修改的panel_simple_dsi_driver的name属性字段为panel-jc-050hd134：
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

修改drm_display_mode结构体：
```
static const struct drm_display_mode jc_050hd134_mode = { // 修改结构体名字为 jc_050hd134_mode
	.clock	     = 65000, //像素时钟 单位为khz，计算公式为：fps * (htotal + vtotal)
	.hdisplay    = 720, //可视区域宽
	.hsync_start = 720 + 32, //hdisplay + hfp
	.hsync_end   = 720 + 32 + 20, //hsync_start + hsa
	.htotal	     = 720 + 32 + 20 + 20, //hsync_end + hbp
	.vdisplay    = 1280, // 可视区域高
	.vsync_start = 1280 + 20, //vdisplay + vfp
	.vsync_end   = 1280 + 20 + 4, // vsync_start + vsa
	.vtotal	     = 1280 + 20 + 4 + 20, // vsync_end + vbp
	.flags	     = DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_NVSYNC, //极性
};
```

修改panel_desc_dsi结构体：
```
static const struct panel_desc_dsi jc_050hd134 = { //结构体重命名为 jc_050hd134
	.desc =
		{
			.modes	   = &jc_050hd134_mode, // 指向时序结构体
			.num_modes = 1, // 这个面板有多少组时序，一般来讲只有一组
			.bpc	   = 8, // 每种色彩占多少bit？RGB888 即每个颜色占8bit，RGB666 即每个颜色占6bit
			.size = // 可视区域的物理大小，以mm（毫米）为单位
				{
					.width	= 62,
					.height = 110,
				},
			.connector_type = DRM_MODE_CONNECTOR_DSI,
		},
	.flags	= MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_SYNC_PULSE, // DSI模式标志位，这里表明该面板工作在video模式下，并且是non-burst模式
    //更多标志位，请参见kernel/include/drm/drm_mipi_dsi.h
	.format = MIPI_DSI_FMT_RGB888,// 面板色彩类型
	.lanes	= 4, //数据lane数
};
```

修改dsi_of_match，为了绑定设备树做准备：
```
static const struct of_device_id dsi_of_match[] = {
	{.compatible = "jc-050hd134", .data = &jc_050hd134},
	{
		/* sentinel */
	}};
MODULE_DEVICE_TABLE(of, dsi_of_match);
```

修改panel_simple_dsi_init函数，这个函数实际上调用dsi_dcs_write_seq这个函数往面板里面写入mipi初始化序列。

如果您从屏幕厂商拿到的初始化参数形如下面这种：
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

转换为驱动代码：
```
dsi_dcs_write_seq(dsi, 0xb9, 0xF1, 0x12, 0x83);
dsi_dcs_write_seq(dsi, 0xBA, 0x33, 0x81, 0x05, 0xF9, 0x0E, 0x0E, 0x20, 0x00, 0x00, 0x00,
			  0x00, 0x00, 0x00, 0x00, 0x44, 0x25, 0x00, 0x91, 0x0A, 0x00, 0x00, 0x02,
			  0x4F, 0xD1, 0x00, 0x00, 0x37);
......
```

可以看到厂商提供的两组初始化序列里，第一个写入的是这组初始化序列的长度，转换为驱动代码时不用带上，会在写入时自动计算。

如果您从屏幕厂商拿到的初始化序列形如下列这种：
```
panel-init-sequence-zero = [
 39 00 04 B9 FF 83 94
 15 00 02 36 01
 39 00 07 BA 63 03 68 6B B2 C0
 39 00 0B B1 48 12 72 09 32 54 71 71 57 47
......
]
```

转换为驱动代码：
```
	dsi_dcs_write_seq(dsi, 0xb9, 0xff, 0x83, 0x94);
	dsi_dcs_write_seq(dsi, 0x36, 0x01);
	dsi_dcs_write_seq(dsi, 0xba, 0x63, 0x03, 0x68, 0x6b, 0xb2, 0xc0);
	dsi_dcs_write_seq(dsi, 0xb1, 0x48, 0x12, 0x72, 0x09, 0x32, 0x54, 0x71, 0x71, 0x57, 0x47);
```

在这里，我们以第一组序列39 00 04 B9 FF 83 94为例，进行解释：

39 表示本组初始化序列是按照MIPI_DSI_DCS_LONG_WRITE (0x39)数据类型写入，转换为驱动代码时不需要填写，会根据本组参数长度自动生成
00 表示本次写入之后，睡眠0ms，可以调用msleep(ms)函数实现
04 标识本次写入序列的长度，会自动生成不用填写
后续的就是初始化序列了

对于上电-复位时序有要求的屏幕，可以修改panel_simple_prepare和panel_simple_unprepare这两个函数中的gpio行为，这两个函数分别对应初始化和反初始化状态。

修改kernel/drivers/gpu/drm/panel/Kconfig和kernel/drivers/gpu/drm/panel/Makefile

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

### Kernel 设备树层面

在板级设备树，此处以kernel/arch/arm64/boot/dts/hobot/x5-evb.dts为例

在mipi_dsi0节点添加下列属性：
```
&mipi_dsi0 {

	status = "okay";
	dsi_panel0@0 {
		compatible = "jc-050hd134";//此处与 dsi_of_match 一致
		reg = <0>;

		pinctrl-names = "default";
		pinctrl-0 = <&lsio_gpio0_14>; // 与复位管脚相关
		reset-gpios = <&ls_gpio0_porta 14 GPIO_ACTIVE_HIGH>; // 与复位管脚相关 与什么电平有效需要根据您的屏幕来决定
		backlight = <&dsi_backlight>; // 背光相关

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
	pwms = <&lpwm1 1 1000000>; //与硬件PCB有关，检查您的背光方波信号来源于哪里 这里表示背光信号来源于LPWM1_1
    // 其余属性和信息，请参考 kernel/Documentation/devicetree/bindings/leds/backlight/pwm-backlight.yaml
};
```

## 3.编译
请先参考7.2.1章搭建好编译环境，并能成功编译出镜像之后再做下面操作！

执行`./mk_kernel.sh menuconfig`进入内核的配置菜单，按照以下路径进入Panels编译选项：
```
Device Drivers  --->
    Graphics support  --->
        Display Panels  --->
```
找到JC 050HD134 panel，按下空格，将其作为模块编译。然后保存配置并退出。

执行`./mk_kernel.sh`编译内核，在`deploy/kernel/modules/lib/modules/6.1.83/kernel/drivers/gpu/drm/panel`目录下可以找到对应的驱动文件`panel-jc-050hd134.ko`

执行`./mk_deb.sh hobot-dtb`,`./mk_deb.sh hobot-boot`,`./mk_deb.shhobot-kernel-headers`在`deploy/deb_pkgs/`目录下得到`hobot-dtb*.dtb`，`hobot-boot*.dtb`，`hobot-kernel-headers*.dtb`

这三个deb包可以直接传给设备安装，更新设备上的设备树，内核及驱动文件，内核头文件，也可以执行`./pack_image.sh`编进文件系统中

## 4.测试

RDK X5上电后会加载显示相关驱动，显示ubuntu桌面，需要打开`/etc/init.d/S70loadko`，把`modprobe panel-jc-050hd134`加到`modprobe vio_n2d`前

由于RDK X5 系统默认采用HDMI输出，需要通过命令切换到MIPI DSI显示方式。
```bash
mv /etc/X11/xorg.conf.d/xorg_dsi_ignore.conf /etc/X11/xorg.conf.d/xorg_dsi_ignore.conf.disable
mv /etc/X11/xorg.conf.d/xorg_hdmi_ignore.conf.disable /etc/X11/xorg.conf.d/xorg_hdmi_ignore.conf
```
也可以通过srpi-config来选择输出方式，可以参考 [Dsiplay Chose DSI or HDMI](../../../System_configuration/srpi-config#display-options) 章节

重启设备，就可以在LCD屏上看到系统桌面桌面了

如果LCD未显示，请逐步确认以下内容

1，该款屏幕是否有背光或者控制芯片，相关的驱动有没有加载成功；

2，确认DRM，MIPI驱动是否加载成功，使用`dmesg | grep drm`；
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

`vs-drm 3e000000.disp_apb:display-subsystem: bound 3e060000.mipi_dsi0 (ops dsi_component_ops [vs_drm])`表示mipi_dsi加载成功。

`[drm] Initialized vs-drm 1.0.0 20191101 for 3e000000.disp_apb:display-subsystem on minor 0`表示显示驱动加载成功。

3，执行`modetest -M vs-drm -c`查看connectors的状态：
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

4，使用`modetest`输出测试图镜像测试命令进行测试

使用`sudo systemctl stop lightdm`关闭lightdm桌面

使用`modetest -M vs-drm -a -s 73@31:800x480 -P 33@31:800x480@NV12`命令进行测试

如果一切顺利，连接的屏幕将会亮起并显示下图的pattern：

![image-20220518111319607](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/02_linux_development/driver_development_x5/screenshot-20241125-162524.png)

## 5.备注
目前，RDK X5已经支持7款微雪 LCD屏幕，使用方法可以参考 [显示屏使用](../../hardware_development/rdk_x5/display) 章节

驱动已经集成到内核中
```
kernel/drivers/gpu/drm/panel/panel-wh-cm480.c
kernel/drivers/gpu/drm/panel/panel-waveshare-dsi.c
```
设备树的修改通过dtoverlay来实现，详情前参考`source/hobot-display`

您也可以参考这种方式支持更多LCD屏幕
