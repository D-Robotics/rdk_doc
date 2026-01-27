---
sidebar_position: 2
---

# Camera点亮

## HBN sensor 点亮

### 范围

本章节概述了 RDK-S100 camera bring up 的过程，用于帮助读者快速了解并掌握
RDK-S100 camera 框架，如何快速的新增 camera 配置，并点亮 camera。

该部分内容以 RDK-S100 开发板 + imx219 camera
模组为例，进行配置讲解，其他硬件平台或者 camera 模组以实际情况为准。
![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/03_multimedia_development/02_S100/camera_bringup/camera_bringup_01.png)

### 准备工作

硬件资源：RDK-S100 开发板、camera 模组。

软件资源：系统 SDK、camera 驱动源码、sensor datasheet、sensor 的 initialize
settings 等。

RDK-S100 开发板 camera 相关硬件资源如下：

| RDK-S100                       | MIPI host    | I2C   | gpio_en                          | gpio_lpwm_mclk                           | 其他                                                            |
|--------------------------------|------------------|-------|-----------------------------------|-----------------------------------|-----------------------------------------------------------------|
| RX0<br /> 可接 imx219 模组 | **0**<br />  4 lane | **1** | SPI1_CSN0<br />  gpio_number:502 | 可通过拨码开关进行选择<br />  • LPWM0_DOUT0<br />  gpio_number:456<br />  • mclk 24Mhz  | 注意： imx219 模组本身外接 24M 晶振，所以不需要 SOC 端输出 mclk |
| RX1<br /> 可接 imx219 模组 | **1**<br />   4 lane | **2** | SD_WPROT<br />  gpio_number:494 | 可通过拨码开关进行选择<br />  • LPWM0_DOUT1<br /> gpio_number:457<br />  • mclk 24Mhz  | 注意： imx219 模组本身外接 24M 晶振，所以不需要 SOC 端输出 mclk |
| RX4<br /> 用于接 serdes    | **4**<br />  4 lane | **3** | 暂无介绍                          | 暂无介绍                          | 解串器 max96712, addr: 0x29 poc: max20087, addr: 0x28           |

硬件连接示意图：

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/03_multimedia_development/02_S100/camera_bringup/camera_bringup_02.png)


### 添加新 sensor 点亮步骤

RDK-S100 平台进行**新硬件**和**新 camera** 适配时，需要修改平台设备树
dts，camera 驱动库及相关配置文件即可，系统库一般无需改动。

#### dts 修改

##### sensor gpio 配置

确保新硬件使用的 sensor gpio 在 drobot-s100-pinctrl.dtsi --\> pinctrl_video --\>
video_gpio节点中有配置，这样在开机启动时，系统才会将对应的 pin 设置为
gpio，用户程序方可以操作 pin。

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/03_multimedia_development/02_S100/camera_bringup/camera_bringup_03.png)

vcon 是 RDK-S100 camera 用于管理 sensor 硬件相关的 dts 节点，如果 sensor
需要对应的时序才能正常启动，则需要在该节点中配置对应的
gpio。请根据硬件连接的实际情况配置，该相关信息可以从原理图及 pin list 中获取。

```c
// dts： 在对应 vcon node 中设置 gpio，注意 vcon 端口号与 mipi rx 端口号 一 一对应
// vcon0 -- RX0
// ....
// vcon3 -- RX3
&vin_vcon0 {
        bus = <2>;
        gpio_poc = <0>;
        gpio_des = <0>;
        sensor_err = <0>;
        //gpio_oth = <444 445>; // imx219 无需配置，所以这里是注释掉，为空
        lpwm_chn = <0 1 2 3>;
        rx_phy = <2 0>;
};
```

##### sensor i2c 配置

I2C bus number 需要在 dts vcon 中与 MIPI RX
端口进行绑定，请根据硬件连接的实际情况配置，该相关信息可以从原理图中获取。

```c
// 在对应 vcon 中设置 i2c bus，如 RX0 设置 I2C2
&vin_vcon0 {
        bus = <2>;
        gpio_poc = <0>;
        gpio_des = <0>;
        sensor_err = <0>;
        lpwm_chn = <0 1 2 3>;
        rx_phy = <2 0>;
};
```

##### mclk 配置

RDK-S100 底座硬件暂时不支持 SOC 输出的 mclk 连接到 sensor
模组，目前只支持外带晶振的模组。

##### dts 修改验证

一般 dts 配置正确，硬件正确连接后，保证 sensor 供电及 mclk 正常 ，便可以使用 i2cdetect 检测到模组的 i2c 地址。
通过 echo 命令进行控制 sensor 上电或者 reset （注：该说明使用 imx219模组无需操作 gpio）

```c
echo 502 > /sys/class/gpio/export
echo out > /sys/class/gpio/gpio502/direction
echo 1 > /sys/class/gpio/gpio502/value
echo 502 > /sys/class/gpio/unexport
```

使用 i2cdetect 检测 sensor i2c地址。如果检测到正确的地址，如下图所示，则表示 dts 配置正确，否则需要检查dts 配置。

| ![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/03_multimedia_development/02_S100/camera_bringup/camera_bringup_04.png) | ![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/03_multimedia_development/02_S100/camera_bringup/camera_bringup_05.png) |
|--------------------------------------|--------------------------------------|


#### sensor 驱动文件添加

不同厂家的 sensor，都会搭配风格各异的 driver 和 setting。因此需要将原厂 sensor
驱动，转换成 RDK-S100 camera 驱动代码，并编译生成 so 库，然后将 so
库拷贝到设备的 /usr/hobot/lib/sensor/ 目录下。**需要说明的是，在 mipi start
之前，必须保证 sensor 没有开流。**

系统 SDK 目录 hobot-camera/drivers/sensor 下提供了 sensor 驱动模板文件
imx219_utility.c 以及适配过的其他 sensor 驱动，当添加新 camera sensor
支持时，可以仿照该部分文件进行修改。

```c
 #ifdef CAMERA_FRAMEWORK_HBN
 SENSOR_MODULE_F(imx219, CAM_MODULE_FLAG_A16D8);
 sensor_module_t imx219 = {
         .module = SENSOR_MNAME(imx219),
 #else
 sensor_module_t imx219 = {
         .module = "imx219",
 #endif
         .init = sensor_init,
         .start = sensor_start,
         .stop = sensor_stop,
         .deinit = sensor_deinit,
         .aexp_gain_control = sensor_aexp_gain_control,
         .aexp_line_control = sensor_aexp_line_control,
         .power_on = sensor_poweron,
         .power_off = sensor_poweroff,
         .userspace_control = sensor_userspace_control,
 };
```

如上代码所示，RDK-S100 camera 框架下的 sensor 驱动接口包含在 sensor_module_t
的结构体中，文件名、结构体名和 module 字段要统一，例如文件名为
imx219_utility.c，那么结构体名和 module 字段要统一为 imx219。对于新 sensor
点亮，下列函数需要用户自行实现：

• init：sensor 初始化、setting 下发

• deinit：sensor 去初始化

• start：sensor 开流

• stop：sensor 关流

• power on: sensor 上电

• power off: sensor 下电

• aexp_gain_control: sensor gain 增益控制

• aexp_line_control: sensor line 曝光控制

• userspace_control: 用户回调功能开启控制

对于 3A
控制，系统支持驱动注册和应用层回调两种方式，默认使用应用层回调函数的方式，接口定义如下：

| 函数              | 功能               | 传入参数                                                                                                                                                                                 |
|-------------------|--------------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| aexp_gain_control | sensor 增益控制    | info：sensor 总线信息 mode：senosr 运行模式；linear/hdr/pwl again：sensor again 参数，最大4个 dgain：sensor dgain 参数，最大4个 gain_num：sensor gain 参数个数                           |
| aexp_line_control | sensor 曝光控制    | info：sensor 总线信息 mode：senosr 运行模式；linear/hdr/pwl line：sensor line 参数，最大4个 line_num：sensor line 参数个数                                                               |
| awb_control       | sensor 端 awb 控制 | info：sensor 总线信息； mode：senosr 运行模式；linear/hdr/pwl rgain：sensor rgain bgain：sensor bgain grgain：sensor rrgain gbgain：sensor gbgain                                        |
| userspace_control | hal 层各类控制开关 | port：sensor 端口号 enable：使能用户回调控制开关，默认全部关闭。 位定义： \#define HAL_LINE_CONTROL 0x00000001 \#define HAL_GAIN_CONTROL 0x00000002 \#define HAL_AWB_CONTROL 0x00000004  |

如下代码是对 sensor 驱动主要结构体的初始化，需要根据每个 sensor
的实际情况来对应填写

```c
// sensor 实际输出的宽度
turning_data->sensor_data.active_width = 1920;
// sensor 实际输出的高度
turning_data.sensor_data.active_height = 1080;

// 每秒多少曝光行，计算公式为1/每行时间或者(fps*vts)，其中vts在不同sensor的里面的名字可能有所不同，
//可能是frame_length、vts等，但含义都是每帧的所含的总共行数，
//包括有效行和blanking。lines_per_second 亦可以理解成HMAX,需要注意的是有些sensor 无HMAX 概念
turning_data.sensor_data.lines_per_second = vts * sensor_info->fps;

// 短曝光最大曝光时间(短曝光每帧最大曝光行数),短曝光每帧最大曝光行数可以通过公式
//单帧曝光时间/单行曝光时间，
//即(1/fps)/(1/lines_per_second)
turning_data.sensor_data.exposure_time_max = vts;

// a_gain最大倍数，举一个例子，turning_data.sensor_data.analog_gain_max =126，
//最大倍率计算公式就是2^(X/32)，其中X就是126，X这个值每一个sensor厂家不一样，
//需要从sensor手册或者sensor厂家查找得到；最大倍率也可以这样得到：先获取sensor最大增益，
//然后查找J5-ISP的增益表，得到对应的索引值，该索引值就是最大倍率。
turning_data.sensor_data.analog_gain_max = 109;
turning_data.sensor_data.digital_gain_max = 0;

//短曝光最小曝光时间(短曝光每帧最小曝光行数)，
//每行的曝光时间可以用公式1秒/(帧率*(有效行+blank))即1/lines_per_second推算得出
turning_data.sensor_data.exposure_time_min = 1;

//长曝光最大曝光时间(长曝光每帧最大曝光行数)
turning_data.sensor_data.exposure_time_long_max = vts;

// 填充 sensor 位宽data_width、bayer_start(RGGB pattern start (R/Gr/Gb/B))、
// bayer_pattern(RGGB/RCCC/RIrGB/RGIrB) 信息
sensor_data_bayer_fill(&turning_data.sensor_data, 10, (uint32_t)BAYER_START_R, (uint32_t)BAYER_PATTERN_RGGB);

// 填充 exposure_max_bit_width(pwl mode bits ) 信息
sensor_data_bits_fill(&turning_data.sensor_data, 12);

// setting stream ctrl
// 开流、关流
turning_data.stream_ctrl.data_length = 1;

// again lut表，fireware根据index索引lut表，查找sensor对应的寄存器值，lut表区分a_gain/d_gain，
// lut表格大小：again_lut[again_control_num][256], dgain_lut[dgain_control_num][256]
turning_data.normal.again_lut = malloc(256 * sizeof(uint32_t));
if (turning_data.normal.again_lut != NULL)
{
    memset(turning_data.normal.again_lut, 0xff, 256 * sizeof(uint32_t));
    memcpy(turning_data.normal.again_lut, imx219_gain_lut,
           sizeof(imx219_gain_lut));
}

turning_data.normal.dgain_lut = malloc(256*sizeof(uint32_t));
if (turning_data.normal.dgain_lut != NULL) {
        memset(turning_data.normal.dgain_lut, 0xff, 256*sizeof(uint32_t));
        memcpy(turning_data.normal.dgain_lut, imx219_dgain_lut,
                sizeof(imx219_dgain_lut));
}
```

• turning_data.sensor_data.active_width：sensor 实际输出的宽度。

• turning_data.sensor_data.active_height：sensor 实际输出的高度。

• turning_data.sensor_data.analog_gain_max：a_gain
    最大倍数，举一个例子，turning_data.sensor_data.analog_gain_max
    =126，最大倍率计算公式就是2\^(X/32)，其中 X 就是 126，X这个值每一个 sensor
    厂家不一样，需要从 sensor 手册或者 sensor 厂家查找得到。

• turning_data.sensor_data.digital_gain_max：d_gain 最大倍数。

• turning_data.sensor_data.exposure_time_min：短曝光最小曝光时间(短曝光每帧最小曝光行数)，每行的曝光时间可以用公式1秒/(帧率\*(有效行+blank))即1/lines_per_second推算得出。

• turning_data.sensor_data.exposure_time_max：短曝光最大曝光时间(短曝光每帧最大曝光行数)，短曝光每帧最大曝光行数可以通过公式单帧曝光时间/单行曝光时间，即(1/fps)/(1/lines_per_second)。

• turning_data.sensor_data.exposure_time_long_max：长曝光最大曝光时间(长曝光每帧最大曝光行数)，HDR
    sensor 会用到。

• turning_data.sensor_data.lines_per_second：每秒多少曝光行，计算公式为1/每行时间或者(fps\*vts)，其中vts在不同
    sensor
    的里面的名字可能有所不同，可能是frame_length、vts等，但含义都是每帧的所含的总共行数，包括有效行和
    blanking。lines_per_second 亦可以理解成HMAX,需要注意的是有些 sensor 无 HMAX
    概念。

• turning_data.normal.again_lut：again lut 表，fireware 根据 index 索引 lut
    表，查找 sensor 对应的寄存器值，lut 表区分 a_gain/d_gain，lut
    表格大小：again_lut[again_control_num][256],
    dgain_lut[dgain_control_num][256]。

注意事项1：当某个 gain 值不存在时，该位填充 0xffffffff，分配 gain
时，程序会向下查找，直到可以查到可以分配的 gain，下发到 kernel 中的 lut
表需要是完成高低位转换，避免在 kernel 中进行，比如gain = 0x1234，写入寄存器
0x3012，0x3013，有些 sensor 在 0x3012 中写 0x12，有些 sensor 在 0x3013 中写
0x12，在 hal 中转换屏蔽该差异性；

注意事项2：lut 表示 [0,255] 共 256 gain 控制，转换公式为 2\^(x/32)，即实际的
gain 倍数为[2\^(0/32),2\^(255/32)]， gain 的控制曲线为 log 类型，即任何一颗
sensor 的 gain 的控制被离散为 256 个控制点，原因是现在 3a 的控制算法给出 256
个控制点，给出更多的控制点并不会提高 gain 的控制精度

camera 在 mipi start 之前，需要保证 sensor 没有开流，在 camera sensor init
settings 中进行更改。

```c
static uint32_t imx219_linear_init_setting[] = {
    ....
    // 0x0100,0x01,  // 在 setting 最后不包含 开流 的配置
}
```

当 sensor 驱动和 setting 编写完成后，拷贝 \*_utility.c 和 \*_setting.h 到 SDK
对应目录中，并重新编译 SDK 生成 sensor 驱动库，生成文件位于
out/deploy/rootfs/usr/hobot/lib/sensor 中。

一般代码结构没有问题，即使 tuning_data 参数配置有不当的地方，框架也能正常加载 sensor驱动。如果 logcat 有 sensor so check 失败或者加载失败，则需要检查代码结构，是否按照 HBN 框架来编写。


#### 用户程序

参考 SDK 已有的用户程序，包含 CIM、ISP 的参数配置，这些配置需要根据具体的 sensor
的分辨率，帧率，数据格式进行配置。下面列出文件中需要单独配置的部分，其余部分可保持默认值，无需关注。

##### mipi 配置

| 字段                              | 描述                                                                                                               |
|-----------------------------------|--------------------------------------------------------------------------------------------------------------------|
| rx_enable                         | MIPI 接收(RX) 设备使能，使能对应的 MIPI RX 端口，默认填 1。 注意 该字段不是配置 MIPI RX 端口号，只是使能 MIPI RX。 |
| phy                               | 0: 代表 mipi dphy。                                                                                                |
| lane                              | mipi lane 数，目前每一个 MIPI RX 默认支持 4 lane。                                                                 |
| datatype                          | mipi 输入的数据格式，与 sensor 配置保持一致。 常见的如下： RAW8：0x2A RAW10: 0x2B RAW12: 0x2C YUV422 8-bit: 0x1E   |
| fps                               | 帧率，供计算 MIPI 一些配置使用，按照 sensor 输出帧率填写即可，可从 FAE 获取。                                      |
| mipiclk                           | mipi 总传输率 (所有 LANE)，可从 FAE 获取，一般 FAE 提供 sensor init setting 时有描述。                             |
| width                             | 输入图像 宽度 piexl。                                                                                              |
| height                            | 输入图像 高度 piexl。                                                                                              |
| linelenth                         | mipi linelenth， 根据 sensor 实际情况配置，可从sensor spec 寄存器读取，或者实际硬件测量。                          |
| framelenth                        | mipi framelenth， 根据 sensor 实际情况配置，可从sensor spec 寄存器读取，或者实际硬件测量。                         |
| settle                            | mipi settle， phy 的 settle 时间配置，可实际硬件测量。报 mipi phy 错时可调整， 0 - 120 范围。                      |
| channel_num                       | mipi 虚拟通道 number，linear mode 填 1，HDR DOL2 mode 填 2。                                                       |
| channel_sel[MIPIHOST_CHANNEL_NUM] | mipi 虚拟通道对应的 ipi channel。                                                                                  |

:::tip 商业支持
商业版提供更完整的功能支持、更深入的硬件能力开放和专属的定制内容。为确保内容合规、安全交付，我们将通过以下方式开放商业版访问权限。

商业版本获取流程：
1. 填写问卷：提交您的机构信息、使用场景等基本情况
2. 签署保密协议（NDA）：我们将根据提交信息与您联系，双方确认后签署保密协议
3. 内容释放：完成协议签署后，我们将通过私有渠道为您开放商业版本资料
  
如您希望获取商业版内容，请点击下方链接填写问卷，我们将在 3 ～ 5 个工作日内与您联系：
https://horizonrobotics.feishu.cn/share/base/form/shrcnpBby71Y8LlixYF2N3ENbre
:::

##### camera sensor 配置

| 字段                         | 描述                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       |
|------------------------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| name[CAMERA_MODULE_NAME_LEN] | camera 模组名称，需要和 sensor lib名称对应，如：sensor 驱动名称为：libimx219.so，那么 name 为 imx219                                                                                                                                                                                                                                                                                                                                                                                       |
| addr                         | sensor 设备地址，一般是 i2c 7位地址。                                                                                                                                                                                                                                                                                                                                                                                                                                                      |
| sensor_mode                  | sensor 工作模式： 1：NORMAL_M，linear 模式 2：DOL2_M，hdr 2帧合成1帧 3：DOL3_M，hdr 3帧合成1帧 4: DOL4_M，hdr 4帧合成1帧 5: PWL_M，hdr 模式 sensor 内部合成                                                                                                                                                                                                                                                                                                                                |
| gpio_enable                  | 是否使用 gpio 控制 camera sensor 的引脚，以满足 sensor 上下电的时序要求。如：使用 gpio 来控制 sensor XSHUTDN 引脚。**注意**：需要在 dts 中配置对应的 gpio number。 0: 不使用 gpio 来控制。  非 0: 使用 gpio 来控制 sensor，按照 bit 来使能 gpio 数量。 比如: 0x07 则代表使能 [a, b, c] 3 个 gpio。                                                                                                                                                                                         |
| gpio_level                   | 如果选择 gpio_enable_bit，则可以配置 gpio_level 来控制 sensor 引脚高低电平。某个 gpio bit 与 sensor 管脚高低电平关系如下： 0: 先输出低电平，sleep 1s (休眠时间可以在 sensor 驱动文件 power_on 函数中，通过 usleep 自行定义)，再输出高电平。 1: 先输出高电平，sleep 1s，再输出低电平 比如：0x05 = 101，从 bit0 到 bit2 分别代表 gpio a 先输出高电平，再输出低电平，gpio b 先输出低电平，再输出高电平，gpio c 先输出高电平，再输出低电平。 **注意**：需要根据 sensor spec 上电时序来自定义。 |
| fps                          | sensor 帧率配置                                                                                                                                                                                                                                                                                                                                                                                                                                                                            |
| width                        | sensor 出图宽度 pixel                                                                                                                                                                                                                                                                                                                                                                                                                                                                      |
| height                       | sensor 出图高度 piexl                                                                                                                                                                                                                                                                                                                                                                                                                                                                      |
| format                       | sensor mipi 数据类型，常见的如下： RAW8：0x2A RAW10: 0x2B RAW12: 0x2C YUV422 8-bit: 0x1E                                                                                                                                                                                                                                                                                                                                                                                                   |
| extra_mode                   | 模组索引配置，部分 sensor 驱动中会用到                                                                                                                                                                                                                                                                                                                                                                                                                                                     |
| config_index                 | 功能配置，部分 sensor 驱动中会用到                                                                                                                                                                                                                                                                                                                                                                                                                                                         |
| calib_lname                  | sensor 效果库路径，默认路径为 /usr/hobot/lib/sensor                                                                                                                                                                                                                                                                                                                                                                                                                                        |
| end_flag                     | 固定为 CAMERA_CONFIG_END_FLAG                                                                                                                                                                                                                                                                                                                                                                                                                                                              |

##### vio 配置

| 字段1 | 字段2                | 字段3                                 | 描述                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               |
|-------|----------------------|---------------------------------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| VIN   | cim                  | mipi_en                               | 使能 mipi 接口                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     |
|       |                      | mipi_rx                               | mipi rx 端口号                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     |
|       |                      | vc_index                              | mipi virtual index，mipi 虚拟通道，默认填写 0 即可                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 |
|       |                      | ipi_channel                           | ipi channel number，linear mode 为 1， hdr mode dol2 为 2                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          |
|       |                      | cim_isp_flyby                         | cim/sif online 到 isp。 0: sif offline 到 isp，数据经过 ddr。 1：sif online 到 isp，数据不经过 ddr。                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               |
|       | input channel        | format                                | vin format，sensor 输出 format                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     |
|       |                      | width                                 | sensor 输出分辨率 宽度 pixel                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       |
|       |                      | height                                | sensor 输出分辨率 高度 pixel                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       |
|       | output channel / ddr | ddr_en                                | 数据是否 dump 到 DDR                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               |
|       |                      | wstride                               | 设置为 0，驱动会自动计算 wstride。                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 |
|       |                      | format                                | 下 ddr 时，设置的 sensor format                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    |
|       |                      | buffers_num                           | cim/sif 下 ddr 的buffer number，设置为 1 - 6                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       |
|       |                      | flags                                 | 一般在程序设置                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     |
|       |                      |                                       | HB_MEM_USAGE_CPU_READ_OFTEN \| HB_MEM_USAGE_CPU_WRITE_OFTEN \| HB_MEM_USAGE_CACHED                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 |
| ISP   | base                 | hw_id 和 slot_id                      | CIM 硬件直连 ISP 情况： hw_id 需要与 cim 的 rx_index 一一对应。 在sched_mode配置为1的情况下，CIM online ISP slot_id 取值0 \~ 3，与cim vc_index一一对应。 在sched_mode配置为2的情况下，slot_id 固定为0，cim vc_index可视sensor实际接入情况设置为0 \~ 3。 CIM DDR 连接 ISP 情况： hw_id 无限制，可视 sensor 实际接入情况和项目需要选择hw_id。 slot id只需从4开始到11即可。 注意：在多路压力场景下，采用CIM DDR的连接方式情况时，大分辨率的sensor通路连接尽量选择slot id取值较小的ISP通道上，可保证对sensor实时控制。                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 |
|       |                      | sched_mode                            | ISP的工作模式， 1: 表示manual，软件调度的方式， 2: 表示passthru模式，全online独占ISP的工作模式。                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   |
|       |                      | width                                 | 输入图像高度                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       |
|       |                      | height                                | 输入图像宽度                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       |
|       |                      | frame_rate                            | 输入帧率，无实际效果                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               |
|       |                      | algo_state                            | 2a 的开关参数                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      |
|       | output channel       | stream_output_mode 和 axi_output_mode | isp 模式                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           |

#### 板端运行程序

执行对应的测试程序

#### isp 图像预览

**SDK 代码添加 tuning 程序**

修改 /app/tuning_tool/scripts/tuning_menu.sh 文件，仿照已有的
sensor，进行添加。

```c
ITEM_IMX219_RGGB="module:Raw10_IMX219_RDK-S100"
IMX219_RGGB_Raw10_IMX219_RDK-S100()
{
        IDESC="imx219 rggb raw10 RDK-S100"
        setup_case ${folder}/tuning_imx219_cim_isp_1080p
}
```

在 /app/tuning_tool/cfg/matrix 目录下建立 tuning_imx219_cim_isp_1080p
文件夹，并添加对应的 hb_j6dev.json mipi.json vpm_config.json 三个文件。

编译 SDK 系统代码，确保板端已经包含修改和添加的文件。

**板端执行 tuning 程序**

```c
cd /app/tuning_tool/scripts
bash run_tuning.sh 
# 按照交互页面提示，选择对应的 sensor
```

**图像预览**

1. [点击此处](../../../01_Quick_start/download.md#系统镜像与工具)下载图像浏览工具 hbplayer。
2. 打开 hbplayer 并设置网络地址（PC需要与板子可以ping通），点 apply 设置生效，并点 connect 则可以看到实时视频流。实时预览操作示意如图所示。

    ![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/03_multimedia_development/02_S100/camera_bringup/camera_bringup_06.png)

### 错误码

下面是 sensor 常见的错误码及简单的排查方向：

| 错误码 | 定义                        | 排查方向                                                        |
|--------|-----------------------------|-----------------------------------------------------------------|
| 203    | HB_CAM_INIT_FAIL            | sensor 初始化失败，一般可能 i2c 不通，配置的 sensor mode 不支持 |
| 205    | HB_CAM_START_FAIL           | sensor 启动失败，一般可能 i2c 不通，配置的 sensor mode 不支持   |
| 207    | HB_CAM_I2C_WRITE_FAIL       | sensor i2c 不通。                                               |
| 217    | HB_CAM_SENSOR_POWERON_FAIL  | sensor 上电失败，可能是 sensor gpio 配置错误。                  |
| 218    | HB_CAM_SENSOR_POWEROFF_FAIL | sensor 下电失败，可能是 sensor gpio 配置错误。                  |

### FAQ

**control-tool 使用说明**

进入tuning 目录，cd /app/tuning_tool/control_tool

按照交互界面提示，执行启动脚本 sh server_isp\*_8000.sh，ISP 硬件具有两个 IP
核，每个核可以单独运行，若需要启动 isp 的控制则运行脚本 sh server_isp0_8000.sh.

启动方式如图所示。

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/03_multimedia_development/02_S100/camera_bringup/camera_bringup_07.png)

脚本会自动识别板子ip, 默认检查eth1网卡ip地址。若需要修改为启动eth0
网卡，修改脚本eth_id=eth0。修改位置如图所示。

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/03_multimedia_development/02_S100/camera_bringup/camera_bringup_08.png)

修改通信地址示意图

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/03_multimedia_development/02_S100/camera_bringup/camera_bringup_09.png)

## V4L2 sensor 点亮

### V4L2 sensor驱动编写说明
S100 Camsys sensor v4l2 驱动软件框架为标准的v4l2 sub device驱动。
![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/03_multimedia_development/02_S100/camera_bringup/camera_bringup_10.png)
下面以IMX219驱动为例，介绍MIPI直连sensor v4l2 驱动开发流程，imx219驱动源码位于：kernel/drivers/media/i2c/imx219.c  

####定义sensor私有结构体
imx219私有结构体如下：
```c
struct imx219 {
        struct v4l2_subdev sd;
        struct media_pad pad;
        struct i2c_client *i2c_client;
        ...
        struct v4l2_ctrl *xxx_ctrl;
        ...
};
```
sd: v4l2 sub device 句柄，用于操作subdev ops；
pad: media pad，用于和后级模块建立media链接关系；
i2c_client: i2c client 句柄，用来通过i2c总线与sensor交互；
xxx_ctrl: v4l2控制属性，例如exposure、flip、blank控制，非必须实现；

#### V4L2回调函数实现

符合v4l2标准的sensor驱动需要实现一些ops函数，V4L2框架会通过ops函数控制sensor
```c
static const struct v4l2_subdev_ops imx219_subdev_ops = {
        .core = &imx219_core_ops,
        .video = &imx219_video_ops,
        .pad = &imx219_pad_ops,
};
```
实现v4l2 subdev ops回调，其中包含core ops、video ops、pad ops。
```c
static const struct v4l2_subdev_pad_ops imx219_pad_ops = {
        .enum_mbus_code = imx219_enum_mbus_code,
        .get_fmt = imx219_get_pad_format,
        .set_fmt = imx219_set_pad_format,
        .enum_frame_size = imx219_enum_frame_size,
};
```
pad ops定义了一些格式配置、格式协商的回调接口，必须实现。
```c
static const struct v4l2_subdev_video_ops imx219_video_ops = {
        .s_stream = imx219_set_stream,
};
```
video ops主要定义了sensor开关流的接口，必须实现。
```c
static const struct v4l2_subdev_core_ops imx219_core_ops = {
        .subscribe_event = v4l2_ctrl_subdev_subscribe_event,
        .unsubscribe_event = v4l2_event_subdev_unsubscribe,
};
```

core ops定义了一些如ioctl event实现等，可选实现。
```c
static const struct v4l2_subdev_internal_ops imx219_internal_ops = {
        .open = imx219_open,
};
```
internal ops主要定义了一些ops用于管理子设备的生命周期，按需实现open、close、release等回调。

#### sensor probe函数
```c
static int imx219_probe(struct i2c_client *client)
{
        imx219 = devm_kzalloc(&client->dev, sizeof(*imx219), GFP_KERNEL); // 1
        if (!imx219)
                return -ENOMEM;
        ...        
        v4l2_i2c_subdev_init(&imx219->sd, client, &imx219_subdev_ops);  // 2

        imx219->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE |
                        ┆   V4L2_SUBDEV_FL_HAS_EVENTS;
        imx219->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
        imx219->pad.flags = MEDIA_PAD_FL_SOURCE;
        ret = media_entity_pads_init(&imx219->sd.entity, 1, &imx219->pad);  // 3

        ret = v4l2_async_register_subdev_sensor(&imx219->sd);  // 4
        
        ...
}
```
1. 初始化sensor结构体，分配内存；
2. 初始化一个 v4l2 subdev，并绑定到 I2C client；
3. 初始化 media entity 的 pad 信息，让media controller知道当前sensor有一个输出pad，可以连接到后级模块；
4. 异步注册，把当前sensor subdev注册到v4l2框架；

#### sensor device tree
S100默认加载imx219设备树，设备树组织格式如下面所示，如果接入其他的mipi sensor，需要以dts overlay的方式覆盖掉imx219的dts。
```c
&i2c1 {
        status = "okay";

        imx219@10 {
                status = "okay";
                compatible = "sony,imx219";
                ...
                reg = <0x10>; // sensor i2c地址
                ...
                port {
                        cam_to_mipi_csi0: endpoint {  // MIPI 相关属性
                                remote-endpoint = <&rdk_s100_mipi_csi0_from_cam>;  // 对接到mipi RX0
                                clock-lanes = <0>;
                                data-lanes = <1 2>;
                                link-frequencies =
                                        /bits/ 64 <456000000>;
                                virtual-channel = <0>;
                        };
                };
        };
};

&mipi_host0 {
        ports {
                port@0 {
                        rdk_s100_mipi_csi0_from_sensor0: endpoint {
                                remote-endpoint = <&sensor0_to_mipi_csi0>;
                                clock-lanes = <0>;
                                data-lanes = <1 2>;    // mipi data lane 为 2lane
                                lane-rate = <1728>;    // mipi 速率
                                vc_id = <0>;            // sensor 输出的 virtual channel
                                emb-en = <1>;            // sensor 输出是否包含 embedded data
                        };
                };
        };
};
```




### V4L2 GMSL SerDes接口调用说明
S100 Camsys支持接入美信加串器的sensor，camera子板默认搭载美信解串器MAX96712。GMSL sensor同样作为一个v4l2 subdev接入v4l2框架，这里加串器及解串器驱动为gmsl sensor驱动提供操作函数集，不实现为v4l2 subdev。
![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/03_multimedia_development/02_S100/camera_bringup/camera_bringup_11.png)


serdes 相关的一些数据结构及回调函数定义在 kernel/include/media/i2c/serdes_core.h，需要包含该头文件，#include \<media/i2c/serdes_core.h\>
本小结以0820C GMSL sensor为例，介绍camsys gmsl sensor开发。
####sensor结构体新增成员
```c
struct ar0820 {
        ...
        struct serdes_device *ser_dev;
        struct serdes_device *dser_dev;
        struct serdes_ctx g_ctx;
        ..
};
```

sensor驱动需要包含ser_dev与dser_dev两个结构体，用来操作加串和解串;
需要包含serdes contex g_ctx成员，用来保存serdes相关属性，其中主要使用的结构体成员说明如下：
```c
struct serdes_ctx {
        u32 serdes_csi_link;    // 在 sensor 驱动中保存解串器的port 值
        u32 ser_reg;            //加串器i2c地址映射目标值
        u32 sdev_reg;            // sensor 实际i2c地址
        u32 sdev_def;            // sensor i2c地址映射目标值
        struct device *sen_dev;
        u32 lane_num;            // 在 sensor 驱动中保存 sensor与加串器连接的mipi data lane数
        u32 data_type;           // 在 sensor 驱动中保存 sensor输出的数据类型
        u32 dst_vc;               // 在 sensor 驱动中保存 sensor输出的virtual channel
};
```

#### serdes回调函数
加串器与解串器都提供了下面的回调函数，供sensor driver中调用，调用需要使用SERDES_OP宏
```c
/* 默认返回值大于等于 0 代表操作成功，返回值小于 0 则代表操作失败 */
struct serdes_ops {
        /* 加串器和解串器做初始化用，只下一些基础配置 */
        int (*init)(struct serdes_device *serdes_dev);
        /* 目前是给 d457 -> max9295a 做额外初始化用，将 max929a 4个 pipe 都使能 */
        int (*init_ex)(struct serdes_device *serdes_dev);
        /* 预留 */
        int (*reset)(struct serdes_device *dev);
        /* 将解析的 dts 值，通过 serdes_ctx 传递给加串器和解串器 */
        int (*set_ctx)(struct serdes_device *serdes_dev,
                ┆      struct serdes_ctx *ctx);
        /* 用于解串器建立 link 使用，默认 setting 是没有使能 link，在该ops 中会使能 link */
        int (*setup_link)(struct serdes_device *serdes_dev,
                        ┆ struct device *sen_dev);
        /* remote_contrl_get -> map_addr -> remote_contrl_put 配套使用，在 sensor/加串器 地址重映射期间，保证 sensor/加串器 的稳定性 */
        int (*remote_contrl_get)(struct serdes_device *serdes_dev,
                                ┆struct device *sen_dev);
        int (*remote_contrl_put)(struct serdes_device *serdes_dev);
        /* 加串器调用，重新映射加串器和sensor 的i2c 地址 */
        int (*map_addr)(struct serdes_device *serdes_dev);
        /* 加串器拉高某个 mfp */
        int (*enable_mfp)(struct serdes_device *serdes_dev, uint8_t gpio_index);
        /* 加串器拉低某个 mfp */
        int (*clear_mfp)(struct serdes_device *serdes_dev, uint8_t gpio_index);
        /* 解串器打开 mipi tx，开始出流，加串器默认是打开的，所以无需主动调用 */
        int (*set_stream)(struct serdes_device *serdes_dev,
                        struct device *sen_dev, int enable);
        /* 解串器和加串器配置 gmsl video pipe 属性，属性由 dts 解析得到，默认配置加串器/解串器 pipe-z */
        int (*set_pipe)(struct serdes_device *serdes_dev,
                        struct device *sen_dev);
        /* 针对复杂场景，解串器和加串器配置 gmsl video pipe 数据，可根据每个 pipe 灵活配置 */
        int (*set_pipe_ex)(struct serdes_device *serdes_dev, struct device *sen_dev,
                        uint8_t pipe, uint8_t vc_id, uint8_t data_type);
        /* 通过 virtual channel 值，查看解串器是否还有空余的 pipe，返回空余的 pipe id(0-3)，
           d457 sensor 在出流前使用，与 release_pipe_id 配套使用 */
        int (*get_pipe_id)(struct serdes_device *serdes_dev,
                        uint8_t vc_id);
        /* 使用完解串器 video pipe，释放对应的 video pipe*/
        int (*release_pipe_id)(struct serdes_device *serdes_dev,
                        uint8_t pipe_id);
};
```

1. 在sensor probe中需要调用一些serdes的ops来做一些软件初始化，解析 dts 值，并通过 set_ctx 分别传递给加串器和解串器驱动。
```c
ret = SERDES_OP(priv->ser_dev, set_ctx, priv->ser_dev, &priv->g_ctx);
ret = SERDES_OP(priv->dser_dev, set_ctx, priv->dser_dev, &priv->g_ctx);
```
分别调用加串器和解串器的set_ctx函数与加解串器建立软件上的链接关系；
```c
        ret = SERDES_OP(priv->dser_dev, init, priv->dser_dev);
        ret = SERDES_OP(priv->dser_dev, setup_link, priv->dser_dev, sen_dev);
        ret = SERDES_OP(priv->dser_dev, remote_contrl_get, priv->dser_dev,
        ret = SERDES_OP(priv->ser_dev, map_addr, priv->ser_dev);
        ret = SERDES_OP(priv->dser_dev, remote_contrl_put, priv->dser_dev);
        ret = SERDES_OP(priv->ser_dev, init, priv->ser_dev);
        ret = SERDES_OP(priv->ser_dev, set_pipe, priv->ser_dev, sen_dev);
        ret = SERDES_OP(priv->dser_dev, set_pipe, priv->dser_dev, sen_dev);
        ret = SERDES_OP(priv->ser_dev, clear_mfp, priv->ser_dev,
                        priv->mfp_reset);
        ret = SERDES_OP(priv->ser_dev, enable_mfp, priv->ser_dev,
                        priv->mfp_reset);
```

调用ops做一些加解串器link、addr、pipe、mfp等初始化；
2. 在s_stream中配置解串器开流，加串器mfp使能等：
```c
SERDES_OP(priv->ser_dev, enable_mfp, priv->ser_dev,priv->mfp_trigger);
SERDES_OP(priv->dser_dev, set_stream, priv->dser_dev, sen_dev, 1);
```

#### sensor device tree
S100 v4l2 gmsl sensor默认加载0820c的dts，gmsl sensor设备树组织格式如下面所示：
```c
ar0820@11 {
                compatible="d-robotics,ar0820";
                reg = <0x11>;     // map后的地址
                addr = <0x10>;    // sensor i2c实际地址
                ......
                mfp-reset = <0>;  // reset连接到加串器的mfp
                mfp-trigger = <7>;// trigger pin链接到加串器的mfp
                d-robotics,serdes-ser-device = <&ser_a>;  // 链接至linkA上的加串
                d-robotics,serdes-dser-device = <&dser>;  // 接入deserial
                status = "okay";

                port {
                        cam_0_to_mipi_csi4: endpoint {    // 接入mipi rx4
                                remote-endpoint = <&mipi_csi4_from_cam_0>;
                                virtual-channel = <0>;
                        };
                };
};
```

### Sensor dtbo 文件编写配置说明
S100 uboot 支持 DTB Overlay 功能，可以在不修改当前启动使用的dts文件的情况下，通过编写配置对应的dtbo文件。对当前启动使用的dtb文件进行增/改（不支持删减）的功能
#### sensor dtbo 文件生成
1. 编写dtso文件
```c
#include <dt-bindings/gpio/gpio.h>

/dts-v1/;
/plugin/;

/ {
    fragment@1 {
        target-path = "/soc/i2c@39450000/";
            __overlay__ {
                status = "okay";
                d457@11 {
                    compatible="intel,d4xx";
                    reg = <0x11>;
                    def-addr = <0x10>;
                    width = <640>;
                    height = <480>;
                    cam-type = "Depth";
                    data_type = <0x2e>;
                    lane_num = <2>;
                    vc_id = <0>;
                    d-robotics,serdes-ser-device = <&ser_a>;
                    d-robotics,serdes-dser-device = <&dser>;
                    status = "okay";

                    port {
                        sensor_0_to_mipi_csi4: endpoint {
                            remote-endpoint = <&mipi_csi4_from_sensor_0>;
                            virtual-channel = <0>;
                        };
                    };
                };
           };
      };
};
```
2. 在板端编译生成dtbo
  - 安装dtc 工具
```c
sudo apt install device-tree-compiler -y
```
  - 预处理dtso 文件
```c
#当编写的dtso中include 头文件 或者 有定义时，才需要用以下命令预处理dtso文件

#获取dts 头文件路径
HEADER_DIR=$(find /usr/src -maxdepth 1 -type d -name "linux-headers-*" | sort -Vr | head -n 1)
DTS_HEAD_PATH="$HEADER_DIR/include"


#将编写的dtso文件预处理生成dtbi文件
cpp -nostdinc -I "$DTS_HEAD_PATH" sample.dtso > sample.dtbi
```
  - 编译生成dtbo 文件
    如果有dtbi文件，则通过dtbi文件生成最终的dtbo文件
```c
dtc -q -@ -I dts -O dtb -o sample.dtbo sample.dtbi
```
    如果没有dtbi文件，则通过编写的dtso文件生成最终的dtbo文件
```c
dtc -q -@ -I dts -O dtb -o sample.dtbo sample.dtso
```
#### sensor dtbo 开机自动生效配置
1. 将编译生成的dtbo 文件放置到 /boot/overlays 目录下
  若板端没有/boot/overlays目录，用户可自行添加/boot/overlays 目录， 或者通过安装hobot-camera.deb
来获取/boot/overlays 目录 和 d457 sensor dtbo 文件
![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/03_multimedia_development/02_S100/camera_bringup/camera_bringup_12.png)

2. 修改config.txt 文件，配置要添加的dtbo文件
若该位置没有config.txt文件， 用户可自行添加config.txt 文件
![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/03_multimedia_development/02_S100/camera_bringup/camera_bringup_13.png)
  按照下面的方法，修改config.txt
dtbo_file_path=/overlays/v0p5_d457_2v_depth_color.dtbo

3. 重启板子，使能配置的dtbo文件。在debug 版本的uboot log中，可以检查加载dtbo 的情况
![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/03_multimedia_development/02_S100/camera_bringup/camera_bringup_14.png)
  
### Sensor gain LUT表编写说明
RAW格式的sensor，对接到S100 ISP图像系统，除了编写sensor v4l2 驱动，另外需要制作一个so，存放sensor增益lut转换表，包括again lut、dgain lut等。人眼对亮度的感知更接近对数尺度而非线性尺度，dB单位更符合这种感知，S100 ISP这里sensor增益lut表存放的gain数组，需要存放db单位连续的sensor gain 寄存器配置值，供ISP调节sensor增益时找到对应的sensor寄存器值，下发到sensor。下面以imx219为例，介绍v4l2 sensor lut so如何制作。

imx219 sensor gain lut表制作目录在sdk中位于hobot-camera/v4l2/v4l2_helper/imx219_v4l2;

1. 添加\<sensor_name\>_camera_helper.c文件、Makefile及版本文件version.mk
```c
imx219_v4l2
├── imx219_camera_helper.c
├── Makefile
└── version.mk
```

2. 在xxx_helper.c文件中，制作again lut表及dgain lut表，lut表为一个uint32类型的数组，最多256个成员，每个成员为gain寄存器的配置值，相邻成员对应的db要连续，以imx219为例：
```c
static uint32_t imx219_again_lut[] = {
        0x00,   // 0db
        0x05,   // 约0.2db
        0x0B,   // 约0.4db
        0x0F,   // 约0.6db
        0x15,   // 约0.8db
        ......
        0xE7,   // 约20.4db
        0xE8,   // 约20.6db
        0xffff, // end flag
};

static uint32_t imx219_dgain_lut[] = {
        0x0100,  // 0db
        0x0106,  // 约0.2db
        0x010c,  // 约0.4db
        0x0112,  // 约0.6db
        ......
        0x0f53,  // 约23.6db
        0x0fa9,  // 约23.8db
        0x0fd9,  // 约24.0db
        0xffff,  // end flag
};
```
lut表最后一个成员固定为0xffff。

3. 编写again index to reg、dgain index to reg callback函数，及获取callback的接口，复用219的即可：
```c
typedef uint32_t (*AGainIndexToReg_t)(uint8_t);  // 传入uint8 index，获得uint32 sensor寄存器配置值
typedef uint32_t (*DGainIndexToReg_t)(uint8_t);  // 同上

typedef struct {
        AGainIndexToReg_t again_index_to_reg_callback;
        DGainIndexToReg_t dgain_index_to_reg_callback;
} Callbacks;   // callback结构体，不需要更改

uint32_t again_index_to_reg_function(uint8_t isp_index) 
{
        if (isp_index >= sizeof(imx219_again_lut)/sizeof(uint32_t))
                isp_index = sizeof(imx219_again_lut)/sizeof(uint32_t) - 1;
        return imx219_again_lut[isp_index];
}

uint32_t dgain_index_to_reg_function(uint8_t isp_index) 
{
        if (isp_index >= sizeof(imx219_dgain_lut)/sizeof(uint32_t))
                isp_index = sizeof(imx219_dgain_lut)/sizeof(uint32_t) - 1;

        return imx219_dgain_lut[isp_index];
}

Callbacks cb = {again_index_to_reg_function,
                dgain_index_to_reg_function,};

//get_index_to_reg_callbacks
Callbacks* get_index_to_reg_callbacks() {
        return &cb;
}
```

生成的so命名为lib\<sensor_name\>_v4l2.so，运行时会自动匹配并dlopen该so，调用符号最终获取到lut表。

### 曝光同步sensor驱动说明
S100 camsys serdes提供了trigger相关接口，sensor驱动中可以调用来配置lpwm硬件、使能lpwm。
硬件同步曝光目前仅支持gmsl sensor，sensor dts中需要配置正确的trigger mfp管脚。
```c
SERDES_OP(priv->dser_dev, trigger_cfg, priv->dser_dev, sen_dev, period, duty);
```
在sensor的初始化配置format格式中调用trigger_cfg，下发lpwm配置
period单位为ns，计算方式为（1000000/fps)*1000
duty单位为ns，没有特殊要求可以配置为10000
```c
SERDES_OP(priv->dser_dev, trigger_enable, priv->dser_dev, sen_dev, enable);
```
在stream开关流时，调用trigger_enable打开或关闭lpwm输出
