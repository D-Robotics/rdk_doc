---
sidebar_position: 2
---
```mdx-code-block
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';
```

# RDK IMU Module（BMI088）

## 产品简介

RDK IMU Module 为地瓜机器人官方 IMU 配件，采用 Bosch **BMI088** 方案，集成三轴陀螺仪与三轴加速度计，可通过 **I2C** 或 **SPI** 与 RDK 开发板连接。支持通过 Linux IIO 子系统读取原始数据。

:::info 适用平台
本文档针对 **RDK X5 / RDK X5 Module**，系统镜像建议 **3.4.x**（兼容 3.5.x）。
:::

## 安装方法

### 硬件连接

<Tabs groupId="bmi088-interface">
<TabItem value="i2c" label="I2C 接口">

![rdk_x5_bmi088_hw_connect.png](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/07_accessory_instructions/imu/rdk_x5_bmi088_hw_connect.png)

1. 将 RDK IMU Module 按说明书接入 RDK X5 的 40pin 排针。
2. 确认供电与 I2C 引脚连接牢固。

使用 I2C 的接口的时候，检查连接是否正常的时候可以使用如下命令来检查 IMU 能被探测到

```shell
i2cdetect -y -r 5
```

![rdk_x5_bmi088_i2cdetect.png](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/07_accessory_instructions/imu/rdk_x5_bmi088_i2cdetect.png)


</TabItem>
<TabItem value="spi" label="SPI 接口">

1. 将 RDK IMU Module 按说明书接入 RDK X5 的 40pin 排针。
2. 特别注意载板上面的排针，是否都插入了 SPI 这一侧

![rdk_x5_bmi088_hw_connect_spi.png](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/07_accessory_instructions/imu/rdk_x5_bmi088_hw_connect_spi.png)

</TabItem>
</Tabs>

### 软件配置

在全屏终端中运行 `sudo srpi-config`，按接口类型选择对应 IMU 项：

<Tabs groupId="bmi088-interface">
<TabItem value="i2c" label="I2C 接口">

1. 进入 `3 Interface Options` → `I6 IMU`

![rdk_x5_bmi088_srpiconfig_1.png](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/07_accessory_instructions/imu/rdk_x5_bmi088_srpiconfig_1.png)

![rdk_x5_bmi088_srpiconfig_2.png](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/07_accessory_instructions/imu/rdk_x5_bmi088_srpiconfig_2.png)

2. 选择 `BMI088-I2C-Interface`

![rdk_x5_bmi088_srpiconfig_3.png](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/07_accessory_instructions/imu/rdk_x5_bmi088_srpiconfig_3.png)

3. 选择 `Finish`，确认后选择 `Yes` 重启

![rdk_x5_bmi088_srpiconfig_4.png](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/07_accessory_instructions/imu/rdk_x5_bmi088_srpiconfig_4.png)

![rdk_x5_bmi088_srpiconfig_5.png](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/07_accessory_instructions/imu/rdk_x5_bmi088_srpiconfig_5.png)






</TabItem>
<TabItem value="spi" label="SPI 接口">

1. 进入 `3 Interface Options` → `I6 IMU`

![rdk_x5_bmi088_srpiconfig_6.png](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/07_accessory_instructions/imu/rdk_x5_bmi088_srpiconfig_6.png)

![rdk_x5_bmi088_srpiconfig_7.png](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/07_accessory_instructions/imu/rdk_x5_bmi088_srpiconfig_7.png)

2. 选择 BMI088 的 **SPI** 相关选项（与 I2C 选项区分，以 `srpi-config` 菜单为准）

![rdk_x5_bmi088_srpiconfig_8.png](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/07_accessory_instructions/imu/rdk_x5_bmi088_srpiconfig_8.png)

3. 选择 `Finish`，确认后重启

![rdk_x5_bmi088_srpiconfig_9.png](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/07_accessory_instructions/imu/rdk_x5_bmi088_srpiconfig_9.png)

![rdk_x5_bmi088_srpiconfig_10.png](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/07_accessory_instructions/imu/rdk_x5_bmi088_srpiconfig_10.png)

</TabItem>
</Tabs>

### 卸载方法

1. 进入 `srpi-config` → `3 Interface Options` → `I6 IMU`
2. 选择 `UNSET` 卸载 IMU 驱动与相关配置
3. 断电后取下 IMU 模组

## 运行

<Tabs groupId="bmi088-verify">
<TabItem value="i2c-verify" label="I2C 接口">

### 1. 检查设备

**I2C 接口**下，扫描 I2C 总线（RDK X5 常用总线号为 `5`，以实际接线为准）：

```shell
i2cdetect -y -r 5
```
![rdk_x5_bmi088_i2cdetect_2.png](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/07_accessory_instructions/imu/rdk_x5_bmi088_i2cdetect_2.png)

若扫描结果出现一个地址为 `UU`、另一个为 `69`，需**断电重启**后再继续。

查看 input 事件节点（event 序号随挂载顺序变化）：

```shell
ls /dev/input
```

查看 IIO 设备节点：

```shell
ls /sys/bus/iio/devices/
```

### 2. 基础数据验证

直接读取 IIO 节点验证（`iio:device` 序号以 `ls /sys/bus/iio/devices/` 为准）：

```shell
cat /sys/bus/iio/devices/iio:device1/gyr_val
```

能读到正确的 `name` 与数值即表示 I2C 通路正常。

</TabItem>
<TabItem value="spi-verify" label="SPI 接口">

### 1. 检查设备

针对这款 IMU 我们可以直接检查是否有注册成功
```shell
dmesg | grep BS_LOG
```
![rdk_x5_bmi088_spi_dmesg.png](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/07_accessory_instructions/imu/rdk_x5_bmi088_spi_dmesg.png)

### 2. 基础数据验证

直接读取 IIO 节点验证（`iio:device` 序号以 `ls /sys/bus/iio/devices/` 为准）：

```shell
cat /sys/bus/iio/devices/iio:device1/gyr_val
```

能读到正确的 `name` 与数值即表示 SPI 通路正常。参考如下：

![rdk_x5_bmi088_spi_cat_gyr_name.png](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/07_accessory_instructions/imu/rdk_x5_bmi088_spi_cat_gyr_name.png)

![rdk_x5_bmi088_spi_cat_gyr_val.png](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/07_accessory_instructions/imu/rdk_x5_bmi088_spi_cat_gyr_val.png)

![rdk_x5_bmi088_spi_cat_acc_val.png](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/07_accessory_instructions/imu/rdk_x5_bmi088_spi_cat_acc_val.png)

注意：IIO 的实际节点编号可能根据实际注册的设备有所不同，请以板端实际情况为准

</TabItem>
</Tabs>

## 常见问题排查

- **i2cdetect 出现一个 UU、一个 69**：断电重启开发板后再扫描。
- **找不到 IIO 设备**：确认 `srpi-config` 已选择正确接口类型并已重启。
- **device 序号与文档示例不一致**：以板上实际注册的结果为准。

