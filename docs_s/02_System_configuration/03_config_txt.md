---
sidebar_position: 3
---

# 2.3 config.txt 文件配置
:::warning
- 所有配置文件内的配置，均可以在Uboot内手动覆盖。Uboot内手动配置（在Uboot命令行使用setenv）的优先级**高于**配置文件内的配置。完整环境变量优先级：`setenv > 配置文件 > 上一次启动saveenv`；
- 本章内容均以“配置文件”指代**默认路径**为`/boot/config.txt`的配置文件；
- 使用配置文件时，会需要修改启动分区的内容，这与[AVB](https://source.android.com/docs/security/features/verifiedboot)的要求是冲突的，所以本功能在使能AVB时（AVB功能**默认不使能**）不能使用；
:::

## 使用指南
:::info 提示
- 配置文件默认格式为`<key>=<value>`，第一个`=`后面的所有内容均为`=`前的`<key>`的配置值；
- 配置文件单行配置不能超过1024字符；
- 配置文件内的配置默认不会被保存为Uboot的默认配置；
:::

### 配置内核bootargs（内核cmdline）
修改配置文件，添加：`bootargs=<自定义bootargs>`，例如：
```
# Add cpu isolation configuration
bootargs=isolcpus=1-2
```

### 修改内核启动打印等级
修改配置文件，添加：`loglevel=<自定义打印等级>`，例如：
```
# Add kernel loglevel configuration
loglevel=8
```

### 临时修改dts
#### 使能或失能特定节点
修改配置文件，添加：`fdt-enable=<dts节点1全路径>;<dts节点2全路径>;`；`fdt-disable=<dts节点1全路径>;<dts节点2全路径>;`，例如：
```
# Enable kernel dts node
fdt-enable=/soc/uart@394C0000;

# Disable kernel dts node
fdt-disable=/soc/uart@394C0000;
```

:::info 提示
- 示例中配置行末尾的“;”不可省略；
- dts节点全路径可以在板端`/proc/device-tree`下获取，例如：
    ```shell
      root@ubuntu:~# realpath --relative-to=/proc/device-tree/ /proc/device-tree/soc/uart@394C0000
      soc/uart@394C0000
    ```
    注意命令获取到的路径需要添加行首的"/"；
:::

#### 配置DTB Overlay文件

DTB Overlay相关说明如下：
1. Kernel V6.1 官方文档：[Devicetree Overlay Notes](https://kernel.org/doc/html/v6.1/devicetree/overlay-notes.html);
2. Uboot V2022.10 官方文档：[Device Tree Overlays](https://docs.u-boot.org/en/v2022.10/usage/fdt_overlays.html);

简单介绍：DTB Overlay文件，是可以在不修改当前启动使用的dts文件的情况下，对当前启动使用的dtb文件进行**增/改**（不支持删减）的功能。

DTB Overlay示例文件如下：
```dts
/*
 * Sample dtb overlay source file
 * spi0_cs1_dev.dtso
*/
/dts-v1/;
/plugin/;

&spi0 {
	slave@1 {
		compatible = "sample-compatible-str";
		spi-max-frequency = <32000000>;
		reg = <1>;
	};
};

```
:::info 提示
需要根据从设备实际情况修改：
1. `compatible`字段：需要修改为从设备实际驱动的`compatible`字段；
2. `spi-max-frequency`字段：需要修改为从设备实际支持的最高速度；
:::

DTB Overlay编译示例，以下命令可以在Host端/RDK板端执行，假设`spi0_cs1_dev.dtso`文件路径位于`~/rdk_dtbo/spi0_cs1_dev.dtso`：
```
# Install device-tree-compiler
sudo apt install device-tree-compiler -y

# Compile dtbo files
dtc -I dts -O dtb -o ~/rdk_dtbo/spi0_cs1_dev.dtbo ~/rdk_dtbo/spi0_cs1_dev.dtso

# Copy generated dtbo file to /boot for further usage
sudo cp ~/rdk_dtbo/spi0_cs1_dev.dtbo /boot
```

编译示例输出：
```
sunrise@ubuntu:~/rdk_dtbo$ dtc -I dts -O dtb -o ~/rdk_dtbo/spi0_cs1_dev.dtbo ~/rdk_dtbo/spi0_cs1_dev.dtso
/home/sunrise/rdk_dtbo/spi0_cs1_dev.dtso:12.3-13: Warning (reg_format): /fragment@0/__overlay__/slave@1:reg: property has invalid length (4 bytes) (#address-cells == 2, #size-cells == 1)
/home/sunrise/rdk_dtbo/spi0_cs1_dev.dtbo: Warning (pci_device_reg): Failed prerequisite 'reg_format'
/home/sunrise/rdk_dtbo/spi0_cs1_dev.dtbo: Warning (pci_device_bus_num): Failed prerequisite 'reg_format'
/home/sunrise/rdk_dtbo/spi0_cs1_dev.dtbo: Warning (simple_bus_reg): Failed prerequisite 'reg_format'
/home/sunrise/rdk_dtbo/spi0_cs1_dev.dtbo: Warning (i2c_bus_reg): Failed prerequisite 'reg_format'
/home/sunrise/rdk_dtbo/spi0_cs1_dev.dtbo: Warning (spi_bus_reg): Failed prerequisite 'reg_format'
/home/sunrise/rdk_dtbo/spi0_cs1_dev.dtso:9.10-13.4: Warning (avoid_default_addr_size): /fragment@0/__overlay__/slave@1: Relying on default #address-cells value
/home/sunrise/rdk_dtbo/spi0_cs1_dev.dtso:9.10-13.4: Warning (avoid_default_addr_size): /fragment@0/__overlay__/slave@1: Relying on default #size-cells value
/home/sunrise/rdk_dtbo/spi0_cs1_dev.dtbo: Warning (avoid_unnecessary_addr_size): Failed prerequisite 'avoid_default_addr_size'
/home/sunrise/rdk_dtbo/spi0_cs1_dev.dtbo: Warning (unique_unit_address): Failed prerequisite 'avoid_default_addr_size'
sunrise@ubuntu:~/rdk_dtbo$ ls
spi0_cs1_dev.dtbo  spi0_cs1_dev.dtso
```
:::info 提示
一般来说，编译中报的`Warning`级别的打印可以忽略。
:::

修改配置文件，添加：`dtbo_file_path=</boot分区下的相对路径>`，例如：
```
# Set dtbo file path relative to /boot partition
dtbo_file_path=/spi0_cs1_dev.dtbo
```

重启后，可以看到设备树SPI0(spi@39800000)的路径下，新的从设备节点生成了：
```shell
sunrise@ubuntu:~$ ls /proc/device-tree/soc/spi@39800000/slave@1/
compatible  name  reg  spi-max-frequency
```

如果想要自定义dtbo文件所在分区，则可以添加：`dtbo_dev_part=<设备号>:<16进制分区号>`，RDK S100默认设备号为"0"，分区号可以通过`/dev/block/platform/by-name/`路径获取，以下以`userdata`分区为例：
```
# Set dtbo file device number and partition number:
dtbo_dev_part=0:0x10
```

获取分区号的方法：`ls -l /dev/block/platform/by-name/<分区名>`，例如：
```shell
root@ubuntu:~# ls -l /dev/block/platform/by-name/userdata
lrwxrwxrwx 1 root root 15 Jun  4 22:17 /dev/block/platform/by-name/userdata -> /dev/mmcblk0p16
```

## 自定义config.txt指南
地瓜Uboot会根据当前启动使用的储存介质和分区，自动获取默认的配置文件所在分区。

客户可以通过Uboot内的环境变量来自定义下一次启动使用的配置文件的储存介质和分区，步骤如下：
  1. 启动过程中停止并进入Uboot命令行；
  2. 以下环境变量可以用于自定义配置文件，每个变量均可单独使用：
     1. `boot_config_f`：改变默认寻找的配置文件名称，例如`setenv boot_config_f test.txt`，下一次启动的配置文件获取，就会去寻找文件名为`test.txt`的文件，而不是`config.txt`
     2. `boot_config_dev_part`：改变默认寻找配置文件的分区，例如`setenv boot_config_dev_part 0:0xd`，下一次启动的配置文件获取，就会去当前启动介质的第13个分区(0xd)寻找配置文件；
     3. `boot_config_intf`：改变默认寻找配置文件的储存介质，例如`setenv boot_config_intf scsi`。
  3. 保存环境变量：`saveenv`

## config.txt解析开发指南
配置文件的解析功能代码路径位于Uboot目录的：`board/hobot/common/drobot_boot_config.c`文件内。
