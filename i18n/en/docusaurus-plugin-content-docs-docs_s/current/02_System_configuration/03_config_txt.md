---
sidebar_position: 3
---

# 2.3 config.txt File Configuration
:::warning
- All settings in configuration files can be manually overridden within U-Boot. Manually configured settings in U-Boot (using `setenv` in the U-Boot command line) take **precedence over** those in configuration files. The full environment variable priority order is: `setenv > config file > last boot's saveenv`;
- Throughout this chapter, the term "configuration file" refers to the file located at the **default path** `/boot/config.txt`;
- Using a configuration file requires modifying the contents of the boot partition, which conflicts with the requirements of [AVB](https://source.android.com/docs/security/features/verifiedboot). Therefore, this feature is unavailable when AVB is enabled (AVB is **disabled by default**);
:::

## Usage Guide
:::info Note
- The default format of the configuration file is `<key>=<value>`. Everything after the first `=` constitutes the value for the `<key>` preceding it;
- Each line in the configuration file must not exceed 1024 characters;
- Settings in the configuration file are not saved as U-Boot's default environment by default;
:::

### Configure kernel bootargs (kernel cmdline)
Edit the configuration file and add: `bootargs=<custom bootargs>`, for example:
```
# Add cpu isolation configuration
bootargs=isolcpus=1-2
```

### Modify kernel boot log level
Edit the configuration file and add: `loglevel=<custom log level>`, for example:
```
# Add kernel loglevel configuration
loglevel=8
```

### Temporarily modify the device tree (DTS)
#### Enable or disable specific nodes
Edit the configuration file and add:  
`fdt-enable=<full path of DTS node 1>;<full path of DTS node 2>;`  
`fdt-disable=<full path of DTS node 1>;<full path of DTS node 2>;`  
For example:
```
# Enable kernel dts node
fdt-enable=/soc/uart@394C0000;

# Disable kernel dts node
fdt-disable=/soc/uart@394C0000;
```

:::info Note
- The trailing semicolon (`;`) in the example lines **must not be omitted**;
- The full path of a DTS node can be obtained from `/proc/device-tree` on the target board. For example:
    ```shell
      root@ubuntu:~# realpath --relative-to=/proc/device-tree/ /proc/device-tree/soc/uart@394C0000
      soc/uart@394C0000
    ```
    Note that a leading `/` must be prepended to the path obtained from this command;
:::

#### Configure DTB Overlay Files

References for DTB Overlay:
1. Kernel v6.1 official documentation: [Devicetree Overlay Notes](https://kernel.org/doc/html/v6.1/devicetree/overlay-notes.html);
2. U-Boot v2022.10 official documentation: [Device Tree Overlays](https://docs.u-boot.org/en/v2022.10/usage/fdt_overlays.html);

Brief introduction: A DTB Overlay file allows **adding or modifying** (but not removing) nodes in the currently used DTB without altering the original DTS file used for booting.

Example DTB Overlay source file:
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
:::info Note
The following fields must be adjusted according to your actual peripheral device:
1. `compatible` field: must match the actual driver's `compatible` string of the peripheral;
2. `spi-max-frequency` field: must be set to the maximum SPI frequency actually supported by the peripheral;
:::

DTB Overlay compilation example (runnable on either host or RDK board). Assuming the `spi0_cs1_dev.dtso` file is located at `~/rdk_dtbo/spi0_cs1_dev.dtso`:
```
# Install device-tree-compiler
sudo apt install device-tree-compiler -y

# Compile dtbo files
dtc -I dts -O dtb -o ~/rdk_dtbo/spi0_cs1_dev.dtbo ~/rdk_dtbo/spi0_cs1_dev.dtso

# Copy generated dtbo file to /boot for further usage
sudo cp ~/rdk_dtbo/spi0_cs1_dev.dtbo /boot
```

Example compilation output:
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
:::info Note
Generally, `Warning`-level messages during compilation can be safely ignored.
:::

Edit the configuration file and add: `dtbo_file_path=<relative path under /boot partition>`, for example:
```
# Set dtbo file path relative to /boot partition
dtbo_file_path=/spi0_cs1_dev.dtbo
```

After rebooting, you can verify that the new slave device node appears under the SPI0 (`spi@39800000`) node in the device tree:
```shell
sunrise@ubuntu:~$ ls /proc/device-tree/soc/spi@39800000/slave@1/
compatible  name  reg  spi-max-frequency
```

To specify a custom partition for the DTBO file, add: `dtbo_dev_part=<device number>:<hex partition number>`. On RDK S100, the default device number is "0". The partition number can be obtained from the `/dev/block/platform/by-name/` path. For example, using the `userdata` partition:
```
# Set dtbo file device number and partition number:
dtbo_dev_part=0:0x10
```

How to obtain the partition number: `ls -l /dev/block/platform/by-name/<partition name>`, for example:
```shell
root@ubuntu:~# ls -l /dev/block/platform/by-name/userdata
lrwxrwxrwx 1 root root 15 Jun  4 22:17 /dev/block/platform/by-name/userdata -> /dev/mmcblk0p16
```

## Custom config.txt Guide
Digua U-Boot automatically determines the default partition containing the configuration file based on the current boot storage medium and partition.

Users can customize the storage medium and partition for the configuration file used on the next boot via U-Boot environment variables, as follows:
  1. Halt the boot process and enter the U-Boot command line;
  2. The following environment variables can be used to customize the configuration file location (each can be used independently):
     1. `boot_config_f`: changes the default configuration filename. For example, `setenv boot_config_f test.txt` will cause the next boot to look for a file named `test.txt` instead of `config.txt`;
     2. `boot_config_dev_part`: changes the default partition where the configuration file is searched. For example, `setenv boot_config_dev_part 0:0xd` will cause the next boot to look for the configuration file in partition 13 (0xd) of the current boot medium;
     3. `boot_config_intf`: changes the default storage interface. For example, `setenv boot_config_intf scsi`;
  3. Save the environment variables: `saveenv`

## config.txt Parsing Development Guide
The source code implementing configuration file parsing is located in the U-Boot directory at: `board/hobot/common/drobot_boot_config.c`.