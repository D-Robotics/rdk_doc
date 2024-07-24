---
sidebar_position: 11
---

# 修改BPU保留内存大小

## 临时设置BPU保留内存

当ion选择cma作为内存池时，通过cma区域来为BPU mem划分空间，以此兼顾cma的灵活性和预留空间的效率，该空间大小可在系统启动后通过修改/sys节点修改：

```bash
echo 100 > /sys/class/misc/ion/cma_carveout_size
```

通过上述方式修改该空间大小单位为Mbyte，需根据实际场景需求做不同配置(如多路场景下若vio报ion_alloc错误，则可适当降低该空间大小，最小可到0)，设置0值表示BPU只使用cma来动态分配(若无sys该节点表示该版本暂不支持此方式配置)。

注意：只有当没有用户使用BPU_MEM时才可以修改成功，由于该空间为从cma中申请的连续物理地址空间，可申请到的最大大小无法达到cma总大小。当BPU_MEM无法从该空间中申请到足够的内存时，系统会尝试从该空间之外的cma空间申请。

因为这部分预留是从cma中分配出的连续物理空间，所以设置有可能失败，设置了之后再cat一下这个节点，成功的话就是设置的值，失败就是0。

## 在设备树中设置ion_cam size

1、串口或者ssh终端登录X3Pi

2、确认当前硬件使用的dtb文件

RDK X3是 `hobot-x3-pi.dtb`  
RDK X3 Module是 `hobot-x3-cm.dtb`

可以通过 `cat /sys/firmware/devicetree/base/model`  命令确定

3、使用以下命令把dtb文件转成方便阅读的dts文件：

```
dtc -I dtb -O dts -o hobot-x3-pi.dts /boot/hobot/hobot-x3-pi.dtb 
```

其中，/boot/hobot/hobot-x3-pi.dtb 是要编辑的DTB文件的路径。该命令将DTB文件转换为DTS文件（设备树源文件）。
在文本编辑器中，可以编辑DTS文件并保存更改。

4、修改 ion size

打开dts文件后， 找到 ion_cma 节点，修改 alloc-ranges 和 size 属性中的 0x2a000000 为需要的内存大小值，在修改此值之前，请确保明确了解它的含义，包括允许的设置范围。

```
ion_cma {
		compatible = "shared-dma-pool";
		alloc-ranges = <0x00 0x4000000 0x00 0x2a000000>;
		alignment = <0x00 0x100000>;
		size = <0x00 0x2a000000>;
		reusable;
};
```

例如，如果要将 ion_cma size 设置为 1.5GB，可以将其更改为下面的示例。

```
ion_cma {
		compatible = "shared-dma-pool";
		alloc-ranges = <0x00 0x4000000 0x00 0x5dc00000>;
		alignment = <0x00 0x100000>;
		size = <0x00 0x5dc00000>;
		reusable;
};
```

5、保存修改后，使用以下命令将 DTS 文件转换回 DTB 格式。在执行此操作之前，请备份原始文件。

```
dtc -I dts -O dtb -o /boot/hobot/hobot-x3-pi.dtb hobot-x3-pi.dts
```

保存后，建议将其转换回 dts 文件并确认修改是否正确，以避免因笔误等原因导致修改的值不符合预期。

6、最后，重启您的系统以使更改生效

注意事项：

- 修改DTB文件可能会影响您的系统的稳定性和安全性。在修改DTB文件之前，请确保您了解您要更改的内容的含义，并备份原始DTB文件以防止意外错误。
- /boot/hobot/ 下文件由地平线软件包管理，如果升级了系统软件，则用户的修改会被重置为默认配置（672MB）
