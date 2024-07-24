---
sidebar_position: 11
---
# Modifying BPU Reserved Memory Size

## Temporarily setting BPU reserved memory

When ion selects cma as the memory pool, the space for BPU mem is allocated through the cma region to balance the flexibility of cma and the efficiency of reserved space. The size of this space can be modified after system startup by modifying the /sys node:

```bash
echo 100 > /sys/class/misc/ion/cma_carveout_size
```

The above command modifies the size of the space in Mbyte. Different configurations should be made based on actual scenario requirements (e.g., if vio reports ion_alloc error in multi-way scenario, the size of the space can be reduced accordingly, with a minimum value of 0). Setting the value to 0 indicates that BPU only uses cma for dynamic allocation (if there is no sys node, it means this version does not support this configuration method).

Note: The modification can only be successful when BPU_MEM is not used by any user. Since this space is a continuous physical address space allocated from cma, the maximum size that can be allocated cannot reach the total size of cma. When BPU_MEM cannot allocate enough memory from this space, the system will attempt to allocate from cma space outside this space.

Because this reserved space is allocated from cma as a continuous physical space, setting it may fail. After setting it, you can use the "cat" command to check the value of this node. If it shows the set value, it means the setting is successful. If it shows 0, it means the setting failed.

## Setting ion_cam size in device tree

1. Log in to X3Pi through a serial port or ssh terminal.

2. Confirm the dtb file currently used by the hardware.

For RDK X3, it is `hobot-x3-pi.dtb`.
For RDK X3 Module, it is `hobot-x3-cm.dtb`.

You can use the `cat /sys/firmware/devicetree/base/model` command to determine it.

3. Use the following command to convert the dtb file into a readable dts file:

```
dtc -I dtb -O dts -o hobot-x3-pi.dts /boot/hobot/hobot-x3-pi.dtb 
```

where `/boot/hobot/hobot-x3-pi.dtb` is the path of the DTB file to be edited. This command converts the DTB file to a DTS file (device tree source file). In a text editor, you can edit the DTS file and save the changes.

4. Modify the ion size

After opening the dts file, locate the ion_cma node, and modify the 0x2a000000 in the alloc-ranges and size attributes to the desired memory size value. Before modifying this value, make sure you have a clear understanding of its meaning, including the allowed range of settings.


```
ion_cma {
	compatible = "shared-dma-pool";
	alloc-ranges = <0x00 0x4000000 0x00 0x5dc00000>;
	alignment = <0x00 0x100000>;
	size = <0x00 0x5dc00000>;
	reusable;
};
```

For example, if you want to set the ion_cma size to 1.5GB, you can change it as follows.

```
ion_cma {
	compatible = "shared-dma-pool";
	alloc-ranges = <0x00 0x4000000 0x00 0x5dc00000>;
	alignment = <0x00 0x100000>;
	size = <0x00 0x5dc00000>;
	reusable;
};
```

5. Save the changes and convert the DTS file back to DTB format using the following command. Make sure to backup the original file before performing this operation.

```
dtc -I dts -O dtb -o /boot/hobot/hobot-x3-pi.dtb hobot-x3-pi.dts
```

After saving, it is recommended to convert it back to a dts file and confirm if the modifications are correct to avoid unexpected errors due to typos or other reasons.

6. Finally, restart your system to apply the changes.

Note:

- Modifying the DTB file may affect the stability and security of your system. Before modifying the DTB file, make sure you understand the meaning of the content you want to change and backup the original DTB file to prevent accidental errors.
- The files under /boot/hobot/ are managed by the D-Robotics package. If the system software is upgraded, user modifications will be reset to the default configuration (672MB).