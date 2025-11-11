---
sidebar_position: 2
---
## EtherCAT Usage Guide
:::warning
<font color="red">**Note:**</font> The EtherCAT protocol is mutually exclusive with the Ethernet protocol and **cannot coexist**.
:::
1. Start the EtherCAT service:
    ```
    systemctl start ethercat.service
    ```
2. Use user-space commands to check the master:
    ```
    sudo ethercat master
    # Sample output:
    sunrise@ubuntu:~$ sudo ethercat master
    Master0
      Phase: Idle
      Active: no
      Slaves: 0
      Ethernet devices:
        Main: c8:30:76:63:2d:93 (attached)
        Link: UP
        Tx frames:   9477
        Tx bytes:    568620
        Rx frames:   0
        Rx bytes:    0
        Tx errors:   0
        Tx frame rate [1/s]:    124    125     89
        Tx rate [KByte/s]:      7.3    7.3    5.2
        Rx frame rate [1/s]:      0      0      0
        Rx rate [KByte/s]:      0.0    0.0    0.0
      Common:
        Tx frames:   9477
        Tx bytes:    568620
        Rx frames:   0
        Rx bytes:    0
        Lost frames: 9477
        Tx frame rate [1/s]:    124    125     89
        Tx rate [KByte/s]:      7.3    7.3    5.2
        Rx frame rate [1/s]:      0      0      0
        Rx rate [KByte/s]:      0.0    0.0    0.0
        Loss rate [1/s]:        124    125     89
        Frame loss [%]:       100.0  100.0  100.0
    Distributed clocks:
      Reference clock:   None
      DC reference time: 0
      Application time:  0
    ```
3. Configure IgH service to start automatically at boot:
   ```
   sudo cp script/ethercat.service /lib/systemd/system/
   sudo systemctl enable ethercat
   ```

## EtherCAT Development Guide

### Software Stack
The Digua RDK S100 provides the EtherCAT-IgH v1.5 software stack by default. The [EtherCAT-IgH software stack](https://docs.etherlab.org/ethercat/1.5/pdf/ethercat_doc.pdf) is currently the mainstream open-source EtherCAT master protocol implementation.

EtherCAT official website: [EtherLab | EtherCAT](https://etherlab.org/en_GB/ethercat)  
EtherCAT open-source code repository: [Gitlab | EtherLab - EtherCAT](https://gitlab.com/etherlab.org/ethercat)

### Compilation and Deployment
#### Host-side Build
Host-side builds support two methods:
1. Build and deploy a standalone Debian package
   ```shell
   # Construct debian package
   ./mk_debs.sh hobot-ethercat

   # Deploy
   ## Transfer the generated out/product/deb_packages/hobot-ethercat_<***>_arm64.deb package to RDK S100, where "<***>" represents the version string
   ## On RDK S100, assuming the Debian package has been transferred to /userdata
   dpkg -i /userdata/hobot-ethercat_4.0.4-20250827135836_arm64.deb
   ```

   The hobot-ethercat build includes two major components: kernel modules and user-space applications. Building the kernel modules depends on locally built kernel artifacts. If the user hasn't built the kernel locally, the kernel module build step will be automatically skipped.

2. Full Build  
   By default, the rdk-gen build system integrates the hobot-ethercat Debian package (including both kernel modules and user-space applications) into the disk image after a full build.

#### On-device (Board-side) Build
1. Clone the source code:
   ```shell
   git clone https://gitlab.com/etherlab.org/ethercat.git -b stable-1.5
   ```
2. Build
   ```shell
   # Install build dependencies
   sudo apt install automake libtool m4 autoconf

   # Setup kernel module build environment
   sudo apt install flex bison
   sudo make -C /lib/modules/$(uname -r)/build prepare

   # Setup build environment
   cd ethercat
   ./bootstrap

   ./configure --enable-kernel --enable-generic --enable-igb --disable-eoe --enable-hrtimer --disable-8139too --with-linux-dir=/lib/modules/$(uname -r)/build/

   # Compile and install
   make -j
   make modules -j
   sudo make install
   sudo make modules_install
   ```
3. Edit `/usr/local/etc/ethercat.conf` and add the following:
   ```
   MASTER0_DEVICE="eth0" // Device or MAC
   DEVICE_MODULES="generic"
   ```