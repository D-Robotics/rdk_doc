---
sidebar_position: 2
---
## EtherCAT使用指南
:::warning
<font color="red">**注意：**</font>EtherCAT协议与以太网协议互斥，**无法共存**。
:::
1. 启动EtherCAT服务：
    ```
    systemctl start ethercat.service
    ```
2. 使用用户层命令检查主站：
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
3. 配置IgH服务自启动：
   ```
   sudo cp script/ethercat.service /lib/systemd/system/
   sudo systemctl enable ethercat
   ```

## EtherCAT开发指南

### 软件栈
地瓜RDK S100 默认提供EtherCAT-IgH 1.5版本软件栈。[EtherCAT-IgH软件栈](https://docs.etherlab.org/ethercat/1.5/pdf/ethercat_doc.pdf)是目前主流的开源EtherCAT主站协议。

EtherCAT官网：[EtherLab | EtherCAT](https://etherlab.org/en_GB/ethercat)
EtherCAT开源代码仓库：[Gitlab | EtherLab - EtherCAT](https://gitlab.com/etherlab.org/ethercat)

### 编译和部署
#### Host端构建
Host端构建支持两种构建方式：
1. 单独编译debian包并部署
   ```shell
   # Construct debian package
   ./mk_debs.sh hobot-ethercat

   # Deploy
   ## Transfer generated out/product/deb_packages/hobot-ethercat_<***>_arm64.deb package to RDK S100, the "<***>" is the version string
   ## On RDK S100, presuming debian package is transferred to /userdata
   dpkg -i /userdata/hobot-ethercat_4.0.4-20250827135836_arm64.deb
   ```

   hobot-ethercat的构建包括内核模块+用户层应用两个大模块。内核模块构建依赖用户本地构建的内核输出物。在用户没有在本地构建过内核时会自动跳过内核模块的构建。

2. 整编 \
   默认rdk-gen构建系统在整编disk镜像后，镜像内会默认集成hobot-ethercat debian 包，内部包括内核模块及用户层应用。

#### 板端构建
1. 下载源码：
   ```shell
   git clone https://gitlab.com/etherlab.org/ethercat.git -b stable-1.5
   ```
2. 构建
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
3. 编辑`/usr/local/etc/ethercat.conf`，添加以下内容：
   ```
   MASTER0_DEVICE="eth0" // Device or MAC
   DEVICE_MODULES="generic"
   ```
