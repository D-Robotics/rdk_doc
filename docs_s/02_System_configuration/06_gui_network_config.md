---
sidebar_position: 6
---

# 2.6 GUI 界面配置网络流程 （待更新图片）

本章节介绍在`Ubuntu`系统内通过 GUI 界面对 ETH 网络进行静态 `IP`、`DNS`、`Proxy` 配置的方法。

## 修改静态 IP、DNS 配置

1. 进入桌面后点击左下角打开应用列表，选择`settings`应用，在跳出来的界面中选择`Network`。

![image-show-app](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/02_System_configuration/image/gui_network_config/image-show-app.jpg)

2. `Realtek Ethernet` 和`Mircochip Ethernet`分别对应不同的物理网口配置，配置与实物对应如下:

![image-phy-eth](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/02_System_configuration/image/gui_network_config/image-phy-eth.png)

![image-sel-eth](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/02_System_configuration/image/gui_network_config/image-sel-eth.png)

3. 以修改`Reltek Ethernet` 为例,点击修改 eth1 选项中的齿轮，在跳出来的界面中选择`IPV4`, 选择`Manual`手动配置，在下方`Addresser`栏中写入`IP`地址，掩码和网关。

![image_set_static_ip](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/02_System_configuration/image/gui_network_config/image_set_static_ip.png)

4. 下拉，在下方 DNS 栏中输入 DNS 配置。

![image_set_static_dns](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/02_System_configuration/image/gui_network_config/image_set_static_dns.png)

5. 完成配置后注意选中`eth1`出现`√`选项

![image-sel_ok](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/02_System_configuration/image/gui_network_config/image-sel_ok.jpg)

## 修改 Proxy 配置

与修改静态`IP`类似, 修改`Proxy`配置步骤如下:

1. 进入桌面后点击左下角打开应用列表，选择`settings`应用，在跳出来的界面中选择`Network`。

2. 下拉选择`Network Proxy`的齿轮进入配置。

![image-proxy](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/02_System_configuration/image/gui_network_config/image-proxy.jpg)

3. 在跳出来的界面中填写所需配置即可。

![image-proxy_set](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/02_System_configuration/image/gui_network_config/image-proxy_set.png)
