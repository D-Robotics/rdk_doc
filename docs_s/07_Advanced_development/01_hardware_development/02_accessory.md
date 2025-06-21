---
sidebar_position: 2
---

# 7.1.2 配件清单

:::info

配件清单根据适配情况，定期更新

:::

认证配件清单是指经过 D-Robotics 官方认证，可以适配 RDK S100 开发板的第三方配件清单。主要包含基础配件(电源、外壳、散热器)、摄像头、显示屏等类别，清单内容会不定期更新，逐步添加更多的配件型号。

## 基础配件{#basic_accessories}

| 类型 | 供应商 | 型号        | 描述           | 购买链接                                          |
| ---- | ------ | ----------- | -------------- | ------------------------------------------------- |
| 电源 | 悠品   | PD-1904740T | 19V/4.74A 电源 | [购买链接](https://item.jd.com/100048009312.html) |

## 摄像头

| 型号                              | 软件支持                              | 描述                                                                                                                               | 供应商   | 购买链接                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   |
| --------------------------------- | ------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------- | -------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| IMX219 摄像头（兼容树莓派 5）     | 支持 V4L2 接口及 HBN 接口             | <li>IMX219 传感器</li><li>800W 像素</li><li> FOV 120 度</li><li>MIPI 接口</li>                                                     | 微雪     | [购买链接](https://www.waveshare.net/shop/IMX219-120-Camera.htm)                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           |
| SC230AI 双目摄像头（V3 硬件版本） | 支持 HBN 接口                         | <li>SC230AI 传感器</li><li>200W 像素</li><li>FOV 178 度</li><li>MIPI 接口</li>                                                     | 亚博     | [购买连接](https://item.taobao.com/item.htm?spm=a1z0d.6639537%2F202410.item.d893698797289.57177484f3aFnW&id=893698797289&from=cart&skuId=5743852141425&pisk=gGwSXq90eUY5z6kpAbj2h4Y-9QHQRiWNdHiLjkpyvYH-RvZQXXpFZYlIdr34U4rre2MQAlhL82ryqS4aXLyPZvJIEvDd7NWNQQroKvd-0HY9rqnIx9J-ppHoHzuA1KXNQuqX40IaDOyeid1I2BpLJbhvkD0x20nJeiMxjDDpeXpJME3mkvp-wXEYDcnp2vppwqdx40LJ2LhpDt3mvv3LJvIjHqmKaPXj0_g8Vg535mYFGOESlppLhmQoWu_WdmymVVu_289ppMmSNVE-laL2yRhYxXwHxpGL254nXzLRR22bD8FTWZRry5iQbWaAYBa_l5HjmSbBOVNL45VKGgB8cYgStmhJfIH79z2IZ7thjoH8o50iwsbmc8yZOVcXkaE4c4h_Oz7Dp4P_DrGa3EJZB54bdfHO4gpZ5c7HOnGMdmgNciOHtITJ5uFVG7rjwmm27isXPBc-mmgNciOHtbnm0GSfcUOh.) |
| SC132GS 双目摄像头                | 支持 HBN 接口                         | <li>SC132GS 传感器</li><li>200W 像素</li><li> FOV 157 度</li><li>MIPI 接口</li>                                                    | 御光     | 原厂直采                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   |
| SG8S-AR0820C-5300-G2A             | 支持 V4L2 接口及 HBN 接口             | <li>ONSEMI 8.3MP AR0820 RGGB 传感器</li><li>FOV 120 度</li><li>1920\*1080@30fps</li><li>加串器 MAXIM max9295a</li>GMSL2/FAKRA 接口 | 森云智能 | [购买链接](https://www.sensing-world.com/pd.jsp?id=26)                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     |
| LEC28736A11（X3C 模组）           | 支持 HBN 接口                         | <li>OX03C10</li><li>200W 像素</li><li>RGGB 传感器</li><li>加串器 MAXIM MAX96717F</li><li>GMSL2/FAKRA 接口</li>                     | 联创电子 | 原厂直采                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   |
| Intel Realsense D457              | 支持 V4L2 接口，暂不支持 librealsense | <li>Z16 深度数据</li><li>1280x720@30fps</li><li>加串器 MAXIM MAX9295A</li><li>GMSL2/FAKRA 接口</li>                                | Intel    | [购买链接](https://www.intelrealsense.com/depth-camera-d457/)                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              |
| Intel RealSense D435i             | 支持 V4L2 接口                        | <li>深度传感器 1280x720@90fps FOV 87°×58°</li> <li>RGB 传感器 1920x1080@30fps FOV 69°×42°</li><li>USB 接口</li>                    | Intel    | [购买链接](https://store.intelrealsense.com/buy-intel-realsense-depth-camera-d435i.html)                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   |

## WiFi 模组

| 类型           | 供应商     | 型号               | 描述                                                                                                                                                   | 购买链接                                            |
| -------------- | ---------- | ------------------ | ------------------------------------------------------------------------------------------------------------------------------------------------------ | --------------------------------------------------- |
| M.2 2230 E key | D-Robotics | RDK WIFI Module    | 基于 Infineon CYW55572 ，Wi-Fi 6 + Bluetooth 5.3，双频并发，支持 1024-QAM 和 OFDMA，高速低延迟，工规产品，适用于高性能边缘设备、工业物联网和智能终端。 | 请联系售后                                          |
| M.2 2230 E key | AzureWave  | AW-XM612MA-PUR     | 基于 Infineon CYW55572 ，Wi-Fi 6 + Bluetooth 5.3，双频并发，支持 1024-QAM 和 OFDMA，高速低延迟，工规产品，适用于高性能边缘设备、工业物联网和智能终端。 | 请联系售后                                          |
| M.2 2230 E key | gxlinkstar | RTL8822CE+内置天线 | 2.4G/5GHz 双频，不支持蓝牙使用                                                                                                                         | [购买链接](https://item.jd.com/10115715024347.html) |

## 传感器

### IMU

| 类型  | 供应商   | 型号                  | 描述                                     | 购买链接                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    |
| ----- | -------- | --------------------- | ---------------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| RS485 | 维特智能 | WT61PC-485+485 连接线 | <li>六轴加速度计</li><li>六轴陀螺仪</li> | [购买链接](https://detail.tmall.com/item.htm?abbucket=8&id=598376182207&pisk=gOls0DNIoGj1FlFAlRYFOsoQIsPjGeRrBZaxrqCNk5FODsiZyA7qgAkfGDiuHsuwIETbAmqaQ5ENGSgjmmyxgolfSXi5g1-MIqCbgSKy4QRrSVP0M3rWI97RSrzfDGI4WkFLgrGi-qQrSVV0DwSvUv3GlXIGsiUYDeNLlzXTMPCA9eabPoFYX-IKJrELMSEA62IL7z6OHsnA9BUgoGBYBseLpr4uMonYBegLxrNYDv9NOrpTyVTwWLchWOHmWk1AMfdgCusLfy4EOM2gVVh1Gs_05RZ-WktOq3X8pq2jiZfbxVHomznOXnZmO2hte0KFQlH7l4kjvEWjVDM8Ofw1k3FZfXFK1RAlN0ebfvhTdZCx72ZLFxi6fOqtQDwuJJQXZyozvVctdEjKW0r_6yeewUgLHvlgEf-d17Ho75kx2HWaXYaA4-CzVBYlhwwlGyZyRe6cn9sOVVKfVkh8By464eTCjtyT-yZWRes1eRU35DLBRGBd.&rn=8b71112fa9ac193128b4a8625125e912&skuId=5660200207716&spm=a1z10.3-b-s.w4011-23008269807.65.52764094tkJGsV) |

### Lidar

| 类型     | 供应商 | 型号         | 描述                  | 购买链接                                                   |
| -------- | ------ | ------------ | --------------------- | ---------------------------------------------------------- |
| Ethernet | 觅道   | LIVOXMID-360 | <li>多线激光雷达</li> | [购买链接](https://store.dji.com/cn/product/livox-mid-360) |

## 显示屏

| 类型 | 供应商 | 型号            | 描述                                         | 购买链接                                                                          |
| ---- | ------ | --------------- | -------------------------------------------- | --------------------------------------------------------------------------------- |
| HDMI | 微雪   | 10 英寸触控屏   | 分辨率 1280x800，钢化玻璃面板，高色域触控屏  | [购买链接](https://www.waveshare.net/shop/10.1HP-CAPLCD-Monitor.htm)              |
| HDMI | 微雪   | 13.3 英寸触控屏 | 分辨率 1920x1080，钢化玻璃面板，高色域触控屏 | [购买链接](https://www.waveshare.net/shop/13.3inch-HDMI-LCD-H-with-Holder-V2.htm) |

## 转接板

| 类型       | 供应商 | 型号             | 描述                                     | 购买链接                                                        |
| ---------- | ------ | ---------------- | ---------------------------------------- | --------------------------------------------------------------- |
| 音频转接板 | 微雪   | Audio Driver HAT | 支持环形 4MIC 录音，双通道播放，音频回采 | [购买链接](https://www.waveshare.net/shop/Audio-Driver-HAT.htm) |
