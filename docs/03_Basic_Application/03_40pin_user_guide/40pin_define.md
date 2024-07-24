---
sidebar_position: 1
---

# 3.3.1 管脚定义与应用

开发板上的40PIN功能管脚，接口定义请查看[40PIN管脚定义](#40pin_define)章节。

## 管脚复用关系配置

40PIN的管脚会按照 [40PIN管脚定义](#40pin_define) 所示，默认使能UART、SPI、I2C、I2S等专用功能，如需将特定管脚配置成GPIO功能，需要通过`srpi-config`图形化配置工具进行配置。

注意，`srpi-config`程序需要在**全屏命令行窗口**中运行，方法如下：

```
sudo srpi-config
```

![image-20220511173307239](./image/40pin_user_guide/image-20220511173307239.png)

`okay`配置对应管脚为专用功能，`disabled`配置对应管脚为GPIO模式，配置 **重启** 后生效。

- 键盘上、下键选择功能项， 回车键开关功能
- 键盘左、右键选择  Select 和 Exit ，回车键确认

## 40PIN管脚定义{#40pin_define}

开发板提供40PIN标准接口，方便用户进行外围扩展，其中数字IO采用3.3V电平设计。40PIN接口定义如下：  
![image-20220828203147852](./image/40pin_user_guide/image-20220828203147852.png)

开发板40PIN接口位置提供了丝印网表，方便用户对照操作，PIN1、PIN40位置如下：  
![image-20220828203207798](./image/40pin_user_guide/image-20220828203207798.png)

:::info
RDK X3 Module 外扩40PIN管脚及其定义如下：
:::
![image-20230510155124570](./image/40pin_user_guide/image-20230510155124570.png)

:::info
RDK Ultra 外扩40PIN管脚及其定义如下：
:::
![image-20230510155124570](./image/40pin_user_guide/image-20230830194924570.png)

## GPIO读写操作示例

<iframe src="//player.bilibili.com/player.html?aid=700903305&bvid=BV1rm4y1E73q&cid=1196557887&page=16" scrolling="no" border="0" frameborder="no" framespacing="0" width="100%" height="500" allowfullscreen="true"> </iframe>

开发板 `/app/40pin_samples/` 目录下，预置了多种40PIN管脚的功能测试代码，包括gpio的输入/输出测试、PWM、I2C、SPI、UART等测试。所有测试程序均使用python语言编写，详细信息可以查阅 [40PIN 功能使用](../python_development/40pin_user_guide/40pin_define.md)。

以`/app/40pin_samples/button_led.py`为例，该程序配置`38`号管脚为输入，配置`36`号管脚配置为输出，并根据`38`号管脚的输入状态来控制`36`号管脚的输出状态。

## 环境准备
使用杜邦线连接`38`号管脚到3.3v or GND，以控制其高低电平。

## 运行方式
执行 `button_led.py` 程序，以启动GPIO读写程序

  ```bash
  sunrise@ubuntu:~$ cd /app/40pin_samples/
  sunrise@ubuntu:/app/40pin_samples$ sudo python3 ./button_led.py
  ```

## 预期效果
通过控制`38`号管脚的高低电平，可以改变 `36`号管脚的输出电平值。

  ```bash
  sunrise@ubuntu:/app/40pin_samples$ sudo python3 ./button_led.py
  Starting demo now! Press CTRL+C to exit
  Outputting 0 to Pin 36
  Outputting 1 to Pin 36
  Outputting 0 to Pin 36
  ```