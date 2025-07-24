---
sidebar_position: 4
---

# 5.1.4 运行“Hello World”

```mdx-code-block
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';
```

前提：已通过deb包或者源码安装的方式成功安装TogetheROS.Bot

启动两个终端，均ssh登陆至RDK或X86平台设备

<Tabs groupId="tros-distro">
<TabItem value="foxy" label="Foxy">

第一个终端运行

```shell
source /opt/tros/setup.bash
ros2 run examples_rclcpp_minimal_subscriber subscriber_member_function
```

第二个终端运行

```shell
source /opt/tros/setup.bash
ros2 run examples_rclcpp_minimal_publisher publisher_member_function
```

</TabItem>

<TabItem value="humble" label="Humble">

安装`Hello World` example对应的package：

```shell
sudo apt update
sudo apt install ros-humble-examples-rclcpp-minimal-publisher ros-humble-examples-rclcpp-minimal-subscriber
```

:::caution **注意**
**如果`sudo apt update`命令执行失败或报错，请查看[常见问题](/docs/08_FAQ/01_hardware_and_system.md)章节的`Q10: apt update 命令执行失败或报错如何处理？`解决。**
:::

第一个终端运行

```shell
source /opt/tros/humble/setup.bash
ros2 run examples_rclcpp_minimal_subscriber subscriber_member_function
```

第二个终端运行

```shell
source /opt/tros/humble/setup.bash
ros2 run examples_rclcpp_minimal_publisher publisher_member_function
```

</TabItem>

</Tabs>


运行效果如下图

![hello world](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/01_quick_start/image/hello_world/hello_world.png)
可以看到左侧终端作为pub，在不断发送“'Hello, world! N”，右侧终端作为sub端不断收到“'Hello, world! N”

OK tros.b目前已成功安装并验证！
