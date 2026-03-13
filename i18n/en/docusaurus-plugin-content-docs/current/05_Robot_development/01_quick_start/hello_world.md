---
sidebar_position: 4
---
# 5.1.4 Hello World

```mdx-code-block
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';
```

Prerequisite: TogetheROS.Bot has been successfully installed via deb package or source code.

Open two terminals and ssh login to the RDK device.

<Tabs groupId="tros-distro">
<TabItem value="foxy" label="Foxy">

In the first terminal, run:

```shell
source /opt/tros/setup.bash
ros2 run examples_rclcpp_minimal_subscriber subscriber_member_function
```

In the second terminal, run:

```shell
source /opt/tros/setup.bash
ros2 run examples_rclcpp_minimal_publisher publisher_member_function
```

</TabItem>

<TabItem value="humble" label="Humble">

Install the package corresponding to the `Hello World` example:

```shell
sudo apt update
sudo apt install ros-humble-examples-rclcpp-minimal-publisher ros-humble-examples-rclcpp-minimal-subscriber
```

:::caution **Note**
**If the `sudo apt update` command fails or reports an error, please refer to `Q10: How to solve the failure or error of the apt update command?` in the [FAQ](../../08_FAQ/01_hardware_and_system.md) section for solutions.**
:::

In the first terminal, run:

```shell
source /opt/tros/humble/setup.bash
ros2 run examples_rclcpp_minimal_subscriber subscriber_member_function
```

In the second terminal, run:

```shell
source /opt/tros/humble/setup.bash
ros2 run examples_rclcpp_minimal_publisher publisher_member_function
```

</TabItem>

</Tabs>

The running effect is shown in the following image:

![hello world](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/01_quick_start/image/hello_world/hello_world.png)

As you can see, the left terminal acts as the publisher, continuously sending "Hello, world! N", and the right terminal acts as the subscriber, continuously receiving "Hello, world! N".

OK, tros.b has been successfully installed and verified!