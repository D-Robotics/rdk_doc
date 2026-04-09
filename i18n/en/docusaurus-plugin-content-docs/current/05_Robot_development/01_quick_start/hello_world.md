---
sidebar_position: 4
---
# 5.1.4 Hello World

```mdx-code-block
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';
```

Prerequisite: TogetheROS.Bot has been successfully installed via either the deb package or source code.

Open two terminals and SSH into your RDK or x86 platform device in both.

<Tabs groupId="tros-distro">
<TabItem value="foxy" label="Foxy">

Run the following in the first terminal:

```shell
source /opt/tros/setup.bash
ros2 run examples_rclcpp_minimal_subscriber subscriber_member_function
```

Run the following in the second terminal:

```shell
source /opt/tros/setup.bash
ros2 run examples_rclcpp_minimal_publisher publisher_member_function
```

</TabItem>

<TabItem value="humble" label="Humble">

Install the packages corresponding to the "Hello World" example:

```shell
sudo apt update
sudo apt install ros-humble-examples-rclcpp-minimal-publisher ros-humble-examples-rclcpp-minimal-subscriber
```

:::caution **Note**  
**If the `sudo apt update` command fails or returns an error, please refer to the FAQ section [Common Issues](../../08_FAQ/01_hardware_and_system.md), specifically `Q10: How to resolve failures or errors when running apt update?` for solutions.**
:::

Run the following in the first terminal:

```shell
source /opt/tros/humble/setup.bash
ros2 run examples_rclcpp_minimal_subscriber subscriber_member_function
```

Run the following in the second terminal:

```shell
source /opt/tros/humble/setup.bash
ros2 run examples_rclcpp_minimal_publisher publisher_member_function
```

</TabItem>

</Tabs>


The execution result is shown below:

![hello world](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/01_quick_start/image/hello_world/hello_world.png)

As shown, the left terminal acts as the publisher (pub), continuously sending "'Hello, world! N", while the right terminal acts as the subscriber (sub), continuously receiving "'Hello, world! N".

Great! tros.b has now been successfully installed and verified!