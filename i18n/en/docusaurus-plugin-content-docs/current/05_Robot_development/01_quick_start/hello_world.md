---
sidebar_position: 4
---
# 5.1.4 Hello World

```mdx-code-block
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';
```

Prerequisite: TogetheROS.Bot has been successfully installed via deb package or source code.

Open two terminals and ssh login to the horizon RDK device.

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

![hello world](./image/hello_world/hello_world.png "hello world")

As you can see, the left terminal acts as the publisher, continuously sending "Hello, world! N", and the right terminal acts as the subscriber, continuously receiving "Hello, world! N".

OK, tros.b has been successfully installed and verified!