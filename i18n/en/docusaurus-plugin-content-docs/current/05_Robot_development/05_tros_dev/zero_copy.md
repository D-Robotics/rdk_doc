---
sidebar_position: 1
---
# 5.5.1 Using "zero-copy"

```mdx-code-block
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';
```

## Background

Communication is a fundamental function of the robot development engine. When using native ROS for large-scale data communication, there may be issues such as high latency and system load. TogetheROS.Bot implements the "zero-copy" feature based on the Horizon Systems software library hbmem, which enables zero-copy transmission of data across processes, greatly reducing transmission latency and system resource usage for large data blocks. This section explains how to use the tros.b hbmem interface to create a publisher and subscriber node for large-scale data transmission, and calculate transmission latency.

:::info
- The tros.b Foxy version adds a "zero-copy" function based on ROS2 Foxy.
- The tros.b Humble version uses the "zero-copy" function of ROS2 Humble. For specific usage, please refer to the ROS2 official [document](https://docs.ros.org/en/humble/Tutorials/Advanced/FastDDS-Configuration.html#) and [code](https://github.com/ros2/demos/blob/humble/demo_nodes_cpp/src/topics/talker_loaned_message.cpp).
:::

## Prerequisites

1. tros.b has been successfully installed following the guide [Installation](../quick_start/install_tros.md).
2. Familiarity with ROS2 nodes, topics, QoS, as well as creating packages and using custom messages. For detailed tutorials, please refer to the [official ROS2 documentation](https://docs.ros.org/en/foxy/Tutorials.html).
3. The ROS2 package build system ament_cmake has been installed. Installation command: `apt update; apt-get install python3-catkin-pkg; pip3 install empy`.
4. The ROS2 build tool colcon has been installed. Installation command: `pip3 install -U colcon-common-extensions`.

## Usage

### 1. Create a package

Open a new terminal and source the tros.b setup script to ensure that the `ros2` command can be run.

<Tabs groupId="tros-distro">
<TabItem value="foxy" label="Foxy">

```bash
source /opt/tros/setup.bash
```

</TabItem>
<TabItem value="humble" label="Humble">

```bash
source /opt/tros/humble/setup.bash
```

</TabItem>
</Tabs>

Create a workspace using the following command. For detailed instructions, refer to the ROS2 official tutorial [Creating a workspace](https://docs.ros.org/en/foxy/Tutorials/Workspace/Creating-A-Workspace.html).

```shell
mkdir -p ~/dev_ws/src
cd ~/dev_ws/src
```

Run the following command to create a package.

```shell
ros2 pkg create --build-type ament_cmake hbmem_pubsub
```

### 2. Create a custom message

#### 2.1 Create a message file

Run the following command to create a `msg` directory to store the custom message file.

```shell
cd ~/dev_ws/src/hbmem_pubsub
mkdir msg
```

In the `msg` directory, create a new file named `SampleMessage.msg` with the following content:

```idl
int32 index
uint64 time_stamp
uint8[4194304] data

uint32 MAX_SIZE=4194304
```

#### 2.2 Dependency Compilation

Return to the `~/dev_ws/src/hbmem_pubsub` directory and modify `package.xml`. Add the following content under `<buildtool_depend>ament_cmake</buildtool_depend>`:

```xml
  <build_depend>rosidl_default_generators</build_depend>
  <exec_depend>rosidl_default_runtime</exec_depend>
  <member_of_group>rosidl_interface_packages</member_of_group>
```

#### 2.3 Compilation Script

Modify `CMakeLists.txt` and add the following content under `# find_package(<dependency> REQUIRED)` to compile the msg:

```cmake
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SampleMessage.msg"
)
```

### 3. Create Message Publishing Node

#### 3.1 Create a New Message Publishing Node File

In the `~/dev_ws/src/hbmem_pubsub/src` directory, create a new file named `publisher_hbmem.cpp`. This file is used to create the publisher node. The code and explanation are as follows:

<Tabs groupId="tros-distro">
<TabItem value="foxy" label="Foxy">

```c++
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "hbmem_pubsub/msg/sample_message.hpp"

using namespace std::chrono_literals;

class MinimalHbmemPublisher : public rclcpp::Node {
 public:
  MinimalHbmemPublisher() : Node("minimal_hbmem_publisher"), count_(0) {
    // Create publisher_hbmem, topic is "topic", QoS is KEEPLAST(10), and default reliability
    publisher_ = this->create_publisher_hbmem<hbmem_pubsub::msg::SampleMessage>(
        "topic", 10);

    // Timer, calls timer_callback every 40 milliseconds for message publishing
    timer_ = this->create_wall_timer(
        40ms, std::bind(&MinimalHbmemPublisher::timer_callback, this));
  }

 private:
  // Timer callback function
  void timer_callback() {
    // Get the message to send
    auto loanedMsg = publisher_->borrow_loaned_message();
    // Check if the message is valid, it may become invalid if borrowing the message fails
    if (loanedMsg.is_valid()) {
      // Get the actual message by reference
      auto& msg = loanedMsg.get();
      
      // Get the current time in microseconds
      auto time_now =
          std::chrono::duration_cast<std::chrono::microseconds>(
              std::chrono::steady_clock::now().time_since_epoch()).count();
      
      // Assign values to the index and time_stamp of the message
      msg.index = count_;
      msg.time_stamp = time_now;
      
      // Print the message to be sent
      RCLCPP_INFO(this->get_logger(), "message: %d", msg.index);
      publisher_->publish(std::move(loanedMsg));
      // Note that loanedMsg is no longer valid after publishing
      // Increment the counter
      count_++;
    } else {
      // Failed to get the message, discard it
      RCLCPP_INFO(this->get_logger(), "Failed to get LoanMessage!");
    }
  }
  
  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

  // Hbmem publisher
  rclcpp::PublisherHbmem<hbmem_pubsub::msg::SampleMessage>::SharedPtr publisher_;

  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalHbmemPublisher>());
  rclcpp::shutdown();
  return 0;
}

```

</TabItem>
<TabItem value="humble" label="Humble">

```c++
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "hbmem_pubsub/msg/sample_message.hpp"

using namespace std::chrono_literals;

class MinimalHbmemPublisher  : public rclcpp::Node {
 public:
  MinimalHbmemPublisher () : Node("minimal_hbmem_publisher"), count_(0) {
    publisher_ = this->create_publisher<hbmem_pubsub::msg::SampleMessage>(
        "topic", rclcpp::SensorDataQoS());

    timer_ = this->create_wall_timer(
        40ms, std::bind(&MinimalHbmemPublisher ::timer_callback, this));
  }

 private:
  void timer_callback() {
    auto loanedMsg = publisher_->borrow_loaned_message();
    if (loanedMsg.is_valid()) {
      auto& msg = loanedMsg.get();
      
      auto time_now =
          std::chrono::duration_cast<std::chrono::microseconds>(
              std::chrono::steady_clock::now().time_since_epoch()).count();
      
      msg.index = count_;
      msg.time_stamp = time_now;
      
      RCLCPP_INFO(this->get_logger(), "message: %d", msg.index);
      publisher_->publish(std::move(loanedMsg));
      count_++;
    } else {
      RCLCPP_INFO(this->get_logger(), "Failed to get LoanMessage!");
    }
  }
  
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<hbmem_pubsub::msg::SampleMessage>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalHbmemPublisher>());
  rclcpp::shutdown();
  return 0;
}
```

</TabItem>
</Tabs>

#### 3.2 Compilation Dependencies

Go back to the `~/dev_ws/src/hbmem_pubsub` directory, modify the `package.xml`, and add the `rclcpp` dependency under `<member_of_group>rosidl_interface_packages</member_of_group>`:


```xml
  <depend>rclcpp</depend>
```
#### 3.3 Compilation Script

Modify `CMakeLists.txt`, add the following content below the `rosidl_generate_interfaces` statement to complete the publisher compilation:

```cmake
find_package(rclcpp REQUIRED)

add_executable(talker src/publisher_hbmem.cpp)
ament_target_dependencies(talker rclcpp)
rosidl_target_interfaces(talker
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS
  talker
  DESTINATION lib/${PROJECT_NAME})
```

### 4. Create Message Reception Node

#### 4.1 Create a New Message Reception Node File

Create a new file `subscriber_hbmem.cpp` in the `~/dev_ws/src/hbmem_pubsub/src` directory to establish a subscriber node. The specific code and explanation are as follows:

<Tabs groupId="tros-distro">
<TabItem value="foxy" label="Foxy">

```c++
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "hbmem_pubsub/msg/sample_message.hpp"


class MinimalHbmemSubscriber  : public rclcpp::Node {
 public:
  MinimalHbmemSubscriber () : Node("minimal_hbmem_subscriber") {
    // Create subscription_hbmem with topic "sample", QoS as KEEPLAST(10), and default reliability
    // Message callback function is topic_callback
    subscription_ =
        this->create_subscription_hbmem<hbmem_pubsub::msg::SampleMessage>(
            "topic", 10,
            std::bind(&MinimalHbmemSubscriber ::topic_callback, this,
                      std::placeholders::_1));
  }

 private:
  // Message callback function
  void topic_callback(
      const hbmem_pubsub::msg::SampleMessage::SharedPtr msg) const {
    // Note that msg can only be used within the callback function, the message will be released after the callback function returns
    // Get current time
    auto time_now =
        std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now().time_since_epoch())
            .count();
    // Calculate delay and print it out
    RCLCPP_INFO(this->get_logger(), "msg %d, time cost %dus", msg->index,
                time_now - msg->time_stamp);
  }
  
  // hbmem subscription
  rclcpp::SubscriptionHbmem<hbmem_pubsub::msg::SampleMessage>::SharedPtr
      subscription_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalHbmemSubscriber>());
  rclcpp::shutdown();
  return 0;
}

```

</TabItem>
<TabItem value="humble" label="Humble">

```c++
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "hbmem_pubsub/msg/sample_message.hpp"

class MinimalHbmemSubscriber  : public rclcpp::Node {
 public:
  MinimalHbmemSubscriber () : Node("minimal_hbmem_subscriber") {
    subscription_ =
        this->create_subscription<hbmem_pubsub::msg::SampleMessage>(
            "topic", rclcpp::SensorDataQoS(),
            std::bind(&MinimalHbmemSubscriber ::topic_callback, this,
                      std::placeholders::_1));
  }

 private:
  void topic_callback(
      const hbmem_pubsub::msg::SampleMessage::SharedPtr msg) const {
    auto time_now =
        std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now().time_since_epoch())
            .count();
    RCLCPP_INFO(this->get_logger(), "msg %d, time cost %dus", msg->index,
                time_now - msg->time_stamp);
  }
  
  // hbmem subscription
  rclcpp::Subscription<hbmem_pubsub::msg::SampleMessage>::SharedPtr
      subscription_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalHbmemSubscriber>());
  rclcpp::shutdown();
  return 0;
}
```

</TabItem>
</Tabs>

#### 4.2 Build script

Go back to the `~/dev_ws/src/hbmem_pubsub` directory, as we have added the `rclcpp` dependency in the `package.xml`, there is no need to modify it.

Modify the `CMakeLists.txt` file, add the following content below the `install` statement, to complete the build of the subscriber:

```cmake
add_executable(listener src/subscriber_hbmem.cpp)
ament_target_dependencies(listener rclcpp)
rosidl_target_interfaces(listener
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS
  listener
  DESTINATION lib/${PROJECT_NAME})
```


### 5. Build

The directory structure of the entire workspace is as follows:

```shell
dev_ws/
└── src
    └── hbmem_pubsub
        ├── CMakeLists.txt
        ├── include
        │   └── hbmem_pubsub
        ├── msg
        │   └── SampleMessage.msg
        ├── package.xml
        └── src
            ├── publisher_hbmem.cpp
            └── subscriber_hbmem.cpp
```

entire `package.xml`as follows:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>hbmem_pubsub</name>
  <version>0.0.0</version>
  <description>TODO: Package description</description>
  <maintainer email="root@todo.todo">root</maintainer>
  <license>TODO: License declaration</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <build_depend>rosidl_default_generators</build_depend>
  <exec_depend>rosidl_default_runtime</exec_depend>
  <member_of_group>rosidl_interface_packages</member_of_group>

  <depend>rclcpp</depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>

```

The complete `CMakeLists.txt` content is as follows:
```cmake
cmake_minimum_required(VERSION 3.5)
project(hbmem_pubsub)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SampleMessage.msg"
)

find_package(rclcpp REQUIRED)

add_executable(talker src/publisher_hbmem.cpp)
ament_target_dependencies(talker rclcpp)
rosidl_target_interfaces(talker
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS
  talker
  DESTINATION lib/${PROJECT_NAME})

add_executable(listener src/subscriber_hbmem.cpp)
ament_target_dependencies(listener rclcpp)
rosidl_target_interfaces(listener
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS
  listener
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

```


In the workspace root directory `~/dev_ws`, build the package:

```shell
colcon build --packages-select hbmem_pubsub
```

If the `colcon` command is not installed, use the following command to install it:

```shell
pip3 install -U colcon-common-extensions
```

### 6. Run

Open a new terminal, cd to the dev_ws directory, source tros.b and the current workspace setup file:

<Tabs groupId="tros-distro">
<TabItem value="foxy" label="Foxy">

```bash
source /opt/tros/setup.bash
cd ~/dev_ws
. install/setup.bash
ros2 run hbmem_pubsub talker
```

</TabItem>
<TabItem value="humble" label="Humble">

```bash
source /opt/tros/humble/setup.bash
cd ~/dev_ws
. install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/opt/tros/humble/lib/hobot_shm/config/shm_fastdds.xml
export RMW_FASTRTPS_USE_QOS_FROM_XML=1
export ROS_DISABLE_LOANED_MESSAGES=0
ros2 run hbmem_pubsub talker
```

</TabItem>
</Tabs>

On the terminal, the following prints will appear:

```text
[INFO] [1649227473.431381673] [minimal_hbmem_publisher]: message: 0
[INFO] [1649227473.470746697] [minimal_hbmem_publisher]: message: 1
[INFO] [1649227473.510923361] [minimal_hbmem_publisher]: message: 2
[INFO] [1649227473.550886783] [minimal_hbmem_publisher]: message: 3
[INFO] [1649227473.590664377] [minimal_hbmem_publisher]: message: 4
[INFO] [1649227473.630857041] [minimal_hbmem_publisher]: message: 5
```

Open another terminal and `cd` to the `dev_ws` directory, then source the setup file and run the listener node:

<Tabs groupId="tros-distro">
<TabItem value="foxy" label="Foxy">

```bash
source /opt/tros/setup.bash
cd ~/dev_ws
. install/setup.bash

ros2 run hbmem_pubsub listener
```

</TabItem>
<TabItem value="humble" label="Humble">

```bash
source /opt/tros/humble/setup.bash
cd ~/dev_ws
. install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/opt/tros/humble/lib/hobot_shm/config/shm_fastdds.xml
export RMW_FASTRTPS_USE_QOS_FROM_XML=1
export ROS_DISABLE_LOANED_MESSAGES=0
ros2 run hbmem_pubsub listener
```

</TabItem>
</Tabs>

On the terminal, the following prints will appear, indicating that the subscriber has successfully received the messages sent by the publisher:

```text
[INFO] [1649227450.387089523] [minimal_hbmem_subscriber]: msg 10, time cost 1663us
[INFO] [1649227450.427071280] [minimal_hbmem_subscriber]: msg 11, time cost 1713us
[INFO] [1649227450.466993413] [minimal_hbmem_subscriber]: msg 12, time cost 1622us
[INFO] [1649227450.507029960] [minimal_hbmem_subscriber]: msg 13, time cost 1666us
[INFO] [1649227450.546146910] [minimal_hbmem_subscriber]: msg 14, time cost 998us
[INFO] [1649227450.587002681] [minimal_hbmem_subscriber]: msg 15, time cost 1768us
```

You can use `Ctrl+C` to end the execution of each node.

## Summary of this section

<Tabs groupId="tros-distro">
<TabItem value="foxy" label="Foxy">

If you have learned how to use ROS2 publishers and subscribers, it is easy to switch to using hbmem publishers and subscribers. You just need to make the following changes:

- **rclcpp::Publisher** to **rclcpp::PublisherHbmem**
- **create_publisher** to **create_publisher_hbmem**
- **rclcpp::Subscription** to **rclcpp::SubscriptionHbmem**
- **create_subscription** to **create_subscription_hbmem**
- In the publisher, call **borrow_loaned_message** to get the message, then **check if the message is available**, and if it is, assign the value and send it.
- In the subscriber, process the received message in the callback function, and **the received message can only be used in the callback function**. Once the callback function is executed, the message will be released.

Note:- Using zero-copy based on hbmem will occupy ion memory. If multiple large messages publishers are created, there may be insufficient ion memory, resulting in creation failure issues.

- When creating a publisher, ion memory that is three times the size of KEEPLAST multiplied by the number of messages will be requested at one time (up to 256MB maximum), which is used for message transmission and will not be dynamically allocated afterwards. If there is an error in message handling on the subscriber side or if it is not processed in a timely manner, the message buffer may be fully occupied, causing the publisher to continuously fail to obtain available messages.

</TabItem>
<TabItem value="humble" label="Humble">

If you have mastered how to use ROS2 publisher and subscriber, it is easy to switch to using zero-copy publisher and subscriber. You only need to make the following changes when using it:

- **publisher** must first call **borrow_loaned_message** to obtain the message before sending it, and then **confirm whether the message is available**. If it is available, assign it and send it.
- **subscription** processes the received message in the callback function, and **the received message can only be used** in the callback function. After the callback function is executed, the message will be released
- Before **running** the program, use the export command to configure a zero-copy environment in the running terminal.

</TabItem>
</Tabs>

## Limitations

Compared to the publisher/subscriber data transmission method in ROS2, using zero-copy transmission based on hbmem has the following limitations:

- QOS History only supports KEEPLAST and does not support KEEPALL. Additionally, KEEPLAST cannot be set too large due to memory limitations. Currently, it is set to a maximum of 256MB memory usage.
- The size of the transmitted message is fixed, meaning the `sizeof` value of the message remains unchanged and cannot include variable-length data types such as strings or dynamic arrays.
- It can only be used for inter-process communication on the same device and cannot be transmitted across devices.
- Publisher messages need to be obtained first and then assigned for sending, and it needs to be checked if the acquisition is successful.
- The validity period of received messages on the subscriber side is limited to the callback function and cannot be used outside of the callback function.