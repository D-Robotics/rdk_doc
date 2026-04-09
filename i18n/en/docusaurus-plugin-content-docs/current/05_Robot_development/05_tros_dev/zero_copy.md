---
sidebar_position: 1
---

# 5.5.1 Using "zero-copy"

```mdx-code-block
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';
```

## Feature Background

Communication is a fundamental capability in robotic development engines. Native ROS2 Foxy exhibits issues such as high latency and elevated system load when handling large-volume data communication. TogetheROS.Bot Foxy implements the "zero-copy" feature based on the RDK system software library hbmem, enabling zero-copy data transmission across processes. This significantly reduces transmission latency and system resource consumption for large data blocks. This section describes how to use tros.b Foxy and Humble to create publisher and subscriber nodes for large data transmission and measure the transmission latency.

:::info
- The tros.b Foxy version adds the "zero-copy" feature on top of ROS2 Foxy.
- The tros.b Humble version leverages the native "zero-copy" functionality provided by ROS2 Humble. For specific usage instructions, please refer to the official ROS2 [documentation](https://docs.ros.org/en/humble/Tutorials/Advanced/FastDDS-Configuration.html#) and [code example](https://github.com/ros2/demos/blob/humble/demo_nodes_cpp/src/topics/talker_loaned_message.cpp).
:::

## Prerequisites

You have successfully installed tros.b following the [Installation Guide](../01_quick_start/install_tros.md), and you are already familiar with basic ROS2 concepts such as nodes, topics, QoS policies, as well as how to create packages and use custom messages. Detailed tutorials can be found in the [official ROS2 documentation](https://docs.ros.org/en/foxy/Tutorials.html).

You also need ROS2 package building and compilation tools. Install them using the command: `sudo apt install ros-dev-tools`

## Task Steps

### 1. Create a Package

Open a new terminal, source the tros.b setup script to ensure the `ros2` command is available.

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

Use the following commands to create a workspace. For more details, refer to the ROS2 official tutorial on [Creating a workspace](https://docs.ros.org/en/foxy/Tutorials/Workspace/Creating-A-Workspace.html).

```shell
mkdir -p ~/dev_ws/src
cd ~/dev_ws/src
```

Run the following command to create a package:

```shell
ros2 pkg create --build-type ament_cmake hbmem_pubsub
```

### 2. Create a Custom Message

#### 2.1 Create a Message File

Run the following commands to create a `msg` directory for storing your custom message file:

```shell
cd ~/dev_ws/src/hbmem_pubsub
mkdir msg
```

Create a new file named `SampleMessage.msg` inside the `msg` directory with the following content:

```idl
int32 index
uint64 time_stamp
uint8[4194304] data

uint32 MAX_SIZE=4194304
```

#### 2.2 Build Dependencies

Navigate back to the `~/dev_ws/src/hbmem_pubsub` directory and modify `package.xml`. Add the following lines below `<buildtool_depend>ament_cmake</buildtool_depend>`:

```xml
<build_depend>rosidl_default_generators</build_depend>
<exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>
```

#### 2.3 Build Script

Modify `CMakeLists.txt` and add the following content below the line `# find_package(<dependency> REQUIRED)` to enable message compilation:

```cmake
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SampleMessage.msg"
)
```

### 3. Create a Message Publisher Node

#### 3.1 Create the Publisher Node File

Create a new file named `publisher_hbmem.cpp` in the `~/dev_ws/src/hbmem_pubsub/src` directory to implement the publisher node. The code and explanations are as follows:

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
    // Create a zero-copy publisher on the topic "topic"
    publisher_ = this->create_publisher_hbmem<hbmem_pubsub::msg::SampleMessage>(
        "topic", rclcpp::SensorDataQoS());

    // Set up a timer that calls timer_callback every 40 milliseconds to send messages
    timer_ = this->create_wall_timer(
        40ms, std::bind(&MinimalHbmemPublisher::timer_callback, this));
  }

 private:
  // Timer callback function
  void timer_callback() {
    // Borrow a loaned message for publishing
    auto loanedMsg = publisher_->borrow_loaned_message();
    // Check if the message is valid; borrowing might fail
    if (loanedMsg.is_valid()) {
      // Access the actual message via reference
      auto& msg = loanedMsg.get();
      
      // Get current timestamp in microseconds
      auto time_now =
          std::chrono::duration_cast<std::chrono::microseconds>(
              std::chrono::steady_clock::now().time_since_epoch()).count();
      
      // Assign values to index and time_stamp fields
      msg.index = count_;
      msg.time_stamp = time_now;
      
      // Log the message being sent
      RCLCPP_INFO(this->get_logger(), "message: %d", msg.index);
      publisher_->publish(std::move(loanedMsg));
      // Note: loanedMsg is no longer valid after publishing
      // Increment the counter
      count_++;
    } else {
      // Failed to borrow a message; skip this cycle
      RCLCPP_INFO(this->get_logger(), "Failed to get LoanMessage!");
    }
  }
  
  // Timer handle
  rclcpp::TimerBase::SharedPtr timer_;

  // Zero-copy publisher
  rclcpp::PublisherHbmem<hbmem_pubsub::msg::SampleMessage>::SharedPtr publisher_;
  
  // Message counter
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
    // Create publisher_hbmem with topic name "topic"
    publisher_ = this->create_publisher<hbmem_pubsub::msg::SampleMessage>(
        "topic", rclcpp::SensorDataQoS());

    // Timer that invokes timer_callback every 40 milliseconds to send messages
    timer_ = this->create_wall_timer(
        40ms, std::bind(&MinimalHbmemPublisher ::timer_callback, this));
  }

 private:
  // Timer callback function
  void timer_callback() {
    // Obtain the message to be sent
    auto loanedMsg = publisher_->borrow_loaned_message();
    // Check if the message is valid; it might become invalid if message acquisition fails
    if (loanedMsg.is_valid()) {
      // Get the actual message by reference
      auto& msg = loanedMsg.get();
      
      // Get current time in microseconds
      auto time_now =
          std::chrono::duration_cast<std::chrono::microseconds>(
              std::chrono::steady_clock::now().time_since_epoch()).count();
      
      // Assign values to index and time_stamp fields of the message
      msg.index = count_;
      msg.time_stamp = time_now;
      
      // Print the message being sent
      RCLCPP_INFO(this->get_logger(), "message: %d", msg.index);
      publisher_->publish(std::move(loanedMsg));
      // Note: after publishing, loanedMsg becomes invalid
      // Increment counter
      count_++;
    } else {
      // Failed to acquire message; discard this message
      RCLCPP_INFO(this->get_logger(), "Failed to get LoanMessage!");
    }
  }
  
  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

  // hbmem publisher
  rclcpp::Publisher<hbmem_pubsub::msg::SampleMessage>::SharedPtr publisher_;
  
  // Counter
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

#### 3.2 Build Dependencies

Return to the `~/dev_ws/src/hbmem_pubsub` directory and modify `package.xml`. Add the `rclcpp` dependency below the line `<member_of_group>rosidl_interface_packages</member_of_group>`:

```xml
  <depend>rclcpp</depend>
```

#### 3.3 Build Script

Modify `CMakeLists.txt` and add the following content below the `rosidl_generate_interfaces` statement to complete the publisher compilation:

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

### 4. Create a Message Subscriber Node

#### 4.1 Create a New Subscriber Node File

Create a new file named `subscriber_hbmem.cpp` in the `~/dev_ws/src/hbmem_pubsub/src` directory to implement the subscriber node. The specific code and explanations are as follows:

<Tabs groupId="tros-distro">
<TabItem value="foxy" label="Foxy">

```c++
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "hbmem_pubsub/msg/sample_message.hpp"

class MinimalHbmemSubscriber  : public rclcpp::Node {
 public:
  MinimalHbmemSubscriber () : Node("minimal_hbmem_subscriber") {
    // Create subscription_hbmem with topic name "topic"
    // Message callback function is topic_callback
    subscription_ =
        this->create_subscription_hbmem<hbmem_pubsub::msg::SampleMessage>(
            "topic", rclcpp::SensorDataQoS(),
            std::bind(&MinimalHbmemSubscriber ::topic_callback, this,
                      std::placeholders::_1));
  }

 private:
  // Message callback function
  void topic_callback(
      const hbmem_pubsub::msg::SampleMessage::SharedPtr msg) const {
    // Note: msg can only be used within the callback function; it will be released once the callback returns
    // Get current time
    auto time_now =
        std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now().time_since_epoch())
            .count();
    // Calculate latency and print it
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
    // Create subscription_hbmem with topic name "topic"
    // Message callback function is topic_callback
    subscription_ =
        this->create_subscription<hbmem_pubsub::msg::SampleMessage>(
            "topic", rclcpp::SensorDataQoS(),
            std::bind(&MinimalHbmemSubscriber ::topic_callback, this,
                      std::placeholders::_1));
  }

 private:
  // Message callback function
  void topic_callback(
      const hbmem_pubsub::msg::SampleMessage::SharedPtr msg) const {
    // Note: msg can only be used within the callback function; it will be released once the callback returns
    // Get current time
    auto time_now =
        std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now().time_since_epoch())
            .count();
    // Calculate latency and print it
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
#### 4.2 Compilation Script

Return to the `~/dev_ws/src/hbmem_pubsub` directory. Since the dependency on `rclcpp` has already been added to `package.xml`, there is no need to modify `package.xml`.

Modify `CMakeLists.txt` and add the following content below the `install` statement to complete the compilation of the subscriber:

```cmake
add_executable(listener src/subscriber_hbmem.cpp)
ament_target_dependencies(listener rclcpp)
rosidl_target_interfaces(listener
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS
  listener
  DESTINATION lib/${PROJECT_NAME})
```

### 5. Compilation

The complete workspace directory structure is as follows:

```shell
dev_ws/
└── src
    └── hbmem_pubsub
        ├── CMakeLists.txt
        ├── include
        │   └── hbmem_pubsub
        ├── msg
        │   └── SampleMessage.msg
        ├── package.xml
        └── src
            ├── publisher_hbmem.cpp
            └── subscriber_hbmem.cpp
```

The complete content of `package.xml` is as follows:

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

The complete content of `CMakeLists.txt` is as follows:

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

In the workspace root directory `~/dev_ws`, compile the package:

```shell
colcon build --packages-select hbmem_pubsub
```

If you receive a message indicating that the `colcon` command is not installed, install it using the following command:

```shell
sudo apt install ros-dev-tools
```

### 6. Execution

Open a new terminal, `cd` into the `dev_ws` directory, and source the tros.bash and current workspace setup file:

<Tabs groupId="tros-distro">
<TabItem value="foxy" label="Foxy">

```bash
source /opt/tros/setup.bash
cd ~/dev_ws
. install/setup.bash
# Run the talker node:
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
# Run the talker node:
ros2 run hbmem_pubsub talker
```

</TabItem>
</Tabs>


The terminal will display output similar to the following:

```text
[INFO] [1649227473.431381673] [minimal_hbmem_publisher]: message: 0
[INFO] [1649227473.470746697] [minimal_hbmem_publisher]: message: 1
[INFO] [1649227473.510923361] [minimal_hbmem_publisher]: message: 2
[INFO] [1649227473.550886783] [minimal_hbmem_publisher]: message: 3
[INFO] [1649227473.590664377] [minimal_hbmem_publisher]: message: 4
[INFO] [1649227473.630857041] [minimal_hbmem_publisher]: message: 5
```

Open another new terminal, again `cd` into the `dev_ws` directory, source the setup file, and then run the listener node:

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

The terminal will display the following output, indicating that the subscriber has successfully received messages sent by the publisher:

```text
[INFO] [1649227450.387089523] [minimal_hbmem_subscriber]: msg 10, time cost 1663us
[INFO] [1649227450.427071280] [minimal_hbmem_subscriber]: msg 11, time cost 1713us
[INFO] [1649227450.466993413] [minimal_hbmem_subscriber]: msg 12, time cost 1622us
[INFO] [1649227450.507029960] [minimal_hbmem_subscriber]: msg 13, time cost 1666us
[INFO] [1649227450.546146910] [minimal_hbmem_subscriber]: msg 14, time cost 998us
[INFO] [1649227450.587002681] [minimal_hbmem_subscriber]: msg 15, time cost 1768us
```

Press `Ctrl+C` to terminate each node.

## Summary of This Section

<Tabs groupId="tros-distro">
<TabItem value="foxy" label="Foxy">

If you are already familiar with using publishers and subscribers in ROS 2, it is straightforward to switch to zero-copy publishers and subscribers based on hbmem. You only need to make the following changes:

- Replace **rclcpp::Publisher** with **rclcpp::PublisherHbmem**
- Replace **create_publisher** with **create_publisher_hbmem**
- Replace **rclcpp::Subscription** with **rclcpp::SubscriptionHbmem**
- Replace **create_subscription** with **create_subscription_hbmem**
- Before sending a message, the **publisher** must first call **borrow_loaned_message** to obtain the message, **verify whether the message is available**, and only then assign values and publish it.
- The **subscription** processes the received message within its callback function, and **the received message can only be used inside the callback function**—it will be released once the callback finishes execution.

Notes:

- Using hbmem-based zero-copy consumes ION memory. If multiple publishers transmitting large messages are created, ION memory exhaustion may occur, causing creation failures.

- When creating a publisher, ION memory equal to three times the size of the KEEPLAST message buffer (capped at 256 MB) is allocated once for message transmission and will not be dynamically allocated afterward. If the subscriber fails to process messages correctly or in a timely manner, all message buffers may become occupied, preventing the publisher from obtaining an available message.

</TabItem>
<TabItem value="humble" label="Humble">

If you are already familiar with using publishers and subscribers in ROS 2, it is straightforward to switch to zero-copy publishers and subscribers. You only need to make the following changes:

- Before sending a message, the **publisher** must first call **borrow_loaned_message** to obtain the message, **verify whether the message is available**, and only then assign values and publish it.
- The **subscription** processes the received message within its callback function, and **the received message can only be used inside the callback function**—it will be released once the callback finishes execution.
- Before **running** your program, configure the zero-copy environment in the terminal using export commands.

</TabItem>
</Tabs>

## Usage Limitations

Compared to standard ROS 2 publisher/subscriber data transmission, zero-copy transmission has the following limitations:

- QoS History only supports KEEPLAST; KEEPALL is not supported. Additionally, KEEPLAST cannot be set too large due to memory constraints (currently capped at 256 MB total memory usage).
- The message size must be fixed; i.e., the `sizeof` value of the message must remain constant. Messages cannot contain variable-length data types such as strings or dynamic arrays.
- For TROS Humble, it is recommended to use RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT for QoS Reliability (we recommend directly using rclcpp::SensorDataQoS() to configure QoS). RMW_QOS_POLICY_RELIABILITY_RELIABLE exhibits stability issues in various communication scenarios.
- Zero-copy communication can only be used for inter-process communication on the same device and cannot be used across different devices.
- Publishers must first acquire a message before assigning values and publishing it, and must verify whether acquisition was successful.
- Messages received by subscribers are only valid within the callback function and cannot be used outside of it.