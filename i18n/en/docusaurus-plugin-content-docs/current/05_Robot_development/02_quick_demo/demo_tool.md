---
sidebar_position: 7
---

# 5.2.7 Tools

```mdx-code-block
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';
```

## Image Publishing Tool

### Feature Overview

The image publishing tool supports batch reading of local image or video files and publishes them in ROS message format, thereby improving algorithm debugging and deployment efficiency.

For image publishing, it supports reading images in JPEG/JPG/PNG/NV12 formats and can either publish compressed images directly or convert compressed images to NV12 format before publishing.

For video publishing, it supports H264/H265/MP4 formats. After reading a video file, it extracts the relevant video stream for publishing.

Code repository: (https://github.com/D-Robotics/hobot_image_publisher.git)

### Supported Platforms

| Platform | Runtime Environment |
| ------- | ------------ |
| RDK X3, RDK X3 Module | Ubuntu 20.04 (Foxy), Ubuntu 22.04 (Humble) |
| RDK X5, RDK X5 Module, RDK S100 | Ubuntu 22.04 (Humble) |
| RDK Ultra | Ubuntu 20.04 (Foxy) |
| X86     | Ubuntu 20.04 (Foxy) |

:::caution
The X86 platform does not support decoding H.264 or H.265 videos into NV12 format; therefore, H.264/H.265 video publishing functionality cannot be demonstrated on X86 platforms.  
RDK Ultra does not support decoding H.264 videos into NV12 format; therefore, H.264 video publishing functionality cannot be demonstrated on RDK Ultra platforms.
:::

### Prerequisites

#### RDK Platform

1. RDK has been flashed with Ubuntu 20.04/Ubuntu 22.04 system image.

2. tros.b has been successfully installed on RDK.

3. A PC capable of accessing the RDK over the network.

#### X86 Platform

1. The X86 environment has been set up with Ubuntu 20.04 system image.

2. The X86 version of tros.b has been installed in the X86 environment.

### Image Publishing Usage Guide

Continuously read a single local NV12-format image and publish it. Use the image codec module to compress and encode the image into JPEG format, then display it on a PC web browser.

#### RDK/X86 Platform

<Tabs groupId="tros-distro">
<TabItem value="foxy" label="Foxy">

```bash
# Configure tros.b environment
source /opt/tros/setup.bash
```

</TabItem>

<TabItem value="humble" label="Humble">

```bash
# Configure tros.b environment
source /opt/tros/humble/setup.bash
```

</TabItem>

</Tabs>

```shell
# Copy required image files for the demo from tros.b installation path
cp -r /opt/tros/${TROS_DISTRO}/lib/hobot_image_publisher/config/ .

# Launch the launch file
ros2 launch hobot_image_publisher hobot_image_publisher_demo.launch.py
```

### Image Publishing Result Analysis

The following information appears in the terminal output:

```text
[INFO] [launch]: All log files can be found below /root/.ros/log/2022-08-19-12-58-02-288516-ubuntu-24492
[INFO] [launch]: Default logging verbosity is set to INFO
webserver has launch
[INFO] [hobot_image_pub-1]: process started with pid [24511]
[INFO] [hobot_codec_republish-2]: process started with pid [24513]
[INFO] [websocket-3]: process started with pid [24519]
```

The log output indicates that the webserver has launched successfully, and hobot_image_pub, hobot_codec_republish, and websocket are all running normally.

Enter `http://IP:8000` in a browser on your PC to view the displayed image (replace IP with the RDK/X86 device's IP address):

![hobot_img_pub](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/02_quick_demo/image/demo_tool/show.png)

### Video Publishing Usage Guide

Read the local `video.list` file to obtain video file paths listed within it, then continuously read and publish these video files. First, use the image codec module to decode the video stream into NV12-format images, then use the same module to compress and encode these images into JPEG format for display on a PC web browser.

#### RDK Platform

<Tabs groupId="tros-distro">
<TabItem value="foxy" label="Foxy">

```bash
# Configure tros.b environment
source /opt/tros/setup.bash
```

</TabItem>

<TabItem value="humble" label="Humble">

```bash
# Configure tros.b environment
source /opt/tros/humble/setup.bash
```

</TabItem>

</Tabs>

```shell
# Copy required configuration files for the demo from tros.b installation path
cp -r /opt/tros/${TROS_DISTRO}/lib/hobot_image_publisher/config/ .

# Launch the launch file
ros2 launch hobot_image_publisher hobot_image_publisher_videolist_demo.launch.py
```

#### X86 Platform

<Tabs groupId="tros-distro">
<TabItem value="foxy" label="Foxy">

```bash
# Configure tros.b environment
source /opt/tros/setup.bash
```

</TabItem>

<TabItem value="humble" label="Humble">

```bash
# Configure tros.b environment
source /opt/tros/humble/setup.bash
```

</TabItem>

</Tabs>

```shell
# Copy required configuration files for the demo from tros.b installation path
cp -r /opt/tros/${TROS_DISTRO}/lib/hobot_image_publisher/config/ .

# Launch the image publishing node using a local MP4 video file (parameters can be customized as needed). Web display is currently unsupported.
/opt/tros/${TROS_DISTRO}/lib/hobot_image_publisher/hobot_image_pub --ros-args -p image_source:=./config/video.list -p fps:=30 -p image_format:=mp4
```

### Video Publishing Result Analysis

The following information appears in the terminal output:

```text
[INFO] [launch]: All log files can be found below /root/.ros/log/2022-10-22-21-44-03-663907-ubuntu-702475
[INFO] [launch]: Default logging verbosity is set to INFO
webserver has launch
[INFO] [hobot_image_pub-1]: process started with pid [702597]
[INFO] [hobot_codec_republish-2]: process started with pid [702599]
[INFO] [hobot_codec_republish-3]: process started with pid [702601]
[INFO] [websocket-4]: process started with pid [702603]
```

The log output indicates that the webserver has launched successfully, and hobot_image_pub, hobot_codec_republish, and websocket are all running normally.

Enter `http://IP:8000` in a browser on your PC to view the displayed image (replace IP with the RDK/X86 device's IP address):

![hobot_img_pub](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/02_quick_demo/image/demo_tool/mp4show.jpg)


## Trigger Recording Tool

### Feature Overview
The so-called Trigger, based on a pre-defined Trigger mechanism, monitors changes in messages subscribed by the Trigger module—for example, changes in the number of detection bounding boxes or changes in cart control information—to trigger corresponding Trigger events and record ROS2 messages within a specified time interval. This helps developers locate and reproduce perception and planning/control issues in robotic scenarios.

The `trigger_node` package is a fundamental Trigger module developed by D-Robotics based on ROS2. It enables recording specified rosbag data upon triggering a Trigger event. The package supports directly subscribing to topics of type `ai_msg/msg/PerceptionTargets`. Within the topic callback function, it determines whether to trigger a Trigger event, records the associated rosbag upon triggering, saves the Trigger event information, and finally publishes the Trigger event as a topic of type `std_msgs/msg/String`.

The example presented in this chapter is a Trigger module developed by D-Robotics based on its custom Trigger base module. This example demonstrates subscribing to trash detection bounding box information and determining whether to trigger a Trigger event based on whether the number of detected trash bounding boxes is greater than or equal to 3. If the number of bounding boxes is ≥3, a Trigger event is triggered.

Code repository: (https://github.com/D-Robotics/hobot_trigger.git)

Application scenarios: Robot data closed-loop pipelines and robot Trigger event reporting scenarios. It can be integrated with perception, planning/control, and other tasks to record rosbag data at the moment a Trigger event occurs.

### Supported Platforms

| Platform                | Runtime Environment                          | Example Functionality                                           |
| ----------------------- | -------------------------------------------- | --------------------------------------------------------------- |
| RDK X3, RDK X3 Module   | Ubuntu 20.04 (Foxy), Ubuntu 22.04 (Humble)   | · Launch MIPI/USB camera; recorded rosbag data is saved locally |

### Usage Instructions

#### Trigger Initialization Configuration

The Trigger base module defines parameters required for initialization configuration.

The `config_file` uses JSON format, with the following specific configuration:

```bash
{ 
  "domain": Domain of the Trigger event. For example, robotic vacuum cleaners, humanoid robots, etc. Different Trigger types are distinguished by domain to categorize Triggers for different robotic application domains.

  "desc": Description of the Trigger module.

  "duration_ts_back": Duration (in milliseconds) to record after the Trigger event occurs.

  "duration_ts_front": Duration (in milliseconds) to record before the Trigger event occurs.
  
  "level": Priority level of the Trigger event. When multiple different Triggers occur simultaneously, a central node can be used to filter high-priority or low-priority Trigger events.
  
  "src_module_id": Module ID where the Trigger originates. Used to manage different Trigger modules to meet business requirements for managing multiple Trigger modules.
  
  "status": Trigger status: '0' = disabled, '1' = enabled.
  
  "strategy_version": Version number of the Trigger module's strategy.
  
  "topics": List of topics to be recorded, including topic names.
  
  "trigger_type": Trigger type ID. Each Trigger module may support multiple triggering conditions; for instance, detecting 2 pieces of trash is one type, while detecting 3 pieces is another.
  
  "unique_id": Unique device identifier.
  
  "version": Version information of the Trigger module.
  
  "extra_kv": Additional redundant or extensible information can be recorded here.
}
```

#### Trigger Event Triggering Configuration

In the `trigger_node` base class, a `Config` struct is defined. Some fields in this struct align with the initialization configuration, while others must be filled in based on actual conditions at the time of Trigger activation.

When users perform secondary development based on `trigger_node`, they only need to instantiate a struct variable each time a Trigger occurs, populate it with relevant information such as `"timestamp"`, `"gps_pos"`, etc., and push it into the Trigger event recording queue `"requests_"`.

Based on this, users can develop custom Trigger modules. For more details, please refer to the implementation of `trigger_node_example` in the code repository.

Code repository: (https://github.com/D-Robotics/hobot_trigger.git)

The struct definition is as follows:

```c++
struct Config {
  std::string domain;       // Domain of the Trigger event
  std::string desc;         // Description of the Trigger
  long duration_ts_back;    // Duration (ms) to record after Trigger occurrence
  long duration_ts_front;   // Duration (ms) to record before Trigger occurrence
  GPS_POS gps_pos;          // GPS position
  int level;                // Priority level
  std::string rosbag_path;  // Local file path of the rosbag generated after Trigger
  int src_module_id;        // Module ID where the Trigger originated
  int status;               // Trigger status
  std::string strategy_version; // Strategy version number
  long timestamp;           // Timestamp when the Trigger occurred
  std::vector<std::string> topics;    // List of topics to record, including topic names and types
  int trigger_type;         // Trigger type ID
  std::string unique_id;    // Unique device identifier
  std::string version;      // Trigger module version info
  std::vector<EXTRA_KV> extra_kv;   // Additional key-value information
};
```

### Prerequisites

#### RDK Platform

1. RDK has been flashed with Ubuntu 20.04 or Ubuntu 22.04 system image.
2. TogetherROS.Bot has been successfully installed on the RDK.
3. Install the package: `apt install tros-humble-trigger-node-example`.

### Usage Guide

#### RDK Platform

**Publish images using a MIPI camera**

<Tabs groupId="tros-distro">
<TabItem value="foxy" label="Foxy">

```bash
# Configure the TogetheROS.Bot environment
source /opt/tros/setup.bash
```

</TabItem>

<TabItem value="humble" label="Humble">

```bash
# Install mcap package
apt install ros-humble-rosbag2-storage-mcap

# Configure the TogetheROS.Bot environment
source /opt/tros/humble/setup.bash
```

</TabItem>

</Tabs>

```shell
# Copy required configuration files for the example from the TogetheROS.Bot installation path.
cp -r /opt/tros/${TROS_DISTRO}/lib/mono2d_trash_detection/config/ .
cp -r /opt/tros/${TROS_DISTRO}/lib/trigger_node_example/config/ .

# Configure MIPI camera
export CAM_TYPE=mipi

# Launch the example
ros2 launch trigger_node_example hobot_trigger_example.launch.py
```

**Publish images using a USB camera**

<Tabs groupId="tros-distro">
<TabItem value="foxy" label="Foxy">

```bash
# Configure the TogetheROS.Bot environment
source /opt/tros/setup.bash
```

</TabItem>

<TabItem value="humble" label="Humble">

```bash
# Configure the TogetheROS.Bot environment
source /opt/tros/humble/setup.bash
```

</TabItem>

</Tabs>

```shell
# Copy required configuration files for the example from the TogetheROS.Bot installation path.
cp -r /opt/tros/${TROS_DISTRO}/lib/mono2d_trash_detection/config/ .
cp -r /opt/tros/${TROS_DISTRO}/lib/trigger_node_example/config/ .

# Configure USB camera
export CAM_TYPE=usb

# Launch the example
ros2 launch trigger_node_example hobot_trigger_example.launch.py
```


### Result Analysis

**Publish images using MIPI camera**

After package initialization, the terminal outputs the following information:

```shell
  [INFO] [launch]: All log files can be found below /root/.ros/log/2023-05-13-17-31-53-158704-ubuntu-2981490
   [INFO] [launch]: Default logging verbosity is set to INFO
   [INFO] [trigger_node_example-1]: process started with pid [2981766]
   [trigger_node_example-1] [WARN] [1683970314.850652382] [hobot_trigger]: Parameter:
   [trigger_node_example-1]  cache_path: /home/hobot/recorder/
   [trigger_node_example-1]  config_file: config/trigger_config.json
   [trigger_node_example-1]  format: mcap
   [trigger_node_example-1]  isRecord(1:record, 0:norecord): 1
   [trigger_node_example-1]  agent_msg_sub_topic_name: /hobot_agent
   [trigger_node_example-1]  event_msg_sub_topic_name: /ai_msg_mono2d_trash_detection
   [trigger_node_example-1]  msg_pub_topic_name: /hobot_trigger
   [trigger_node_example-1]  config detail: {"domain":"robot","desc":"trigger lane","duration_ts_back":5000,"duration_ts_front":5000,"level":1,"rosbag_path":"","src_module_id":203,"timestamp":-1,"topic":["/image_raw/compressed","/ai_msg_mono2d_trash_detection"],"trigger_type":1110,"unique_id":"v1.0.0\n","version":"v1.0.0\n"}
   [trigger_node_example-1] [WARN] [1683970314.893573769] [hobot_trigger]: TriggerNode Init Succeed!
   [trigger_node_example-1] [WARN] [1683970314.898132256] [example]: TriggerExampleNode Init.
   [trigger_node_example-1] [WARN] [1683970315.931225440] [example]: Trigger Event!
   [trigger_node_example-1] [WARN] [1683970322.178604839] [rosbag2_storage_mcap]: no message indices found, falling back to reading in file order
   [trigger_node_example-1] [WARN] [1683970323.007470033] [hobot_trigger]: Trigger Event Report. Trigger module id: 203, type id: 1110
   [trigger_node_example-1]  Report message: {"domain":"","desc":"trigger lane","duration_ts_back":5000,"duration_ts_front":5000,"level":1,"rosbag_path":"trigger/OriginBot002_20230513-173155-931/OriginBot002_20230513-173155-931_0.mcap","src_module_id":203,"timestamp":1683970315931,"topic":["/image_raw/compressed","/ai_msg_mono2d_trash_detection"],"trigger_type":1110,"unique_id":"bot","version":"v1.0.0"}
```

After running, the rosbag data generated by the Trigger will be recorded in the "trigger" directory under the current working directory. The recorded rosbag data can be played back in Foxglove. For instructions on how to play rosbag files in Foxglove, please refer to Section 2.2 Data Visualization — Foxglove Visualization in the manual.

Playback effect in Foxglove:

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/02_quick_demo/image/demo_tool/trigger_example_trash_det.gif)

Note: This Trigger example records 5 seconds of data before and 5 seconds after the event occurs. Additionally, at the exact moment of the event, the reason for the Trigger activation is logged: a piece of trash was thrown into the scene, bringing the total number of trash items in the scene to three, thereby triggering the Trigger.


### Extended Features

#### Sending Tasks to the Trigger Module

The Trigger module supports receiving Trigger tasks from other nodes to control its configuration. Tasks are sent by publishing std_msgs/String messages containing JSON-formatted strings to a designated topic, delivering the task protocol to the Trigger module.

##### Trigger Task Protocol
```json
{
   "version": "v0.0.1_20230421",       // Version information of the Trigger module.
   "trigger_status": true,             // Trigger status: 'false' means disabled, 'true' means enabled.
   "strategy": [
      {
            "src_module_id": 203,      // Module ID where the Trigger originates.
            "trigger_type": 1110,      // Trigger type ID.
            "level": 1,                // Priority level of the Trigger event.
            "desc": "",                // Description of the Trigger module.
            "duration_ts_back": 5000,  // Duration (in ms) to record after the Trigger occurs.
            "duration_ts_front": 3000  // Duration (in ms) to record before the Trigger occurs.
      }
   ]
}
```


##### Execution

Based on the previously launched Trigger node, publish a std_msgs/String message to the topic "/hobot_agent" in another terminal.

<Tabs groupId="tros-distro">
<TabItem value="foxy" label="Foxy">

```bash
# Set up the tros.b environment
source /opt/tros/setup.bash
```

</TabItem>

<TabItem value="humble" label="Humble">

```bash
# Set up the tros.b environment
source /opt/tros/humble/setup.bash
```

</TabItem>

</Tabs>

```shell
# Publish a std_msgs/String message to the topic "/hobot_agent"
ros2 topic pub /hobot_agent std_msgs/String "data: '{\"version\":\"v0.0.1_20230421\",\"trigger_status\":true,\"strategy\":[{\"src_module_id\":203,\"trigger_type\":1110,\"status\":true,\"level\":1,\"desc\":\"test\",\"duration_ts_back\":5000,\"duration_ts_front\":3000}]}'"
```

##### Log Messages
```shell
   [WARN] [1691670626.026737642] [hobot_trigger]: TriggerNode Init Succeed!
   [WARN] [1691670626.026859316] [example]: TriggerExampleNode Init.
   [INFO] [1691670626.517232775] [TriggerNode]: Updated Trigger Config: {"domain":"robot","desc":"trigger lane","duration_ts_back":5000,"duration_ts_front":3000,"gps_pos":{"latitude":-1,"longitude":-1},"level":1,"rosbag_path":"","src_module_id":203,"strategy_version":"Robot_sweeper_V1.0_20230526","timestamp":0,"topic":["/image_raw/compressed","/ai_msg_mono2d_trash_detection","/hobot_visualization"],"trigger_type":1110,"unique_id":"OriginBot002","version":"v0.0.1_20230421","extra_kv":[]}
```
Analysis: When sending a configuration task to the Trigger module, the configuration of the Trigger node is successfully updated. (The updated log appears when the Trigger node's log level is set to INFO.)
```