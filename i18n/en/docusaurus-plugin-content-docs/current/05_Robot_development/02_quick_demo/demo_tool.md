---
sidebar_position: 5
---
# 5.2.7 Tools

```mdx-code-block
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';
```

## Image Publishing Tool

### Introduction

The Image Publishing Tool supports reading local image or video files in batches and publishing them in ROS message, thereby improving algorithm debugging and deployment efficiency.

For image publishing, it supports reading JPEG/JPG/PNG/NV12 format images and publishing compressed images or converting compressed images to NV12 format for publishing.

For video publishing, it supports H264/H265/MP4 formats. After reading the video file, it extracts the relevant video stream for publishing.

Code repository:  (https://github.com/D-Robotics/hobot_image_publisher.git)

### Supported Platforms

| Platform | System |
| ---------| ----------------|
| RDK X3, RDK X3 Module, RDK X5  | Ubuntu 20.04 (Foxy), Ubuntu 22.04 (Humble) |

### Preparations

#### RDK

1. The RDK has been burned with the  Ubuntu 20.04/22.04 system image provided by D-Robotics.

2. The RDK has successfully installed tros.b.

3. The RDK can be accessed via network from a PC.

### Usage of a image

Read a local NV12 image in a loop and publish it. Use the image codec module to compress the image and encode it into JPEG format, and display the image on the PC's web interface.

#### RDK

<Tabs groupId="tros-distro">
<TabItem value="foxy" label="Foxy">

```bash
# Configure the tros.b environment
source /opt/tros/setup.bash
```

</TabItem>

<TabItem value="humble" label="Humble">

```bash
# Configure the tros.b environment
source /opt/tros/humble/setup.bash
```

</TabItem>

</Tabs>

```shell
# Copy the configuration file required for running the example from the installation path of tros.b.
cp -r /opt/tros/${TROS_DISTRO}/lib/hobot_image_publisher/config/ .

# Start the launch file
ros2 launch hobot_image_publisher hobot_image_publisher_demo.launch.py
```

### Result Analysis

The terminal output during runtime is as follows:

```text
[INFO] [launch]: All log files can be found below /root/.ros/log/2022-08-19-12-58-02-288516-ubuntu-24492
[INFO] [launch]: Default logging verbosity is set to INFO
webserver has launch
[INFO] [hobot_image_pub-1]: process started with pid [24511]
[INFO] [hobot_codec_republish-2]: process started with pid [24513]
[INFO] [websocket-3]: process started with pid [24519]
```

The log output shows that the webserver has been started, and hobot_image_pub, hobot_codec_republish, and websocket are all running properly.

To view the image display effect, open a web browser on the PC and enter  `http://IP:8000` (where IP is the IP address of the RDK):

![hobot_img_pub](/../static/img/05_Robot_development/02_quick_demo/image/demo_tool/show.png )

### Usage of a video

Read the video.list file locally, obtain the paths of the video files in the list file, and publish them in a loop. First, use the image codec module to decode the video stream into NV12 format images, and then use the image codec module to compress and encode the images into JPEG format for display on the web of the PC.

#### RDK

<Tabs groupId="tros-distro">
<TabItem value="foxy" label="Foxy">

```bash
# Configure the tros.b environment
source /opt/tros/setup.bash
```

</TabItem>

<TabItem value="humble" label="Humble">

```bash
# Configure the tros.b environment
source /opt/tros/humble/setup.bash
```

</TabItem>

</Tabs>

```shell
# Copy the image files needed for the example from the installation path of tros.b
cp -r /opt/tros/lib/hobot_image_publisher/config/ .

# Start the launch file
ros2 launch hobot_image_publisher hobot_image_publisher_videolist_demo.launch.py
```

### Result Analysis

The following information is displayed in the terminal output during execution:

```text
[INFO] [launch]: All log files can be found below /root/.ros/log/2022-10-22-21-44-03-663907-ubuntu-702475
[INFO] [launch]: Default logging verbosity is set to INFO
webserver has launched
[INFO] [hobot_image_pub-1]: process started with pid [702597]
[INFO] [hobot_codec_republish-2]: process started with pid [702599]
[INFO] [hobot_codec_republish-3]: process started with pid [702601]
[INFO] [websocket-4]: process started with pid [702603]
```

The output log shows that the webserver has started and hobot_image_pub, hobot_codec_republish, and websocket are all running normally.

To view the image display effect, enter `http://IP:8000` in the browser on the PC (where IP is the IP address of the RDK):

![hobot_img_pub](/../static/img/05_Robot_development/02_quick_demo/image/demo_tool/mp4show.jpg )


## Trigger Recording Tool

### Introduction

Trigger is a mechanism that detects changes in the subscribed messages of the Trigger module, such as changes in the number of detection boxes or changes in car control information. It triggers corresponding events to record the specified time interval of ROS2 messages, helping developers locate and reproduce perception, control, and other issues in robot scenes.

The trigger_node package is a Trigger module developed by D-Robotics based on ROS2, which is used to obtain specified rosbag data after triggering events. The package supports direct subscription to ai_msg/msg/PerceptionTargets topics. In the topic callback function, Trigger events are triggered and rosbag packages related to the Trigger events are recorded. Finally, the Trigger event information is saved and published in std_msg/msg/String type Trigger event topics.

This chapter provides an example of using the Trigger module. The example demonstrates the functionality of subscribing to garbage detection box information and triggering a Trigger event based on whether the number of garbage detection boxes is greater than or equal to 3.

Code Repository:  (https://github.com/D-Robotics/hobot_trigger.git)

Application Scenarios: data closed-loop link, robot Trigger event reporting scenario, can be combined with perception, control, and other tasks to record rosbag data when Trigger events occur.

### Supported Platforms

| Platform | System | Function |
| --- | --- | --- |
| RDK X3, RDK X3 Module| Ubuntu 20.04 (Foxy), Ubuntu 22.04 (Humble) | Start MIPI/USB camera, trigger recording, and record rosbag data |

### Usage

#### Initialization

The Trigger module defines the parameters required for initialization configuration.

The `config_file` is json, and the configuration is as follows:

```bash
{ 
  "domain": Trigger event domain. For example, cleaning robot, humanoid robot, etc. Different Trigger types are differentiated by domain for different types of robots.

  "desc": Trigger module description.

  "duration_ts_back": Duration after Trigger occurrence for recording.

  "duration_ts_front": Duration before Trigger occurrence for recording.
  
  "level": Priority of Trigger event. When multiple different Triggers occur, a central node can be used to filter high-priority or low-priority Trigger events.
  
  "src_module_id": Module ID where Trigger occurred, used for managing different Trigger modules to meet different business requirements.
  
  "status": Trigger status, '0': off, '1': on.
  
  "strategy_version": Version number of the Trigger module strategy.
  
  "topics": List of topics that need to be recorded, including topic names.
  
  "trigger_type": Trigger type ID. Each Trigger module may have more than one triggering condition, for example, detecting 2 pieces of garbage is one type, detecting 3 pieces of garbage is another type.
  
  "unique_id": Unique device identifier.
  
  "version": Trigger module version information.
  
  "extra_kv": Other redundant extension information can be recorded here.
}
  ```

#### Configuration Description

In the `trigger_node` base class, the `Config` structure is defined, in which some configurations are consistent with the Trigger configuration during initialization, while the remaining content needs to be filled according to the actual situation when the Trigger is triggered.

When users develop based on `Trigger_node`, they only need to instantiate a structure variable each time the Trigger occurs, fill in the relevant information when the Trigger occurs in the structure variable, such as "timestamp", "gps_pos", etc., and pass it to the Trigger event recording queue `requests_`.

Based on this, users can develop custom Trigger modules. For more information, please refer to the implementation method of `trigger_node_example` in the code repository.

Code repository:  (https://github.com/D-Robotics/hobot_trigger.git)

The structure information is as follows:

```c++
struct Config {
  std::string domain;       // Trigger event domain
  std::string desc;         // Trigger description information
  long duration_ts_back;    // Duration of recording after Trigger occurs
  long duration_ts_front;   // Duration of recording before Trigger occurs
  GPS_POS gps_pos;          // GPS positioning
  int level;                // Priority level
  std::string rosbag_path;  // Local file path of rosbag after Trigger occurs
  int src_module_id;        // Module that Trigger occurs
  int status;               // Trigger status
  std::string strategy_version; // Strategy version number
  long timestamp;           // Trigger occurrence timestamp
  std::vector<std::string> topics;    // List of topics to be recorded, including topic names and types
  int trigger_type;         // Trigger type
  std::string unique_id;    // Device unique identifier
  std::string version;      // Trigger version information
  std::vector<EXTRA_KV> extra_kv;   // Additional information
};
```

### Preparation

#### RDK

1. RDK has burned the  Ubuntu 20.04/22.04 system image provided by D-Robotics.

2. RDK has successfully installed TogetheROS.Bot.

### Usage

#### RDK

**Publish Images with MIPI Camera**

<Tabs groupId="tros-distro">
<TabItem value="foxy" label="Foxy">

```bash
# Configure the tros.b environment
source /opt/tros/setup.bash
```

</TabItem>

<TabItem value="humble" label="Humble">

```bash
apt install ros-humble-rosbag2-storage-mcap

# Configure the tros.b environment
source /opt/tros/humble/setup.bash
```

</TabItem>

</Tabs>

```shell
# Copy the necessary configuration files for the example to run from the installation path of tros
cp -r /opt/tros/lib/mono2d_trash_detection/config/ .
cp -r /opt/tros/lib/trigger_node_example/config/ .

# Configure MIPI camera
export CAM_TYPE=mipi

# Launch the launch file
ros2 launch trigger_node_example hobot_trigger_example.launch.py
```

**Publishing Images using a USB Camera**

<Tabs groupId="tros-distro">
<TabItem value="foxy" label="Foxy">

```bash
# Configure the tros.b environment
source /opt/tros/setup.bash
```

</TabItem>

<TabItem value="humble" label="Humble">

```bash
# Configure the tros.b environment
source /opt/tros/humble/setup.bash
```

</TabItem>

</Tabs>

```shell
# Copy the necessary configuration files from the installation path of tros.
cp -r /opt/tros/lib/mono2d_trash_detection/config/ .
cp -r /opt/tros/lib/trigger_node_example/config/ .

# Configure the USB camera
export CAM_TYPE=usb

# Launch the launch file
ros2 launch trigger_node_example hobot_trigger_example.launch.py
```


### Result Analysis

**Publishing Images using a MIPI Camera**

After package initialization, the following information will be displayed in the terminal:

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
   [trigger_node_example-1] [WARN] [1683970323.007470033] [hobot_trigger]: Trigger Event Report. Trigger moudle id: 203, type id: 1110
   [trigger_node_example-1]  Report message: {"domain":"","desc":"trigger lane","duration_ts_back":5000,"duration_ts_front":5000,"level":1,"rosbag_path":"trigger/OriginBot002_20230513-173155-931/OriginBot002_20230513-173155-931_0.mcap","src_module_id":203,"timestamp":1683970315931,"topic":["/image_raw/compressed","/ai_msg_mono2d_trash_detection"],"trigger_type":1110,"unique_id":"bot","version":"v1.0.0"}

```

The rosbag generated by Trigger will be recorded in the "trigger" directory of the current running directory. The recorded rosbag data can be played in foxglove. For instructions on how to play rosbag files in foxglove, refer to Section 2.2 Display - foxglove in the manual.The effect is as follows:

![](/../static/img/05_Robot_development/02_quick_demo/image/demo_tool/trigger_example_trash_det.gif)

This Trigger example records data 5 seconds before and after an event. At the same time, you can see that the Trigger event occurred in the middle of the event, and the reason for the Trigger event was recorded: a piece of trash was thrown into the scene, causing the number of trash in the scene to reach three, triggering the Trigger.


### Additional Functionality

#### Sending Tasks to Trigger Module

The Trigger module supports receiving Trigger tasks from other nodes to control Trigger configuration. The tasks can be sent through publishing std_msgs topic messages, where the message data is in json format as a string. Send the task protocol to the Trigger module.

##### Trigger Protocol
```json
{
   "version": "v0.0.1_20230421",       // Trigger module version information.
   "trigger_status": true,             // Trigger status, 'false': off, 'true': on.
   "strategy": [
      {
            "src_module_id": 203,      // ID of the module where the Trigger occurs.
            "trigger_type": 1110,      // Trigger type ID.
            "level": 1,                // Priority of the Trigger event.
            "desc": "",                // Description of the Trigger module.
            "duration_ts_back": 5000,  // Duration of recording after Trigger occurred.
            "duration_ts_front": 3000  // Duration of recording before Trigger occurred.
      }
   ]
}
```

##### Running

Based on starting the Trigger node earlier, in another terminal, publish a topic message named "/hobot_agent" using std_msgs.

<Tabs groupId="tros-distro">
<TabItem value="foxy" label="Foxy">

```bash
# Configure the tros.b environment
source /opt/tros/setup.bash
```

</TabItem>

<TabItem value="humble" label="Humble">

```bash
# Configure the tros.b environment
source /opt/tros/humble/setup.bash
```

</TabItem>

</Tabs>

```shell
# Publish a topic message named "/hobot_agent" using std_msgs
ros2 topic pub /hobot_agent std_msgs/String "data: '{\"version\":\"v0.0.1_20230421\",\"trigger_status\":true,\"strategy\":[{\"src_module_id\":203,\"trigger_type\":1110,\"status\":true,\"level\":1,\"desc\":\"test\",\"duration_ts_back\":5000,\"duration_ts_front\":3000}]}'"
```

##### Log
```shell
[WARN] [1691670626.026737642] [hobot_trigger]: TriggerNode Init Succeed!
[WARN] [1691670626.026859316] [example]: TriggerExampleNode Init.
[INFO] [1691670626.517232775] [TriggerNode]: Updated Trigger Config: {"domain":"robot","desc":"trigger lane","duration_ts_back":5000,"duration_ts_front":3000,"gps_pos":{"latitude":-1,"longitude":-1},"level":1,"rosbag_path":"","src_module_id":203,"strategy_version":"Robot_sweeper_V1.0_20230526","timestamp":0,"topic":["/image_raw/compressed","/ai_msg_mono2d_trash_detection","/hobot_visualization"],"trigger_type":1110,"unique_id":"OriginBot002","version":"v0.0.1_20230421","extra_kv":[]}
```
Analysis: When sending configuration tasks to the Trigger module, the configuration of the Trigger node can be successfully updated (the log of the Trigger node shows INFO to see the log update).