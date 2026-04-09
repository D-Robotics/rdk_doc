---
sidebar_position: 8
---

# 5.4.8 Parking Spot Search for Cart

```mdx-code-block
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';
```

## Feature Overview

The Parking Spot Search Control App guides the robot to a parking spot using a parking spot detection algorithm, controlling its movements—including left/right rotation and forward/backward translation. The App consists of MIPI image acquisition, parking spot detection algorithm, parking spot search control strategy, image encoding/decoding, and a web-based visualization frontend. The workflow is shown in the figure below:

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/04_apps/image/parking_search/msg_workflow.png)

The App directly controls a physical cart via control commands issued by the parking spot search control strategy. It can also be tested using a simulated virtual cart within the Gazebo simulation environment on a PC.

Code repository: (https://github.com/D-Robotics/parking_search.git)

## Supported Platforms

| Platform                | Execution Mode                              | Example Functionality                                                                 |
| ----------------------- | ------------------------------------------- | ------------------------------------------------------------------------------------- |
| RDK X3, RDK X3 Module   | Ubuntu 20.04 (Foxy), Ubuntu 22.04 (Humble) | Launch MIPI/USB camera to capture images, perform parking area detection and spot search, and demonstrate results through real-cart motion |

## Design Description

1. **Field-of-view Configuration**:

The field-of-view scene is divided into three regions: "Left," "Center," and "Right." The IoU (Intersection over Union) of parking areas and driving areas within each region is calculated. Based on predefined thresholds, the type of each region is determined to facilitate motion decisions for the cart.

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/04_apps/image/parking_search/view_area.png)

2. **Threshold Settings**:

| Field-of-view Region | Left | Center | Right |
| -------------------- | ---- | ------ | ----- |
| Parking Area IoU     | 0.6  | 0.7    | 0.6   |
| Driving Area IoU     | 0.8  | 0.9    | 0.8   |

3. **Category Settings**:

| Field-of-view Region | Road | Background | Lane Markings | Guide Lines | Parking Lines | Parking Area | Parking Pole | Wheel Stopper |
| -------------------- | ---- | ---------- | ------------- | ----------- | ------------- | ------------ | ------------ | ------------- |
| Parking Area IoU     |      |            |               |             | √             | √            |              |               |
| Driving Area IoU     | √    |            | √             | √           | √             | √            |              |               |

Note: In practical detection scenarios, due to inherent limitations in algorithmic precision (not reaching 100%), driving areas may occasionally be misclassified as parking areas. Therefore, when calculating IoU for driving areas, categories associated with parking areas are also included.

4. **Algorithm Workflow**:

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/05_tros_dev/image/mono2d_trash_detection/workflow.png)

## Prerequisites

### RDK Platform

1. RDK has been flashed with Ubuntu 20.04 or Ubuntu 22.04 system image.
2. TogetheROS.Bot has been successfully installed on RDK.
3. An MIPI or USB camera has been installed on RDK.
4. A Guyue Home Cart is available as the controlled lower-level device.

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/04_apps/image/parking_search/car.jpg)

## Usage Instructions

### RDK Platform

Place the cart on a level surface and adjust the camera angle to be horizontal. After launching the Parking Spot Search App, the cart automatically makes decisions based on the parking area detection algorithm and controls its motion until it locates a parking spot, enters it, and stops.

After the App starts, you can view the images published by the sensor along with corresponding algorithm results rendered in a web browser on your PC (access via `http://IP:8000`, where IP is the RDK's IP address).

To enable rendering on the web interface, open the settings menu in the top-right corner and select the “Full-image Segmentation” option (refer to Section 4.2 Boxs Application Algorithm – Outdoor Parking Area Detection).

Start the Guyue Home Cart and run the lower-level control node on the RDK:

<Tabs groupId="tros-distro">
<TabItem value="foxy" label="Foxy">

```shell
source /opt/tros/setup.bash
source /userdata/originbot/local_setup.bash
ros2 run originbot_base originbot_base
```

</TabItem>

<TabItem value="humble" label="Humble">

```shell
source /opt/tros/humle/setup.bash
source /userdata/originbot/local_setup.bash
ros2 run originbot_base originbot_base
```

</TabItem>

</Tabs>

Upon successful launch, the RDK outputs the following log messages:

```shell
Loading parameters:
 - port name: ttyS3
 - correct factor vx: 1.0000
 - correct factor vth: 1.000000
[INFO] [1662551769.540781132] [originbot_base]: originbot serial port opened
[INFO] [1662551769.741758424] [originbot_base]: IMU calibration ok.
[INFO] [1662551769.742268424] [originbot_base]: OriginBot Start, enjoy it.
```

**Publishing Images Using an MIPI Camera**

<Tabs groupId="tros-distro">
<TabItem value="foxy" label="Foxy">

```shell
# Configure the tros.b environment
source /opt/tros/setup.bash

# Copy required configuration files for the example from the tros.b installation path.
cp -r /opt/tros/${TROS_DISTRO}/lib/parking_perception/config/ .

# Configure MIPI camera
export CAM_TYPE=mipi

# Launch the launch file
ros2 launch parking_search parking_search.launch.py
```

</TabItem>

<TabItem value="humble" label="Humble">

```shell
# Configure the tros.b environment
source /opt/tros/humble/setup.bash

# Copy required configuration files for the example from the tros.b installation path.
cp -r /opt/tros/${TROS_DISTRO}/lib/parking_perception/config/ .

# Configure MIPI camera
export CAM_TYPE=mipi

# Launch the launch file
ros2 launch parking_search parking_search.launch.py
```

</TabItem>

</Tabs>

**Publishing Images Using a USB Camera**

<Tabs groupId="tros-distro">
<TabItem value="foxy" label="Foxy">

```shell
# Configure the tros.b environment
source /opt/tros/setup.bash

# Copy required configuration files for the example from the tros.b installation path.
cp -r /opt/tros/${TROS_DISTRO}/lib/parking_perception/config/ .

# Configure USB camera
export CAM_TYPE=usb

# Launch the launch file
ros2 launch parking_search parking_search.launch.py
```

</TabItem>

<TabItem value="humble" label="Humble">

```shell
# Configure the tros.b environment
source /opt/tros/humble/setup.bash

# Copy required configuration files for the example from the tros.b installation path.
cp -r /opt/tros/${TROS_DISTRO}/lib/parking_perception/config/ .

# Configure USB camera
export CAM_TYPE=usb

# Launch the launch file
ros2 launch parking_search parking_search.launch.py
```

</TabItem>

</Tabs>

## Result Analysis
1. While the robot car is searching and moving forward within the driving area, the RDK runtime terminal outputs the following log messages, indicating that the car is being controlled to move forward at a speed of 0.1 m/s (`do move, direction: 0, step: 0.100000`).

```shell
[parking_search-4] [WARN] [1661942399.306904646] [ParkingSearchEngine]: do move, direction: 0, step: 0.100000
[parking_search-4] [WARN] [1661942399.343490021] [ParkingSearchEngine]: do move, direction: 0, step: 0.100000
[parking_perception-3] [WARN] [1661942399.347396979] [parking_perception]: input fps: 29.97, out fps: 29.67
[parking_search-4] [WARN] [1661942399.410602188] [ParkingSearchEngine]: do move, direction: 0, step: 0.100000
[parking_search-4] [WARN] [1661942399.449585563] [ParkingSearchEngine]: do move, direction: 0, step: 0.100000
```

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/04_apps/image/parking_search/cap1.gif)

2. After the robot car detects a parking spot and begins turning, the RDK runtime terminal outputs the following log messages:

```shell
[parking_search-4] [WARN] [1662539779.408424498] [ParkingSearchEngine]: do rotate, direction: 2, step: 0.100000
[parking_search-4] [WARN] [1662539779.442805415] [ParkingSearchEngine]: do rotate, direction: 2, step: 0.100000
[parking_search-4] [WARN] [1662539779.483669831] [ParkingSearchEngine]: do rotate, direction: 2, step: 0.100000
[parking_search-4] [WARN] [1662539779.522690915] [ParkingSearchEngine]: do rotate, direction: 2, step: 0.100000
[parking_search-4] [WARN] [1662539779.563660873] [ParkingSearchEngine]: do rotate, direction: 2, step: 0.100000
[parking_perception-3] [WARN] [1662539779.595755290] [parking_perception]: input fps: 29.87, out fps: 29.63
[parking_search-4] [WARN] [1662539779.604272498] [ParkingSearchEngine]: do rotate, direction: 2, step: 0.100000
```

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/04_apps/image/parking_search/cap2.gif)

3. After the robot car confirms the parking spot, moves forward, and finally stops, the RDK runtime terminal outputs the following log messages:

```shell
[parking_search-4] [WARN] [1662539796.196264298] [ParkingSearchEngine]: do move, direction: 0, step: 0.100000
[parking_search-4] [WARN] [1662539796.227805589] [ParkingSearchEngine]: Find Target, current count: 398, target count: 400
[parking_search-4] [WARN] [1662539796.267424798] [ParkingSearchEngine]: do move, direction: 0, step: 0.100000
[parking_search-4] [WARN] [1662539796.317332964] [ParkingSearchEngine]: Find Target, current count: 399, target count: 400
[parking_search-4] [WARN] [1662539796.346787673] [ParkingSearchEngine]: do move, direction: 0, step: 0.100000
[parking_search-4] [WARN] [1662539796.386203756] [ParkingSearchEngine]: Find Target, current count: 400, target count: 400
[parking_perception-3] [WARN] [1662539796.428427089] [ParkingSearchEngine]: input fps: 29.90, out fps: 29.74
[parking_search-4] [WARN] [1662539796.465178589] [ParkingSearchEngine]: Parking Area Arrived !!!
[parking_search-4] [WARN] [1662539796.506218048] [ParkingSearchEngine]: Parking Area Arrived !!!
[parking_search-4] [WARN] [1662539796.547036881] [ParkingSearchEngine]: Parking Area Arrived !!!
```

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/04_apps/image/parking_search/cap3.gif)

On the PC side, you can use the `ros2 topic list` command in the terminal to query the topics published by the RDK:

```shell
$ ros2 topic list
/ai_msg_parking_perception
/cmd_vel
/hbmem_img080a1b02022201080403012021072312
/image
/imu
/odom
/originbot_status
/parameter_events
/rosout
/tf
```

Among these, `/image_jpeg` is the JPEG-encoded image published by the RDK after capturing from the MIPI sensor; `/ai_msg_parking_perception` is the algorithm message published by the RDK containing parking spot detection information; and `/cmd_vel` is the motion control command published by the RDK.