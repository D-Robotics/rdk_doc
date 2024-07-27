---
sidebar_position: 8
---

# 5.4.8 Car Parking Space Search

```mdx-code-block
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';
```

## Introduction

The Car Parking Space Search control app guides the robot to move to a parking space using parking detection algorithms, including left and right rotation and forward and backward translation. The app consists of MIPI image acquisition, parking detection algorithm, parking search control strategy, image encoding and decoding, and web display end. The process is shown in the following image:

![](/../static/img/05_Robot_development/04_apps/image/parking_search/msg_workflow.png)

The app directly controls the physical car through control commands published by the parking search control strategy, and can also use the virtual car in the PC side Gazebo simulation environment for testing.

Code repository:  (https://github.com/D-Robotics/parking_search.git)

## Supported Platforms

| Platform         | System | Function                            |
| ---------------- | -------------- | ------------------------------------------- |
| RDK X3, RDK X3 Module | Ubuntu 20.04 (Foxy), Ubuntu 22.04 (Humble)   | Start MIPI/USB camera to capture images, and perform parking area detection and parking space search, finally demonstrating the search effect through the motion of the actual car. |

## Design Description

1. Field of view settings:

Divide the field of view scene into "left", "middle", and "right" regions. Calculate the IOU (Intersection over Union) of the parking area and the driving area in each region, and determine the corresponding region type based on the threshold, thus completing the decision-making of the car's movement.

![](/../static/img/05_Robot_development/04_apps/image/parking_search/view_area.png)

2. Threshold settings:

| Field of view region | Left | Middle | Right |
| ------------------- | ---- | ------ | ----- |
| Parking area IOU    | 0.6  | 0.7    | 0.6   |
| Driving area IOU    | 0.8  | 0.9    | 0.8   |

3. Category settings:

| Field of view region | Road | Background | Lane line | Sign line | Parking line | Parking area | Parking pole | Parking lock |
| ------------------- | ---- | ---------- | --------- | --------- | ------------ | ------------ | -------------| ------------- |
| Parking area IOU    |      |            |           |           | √            | √            |              |              |
| Driving area IOU    | √    |            | √         | √         | √            | √            |              |              |

Note: In actual detection, due to the fact that the detection accuracy of the algorithm itself cannot reach 100%, there may be cases where the driving area is mistakenly detected as the parking area. Therefore, when calculating the driving area, the parking area category is included.

4. Algorithm flow:

![](/../static/img/05_Robot_development/04_apps/image/parking_search/workflow.png)

## Preparation

### RDK

1. The RDK has been flashed with the  Ubuntu 20.04/22.04 system image provided by D-Robotics.

2. The RDK has successfully installed TogetheROS.Bot.

3. The RDK has installed a MIPI or USB camera.

4. The car is used as the control device:

![](/../static/img/05_Robot_development/04_apps/image/parking_search/car.jpg)

## Usage

### RDK

Place the car on a level surface, adjust the camera angle to be horizontal, and run the Parking Search App. The car will automatically make decisions and control its movement based on the results of the parking area detection algorithm until it finds a parking space and stops.

After the app is launched, you can view the images published by the sensors and the corresponding algorithm results on the PC browser (enter http://IP:8000 in the browser, where IP is the IP address of the RDK).

Open the Web interface and click on the settings in the upper right corner of the page. Select the "Full Image Segmentation" option to display the rendered effect.

Start the car and run the control node on the RDK:

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

After the startup is successful, the RDK will output log information:

```shell
Loading parameters:
 - port name: ttyS3
 - correct factor vx: 1.0000
 - correct factor vth: 1.000000
[INFO] [1662551769.540781132] [originbot_base]: originbot serial port opened
[INFO] [1662551769.741758424] [originbot_base]: IMU calibration ok.
[INFO] [1662551769.742268424] [originbot_base]: OriginBot Start, enjoy it.
```

**Publishing Images Using MIPI Camera**

<Tabs groupId="tros-distro">
<TabItem value="foxy" label="Foxy">

```shell
source /opt/tros/setup.bash
cp -r /opt/tros/${TROS_DISTRO}/lib/parking_perception/config/ .
export CAM_TYPE=mipi

ros2 launch parking_search parking_search.launch.py
```

</TabItem>

<TabItem value="humble" label="Humble">

```shell
source /opt/tros/humble/setup.bash
cp -r /opt/tros/${TROS_DISTRO}/lib/parking_perception/config/ .
export CAM_TYPE=mipi

ros2 launch parking_search parking_search.launch.py
```

</TabItem>

</Tabs>

**Publishing images using a USB camera**

<Tabs groupId="tros-distro">
<TabItem value="foxy" label="Foxy">

```shell
source /opt/tros/setup.bash
cp -r /opt/tros/${TROS_DISTRO}/lib/parking_perception/config/ .
export CAM_TYPE=usb

ros2 launch parking_search parking_search.launch.py
```

</TabItem>

<TabItem value="humble" label="Humble">

```shell
source /opt/tros/humble/setup.bash
cp -r /opt/tros/${TROS_DISTRO}/lib/parking_perception/config/ .
export CAM_TYPE=usb

ros2 launch parking_search parking_search.launch.py
```

</TabItem>

</Tabs>

## Result Analysis

1. When the car searches and moves forward in the driving area, the log information is outputted in the RDK terminal. The car is controlled to move forward at a speed of 0.1m/s (do move, direction: 0, step: 0.100000).

```shell
[parking_search-4] [WARN] [1661942399.306904646] [ParkingSearchEngine]: do move, direction: 0, step: 0.100000
[parking_search-4] [WARN] [1661942399.343490021] [ParkingSearchEngine]: do move, direction: 0, step: 0.100000
[parking_perception-3] [WARN] [1661942399.347396979] [parking_perception]: input fps: 29.97, out fps: 29.67
[parking_search-4] [WARN] [1661942399.410602188] [ParkingSearchEngine]: do move, direction: 0, step: 0.100000
[parking_search-4] [WARN] [1661942399.449585563] [ParkingSearchEngine]: do move, direction: 0, step: 0.100000
```

![](/../static/img/05_Robot_development/04_apps/image/parking_search/cap1.gif)

2. When the car finds a parking space and turns, the log information is outputted in the RDK terminal:

```shell
[parking_search-4] [WARN] [1662539779.408424498] [ParkingSearchEngine]: do rotate, direction: 2, step: 0.100000
[parking_search-4] [WARN] [1662539779.442805415] [ParkingSearchEngine]: do rotate, direction: 2, step: 0.100000
[parking_search-4] [WARN] [1662539779.483669831] [ParkingSearchEngine]: do rotate, direction: 2, step: 0.100000
[parking_search-4] [WARN] [1662539779.522690915] [ParkingSearchEngine]: do rotate, direction: 2, step: 0.100000
[parking_search-4] [WARN] [1662539779.563660873] [ParkingSearchEngine]: do rotate, direction: 2, step: 0.100000
[parking_perception-3] [WARN] [1662539779.595755290] [parking_perception]: input fps: 29.87, out fps: 29.63
[parking_search-4] [WARN] [1662539779.604272498] [ParkingSearchEngine]: do rotate, direction: 2, step: 0.100000![](/../static/img/05_Robot_development/04_apps/image/parking_search/cap2.gif)

3. When the car determines the parking space and moves forward to stop, the RDK terminal outputs log information:

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

![](/../static/img/05_Robot_development/04_apps/image/parking_search/cap3.gif)

On the PC terminal, you can use the `ros2 topic list` command to query the topic information of the RDK:

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

Among them, `/image_jpeg` is the JPEG-encoded image published by the RDK after capturing images from the MIPI sensor, `/ai_msg_parking_perception` is the algorithm message published by the RDK containing parking detection information, and `/cmd_vel` is the motion control command published by the RDK.