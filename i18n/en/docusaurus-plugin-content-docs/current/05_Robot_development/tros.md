---
slug: /
sidebar_position: 0
---

# TogetheROS.Bot
TogetheROS.Bot is a robot operating system launched by Horizon Robotics for robot manufacturers and developers. It aims to unleash the intelligent potential of robot scenarios, enabling developers and commercial customers to develop robots efficiently and conveniently, and create competitive intelligent robot products.

Horizon Robotics Developer Kits, referred to as [Horizon RDK](https://developer.horizon.cc/documents_rdk/), are built on Horizon intelligent chips, including **RDK X3 (Sunrise X3)**, **RDK X3 Module (Sunrise X3 Module)**. Currently, TogetheROS.Bot supports running on the Horizon RDK platform. The Horizon RDK platform covers all the functions shown in the diagram below, improving user algorithm development and verification efficiency, and enabling quick migration to the Horizon RDK platform.

![TROS-Diagram](./image/TogetheROS.png)

The code of TogetheROS.Bot is hosted on GitHub under the HorizonRDK organization link: [HorizonRDK](https://github.com/HorizonRDK).

## Communication
Communication is Horizon's optimized and extended communication component on ROS2 Foxy/Humble version.

Main Features include:

The blue sections indicate the modules optimized and added by Horizon, and the main features of TogetheROS.Bot are as follows:

- Provide "hobot_sensor" to adapt to commonly used robot sensors, saving development time and focusing on core competitiveness.
- Provide "hobot_dnn" to simplify on-board algorithm model inference and deployment, unlocking BPU computing power and lowering the threshold for intelligent algorithm usage.
- Provide "hobot_codec" to accelerate video encoding and decoding through a combination of software and hardware, saving CPU resources and improving parallel processing capability.
- Provide "hobot_cv" to enhance the performance of common computer vision operators through a combination of software and hardware, saving CPU resources and improving runtime efficiency.
- Provide "hobot Render" for web-based and HDMI dynamic visualization, real-time rendering of algorithm results (limited to web-based), facilitating display and debugging.
- Add "zero-copy" inter-process zero-copy communication mechanism to reduce data transmission latency and system resource consumption.
- Enhance middleware software debugging and performance tuning tools, improve problem localization efficiency, and facilitate system performance optimization.
- Fully compatible with ROS2 Foxy/Humble version, facilitating the reuse of ROS2 toolkits and speeding up prototype verification.
- Support minimal and modular pruning, facilitating deployment in resource-constrained embedded products as needed.

## Boxs
Boxs is an intelligent algorithm package launched by Horizon Robotics for robot manufacturers and developers based on TogetheROS.Bot. It aims to improve the efficiency of integrating and implementing robot intelligent algorithms based on the Horizon robot operating system.

- Image detection algorithms such as FCOS, YOLO, FasterRCNN, Efficientdet, Mobilenet_ssd;
- Image classification models such as Mobilenet
- Semantic segmentation models such as Unet
- Application algorithm models such as human detection and tracking, gesture recognition, human hand keypoint detection, monocular height network, monocular 3D detection, speech processing,VIO,etc.

## Apps
Apps are algorithm application examples developed based on the Horizon robot operating system's Communication and Boxs. They aim to establish a complete chain of image input, perception, strategy, etc., demonstrate application effects, and accelerate the development efficiency of customer demos.

## Common Term Definitions

| Term                             | Definition                                               |
| ---------------------------------| --------------------------------------------------------|
| zero-copy                        | Inter-process zero-copy communication method             |
| BPU                              | neural network algorithm processing unit             |
| hobot dnn                        | Encapsulation of BPU-based model inference functionality |
| SLAM                              | Simultaneous Localization and Mapping |
| DOA                               | Direction of Arrival                     |
| ASR                               | Automatic Speech Recognition             |
| TogetheROS.Bot                    | Together Robot Operating System for robot    |
| tros.b                            | TogetheROS.Bot abbreviation               |
| RDK                               | Robotics Developer Kits                   |
| BEV                               | Birds Eye View                   |