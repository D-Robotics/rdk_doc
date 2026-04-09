---
sidebar_position: 0
---

# Introduction to TogetheROS.Bot  
TogetheROS.Bot is a robot operating system launched by D-Robotics for robot manufacturers and ecosystem developers, aiming to unlock the intelligent potential of robotic applications and empower ecosystem developers and commercial customers to efficiently and conveniently develop competitive intelligent robot products.

TogetheROS.Bot supports execution on the RDK platform and also provides a simulator version that runs on x86 platforms. The RDK platform encompasses all functionalities shown in the diagram below, while the x86 platform supports experiencing certain features via image replay, thereby improving the efficiency of algorithm development and validation and enabling rapid migration to the RDK platform.

![TROS-Diagram](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/image/TogetheROS.png)

The source code of TogetheROS.Bot is hosted on GitHub under the [D-Robotics organization](https://github.com/D-Robotics).

## Communication Component

Communication is a functional enhancement and extension built upon the core communication components of ROS2 Foxy and Humble versions.

Key features are as follows:

The blue sections indicate optimized or newly added modules. The main features of TogetheROS.Bot include:

- Provides “hobot_sensor” to support commonly used robot sensors, saving development time and allowing focus on core competencies  
- Provides “hobot_dnn” to simplify on-device algorithm model inference and deployment, unleashing BPU computing power and lowering the barrier to using intelligent algorithms  
- Provides “hobot_codec” combining software and hardware to accelerate video encoding/decoding, conserving CPU resources and enhancing parallel processing capabilities  
- Provides “hobot_cv” combining software and hardware to boost performance of common computer vision operators, conserving CPU resources and improving runtime efficiency  
- Provides “hobot Render” with dynamic visualization capabilities for both Web and HDMI outputs, enabling real-time rendering of algorithm results (Web only), facilitating demonstration and debugging  
- Introduces a “zero-copy” inter-process zero-copy communication mechanism to reduce data transmission latency and lower system resource consumption  
- Enriches middleware debugging and performance tuning tools to improve issue localization efficiency and facilitate system performance optimization  
- Maintains full API compatibility with ROS2 Foxy/Humble versions, enabling seamless reuse of the rich ROS tool ecosystem and accelerating prototype validation  
- Supports minimal and modular customization, making it easy to deploy on resource-constrained embedded products according to specific requirements  

## Boxs Algorithm Repository

Boxs is an intelligent algorithm package introduced by D-Robotics for robot manufacturers and ecosystem developers based on TogetheROS.Bot, designed to enhance the efficiency of integrating and deploying intelligent algorithms on robots powered by the D-Robotics RDK robot operating system.

- Image detection algorithms such as FCOS, YOLO, FasterRCNN, EfficientDet, and Mobilenet_SSD  
- Image classification models such as Mobilenet  
- Semantic segmentation models such as Unet  
- Application-oriented algorithms including human detection and tracking, gesture recognition, hand keypoint detection, monocular height estimation network, monocular 3D detection, speech processing, etc.

## Apps Application Examples

Apps are algorithm application examples developed based on the Communication component and Boxs of the D-Robotics RDK robot operating system, aiming to establish an end-to-end pipeline covering image input, perception, decision-making, and other modules, demonstrate application outcomes, and accelerate customer demo development.

## Glossary of Common Terms

| Term                              | Definition                                                  |
| ----------------------------------| -----------------------------------------------------------|
| zero-copy                         | Zero-copy inter-process communication method                |
| hobot dnn                         | BPU-based model inference functionality encapsulation       |
| SLAM                              | Simultaneous Localization and Mapping                        |
| DOA                               | Direction of Arrival (sound source localization)            |
| ASR                               | Automatic Speech Recognition                                |
| TogetheROS.Bot                    | TogetheROS.Bot Robot Operating System                       |
| tros.b                            | Abbreviation for TogetheROS.Bot                             |