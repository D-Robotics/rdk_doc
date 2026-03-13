# 6.1 Solution List

## Quadruped Robot Locomotion Control Solution

- [Community Link](https://developer.d-robotics.cc/forumDetail/286589986407833603)
- Based on the RDK S100 development kit, multiple bio-inspired gaits from the CoRL 2022 award-winning paper "Walk These Ways" have been successfully reproduced on the Unitree Go2 quadruped robot, including galloping, bounding, trotting, and pacing. The real-world performance is stable, demonstrating excellent disturbance rejection and terrain adaptability.
- The RDK S100 enables users to leverage existing simulation-based training methods, allowing trained models to be directly deployed onto physical robots with high efficiency and outstanding accuracy, significantly accelerating the practical application of embodied intelligence algorithms.

## Small Humanoid + Voice Control Solution

- [Community Link](https://developer.d-robotics.cc/forumDetail/289424808704600178)
- Using the RDK S100 developer kit, complex motion control algorithms have been efficiently migrated and hardware-adapted onto the Gaoqing Mechatronics bipedal robot "Xiao Pi." Maintaining 99.9% quantized control accuracy with only 0.6ms inference latency, RDK S100 delivers unprecedented real-time responsiveness for educational and research robots. From stable standing and in-place turning to smooth walking, RDK S100 provides robust computational power for Xiao Pi’s voice-controlled operations.
- Baidu PaddlePaddle’s MDTC voice wake-up model achieves ultra-fast performance at 830 FPS with >95% accuracy, while HuggingFace’s Wav2Vec2 large speech model runs at 30 FPS with a low word error rate of 2.1%, combined with ultra-low resource consumption (only 26MB ION memory usage), enabling highly efficient and accurate human-robot interaction.
- RDK S100’s powerful heterogeneous architecture helps Xiao Pi build a more efficient and intelligent motion control "brain." Through real-time dynamic stability optimization and multi-threaded resource scheduling, RDK S100 efficiently supports cutting-edge algorithms such as pose estimation and reinforcement learning-based control policies, enabling Xiao Pi not only to precisely execute basic actions like moving forward, turning, and rotating in place, but also to perform complex tasks like dynamic obstacle avoidance and achieve more natural human-robot interaction.

## Low-Speed Autonomous Vehicle Precise Environmental Perception Solution

- [Community Link](https://developer.d-robotics.cc/forumDetail/289425139417082881)
- In BEV (Bird's Eye View) multi-task real-time perception algorithms, the RDK S100 achieves an inference frame rate of 60 FPS, total inference latency of 17 ms, and only ~0.8% NDS quantization loss, enabling simultaneous output of 3D obstacle detection, object classification, and BEV semantic segmentation—providing structured perception support for path planning and behavioral decision-making in dynamic, complex environments.
- For dense LiDAR point cloud processing, the CenterPoint 3D perception algorithm running on RDK S100 achieves 90 FPS inference frame rate, 60 ms total inference latency, and approximately 0.2% NDS quantization loss, enabling efficient spatial target parsing and rapid point cloud processing with feature extraction.

## LeRobot Fully Open-Source Dual-Arm Solution

- [Community Link](https://developer.d-robotics.cc/forumDetail/289424806557116771)
- Based on the RDK S100 developer kit, the end-to-end embodied intelligence algorithm ACT Policy from HuggingFace’s fully open-source LeRobot dual-arm platform has been successfully reproduced, enabling smooth, stutter-free autonomous shirt-folding with dual arms. Including 3D printing, various components, and the RDK S100 robotics developer kit, the total cost of this complete solution is under RMB 5,000.
- Serving as the main computing platform, RDK S100 delivers powerful performance for ACT Policy inference: its BPU provides 128 TOPS of computational power, generating 50 sets of robotic arm motions in just 46 milliseconds. During arm motion execution, the BPU enters a resting state with 0% computational utilization, offering exceptional energy efficiency for real-time manipulation scenarios such as dynamic grasping.

## Bipedal Robot Complex Gait Control Solution

- [Community Link](https://developer.d-robotics.cc/forumDetail/289424806557116663)
- Using the RDK S100 developer kit, complex motion control algorithms have been efficiently migrated and hardware-adapted onto the Agility Robotics TRON1 bipedal robot. Maintaining 99.99% model accuracy, the system achieves inference speeds of tens of thousands of frames per second—delivering a 10x performance improvement over CPU—and providing strong real-time responsiveness for complex motion scenarios.
- The entire deployment process on the TRON1 bipedal robot takes only one hour. Without modifying the original algorithm framework, native migration from CPU to BPU is achieved through standardized model import interfaces and single-parameter quantization configuration, enabling seamless transition from simulation to real-world deployment and significantly lowering the barrier and cost of cross-platform development.