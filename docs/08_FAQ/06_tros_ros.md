---
sidebar_position: 6
---
 
# 8.6 TROS/ROS 开发

本节主要解答与地瓜机器人操作系统（TogetheROS.Bot, tros.b）以及通用ROS/ROS2在RDK平台上开发和使用相关的常见疑问。

### Q1: TROS相关的软件包运行发生错误，推荐的预先排查步骤有哪些？
**A:**
1.  **确保您的tros软件包都是最新的：** 许多问题可能在新版本中已被修复。在确保D-Robotics官方APT源（如 `sunrise.horizon.cc` 或最新的 `archive.d-robotics.cc`）配置正确后，执行以下命令更新所有tros相关的包：
    ```bash
    sudo apt update && sudo apt upgrade
    ```
    在提问时，也请使用 `rdkos_info`、`apt list --installed | grep tros`、`apt show <tros_package_name>` 等命令提供您的系统和软件包版本信息。
2.  **尝试定位问题出现的ROS节点：**
    * 参考对应功能包的launch启动文件，将其中的日志级别（log level）参数修改为 `debug`（例如，在Node的arguments中添加 `['--ros-args', '--log-level', 'DEBUG']`）。
    * 重新运行launch文件，根据详细的debug日志输出来定位具体是哪个节点（node）出现了问题。
    * TROS功能包的launch文件通常位于 `/opt/tros/<tros_distro>/share/<package_name>/launch/` 目录下（例如 `/opt/tros/humble/share/mipi_cam/launch/`）。
    * ROS2的日志文件通常存储在 `~/.ros/log/` 或 `/root/.ros/log/` 目录下。在排查前，可以先执行 `rm -rf ~/.ros/log/*` (或对应root路径) 清空旧日志，然后重新运行功能包以收集最新的、与当前问题相关的日志。
3.  **重新安装相关的tros功能包：**
    * 如果怀疑是某个tros功能包的配置文件被改乱或安装不完整，可以尝试重装该软件包。
    * 以 `hobot_usb_cam` 为例，重装步骤大致如下：
        1.  查找精确的包名：`apt list --installed | grep hobot-usb-cam` (根据实际情况调整搜索词)
        2.  卸载软件包：`sudo apt remove <tros_package_name_found>` (例如 `sudo apt remove tros-hobot-usb-cam`)
        3.  确保APT源配置正确并更新缓存：`sudo apt update`
        4.  重新安装软件包：`sudo apt install <tros_package_name_found>`

### Q2: TROS和标准的ROS2有什么区别？TROS Foxy版本如何升级到TROS Humble版本？
**A:**
* **TROS与ROS2的关系：**
    * TROS (TogetheROS.Bot) 是D-Robotics基于开源的ROS2（Robot Operating System 2）针对其RDK硬件平台进行优化和适配后发布的机器人操作系统。
    * 它通常基于某个ROS2的LTS（长期支持）版本进行构建，例如：
        * 在RDK OS 2.x (基于Ubuntu 20.04) 上，TROS通常基于 **ROS2 Foxy Fitzroy**。
        * 在RDK OS 3.x (基于Ubuntu 22.04) 上，TROS通常基于 **ROS2 Humble Hawksbill**。
    * TROS在标准的ROS2基础上，集成了D-Robotics硬件（如BPU、VPU、JPU、Sensor等）的驱动、硬件加速库、以及针对机器人常用功能的优化方案和示例。
    * TROS与对应版本的标准ROS2在核心API和通信机制上是**完全兼容**的，这意味着为标准ROS2 Foxy/Humble开发的节点和服务通常可以直接或稍作修改后在对应版本的TROS上运行，反之亦然。它们可以相互通信。

* **TROS版本升级 (例如从Foxy到Humble)：**
    * 由于TROS的版本与RDK OS的底层Ubuntu版本紧密绑定（例如Foxy对应Ubuntu 20.04，Humble对应Ubuntu 22.04），**通常无法直接通过`apt upgrade`等命令将TROS从一个大的LTS版本（如Foxy）升级到另一个大的LTS版本（如Humble）**。
    * **正确的升级方式是：** 通过烧录包含了新版本TROS（如Humble）的、对应新Ubuntu版本（如22.04）的RDK OS完整系统镜像，来完成整个系统和TROS的升级。

* **板卡上的ROS2运行环境：**
    * RDK板卡上安装的TROS本身就是一个完整的ROS2运行环境。
    * 您也可以在TROS之外，在板卡上自行安装其他标准ROS2发行版（如Foxy, Humble）或ROS1（如Noetic），它们可以与TROS共存，但如前所述，**一个终端会话中只能source一个ROS环境**。
    * `colcon` 是ROS2常用的构建工具，如果您的系统镜像中未预装，可能需要手动安装：
        ```bash
        sudo apt update
        sudo apt install python3-colcon-common-extensions python3-catkin-pkg-modules python3-rosdep
        # 或者通过pip安装：
        # pip3 install -U colcon-common-extensions empy
        ```
* **注意：** 任何在x86平台直接编译（而非交叉编译）的ROS功能包都不能直接在ARM架构的RDK板卡上运行，反之亦然。需要确保程序是为目标平台的架构编译的。

### Q3: TROS是如何安装在RDK板卡上的？是否需要手动安装？
**A:**
* TROS通常在您烧录官方提供的RDK OS系统镜像时，就已经**内置并预装**在板卡上了。您不需要在烧录完系统后再手动执行完整的TROS安装流程。
* 您可以通过APT包管理器来更新或增量安装TROS的各个功能包。在确保D-Robotics官方APT源配置正确的前提下，执行 `sudo apt update && sudo apt upgrade` 会更新已安装的TROS包到最新版本。
* 旧版本中可能存在的`hhp`工具或手动建立软链接的步骤，在新版的TROS中通常已不再需要。

### Q4: TROS相关功能包的源代码在哪里可以找到？
**A:**
* **TROS手册：** D-Robotics官方的TROS用户手册中，在介绍各个核心功能包或示例时，通常会提供对应源码的GitHub仓库链接。
* **NodeHub：** 如果功能包是作为NodeHub（D-Robotics应用商店或组件平台）的一部分提供，其相关介绍页面通常也会包含源码链接。
* **GitHub D-Robotics组织：** 大部分TROS相关的开源功能包都托管在GitHub上的 **D-Robotics** 组织下 ([https://github.com/D-Robotics](https://github.com/D-Robotics))。您可以在该组织内通过搜索功能包的名称（或部分名称）来查找其源码仓库。
* **README文档：** 通常，每个TROS功能包的源码仓库中都会包含一个详细的`README.md`文件，其中会说明该功能包的编译方法、使用说明、参数配置、依赖项等重要信息。

### Q5: TROS功能包的源码编译有哪些注意事项？
**A:**
1.  **何时需要源码编译：**
    * **体验功能：** 如果您只是想体验TROS的已有功能，通常**不需要**进行源码编译。直接烧录最新的RDK OS系统镜像，参考手册运行预编译好的功能包即可。
    * **二次开发：** 如果您需要在某个官方提供的TROS功能包基础上进行修改和二次开发，那么您需要下载该功能包的源码，并在其基础上进行修改和编译。这种情况下，通常建议直接在**RDK板卡端**进行编译（如果板卡资源允许且安装了必要的编译工具），或者在配置好的**交叉编译Docker环境**中进行。
    * **完整构建TROS：** 如果您需要从零开始构建整个TROS发行版（例如，为了深度定制或移植到新的硬件平台），这是一个非常复杂的过程，通常需要使用官方提供的**交叉编译Docker环境**，在性能强劲的x86 Ubuntu开发机上进行。

2.  **交叉编译Docker环境：**
    * **版本对应：** 确保您拉取的交叉编译Docker镜像版本与您目标TROS版本（Foxy或Humble）以及目标RDK OS版本相对应。
    * **源码分支：** 从GitHub等平台拉取TROS功能包源码时，请确保切换到与您的目标TROS版本对应的正确分支（例如，`foxy`分支、`humble`分支或特定的release tag），**避免直接使用`main`或`develop`等开发分支**，除非您明确知道其兼容性。

3.  **依赖问题：**
    * 在源码编译过程中，可能会遇到缺少依赖库（“缺包少库”）的问题。解决这类编译依赖问题是C/C++开发者应具备的基本技能。
    * 仔细阅读报错信息，确定缺少的库或头文件名称。
    * 尝试使用`apt search <package_name>`查找对应的Debian包，并使用`sudo apt install <package_name-dev>`（通常开发包带有`-dev`后缀）进行安装。
    * 对于ROS自身的依赖，可以使用 `rosdep` 工具来安装：
        ```bash
        sudo apt install python3-rosdep
        sudo rosdep init # 只需要执行一次
        rosdep update
        cd <your_ros_workspace_root>
        rosdep install --from-paths src --ignore-src -r -y
        ```
    * 社区通常无法对 индивидуальные 编译环境的依赖问题提供一对一支持。

### Q6: 在RDK板卡上尝试安装标准ROS2时报错，怎么办？
**A:** 在RDK板卡上（可能已经预装了TROS）自行安装标准的ROS2发行版（如Foxy, Humble）时，如果遇到问题：
1.  **使用推荐的安装工具：**
    * 可以尝试使用社区中广受好评的第三方ROS安装工具，例如“小鱼的一键安装系列”（FishROS）。这些工具通常会处理好软件源配置、依赖安装等繁琐步骤。
        ```bash
        wget http://fishros.com/install -O fishros && bash fishros
        ```
2.  **从源码安装（如果工具安装失败）：**
    * 如果一键安装工具也失败，您可以尝试从“小鱼”的GitHub仓库克隆其安装脚本的源码，并手动执行Python安装脚本。这有时能提供更详细的错误输出或允许您进行一些自定义修改。
        ```bash
        git clone https://github.com/fishros/install
        cd install
        sudo python3 install.py
        ```
3.  **检查网络和软件源：** 确保板卡网络连接正常，并且能够访问ROS官方的软件源（`packages.ros.org`）以及Ubuntu的官方软件源。
4.  **查看错误日志：** 仔细阅读安装过程中出现的任何错误信息，它们通常会指出问题的具体原因（如依赖冲突、下载失败、编译错误等）。

### Q7: TROS中的多媒体方案（如视频流处理、编解码）有哪些推荐的参考资源？
**A:** D-Robotics官方TROS手册中通常会有专门的章节或示例介绍如何在ROS2环境中使用RDK的多媒体能力。
* **社区手册 - 机器人开发 - 应用示例 - 视频应用 (video_boxs)：**
    [https://developer.d-robotics.cc/rdk_doc/Robot_development/apps/video_boxs](https://developer.d-robotics.cc/rdk_doc/Robot_development/apps/video_boxs)
    (请确认此链接为最新且有效。)
    这个链接指向的页面或其子页面通常会包含：
    * 如何使用MIPI摄像头或USB摄像头发布图像话题。
    * 如何使用硬件编解码器（hobot_codec）进行图像/视频的编码（如H.264, H.265, MJPEG）和解码。
    * 如何在ROS2节点间高效传输图像数据（可能涉及零拷贝技术）。
    * 相关的示例代码和launch文件。
    ![TROS多媒体方案示意图](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/08_FAQ/image/tros1.png)
    ![TROS多媒体方案示意图](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/08_FAQ/image/tros2.png)
    ![TROS多媒体方案示意图](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/08_FAQ/image/tros3.png)

### Q8: 启动USB或MIPI摄像头节点后，终端提示“标定数据不存在”（例如 `[usb_camera_calibration.yaml] does not exist!`），这是正常的吗？
**A:** 这个提示本身**通常是正常的，不一定代表摄像头无法工作**。
* **日志级别：** 许多ROS节点的默认日志级别是`INFO`或`WARN`。当摄像头节点启动时，它会尝试加载相机的内参标定文件（通常是一个`.yaml`文件，包含了相机的焦距、畸变系数等参数）。如果找不到这个文件，它会输出一个警告信息，但通常会继续使用一组默认的或无标定的参数来运行。
* **功能验证：**
    * 即使出现这个警告，摄像头本身可能已经在正常发布图像数据了。
    * 您可以在另一个终端中（source好TROS环境后）执行以下命令来验证：
        1.  `ros2 topic list`：查看是否有图像话题（如 `/image_raw`, `/image_color`, `/hbmem_img` 等）正在发布。
        2.  `ros2 topic hz /your_image_topic_name`：查看该图像话题的发布频率。
        3.  `ros2 topic echo /your_image_topic_name`：查看是否有图像消息数据正在输出（数据量会很大，很快会刷屏，主要看是否有数据流）。
        4.  使用`rqt_image_view`（如果在PC端或板卡Desktop环境）订阅该图像话题，看是否能显示画面。
* **何时需要标定文件：** 如果您的应用需要精确的图像测量、三维重建、或者需要对图像进行去畸变处理，那么提供正确的相机标定文件就非常重要。如果只是简单地显示图像或进行一些不依赖精确像素对应的AI推理，缺少标定文件可能影响不大。

### Q9: 使用TROS的WebSocket可视化功能时，浏览器页面上不显示图像或AI结果，可能是什么原因？
**A:** WebSocket可视化不显示内容，可能的原因有很多，需要逐步排查：
1.  **确保相关ROS节点正常运行：**
    * **图像发布节点：** 必须有一个节点（如`mipi_cam`, `usb_cam`, 或图像回放节点）正在发布图像话题。
    * **AI推理节点（如果需要显示AI结果）：** 必须有一个节点正在进行AI推理并发布AI结果话题。
    * **WebSocket节点本身 (`hobot_websocket` 或类似)：** 该节点负责将ROS话题数据转换为WebSocket消息发送给浏览器。
    * 使用 `ros2 node list` 和 `ros2 topic list` 检查这些节点和话题是否都处于活动状态。

2.  **网络连接与IP地址：**
    * 确保您的电脑（运行浏览器）和RDK板卡（运行WebSocket服务）在同一个局域网内。
    * 在浏览器中访问的IP地址必须是RDK板卡的正确IP地址。
    * 检查是否存在IP地址冲突或路由问题。

3.  **代理服务器设置：**
    * 如果您电脑的网络连接配置了代理服务器，请检查代理设置是否可能阻止了对局域网内IP地址（RDK板卡）的直接访问。尝试临时禁用代理或配置代理例外规则。

4.  **WebSocket节点参数与AI消息同步 (针对AI结果不显示)：**
    * `hobot_websocket`节点在启动时，如果其参数`only_show_image`设置为`False`（即期望同时显示图像和AI结果），它可能需要接收到第一个AI结果消息 (`ai_msg`) 后，才能开始同步图像和AI数据并进行渲染。
    * **检查：** 确保您的AI推理节点确实检测到了目标并发布了至少一帧AI结果。如果AI节点一直没有输出，WebSocket端可能因为等待第一帧AI数据而表现为不显示任何叠加结果。

5.  **网络带宽与质量：**
    * 传输未压缩的图像数据（尤其是高分辨率、高帧率）对网络带宽要求较高。如果您的RDK板卡与电脑之间的网络连接质量不佳（例如，Wi-Fi信号弱、网络拥堵、使用手机热点带宽不足），可能导致WebSocket数据传输卡顿、延迟过高或失败。
    * 尝试降低图像分辨率或帧率，或者使用压缩图像格式进行传输。

6.  **RDK板卡端CPU负载过高：**
    * 如果在RDK板卡自身的图形桌面环境（通过VNC或直连显示器）中打开浏览器来查看WebSocket的渲染结果，板卡的CPU可能会同时承担ROS节点运行、WebSocket服务、图形桌面渲染以及浏览器渲染等多重负载，导致性能瓶颈，进程无法正常运行。
    * **建议：** 通常建议在另一台PC上打开浏览器来访问RDK上的WebSocket服务。

7.  **浏览器兼容性或缓存：**
    * 尝试清除浏览器缓存，或使用不同的浏览器（如Chrome, Firefox, Edge的最新版本）进行测试。
    * 查看浏览器开发者工具的控制台（Console）和网络（Network）标签页，看是否有JavaScript错误、WebSocket连接错误或资源加载失败等问题。

8.  **刷新页面：** 有时，简单的刷新网页（Ctrl+R 或 Cmd+R）可能会解决临时的卡顿或连接问题。

### Q10: 使用TROS的智能语音功能时报错，或者想使用自己的USB麦克风，应该如何配置？
**A:**
1.  **检查声卡设备：**
    * 首先，确认您的麦克风设备（无论是板载的还是USB外接的）已被系统正确识别。使用以下命令查看已识别的声卡：
        ```bash
        cat /proc/asound/cards
        ls /dev/snd/
        ```
        `cat /proc/asound/cards` 会列出声卡及其序号（如card 0, card 1）。`ls /dev/snd/` 会显示PCM设备节点（如 `pcmC0D0c` 表示card 0, device 0, capture）。

2.  **配置TROS语音节点的麦克风设备号：**
    * TROS的智能语音相关节点（例如，负责录音或语音识别的节点）通常会有一个参数（例如在launch文件或参数配置文件中）用于指定使用哪个麦克风设备。这个参数的名称可能是 `micphone_name`、`device_name`、`alsa_device` 或类似。
    * 该参数的值通常是ALSA设备名，格式为 `hw:X,Y`，其中 `X` 是声卡序号 (Card Number)，`Y` 是该声卡上的PCM设备序号 (Device Number)。
    * **默认值：** 可能默认为 `"hw:0,0"`，表示使用声卡0上的设备0。
    * **修改：** 如果您的目标麦克风（例如USB麦克风）被识别为声卡1上的设备0的录音端点（capture, 通常设备节点名以 `c` 结尾，如 `pcmC1D0c`），那么您需要将该参数值修改为 `"hw:1,0"`。
    * **示例：** 如果 `cat /proc/asound/cards` 显示您的USB麦克风是 `card 1`，并且 `arecord -l` (列出录音设备) 显示其对应的PCM设备是 `device 0`，则参数应设为 `hw:1,0`。

3.  **检查ALSA音量和静音设置：**
    * 使用 `alsamixer` 命令（在终端中运行），按 `F6` 选择正确的声卡，然后按 `F4` 查看并调整录音（Capture）相关的音量控制（如 'Mic', 'Capture', 'ADC PGA Gain' 等），确保它们没有被静音（Muted，通常显示为MM，按M键切换）并且音量设置在合适范围。

4.  **权限问题：** 确保运行语音节点的进程有权限访问音频设备。

### Q11: 为什么不建议在RDK这种嵌入式终端设备上直接运行Rviz或Gazebo？推荐的做法是什么？
**A:**
* **资源消耗：** Rviz (ROS Visualization tool) 和 Gazebo (robot simulator) 都是功能强大但资源消耗巨大的软件。它们需要较强的CPU处理能力、大量的内存以及（尤其是Gazebo和带有复杂3D渲染的Rviz配置）良好的GPU图形加速能力。
* **嵌入式设备限制：** RDK系列作为嵌入式开发板，其CPU、内存和图形处理能力通常远不及标准的PC或工作站。在RDK板卡上直接运行Rviz或Gazebo：
    * 可能会导致板卡资源耗尽，系统运行极其缓慢甚至卡死。
    * 即使能勉强运行，用户体验也会非常差，可视化效果不佳，仿真速度极慢。
    * 会严重影响板卡上其他机器人核心程序（如感知、决策、控制节点）的实时性和性能。

* **推荐做法：**
    1.  **分布式ROS网络：** 利用ROS/ROS2的分布式特性，将Rviz或Gazebo运行在与RDK板卡**处于同一局域网下的一台性能较好的PC或Ubuntu虚拟机**上。
    2.  **话题订阅/发布：**
        * RDK板卡上的节点负责发布传感器数据（如图像、点云、里程计等）、机器人状态、AI检测结果等话题。
        * PC上运行的Rviz订阅这些来自RDK板卡的话题，进行数据显示和可视化。
        * PC上运行的Gazebo可以仿真机器人模型和环境，并通过ROS话题与RDK板卡上的控制节点进行交互（例如，RDK发送控制指令给Gazebo中的机器人，Gazebo发布仿真传感数据给RDK）。
    3.  **网络配置：** 确保PC和RDK板卡之间的网络配置正确，ROS_DOMAIN_ID（ROS2）或ROS_MASTER_URI/ROS_IP（ROS1）设置正确，使得它们可以相互发现和通信。对于ROS2，通常只要在同一网络且DOMAIN_ID相同即可自动发现。
    4.  **虚拟机配置：** 如果在PC上使用Ubuntu虚拟机运行Rviz/Gazebo，请确保虚拟机的网络模式设置为“桥接模式 (Bridged Adapter)”，这样虚拟机才能获得与RDK板卡在同一网段的独立IP地址。

通过这种方式，可以将计算密集型的可视化和仿真任务放在PC上，而RDK板卡则专注于运行其实时机器人应用程序，从而保证整体系统的性能和稳定性。

### Q12: RDK X3的内核版本较老，是否还能支持使用RealSense D435i深度相机？如何安装？
**A:**
* **支持情况：** 尽管RDK X3的Linux内核版本可能相对较老（例如4.14.x），但D-Robotics官方通常会在其发布的RDK OS系统镜像中，针对常用的外设（如Intel RealSense系列深度相机）**预先打入或集成了所需的内核补丁 (patches) 和驱动模块**。
* **安装方法：** 因此，您通常**不需要**自行从RealSense SDK源码编译内核模块。D-Robotics官方会提供通过APT软件包管理器直接安装RealSense相机驱动和相关工具的方式。
    * **官方教程：** 请参考D-Robotics开发者社区或RDK X3用户手册中关于“传感器Demo”或“外设支持”章节中针对RealSense D435i（或其他型号）的说明。
        例如，此链接曾提供相关指导：[RDK文档 - RealSense图像采集](https://developer.d-robotics.cc/rdk_doc/Robot_development/quick_demo/demo_sensor#realsense%E5%9B%BE%E5%83%8F%E9%87%87%E9%9B%86)。
    * **APT安装：** 安装过程通常是在配置好D-Robotics官方APT源后，执行类似 `sudo apt install librealsense2-dkms librealsense2-utils librealsense2-dev` 的命令。
* **使用：** 安装完成后，您就可以在RDK X3上使用RealSense SDK (librealsense2) 提供的API来获取深度图、彩色图、红外图以及IMU数据（对于D435i）。
* **具体API和数据获取：** 关于如何使用librealsense2的API来获取特定数据流、配置相机参数等具体编程问题，请参考Intel RealSense官方的SDK文档和示例代码。

### Q13: 如何为TROS配置零拷贝（Zero-Copy）数据传输环境？
**A:** 零拷贝是一种优化ROS2节点间大数据（如图像）传输性能的技术，通过共享内存等机制避免不必要的内存拷贝。TROS（基于ROS2）也支持零拷贝。

* **TROS Foxy版本 (基于ROS2 Foxy)：**
    * ROS2 Foxy自身对零拷贝的支持尚不完善，D-Robotics在TROS Foxy中可能通过自定义的共享内存方案（如 `hobot_shm` 包）来增强或实现类似零拷贝的功能。
    * 请参考TROS Foxy版本对应的官方文档或 `hobot_shm` 包的README，了解其特定的配置和使用方法。

* **TROS Humble版本 (基于ROS2 Humble)：**
    * ROS2 Humble对零拷贝（特别是通过借贷消息 Loaned Messages 和DDS的共享内存传输）的支持更为成熟和标准化。
    * **配置方法：** 通常涉及设置一系列环境变量来启用Fast DDS（一种DDS实现）的共享内存传输特性。具体步骤请参考本FAQ文档“8.3 应用和示例”章节中 **Q15: 在TROS Humble版本中如何配置和使用零拷贝（Zero-Copy）数据传输？** 的详细解答。
        简要回顾关键环境变量：
        ```bash
        export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
        export FASTRTPS_DEFAULT_PROFILES_FILE=/opt/tros/humble/lib/hobot_shm/config/shm_fastdds.xml # 路径需确认
        export RMW_FASTRTPS_USE_QOS_FROM_XML=1
        export ROS_DISABLE_LOANED_MESSAGES=0
        ```
* **通用参考：**
    * D-Robotics官方文档中关于TROS通信优化或特定Demo（如图像传输Demo）的章节，通常会包含零拷贝的配置指南。
        例如：[RDK文档 - ROS通信 - 零拷贝配置](https://developer.d-robotics.cc/rdk_doc/Robot_development/quick_demo/demo_communication) (请确认链接的最新有效性)。

### Q14: 除了D-Robotics官方的APT源，是否有其他ROS2的公开软件源可以使用？
**A:** 是的。标准的ROS2发行版（如Foxy, Humble, Iron等）都有其官方的APT软件源，由Open Robotics（现为Intrinsic）维护。
* **ROS2官方源：**
    * 地址通常是 `http://packages.ros.org/ros2/ubuntu`。
    * 在安装标准ROS2或某些依赖于标准ROS2包的第三方软件时，通常需要将此源添加到您的系统中。
    * 添加方法（以Humble为例，适用于Ubuntu Jammy 22.04）：
        1.  **设置区域设置 (locale)：**
            ```bash
            sudo apt update && sudo apt install locales
            sudo locale-gen en_US en_US.UTF-8
            sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
            export LANG=en_US.UTF-8
            ```
        2.  **添加ROS2 GPG密钥并添加源：**
            ```bash
            sudo apt install software-properties-common
            sudo add-apt-repository universe
            sudo apt update && sudo apt install curl -y
            sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
            echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
            ```
        3.  **更新APT缓存：**
            ```bash
            sudo apt update
            ```
    * 之后您就可以通过 `sudo apt install ros-humble-desktop` (安装桌面完整版) 或 `ros-humble-<package_name>` (安装特定包) 来安装标准ROS2 Humble的软件包了。

* **国内镜像源：**
    * 为了加速下载，国内的一些高校或机构（如清华大学TUNA、中科大LUG、上海交大SJTUG等）也提供了ROS2官方软件源的镜像。您可以将上述 `packages.ros.org` 的地址替换为这些镜像站的地址。具体地址请查询对应镜像站的帮助文档。

**注意：** 当系统中同时存在D-Robotics TROS的源和ROS2官方源时，`apt`在安装或更新软件包时会根据包的版本和优先级进行选择。通常，TROS的包会针对RDK硬件有特定优化。
