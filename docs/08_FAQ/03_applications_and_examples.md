---
sidebar_position: 3
---

# 8.3 应用开发、编译与示例

本节主要解答与在RDK平台上安装/使用第三方库、编译应用程序、运行官方示例以及相关问题。

如需交叉编译部署，请参考[交叉编译环境部署](https://developer.d-robotics.cc/forumDetail/112555549341653662)

### Q1: 第三方库在RDK上的安装/交叉编译和使用方法是怎样的？
**A:**
* **板端直接安装：** 如果第三方库提供了适用于ARM架构的预编译包（例如 `.deb` 文件），或者可以通过包管理器（如 `apt`）直接安装，那么可以在RDK板卡上直接进行安装。对于Python库，如果Pypi上有对应的arm64 wheels包，也可以直接 `pip install`。
* **交叉编译：** 如果第三方库需要从源码编译，推荐在PC开发主机上进行交叉编译，然后将编译产物部署到RDK板卡上。
    * **环境部署：** 详细的交叉编译环境搭建步骤，请参考地平线开发者社区的教程：[交叉编译环境部署](https://developer.d-robotics.cc/forumDetail/112555549341653662)
    * **编译步骤：** 通常需要配置CMake Toolchain文件，指定交叉编译器、目标系统Sysroot等。

### Q2: 在编译大型程序（如C++项目、ROS功能包）的过程中，如果系统提示编译进程被“kill”或出现内存不足相关的错误日志，应该如何解决？
**A:** 编译大型项目时，如果物理内存不足，Linux系统的OOM (Out Of Memory) killer机制可能会杀死消耗内存最多的进程（通常是编译器进程如`cc1plus`、`ld`等），导致编译失败。
**解决方法：** 增加系统的交换空间 (Swap)。Swap是硬盘上的一块区域，当物理内存不足时，系统可以将部分不常用的内存数据暂时存放到Swap中，从而释放物理内存供当前任务使用。虽然Swap比物理内存慢，但可以有效防止因瞬时内存不足导致的编译失败。

**增加Swap空间的步骤示例 (创建一个1GB的Swap文件)：**
```bash
# 1. （可选）创建一个目录用于存放Swap文件，或者直接在根目录创建
sudo mkdir -p /swapfile_custom_dir 
cd /swapfile_custom_dir

# 2. 使用dd命令创建一个指定大小的空文件 (bs=1M表示块大小为1MB, count=1024表示1024个块，即1GB)
sudo dd if=/dev/zero of=swap bs=1M count=1024 

# 3. 设置正确的文件权限 (只有root用户可读写)
sudo chmod 0600 swap 

# 4. 将该文件格式化为Swap分区
sudo mkswap -f swap 

# 5. 启用Swap分区
sudo swapon swap 

# 6. 验证Swap空间是否已启用 (会显示Swap总量和已用量)
free -h
swapon --show
```

**使其开机自动挂载 (可选但推荐)：**
编辑 `/etc/fstab` 文件，在末尾添加一行（假设您的swap文件路径是`/swapfile_custom_dir/swap`）：
```bash
/swapfile_custom_dir/swap none swap sw 0 0
```

**参考教程：** [Swap使用教程](https://developer.d-robotics.cc/forumDetail/98129467158916281)

### Q3: 如何运行GC4633 MIPI摄像头的示例程序？
**A:** 地平线官方通常会提供基于常见MIPI摄像头（如F37、GC4663）的AI算法示例（例如FCOS目标检测）。这些示例一般会自动检测连接的摄像头型号并进行算法推理。

**运行步骤示例 (以 `/app/ai_inference/03_mipi_camera_sample` 为例)：**
1.  确保GC4663（或其他兼容的MIPI摄像头）已正确连接到RDK板卡的MIPI CSI接口，并且板卡已上电。
2.  通过SSH或串口登录到板卡系统。
3.  进入示例程序所在的目录：
    ```bash
    cd /app/ai_inference/03_mipi_camera_sample 
    # 注意：具体路径可能因RDK系统版本和镜像内容而略有不同。
    ```
4.  使用`sudo`权限运行Python示例脚本：
    ```bash
    sudo python3 mipi_camera.py
    ```
5.  如果示例设计为通过HDMI输出，请确保RDK板卡的HDMI接口已连接到显示器。运行后，您应该能在显示器上看到摄像头捕捉的实时画面以及AI算法处理后的结果（例如检测框、分类标签等）。

### Q4: 使用`rqt_image_view`查看RDK通过ROS发布的RGB888 RAW图像时，感觉非常卡顿，甚至无法接收图像，是什么原因？
**A:** 这个问题通常与ROS2中间件DDS的配置以及网络传输效率有关，特别是当传输未压缩的大尺寸原始图像数据时。
* **原因分析：**
    * 默认的FastDDS在UDP协议层可能没有实现有效的MTU（最大传输单元）分片。当发布的图像数据包大小超过网络路径上的MTU时，IP层会进行分片。
    * 大量的IP分片对许多常见的路由器、交换机或网卡来说处理负担较重，可能导致无法有效缓冲所有分片。
    * 在UDP传输中，如果任何一个IP分片丢失，整个UDP包（即整个图像帧）通常就会被丢弃，或者需要等待重传（如果上层有相关机制，但ROS图像话题通常不保证可靠传输），这会导致严重的卡顿或图像丢失。这种情况有时被称为“IP fragmentation attack”的类似表现，即大量分片导致网络拥堵和丢包。
* **解决方法：**
    1.  **更换DDS实现：** 尝试将ROS2的RMW (ROS Middleware) 实现从默认的`rmw_fastrtps_cpp`切换到`rmw_cyclonedds_cpp`。CycloneDDS在处理大数据包和网络分片方面有时表现更优。
        在终端执行以下命令来切换DDS（仅对当前终端会话有效，或可加入到`.bashrc`）：
        ```bash
        export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
        ```
        然后重新启动您的ROS节点。
    2.  **降低传输数据量：**
        * **发送压缩格式图像：** 考虑在RDK板卡端将原始图像（如RGB888）压缩为JPEG或PNG等格式后再通过ROS话题发布。这会显著减小每帧图像的数据量。您可以在PC端订阅压缩图像话题，并由`rqt_image_view`（或自定义节点）进行解压显示。
        * **降低分辨率或帧率：** 如果应用允许，适当降低发布图像的分辨率或帧率也能有效减少网络负担。

### Q5: 地平线提供的Linux镜像（特指经过最小裁剪的系统，非完整Ubuntu Desktop/Server）是否支持在板卡端直接进行编译操作？
**A:** 地平线为RDK提供的部分Linux镜像，特别是那些为嵌入式部署而经过最小化裁剪的rootfs（根文件系统），可能**不包含**完整的编译工具链（如GCC, G++, make, CMake等）和开发所需的头文件、库文件。
* **结论：** 这类最小化Linux镜像通常**无法支持或不适合**在板卡端直接进行复杂的源码编译工作。
* **推荐做法：** 对于需要在RDK上运行的应用程序，推荐采用**交叉编译**的方式。即在PC开发主机（如Ubuntu PC）上配置好针对RDK目标平台的交叉编译环境，在PC上完成编译后，再将生成的可执行文件和相关依赖部署到RDK板卡上运行。

### Q6: 在地平线提供的最小化Linux镜像上如何运行官方手册中提供的示例（这些示例通常以Ubuntu系统环境为例）？
**A:** 官方手册中的示例（尤其是TROS/ROS相关的示例）通常是在功能更完整的Ubuntu系统环境下演示的。要在最小化的Linux镜像（可能没有预装Python解释器或完整的ROS环境）上运行这些示例（特别是C++编写的ROS节点），需要做一些调整：

* **Ubuntu系统与Linux镜像启动示例的差异：**
    * **环境配置：**
        * **Ubuntu系统：** 通常使用 `source /opt/tros/setup.bash` (或对应ROS版本的setup.bash) 来配置TROS/ROS环境，这个脚本会设置大量的环境变量（如`PATH`, `LD_LIBRARY_PATH`, `AMENT_PREFIX_PATH`等）。
        * **Linux镜像：** 可能需要手动设置关键的环境变量，特别是 `LD_LIBRARY_PATH` 以确保程序能找到所需的共享库。日志路径 `ROS_LOG_DIR` 也可能需要手动指定到一个可写的位置。
    * **配置文件拷贝：** 无论哪种系统，运行示例前通常都需要将示例依赖的配置文件（如模型配置、参数文件等）从TROS/ROS安装路径下拷贝到当前工作目录或指定路径。
    * **启动方式：**
        * **Ubuntu系统：** 常使用 `ros2 run <package_name> <executable_name>` 或 `ros2 launch <package_name> <launch_file_name>` 来启动节点或启动文件。
        * **Linux镜像：** 由于可能没有完整的`ros2`命令行工具或launch系统，通常需要直接运行编译好的C++可执行程序，并通过命令行参数的方式传递原本在launch文件中设置的参数。

* **将launch脚本内容转换为Linux镜像上的直接执行命令（以一个C++的`dnn_node_example`为例）：**

    1.  **分析Ubuntu上的启动命令：**
        ```bash
        # Ubuntu: 配置tros.b环境
        source /opt/tros/setup.bash

        # Ubuntu: 从tros.b的安装路径中拷贝出运行示例需要的配置文件。config中为example使用的模型，回灌使用的本地图片
        cp -r /opt/tros/${TROS_DISTRO}/lib/dnn_node_example/config/ .

        # Ubuntu: 使用本地jpg格式图片进行回灌预测，并存储渲染后的图片
        ros2 launch dnn_node_example dnn_node_example_feedback.launch.py
        ```

    2.  **找到launch脚本并分析其内容：**
        * **查找launch脚本路径：**
            ```bash
            # find /opt/tros/ -name dnn_node_example_feedback.launch.py
            /opt/tros/share/dnn_node_example/launch/dnn_node_example_feedback.launch.py
            ```
        * **查看launch脚本内容 (Python launch file)：**
            ```python
            # dnn_node_example_feedback.launch.py (主要内容节选)
            def generate_launch_description():
                config_file_launch_arg = DeclareLaunchArgument(
                    "dnn_example_config_file", default_value=TextSubstitution(text="config/fcosworkconfig.json")
                )

                img_file_launch_arg = DeclareLaunchArgument(
                    "dnn_example_image", default_value=TextSubstitution(text="config/test.jpg")
                )

                # 拷贝config中文件
                dnn_node_example_path = os.path.join(
                    get_package_prefix('dnn_node_example'),
                    "lib/dnn_node_example")
                # print("dnn_node_example_path is ", dnn_node_example_path) # 这行通常在launch中不直接打印
                # cp_cmd = "cp -r " + dnn_node_example_path + "/config ."
                # print("cp_cmd is ", cp_cmd) # 这行通常在launch中不直接打印
                # os.system(cp_cmd) # launch文件通常不直接执行shell命令拷贝，而是依赖ament_cmake的install规则

                return LaunchDescription([
                    config_file_launch_arg,
                    img_file_launch_arg,
                    Node(
                        package='dnn_node_example',
                        executable='example', # 可执行文件名
                        output='screen',
                        parameters=[         # 传递给可执行程序的参数
                            {"feed_type": 0},
                            {"config_file": LaunchConfiguration('dnn_example_config_file')}, 
                            {"image": LaunchConfiguration('dnn_example_image')},            
                            {"image_type": 0},
                            {"dump_render_img": 1}
                        ],
                        arguments=['--ros-args', '--log-level', 'info']
                    )
                ])
            ```
        从launch脚本中，我们可以知道它启动了`dnn_node_example`包中的名为`example`的可执行文件，并传递了一系列参数。

    3.  **找到可执行程序路径：**
        在TROS安装路径下查找该可执行文件：
        ```bash
        # find /opt/tros/ -name example -executable -type f 
        # (更精确的查找方式可能是基于package名)
        # 通常位于 /opt/tros/${TROS_DISTRO}/lib/<package_name>/<executable_name>
        # 示例路径: /opt/tros/humble/lib/dnn_node_example/example 
        ```
        (假设 `TROS_DISTRO` 环境变量在Linux镜像上未设置，您需要知道实际的发行版名称，如 `humble` 或 `foxy`)

    4.  **在Linux镜像上构造启动命令：**
        * **配置环境：**
            ```bash
            # 假设TROS库文件位于/opt/tros/humble/lib (具体路径需确认)
            export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/opt/tros/humble/lib/ 
            # 指定一个可写的日志目录
            export ROS_LOG_DIR=/userdata/  # 或者 /tmp/roslogs/
            mkdir -p $ROS_LOG_DIR
            ```
        * **拷贝配置文件：** (与Ubuntu上类似)
            ```bash
            # 例如，对于humble版本:
            cp -r /opt/tros/humble/lib/dnn_node_example/config/ .
            ```
        * **直接运行可执行程序并传递参数：**
            ROS2节点参数通常通过 `--ros-args -p <param_name>:=<param_value>` 的形式传递。
            ```bash
            /opt/tros/humble/lib/dnn_node_example/example \
                --ros-args \
                -p feed_type:=0 \
                -p config_file:="config/fcosworkconfig.json" \
                -p image:="config/test.jpg" \
                -p image_type:=0 \
                -p dump_render_img:=1 \
                --log-level info
            ```

    * **完整的Linux镜像上运行示例脚本可能如下：**
        ```bash
        #!/bin/bash

        # 1. 配置环境
        # 根据实际TROS版本和安装路径调整
        TROS_DISTRO_NAME="humble" # 或者 "foxy" 等
        TROS_INSTALL_LIB_DIR="/opt/tros/${TROS_DISTRO_NAME}/lib"
        export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:${TROS_INSTALL_LIB_DIR}
        # 如果有其他依赖的库路径，也需要加入
        # export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:${TROS_INSTALL_LIB_DIR}/aarch64-linux-gnu # 示例

        export ROS_LOG_DIR=/userdata/ros_logs_$(date +%s)
        mkdir -p $ROS_LOG_DIR
        echo "ROS logs will be stored in $ROS_LOG_DIR"

        # 2. 准备工作目录和配置文件
        WORK_DIR="/tmp/dnn_example_run_$(date +%s)" # 使用时间戳避免冲突
        mkdir -p $WORK_DIR
        cd $WORK_DIR
        echo "Working directory: $(pwd)"

        CONFIG_SOURCE_DIR="${TROS_INSTALL_LIB_DIR}/dnn_node_example/config"
        if [ -d "$CONFIG_SOURCE_DIR" ]; then
            echo "Copying config files from $CONFIG_SOURCE_DIR to $(pwd)/config"
            mkdir -p config
            cp -r $CONFIG_SOURCE_DIR/* ./config/
        else
            echo "Error: Config source directory $CONFIG_SOURCE_DIR not found."
            exit 1
        fi

        # 3. 运行可执行程序
        EXECUTABLE_PATH="${TROS_INSTALL_LIB_DIR}/dnn_node_example/example"
        if [ ! -f "$EXECUTABLE_PATH" ]; then
            echo "Error: Executable $EXECUTABLE_PATH not found."
            exit 1
        fi

        echo "Starting DNN example..."
        $EXECUTABLE_PATH \
            --ros-args \
            -p feed_type:=0 \
            -p config_file:="config/fcosworkconfig.json" \
            -p image:="config/test.jpg" \
            -p image_type:=0 \
            -p dump_render_img:=1 \
            --log-level info

        echo "DNN example finished. Check $WORK_DIR for output and $ROS_LOG_DIR for logs."
        ```

    :::tip
    * 除了使用环境变量`ROS_LOG_DIR`设置log路径外，还可以通过启动参数`--ros-args --disable-external-lib-logs`禁止node输出log到文件，使日志直接打印到控制台。
        例如：
        ```bash
        $EXECUTABLE_PATH --ros-args --disable-external-lib-logs \
            -p feed_type:=0 -p image_type:=0 -p dump_render_img:=1 
        ```
    * 详细的ROS2日志说明可以参考：[ROS2官方文档 - About Logging](https://docs.ros.org/en/humble/Concepts/Intermediate/About-Logging.html)
    :::

### Q7: 如何快速查找ROS/TROS中launch启动脚本文件的具体路径？
**A:** 当您知道一个launch脚本的文件名（例如 `dnn_node_example.launch.py`），但需要修改它或查看其内容时，可以在RDK板卡的TROS安装路径（通常是 `/opt/tros/`）下使用 `find` 命令来查找。

**查找示例：**
```bash
# 查找名为 dnn_node_example.launch.py 的文件
find /opt/tros/ -name dnn_node_example.launch.py
```

### Q8: 交叉编译TogetheROS.Bot (tros.b) 完整源码时速度很慢，有什么方法可以加速吗？
**A:** 完整编译tros.b的所有package确实需要较长时间（例如，在8核CPU、32GB内存的PC上可能需要20分钟左右）。以下是两种加速编译的方法：

1.  **使用最小化编译脚本：**
    * 地平线提供的tros.b编译脚本中，通常除了`all_build.sh`（完整编译）之外，还会提供一个`minimal_build.sh`（最小化编译）的选项。
    * 最小化编译通常会跳过编译算法示例（examples）和测试用例（tests）等非核心功能包，从而显著减少编译时间。
    * **使用方法：** 在您进行交叉编译配置的步骤中，将原本调用`./robot_dev_config/all_build.sh`的命令替换为调用`./robot_dev_config/minimal_build.sh`。

2.  **手动忽略不需要编译的package：**
    * Colcon（ROS2的构建工具）支持通过在特定package的源码目录下放置一个名为`COLCON_IGNORE`的空文件来忽略该package的编译。
    * **步骤：**
        1.  首先，确定您不需要哪些package。这些package的源码通常是在编译前通过`.repos`文件（例如 `robot_dev_config/ros2_release.repos`）下载到`src/`目录下的。
        2.  查看`.repos`文件，找到您想忽略的package的源码路径。例如，如果`.repos`文件中有如下配置：
            ```yaml
            ament/google_benchmark_vendor:
              type: git
              url: [https://github.com/ament/google_benchmark_vendor.git](https://github.com/ament/google_benchmark_vendor.git)
              version: 0.0.7
            ```
            这说明`google_benchmark_vendor`这个package的源码会被下载到 `src/ament/google_benchmark_vendor/` 路径下。
        3.  在该package的源码根目录下创建一个空的`COLCON_IGNORE`文件：
            ```bash
            touch src/ament/google_benchmark_vendor/COLCON_IGNORE
            ```
        4.  这样，在下次执行`colcon build`时，这个package就会被跳过。您可以对多个不需要的package执行此操作。

### Q9: RDK板卡上安装了官方的tros.b之后，是否还支持安装和使用其他版本的ROS（如ROS1或不同发行版的ROS2）？
**A:** **支持。**
* 在RDK板卡上安装了地平线的tros.b（例如基于ROS2 Humble）之后，您仍然可以尝试安装其他版本的ROS，包括ROS1（如Noetic, Melodic）或其他ROS2发行版（如Foxy, Galactic等，如果它们支持ARM64架构且您能找到或自行编译安装包）。
* 不同的ROS版本可以共存于系统中，它们通常安装在不同的路径下（例如ROS1在`/opt/ros/noetic/`，ROS2 Humble在`/opt/ros/humble/`，tros.b可能在`/opt/tros/humble/`）。

    :::caution **重要注意事项**
    **一个终端会话中只能source一个ROS版本的环境！**
    * 如果您在一个终端中执行了 `source /opt/tros/humble/setup.bash` 来激活tros.b (Humble) 的环境，那么在该终端中就**不能**再source其他ROS版本（如 `source /opt/ros/foxy/setup.bash` 或 `source /opt/ros/noetic/setup.bash`）的环境，反之亦然。
    * 同时source多个ROS版本的环境会导致环境变量冲突（如`PATH`, `LD_LIBRARY_PATH`, `PYTHONPATH`, `AMENT_PREFIX_PATH`, `ROS_PACKAGE_PATH`等），使得ROS命令和程序行为错乱。
    * 如果您需要在不同的ROS版本间切换，请为每个版本打开独立的终端会话，并在各自的会话中source对应的`setup.bash`文件。
    :::

* **tros.b与ROS2 Foxy/Humble的兼容性：**
    * 地平线的tros.b通常是基于某个ROS2 LTS版本（如Foxy或Humble）进行构建和优化的，并与之保持API接口兼容。这意味着，如果您的tros.b是基于Humble的，那么您通常可以直接使用为标准ROS2 Humble开发的工具和库，而无需再单独安装一遍ROS2 Humble（除非您需要标准ROS2 Desktop完整版中的某些特定工具，而tros.b中未包含）。

### Q10: 使用`colcon build`命令编译ROS2 package时报错 `AttributeError: module 'pyparsing' has no attribute 'operatorPrecedence'`，如何解决？
**A:** 这个错误 `AttributeError: module 'pyparsing' has no attribute 'operatorPrecedence'` 通常是由于系统中安装的`python3-catkin-pkg`（一个用于解析ROS package.xml文件的Python库）版本过低，而它依赖的`pyparsing`库版本与其不兼容，或者`python3-catkin-pkg`自身的功能不完备导致的。

**解决方法：** 尝试升级`python3-catkin-pkg`到ROS官方源提供的较新版本。

**步骤如下：**
1.  **添加ROS官方的APT软件源**（如果尚未添加）：
    这一步是为了确保能从ROS官方获取到最新兼容的`python3-catkin-pkg`版本。
    ```bash
    sudo apt update && sudo apt install curl gnupg2 lsb-release
    sudo curl -sSL [https://raw.githubusercontent.com/ros/rosdistro/master/ros.key](https://raw.githubusercontent.com/ros/rosdistro/master/ros.key) -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] [http://packages.ros.org/ros2/ubuntu](http://packages.ros.org/ros2/ubuntu) $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    ```
    注意：`$(source /etc/os-release && echo $UBUNTU_CODENAME)` 会自动获取您当前Ubuntu系统的代号（如 `focal` for 20.04, `jammy` for 22.04）。

2.  **移除旧版本的 `python3-catkin-pkg`**（可选，但有时有助于干净安装）：
    ```bash
    sudo apt remove python3-catkin-pkg
    ```

3.  **更新APT缓存并安装新版本的 `python3-catkin-pkg`：**
    ```bash
    sudo apt update
    sudo apt install python3-catkin-pkg
    ```
4.  安装/升级完成后，再次尝试执行 `colcon build` 命令。

如果问题依旧，可能还需要检查 `pyparsing` 库本身的版本，并确保它与新安装的 `python3-catkin-pkg` 版本兼容。有时可能需要通过 `pip` 来管理 `pyparsing` 的版本。

### Q11: 如何查看tros.b的版本信息？
**A:** tros.b安装完成后，登录到RDK系统，并使用以下命令查看tros.b元包（meta-package）的版本信息，这通常也代表了整个tros.b发行版的基础版本：
```bash
apt show tros
```

**示例输出 (RDK OS 2.x 版本系统，tros.b 2.0.0):**

```bash
Package: tros
Version: 2.0.0-20230523223852
Maintainer: kairui.wang <kairui.wang@horizon.ai>
Installed-Size: unknown
Depends: hobot-models-basic, tros-ros-base, tros-ai-msgs, ... (大量依赖包)
Download-Size: 980 B
APT-Manual-Installed: yes
APT-Sources: [http://archive.d-robotics.cc/ubuntu-rdk](http://archive.d-robotics.cc/ubuntu-rdk) focal/main arm64 Packages
Description: TogetheROS Bot
```

**示例输出 (RDK OS 1.x 版本系统，tros.b 1.1.6):**

```bash
Package: tros
Version: 1.1.6
Section: utils
Maintainer: kairui.wang <kairui.wang@horizon.ai>
Installed-Size: 1,536 MB
Pre-Depends: hhp-verify
Depends: symlinks, locales, hhp-verify, hobot-models-basic, hobot-arm64-libs (>= 1.1.6)
Apt-Sources: [http://archive.d-robotics.cc/ubuntu-ports](http://archive.d-robotics.cc/ubuntu-ports) focal/main arm64 Packages
Date: 2023
```

### Q12: 地平线tros.b的1.x版本和2.x版本（及更新版本）之间有什么主要说明和差异？
**A:**
* **和系统版本、RDK平台硬件对应关系：**
    * **2.x版本tros.b (及后续如3.x等)：**
        * 通常仅支持对应大版本的RDK OS系统（例如，tros.b 2.x支持RDK OS 2.x）。
        * 支持RDK X3、RDK X3 Module等较新的全系列硬件。
        * 未来tros.b的新增功能和主要维护会集中在这些较新的版本上。
        * 代码通常托管在GitHub上的 `D-Robotics` 组织下。
    * **1.x版本tros.b：**
        * 属于历史版本。
        * 仅支持较早的1.x版本RDK OS系统和特定的RDK硬件（如早期的RDK X3）。
        * 未来1.x版本tros.b可能仅发布关键问题修复，不再有新功能迭代。
        * 代码曾托管在gitlab或其他内部平台。
        * 参考链接 (历史版本文档)：[1.x版本tros.b说明](https://developer.d-robotics.cc/api/v1/fileData/TogetherROS/index.html)

    :::caution **注意**
    1.x版本tros.b**无法**通过`apt`命令直接升级到2.x或更新版本的tros.b。如果需要使用新版tros.b，必须先将RDK板卡的整个操作系统通过烧录镜像的方式升级到支持新版tros.b的RDK OS版本，然后再安装对应版本的tros.b。
    参考：[安装对应板卡系统](../01_Quick_start/install_os/rdk_x3.md) (请将链接替换为实际有效的文档路径)
    :::

* **功能差异：**
    * 基础的ROS2核心功能在兼容版本间是相同的。
    * 地平线针对其硬件特性优化的功能、新增的特定package以及最新的AI算法支持等，通常会优先或仅在2.x及更新版本的tros.b中提供。
* **安装包管理方式不同：**
    * **1.x版本tros.b：** 可能采用一个较大的整体安装包文件。
    * **2.x版本tros.b (及更新版本)：** 通常会根据功能模块将tros.b拆分为多个更细粒度的Debian软件包（如`tros-ros-base`, `tros-dnn-node`, `tros-mipi-cam`等），用户可以按需安装。对于开发者而言，通过`apt install tros`（元包）或`apt install <specific_tros_package>`来安装，使用体验上差异不大。
* **使用差异：**
    * **apt安装和升级方法：** 基本的`apt`命令使用方式是类似的，但软件源和包名可能会有区别。
    * **源码编译方法：** 编译流程和工具（如Colcon）基本一致，但依赖的ROS2基础版本和特定库版本会有所不同。
    * **示例的launch启动脚本：** 2.x及更新版本的tros.b，其示例的launch启动脚本文件名、参数、依赖关系等可能进行了优化和调整，与1.x版本不完全兼容。请务必参考对应tros.b版本的手册来运行示例。

### Q13: 使用WEB浏览器（如Chrome, Edge, Firefox）通过IP地址加端口号（例如 `http://<RDK_IP>:8000`）访问RDK上运行的WEB服务（如TROS的Websocket可视化示例）时，页面打开失败，可能是什么原因？
**A:** 如果浏览器无法打开RDK上托管的WEB页面，可能的原因如下：

* **Nginx服务冲突或未正确启动 (针对某些依赖Nginx的WEB示例)：**
    * **问题原因：** 如果RDK板卡上已经因为其他应用（例如，之前运行过某个不带特定端口号的WEB展示示例，它可能已启动了一个全局的Nginx服务监听80端口）而启动了Nginx服务，那么当您尝试启动一个新的、也想使用Nginx（或者特定端口）的WEB示例时，可能会因为Nginx已在运行或端口已被占用而导致新服务无法正常启动或监听在预期端口。
    * **解决方法：**
        1.  **检查并停止现有Nginx进程：** SSH登录到RDK板卡，使用 `ps aux | grep nginx` 查看是否有Nginx进程在运行。如果有，尝试使用 `sudo systemctl stop nginx` (如果是systemd服务) 或 `sudo pkill nginx` 来停止它们。
        2.  **重启RDK板卡：** 一个简单粗暴但有效的方法是重启板卡，以确保所有旧的服务进程都已关闭。
        3.  然后再重新运行您的目标WEB示例。

* **网络连接问题：**
    * 确保您的PC和RDK板卡在同一局域网内，且网络通畅（PC能ping通RDK的IP地址）。
    * 检查RDK板卡的IP地址是否正确。

* **防火墙问题：**
    * PC端或网络中的防火墙可能阻止了对RDK板卡目标端口（如8000）的访问。请检查并配置防火墙规则允许该端口的通信。
    * RDK板卡自身的防火墙（如`ufw`，虽然默认可能未开启）如果配置不当也可能阻止外部访问。

* **WEB服务本身未成功启动或监听错误：**
    * SSH登录到RDK板卡，检查您期望运行的WEB服务（例如TROS的`hobot_websocket`节点或其他Python HTTP服务器等）是否真的已经成功启动，并且正在监听您尝试访问的IP地址和端口号。
    * 查看该服务在板卡端的日志输出，看是否有报错信息。
    * 使用 `netstat -tulnp | grep <端口号>` (例如 `netstat -tulnp | grep 8000`) 命令在板卡上查看该端口是否真的处于LISTEN状态。

* **浏览器缓存或代理问题：**
    * 尝试清除浏览器缓存或使用浏览器的隐身/无痕模式访问。
    * 如果您PC的网络配置中使用了代理服务器，请检查代理设置是否影响了对局域网内IP的直接访问。

### Q14: 通过WEB浏览器访问TROS的Websocket可视化示例时，只显示摄像头图像，但没有AI感知结果（如检测框、骨骼点等）被渲染出来，是什么原因？
**A:** 如果Websocket可视化页面能显示图像但没有AI结果，通常表示图像数据流是通畅的，但AI结果数据流可能存在问题，或者前端渲染逻辑未被正确触发。

1.  **检查Web Node启动命令参数：**
    * 许多TROS的Websocket节点（如`hobot_websocket`）在启动时可以通过参数来控制是否渲染AI感知结果。请仔细检查您启动该节点的`ros2 launch`或`ros2 run`命令，确保相关的参数（例如，可能是类似 `display_ai_results:=true` 或 `render_perception:=true` 的参数）已正确设置以开启感知结果的渲染。
    * 具体参数名称和用法，请查阅对应Websocket包（如`hobot_websocket`）的README文档或launch文件。例如：[hobot_websocket README 参数说明](https://github.com/D-Robotics/hobot_websocket#%E5%8F%82%E6%95%B0)

2.  **检查Web Node启动终端的日志：**
    * 在RDK板卡上启动Websocket节点的那个终端窗口中，仔细查看是否有任何错误（ERROR）或警告（WARN）日志输出。这些日志可能会提示AI结果处理或发送环节的问题。

3.  **确认是否有AI感知结果数据正在发布：**
    * AI感知结果（如检测框、姿态点等）通常是通过另外的ROS话题发布的（例如，类型可能是自定义的AI消息 `*_msgs/AiMsg`）。
    * 在一个新的终端中（source好TROS环境后），使用 `ros2 topic list` 查看当前所有活跃的话题列表，确认是否存在发布AI感知结果的话题。
    * 如果话题存在，使用 `ros2 topic echo /the_ai_result_topic_name` (请替换为实际的AI结果话题名) 来实时查看是否有数据正在该话题上发布。如果长时间没有数据输出，说明上游的AI推理节点可能没有正常工作或没有检测到目标。

4.  **检查是否意外启动了多个Web Node实例：**
    * 如果因为某些原因，您在板卡上意外地启动了多个Websocket节点实例，它们可能会相互干扰，或者浏览器连接到了一个没有正确接收或处理AI数据的实例。
    * 在板卡上使用 `ps aux | grep web` (或更具体的进程名) 命令检查是否有多个Websocket服务进程在运行。如果有，请使用 `kill <PID>` 命令停止所有多余的Websocket进程，然后只启动一个实例。

5.  **前端与后端数据同步或渲染逻辑问题：**
    * 确保Websocket服务器（后端，在RDK上运行）与浏览器客户端（前端）之间的消息格式和协议版本是匹配的。
    * 检查浏览器开发者工具的控制台（Console）和网络（Network）标签页，看是否有JavaScript错误或Websocket通信错误。

### Q15: 在TROS Humble版本中如何配置和使用零拷贝（Zero-Copy）数据传输？
**A:** 零拷贝是一种高效的数据传输机制，它允许数据在ROS节点间传递时避免不必要的内存拷贝，从而降低延迟、减少CPU占用，特别适用于传输大的数据块如图像。TROS Humble版本（基于ROS2 Humble）支持利用Fast DDS（默认的DDS实现之一）的共享内存 (Shared Memory, SHM) 传输特性来实现零拷贝。

**配置步骤 (适用于Ubuntu系统和Linux系统)：**

1.  **设置必要的环境变量：**
    在运行ROS节点的终端中，执行以下命令来配置Fast DDS使用共享内存进行传输：
    ```bash
    # 1. 确保RMW实现是Fast DDS (通常Humble默认就是，但显式设置更保险)
    export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

    # 2. 指定Fast DDS的配置文件路径，该文件启用了共享内存传输
    #    注意：此路径是示例，请根据您实际的TROS Humble安装路径进行调整
    #    通常在 /opt/tros/humble/lib/hobot_shm/config/shm_fastdds.xml 或类似位置
    export FASTRTPS_DEFAULT_PROFILES_FILE=/opt/tros/humble/lib/hobot_shm/config/shm_fastdds.xml 

    # 3. 强制Fast DDS从XML配置文件中加载QoS设置
    export RMW_FASTRTPS_USE_QOS_FROM_XML=1

    # 4. 启用ROS2的借贷消息 (Loaned Messages) 机制，这是实现零拷贝的关键
    export ROS_DISABLE_LOANED_MESSAGES=0 
    ```
    *
    这些环境变量的详细说明可以参考ROS2官方文档或Fast DDS的文档，例如：
    * [ROS 2 using Fast DDS middleware](https://fast-dds.docs.eprosima.com/en/latest/fastdds/ros2/ros2.html)
    * 地平线官方`hobot_shm`包的README：[hobot_shm README_cn.md](https://github.com/D-Robotics/hobot_shm/blob/develop/README_cn.md) (请访问最新的官方链接)

2.  **启动支持零拷贝的ROS节点：**
    * 发布数据的节点（Publisher）和订阅数据的节点（Subscriber）都需要在其代码中支持并使用借贷消息API。地平线官方提供的部分TROS包（如`mipi_cam`、`hobot_codec`等）可能已经适配了零拷贝。
    * 例如，启动`mipi_cam`节点发布共享内存图像：
        ```bash
        # 先source TROS Humble环境
        source /opt/tros/humble/setup.bash
        # (然后设置上面的环境变量)
        ros2 launch mipi_cam mipi_cam.launch.py mipi_video_device:=F37 
        ```
        *
    * 启动`hobot_codec`节点通过共享内存订阅图像并进行处理：
        ```bash
        # (同样需要source环境和设置环境变量)
        ros2 launch hobot_codec hobot_codec.launch.py codec_in_mode:=shared_mem codec_in_format:=nv12 codec_out_mode:=ros codec_out_format:=jpeg codec_sub_topic:=/hbmem_img codec_pub_topic:=/image_jpeg
        ```
        *

**检查是否成功使用零拷贝：**
* 当支持零拷贝的发布者和订阅者成功通过共享内存进行通信时，系统会在 `/dev/shm/` 目录下创建一些内存映射文件。您可以通过以下命令查看：
    ```bash
    ls -lthr /dev/shm/fast_datasharing* /dev/shm/fastrtps_*
    ```
    *
    如果看到有类似 `fast_datasharing_...` 的文件被创建，并且文件大小与传输的数据（如图像帧大小）相关，则表明共享内存传输可能已启用。
* 还可以使用 `lsof` 命令查看哪些进程正在使用这些共享内存文件：
    ```bash
    sudo lsof /dev/shm/fast_datasharing*
    ```
    *
    输出中应该能看到您的发布者和订阅者进程。

**禁用零拷贝功能：**
* 如果因某些原因需要禁用零拷贝（例如调试或兼容性问题），可以通过设置以下环境变量来实现，它具有最高优先级：
    ```bash
    export ROS_DISABLE_LOANED_MESSAGES=1
    ```
    *
* 禁用零拷贝的详细说明参考ROS2官方文档：[How to disable loaned messages](https://docs.ros.org/en/humble/How-To-Guides/Configure-ZeroCopy-loaned-messages.html#how-to-disable-loaned-messages)

**注意：**
* 确保`FASTRTPS_DEFAULT_PROFILES_FILE`指向的XML配置文件 (`shm_fastdds.xml`) 内容是正确的，并且确实启用了共享内存传输的相关配置。
* 零拷贝的成功启用依赖于发布者和订阅者两端都正确支持和配置了借贷消息和共享内存传输。