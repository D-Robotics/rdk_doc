---
sidebar_position: 3
---

# 8.3 Application Development, Compilation, and Examples

This section mainly answers questions related to installing/using third-party libraries, compiling applications, running official examples, and related issues on the RDK platform.

For cross-compilation and deployment, please refer to [Cross-compilation Environment Deployment](https://developer.d-robotics.cc/forumDetail/112555549341653662).

### Q1: How to install/cross-compile and use third-party libraries on RDK?
**A:**
* **Direct installation on the board:** If the third-party library provides a precompiled package for ARM architecture (such as a `.deb` file), or can be installed directly via a package manager (like `apt`), you can install it directly on the RDK board. For Python libraries, if there is a corresponding arm64 wheels package on Pypi, you can also use `pip install` directly.
* **Cross-compilation:** If the third-party library needs to be compiled from source, it is recommended to cross-compile on the PC development host and then deploy the compiled artifacts to the RDK board.
    * **Environment setup:** For detailed steps on setting up the cross-compilation environment, please refer to the tutorial on the Horizon Developer Community: [Cross-compilation Environment Deployment](https://developer.d-robotics.cc/forumDetail/112555549341653662)
    * **Compilation steps:** Usually, you need to configure a CMake Toolchain file, specify the cross-compiler, target system Sysroot, etc.

### Q2: When compiling large programs (such as C++ projects or ROS packages), if the system prompts that the compilation process is "killed" or there are error logs related to insufficient memory, how to solve it?
**A:** When compiling large projects, if physical memory is insufficient, the Linux OOM (Out Of Memory) killer may kill the process that consumes the most memory (usually compiler processes like `cc1plus`, `ld`, etc.), causing the compilation to fail.
**Solution:** Increase the system's swap space. Swap is an area on the hard disk that the system can use to temporarily store less frequently used memory data when physical memory is insufficient, thus freeing up physical memory for current tasks. Although swap is slower than physical memory, it can effectively prevent compilation failures due to temporary memory shortages.

**Example steps to add 1GB swap space:**
```bash
# 1. (Optional) Create a directory to store the swap file, or create it in the root directory
sudo mkdir -p /swapfile_custom_dir 
cd /swapfile_custom_dir

# 2. Create an empty file of the specified size (bs=1M means block size is 1MB, count=1024 means 1024 blocks, i.e., 1GB)
sudo dd if=/dev/zero of=swap bs=1M count=1024 

# 3. Set correct file permissions (only root can read/write)
sudo chmod 0600 swap 

# 4. Format the file as swap
sudo mkswap -f swap 

# 5. Enable the swap partition
sudo swapon swap 

# 6. Verify if swap is enabled (shows total and used swap)
free -h
swapon --show
```

**To enable swap at boot (optional but recommended):**
Edit the `/etc/fstab` file and add the following line at the end (assuming your swap file path is `/swapfile_custom_dir/swap`):
```bash
/swapfile_custom_dir/swap none swap sw 0 0
```

**Reference tutorial:** [Swap Usage Tutorial](https://developer.d-robotics.cc/forumDetail/98129467158916281)

### Q3: How to run the GC4633 MIPI camera sample program?
**A:** Horizon officially provides AI algorithm samples (such as FCOS object detection) based on common MIPI cameras (such as F37, GC4663). These samples usually automatically detect the connected camera model and perform algorithm inference.

**Example steps (using `/app/ai_inference/03_mipi_camera_sample` as an example):**
1.  Make sure the GC4663 (or other compatible MIPI camera) is properly connected to the RDK board's MIPI CSI interface and the board is powered on.
2.  Log in to the board system via SSH or serial port.
3.  Enter the directory where the sample program is located:
    ```bash
    cd /app/ai_inference/03_mipi_camera_sample 
    # Note: The specific path may vary depending on the RDK system version and image content.
    ```
4.  Run the Python sample script with `sudo`:
    ```bash
    sudo python3 mipi_camera.py
    ```
5.  If the sample is designed to output via HDMI, make sure the RDK board's HDMI interface is connected to a monitor. After running, you should see the real-time image captured by the camera and the AI algorithm results (such as detection boxes, classification labels, etc.) on the monitor.

### Q4: When using `rqt_image_view` to view RGB888 RAW images published by RDK via ROS, it feels very laggy or even unable to receive images. What is the reason?
**A:** This problem is usually related to the configuration of the ROS2 middleware DDS and network transmission efficiency, especially when transmitting uncompressed large-size raw image data.
* **Analysis:**
    * The default FastDDS may not implement effective MTU (Maximum Transmission Unit) fragmentation at the UDP protocol layer. When the published image data packet size exceeds the MTU on the network path, the IP layer will fragment it.
    * A large number of IP fragments can be a heavy burden for many common routers, switches, or network cards, possibly leading to insufficient buffering for all fragments.
    * In UDP transmission, if any IP fragment is lost, the entire UDP packet (i.e., the entire image frame) is usually discarded, or needs to wait for retransmission (if the upper layer has such a mechanism, but ROS image topics usually do not guarantee reliable transmission), which can cause severe lag or image loss. This is sometimes referred to as an "IP fragmentation attack"-like phenomenon, i.e., a large number of fragments causing network congestion and packet loss.
* **Solutions:**
    1.  **Switch DDS implementation:** Try switching the ROS2 RMW (ROS Middleware) implementation from the default `rmw_fastrtps_cpp` to `rmw_cyclonedds_cpp`. CycloneDDS sometimes performs better in handling large data packets and network fragmentation.
        In the terminal, run the following command to switch DDS (only effective for the current terminal session, or add to `.bashrc`):
        ```bash
        export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
        ```
        Then restart your ROS nodes.
    2.  **Reduce transmission data volume:**
        * **Send compressed images:** Consider compressing the raw image (such as RGB888) to JPEG or PNG format on the RDK board before publishing via ROS topic. This will significantly reduce the data volume per frame. You can subscribe to the compressed image topic on the PC and use `rqt_image_view` (or a custom node) to decompress and display.
        * **Reduce resolution or frame rate:** If the application allows, appropriately reducing the published image resolution or frame rate can also effectively reduce network load.

### Q5: Does the Linux image provided by Horizon (specifically the minimal system, not the full Ubuntu Desktop/Server) support direct compilation on the board?
**A:** Some Linux images provided by Horizon for RDK, especially those with a minimal rootfs (root filesystem) for embedded deployment, may **not include** a complete compilation toolchain (such as GCC, G++, make, CMake, etc.) and the required header files and libraries.
* **Conclusion:** Such minimal Linux images usually **cannot or are not suitable** for complex source code compilation directly on the board.
* **Recommended practice:** For applications that need to run on RDK, it is recommended to use **cross-compilation**. That is, configure a cross-compilation environment for the RDK target platform on a PC development host (such as Ubuntu PC), complete the compilation on the PC, and then deploy the generated executables and dependencies to the RDK board.

### Q6: How to run the official manual examples (usually demonstrated in Ubuntu system environment) on the minimal Linux image provided by Horizon?
**A:** The official manual examples (especially TROS/ROS-related examples) are usually demonstrated in a more complete Ubuntu system environment. To run these examples (especially C++ ROS nodes) on a minimal Linux image (which may not have a pre-installed Python interpreter or a complete ROS environment), some adjustments are needed:

* **Differences between Ubuntu system and Linux image when starting examples:**
    * **Environment configuration:**
        * **Ubuntu system:** Usually uses `source /opt/tros/setup.bash` (or the corresponding ROS version's setup.bash) to configure the TROS/ROS environment, which sets many environment variables (such as `PATH`, `LD_LIBRARY_PATH`, `AMENT_PREFIX_PATH`, etc.).
        * **Linux image:** You may need to manually set key environment variables, especially `LD_LIBRARY_PATH` to ensure the program can find the required shared libraries. The log path `ROS_LOG_DIR` may also need to be manually set to a writable location.
    * **Copying configuration files:** In either system, you usually need to copy the configuration files (such as model configs, parameter files, etc.) required by the example from the TROS/ROS installation path to the current working directory or a specified path before running the example.
    * **Startup method:**
        * **Ubuntu system:** Commonly uses `ros2 run <package_name> <executable_name>` or `ros2 launch <package_name> <launch_file_name>` to start nodes or launch files.
        * **Linux image:** Since there may not be a complete `ros2` CLI tool or launch system, you usually need to run the compiled C++ executable directly and pass parameters via command line arguments.

* **Convert launch script content to direct execution commands on Linux image (using a C++ `dnn_node_example` as an example):**

    1.  **Analyze the startup command on Ubuntu:**
        ```bash
        # Ubuntu: Configure tros.b environment
        source /opt/tros/setup.bash

        # Ubuntu: Copy the config files required by the example from the tros.b installation path
        cp -r /opt/tros/${TROS_DISTRO}/lib/dnn_node_example/config/ .

        # Ubuntu: Use a local jpg image for inference and store the rendered image
        ros2 launch dnn_node_example dnn_node_example_feedback.launch.py
        ```

    2.  **Find and analyze the launch script:**
        * **Find the launch script path:**
            ```bash
            # find /opt/tros/ -name dnn_node_example_feedback.launch.py
            /opt/tros/share/dnn_node_example/launch/dnn_node_example_feedback.launch.py
            ```
        * **View the launch script content (Python launch file):**
            ```python
            # dnn_node_example_feedback.launch.py (main content excerpt)
            def generate_launch_description():
                config_file_launch_arg = DeclareLaunchArgument(
                    "dnn_example_config_file", default_value=TextSubstitution(text="config/fcosworkconfig.json")
                )

                img_file_launch_arg = DeclareLaunchArgument(
                    "dnn_example_image", default_value=TextSubstitution(text="config/test.jpg")
                )

                return LaunchDescription([
                    config_file_launch_arg,
                    img_file_launch_arg,
                    Node(
                        package='dnn_node_example',
                        executable='example', # Executable name
                        output='screen',
                        parameters=[
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
        From the launch script, we know it starts the `example` executable in the `dnn_node_example` package and passes a series of parameters.

    3.  **Find the executable path:**
        In the TROS installation path, find the executable:
        ```bash
        # find /opt/tros/ -name example -executable -type f 
        # Usually located at /opt/tros/${TROS_DISTRO}/lib/<package_name>/<executable_name>
        # Example: /opt/tros/humble/lib/dnn_node_example/example 
        ```
        (Assume `TROS_DISTRO` is not set on the Linux image, you need to know the actual distro name, such as `humble` or `foxy`)

    4.  **Construct the startup command on the Linux image:**
        * **Configure environment:**
            ```bash
            # Assume TROS libraries are in /opt/tros/humble/lib (adjust as needed)
            export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/opt/tros/humble/lib/ 
            # Specify a writable log directory
            export ROS_LOG_DIR=/userdata/  # or /tmp/roslogs/
            mkdir -p $ROS_LOG_DIR
            ```
        * **Copy config files:** (same as on Ubuntu)
            ```bash
            # For humble version:
            cp -r /opt/tros/humble/lib/dnn_node_example/config/ .
            ```
        * **Run the executable and pass parameters:**
            ROS2 node parameters are usually passed as `--ros-args -p <param_name>:=<param_value>`.
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

    * **A complete example script for running on the Linux image:**
        ```bash
        #!/bin/bash

        # 1. Configure environment
        # Adjust according to actual TROS version and installation path
        TROS_DISTRO_NAME="humble" # or "foxy", etc.
        TROS_INSTALL_LIB_DIR="/opt/tros/${TROS_DISTRO_NAME}/lib"
        export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:${TROS_INSTALL_LIB_DIR}

        export ROS_LOG_DIR=/userdata/ros_logs_$(date +%s)
        mkdir -p $ROS_LOG_DIR
        echo "ROS logs will be stored in $ROS_LOG_DIR"

        # 2. Prepare working directory and config files
        WORK_DIR="/tmp/dnn_example_run_$(date +%s)"
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

        # 3. Run the executable
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
    * Besides using the `ROS_LOG_DIR` environment variable to set the log path, you can also use the startup parameter `--ros-args --disable-external-lib-logs` to disable node logging to files and print logs directly to the console.
        For example:
        ```bash
        $EXECUTABLE_PATH --ros-args --disable-external-lib-logs \
            -p feed_type:=0 -p image_type:=0 -p dump_render_img:=1 
        ```
    * For detailed ROS2 logging instructions, see: [ROS2 Official Docs - About Logging](https://docs.ros.org/en/humble/Concepts/Intermediate/About-Logging.html)
    :::

### Q7: How to quickly find the specific path of a launch script file in ROS/TROS?
**A:** If you know the name of a launch script (such as `dnn_node_example.launch.py`) but need to modify or view its content, you can use the `find` command under the TROS installation path on the RDK board (usually `/opt/tros/`).

**Example:**
```bash
# Find the file named dnn_node_example.launch.py
find /opt/tros/ -name dnn_node_example.launch.py
```

### Q8: Cross-compiling the full source code of TogetheROS.Bot (tros.b) is very slow. How can I speed it up?
**A:** Compiling all packages of tros.b does take a long time (for example, about 20 minutes on an 8-core CPU, 32GB RAM PC). Here are two ways to speed up compilation:

1.  **Use the minimal build script:**
    * The tros.b build scripts provided by Horizon usually include a `minimal_build.sh` option in addition to `all_build.sh` (full build).
    * Minimal build usually skips compiling algorithm examples and test cases, significantly reducing build time.
    * **Usage:** In your cross-compilation configuration steps, replace the original `./robot_dev_config/all_build.sh` command with `./robot_dev_config/minimal_build.sh`.

2.  **Manually ignore unnecessary packages:**
    * Colcon (the ROS2 build tool) supports ignoring a package by placing an empty file named `COLCON_IGNORE` in the source directory of that package.
    * **Steps:**
        1.  Identify which packages you don't need. These packages' source code is usually downloaded to the `src/` directory before compilation via a `.repos` file (such as `robot_dev_config/ros2_release.repos`).
        2.  Check the `.repos` file to find the source path of the package you want to ignore. For example:
            ```yaml
            ament/google_benchmark_vendor:
              type: git
              url: https://github.com/ament/google_benchmark_vendor.git
              version: 0.0.7
            ```
            This means the `google_benchmark_vendor` package source will be downloaded to `src/ament/google_benchmark_vendor/`.
        3.  Create an empty `COLCON_IGNORE` file in the package's source root directory:
            ```bash
            touch src/ament/google_benchmark_vendor/COLCON_IGNORE
            ```
        4.  When you run `colcon build` next time, this package will be skipped. You can do this for multiple packages.

### Q9: After installing the official tros.b on the RDK board, can I still install and use other versions of ROS (such as ROS1 or different ROS2 distributions)?
**A:** **Yes.**
* After installing Horizon's tros.b (e.g., based on ROS2 Humble) on the RDK board, you can still try to install other versions of ROS, including ROS1 (such as Noetic, Melodic) or other ROS2 distributions (such as Foxy, Galactic, etc., if they support ARM64 and you can find or build the packages).
* Different ROS versions can coexist on the system, usually installed in different paths (e.g., ROS1 in `/opt/ros/noetic/`, ROS2 Humble in `/opt/ros/humble/`, tros.b may be in `/opt/tros/humble/`).

    :::caution **Important Note**
    **Only one ROS version can be sourced in a single terminal session!**
    * If you run `source /opt/tros/humble/setup.bash` in a terminal to activate tros.b (Humble), you **cannot** source another ROS version (such as `source /opt/ros/foxy/setup.bash` or `source /opt/ros/noetic/setup.bash`) in the same terminal, and vice versa.
    * Sourcing multiple ROS versions in the same session will cause environment variable conflicts (`PATH`, `LD_LIBRARY_PATH`, `PYTHONPATH`, `AMENT_PREFIX_PATH`, `ROS_PACKAGE_PATH`, etc.), leading to abnormal ROS commands and program behavior.
    * If you need to switch between different ROS versions, open a separate terminal session for each version and source the corresponding `setup.bash` file.
    :::

* **tros.b and ROS2 Foxy/Humble compatibility:**
    * Horizon's tros.b is usually built and optimized based on a specific ROS2 LTS version (such as Foxy or Humble) and maintains API compatibility. This means if your tros.b is based on Humble, you can usually use tools and libraries developed for standard ROS2 Humble directly, without installing ROS2 Humble separately (unless you need specific tools from the full ROS2 Desktop not included in tros.b).

### Q10: When compiling a ROS2 package with `colcon build`, an error `AttributeError: module 'pyparsing' has no attribute 'operatorPrecedence'` occurs. How to solve it?
**A:** This error is usually caused by an outdated version of `python3-catkin-pkg` (a Python library for parsing ROS package.xml files), which is incompatible with the installed version of `pyparsing`, or due to incomplete functionality in `python3-catkin-pkg`.

**Solution:** Try upgrading `python3-catkin-pkg` to a newer version provided by the official ROS repository.

**Steps:**
1.  **Add the official ROS APT repository** (if not already added):
    ```bash
    sudo apt update && sudo apt install curl gnupg2 lsb-release
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    ```
    Note: `$(source /etc/os-release && echo $UBUNTU_CODENAME)` automatically gets your Ubuntu codename (e.g., `focal` for 20.04, `jammy` for 22.04).

2.  **Remove the old version of `python3-catkin-pkg`** (optional, but sometimes helps for a clean install):
    ```bash
    sudo apt remove python3-catkin-pkg
    ```

3.  **Update APT cache and install the new version of `python3-catkin-pkg`:**
    ```bash
    sudo apt update
    sudo apt install python3-catkin-pkg
    ```
4.  After installation/upgrade, try running `colcon build` again.

If the problem persists, you may also need to check the version of the `pyparsing` library and ensure it is compatible with the new `python3-catkin-pkg`. Sometimes you may need to manage the `pyparsing` version via `pip`.

### Q11: How to check the version information of tros.b?
**A:** After installing tros.b, log in to the RDK system and use the following command to check the version information of the tros.b meta-package, which usually represents the base version of the entire tros.b distribution:
```bash
apt show tros
```

**Example output (RDK OS 2.x, tros.b 2.0.0):**
```bash
Package: tros
Version: 2.0.0-20230523223852
Maintainer: kairui.wang <kairui.wang@horizon.ai>
Installed-Size: unknown
Depends: hobot-models-basic, tros-ros-base, tros-ai-msgs, ... (many dependencies)
Download-Size: 980 B
APT-Manual-Installed: yes
APT-Sources: http://archive.d-robotics.cc/ubuntu-rdk focal/main arm64 Packages
Description: TogetheROS Bot
```

**Example output (RDK OS 1.x, tros.b 1.1.6):**
```bash
Package: tros
Version: 1.1.6
Section: utils
Maintainer: kairui.wang <kairui.wang@horizon.ai>
Installed-Size: 1,536 MB
Pre-Depends: hhp-verify
Depends: symlinks, locales, hhp-verify, hobot-models-basic, hobot-arm64-libs (>= 1.1.6)
Apt-Sources: http://archive.d-robotics.cc/ubuntu-ports focal/main arm64 Packages
Date: 2023
```

### Q12: What are the main notes and differences between Horizon tros.b 1.x and 2.x (and newer) versions?
**A:**
* **Correspondence with system version and RDK hardware:**
    * **2.x tros.b (and later, such as 3.x):**
        * Usually only supports the corresponding major version of RDK OS (e.g., tros.b 2.x supports RDK OS 2.x).
        * Supports newer hardware such as RDK X3, RDK X3 Module, etc.
        * New features and main maintenance will focus on these newer versions.
        * Code is usually hosted under the `D-Robotics` organization on GitHub.
    * **1.x tros.b:**
        * Historical version.
        * Only supports earlier 1.x RDK OS and specific RDK hardware (such as early RDK X3).
        * Future 1.x versions may only receive critical fixes, with no new features.
        * Code was once hosted on GitLab or other internal platforms.
        * Reference link (historical version docs): [1.x tros.b Documentation](https://developer.d-robotics.cc/api/v1/fileData/TogetherROS/index.html)[Chinese Only]

    :::caution **Note**
    1.x tros.b **cannot** be upgraded directly to 2.x or newer via `apt`. To use the new version, you must first upgrade the entire RDK board OS to a version that supports the new tros.b by flashing the image, then install the corresponding tros.b version.
    See: [Install the corresponding board OS](../01_Quick_start/install_os/rdk_x3.md) (replace with the actual valid doc path)
    :::

* **Feature differences:**
    * The core ROS2 functionality is the same between compatible versions.
    * Horizon's hardware-optimized features, new specific packages, and the latest AI algorithm support are usually prioritized or only provided in 2.x and newer tros.b versions.
* **Package management differences:**
    * **1.x tros.b:** May use a large monolithic installation package.
    * **2.x tros.b (and newer):** Usually splits tros.b into multiple finer-grained Debian packages (such as `tros-ros-base`, `tros-dnn-node`, `tros-mipi-cam`, etc.), allowing users to install as needed. For developers, installing via `apt install tros` (meta-package) or `apt install <specific_tros_package>` is similar.
* **Usage differences:**
    * **apt installation and upgrade:** The basic `apt` command usage is similar, but the repository and package names may differ.
    * **Source compilation:** The compilation process and tools (such as Colcon) are basically the same, but the underlying ROS2 version and specific library versions may differ.
    * **Example launch scripts:** In 2.x and newer tros.b, example launch script names, parameters, dependencies, etc., may be optimized and adjusted, and are not fully compatible with 1.x. Always refer to the manual for your tros.b version.

### Q13: When accessing a web service running on RDK (such as a TROS Websocket visualization example) via a web browser (e.g., Chrome, Edge, Firefox) using IP and port (e.g., `http://<RDK_IP>:8000`), the page fails to open. What could be the reason?
**A:** If the browser cannot open the web page hosted on RDK, possible reasons include:

* **Nginx service conflict or not started correctly (for some web examples that depend on Nginx):**
    * **Reason:** If Nginx is already running on the RDK board (e.g., started by another application or a previous web demo), starting a new web example that also wants to use Nginx (or a specific port) may fail due to Nginx already running or the port being occupied.
    * **Solution:**
        1.  **Check and stop existing Nginx processes:** SSH into the RDK board, use `ps aux | grep nginx` to check for Nginx processes. If found, try `sudo systemctl stop nginx` (if it's a systemd service) or `sudo pkill nginx` to stop them.
        2.  **Reboot the RDK board:** A simple but effective way to ensure all old service processes are closed.
        3.  Then rerun your target web example.

* **Network connection issues:**
    * Make sure your PC and the RDK board are on the same LAN and the network is smooth (PC can ping the RDK IP).
    * Check if the RDK board's IP address is correct.

* **Firewall issues:**
    * The firewall on the PC or in the network may block access to the RDK board's target port (such as 8000). Check and configure firewall rules to allow communication on that port.
    * The RDK board's own firewall (such as `ufw`, though usually not enabled by default) may also block external access if misconfigured.

* **Web service not started or listening error:**
    * SSH into the RDK board and check if the web service you expect to run (such as TROS's `hobot_websocket` node or other Python HTTP servers) has actually started and is listening on the expected IP and port.
    * Check the service's log output for errors.
    * Use `netstat -tulnp | grep <port>` (e.g., `netstat -tulnp | grep 8000`) on the board to check if the port is in LISTEN state.

* **Browser cache or proxy issues:**
    * Try clearing the browser cache or using incognito/private mode.
    * If your PC uses a proxy server, check if the proxy settings affect direct access to LAN IPs.

### Q14: When accessing the TROS Websocket visualization example via a web browser, only the camera image is displayed, but no AI perception results (such as detection boxes, keypoints, etc.) are rendered. What is the reason?
**A:** If the Websocket visualization page shows the image but no AI results, it usually means the image data stream is working, but the AI result data stream may have issues, or the frontend rendering logic is not triggered correctly.

1.  **Check Web Node startup command parameters:**
    * Many TROS Websocket nodes (such as `hobot_websocket`) can control whether to render AI perception results via parameters at startup. Carefully check your launch command to ensure relevant parameters (such as `display_ai_results:=true` or `render_perception:=true`) are set to enable perception result rendering.
    * For specific parameter names and usage, see the corresponding Websocket package's README or launch file. For example: [hobot_websocket README Parameters](https://github.com/D-Robotics/hobot_websocket#%E5%8F%82%E6%95%B0)

2.  **Check the Web Node startup terminal logs:**
    * In the terminal where you started the Websocket node on the RDK board, check for any ERROR or WARN logs. These may indicate issues with AI result processing or sending.

3.  **Confirm if AI perception result data is being published:**
    * AI perception results (such as detection boxes, keypoints, etc.) are usually published via separate ROS topics (e.g., custom AI message types like `*_msgs/AiMsg`).
    * In a new terminal (after sourcing the TROS environment), use `ros2 topic list` to see all active topics and confirm if there is a topic publishing AI perception results.
    * If the topic exists, use `ros2 topic echo /the_ai_result_topic_name` (replace with the actual topic name) to view if data is being published. If there is no output for a long time, the upstream AI inference node may not be working or not detecting targets.

4.  **Check for multiple Web Node instances accidentally started:**
    * If you accidentally started multiple Websocket node instances on the board, they may interfere with each other, or the browser may connect to an instance not correctly receiving or processing AI data.
    * Use `ps aux | grep web` (or a more specific process name) on the board to check for multiple Websocket service processes. If found, use `kill <PID>` to stop all extra processes, then start only one instance.

5.  **Frontend-backend data sync or rendering logic issues:**
    * Ensure the Websocket server (backend, running on RDK) and browser client (frontend) have matching message formats and protocol versions.
    * Check the browser developer tools Console and Network tabs for JavaScript errors or Websocket communication errors.

### Q15: How to configure and use zero-copy data transmission in TROS Humble?
**A:** Zero-copy is an efficient data transmission mechanism that allows data to be passed between ROS nodes without unnecessary memory copying, reducing latency and CPU usage, especially for large data blocks like images. TROS Humble (based on ROS2 Humble) supports zero-copy via Fast DDS's Shared Memory (SHM) transport.

**Configuration steps (for Ubuntu and Linux systems):**

1.  **Set the required environment variables:**
    In the terminal where you run ROS nodes, execute:
    ```bash
    # 1. Ensure RMW implementation is Fast DDS (usually default in Humble, but set explicitly for safety)
    export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

    # 2. Specify the Fast DDS config file path that enables shared memory transport
    #    Adjust the path according to your actual TROS Humble installation
    #    Usually at /opt/tros/humble/lib/hobot_shm/config/shm_fastdds.xml or similar
    export FASTRTPS_DEFAULT_PROFILES_FILE=/opt/tros/humble/lib/hobot_shm/config/shm_fastdds.xml 

    # 3. Force Fast DDS to load QoS settings from XML config
    export RMW_FASTRTPS_USE_QOS_FROM_XML=1

    # 4. Enable ROS2 loaned messages (key for zero-copy)
    export ROS_DISABLE_LOANED_MESSAGES=0 
    ```
    For details, see the ROS2 or Fast DDS documentation, e.g.:
    * [ROS 2 using Fast DDS middleware](https://fast-dds.docs.eprosima.com/en/latest/fastdds/ros2/ros2.html)
    * Horizon official `hobot_shm` README: [hobot_shm README_cn.md](https://github.com/D-Robotics/hobot_shm/blob/develop/README_cn.md)

2.  **Start ROS nodes that support zero-copy:**
    * Both the publisher and subscriber nodes need to support and use the loaned message API in their code. Some Horizon TROS packages (such as `mipi_cam`, `hobot_codec`, etc.) may already support zero-copy.
    * For example, start the `mipi_cam` node to publish shared memory images:
        ```bash
        # Source TROS Humble environment first
        source /opt/tros/humble/setup.bash
        # (Then set the above environment variables)
        ros2 launch mipi_cam mipi_cam.launch.py mipi_video_device:=F37 
        ```
    * Start the `hobot_codec` node to subscribe to images via shared memory and process:
        ```bash
        # (Also source environment and set variables)
        ros2 launch hobot_codec hobot_codec.launch.py codec_in_mode:=shared_mem codec_in_format:=nv12 codec_out_mode:=ros codec_out_format:=jpeg codec_sub_topic:=/hbmem_img codec_pub_topic:=/image_jpeg
        ```

**Check if zero-copy is enabled:**
* When zero-copy communication is successful, some memory-mapped files will be created under `/dev/shm/`. You can check with:
    ```bash
    ls -lthr /dev/shm/fast_datasharing* /dev/shm/fastrtps_*
    ```
    If you see files like `fast_datasharing_...` and their size matches the data being transmitted (e.g., image frame size), shared memory transport is likely enabled.
* You can also use `lsof` to see which processes are using these shared memory files:
    ```bash
    sudo lsof /dev/shm/fast_datasharing*
    ```
    The output should show your publisher and subscriber processes.

**Disable zero-copy:**
* To disable zero-copy (for debugging or compatibility), set:
    ```bash
    export ROS_DISABLE_LOANED_MESSAGES=1
    ```
* For details, see: [How to disable loaned messages](https://docs.ros.org/en/humble/How-To-Guides/Configure-ZeroCopy-loaned-messages.html#how-to-disable-loaned-messages)

**Note:**
* Make sure the `FASTRTPS_DEFAULT_PROFILES_FILE` XML config (`shm_fastdds.xml`) is correct and enables shared memory transport.
* Zero-copy requires both publisher and subscriber to support and configure loaned messages and shared memory transport.
