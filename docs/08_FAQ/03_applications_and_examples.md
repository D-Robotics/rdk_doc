---
sidebar_position: 3
---

# 8.3 应用和示例

## 第三方库在RDK上的安装/交叉编译和使用

如需交叉编译部署，请参考[交叉编译环境部署](https://developer.horizon.cc/forumDetail/112555549341653662)

## 编译大型程序过程中提示编译进程被kill错误日志如何解决？

参考[Swap使用教程](https://developer.horizon.cc/forumDetail/98129467158916281)

```shell
sudo mkdir -p /swapfile 
cd /swapfile 
sudo dd if=/dev/zero of=swap bs=1M count=1024 
sudo chmod 0600 swap 
sudo mkswap -f swap 
sudo swapon swap 
free
```

## 如何运行GC4633摄像头示例？

python已提供基于F37和GC4663 camera的FCOS算法示例，会自动检测camera进行算法推理。

```bash
cd /app/ai_inference/03_mipi_camera_sample
sudo python3 mipi_camera.py
```

然后HDMI接屏幕可以渲染图像和算法展示结果。

## 使用rqt_image_view查看RDK发布的RGB888 RAW图卡顿，甚至无法接收?

FastDDS在UDP协议层没有实现MTU分片，导致IP层进行分片，当UDP数据过大时由于常见路由器、网卡均无法缓冲大量分片，且某一分片丢失会导致重传所有分片，进而形成IP fragmentation attack，表现为同一网段机器通信卡顿。更换cycloneDDS（命令：export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp），或者发送小分辨率jpeg格式图片降低传输数据量。

## Linux镜像是否支持板端编译?

Linux镜像rootfs经过最小剪裁，无法支持板端编译。

## Linux镜像上如何运行示例

手册中的示例是以Ubuntu系统举例介绍，示例的运行依赖python，这些示例同样可以运行在烧录Linux镜像（无python）的RDK上。

- Ubuntu系统和Linux镜像启动示例说明

在Ubuntu系统中，启动一个示例分为三步：

1、使用`source /opt/tros/setup.bash`命令配置`tros.b`环境

2、拷贝需要的配置文件到执行路径下

3、使用`ros2 run`或者`ros2 launch`启动`tros.b`中的package

在Linux镜像上，这三步分别为：

1、使用`export LD_LIBRARY_PATH`命令配置`tros.b`环境；使用`export ROS_LOG_DIR`命令修改存储log文件的路径。

2、拷贝需要的配置文件到执行路径下

3、启动示例程序，并指定启动参数。对于使用C++编写的package，每个package对应于一个可执行程序。

下面使用算法推理示例举例，说明如何将launch脚本内容转换成Linux镜像上的命令。示例输入一张本地图片进行推理，得到渲染后的图片并保存在本地。

- Ubuntu系统上启动命令

命令如下：

```shell
# 配置tros.b环境
source /opt/tros/setup.bash

# 从tros.b的安装路径中拷贝出运行示例需要的配置文件。config中为example使用的模型，回灌使用的本地图片
cp -r /opt/tros/${TROS_DISTRO}/lib/dnn_node_example/config/ .

# 使用本地jpg格式图片进行回灌预测，并存储渲染后的图片
ros2 launch dnn_node_example dnn_node_example_feedback.launch.py
```

- launch脚本路径

此示例使用`ros2 launch`启动了`dnn_node_example package`，在RDK的`tros.b`安装路径`/opt/tros/`中查找launch脚本`dnn_node_example_feedback.launch.py`所在路径：

```shell
# find /opt/tros/ -name dnn_node_example_feedback.launch.py
/opt/tros/share/dnn_node_example/launch/dnn_node_example_feedback.launch.py
```

- launch脚本内容

launch脚本`dnn_node_example_feedback.launch.py`主要内容如下：

```python
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
    print("dnn_node_example_path is ", dnn_node_example_path)
    cp_cmd = "cp -r " + dnn_node_example_path + "/config ."
    print("cp_cmd is ", cp_cmd)
    os.system(cp_cmd)

    return LaunchDescription([
        config_file_launch_arg,
        img_file_launch_arg,
        # 启动单目rgb人体、人头、人脸、人手框和人体关键点检测pkg
        Node(
            package='dnn_node_example',
            executable='example',
            output='screen',
            parameters=[
                {"feed_type": 0},
                {"config_file": LaunchConfiguration(
                    'dnn_example_config_file')},
                {"image": LaunchConfiguration('dnn_example_image')},
                {"image_type": 0},
                {"dump_render_img": 1}
            ],
            arguments=['--ros-args', '--log-level', 'info']
        )
    ])
```

- launch脚本说明

launch脚本支持通过参数`dnn_example_config_file`指定配置文件选择运行的算法，参数`dnn_example_image`指定算法推理使用的图片。示例启动时未指定，使用的是默认配置。

launch脚本中`package`参数指定了启动的package名为`dnn_node_example`，`executable`参数指定可执行程序名为`example`，`parameters`参数指定了传给可执行程序的参数。ros2 launch的详细使用说明参考[ROS2手册](http://docs.ros.org/en/foxy/Tutorials/Intermediate/Launch/Launch-Main.html)。

- package和可执行程序路径

在RDK的`tros.b`安装路径`/opt/tros/`中查找：

```shell
# find /opt/tros/ -name dnn_node_example -type d
/opt/tros/lib/dnn_node_example
/opt/tros/share/dnn_node_example

# ls /opt/tros/lib/dnn_node_example
config  example
```

可以看到可执行程序`example`在`/opt/tros/lib/dnn_node_example`路径下。

- Linux镜像上启动可执行程序

运行上一步查找到的可执行程序，并且带上launch脚本中`parameters`参数：`/opt/tros/lib/dnn_node_example/example --ros-args -p feed_type:=0 -p image_type:=0 -p dump_render_img:=1`。

- 完整的Linux镜像上运行示例

```shell
# 配置tros.b环境
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/opt/tros/lib/
export ROS_LOG_DIR=/userdata/

# 从tros.b的安装路径中拷贝出运行示例需要的配置文件。config中为example使用的模型，回灌使用的本地图片
cp -r /opt/tros/${TROS_DISTRO}/lib/dnn_node_example/config/ .

# 使用本地jpg格式图片进行回灌预测，并存储渲染后的图片
/opt/tros/lib/dnn_node_example/example --ros-args -p feed_type:=0 -p image_type:=0 -p dump_render_img:=1
```

:::tip
除了使用环境变量`ROS_LOG_DIR`设置log路径外，还可以通过启动参数`--ros-args --disable-external-lib-logs`禁止node输出log到
文件。

使用举例：
```bash
# 配置tros.b环境
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/opt/tros/lib/

# 从tros.b的安装路径中拷贝出运行示例需要的配置文件。config中为example使用的模型，回灌使用的本地图片
cp -r /opt/tros/${TROS_DISTRO}/lib/dnn_node_example/config/ .

# 使用本地jpg格式图片进行回灌预测，并存储渲染后的图片
/opt/tros/lib/dnn_node_example/example --ros-args --disable-external-lib-logs --ros-args -p feed_type:=0 -p image_type:=0 -p dump_render_img:=1
```

详细说明参考[About-Logging](https://docs.ros.org/en/humble/Concepts/Intermediate/About-Logging.html)。
:::

## 如何查找launch启动脚本所在路径

手册中的示例使用`ros2 launch`启动脚本，例如**Boxs算法仓库**章节中**工具链参考算法**小节使用的启动脚本文件为`dnn_node_example.launch.py`，当需要修改log级别等配置时，需要先在RDK的`tros.b`安装路径`/opt/tros/`下查找此脚本文件所在路径。

例如查找launch脚本`dnn_node_example.launch.py`所在路径命令如下：

```shell
# find /opt/tros/ -name dnn_node_example.launch.py
/opt/tros/share/dnn_node_example/launch/dnn_node_example.launch.py
```

## 交叉编译TogetheROS.Bot源码速度慢

tros.b的package较多，源码编译需要一些时间（8核CPU & 32G内存配置下编译完成大概需要20分钟）。有两种加速方法：

1 最小化编译

编译脚本中提供了两种编译方式，分别为`all_build.sh`完整编译（手册交叉编译章节中默认的编译方式）和`minimal_build.sh`最小化编译。其中最小化编译不编译算法示例和测试用例，编译速度快。

使用最小化编译的方法是将手册交叉编译章节中配置编译选项命令`./robot_dev_config/all_build.sh`替换为`./robot_dev_config/minimal_build.sh`。

2 手动忽略编译不需要的package

在package源码目录下生成`COLCON_IGNORE`文件，编译的时候将会忽略编译此package。

下载下来的package源码目录在`robot_dev_config/ros2_release.repos`中指定，例如下载`google_benchmark_vendor`时配置如下：

```text
  ament/google_benchmark_vendor:
    type: git
    url: https://github.com/ament/google_benchmark_vendor.git
    version: 0.0.7
```

说明`google_benchmark_vendor` package源码下载在了`src/ament/google_benchmark_vendor`路径下，因此执行命令`touch src/ament/google_benchmark_vendor/COLCON_IGNORE`忽略`google_benchmark_vendor` package的编译。

## 是否支持安装使用其他版本ROS?

支持。

RDK上安装了tros.b之后，还可以安装使用其他版本的ROS，包括ROS1。

:::caution **注意**
一个终端下只能source一个版本的ROS。例如source了tros之后不能再source ROS2 Foxy或者ROS1，或者source了ROS2 Foxy或者ROS1之后不能再source tros。
:::

此外tros.b与ROS foxy版本接口完全兼容，不需要安装ROS foxy也能够复用ROS丰富工具包。

## colcon编译报错

如果使用`colcon build`命令编译pkg报如下错误：

```shell
root@ubuntu:~/hobot_cam# colcon build
[4.933s] ERROR:colcon.colcon_core.package_identification:Exception in package identification extension 'ros' in 'hobot_cam': module 'pyparsing' has no attribute 'operatorPrecedence'
Traceback (most recent call last):
  File "/usr/lib/python3/dist-packages/catkin_pkg/condition.py", line 23, in evaluate_condition
    expr = _get_condition_expression()
  File "/usr/lib/python3/dist-packages/catkin_pkg/condition.py", line 44, in _get_condition_expression
    _condition_expression = pp.operatorPrecedence(
AttributeError: module 'pyparsing' has no attribute 'operatorPrecedence'
```
可能是`python3-catkin-pkg`版本较低，condition功能支持不完备。

**解决方法**

升级`python3-catkin-pkg`版本，步骤如下：

```shell
# 添加ROS apt源
sudo apt update && sudo apt install curl gnupg2 lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# 删除老版本
sudo apt remove python3-catkin-pkg

# 安装新版本
sudo apt update
sudo apt install python3-catkin-pkg
```

## 如何查看tros.b版本

tros.b安装完成后，登录系统并使用命令`apt show tros`查看tros.b版本。

2.x版本（以2.0.0版本为例说明）tros.b信息如下：

```shell
root@ubuntu:~# apt show tros
Package: tros
Version: 2.0.0-20230523223852
Maintainer: kairui.wang <kairui.wang@horizon.ai>
Installed-Size: unknown
Depends: hobot-models-basic, tros-ros-base, tros-ai-msgs, tros-audio-control, tros-audio-msg, tros-audio-tracking, tros-body-tracking, tros-dnn-benchmark-example, tros-dnn-node, tros-dnn-node-example, tros-dnn-node-sample, tros-elevation-net, tros-gesture-control, tros-hand-gesture-detection, tros-hand-lmk-detection, tros-hbm-img-msgs, tros-hobot-app-xrrobot-body-tracking, tros-hobot-app-xrrobot-gesture-control, tros-hobot-codec, tros-hobot-cv, tros-hobot-falldown-detection, tros-hobot-hdmi, tros-hobot-image-publisher, tros-hobot-mot, tros-hobot-usb-cam, tros-image-subscribe-example, tros-img-msgs, tros-imu-sensor, tros-line-follower-model, tros-line-follower-perception, tros-mipi-cam, tros-mono2d-body-detection, tros-mono2d-trash-detection, tros-mono3d-indoor-detection, tros-parking-perception, tros-parking-search, tros-rgbd-sensor, tros-websocket, tros-xrrobot, tros-xrrobot-msgs
Download-Size: 980 B
APT-Manual-Installed: yes
APT-Sources: http://sunrise.horizon.cc/ubuntu-rdk focal/main arm64 Packages
Description: TogetheROS Bot

root@ubuntu:~#

```

1.x版本（以1.1.6版本为例说明）tros.b信息如下：

```shell
root@ubuntu:~# apt show tros
Package: tros
Version: 1.1.6
Section: utils
Maintainer: kairui.wang <kairui.wang@horizon.ai>
Installed-Size: 1,536 MB
Pre-Depends: hhp-verify
Depends: symlinks, locales, hhp-verify, hobot-models-basic, hobot-arm64-libs (>= 1.1.6)
Apt-Sources: http://sunrise.horizon.cc/ubuntu-ports focal/main arm64 Packages
Date: 2023-03-24_17-29-12
Download-Size: 116 MB
APT-Manual-Installed: yes
Description: TogetherROS

N: There are 7 additional records. Please use the '-a' switch to see them.
root@ubuntu:~#

```

## 1.x和2.x版本tros.b说明

**和系统版本、RDK平台硬件对应关系**

- 2.x版本tros.b：仅支持2.x版本系统；支持RDK、RDK Module等全系列硬件；未来tros.b的新增功能将会发布在2.x版本tros.b；代码托管在github。

- [1.x版本tros.b](https://developer.horizon.cc/api/v1/fileData/TogetherROS/index.html)：历史版本；仅支持1.x版本系统和RDK；未来1.x版本tros.b仅发布问题修复版本；代码托管在gitlab。

:::caution **注意**
1.x版本tros.b无法通过apt命令直接升级到2.x版本tros.b，需要以烧录镜像的方式重新[安装系统](https://developer.horizon.cc/documents_rdk/installation/install_os)后再安装2.x版本tros.b。
:::

**功能差异**

- 基础功能相同。未来tros.b的新增功能将会只基于2.x版本发布。

- 安装包管理方式不同。1.x版本tros.b只有一个安装包文件，2.x版本tros.b根据功能分别进行安装包的打包和发布。对于开发者，不需要关心安装包管理方式的变化。

**使用差异**

- apt安装和升级，以及源码编译方法不变（详见**系统安装**章节）。

- **示例的launch启动脚本不同**。对启动脚本的文件名、依赖进行了优化，应用示例引用依赖模块的launch脚本并配置参数。2.x版本tros.b示例的启动脚本参考本手册。

## WEB浏览器打开页面失败

问题现象：浏览器输入`http://IP:8000`（IP为RDK的IP地址）地址后打开页面失败，其可能原因如下：

**nginx服务已启动**

问题原因：如果RDK上已经启动过nginx服务，例如运行过RDK中的WEB展示示例（不带端口号，此时在浏览器中输入`http://IP` 地址能够打开页面），再启动tros.b的WEB展示示例时不会再启动nginx服务，因此指定端口号会打开页面失败。	

解决方法：kill掉RDK上正在运行的nginx进程或者重启RDK。

## WEB浏览器只显示图像，无感知结果渲染

1、检查web node启动命令是否开启渲染感知结果功能。详细参数说明查看[hobot_websocket的README](https://github.com/HorizonRDK/hobot_websocket#%E5%8F%82%E6%95%B0)

2、检查web node启动终端是否有错误log输出，如果有请按照提示信息进行排查。

3、使用`ros2 topic echo [话题名]`命令确认是否有感知结果数据。

4、使用`ps -x`命令检查是否有启动多个web node，如果有请使用`kill`命令停止所有web node进程后再启动。

## TROS Humble版本使用零拷贝

**Ubuntu系统**

参考[hobot_shm](https://github.com/HorizonRDK/hobot_shm/blob/develop/README_cn.md
)的使用说明。

**Linux系统**

使用以下命令设置零拷贝环境：

```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/opt/tros/humble/lib/hobot_shm/config/shm_fastdds.xml
export RMW_FASTRTPS_USE_QOS_FROM_XML=1
export ROS_DISABLE_LOANED_MESSAGES=0
```

以上命令同样适用于Ubuntu系统。

环境变量的说明参考[ROS 2 using Fast DDS middleware](https://fast-dds.docs.eprosima.com/en/latest/fastdds/ros2/ros2.html)。

**检查是否使用零拷贝传输数据**

启动程序后，使用命令查看是否有内存映射文件生成，如果有说明已经使用零拷贝传输数据：

```bash
ll -thr /dev/shm/fast_datasharing* /dev/shm/fastrtps_*
```

使用举例：

- 设置零拷贝环境

```bash
root@ubuntu:~# source /opt/tros/humble/setup.bash
root@ubuntu:~# export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
root@ubuntu:~# export FASTRTPS_DEFAULT_PROFILES_FILE=/opt/tros/humble/lib/hobot_shm/config/shm_fastdds.xml
root@ubuntu:~# export RMW_FASTRTPS_USE_QOS_FROM_XML=1
root@ubuntu:~# export ROS_DISABLE_LOANED_MESSAGES=0
root@ubuntu:~# ll -thr /dev/shm/fast_datasharing* /dev/shm/fastrtps_*
ls: cannot access '/dev/shm/fast_datasharing*': No such file or directory
ls: cannot access '/dev/shm/fastrtps_*': No such file or directory
```

可以看到，只设置零拷贝环境的情况下没有生成内存映射文件，因为内存映射文件需要由程序创建。

- 运行mipi_cam node通过零拷贝发布数据

```bash
source /opt/tros/humble/setup.bash
ros2 launch mipi_cam mipi_cam.launch.py mipi_video_device:=F37
```

- 再次查看内存映射文件

```bash
root@ubuntu:~# ll -thr /dev/shm/fast_datasharing* /dev/shm/fastrtps_*
-rw-r--r-- 1 root root    0 Mar 26 14:01 /dev/shm/fastrtps_311b4cf8328b77f9_el
-rw-r--r-- 1 root root 537K Mar 26 14:01 /dev/shm/fastrtps_311b4cf8328b77f9
-rw-r--r-- 1 root root  36M Mar 26 14:01 /dev/shm/fast_datasharing_01.0f.1d.90.d8.ac.a8.ff.01.00.00.00_0.0.1f.3
-rw-r--r-- 1 root root    0 Mar 26 14:17 /dev/shm/fastrtps_eef6d2045292439c_el
-rw-r--r-- 1 root root 537K Mar 26 14:17 /dev/shm/fastrtps_eef6d2045292439c
-rw-r--r-- 1 root root    0 Mar 26 14:17 /dev/shm/fastrtps_port17913_el
-rw-r--r-- 1 root root  52K Mar 26 14:17 /dev/shm/fastrtps_port17913
-rw-r--r-- 1 root root  36M Mar 26 14:17 /dev/shm/fast_datasharing_01.0f.1d.90.21.42.cb.90.01.00.00.00_0.0.1f.3
```

以上log显示，启用零拷贝功能并且运行mipi_cam node后，在/dev/shm目录下出现了多个文件，说明mipi_cam node**支持使用零拷贝**发布数据。

- 启动零拷贝消息订阅

```bash
source /opt/tros/humble/setup.bash
ros2 launch hobot_codec hobot_codec.launch.py codec_in_mode:=shared_mem codec_in_format:=nv12 codec_out_mode:=ros codec_out_format:=jpeg codec_sub_topic:=/hbmem_img codec_pub_topic:=/image_jpeg
```

- 再次查看内存映射文件

```bash
root@ubuntu:~# ll -thr /dev/shm/fast_datasharing* /dev/shm/fastrtps_*
-rw-r--r-- 1 root root    0 Mar 26 14:01 /dev/shm/fastrtps_311b4cf8328b77f9_el
-rw-r--r-- 1 root root 537K Mar 26 14:01 /dev/shm/fastrtps_311b4cf8328b77f9
-rw-r--r-- 1 root root  36M Mar 26 14:01 /dev/shm/fast_datasharing_01.0f.1d.90.d8.ac.a8.ff.01.00.00.00_0.0.1f.3
-rw-r--r-- 1 root root    0 Mar 26 14:17 /dev/shm/fastrtps_eef6d2045292439c_el
-rw-r--r-- 1 root root 537K Mar 26 14:17 /dev/shm/fastrtps_eef6d2045292439c
-rw-r--r-- 1 root root    0 Mar 26 14:17 /dev/shm/fastrtps_port17913_el
-rw-r--r-- 1 root root  52K Mar 26 14:17 /dev/shm/fastrtps_port17913
-rw-r--r-- 1 root root  36M Mar 26 14:17 /dev/shm/fast_datasharing_01.0f.1d.90.21.42.cb.90.01.00.00.00_0.0.1f.3
-rw-r--r-- 1 root root    0 Mar 26 14:19 /dev/shm/fastrtps_dbda9faf3f77dee0_el
-rw-r--r-- 1 root root 537K Mar 26 14:19 /dev/shm/fastrtps_dbda9faf3f77dee0
-rw-r--r-- 1 root root    0 Mar 26 14:19 /dev/shm/fastrtps_port17915_el
-rw-r--r-- 1 root root  52K Mar 26 14:19 /dev/shm/fastrtps_port17915
-rw-r--r-- 1 root root  22K Mar 26 14:19 /dev/shm/fast_datasharing_01.0f.1d.90.23.5d.bd.63.01.00.00.00_0.0.1e.4
root@ubuntu:~#
root@ubuntu:~# lsof /dev/shm/fast_datasharing*
COMMAND       PID USER  FD   TYPE DEVICE SIZE/OFF     NODE NAME
mipi_cam  2507297 root mem    REG   0,17 37327756 18131245 /dev/shm/fast_datasharing_01.0f.1d.90.21.42.cb.90.01.00.00.00_0.0.1f.3
mipi_cam  2507297 root mem    REG   0,17    21656 18149313 /dev/shm/fast_datasharing_01.0f.1d.90.23.5d.bd.63.01.00.00.00_0.0.1e.4
hobot_cod 2514211 root mem    REG   0,17 37327756 18131245 /dev/shm/fast_datasharing_01.0f.1d.90.21.42.cb.90.01.00.00.00_0.0.1f.3
hobot_cod 2514211 root mem    REG   0,17    21656 18149313 /dev/shm/fast_datasharing_01.0f.1d.90.23.5d.bd.63.01.00.00.00_0.0.1e.4
```

可以看到，/dev/shm目录下出现了新的内存映射文件，并且这些文件被mipi_cam和hobot_codec进程占用，说明hobot_codec node在通过零拷贝订阅mipi_cam node发布的数据。


**禁用零拷贝功能**

通过环境变量`ROS_DISABLE_LOANED_MESSAGES`禁止零拷贝功能，具有最高控制优先级：

```bash
export ROS_DISABLE_LOANED_MESSAGES=1
```

禁用零拷贝功能配置详细说明参考[how-to-disable-loaned-messages](https://docs.ros.org/en/humble/How-To-Guides/Configure-ZeroCopy-loaned-messages.html#how-to-disable-loaned-messages)。
