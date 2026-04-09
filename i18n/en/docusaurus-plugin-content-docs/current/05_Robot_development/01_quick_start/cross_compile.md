---
sidebar_position: 3
---

# 5.1.3 Source Code Installation

```mdx-code-block
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';
```

This section describes how to install TogetheROS.Bot from source code on both RDK and X86 platforms.

## RDK Platform

Prerequisites:

- Your development machine can access the [D-Robotics](https://github.com/D-Robotics) organization normally.
- Docker is already installed on your development machine.

### Building tros.b

#### 1 Using Docker Image

All operations in this section are performed on the development machine.

<Tabs groupId="tros-distro">
<TabItem value="foxy" label="Foxy">

```shell
## Create directory
cd /mnt/data/kairui.wang/test
mkdir -p cc_ws/tros_ws/src
## Download Docker image for cross-compilation
wget http://archive.d-robotics.cc/TogetheROS/cross_compile_docker/pc_tros_v1.0.5.tar.gz
## Load Docker image
sudo docker load --input pc_tros_v1.0.5.tar.gz 
## Check the image ID corresponding to pc_tros
sudo docker images
## Start Docker container with mounted directories
sudo docker run -it --entrypoint="/bin/bash" -v LOCAL_PC_DIR:DOCKER_DIR IMAGE_ID  
# Example: sudo docker run -it --entrypoint="/bin/bash" -v /mnt/data/kairui.wang/test:/mnt/test 9c2ca340973e
```

</TabItem>
<TabItem value="humble" label="Humble">


```shell
## Create directory
cd /mnt/data/kairui.wang/test
mkdir -p cc_ws/tros_ws/src
## Download Docker image for cross-compilation
wget http://archive.d-robotics.cc/TogetheROS/cross_compile_docker/pc_tros_ubuntu22.04_v1.0.0.tar.gz
## Load Docker image
sudo docker load --input pc_tros_ubuntu22.04_v1.0.0.tar.gz 
## Check the image ID corresponding to pc_tros
sudo docker images
## Start Docker container with mounted directories
sudo docker run -it --entrypoint="/bin/bash" -v LOCAL_PC_DIR:DOCKER_DIR IMAGE_ID  
# Example: sudo docker run -it --entrypoint="/bin/bash" -v /mnt/data/kairui.wang/test:/mnt/test 4cbdb9d61e19
```

</TabItem>
</Tabs>


#### 2 Obtaining tros.b Source Code

All operations in this section are performed inside the Docker container on the development machine.

Here we use `/mnt/test` inside the Docker container as an example.

<Tabs groupId="tros-distro">
<TabItem value="foxy" label="Foxy">

```shell
cd /mnt/test/cc_ws/tros_ws
## Clone configuration files
git clone https://github.com/D-Robotics/robot_dev_config.git -b foxy 
## Navigate into robot_dev_config and run 'git tag --list' to view available release versions
## Use 'git reset --hard [TAG]' to specify a particular release version. See the section "Building a Specific Version of tros.b" on this page for details.
## Pull source code
vcs-import src < ./robot_dev_config/ros2_release.repos 
```

</TabItem>
<TabItem value="humble" label="Humble">


```shell
cd /mnt/test/cc_ws/tros_ws
## Clone configuration files
git clone https://github.com/D-Robotics/robot_dev_config.git -b develop 
## Navigate into robot_dev_config and run 'git tag --list' to view available release versions
## Use 'git reset --hard [TAG]' to specify a particular release version. See the section "Building a Specific Version of tros.b" on this page for details.
## Pull source code
vcs-import src < ./robot_dev_config/ros2_release.repos 
```

</TabItem>
</Tabs>

The overall project directory structure is as follows:

```text
├── cc_ws
│   ├── sysroot_docker
│   │   ├── etc
│   │   ├── lib -> usr/lib
│   │   ├── opt
│   │   └── usr
│   └── tros_ws
│       ├── robot_dev_config
│       └── src
```

- The `tros_ws/robot_dev_config` directory contains configuration files and scripts required for code fetching, building, and packaging.
- The `tros_ws/src` directory stores the fetched source code.
- The `sysroot_docker` directory contains headers and libraries required for cross-compilation, mirroring the root (`/`) directory of the RDK. For example, the media library resides at `sysroot_docker/usr/lib/hbmedia/` in the Docker environment, corresponding to `/usr/lib/hbmedia/` on the RDK.

During compilation, the build script `robot_dev_config/aarch64_toolchainfile.cmake` uses the `CMAKE_SYSROOT` macro to specify the path to `sysroot_docker`.

:::info
For the tag (version) of `robot_dev_config`, please refer to the [Release Notes](../01_quick_start/changelog.md) section.
:::

#### 3 Cross-compilation

All operations in this section are performed inside the Docker container on the development machine.

```shell
## Build tros.b for X3 using build.sh
bash ./robot_dev_config/build.sh -p X3

## Build tros.b for RDK Ultra using build.sh
bash ./robot_dev_config/build.sh -p Rdkultra

## Build tros.b for X5 using build.sh
bash ./robot_dev_config/build.sh -p X5

## Build tros.b for S100 using build.sh
bash ./robot_dev_config/build.sh -p S100
```

Upon successful compilation, you will see a message indicating that N packages have been built successfully.

If you perform a minimal build using `minimal_build.sh`, you can further reduce the deployment package size by running `./minimal_deploy.sh -d "install_path"`.

### Installing tros.b

Copy the generated `install` directory to the RDK and rename it to `tros`. Here, we place the deployment package under `/opt/tros` to maintain consistency with the directory used in deb package installations.

### Building a Specific Version of tros.b

By default, step 2 (**Obtaining tros.b Source Code**) in the **Building tros.b** section fetches the latest version of tros.b source code. If you need to obtain source code for a specific released version, modify this step as follows:

```bash
## Clone configuration files
git clone https://github.com/D-Robotics/robot_dev_config.git
cd robot_dev_config
## List available release versions
git tag --list
## Switch to a specific version; here we use tros.b 2.0.0 as an example
git reset --hard tros_2.0.0
cd ..
## Pull source code
vcs-import src < ./robot_dev_config/ros2_release.repos
```

:::info
For the tag (version) of `robot_dev_config`, please refer to the [Release Notes](../01_quick_start/changelog.md) section.
:::

## X86 Platform

### System Requirements

You must use a 64-bit Ubuntu 20.04 system. Alternatively, you may use the RDK cross-compilation Docker image, but both compilation and execution must be performed entirely within the Docker container.

### System Configuration

#### Setting Locale

Ensure your locale supports UTF-8:

```shell
locale  # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings
```

#### Adding APT Repository

```shell
# First, ensure Ubuntu Universe repository is enabled
sudo apt install software-properties-common
sudo add-apt-repository universe

sudo apt update && sudo apt install curl

# Add the official ROS2 repository
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Add the tros.b official repository
sudo curl -sSL http://archive.d-robotics.cc/keys/sunrise.gpg -o /usr/share/keyrings/sunrise.gpg
echo "deb [arch=amd64 signed-by=/usr/share/keyrings/sunrise.gpg] http://archive.d-robotics.cc/ubuntu-rdk-sim focal main" | sudo    tee /etc/apt/sources.list.d/sunrise.list > /dev/null
```

#### Install ROS Tool Packages

```shell
sudo apt update && sudo apt install -y \
  libbullet-dev \
  python3-pip \
  python3-pytest-cov \
  ros-dev-tools
```

### Obtain tros.b Source Code

```shell
git config --global credential.helper store

mkdir -p ~/cc_ws/tros_ws/src
cd ~/cc_ws/tros_ws/

git clone https://github.com/D-Robotics/robot_dev_config.git -b develop 
vcs-import src < ./robot_dev_config/ros2_release.repos
```

### Install Dependencies

Install packages required for building from source:

```shell
# install some pip packages needed for testing
python3 -m pip install -U \
  argcomplete \
  flake8-blind-except \
  flake8-builtins \
  flake8-class-newline \
  flake8-comprehensions \
  flake8-deprecated \
  flake8-docstrings \
  flake8-import-order \
  flake8-quotes \
  pytest-repeat \
  pytest-rerunfailures \
  pytest

# install Fast-RTPS dependencies
sudo apt install --no-install-recommends -y \
  libasio-dev \
  libtinyxml2-dev

# install Cyclone DDS dependencies
sudo apt install --no-install-recommends -y \
  libcunit1-dev

# install tros.b basic models
sudo apt install --no-install-recommends -y \
  hobot-models-basic

# install other packages dependencies
sudo apt install --no-install-recommends -y \
  qt5-qmake \
  libpyside2-dev \
  libshiboken2-dev \
  pyqt5-dev \
  python3-pyqt5 \
  python3-pyqt5.qtsvg \
  python3-pyside2.qtsvg \
  python3-sip-dev \
  shiboken2 \
  libyaml-dev \
  qtbase5-dev \
  libzstd-dev \
  libeigen3-dev \
  libxml2-utils \
  libtinyxml-dev \
  libssl-dev \
  python3-numpy \
  libconsole-bridge-dev \
  pydocstyle \
  libqt5core5a \
  libqt5gui5 \
  libgtest-dev \
  cppcheck \
  tango-icon-theme \
  libqt5opengl5 \
  libqt5widgets5 \
  python3-lark \
  libspdlog-dev \
  google-mock \
  clang-format \
  python3-flake8 \
  libbenchmark-dev \
  python3-pygraphviz \
  python3-pydot \
  python3-psutil \
  libfreetype6-dev \
  libx11-dev \
  libxaw7-dev \
  libxrandr-dev \
  libgl1-mesa-dev \
  libglu1-mesa-dev \
  python3-pytest-mock \
  python3-mypy \
  default-jdk \
  libcunit1-dev \
  libopencv-dev \
  python3-ifcfg \
  python3-matplotlib \
  graphviz \
  uncrustify \
  python3-lxml \
  libcppunit-dev \
  libcurl4-openssl-dev \
  python3-mock \
  python3-nose \
  libsqlite3-dev \
  pyflakes3 \
  clang-tidy \
  python3-lttng \
  liblog4cxx-dev \
  python3-babeltrace \
  python3-pycodestyle \
  libassimp-dev \
  libboost-dev \
  libboost-python-dev \
  python3-opencv \
  libboost-python1.71.0
```

### Build

```shell
# Build using build.sh
bash ./robot_dev_config/build.sh -p X86
```

Upon successful compilation, a message indicating that a total of N packages have been built successfully will be displayed.

### Install tros.b

Copy the generated `install` directory to `/opt` and rename it to `tros`, aligning with the directory structure used by deb package installations.

## Common Issues

**Q1:** How can I verify whether VCS successfully fetched the code?

**A1:** As shown in the figure below, during the `vcs import` process, a printed dot (`.`) indicates successful retrieval of a repository, whereas an `E` indicates failure. The specific failed repository can be identified from the log output after execution. In such cases, you may either delete the contents under the `src` directory and re-run `vcs import`, or manually clone the failed repository.

![vcs_import](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/01_quick_start/image/cross_compile/vcs_import_error.png)

**Q2:** What if I cannot pull code from GitHub due to network restrictions?

**A2:** You can directly download the required version of the source code from the [TogetheROS file server](http://archive.d-robotics.cc/TogetheROS/source_code/). For example, the file `tros_2.0.0_source_code.tar.gz` corresponds to tros.b version 2.0.0.