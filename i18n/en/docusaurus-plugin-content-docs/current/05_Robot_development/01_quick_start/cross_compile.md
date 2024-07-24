---
sidebar_position: 3
---
# 5.1.3 Source Code Installation

```mdx-code-block
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';
```

This section explains how to install TogetheROS.Bot on the Horizon RDK using source code.

## Horizon RDK Platform

Prerequisites:

- The development machine can access the Horizon Robotics organization on [GitHub](https://github.com/HorizonRDK).
- Docker is installed on the development machine.

### Compile

#### 1 Load docker image

All the following operations are performed within the Docker environment on the development machine.

<Tabs groupId="tros-distro">
<TabItem value="foxy" label="Foxy">

```shell
## Create a directory
cd /mnt/data/kairui.wang/test
mkdir -p cc_ws/tros_ws/src
## Obtain the Docker for cross-compilation
wget http://sunrise.horizon.cc/TogetheROS/cross_compile_docker/pc_tros_v1.0.5.tar.gz
## Load the Docker image
sudo docker load --input pc_tros_v1.0.5.tar.gz
## Check the corresponding image ID for pc_tros
sudo docker images
## Launch Docker and mount the directory
sudo docker run -it --entrypoint="/bin/bash" -v PC local directory: Docker directory imageID, here is an example using:
sudo docker run -it --entrypoint="/bin/bash" -v /mnt/data/kairui.wang/test:/mnt/test 9c2ca340973e
```

</TabItem>
<TabItem value="humble" label="Humble">

```shell
## Create a directory
cd /mnt/data/kairui.wang/test
mkdir -p cc_ws/tros_ws/src
## Obtain the Docker for cross-compilation
wget http://sunrise.horizon.cc/TogetheROS/cross_compile_docker/pc_tros_ubuntu22.04_v1.0.0.tar.gz
## Load the Docker image
sudo docker load --input pc_tros_ubuntu22.04_v1.0.0.tar.gz 
## Check the corresponding image ID for pc_tros
sudo docker images
## Launch Docker and mount the directory
sudo docker run -it --entrypoint="/bin/bash" -v PC local directory: Docker directory imageID, here is an example using:
sudo docker run -it --entrypoint="/bin/bash" -v /mnt/data/kairui.wang/test:/mnt/test 9c2ca340973e
```

</TabItem>
</Tabs>

#### 2 Obtain the Code

All the following operations are performed within the Docker environment on the development machine.

Here, we take the /mnt/test directory in Docker as an example.

<Tabs groupId="tros-distro">
<TabItem value="foxy" label="Foxy">

```shell
cd /mnt/test/cc_ws/tros_ws
## Obtain the configuration file
git clone https://github.com/HorizonRDK/robot_dev_config.git -b foxy
## Execute cd robot_dev_config and use the "git tag --list" command to view the available release versions
## Use the "git reset --hard [tag number]" command to specify the release version. For detailed instructions, refer to the "Compile Specific Version tros.b" section on this page
## Pull the source code
vcs-import src < ./robot_dev_config/ros2_release.repos
```

</TabItem>
<TabItem value="humble" label="Humble">


```shell
cd /mnt/test/cc_ws/tros_ws
## Obtain the configuration file
git clone https://github.com/HorizonRDK/robot_dev_config.git -b develop 
## Execute cd robot_dev_config and use the "git tag --list" command to view the available release versions
## Use the "git reset --hard [tag number]" command to specify the release version. For detailed instructions, refer to the "Compile Specific Version tros.b" section on this page
## Pull the source code
vcs-import src < ./robot_dev_config/ros2_release.repos 
```

</TabItem>
</Tabs>

The directory structure of the entire project is as follows:

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

The `tros_ws/robot_dev_config` path contains the configuration and script files needed for code fetching, compilation, and packaging. The `tros_ws/src` path stores the fetched code. The `sysroot_docker` path contains the header files and libraries required for cross-compilation, corresponding to the `/` directory of the Horizon RDK. For example, the path for the media library in `sysroot_docker` is `sysroot_docker/usr/lib/hbmedia/`, while the path in the Horizon RDK is `/usr/lib/hbmedia/`.

During compilation, the installation path of `sysroot_docker` is specified through the `CMAKE_SYSROOT` macro in the `robot_dev_config/aarch64_toolchainfile.cmake` compilation script.

#### 3 Cross-Compilation

All of these operations are performed inside the docker on the development machine.

```shell
## Compile tros.b version X3 using build.sh
bash ./robot_dev_config/build.sh -p X3
```

After successful compilation, a message will prompt: N packages compiled and passed.

If using minimal_build.sh for minimal compilation, you can further compress the deployment package size by executing `./minimal_deploy.sh -d "install_path"`.

### Install

Copy the compiled directory to the Horizon RDK and rename it as tros. Here, we place the deployment package in the /opt/tros directory to be consistent with the deb installation directory.

### Compile a specific version

In the section **Compile**, in the step 2 **Obtain the Code**, the default is to fetch the latest version of tros.b source code. If you need to get a specific release version of the source code, you need to make the following modifications:

```bash
## Get the configuration file
git clone https://github.com/HorizonRDK/robot_dev_config.git -b develop 
cd robot_dev_config
## View available release versions
git tag --list
## Switch to the specified version number, here we take tros.b 2.0.0 as an example
git reset --hard tros_2.0.0
cd ..
## Pull code
vcs-import src < ./robot_dev_config/ros2_release.repos
```

## FAQ

Q1: How to determine if VCS successfully pulled the code?

A1: As shown in the image below, during the vcs import process, a "." indicates a successful repo pull, and an "E" indicates a failed repo pull. Specific failed repos can be seen in the log after execution. If this happens, you can try deleting the contents in the src directory and re-run vcs import or manually pull the failed repos.

![vcs_import](./image/cross_compile/vcs_import_error.png "vcs_import")

Q2: Limited conditions prevent code retrieval from GitHub

A2: You can directly download the desired version of the code from the [TogetheROS File Server](http://sunrise.horizon.cc/TogetheROS/source_code/). For example, the `tros_2.0.0_source_code.tar.gz` file corresponds to version 2.0.0 of tros.b.