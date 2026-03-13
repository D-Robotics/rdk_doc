---
sidebar_position: 1
---
# Environment Installation

This chapter mainly introduces the complete deployment method of the D-Robotics algorithm toolchain development environment.

## RDK-X3

### Instructions for Using Deliverables {#deliverables_instructions}

Before deploying the algorithm toolchain environment, please download the **Embedded Application Development Sample Delivery Package** provided by D-Robotics to the Linux development machine environment.

Unpack the SDK source code package of the algorithm toolchain:

```bash
// The sample version is v1.0.0

wget -c ftp://oeftp@sdk.d-robotics.cc/RDK/rdk-x3-ultra/Ai_Toolchain_Package-release-v1.1.57-OE-v1.0.0.tar.xz --ftp-password=Oeftp~123$%

// More model conversion examples, available for download as needed!
//wget -c ftp://oeftp@sdk.d-robotics.cc/RDK/rdk-x3-ultra/horizon_model_convert_sample.tar.gz --ftp-password=Oeftp~123$%
tar -xvf Ai_Toolchain_Package-release-vX.X.X-OE-vX.X.X.tar.xz
```


#### Description of the Directory Structure of the Sample Package Source Code

Extract the algorithm toolchain SDK source code package:

```bash
  // The sample version is v1.0.0

  tar -xvf Ai_Toolchain_Package-release-v1.1.57-OE-v1.0.0.tar.xz
```

The directory structure after unpacking is as follows:

-   **ai_benchmark**: This directory provides evaluation examples for common classification, detection, and segmentation models, including performance evaluation and accuracy evaluation.

-   **horizon_runtime_sample**: This directory provides on-board examples for fixed-point models.

-   **package**: This directory contains some basic libraries and components for running the released artifacts.

    1. The ``board`` folder contains the executable program at the board side.

    2. The ``host`` folder contains the environment dependencies, tool dependencies, and libdnn libraries and headers related to model inference in the x86 development environment.



### Development Machine Deployment {#machine_deploy}

For the environment deployment of the development machine, D-Robotics supports Docker deployment.

#### Development Machine Preparation

In order to use the algorithm toolchain smoothly, D-Robotics recommends that your selected development machine should meet the following requirements:

  | Hardware/Operating System | Requirements                                 |
  |---------------|------|
  | CPU           | CPU I3 or higher or the equivalent E3/E5 processor    |
  | Memory          | 16GB or higher                        |
  | GPU(optional)     | CUDA11.6, driver version Linux:>= 510.39.01*<br/>Compatible graphics cards include but are not limited to:<br/>1) GeForce RTX 3090<br/>2) GeForce RTX 2080 Ti<br/>3) NVIDIA TITAN V<br/>4) Tesla V100S-PCIE-32GB
  | System          | Ubuntu 20.04         |For more information about compatibility issues between CUDA and graphics cards, please refer to the [**NVIDIA official website**](https://docs.nvidia.com/deploy/cuda-compatibility/).

#### Using Docker Environment

To help you quickly use the algorithm toolchain, D-Robotics provides Docker images with complete development environments, which greatly simplifies the deployment process.

Before reading this section, we hope that Docker's basic environment has been preinstalled on your development machine. The Docker basic environment information required by D-Robotics is as follows:

- Docker (>=1.12, recommended version 18.03.0-ce), installation guide: https://docs.docker.com/install/
- NVIDIA Docker (2.0.3), installation guide [**NVIDIA/nvidia-docker**](https://github.com/nvidia/nvidia-docker/wiki).

After completing the Docker environment installation, you need to add the non-root user to the Docker user group. Refer to the following command:

```bash
  sudo groupadd docker
  sudo gpasswd -a ${USER} docker
  sudo systemctl restart docker  # CentOS7/Ubuntu
  # re-login
```

The address of the Docker image needed in this section is as follows:

- [**D-Robotics Docker Hub GPU Docker**](https://hub.docker.com/r/openexplorer/ai_toolchain_ubuntu_20_x3j5_gpu)

The naming format of the image file is:

- GPU version docker: `openexplorer/ai_toolchain_ubuntu_20_x3j5_gpu:{version}`


:::tip Tip

  When executing the command, replace `{version}` with **the latest version of the Docker image** you obtained, for example: [**D-Robotics docker hub GPU Docker**](https://hub.docker.com/r/openexplorer/ai_toolchain_ubuntu_20_x3j5_gpu) The current latest version is `openexplorer/ai_toolchain_ubuntu_20_x3j5_gpu:v1.0.0`.

  For the local Docker image package version, you can contact D-Robotics Technical Support Team to obtain it.

  A development machine does not necessarily have a GPU card. Generally, a CPU development machine is used to load Docker images for model conversion!
:::

Before using each image file for the first time, you need to pull the image.

- The command to pull the image is:

  ```bash
    docker pull openexplorer/ai_toolchain_ubuntu_20_x3j5_gpu:v1.0.0
  ```

Then run the Docker container by executing the following command.

- For CPU development machine Docker container, execute the following command:- Execute the following command in the Docker container on the GPU development machine:

  ```bash
    // Run the Docker image command

    export version=v1.0.0

    export ai_toolchain_package_path=/home/users/xxx/ai_toolchain_package

    export dataset_path=/home/users/xxx/data/

    docker run -it --runtime=nvidia -e NVIDIA_DRIVER_CAPABILITIES=compute,utility \
      -e NVIDIA_VISIBLE_DEVICES=all --rm --shm-size="15g" \
      -v "$ai_toolchain_package_path":/open_explorer \
      -v "$dataset_path":/data \
      openexplorer/ai_toolchain_ubuntu_20_x3j5_gpu:"${version}"
  ```

  

- On the GPU development machine, execute the following command within a Docker container:

    ```bash
    # Command to run the docker image

    export version=v1.0.0
    export ai_toolchain_package_path=/home/users/xxx/ai_toolchain_package
    export dataset_path=/home/users/xxx/data/

    docker run -it --runtime=nvidia \
        -e NVIDIA_DRIVER_CAPABILITIES=compute,utility \
        -e NVIDIA_VISIBLE_DEVICES=all \
        --rm \
        --shm-size="15g" \
        -v "$ai_toolchain_package_path":/open_explorer \
        -v "$dataset_path":/data \
        openexplorer/ai_toolchain_ubuntu_20_x3j5_gpu:"${version}"
    ```



:::info Note

  When executing the above command:
  
  - ``dataset_path`` is the directory of the dataset files. If the directory does not exist, it will cause loading issues. Please create the directory before running the command.
  
  - Public datasets can be downloaded from the following links:
  
      VOC: http://host.robots.ox.ac.uk/pascal/VOC/ (Use the VOC2012 version)
  
      COCO: https://cocodataset.org/#download
  
      ImageNet: https://www.image-net.org/download.php
  
      Cityscapes: https://github.com/mcordts/cityscapesScripts
  
      CIFAR-10: http://www.cs.toronto.edu/~kriz/cifar.htmlFlyingChairs: https://lmb.informatik.uni-freiburg.de/resources/datasets/FlyingChairs.en.html

KITTI3D: https://www.cvlibs.net/datasets/kitti/eval_object.php?obj_benchmark=3d

CULane: https://xingangpan.github.io/projects/CULane.html

nuScenes: https://www.nuscenes.org/nuscenes
:::
You have successfully entered the complete algorithm toolchain development environment through the Docker image. You can type `hb_mapper --help` command to verify if you can get help information:

```bash
  [root@d67382e74eea open_explorer]# hb_mapper --help
  Usage: hb_mapper [OPTIONS] COMMAND [ARGS]...

    hb_mapper is an offline model transform tool provided by horizon.

  Options:
    --version  Show the version and exit.
    --help     Show this message and exit.

  Commands:
    checker    check whether the model meet the requirements.
    infer      inference and dump output feature as float vector.
    makertbin  transform caffe model to quantization model, generate runtime...
```

If the hb_mapper tool has output logs properly, it means that the environment has been successfully installed and deployed. Please go to the **Board Deployment** section for board environment installation.


### Board Deployment

Board deployment requires you to update the board image to the latest version according to the flashing instructions. Please refer to the [**Install OS**](../../../01_Quick_start/install_os/rdk_x3.md) section for the upgrade method. After the upgrade is completed, copy the relevant supplementary files to the development board.

#### Preparation of Supplementary Files

Some supplementary tools of the algorithm toolchain are not included in the system image. These tools have been placed in the ``Ai_Toolchain_Package-release-vX.X.X-OE-vX.X.X/package/`` package. 
Enter the ``Ai_Toolchain_Package-release-vX.X.X-OE-vX.X.X/package/board`` directory and execute the installation script.
The command is as follows:

```bash
  // For RDK X3 development board, execute the command
  bash install_xj3.sh ${board_ip}

  // For RDK Ultra development board, execute the command
  bash install_ultra.sh ${board_ip}
```
:::info Note

``${board_ip}`` is the IP address you have set for the development board. Please ensure that the IP is accessible from the development machine.
After successful installation, restart the development board and execute ``hrt_model_exec`` on the development board to verify the successful installation.
:::

## RDK-X5
### <span id="ai_toolchain_package"/>Instructions for Using Deliverables

:::tip

This section only demonstrates how to use the algorithm toolchain. The latest version of the toolchain installation package and offline documentation can be obtained by visiting [**"Digu X5 Algorithm Toolchain Release Notes"**](https://developer.d-robotics.cc/forumDetail/251934919646096384).

:::

Before deploying the algorithm toolchain environment, please download D-Robotics' **Embedded Application Development Sample Deliverables Package** to your Linux development host.

Download the Embedded Application Development Sample Deliverables Package:

```bash
  // Example version: V1.2.6

  wget -c ftp://oeftp@sunrise.horizon.cc:10021/runtime_package/Ai_Toolchain_Package-release-v1.23.8-OE-v1.2.6.tar.xz --ftp-password=Oeftp~123$%

  // More model conversion examples—download as needed!
  //wget -c ftp://oeftp@sunrise.horizon.cc:10021/model_convert_sample/horizon_model_convert_sample.tar.xz --ftp-password=Oeftp~123$%
```

#### Description of Sample Package Source Code Directory Structure

Extract the algorithm toolchain SDK source code package:

```bash
  // Example version: V1.2.6

  tar -xvf Ai_Toolchain_Package-release-v1.23.8-OE-v1.2.6.tar.xz
```

The extracted directory structure is as follows:

-   **ai_benchmark**: Contains evaluation examples for common classification, detection, and segmentation models, including both performance and accuracy evaluations.

-   **horizon_runtime_sample**: Provides on-board examples for fixed-point models.

-   **package**: Contains basic libraries and components required for running the deliverables.

    1. The ``board`` folder contains executable programs for the target board.

    2. The ``host`` folder contains environment dependencies, tool dependencies, and libdnn libraries and header files related to model inference for the x86 development environment.


### Host Setup

For host environment setup, the X5 algorithm toolchain supports deployment via Docker.

#### Host Requirements

To ensure smooth usage of the algorithm toolchain, the X5 algorithm toolchain recommends that your development host meets the following requirements:

  | Hardware / OS | Requirement                                 |
  |---------------|---------------------------------------------|
  | CPU           | Intel Core i3 or higher, or equivalent E3/E5 processor |
  | Memory        | 16 GB or more                               |
  | GPU (Optional)| CUDA 11.6, driver version Linux: >= 510.39.01*<br/>Compatible GPUs include but are not limited to:<br/>1) GeForce RTX 3090<br/>2) GeForce RTX 2080 Ti<br/>3) NVIDIA TITAN V<br/>4) Tesla V100S-PCIE-32GB<br/>5) A100 |
  | OS            | Ubuntu 20.04                                |

For more information on CUDA and GPU compatibility, please refer to [**NVIDIA Official Documentation**](https://docs.nvidia.com/deploy/cuda-compatibility/).

#### Using the Docker Environment

To help you quickly get started with the algorithm toolchain, the X5 algorithm toolchain provides a Docker image containing a complete development environment, greatly simplifying deployment.

Before proceeding, we assume Docker is already installed on your development host.  
The required Docker base environment for the X5 algorithm toolchain is as follows:

- Docker (version 19.03 or higher; version 19.03 is recommended). See [**Docker Installation Guide**](https://docs.docker.com/install/).
- NVIDIA Container Toolkit (versions 1.13.1–1.13.5; version 1.13.5 is recommended). See [**NVIDIA Container Toolkit Installation Guide**](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html).

After installing the Docker environment, add your non-root user to the Docker group. Use the following commands as reference:

```bash
  sudo groupadd docker
  sudo gpasswd -a ${USER} docker
  sudo service docker restart
```

The Docker image addresses required for this section are as follows:

- [**X5 Algorithm Toolchain Docker Hub CPU Image**](https://hub.docker.com/r/openexplorer/ai_toolchain_ubuntu_20_x5_cpu)
- [**X5 Algorithm Toolchain Docker Hub GPU Image**](https://hub.docker.com/r/openexplorer/ai_toolchain_ubuntu_20_x5_gpu)

Image naming conventions:

- CPU version: ``openexplorer/ai_toolchain_ubuntu_20_x5_cpu:{version}``
- GPU version: ``openexplorer/ai_toolchain_ubuntu_20_x5_gpu:{version}``


**Tip:**
>1. Replace ``{version}`` in commands with the **version information** found in the **``ai_toolchain_package``** directory within the **X5 SDK deliverables package**.
>2. For local Docker image versions, please contact the technical support team.
>3. A GPU is not mandatory on the development host—typically, a CPU-based host can load the Docker image to perform model conversion!

Each image must be pulled before its first use.

- Pull command example:

  ```bash
    docker pull openexplorer/ai_toolchain_ubuntu_20_x5_cpu:v1.2.6
  ```

Then run the following command to start the Docker container.

- For CPU-based development hosts:

  ```bash
    // Command to run the Docker image

    export version=v1.2.6

    export ai_toolchain_package_path=/home/users/xxx/ai_toolchain_package

    export dataset_path=/home/users/xxx/data/

    docker run -it --rm \
      -v "$ai_toolchain_package_path":/open_explorer \
      -v "$dataset_path":/data \
      openexplorer/ai_toolchain_ubuntu_20_x5_cpu:"${version}"
  ```
- For GPU-based development hosts:

  ```bash
    // Command to run the Docker image

    export version=v1.2.6

    export ai_toolchain_package_path=/home/users/xxx/ai_toolchain_package

    export dataset_path=/home/users/xxx/data/

    docker run -it --rm \
      --gpus all \ # Add this flag when starting the container to enable GPU access
      --shm-size=15g \ # Adjust shared memory size
      -v "$ai_toolchain_package_path":/open_explorer \
      -v "$dataset_path":/data \
      openexplorer/ai_toolchain_ubuntu_20_x5_gpu:"${version}"
  ```

**Note:**

- ``dataset_path`` is the directory for your dataset files. If this directory does not exist, the mount will fail—please create it before running the command.

- Public datasets can be downloaded from the following links:

  [**VOC (use VOC2012):**](http://host.robots.ox.ac.uk/pascal/VOC/)

  [**COCO:**](https://cocodataset.org/#download)

  [**ImageNet:**](https://www.image-net.org/download.php)

  [**Cityscapes:**](https://github.com/mcordts/cityscapesScripts)

  [**CIFAR-10:**](http://www.cs.toronto.edu/~kriz/cifar.html)

  [**FlyingChairs:**](https://lmb.informatik.uni-freiburg.de/resources/datasets/FlyingChairs.en.html)

  [**KITTI3D:**](https://www.cvlibs.net/datasets/kitti/eval_object.php?obj_benchmark=3d)

  [**CULane:**](https://xingangpan.github.io/projects/CULane.html)

  [**nuScenes:**](https://www.nuscenes.org/nuscenes)


At this point, you have successfully entered a complete algorithm toolchain development environment via the Docker image.  
You can type the command ``hb_mapper --help`` to verify that help information is displayed correctly:

```bash
  [root@d67382e74eea open_explorer]# hb_mapper --help
  Usage: hb_mapper [OPTIONS] COMMAND [ARGS]...

    hb_mapper is an offline model transform tool provided by horizon.

  Options:
    --version  Show the version and exit.
    --help     Show this message and exit.

  Commands:
    checker    check whether the model meet the requirements.
    infer      inference and dump output feature as float vector.
    makertbin  transform caffe model to quantization model, generate runtime...
```
If the `hb_mapper` tool has successfully output logs, it indicates that the environment has been properly installed and deployed. Please proceed to the [**Board Deployment**](./environment_config.html#span-id-board-env-install) section to install the environment on the development board.

### <span id="board_env_install"/>Board Deployment

For board deployment, you need to follow the flashing instructions to update the development board image to the latest version. For upgrade procedures, please refer to the [**Install System**](../../../01_Quick_start/install_os/rdk_x5.md) section. After completing the upgrade, simply copy the relevant supplementary files onto the development board.

Some supplementary tools of the algorithm toolchain are not included in the system image. These tools are already provided in the ``Ai_Toolchain_Package-release-vX.X.X-OE-vX.X.X/package/`` installation package.  
Navigate to ``Ai_Toolchain_Package-release-vX.X.X-OE-vX.X.X/package/board`` and execute the install script.  
Run the following command:

```bash
  bash install.sh ${board_ip}
```

**Note:**
- ``${board_ip}`` is the IP address you assigned to your development board. Ensure this IP is accessible from your development host.
- After successful installation, reboot the development board and run ``hrt_model_exec`` on the board to verify whether the installation succeeded.

## RDK-Ultra

### Instructions for Using Deliverables {#deliverables_instructions}

Before deploying the algorithm toolchain environment, please download D-Robotics' **Embedded Application Development Sample Deliverables Package** to your Linux development host environment.

Download the Embedded Application Development Sample Deliverables Package:
ftp://oemng@42.62.85.52:10021/RDK/rdk-x3-ultra/
```bash
  // Example version: V1.0.0

  wget -c ftp://oeftp@sunrise.horizon.cc:10021/RDK/rdk-x3-ultra/Ai_Toolchain_Package-release-v1.1.57-OE-v1.0.0.tar.xz --ftp-password=Oeftp~123$%

  // Additional model conversion examples—download as needed!
  //wget -c ftp://oeftp@sunrise.horizon.cc:10021/RDK/rdk-x3-ultra/horizon_model_convert_sample.tar.gz --ftp-password=Oeftp~123$%
```

#### Description of Sample Package Source Directory Structure

Extract the algorithm toolchain SDK source package:

```bash
  // Example version: V1.0.0

  tar -xvf Ai_Toolchain_Package-release-v1.1.57-OE-v1.0.0.tar.xz
```

The extracted directory structure is as follows:

- **ai_benchmark**: Contains evaluation examples for common classification, detection, and segmentation models, including both performance and accuracy evaluations.

- **horizon_runtime_sample**: Provides on-board examples for fixed-point models.

- **package**: Contains basic libraries and components required for running deliverables.

    1. The ``board`` folder contains executable programs for the board.
    2. The ``host`` folder includes environment dependencies, tool dependencies, and libdnn libraries and header files related to model inference for the x86 development environment.

### Host Deployment {#machine_deploy}

For host environment deployment, D-Robotics supports deployment via Docker.

#### Host Preparation

To ensure smooth usage of the algorithm toolchain, D-Robotics recommends that your development host meet the following requirements:

| Hardware / OS | Requirements |
|---------------|--------------|
| CPU           | Intel Core i3 or equivalent (e.g., Xeon E3/E5 series) |
| Memory        | 16 GB or higher |
| GPU (Optional)| CUDA 11.6, Driver version Linux: >= 510.39.01*<br/>Compatible GPUs include but are not limited to:<br/>1) GeForce RTX 3090<br/>2) GeForce RTX 2080 Ti<br/>3) NVIDIA TITAN V<br/>4) Tesla V100S-PCIE-32GB |
| OS            | Ubuntu 20.04 |

For more details regarding CUDA and GPU compatibility, please refer to [**NVIDIA Official Documentation**](https://docs.nvidia.com/deploy/cuda-compatibility/).

#### Using the Docker Environment

To help you quickly get started with the algorithm toolchain, D-Robotics provides a Docker image containing a complete development environment, significantly simplifying the deployment process.

Before proceeding with this section, please ensure Docker is already installed on your development host.  
D-Robotics requires the following Docker base environment:

- Docker (version 19.03 or higher; version 19.03 is recommended). See [**Docker Installation Guide**](https://docs.docker.com/install/).
- NVIDIA Container Toolkit (versions 1.13.1–1.13.5; version 1.13.5 is recommended). See [**NVIDIA Container Toolkit Installation Guide**](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html).

After installing the Docker environment, add your non-root user to the Docker group using the following commands:

```bash
  sudo groupadd docker
  sudo gpasswd -a ${USER} docker
  sudo service docker restart
```

The Docker image required for this section can be found at the following address:

- [**D-Robotics Docker Hub GPU Docker**](https://hub.docker.com/r/openexplorer/ai_toolchain_ubuntu_20_x3j5_gpu)

Image naming convention:

- GPU version Docker image: ``openexplorer/ai_toolchain_ubuntu_20_x3j5_gpu:{version}``

:::tip Tip

When executing commands, replace ``{version}`` with the **latest Docker image version** you obtained. For example, the current latest version listed on [**D-Robotics Docker Hub GPU Docker**](https://hub.docker.com/r/openexplorer/ai_toolchain_ubuntu_20_x3j5_gpu) is ``openexplorer/ai_toolchain_ubuntu_20_x3j5_gpu:v1.0.0``.

For local Docker image packages, please contact the D-Robotics technical support team.

Note: Your development host does not necessarily require a GPU. In most cases, you can perform model conversion using a CPU-only host by loading the Docker image!
:::

Each Docker image must be pulled before its first use.

- Pull the image using the following command:

  ```bash
    docker pull openexplorer/ai_toolchain_ubuntu_20_x3j5_gpu:v1.0.0
  ```

Then run the Docker container using the commands below.

- For CPU-only development hosts, run the following command:

  ```bash
    // Command to run the Docker image

    export version=v1.0.0

    export ai_toolchain_package_path=/home/users/xxx/ai_toolchain_package

    export dataset_path=/home/users/xxx/data/

    docker run -it --rm \
      -v "$ai_toolchain_package_path":/open_explorer \
      -v "$dataset_path":/data \
      openexplorer/ai_toolchain_ubuntu_20_x3j5_gpu:"${version}"
  ```

- For GPU-enabled development hosts, run the following command:

  ```bash
    // Command to run the Docker image

    export version=v1.0.0

    export ai_toolchain_package_path=/home/users/xxx/ai_toolchain_package

    export dataset_path=/home/users/xxx/data/

    docker run -it --rm \
      --gpus all \ # Add this flag when launching the container to enable GPU access
      --shm-size=15g \ # Adjust shared memory size
      -v "$ai_toolchain_package_path":/open_explorer \
      -v "$dataset_path":/data \
      openexplorer/ai_toolchain_ubuntu_20_x3j5_gpu:"${version}"
  ```

:::info Note

When executing the above commands:

- ``dataset_path`` refers to your dataset directory. If this directory does not exist, an error will occur—please create it before running the command.

- Public datasets can be downloaded from the following links:

    VOC: http://host.robots.ox.ac.uk/pascal/VOC/ (use VOC2012)

    COCO: https://cocodataset.org/#download

    ImageNet: https://www.image-net.org/download.php

    Cityscapes: https://github.com/mcordts/cityscapesScripts

    CIFAR-10: http://www.cs.toronto.edu/~kriz/cifar.html

    FlyingChairs: https://lmb.informatik.uni-freiburg.de/resources/datasets/FlyingChairs.en.html

    KITTI3D: https://www.cvlibs.net/datasets/kitti/eval_object.php?obj_benchmark=3d

    CULane: https://xingangpan.github.io/projects/CULane.html

    nuScenes: https://www.nuscenes.org/nuscenes
:::

At this point, you have successfully entered the complete algorithm toolchain development environment via the Docker image.  
You can type the command ``hb_mapper --help`` to verify that help information is displayed correctly:

```bash
  [root@d67382e74eea open_explorer]# hb_mapper --help
  Usage: hb_mapper [OPTIONS] COMMAND [ARGS]...

    hb_mapper is an offline model transform tool provided by horizon.

  Options:
    --version  Show the version and exit.
    --help     Show this message and exit.

  Commands:
    checker    check whether the model meet the requirements.
    infer      inference and dump output feature as float vector.
    makertbin  transform caffe model to quantization model, generate runtime...
```
If the hb_mapper tool has successfully output logs, it indicates that the environment has been properly installed and deployed. Please proceed to the **Development Board Deployment** section to install the environment on the development board.


### Development Board Deployment


For development board deployment, you need to follow the flashing instructions to update the development board image to the latest version. For the upgrade procedure, please refer to the [**Install OS**](../../../01_Quick_start/install_os/rdk_ultra.md) section. After completing the upgrade, simply copy the relevant supplementary files onto the development board.


#### Preparation of Supplementary Files

Some supplementary tools from the algorithm toolchain are not included in the system image. These tools are already provided in the ``Ai_Toolchain_Package-release-vX.X.X-OE-vX.X.X/package/`` installation package.  
Navigate to ``Ai_Toolchain_Package-release-vX.X.X-OE-vX.X.X/package/board`` and execute the install script.  
Run the following command:

```bash
  // If using the RDK Ultra development board, run:
  bash install_ultra.sh ${board_ip}
```
:::info Note

  Here, ``${board_ip}`` is the IP address you assigned to the development board. Ensure this IP is accessible from your development host.
  After successful installation, reboot the development board and run ``hrt_model_exec`` on the board to verify whether the installation succeeded.
:::





## Version Management Tool Usage Guide

This section introduces the usage of the ddk_vcs version management tool, helping developers understand the versions of algorithm toolchain dependency packages in their current development host environment.

## Instruction for Version Control Tool

This chapter mainly introduces the usage instructions for the ddk_vcs version control tool, in order to help developers understand the current version status of the algorithm toolchain dependencies in the development machine environment.

:::tip Tip
The version control tool is mainly used for debugging when errors occur during model PTQ conversion using the Docker environment. If the model conversion function is normal, you can skip reading this chapter.
:::

The version control tool has the following functionalities:

- ddk_vcs list;
- ddk_vcs install; 
- ddk_vcs uninstall; 
- ddk_vcs patch; 
- ddk_vcs show; 


### ddk_vcs list

ddk_vcs list is used to list the installed software packages.

When executing this command without any parameters, the result will display the information of the installed modules. The usage example is as follows:

```bash

  [horizon@gpu-dev067 ai_toolchain]$ ddk_vcs list
  Host package version: v2.0.3
  The following packages versions
  Platform        Package         Version MD5
  --------------- --------------- ------- -------------
  aarch_64        appsdk          032419  093e13b44e
  aarch_64        dnn             1.8.1g  aff0f6f4de
  x86_64_gcc5.4.0 dnn_x86         1.8.1g  e8e6bf9ed5
  x86             horizon-nn      0.13.3  origin:0.13.3
  x86             horizon-nn-gpu  0.13.3  origin:N/A
  x86             horizon-tc-ui   1.6.4   origin:1.6.4
  x86             hbdk            3.28.3  origin:3.28.3
```
:::info Note
 The origin information in the last few lines will be updated to the current version in the environment after each installation using the install script in the toolchain SDK package. It will not change when using ddk_vcs for installation, only the value of Version will change.
:::

When using the ``-p`` parameter, it will display the available module versions that can be installed. You can install them using ``ddk_vcs install``. The usage example is as follows:

```bash[horizon@gpu-dev004]$ ddk_vcs list -p
Host package version: 1.5.1
The following packages versions
Platform        Local Package                 Version MD5
--------------- ----------- ------- ----------
aarch_64        appsdk_1.9.0.tar.gz           1.9.0   bf01140c9d
aarch_64        bpu_predict_1.10.2.tar.gz     1.10.2  5b6e5dd6c5
aarch_64        dnn_1.1.2a.tar.gz             1.1.2a  fdb5729f4f
x86_64_gcc5.4.0 bpu_predict_1.10.2.tar.gz     1.10.2  4dbdd980a7
x86_64_gcc5.4.0 dnn_x86_1.1.2a.tar.gz         1.1.2a  5bf5fcd4fe
```

### ddk_vcs install

ddk_vcs install is used to install packages.
Users can install the corresponding module tar package directly using "ddk_vcs install". The specified platform needs to be specified during installation. The usage is as follows:

```bash

  [horizon@gpu-dev004]$ ddk_vcs install bpu_predict_1.10.2.tar.gz -p aarch_64
  bpu_predict installed successfully, version: 1.10.2, platform: aarch_64
  [horizon@gpu-dev067 ai_toolchain]$ ddk_vcs install hbdk-3.28.3-py3-none-linux_x86_64.whl  horizon_nn-0.13.3-py3-none-any.whl
  hbdk-3.28.3-py3-none-linux_x86_64.whl installed successfully
  horizon_nn-0.13.3-py3-none-any.whl installed successfully
```
After using "ddk_vcs list -p", users can obtain the version information of each module package in their current host package.
Then, "ddk_vcs install" can be used to conveniently switch between different versions. The usage is as follows:

```bash

  [horizon@gpu-dev004]$ ddk_vcs install bpu_predict==1.7.2  --platform aarch_64
  bpu_predict installed successfully, version: 1.7.2, platform: aarch_64
```
If the corresponding version is not available locally, you can specify the installation package location for installation.

### ddk_vcs uninstall

ddk_vcs uninstall is used to uninstall a specific module. The usage is as follows:

```bash

  [horizon@gpu-dev004]$ ddk_vcs uninstall bpu_predict --platform aarch_64
  Start to uninstall modules, platform: aarch_64
  bpu_predict uninstalled successfully, version: 1.10.2, platform: aarch_64
```
### ddk_vcs patch


"ddk_vcs patch ddk_patch.tar.gz" is used to install pre-made patch packages. The usage is as follows:

```bash

  [horizon@gpu-dev004]$ ddk_vcs patch ddk_patch.tar.gz
  bpu_predict installed successfully, version: 1.7.2_patch0, platform: aarch64
```
### ddk_vcs show


ddk_vcs show is used to display information about the installed packages. Use ``ddk_vcs show [module name]`` to display the information of the corresponding module. The usage is as follows:

```bash

  [horizon@gpu-dev004]$ ddk_vcs show bpu_predict
  Host package version 1.5.1
  The following packages versions
  Platform        Package     Version       MD5
  --------------- ----------- ------------- ----------
  aarch_64        bpu_predict 1.10.2        5b6e5dd6c5
  x86_64_gcc5.4.0 bpu_predict 1.10.2_patch1 d4f8e37921
```
If there are dependencies with the same name in two architectures, the ``-p/--platform`` option can be used to filter by architecture name. The usage is as follows:

```bash

  [horizon@gpu-dev004]$ ddk_vcs show bpu_predict -p aarch_64
  Host package version 1.5.1
  The following packages versions
  Platform Package     Version MD5
  -------- ----------- ------- ----------
  aarch_64 bpu_predict 1.10.2  5b6e5dd6c5
```
