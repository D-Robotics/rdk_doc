---
sidebar_position: 1
---

# 4.3.1 ModelZoo Overview

## Product Introduction

This product serves as the model sample repository (Model Zoo) for the RDK series development boards, aiming to provide developers with a variety of model cases that can be directly deployed on the board.

:::tip Tip

The GitHub repository for the Model Zoo is available here: https://github.com/D-Robotics/rdk_model_zoo
:::

Through this repository, developers can access the following resources:

1. **Diverse Sweet Potato Heterogeneous Models**: The repository includes a variety of .bin models suitable for direct deployment on the board, applicable to multiple scenarios with broad versatility. These models span various fields, including but not limited to image classification, object detection, semantic segmentation, and natural language processing. Each model has been carefully selected and optimized for efficient performance.
2. **Comprehensive Usage Guides**: Each model is accompanied by a Jupyter Notebook with a detailed model overview, usage instructions, example code, and annotations to help developers get started quickly. For certain models, we also provide performance evaluation reports and tuning recommendations to assist developers in customizing and optimizing the models according to specific needs.
3. **Integrated Development Tools**: We provide developers with a Python interface, `bpu_infer_lib`, for rapid model deployment on the RDK series development boards. By studying the Jupyter Notebooks included with the models, such as data preprocessing scripts and inference methods, developers can quickly master the use of this interface, significantly streamlining the model development and deployment process.

## Environment Preparation

Developers should first prepare an RDK development board corresponding to their branch and go to the D-Robotics official website to complete [hardware preparation, driver installation, software download, and image burning](https://developer.d-robotics.cc/rdk_doc/en/install_os). For X3 and X5 images, please choose versions above 3.0.0.

After completing the hardware connection and network configuration, use MobaXTerm to [remotely log in to the development board](https://developer.d-robotics.cc/rdk_doc/Quick_start/remote_login). Connect the development board to the network [here](https://developer.d-robotics.cc/rdk_doc/en/System_configuration/network_blueteeth)。

Use pip to install the corresponding Python libraries:

1. bpu_infer_lib

If using RDK X5:
```
pip install bpu_infer_lib_x5 -i http://archive.d-robotics.cc/simple/ --trusted-host archive.d-robotics.cc
```

If using RDK X3:
```
pip install bpu_infer_lib_x3 -i http://archive.d-robotics.cc/simple/ --trusted-host archive.d-robotics.cc
```

2. jupyterlab
```
pip install jupyterlab
```

Then you can pull the Model Zoo repository with the following command:
```
git clone https://github.com/D-Robotics/rdk_model_zoo
```

Note: The branch cloned by git clone defaults to the RDK X5 branch. If the actual development board used is another product in the RDK series, please use the git checkout command to switch branches. Here is an example for switching to the RDK X3 branch:

```
git checkout rdk_x3
```

After cloning, use the following command to enter the Model Zoo directory:
```
cd rdk_model_zoo
```

Then use the following command to enter Jupyter Lab (Note: The IP address is the actual IP used when logging in to the board):
```
jupyter lab --allow-root --ip 10.112.148.68
```
![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/04_Algorithm_Application/03_model_zoo/image/jupyter_start.png)

After using the command, the above log will appear. Hold Ctrl and click the link shown in the figure with the left mouse button to enter Jupyter Lab (as shown in the figure below). Double-click demos to select a model and experience RDK Model Zoo.

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/04_Algorithm_Application/03_model_zoo/image/into_jupyter.png)

## Module Introduction

The RDK series Model Zoo is generally divided into the following modules (this part takes RDK X5 as an example, please switch to the corresponding branch according to the actual situation):

1. **[Large Models](https://github.com/D-Robotics/rdk_model_zoo/tree/main/demos/llm)**
2. **[Image Classfication](https://github.com/D-Robotics/rdk_model_zoo/tree/main/demos/classification)**
3. **[Object Detection](https://github.com/D-Robotics/rdk_model_zoo/tree/main/demos/detect)**
4. **[Instance Segmentation](https://github.com/D-Robotics/rdk_model_zoo/tree/main/demos/Instance_Segmentation)**

Developers can jump to the corresponding module to experience the deployment of models on RDK series development boards.

## Usage Guide

In Jupyter Lab, after selecting a model's notebook and entering, developers will arrive at an interface similar to the following:

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/04_Algorithm_Application/03_model_zoo/image/basic_usage.png)

Here, taking the YOLO World model as an example, users only need to click the double triangle button in the above figure to run all cells. Drag the mouse to the bottom to see the result display:

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/04_Algorithm_Application/03_model_zoo/image/basic_usage_res.png)

Developers can also choose to run cells one by one. In this case, just press Shift + Enter to complete the current cell and move to the next cell.