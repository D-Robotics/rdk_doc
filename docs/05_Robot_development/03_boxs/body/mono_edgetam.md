---
sidebar_position: 9
---

# 目标跟踪分割(EdgeTAM)

```mdx-code-block
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';
```

## 功能介绍

EdgeTAM (Edge Track Anything Model) 是基于 Facebook Research 开源的 [EdgeTAM](https://github.com/facebookresearch/EdgeTAM) 模型在 RDK 平台上部署的目标跟踪与分割应用。通过点提示或框提示，在视频流中对任意目标进行持续跟踪与分割。

`mono_edgetam` 包含两个子工程：

- **mono_edgetam_prompt**：负责 prompt 初始化，基于输入图像和点/框提示做模型推理，生成并保存 memory 特征文件供下游使用。
- **mono_edgetam_track**：负责连续跟踪分割，加载 prompt 阶段保存的特征信息后进行后续帧的跟踪并发布分割结果。

使用流程：先启动 `mono_edgetam_prompt` 对目标进行初始化，再启动 `mono_edgetam_track` 加载初始化特征开始跟踪。

代码仓库：

 (https://github.com/D-Robotics/mono_edgetam)

应用场景：EdgeTAM能够通过点/框提示对任意目标进行持续的跟踪与分割，可实现视频目标分割、交互式视频编辑等功能，主要应用于自动驾驶、视频分析、智能交互等领域。

## 支持平台

| 平台                     | 运行方式     | 示例功能                                                 |
| ------------------------ | ------------ | -------------------------------------------------------- |
| RDK S100, RDK S100P | Ubuntu 22.04 (Humble) | 启动MIPI/USB摄像头/本地回灌，并通过Web展示推理渲染结果 |

## 算法信息

| 模型 | 平台 | 输入尺寸 | 推理帧率(fps) |
| ---- | ---- | ------------ | ---- |
| EdgeTAM Prompt | S100 | 1x1024x1024x3 | - |
| EdgeTAM Track | S100 | 1x1024x1024x3 | - |

## 准备工作

### RDK平台

1. RDK已烧录好Ubuntu 22.04系统镜像。

2. RDK已成功安装TogetheROS.Bot。

3. RDK已安装MIPI或者USB摄像头，无摄像头的情况下通过回灌本地JPEG格式图片的方式体验算法效果。

4. 确认PC机能够通过网络访问RDK。

### 下载模型与数据

```shell
# 下载prompt模型
wget https://archive.d-robotics.cc/downloads/models/edgetam/s100/model_prompt_to_memory_points.hbm

# 下载track模型
wget https://archive.d-robotics.cc/downloads/models/edgetam/s100/model_track_step_s7.hbm

# 下载示例数据集
wget https://archive.d-robotics.cc/downloads/models/edgetam/bedroom.tar
tar -xvf bedroom.tar
```

## 使用介绍

EdgeTAM 跟踪分割包含两个阶段：**提示阶段** 和 **跟踪阶段**。

1. 提示阶段：跟踪前需要获取一个 **traget embedding** 提示特征用于跟踪。这一步通过 sam 机制, 通过给图像一个点/框作为提示词，生成图像的分割结果和 **traget embedding** 提示特征。注意目标物体移动到提示点/框区域内。可在本文最下方 **进阶使用** 修改点/框区域。

2. 跟踪阶段：关闭提示阶段后, 加载上一个 **traget embedding** 提示特征, 进行跟踪。
两个节点不支持同时启动。

### 1. 启动 mono_edgetam_prompt（提示阶段）

prompt 初始化节点基于输入图像和点/框提示做模型推理，生成 **traget embedding** 提示特征文件保存到本地，供后续跟踪节点加载。

**mipi摄像头发布图片**

<Tabs groupId="tros-distro">

<TabItem value="humble" label="Humble">

```shell
# 配置ROS2环境
source /opt/tros/humble/setup.bash

# 配置MIPI摄像头
export CAM_TYPE=mipi

# 启动launch文件
ros2 launch mono_edgetam_prompt mono_edgetam_prompt.launch.py edgetam_prompt_mode:=0
```

</TabItem>

</Tabs>

**使用usb摄像头发布图片**

<Tabs groupId="tros-distro">

<TabItem value="humble" label="Humble">

```shell
# 配置ROS2环境
source /opt/tros/humble/setup.bash

# 配置USB摄像头
export CAM_TYPE=usb

# 启动launch文件
ros2 launch mono_edgetam_prompt mono_edgetam_prompt.launch.py edgetam_prompt_mode:=0
```

</TabItem>

</Tabs>

**使用单张回灌图片**

<Tabs groupId="tros-distro">

<TabItem value="humble" label="Humble">

```shell
# 配置ROS2环境
source /opt/tros/humble/setup.bash

# 配置回灌图片
export CAM_TYPE=fb

# 启动launch文件
ros2 launch mono_edgetam_prompt mono_edgetam_prompt.launch.py edgetam_prompt_mode:=0
```

</TabItem>

</Tabs>

#### Web 展示

在PC端的浏览器输入 `http://IP:8000` 即可查看图像和算法渲染效果（IP为RDK的IP地址），打开界面右上角设置，选中"Full Image Segmentation"选项，可以显示分割渲染效果。

提示阶段渲染效果：

![](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/03_boxs/body/image/render_frame0.png)

提示阶段节点完成一次推理后，会自动将生成的特征文件保存到当前工作目录，供后续跟踪节点加载使用。

**提示词保存说明**

- **保存时机**：节点接收到图像并完成推理后，会立即将 memory 特征张量写入本地文件。选择分割结果最好的帧退出, 这时保存当前目前的跟踪特征。
- **生成文件**：
  - `cond_maskmem_features.bin`：mask memory 特征文件
  - `cond_maskmem_pos_enc.bin`：memory positional encoding 文件
  - `cond_obj_ptr.bin`：object pointer 文件

:::warning
**注意**：跟踪节点启动时将从当前工作目录加载这些特征文件。如果在提示阶段和跟踪阶段之间切换了工作目录，请将生成的特征文件拷贝到跟踪节点的工作目录下，或将两个阶段在相同目录下运行。
:::

### 2. 启动 mono_edgetam_track（跟踪阶段）

跟踪节点加载特征文件（`cond_maskmem_features.bin`、`cond_maskmem_pos_enc.bin`、`cond_obj_ptr.bin`），对视频流中的目标进行连续跟踪与分割，并发布分割结果。

**注意**：在**相同目录**下运行 `mono_edgetam_track` 跟踪节点，确保跟踪节点加载的是提示阶段生成的特征文件。

**mipi摄像头发布图片**

<Tabs groupId="tros-distro">

<TabItem value="humble" label="Humble">

```shell
# 配置ROS2环境
source /opt/tros/humble/setup.bash

# 配置MIPI摄像头
export CAM_TYPE=mipi

# 启动launch文件
ros2 launch mono_edgetam_track mono_edgetam_track.launch.py edgetam_is_overwrite_features:=0
```

</TabItem>

</Tabs>

**使用usb摄像头发布图片**

<Tabs groupId="tros-distro">

<TabItem value="humble" label="Humble">

```shell
# 配置ROS2环境
source /opt/tros/humble/setup.bash

# 配置USB摄像头
export CAM_TYPE=usb

# 启动launch文件
ros2 launch mono_edgetam_track mono_edgetam_track.launch.py edgetam_is_overwrite_features:=0
```

</TabItem>

</Tabs>

**使用单张回灌图片**

<Tabs groupId="tros-distro">

<TabItem value="humble" label="Humble">

```shell
# 配置ROS2环境
source /opt/tros/humble/setup.bash

# 配置回灌图片
export CAM_TYPE=fb

# 启动launch文件
ros2 launch mono_edgetam_track mono_edgetam_track.launch.py edgetam_is_overwrite_features:=0
```

</TabItem>

</Tabs>


## 结果分析

### Web 展示

在PC端的浏览器输入 `http://IP:8000` 即可查看图像和算法渲染效果（IP为RDK的IP地址），打开界面右上角设置，选中"Full Image Segmentation"选项，可以显示分割渲染效果。

跟踪阶段渲染效果：

![](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/03_boxs/body/image/render_frames.gif)

## 进阶使用

### 1. 修改提示模式（仅提示阶段）

prompt 模式通过 `edgetam_prompt_mode` 参数设置：

- `0`：**框类提示**（默认） — 算法使用边界框来定义目标区域。框内的物体将被作为目标进行跟踪与分割。
- `1`：**点类提示** — 算法使用点坐标来定义目标区域。基于 SAM 机制，算法会选择指定点附近最显著的目标物体。可以通过指定最多两个点来细化选择范围。

**点类提示如何选择区域**

当设置 `edgetam_prompt_mode:=1`（点类提示模式）时，需要指定至少一个点坐标。每个点通过一个 `rect`（`width=0, height=0`）来表示，其中 `x_offset` 和 `y_offset` 定义了该点在图像上的像素坐标。算法利用 SAM 的分割能力来识别给定点附近的目标物体。

以下示例中指定了两个点：
- 第一个点 `{x_offset: 210, y_offset: 350}` 作为 **正向点**，算法将包含该位置附近的目标物体。
- 第二个点 `{x_offset: 250, y_offset: 220}` 作为额外的参考点，帮助算法更精准地定位目标物体。

在同一个目标上使用两个（或更多）正向点可以产生更准确的分割掩膜。如果目标与背景分离较好，也可以使用单个点。

### 2. 动态修改提示词（仅提示阶段）

启动节点的同时，可以通过 topic 发布动态修改提示框/点：

<Tabs groupId="tros-distro">

<TabItem value="humble" label="Humble">

```shell
# 配置ROS2环境
source /opt/tros/humble/setup.bash
```

</TabItem>

</Tabs>

```shell
# 在另一个终端发布'框'提示
# rect 参数说明:
#   x_offset: 边界框左上角 X 坐标（单位：像素）
#   y_offset: 边界框左上角 Y 坐标（单位：像素）
#   width: 边界框宽度（单位：像素），设为 0 表示点提示模式
#   height: 边界框高度（单位：像素），设为 0 表示点提示模式
#   当 width > 0 且 height > 0 时：(x_offset, y_offset) 为框提示的左上角
#   当 width = 0 且 height = 0 时：(x_offset, y_offset) 为点提示的坐标
ros2 topic pub /hobot_dnn_detection ai_msgs/msg/PerceptionTargets \
  '{"targets": [{"rois": [{"rect": {"x_offset": 240, "y_offset": 135, "width": 480, "height": 270}, "type": "anything"}]}]}'
```

```shell
# 或发布'点'提示（设置框宽高为0）
# 以下两个点定义了两个正向点，帮助算法更准确地定位目标物体
# rect 参数说明:
#   x_offset: 提示点 X 坐标（单位：像素）
#   y_offset: 提示点 Y 坐标（单位：像素）
ros2 topic pub /hobot_dnn_detection ai_msgs/msg/PerceptionTargets \
  '{"targets": [{"rois": [{"rect": {"x_offset": 210, "y_offset": 350, "width": 0, "height": 0}, "type": "anything"}, {"rect": {"x_offset": 250, "y_offset": 220, "width": 0, "height": 0}, "type": "anything"}]}]}'
```
