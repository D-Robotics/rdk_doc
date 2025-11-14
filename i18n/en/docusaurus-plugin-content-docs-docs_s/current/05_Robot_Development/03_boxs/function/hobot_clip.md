---
sidebar_position: 1
---
# CLIP


```mdx-code-block
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';
```

## Introduction

[CLIP](https://github.com/openai/CLIP/) is a multimodal machine learning model proposed by OpenAI. This model uses contrastive learning on large-scale image-text pairs to process both images and text, mapping them into a shared vector space. This example demonstrates the functionality of using CLIP for image management and text query on the RDK platform.

Code repository:  (https://github.com/D-Robotics/hobot_clip.git)

Application scenario: Using CLIP image feature extractor to manage images, usr text or image to query images, etc.

# Component

The project consists of four parts.

- [clip_encode_image](https://github.com/D-Robotics/hobot_clip/tree/develop/clip_encode_image): an dnn node for the CLIP image encoder, currently supporting two modes:
  - Local mode: Supports input backpropagation, outputting text encoding features.
  - Service mode: Based on ROS Action Server, supports client nodes sending inference requests and calculating the returned text encoding features.

- [clip_encode_text](https://github.com/D-Robotics/hobot_clip/tree/develop/clip_encode_text): an dnn node for the CLIP text encoder, currently supporting two modes:
  - Local mode: Supports input backpropagation, outputting text encoding features.
  - Service mode: Based on ROS Action Server, supports client nodes sending inference requests and calculating the returned text encoding features.

- [clip_manage](https://github.com/D-Robotics/hobot_clip/tree/develop/clip_manage): CLIP relay node responsible for clienting and servicing. Currently, it supports two modes:
  - Storage mode: Send encoding requests to the image encoding node clip_encode_image, retrieve image encoding features from the target folder, and store the image encoding features in the local SQLite database.
  - Query mode: Send an encoding request to the text encoding node clip_encode_text to obtain the encoding features of the target text. Next step, match the text features with image features in the database to obtain the matching results.

- [clip_msgs](https://github.com/D-Robotics/hobot_clip/tree/develop/clip_msgs): CLIP app topic definition, action server control msg definition。

## Supported Platforms

| Platform             | System | Function                                            |
| -------------------- | ---------------- | ------------------------------------------------------------|
| RDK X5 | Ubuntu 22.04 (Humble) | Start CLIP Storage/Query mode, Storage database saved locally while query results display on the Web|
| RDK S100, RDK S100P | Ubuntu 22.04 (Humble) | Start CLIP Storage/Query mode, Storage database saved locally while query results display on the Web|

## Preparation

### RDK

1. The RDK has burned the  Ubuntu 22.04 system image provided by D-Robotics.

2. The RDK has successfully installed TogetheROS.Bot.

### Dependency Installation

```shell
pip3 install onnxruntime
pip3 install ftfy
pip3 install wcwidth
pip3 install regex
```

### Model Download
```shell
# Download the model file from the web.
wget http://archive.d-robotics.cc/models/clip_encode_text/text_encoder.tar.gz
sudo tar -xf text_encoder.tar.gz -C config
```

## Usage

### RDK

**Mode One: Storage**

Set clip_mode to "0" to store the image files from the "/root/config" directory into the "clip.db" database.

(Users can change the image folder path "clip_storage_folder" and the database name "clip_db_file" as needed. It is recommended to use absolute paths.)

<Tabs groupId="tros-distro">

<TabItem value="x5" label="RDK X5">

```shell
# Configure the tros.b environment
source /opt/tros/humble/setup.bash

# Copy the configuration file required for running the example from the installation path of tros.b.
cp -r /opt/tros/${TROS_DISTRO}/lib/clip_encode_image/config/ .

# Start the launch file
ros2 launch clip_manage hobot_clip_manage.launch.py clip_mode:=0 clip_db_file:=clip.db clip_storage_folder:=/root/config
```

</TabItem>

<TabItem value="s100" label="RDK S100">

```shell
# Configure the tros.b environment
source /opt/tros/humble/setup.bash

# Copy the configuration file required for running the example from the installation path of tros.b.
cp -r /opt/tros/${TROS_DISTRO}/lib/clip_encode_image/config/ .

# Start the launch file
ros2 launch clip_manage hobot_clip_manage.launch.py clip_mode:=0 clip_image_model_file_name:=config/full_model_11.hbm clip_db_file:=clip.db clip_storage_folder:=/root/config
```

</TabItem>

</Tabs>

**Mode Two: Query**

Set clip_mode to "1", set database path to "clip.db" and set query text "a diagram". the query result is saved in "result" folder.

(Users can change the database name "clip_db_file", query text "clip_text", and query result path "clip_result_folder" as needed. It is recommended to use absolute paths.)

<Tabs groupId="tros-distro">

<TabItem value="x5" label="RDK X5">

```shell
# Configure the tros.b environment
source /opt/tros/humble/setup.bash

# Start the launch file
ros2 launch clip_manage hobot_clip_manage.launch.py clip_mode:=1 clip_db_file:=clip.db clip_result_folder:=result clip_text:="a diagram"
```
</TabItem>

<TabItem value="s100" label="RDK S100">

```shell
# Configure the tros.b environment
source /opt/tros/humble/setup.bash

# Start the launch file
ros2 launch clip_manage hobot_clip_manage.launch.py clip_mode:=1 clip_image_model_file_name:=config/full_model_11.hbm clip_db_file:=clip.db clip_result_folder:=result clip_text:="a diagram"
```
</TabItem>

</Tabs>

**Display Query Result**

Run command in another terminal:

```shell
cp -r /opt/tros/${TROS_DISTRO}/lib/clip_manage/config/index.html .
python -m http.server 8080
```

## Result Analysis

**Mode One: Storage**

The following information will be displayed in the terminal:

```shell
[clip_manage-3] [WARN] [0000434374.492834334] [image_action_client]: Action client recved goal
[clip_manage-3] [WARN] [0000434374.493161250] [image_action_client]: Action client got lock
[clip_manage-3] [WARN] [0000434374.493402834] [image_action_client]: Sending goal, type: 1, urls size: 0
[clip_encode_image-1] [WARN] [0000434374.494557250] [encode_image_server]: Received goal request with type: 1
[clip_encode_image-1] [WARN] [0000434374.495408375] [encode_image_server]: Executing goal
[clip_encode_image-1] [WARN] [0000434379.674204836] [ClipImageNode]: Sub img fps: 1.58, Smart fps: 1.58, preprocess time ms: 1422, infer time ms: 218, post process time ms: 0
[clip_encode_image-1] [WARN] [0000434380.881684628] [ClipImageNode]: Sub img fps: 3.31, Smart fps: 3.31, preprocess time ms: 44, infer time ms: 216, post process time ms: 0
[clip_encode_image-1] [WARN] [0000434380.882277045] [encode_image_server]: Goal complete, task_result: 1
[clip_manage-3] [WARN] [0000434381.704573129] [image_action_client]: Get Result errorcode: 0
[clip_manage-3] [WARN] [0000434381.704934504] [ClipNode]: Storage finish, current num of database: 7.
```

**Mode Two: Query**

The following information will be displayed in the terminal:

```shell
[clip_manage-3] [WARN] [0000435148.509009119] [ClipNode]: Query start, num of database: 7.
[clip_manage-3] [WARN] [0000435148.509820786] [ClipNode]: Query finished! Cost 1 ms.
[clip_encode_text_node-2] [WARN] [0000435148.514026703] [clip_encode_text_node]: Clip Encode Text Node work success.
[clip_manage-3] [WARN] [0000435148.532558536] [ClipNode]: Query Result config/CLIP.png, similarity: 0.289350
[clip_manage-3] [WARN] [0000435148.540040328] [ClipNode]: Query Result config/dog.jpg, similarity: 0.228837
[clip_manage-3] [WARN] [0000435148.547667078] [ClipNode]: Query Result config/target_class.jpg, similarity: 0.224744
[clip_manage-3] [WARN] [0000435148.555092286] [ClipNode]: Query Result config/target.jpg, similarity: 0.207572
[clip_manage-3] [WARN] [0000435148.562450494] [ClipNode]: Query Result config/raw_unet.jpg, similarity: 0.198459
[clip_manage-3] [WARN] [0000435148.569500536] [ClipNode]: Query Result config/people.jpg, similarity: 0.174074
[clip_manage-3] [WARN] [0000435148.576885453] [ClipNode]: Query Result config/test.jpg, similarity: 0.174074
[clip_manage-3] [WARN] [0000435148.584450703] [text_action_client]: Get Result errorcode: 0
```

**Display Query Result**

Use Google Chrome or Edge and enter http://IP:8080 to view the image retrieval results (where IP is the device's IP address).

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/03_boxs/function/image/box_adv/query_display.png)

Result Analysis: You can sequentially see the retrieval results based on the similarity between the query text and images. Among them, only the CLIP.png image is provided for this example, while the other images are from the user's actual configuration. Therefore, it is expected that only the first image in the visualization result will be the same as in the example.
