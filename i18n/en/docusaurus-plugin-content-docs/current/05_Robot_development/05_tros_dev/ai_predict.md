---
sidebar_position: 2
---

# 5.5.2 Model Inference

```mdx-code-block
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';
```

## Supported Platforms

| Platform                     | Runtime Environment                         |
| ---------------------------- | ------------------------------------------- |
| RDK X3, RDK X3 Module        | Ubuntu 20.04 (Foxy), Ubuntu 22.04 (Humble)  |

:::info
The theoretical content introduced in this section applies to all platforms. The code examples are based on the `RDK X3` platform; adapting them for other platforms may require code modifications.
:::

## Model Inference Development

### Functional Background

`hobot_dnn` is an on-device algorithm inference framework within the TogetherROS.Bot software stack. On the RDK platform, it leverages the BPU processor to perform algorithm inference by building upon D-Robotics' algorithm inference framework and ROS2 Node architecture. It provides robot application developers with simpler and more user-friendly model integration interfaces, including model management, input processing and result parsing based on model descriptions, and memory allocation management for model outputs.

By reading this section, users can create and run a human detection algorithm node on the RDK using `hobot_dnn` and models provided by D-Robotics. With components provided by tros.b, users can subscribe to images captured and published by a camera, perform algorithmic inference to detect human bounding boxes, apply a multi-object tracking (`multi-target tracking`, or `MOT`) algorithm to track detected boxes and assign target IDs, and finally render and display the original image, human detection boxes, and tracking results in real time on a web browser running on a PC.

### Prerequisites

1. An RDK development board with the following software installed:
   - Ubuntu 20.04 or Ubuntu 22.04 system image.
   - tros.b software packages.
   - ROS2 package build and compilation tools. Install command: `sudo apt install ros-dev-tools`

2. An F37 or GC4663 camera installed on the RDK.

3. A PC capable of accessing the RDK over the network.

For detailed usage instructions of `hobot_dnn`, please refer to the [README.md](https://github.com/D-Robotics/hobot_dnn/blob/develop/README.md) and the [API documentation](https://github.com/D-Robotics/hobot_dnn/blob/develop/dnn_node/docs/API-Manual/API-Manual.md) in the `hobot_dnn` repository. The logical workflow of using `hobot_dnn` is as follows:

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/05_tros_dev/image/ai_predict/dnnnode_workflow.jpg)

Even without prior knowledge of the `hobot_dnn` workflow, users can follow the procedures in this section to develop a model inference example using `hobot_dnn`.

### Task Content

#### 1. Create a Package

<Tabs groupId="tros-distro">
<TabItem value="foxy" label="Foxy">

```shell
source /opt/tros/local_setup.bash
```

</TabItem>

<TabItem value="humble" label="Humble">

```shell
source /opt/tros/humble/local_setup.bash
```

</TabItem>

</Tabs>


```shell
mkdir -p ~/dev_ws/src
cd ~/dev_ws/src
ros2 pkg create --build-type ament_cmake cpp_dnn_demo --dependencies rclcpp sensor_msgs hbm_img_msgs ai_msgs dnn_node hobot_mot
cd cpp_dnn_demo
touch src/body_det_demo.cpp
```

#### 2. Write the Example Code

After creation, the project directory structure should look like this:

```shell
root@ubuntu:~# cd ~
root@ubuntu:~# tree dev_ws/
dev_ws/
└── src
    └── cpp_dnn_demo
        ├── CMakeLists.txt
        ├── include
        │   └── cpp_dnn_demo
        ├── package.xml
        └── src
            └── body_det_demo.cpp

5 directories, 3 files
```

On the RDK, use a text editor such as vi/vim to open the newly created source file:  
`vi ~/dev_ws/src/cpp_dnn_demo/src/body_det_demo.cpp`

Copy the following code into the file:

```c++
#include "dnn_node/dnn_node.h"
#include "dnn_node/util/image_proc.h"
#include "dnn_node/util/output_parser/detection/fasterrcnn_output_parser.h"
#include "sensor_msgs/msg/image.hpp"
#include "ai_msgs/msg/perception_targets.hpp"
#include "hbm_img_msgs/msg/hbm_msg1080_p.hpp"
#include "hobot_mot/hobot_mot.h"

// Define the algorithm inference output data structure, adding a message header member
struct FasterRcnnOutput : public hobot::dnn_node::DnnNodeOutput {
  std::shared_ptr<std_msgs::msg::Header> image_msg_header = nullptr;
};

// Inherit from the DnnNode abstract base class to create an algorithm inference node
class BodyDetNode : public hobot::dnn_node::DnnNode {
 public:
  BodyDetNode(const std::string &node_name = "body_det",
  const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

 protected:
  // Implement the pure virtual function from the base class to configure node parameters
  int SetNodePara() override;
  // Implement the virtual function from the base class to encapsulate parsed model output into a ROS message and publish it
  int PostProcess(const std::shared_ptr<hobot::dnn_node::DnnNodeOutput> &node_output)
    override;

 private:
  // Width and height of the input image expected by the model
  int model_input_width_ = -1;
  int model_input_height_ = -1;
  // Model output index corresponding to human detection bounding boxes
  const int32_t box_output_index_ = 1;
  // Set of output indices for bounding boxes
  const std::vector<int32_t> box_outputs_index_ = {box_output_index_};

  // Image message subscriber
  rclcpp::Subscription<hbm_img_msgs::msg::HbmMsg1080P>::ConstSharedPtr
      ros_img_subscription_ = nullptr;
  // Publisher for algorithm inference results
  rclcpp::Publisher<ai_msgs::msg::PerceptionTargets>::SharedPtr
      msg_publisher_ = nullptr;
  // Multi-object tracking (MOT) algorithm engine
  std::shared_ptr<HobotMot> hobot_mot_ = nullptr;

  // Callback function for subscribed image messages
  void FeedImg(const hbm_img_msgs::msg::HbmMsg1080P::ConstSharedPtr msg);
};

BodyDetNode::BodyDetNode(const std::string & node_name, const rclcpp::NodeOptions & options) :
  hobot::dnn_node::DnnNode(node_name, options) {
  // Use the SetNodePara() method implemented in the BodyDetNode subclass during Init() to initialize algorithm inference
  if (Init() != 0 ||
    GetModelInputSize(0, model_input_width_, model_input_height_) < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("dnn_demo"), "Node init fail!");
    rclcpp::shutdown();
  }

  // Create a message subscriber to receive image messages from the camera node
  ros_img_subscription_ =
          this->create_subscription<hbm_img_msgs::msg::HbmMsg1080P>(
          "/hbmem_img", 10, std::bind(&BodyDetNode::FeedImg, this, std::placeholders::_1));
  // Create a message publisher to send out algorithm inference results
  msg_publisher_ = this->create_publisher<ai_msgs::msg::PerceptionTargets>(
      "/cpp_dnn_demo", 10);
  // Initialize the multi-object tracking (MOT) algorithm engine
  hobot_mot_ = std::make_shared<HobotMot>("config/iou2_method_param.json");
}

int BodyDetNode::SetNodePara() {
  if (!dnn_node_para_ptr_) return -1;
  // Specify the model file path and model name used for algorithm inference
  dnn_node_para_ptr_->model_file = "config/multitask_body_kps_960x544.hbm";
  dnn_node_para_ptr_->model_name = "multitask_body_kps_960x544";
  return 0;
}

void BodyDetNode::FeedImg(const hbm_img_msgs::msg::HbmMsg1080P::ConstSharedPtr img_msg) {
  if (!rclcpp::ok()) {
    return;
  }

  // Validate the received image message; this example only supports NV12 format
  if (!img_msg) return;
  if ("nv12" != std::string(reinterpret_cast<const char*>(img_msg->encoding.data()))) {
    RCLCPP_ERROR(rclcpp::get_logger("dnn_demo"), "Only support nv12 img encoding!");
    return;
  }

  // Create model input data using the method provided by DnnNode, scaled to the model's expected input resolution
  auto inputs = std::vector<std::shared_ptr<hobot::dnn_node::DNNInput>>{
    hobot::dnn_node::ImageProc::GetNV12PyramidFromNV12Img(
      reinterpret_cast<const char*>(img_msg->data.data()),
      img_msg->height, img_msg->width, model_input_height_, model_input_width_)};

// Create model output data and populate the message header information  
  auto dnn_output = std::make_shared<FasterRcnnOutput>();  
  dnn_output->image_msg_header = std::make_shared<std_msgs::msg::Header>();  
  dnn_output->image_msg_header->set__frame_id(std::to_string(img_msg->index));  
  dnn_output->image_msg_header->set__stamp(img_msg->time_stamp);  

  // Run inference in asynchronous mode  
  Run(inputs, dnn_output, nullptr, false);  
}  

int BodyDetNode::PostProcess(const std::shared_ptr<hobot::dnn_node::DnnNodeOutput> &node_output) {  
  if (!rclcpp::ok()) {  
    return 0;  
  }  
    
  // Validate the output data  
  if (node_output->output_tensors.empty() ||  
    static_cast<int32_t>(node_output->output_tensors.size()) < box_output_index_) {  
    RCLCPP_ERROR(rclcpp::get_logger("dnn_demo"), "Invalid outputs");  
    return -1;  
  }  

  // Create parsed output data  
  // The dimension of detection results equals the number of detected object categories  
  std::vector<std::shared_ptr<hobot::dnn_node::parser_fasterrcnn::Filter2DResult>>  
      results;  
  // Keypoint data  
  std::shared_ptr<hobot::dnn_node::parser_fasterrcnn::LandmarksResult> output_body_kps = nullptr;  

  // Use the built-in Parse method from hobot dnn to parse algorithm outputs  
  if (hobot::dnn_node::parser_fasterrcnn::Parse(node_output, nullptr,  
  box_outputs_index_, -1, -1, results, output_body_kps) < 0) {  
    RCLCPP_ERROR(rclcpp::get_logger("dnn_node_sample"),  
                "Parse node_output fail!");  
    return -1;  
  }  

  auto filter2d_result = results.at(box_output_index_);  
  if (!filter2d_result) return -1;  

  // Convert human detection boxes from algorithm inference output into MOT algorithm input data type  
  std::vector<MotBox> in_box_list;  
  for (auto& rect : filter2d_result->boxes) {  
    in_box_list.emplace_back(  
        MotBox(rect.left, rect.top, rect.right, rect.bottom, rect.conf));  
  }  
    
  // Calculate the current frame's timestamp based on the message header  
  auto fasterRcnn_output =  
      std::dynamic_pointer_cast<FasterRcnnOutput>(node_output);  
  time_t time_stamp =  
      fasterRcnn_output->image_msg_header->stamp.sec * 1000 +  
      fasterRcnn_output->image_msg_header->stamp.nanosec / 1000 / 1000;  
    
  // Create MOT algorithm outputs: human detection boxes with assigned track IDs and disappeared track IDs  
  std::vector<MotBox> out_box_list;  
  std::vector<std::shared_ptr<MotTrackId>> disappeared_ids;  

  // Run multi-object tracking algorithm  
  if (hobot_mot_->DoProcess(in_box_list,  
                            out_box_list,  
                            disappeared_ids,  
                            time_stamp,  
                            model_input_width_,  
                            model_input_height_) < 0) {  
    RCLCPP_ERROR(rclcpp::get_logger("dnn_demo"), "Do mot fail");  
    return -1;  
  }  

  // Create ROS Msg for publishing inference results  
  ai_msgs::msg::PerceptionTargets::UniquePtr pub_data(  
      new ai_msgs::msg::PerceptionTargets());  

  // Populate message header into ROS Msg  
  pub_data->header.set__stamp(fasterRcnn_output->image_msg_header->stamp);  
  pub_data->header.set__frame_id(fasterRcnn_output->image_msg_header->frame_id);  

  // Populate algorithm inference output FPS into ROS Msg  
  if (node_output->rt_stat) {  
    pub_data->set__fps(round(node_output->rt_stat->output_fps));  
    // If algorithm inference statistics have been updated, output FPS statistics for model input and output  
    if (node_output->rt_stat->fps_updated) {  
      RCLCPP_WARN(rclcpp::get_logger("dnn_demo"),  
                  "input fps: %.2f, out fps: %.2f",  
                  node_output->rt_stat->input_fps,  
                  node_output->rt_stat->output_fps);  
    }  
  }  

  for (auto& rect : out_box_list) {  
    // Validate the validity of multi-object tracking results  
    if (rect.id < 0) {  
      continue;  
    }  
    // Populate multi-object tracking results and detection boxes into ROS Msg  
    ai_msgs::msg::Target target;  
    target.set__type("person");  
    target.set__track_id(rect.id);  
    ai_msgs::msg::Roi roi;  
    roi.type = "body";  
    roi.rect.set__x_offset(rect.x1);  
    roi.rect.set__y_offset(rect.y1);  
    roi.rect.set__width(rect.x2 - rect.x1);  
    roi.rect.set__height(rect.y2 - rect.y1);  
    target.rois.emplace_back(roi);  
    pub_data->targets.emplace_back(std::move(target));  
  }  

  // Populate disappeared targets into ROS Msg  
  for (const auto& id_info : disappeared_ids) {  
    if (id_info->value < 0 ||  
        hobot_mot::DataState::INVALID == id_info->state_) {  
      continue;  
    }  
    ai_msgs::msg::Target target;  
    target.set__type("person");  
    target.set__track_id(id_info->value);  
    ai_msgs::msg::Roi roi;  
    roi.type = "body";  
    target.rois.emplace_back(roi);  
    pub_data->disappeared_targets.emplace_back(std::move(target));  
  }  

  // Publish ROS Msg  
  msg_publisher_->publish(std::move(pub_data));  

  return 0;  
}  

int main(int argc, char** argv) {  
  rclcpp::init(argc, argv);  
  rclcpp::spin(std::make_shared<BodyDetNode>());  
  rclcpp::shutdown();  
  return 0;  
}  

```

##### 2.1 Node Design  

The example human detection algorithm Node primarily consists of three logically independent components.  

**(1) Node Initialization and Startup**  

Configure model information used by the algorithm, create publishers for algorithm inference messages and subscribers for image messages, and initialize the multi-object tracking algorithm engine.  

**(2) Message Subscription and Algorithm Inference**  

When creating the image message subscriber, register the message callback `FeedImg` to process image data and perform algorithm model inference. The callback does not wait for inference completion.  

**(3) Processing and Publishing Inference Results**  

After algorithm inference completes, the registered callback `PostProcess` outputs inference results. Within this callback, detection results are processed by the multi-object tracking algorithm (`HobotMot`) before publishing the algorithm inference result messages.  

The Node’s design and workflow logic are illustrated in the following diagram:  

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/05_tros_dev/image/ai_predict/node_architecture.jpg)  


##### 2.2 Code Explanation  

**Include Header Files**  

- Inference framework header `dnn_node/dnn_node.h` for algorithm model management and inference.  
- Algorithm input processing header `dnn_node/util/image_proc.h` for preprocessing input images.  
- Algorithm output parsing header `dnn_node/util/output_parser/detection/fasterrcnn_output_parser.h` for parsing structured data (in this example, human detection bounding boxes) from model outputs after inference.  
- ROS message headers for message subscription and publishing.  
- MOT algorithm engine header for performing object tracking on detected human bounding boxes.  

```c++  
#include "dnn_node/dnn_node.h"  
#include "dnn_node/util/image_proc.h"  
#include "dnn_node/util/output_parser/detection/fasterrcnn_output_parser.h"  
#include "sensor_msgs/msg/image.hpp"  
#include "ai_msgs/msg/perception_targets.hpp"  
#include "hbm_img_msgs/msg/hbm_msg1080_p.hpp"  
#include "hobot_mot/hobot_mot.h"  
```  

**Define Algorithm Inference Output Data Structure**  

Inherit from the base class `DnnNodeOutput` in `hobot_dnn`, and add a message header member to associate inference output with corresponding image information.  

```C++  
struct FasterRcnnOutput : public hobot::dnn_node::DnnNodeOutput {  
  std::shared_ptr<std_msgs::msg::Header> image_msg_header = nullptr;  
};  
```  

**Create Algorithm Inference Node**  

Inherit from the virtual base class `DnnNode` in `hobot_dnn`, define the algorithm inference node `BodyDetNode`, and implement the virtual interfaces declared in `DnnNode`.  

  `int SetNodePara()`: Configure model parameters.
`int PostProcess(const std::shared_ptr<hobot::dnn_node::DnnNodeOutput> &node_output)`: Callback for inference results. After parsing and structuring the model output data, it packages the results into a ROS message and publishes it.

```c++
class BodyDetNode : public hobot::dnn_node::DnnNode {
 public:
  BodyDetNode(const std::string &node_name = "body_det",
  const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

 protected:
  int SetNodePara() override;
  int PostProcess(const std::shared_ptr<hobot::dnn_node::DnnNodeOutput> &node_output)
    override;
```

**Implementing the Constructor of the BodyDetNode Subclass**

In the constructor of the `BodyDetNode` subclass, the node is initialized. Additionally, the input image dimensions required by the model—including width (`model_input_width_`) and height (`model_input_height_`)—are obtained via the `GetModelInputSize` interface. These dimensions are used during preprocessing; different models typically require different input resolutions.

Using zero-copy communication, an image message subscriber is created to subscribe to image messages from the camera node for model inference. The subscribed topic is `/hbmem_img`, and the message type is `hbm_img_msgs::msg::HbmMsg1080P`, defined in `tros.b`.

A message publisher is created to publish algorithm inference results. The published topic is `/cpp_dnn_demo`, and the message type is `ai_msgs::msg::PerceptionTargets`, defined in `tros.b`.

A Multi-Object Tracking (MOT) algorithm engine is instantiated to perform object tracking on each detected human bounding box.

Implementation of the `BodyDetNode` constructor:

```c++
BodyDetNode::BodyDetNode(const std::string & node_name, const rclcpp::NodeOptions & options) :
  hobot::dnn_node::DnnNode(node_name, options) {
  // Use the SetNodePara() method implemented in the BodyDetNode subclass to initialize algorithm inference in Init()
  if (Init() != 0 ||
    GetModelInputSize(0, model_input_width_, model_input_height_) < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("dnn_demo"), "Node init fail!");
    rclcpp::shutdown();
  }

  // Create a message subscriber to receive image messages from the camera node
  ros_img_subscription_ =
          this->create_subscription<hbm_img_msgs::msg::HbmMsg1080P>(
          "/hbmem_img", 10, std::bind(&BodyDetNode::FeedImg, this, std::placeholders::_1));
  // Create a message publisher to publish algorithm inference results
  msg_publisher_ = this->create_publisher<ai_msgs::msg::PerceptionTargets>(
      "/cpp_dnn_demo", 10);
  // Create a Multi-Object Tracking (MOT) algorithm engine
  hobot_mot_ = std::make_shared<HobotMot>("config/iou2_method_param.json");
}
```

Here, `Init()` is an interface defined and implemented in the base class `DnnNode`. It performs initialization for algorithm inference by assembling the pipeline; the specific `SetNodePara()` step must be implemented by the user (in the subclass). The initialization pipeline proceeds as follows:

```c++
int DnnNode::Init() {
  RCLCPP_INFO(rclcpp::get_logger("dnn"), "Node init.");

  int ret = 0;
  // 1. set model info in node para
  ret = SetNodePara();
  if (ret != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("dnn"), "Set node para failed!");
    return ret;
  }

  // check node para
  if (ModelTaskType::InvalidType == dnn_node_para_ptr_->model_task_type) {
    RCLCPP_ERROR(rclcpp::get_logger("dnn"), "Invalid model task type");
    return -1;
  }

  // 2. model init
  ret = dnn_node_impl_->ModelInit();
  if (ret != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("dnn"), "Model init failed!");
    return ret;
  }

  // 3. set output parser
  ret = SetOutputParser();
  if (ret != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("dnn"), "Set output parser failed!");
    return ret;
  }

  // 4. task init
  ret = dnn_node_impl_->TaskInit();
  if (ret != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("dnn"), "Task init failed!");
    return ret;
  }

  return ret;
}
```

**Configuring Model Parameters**

Configure the model file path and model name used for algorithm inference.

```c++
int BodyDetNode::SetNodePara() {
  if (!dnn_node_para_ptr_) return -1;
  dnn_node_para_ptr_->model_file = "config/multitask_body_kps_960x544.hbm";
  dnn_node_para_ptr_->model_name = "multitask_body_kps_960x544";
  return 0;
}
```

**Implementing the Image Subscription Callback**

Create model input data of type `DNNInput`. The subscribed message contains image information (encoding format, content data, resolution, etc.). Using the image preprocessing interface provided by `hobot_dnn`, namely `hobot::dnn_node::ImageProc::GetNV12PyramidFromNV12Img`, the subscribed `nv12`-formatted image is converted into the model input data format according to the model’s required input resolution (`model_input_width_` and `model_input_height_`). These dimensions are queried from the loaded model via the `GetModelInputSize` interface in the `BodyDetNode` constructor. The interface is defined as follows:

```c++
//   - [in] in_img_data: image data
//   - [in] in_img_height: image height
//   - [in] in_img_width: image width
//   - [in] scaled_img_height: model input height
//   - [in] scaled_img_width: model input width
std::shared_ptr<NV12PyramidInput> GetNV12PyramidFromNV12Img(
    const char* in_img_data,
    const int& in_img_height,
    const int& in_img_width,
    const int& scaled_img_height,
    const int& scaled_img_width);
```

Create model output data of type `FasterRcnnOutput`. The subscribed message includes a message header (frame_id and timestamp). This header is copied into the output data to associate the inference result with the corresponding input image.

Start inference. Use the `Run` interface from the base class `DnnNode` to execute inference asynchronously. The fourth parameter set to `false` enables asynchronous mode, which offers higher efficiency. The `Run` interface is defined as follows:

```c++
  // - Parameters:
  //   - [in] inputs: list of smart pointers to input data
  //   - [in] outputs: smart pointer to output data
  //   - [in] rois: ROI cropping data, valid only for ModelRoiInferType models
  //   - [in] is_sync_mode: inference mode; true for synchronous, false for asynchronous
  //   - [in] alloctask_timeout_ms: timeout (in ms) for acquiring an inference task;
  //                                default (-1) means wait indefinitely until successful
  //   - [in] infer_timeout_ms: inference timeout (in ms); default is 1000 ms
  int Run(std::vector<std::shared_ptr<DNNInput>> &inputs,
          const std::shared_ptr<DnnNodeOutput> &output = nullptr,
          const std::shared_ptr<std::vector<hbDNNRoi>> rois = nullptr,
          const bool is_sync_mode = true,
          const int alloctask_timeout_ms = -1,
          const int infer_timeout_ms = 1000);
```

The complete implementation of the image subscription callback `FeedImg` is as follows:

```c++
void BodyDetNode::FeedImg(const hbm_img_msgs::msg::HbmMsg1080P::ConstSharedPtr img_msg) {
  if (!rclcpp::ok()) {
    return;
  }

  // Validate the subscribed image message; this example only supports NV12 format
  if (!img_msg) return;
  if ("nv12" != std::string(reinterpret_cast<const char*>(img_msg->encoding.data()))) {
    RCLCPP_ERROR(rclcpp::get_logger("dnn_demo"), "Only support nv12 img encoding!");
    return;
  }

  // Create model input data using the method provided by hobot_dnn, based on the model's required input resolution
  auto inputs = std::vector<std::shared_ptr<hobot::dnn_node::DNNInput>>{
    hobot::dnn_node::ImageProc::GetNV12PyramidFromNV12Img(
      reinterpret_cast<const char*>(img_msg->data.data()),
      img_msg->height, img_msg->width, model_input_height_, model_input_width_)};
      
  // Create model output data and populate its message header
  auto dnn_output = std::make_shared<FasterRcnnOutput>();
  dnn_output->image_msg_header = std::make_shared<std_msgs::msg::Header>();
  dnn_output->image_msg_header->set__frame_id(std::to_string(img_msg->index));
  dnn_output->image_msg_header->set__stamp(img_msg->time_stamp);

  // Run inference in asynchronous mode
  Run(inputs, dnn_output, nullptr, false);
}
```

**Implementing the Inference Result Callback**

The inference result callback processes the parsed and structured model output through the MOT algorithm, producing human detection boxes with assigned target IDs and a list of disappeared target IDs. These results are then packaged into a ROS message and published.

Create structured inference result data of type `Filter2DResult`.

Use the built-in `Parse` method from `hobot_dnn` to parse the output of the human detection algorithm.

Run the multi-object tracking algorithm. Convert the human detection boxes from the inference output into the input data format expected by the MOT algorithm. Compute the current frame’s timestamp based on the message header. After processing by the MOT algorithm, obtain detection boxes with assigned target IDs and a list of disappeared target IDs.

Publish the inference results. Construct a ROS message, populate it with the corresponding image message header (frame ID and timestamp), detection boxes with target IDs, frame rate statistics from inference, and the list of disappeared target IDs. The published ROS message can be subscribed to and used by other ROS nodes.

```c++
int BodyDetNode::PostProcess(const std::shared_ptr<hobot::dnn_node::DnnNodeOutput> &node_output) {
  if (!rclcpp::ok()) {
    return 0;
  }
  
  // Validate the output data
  if (node_output->output_tensors.empty() ||
    static_cast<int32_t>(node_output->output_tensors.size()) < box_output_index_) {
    RCLCPP_ERROR(rclcpp::get_logger("dnn_demo"), "Invalid outputs");
    return -1;

  // Create parsed output data
  // The dimension of detection box results equals the number of detected object categories
  std::vector<std::shared_ptr<hobot::dnn_node::parser_fasterrcnn::Filter2DResult>>
      results;
  // Keypoint data
  std::shared_ptr<hobot::dnn_node::parser_fasterrcnn::LandmarksResult> output_body_kps = nullptr;

  // Use the built-in Parse method from hobot dnn to parse algorithm outputs
  if (hobot::dnn_node::parser_fasterrcnn::Parse(node_output, nullptr,
  box_outputs_index_, -1, -1, results, output_body_kps) < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("dnn_node_sample"),
                "Parse node_output fail!");
    return -1;
  }

  auto filter2d_result = results.at(box_output_index_);
  if (!filter2d_result) return -1;

  // Convert human detection boxes from algorithm inference output into MOT algorithm input data type
  std::vector<MotBox> in_box_list;
  for (auto& rect : filter2d_result->boxes) {
    in_box_list.emplace_back(
        MotBox(rect.left, rect.top, rect.right, rect.bottom, rect.conf));
  }
  
  // Calculate current frame timestamp based on message header
  auto fasterRcnn_output =
      std::dynamic_pointer_cast<FasterRcnnOutput>(node_output);
  time_t time_stamp =
      fasterRcnn_output->image_msg_header->stamp.sec * 1000 +
      fasterRcnn_output->image_msg_header->stamp.nanosec / 1000 / 1000;
  
  // Create MOT algorithm output: human detection boxes with target IDs and disappeared target IDs
  std::vector<MotBox> out_box_list;
  std::vector<std::shared_ptr<MotTrackId>> disappeared_ids;

  // Run multi-object tracking algorithm
  if (hobot_mot_->DoProcess(in_box_list,
                            out_box_list,
                            disappeared_ids,
                            time_stamp,
                            model_input_width_,
                            model_input_height_) < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("dnn_demo"), "Do mot fail");
    return -1;
  }

  // Create ROS Msg for publishing inference results
  ai_msgs::msg::PerceptionTargets::UniquePtr pub_data(
      new ai_msgs::msg::PerceptionTargets());

  // Fill message header into ROS Msg
  pub_data->header.set__stamp(fasterRcnn_output->image_msg_header->stamp);
  pub_data->header.set__frame_id(fasterRcnn_output->image_msg_header->frame_id);

  // Fill algorithm inference output FPS into ROS Msg
  if (node_output->rt_stat) {
    pub_data->set__fps(round(node_output->rt_stat->output_fps));
    // If algorithm inference statistics have been updated, output FPS statistics for model input and output
    if (node_output->rt_stat->fps_updated) {
      RCLCPP_WARN(rclcpp::get_logger("dnn_demo"),
                  "input fps: %.2f, out fps: %.2f",
                  node_output->rt_stat->input_fps,
                  node_output->rt_stat->output_fps);
    }
  }

  for (auto& rect : out_box_list) {
    // Validate the validity of object tracking results
    if (rect.id < 0) {
      continue;
    }
    // Fill object tracking results and detection boxes into ROS Msg
    ai_msgs::msg::Target target;
    target.set__type("person");
    target.set__track_id(rect.id);
    ai_msgs::msg::Roi roi;
    roi.type = "body";
    roi.rect.set__x_offset(rect.x1);
    roi.rect.set__y_offset(rect.y1);
    roi.rect.set__width(rect.x2 - rect.x1);
    roi.rect.set__height(rect.y2 - rect.y1);
    target.rois.emplace_back(roi);
    pub_data->targets.emplace_back(std::move(target));
  }

  // Fill disappeared targets into ROS Msg
  for (const auto& id_info : disappeared_ids) {
    if (id_info->value < 0 ||
        hobot_mot::DataState::INVALID == id_info->state_) {
      continue;
    }
    ai_msgs::msg::Target target;
    target.set__type("person");
    target.set__track_id(id_info->value);
    ai_msgs::msg::Roi roi;
    roi.type = "body";
    target.rois.emplace_back(roi);
    pub_data->disappeared_targets.emplace_back(std::move(target));
  }

  // Publish ROS Msg
  msg_publisher_->publish(std::move(pub_data));

  return 0;
}
```

The dnn node includes built-in model output parsing methods for various detection, classification, and segmentation algorithms. After installing `tros.b` on RDK, you can query supported parsing methods as follows:

```shell
root@ubuntu:~# tree /opt/tros/include/dnn_node/util/output_parser
/opt/tros/include/dnn_node/util/output_parser
├── classification
│   └── ptq_classification_output_parser.h
├── detection
│   ├── fasterrcnn_output_parser.h
│   ├── fcos_output_parser.h
│   ├── nms.h
│   ├── ptq_efficientdet_output_parser.h
│   ├── ptq_ssd_output_parser.h
│   ├── ptq_yolo2_output_parser.h
│   ├── ptq_yolo3_darknet_output_parser.h
│   └── ptq_yolo5_output_parser.h
├── perception_common.h
├── segmentation
│   └── ptq_unet_output_parser.h
└── utils.h

3 directories, 12 files
```

As shown above, under the path `/opt/tros/include/dnn_node/util/output_parser`, there are three subdirectories: `classification`, `detection`, and `segmentation`, corresponding respectively to model output parsing methods for classification, detection, and segmentation algorithms.

`perception_common.h` defines the data types for parsed perception results.

The following table lists algorithms along with their corresponding output parsing methods:

| Algorithm Category | Algorithm                 | Output Parsing Method |
| ---------------------- | ---------------------- | ----------- |
| Object Detection       | [FCOS](../03_boxs/detection/fcos.md)           | fcos_output_parser.h         |
| Object Detection       | [EfficientNet_Det](../03_boxs/detection/efficientnet.md)           | ptq_efficientdet_output_parser.h         |
| Object Detection       | [MobileNet_SSD](../03_boxs/detection/mobilenet.md)        |   ptq_ssd_output_parser.h       |
| Object Detection       | [YoloV2](../03_boxs/detection/yolo.md)       |   ptq_yolo2_output_parser.h       |
| Object Detection       | [YoloV3](../03_boxs/detection/yolo.md)       |    ptq_yolo3_darknet_output_parser.h       |
| Object Detection       | [YoloV5](../03_boxs/detection/yolo.md)       |  ptq_yolo5_output_parser.h        |
| Human Detection       | [FasterRcnn](../03_boxs/body/mono2d_body_detection.md)             |  fasterrcnn_output_parser.h       |
| Image Classification       | [mobilenetv2](../03_boxs/classification/mobilenetv2.md)  |  ptq_classification_output_parser.h        |
| Semantic Segmentation       | [mobilenet_unet](../03_boxs/segmentation/mobilenet_unet.md)      |  ptq_unet_output_parser.h        |


**Entry Function**

Create an instance of `BodyDetNode`. In the constructor of `BodyDetNode`, initialize and start the inference task, which continues running until the user sends a termination signal.

```c++
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BodyDetNode>());
  rclcpp::shutdown();
  return 0;
}
```


##### 2.3 Build Dependencies

In Step 1, the package `cpp_dnn_demo` was created using the `ros2 pkg create` command. The files `CMakeLists.txt` and `package.xml` have already been automatically generated under the path `dev_ws/src/cpp_dnn_demo`.

The `package.xml` file automatically includes build dependencies for required packages, including `rclcpp`, `sensor_msgs`, `ai_msgs`, `hbm_img_msgs`, `dnn_node`, and `hobot_mot`. Among these, `ai_msgs` defines the algorithm output message format in TogetherROS, `hbm_img_msgs` defines the image message format used for zero-copy communication in TogetherROS, `dnn_node` is the algorithm inference framework, and `hobot_mot` is the multi-object tracking algorithm. These packages are already installed when TogetherROS is installed.

##### 2.4 Build Script

Add package dependencies and build/install information in `CMakeLists.txt`.

(1) Add dependencies for the multi-object tracking algorithm and algorithm inference engine libraries

```cmake
link_directories(
  /opt/tros/lib/
  /usr/lib/hbbpu/
)
```

(2) Add package compilation information

```cmake
add_executable(${PROJECT_NAME}
  src/body_det_demo.cpp
)

ament_target_dependencies(
  ${PROJECT_NAME}
  rclcpp
  dnn_node
  sensor_msgs
  ai_msgs
  hobot_mot
  hbm_img_msgs
)
```

(3) Add package installation instructions to enable running the compiled package via `ros2 run`:

```cmake
install(
  TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
```

The complete CMakeLists.txt is as follows:

```cmake
cmake_minimum_required(VERSION 3.5)
project(cpp_dnn_demo)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ai_msgs REQUIRED)
find_package(hbm_img_msgs REQUIRED)
find_package(dnn_node REQUIRED)
find_package(hobot_mot REQUIRED)

link_directories(
  /opt/tros/lib/  
  /usr/lib/hbbpu/
)

add_executable(${PROJECT_NAME}
  src/body_det_demo.cpp
)

ament_target_dependencies(
  ${PROJECT_NAME}
  rclcpp
  dnn_node
  sensor_msgs
  ai_msgs
  hobot_mot
  hbm_img_msgs
)

# Install executables
install(
  TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()
```

#### 3 Compilation and Execution

##### 3.1 Compilation

On an RDK with tros.b installed, execute the following commands to compile the package:

<Tabs groupId="tros-distro">
<TabItem value="foxy" label="Foxy">

```shell
source /opt/tros/local_setup.bash
```

</TabItem>

<TabItem value="humble" label="Humble">

```shell
source /opt/tros/humble/local_setup.bash
```

</TabItem>

</Tabs>


```shell
cd ~/dev_ws

# Compile the package
colcon build --packages-select cpp_dnn_demo
```

If compilation succeeds, a deployment package named `install` for `cpp_dnn_demo` will be generated in the workspace directory, and the terminal output will show:

```shell
Starting >>> cpp_dnn_demo
[Processing: cpp_dnn_demo]                              
Finished <<< cpp_dnn_demo [32.7s]                       

Summary: 1 package finished [33.4s]
```

##### 3.2 Common Compilation Errors

1. ModuleNotFoundError: No module named 'ament_package'

Specific error message:

```
# colcon build --packages-select cpp_dnn_demo
Starting >>> cpp_dnn_demo
--- stderr: cpp_dnn_demo                         
CMake Error at CMakeLists.txt:19 (find_package):
  By not providing "Findament_cmake.cmake" in CMAKE_MODULE_PATH this project
  has asked CMake to find a package configuration file provided by
  "ament_cmake", but CMake did not find one.

  Could not find a package configuration file provided by "ament_cmake" with
  any of the following names:

    ament_cmakeConfig.cmake
    ament_cmake-config.cmake

  Add the installation prefix of "ament_cmake" to CMAKE_PREFIX_PATH or set
  "ament_cmake_DIR" to a directory containing one of the above files.  If
  "ament_cmake" provides a separate development package or SDK, be sure it
  has been installed.


---
Failed   <<< cpp_dnn_demo [2.83s, exited with code 1]

Summary: 0 packages finished [3.44s]
  1 package failed: cpp_dnn_demo
  1 package had stderr output: cpp_dnn_demo
```

This indicates that the ROS2 environment has not been configured properly. Verify your environment setup by entering the `ros2` command in the terminal:

```
# ros2

-bash: ros2: command not found
```

If you receive a "command not found" error, the ROS2 environment hasn't been set up correctly. Check whether the command `source /opt/tros/setup.bash` was executed successfully. A successful setup yields output similar to:

```
# ros2

usage: ros2 [-h] Call `ros2 <command> -h` for more detailed usage. ...

ros2 is an extensible command-line tool for ROS 2.

optional arguments:
  -h, --help            show this help message and exit
```

2. Unable to locate the `dnn_node` package

Specific error message:

```
colcon build --packages-select cpp_dnn_demo
Starting >>> cpp_dnn_demo
[Processing: cpp_dnn_demo]
--- stderr: cpp_dnn_demo
CMake Error at CMakeLists.txt:22 (find_package):
 By not providing "Finddnn_node.cmake" in CMAKE_MODULE_PATH this project has
 asked CMake to find a package configuration file provided by "dnn_node",
 but CMake did not find one.
Could not find a package configuration file provided by "dnn_node" with any
 of the following names:
dnn_nodeConfig.cmake
dnn_node-config.cmake
Add the installation prefix of "dnn_node" to CMAKE_PREFIX_PATH or set
 "dnn_node_DIR" to a directory containing one of the above files. If
 "dnn_node" provides a separate development package or SDK, be sure it has
 been installed.
Failed <<< cpp_dnn_demo [59.7s, exited with code 1]
Summary: 0 packages finished [1min 1s]
 1 package failed: cpp_dnn_demo
 1 package had stderr output: cpp_dnn_demo
```

This indicates that the `hobot_dnn` environment hasn't been configured properly. Check whether `/opt/tros/share/dnn_node` exists.

##### 3.3 Execution
To better demonstrate the algorithm inference performance and showcase perception capabilities, we utilize the MIPI camera image acquisition, image encoding, and WEB data visualization Nodes from tros.b to provide data sensing and visualization functionalities. This enables publishing camera-captured images on the RDK, performing algorithmic inference to detect human bounding boxes, and rendering both the original image and detection results in real time within a web browser on a PC.

The runtime system pipeline is shown below:

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/05_tros_dev/image/ai_predict/pipeline.jpg)

Four nodes run on the RDK, among which the algorithm inference node is the focus of this example.

The system startup procedure is as follows:

(1) On the RDK, open Terminal 1 and launch the algorithm inference node:

<Tabs groupId="tros-distro">
<TabItem value="foxy" label="Foxy">

```shell
source /opt/tros/local_setup.bash
```

</TabItem>

<TabItem value="humble" label="Humble">

```shell
source /opt/tros/humble/local_setup.bash
```

</TabItem>

</Tabs>

```shell
cd ~/dev_ws

# Configure the cpp_dnn_demo environment
source ./install/local_setup.bash

# Copy required configuration files for running the example from tros.b installation path.
# Model file
mkdir -p config && cp /opt/tros/lib/dnn_benchmark_example/config/X3/multitask_body_kps_960x544.hbm config/
# Multi-object tracking configuration file
cp -r /opt/tros/${TROS_DISTRO}/lib/hobot_mot/config/iou2_method_param.json config/


# Run the cpp_dnn_demo package
ros2 run cpp_dnn_demo cpp_dnn_demo --ros-args --log-level warn
```

(2) On the RDK, open Terminal 2 and launch the image publishing, encoding, and visualization Nodes from tros.b.

Since multiple Nodes need to be started, we use a launch script to start them in batch. Create a launch script named `cpp_dnn_demo.launch.py` in any directory on the RDK with the following content:

```python
import os

from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory
from ament_index_python.packages import get_package_prefix

def generate_launch_description():
    web_service_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('websocket'),
                'launch/websocket_service.launch.py'))
    )

    return LaunchDescription([
        web_service_launch_include,
        # Launch image publishing package
        Node(
            package='mipi_cam',
            executable='mipi_cam',
            output='screen',
            parameters=[
                {"out_format": "nv12"},
                {"image_width": 960},
                {"image_height": 544},
                {"io_method": "shared_mem"},
                {"video_device": "F37"}
            ],
            arguments=['--ros-args', '--log-level', 'error']
        ),
        # Launch JPEG image encoding & republishing package
        Node(
            package='hobot_codec',
            executable='hobot_codec_republish',
            output='screen',
            parameters=[
                {"channel": 1},
                {"in_mode": "shared_mem"},
                {"in_format": "nv12"},
                {"out_mode": "ros"},
                {"out_format": "jpeg"},
                {"sub_topic": "/hbmem_img"},
                {"pub_topic": "/image_jpeg"}
            ],
            arguments=['--ros-args', '--log-level', 'error']
        ),
        # Launch web visualization package
        Node(
            package='websocket',
            executable='websocket',
            output='screen',
            parameters=[
                {"image_topic": "/image_jpeg"},
                {"image_type": "mjpeg"},
                {"smart_topic": "/cpp_dnn_demo"}
            ],
            arguments=['--ros-args', '--log-level', 'error']
        )
    ])
```

Use the launch script as follows:

<Tabs groupId="tros-distro">
<TabItem value="foxy" label="Foxy">

```shell
source /opt/tros/local_setup.bash
```

</TabItem>

<TabItem value="humble" label="Humble">

```shell
source /opt/tros/humble/local_setup.bash
```

</TabItem>

</Tabs>

```shell
# Launch image publishing, encoding, and visualization nodes
ros2 launch cpp_dnn_demo.launch.py
```

##### 3.4 Common Errors During Execution

If you encounter the following error upon startup:

`error while loading shared libraries: libdnn_node.so: cannot open shared object file: No such file or directory`

This indicates that the hobot_dnn environment setup has failed. Use the command `ros2 pkg prefix dnn_node` to verify whether the dnn_node package exists.

##### 3.5 Execution Results

Upon successful execution, the terminal outputs the following messages:

```shell
root@ubuntu:~/dev_ws# ros2 run cpp_dnn_demo cpp_dnn_demo
[BPU_PLAT]BPU Platform Version(1.3.1)!
[C][154775][10-25][00:33:53:266][configuration.cpp:49][EasyDNN]EasyDNN version: 0.4.11
[HBRT] set log level as 0. version = 3.14.5
[DNN] Runtime version = 1.9.7_(3.14.5 HBRT)
[WARN] [1666629233.325690884] [dnn]: Run default SetOutputParser.
[WARN] [1666629233.326263403] [dnn]: Set output parser with default dnn node parser, you will get all output tensors and should parse output_tensors in PostProcess.
(MOTMethod.cpp:34): MOTMethod::Init config/iou2_method_param.json

(IOU2.cpp:29): IOU2 Mot::Init config/iou2_method_param.json

[WARN] [1666629234.410291616] [dnn_demo]: input fps: 31.22, out fps: 31.28
[WARN] [1666629235.410357068] [dnn_demo]: input fps: 30.00, out fps: 30.00
[WARN] [1666629236.444863458] [dnn_demo]: input fps: 30.01, out fps: 29.98
[WARN] [1666629237.476656118] [dnn_demo]: input fps: 30.00, out fps: 30.07
[WARN] [1666629238.478156431] [dnn_demo]: input fps: 30.01, out fps: 29.97
[WARN] [1666629239.510039629] [dnn_demo]: input fps: 30.01, out fps: 30.07
[WARN] [1666629240.511561150] [dnn_demo]: input fps: 30.00, out fps: 29.97
[WARN] [1666629241.543333811] [dnn_demo]: input fps: 30.01, out fps: 30.07
[WARN] [1666629242.544654089] [dnn_demo]: input fps: 30.01, out fps: 29.97
[WARN] [1666629243.576435625] [dnn_demo]: input fps: 30.01, out fps: 30.07
```

The log output shows that during initialization, the algorithm inference uses a model with an input image resolution of 960x544, employs a single inference task, and utilizes the MOT algorithm engine configured by the file `config/iou2_method_param.json`. During inference, both input and output frame rates are approximately 30 fps, with statistics refreshed once per second.

On the RDK, use the `ros2` command to query and display the message content published by the inference node on the `/cpp_dnn_demo` topic:

<Tabs groupId="tros-distro">
<TabItem value="foxy" label="Foxy">

```shell
root@ubuntu:~# source /opt/tros/local_setup.bash
```

</TabItem>

<TabItem value="humble" label="Humble">

```shell
root@ubuntu:~# source /opt/tros/humble/local_setup.bash
```

</TabItem>
</Tabs>

```shell
root@ubuntu:~# ros2 topic list
/cpp_dnn_demo
/hbmem_img08172824022201080202012021072315
/image_jpeg
/parameter_events
/rosout
root@ubuntu:~# ros2 topic echo /cpp_dnn_demo
header:
  stamp:
    sec: 1659938514
    nanosec: 550421888
  frame_id: '7623'
fps: 30
perfs: []
targets:
- type: person
  track_id: 1
  rois:
  - type: body
    rect:
      x_offset: 306
      y_offset: 106
      height: 416
      width: 151
      do_rectify: false
  attributes: []
  points: []
  captures: []
- type: person
  track_id: 2
  rois:
  - type: body
    rect:
      x_offset: 135
      y_offset: 89
      height: 423
      width: 155
      do_rectify: false
  attributes: []
  points: []
  captures: []
- type: person
  track_id: 3
  rois:
  - type: body
    rect:
      x_offset: 569
      y_offset: 161
      height: 340
      width: 123
      do_rectify: false
  attributes: []
  points: []
  captures: []
- type: person
  track_id: 4
  rois:
  - type: body
    rect:
      x_offset: 677
      y_offset: 121
      height: 398
      width: 123
      do_rectify: false
  attributes: []
  points: []
  captures: []
- type: person
  track_id: 5
  rois:
  - type: body
    rect:
      x_offset: 478
      y_offset: 163
      height: 348
      width: 103
      do_rectify: false
  attributes: []
  points: []
  captures: []
disappeared_targets: []
---

```

The output message from the `/cpp_dnn_demo` topic indicates that the algorithm has detected five human bounding boxes (with `rois` type as `body`) and provided the coordinates (`rect`) and corresponding object tracking results (`track_id`) for each detection box.

Enter `http://IP:8000` in a web browser on your PC (where IP is the RDK's IP address; for example, `10.64.28.88` in this demonstration) to view real-time images along with the algorithm inference visualization:

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/05_tros_dev/image/ai_predict/render.jpg)

Each detection box displays its type (e.g., `body` indicating a human body detection) and the associated tracking ID. The `fps` field in the bottom-left corner of the browser shows the real-time inference frame rate.

Press `Ctrl+C` to exit the program.

### Summary of This Section

This section introduced how to use models provided by D-Robotics to create and run a human detection inference example based on `hobot_dnn`. It demonstrated using images published from a camera, obtaining algorithm outputs, and rendering both images and inference results in real time on a PC web browser.

Users can refer to the [README.md](https://github.com/D-Robotics/hobot_dnn/blob/develop/README.md) and the [API documentation](https://github.com/D-Robotics/hobot_dnn/blob/develop/docs/API-Manual/API-Manual.md) in `hobot_dnn` for more advanced inference functionalities.

## Building Algorithm Workflows

### Background

ROS 2 nodes ([Node](http://docs.ros.org/en/foxy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Nodes/Understanding-ROS2-Nodes.html)) decompose complex robotic software systems into multiple functionally and logically independent modules. For instance, a robotic application may consist of numerous sensor and perception algorithm nodes. These nodes exchange data via "topics" ([Topic](http://docs.ros.org/en/foxy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html)), forming a directed acyclic graph (DAG) where different ROS 2 nodes connect through topics to implement the overall robotic software system.

The TogetheROS.Bot software stack provides a rich set of robotic development components and algorithm nodes. Sensor nodes support capturing image data from cameras and publishing it for perception algorithms. Specifically, a hand detection algorithm node consumes this image data to perform inference and publishes hand bounding box results. Subsequently, a hand keypoint detection algorithm node uses both the original image data and the hand bounding box results to infer and output hand keypoint detections. Thus, the hand keypoint detection node must subscribe—via ROS 2 topics—to the hand bounding box messages published by the hand detection node.

By reading this section, users will learn how to use sensor nodes, hand detection nodes, and hand keypoint detection nodes from tros.b on an RDK, connecting them through ROS 2 topic communication to build complex robotic perception pipelines.

### Prerequisites

1. An RDK development board with the following software installed:
   - Ubuntu 20.04 or Ubuntu 22.04 system image.
   - The tros.b software package.

2. An F37 or GC4663 camera installed on the RDK.

3. A PC on the same network subnet as the RDK (either wired or connected to the same Wi-Fi network; the first three segments of the IP addresses must match), with the following environment installed:
   - Ubuntu 20.04 or Ubuntu 22.04 OS.
   - [ROS 2 Foxy Desktop](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html).
   - [rqt graphical tools](http://docs.ros.org/en/foxy/Concepts/About-RQt.html).

### Task Description

#### 1. Launch the Data Acquisition Node

Open a terminal on the RDK and start an image publishing node to capture image data from the F37 camera and publish it for algorithm inference:

<Tabs groupId="tros-distro">
<TabItem value="foxy" label="Foxy">

```shell
source /opt/tros/local_setup.bash
```

</TabItem>

<TabItem value="humble" label="Humble">

```shell
source /opt/tros/humble/local_setup.bash
```

</TabItem>

</Tabs>

```shell
# Launch the node
ros2 run mipi_cam mipi_cam --ros-args -p out_format:=nv12 -p image_width:=960 -p image_height:=544 -p video_device:=F37 -p io_method:=shared_mem --log-level warn
```

#### 2. Launch the Hand Detection Algorithm Node

Open another terminal on the RDK and start the hand detection algorithm node, which subscribes to the image messages published by the data acquisition node, detects hands, and publishes hand bounding box messages.

The launch command specifies the published topic as `hobot_hand_detection`.

<Tabs groupId="tros-distro">
<TabItem value="foxy" label="Foxy">

```shell
source /opt/tros/local_setup.bash
```

</TabItem>

<TabItem value="humble" label="Humble">

```shell
source /opt/tros/humble/local_setup.bash
```

</TabItem>

</Tabs>

```shell
# Copy required configuration files for the example from the tros.b installation path.
cp -r /opt/tros/${TROS_DISTRO}/lib/mono2d_body_detection/config/ .
# Launch the node
ros2 run mono2d_body_detection mono2d_body_detection --ros-args --log-level warn --ros-args -p ai_msg_pub_topic_name:=hobot_hand_detection
```

#### 3. Launch the Hand Keypoint Detection Algorithm Node
Open a terminal on the RDK to launch the hand keypoint detection algorithm Node, which subscribes to image messages published by the data collection Node and hand bounding box messages published by the hand detection algorithm Node.

The launch command specifies the published message topic as `hobot_hand_lmk_detection` and the subscribed message topic as `hobot_hand_detection`.

<Tabs groupId="tros-distro">
<TabItem value="foxy" label="Foxy">

```shell
source /opt/tros/local_setup.bash
```

</TabItem>

<TabItem value="humble" label="Humble">

```shell
source /opt/tros/humble/local_setup.bash
```

</TabItem>

</Tabs>

```shell
# Copy the required configuration files for running the example from the tros.b installation path.
cp -r /opt/tros/${TROS_DISTRO}/lib/hand_lmk_detection/config/ .
# Launch the Node
ros2 run hand_lmk_detection hand_lmk_detection --ros-args --log-level warn --ros-args -p ai_msg_pub_topic_name:=hobot_hand_lmk_detection -p ai_msg_sub_topic_name:=hobot_hand_detection
```

#### 4 View the Algorithm Inference Output

Open a terminal on the RDK and use ROS2 commands to inspect the topic messages published by the inference Node.

**View hand bounding box detection messages published by the hand detection algorithm Node**

Query command:

<Tabs groupId="tros-distro">
<TabItem value="foxy" label="Foxy">

```shell
source /opt/tros/local_setup.bash
```

</TabItem>

<TabItem value="humble" label="Humble">

```shell
source /opt/tros/humble/local_setup.bash
```

</TabItem>

</Tabs>

```shell
# Launch the Node
ros2 topic echo /hobot_hand_detection
```

Output:

```
header:
  stamp:
    sec: 1660034025
    nanosec: 429969208
  frame_id: '8049'
fps: 30
targets:
- type: person
  track_id: 10
  rois:
  - type: hand
    rect:
      x_offset: 619
      y_offset: 128
      height: 229
      width: 168
      do_rectify: false
  attributes: []
  points: []
  captures: []
disappeared_targets: []
```

As shown above, the message published by the hand bounding box detection algorithm Node contains one hand detection result (with roi type "hand").

**View hand keypoint detection messages published by the hand keypoint detection algorithm Node**

Query command:

<Tabs groupId="tros-distro">
<TabItem value="foxy" label="Foxy">

```shell
source /opt/tros/local_setup.bash
```

</TabItem>

<TabItem value="humble" label="Humble">

```shell
source /opt/tros/humble/local_setup.bash
```

</TabItem>

</Tabs>

```shell
# Launch the Node
ros2 topic echo /hobot_hand_lmk_detection
```

Output:

```
header:
  stamp:
    sec: 1660034025
    nanosec: 429969208
  frame_id: '8049'
fps: 30
targets:
- type: person
  track_id: 10
  rois:
  - type: hand
    rect:
      x_offset: 619
      y_offset: 128
      height: 229
      width: 168
      do_rectify: false
  attributes: []
  points:
  - type: hand_kps
    point:
    - x: 715.2421875
      y: 348.0546875
      z: 0.0
    - x: 673.4921875
      y: 315.8515625
      z: 0.0
    - x: 655.2265625
      y: 294.3828125
      z: 0.0
    - x: 639.5703125
      y: 262.1796875
      z: 0.0
    - x: 621.3046875
      y: 229.9765625
      z: 0.0
    - x: 686.5390625
      y: 247.8671875
      z: 0.0
    - x: 683.9296875
      y: 201.3515625
      z: 0.0
    - x: 683.9296875
      y: 176.3046875
      z: 0.0
    - x: 681.3203125
      y: 147.6796875
      z: 0.0
    - x: 712.6328125
      y: 240.7109375
      z: 0.0
    - x: 717.8515625
      y: 194.1953125
      z: 0.0
    - x: 720.4609375
      y: 161.9921875
      z: 0.0
    - x: 723.0703125
      y: 129.7890625
      z: 0.0
    - x: 736.1171875
      y: 247.8671875
      z: 0.0
    - x: 743.9453125
      y: 201.3515625
      z: 0.0
    - x: 749.1640625
      y: 172.7265625
      z: 0.0
    - x: 749.1640625
      y: 140.5234375
      z: 0.0
    - x: 759.6015625
      y: 262.1796875
      z: 0.0
    - x: 770.0390625
      y: 226.3984375
      z: 0.0
    - x: 775.2578125
      y: 204.9296875
      z: 0.0
    - x: 775.2578125
      y: 179.8828125
      z: 0.0
    confidence: []
  captures: []
disappeared_targets: []
```

As can be seen, after the hand keypoint detection algorithm Node subscribes to the hand detection bounding box message and uses it for inference, the published message contains both a hand bounding box and hand keypoint detection results (`roi` type is `hand`, `points` type is `hand_kps`). The content of the published hand bounding box message originates from the subscribed hand bounding box message and matches the data queried in the previous step.

#### 5 Graph Formed by Chained Nodes

Open a terminal on the RDK and use ROS2 commands to inspect the running Nodes and Topics:

<Tabs groupId="tros-distro">
<TabItem value="foxy" label="Foxy">

```shell
root@ubuntu:~# source /opt/tros/local_setup.bash
```

</TabItem>

<TabItem value="humble" label="Humble">

```shell
root@ubuntu:~# source /opt/tros/humble/local_setup.bash
```

</TabItem>

</Tabs>

```shell
# Query Node information
root@ubuntu:~# ros2 node list
/hand_lmk_det
/mipi_cam
/mono2d_body_det
# Query Topic information
root@ubuntu:~# ros2 topic list
/hbmem_img08172824022201080202012021072315
/hobot_hand_detection
/hobot_hand_lmk_detection
/image_raw
/parameter_events
/rosout
```

We can see that three Nodes are running on the RDK.

On a PC (**the PC must be on the same network segment as the RDK**), you can use the Node Graph feature in `rqt` to visualize the Nodes running on the RDK, their published/subscribed topics, and the graph formed by these Nodes via Topics, as shown in the figure below:

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/05_tros_dev/image/ai_predict/rosgraph_handlmk.jpg)

Elliptical boxes represent Node names, and rectangular boxes represent Topic names. As shown, the entire graph consists of three Nodes and two Topics.

- `mipi_cam` (sensor Node) acts as the starting point, capturing images from the camera and publishing them.
- `mono2d_body_det` (perception algorithm Node) serves as an intermediate node, subscribing to image data published by the `mipi_cam` Node to perform hand bounding box detection.
- `hand_lmk_det` (perception algorithm Node) acts as the endpoint, subscribing to both the image data from `mipi_cam` and the hand detection results from `mono2d_body_det` to perform hand keypoint detection.

### Summary of This Section

This section introduced how to use sensor Nodes and algorithm Nodes for hand detection and hand keypoint detection from `tros.b` on the RDK. By leveraging ROS2 Topic-based communication, two perception algorithm Nodes are chained together to implement a pipeline: capturing images from a camera, performing inference through algorithms, and publishing detected hand keypoint messages.

Based on the algorithm chaining principle described in this section, users can chain additional algorithm Nodes on the RDK to develop rich and functional robotic algorithm applications.