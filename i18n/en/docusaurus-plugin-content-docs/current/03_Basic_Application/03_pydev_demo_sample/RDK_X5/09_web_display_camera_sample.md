---
sidebar_position: 11
---

# Web Display Camera

## Example Introduction
The Web Display Camera example is a **Python interface** development code example located in `/app/pydev_demo/09_web_display_camera_sample/`. It is used to demonstrate how to use the onboard MIPI camera for real-time object detection and push the detection results to a web browser via WebSocket for real-time display. This example uses the YOLOv5x object detection model to perform real-time inference on the video stream captured by the MIPI camera and pushes the image and detection box information to the web client via the WebSocket protocol.

## Effect Demonstration
After running, access the development board's IP address via a browser to view the camera feed and detection results in real time on the web page.

![output-img](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/RDK_X5_Newly/pydev_11_running.png)

## Hardware Preparation

### Hardware Connection
1. Prepare an RDK development board.
2. Connect the officially adapted MIPI camera.
3. Connect an Ethernet cable to the development board.
4. Connect the power cable.
![connect-img](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/RDK_X5_Newly/pydev_05_hw_connect.png)

## Quick Start

### Code Location on the Board
Navigate to `/app/pydev_demo/09_web_display_camera_sample/` to see that the Web Display Camera example includes the following files:
```
root@ubuntu:/app/pydev_demo/09_web_display_camera_sample# tree
.
├── coco_classes.names
├── mipi_camera_web_yolov5x.py
├── start_nginx.sh
├── webservice
│   ├── client_body_temp
│   ├── conf
│   │   ├── nginx.conf
│   │   ├── ...
│   ├── html
│   │   ├── assets
│   │   ├── index.html
│   │   └── ...
│   ├── logs
│   ├── proxy_temp
│   └── sbin
└── x3_pb2.py
```

### Compilation and Execution
First, start the Nginx server, then run the Python script:

:::info Note

When using the IMX477 camera, you need to set `fps=50` in the `mipi_camera_web.py` file (around line 39).

:::

```bash
# Navigate to the example directory
cd /app/pydev_demo/09_web_display_camera_sample/

# Start the Nginx server
cd webservice/
./sbin/nginx -p .

# (If ./sbin/nginx does not have execute permission, add it using the command below)
# chmod +x ./sbin/nginx

# Run the Web camera example
# Return to the example directory
cd ..
python mipi_camera_web_yolov5x.py
```

### Execution Effect

After running, the program starts a web service. You can access the development board's IP address via a browser to view the real-time video stream and object detection results.

In the browser, visit http://development-board-IP, defaulting to http://192.168.127.10.
![pydev_05_wb_disp_web_img](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/RDK_X5_Newly/pydev_05_wb_disp_web_img.png)

Click on `web Display` shown in the browser to see the real-time video stream and object detection results. The effect can be seen in the [Effect Demonstration section](#effect-demonstration) at the beginning of the article.

## Detailed Description

### Example Program Parameter Options

This example supports command-line parameter configuration. If no parameters are specified, the program will automatically load the test image in the same directory for inference using default values.

| Parameter | Description | Type | Default Value |
|-----------|-------------|------|----------------|
| `--model-path` | BPU quantized model path (`.bin`) | str | `/app/model/basic/yolov5x_672x672_nv12.bin` |
| `--priority` | Inference priority (0~255, 255 is highest) | int | `0` |
| `--bpu-cores` | List of BPU core indices (e.g., `0 1`) | int list | `[0]` |
| `--label-file` | Class label file path | str | `coco_classes.names` |
| `--nms-thres` | IoU threshold for Non-Maximum Suppression | float | `0.45` |
| `--score-thres` | Detection confidence threshold | float | `0.25` |

### Software Architecture Description

This section introduces the software architecture and workflow of the Web Display Camera example, describing the complete execution process from initialization to web push, helping to understand the overall code structure and data flow.

<center>
![software_arch](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/RDK_X5_Newly/pydev_05_wb_disp_cam_software_arch.png)
</center>

The Web Display Camera example involves not only the `mipi_camera_web.py` source code but also the Nginx server and web receiver. Therefore, the software architecture is slightly more complex than previous examples and includes the following core components:

1. **MIPI Camera Capture** - Use `srcampy.Camera()` to initialize the MIPI camera and capture the video stream.

2. **Object Detection Model** - Load the YOLOv5x object detection model to perform real-time inference on video frames.

3. **Video Encoding** - Use `srcampy.Encoder()` to encode video frames into JPEG format.

4. **WebSocket Service** - Create a WebSocket server to push video frames and detection results in real time.

5. **Web Frontend** - Provide HTML pages and JavaScript code to display video and detection results in the browser.

6. **Nginx Server** - Provide static file serving and HTTP proxy functionality.

### API Process Description
This section lists the main API interfaces used in the example program, explaining the functionality, input parameters, and return values of each interface to help developers quickly understand the code implementation details and interface invocation methods.

<center>
![API_Flow](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/RDK_X5_Newly/pydev_11_01_flow.png)
</center>

1. `load_class_names(class_file)`

   Load class names. Input: class file path. Returns: list of class names.

2. `srcampy.Camera()`

   Create a MIPI camera object.

3. `cam.open_cam(pipe_id, video_index, fps, width_list, height_list)`

   Open the camera. Input: channel ID, video index, frame rate, width list [512, 1920], height list [512, 1072].

4. `hbm_runtime.HB_HBMRuntime(model_path)`

   Load the YOLOv5x model. Input: model file path.

5. `WebSocketServer(host, port, model_handler, camera_manager, detection_classes)`

   Create a WebSocket server instance. Input: host address, port number, model handler, camera manager, class list.

6. `websockets.serve(handler, host, port)`

   Start the WebSocket server. Input: handler function, host address (0.0.0.0), port number (8080).

7. `cam.get_img(chn, width, height)`

   Capture the raw camera image. Input: channel ID (2), width (1920), height (1072). Returns: NV12 format image data.

8. `split_nv12_bytes(img, width, height)`

   Split the NV12 image into Y and UV components. Input: NV12 image data, width (1920), height (1072). Returns: Y component, UV component.

9. `resize_nv12_yuv(y, uv, target_w, target_h)`

   Resize the NV12 image. Input: Y component, UV component, target width (672), target height (672). Returns: resized Y and UV components.

10. `model.run(input_tensor)`

    Perform model inference. Input: preprocessed input tensor dictionary. Returns: model output dictionary.

11. `dequantize_outputs(outputs, output_quants)`

    Dequantize the outputs. Input: model output dictionary, output quantization parameters. Returns: float32 type data.

12. `decode_outputs(output_names, fp32_outputs, strides, anchors, classes_num)`

    Decode the YOLO model outputs. Input: list of output names, dequantized outputs, strides, anchors, number of classes (80). Returns: prediction results.

13. `filter_predictions(predictions, score_threshold)`

    Filter prediction results. Input: prediction results, confidence threshold. Returns: detection boxes, confidences, classes.

14. `NMS(boxes, scores, classes, iou_threshold)`

    Perform Non-Maximum Suppression. Input: detection boxes, confidences, classes, IoU threshold. Returns: retained indices.

15. `scale_coords_back(boxes, orig_w, orig_h, model_w, model_h, resize_type)`

    Scale detection boxes back to the original image size. Input: detection boxes, original image width (1920), original image height (1072), model input width (672), model input height (672), resize type. Returns: scaled detection boxes.

16. `serialize_message(frame_message, data, detection_classes)`

    Serialize detection results and images into a Protocol Buffer message. Input: frame message object, list of detection results, list of class names. Returns: serialized byte stream.

17. `websocket.send(serialized_buf)`

    Send a message to the client. Input: serialized data (byte stream).

18. `cam.close_cam()`

    Close the camera.

### FAQ
Q: What should I do if the example reports a port already in use?  
A: Check if another program is using port 8080. Use the `netstat -tlnp` command to check port usage.

Q: What should I do if the browser cannot access the video stream?  
A: Check the network connection of the development board and ensure that the firewall is not blocking access to the relevant ports.

Q: What should I do if the video stream latency is high?  
A: Try lowering the video resolution or frame rate, or use a lighter object detection model.

Q: How can I modify the web frontend interface?  
A: You can modify the HTML, CSS, and JavaScript files in the `webservice/html/` directory to customize the interface.

Q: How can I add new detection classes?  
A: Modify the `get_classes()` function to add new class names, and retrain the model or use a model that supports the new classes.

Q: How can I save the video stream?  
A: Add video saving logic to the code, for example, using OpenCV's `VideoWriter` class to save video files.