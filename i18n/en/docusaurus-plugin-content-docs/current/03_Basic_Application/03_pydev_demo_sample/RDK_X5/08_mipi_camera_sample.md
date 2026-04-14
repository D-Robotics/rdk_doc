---
sidebar_position: 10
---

# MIPI Camera Real-time Detection

## Example Introduction
The MIPI camera real-time detection example is a **Python interface** development code example located in `/app/pydev_demo/08_mipi_camera_sample`. It demonstrates how to use the onboard MIPI camera for real-time object detection. This example uses the YOLOv5x object detection model to perform real-time inference on the video stream captured by the MIPI camera, and displays the detection results via HDMI, outputting bounding box information.

Included examples:
```
root@ubuntu:/app/pydev_demo/08_mipi_camera_sample$ tree
.
├── 01_mipi_camera_yolov5s.py
├── 02_mipi_camera_dump.py
├── 03_mipi_camera_scale.py
├── 04_mipi_camera_crop_scale.py
├── 05_mipi_camera_streamer.py
└── coco_classes.names
```

## Effect Demonstration

### 01 Real-time Detection Effect

:::info Visualization of Detection Results

To view the real-time camera feed and visualization of detection results on a display, you need to:

1. **Connect an external display**: Connect the development board to a monitor using an HDMI cable
2. **Special handling for Desktop version**: If using the Desktop version system, first execute the following command to stop the desktop service:
   ```bash
   sudo systemctl stop lightdm
   ```
3. **Remote connection**: Connect to the board via SSH
4. **Run the code**: After executing the example program, you will see the real-time detection results on the connected display

:::

![output-img](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/RDK_X5_Newly/pydev_08_mipi_camera_yolov5s_runing.jpg)

### 02 Image Capture and Save Effect

After running, multiple YUV format image files will be saved in the same directory as the script, with a default resolution of 1920x1080.

![output-img](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/RDK_X5_Newly/pydev_08_mipi_camera_dump.png)

### 03 Image Scaling Effect

After running, scaled YUV image files will be saved in the same directory as the script, with a default resolution of 640x360.

![output-img](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/RDK_X5_Newly/pydev_08_mipi_camera_scale.png)

### 04 Image Cropping and Scaling Effect

After running, cropped and scaled YUV image files (NV12 format) will be saved in the same directory as the script. By default, the center of the image is cropped and scaled. Adjusting the cropping position yields the following YUV image.

![output-img](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/RDK_X5_Newly/pydev_08_mipi_camera_crop_scale.png)

### 05 Real-time Streaming Effect

After running, the camera feed is displayed in real-time on the HDMI screen (streaming test). Note that for the Desktop version, you need to first execute `sudo systemctl stop lightdm` to stop the desktop service.

![output-img](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/RDK_X5_Newly/pydev_08_mipi_camera_stream.gif)

## Hardware Preparation

### Hardware Connection
1. Prepare an RDK development board
2. Connect the officially adapted MIPI camera
3. Connect the monitor and development board via an HDMI cable
4. Connect the power cable and network cable

![connect-img](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/RDK_X5_Newly/pydev_03_hw_connect.png)

## Quick Start

### Code and Board Location
The example files are located at `/app/pydev_demo/08_mipi_camera_sample`

### Compilation and Execution
Python examples do not require compilation; they can be run directly:

Running 01_mipi_camera_yolov5s.py:

```
cd /app/pydev_demo/08_mipi_camera_sample
python 01_mipi_camera_yolov5s.py
```

Running 02_mipi_camera_dump.py:

```
cd /app/pydev_demo/08_mipi_camera_sample
python 02_mipi_camera_dump.py -f 30 -c 10 -w 1920 -h 1080
```

Running 03_mipi_camera_scale.py:

```
cd /app/pydev_demo/08_mipi_camera_sample

# This example requires input.yuv as input. Here we use output0.yuv from the previous example as input, execute the copy command
cp output0.yuv input.yuv

# Then run the example
python 03_mipi_camera_scale.py -i input.yuv -o output_640x360.yuv -w 640 -h 360 --iwidth 1920 --iheight 1080
```

Running 04_mipi_camera_crop_scale.py:

```
cd /app/pydev_demo/08_mipi_camera_sample
python 04_mipi_camera_crop_scale.py -i input.yuv -o output_640x480.yuv -w 640 -h 480 --iwidth 1920 --iheight 1080 -x 304 -y 304 --crop_w 896 --crop_h 592
```

Running 05_mipi_camera_streamer.py:
```
cd /app/pydev_demo/08_mipi_camera_sample
python 05_mipi_camera_streamer.py -w 1920 -h 1080
```

<!-- ### Execution Effect
Execution effect of 01_mipi_camera_yolov5s.py:
After running, the program initializes the MIPI camera and HDMI display, and starts real-time object detection. The detection results are displayed via HDMI.

Execution effect of 02_mipi_camera_dump.py:
After successful execution, multiple yuv files will be stored in the script's directory.

Execution effect of 03_mipi_camera_scale.py:
After successful execution, scaled yuv files will be stored in the script's directory.

Execution effect of 04_mipi_camera_crop_scale.py:
After successful execution, scaled yuv files will be stored in the script's directory. Note: The crop width must be an integer multiple of 16 (i.e., aligned to 16 bytes).

Execution effect of 05_mipi_camera_streamer.py:
After successful execution, the screen will display the real-time feed. -->

## Detailed Introduction

### Example Program Parameter Options
#### Parameter Description for 01_mipi_camera_yolov5s.py Example

The MIPI camera real-time detection example does not require command-line parameters; just run it directly. The program will automatically detect and use the onboard MIPI camera.

#### Parameter Description for 02_mipi_camera_dump.py Example

| Parameter | Description | Type | Example |
|-----------|-------------|------|---------|
| `-f` | Frame rate (FPS) | int | `30` |
| `-c` | Number of frames to capture (count) | int | `10` |
| `-w` | Image width | int | `1920` |
| `-h` | Image height | int | `1080` |

#### Parameter Description for 03_mipi_camera_scale.py Example

| Parameter | Description | Type | Example |
|-----------|-------------|------|---------|
| `-i` | Input YUV file path | str | `input.yuv` |
| `-o` | Output file path | str | `output_scale.yuv` |
| `-w` | Output image width | int | `640` |
| `-h` | Output image height | int | `360` |
| `--iwidth` | Input image width | int | `1920` |
| `--iheight` | Input image height | int | `1080` |

#### Parameter Description for 04_mipi_camera_crop_scale.py Example

| Parameter | Description | Type | Example |
|-----------|-------------|------|---------|
| `-i` | Input YUV file path | str | `input.yuv` |
| `-o` | Output file path | str | `output_crop_scale.yuv` |
| `-w` | Output image width | int | `640` |
| `-h` | Output image height | int | `480` |
| `--iwidth` | Original input image width | int | `1920` |
| `--iheight` | Original input image height | int | `1080` |
| `-x` | X coordinate of the top-left corner of the crop area | int | `304` |
| `-y` | Y coordinate of the top-left corner of the crop area | int | `304` |
| `--crop_w` | Width of the crop area | int | `896` |
| `--crop_h` | Height of the crop area | int | `592` |

#### Parameter Description for 05_mipi_camera_streamer.py Example

| Parameter | Description | Type | Example |
|-----------|-------------|------|---------|
| `-w` | Output image width | int | `1920` |
| `-h` | Output image height | int | `1080` |

### Software Architecture Description

This section describes the software architecture and workflow of the MIPI camera real-time detection examples, explaining the complete execution process of each example program from initialization to completion, helping to understand the overall code structure and data flow.

#### Real-time Object Detection Example Software Architecture

<center>
![software_arch](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/RDK_X5_Newly/pydev_10_mipi_sample_software_arch1.png)
</center>

1. **Model Loading** - Load the model file using the `hbm_runtime` module
2. **Camera Initialization** - Initialize the MIPI camera
3. **Display Initialization** - Initialize the HDMI display
4. **Device Binding** - Bind the camera output to the display
5. **Image Capture** - Obtain video frames from the MIPI camera
6. **Image Preprocessing** - Scale the image to the model input size, format conversion
7. **Model Inference** - Perform YOLOv5x forward inference on the BPU
8. **Result Post-processing** - Decode output, filter low-confidence results, NMS deduplication, coordinate mapping
9. **Result Visualization** - Draw bounding boxes and labels on the original image
10. **Display Output** - Output results via HDMI, display FPS and detection information on the console

#### Image Capture and Save Example Software Architecture

<center>
![software_arch](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/RDK_X5_Newly/pydev_10_mipi_sample_software_arch2.png)
</center>

1. **Camera Initialization** - Initialize the MIPI camera
2. **Parameter Configuration** - Set capture frame rate, resolution, number of frames to capture
3. **Image Capture** - Continuously capture the specified number of image frames
4. **File Saving** - Save captured images as YUV format files

#### Image Scaling Example Software Architecture

<center>
![software_arch](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/RDK_X5_Newly/pydev_10_mipi_sample_software_arch3.png)
</center>

1. **Parameter Retrieval** - Parse command-line parameters to get input/output file paths and image dimensions
2. **VPS Initialization** - Create a VPS object, open a hardware scaling channel
3. **File Reading** - Read the input YUV file
4. **Hardware Scaling** - Complete image scaling via hardware VPS
5. **Result Saving** - Save the scaled image as a new YUV file
6. **Resource Cleanup** - Close VPS, release hardware resources

#### Image Cropping and Scaling Example Software Architecture

<center>
![software_arch](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/RDK_X5_Newly/pydev_10_mipi_sample_software_arch4.png)
</center>

1. **Parameter Retrieval** - Parse command-line parameters to get input/output file paths, image dimensions, and crop area coordinates
2. **VPS Initialization** - Create a VPS object, open a hardware cropping and scaling channel
3. **File Reading** - Read the input YUV file
4. **Hardware Processing** - Complete image cropping via hardware VPS
5. **Result Saving** - Save the processed image as a new YUV file
6. **Resource Cleanup** - Close VPS, release hardware resources

#### Real-time Streaming Display Example Software Architecture

<center>
![software_arch](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/RDK_X5_Newly/pydev_10_mipi_sample_software_arch5.png)
</center>

1. **Parameter Retrieval** - Parse command-line parameters to get display resolution
2. **Display Initialization** - Create a display object, initialize the HDMI display layer
3. **Camera Initialization** - Create a camera object, open the MIPI camera
4. **Device Binding** - Use hardware binding to directly connect the camera data stream to the display
5. **Real-time Streaming** - The camera feed is continuously output to the HDMI display via the hardware path
6. **Device Unbinding** - Unbind the camera from the display
7. **Resource Cleanup** - Close the display and camera, release hardware resources

### API Flow Description

This section lists the main API interfaces used in the example programs, describing the functionality, input parameters, and return values of each interface, helping developers quickly understand code implementation details and interface calling methods.

#### Main Interfaces for Real-time Object Detection:

<center>
![API_Flow](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/RDK_X5_Newly/pydev_10_01_flow1.png)
</center>

1. `srcampy.Camera()`

   Create a MIPI camera object

2. `get_display_res()`

   Get the HDMI display resolution, returns: width, height

3. `cam.open_cam(pipe_id, video_index, fps, width, height, raw_height, raw_width)`
   
   Open the camera, inputs: pipeline channel number corresponding to the camera, host number corresponding to the camera, frame rate, output width, output height, raw width, raw height

4. `srcampy.Display()`

   Create an HDMI display object

5. `disp.display(layer, width, height)`

   Initialize the display layer, inputs: display layer number, width, height

6. `srcampy.bind(camera, display)`

   Bind the camera and display, inputs: camera object, display object

7. `hbm_runtime.HB_HBMRuntime(model_path)`

   Load the model, input: model file path

8. `model.set_scheduling_params(priority, bpu_cores)`

   Set model scheduling parameters, inputs: priority (0-255), list of BPU cores
<!-- 9. `common.print_model_info(model)` - Print model information (input/output dimensions, quantization parameters, etc.) -->

9. `load_class_names(class_file)`

   Load class names, input: class file path, returns: list of class names

10. `cam.get_img(chn, width, height)`

    Get a camera image frame, inputs: channel number (default is 2), width, height, returns: NV12 format image data

11. `split_nv12_bytes(img, width, height)`

    Split the Y and UV components of an NV12 image, inputs: NV12 image data, width, height, returns: Y component, UV component

12. `resize_nv12_yuv(y, uv, target_w, target_h)`

    Scale an NV12 image, inputs: Y component, UV component, target width, target height, returns: scaled Y and UV components

13. `model.run(input_tensor)`

    Perform model inference, input: preprocessed input tensor dictionary, returns: model output dictionary

14. `dequantize_outputs(outputs, output_quants)`

    Dequantize the results, inputs: model output dictionary, output quantization parameters, returns: float32 type data

15. `decode_outputs(output_names, fp32_outputs, strides, anchors, classes_num)`

    Decode YOLO model outputs, inputs: list of output names, dequantized outputs, strides, anchors, number of classes, returns: predictions

16. `filter_predictions(predictions, score_threshold)`

    Filter predictions, inputs: predictions, confidence threshold, returns: bounding boxes, confidences, classes

17. `NMS(boxes, scores, classes, iou_threshold)`

    Perform non-maximum suppression, inputs: bounding boxes, confidences, classes, IoU threshold, returns: kept indices

18. `scale_coords_back(boxes, orig_w, orig_h, model_w, model_h, resize_type)`

    Scale bounding boxes back to original image dimensions, inputs: bounding boxes, original width/height, model input width/height, resize type, returns: scaled bounding boxes

19. `draw_detections_on_disp(display, boxes, cls_ids, scores, class_names, color_map, chn)`

    Draw detection results on the display layer, inputs: display object, bounding boxes, class IDs, confidences, class list, color map, channel number

20. `srcampy.unbind(camera, display)`
    
    Unbind the camera and display

21. `cam.close_cam()`
    
    Close the camera

22. `disp.close()`

    Close the display

#### Main Interfaces for Image Capture and Save:

<center>
![API_Flow](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/RDK_X5_Newly/pydev_10_01_flow2.png)
</center>

1. `libsrcampy.Camera()`

   Create a MIPI camera object

2. `cam.open_cam(pipe_id, video_index, fps, width, height,)`

   Open the camera, inputs: pipeline channel number corresponding to the camera, host number corresponding to the camera, frame rate, width, height

3. `cam.get_img(chn)`

   Get an image, input: module for image acquisition, returns: YUV format image data

4. `file.write(data)`

   Write file data, input: image data

5. `cam.close_cam()`

   Close the camera

#### Main Interfaces for Image Scaling:

<center>
![API_Flow](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/RDK_X5_Newly/pydev_10_01_flow3.png)
</center>

1. `libsrcampy.Camera()`

   Create a VPS (Video Processing System) object

2. `vps.open_vps(grp_id, chn_id, input_w, input_h, output_w, output_h)`

   Open a VPS channel, inputs: group ID, channel ID, input width/height, output width/height

3. `file.read()`

   Read file data, returns: image data (byte stream)

4. `vps.set_img(img_data)`

   Set input image data, input: YUV image data (byte stream)

5. `vps.get_img(chn, width, height)`

   Get the processed image, inputs: channel number, width, height, returns: processed YUV image data (byte stream)

6. `file.write(data)`

   Write the processed image data, input: image data (byte stream)

7. `vps.close_cam()`

   Close VPS

#### Main Interfaces for Image Cropping and Scaling:

<center>
![API_Flow](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/RDK_X5_Newly/pydev_10_01_flow4.png)
</center>

1. `libsrcampy.Camera()`

   Create a VPS (Video Processing System) object

2. `vps.open_vps(grp_id, chn_id, input_w, input_h, output_w, output_h, crop_rect)`

   Open a VPS channel and set the crop area, inputs: group ID, channel ID, input width/height, output width/height, crop area [x, y, w, h]

3. `file.read()`

   Read file data, returns: image data (byte stream)

4. `vps.set_img(img_data)`

   Set input image data, input: YUV image data (byte stream)

5. `vps.get_img(chn, width, height)`

   Get the processed image, inputs: channel number, width, height, returns: processed YUV image data (NV12 format, byte stream)

6. `file.write(data)`

   Write the processed image data, input: image data (byte stream)

7. `vps.close_cam()`

   Close VPS

#### Main Interfaces for Real-time Streaming Display:

<center>
![API_Flow](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/RDK_X5_Newly/pydev_10_01_flow5.png)
</center>

1. `libsrcampy.Display()`

   Create an HDMI display object

2. `disp.display(layer, width, height)`

   Initialize the display layer, inputs: display layer number, width, height

3. `libsrcampy.Camera()`

   Create a MIPI camera object

4. `cam.open_cam(pipe_id, video_index, fps, width, height,)`

   Open the camera, inputs: pipeline channel number corresponding to the camera, host number corresponding to the camera, frame rate, width, height

5. `libsrcampy.bind(camera, display)`

   Bind the camera and display, inputs: camera object, display object, returns: binding result

6. `libsrcampy.unbind(camera, display)`

   Unbind the camera and display

7. `disp.close()`

   Close the display

8. `cam.close_cam()`

   Close the camera

### FAQ

Q: What should I do if the example prompts camera initialization failure?  
A: Please check if the MIPI camera is properly connected and ensure the camera driver is loaded correctly. Try restarting the device.

Q: What should I do if the HDMI display is abnormal or has no output?  
A: Please check the HDMI connection and ensure the display service has been stopped (e.g., using systemctl stop lightdm).

Q: How can I adjust the detection threshold?  
A: Modify the value of `--score-thres` in the code; for example, changing it to 0.5 can increase detection sensitivity.

Q: How can I change the display resolution?  
A: Modify the `sensor_width` and `sensor_height` variables in the code, but note whether the display device supports that resolution.

Q: What should I do if the frame rate is very low when running the example?  
A: Try using a lighter model or adjust the camera's capture resolution.

Q: How can I save the detection result images?  
A: You can add image saving logic to the code, such as using `cv2.imwrite()` to save the processed image.

<!-- Q: How can I add new detection categories?  
A: You need to modify the `get_classes()` function to add new category names, and retrain the model or use a model that supports the new categories. -->