---
sidebar_position: 5
---

# 4.1.5 Inference Based on USB Camera {#usb}

## Object Detection Algorithm - FCOS

This example mainly implements the following functions:

1. Load the `fcos` object detection algorithm model (trained on the COCO dataset with 80 object categories).
2. Read the video stream from the USB camera and perform inference.
3. Parse the model output and render the results to the original video stream.
4. Output the rendered video stream via the `HDMI` interface.

### How to Run

Please refer to [USB Camera AI Inference](/first_application/usb_camera) for instructions on how to quickly run this example.

### Code Analysis
- Import algorithm inference module `hobot_dnn`, video output module `hobot_vio`, `numpy`, `opencv`, `colorsys`, etc.

    ```
    from hobot_dnn import pyeasy_dnn as dnn
    from hobot_vio import libsrcampy as srcampy
    import numpy as np
    import cv2
    import colorsys
    ```

- Load model files

    Call the `load` method to load the model files and return a list of `hobot_dnn.pyeasy_dnn.Model` class.

    ```shell
    models = dnn.load('../models/fcos_512x512_nv12.bin')
    ```

    The input of the `fcos` model is `1x3x512x512` data in `NCHW` format. The output consists of 15 groups of data that represent the detected object bounding boxes. The example defines the `print_properties` function to output the input and output parameters of the model:

    ```python
    # print properties of input tensor
    print_properties(models[0].inputs[0].properties)
    # print properties of output tensor
    print(len(models[0].outputs))
    for output in models[0].outputs:
        print_properties(output.properties)
    ```

- Data preprocessing

Use OpenCV to open the USB camera device node `/dev/video8`, get real-time images, and resize the images to fit the input tensor size of the model.

```python
# open usb camera: /dev/video8
cap = cv2.VideoCapture(8)
if(not cap.isOpened()):
    exit(-1)
print("Open usb camera successfully")
# set the output of usb camera to MJPEG, solution 640 x 480
codec = cv2.VideoWriter_fourcc( 'M', 'J', 'P', 'G' )
cap.set(cv2.CAP_PROP_FOURCC, codec)
cap.set(cv2.CAP_PROP_FPS, 30) 
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
```

Then convert the BGR format image to NV12 format that fits the model input.

```python
nv12_data = bgr2nv12_opencv(resized_data)
```

- Model Inference

    Call the `forward` interface of the [Model](../pydev_dnn_api#model) class for inference. The model will output 15 sets of data representing the detected object bounding boxes.

    ```python
    outputs = models[0].forward(nv12_data)
    ```

- Post-processing

    The post-processing function `postprocess` in the example will process the object category, bounding box, and confidence information output by the model.

    ```python
    prediction_bbox = postprocess(outputs, input_shape, origin_img_shape=(1080,1920))
    ```

- Visualize the Detection Results

    The example renders the algorithm results and the original video stream, and outputs them through the `HDMI` interface for real-time preview on a monitor. The Display function of the hobot_vio module is used for displaying. For more information about this module, please refer to the [Display section](../pydev_multimedia_api_x3/object_display.md).

    ```python
    # create display object
    disp = srcampy.Display()
    # set solution to 1920 x 1080
    disp.display(0, 1920, 1080)

    # if the solution of image is not 1920 x 1080, do resize
    if frame.shape[0]!=1080 and frame.shape[1]!=1920:
        frame = cv2.resize(frame, (1920,1080), interpolation=cv2.INTER_AREA)

    # render the detection results to image
    box_bgr = draw_bboxs(frame, prediction_bbox)

    # convert BGR to NV12
    box_nv12 = bgr2nv12_opencv(box_bgr)
    # do display
    disp.set_img(box_nv12.tobytes())
    ```

