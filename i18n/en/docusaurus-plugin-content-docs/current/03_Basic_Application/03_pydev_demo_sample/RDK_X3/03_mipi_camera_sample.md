---
sidebar_position: 9
---

# MIPI Camera Sample

## Introduction

The MIPI camera sample is a **Python API** example under `/app/pydev_demo/03_mipi_camera_sample` that shows how to use the on-board MIPI camera for real-time object detection. It runs an FCOS model on the live stream and draws results on HDMI while printing boxes and FPS.

## Demo

![output-img](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/pydev_03_runing.png)

## Hardware setup

### Connections
1. One RDK board  
2. Official compatible MIPI camera  
3. HDMI cable to a display  
4. Power and Ethernet  

![connect-img](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/pydev_03_hw_connect.png)

## Quick start

### Code location on device

The sample file is `/app/pydev_demo/03_mipi_camera_sample/mipi_camera.py`.

### Build and run
Python samples do not require compilation; run directly.

### Sample output

After start, the program initializes the MIPI camera and HDMI, runs real-time detection, draws on HDMI, and prints detections and FPS in the console.

## Details

### Command-line options
No arguments are required; the on-board MIPI camera is detected automatically.

### Software architecture

1. Load the FCOS model  
2. Initialize the MIPI camera with `srcampy.Camera()`  
3. Initialize HDMI display  
4. Bind camera output to display  
5. Real-time loop: grab frame → infer → post-process → draw → print FPS  

<center>
![software_arch](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/pydev_03_mipi_camera_sample_software_arch.png)
</center>

### API flow

<center>
![API_Flow](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/pydev_03_mipi_camera_sample_api_flow.png)
</center>

### FAQ

**Q:** Camera initialization fails.\
**A:** Check the MIPI connection and drivers; reboot if needed.

**Q:** No or bad HDMI output.\
**A:** Check HDMI and stop the desktop manager if required (`systemctl stop lightdm`).

**Q:** How to change the score threshold?\
**A:** Edit `fcos_postprocess_info.score_threshold` (e.g. `0.5`).

**Q:** How to change display resolution?\
**A:** Adjust `sensor_width` / `sensor_height` if the monitor supports it.

**Q:** Low FPS.\
**A:** Try a lighter model or lower capture resolution.

**Q:** How to save result images?\
**A:** Add saving logic such as `cv2.imwrite()`.

**Q:** How to add classes?\
**A:** Update `get_classes()` and use a model trained for those classes.

