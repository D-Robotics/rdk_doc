---
sidebar_position: 10
---

# Web Display Camera Sample

## Introduction

The Web display camera sample under `/app/pydev_demo/05_web_display_camera_sample/` is a **Python API** example that streams MIPI camera video and detection results to a browser over a Web service. It combines MIPI capture, object detection, video encoding, and WebSocket delivery.

## Demo

![output-img](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/pydev_05_runing.png)

## Hardware setup

### Connections
1. RDK board  
2. Official MIPI camera  
3. Ethernet  
4. Power  

![connect-img](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/pydev_05_hw_connect.png)

## Quick start

### Code location on device

```
root@ubuntu:/app/pydev_demo/05_web_display_camera_sample# tree
.
├── mipi_camera_web.py
├── start_nginx.sh
├── webservice
│ ├── conf
│ │ ├── nginx.conf
│ │ └── ...
│ ├── html
│ │ ├── index.html
│ │ ├── assets
│ │ └── ...
│ ├── logs
│ └── sbin
│ └── nginx
└── x3_pb2.py
```

### Build and run

Start Nginx, then run the Python script:

:::info Note

With an IMX477 sensor, set `fps=50` in `mipi_camera_web.py` (around line 39).

:::

```bash
# Start Nginx
./start_nginx.sh

# Run the Web camera sample
python3 mipi_camera_web.py
```

### Sample output

A Web server starts; open the board IP in a browser (default example: `http://192.168.127.10`).

![pydev_05_wb_disp_web_img](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/pydev_05_wb_disp_web_img.png)

Click **web Display** in the page to see the live stream and detections (see [Demo](#demo)).

## Details

### Command-line options
No arguments required; MIPI camera and WebSocket are initialized automatically.

### Software architecture

1. MIPI capture with `srcampy.Camera()`  
2. FCOS detection on each frame  
3. JPEG encode with `srcampy.Encoder()`  
4. WebSocket server pushing frames and results  
5. Web UI (HTML/JS)  
6. Nginx for static files and proxy  

<center>
![software_arch](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/pydev_05_wb_disp_cam_software_arch.png)
</center>

### API flow

1. `cam.open_cam()`  
2. `enc.encode()`  
3. `models = pyeasy_dnn.load('../models/fcos_512x512_nv12.bin')`  
4. `websockets.serve(web_service, "0.0.0.0", 8080)`  
5. Loop: grab frame → detect → JPEG encode → WebSocket push  

<center>
![API_Flow](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/pydev_05_wb_disp_cam_api_flow.png)
</center>

### FAQ

**Q:** Port in use.\
**A:** Check port 8080 with `netstat -tlnp`.

**Q:** Browser cannot open the stream.\
**A:** Check network and firewall rules.

**Q:** High latency.\
**A:** Lower resolution/FPS or use a lighter detector.

**Q:** Customize the Web UI.\
**A:** Edit files under `webservice/html/`.

**Q:** Add classes.\
**A:** Update `get_classes()` and use a compatible model.

**Q:** Save video.\
**A:** Add recording with OpenCV `VideoWriter`, for example.

