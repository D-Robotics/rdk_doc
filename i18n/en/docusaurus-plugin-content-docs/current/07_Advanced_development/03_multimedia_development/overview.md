---
sidebar_position: 1
---
# 7.3.1 System Overview

## Overview

The multimedia interface provided by D-Robotics includes system control, video input, video processing (ISP image processor, cropping, scaling, rotation, correction), H.264/H.265/JPEG/MJPEG encoding and decoding, video output display, etc. The interface is abstracted and encapsulated, which is a collection of low-level interfaces. These low-level interfaces support more flexible application development by better controlling the underlying hardware modules.

## Multimedia System Architecture

The main internal processing flow of D-Robotics's multimedia processing is shown in the following figure, which is mainly divided into video input (VIN), video processing (VPS), video encoding (VENC), video decoding (VDEC), video output (VOT), region processing (REGION), AI algorithm inference (BPU), and other modules.

![X3-ss_mm_system_topology](./image/overview/X3-ss_mm_system_topology-16485465559782.png)

## Terminology Conventions{#terminology}

| Abbreviation | Full Name                           | Explanation                                                  |
| ------------ | ----------------------------------- | ------------------------------------------------------------ |
| VIN          | Video IN                            | Includes video processing access, image signal processor, distortion correction, and anti-shake processing. It receives and processes data from sensors, and can also directly receive image data from memory. |
| VPS          | Video Process System                | Includes image rotation, image cropping, and scaling functions, which can output images of different resolutions from the same input source. The input source can be the VIN module or image data in memory. |
| VENC         | Video Encode                        | The VENC encoding module supports H.264/H.265/JPEG/MJPEG encoding. The data processed by the VPS module can be encoded into a bitstream output according to different protocols by the encoding module. |
| VDEC         | Video Decode                        | The VDEC decoding module supports H.264/H.265/JPEG/MJPEG decoding. It can decode the encoded bitstream and hand it over to the VPS module for further processing, and then output it to the VOT module for display. |
| VPU          | Video Processing Unit               | Video processing unit, responsible for video encoding and decoding functions. |
| JPU          | JPEG Processing Unit                | JPEG image processing unit, responsible for JPEG and MJPEG encoding and decoding functions. |
| VOT          | Video Output                        | The video output module receives image data from VPS and VDEC and can output to a display device.  |
| VIO          | Video IN/OUT                        | Video input and output, including VIN and VOT modules.       |
| MIPI         | Mobile Industry Processor Interface | Mobile industry processor interface.                         |
| CSI          | Camera Serial Interface             | Camera serial interface. The CSI interface and the DSI interface belong to the same family and are interface specifications formulated by MIPI (Mobile Industry Processor Interface Alliance). |
| DVP          | Digital Video Port                  | Digital video port.                                          |
| SIF          | Sensor Interface                    | Sensor interface used to receive MIPI, DVP, or image data from memory. |
| ISP          | Image Signal Processor              | Image signal processor, responsible for fine-tuning the image effect. |
| LDC          | Lens Distortion Correction          | Lens distortion correction.                                  |
| DIS          | Digital Image Stabilizer            | Digital image stabilization.                                 |
| DWE          | Dewarp Engine                       | Dewarp engine, which integrates LDC and DIS, including LDC distortion correction and DIS statistical results. |
| IPU          | Image Process Unit                  | Image signal processing unit, supports image rotation, image cropping, and scaling functions. |
| GDC          | Geometrical Distortion Correction   | Geometrical distortion correction.                            |
| PYM          | Pyramid                             | Image pyramid.                                               |
| OSD          | On Screen Display                   | Video image overlay display.                                 |
| BPU          | Brain Process Unit                  | Programmable AI acceleration engine independently developed by D-Robotics. |
| HAL          | Hardware Abstraction Layer          | Hardware abstraction layer.                                  |
| FW           | Firmware                            | Firmware.                                                    |
| Sensor       | Sensor                              | Unless otherwise specified, it refers to CMOS image sensors.  |