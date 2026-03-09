---
sidebar_position: 7
---

# 5.6 Known Issues

1. **Problem Description: A small amount of black edges appear on the image border.**

        ![problem_description](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/magicbox/zh/black_border.jpg)

    - **Cause Analysis**: This phenomenon is not caused by issues with the camera hardware or calibration, but is a normal occurrence during distortion correction and stereo rectification. The current parameter configuration is designed to retain as large a field of view (FOV) as possible, which may result in a small area with no effective pixel data at the image edges.
    - **Scope of Impact**: Does not affect image quality or algorithm performance.
    - **Improvement Plan**: This will be improved in subsequent software versions by optimizing correction parameters or implementing an automatic cropping strategy. The improvement will be delivered through a software version upgrade.