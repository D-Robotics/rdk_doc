---
sidebar_position: 7
---

# 5.6 Known Issues

1. **Issue Description: Slight black borders appear at the image edges.**

         ![problem_description](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/magicbox/zh/black_border.jpg)

    - Root Cause: This phenomenon is not caused by camera hardware issues or calibration errors. Instead, it is a normal occurrence during distortion correction and stereo rectification. The current parameter configuration prioritizes preserving a larger field of view (FOV), which may result in small regions without valid pixels at the image edges.  
    - Impact: This does not affect camera image quality or algorithm performance.  
    - Fix Plan: Future software versions will improve this issue by optimizing rectification parameters or implementing automatic cropping strategies. Users are advised to upgrade to the latest software version when available.