---
sidebar_position: 7
---

# 5.6 已知问题

1. **问题描述：图像边缘出现少量黑边。**

         ![problem_description](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/magicbox/zh/black_border.jpg)

    - 问题原因：此现象并不是相机硬件或标定存在问题，而是在进行畸变校正和双目极线矫正（stereo rectify）时的一种正常现象。当前参数配置为了尽可能保留更大的视场范围（FOV），在图像边缘可能会产生少量无有效像素区域。  
    - 影响范围：不影响相机成像质量和算法使用。  
    - 修复计划：在后续软件版本中通过优化矫正参数或自动裁剪策略进行改进，优化升级软件版本。



