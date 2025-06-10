---
sidebar_position: 5
---

# 1.5 算法体验

Video: https://www.bilibili.com/video/BV1rm4y1E73q/?p=17

开发板上安装了`test_mobilenetv1.py` 程序用于测试mobilenet v1图像分类算法功能，该程序读取 `zebra_cls.jpg` 静态图片作为模型的输入，并在命令行终端输出分类结果`cls id: 340 Confidence: 0.991851`


## 运行方式
执行 `test_mobilenetv1.py` 程序

  ```bash
  sunrise@ubuntu:~$ cd /app/pydev_demo/01_basic_sample/
  sunrise@ubuntu:/app/pydev_demo/01_basic_sample$ sudo ./test_mobilenetv1.py
  ```

## 预期效果
输出图像分类算法的预测结果，id和confidence。

`zebra_cls.jpg`是一张斑马的图片，按照`ImageNet`数据集的分类，返回结果id为340， 置信度为0.991851。

```shell
========== Classification result ==========
cls id: 340 Confidence: 0.991851
```

![zebra_cls](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/04_Algorithm_Application/01_pydev_dnn_demo/image/pydev_dnn_demo/zebra_cls.jpg)




