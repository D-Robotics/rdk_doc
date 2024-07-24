---
sidebar_position: 5
---

# 1.5 算法体验

<iframe src="//player.bilibili.com/player.html?aid=700903305&bvid=BV1rm4y1E73q&cid=1196558179&page=17" scrolling="no" border="0" frameborder="no" framespacing="0" width="100%" height="500" allowfullscreen="true"> </iframe>

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

![zebra_cls](../../static/img/01_Quick_start/image/classification/zebra_cls.jpg)




