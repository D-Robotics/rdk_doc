---
sidebar_position: 2
---

# 4.1.2 模型推理库导入

`hobot_dnn`模型推理库，已预装到开发板Ubuntu系统，用户可以通过导入模块，查看版本信息。

```shell
sunrise@ubuntu:~$ sudo python3
Python 3.8.10 (default, Mar 15 2022, 12:22:08) 
Type "help", "copyright", "credits" or "license" for more information.
>>> from hobot_dnn import pyeasy_dnn as dnn
>>> dir(dnn)
['Model', 'TensorProperties', '__doc__', '__file__', '__loader__', '__name__', '__package__', '__spec__', 'load', 'pyDNNTensor']
```

`hobot_dnn`推理库主要使用的类和接口如下：

- **Model** ： AI 算法模型类，执行加载算法模型、推理计算， 更多信息请查阅 [Model](/python_development/pydev_dnn_api#model) 。
- **pyDNNTensor**：AI 算法输入、输出数据 tensor 类， 更多信息请查阅 [pyDNNTensor](/python_development/pydev_dnn_api) 。
- **TensorProperties** ：模型输入 tensor 的属性类， 更多信息请查阅 [TensorProperties](/python_development/pydev_dnn_api) 。
- **load**：加载算法模型，更多信息请查阅 [API接口](/python_development/pydev_dnn_api) 。


