---
sidebar_position: 2
---

# 4.1.2 模型推理库导入

`hobot_dnn` 模型推理库，已预装到 RDK OS 系统中，用户可以通过导入模块，查看属性信息。

```shell
sunrise@ubuntu:~$ sudo python3 -c "from hobot_dnn import pyeasy_dnn as dnn; print(dir(dnn))"
['Model', 'TensorProperties', '__doc__', '__file__', '__loader__', '__name__', '__package__', '__spec__', 'load', 'pyDNNTensor']
```

`hobot_dnn`推理库主要使用的类和接口如下：

- **Model** ： 算法模型类，执行加载算法模型、推理计算， 更多信息请查阅 [Model](./pydev_dnn_api.md) 。
- **pyDNNTensor**：算法数据输入、输出数据 tensor 类， 更多信息请查阅 [pyDNNTensor](./pydev_dnn_api.md) 。
- **TensorProperties** ：模型输入 tensor 的属性类， 更多信息请查阅 [TensorProperties](./pydev_dnn_api.md) 。
- **load**：加载算法模型，更多信息请查阅 [API接口](./pydev_dnn_api.md) 。

