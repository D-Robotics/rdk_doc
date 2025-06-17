---
sidebar_position: 2
---
# 4.1.2 Importing the Model Inference Library

The `hobot_dnn` model inference library is pre-installed on the Ubuntu system of the development board. Users can import the module and check the version information.

```shell
sunrise@ubuntu:~$ sudo python3
Python 3.8.10 (default, Mar 15 2022, 12:22:08) 
Type "help", "copyright", "credits" or "license" for more information.
>>> from hobot_dnn import pyeasy_dnn as dnn
>>> dir(dnn)
['Model', 'TensorProperties', '__doc__', '__file__', '__loader__', '__name__', '__package__', '__spec__', 'load', 'pyDNNTensor']
```

The main classes and interfaces used in the `hobot_dnn` inference library are as follows:

- **Model**: AI algorithm model class, used for loading algorithm models and performing inference calculations. For more information, please refer to the [Model](./pydev_dnn_api.md) documentation.
- **pyDNNTensor**: AI algorithm input and output data tensor class. For more information, please refer to the [pyDNNTensor](./pydev_dnn_api.md) documentation.
- **TensorProperties**: Class for the properties of the input tensor of the model. For more information, please refer to the [TensorProperties](./pydev_dnn_api.md) documentation.
- **load**: Load algorithm models. For more information, please refer to the [API interface](./pydev_dnn_api.md) documentation.