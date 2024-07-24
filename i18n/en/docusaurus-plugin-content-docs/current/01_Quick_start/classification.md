---
sidebar_position: 5
---
# 1.5 Image Classification Algorithm Example

<iframe width="560" height="315" src="https://www.youtube.com/embed/lGFel8uabLY?si=TijOhxNW8YMLIHGv" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

The development board is installed with the program `test_mobilenetv1.py` for testing the functionality of the mobilenet v1 image classification algorithm. This program reads the static image `zebra_cls.jpg` as the input of the model, and outputs the classification result `cls id: 340 Confidence: 0.991851` in the command line terminal.


## Execution Method
Execute the program `test_mobilenetv1.py`

  ```bash
  sunrise@ubuntu:~$ cd /app/pydev_demo/01_basic_sample/
  sunrise@ubuntu:/app/pydev_demo/01_basic_sample$ sudo ./test_mobilenetv1.py
  ```

## Expected Effect
Output the predicted result of the image classification algorithm, id and confidence.

`zebra_cls.jpg` is an image of a zebra. According to the classification of the `ImageNet` dataset, the returned result id is 340, with a confidence of 0.991851.

```shell
========== Classification result ==========
cls id: 340 Confidence: 0.991851
```

![zebra_cls](../../../../../static/img/01_Quick_start/image/classification/zebra_cls.jpg)