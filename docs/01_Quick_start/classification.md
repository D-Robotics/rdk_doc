---
sidebar_position: 6
---

# 1.6 算法体验

Video: https://www.bilibili.com/video/BV1rm4y1E73q/?p=17

开发板上安装了`test_mobilenetv1.py` 程序用于测试mobilenet v1图像分类算法功能，该程序读取 `zebra_cls.jpg` 静态图片作为模型的输入，并在命令行终端输出分类结果`cls id: 340 Confidence: 0.991851`


## 运行方式一：命令行执行
执行 `test_mobilenetv1.py` 程序

  ```bash
  sunrise@ubuntu:~$ cd /app/pydev_demo/01_basic_sample/
  sunrise@ubuntu:/app/pydev_demo/01_basic_sample$ sudo ./test_mobilenetv1.py
  ```

### 预期效果
输出图像分类算法的预测结果，id和confidence。

`zebra_cls.jpg`是一张斑马的图片，按照`ImageNet`数据集的分类，返回结果id为340， 置信度为0.991851。

```shell
========== Classification result ==========
cls id: 340 Confidence: 0.991851
```

![zebra_cls](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/04_Algorithm_Application/01_pydev_dnn_demo/image/pydev_dnn_demo/zebra_cls.jpg)


## 运行方式二：使用 RDK Studio 体验图像分类算法

:::info 说明

- RDK Studio 下载链接：[点此下载](https://developer.d-robotics.cc/rdkstudio)
- RDK Studio 使用指南：[点此查看](../01_Quick_start/09_RDK_Studio/01_rdk_studio.md)

:::


1. 点击 `基础静态图片推理` 示例下的 Node-RED。
   
    ![示例页面](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/left_menu_example_application_image_zebra.png)

2. 进入示例应用流程界面。
   
    :::tip 提示

    点击 RDK Studio 右上角  ![链接图标](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/left_menu_example_application_link_browser.png) 图标可以快速在浏览器中打开示例！

    :::
        
   ![基础静态图片推理示例页面](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/left_menu_example_application_image_zebra_home.png)

3. 点击 `查看图片` 指令，自动展示图片。

    ![示例图片](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/left_menu_example_application_image_zebra_display-image.png)

4. 点击右侧调试图标将右侧边栏定位至调试窗口，点击`运行` ，可在调试窗口查看图像分类算法输出结果；依次点击其他`运行`，可查看不同算法的输出结果。
      
    ![示例图片](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/left_menu_example_application_image_zebra_result.png)


    :::warning 注意
    
    如果对节点、流程等进行了修改，需点击右上角 ![部署图片](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/left_menu_example_application_deploy.png) 按钮后才能生效！
    
       
    :::

5. 点击右上角 ` × ` 图标，选择 “关闭程序” 退出 Node-RED 应用。

    ![示例图片](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/left_menu_example_application_nodered_close.png)




