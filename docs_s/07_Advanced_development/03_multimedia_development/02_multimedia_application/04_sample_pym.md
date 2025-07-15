# sample_pym 使用说明
## 功能概述
sample_pym 将 YUV 文件读入 hbm 申请的内存，并传入 PYM，PYM按照金字塔图层的方式处理，最后将处理好的 YUV 数据 dump 到文件系统中。

### 代码位置及目录结构
- 代码位置 `/app/multimedia_samples/sample_pym`
- 目录结构
```
sample_pym/
├── Makefile
└── sample_pym.c
```

## 编译

在源码路径下执行 `make` 命令即可完成编译：

```Shell
cd /app/multimedia_samples/sample_pym
make
```

## 运行
### 程序运行方法
直接执行程序 `./sample_pym` 可以获得帮助信息：

### 程序参数选项说明
```
./sample_pym
Usage: sample_pym [OPTIONS]
Options:
-i, --input_file FILE   Specify the input file
-w, --input_width WIDTH Specify the input width
-h, --input_height HEIGHT       Specify the input height
-f, --feedback                  Specify feedback mode
-V, --verbose           Enable verbose mode
```
- i: 指定输入的YUV文件，测试程序使用 NV12 格式的文件作为输入
- w: 输入 YUV 图像的宽度
- h: 输入 YUV 图像的高度
- f: 指定pym运行的模式， 默认按照Vflow的方式运行
### 运行效果
以输入分辨率为 1920 x 1080 的 YUV 图片为例，执行 `./sample_pym -i  ../resource/nv12_1920x1080.yuv -w 1920 -h 1080`。

把一张 YUV 图像送入 PYM，并且初始化6个通道，分别进行 1、1/2、1/4、1/8、1/16、1/32 倍率的缩小操作，并且把处理后的图像保存为 yuv 图像：

  - 0 通道输出输入图像的原分辨率: 1920 x 1080。
  - 1 通道输出宽和高各缩小2倍的处理: 960 x 540。
  - 2 通道输出宽和高各缩小4倍的处理: 480 x 270。
  - 3 通道输出宽和高各缩小8倍的处理: 240 x 134。
  - 4 通道输出宽和高各缩小16倍的处理: 120 x 66。
  - 5 通道输出宽和高各缩小32倍的处理: 60 x 32。

输出 log 如下:
```
pym vnode work mode: vflow
Using input file:../resource/nv12_1920x1080.yuv, input:1920x1080
(read_yuvv_nv12_file):file read(../resource/nv12_1920x1080.yuv), y-size(2073600)

pym config:
        ichn input width = 1920, height = 1080
        ochn[0] ratio= 1, width = 1920, height = 1080 wstride=1920 vstride=1080 out[1920*1080]
        ochn[1] ratio= 2, width = 960, height = 540 wstride=960 vstride=540 out[960*540]
        ochn[2] ratio= 4, width = 480, height = 270 wstride=480 vstride=270 out[480*270]
        ochn[3] ratio= 8, width = 240, height = 134 wstride=240 vstride=134 out[240*134]
        ochn[4] ratio= 16, width = 120, height = 66 wstride=128 vstride=66 out[120*66]
        ochn[5] ratio= 32, width = 60, height = 32 wstride=64 vstride=32 out[60*32]
```

注意：
1. pym模块输出的宽度按照16字节对齐，在浏览图片时需要注意 width 与 wstride 参数不同的情况
