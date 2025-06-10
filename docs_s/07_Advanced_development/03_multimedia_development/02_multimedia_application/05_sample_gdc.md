# sample_gdc 使用说明

sample_gdc 目录下是用于演示如何使用 GDC 的示例程序，主要功能介绍如下：

1. `gdc_static_valid`: 读取本地 `gdc.bin` 文件和原始 YUV 文件送入 GDC 做变化处理后保存为 YUV 文件。
2. `gdc_stress_test`: 读取本地 `gdc.bin` 文件并将原始 YUV 文件循环送入 GDC 做 GDC 性能测试。
3. `gdc_equisolid`: 读取本地 NV12 的 YUV 图，把图片送入 GDC 做（全景 panoramic）校正处理。

## gdc_static_valid

### 功能概述

gdc_static_valid 程序会读取本地 NV12 的 YUV 图，把 gdc.bin 和图片一起送入 GDC 做变换处理，最后把结果保存为本地 NV12 格式的 YUV 图。

#### 代码位置及目录结构

- 代码位置 `app/samples/platform_samples/sample_gdc`
- 目录结构

```bash
sample_gdc/
├── 3-gdc_static_valid
│   ├── Makefile
│   ├── gdc_static_valid.c
│   └── test_res
│       ├── test_image_1920x1080.jpg
│       └── test_image_1920x1080.yuv
```

### 编译部署

#### 编译

- 进入 sample_gdc 目录，执行 `make` 编译
- 输出成果物是 sample_gdc/3-gdc_static_valid 目录下的 `gdc_static_valid`

#### 程序部署

刷写系统软件镜像后 , 本 sample 的可执行文件位于板端 : /app/multimedia_samples/sample_gdc/3-gdc_static_valid。

### 运行

#### 程序运行方法

直接执行程序 `./gdc_static_valid` 可以获得帮助信息：

```bash
Usage: gdc_static_valid [OPTIONS]
Options:
	c, --config <gdc_bin_file>    Specify the gdc configuration bin file.
	i, --input <input_file>       Specify the input image file.
	o, --output <output_file>     Specify the output image file.
	w, --iw <input_width>         Specify the width of the input image.
	h, --ih <input_height>        Specify the height of the input image.
	x, --ow [output_width]        Specify the width of the output image (optional).
	y, --oh [output_height]       Specify the height of the output image (optional).
	f, --feedback                 Specify feedback mode

If --ow and --oh are not specified, they will default to the input width and height, respectively.
```

#### 程序参数选项说明

gdc_static_valid 的选项参数说明：

参数选项：

- `c, --config`：指定 gdc.bin 配置文件
- `i, --input`：指定输入 NV12图像
- `o, --output`：指定输出 NV12图像
- `w, --iw`：指定输入图像水平方向分辨率（宽）
- `h, --ih`：指定输入图像垂直方向分辨率（高）
- `x, --ow`：指定输出图像水平方向分辨率（宽）（可选），默认和 --iw 保持一样
- `y, --oh`：指定输出图像垂直方向分辨率（高）（可选），默认和 --ih 保持一样

#### 运行效果

执行命令完成静态图片的矫正验证：

```bash
cd 3-gdc_static_valid
chmod +x gdc_static_valid
./gdc_static_valid -c ../../vp_sensors/gdc_bin/imx219_gdc.bin -i test_res/test_image_1920x1080.yuv -o gdc_output_1920x1080.yuv -w 1920 -h 1080
```

运行日志：

```shell
GDC vnode work mode: vflow
config file: ../../vp_sensors/gdc_bin/imx219_gdc.bin
input image: test_res/test_image_1920x1080.yuv
output image: gdc_output_1920x1080.yuv
input:1920x1080
output:1920x1080
(read_yuvv_nv12_file):file read(test_res/test_image_1920x1080.yuv), y-size(2073600)
handle 34661 GDC dump yuv 1920x1080(stride:1920), buffer size: 2073600 + 1036800 frame id: 0, timestamp: 0
```

## gdc_stress_test

### 功能概述

gdc_stress_test 程序会读取本地 NV12 的 YUV 图，把 gdc.bin 和图片一起送入 GDC 做变换处理，最后把结果保存为本地 NV12 格式的 YUV 图，可以指定执行 GDC 处理的次数，记录运行时间，并计算帧率（FPS）和总耗时。

#### 代码位置及目录结构

- 代码位置 `app/samples/platform_samples/sample_gdc`
- 目录结构

```bash
sample_gdc/
├── 4-gdc_stress_test
│   ├── Makefile
│   ├── gdc_1920x1080.bin
│   ├── gdc_stress_test.c
│   ├── test.sh
│   └── test_res
│       ├── test_image_1920x1080.jpg
│       └── test_image_1920x1080.yuv
```

### 编译部署

#### 编译

- 进入 sample_gdc 目录，执行 `make` 编译
- 输出成果物是 sample_gdc/4-gdc_stress_test 目录下的 `gdc_stress_test`

#### 程序部署

刷写系统软件镜像后 , 本 sample 的可执行文件位于板端 : /app/multimedia_samples/sample_gdc/4-gdc_stress_test。

### 运行

#### 程序运行方法

直接执行程序 `./gdc_stress_test` 可以获得帮助信息：

```bash
Usage: gdc_stress_test [OPTIONS]
Options:
	c, --config <gdc_bin_file>    Specify the gdc configuration bin file.
	i, --input <input_file>       Specify the input image file.
	o, --output <output_file>     Specify the output image file.
	w, --iw <input_width>         Specify the width of the input image.
	h, --ih <input_height>        Specify the height of the input image.
	x, --ow [output_width]        Specify the width of the output image (optional).
	y, --oh [output_height]       Specify the height of the output image (optional).
	C, --Count [run count]        Specify gdc sendframe and getframe count.
	p, --p [process id]           Specify id of process.
	f, --feedback                 Specify feedback mode

If --ow and --oh are not specified, they will default to the input width and height, respectively.
```

#### 程序参数选项说明

gdc_stress_test 的选项参数说明：

参数选项：

- `c, --config`：指定 gdc.bin 配置文件
- `i, --input`：指定输入 NV12图像
- `o, --output`：指定输出 NV12图像
- `w, --iw`：指定输入图像水平方向分辨率（宽）
- `h, --ih`：指定输入图像垂直方向分辨率（高）
- `x, --ow`：指定输出图像水平方向分辨率（宽）（可选），默认和 --iw 保持一样
- `y, --oh`：指定输出图像垂直方向分辨率（高）（可选），默认和 --ih 保持一样
- `C, --Count`：指定发送给 GDC 模块的图像的次数。
- `p, --p`：指定进程的 id。
- `f, --feedback`：使用回灌的方式。

#### 运行效果

执行命令：

```bash
cd 4-gdc_stress_test
chmod +x gdc_stress_test
sh test.sh
```

运行日志：

```shell
root@ubuntu:/app/multimedia_samples/sample_gdc/4-gdc_stress_test# GDC vnode work mode: vflow
GDC vnode work mode: vflow
GDC vnode work mode: vflow
config file: ./gdc_1920x1080.bin
config file: ./gdc_1920x1080.bin
config file: ./gdc_1920x1080.bin
input image: ./test_res/test_image_1920x1080.yuv
input image: ./test_res/test_image_1920x1080.yuv
input image: ./test_res/test_image_1920x1080.yuv
output image: ./gdc_output_1920x1080.yuv
output image: ./gdc_output_1920x1080.yuv
output image: ./gdc_output_1920x1080.yuv
input:1920x1080
input:1920x1080
input:1920x1080
output:1920x1080
output:1920x1080
output:1920x1080
(read_yuvv_nv12_file):file read(./test_res/test_image_1920x1080.yuv), y-size(2073600)
(read_yuvv_nv12_file):file read(./test_res/test_image_1920x1080.yuv), y-size(2073600)
(read_yuvv_nv12_file):file read(./test_res/test_image_1920x1080.yuv), y-size(2073600)
gdc temp fps [process3] = 150
gdc temp fps [process2] = 125
gdc temp fps [process1] = 125
gdc temp fps [process3] = 125
gdc temp fps [process2] = 116
gdc temp fps [process1] = 116
gdc temp fps [process3] = 116
gdc temp fps [process2] = 150
gdc temp fps [process1] = 150
gdc temp fps [process3] = 150
gdc temp fps [process2] = 137
.......
Gdc time consuming [process2]: 70
fps average gdc [process2] = 142
Gdc time consuming [process1]: 70
fps average gdc [process1] = 142
Gdc time consuming [process3]: 70
fps average gdc [process3] = 142
```

## gdc_equisolid
### 功能概述

gdc_equisolid 程序会读取本地 NV12 的 YUV 图，把图片送入 GDC 做（全景 panoramic）校正处理，最后把校正的结果结果保存为本地 NV12 格式的 YUV 图。

#### 代码位置及目录结构

- 代码位置 `app/samples/platform_samples/sample_gdc`
- 目录结构

```bash
sample_gdc/
├── 5-gdc_equisolid
│   ├── Makefile
│   └── gdc_equisolid.c
```

### 编译部署

#### 编译

- 进入 sample_gdc 目录，执行 `make` 编译
- 输出成果物是 sample_gdc/5-gdc_equisolid 目录下的 `gdc_equisolid`

#### 程序部署

刷写系统软件镜像后 , 本 sample 的可执行文件位于板端 : /app/multimedia_samples/sample_gdc/5-gdc_equisolid。

### 运行

#### 程序运行方法

直接执行程序 `./gdc_equisolid -h` 可以获得帮助信息：

```bash
Usage: gdc_equisolid [OPTIONS]
Options:
	i, --input <input_file>       Specify the input image file.
	o, --output <output_file>     Specify the output image file.
	w, --iw <input_width>         Specify the width of the input image.
	h, --ih <input_height>        Specify the height of the input image.
	f, --feedback                 Specify feedback mode
```

#### 程序参数选项说明

gdc_equisolid 的选项参数说明：

参数选项：

- `i, --input`：指定输入 NV12图像
- `o, --output`：指定输出 NV12图像（可选）
- `w, --iw`：指定输入图像水平方向分辨率（宽）
- `h, --ih`：指定输入图像垂直方向分辨率（高）
- `f, --feedback`：使用回灌的方式。

#### 运行效果

执行命令完成静态图片的（全景 panoramic）校正验证：

```bash
cd gdc_equisolid
chmod +x gdc_equisolid
./gdc_equisolid -i ../3-gdc_static_valid/test_res/test_image_1920x1080.yuv --iw 1920 --ih 1080
```

运行日志：

```bash
GDC vnode work mode: vflow
input file: ../3-gdc_static_valid/test_res/test_image_1920x1080.yuv
output file: gdc_output_1920x1080.yuv
input:1920x1080
(read_yuvv_nv12_file):file read(../3-gdc_static_valid/test_res/test_image_1920x1080.yuv), y-size(2073600)
handle 34661 GDC dump yuv 1920x1080(stride:1920), buffer size: 2073600 + 1036800 frame id: 0, timestamp: 0
```
