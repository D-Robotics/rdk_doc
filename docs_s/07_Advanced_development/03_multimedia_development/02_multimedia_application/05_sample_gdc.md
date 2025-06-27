# sample_gdc 使用说明

sample_gdc 目录下是用于演示如何使用 GDC 的示例程序，主要功能介绍如下：

1. `generate_custom_config.py`: 生成 GDC 的矫正标定配置参数。
2. `generate_bin`: 读取本地 json 配置文件，生成对应的 `gdc.bin` 文件。
3. `gdc_static_valid`: 读取本地 `gdc.bin` 文件和原始 YUV 文件送入 GDC 做变化处理后保存为 YUV 文件。
4. `gdc_stress_test`: 读取本地 `gdc.bin` 文件并将原始 YUV 文件循环送入 GDC 做 GDC 性能测试。
5. `gdc_equisolid`: 读取本地 NV12 的 YUV 图，把图片送入 GDC 做（全景 panoramic）校正处理。
6. `gdc_transformation`: 读取本地 的 json 配置文件，把图片送入 GDC 做180线性变换、圆柱形变换、等距变换和梯形校正+去畸变处理。

## 1-custom_config

### 功能概述

本示例旨在展示如何通过 custom 变换的方式，预先准备输入图像，并生成用于指导 GDC 矫正的标定参数文件。

### 代码位置及目录结构

- 代码位置 `/app/multimedia_samples/sample_gdc/`
- 目录结构

```bash
sample_gdc/
├── 1-custom_config
│   ├── Makefile
│   ├── chessboard
│   ├── chessboard.png
│   ├── custom_config.txt
│   └── generate_custom_config.py
```

### 开发和使用流程

![S100-gdc](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/03_multimedia_development/02_S100/02_multimedia_application/sample_gdc/S100-gdc.jpg)

在 PC 上使用 `generate_custom_config.py` 程序生成 GDC 的矫正标定配置参数。

- 准备棋盘格图片（chessboard.png）， 可以打印出来，或者用显示器预览。

- 使用目标 Camera Sensor 在不同角度下拍摄棋盘格图片，拍摄15张左右，建议多拍一些。

  ![Checkerboard_Image](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/03_multimedia_development/02_S100/02_multimedia_application/sample_gdc/Checkerboard_Image.png)

- 以上面的棋盘格图片作为输入，执行以下 python 程序（确保系统支持 Python 3，和安装了 `opencv-python` 库），生成 GDC 的矫正标定参数文件（custom_config.txt）：

  ```bash
  cd 1-custom_config
  python3 ./generate_custom_config.py
  ```

:::caution 注意
注意事项：拍摄棋盘格的时候尽量距离远一点，棋盘格占画面较大时容易导致 python 程序识别失败
:::

  在字符终端运行时的日志：

  ```bash
  No graphical environment detected. Skipping display of images.
  Input images directory: ./chessboard
  Test image file: ./chessboard/vlcsnap-2024-05-06-09h53m19s733.jpg
  Output file: custom_config.txt
  i: 0
  i: 1
  ... 省略 ...
  i: 15
  Intrinsic matrix (mtx):
   [[784.57179685   0.         939.01168998]
   [  0.         784.35388599 554.71639175]
   [  0.           0.           1.        ]]
  Distortion coefficients (dist):
   [[-3.16520533e-01  1.02422375e-01 -2.60692201e-04  7.23624256e-04
	-1.44726239e-02]]
  Rotation vectors (rvecs):
   (array([[-0.07424795],
		 [ 0.17820099],
		 [-0.04684888]]), array([[0.44453246],
		 [0.01540531],
		 [0.03596364]]), array([[-0.38217217],
		 [-0.4097845 ],
		 [ 0.16419408]]), array([[-0.0713388 ],
		 [-0.04356189],
		 [ 0.00918689]]), array([[ 0.40326625],
		 [-0.65705694],
		 [ 0.07152059]]), array([[-0.28933582],
		 [ 0.07433653],
		 [ 0.09031559]]), array([[-0.1353966 ],
		 [-0.61689018],
		 [-0.06274773]]), array([[ 0.02193021],
		 [-0.52159079],
		 [ 0.08910704]]), array([[-0.10002557],
		 [-0.25580186],
		 [-0.08683701]]), array([[-0.37827059],
		 [-0.98115358],
		 [ 0.175653  ]]), array([[0.33652166],
		 [0.12869965],
		 [0.03961734]]), array([[ 0.40214666],
		 [-0.38128926],
		 [-0.0051477 ]]), array([[-0.04168182],
		 [ 0.0922567 ],
		 [-0.03109347]]))
  Translation vectors (tvecs):
   (array([[ -71.275847  ],
		 [-106.35748096],
		 [ 224.27593215]]), array([[  3.80981104],
		 [-81.7694762 ],
		 [183.44645072]]), array([[ -81.59651772],
		 [-122.67796656],
		 [ 240.51602851]]), array([[ -86.84292041],
		 [-114.12437351],
		 [ 203.11264832]]), array([[-51.05204629],
		 [-57.97011932],
		 [168.23228488]]), array([[ -25.79903668],
		 [-139.06100345],
		 [ 248.06228137]]), array([[-193.45812779],
		 [-116.55450795],
		 [ 135.39187735]]), array([[ -91.12813213],
		 [-111.74470892],
		 [ 150.61397733]]), array([[ -89.4296275 ],
		 [-117.79736921],
		 [ 196.30732053]]), array([[-156.41212161],
		 [-146.02467216],
		 [ 191.13039641]]), array([[ -60.65720996],
		 [-111.66378957],
		 [ 191.3402483 ]]), array([[-56.56159122],
		 [-72.97166982],
		 [149.97470497]]), array([[ -60.28586566],
		 [-111.66429463],
		 [ 196.1187917 ]]))
  New camera matrix (newcameramtx):
   [[784.57179685   0.         939.01168998]
   [  0.         784.35388599 554.71639175]
   [  0.           0.           1.        ]]
  Validation of distortion correction
  Saving mapx and mapy to 'custom_config.txt'
  No graphical environment detected. The output image has been saved to 'custom_config.txt'.
  ```

  如果在图形化桌面的终端运行，会显示执行过程中的标定和矫正效果：

  ![Calibration_Process](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/03_multimedia_development/02_S100/02_multimedia_application/sample_gdc/Calibration_Process.png)

  ![Correction_Effect](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/03_multimedia_development/02_S100/02_multimedia_application/sample_gdc/Correction_Effect.png)

generate_custom_config.py 的选项参数：

```bash
usage: generate_custom_config.py [-h] [-i INPUT_IMAGES_DIR] [-t TEST_IMAGE]
								 [-o OUTPUT_FILE]

Gdc calibration and image undistortion.

optional arguments:
  -h, --help            show this help message and exit
  -i INPUT_IMAGES_DIR, --input_images_dir INPUT_IMAGES_DIR
						Directory containing the chessboard images.
  -t TEST_IMAGE, --test_image TEST_IMAGE
						File path of the image to be undistorted.
  -o OUTPUT_FILE, --output_file OUTPUT_FILE
						File path for the output configuration.
```

## 2-generate_bin

### 功能概述

本程序通过读取本地 `gdc_bin_custom_config.json`  配置文件，生成对应的 `gdc.bin` 文件。

#### 代码位置及目录结构

- 代码位置 `/app/multimedia_samples/sample_gdc/`
- 目录结构

```bash
sample_gdc/
├── 2-generate_bin
│   ├── Makefile
│   ├── gdc_bin_custom_config.json
│   └── generate_bin.c
```

### 编译部署

####  编译

- 进入 sample_gdc 目录，执行 `make` 编译
- 输出成果物是 sample_gdc/2-generate_bin 目录下的 `generate_bin`

####  程序部署

刷写系统软件镜像后 , 本 sample 的可执行文件位于板端 : /app/multimedia_samples/sample_gdc/2-generate_bin。

### 运行

#### 程序运行方法

直接执行程序 `./generate_bin -h` 可以获得帮助信息：

```shell
./generate_bin -h
genereate_bin [-c json_config_file] [-o output_file]
```

#### 程序参数选项说明

**选项：**

- `[-c json_config_file]`：指定配置 GDC 的模块的 json 参数文件（可选），默认为 `./gdc_bin_custom_config.json`。

- `[-o output_file]` ：指定输出 GDC bin 文件的路径（可选），默认为 `./gdc.bin`。

#### 运行效果

执行命令：

```shell
cd 2-generate_bin
chmod +x generate_bin
./generate_bin
```

运行日志：

```shell
Gdc bin custom config: ./gdc_bin_custom_config.json
Generate gdc bin file: ./gdc.bin
gdc gen cfg_buf 0xffff82090010, size 10972
Generate bin file size:10972
```

## gdc_static_valid

### 功能概述

gdc_static_valid 程序会读取本地 NV12 的 YUV 图，把 gdc.bin 和图片一起送入 GDC 做变换处理，最后把结果保存为本地 NV12 格式的 YUV 图。

#### 代码位置及目录结构

- 代码位置 `/app/multimedia_samples/sample_gdc/`
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

- 代码位置 `/app/multimedia_samples/sample_gdc/`
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

- 代码位置 `/app/multimedia_samples/sample_gdc/`
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

## 6-gdc_transformation

### 功能概述

本文的 gdc_transformation 实现 GDC 模块将回灌输入的图像进行180线性变换、圆柱形变换、等距变换和梯形校正+去畸变。

#### 软件架构说明：

gdc_transformation 程序采用回灌流程，即从系统存储中读取原始 YUV 文件和 GDC Tool 生成的 json 文件，作为 GDC 的输入图像。 依赖`libgdcbin.so`将 GDC 坐标点通过计算，把图像的变换结果保存为本地 NV12 格式的 YUV 图。

使用 GDC Tool 生成生成的 json 文件都在 gdc_res 目录下，当前 gdc_res 目录有四个 json 文件，变换效果分别是 Affine、Equisolid(cylinder)、Equidistant、Keystone+dewarping，gdc_transformation 会根据这四个 json 去 dump 出4张 YUV 图。

注意：gdc_res 目录下有几个 json 文件就会 dump 出几张 YUV 图片。

#### 代码位置及目录结构

- 代码位置 `/app/multimedia_samples/sample_gdc/`
- 目录结构

```bash
sample_gdc/
└── 6-gdc_transformation
	├── Makefile
	├── gdc_res
	│   ├── Affine.json
	│   ├── Equidistant.json
	│   ├── Equisolid_cylinder.json
	│   ├── Keystone_dewarping.json
	│   └── test_building_1920x1080.yuv
	└── gdc_transformation.c
```
根目录包含 Makefile,gdc_res 目录中包含了资源文件,比如 GDC Tool 生成生成的 json 文件、YUV 图像；gdc_transformation.c 是 main 入口的所在文件。

### 编译部署

#### 编译

- 进入 sample_gdc 目录，执行 `make` 编译
- 输出成果物是 sample_gdc/6-gdc_transformation 目录下的 `gdc_transformation`

#### 程序部署

刷写系统软件镜像后 , 本 sample 的可执行文件位于板端 : /app/multimedia_samples/sample_gdc/6-gdc_transformation。

### 运行

#### 程序运行方法

直接执行程序 `./gdc_transformation` 可以获得帮助信息：

```bash
Usage: gdc_transformation [OPTIONS]
Options:
  i, --input <input_file>       Specify the input image file.
  x, --ix <input_width>         Specify the width of the input image.
  y, --iy <input_height>        Specify the height of the input image.
```

#### 程序参数选项说明

gdc_transformation 的选项参数说明：

参数选项：

- `i, --input`：指定输入 NV12图像
- `x, --ix`：指定输入图像水平方向分辨率（宽）
- `y, --iy`：指定输入图像垂直方向分辨率（高）

#### 运行效果

执行命令完成静态图片的 transformation 验证：

```bash
cd 6-gdc_transformation
chmod +x gdc_transformation
./gdc_transformation -i gdc_res/test_building_1920x1080.yuv --ix 1920 --iy 1080
```

运行日志：

```bash
# ./gdc_transformation -i gdc_res/test_building_1920x1080.yuv --ix 1920 --iy 1080
input file: gdc_res/test_building_1920x1080.yuv
input:1920x1080
(read_yuvv_nv12_file):file read(gdc_res/test_building_1920x1080.yuv), y-size(2073600)
gdc gen cfg_buf 0xaaab00d46460, size 5956
Dump image to file(Equidistant.yuv), size(2073600) + size1(1036800) succeeded
gdc gen cfg_buf 0xaaab00d868b0, size 7756
Dump image to file(Affine.yuv), size(2073600) + size1(1036800) succeeded
gdc gen cfg_buf 0xaaab00d868b0, size 5884
Dump image to file(Keystone_dewarping.yuv), size(2073600) + size1(1036800) succeeded
gdc gen cfg_buf 0xaaab00d868b0, size 8548
Dump image to file(Equisolid_cylinder.yuv), size(2073600) + size1(1036800) succeeded
```

gdc_transformation 的选项参数说明：

```bash
#./gdc_transformation -h
Usage: gdc_transformation [OPTIONS]
Options:
  i, --input <input_file>       Specify the input image file.
  x, --ix <input_width>         Specify the width of the input image.
  y, --iy <input_height>        Specify the height of the input image.
```

#### 运行效果说明
原始图像如下图所示：

![Original_Image.png](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/03_multimedia_development/02_S100/02_multimedia_application/sample_gdc/Original_Image.png)

分别对每个 json 解析变换后，输出4张处理后的 NV12 格式的 YUV 图像，效果如下图所示：

![Transformed_Effect](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/03_multimedia_development/02_S100/02_multimedia_application/sample_gdc/Transformed_Effect.png)
