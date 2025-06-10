# sample_gpu_3d 使用说明
3D GPU 支持以下标准 API：
- OpenGLES
- OpenCL
- Vulkan

针对其中两种 API 提供了相应的示例代码：
1. OpenCL 示例
2. OpenGLES 示例

请根据具体需求选择对应的 API 示例进行参考和使用。

## OpenCL
### sample_matrix_multiply
#### 功能概述
功能描述：`sample_matrix_multiply` 使用 3D GPU 和 CPU 进行相同的矩阵运行，并打印两者的耗时

##### 代码位置及目录结构
- 代码位置 `/app/multimedia_samples/sample_gpu_3d/cl/sample_matrix_multiply`
- 目录结构
```
└── sample_matrix_multiply
    ├── Makefile
    └── matrix_multiply.c
```

#### 编译

- 进入 sample_matrix_multiply 目录，执行 `make` 编译
- 输出成果物是 sample_matrix_multiply 源码目录下的 `matrix_multiply`

#### 运行
##### 程序运行方法
执行可执行程序：`./matrix_multiply`
##### 程序参数选项说明
无
##### 运行效果
执行命令：
`./matrix_multiply`
运行日志：
```sh
./matrix_multiply
CPU execution time: 0.997923 seconds
OpenCL execution time: 0.038153 seconds
Matrices are identical!
```
效果说明
执行相同的矩阵乘法运算：
1. CPU 耗时： 0.997923 seconds
2. GPU 耗时： 0.038153 seconds

总耗时中可以看出在做矩阵乘法运算时， GPU 比 CPU 有更高得性能

## OpenGL ES

### sample_bezier
#### 功能概述
功能描述：`sample_bezier` 使用 3D GPU 画一条贝塞尔曲线，通过两种方式展示
1. 显示器桌面
2. 保存图片

##### 代码位置及目录结构
- 代码位置 `/app/multimedia_samples/sample_gpu_3d/gles/sample_bezier`
- 目录结构
```
sample_bezier/
├── bezier.c
└── Makefile
```

#### 编译
- 进入 sample_bezier 目录，执行 `make` 编译
- 输出成果物是 sample_bezier 源码目录下的 `bezier`

#### 运行
##### 程序运行方法
程序执行前需要做如下准备工作：
1. RDKS100 通过 HDMI 接口接上显示器
2. RDKS100 接上鼠标和键盘，通过显示器界面登录到Ubuntu系统中

执行程序：
1. 进入目录：`cd /app/multimedia_samples/sample_gpu_3d/gles/sample_bezier`
2. 执行程序：`./bezier`

注意：执行程序的用户与图像界面登录的用户不一致时需要执行如下命令: (用户可以通过 `ssh` 或 串口 登录的系统中执行程序)
1. 以 `sunrise` 用户为例, `id -u sunrise` 命令查看 sunrise 用户的用户 ID 为 1000
2. 通过 `export` 命令导出 环境变量 `WAYLAND_DISPLAY`， 注意命令中的 1000 就是步骤 1 查询得到的用户 ID

```shell
root@ubuntu:/app/multimedia_samples/sample_gpu_3d/gles/sample_bezier# id -u sunrise
1000

root@ubuntu:/app/multimedia_samples/sample_gpu_3d/gles/sample_bezier# export WAYLAND_DISPLAY=/run/user/1000/wayland-0
```

##### 程序参数选项说明
无参数选项

##### 运行效果

运行日志：无

效果说明:
1. 显示器桌面会显示一个窗口： 窗口中显示一条 红色的贝塞尔曲线
2. 窗口中显示的内容也会保存成文件：当前目录下，会生成如下 bmp 格式的图像文件： `bezier.bmp`
