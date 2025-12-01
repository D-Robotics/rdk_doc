# sample_gpu_3d Usage Guide  
## Function Overview  
The 3D GPU supports the following standard APIs:  
- OpenGLES  
- OpenCL  
- Vulkan  

Sample code is provided for two of these APIs:  
1. OpenCL Example  
2. OpenGLES Example  

Please select the corresponding API example based on your specific requirements for reference and usage.  

## OpenCL  
### sample_matrix_multiply  
#### Function Overview  
Description: `sample_matrix_multiply` performs the same matrix multiplication operation using both the 3D GPU and CPU, and prints the execution time for each.  

##### Code Location and Directory Structure  
- Code location: `/app/multimedia_samples/sample_gpu_3d/cl/sample_matrix_multiply`  
- Directory structure:  
```
└── sample_matrix_multiply
    ├── Makefile
    └── matrix_multiply.c
```

#### Compilation  

- Enter the `sample_matrix_multiply` directory and run `make` to compile.  
- The output binary is `matrix_multiply`, located in the `sample_matrix_multiply` source directory.  

#### Execution  
##### How to Run the Program  
Execute the binary: `./matrix_multiply`  
##### Program Parameter Options  
None  
##### Execution Result  
Command executed:  
`./matrix_multiply`  
Log output:  
```sh
./matrix_multiply
CPU execution time: 0.997923 seconds
OpenCL execution time: 0.038153 seconds
Matrices are identical!
```  
Result explanation:  
The same matrix multiplication operation was performed:  
1. CPU execution time: 0.997923 seconds  
2. GPU execution time: 0.038153 seconds  

From the total execution time, it is evident that the GPU demonstrates significantly higher performance than the CPU for matrix multiplication operations.  

## OpenGL ES  

### sample_bezier  
#### Function Overview  
Description: `sample_bezier` uses the 3D GPU to draw a Bézier curve, displayed in two ways:  
1. On the desktop monitor  
2. Saved as an image file  

##### Code Location and Directory Structure  
- Code location: `/app/multimedia_samples/sample_gpu_3d/gles/sample_bezier`  
- Directory structure:  
```
sample_bezier/
├── bezier.c
└── Makefile
```

#### Compilation  
- Enter the `sample_bezier` directory and run `make` to compile.  
- The output binary is `bezier`, located in the `sample_bezier` source directory.  

#### Execution  
##### How to Run the Program  
Before running the program, complete the following preparations:  
1. Connect the RDKS100 to a monitor via HDMI.  
2. Connect a mouse and keyboard to the RDKS100, and log into the Ubuntu system through the monitor's graphical interface.  

To run the program:  
1. Navigate to the directory: `cd /app/multimedia_samples/sample_gpu_3d/gles/sample_bezier`  
2. Execute the program: `./bezier`  

Note: If the user executing the program differs from the user logged into the graphical interface, run the following commands (e.g., if the program is executed by a user logged in via `ssh` or serial console):  
1. For example, using the user `sunrise`, run `id -u sunrise` to check that the user ID is 1000.  
2. Export the environment variable `WAYLAND_DISPLAY` using the `export` command. Note that `1000` in the command below is the user ID obtained in step 1.  

```shell
root@ubuntu:/app/multimedia_samples/sample_gpu_3d/gles/sample_bezier# id -u sunrise
1000

root@ubuntu:/app/multimedia_samples/sample_gpu_3d/gles/sample_bezier# export WAYLAND_DISPLAY=/run/user/1000/wayland-0
```

##### Program Parameter Options  
No parameters  

##### Execution Result  

Log output: None  

Result explanation:  
1. A window appears on the desktop monitor, displaying a red Bézier curve.  
2. The content shown in the window is also saved as a file: a BMP image named `bezier.bmp` is generated in the current directory.