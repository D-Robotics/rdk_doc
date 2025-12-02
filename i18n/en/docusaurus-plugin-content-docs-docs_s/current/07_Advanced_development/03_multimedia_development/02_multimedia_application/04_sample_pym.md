# sample_pym Usage Instructions  
## Function Overview  
sample_pym reads a YUV file into memory allocated by hbm, passes it to PYM, which processes it in a pyramid layer manner, and finally dumps the processed YUV data to the file system.

### Code Location and Directory Structure  
- Code location: `/app/multimedia_samples/sample_pym`  
- Directory structure:  
```
sample_pym/
├── Makefile
└── sample_pym.c
```

## Compilation  

Run the `make` command in the source code directory to compile:  

```Shell
cd /app/multimedia_samples/sample_pym
make
```

## Execution  
### How to Run the Program  
Execute the program directly with `./sample_pym` to display help information:  

### Program Argument Options  
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
- `-i`: Specifies the input YUV file. The test program uses files in NV12 format as input.  
- `-w`: Width of the input YUV image.  
- `-h`: Height of the input YUV image.  
- `-f`: Specifies the PYM operating mode. By default, it runs in Vflow mode.  

### Execution Example  
Taking a YUV image with input resolution 1920×1080 as an example, run:  
`./sample_pym -i /app/res/assets/nv12_1920x1080.yuv -w 1920 -h 1080`.

This feeds a YUV image into PYM, initializes six channels, performs downsampling operations at scales of 1, 1/2, 1/4, 1/8, 1/16, and 1/32 respectively, and saves the processed images as YUV files:

  - Channel 0 outputs the original resolution: 1920 × 1080.  
  - Channel 1 outputs width and height each halved: 960 × 540.  
  - Channel 2 outputs width and height each quartered: 480 × 270.  
  - Channel 3 outputs width and height each reduced by 1/8: 240 × 134.  
  - Channel 4 outputs width and height each reduced by 1/16: 120 × 66.  
  - Channel 5 outputs width and height each reduced by 1/32: 60 × 32.  

Output log as follows:  
```
pym vnode work mode: vflow
Using input file:/app/res/assets/nv12_1920x1080.yuv, input:1920x1080
(read_yuvv_nv12_file):file read(/app/res/assets/nv12_1920x1080.yuv), y-size(2073600)

pym config:
        ichn input width = 1920, height = 1080
        ochn[0] ratio= 1, width = 1920, height = 1080 wstride=1920 vstride=1080 out[1920*1080]
        ochn[1] ratio= 2, width = 960, height = 540 wstride=960 vstride=540 out[960*540]
        ochn[2] ratio= 4, width = 480, height = 270 wstride=480 vstride=270 out[480*270]
        ochn[3] ratio= 8, width = 240, height = 134 wstride=240 vstride=134 out[240*134]
        ochn[4] ratio= 16, width = 120, height = 66 wstride=128 vstride=66 out[120*66]
        ochn[5] ratio= 32, width = 60, height = 32 wstride=64 vstride=32 out[60*32]
```

Note:  
1. The width output by the PYM module is aligned to 16-byte boundaries. When viewing images, pay attention to cases where the `width` differs from the `wstride` parameter.