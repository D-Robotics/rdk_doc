# Example Code Introduction

This chapter mainly introduces example code for the multimedia modules on the chip, primarily covering usage examples of multimedia hardware acceleration modules. It includes single-module functional examples, multi-module cascaded examples, and application solution examples. Users can leverage these examples to quickly perform functional evaluations, and referring to the implementation code will help them get started rapidly and develop applications tailored to their specific needs.

By using the examples provided in this chapter, users will be able to:

- Understand the basic functionality and applicable scenarios of each hardware acceleration module.
- Learn how to efficiently combine multiple modules to solve real-world problems.
- Quickly master the usage of the example code, thereby saving development time.

## Example Source Code Directory Structure

All example source code is stored in the `/app/multimedia_samples` directory on the device.

The source code directory structure is organized and named according to functionality and scenarios as follows:

```
.
├── chip_base_test    # Driver functional unit tests; please refer to [BSP Development Guide - Driver Functional Unit Tests] for usage instructions
├── Makefile.in       # Makefile parameter configuration, including cross-compilation toolchain, header file and library references, etc.
├── README.md
├── sample_codec      # Example code for video/image codec modules, including encoding and decoding for H264/H265/JPEG/MJPEG
├── sample_gdc        # Example code demonstrating various transformation modes supported by the GDC module
├── sample_isp        # Example code for the ISP module, including ISP initialization and retrieving ISP-processed data
├── sample_pipeline   # Example code cascading multiple functional modules, e.g., VIN->ISP->PYM->GDC->CODEC encoding data pipeline functional test
├── sample_vin        # Initialize Camera Sensor and acquire images from the VIN module
├── sample_pym        # Initialize the PYM module and use PYM to downscale images
├── sample_gpu_3d     # Examples demonstrating 3D GPU usage via both OpenCL and OpenGLES APIs
├── sunrise_camera    # Integrated application solution example leveraging most modules, supporting smart cameras and intelligent analysis boxes; video streams and algorithm results can be viewed via a web browser
├── utils             # Common utility functions and structures
└── vp_sensors        # Camera Sensor configuration code, used by other modules requiring Camera Sensor functionality
```

Note: The `vp_sensors` directory contains supported `Camera Sensor` drivers and configuration files, and is not a standalone example program. For instructions on adding new `Camera Sensors`, please refer to `vp_sensors/README.md`.

## Example Usage Guide

All example programs and their dependent resource files are installed by default in the `/app/multimedia_samples` directory on the development board. Users can directly log in to the board and run these examples. For instance, to use the `sample_pym` example, follow these commands:

```
#1. Enter the source code directory
cd /app/multimedia_samples/sample_pym

#2. Compile the program
make

#3. Run sample_pym to display help information

./sample_pym
Usage: sample_pym [OPTIONS]
Options:
-i, --input_file FILE   Specify the input file
-w, --input_width WIDTH Specify the input width
-h, --input_height HEIGHT       Specify the input height
-f, --feedback                  Specify feedback mode
-V, --verbose           Enable verbose mode
```

All other examples can be used in the same manner.

## Example Development Guide

### Compilation Instructions

Dependencies for the example code in this module are as follows:

1. Depends on header files and libraries generated from BSP source compilation (e.g., hbre), as well as header files and dynamic libraries such as ffmpeg in the root filesystem. Library and path references are specified in the Makefile under each Sample directory:
```
LDFLAGS += -lvpf -lvio -lcam -lhbmem
LDFLAGS := -L/usr/hobot/lib
INCDIR := -I/usr/hobot/include -I/usr/include/cjson/
```

2. Depends on Sensor configuration files in the `/app/multimedia_samples/vp_sensors` directory. File references are specified in `/app/multimedia_samples/Makefile.in` as follows:
```
INCS += -I . \
	-I ${PLATFORM_SAMPLES_DIR}/utils/ \
	-I ${PLATFORM_SAMPLES_DIR}/vp_sensors

SRC_PATH := ${PLATFORM_SAMPLES_DIR}/utils
SRC_PATH += $(shell find ${PLATFORM_SAMPLES_DIR}/vp_sensors -maxdepth 1 -type d)
```

- **Compile all examples together**

	Currently not supported.

- **Compile individual examples**

1. Log in to the device.
2. Navigate to each example directory under `/app/multimedia_samples/` for independent development and compilation. For example, to compile the `sample_vin` example individually:
	```
	cd /app/multimedia_samples/sample_vin/get_vin_data
	make
	```
3. The compiled executable resides in the current directory and can be run directly. For example, `get_vin_data` is the compiled executable:
	```
	root@ubuntu:/app/multimedia_samples/sample_vin/get_vin_data# ls
	get_vin_data  get_vin_data.c  get_vin_data.o  Makefile
	```

## Common Issues

### Insufficient space in `/app` directory causing example execution failure

**Cause:** The `/app` directory is mounted from the app partition. Some example programs generate large amounts of video/image data during execution, potentially filling up the available space and causing unexpected behavior.

**Solution:** Copy the program to another directory for execution, or clean up unnecessary data.

### Kernel logs flooding the serial terminal

To reduce kernel log verbosity on the board and avoid excessive logging that interferes with observing program execution flow, run the following command:

```
echo 4 > /proc/sys/kernel/printk
```