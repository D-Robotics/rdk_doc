---
sidebar_position: 10
---

# 3D GPU Performance Testing

## Testing Principles
3D GPU performance testing is primarily used to evaluate GPU performance under various computational and rendering workloads. Based on different GPU application scenarios, common tests can be categorized as follows:
- **OpenGLES (Rendering Performance Test)**: Uses `glmark2` to assess the GPU's graphics rendering capabilities, such as geometry processing, texture fill rate, and shader performance.
- **OpenCL (Compute Performance Test)**: Uses `clpeak` to evaluate GPU performance in general-purpose computing (GPGPU) tasks, including floating-point operations, memory bandwidth, and integer computations.

## Test Preparation
1. The S100 development board contains the `clpeak` patch and build instructions. You can `cd` into the following directory to view them:
	```bash
	cd /app/chip_base_test/10_gpu_3d_test/
	ls clpeak
	0001-feat-build-add-aarch64-toolchain-for-S100-cross-comp.patch  Readme.md
	```

## Testing Methods

### Rendering Performance Testing Method
1. Install `glmark2` for testing:
   ```bash
   apt install glmark2-es2-wayland -y
   ```

2. Connect a monitor and enter the desktop environment:
	- On the login screen, click the small gear icon in the bottom-right corner and select either the GNOME or Ubuntu (Wayland) session.
	- The program outputs via HDMI to the monitor, allowing you to visually observe the 3D GPU rendering performance.

3. Run `glmark2-es2-wayland`.

4. The following results will be printed:
```bash
=======================================================
	glmark2 2021.02
=======================================================
	OpenGL Information
	GL_VENDOR:     ARM
	GL_RENDERER:   Mali-G78AE
	GL_VERSION:    OpenGL ES 3.2 v1.r50p0-00eac0.5cc01fb93091eb33e0f41313c0fa19a5
=======================================================
[build] use-vbo=false: FPS: 2490 FrameTime: 0.402 ms
[build] use-vbo=true: FPS: 2403 FrameTime: 0.416 ms
[texture] texture-filter=nearest: FPS: 2882 FrameTime: 0.347 ms
[texture] texture-filter=linear: FPS: 2648 FrameTime: 0.378 ms
[texture] texture-filter=mipmap: FPS: 2704 FrameTime: 0.370 ms
[shading] shading=gouraud: FPS: 2218 FrameTime: 0.451 ms
[shading] shading=blinn-phong-inf: FPS: 2339 FrameTime: 0.428 ms
[shading] shading=phong: FPS: 2211 FrameTime: 0.452 ms
[shading] shading=cel: FPS: 2292 FrameTime: 0.436 ms
[bump] bump-render=high-poly: FPS: 1530 FrameTime: 0.654 ms
[bump] bump-render=normals: FPS: 3052 FrameTime: 0.328 ms
[bump] bump-render=height: FPS: 3011 FrameTime: 0.332 ms
[effect2d] kernel=0,1,0;1,-4,1;0,1,0;: FPS: 2187 FrameTime: 0.457 ms
[effect2d] kernel=1,1,1,1,1;1,1,1,1,1;1,1,1,1,1;: FPS: 826 FrameTime: 1.211 ms
[pulsar] light=false:quads=5:texture=false: FPS: 2928 FrameTime: 0.342 ms
[desktop] blur-radius=5:effect=blur:passes=1:separable=true:windows=4: FPS: 723 FrameTime: 1.383 ms
[desktop] effect=shadow:windows=4: FPS: 1914 FrameTime: 0.522 ms
[buffer] columns=200:interleave=false:update-dispersion=0.9:update-fraction=0.5:update-method=map: FPS: 711 FrameTime: 1.406 ms
[buffer] columns=200:interleave=false:update-dispersion=0.9:update-fraction=0.5:update-method=subdata: FPS: 684 FrameTime: 1.462 ms
[buffer] columns=200:interleave=true:update-dispersion=0.9:update-fraction=0.5:update-method=map: FPS: 947 FrameTime: 1.056 ms
[ideas] speed=duration: FPS: 1532 FrameTime: 0.653 ms
[jellyfish] <default>: FPS: 1360 FrameTime: 0.735 ms
[terrain] <default>: FPS: 128 FrameTime: 7.812 ms
[shadow] <default>: FPS: 1909 FrameTime: 0.524 ms
[refract] <default>: FPS: 328 FrameTime: 3.049 ms
[conditionals] fragment-steps=0:vertex-steps=0: FPS: 3037 FrameTime: 0.329 ms
[conditionals] fragment-steps=5:vertex-steps=0: FPS: 2837 FrameTime: 0.352 ms
[conditionals] fragment-steps=0:vertex-steps=5: FPS: 3015 FrameTime: 0.332 ms
[function] fragment-complexity=low:fragment-steps=5: FPS: 2964 FrameTime: 0.337 ms
[function] fragment-complexity=medium:fragment-steps=5: FPS: 2603 FrameTime: 0.384 ms
[loop] fragment-loop=false:fragment-steps=5:vertex-steps=5: FPS: 2922 FrameTime: 0.342 ms
[loop] fragment-steps=5:fragment-uniform=false:vertex-steps=5: FPS: 2921 FrameTime: 0.342 ms
[loop] fragment-steps=5:fragment-uniform=true:vertex-steps=5: FPS: 2819 FrameTime: 0.355 ms
=======================================================
								  glmark2 Score: 2093
=======================================================
```

### Compute Performance Testing Method
1. Execute the following commands:
	```sh
	cd clpeak
	./clpeak
	```

2. The following results will be printed:
	```sh
	Platform: ARM Platform
	Device: Mali-G78AE r0p1
		Driver version  : 3.0 (Linux ARM64)
		Compute units   : 2
		Clock frequency : 800 MHz

		Global memory bandwidth (GBPS)
		float   : 10.81
		float2  : 11.32
		float4  : 11.44
		float8  : 9.91
		float16 : 4.91

		Single-precision compute (GFLOPS)
		float   : 100.67
		float2  : 101.59
		float4  : 101.58
		float8  : 101.26
		float16 : 100.54

		Half-precision compute (GFLOPS)
		half   : 101.24
		half2  : 201.40
		half4  : 201.98
		half8  : 200.92
		half16 : 200.11

		No double precision support! Skipped

		Integer compute (GIOPS)
		int   : 25.53
		int2  : 25.53
		int4  : 25.53
		int8  : 25.53
		int16 : 25.54

		Integer compute Fast 24bit (GIOPS)
		int   : 25.52
		int2  : 25.53
		int4  : 25.54
		int8  : 25.53
		int16 : 25.53

		Integer char (8bit) compute (GIOPS)
		char   : 25.53
		char2  : 50.96
		char4  : 101.64
		char8  : 101.65
		char16 : 101.63

		Integer short (16bit) compute (GIOPS)
		short   : 25.53
		short2  : 50.97
		short4  : 50.98
		short8  : 50.99
		short16 : 50.96

		Transfer bandwidth (GBPS)
		enqueueWriteBuffer              : 13.01
		enqueueReadBuffer               : 13.02
		enqueueWriteBuffer non-blocking : 13.06
		enqueueReadBuffer non-blocking  : 13.04
		enqueueMapBuffer(for read)      : 21648.02
			memcpy from mapped ptr        : 13.06
		enqueueUnmap(after write)       : 31956.60
			memcpy to mapped ptr          : 12.89

		Kernel launch latency : 96.12 us
	```

## Test Metrics

### Rendering Performance Metrics

Higher FPS (frames per second) values in the test results indicate better GPU performance in that specific test scenario. Below is a line-by-line analysis of the test results:

#### **General Rendering Tests**

| Test Item | FPS | Description |
|-----------|-----|-------------|
| **use-vbo=false** | 2490 | Performance without VBO (Vertex Buffer Object) |
| **use-vbo=true** | 2403 | VBO enabled, reducing CPU-GPU interaction, slightly higher efficiency |
| **texture-filter=nearest/linear/mipmap** | 2648~2882 | Different texture filtering methods have minimal performance impact; all run efficiently |
| **shading=gouraud/phong/blinn-phong-inf/cel** | 2211~2339 | Shader switching causes minor performance differences |
| **bump-render=high-poly/normals/height** | 1530~3052 | Normal mapping techniques show significant performance variation |

#### **Complex Computation Tests**

| Test Item | FPS | Description |
|-----------|-----|-------------|
| **effect2d (convolution kernel filtering)** | 2187 / 826 | Large convolution kernels in 2D image processing significantly impact performance |
| **desktop (window blur/shadow)** | 723 / 1914 | Blur and shadow effects impose notably different GPU loads |
| **buffer (large-scale data updates)** | 684 / 711 / 947 | Data update strategies greatly affect performance |
| **ideas (particle system)** | 1532 | Dynamic particle animation performs well |
| **jellyfish (jellyfish simulation)** | 1360 | Tests GPU animation and complex deformation performance |
| **terrain (terrain rendering)** | 128 | High terrain complexity leads to significant FPS drop |
| **refract (refraction)** | 328 | Simulating light refraction incurs high computational cost |

#### **Compute-Oriented Tests**

| Test Item | FPS | Description |
|-----------|-----|-------------|
| **conditionals (branching computation)** | 2837~3037 | GPU handles conditional logic very efficiently |
| **function (increasingly complex fragment computation)** | 2603~2964 | More complex shader functions lead to performance degradation |
| **loop (loop computation)** | 2819~2922 | GPU demonstrates stable and robust loop execution capability |

- This score demonstrates the Mali-G78AE GPU’s strong capabilities in modern graphics rendering tasks, especially under high-resolution scenarios. It exhibits excellent floating-point computation and parallel processing performance across VBO usage, texture sampling, conditional logic, and loop computations.

### Compute Performance Metrics

#### Memory Bandwidth (GBPS)

```bash
float   : 10.81
```float2  : 11.32  
float4  : 11.44  
float8  : 9.91  
float16 : 4.91  
```

- Memory bandwidth initially increases and then decreases as vector size grows, with float4 achieving the highest bandwidth performance.  
- Overall memory bandwidth performance is excellent, making it suitable for large-scale data throughput scenarios.

#### Single-Precision Compute Performance (GFLOPS)

```bash
float   : 100.67  
float2  : 101.59  
float4  : 101.58  
float8  : 101.26  
float16 : 100.54  
```

- Single-precision performance remains stable across all vector widths, indicating strong SIMD parallel capability in the compute cores.  
- Performance exceeds 100 GFLOPS, significantly outperforming the average embedded GPU, making it well-suited for general-purpose graphics and compute tasks.

#### Half-Precision Compute Performance (GFLOPS)

```bash
half   : 101.24  
half2  : 201.40  
half4  : 201.98  
half8  : 200.92  
half16 : 200.11  
```

- Half-precision compute performance is substantially higher than single-precision, with half4 and wider configurations exceeding 200 GFLOPS.  
- Highly suitable for applications requiring high throughput but lower precision, such as AI inference and image preprocessing.

#### Integer Compute Performance (GIOPS)

```bash
int   : 25.53  
int2  : 25.53  
int4  : 25.53  
int8  : 25.53  
int16 : 25.54  
```

- Integer compute performance remains stable and does not scale with vector width.  
- Well-suited for logic operations, encryption, indexing, and similar workloads.

#### Byte and Short Integer Compute Performance (GIOPS)

```bash
char16 : 101.63  
short16: 50.96  
```

- 8-bit (char) integer performance peaks at over 101 GIOPS, while 16-bit (short) integer performance approaches 51 GIOPS.  
- The GPU is highly efficient when processing 8-bit data such as image pixels.

#### Transfer Bandwidth (GBPS)

```bash
enqueueWriteBuffer              : 13.01  
enqueueReadBuffer               : 13.02  
enqueueWriteBuffer non-blocking : 13.06  
enqueueReadBuffer non-blocking  : 13.04  
enqueueMapBuffer(for read)      : 21648.02  
  memcpy from mapped ptr        : 13.06  
enqueueUnmap(after write)       : 31956.60  
  memcpy to mapped ptr          : 12.89  
```

- Bandwidth between GPU and main memory is solid, with stable read/write performance around 13 GBPS.  
- It is recommended to fully leverage non-blocking transfers and the Map/Unmap mechanism to improve overall performance.

#### Kernel Launch Latency

```bash
Kernel launch latency : 96.12 us  
```

- Kernel launch latency is relatively high, possibly due to driver scheduling or OpenCL runtime overhead.  
- This may create a performance bottleneck for real-time computing applications (e.g., computer vision); thus, minimizing the number of kernel invocations is advised.

## Common Issues

1. Rendering performance test fails during execution  
- Issue description: Error log as follows

```bash
root@ubuntu:~#  glmark2-es2-wayland  
Error: main: Could not initialize canvas  
```

- Root cause: No suitable display connector detected.  
- Solution: Connect an HDMI monitor and run the test from the desktop environment.