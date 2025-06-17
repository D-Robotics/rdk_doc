---
sidebar_position: 7
---
# CPU Performance Test

## Test Description

This test uses the `Coremark` tool for testing. The source code and compiled software are located in the `10-cpu_performance` directory. CoreMark is a benchmark test program that mainly aims to test the performance of processor cores. The standard testing method of CoreMark is to run the CoreMark program multiple times within a unit of time under certain configuration parameters, and the industry score is presented as `Coremark` / `CPU clock Mhz` / `Core num`, that is, `coremark runs per second` / `cpu clock frequency` / `number of cpu cores`, resulting in a final score.

## Test Method

1. Unzip `coremark-main.zip` and enter the `coremark-main` folder.
2. Execute `./coremark_single 0x0 0x0 0x66 0 7 1 2000 > ./run1.log`, wait for the program to finish; execute `./coremark_multi 0x0 0x0 0x66 0 7 1 2000 > ./run2.log`, wait for the program to finish.
3. Check the **single core** test score in `run1.log`, as follows:

```yaml
2K performance run parameters for coremark.
CoreMark Size    : 666
Total ticks      : 20830
Total time (secs): 20.830000
Iterations/Sec   : 5280.844935
Iterations       : 110000
Compiler version : GCC6.5.0
Compiler flags   :  -O3 -funroll-all-loops -static --param max-inline-insns-auto=550 -DPERFORMANCE_RUN=1  -lrt
Memory location  : Please put data memory location here
                        (e.g. code in flash, data on heap etc)
seedcrc          : 0xe9f5
[0]crclist       : 0xe714
[0]crcmatrix     : 0x1fd7
[0]crcstate      : 0x8e3a
[0]crcfinal      : 0x33ff
Correct operation validated. See README.md for run and reporting rules.
CoreMark 1.0 : 5280.844935 / GCC6.5.0  -O3 -funroll-all-loops -static --param max-inline-insns-auto=550 -DPERFORMANCE_RUN=1  -lrt / Heap
```

Note the column `Iterations/Sec`, which indicates how many iterations per second, which is the `coremark` score in the formula above. 
According to the formula, the single-core score for this x3 is `5280.844935` / `1200` (default frequency) / `1` = `4.400`, which falls within the normal range.

The **multi-core** score is saved in `./run2.log`. The calculation of the multi-core score is similar to that of the single-core score and is not repeated here.

## Test Metrics

- Single core score X > 4.2
- Quad-core score X > 4.2

## AppendixThe cross-compilation process for `coremark` is as follows:

1. Go to the `coremark-main` directory and replace the `CC` compiler path in `aarch64/core_portme.mak` with your own `gcc` path for cross-compilation.
2. Execute `make PORT_DIR=aarch64 XCFLAGS="-O3 -funroll-all-loops -static --param max-inline-insns-auto=550 -DPERFORMANCE_RUN=1" REBUILD=1 run1.log` to compile the **single-core** test program; execute `make PORT_DIR=aarch64 XCFLAGS="-O3 -funroll-all-loops -static --param max-inline-insns-auto=550 -DPERFORMANCE_RUN=1 -DMULTITHREAD=4  -DUSE_PTHREAD -pthread" REBUILD=1 run1.log` to compile the **4-core** test program, where the `-DMULTITHREAD=` parameter controls the number of cores for the test program.