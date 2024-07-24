---
sidebar_position: 7
---

# CPU性能测试

## 测试说明

本测试使用`Coremark`工具进行测试，源码和编译好的软件放在`10-cpu_performace`目录下。
CoreMark是一项基准测试程序，其主要目标是测试处理器核心性能，CoreMark标准的测试方法就是在某配置参数组合下单位时间内跑了多少次CoreMark程序，
业界的分数呈现为 `Coremark` / `CPU clock Mhz` / `Core num`，即 `coremark每秒跑的次数` / `cpu时钟频率` / `cpu的核数`，最终得到一个评分。

## 测试方法

1. 解压`coremark-main.zip`，并进入`coremark-main`文件夹
2. 执行`./coremark_single 0x0 0x0 0x66 0 7 1 2000 > ./run1.log`，等待程序执行完毕;执行`./coremark_multi 0x0 0x0 0x66 0 7 1 2000 > ./run2.log`，等待程序执行完毕。
3. 查看`run1.log`里面的**单核**测试成绩，参考如下：

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

注意到`Iterations/Sec`这栏，表示每秒钟迭代多少次，也就是我们上面公式的`coremark`分数。  
根据公式，这颗x3的单核分数为`5280.844935`/`1200`（默认频率）/`1` = `4.400`。属于正常范围。  

`./run2.log`里面保存着**多核心**的成绩，计算多核分数和单核分数类似，此处不再赘述。  

## 测试指标

- 单核分数 X > 4.2
- 四核分数 X > 4.2

## 附录

交叉编译`coremark`流程如下：

1. 进入`coremark-main`目录，将`aarch64/core_portme.mak`中的`CC`编译器路径换成自己的用于交叉编译的`gcc`路径。
2. 执行`make PORT_DIR=aarch64 XCFLAGS="-O3 -funroll-all-loops -static --param max-inline-insns-auto=550 -DPERFORMANCE_RUN=1" REBUILD=1 run1.log`编译**单核**测试程序；执行`make PORT_DIR=aarch64 XCFLAGS="-O3 -funroll-all-loops -static --param max-inline-insns-auto=550 -DPERFORMANCE_RUN=1 -DMULTITHREAD=4  -DUSE_PTHREAD -pthread" REBUILD=1 run1.log`编译生成**4核**测试程序，其中`-DMULTITHREAD=`参数用于控制生成几核心的测试程序。