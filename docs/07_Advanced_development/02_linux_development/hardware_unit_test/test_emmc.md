---
sidebar_position: 2
---

# EMMC相关测试

对于EMMC来讲，主要关心它的稳定性和性能。

## EMMC稳定性测试

### 测试方法

1. 使用开源工具`iozone`对EMMC进行文件系统读写测试。
2. 进入到`test_tools/02_emmc`文件夹下，执行`sh emmc_stability_test.sh &`脚本对EMMC文件系统读写测试。

### 测试标准

1. 高温：45°、低温：-10°、常温下，程序正常执行，不会出现重启挂死的情况。
2. LOG中没有`fail`、`error`、`timeout`等异常打印。
3. 能稳定运行48小时。

## EMMC性能测试

### 测试方法

1. 使用开源工具`iozone`对EMMC文件系统读写速度进行测试。
2. Read上限：172.8MB/s、Write上限：35MB/s。
3. 进入到`test_tools/02_emmc`文件夹下，执行`sh emmc_performance_test.sh &`脚本。

### 测试标准

1. 常温环境下，程序正常执行，不会出现重启挂死等异常。
2. LOG中无`fail`、`error`、`timeout`等异常打印。
3. 统计实际测试读写速度是否符合性能指标。
4. 稳定运行48小时。