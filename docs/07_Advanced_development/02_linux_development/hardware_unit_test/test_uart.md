---
sidebar_position: 3
---

# UART压力测试

## 测试方法

1. 镜像要打开`uart1`和`uart2`节点（双发双收），硬件上将`uart1`的`TX`、`RX`与`uart2`的`RX`、`TX`短接。
2. 执行测试脚本：`sh uart1test.sh &`、`sh uart2test.sh &`。

## 测试标准

1. 高温：45°、低温：-10°、常温下，程序正常执行，不会出现重启挂死的情况。
2. LOG中没有`fail`、`error`、`timeout`等异常打印。
3. 能稳定运行48小时。

## 附录

测试源码以及编译可以参考[UART驱动调试指南](