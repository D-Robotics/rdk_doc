---
sidebar_position: 3
---

# 串口压力测试

## 测试原理

串口压测（ UART Stress Test）是一种通过大量数据交换和高负载操作来验证串口通信稳定性和性能的方法。它的原理主要是通过发送和接收大量的数据、模拟真实的串口使用场景，具体原理包括以下几个方面：

- 数据发送与接收：串口压测的核心原理是通过将大量的数据发送到串口设备，然后再从串口设备接收数据。
  - 发送数据：通过脚本或程序生成大规模的数据包，通过串口接口发送到目标设备。
  - 接收数据：目标设备将接收到的数据返回（例如通过串口的回环测试或外部设备响应），压测程序会接收这些数据并进行比对和验证。
- 波特率和数据帧设置：波特率（ Baud Rate）是串口通信的传输速率，通常以每秒传输的比特数（ bps）表示，串口通信还包括其他参数，如：
  - 数据位（ Data bits）：每个数据帧中的数据位数（常见有 8 位、 7 位等）。
  - 停止位（ Stop bits）：指示数据传输结束的位（通常为 1 或 2 位）。
  - 校验位（ Parity bits）：用于检查数据传输中的错误（奇偶校验）。

### 测试内容

**1. 测试过程：**
`uartstress.sh` 脚本使用的回环测试（ Loopback Test）是串口测试中常见的一种方法，其原理是将发送端的数据通过物理连接或其他方式发送回接收端，从而验证数据传输的正常与否。在源码 `uart_test.c` 中， `perform_single_loopback_test()` 函数实现了这种测试。其步骤如下：

- 打开串口并设置相关参数 `open_uart()`。
- 初始化信号量 `sem_init()`，确保线程在合适的时机进行同步。
- 创建发送线程 `uart_send_thread`，接收线程 `uart_recv_thread`，和数据检查线程 `check_recv_thread`。
  - 发送线程负责发送数据。
  - 接收线程负责接收数据并存储到缓冲区。
  - 数据检查线程负责比较接收到的数据与发送的数据是否一致。
- 当所有线程都完成工作时， `pthread_join()` 等待所有线程结束，然后结束测试。

**2. 命令解析：**

- 测试命令：`uart_test" -l -s 1024 -c "$StressCount" -b "$Baudrate" -d "$Device" > "$uart_test_log_file"`
- 参数解析：
  - `-l`：执行回环测试（ Loopback Test ）。
  - `-s 1024`：指定每次测试的数据大小为 1024 字节。
  - `-c "$StressCount"`：指定压力测试的次数，即测试重复的次数。
  - `-b "$Baudrate"`：设置串口通信的波特率，变量 $Baudrate 会根据实际传入的值设置。
  - `-d "$Device"`：指定串口设备，变量 $Device 是串口设备路径。
  - `-> "$uart_test_log_file"`：将测试结果输出到指定的日志文件 "$uart_test_log_file"。

## 准备工作

### 压测脚本使用说明

串口压测支持输入后缀 -h 查看命令参数的说明 ，例如：

```shell
sunrise@ubuntu:/app/chip_base_test/03_uart_test$ ./uartstress.sh -h
Usage: ./uartstress.sh [options]

Options:
  -b <baudrate>    Set the UART baud rate (default: 115200).
  -d <device>      Set the UART device (default: /dev/ttyS2).
  -c <count>       Set the stress count (default: 100).
  -o <directory>   Set the output directory for logs (default: ../log).
  -h               Show this help message and exit.
```

各参数解析如下：

- `-b <baudrate>`：设置波特率，默认值是 115200 。。
- `-d <device>`：该选项用于指定测试的串口设备，默认值为 /dev/ttyS2 。
- `-c <count>`：指定了测试的压力次数，默认值是 100 。
- `-o <directory>`：设置日志输出目录，默认值为 ../log。

**示例：**
例如，使用命令： `./uartstress.sh -b 115200 -d /dev/ttyS2 -c 50 -o /app/chip_base_test` 自定义波特率为 115200 ，串口设备为 ttyS2( 与实际使用 uart 对应 )，循环次数为 50 次，输出目录为 /app/chip_base_test 。

### 执行程序使用说明

`uart_test` 支持多种测试方法，支持的命令行选项如下：

```shell
Usage: uart_test [OPTIONS]
Options:
  -s, --size      : 指定测试数据大小（ KB），默认为 1024KB，最大为 20480KB（ 20MB）
  -b, --baudrate  : 指定 UART 的波特率，默认为 115200
  -c, --count     : 指定测试迭代次数，默认为无限循环
  -d, --device    : 指定 UART 设备路径
  -l, --loopback  : 启用 UART 回环测试（串口回环）
  -r, --read-only : 启用 UART 只读模式测试
  -w, --write-only: 启用 UART 只写模式测试
  -D, --dual-loopback : 启用 UART 双路双通回环测试（需要使用 --uart2 参数）
  -u, --uart2     : 指定第二个 UART 设备路径（双路回环测试必须）
  -V, --verbose   : 设置日志输出模式，按照对 bit 位的与运算输出不同模块的调试日志，目前支持 1: 输出发送模块的日志； 2 ：输出接收模块的日志； 4 ：输出数据校验模块的日志。数字相加后可以输出多个模块的日志，例如设置 3 则会输出发送与接收两个模块的日志
  -h, --help      : 显示帮助信息
```

如要更改测试模式，只需将脚本 `uartstress.sh` 中 `-l` 改成 其他测试模式即可，例如，（ UART 只写模式测试）：

```shell
"${script_dir}/uart_test" -w -s 1024 -c "$StressCount" -b "$Baudrate" -d "$Device" > "$uart_test_log_file"
```

之后将 uart2_rx 与 uart2_tx 通过 ttl 串口转接模块连接至 pc 端，打开串口工具即可接收发送内容。

```shell
Test uart device:/dev/ttyS2
Test size: 2 KBytes, baudrate: 115200
Performing uart send...
Starting send thread
This is uart send test 1 times
This is uart send test 2 times
This is uart send test 3 times
```

### 注意事项

为了 RDK_S100 接口良好的扩展性，目前 i2c5 和 uart2 是可以通过拨码开关进行切换的，运行该测试时需要修改设备树管脚复用关系重新编译dtb安装，并把拨码开关拨到正确的位置。

**1.** 测试 uart2 需要修改设备树如下：

```diff
diff --git a/kernel-dts/rdk-v0p5.dtsi b/kernel-dts/rdk-v0p5.dtsi
index 504b21b..8d72794 100644
--- a/kernel-dts/rdk-v0p5.dtsi
+++ b/kernel-dts/rdk-v0p5.dtsi
@@ -210,6 +210,10 @@
        status = "okay";

 };

+&i2c5 {
+       status = "disabled";
+};

 &i2c4 {
        status = "okay";
@@ -267,6 +271,10 @@
        };
 };

+&uart2 {
+       status="okay";
+};
+
```

**2.** 根据下面的 RDK_S100 实物图并找到 uart2 对应并将拨码开关往右拨到 连接 uart2 的位置：

![Actual_device_diagram](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/images_to_upload/Actual_device_diagram.png)

**3.** 查看 RDK_S100 原理图并找到 uart2 对应的引脚与连接器位置，如图：

![Schematic_diagram](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/images_to_upload/Schematic_diagram.png)


并将双母头杜邦线将 uart2_tx 与 uart2_rx 相连。

![Connection_diagram](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/images_to_upload/Connection_diagram.png)

**4.** 确认在 /app/chip_base_test/03_uart_test 路径下存在 `uartstress.sh`、`uart_test.c`、`uart_test` 三个文件。

```shell
sunrise@ubuntu:/app/chip_base_test/03_uart_test$ tree
.
├── Makefile
├── Readme.md
├── uart_test.c
└── uartstress.sh
```

**4.** 可使用编译命令重新生成执行文件，命令如下：

```shell
gcc -o uart_test uart_test.c
```


确保完成准备工作后，运行测试命令：

```shell
./uartstress.sh
```

运行一段时间后，日志打印结果如下：

```shell
sunrise@ubuntu:/app/chip_base_test/03_uart_test# ./uartstress.sh
Uart test starting...
Test configuration:
  Baudrate: 115200
  Device: /dev/ttyS2
  Stress count: 100
  Output directory: /app/chip_base_test/log
  Log file: /app/chip_base_test/log/uart_test_log2.txt
```

此时发现日志中没有打印其他信息，可直接在 /app/chip_base_test/log/ 路径下，查看日志。

```shell
sunrise@ubuntu:# cat /app/chip_base_test/log/uart_test_log2.txt
Test uart device:/dev/ttyS2
Test size: 512 KBytes, baudrate: 115200
Performing uart recv...
Performing uart send...
Performing data check...
Starting send thread
Starting receive thread
This is receive test 1 times
This is uart send test 1 times
This is receive test 2 times
Data verification successful. Received data matches sent data. Test total data count: 0x80000
This is uart send test 2 times
This is receive test 3 times
Data verification successful. Received data matches sent data. Test total data count: 0x100000
This is uart send test 3 times
This is receive test 4 times
Data verification successful. Received data matches sent data. Test total data count: 0x180000
This is uart send test 4 times
```

## 测试指标

测试程序启动后会在 `/app/chip_base_test/log/` 目录下生成文件如下：

- uart_test_log*.txt：记录压测时的打印信息与当前状态。

测试目标是确保系统能够在 48 小时内稳定运行，不发生重启或挂死的情况。为确保测试过程中的稳定性，可通过以下命令检查日志文件中是否存在 `fail`、 `error`、 `timeout` 等异常信息：

```shell
cd "/app/chip_base_test/log/" && grep -iE 'error|fail|timeout' uart_test_log*.txt
```

### 串口压测结果

运行测试脚本结束后检测 log 日志，并未出现异常状态信息，说明压测合格。

```shell
This is receive test 48 times
Data verification successful. Received data matches sent data. Test total data count: 0x1780000
This is uart send test 48 times
This is receive test 49 times
Data verification successful. Received data matches sent data. Test total data count: 0x1800000
This is uart send test 49 times
This is receive test 50 times
Data verification successful. Received data matches sent data. Test total data count: 0x1880000
This is uart send test 50 times
```
