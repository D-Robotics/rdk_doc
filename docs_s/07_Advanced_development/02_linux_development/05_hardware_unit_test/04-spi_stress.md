---
sidebar_position: 4
---

# SPI 压力测试

## 测试原理

SPI 总线测试工具（ spidev_tc ）用于测试 SPI 设备的基本功能和性能，主要通过配置不同的参数来模拟高负载环境，验证 SPI 总线的稳定性、吞吐量、延迟、错误率等关键性能指标。

### 测试内容

**1. 测试过程：**
SPI 压力测试的核心部分是通过回环测试进行数据的传输和接收，源码中体现主要内容如下：

- 数据传输测试：使用 `transfer_buf` 和 `transfer_read_write` 函数分别进行纯数据发送、读取和验证。数据包的长度由 `transfer_size` 参数控制，并且测试会多次循环进行（由 iterations 控制）。
- 吞吐量和速率计算：通过全局变量 `_read_count` 和 `_write_count` 记录读取和写入的字节数，并周期性计算和输出传输速率。`show_transfer_rate` 函数用于计算并输出吞吐量（单位： kbps）。
- 传输模式切换：通过 SPI 模式、位数和速度等参数的不同组合，测试 SPI 设备在不同条件下的性能表现。例如，可以设置数据 buffer 的大小或者 SPI 总线速度（ speed），测试设备在不同条件下的稳定性和性能。

**2. 命令解析：**

- 测试命令：`$\{script_dir}/spidev_tc" -D "$Device" -s "$spi_speed" -I "$StressCount" -e 3 -S 32 > "$spi_test_log_file" 2>&1`
- 参数解析：
  - `-D "$Device"`：该选项指定了要操作的 SPI 设备文件（如 /dev/spidev2.0 ）。
  - `-s "$spi_speed"`：该选项指定了 SPI 总线的通信速度（单位是赫兹， Hz）。
  - `-I "$StressCount"`：该选项指定了测试的次数。
  - `-e 3`：该选项指定了测试模式或类型，-e 后面的数字通常表示不同的测试类型，例如，脚本中 3 代表回环测试。
  - `-S 32`：该选项指定每次测试的传输数据大小，单位是字节。
  - `> "$spi_test_log_file" 2>&1`：这是命令行的输出重定向部分。
    - > "$spi_test_log_file"：将标准输出（ stdout）重定向到指定的日志文件（$spi_test_log_file）。
    - 2>&1 ：将标准错误输出也会重定向到同一个日志文件，这样所有输出（包括错误信息）都会被记录到日志文件中。

## 准备工作

### <span id="test-script-user-manual"/> 压测脚本使用说明

串口压测支持输入后缀 -h 查看命令参数的说明 ，例如：

```shell
root@buildroot:/app/multimedia_samples/chip_base_test/04_spi_test# ./spistress.sh -h
Usage: ./spistress.sh [options]

Options:
  -d <device>      Set the SPI device to test (default: /dev/spidev0.0).
  -c <count>       Set the stress test count (default: 100).
  -s <speed>       Set the SPI speed in Hz (default: 12000000).
  -o <directory>   Set the output directory for logs (default: '../log').
  -h               Show this help message and exit.
```

各参数解析如下：

- `-d <device>`：指定要测试的 SPI 设备，默认设备路径为 `/dev/spidev0.0` 。
- `-c <count>`：设置压力测试的次数。
- `-s <speed>`：设置 SPI 的速度，单位是 Hz，默认值为 12000000 （即 12 MHz）。
- `-o <directory>`：设置日志输出目录，默认值为 ../log。

**示例：**
例如，使用命令： `./spistress.sh -d /dev/spidev0.0 -c 500 -s 24000000 -o /userdata/spi_test_logs` 自定义测试 SPI 设备 /dev/spidev0.0 ，设置传输速度为 24 MHz，进行 500 次测试，输出目录为 /app/multimedia_samples/chip_base_test 。

`spidev_tc` 源码中的详细参数与设置命令解析如下：

```shell
-D --device: 指定使用的 SPI 设备。
-s, --speed: 设置最大速度（ Hz）。
-d, --delay: 设置延迟（微秒）。
-b, --bpw: 设置每字的位数。
-i, --input: 从文件中输入数据（例如："test.bin"）。
-o, --output: 将数据输出到文件（例如："results.bin"）。
-l, --loop: 启用回环测试。
-H, --cpha: 时钟相位。
-O, --cpol: 时钟极性。
-L, --lsb: 低位优先。
-C, --cs-high: 片选信号为高电平有效。
-3, --3wire: SI/SO 信号共享。
-v, --verbose: 详细模式（显示发送缓冲区）。
-p : 发送数据（例如："1234\xde\xad"）。
-N, --no-cs: 禁用片选信号。
-R, --ready: 从机拉低以暂停。
-2, --dual: 双线传输。
-4, --quad: 四线传输。
-S, --size: 指定测试的数据大小（字节）。
-I, --iter: 迭代次数，当设置 `-e` 扩展测试模式时，默认是无限循环。
-e, --mode: 指定测试模式， 1: 读取， 2: 写入， 3: 读和写。
-h, --help: 显示帮助信息。
```

**1.** 查看 RDK_S100 原理图并找到 SPI0_MOSI 和 SPI0_MISO 对应的引脚与连接器位置，如图：

![Spi_Schematic_diagram](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/images_to_upload/Spi_Schematic_diagram.png)

并将双母头杜邦线将 SPI0_MOSI 与 SPI0_MISO 相连，位置如图：

![Spi_Connection_diagram](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/images_to_upload/Spi_Connection_diagram.png)

:::caution 注意
RDKS100 Acore支持2路SPI，且SPI0，SPI1只能做SPI Master。
SPI内部回环测试仅SPI Master支持，其原理是SPI硬件IP的tx fifo将数据发给rx fifo从而形成回环。
:::

**3.** 确认 SDK 中 `/app/multimedia_samples/chip_base_test/04_spi_test` 路径下，或者板端 `root@buildroot:/app/multimedia_samples/chip_base_test/04_spi_test#` 路径下存在 `spistress.sh`、`spidev_tc.c`、`spidev_tc` 三个文件。

```shell
(base) root@//app/multimedia_samples/chip_base_test/04_spi_test# tree
.
├── spidev_tc
├── spidev_tc.c
└── spistress.sh
```

**4.** 如果用户需要自定义测试模式和功能，建议在 SDK 编译环境中，使用编译命令重新生成执行文件，命令如下：

```shell
gcc -o spidev_tc spidev_tc.c
```

## 测试方法

确保完成准备工作后，运行测试命令：

```shell
./spistress.sh
```

运行一段时间后，日志打印结果如下：

```shell
root@buildroot:/app/multimedia_samples/chip_base_test/04_spi_test# ./spistress.sh
SPI test starting...
Test configuration:
  Device: /dev/spidev0.0
  Stress Count: 100
  SPI Speed: 12000000 Hz
  Output Directory: /app/multimedia_samples/chip_base_test/log
  Log file: /app/multimedia_samples/chip_base_test/log/spi_test_log3.txt
SPI test completed successfully! Log saved to: /app/multimedia_samples/chip_base_test/log/spi_test_log3.txt
```

此时发现日志中没有打印其他信息，可直接在 /app/multimedia_samples/chip_base_test/log/ 路径下，查看 spi_test_log1 日志。

```shell
root@buildroot:/app/multimedia_samples/chip_base_test# cat log/spi_test_log1.txt
spi mode: 0x0
bits per word: 8
max speed: 12000000 Hz (12000 kHz)
Userspace spi read and write test, test_len=32 iterations=100
Test times: 0 Data verification Successful
Test times: 1 Data verification Successful
Test times: 2 Data verification Successful
Test times: 3 Data verification Successful
.....
Test times: 98 Data verification Successful
Test times: 99 Data verification Successful
```

## 测试指标

测试程序启动后会在 `/app/multimedia_samples/chip_base_test/log/` 目录下生成文件如下：

- spi_test_log*.txt：记录压测时的打印信息与当前状态。

测试目标是确保系统能够在 48 小时内稳定运行，不发生重启或挂死的情况。为确保测试过程中的稳定性，可通过以下命令检查日志文件中是否存在 `fail`、 `error`、 `timeout` 等异常信息：

```shell
cd "/app/multimedia_samples/chip_base_test/log/" && grep -iE 'error|fail|timeout' spi_test_log*.txt
```

### SPI 压测结果

运行测试脚本结束后检测 log 日志，并未出现异常状态信息，说明压测合格。

```shell
Test times: 97 Data verification Successful
Test times: 98 Data verification Successful
Test times: 99 Data verification Successful
```

### 注意事项

#### SPI 内部回环测试
**1.** SPI 内部回环测试，只需将脚本 `./spistress.sh` 中 `-e` 改成其他测试模式的参数即可，例如，（ SPI 读写模式测试 ）：

```shell
./spidev_tc -D /dev/spidev0.0 -s 12000000 -I 1 -e 3 -S 32 -v
spi mode: 0x0
bits per word: 8
max speed: 12000000 Hz (12000 kHz)
Userspace spi read and write test, test_len=32 iterations=1
TX | 67 C6 69 73 51 FF 4A EC 29 CD BA AB F2 FB E3 46 7C C2 54 F8 1B E8 E7 8D 76 5A 2E 63 33 9F C9 9A  |g.isQ.J.)......F|.T.....vZ.c3...|
RX | 67 C6 69 73 51 FF 4A EC 29 CD BA AB F2 FB E3 46 7C C2 54 F8 1B E8 E7 8D 76 5A 2E 63 33 9F C9 9A  |g.isQ.J.)......F|.T.....vZ.c3...|
Test times: 0 Data verification Successful
```

#### SPI 外部回环测试

可以准备一块 RDK_S100 开发板，将SPI的4根线飞好。Master可以选择SPI0，SPI Slave选择外部SPI设备（客户自行选择）。

![SPI_wiring_diagram](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/images_to_upload/SPI_wiring_diagram.png)

保证Master和Slave的SPI波特率、传输模式等设置正确，Slave设备侧先执行接收数据测试命令；

```shell
./spidev_tc -D /dev/spidev0.0 -s 12000000 -I 1 -e 2 -S 32 -v
bits per word: 8
max speed: 1000000 Hz (1000 KHz)
userspace spi write test, len=10 times=1
test, times=0
TX | 67 C6 69 73 51 FF 4A EC 29 CD __ __ __ __ __ __ __ __ __ __ __ __ __ __ __
```

```shell
./spidev_tc -D /dev/spidev0.0 -s 12000000 -I 10 -e 1 -S 32 -v
spi mode: 0x0
bits per word: 8
max speed: 1000000 Hz (1000 KHz)
userspace spi read test, len=10 times=1
test, times=0
RX | 67 C6 69 73 51 FF 4A EC 29 CD __ __ __ __ __ __ __ __ __ __ __ __ __ __ __
```

:::caution 注意
性能说明：SPI最大通信速率参考值是30Mbps，实际可能受到系统压力等综合因素影响而上下浮动，因此建议以实测为准。如果测试出现FIFO overrun/underrun提示，建议尝试降低通信速率。

注：在进行外部回环测试时，需要先执行 SPI Slave 程序，再执行 SPI Master 程序。假如先执行 SPI Master 程序，后执行 SPI Slave 程序，可能会由于 Master 与 Slave 不同步导致 SPI 接收数据出现丢失。
:::
