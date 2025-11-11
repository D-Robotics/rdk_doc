---
sidebar_position: 4
---

# SPI Stress Test

## Test Principle

The SPI bus test tool (`spidev_tc`) is used to verify the basic functionality and performance of SPI devices. It simulates high-load conditions by configuring various parameters to evaluate key performance metrics such as SPI bus stability, throughput, latency, and error rate.

### Test Content

**1. Test Procedure:**  
The core of the SPI stress test involves loopback testing for data transmission and reception. The main logic is reflected in the source code as follows:

- **Data Transfer Test**: The functions `transfer_buf` and `transfer_read_write` are used to perform pure data transmission, reading, and verification. The packet length is controlled by the `transfer_size` parameter, and the test runs repeatedly for a number of iterations (controlled by `iterations`).
- **Throughput and Rate Calculation**: Global variables `_read_count` and `_write_count` record the number of bytes read and written, respectively. The transfer rate is periodically calculated and printed. The function `show_transfer_rate` computes and outputs the throughput (in kbps).
- **Transfer Mode Switching**: By varying SPI mode, bit width, and speed parameters, the tool tests SPI device performance under different conditions. For example, users can adjust the data buffer size or SPI bus speed (`speed`) to evaluate device stability and performance under varying loads.

**2. Command Parsing:**

- **Test Command**:  
  `$\{script_dir}/spidev_tc" -D "$Device" -s "$spi_speed" -I "$StressCount" -e 3 -S 32 > "$spi_test_log_file" 2>&1`
- **Parameter Explanation**:
  - `-D "$Device"`: Specifies the SPI device file to operate on (e.g., `/dev/spidev2.0`).
  - `-s "$spi_speed"`: Sets the SPI bus communication speed in Hertz (Hz).
  - `-I "$StressCount"`: Specifies the number of test iterations.
  - `-e 3`: Specifies the test mode or type. The number following `-e` usually indicates different test types; for example, `3` in the script represents loopback testing.
  - `-S 32`: Specifies the data size per transfer in bytes.
  - `> "$spi_test_log_file" 2>&1`: Redirects command output.
    - `> "$spi_test_log_file"`: Redirects standard output (`stdout`) to the specified log file (`$spi_test_log_file`).
    - `2>&1`: Redirects standard error (`stderr`) to the same log file, ensuring all output (including error messages) is captured.

## Preparation

### <span id="test-script-user-manual"/> Stress Test Script Usage Instructions

The serial stress test script supports the `-h` suffix to display command parameter descriptions. For example:

```shell
sunrise@ubuntu:/app/chip_base_test/04_spi_test# ./spistress.sh -h
Usage: ./spistress.sh [options]

Options:
  -d <device>      Set the SPI device to test (default: /dev/spidev0.0).
  -c <count>       Set the stress test count (default: 100).
  -s <speed>       Set the SPI speed in Hz (default: 12000000).
  -o <directory>   Set the output directory for logs (default: '../log').
  -h               Show this help message and exit.
```

Parameter details:

- `-d <device>`: Specifies the SPI device to test. Default path is `/dev/spidev0.0`.
- `-c <count>`: Sets the number of stress test iterations.
- `-s <speed>`: Sets the SPI speed in Hz. Default value is 12000000 (i.e., 12 MHz).
- `-o <directory>`: Sets the log output directory. Default is `../log`.

**Example:**  
For instance, the command:  
`./spistress.sh -d /dev/spidev0.0 -c 500 -s 24000000 -o /userdata/spi_test_logs`  
customizes the test for SPI device `/dev/spidev0.0`, sets the transfer speed to 24 MHz, runs 500 iterations, and saves logs to `/app/chip_base_test`.

Detailed parameters and command explanations from the `spidev_tc` source code are as follows:

```shell
-D --device: Specify the SPI device to use.
-s, --speed: Set maximum speed (Hz).
-d, --delay: Set delay (microseconds).
-b, --bpw: Set bits per word.
-i, --input: Input data from a file (e.g., "test.bin").
-o, --output: Output data to a file (e.g., "results.bin").
-l, --loop: Enable loopback test.
-H, --cpha: Clock phase.
-O, --cpol: Clock polarity.
-L, --lsb: Least significant bit first.
-C, --cs-high: Chip select active high.
-3, --3wire: Shared SI/SO signal.
-v, --verbose: Verbose mode (display transmit buffer).
-p : Send data (e.g., "1234\xde\xad").
-N, --no-cs: Disable chip select.
-R, --ready: Slave pulls low to pause.
-2, --dual: Dual-line transfer.
-4, --quad: Quad-line transfer.
-S, --size: Specify test data size (bytes).
-I, --iter: Number of iterations; defaults to infinite loop when `-e` extended test mode is set.
-e, --mode: Specify test mode: 1=read, 2=write, 3=read and write.
-h, --help: Display help information.
```

**1.** Refer to the RDK_S100 schematic to locate the pins and connector positions for SPI0_MOSI and SPI0_MISO, as shown below:

![Spi_Schematic_diagram](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/images_to_upload/Spi_Schematic_diagram.png)

Then, connect SPI0_MOSI and SPI0_MISO using a female-to-female Dupont jumper wire, as illustrated:

![Spi_Connection_diagram](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/images_to_upload/Spi_Connection_diagram.png)

:::caution Note
The RDKS100 Acore supports two SPI interfaces, and both SPI0 and SPI1 can only operate as SPI Masters.  
Internal SPI loopback testing is only supported in SPI Master mode. The principle is that the SPI hardware IP’s TX FIFO sends data directly to the RX FIFO, forming a loopback.
:::

**3.** Confirm that the files `spistress.sh`, `spidev_tc.c`, and `spidev_tc` exist either in the SDK path `/app/chip_base_test/04_spi_test` or on the board at `sunrise@ubuntu:/app/chip_base_test/04_spi_test#`.

```shell
(base) root@//app/chip_base_test/04_spi_test# tree
.
├── spidev_tc
├── spidev_tc.c
└── spistress.sh
```

**4.** If users need to customize test modes or features, it is recommended to rebuild the executable in the SDK compilation environment using the following command:

```shell
gcc -o spidev_tc spidev_tc.c
```

## Test Procedure

After completing the preparations, run the test command:

```shell
./spistress.sh
```

After running for a while, the log output appears as follows:

```shell
sunrise@ubuntu:/app/chip_base_test/04_spi_test# ./spistress.sh
SPI test starting...
Test configuration:
  Device: /dev/spidev0.0
  Stress Count: 100
  SPI Speed: 12000000 Hz
  Output Directory: /app/chip_base_test/log
  Log file: /app/chip_base_test/log/spi_test_log3.txt
SPI test completed successfully! Log saved to: /app/chip_base_test/log/spi_test_log1.txt
```

If no additional information appears in the console output, check the log file `spi_test_log1.txt` directly under `/app/chip_base_test/log/`:

```shell
sunrise@ubuntu:/app/chip_base_test# cat log/spi_test_log1.txt
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

## Test Metrics

After the test program starts, the following files are generated in the `/app/chip_base_test/log/` directory:

- `spi_test_log*.txt`: Records printed messages and current status during stress testing.

The test objective is to ensure the system can run stably for 48 hours without rebooting or hanging. To verify stability during testing, use the following command to check for abnormal keywords such as `fail`, `error`, or `timeout` in the log files:

```shell
cd "/app/chip_base_test/log/" && grep -iE 'error|fail|timeout' spi_test_log*.txt
```

### SPI Stress Test Results

After the test script completes, inspect the log files. If no abnormal status messages appear, the stress test passes.

```shell
Test times: 97 Data verification Successful
Test times: 98 Data verification Successful
Test times: 99 Data verification Successful
```

### Notes

#### SPI Internal Loopback Test
**1.** For internal SPI loopback testing, simply modify the `-e` parameter in the script `./spistress.sh` to select a different test mode. For example, to perform an SPI read-write mode test:

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

#### SPI External Loopback Test

Prepare an RDK_S100 development board and wire out the four SPI lines. The Master can use SPI0, while the SPI Slave uses an external SPI device (selected by the user).

![SPI_wiring_diagram](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/images_to_upload/SPI_wiring_diagram.png)

Ensure that the SPI baud rate and transfer mode settings match between Master and Slave. On the Slave device side, first execute the data reception test command;```shell
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

:::caution Note
Performance Note: The reference maximum SPI communication rate is 30 Mbps. The actual rate may vary due to system load and other factors; therefore, real-world testing is recommended. If you encounter FIFO overrun/underrun warnings during testing, consider reducing the communication speed.

Note: When performing an external loopback test, execute the SPI Slave program first, followed by the SPI Master program. If the SPI Master program runs before the SPI Slave program, data loss may occur due to lack of synchronization between Master and Slave.
:::