# AutoTest Usage Guide

AutoTest provides a flexible automated testing solution that supports customizing stress test duration and test iterations via the configuration file `config.ini`. It also allows users to extend test cases according to their specific requirements, thereby fulfilling diverse testing needs. Developed on top of driver unit tests, this tool implements automated testing functionality while remaining independent from functional unit tests and fully reusing existing code resources.

## AutoTest Directory Structure

AutoTest uses the `startup.sh` script to read the `config/config.ini` configuration file to launch tests. Users can customize test content by modifying the `config/config.ini` file. When performing stability or stress tests across multiple functionalities, this approach significantly reduces repetitive configuration efforts. Currently, AutoTest has already reused the following test items from driver functional unit tests:

```bash
#:~/sdk/source/hobot-multimedia-samples/debian/app/multimedia_samples/chip_base_test$ tree
.
├── 01_cpu_bpu_ddr
│   └── scripts
│       └── stress_test.sh         # CPU-BPU-DDR stress test
├── 02_emmc
│   └── emmc_stability_test.sh     # eMMC stability test
├── 03_uart_test
│   └── uartstress.sh              # UART stress test
├── 04_spi_test
│   └── spistress.sh               # SPI stress test
├── config
│   └── config.ini                 # Configuration file for automated testing
└── startup.sh                     # Script for auto-start configuration
```

## Description of the config.ini Configuration File

The `config.ini` file is used to configure and manage AutoTest test items. By adjusting parameters within this file, users can quickly select desired tests and specify test parameters. Below is the file structure and configuration instructions:

### Configuration File Structure

Each test item starts with `[Test Item Name]` and includes the following fields:

- **Status**: Enable status, which can be set to `enabled` (active) or `disabled` (inactive).
- **Description**: A brief description of the test item, explaining its purpose.
- **ExecStart**: Path to the test script along with execution parameters, defining how the specific test is run.

### Configuration File Example

```bash
[CpuAndBpu]
Status=enabled
Description=CPU, BPU, and DDR stress test
ExecStart=/app/chip_base_test/01_cpu_bpu_ddr/scripts/stress_test.sh -t 24h

[EmmcStablity]
Status=enabled
Description=eMMC stability test
ExecStart=/app/chip_base_test/02_emmc/emmc_stability_test.sh -t 24h

[UART]
Status=enabled
Description=UART stress test
ExecStart=/app/chip_base_test/03uart_test/uartstress.sh -b 115200 -d /dev/ttyS2 -c 1000000

[SPI]
Status=enabled
Description=SPI stress test
ExecStart=/app/chip_base_test/04_spi_test/spistress.sh -d /dev/spidev0.0 -c 1000000
```

### Usage Instructions

1. **Edit the configuration file:**
   Open and edit the `config/config.ini` file, adjusting the `Status` value for each test item as needed:
   - Set to `enabled`: activate the test item.
   - Set to `disabled`: deactivate the test item.

2. **Modify test parameters:**
   Adjust the command and parameters in `ExecStart` according to actual requirements. For example:
   - Modify test duration (e.g., `-t 24h` means 24 hours of testing).
   - Adjust device paths (e.g., `/dev/ttyS2` or `/dev/spidev0.0`).
   - Tune other test options (e.g., loop count `-c` or baud rate `-b`).
   - All current test scripts support the `-h` option, allowing users to consult help information for parameter adjustments.

## Launching Tests with startup.sh

After configuring `config.ini`, users can start AutoTest by running the `startup.sh` script. The script supports multiple launch methods to meet different usage scenarios:

- **Manual launch or debugging of test items**

  Suitable for scenarios where the official system image is used and manual execution or debugging of test items is required. Execute the following command to start testing:

  ```bash
  /app/chip_base_test/startup.sh
  # or
  cd /app/chip_base_test
  ./startup.sh
  ```

  This method facilitates debugging individual test items or observing test execution.

- **Auto-start or debugging of test items**

  Once test items have been configured and debugged successfully, `startup.sh` can also be added to the `rc.local` auto-start script to enable automatic testing upon board power-up.

## Viewing Test Logs

During test execution, the system generates relevant log files that users can inspect to monitor test progress or results. By default, log files are saved in the following path:

```bash
/app/chip_base_test/log
```

### Changing the Log Storage Path

Users can change the log storage location using either of the following two methods:

1. **Moving the directory:**
   Logs are saved by default under the `chip_base_test` directory. Users can simply move the entire `chip_base_test` directory to a new location, for example:

   ```bash
   mv /app/chip_base_test /userdata/chip_base_test
   ```

   After moving, log files will automatically be saved to the new path, for example:

   ```text
   /userdata/chip_base_test/log
   ```

2. **Customizing the log output path**
   Driver unit test scripts support the `-o` option to customize the log directory. Users can edit the `config.ini` file and add the `-o` option to the `ExecStart` parameter of the corresponding test item. For example:

   ```text
   ExecStart=/app/chip_base_test/01_cpu_bpu_ddr/scripts/stress_test.sh -t 24h -o /userdata/logs
   ```

   This configuration saves logs to the `/userdata/logs` directory.

By flexibly adjusting the log storage path, users can manage and review test results more conveniently while meeting log storage requirements in different environments.

## Adding New Test Items

AutoTest supports flexible extension of test items through the `config.ini` file and scripts. Users can add new test functionalities as needed. Below are the steps for adding a new test item:

### Writing a Test Script

Before adding a new test item, prepare or write the corresponding test script and ensure it can run independently. The test script should include the following elements:

- Clear input parameters (e.g., test duration, device path, etc.).
- Log output functionality, preferably supporting the `-o` parameter to customize the log path.

Save the test script in an appropriate location, for example:

```bash
/app/chip_base_test/new_test/new_test.sh
```

### Configuring the `config.ini` File

Add a new test item configuration block in `config/config.ini`, defining the following fields:

- **Status**: Set to `enabled` to activate the test, or `disabled` to keep it inactive temporarily.
- **Description**: Describe the test functionality for easy identification.
- **ExecStart**: Specify the test script path and its runtime parameters.

Example configuration:

```text
[NewTest]
Status=enabled
Description=New feature stability or stress test
ExecStart=/app/chip_base_test/new_test/new_test.sh -t 12h -o /userdata/new_test_logs
```

Field explanations:

- The **`-t`** parameter specifies test duration, e.g., `12h` for 12 hours.
- The **`-o`** parameter specifies the log storage path, e.g., `/userdata/new_test_logs`.

### Launching the Test

After completing the configuration, start the test using the `startup.sh` script:

```bash
/app/chip_base_test/startup.sh
```

The `startup.sh` script will automatically load and execute the newly added test item based on the `config.ini` configuration.

### Verifying the Test Item

After running the test, verify the following to ensure the new test item works correctly:

- Whether log files are generated as expected and contain complete records.
- Whether test results meet functional expectations.
- Whether parameters configured in `config.ini` take effect correctly.