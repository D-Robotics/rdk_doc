# AutoTest 使用方法

AutoTest 提供了一种灵活的自动化测试解决方案，支持通过配置文件 `config.ini` 自定义压测时长和测试次数，并允许用户根据需求扩展测试用例，以满足不同的测试需求。该工具在驱动单元测试的基础上开发，实现了自动化测试功能，同时与功能单元的独立测试相互独立，并充分复用现有代码资源。

## AutoTest 目录结构

AutoTest 使用 `startup.sh` 脚本读取 `config/config.ini` 配置文件来启动测试。用户可以通过修改 `config/config.ini` 文件自定义测试内容。在需要进行多个功能的稳定性或压力测试时，这种方式能显著减少用户重复配置测试环境的工作量。目前，AutoTest 已经复用了驱动功能单元测试中的以下测试项。

```bash
#:~/sdk/source/hobot-multimedia-samples/debian/app/multimedia_samples/chip_base_test$ tree
.
├── 01_cpu_bpu_ddr
│   └── scripts
│       └── stress_test.sh         # CPU-BPU-DDR 压力测试
├── 02_emmc
│   └── emmc_stability_test.sh     # eMMC 稳定性测试
├── 03_uart_test
│   └── uartstress.sh              # 串口压力测试
├── 04_spi_test
│   └── spistress.sh               # SPI 压力测试
├── config
│   └── config.ini                 # 自动化测试的配置文件
└── startup.sh                     # 配置自启动的脚本程序
```

## config.ini 配置文件说明

`config.ini` 文件用于配置和管理 AutoTest 的测试项，通过调整其中的参数，用户可以快速选择需要执行的测试并指定测试参数。以下是文件结构及配置说明：

### 配置文件结构

每个测试项以 `[测试项名称]` 开头，包含以下字段：

- **Status**:  启用状态，可设置为 `enabled`（启用）或 `disabled`（禁用）。
- **Description**:  测试项的简要描述，说明测试内容。
- **ExecStart**:  测试脚本的路径及运行参数，定义具体测试的执行方式。

### 配置文件示例

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

### 使用说明

1. **编辑配置文件：**
   打开并编辑 `config/config.ini` 文件，根据需要调整每个测试项的 `Status` 值：
   - 设置为 `enabled`：启用测试项。
   - 设置为 `disabled`：禁用测试项。
2. **修改测试参数：**
   根据实际需求，调整 `ExecStart` 中的命令和参数。例如：
   - 修改测试时长（如 `-t 24h` 表示测试持续 24 小时）。
   - 调整设备路径（如 `/dev/ttyS2` 或 `/dev/spidev0.0`）。
   - 调整其他测试选项（如循环次数 `-c` 或波特率 `-b`）。
   - 目前支持的所有测试项的脚本程序都支持 `-h` 选项，可以查阅命令帮助信息来调整参数。

## 使用 startup.sh 启动测试

在完成 `config.ini` 配置后，用户可以通过运行 `startup.sh` 脚本启动 AutoTest。脚本支持多种启动方式，以满足不同的使用需求：

- **手动启动或调试测试项**

  适用于使用官方系统镜像并需要手动运行或调试测试项的场景。执行以下命令启动测试：

  ```bash
  /app/chip_base_test/startup.sh
  # 或者
  cd /app/chip_base_test
  ./startup.sh
  ```

​	此方式便于调试单项测试内容或观察测试执行情况。

- **自启动或调试测试项**
  测试项已经配置完成且调试通过时，也可以将startup.sh 放到 rc.local 自启动脚本中进行自启动测试，需要开发板在上电后自动运行测试的场景

## 查看测试日志

在测试运行期间，系统会生成相关日志文件，用户可以通过这些日志检查测试的进展或结果。默认情况下，日志文件保存在以下路径：

```bash
/app/chip_base_test/log
```

### 修改日志保存路径

用户可以通过以下两种方式更改日志的保存位置：

1. **移动目录：**
    日志默认保存在 `chip_base_test` 目录下。用户只需将整个 `chip_base_test` 目录移动到新的位置，例如：

   ```bash
   mv /app/chip_base_test /userdata/chip_base_test
   ```

   移动后，日志文件将自动保存到新的路径，例如：

   ```text
   /userdata/chip_base_test/log
   ```

2. **自定义日志输出路径**
   驱动单元测试程序的脚本支持通过 `-o` 选项自定义日志目录。用户可以编辑 `config.ini` 配置文件，在对应测试项的 `ExecStart` 参数中添加 `-o` 选项。例如：

   ```text
   ExecStart=/app/chip_base_test/01_cpu_bpu_ddr/scripts/stress_test.sh -t 24h -o /userdata/logs
   ```

   此配置会将日志保存到 `/userdata/logs` 目录下。

通过灵活调整日志保存路径，用户可以更方便地管理和查看测试结果，同时满足不同环境下的日志存储需求。

## 新增测试项

AutoTest 支持通过配置 `config.ini` 文件和脚本灵活扩展测试项，用户可以根据需求添加新的测试功能。以下是新增测试项的操作步骤：

### 编写测试脚本

在新增测试项前，需要先编写或准备对应的测试脚本，并确保脚本可以独立运行。测试脚本应包含以下要素：

- 明确的输入参数（如测试时长、设备路径等）。
- 日志输出功能，建议支持 `-o` 参数自定义日志路径。

将测试脚本保存到适当位置，例如：

```bash
/app/chip_base_test/new_test/new_test.sh
```

### 配置 `config.ini` 文件

在 `config/config.ini` 中新增测试项配置段落，定义以下字段：

- **Status**: 设置为 `enabled` 启用测试，或设置为 `disabled` 暂不启用。
- **Description**: 描述测试功能，方便识别测试项。
- **ExecStart**: 填写测试脚本路径及其运行参数。

示例配置：

```text
[NewTest]
Status=enabled
Description=New feature stability or stress test
ExecStart=/app/chip_base_test/new_test/new_test.sh -t 12h -o /userdata/new_test_logs
```

字段说明：

- **`-t`** 参数指定测试时长，例如 `12h` 表示 12 小时。
- **`-o`** 参数指定日志保存路径，例如 `/userdata/new_test_logs`。

### 启动测试

完成配置后，通过 `startup.sh` 脚本启动测试：

```bash
/app/chip_base_test/startup.sh
```

`startup.sh` 会根据 `config.ini` 文件中的配置自动加载并运行新增的测试项。

### 验证测试项

运行测试后，检查以下内容以确保新增测试项正常工作：

- 日志文件是否按预期生成，且内容记录完整。
- 测试结果是否符合功能预期。
- `config.ini` 配置的参数是否正确生效。
