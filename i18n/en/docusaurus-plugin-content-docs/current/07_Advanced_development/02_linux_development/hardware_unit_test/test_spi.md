---
sidebar_position: 4
---

# SPI Pressure Test

## Testing Method

1. Enter the `test_tools/07_spi_test` directory.
2. The testing script is divided into `master` and `salve` modes. Run the corresponding script based on the mode configured in the SPI driver.
3. Two RDK X3 development boards can be used: one configured as `master` mode and the other as `salve` mode. The configuration process can refer to the [SPI Debugging Guide](../driver_development/driver_spi_dev.md). First, execute the master-side testing script: `sh spitest_master.sh &`, then execute the salve-side testing script: `sh spitest_salve.sh &`. The time interval between the two script executions should be as short as possible.

## Testing Criteria

1. At high temperature (45°), low temperature (-10°), and room temperature, the program should execute normally without any restart or hang-up issues.
2. The LOG should not have any abnormal prints such as `fail`, `error`, `timeout`, etc.
3. The program should be able to run stably for 48 hours.