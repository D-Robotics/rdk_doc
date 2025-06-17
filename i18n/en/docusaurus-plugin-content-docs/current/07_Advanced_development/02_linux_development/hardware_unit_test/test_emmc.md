---
sidebar_position: 2
---
# EMMC Related Testing

For EMMC, the main concerns are its stability and performance.

## EMMC Stability Testing

### Testing Method

1. Use the open source tool "iozone" to perform file system read and write tests on EMMC.
2. Go to the "test_tools/02_emmc" folder and execute the script "sh emmc_stability_test.sh &" to perform file system read and write tests on EMMC.

### Testing Criteria

1. At high temperature (45°), low temperature (-10°), and room temperature, the program should run normally without any restart or hang situations.
2. There should be no abnormal prints in the LOG file, such as "fail", "error", "timeout", etc.
3. The system should run stably for 48 hours.

## EMMC Performance Testing

### Testing Method

1. Use the open source tool "iozone" to test the read and write speeds of the EMMC file system.
2. Read limit: 172.8MB/s, Write limit: 35MB/s.
3. Go to the "test_tools/02_emmc" folder and execute the script "sh emmc_performance_test.sh &".

### Testing Criteria

1. In a normal temperature environment, the program should run without any abnormal restart or hang situations.
2. There should be no abnormal prints in the LOG file, such as "fail", "error", "timeout", etc.
3. Verify if the actual test read and write speeds meet the performance requirements.
4. The system should run stably for 48 hours.