# Overview

Driver functional unit testing is a critical phase for verifying the stability, performance, and functional integrity of system drivers and hardware components. This chapter provides a detailed introduction to the testing methodologies and criteria for various driver functional units, ensuring the system delivers high reliability and exceptional performance in real-world application scenarios.

## <span id="Deploy_test_program"/>Test Code Storage Path

To facilitate unified management and usage of test code, scripts, and tools, all test resources described in this chapter have been integrated into the BSP source code within the SDK delivery package. The storage path is as follows:

- **SDK Delivery Package BSP Path:**  
  `~/sdk/source/hobot-multimedia-samples/debian/app/multimedia_samples/chip_base_test`

When using the officially released SDK delivery package, the provided EVB (Evaluation Board) system image will, by default, install all test programs onto the following path on the development board:

- **Development Board Path:**  
  `/app/chip_base_test`

## Test Environment Preparation

Before conducting driver functional unit tests, ensure the following environment setup steps have been completed:

1. Use the officially released SDK delivery package, which includes the corresponding test code and tools.

2. Confirm the storage path of test tools and scripts on the development board:

   ```bash
   ls /app/chip_base_test
   ```

3. Execute the relevant test programs step by step according to the testing methods described in this chapter, and record and analyze the results.

## Test Execution Method

Developers can run test programs directly on the development board using the following approach:

1. Log in to the development board via serial port or SSH.

2. Navigate to the test program directory:

   ```bash
   cd /app/chip_base_test
   ```

3. Execute the corresponding test script or program—for example, to run CPU / BPU / DDR stress tests:

   ```bash
   cd /app/chip_base_test/01_cpu_bpu_ddr/scripts
   ./stress_test.sh
   ```

## Test Objectives and Criteria

Driver functional unit testing aims to cover the following core objectives:

- **Functional Verification:** Ensure the driver correctly implements its designed functionality.
- **Stability Testing:** Validate driver stability through extended runtime and boundary condition testing.
- **Performance Testing:** Measure performance metrics of the driver and hardware to ensure they meet system requirements.
- **Compatibility Testing:** Verify driver compatibility across different hardware configurations or software versions.

Detailed testing methods, procedures, and pass/fail criteria for each driver type will be elaborated in subsequent chapters.