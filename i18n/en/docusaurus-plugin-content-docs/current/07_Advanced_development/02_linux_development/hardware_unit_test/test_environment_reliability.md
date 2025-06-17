---
sidebar_position: 1
---

# Environment Reliability Testing (Fixed Frequency)

## Test Method

1. Go to the `test_tools/01_cpu_bpu_ddr` folder and execute the `sh cpubpu.sh &` script to perform stress testing on the `bpu` and `cpu`. The current configuration starts dual `bpu` running at a load of up to 99%, and `cpu` at a load of over 90%, with random memory reads and writes.
2. The script allows for customization of the runtime and intensity of the load.
3. Open a terminal to record the running log.

## Test Criteria

1. High temperature: 45°, low temperature: -10°, normal temperature. The program should execute normally without experiencing restarts or hangs.
2. The log should not contain any abnormal prints such as `fail`, `error`, `timeout`, etc.
3. The system should be able to run stably for 48 hours.
4. During the running process, it is necessary to monitor the CPU and BPU usage, ensuring that the CPU usage is close to 90% and the BPU stress is close to 90%.