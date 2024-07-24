---
sidebar_position: 3
---

# UART Pressure Test

## Test Methods

1. Enable the `uart1` and `uart2` nodes on the image (both sending and receiving), and short the `TX` and `RX` pins of `uart1` with the `RX` and `TX` pins of `uart2` on the hardware.
2. Execute the test scripts: `sh uart1test.sh &`, `sh uart2test.sh &`.

## Test Criteria

1. At high temperature (45°C), low temperature (-10°C), and normal temperature, the program should run normally without restart or hanging.
2. There should be no abnormal prints in the LOG such as `fail`, `error`, or `timeout`.
3. The program should run stably for 48 hours.

## Appendix

The test source code and compilation can refer to the [UART Driver Debugging Guide].