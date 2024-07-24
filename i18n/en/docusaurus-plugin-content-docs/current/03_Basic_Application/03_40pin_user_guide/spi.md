---
sidebar_position: 6
---
# 3.3.6 Using SPI

The RDK X3 exposes the `SPI2` bus of the Sunrise X3M chip on physical pins `19, 21, 23, 24` of the 40-pin header, supporting one chip select and operating at 3.3V IO voltage.

The RDK Ultra exposes the `SPI0` bus on physical pins `19, 21, 23, 24, 26` of the 40-pin header, supporting two chip selects and operating at 3.3V IO voltage.

Please refer to `/app/40pin_samples/test_spi.py` for detailed information on how to use SPI.

## Loopback Test
Connect MISO and MOSI pins together on the hardware, then run the spi test program to perform write and read operations. The expected result is that the read data should be identical to the written data.

### Hardware Connection
Connect MISO and MOSI directly together on the hardware using a jumper cap.

![image-20220512101915524](./image/40pin_user_guide/image-20220512101915524.png)

### Test Procedure

- Run `python3 /app/40pin_samples/test_spi.py`
- Select the bus number and chip select number from the printed spi controllers as input options. For example, if you want to test `spidev0.0`, then both `bus num` and `cs num` should be `0`. Press enter to confirm:

```
List of enabled spi controllers:
/dev/spidev0.0  /dev/spidev0.1
Please input SPI bus num:0
Please input SPI cs num:0
```

- Once the program is running correctly, it will continuously print `0x55 0xAA`. If it prints `0x00 0x00`, then the loopback test of SPI has failed.

```
Starting demo now! Press CTRL+C to exit
0x55 0xAA
0x55 0xAA
```

## Test Code

```python
#!/usr/bin/env python3

from __future__ import print_function
import sys
import os
import time

```# Import the spidev module 
import spidev

def BytesToHex(Bytes):
    return ''.join(["0x%02X " % x for x in Bytes]).strip()

def spidevTest():
    # Set the spi bus number (0, 1, 2) and cs (0, 1)
    spi_bus = input("Please input SPI bus num:")
    spi_device = input("Please input SPI cs num:")
    # Create an object of the spidev class to access the Python functions based on spidev
    spi = spidev.SpiDev()
    # Open the spi bus handle
    spi.open(int(spi_bus), int(spi_device))

    # Set spi frequency to 12MHz
    spi.max_speed_hz = 12000000

    print("Starting demo now! Press CTRL+C to exit")

    # Send [0x55, 0xAA] and receive should also be [0x55, 0xAA]
    try:
        while True:
            resp = spi.xfer2([0x55, 0xAA])
            print(BytesToHex(resp))
            time.sleep(1)

    except KeyboardInterrupt:
        spi.close()

if __name__ == '__main__':
    print("List of enabled spi controllers:")
    os.system('ls /dev/spidev*')

    spidevTest()