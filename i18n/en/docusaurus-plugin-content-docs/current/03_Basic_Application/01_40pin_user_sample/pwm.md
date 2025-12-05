---
sidebar_position: 3
---
# 3.1.3 Using PWM

The Hobot.GPIO library only supports PWM on pins with additional hardware PWM controllers. Unlike the RPi.GPIO library, the Hobot.GPIO library does not implement software simulated PWM. 

The RDK X3 RDK Ultra both support 2 PWM channels.

RDK X5 supports four PWM groups, with two output channels per group, providing a total of eight PWM outputs, as shown in the table below:

| PMW Group | PWM Channel | 40PIN Pin |
| --- | ---- | ---- |
| PWM0 | LSIO_PWM_OUT0 | 29 |
| PWM0 | LSIO_PWM_OUT1 | 31 |
| PWM1 | LSIO_PWM_OUT2 | 37 |
| PWM1 | LSIO_PWM_OUT3 | 24 |
| PWM2 | LSIO_PWM_OUT4 | 28 |
| PWM2 | LSIO_PWM_OUT5 | 27 |
| PWM3 | LSIO_PWM_OUT6 | 32 |
| PWM3 | LSIO_PWM_OUT7 | 33 |

By default, PWM3 is enabled on the RDK X5. Additional PWM groups can be enabled via the `srpi-config` system configuration tool, which will remap the corresponding pins for PWM output. The configuration takes effect after reboot.

Select `3 Interface Options` → `I3 Peripheral Bus Config`, then choose the corresponding PWM group and set it to okay.

Refer to `/app/40pin_samples/simple_pwm.py` for detailed information on how to use PWM channels.

## Test Code

Open the PWM channel specified by `output_pin`, with an initial duty cycle of 25%. Increase the duty cycle by 5% every 0.25 seconds until 100% is reached, and then decrease the duty cycle by 5% every 0.25 seconds. When the normal output waveform is present, you can measure the output signal and observe the waveform using an oscilloscope or logic analyzer.

```python
#!/usr/bin/env python3

import Hobot.GPIO as GPIO
import time

# PWM-supporting pins: 32 and 33
output_pin = 33

def main():
    # Pin Setup:
    # Board pin-numbering scheme
    GPIO.setmode(GPIO.BOARD)
    # Supported frequency range for RDK X3: 48KHz ~ 192MHz
    # Supported frequency range for RDK Ultra: 1Hz ~ 12MHz
    # Supported frequency range for RDK X5: 1Hz ~ 12MHz
    p = GPIO.PWM(output_pin, 48000)
    # Initial duty cycle of 25%. Increase by 5% every 0.25 seconds until 100% is reached, then decrease by 5% every 0.25 seconds
    val = 25
    incr = 5
    p.start(val)

    print("PWM running. Press CTRL+C to exit.")
    try:
        while True:
            time.sleep(0.25)
            if val >= 100:
                incr = -incr
            if val <= 0:
                incr = -incr
            val += incr
            p.ChangeDutyCycle(val)
    finally:
        p.stop()
        GPIO.cleanup()

if __name__ == '__main__':
    main()
```