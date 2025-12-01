---
sidebar_position: 3
---

# 3.3.3 PWM Application

The Hobot.GPIO library supports PWM only on pins equipped with an additional hardware PWM controller. Unlike the RPi.GPIO library, Hobot.GPIO does not implement software-emulated PWM. The RDK S100 40-pin hardware supports two LPWM channels.

Please refer to `/app/40pin_samples/simple_pwm.py` for detailed instructions on how to use PWM channels.

:::tip
The pins mentioned below are provided for illustrative purposes only. Port values may differ across platforms; please verify against your actual hardware setup. Alternatively, you can directly use the code under the `/app/40pin_samples/` directory, which has already been verified on the actual board.
:::

### Test Code

Open the PWM channel specified by `output_pin` with an initial duty cycle of 25%. Increase the duty cycle by 5% every 0.25 seconds until it reaches 100%, then decrease it by 5% every 0.25 seconds. While the waveform is being output normally, you can measure the output signal using an oscilloscope or a logic analyzer to observe the waveform.

```python
#!/usr/bin/env python3
import sys
import signal
import Hobot.GPIO as GPIO
import time

def signal_handler(signal, frame):
    sys.exit(0)

# Pins supporting PWM: 32 and 33. When using PWM, ensure these pins are not occupied by other functions.
output_pin = 33

GPIO.setwarnings(False)

def main():
    # Pin Setup:
    # Board pin-numbering scheme
    GPIO.setmode(GPIO.BOARD)
    # Supported frequency range: 48 kHz ~ 192 MHz
    p = GPIO.PWM(output_pin, 48000)
    # Initial duty cycle: 25%. Increase by 5% every 0.25 seconds until reaching 100%, then decrease by 5% every 0.25 seconds.
    val = 25
    incr = 5
    p.ChangeDutyCycle(val)
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
    signal.signal(signal.SIGINT, signal_handler)
    main()

```