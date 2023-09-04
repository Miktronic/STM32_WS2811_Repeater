# STM32_WS2811_Repeater

Input PIN: PB0

The signal of the WS2811 is 5V level. So it should be divided by resistors to drop the level from 5V to 3.3V. In the test case, 1K and 510 ohm resistors are used.

Signal level = (1 / (1 + 0.51)) * 5V = 3.2V

