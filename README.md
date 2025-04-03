# Arduino Core MFL for GD32F103x/GD32F303x MCUs

This repository contains the source code and configuration files of the Arduino MFL Core
for Gigadevice's GD32F103x Cortex-M3 and GD32F303x Cortex-M4 processors.
Board support is currently limited to 3D printer based boards, as this core is used with
the recently upstreamed GD32 MFL HAL for the excellent and widely-used [Marlin Firmware](//https://github.com/MarlinFirmware/Marlin/).

Any additional uses of the core are welcomed.

## Current State and Testing

This is a very new core, with most functionality only tested on the currently supported boards.
Specifically, the Wire I2C, SPI, and upcoming SoftwareSerial libraries could benefit from additional testing.
Any contributions of additional functionality/bug fixes are welcome.

## Installation on Arduino IDE

This core is not yet available as a package in the Arduino IDE cores manager.

## Bugs or Issues

If you find a bug you can submit an issue here on github:

https://github.com/bmourit/ArduinoCore-MFL/issues

Before posting a new issue, please check if the same problem has been already reported by someone else
to avoid duplicates.

## Contributions

Contributions are always welcome. The preferred way to receive code contribution is by submitting a 
Pull Request on github.

## License and credits

This core has been developed by B. Mouritsen and GPLv2 liscened.
Arduino API is GPLv2 licensed.
MFL is LGPLv3 liscensed.

