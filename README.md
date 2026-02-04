# Arduino Core for XCTracer Radio Modules

## Supported boards

### XCTracer
 * Murata 1SJ Radio Module

## Installing

### Board Manager

 1. [Download and install the Arduino IDE](https://www.arduino.cc/en/software/) (at least version v2.3.3)
 2. Start the Arduino IDE
 3. Go into Preferences
 4. Add ```https://github.com/XCTracer/ArduinoRadioCore/blob/main/hardware/stm32l0/package_xctracer.com_boards_index.json``` as an "Additional Board Manager URL"
 5. Open the Boards Manager from the Tools -> Board menu and install "XCTracer Radio Boards"
 6. Select your board from the Tools -> Board menu

## Recovering from a faulty sketch for Tlera Corp Boards

 Sometimes a faulty sketch can render the normal USB Serial based integration into the Arduindo IDE not working. In this case plugin the STM32L0 board and toggle the RESET button while holding down the BOOT button and program a known to be working sketch to go back to a working USB Serial setup.

## Credits

This core is based on and compatible with the [Arduino Core for STM32L0 based boards](https://github.com/GrumpyOldPizza/ArduinoCore-stm32l0)

