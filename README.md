# Arduino Core for XCTracer Radio Board

## Supported boards

### XCTracer
 * Murata 1SJ Radio Board

## Installing

### Board Manager

 1. [Download and install the Arduino IDE](https://www.arduino.cc/en/software/) (at least version v2.3.3)
 2. Start the Arduino IDE
 3. Go into Preferences
 4. Add ```https://github.com/XCTracer/ArduinoRadioCore/raw/refs/heads/main/hardware/stm32l0/package_xctracer.com_boards_index.json``` as an "Additional Board Manager URL"
 5. Open the Boards Manager from the Tools -> Board menu and install "XCTracer Radio Boards"
 6. Select your board from the Tools -> Board menu

#### OS Specific Setup

##### Linux

 1. Go to ~/.arduino15/packages/XCTracerRadio/hardware/stm32l0/```<VERSION>```/drivers/linux/
 2. sudo cp *.rules /etc/udev/rules.d
 3. reboot

#####  Windows

###### STM32 BOOTLOADER driver setup

 1. Download [Zadig](http://zadig.akeo.ie)
 2. Plugin STM32L0 board and toggle the RESET button while holding down the BOOT button
 3. Let Windows finish searching for drivers
 4. Start ```Zadig```
 5. Select ```Options -> List All Devices```
 6. Select ```STM32 BOOTLOADER``` from the device dropdown
 7. Select ```WinUSB (v6.1.7600.16385)``` as new driver
 8. Click ```Replace Driver```

###### USB Serial driver setup (Window XP / Windows 7 only)

 1. Go to ~/AppData/Local/Arduino15/packages/XCTracerRadio/hardware/stm32l0/```<VERSION>```/drivers/windows
 2. Right-click on ```dpinst_x86.exe``` (32 bit Windows) or ```dpinst_amd64.exe``` (64 bit Windows) and select ```Run as administrator```
 3. Click on ```Install this driver software anyway``` at the ```Windows Security``` popup as the driver is unsigned

###### ST-LINK V2.1 driver setup for STMicroelectronics boards

 1. Plugin STMicroelectronics board
 2. Download and install [ST-Link USB Drivers](http://www.st.com/en/embedded-software/stsw-link009.html)

## Recovering from a faulty sketch

Sometimes a faulty sketch can render the normal USB Serial based integration into the Arduindo IDE not working. In this case plugin the Murata 1SJ Radio Board board and toggle the RESET button while holding down the BOOT button and program a known to be working sketch to go back to a working USB Serial setup.

## Credits

This core is based on and compatible with the [Arduino Core for STM32L0 based boards](https://github.com/GrumpyOldPizza/ArduinoCore-stm32l0)

