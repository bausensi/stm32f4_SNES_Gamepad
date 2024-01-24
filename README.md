STM32f4 SNES Gamepad to USB HID Converter Firmware
===============
[This project](https://github.com/ripxorip/stm32f4_Gamepad) was used as a reference.

This is my USB firmware for the WeAct Studio Blackpill and its many clones. This is a device that reads the input of a SNES/SFC controller and translates it to a USB gamepad, allowing the user to use it as if it was a USB joypad. It is based on the mouse example provided by ST by modifying the [usbd_hid_core.c](https://github.com/guitarfriiik/stm32f4_Gamepad/blob/master/usbd_hid_core.c) and overriding the original file at *$STM_SRC_DIR/Libraries/STM32_USB_Device_Library/Class/hid/usbd_hid_core.c*. Since it is using the HID class sources there is no need for extra drivers when running OS X / Linux.
[This document](https://www.nesdev.org/wiki/SNES_controller) was used as a technical reference to talk to the controller.

The firmware was adapted for STM32F401CC devices running on a 25 MHz crystal oscillator. It is a suboptimal choice to get solid USB clocks, but when they sell you a board with it you already have no choice and it's better than tossing it into the garbage. Don't let your USB go to waste if you're unlucky and have such a board.

Tested with an original, first-revision Super Famicom controller.

Wiring
------
|Controller Pin|GPIO Port, Pin|
|-|-|
|CLOCK|GPIOC, 13|
|LATCH|GPIOC, 14|
|DATA|GPIOC, 15|

The MCU's internal pull down resistors are used to protect the signal integrity. The MCU will poll the controller to obtain a bitmask, invert it and pass it to the HID report.

Compilation
-----------
Download and install the following software for your platform
  * The gcc-arm-none-eabi toolchain.
  * openOCD or dfu-util depending on your upload method choice.
  * st-link (optional)
  * The stm32f4 firmware - [Download from ST](http://www.st.com/web/en/catalog/tools/PF257904#)
  * Specify the folder of the firmware in the Makefile and run make.
  * Upload the resuling binary to the board using openOCD, dfu-util or st-link.

Usage
-----
Simply connect the SNES controller to the board and plug the included USB-C port to your computer. The USB Type C port both supplies the board with power and acts as a HID joypad.
