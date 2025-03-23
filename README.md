# usbcan

**usbcan** lets you sniff and inject CAN packets by means of an appropriately configured BluePill+ STM32F103 board and a MCP2515 transceiver. **This is a Linux-only project.**

This project features the following command line tools and modules:

* `usbcanbr`: Linux virtual bridge that forwards frames from a virtual CAN network interface and the BluePill board via USB.
* `usbcan_stm32.ino`: Arduino code for the BluePill+ board.

## Uploading the Arduino code to the BluePill+
* Install [Arduino IDE 2.2.1](https://www.arduino.cc/en/software/) or later.
* Install STM32F103 dependencies [following this tutorial](https://www.instructables.com/How-to-Program-STM32F103C8T6-With-ArduinoIDE/). **Please note:** the "Other Board URL" provided by the tutorial is deprectated, use `https://github.com/stm32duino/BoardManagerFiles/raw/main/package_stmicroelectronics_index.json` instead.
* Install the `AA_MCP2515` Arduino library.
* In the board part number, make sure you selected `BluePill F103C8`.
* Copy and paste [this code](stm32f103/arduino/usbcan_stm32.ino) in Arduino IDE.
* Enter BluePill+ in boot mode and upload it. Reset the board and you are ready to go!

## Building and installing
This is your good old typical CMake project. Just clone from this branch and run in the source folder:

```bash
$ mkdir build
$ cd build
$ cmake ..
$ make
$ sudo make install
```

And, of course, make sure you have `/usr/local/bin` or whatever install directory you chose in your `$PATH`.

## Running `usbcanbr`
Before running `usbcanbr`, make sure you have a virtual CAN device named `vcan0` up and running. This can be done with the following commands:

```bash
$ sudo modprobe can
$ sudo modprobe vcan
$ sudo ip link add dev vcan0 type vcan
$ sudo ip link set up vcan0
```

The command line tool `usbcanbr` accepts two arguments: the USB serial device from which you interact with the BluePill+ (usually `/dev/ttyACM0` or `/dev/ttyUSB0` depending on how your USART is configured) and the virtual CAN device:

```bash
$ usbcanbr /dev/ttyACM0 vcan0
```

And if you got no errors, congratulations! You are now ready to open Wireshark and capture packets from `vcan0`.
