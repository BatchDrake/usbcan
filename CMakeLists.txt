cmake_minimum_required(VERSION 3.8)
project(USBCanTools)

add_executable(
  usbcanbr
  src/usb_can.c
  src/usbcanbr.c
  include/usb_can.h)

target_include_directories(usbcanbr PUBLIC include ${SIGUTILS_INCLUDE_DIRS})
target_link_libraries(usbcanbr m)

add_executable(
  canhello
  src/canhello.c)

install(TARGETS usbcanbr DESTINATION bin)

