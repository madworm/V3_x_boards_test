#!/bin/bash

#
# if you get write errors increase the -B xx value
#
# the bootloader is the typical arduino one and runs at 19200
#

avrdude -c usbtiny -p atmega168 -P usb -b 115200 -e -B 100 -U lock:w:0x3F:m -U lfuse:w:0xFF:m -U hfuse:w:0xDD:m -U efuse:w:0x00:m
avrdude -c usbtiny -p atmega168 -B 1 -P usb -b 115200 -U flash:w:V3_x_boards_test__plus__ATmegaBOOT_168_diecimila.hex -U lock:w:0x0F:m
