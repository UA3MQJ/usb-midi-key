# usb-midi-key

STM32F103 board based USB MIDI key device.

![alt text](https://habrastorage.org/files/85b/0da/219/85b0da21951c4571891f41e2f84ba11f.jpg "Logo Title Text 1")

Button connected to GPIOA - GPIO0. Device generate MIDI messages for host.

# Build

`make bin`

# Flash

Flash by st-utils

`st-flash write usbmidi.bin 0x8000000`
