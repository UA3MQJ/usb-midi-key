# usb-midi-key

STM32F103 board based USB MIDI key device.

![USB-MIDI](https://pp.vk.me/c638427/v638427366/6c06/E_eDuD3WIkU.jpg "USB-MIDI")

Device generate MIDI messages for host.
Send to host: MIDI CC channel 0 num 16 - value of potentiometer.
Pot connected: GND - A0 - 3.3V

Send to host: MIDI note on/off when pressed button.
Button connected to GPIOB - GPIO0.

Device receive MIDI messages from host.
Midi note on/off. When any key pressed - PC13 is on.

# Build

`make bin`

# Flash

Flash by st-utils

`st-flash write usbmidi.bin 0x8000000`
