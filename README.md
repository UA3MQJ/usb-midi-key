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

# Build normal

```console
git clone git@github.com:UA3MQJ/usb-midi-key.git
cd usb-midi-key
git submodule update --init --recursive
cd libopencm3
PATH="/usr/local/gcc-arm-none-eabi-5_4-2016q2/bin/:${PATH}"
make
cd ..
cd src
make bin
```

# Flash normal

Flash by st-utils

`st-flash write usbmidi.bin 0x8000000`

After flash, device detected on name as USB-MIDI VitaSound.Controller-01

# Flash without ST-Link (DFU Mode)

Make DFU bootloader and flash via ST-Link:

```console
cd usb_dfu
make bin
st-flash write usbmidi.bin 0x8000000
```

Short A10 to 3V3 for enable DFU mode. Short A10 to GND for normal working.

Change LD for change memory map. In usb-midi-key/src/makefile comment standart ld file, uncomment stm32f103x8_DFU.ld

Rebuild usbmidi.bin in patch usb-midi-key/src

```console
make clean
make bin
```

Flash via DFU

```console
dfu-util -v -a 0 -s 0x08002000 -D cdcacm.bin
```

2018
