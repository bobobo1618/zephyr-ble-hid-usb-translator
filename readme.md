
# Zephyr ble hid usb translator
Abomination of code that proxies/translates HID from a Bluetooth device to a USB device so that USB keyboards/mice can be used with BIOS/UEFI.

To pair with your keyboard, simply flash this in a nrf52840 board
and it will auto pair with the first keyboard it finds

## How to unpair or re pair with a device

You can plug the USB to your computer leaving the user button pressed.
The device will blink 3 times and it will remove any pairing keys. You
can re pair with your device by putting it in pair mode.

Alternatively, you can flash the firmware with the line `#define RESET_PAIRS`
uncommented. It will delete any pair key and reset the device. Once you have
done that, comment the line and flash again.



## Setup

```
west init -l .
west update
```

## Flashing nrf52840 mdk dongle

```
west build -p -s ./app -b nrf52840_mdk_dongle  

```
```

west build -p -s ./app -b nrf52840_mdk_dongle  -- -DSHIELD=settings_reset

```
## Flash nrf52840dongle_nrf52840
```
west build -p -s ./app -b nrf52840dongle_nrf52840  -- -DSHIELD=reset_button && \
nrfutil pkg generate --hw-version 52 --sd-req=0x00 \
        --application build/zephyr/zephyr.hex \
        --application-version 1 dongle.zip && \
nrfutil dfu usb-serial -pkg dongle.zip -p /dev/ttyACM0
```
