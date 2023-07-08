
# Zephyr ble hid usb translator
This is a project intended to use a bluetooth keyboard as a dongle. 
This allows you to use the keyboard in BIOS and can offer more stability than bluetooth connection

## Installing in a board
Similar to ZMK, download the latest build of the supported board in [github actions](https://github.com/charlesmst/zephyr-ble-hid-usb-translator/actions). For boards that don't use `uf2` bootloader, check []

## How to pair
Once you plug the dongle, it will wait for a keyboard that wants to connect to bluetooth. You should put your keyboard in bluetooth pairing mode. On a ZMK board, you can do that by selecting the profile you want to use for the dongle and pressing the key assigned to `&bt BT_CLR`.

## How to unpair

If you are using a board that supports reset button, simply plug the dongle leaving the button pressed. The board will blink 3 times indicating it is has reseted all the pairing keys it had. You can pair with any keyboard again.

Boards that don't support the reset button should be flashed with the shield `settings_reset` and then flashed again with the `dongle` shield

# Supported boards
Currently the projectg is auto building for using uf2 for:
- [xiao ble](https://www.seeedstudio.com/Seeed-XIAO-BLE-nRF52840-p-5201.html)
- [nordic nrf52840 dongle](https://docs.zephyrproject.org/latest/boards/arm/nrf52840dongle_nrf52840/doc/index.html)
- [nrf52840 mdk dongle](https://wiki.makerdiary.com/nrf52840-mdk-usb-dongle/)
- [nice\_nano\_v2](https://nicekeyboards.com/nice-nano/)


# Shields
Boards that offer a programable button, such as nordic nrf52840 dongle, can be built using reset\_dongle shield. The ones don't support that can use the normal dongle shield.

- `dongle`: The simple dongle implementation.
- `settings_reset`: Clears the dongle pairing keys if you need to pair to another
- `reset_button`: Is the dongle implementation with support to unpair without flashing, by pressing a button while plugging in

# Build locally

Requirements: 
- west

Init the repo with west:
```
west init -l .
west update
```

## Building a board

You can build any of the supported boards by zephyr or the ones offered in this repo

```
west build -p -s ./app -b nrf52840_mdk_dongle  -- -DSHIELD=dongle
```

Copy the `build/zephyr/zephyr.uf2` file to the board.

If you need to reset the dongle you can build the `settings_reset` shield

```

west build -p -s ./app -b nrf52840_mdk_dongle  -- -DSHIELD=settings_reset

```

# Flashing with nrfutil
Boards such as `nrf52840dongle_nrf52840` require you to flash using `nrfutil` tool. Download the `hex` file from github actions.

Using the hex file, use the `nrfutil` to flash it. Once it is in the step to flash, you should press the reset button on the board.
```
nrfutil pkg generate --hw-version 52 --sd-req=0x00 \
        --application build/zephyr/zephyr.hex \
        --application-version 1 dongle.zip && \
nrfutil dfu usb-serial -pkg dongle.zip -p /dev/ttyACM0
```
