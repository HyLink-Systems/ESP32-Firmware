# Wi-Fi setup goal

## Regarding this repository

Currently working on setup components seperately.

> [!NOTE]
> `../wifi-connect-STA.c` will need to have the credentials replaced currently to test out Wi-Fi access point based on what Wi-Fi you want to test connecting to. 

## Current plan in this branch

1. `ESP32C3` is set to SAP (soft access point) mode:
    1. in AP mode, use a mobile device to access served site.
    2. then consumer access AP mode to connect to `esp32c3` to configure local home wifi SSID+password.
    3. after form submission —> reset wifi mode:
        1. SET `esp32c3` to to SAP mode and try wifi `10` times.
        2. TODO — somehow signal (maybe using LEDs) that Wi-Fi success? Fail?
            1. if success — **SAVE** SSID+password combination to NVS.
2. SAP success —> then move onto MQTT
    1. TODO — read sensor loop, send via mqtt

## Wi-Fi STA testing

STA — connect to existing Wi-Fi access point.

*currently referencing —* <https://github.com/espressif/esp-idf/blob/d4cd437ede613fffacc06ac6d6c93a083829022f/examples/wifi/getting_started/station/main/station_example_main.c>

## Wi-Fi SAP testing

SAP — serving a Wi-Fi access point.

TODO:

1. make the `esp32c3` make a Wi-Fi access point, serve a website to accept new network configurations.
2. Save a successful connection to NVS.

## For every boot

- Try SSID/password combinations in NVS until fail — then boot in SAP mode and flash the red light to indicate “manual Wi-Fi setup needed

## Notes regarding development workflow

initial shell setup:

```sh
./install.ps1
./export.ps1
idf.py set-target esp32c3
idf.py build
```
