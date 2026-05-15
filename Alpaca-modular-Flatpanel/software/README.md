# FlipFlatpanel Alpaca

ESP32 Flatpanel / CoverCalibrator project with:
- Servo on IO4
- Position sensor (A0) on IO5
- BME280 on IO8/IO9
- Heater MOSFET on IO16
- Flatpanel PWM on IO13
- UART0 serial monitor TX on IO43, RX on IO44 at 115200 baud
- Web UI with live telemetry
- Position calibration via web UI
- Wi-Fi AP fallback with setup page

## Wi-Fi setup
If no Wi-Fi credentials are stored or connection fails, the ESP starts an AP:
- SSID: `FlipFlatpanel-Setup`
- Password: `flipflatpanel`
- Open `http://192.168.4.1/setup`

## OTA updates
After the device is connected to Wi-Fi, ArduinoOTA is available as `flipflatpanel.local` on port `3232`.
Use the PlatformIO environment `esp32s2_ota` for wireless uploads. Keep `esp32s2` for serial recovery uploads.

## Alpaca discovery
The firmware answers ASCOM Alpaca discovery polls on UDP port `32227` with `{"AlpacaPort":80}`.
Conform Universal should then enumerate the CoverCalibrator through the management endpoint on port `80`.

## Calibration
In the web UI:
- move cover fully open -> "Aktuelle Position = Offen"
- move cover fully closed -> "Aktuelle Position = Geschlossen"

Calibration is stored in Preferences and survives reboot.

## Serial monitor
The firmware writes monitor output to UART0.
Connect the USB-TTL adapter RX to ESP32-S2 IO43, adapter TX to IO44 if input is needed, and GND to GND.
The baud rate is 115200. Pins and baud can be changed in `platformio.ini`.
