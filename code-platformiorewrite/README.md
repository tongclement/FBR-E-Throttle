# PlatformIO Rewrite

This project converts `sketch_jun22b.ino` to a PlatformIO Arduino project for a generic ESP32 development board.

The original compile error happened because `motor` was declared inside `setup()`, so it was out of scope in `loop()`. In this rewrite, the `TMC5160_SPI motor(SPI_CS)` object is global and initialized during `setup()`.

## Build

```sh
pio run
```

## Upload

```sh
pio run --target upload
```

## Serial Monitor

```sh
pio device monitor --baud 115200
```

## Motor Diagnostics

The serial output prints TMC5160 diagnostics. A working SPI connection should report `version=0x30`.

If `IOIN` is `0x00000000`, `0xFFFFFFFF`, or the version is not `0x30`, check:

- ESP32 SPI wiring: `SCK=18`, `MISO=19`, `MOSI=23`, `CS=5` for the default ESP32 SPI pins used by `SPI.begin()`.
- ESP32 GND and TMC5160 logic/power GND are connected together.
- The TMC5160 logic supply and motor supply are both powered.
- `DRV_ENN`/`EN` is active-low, so it must be tied low or driven low by the ESP32.
- The TMC5160 board is in SPI mode, not UART or standalone step/dir mode.

If your enable pin is wired to the ESP32, set `SPI_DRV_ENN` in `src/main.cpp` to that pin number.

## Single APPS Bench Mode

The current firmware uses one APPS signal only:

- `PEDAL_PIN = 34`
- raw APPS ADC range: `0..2500`
- commanded throttle range: `0..200` TMC5160 position steps
- control update period: `20 ms`

APPS2 plausibility checks and TPS closed-loop feedback are intentionally disabled for bench testing until both pedal channels and the throttle position sensor are connected.

If your ESP32 board is not the generic `esp32dev`, change the `board` value in `platformio.ini` to the exact PlatformIO board ID.
