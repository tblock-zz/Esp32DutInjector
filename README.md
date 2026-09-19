# Esp32DutInjector

A serial-controlled signal injector / measurement tool for an ESP32 (classic,
v1) device-under-test (DUT) bench setup. The firmware exposes digital inputs
and outputs, ADC inputs, DAC outputs, a PWM generator and a PWM measurement
input over a simple line-based ASCII command interface on the serial port
(115200 baud, 8N1, commands terminated with `\n`).

## Features

- 2 digital inputs (GPIO 36, GPIO 39)
- 2 digital outputs (GPIO 14, GPIO 27)
- 2 ADC inputs (GPIO 34, GPIO 35, 12 bit, 0..4095)
- 2 DAC outputs (GPIO 25, GPIO 26, 8 bit, 0..255)
- 1 PWM output (GPIO 16, LEDC, 12 bit resolution)
- 1 PWM measurement input (GPIO 22, MCPWM capture, both edges,
  frequency + duty cycle measurement)
- Self-test command `t` for a quick PWM loop check (e.g. with the PWM output
  wired to the PWM input)

The sketch is written for the **arduino-esp32 core 3.3.x** (ESP-IDF 5.x) and
uses the modern MCPWM capture API (`driver/mcpwm_prelude.h`) as well as the
pin-based LEDC API of core 3.x.

## Serial protocol

Every command is a single line terminated by `\n`. Read commands answer with
a decimal value on its own line, write commands only echo the command. With
debug enabled (`CFG_DBG`) every command is echoed as `Cmd:<command>` first.

| Command         | Description                                    | Response          |
| --------------- | ---------------------------------------------- | ----------------- |
| `din <0\|1>`    | read digital input                             | `0` or `1`        |
| `dout <0\|1> <0\|1>` | set digital output                        | –                 |
| `adc <0\|1>`    | read ADC input (12 bit)                        | `0..4095`         |
| `dac <0\|1> <0..255>` | set DAC output                           | –                 |
| `duty 0`        | measure PWM input duty cycle (12 bit)          | `0..4095`         |
| `freq 0`        | measure PWM input frequency in Hz              | `0..4294967295`   |
| `pwm 0 <1..1000000> <0..4095>` | set PWM output frequency / duty | – (error on invalid values) |
| `t`             | self-test: toggles duty 4 ↔ 12 at 1 kHz and prints measured `<freq> <duty> <duty-set>` | `:<f> <d> <dset>` |

Invalid commands or parameters answer with `Error: cmd parse`. If the
received line exceeds the 100 byte command buffer, `Error: command too long`
is returned.

Note: `duty`/`freq` return `0` while no complete period has been captured yet
or when the signal has disappeared (no rising edge for more than
max(2 periods, 100 ms)) — e.g. with a DC level at 0 %/100 % duty.

## Pin mapping

| Function      | GPIO |
| ------------- | ---- |
| Digital in 1  | 36   |
| Digital in 2  | 39   |
| ADC in 1      | 34   |
| ADC in 2      | 35   |
| PWM in        | 22   |
| Digital out 1 | 14   |
| Digital out 2 | 27   |
| DAC out 1     | 25   |
| DAC out 2     | 26   |
| PWM out       | 16   |

## Building and flashing

With [arduino-cli](https://arduino.github.io/arduino-cli/) and the ESP32
board package (`arduino-cli core install esp32:esp32`):

```sh
arduino-cli compile --fqbn esp32:esp32:esp32 .
arduino-cli upload --fqbn esp32:esp32:esp32 -p /dev/ttyUSB0 .
```

`CFG_DBG` (top of `Esp32DutInjector.ino`) selects the debug output port:
when `TRUE`, the command interface and debug output use `Serial` (USB /
UART0); when `FALSE`, `Serial2` on GPIO 23 (RX) / 24 (TX) is used.

## License

Licensed under the [Apache License, Version 2.0](LICENSE).
