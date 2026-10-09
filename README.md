# Esp32DutInjector

A serial-controlled signal injector / measurement tool for an ESP32 (classic,
v1) device-under-test (DUT) bench setup. The firmware exposes digital inputs
and outputs, ADC inputs, DAC outputs, a PWM generator and a PWM measurement
input over a simple line-based ASCII command interface on the serial port
(115200 baud, 8N1, commands terminated with `\n`). All I/O states can also be
controlled directly on the device via a rotary encoder menu on the OLED
display.

## Features

- 2 digital inputs (GPIO 15, GPIO 2)
- 2 digital outputs (GPIO 17, GPIO 18)
- 2 ADC inputs (GPIO 32, GPIO 33, 12 bit, 0..4095)
- 2 DAC outputs (GPIO 25, GPIO 26, 8 bit, 0..255)
- 1 PWM output (GPIO 14, LEDC, 12 bit resolution)
- 1 PWM measurement input (GPIO 27, MCPWM capture, both edges,
  frequency + duty cycle measurement)
- OLED status display (SSD1306 128x64, I2C) showing all I/O states
- Rotary encoder menu (GPIO 13/16/4) to select and edit outputs directly on
  the device
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

Serial commands and the encoder menu write the same state and can be mixed
freely.

## OLED status display

An SSD1306 128x64 OLED (I2C, address `0x3C`) shows all I/O states, refreshed
every 200 ms (50 ms while the encoder menu is active):

```
inp1:0    inp2:1
out1:1    out2:0
adc1:1234 adc2:2345
dac1: 128 dac2: 255
pwmi1:   1000Hz  25%
pwmo1:   1000Hz  25%
```

- `inp1`/`inp2`/`out1`/`out2`: digital state (0/1)
- `adc1`/`adc2`: raw 12-bit ADC values
- `pwmi1`: measured frequency (Hz) and duty cycle (%)
- `dac1`/`dac2`: set DAC values (0..255)
- `pwmo1`: set PWM frequency (Hz) and duty cycle (%)

The item currently selected by the encoder menu is underlined. The value being
edited is shown inverted.

Requires the **U8g2** library (`arduino-cli lib install "U8g2"`). If the
display is not found on the I2C bus, the firmware runs without it (status is
reported on the serial port at startup).

## Rotary encoder menu

A rotary encoder with push button (CLK = GPIO 13, DT = GPIO 16,
button = GPIO 4) controls the outputs directly on the device. The button is
debounced (30 ms) and active high.

| Action            | Effect                                                          |
| ----------------- | --------------------------------------------------------------- |
| Rotate            | move selection through `out1` → `out2` → `dac1` → `dac2` → `pwmo1` (5 detents per item) |
| Button on `out1`/`out2` | toggle the digital output (no edit mode)                    |
| Button on `dac1`/`dac2` | enter value edit: rotate changes the DAC value (±1 per detent), button returns to selection |
| Button on `pwmo1` | enter frequency edit: rotate changes frequency (±10 Hz per detent); next button press switches to duty edit (±20 of 4095 per detent); next button press returns to selection |

All changed values are applied immediately and shown on the display. The step
sizes are defined as `kDacStep`, `kPwmFreqStep` and `kPwmDutyStep` at the top
of `Esp32DutInjector.ino`.

## Pin mapping

| Function        | GPIO |
| --------------- | ---- |
| Digital in 1    | 15   |
| Digital in 2    | 2    |
| ADC in 1        | 32   |
| ADC in 2        | 33   |
| PWM in          | 27   |
| Digital out 1   | 17   |
| Digital out 2   | 18   |
| DAC out 1       | 25   |
| DAC out 2       | 26   |
| PWM out         | 14   |
| OLED SDA        | 21   |
| OLED SCL        | 23   |
| Encoder CLK     | 13   |
| Encoder DT      | 16   |
| Encoder button  | 4    |

Note: the encoder button is active high (button to 3V3, internal pulldown).
GPIO 2 and GPIO 15 are strapping pins — do not pull them in the wrong
direction at reset.

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
