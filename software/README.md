# LeroiAI Odometry Board

ESP32-C6 firmware for the LeroiAI odometry board. Reads two AS5600 magnetic
encoders through a TCA9548A I2C mux, integrates an ADXRS453Z single-axis rate
gyro (Z/yaw) over SPI for heading, and streams the raw incremental encoder
deltas plus heading over UART at 50 Hz.

## Hardware

- **MCU:** ESP32-C6 (DevKitC-1 or equivalent)
- **I2C bus:** SDA = GPIO 6, SCL = GPIO 7, 400 kHz
- **TCA9548A I2C mux** at address `0x70`
  - Channel 6: AS5600 #1 (`0x36`) — encoder 1, physically closest to the MCU
  - Channel 7: AS5600 #2 (`0x36`) — encoder 2
- **ADXRS453Z gyro (SPI2):** SCK = GPIO 10, MOSI = GPIO 11, MISO = GPIO 5,
  CS = GPIO 18, 1 MHz, SPI mode 0
- **UART out (→ TTL/RS485):** TX = GPIO 17, RX = GPIO 16, 115200 baud

See `main/main.c` (`PIN SETTINGS` / `I2C DEVICE SETTINGS` / `SPI / ADXRS453
SETTINGS` sections) for everything that's configurable without code changes.

## Toolchain

- **ESP-IDF v6.0.1** — exact version this code was developed and tested with.
  Other 6.x versions may work but are not verified.
- Target: **esp32c6** (already set in `sdkconfig`).
- Host: Windows / macOS / Linux. Windows install on the development machine is
  at `C:\Espressif`; other machines must install ESP-IDF separately.

### Installing ESP-IDF on a new machine

Follow the [official ESP-IDF setup guide](https://docs.espressif.com/projects/esp-idf/en/v6.0.1/esp32c6/get-started/index.html)
for your OS, selecting v6.0.1 as the version. The Windows installer (EIM) is
the easiest path on Windows.

## Build & Flash

All commands below are run from the `software/` directory.

### 1. Activate the ESP-IDF Environment

**Windows:**
```powershell
# Option 1: Run the shortcut (easiest)
# "ESP-IDF 6.0.1 PowerShell" from the Start menu

# Option 2: Manually activate in PowerShell
. "C:\Espressif\tools\Microsoft.v6.0.1.PowerShell_profile.ps1"
```

**macOS/Linux:**
```bash
. ~/esp/esp-idf/export.sh
```

### 2. Build

```bash
idf.py set-target esp32c6     # Only needed on a fresh checkout / first build
idf.py build
```

### 3. Find the Serial Port

**Windows (Device Manager):**
1. Plug in the ESP32-C6 board via USB.
2. Open **Device Manager** (`Win+X` → Device Manager).
3. Expand **Ports (COM & LPT)**.
4. Look for **Silicon Labs CP210x USB to UART Bridge** (or similar — ESP32
   boards typically use this driver) and note the COM port (e.g. `COM3`).
   - If no device appears, install the [CP210x driver](https://www.silabs.com/developers/usb-to-uart-bridge-vcp-drivers).

**macOS/Linux:**
```bash
ls /dev/tty.* /dev/cu.*       # macOS
ls /dev/ttyUSB* /dev/ttyACM*  # Linux
```

### 4. Flash & Monitor

```bash
# Flash and start monitoring serial output
idf.py -p <PORT> flash monitor

# Windows example:
idf.py -p COM3 flash monitor

# macOS/Linux example:
idf.py -p /dev/ttyUSB0 flash monitor
```

Exit the monitor with **Ctrl+]** (or **Cmd+]** on macOS).

### One-Command Build, Flash & Monitor

```bash
idf.py -p <PORT> build flash monitor
```

### Other Useful Commands

```bash
idf.py fullclean              # Wipe the build directory
idf.py -p <PORT> monitor      # Monitor only, without reflashing
idf.py -p <PORT> erase-flash  # Erase the entire flash (factory reset)
```

## Output Format

UART streams at 50 Hz with the following format (newline-terminated):

```
DENC1=-12,DENC2=8,H=45.3456
```

### Fields
- **DENC1** — Encoder 1 (mux channel 6) *incremental* tick delta since the last
  UART packet. 4096 ticks per revolution. **Not** an accumulated absolute count.
- **DENC2** — Encoder 2 (mux channel 7) incremental tick delta since the last
  UART packet, same units as DENC1.
- **H** — Integrated gyro heading in degrees (Z-axis / yaw only). The ADXRS453
  is a single-axis rate gyro, integrated over time.

Because DENC1/DENC2 are deltas, the host accumulates them to recover absolute
position. The encoder delta wrapping (±2048 tick rollover) is handled on-device.

### Converting Ticks to Distance

```
distance = ticks * π * wheel_diameter / 4096
```

Example with a 2.0 in wheel: 4096 ticks = 1 full revolution = 2π inches ≈
6.28 inches.

## Calibration Notes

- **Gyro bias** is calibrated automatically at startup over ~10 s
  (`GYRO_BIAS_SAMPLES = 2000` × `GYRO_BIAS_DELAY_MS = 5`). **Keep the board
  completely still during this startup window.** See the `HEADING TUNING`
  section in [main.c](main/main.c#L108).

- **Heading sign** (`GYRO_SIGN` in [main.c](main/main.c#L114)) is `+1.0` by
  default. Flip to `-1.0` if a known turn reports the wrong direction.

- **Heading scale** (`HEADING_SCALE` in [main.c](main/main.c#L121)) is `1.00`
  by default. To improve absolute accuracy: rotate the board exactly 360° from
  a reference orientation, note the H delta, and set
  `HEADING_SCALE = 360 / observed_delta`.

- **Gyro deadband** (`GYRO_DEADBAND_DPS` in [main.c](main/main.c#L118))
  suppresses tiny near-zero residuals after bias calibration. Keep it small.

After any calibration changes, rebuild and flash using the commands above.

## TODO

- Update the 81208U 15-inch odom board — either build another board with a new
  layout, or add support for the TOF sensors.
```