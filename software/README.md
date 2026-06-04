# LeroiAI Odometry Board

ESP32-C6 firmware for the LeroiAI odometry board. Reads two AS5600 magnetic
encoders (vertical + horizontal) through a TCA9548A I2C mux, integrates a
WitMotion IMU for heading, runs onboard chord-corrected odometry, and streams
results over UART at 50 Hz.

## Hardware

- **MCU:** ESP32-C6 (DevKitC-1 or equivalent)
- **I2C bus:** SDA = GPIO 6, SCL = GPIO 7, 400 kHz
- **TCA9548A I2C mux** at address `0x70`
  - Channel 0: WitMotion IMU (`0x50`)
  - Channel 6: AS5600 #2 (vertical / forward tracker)
  - Channel 7: AS5600 #1 (horizontal / right tracker)
- **UART out:** TX = GPIO 17, RX = GPIO 16, 115200 baud

See `main/main.c` (`PIN SETTINGS` / `DEVICE SETTINGS` / `ODOMETRY` sections)
for everything that's configurable without code changes.

## Toolchain

- **ESP-IDF v6.0.1** — exact version this code was developed and tested with.
  Other 6.x versions may work but are not verified.
- Host: Windows / macOS / Linux. Windows install is at `C:\esp\v6.0.1\esp-idf`
  on the development machine; other machines must install ESP-IDF separately.

### Installing ESP-IDF on a new machine

Follow the [official ESP-IDF setup guide](https://docs.espressif.com/projects/esp-idf/en/v6.0.1/esp32c6/get-started/index.html)
for your OS, selecting v6.0.1 as the version. The Windows installer (EIM) is
the easiest path on Windows.

## Build & Flash

### Activate ESP-IDF Environment

**Windows:**
```powershell
# Option 1: Run the shortcut (easiest)
# "ESP-IDF 6.0.1 PowerShell" from Start menu

# Option 2: Manually activate in PowerShell
. "C:\Espressif\tools\Microsoft.v6.0.1.PowerShell_profile.ps1"
```

**macOS/Linux:**
```bash
. ~/esp/esp-idf/export.sh
```

### Build Command

From the `software/` directory:

```bash
idf.py set-target esp32c6     # Only needed on first build
idf.py build
```

### Flash to Board

#### Find the Serial Port

**Windows (Device Manager):**
1. Plug in the ESP32-C6 board via USB
2. Open **Device Manager** (`Win+X` → Device Manager, or search "Device Manager")
3. Expand **Ports (COM & LPT)**
4. Look for **Silicon Labs CP210x USB to UART Bridge** (or similar—ESP32 boards use this driver)
5. Note the COM port number (e.g., `COM3`, `COM4`)
   - If no device appears, install the [CP210x driver](https://www.silabs.com/developers/usb-to-uart-bridge-vcp-drivers)

**macOS/Linux:**
```bash
# List available serial ports
ls /dev/tty.* /dev/cu.*    # macOS
ls /dev/ttyUSB* /dev/ttyACM*  # Linux
```

#### Flash Command

Once you have the port, run:

```bash
# Flash and start monitoring output
idf.py -p <PORT> flash monitor

# Example on Windows:
idf.py -p COM3 flash monitor

# Example on macOS/Linux:
idf.py -p /dev/ttyUSB0 flash monitor
```

To exit the monitor, press **Ctrl+]** (or **Cmd+]** on macOS).

### One-Command Build & Upload

Combine build and flash in a single command:

```bash
idf.py -p <PORT> build flash monitor
```

## Output Format

UART streams at 50 Hz with the following format:

```
T=1234,ENC1=-512,ENC2=4096,H=45.3456
```

### Fields
- **T** — Milliseconds since boot (useful for detecting pauses or jitter)
- **ENC1** — Horizontal tracker encoder ticks (4096 ticks per revolution), positive = rightward
- **ENC2** — Vertical tracker encoder ticks (4096 ticks per revolution), positive = forward
- **H** — Heading in degrees (gyro-integrated with bias refinement), CW positive, 0° = facing +Y

### Converting Ticks to Distance

With a wheel diameter of 2.0 inches (as configured):

```
distance = ticks * π * wheel_diameter / 4096
```

Example: 4096 ticks = 1 full revolution = 2π inches ≈ 6.28 inches

## Calibration Notes

- **Gyro bias** is calibrated automatically over the first ~1.5 s after boot
  (300 samples) — keep the board still during startup. Bias is then
  continuously refined whenever the board is detected as stationary (all
  three gyro axes below `STILL_GYRO_THRESHOLD` for `STILL_REQUIRED_MS`).
  See `HEADING TUNING` section in [main.c](main/main.c#L749).

- **Heading scale** (`HEADING_SCALE` in [main.c](main/main.c#L757)) is `1.00` by default.
  To improve absolute accuracy: rotate the board exactly 360° from a
  reference orientation, note the H delta, and set
  `HEADING_SCALE = 360 / observed_delta`.

- **Tracker wheel diameter and offsets** live in the `ODOMETRY` section of
  [main.c](main/main.c#L699). Units are arbitrary but must be consistent:
  - `WHEEL_DIAMETER_VERT` — vertical (forward) tracker wheel
  - `WHEEL_DIAMETER_HORIZ` — horizontal (right) tracker wheel  
  - `VERT_OFFSET` — right-distance from rotation center to vertical tracker
  - `HORIZ_OFFSET` — forward-distance from rotation center to horizontal tracker

After any calibration changes, rebuild and flash using the commands above.
