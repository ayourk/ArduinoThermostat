# eyeSPI Thermostat

[![Build Firmware](https://github.com/ayourk/ArduinoThermostat/actions/workflows/build.yml/badge.svg)](https://github.com/ayourk/ArduinoThermostat/actions/workflows/build.yml)
[![Release](https://img.shields.io/github/v/release/ayourk/ArduinoThermostat)](https://github.com/ayourk/ArduinoThermostat/releases/latest)
[![License](https://img.shields.io/github/license/ayourk/ArduinoThermostat)](LICENSE)

**Version 1.0**

A smart thermostat with indoor/outdoor temperature monitoring, humidity, barometric pressure, and weather prediction. Designed for extreme weather conditions: -60°F to +120°F (-51°C to +49°C).

## Features

- **Indoor Temperature Control**: Standard thermostat functionality with heat/cool control
- **Outdoor Weather Station**: PT100 + BME280/BME680 sensor combination
- **Web Interface**: Remote monitoring with real-time HTML5 Canvas display
- **Extreme Cold Protection**: Automatic BME280 power management below -30°F
- **Touch Screen Display**: ILI9341 TFT with capacitive touch (FT6206/CST826)
- **Dual Network Support**: Simultaneous Ethernet (PoE) and WiFi on ESP32 boards
- **Relay Control**: Optional heating/cooling equipment control via relay FeatherWing

## Hardware

### Controller Options

**Recommended: ESP32-S2 with BME280**
- [Adafruit Feather ESP32-S2 + BME280](https://www.adafruit.com/product/5303) — Built-in environmental sensor, native WiFi, dual network capable

**Ethernet (PoE)**
- [PoE FeatherWing](https://www.tindie.com/products/silicognition/poe-featherwing/) + [M4 Shim](https://www.tindie.com/products/silicognition/m4-shim/)
- [PoE FeatherWing](https://www.tindie.com/products/silicognition/poe-featherwing/) + [RP2040 Shim](https://www.tindie.com/products/silicognition/rp2040-shim/)
- [Adafruit Feather nRF52840](https://www.adafruit.com/product/4062) + PoE FeatherWing
- [Adafruit Feather STM32F405](https://www.adafruit.com/product/4382) + PoE FeatherWing

**WiFi**
- [Adafruit Feather M0 WiFi](https://www.adafruit.com/product/3061) (WINC1500)
- [Adafruit Feather ESP32-S2](https://www.adafruit.com/product/5000) (native WiFi)
- [Adafruit Feather ESP32-S2 + BME280](https://www.adafruit.com/product/5303) (native WiFi + built-in sensor) ⭐ Recommended
- [Adafruit Feather ESP32-S3](https://www.adafruit.com/product/5477) (native WiFi)

**Dual Network (PoE + WiFi)**
- [Adafruit Feather ESP32-S2](https://www.adafruit.com/product/5000) + PoE FeatherWing
- [Adafruit Feather ESP32-S3](https://www.adafruit.com/product/5477) + PoE FeatherWing
- [Adafruit Feather ESP32 V2](https://www.adafruit.com/product/5400) + PoE FeatherWing

### Display Options

- [2.8" TFT EYESPI Breakout](https://www.adafruit.com/product/2090) — Recommended, used with eyeSPI FeatherWing
- [2.8" TFT Touch Shield](https://www.adafruit.com/product/1947)
- [2.4" TFT Touch FeatherWing](https://www.adafruit.com/product/3315)
- Custom: [eyeSPI FeatherWing PCB](https://github.com/ayourk/eyeSPI-FeatherWing-PCB)

### Relay Control (Optional)

- [Adafruit Power Relay FeatherWing](https://www.adafruit.com/product/3191) — 10A/120VAC SPDT relay
  - Close the **D13 jumper** on the back of the relay wing (see Relay-Feather-labeled.xpm)
  - Set `RELAY_PIN` to 13 in the firmware (Note: this is the same GPIO as the LED)
  - Relay draws ~100mA from 3.3V when energized

### Temperature Sensors

#### Indoor (choose one)
- **BME680/688**: [Adafruit BME688](https://www.adafruit.com/product/5046) - Temperature + Humidity + Pressure + Gas
- **BME280**: [Adafruit BME280](https://www.adafruit.com/product/2652) - Temperature + Humidity + Pressure
- **Built-in**: [ESP32-S2 Feather + BME280](https://www.adafruit.com/product/5303) - No external sensor needed
- **PT100**: [Adafruit PT100 Kit](https://www.adafruit.com/product/3328) - High precision temperature only

#### Outdoor (recommended combination)
- **PT100**: Primary temperature sensor
  - [Adafruit MAX31865 Breakout](https://www.adafruit.com/product/3328) - ~$15
  - [Waterproof PT100 Probe](https://www.adafruit.com/product/3290) - ~$10-15
  - Operating range: -200°C to +850°C (-328°F to +1562°F)
  - Accuracy: ±0.15°C

- **BME280**: Humidity + Barometric Pressure
  - [Adafruit BME280](https://www.adafruit.com/product/2652) - ~$12.95
  - Operating range: -40°C to +85°C (-40°F to +185°F)
  - **Note**: Automatically disabled below -30°F to mitigate damage

## Outdoor Sensor Module

The outdoor sensor system uses a two-sensor approach to handle extreme temperatures:

### Why Two Sensors?

No single sensor provides temperature + humidity + pressure across the full -60°F to +120°F range:

| Sensor | Min Temp | Max Temp | Measures |
|--------|----------|----------|----------|
| PT100 | -328°F | +1562°F | Temperature only |
| BME280 | -40°F | +185°F | Temp + Humidity + Pressure |
| BME680 | -40°F | +185°F | Temp + Humidity + Pressure + Gas |

The PT100 handles the full temperature range, while the BME280/BME680 provides humidity and pressure when temperatures are above -30°F.

**Note on BME Sensor Accuracy**: BME280/BME680/BME688 sensors have ±1.0°C temperature accuracy, ±3% humidity accuracy, and ±1 hPa pressure accuracy under typical conditions. However, accuracy degrades at temperature extremes (below -20°C and above +60°C). For this reason, the PT100 is used as the primary temperature sensor, while the BME provides supplemental humidity and pressure data.

### Cold Weather Strategy

When temperature drops below -30°F (-34°C):
1. BME280 is automatically powered off via MOSFET control
2. Humidity becomes meaningless anyway ("too cold to snow")
3. PT100 continues providing accurate temperature readings
4. Web interface shows "Sensor disabled (too cold)"

### Wiring

```
PT100 with MAX31865 (SPI):
  MAX31865       Arduino
  ────────       ───────
  VIN  ───────── 3.3V
  GND  ───────── GND
  CLK  ───────── SCK
  SDO  ───────── MISO
  SDI  ───────── MOSI
  CS   ───────── Pin 12

BME280 with Power Control (I2C):
  BME280         Arduino
  ──────         ───────
  VIN  ───────── 3.3V
  GND  ───┬───── [2N7000 Drain]
          │      [2N7000 Gate] ──[10K]── Pin 7
          │      [2N7000 Source] ──────── GND
  SCL  ───────── SCL
  SDA  ───────── SDA
  SDO  ───────── GND (I2C addr 0x76)
```

## Web Interface

The thermostat provides a web interface for monitoring and configuration:

- **Main page** (`/`) — Real-time HTML5 Canvas of the TFT display with touch support
- **Config page** (`/config`) — Network settings, outdoor unit configuration
- **Update page** (`/update`) — OTA firmware updates and SD card management
- **JSON API** (`/api/state`) — Full thermostat state for integration
- **Sensor endpoints** (`/indoorjson`, `/outdoorjson`) — Individual sensor data
- **SD Card API** — File browser, editor, upload, download, format (see docs)

## 3D Printed Enclosure

For outdoor use, print the enclosure in **ASA filament**:

| Filament | Heat Resist | UV Resist | Cold Perf | Verdict |
|----------|-------------|-----------|-----------|---------|
| **ASA** | 100-105°C | Excellent | Stable | ✅ BEST |
| ABS | 105°C | Poor | OK | ❌ UV degrades |
| PETG | 77-80°C | Moderate | OK | ⚠️ Hot end marginal |
| PLA | 60°C | Poor | Brittle | ❌ NO |

### Recommended ASA Sources
- [Prusament ASA](https://www.prusa3d.com/product/prusament-asa-prusa-orange-850g/) - ~$30-35/850g
- [Polymaker ASA](https://us.polymaker.com/products/polymaker-asa) - ~$25-30/kg
- Sunlu ASA (Amazon) - ~$20-25/kg
- eSun ASA+ (Amazon) - ~$22-28/kg

### Printing Requirements
- Enclosed printer required
- Nozzle: 240-260°C
- Bed: 90-110°C
- Ventilation recommended (styrene fumes)
- Use white/light gray filament to reflect summer heat

## Files

| File | Description |
|------|-------------|
| `Thermostat.ino` | Main thermostat firmware |
| `build-all.sh` | Multi-board build script |
| `arduino_secrets.h` | WiFi/network credentials template |
| `LCD7segment48pt7b.h` | Custom 7-segment LCD font (48pt) |
| `LCD7segment72pt7b.h` | Custom 7-segment LCD font (72pt) |
| `favicon_ico.h` | Embedded favicon for web interface |
| `font_generator.html` | Browser-based LCD font generator tool |
| `generate_font.js` | Font generator JavaScript |
| `lcd_7segment_font.svg` | Source vector glyphs for fonts |

### Documentation

| File | Description |
|------|-------------|
| `PROJECT_DOCUMENTATION.txt` | Complete technical reference |
| `LCD_LAYOUT_REFERENCE.txt` | Display layout coordinates and design |
| `SD-README.txt` | SD card template file documentation |
| `Temperature_Sensor_Comparison.txt` | Comprehensive sensor selection guide |
| `Thermostat_FeatherWing_Design.txt` | Custom FeatherWing PCB design notes |
| `thermostat_wiring.svg` | Hardware wiring diagram |
| `thermostat_eyespi_bridge_map.svg` | Solder bridge configuration guide |

## Hardware Notes

### SD Card Temperature Limits

Standard microSD cards are rated for -25°C (-13°F) to 85°C (185°F). For outdoor installations in extreme cold climates, use industrial-rated cards which support -40°C (-40°F) to 85°C (185°F):
- Kingston Industrial microSD (SDCIT series)
- SanDisk Industrial XI
- ATP Industrial Temperature cards

### ESP32-S2 I2C Power

The ESP32-S2 Feather requires enabling I2C power (GPIO 7) before using STEMMA QT devices:
```cpp
pinMode(PIN_I2C_POWER, OUTPUT);
digitalWrite(PIN_I2C_POWER, HIGH);
delay(10);
```
This is handled automatically by the firmware.

## Building

### Supported Boards

| Board | Network | Flash | RAM | Output |
|-------|---------|-------|-----|--------|
| Feather M4 Express | W5500 Ethernet | 42% | - | .bin |
| Feather M4 CAN | W5500 Ethernet | 42% | - | .bin |
| Feather M0 WiFi | WINC1500 WiFi | 92% | - | .bin |
| Feather M0 Express | W5500 Ethernet | 92% | - | .bin |
| Feather RP2040 | W5500 Ethernet | 3% | 5% | .uf2 |
| Feather ESP32-S2 | WiFi | 40% | 21% | .bin |
| Feather ESP32-S2 | Dual (PoE+WiFi) | 44% | 21% | .bin |
| Feather ESP32-S2 TFT | WiFi | 40% | 21% | .bin |
| Feather ESP32-S3 | WiFi | 41% | 22% | .bin |
| Feather ESP32-S3 | Dual (PoE+WiFi) | 45% | 22% | .bin |
| Feather ESP32-S3 TFT | WiFi | 41% | 22% | .bin |
| Feather ESP32 V2 | WiFi | 35% | 16% | .bin |
| Feather ESP32 V2 | Dual (PoE+WiFi) | 39% | 16% | .bin |
| Feather nRF52840 | W5500 Ethernet | 18% | 5% | .zip |
| Feather nRF52840 Sense | W5500 Ethernet | 18% | 5% | .zip |
| Feather STM32F405 | W5500 Ethernet | 9% | 3% | .bin |

### Using build-all.sh (Recommended)

```bash
# Install dependencies (interactive prompts)
./build-all.sh --install-deps

# Build all primary boards (m4, m0, rp2040, esp32s2, esp32s3, esp32s2dual)
./build-all.sh

# Build specific board(s)
./build-all.sh m4              # Feather M4 with W5500 Ethernet
./build-all.sh esp32s2         # ESP32-S2 with WiFi
./build-all.sh esp32s2dual     # ESP32-S2 with Dual Network (PoE+WiFi)

# Build and flash to connected device
./build-all.sh esp32s2dual --flash

# Build all boards including optional
./build-all.sh all

# Build with verbose output
./build-all.sh -v esp32s2dual

# List available boards
./build-all.sh --help
```

Output binaries are placed in the `binaries/` directory.

### Using Arduino IDE

1. Install board packages via Board Manager:
   - Adafruit SAMD Boards (for M0, M4)
   - Raspberry Pi Pico/RP2040 (for RP2040)
   - esp32 by Espressif (for ESP32 variants)
   - Adafruit nRF52 (for nRF52840)
   - STM32 MCU based boards (for STM32F405)

2. Install required libraries via Library Manager:
   - Adafruit GFX Library
   - Adafruit ILI9341
   - Adafruit FT6206 Library
   - Adafruit BME280 Library
   - Adafruit BME680 Library
   - Adafruit MAX31865 library
   - Adafruit BusIO
   - Adafruit Unified Sensor
   - pt100rtd (clone from [GitHub](https://github.com/drhaney/pt100rtd) — not in Library Manager)
   - ArduinoJson
   - SdFat (Adafruit Fork)
   - Ethernet

3. Copy `arduino_secrets.h` to `arduino_secrets.local.h` and edit with your credentials
4. Open `Thermostat.ino`
5. Select your board from Tools → Board menu
6. Compile and upload

## Configuration

Configuration is done via the web interface or by editing `device.cfg` on the SD card. See `PROJECT_DOCUMENTATION.txt` for detailed configuration options.

### SD Card Templates

Place these files on the SD card to customize the web interface:
- `index.html` — Main page (overrides built-in)
- `config.html` — Configuration page
- `device.cfg` — Device settings

See `SD-README.txt` for template syntax documentation.

## License

GNU General Public License v3.0 - See [LICENSE](LICENSE) for details.

## Contributing

Pull requests welcome! Please test on actual hardware before submitting.

