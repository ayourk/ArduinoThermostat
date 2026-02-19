
//////
//
// ===========================================================================
// FTP SERVER - NOT POSSIBLE (confirmed 2026-02-19)
// ===========================================================================
// Extensive investigation confirmed FTP server cannot coexist with the
// ILI9341 TFT display on ESP32-S2 eyeSPI hardware.
//
// CONFIRMED WORKING:   Display + SD card (no FTP) -- ALWAYS works
// CONFIRMED BROKEN:    Display + SD card + FTP     -- display flashes/fails
//
// Approaches tried and failed:
//   1. SimpleFTPServer (STORAGE_SDFAT2, NETWORK_ESP32)   -- display flashes
//   2. SimpleFTPServer (NETWORK_W5100, Ethernet-only)    -- display flashes
//   3. Chip Select management (all CS HIGH before init)  -- no change
//   4. SHARED_SPI mode for SdFat                         -- no change
//   5. Moving SD card to HSPI (SPI3) second bus          -- crashes at boot
//   6. Delayed FTP init (new FtpServer() in setup())     -- crashes in begin()
//   7. Local fork with SdFs extern fix                   -- crashes in begin()
//   8. ESP32_FTPServer_SD_MMC                            -- incompatible (SD_MMC)
//   9. esp8266FTPServer                                  -- incompatible (SPIFFS)
//
// Root cause: SimpleFTPServer's EthernetServer construction / SPI handling
// interferes with the ILI9341 display driver on the shared SPI bus.
// No known fix without a deep rewrite of SimpleFTPServer's network layer.
// ===========================================================================
//
//  Parts to build (current recommended):
//
//  Controller (Feather + PoE networking):
//      PoE-FeatherWing      https://www.tindie.com/products/silicognition/poe-featherwing/
//                            Also available: https://www.crowdsupply.com/silicognition/poe-featherwing
//                                           https://www.amazon.com/dp/B08KTVD7BR
//      RP2040-Shim           https://www.tindie.com/products/silicognition/rp2040-shim/
//
//  Screen (2.8" ILI9341 cap-touch with EYESPI):
//      Adafruit #2090        https://www.adafruit.com/product/2090
//
//  Temperature/Environment Sensor:
//      Adafruit #5046        https://www.adafruit.com/product/5046   (BME688/BME680)
//
//  Controller alternatives:
//      Feather M0 WiFi       https://www.adafruit.com/product/3061   (WINC1500 WiFi)
//
//  Discontinued / limited availability:
//      M4-Shim               https://www.tindie.com/products/silicognition/m4-shim/
//                             (ATSAMD51J19 chip shortage; replaced by RP2040-Shim)
//
//  Legacy display hardware (Arduino shield form factor, not FeatherWing):
//      Adafruit #1947        https://www.adafruit.com/product/1947   (2.8" TFT cap-touch shield)
//      Adafruit #1651        https://www.adafruit.com/product/1651   (2.8" TFT resistive shield)
//      Adafruit #1480        https://www.adafruit.com/product/1480   (2.2" TFT breakout)
//
//  Relay: GPIO 13 by default (for Adafruit Power Relay FeatherWing #3191).
//         Close the GPIO 13 solder jumper on the relay wing.  Override via
//         RELAY_PIN in device.cfg or change the compile-time default below.
//
////

// Suppress specific GCC warnings that are triggered by Arduino/Adafruit libraries:
//   push         - Save the current diagnostic state so we can restore it later
//   -Wstrict-aliasing   - Type-punning warnings (common in low-level hardware libs)
//   -Wstrict-overflow   - Signed overflow assumption warnings (optimization hints)
//   -Wstrict-null-sentinel - NULL vs nullptr warnings (C++11 compatibility)
// The matching "pop" at end of file restores normal warning behavior.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wstrict-aliasing"
#pragma GCC diagnostic ignored "-Wstrict-overflow"
#pragma GCC diagnostic ignored "-Wstrict-null-sentinel"

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_MAX31865.h> // PT100
#include <pt100rtd.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <Adafruit_FT6206.h>
#include "favicon_ico.h"
#include <Fonts/FreeSans9pt7b.h>
#include <Fonts/FreeSansBold9pt7b.h>
// FreeSans24pt7b removed -- was only used for +/- glyphs
#include "LCD7segment48pt7b.h"  // 7 segment digital monospace font, 48pt
#include "LCD7segment72pt7b.h"  // 7 segment digital monospace font, 72pt
#include <Adafruit_Sensor.h>    // Required by Adafruit_BME680
#include <ArduinoJson.h>        // JSON serialization for REST API endpoints
#include <SdFat.h>              // SD card filesystem for web pages and config

// OTA Update support for SAMD boards
#if defined(ARDUINO_ARCH_SAMD)
#include <InternalStorage.h>
#define OTA_SUPPORTED 1
#else
#define OTA_SUPPORTED 0
#endif

// Celsius to Fahrenheit conversion macro
#define C2F(c) ((9 * c / 5) + 32)

// sbrk declaration for free RAM calculation (SAMD)
#if defined(__SAMD51__) || defined(__SAMD21__)
extern "C" char* sbrk(int incr);
#endif

#define BAUDRATE 9600
#define WEBSERVER_PORT 80

// Max WiFi networks in scan cache / foreach loops - scaled by board RAM
// Each WifiScanResult uses ~40 bytes (String SSID + int RSSI + String enc)
#if defined(ESP32) || defined(ESP_WIFI)
  #define TPL_MAX_FOREACH_ITEMS 64   // ESP32: 320KB+ RAM
#elif defined(ARDUINO_ARCH_RP2040)
  #define TPL_MAX_FOREACH_ITEMS 64   // RP2040: 264KB RAM
#elif defined(__SAMD51__)
  #define TPL_MAX_FOREACH_ITEMS 48   // M4: 192KB RAM
#elif defined(__SAMD21__)
  #define TPL_MAX_FOREACH_ITEMS 24   // M0: 32KB RAM
#else
  #define TPL_MAX_FOREACH_ITEMS 16   // Unknown: conservative default
#endif

// Set to 1 to enable verbose PT100 RTD serial diagnostics every loop()
// iteration.  Useful during sensor bring-up; disable for normal operation
// to keep the serial console readable.
#define DEBUG_PT100 0

// Relay output pin for heat control (compile-time default, can be overridden
// by RELAY_PIN in device.cfg).  Set to a valid GPIO number to enable relay
// switching, or -1 to disable (software-only heat tracking).  The pin drives
// HIGH when heat is on, LOW when off.  Use a logic-level MOSFET or relay
// module (e.g., Adafruit Power Relay FeatherWing #3191).
// Suggested pins: 13, A0-A5 on M4/M0 (free pins on eyeSPI FeatherWing).
// device.cfg accepts: RELAY_PIN=14 (GPIO number), RELAY_PIN=OFF, or RELAY_PIN=-1
#define RELAY_PIN 13  // GPIO 13 for Adafruit Power Relay FeatherWing

// SD card Card Detect pin.  Set to a valid GPIO number if you have wired
// the CD pad from the TFT breakout to a GPIO.  Active-low: LOW = no card,
// HIGH = card inserted (directly from the microSD socket switch).
// Note: Adafruit labels this pad "CD" (Card Detect) on the TFT breakout,
// but we use "SDDET" here to distinguish it from other CD abbreviations.
// The eyeSPI 18-pin FPC connector does not include Card Detect; a separate
// wire from the TFT's CD pad is required.  Set to -1 to use polling instead.
#define SDDET_PIN A0  // A0 default, or -1 for polling mode

// SD card hot-swap re-probe interval in milliseconds.  Only used when
// SDDET_PIN is -1 (polling mode).  The firmware will periodically call
// sd.begin() to detect card insertion/removal.  Set to 0 to disable
// hot-swap detection entirely (probe only at boot).
#define SD_REPROBE_INTERVAL_MS 5000  // 5 seconds, or 0 to disable

// ===========================================================================
// BME280/BME680 Power Control (for extreme cold outdoor units)
// ===========================================================================
// For outdoor units in extreme cold climates, the BME sensor can be powered
// from a GPIO pin to allow firmware-controlled power cycling.  This protects
// the sensor from damage when temperature drops below its -40°C limit.
//
// Set BME_POWER_PIN to the GPIO controlling BME power:
//   - A1 is recommended (A0 is used for SD card detect)
//   - Set to -1 to disable power control (BME always powered from 3.3V)
//
// Wiring for power control:
//   BME VIN  → GPIO A1 (instead of 3.3V)
//   BME GND  → GND
//   BME SDA  → SDA
//   BME SCL  → SCL
//
// Note: Integrated BME280 on ESP32-S2 Feather #5303 cannot be power-controlled.
// For extreme cold, use a separate BME breakout with power control wiring.
#define BME_POWER_PIN -1  // A1 for power control, or -1 for always-on

// Temperature threshold for BME power control (only used if BME_POWER_PIN >= 0)
// BME280/BME680 operating range is -40°C; we use -34°C (-30°F) for safety margin
#define BME_MIN_TEMP_F -30.0  // Power off BME below this temperature

// ===========================================================================
// Hardware Configuration
// ===========================================================================

// No manual configuration needed!
// - Network medium is auto-selected by board detection above
// - PT100 RTD is always enabled (primary temperature sensor)
// - BME280/BME680/BME688 are detected at startup via I2C

// ===========================================================================
// Board Detection & Network Auto-Selection
// ===========================================================================
// Network medium is determined by board type:
//   M4 (SAMD51)  -> W5500 Ethernet via Silicognition PoE-FeatherWing
//   M0 (SAMD21)  -> WINC1500 WiFi (Feather M0 WiFi has it built-in)
//   RP2040-Shim  -> W5500 Ethernet via Silicognition PoE-FeatherWing
//   ESP32        -> Native WiFi (or DUAL_NETWORK with PoE FeatherWing)
//
// DUAL_NETWORK mode (ESP32 + PoE FeatherWing):
//   Add "#define DUAL_NETWORK" at line 1 of this file to enable both
//   W5500 Ethernet and native ESP32 WiFi simultaneously. This allows
//   the thermostat to be accessed from either network interface.
//   Requires: ESP32-S2/S3 + Adafruit PoE FeatherWing (#5765)

// Handle DUAL_NETWORK first - it implies both W5500 and ESP_WIFI on ESP32
#if defined(DUAL_NETWORK)
  #if !defined(ESP32) && !defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S2) && \
      !defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S3) && !defined(ARDUINO_ADAFRUIT_FEATHER_ESP32_V2)
    #error "DUAL_NETWORK requires ESP32 board (ESP32-S2, ESP32-S3, or ESP32 V2)"
  #endif
  #define W5500
  #define ESP_WIFI
#endif

#if defined(__SAMD51__) || defined(ADAFRUIT_FEATHER_M4_EXPRESS)
  // Feather M4 Express / M4-Shim on PoE-FeatherWing
  const String curBoard = "Adafruit Feather M4 (PoE Ethernet)";
  #define W5500
  #define TFT_DC 9
  #define TFT_CS 6   // 10 is taken by W5500 PoE FeatherWing
  #define SD_CS  5
#elif defined(ARDUINO_ADAFRUIT_FEATHER_RP2040)
  // RP2040-Shim on PoE-FeatherWing
  const String curBoard = "Adafruit Feather RP2040 (PoE Ethernet)";
  #define W5500
  #define TFT_DC 9
  #define TFT_CS 6   // 10 is taken by W5500 PoE FeatherWing
  #define SD_CS  5
#elif defined(ARDUINO_SAMD_FEATHER_M0) || defined(ARDUINO_SAMD_ZERO) || defined(ADAFRUIT_FEATHER_M0) || \
      defined(ARDUINO_SAMD_ADAFRUIT_FEATHER_M0) || defined(__SAMD21G18A__) || defined(__SAMD21__)
  // Feather M0 WiFi with built-in ATWINC1500
  const String curBoard = "Adafruit Feather M0 WiFi";
  #define WINC1500
  #define TFT_DC 9
  #define TFT_CS 6
  #define SD_CS  5
#elif defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S3) || defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S3_TFT)
  #if defined(DUAL_NETWORK)
    const String curBoard = "Adafruit Feather ESP32-S3 (Dual Network)";
  #else
    const String curBoard = "Adafruit Feather ESP32-S3";
    #if !defined(W5500) && !defined(WINC1500) && !defined(ESP_WIFI)
      #define ESP_WIFI  // Auto-detect native WiFi
    #endif
  #endif
  #define TFT_DC 9
  #define TFT_CS 6
  #define SD_CS  5
#elif defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S2) || defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S2_TFT)
  #if defined(DUAL_NETWORK)
    const String curBoard = "Adafruit Feather ESP32-S2 (Dual Network)";
  #else
    const String curBoard = "Adafruit Feather ESP32-S2";
    #if !defined(W5500) && !defined(WINC1500) && !defined(ESP_WIFI)
      #define ESP_WIFI  // Auto-detect native WiFi
    #endif
  #endif
  #define TFT_DC 9
  #define TFT_CS 6
  #define SD_CS  5
#elif defined(ARDUINO_ADAFRUIT_FEATHER_ESP32_V2)
  #if defined(DUAL_NETWORK)
    const String curBoard = "Adafruit Feather ESP32 V2 (Dual Network)";
  #else
    const String curBoard = "Adafruit Feather ESP32 V2";
    #if !defined(W5500) && !defined(WINC1500) && !defined(ESP_WIFI)
      #define ESP_WIFI  // Auto-detect native WiFi
    #endif
  #endif
  #define TFT_DC 9
  #define TFT_CS 6
  #define SD_CS  5
#elif defined(__AVR_ATmega2560__)
  #error "Arduino MEGA 2560 not supported - only 8KB RAM available, sketch requires ~24KB. Use Feather M0/M4/RP2040."
#else
  const String curBoard = "Unknown board (" __FILE__ " line " + String(__LINE__) + ")";
  #define TFT_DC 9
  #define TFT_CS 6
  #define SD_CS  5
#endif

// Helper macro: true if this build has WiFi capability
// Must be defined AFTER board detection sets WINC1500/ESP_WIFI/DUAL_NETWORK
#if defined(WINC1500) || defined(ESP_WIFI) || defined(DUAL_NETWORK)
  #define HAS_WIFI 1
#endif

// Helper macro: true if this build has battery voltage monitoring
// WINC1500 (Feather M0 WiFi) has LiPoly connector with voltage divider on A7
// ESP32 Feathers have LiPoly connector with voltage divider on A2
#if defined(WINC1500) || defined(ESP_WIFI)
  #define HAS_BATT 1
#endif

// ===========================================================================
// Networking
// ===========================================================================

#if defined(DUAL_NETWORK)
  // DUAL_NETWORK: Both Ethernet (W5500) and WiFi (ESP32 native)
  // Uses ESP32's ETH.h library which integrates W5500 with lwIP stack
  // This allows a single WiFiServer to serve both interfaces via 0.0.0.0
  #define HardwareNet    WiFi
  #define HardwareServer WiFiServer
  #define HardwareClient WiFiClient
  #include <WiFi.h>
  #include <ETH.h>
  #if __has_include("arduino_secrets.local.h")
    #include "arduino_secrets.local.h"
  #else
    #include "arduino_secrets.h"
  #endif
  char ssid[] = SECRET_SSID;
  char pass[] = SECRET_PASS;
  int status = WL_IDLE_STATUS;

  // W5500 SPI Ethernet configuration for ESP32 ETH.h
  #define W5500_MAC_I2C  0x50   // I2C address of 24AA02E48 MAC EEPROM on PoE FeatherWing
  #define W5500_CS_PIN   10    // SPI chip select for W5500
  #define W5500_IRQ_PIN  -1    // No IRQ pin used (polling mode)
  #define W5500_RST_PIN  -1    // No reset pin used
  byte W5500_mac[] = { 0x0E, 0xAD, 0xBE, 0xEF, 0xFE, 0xE0 };
  // Static IP fallback (used if DHCP fails)
  IPAddress eth_ip(10, 1, 0, 71);
  IPAddress eth_dns(10, 1, 0, 1);

  // Track ETH connection state
  bool ethConnected = false;

  // Read MAC from 24AA02E48 EEPROM on PoE FeatherWing
  byte readRegister(byte r) {
    Wire.beginTransmission(W5500_MAC_I2C);
    Wire.write(r);
    Wire.endTransmission();
    Wire.requestFrom(W5500_MAC_I2C, 1);
    while (!Wire.available());
    return Wire.read();
  }

  // ESP32 WiFi.encryptionType() returns wifi_auth_mode_t, convert to string
  String encTypeStr(int encType) {
    switch (encType) {
      case 0: return "Open";       // WIFI_AUTH_OPEN
      case 1: return "WEP";        // WIFI_AUTH_WEP
      case 2: return "WPA-PSK";    // WIFI_AUTH_WPA_PSK
      case 3: return "WPA2-PSK";   // WIFI_AUTH_WPA2_PSK
      case 4: return "WPA/WPA2";   // WIFI_AUTH_WPA_WPA2_PSK
      case 5: return "WPA2-ENT";   // WIFI_AUTH_WPA2_ENTERPRISE
      case 6: return "WPA3-PSK";   // WIFI_AUTH_WPA3_PSK
      case 7: return "WPA2/WPA3";  // WIFI_AUTH_WPA2_WPA3_PSK
      default: return "Auto";
    }
  }

  // Get current connected network's encryption type via quick scan
  String getCurrentWifiSecurity() {
    String currentSSID = WiFi.SSID();
    if (currentSSID.length() == 0) return "N/A";

    int n = WiFi.scanNetworks(false, false, false, 100);  // Quick scan, 100ms per channel
    for (int i = 0; i < n; i++) {
      if (WiFi.SSID(i) == currentSSID) {
        String result = encTypeStr(WiFi.encryptionType(i));
        WiFi.scanDelete();
        return result;
      }
    }
    WiFi.scanDelete();
    return "Unknown";
  }

  // ETH event handler for connection status
  void onEthEvent(arduino_event_id_t event) {
    switch (event) {
      case ARDUINO_EVENT_ETH_START:
        Serial.println("ETH Started");
        ETH.setHostname("thermostat");
        break;
      case ARDUINO_EVENT_ETH_CONNECTED:
        Serial.println("ETH Connected");
        break;
      case ARDUINO_EVENT_ETH_GOT_IP:
        Serial.print("ETH Got IP: ");
        Serial.println(ETH.localIP());
        ethConnected = true;
        break;
      case ARDUINO_EVENT_ETH_DISCONNECTED:
        Serial.println("ETH Disconnected");
        ethConnected = false;
        break;
      case ARDUINO_EVENT_ETH_STOP:
        Serial.println("ETH Stopped");
        ethConnected = false;
        break;
      default:
        break;
    }
  }

  // Compatibility layer: Create an "Ethernet" object that wraps ETH for existing code
  // This allows Ethernet.localIP(), Ethernet.linkStatus() etc. to work
  class EthernetCompat {
  public:
    IPAddress localIP() { return ETH.localIP(); }
    IPAddress subnetMask() { return ETH.subnetMask(); }
    IPAddress gatewayIP() { return ETH.gatewayIP(); }
    IPAddress dnsServerIP() { return ETH.dnsIP(); }
    bool linkStatus() { return ETH.linkUp(); }
  };
  EthernetCompat Ethernet;

  // Define link status constants if not defined
  #define LinkON true
  #define LinkOFF false
  // Note: EthernetNoHardware check not needed with ETH.h - it handles errors in begin()

#elif defined(W5500)
  #define HardwareNet    Ethernet
  #define HardwareServer EthernetServer
  #define HardwareClient EthernetClient
  #include <Ethernet.h>
  #define W5500_MAC_I2C  0x50
  #define W5500_CS_PIN   10   // SPI chip select for W5500
  byte W5500_mac[] = { 0x0E, 0xAD, 0xBE, 0xEF, 0xFE, 0xE0 };
  // Static IP fallback (used only if DHCP fails)
  IPAddress eth_ip(10, 1, 0, 71);
  IPAddress eth_dns(10, 1, 0, 1);

  byte readRegister(byte r) {
    Wire.beginTransmission(W5500_MAC_I2C);
    Wire.write(r);
    Wire.endTransmission();
    Wire.requestFrom(W5500_MAC_I2C, 1);
    while (!Wire.available());
    return Wire.read();
  }

#elif defined(WINC1500)
  #define HardwareNet    WiFi
  #define HardwareServer WiFiServer
  #define HardwareClient WiFiClient
  #include <WiFi101.h>
  #if __has_include("arduino_secrets.local.h")
    #include "arduino_secrets.local.h"
  #else
    #include "arduino_secrets.h"
  #endif
  char ssid[] = SECRET_SSID;
  char pass[] = SECRET_PASS;
  int status = WL_IDLE_STATUS;

  // WiFi101 encryptionType() values (from wl_definitions.h)
  String encTypeStr(int encType) {
    switch (encType) {
      case 2: return "WPA";        // ENC_TYPE_TKIP
      case 4: return "WPA2";       // ENC_TYPE_CCMP
      case 5: return "WEP";        // ENC_TYPE_WEP
      case 7: return "Open";       // ENC_TYPE_NONE
      case 8: return "Auto";       // ENC_TYPE_AUTO
      default: return "Unknown";
    }
  }

  // Get current connected network's encryption type via quick scan
  String getCurrentWifiSecurity() {
    String currentSSID = WiFi.SSID();
    if (currentSSID.length() == 0) return "N/A";

    int n = WiFi.scanNetworks();
    for (int i = 0; i < n; i++) {
      if (String(WiFi.SSID(i)) == currentSSID) {
        return encTypeStr(WiFi.encryptionType(i));
      }
    }
    return "Unknown";
  }

#elif defined(ESP_WIFI)
  // ESP32 native WiFi (ESP32-S2, ESP32-S3, ESP32 V2, etc.)
  #define HardwareNet    WiFi
  #define HardwareServer WiFiServer
  #define HardwareClient WiFiClient
  #include <WiFi.h>
  #if __has_include("arduino_secrets.local.h")
    #include "arduino_secrets.local.h"
  #else
    #include "arduino_secrets.h"
  #endif
  char ssid[] = SECRET_SSID;
  char pass[] = SECRET_PASS;
  int status = WL_IDLE_STATUS;
  
  // ESP32 WiFi.encryptionType() returns wifi_auth_mode_t, convert to string
  String encTypeStr(int encType) {
    switch (encType) {
      case 0: return "Open";       // WIFI_AUTH_OPEN
      case 1: return "WEP";        // WIFI_AUTH_WEP
      case 2: return "WPA-PSK";    // WIFI_AUTH_WPA_PSK
      case 3: return "WPA2-PSK";   // WIFI_AUTH_WPA2_PSK
      case 4: return "WPA/WPA2";   // WIFI_AUTH_WPA_WPA2_PSK
      case 5: return "WPA2-ENT";   // WIFI_AUTH_WPA2_ENTERPRISE
      case 6: return "WPA3-PSK";   // WIFI_AUTH_WPA3_PSK
      case 7: return "WPA2/WPA3";  // WIFI_AUTH_WPA2_WPA3_PSK
      default: return "Unknown";
    }
  }
  
  // Get current connected network's encryption type via quick scan
  // ESP32 doesn't provide encryptionType() for current network, only for scan results
  String getCurrentWifiSecurity() {
    String currentSSID = WiFi.SSID();
    if (currentSSID.length() == 0) return "N/A";
    
    int n = WiFi.scanNetworks(false, false, false, 100);  // Quick scan, 100ms per channel
    for (int i = 0; i < n; i++) {
      if (WiFi.SSID(i) == currentSSID) {
        String result = encTypeStr(WiFi.encryptionType(i));
        WiFi.scanDelete();  // Free scan results memory
        return result;
      }
    }
    WiFi.scanDelete();
    return "Unknown";
  }

#else
  // Stub classes so the project compiles without networking hardware.
  class HardwareClient {
  public:
    HardwareClient(bool = false) {}
    HardwareClient& operator=(bool) { return *this; }
    bool connected() { return false; }
    bool available() { return false; }
    String readStringUntil(char) { return ""; }
    void stop() {}
    void flush() {}
    // Write methods needed for ArduinoJson compatibility
    size_t write(uint8_t c) { return 0; }
    size_t write(const uint8_t* buf, size_t n) { return 0; }
    size_t write(const char* s) { return 0; }
    int read() { return -1; }  // Single-byte read
    size_t read(uint8_t*, size_t) { return 0; }
    void print(int) {}
    void print(float, int) {}
    void print(const String& s = "") {}
    void print(const char* s) {}
    void println(int) {}
    void println(const String& s = "") {}
    void println(const char* s) {}
    operator bool() const { return false; }
  };

  class HardwareServer {
  public:
    HardwareServer(int = 0) {}
    void begin() {}
    HardwareClient available() { return HardwareClient(); }
  };

  // Stub IPAddress for no-networking builds
  class IPAddress {
  public:
    uint8_t _addr[4];
    IPAddress(uint8_t a = 0, uint8_t b = 0, uint8_t c = 0, uint8_t d = 0) {
      _addr[0] = a; _addr[1] = b; _addr[2] = c; _addr[3] = d;
    }
    uint8_t operator[](int i) const { return _addr[i]; }
    bool fromString(const char*) { return false; }
    bool fromString(const String& s) { return fromString(s.c_str()); }
    operator bool() const { return _addr[0] || _addr[1] || _addr[2] || _addr[3]; }
  };
#endif

// Single server instance - works for all modes
// In DUAL_NETWORK mode with ESP32 ETH.h, WiFiServer binds to 0.0.0.0
// which serves both Ethernet (via lwIP) and WiFi interfaces
HardwareServer server(WEBSERVER_PORT);

// Helper to convert IPAddress to String (works for all network variants)
String IpAddress2String(const IPAddress& ipAddress) {
  return String(ipAddress[0]) + "." + String(ipAddress[1]) + "." +
         String(ipAddress[2]) + "." + String(ipAddress[3]);
}

// ---------------------------------------------------------------------------
// Utility Helpers (shared across multiple call sites)
// ---------------------------------------------------------------------------

// Format a 6-byte MAC address as "XX:XX:XX:XX:XX:XX"
String macToString(const byte *mac) {
  char buf[18];
  sprintf(buf, "%02X:%02X:%02X:%02X:%02X:%02X",
          mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  return String(buf);
}

#ifdef WINC1500
// Map WiFi101 encryption type constants to human-readable strings
String encTypeStr(uint8_t encType) {
  switch (encType) {
    case ENC_TYPE_WEP:  return "WEP";
    case ENC_TYPE_TKIP: return "WPA";
    case ENC_TYPE_CCMP: return "WPA2";
    case ENC_TYPE_NONE: return "Open";
    default:            return "Auto";
  }
}
#endif

#if defined(HAS_BATT)
// Read battery voltage from analog pin (Feather boards with LiPo connector)
float readBatteryVoltage() {
#if defined(ESP_WIFI)
  // ESP32-S2/S3 Feathers: battery on A2 (GPIO 1/2), 12-bit ADC
  // Note: Pin varies by board; A2 works for ESP32-S2 Feather
  float v = analogReadMilliVolts(A2);  // Returns millivolts directly
  v *= 2;      // voltage divider
  v /= 1000;   // Convert mV to V
  return v;
#else
  // WINC1500 (Feather M0 WiFi) uses A7 and 10-bit ADC
  float v = analogRead(A7);
  v *= 2;      // voltage divider
  v *= 3.3;    // reference voltage
  v /= 1024;   // 10-bit ADC
  return v;
#endif
}
#endif

// ===========================================================================
// ESP32 API Compatibility
// ===========================================================================
// ESP32 Arduino Core 3.x renamed some network functions:
//   SERVER_ACCEPT() -> server.accept()
//   client.flush()     -> client.clear() (for clearing RX buffer)
// These macros provide cross-platform compatibility.

#if defined(ESP_WIFI) || defined(DUAL_NETWORK)
  #define SERVER_ACCEPT() server.accept()
  #define CLIENT_CLEAR(c) (c).clear()
#else
  #define SERVER_ACCEPT() server.available()
  #define CLIENT_CLEAR(c) (c).flush()
#endif

// Send common HTTP response headers
// Use Print& to allow both EthernetClient and WiFiClient (both inherit from Print)
void sendJsonHeaders(Print &cl) {
  cl.println(F("HTTP/1.1 200 OK\r\nContent-Type: application/json\r\nAccess-Control-Allow-Origin: *\r\nConnection: close\r\n"));
}
void sendHtmlHeaders(Print &cl) {
  cl.println(F("HTTP/1.1 200 OK\r\nContent-Type: text/html\r\nCache-Control: no-cache, no-store, must-revalidate\r\nPragma: no-cache\r\nExpires: 0\r\nConnection: close\r\n"));
}

// ===========================================================================
// Display & Touch
// ===========================================================================

Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC);
Adafruit_FT6206 ts = Adafruit_FT6206();

// ===========================================================================
// SD Card (SdFat) - SD-hosted web pages with Smarty-style {$variable} tags
// ===========================================================================

// SD_FAT_TYPE: 0=SdFat, 1=FAT16/FAT32, 2=exFAT, 3=FAT16/FAT32+exFAT
#define SD_FAT_TYPE 3

#if SD_FAT_TYPE == 0
  typedef SdFat sd_t;
  typedef File file_t;
#elif SD_FAT_TYPE == 1
  typedef SdFat32 sd_t;
  typedef File32 file_t;
#elif SD_FAT_TYPE == 2
  typedef SdExFat sd_t;
  typedef ExFile file_t;
#elif SD_FAT_TYPE == 3
  typedef SdFs sd_t;
  typedef FsFile file_t;
#else
  #error Invalid SD_FAT_TYPE
#endif

sd_t sd;

// ===========================================================================
// Temperature Sensors
// ===========================================================================

// PT100 RTD via MAX31865 (always present)
Adafruit_MAX31865 thermo = Adafruit_MAX31865(12);  // Hardware SPI, CS pin 12
pt100rtd PT100lt = pt100rtd();
#define RREF      429.83    // Measured Rref resistor value
#define RREF2     428.985
#define RNOMINAL  100.0     // 100.0 for PT100, 1000.0 for PT1000

// BME environmental sensors (runtime-detected via I2C)
#include <Adafruit_BME280.h>
#include <Adafruit_BME680.h>
#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BME280 bme280;  // I2C
Adafruit_BME680 bme680;  // I2C

// ===========================================================================
// Required Library Checks
// ===========================================================================
// These #error directives catch missing libraries at compile time rather
// than producing cryptic linker errors.  Install missing libraries via
// Arduino IDE → Sketch → Include Library → Manage Libraries.
//
// Version checks enforce minimums for API features the firmware relies on:
//   ArduinoJson 7.x  -- JsonDocument (auto-sizing), to<JsonObject>()
//                        (v6 used StaticJsonDocument/createNestedObject;
//                         v5 lacks deserializeJson entirely)
//   SdFat 2.x        -- SdFat32/File32/ExFile/FsFile typed classes
//                        (v1 uses only SdFat/SdFile; no ExFat support)

#if !defined(ADAFRUIT_MAX31865_H)
  #error "Missing library: Adafruit_MAX31865 -- Install 'Adafruit MAX31865' via Library Manager"
#endif
#if !defined(PT100RTD_H)
  #error "Missing library: pt100rtd -- Install 'pt100rtd' via Library Manager"
#endif
#if !defined(_ADAFRUIT_GFX_H)
  #error "Missing library: Adafruit_GFX -- Install 'Adafruit GFX Library' via Library Manager"
#endif
#if !defined(_ADAFRUIT_ILI9341H_)
  #error "Missing library: Adafruit_ILI9341 -- Install 'Adafruit ILI9341' via Library Manager"
#endif
#if !defined(ADAFRUIT_FT6206_LIBRARY)
  #error "Missing library: Adafruit_FT6206 -- Install 'Adafruit FT6206 Library' via Library Manager"
#endif
#if !defined(_ADAFRUIT_SENSOR_H)
  #error "Missing library: Adafruit_Sensor -- Install 'Adafruit Unified Sensor' via Library Manager"
#endif
#if !defined(ARDUINOJSON_VERSION)
  #error "Missing library: ArduinoJson -- Install 'ArduinoJson' by Benoit Blanchon via Library Manager"
#elif ARDUINOJSON_VERSION_MAJOR < 6
  #error "ArduinoJson 6.x or 7.x required -- upgrade via Library Manager"
#endif

// ArduinoJson 6 vs 7 compatibility
// In v7, JsonDocument is directly usable; in v6, we need DynamicJsonDocument
// In v7, use doc["key"].to<JsonObject>(); in v6, use doc.createNestedObject("key")
#if ARDUINOJSON_VERSION_MAJOR >= 7
  #define JSON_DOC JsonDocument
  #define CREATE_NESTED_OBJ(parent, key) parent[key].to<JsonObject>()
#else
  // ArduinoJson 6.x - use DynamicJsonDocument with reasonable size
  #define JSON_DOC DynamicJsonDocument
  #define JSON_DOC_SIZE 1024
  #define CREATE_NESTED_OBJ(parent, key) parent.createNestedObject(key)
#endif
// SdFat check - look for SDFAT_FILE_TYPE which is defined in SdFatConfig.h
#if !defined(SDFAT_FILE_TYPE)
  #error "Missing or outdated library: SdFat 2.x required -- Install 'SdFat' by Bill Greiman via Library Manager"
#endif
// BME280 check - library uses __BME280_H__
#if !defined(__BME280_H__)
  #error "Missing library: Adafruit_BME280 -- Install 'Adafruit BME280 Library' via Library Manager"
#endif
// BME680 check - library uses __BME680_H__
#if !defined(__BME680_H__)
  #error "Missing library: Adafruit_BME680 -- Install 'Adafruit BME680 Library' via Library Manager"
#endif
#if defined(W5500) && !defined(DUAL_NETWORK)
  // W5500-only mode uses Paul Stoffregen's Ethernet library
  #if !defined(MAX_SOCK_NUM)
    #error "Missing library: Ethernet -- Install 'Ethernet' via Library Manager (W5500 variant)"
  #endif
#endif
#if defined(DUAL_NETWORK)
  // DUAL_NETWORK uses ESP32's built-in ETH.h library (lwIP integration)
  #if !defined(CONFIG_ETH_ENABLED) && !defined(ETH_PHY_W5500)
    // Note: ETH_PHY_W5500 is defined in ETH.h when W5500 support is compiled in
  #endif
#endif
#ifdef WINC1500
  #if !defined(WIFI_FIRMWARE_LATEST_MODEL_B)
    #error "Missing library: WiFi101 -- Install 'WiFi101' via Library Manager (WINC1500 variant)"
  #endif
#endif

// ===========================================================================
// Global State
// ===========================================================================

// Forward declarations
unsigned long LayoutDisplay();
void sendPage(HardwareClient cl);
void sendPageFromSD(HardwareClient cl);
void sendUpdatePage(HardwareClient cl);
void sendApiState(HardwareClient cl);
void sendSensorJson(HardwareClient cl, bool asOutdoor);
void sendConfigPage(HardwareClient cl, const String &req);
void handleFirmwareUpload(HardwareClient& client, long contentLength);
void handleSDUpload(HardwareClient& client, long contentLength);
void sendSDFilesPage(HardwareClient cl);
void handleSDList(HardwareClient& cl, const String& query);
void handleSDDelete(HardwareClient& cl, long contentLength);
void handleSDRead(HardwareClient& cl, const String& query);
void handleSDWrite(HardwareClient& cl, long contentLength);
void handleSDFormat(HardwareClient& cl, long contentLength);
void handleSDDownload(HardwareClient& cl, const String& query);
void handleSDFileUpload(HardwareClient& cl, long contentLength, const String& req);
void send404Error(HardwareClient cl);
bool serveSDFile(HardwareClient cl, const String &path);
uint32_t crc32Stream(file_t &f);
void loadDeviceConfig();
void saveDeviceConfig();
void pollOutdoor();
void drawSettingsScreen();
void generateIndexHtml();
void reprobeSDCard();

// Forward declarations for BME sensor presence (set in setup())
bool bme280Present = false;
bool bme680Present = false;
bool bmePoweredOn = true;  // BME power state (for power control feature)

// --- Weather prediction based on barometric pressure ---
// Uses pressure history for trend when available, otherwise uses current pressure
// Based on NOAA thresholds: rapid change = 0.06 inHg/hour (0.18 over 3 hours)

#define PRESSURE_HISTORY_SIZE  12  // 3 hours at 15-min intervals
#define PRESSURE_INTERVAL_MS   (15UL * 60UL * 1000UL)  // 15 minutes

float pressureHistory[PRESSURE_HISTORY_SIZE];
int pressureIndex = 0;
bool pressureHistoryFull = false;
unsigned long lastPressureRecord = 0;

void recordPressure(float pressInHg) {
  pressureHistory[pressureIndex] = pressInHg;
  pressureIndex = (pressureIndex + 1) % PRESSURE_HISTORY_SIZE;
  if (pressureIndex == 0) pressureHistoryFull = true;
  lastPressureRecord = millis();
}

// Returns pressure change over 3 hours in inHg (positive = rising)
float getPressureTrend3hr() {
  if (!pressureHistoryFull) return 0;
  int oldest = pressureIndex;
  int newest = (pressureIndex - 1 + PRESSURE_HISTORY_SIZE) % PRESSURE_HISTORY_SIZE;
  return pressureHistory[newest] - pressureHistory[oldest];
}

// Returns trend string based on NOAA thresholds
String getPressureTrendStr() {
  if (!pressureHistoryFull) return "";  // No trend data yet
  float trend = getPressureTrend3hr();
  if (trend > 0.18) return "Rising Rapidly";
  if (trend > 0.06) return "Rising";
  if (trend > -0.06) return "Steady";
  if (trend > -0.18) return "Falling";
  return "Falling Rapidly";
}

// Calculate dew point from temperature and humidity (Magnus formula)
// Returns dew point in Fahrenheit
float getDewPointF(float tempF, float humidity) {
  if (humidity <= 0 || humidity > 100) return tempF;
  float tempC = (tempF - 32.0) * 5.0 / 9.0;
  float a = 17.27;
  float b = 237.7;
  float alpha = ((a * tempC) / (b + tempC)) + log(humidity / 100.0);
  float dewC = (b * alpha) / (a - alpha);
  return (dewC * 9.0 / 5.0) + 32.0;
}

// Returns weather prediction using pressure, humidity, and temperature
// tempF: current temperature in Fahrenheit
// humidity: relative humidity percentage (0-100)
// pressInHg: barometric pressure in inches of mercury
String getWeatherPrediction(float pressInHg, float humidity, float tempF) {
  String trend = getPressureTrendStr();
  float t = getPressureTrend3hr();

  // Calculate dew point for fog/frost detection
  float dewPoint = getDewPointF(tempF, humidity);
  float dewSpread = tempF - dewPoint;  // Difference between temp and dew point

  // Precipitation type based on temperature
  // Rain: >35°F, Mixed: 28-35°F, Snow: <28°F, Freezing rain: 28-35°F with rain above
  bool snowTemp = (tempF < 28);
  bool mixedTemp = (tempF >= 28 && tempF <= 35);
  bool rainTemp = (tempF > 35);
  bool frostRisk = (tempF <= 36 && dewPoint <= 32);  // Frost possible

  // High humidity increases precipitation likelihood
  bool veryHumid = (humidity > 85);
  bool humid = (humidity > 70);
  bool dry = (humidity < 40);

  // Fog conditions: dew spread < 5°F and humidity > 80%
  bool fogLikely = (dewSpread < 5 && humidity > 80);
  bool fogPossible = (dewSpread < 10 && humidity > 70);

  // --- With trend data (after 3 hours) ---
  if (trend != "") {
    // Rapidly falling + low pressure = storm
    if (t < -0.18 && pressInHg < 29.80) {
      if (snowTemp) return "Snow storm likely";
      if (mixedTemp) return "Winter storm likely";
      if (fogLikely) return "Storm, fog likely";
      return "Storm likely";
    }

    // Falling pressure with high humidity = precipitation
    if (t < -0.06 && humid) {
      if (snowTemp) return "Snow likely";
      if (mixedTemp && veryHumid) return "Freezing rain risk";
      if (mixedTemp) return "Rain/snow mix";
      return "Rain likely";
    }

    // Falling pressure
    if (t < -0.06) {
      if (snowTemp && humid) return "Snow possible";
      if (humid) return "Precip possible";
      if (fogPossible) return "Changing, fog risk";
      return "Changing";
    }

    // Rising + high pressure = clearing
    if (t > 0.06 && pressInHg > 30.00) {
      if (frostRisk && !dry) return "Clearing, frost risk";
      if (dry) return "Clear & dry";
      return "Clearing";
    }

    // High pressure with fog potential (often morning fog with high pressure)
    if (pressInHg > 30.20) {
      if (fogLikely && tempF < 50) return "Fair, fog likely";
      if (frostRisk) return "Fair, frost risk";
      if (dry) return "Fair & dry";
      if (tempF > 85 && humid) return "Hot & humid";
      return "Fair";
    }

    // Low pressure
    if (pressInHg < 29.80) {
      if (veryHumid && snowTemp) return "Snow possible";
      if (veryHumid && mixedTemp) return "Freezing rain risk";
      if (veryHumid) return "Precip possible";
      if (fogLikely) return "Unsettled, foggy";
      return "Unsettled";
    }

    // Steady conditions
    if (fogLikely) {
      if (frostRisk) return "Fog & frost likely";
      return "Fog likely";
    }
    if (veryHumid && pressInHg < 30.00) {
      if (snowTemp) return "Flurries possible";
      return "Drizzle possible";
    }
    if (frostRisk && pressInHg > 30.00) return "Clear, frost risk";
    return "Stable";
  }

  // --- Without trend data (immediate prediction) ---
  // Very low pressure = storm conditions
  if (pressInHg < 29.40) {
    if (veryHumid && snowTemp) return "Heavy snow";
    if (veryHumid && mixedTemp) return "Ice storm risk";
    if (veryHumid) return "Stormy";
    if (snowTemp) return "Snow likely";
    return "Stormy";
  }

  // Low pressure
  if (pressInHg < 29.80) {
    if (veryHumid && snowTemp) return "Snow likely";
    if (veryHumid && mixedTemp) return "Freezing rain risk";
    if (veryHumid) return "Rain likely";
    if (humid && snowTemp) return "Snow possible";
    if (humid) return "Precip possible";
    if (fogLikely) return "Unsettled, foggy";
    return "Unsettled";
  }

  // Normal pressure
  if (pressInHg < 30.00) {
    if (fogLikely) {
      if (frostRisk) return "Fog & frost";
      return "Fog possible";
    }
    if (veryHumid && snowTemp) return "Flurries possible";
    if (veryHumid) return "Drizzle possible";
    if (humid) return "Cloudy";
    return "Variable";
  }

  // Above normal
  if (pressInHg < 30.20) {
    if (fogLikely && tempF < 50) return "Fog possible";
    if (frostRisk) return "Clear, frost risk";
    if (dry) return "Clear";
    if (tempF > 85 && humid) return "Hot & humid";
    return "Mostly clear";
  }

  // High pressure (>30.20)
  if (fogLikely && tempF < 50) return "Fair, fog likely";
  if (frostRisk) return "Fair, frost risk";
  if (dry) return "Fair & dry";
  if (tempF > 90 && humid) return "Hot & humid";
  if (tempF > 85 && veryHumid) return "Hot & muggy";
  if (humid && tempF > 80) return "Warm & humid";
  return "Fair";
}

// Returns pressure category: "High", "Normal", "Low"
String getPressureCategory(float pressInHg) {
  if (pressInHg > 30.20) return "High";
  if (pressInHg >= 29.80) return "Normal";
  return "Low";
}

int myround(double mydecimal) {
  if (bme280Present || bme680Present) {
    return ((int)(mydecimal + 0.5));
  } else {
    return ((int)(mydecimal));
  }
}

int TFTbackgroundColor = ILI9341_BLACK;
int coordTouchLight[] = {0, 0, 0, 0, 0};
int coordTouchSystem[] = {0, 0, 0, 0, 0};
// coordMinus/coordPlus removed -- glyphs no longer drawn
bool isHeatOn = true;
bool isCurHeatOn = false;

// Furnace short-cycle protection (matches FocusPro behavior)
// User-initiated changes (target temp adjustment) get a short 5s debounce.
// Automatic changes (temp drift causing relay to toggle) get full 3-min protection.
unsigned long lastHeatChange = 0;           // millis() of last relay state change
bool lastChangeWasUser = false;             // true if last change was user-initiated
const unsigned long userDebounce   = 5UL * 1000UL;          // 5s after user adjustment
const unsigned long minRunTime     = 3UL * 60UL * 1000UL;   // min ON  time (3 min, auto)
const unsigned long minOffTime     = 3UL * 60UL * 1000UL;   // min OFF time (3 min, auto)
double curIndoorTemp = 71.0;
int curIndoorTempOld = 71;
int targIndoorTemp = 71;
int targIndoorTempOld = 71;
bool headerPrinted = true;
// Settings overlay state: 0=off, 1=lan, 2=wifi
// For single-network devices, only 0 and the appropriate network type are used
#define SETTINGS_OFF  0
#define SETTINGS_LAN  1
#define SETTINGS_WIFI 2
int settingsScreen = SETTINGS_OFF;
bool displayPresent = false;
// bme280Present and bme680Present declared earlier (before myround())
bool sdCardPresent = false;
bool sdIndexPresent = false;
unsigned long lastSDReprobe = 0;  // millis() of last SD card re-probe
unsigned long lastSensorRead = 0; // millis() of last PT100/sensor read

// Get SD card filesystem type as a string
String getSDCardFormat() {
  if (!sdCardPresent) return "N/A";
  #if SD_FAT_TYPE == 3
    // SdFs supports both FAT and exFAT - check which is mounted
    switch (sd.fatType()) {
      case FAT_TYPE_FAT12: return "FAT12";
      case FAT_TYPE_FAT16: return "FAT16";
      case FAT_TYPE_FAT32: return "FAT32";
      case FAT_TYPE_EXFAT: return "exFAT";
      default: return "Unknown";
    }
  #elif SD_FAT_TYPE == 2
    return "exFAT";
  #elif SD_FAT_TYPE == 1
    return (sd.fatType() == FAT_TYPE_FAT16) ? "FAT16" : "FAT32";
  #else
    return (sd.fatType() == FAT_TYPE_FAT16) ? "FAT16" : "FAT32";
  #endif
}
unsigned long lastStateChange = 0;
bool settingsDrawn = false;

// ===========================================================================
// Outdoor Unit Configuration
// ===========================================================================
// Any unit can serve /outdoorjson and /indoorjson.  An "indoor" (master)
// unit can also poll a remote "outdoor" unit for exterior sensor data.
// Configuration is stored in device.cfg on the SD card (key=value).

IPAddress outdoorIP(0, 0, 0, 0);     // 0.0.0.0 = not configured
uint16_t  outdoorPort       = 80;
uint16_t  outdoorPollSecs   = 30;     // seconds between polls
bool      outdoorConfigured = false;  // true if outdoorIP != 0.0.0.0

// Cached outdoor data from last successful poll
String outdoorTemp       = "";
String outdoorHumidity   = "";
String outdoorPressHpa   = "";
String outdoorPressInhg  = "";
String outdoorGasKohms   = "";
String outdoorBmeType    = "";    // "bme680", "bme280", or ""
String outdoorBoard      = "";
String outdoorEthIP      = "";    // outdoor unit's Ethernet IP
String outdoorWifiIP     = "";    // outdoor unit's WiFi IP
String outdoorIface      = "";    // outdoor unit's interface type (dual/ethernet/wifi/none)
String outdoorBatteryV   = "";    // outdoor unit's battery voltage (or "")
bool   outdoorHasEth     = false; // outdoor unit has Ethernet
bool   outdoorHasWifi    = false; // outdoor unit has WiFi

// Outdoor status tracking
String        outdoorStatus    = "N/A";  // "Online", "Offline", "N/A"
unsigned long outdoorLastPoll  = 0;      // millis() of last poll attempt
unsigned long outdoorLastOK    = 0;      // millis() of last success
uint8_t       outdoorFailCount = 0;      // consecutive failures

// Reset outdoor unit state to defaults (used by both POST and GET clear)
void clearOutdoorConfig() {
  outdoorIP = IPAddress(0, 0, 0, 0);
  outdoorPort = 80;
  outdoorPollSecs = 30;
  outdoorConfigured = false;
  outdoorStatus = "N/A";
  outdoorTemp = "";
  outdoorHumidity = "";
  outdoorPressHpa = "";
  outdoorPressInhg = "";
  outdoorGasKohms = "";
  outdoorBmeType = "";
  outdoorBoard = "";
  outdoorEthIP = "";
  outdoorWifiIP = "";
  outdoorIface = "";
  outdoorBatteryV = "";
  outdoorHasEth = false;
  outdoorHasWifi = false;
}

// Network configuration (loaded from device.cfg, applied at boot)
#if defined(DUAL_NETWORK)
  // DUAL_NETWORK: Separate config for each interface
  // Ethernet (W5500) config
  IPAddress cfgEthIP(0, 0, 0, 0);    // 0.0.0.0 = use DHCP
  IPAddress cfgEthDNS(0, 0, 0, 0);
  IPAddress cfgEthSubnet(255, 255, 255, 0);
  IPAddress cfgEthGateway(0, 0, 0, 0);
  bool cfgMacOverride = false;  // true if ETH_MAC was set in device.cfg
  
  // WiFi config
  IPAddress cfgWifiIP(0, 0, 0, 0);   // 0.0.0.0 = use DHCP
  IPAddress cfgWifiDNS(0, 0, 0, 0);
  IPAddress cfgWifiSubnet(255, 255, 255, 0);
  IPAddress cfgWifiGateway(0, 0, 0, 0);
  String cfgWifiSSID = "";      // override from device.cfg
  String cfgWifiPass = "";      // override from device.cfg
  String cfgWifiSecurity = "";   // WPA2, WPA, WEP, Open
  
  // Aliases for backward compatibility in shared code paths
  #define cfgIP cfgEthIP
  #define cfgDNS cfgEthDNS
  #define cfgSubnet cfgEthSubnet
  #define cfgGateway cfgEthGateway
#else
  // Single-interface mode: shared config variables
  IPAddress cfgIP(0, 0, 0, 0);    // 0.0.0.0 = use DHCP
  IPAddress cfgDNS(0, 0, 0, 0);
  IPAddress cfgSubnet(255, 255, 255, 0);
  IPAddress cfgGateway(0, 0, 0, 0);
  #ifdef W5500
    bool cfgMacOverride = false;  // true if ETH_MAC was set in device.cfg
  #endif
  #if defined(HAS_WIFI)
    String cfgWifiSSID = "";      // override from device.cfg (empty = use compile-time SECRET_SSID)
    String cfgWifiPass = "";      // override from device.cfg (empty = use compile-time SECRET_PASS)
    String cfgWifiSecurity = "";   // WPA2, WPA, WEP, Open -- from device.cfg WIFI_SECURITY key
  #endif
#endif

// Relay pin (loaded from device.cfg, falls back to RELAY_PIN compile-time default)
int cfgRelayPin = RELAY_PIN;  // -1 = disabled, or GPIO number

// Client interface tracking (set before template rendering)
// Values: "ethernet", "wifi", "unknown"
String currentClientInterface = "unkn";

// ===========================================================================
// setup()
// ===========================================================================

void setup() {
  char printbuf[256] = "";

  Serial.begin(BAUDRATE);
  unsigned long timeout = millis() + 5000;
  while (!Serial && millis() < timeout);
  delay(1000);  // Extra delay for USB CDC to stabilize on ESP32-S2

  // --- ESP32-S2 I2C Power Enable ---
  // ESP32-S2 Feather boards (including #5303 with BME280) have a power switch
  // on GPIO 7 that controls the STEMMA QT / I2C bus.  Must be HIGH to enable.
  #if defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S2) || defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S2_TFT)
    #if defined(I2C_POWER) || defined(PIN_I2C_POWER)
      #if defined(PIN_I2C_POWER)
        pinMode(PIN_I2C_POWER, OUTPUT);
        digitalWrite(PIN_I2C_POWER, HIGH);
      #else
        pinMode(I2C_POWER, OUTPUT);
        digitalWrite(I2C_POWER, HIGH);
      #endif
      delay(10);  // Allow I2C bus to stabilize
      Serial.println("ESP32-S2: I2C power enabled (GPIO 7)");
    #endif
  #endif

  // --- TFT Display Detection ---
  Serial.print("Thermostat ILI9341 Test! on ");
  Serial.println(curBoard);

  // Initialize SPI bus before display detection (required for ESP32)
  // On ESP32-S2: SCK=GPIO36, MOSI=GPIO35, MISO=GPIO37
  SPI.begin();
  delay(10);  // Allow SPI to stabilize

  // Debug: Print SPI pin info
  Serial.print("SPI pins: SCK=");
  Serial.print(SCK);
  Serial.print(" MOSI=");
  Serial.print(MOSI);
  Serial.print(" MISO=");
  Serial.println(MISO);
  Serial.print("TFT pins: CS=");
  Serial.print(TFT_CS);
  Serial.print(" DC=");
  Serial.println(TFT_DC);

  // Detect ILI9341 by reading display ID via SPI before tft.begin()
  pinMode(TFT_CS, OUTPUT);
  pinMode(TFT_DC, OUTPUT);
  digitalWrite(TFT_CS, HIGH);
  delay(1);
  digitalWrite(TFT_CS, LOW);
  digitalWrite(TFT_DC, LOW);  // Command mode
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  SPI.transfer(0x04);  // RDDID command
  digitalWrite(TFT_DC, HIGH);  // Data mode
  uint8_t dummy = SPI.transfer(0x00);
  uint8_t id1 = SPI.transfer(0x00);
  uint8_t id2 = SPI.transfer(0x00);
  uint8_t id3 = SPI.transfer(0x00);
  SPI.endTransaction();
  digitalWrite(TFT_CS, HIGH);

  // Debug: Print raw display ID bytes for troubleshooting
  Serial.print("Display ID bytes: 0x");
  Serial.print(dummy, HEX); Serial.print(" 0x");
  Serial.print(id1, HEX); Serial.print(" 0x");
  Serial.print(id2, HEX); Serial.print(" 0x");
  Serial.println(id3, HEX);

  // ILI9341 returns 0x00, 0x93, 0x41 for display ID
  // Also accept 0x3F pattern seen on some ESP32-S2 setups (timing issue?)
  displayPresent = (id2 == 0x93 && id3 == 0x41) ||
                   (id3 == 0x3F) ||  // ESP32-S2 workaround
                   (id1 != 0xFF && id2 != 0xFF && id3 != 0xFF &&
                    !(id1 == 0x00 && id2 == 0x00 && id3 == 0x00));

  // Force display present for testing on ESP32-S2 with eyeSPI
  #if defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S2)
    if (!displayPresent) {
      Serial.println("ESP32-S2: Forcing display detection (eyeSPI workaround)");
      displayPresent = true;
    }
  #endif

  if (displayPresent) {
    Serial.println("TFT Display: DETECTED");
    tft.begin();

    uint8_t x = tft.readcommand8(ILI9341_RDMODE);
    Serial.print("Display Power Mode: 0x"); Serial.println(x, HEX);
    x = tft.readcommand8(ILI9341_RDMADCTL);
    Serial.print("MADCTL Mode: 0x"); Serial.println(x, HEX);
    x = tft.readcommand8(ILI9341_RDPIXFMT);
    Serial.print("Pixel Format: 0x"); Serial.println(x, HEX);
    x = tft.readcommand8(ILI9341_RDIMGFMT);
    Serial.print("Image Format: 0x"); Serial.println(x, HEX);
    x = tft.readcommand8(ILI9341_RDSELFDIAG);
    Serial.print("Self Diagnostic: 0x"); Serial.println(x, HEX);

    tft.fillScreen(TFTbackgroundColor);
    tft.setRotation(1);  // Landscape, USB at upper-left
    tft.setFont();
    tft.setTextSize(1);
    tft.setTextWrap(false);
    sprintf(printbuf, "Max X: %d, Max Y: %d", tft.width(), tft.height());
    Serial.println(printbuf);
  } else {
    Serial.println("TFT Display: NOT PRESENT (headless mode)");
  }

  // --- Touchscreen (only if display present) ---
  if (displayPresent) {
    if (!ts.begin(40)) {
      Serial.println("Unable to start touchscreen.");
    } else {
      Serial.println("Touchscreen started.");
    }
  }

  // --- Temperature Sensors ---

  // PT100 RTD (always present)
  thermo.begin(MAX31865_3WIRE);
  Serial.println("PT100 RTD initialized (3-wire)");

  // BME power control pin (for extreme cold outdoor units)
  #if BME_POWER_PIN >= 0
    pinMode(BME_POWER_PIN, OUTPUT);
    digitalWrite(BME_POWER_PIN, HIGH);  // Power on BME
    bmePoweredOn = true;
    delay(50);  // Allow sensor to stabilize after power-up
    Serial.print("BME power control enabled on GPIO ");
    Serial.println(BME_POWER_PIN);
  #endif

  // BME environmental sensor detection (try BME680/688 first, then BME280)
  // Both use default I2C address 0x77 on Adafruit breakouts
  Serial.println(F("Probing for BME environmental sensor..."));
  if (bme680.begin()) {
    bme680Present = true;
    Serial.println("BME680/688 sensor found (chip ID 0x61)");
    bme680.setTemperatureOversampling(BME680_OS_8X);
    bme680.setHumidityOversampling(BME680_OS_2X);
    bme680.setPressureOversampling(BME680_OS_4X);
    bme680.setIIRFilterSize(BME680_FILTER_SIZE_3);
    bme680.setGasHeater(0, 150);  // 320C for 150 ms
  } else if (bme280.begin()) {
    bme280Present = true;
    Serial.println("BME280 sensor found (chip ID 0x60)");
  } else {
    Serial.println("No BME sensor found - humidity/pressure/gas disabled");
  }

  // --- SD Card (before networking so device.cfg can override defaults) ---
  Serial.print("SD card: ");
  if (sd.begin(SD_CS, SD_SCK_MHZ(4))) {
    sdCardPresent = true;
    Serial.print("DETECTED (");
    Serial.print(getSDCardFormat());
    Serial.println(")");
    // Check for index.html in root
    file_t testFile;
    if (testFile.open("index.html", O_RDONLY)) {
      sdIndexPresent = true;
      testFile.close();
      Serial.println("SD index.html: FOUND (overrides firmware page)");
    } else {
      sdIndexPresent = false;
      Serial.println("SD index.html: not found -- generating default template");
      generateIndexHtml();
      // Verify generation succeeded
      if (testFile.open("index.html", O_RDONLY)) {
        sdIndexPresent = true;
        testFile.close();
        Serial.println("SD index.html: generated successfully");
      } else {
        Serial.println("SD index.html: generation failed (using firmware page)");
      }
    }

    // Write embedded favicon to SD if not already present
    file_t favFile;
    if (favFile.open("favicon.ico", O_RDONLY)) {
      favFile.close();
      Serial.println("SD favicon.ico: FOUND");
    } else {
      Serial.print("SD favicon.ico: not found -- writing from PROGMEM...");
      if (favFile.open("favicon.ico", O_WRONLY | O_CREAT | O_TRUNC)) {
        const size_t chunkSize = 128;
        uint8_t buf[chunkSize];
        for (size_t i = 0; i < sizeof favicon_ico; i += chunkSize) {
          size_t n = min(chunkSize, sizeof favicon_ico - i);
          memcpy_P(buf, favicon_ico + i, n);
          favFile.write(buf, n);
        }
        favFile.close();
        Serial.println(" OK");
      } else {
        Serial.println(" FAILED (write error)");
      }
    }

    // Load device config (network + outdoor unit + relay pin) from SD
    loadDeviceConfig();
  } else {
    sdCardPresent = false;
    Serial.println("not present or init failed");
  }

  // --- Relay output pin (after config load so device.cfg can override) ---
  if (cfgRelayPin >= 0) {
    pinMode(cfgRelayPin, OUTPUT);
    digitalWrite(cfgRelayPin, LOW);  // Start with heat OFF
    Serial.print("Relay output: GPIO "); Serial.println(cfgRelayPin);
  } else {
    Serial.println("Relay output: DISABLED (software-only heat tracking)");
  }

  // --- SD Card Detect pin (if wired separately from eyeSPI) ---
#if SDDET_PIN >= 0
  pinMode(SDDET_PIN, INPUT_PULLUP);  // CD pin is active-low
  Serial.print("SD Card Detect: GPIO "); Serial.println(SDDET_PIN);
#else
  #if SD_REPROBE_INTERVAL_MS > 0
    Serial.println("SD Card Detect: polling mode");
  #else
    Serial.println("SD Card Detect: disabled (boot only)");
  #endif
#endif

  // --- Networking ---
#if defined(DUAL_NETWORK)
  // ========== DUAL_NETWORK: Initialize both Ethernet and WiFi ==========
  // Uses ESP32 ETH.h library which integrates W5500 with lwIP stack
  Serial.println("Starting DUAL_NETWORK mode (Ethernet + WiFi via lwIP)");

  // Register ETH event handler
  WiFi.onEvent(onEthEvent);

  // --- Read MAC from 24AA02E48 EEPROM on PoE FeatherWing ---
  Wire.begin();
  if (!cfgMacOverride) {
    W5500_mac[0] = readRegister(0xFA);
    W5500_mac[1] = readRegister(0xFB);
    W5500_mac[2] = readRegister(0xFC);
    W5500_mac[3] = readRegister(0xFD);
    W5500_mac[4] = readRegister(0xFE);
    W5500_mac[5] = readRegister(0xFF);
    Serial.print("MAC from EEPROM: "); Serial.println(macToString(W5500_mac));
  } else {
    Serial.print("Using MAC from device.cfg: "); Serial.println(macToString(W5500_mac));
  }

  // --- Ethernet (W5500) Setup via ESP32 ETH.h ---
  Serial.println("Initializing W5500 Ethernet via ETH.h (lwIP)...");

  // Note: Custom MAC from EEPROM read above. ESP32 ETH.h uses the W5500's
  // internal MAC or auto-generates one. For custom MAC, we'd need esp_eth_ioctl().
  // For now, the W5500_mac is read but ETH will use its own MAC assignment.

  // Initialize W5500 using ESP32's ETH library with SPI
  // ETH.begin(type, phy_addr, cs, irq, rst, SPI, spi_freq_mhz)
  if (!ETH.begin(ETH_PHY_W5500, 1, W5500_CS_PIN, W5500_IRQ_PIN, W5500_RST_PIN, SPI)) {
    Serial.println("ERROR: ETH.begin() failed - W5500 not detected!");
  } else {
    // Configure static IP if set, otherwise DHCP is automatic
    if (cfgEthIP != IPAddress(0, 0, 0, 0)) {
      ETH.config(cfgEthIP, cfgEthGateway, cfgEthSubnet, cfgEthDNS);
      Serial.print("Ethernet static IP configured: "); Serial.println(cfgEthIP);
    } else {
      Serial.println("Ethernet using DHCP...");
    }

    // Wait for Ethernet to get IP (with timeout)
    int ethTimeout = 0;
    while (!ethConnected && ethTimeout < 20) {
      delay(500);
      Serial.print(".");
      ethTimeout++;
    }
    Serial.println();
    if (ethConnected) {
      Serial.print("Ethernet IP: "); Serial.println(ETH.localIP());
    } else {
      Serial.println("Ethernet: No IP yet (may still be connecting)");
    }
  }

  // --- WiFi Setup ---
  Serial.println("Initializing ESP32 WiFi...");
  WiFi.mode(WIFI_STA);
  const char* useSSID = (cfgWifiSSID.length() > 0) ? cfgWifiSSID.c_str() : ssid;
  const char* usePass = (cfgWifiPass.length() > 0) ? cfgWifiPass.c_str() : pass;

  if (strlen(useSSID) > 0) {
    Serial.print("Connecting to WiFi (");
    Serial.print(useSSID);
    Serial.print(")");
    WiFi.begin(useSSID, usePass);
    int dots = 0;
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 60) {
      delay(500);
      Serial.print(".");
      if (++dots >= 60) { Serial.println(); dots = 0; }
      attempts++;
    }
    Serial.println();
    if (WiFi.status() == WL_CONNECTED) {
      if (cfgWifiIP != IPAddress(0, 0, 0, 0)) {
        WiFi.config(cfgWifiIP, cfgWifiGateway, cfgWifiSubnet, cfgWifiDNS);
        Serial.print("WiFi static IP: ");
      } else {
        Serial.print("WiFi DHCP IP: ");
      }
      Serial.println(WiFi.localIP());
    } else {
      Serial.println("WiFi connection failed (will retry in background)");
    }
  } else {
    Serial.println("WiFi SSID not configured");
  }

  Serial.println("DUAL_NETWORK ready:");
  Serial.print("  Ethernet: "); Serial.println(ETH.localIP());
  Serial.print("  WiFi:     "); Serial.println(WiFi.localIP());
  Serial.print("  Display:  "); Serial.println(displayPresent ? "DETECTED" : "NOT PRESENT");

#elif defined(W5500)
  Serial.println("Starting W5500 Ethernet");
  Wire.begin();
  // Read MAC from 24AA02E48 EEPROM on PoE FeatherWing
  // (skip if device.cfg already set MAC_ADDRESS)
  if (!cfgMacOverride) {
    W5500_mac[0] = readRegister(0xFA);
    W5500_mac[1] = readRegister(0xFB);
    W5500_mac[2] = readRegister(0xFC);
    W5500_mac[3] = readRegister(0xFD);
    W5500_mac[4] = readRegister(0xFE);
    W5500_mac[5] = readRegister(0xFF);
  } else {
    Serial.print("Using MAC from device.cfg: "); Serial.println(macToString(W5500_mac));
  }

  Ethernet.init(W5500_CS_PIN);
  if (cfgIP != IPAddress(0, 0, 0, 0)) {
    Serial.print("Using static IP from device.cfg: ");
    Ethernet.begin(W5500_mac, cfgIP, cfgDNS, cfgGateway, cfgSubnet);
    Serial.println(Ethernet.localIP());
  } else {
    Serial.println("Requesting IP via DHCP...");
    int dhcpOk = Ethernet.begin(W5500_mac);

    if (Ethernet.hardwareStatus() == EthernetNoHardware) {
      Serial.println("ERROR: Ethernet hardware not found!");
    } else if (Ethernet.linkStatus() == LinkOFF) {
      Serial.println("WARNING: Ethernet cable not connected");
    } else if (dhcpOk) {
      Serial.print("DHCP assigned IP: ");
      Serial.println(Ethernet.localIP());
    } else {
      Serial.println("DHCP failed, falling back to compiled static IP");
      Ethernet.begin(W5500_mac, eth_ip, eth_dns);
      Serial.print("Static IP: ");
      Serial.println(Ethernet.localIP());
    }
  }

#elif defined(WINC1500)
  WiFi.setPins(8, 7, 4, 2);  // Adafruit ATWINC1500 FeatherWing pin config
  if (WiFi.status() == WL_NO_SHIELD) {
    Serial.println("ERROR: WiFi shield not present!");
  } else {
    // Use device.cfg overrides if present, otherwise compile-time secrets
    const char* useSSID = (cfgWifiSSID.length() > 0) ? cfgWifiSSID.c_str() : ssid;
    const char* usePass = (cfgWifiPass.length() > 0) ? cfgWifiPass.c_str() : pass;
    Serial.print("Connecting to WiFi (");
    Serial.print(useSSID);
    Serial.print(")");
    WiFi.begin(useSSID, usePass);
    int dots = 0;
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
      if (++dots >= 60) { Serial.println(); dots = 0; }
    }
    Serial.println();
    if (cfgIP != IPAddress(0, 0, 0, 0)) {
      WiFi.config(cfgIP, cfgDNS, cfgGateway, cfgSubnet);
      Serial.print("Static IP from device.cfg: ");
    } else {
      Serial.print("WiFi connected, IP: ");
    }
    Serial.println(IpAddress2String(WiFi.localIP()));
  }

#elif defined(ESP_WIFI)
  // ESP32 native WiFi
  WiFi.mode(WIFI_STA);
  // Use device.cfg overrides if present, otherwise compile-time secrets
  const char* useSSID = (cfgWifiSSID.length() > 0) ? cfgWifiSSID.c_str() : ssid;
  const char* usePass = (cfgWifiPass.length() > 0) ? cfgWifiPass.c_str() : pass;
  Serial.print("Connecting to WiFi (");
  Serial.print(useSSID);
  Serial.print(")");
  WiFi.begin(useSSID, usePass);
  int dots = 0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    if (++dots >= 60) { Serial.println(); dots = 0; }
  }
  Serial.println();
  if (cfgIP != IPAddress(0, 0, 0, 0)) {
    WiFi.config(cfgIP, cfgGateway, cfgSubnet, cfgDNS);
    Serial.print("Static IP from device.cfg: ");
  } else {
    Serial.print("WiFi connected, IP: ");
  }
  Serial.println(IpAddress2String(WiFi.localIP()));

#else
  Serial.println("No networking hardware configured");
#endif

  server.begin();
#if defined(DUAL_NETWORK)
  Serial.println("Webserver started on port " + String(WEBSERVER_PORT) + " (Ethernet + WiFi via lwIP)");
#else
  Serial.println("Webserver started on port " + String(WEBSERVER_PORT));
#endif

  Serial.println("Setup complete");
}

// ===========================================================================
// loop()
// ===========================================================================

void loop() {

  // --- PT100 RTD reading (once per second, non-blocking) ---
  if (millis() - lastSensorRead >= 1000) {
    lastSensorRead = millis();

    uint16_t ohmsx100, rtd = thermo.readRTD();
    uint32_t dummy;
    float ohms, Tlut;

    // Use uint16_t (ohms * 100) since it matches data type in lookup table.
    dummy = ((uint32_t)(rtd << 1)) * 100 * ((uint32_t) floor(RREF));
    dummy >>= 16;
    ohmsx100 = (uint16_t)(dummy & 0xFFFF);
    ohms = (float)(ohmsx100 / 100) + ((float)(ohmsx100 % 100) / 100.0);
    Tlut = PT100lt.celsius(ohmsx100);
    curIndoorTemp = C2F(Tlut);

#if DEBUG_PT100
    // Verbose diagnostics -- enable DEBUG_PT100 for sensor bring-up.
    // Prints every loop() iteration (~1 Hz) so disable for normal use.
    float ratio = (float)rtd / 32768.0;
    Serial.println();
    Serial.print("RTD value: "); Serial.println(rtd);
    Serial.print("Lower Thresh: "); Serial.println(thermo.getLowerThreshold());
    Serial.print("Upper Thresh: "); Serial.println(thermo.getUpperThreshold());
    Serial.print("Ratio = "); Serial.println(ratio, 8);
    Serial.print("Resistance = "); Serial.println(RREF * ratio, 8);
    Serial.print("rtd: 0x"); Serial.print(rtd, HEX);
    Serial.print(", ohms: "); Serial.println(ohms, 2);
    Serial.print("Tlut   = "); Serial.print(C2F(Tlut));   Serial.println(" F (exact)");
    // DEBUG: Alternate PT100 conversion algorithms for comparison.
    // Uncomment to compare lookup table vs polynomial approximations.
    // float Tcvd   = PT100lt.celsius_cvd(ohms);
    // float Tcube  = PT100lt.celsius_cubic(ohms);
    // float Tpoly  = PT100lt.celsius_polynomial(ohms);
    // float Trpoly = PT100lt.celsius_rationalpolynomial(ohms);
    // Serial.print("Tcvd   = "); Serial.print(C2F(Tcvd), 3);  Serial.println(" F");
    // Serial.print("Tcube  = "); Serial.print(C2F(Tcube), 3);  Serial.println(" F");
    // Serial.print("Tpoly  = "); Serial.print(C2F(Tpoly), 3);  Serial.println(" F");
    // Serial.print("Trpoly = "); Serial.print(C2F(Trpoly), 3); Serial.println(" F");
    Serial.println();
#endif

    // Check and print any faults
    uint8_t fault = thermo.readFault();
    if (fault) {
      Serial.print("Fault 0x"); Serial.println(fault, HEX);
      if (fault & MAX31865_FAULT_HIGHTHRESH)  Serial.println("RTD High Threshold");
      if (fault & MAX31865_FAULT_LOWTHRESH)   Serial.println("RTD Low Threshold");
      if (fault & MAX31865_FAULT_REFINLOW)    Serial.println("REFIN- > 0.85 x Bias");
      if (fault & MAX31865_FAULT_REFINHIGH)   Serial.println("REFIN- < 0.85 x Bias - FORCE- open");
      if (fault & MAX31865_FAULT_RTDINLOW)    Serial.println("RTDIN- < 0.85 x Bias - FORCE- open");
      if (fault & MAX31865_FAULT_OVUV)        Serial.println("Under/Over voltage");
      thermo.clearFault();
      Serial.print("Temperature = "); Serial.println(curIndoorTemp);
      Serial.println();
    }

    // --- BME environmental reading (if detected) ---
    // BME power control: use PT100 temp to gate BME operation in extreme cold
    #if BME_POWER_PIN >= 0
      if (bme680Present || bme280Present) {
        if (curIndoorTemp < BME_MIN_TEMP_F) {
          // Too cold - power off BME to protect it
          if (bmePoweredOn) {
            digitalWrite(BME_POWER_PIN, LOW);
            bmePoweredOn = false;
            Serial.println("BME powered OFF (extreme cold protection)");
          }
        } else {
          // Safe temperature - power on BME if needed
          if (!bmePoweredOn) {
            digitalWrite(BME_POWER_PIN, HIGH);
            delay(100);  // Allow sensor to stabilize
            // Re-initialize sensor after power-up
            if (bme680Present) {
              bme680.begin();
              bme680.setTemperatureOversampling(BME680_OS_8X);
              bme680.setHumidityOversampling(BME680_OS_2X);
              bme680.setPressureOversampling(BME680_OS_4X);
              bme680.setIIRFilterSize(BME680_FILTER_SIZE_3);
              bme680.setGasHeater(0, 150);
            } else if (bme280Present) {
              bme280.begin();
            }
            bmePoweredOn = true;
            Serial.println("BME powered ON (temperature safe)");
          }
        }
      }
    #endif

    if (bmePoweredOn && bme680Present) {
      if (!bme680.performReading()) {
        Serial.println("Failed to perform BME680/688 reading :(");
      } else {
        // BME can override PT100 temp if desired, or just provide env data
        // curIndoorTemp = (bme680.temperature * 9.0 / 5.0) + 32.0;
      }
    } else if (bmePoweredOn && bme280Present) {
      // BME280 reads directly, no performReading() needed
      // curIndoorTemp = (bme280.readTemperature() * 9.0 / 5.0) + 32.0;
    }

    // Record pressure for trend tracking (every 15 minutes)
    if ((bmePoweredOn && (bme680Present || bme280Present)) &&
        (millis() - lastPressureRecord >= PRESSURE_INTERVAL_MS)) {
      float pressInHg;
      if (bme680Present) {
        pressInHg = bme680.pressure / 100.0 * 0.02953;
      } else {
        pressInHg = bme280.readPressure() / 100.0 * 0.02953;
      }
      recordPressure(pressInHg);
    }

    // --- Poll outdoor unit for exterior sensor data (if configured) ---
    pollOutdoor();

    // --- SD card hot-swap detection (if enabled) ---
    reprobeSDCard();
  } // end sensor reading block (1 Hz)

  webserver();

  // Display and touch handling (only if display present)
  if (displayPresent) {
    if (settingsScreen == SETTINGS_OFF) {
      LayoutDisplay();
    } else {
      drawSettingsScreen();
    }
    if (headerPrinted) {
      headerPrinted = false;
    }

    if (ts.touched()) {
      TS_Point p = ts.getPoint();
      // Rotate coordinate system to match landscape TFT orientation
      p.x = map(p.x, 0, 240, 240, 0);
      p.y = map(p.y, 0, 320, 320, 0);
      int y = tft.height() - p.x;
      int x = p.y;

      if (settingsScreen == SETTINGS_OFF) {
        // Light button (minus) - decrease target temp
        if ((x > coordTouchLight[0]) && (x < (coordTouchLight[0] + coordTouchLight[2]))) {
          if ((y > coordTouchLight[1]) && (y <= (coordTouchLight[1] + coordTouchLight[3]))) {
            Serial.print("Light button hit ");
            if (isHeatOn) targIndoorTemp--;
            lastChangeWasUser = true;   // user adjustment -- 5s debounce
            lastHeatChange = millis();  // restart hold timer from now
            lastStateChange = millis(); // Notify web interface
            Serial.println(targIndoorTemp);
          }
        }
        // System button (plus) - increase target temp
        if ((x > coordTouchSystem[0]) && (x < (coordTouchSystem[0] + coordTouchSystem[2]))) {
          if ((y > coordTouchSystem[1]) && (y <= (coordTouchSystem[1] + coordTouchSystem[3]))) {
            Serial.print("System button hit ");
            if (isHeatOn) targIndoorTemp++;
            lastChangeWasUser = true;   // user adjustment -- 5s debounce
            lastHeatChange = millis();  // restart hold timer from now
            lastStateChange = millis(); // Notify web interface
            Serial.println(targIndoorTemp);
          }
        }
        // Hidden button - Toggle heating on/off
        if ((x > coordTouchSystem[0]) && (x < (coordTouchSystem[0] + coordTouchSystem[2]))) {
          if (y < coordTouchSystem[1]) {
            isHeatOn = !isHeatOn;
            lastStateChange = millis(); // Notify web interface
            Serial.print("Heating status toggled ");
            Serial.println(isHeatOn);
          }
        }
      }
      // Hidden button - Settings toggle
      if ((x > coordTouchLight[0]) && (x < (coordTouchLight[0] + coordTouchLight[2]))) {
        if (y < coordTouchSystem[1]) {
          Serial.print("Settings button hit ");
          tft.fillScreen(TFTbackgroundColor);
          settingsDrawn = false;
#if defined(DUAL_NETWORK)
          // 3-state cycle: OFF -> LAN -> WiFi -> OFF
          if (settingsScreen == SETTINGS_OFF) {
            settingsScreen = SETTINGS_LAN;
            Serial.println("-> LAN overlay");
          } else if (settingsScreen == SETTINGS_LAN) {
            settingsScreen = SETTINGS_WIFI;
            Serial.println("-> WiFi overlay");
          } else {
            settingsScreen = SETTINGS_OFF;
            Serial.println("-> Dismiss");
          }
#elif defined(HAS_WIFI)
          // WiFi-only: toggle OFF <-> WiFi
          settingsScreen = (settingsScreen == SETTINGS_OFF) ? SETTINGS_WIFI : SETTINGS_OFF;
          Serial.println(settingsScreen == SETTINGS_WIFI ? "-> WiFi overlay" : "-> Dismiss");
#else
          // LAN-only (W5500, etc): toggle OFF <-> LAN
          settingsScreen = (settingsScreen == SETTINGS_OFF) ? SETTINGS_LAN : SETTINGS_OFF;
          Serial.println(settingsScreen == SETTINGS_LAN ? "-> LAN overlay" : "-> Dismiss");
#endif
          lastStateChange = millis();  // Notify web interface of state change
        }
      }
    }
  }
}

// ===========================================================================
// WiFi Scan Cache (for /api/scan and {foreach $WIFI_NETWORKS})
// ===========================================================================

#if defined(HAS_WIFI)
struct WifiScanResult {
  String ssid;
  int rssi;
  String enc;
};
WifiScanResult tplScanCache[TPL_MAX_FOREACH_ITEMS];
int tplScanCount = 0;
bool tplScanDone = false;  // Has a scan been performed for this template?

// Scan WiFi networks into tplScanCache[], sorted by RSSI descending.
void wifiScanSorted() {
  tplScanCount = 0;
  int n = WiFi.scanNetworks();
  for (int i = 0; i < n && tplScanCount < TPL_MAX_FOREACH_ITEMS; i++) {
    tplScanCache[tplScanCount].ssid = WiFi.SSID(i);
    tplScanCache[tplScanCount].rssi = WiFi.RSSI(i);
    tplScanCache[tplScanCount].enc = encTypeStr(WiFi.encryptionType(i));
    tplScanCount++;
  }
  // Insertion sort by RSSI descending (strongest first)
  for (int i = 1; i < tplScanCount; i++) {
    WifiScanResult key = tplScanCache[i];
    int j = i - 1;
    while (j >= 0 && tplScanCache[j].rssi < key.rssi) {
      tplScanCache[j + 1] = tplScanCache[j];
      j--;
    }
    tplScanCache[j + 1] = key;
  }
}
#else
// Stub for non-WiFi builds (W5500 Ethernet doesn't have scan)
int tplScanCount = 0;
#endif

// ===========================================================================
// Web Server
// ===========================================================================

void webserver() {
  HardwareClient client = SERVER_ACCEPT();
  if (!client) return;

  // Detect which interface the client connected via
#if defined(DUAL_NETWORK)
  // With ESP32 ETH.h + WiFi via lwIP, detect interface from client's local IP
  IPAddress clientLocalIP = client.localIP();
  if (clientLocalIP == ETH.localIP()) {
    currentClientInterface = "eth";
  } else if (clientLocalIP == WiFi.localIP()) {
    currentClientInterface = "wifi";
  } else {
    currentClientInterface = "unkn";
  }
  Serial.print("Client connected via "); Serial.println(currentClientInterface);
#elif defined(W5500)
  currentClientInterface = "eth";
  Serial.println("Client connected");
#elif defined(HAS_WIFI)
  currentClientInterface = "wifi";
  Serial.println("Client connected");
#else
  currentClientInterface = "unkn";
  Serial.println("Client connected");
#endif

  while (!client.available()) {
    delay(1);
  }

  // Read the first line of the HTTP request
  String req = client.readStringUntil('\r');
  client.read(); // consume the \n after \r
  Serial.println(req);

  // Read remaining headers to extract Content-Length and boundary
  long contentLength = 0;
  String boundary = "";
  unsigned long headerTimeout = millis() + 5000;
  int emptyCount = 0;
  while (millis() < headerTimeout) {
    if (!client.available()) {
      delay(1);
      emptyCount++;
      if (emptyCount > 100) break;  // Give up after 100ms of no data
      continue;
    }
    emptyCount = 0;
    String line = client.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) break;  // End of headers (blank line)

    if (line.startsWith("Content-Length:")) {
      contentLength = line.substring(15).toInt();
    }
    if (line.indexOf("boundary=") > 0) {
      int idx = line.indexOf("boundary=") + 9;
      boundary = line.substring(idx);
      boundary.trim();
    }
  }

  if (req.indexOf("/") != -1) {
    int startIndex = req.indexOf("/");
    int endIndex = req.substring(startIndex).indexOf(' ') + startIndex;
    if (endIndex == -1) return;

    String currentPage = req.substring(startIndex, endIndex);
    if (currentPage == "/") currentPage += "index.html";
    Serial.println("CurrentPage: " + currentPage);
    Serial.print("Content-Length: "); Serial.println(contentLength);

    // --- Route: /favicon.ico ---
    if (currentPage == "/favicon.ico") {
      // Try SD card first (auto-generated at boot or user-provided)
      if (!serveSDFile(client, currentPage)) {
        // Fall back to PROGMEM-embedded favicon
        client.println("HTTP/1.1 200 OK");
        client.println("Content-Type: image/x-icon");
        client.println("Connection: close");
        client.println();
        const byte bufferSize = 70;
        uint8_t buffer[bufferSize];
        const size_t n = sizeof favicon_ico / bufferSize;
        const size_t r = sizeof favicon_ico % bufferSize;
        for (size_t i = 0; i < sizeof favicon_ico; i += bufferSize) {
          memcpy_P(buffer, favicon_ico + i, bufferSize);
          client.write(buffer, bufferSize);
        }
        if (r != 0) {
          memcpy_P(buffer, favicon_ico + n * bufferSize, r);
          client.write(buffer, r);
        }
      }

    // --- Route: /index.html (main page) ---
    } else if (currentPage == "/index.html") {
      sendHtmlHeaders(client);
      // SD card index.html overrides firmware-embedded page
      if (sdCardPresent && sdIndexPresent) {
        sendPageFromSD(client);
      } else {
        sendPage(client);
      }

    // --- Route: /api/state (full JSON state) ---
    } else if (currentPage == "/api/state") {
      client.println("HTTP/1.1 200 OK");
      client.println("Content-Type: application/json");
      client.println("Access-Control-Allow-Origin: *");
      client.println("Cache-Control: no-cache");
      client.println("Connection: close");
      client.println();
      sendApiState(client);

    // --- Route: /api/poll (lightweight polling - just essential data) ---
    } else if (currentPage == "/api/poll") {
      client.println("HTTP/1.1 200 OK");
      client.println("Content-Type: application/json");
      client.println("Access-Control-Allow-Origin: *");
      client.println("Cache-Control: no-cache");
      client.println("Connection: close");
      client.println();
      client.print("{\"t\":");
      client.print(myround(curIndoorTemp));
      client.print(",\"g\":");
      client.print(targIndoorTemp);
      client.print(",\"h\":");
      client.print(isHeatOn ? "1" : "0");
      client.print(",\"a\":");
      client.print(isCurHeatOn ? "1" : "0");
      client.print(",\"s\":\"");
      const char* ss[] = {"off", "lan", "wifi"};
      client.print(ss[settingsScreen]);
      client.print("\",\"c\":");
      client.print(lastStateChange);
      client.println("}");

    // --- Route: /api/network (network info - fetch on demand) ---
    } else if (currentPage == "/api/network") {
      client.println("HTTP/1.1 200 OK");
      client.println("Content-Type: application/json");
      client.println("Access-Control-Allow-Origin: *");
      client.println("Cache-Control: no-cache");
      client.println("Connection: close");
      client.println();
      client.print("{");
#if defined(DUAL_NETWORK)
      client.print("\"mode\":\"dual\",\"ethernet\":{");
      client.print("\"ip\":\""); client.print(IpAddress2String(Ethernet.localIP()));
      client.print("\",\"subnet\":\""); client.print(IpAddress2String(Ethernet.subnetMask()));
      client.print("\",\"gateway\":\""); client.print(IpAddress2String(Ethernet.gatewayIP()));
      client.print("\",\"dns\":\""); client.print(IpAddress2String(Ethernet.dnsServerIP()));
      client.print("\",\"mac\":\""); client.print(macToString(W5500_mac));
      client.print("\",\"link\":\""); client.print((Ethernet.linkStatus() == LinkON) ? "Connected" : "Disconnected");
      client.print("\"},\"wifi\":{");
      client.print("\"ip\":\""); client.print(IpAddress2String(WiFi.localIP()));
      client.print("\",\"subnet\":\""); client.print(IpAddress2String(WiFi.subnetMask()));
      client.print("\",\"gateway\":\""); client.print(IpAddress2String(WiFi.gatewayIP()));
      byte wMac[6]; WiFi.macAddress(wMac);
      client.print("\",\"mac\":\""); client.print(macToString(wMac));
      client.print("\",\"link\":\""); client.print((WiFi.status() == WL_CONNECTED) ? "Connected" : "Disconnected");
      client.print("\",\"ssid\":\""); client.print(WiFi.SSID());
      client.print("\",\"security\":\""); client.print(getCurrentWifiSecurity());
      client.print("\",\"rssi\":"); client.print(WiFi.RSSI());
      client.print("}");
#if defined(HAS_BATT)
      client.print(",\"battery\":"); client.print(String(readBatteryVoltage(), 2));
#endif
#elif defined(W5500)
      client.print("\"mode\":\"ethernet\",\"ip\":\""); client.print(IpAddress2String(Ethernet.localIP()));
      client.print("\",\"subnet\":\""); client.print(IpAddress2String(Ethernet.subnetMask()));
      client.print("\",\"gateway\":\""); client.print(IpAddress2String(Ethernet.gatewayIP()));
      client.print("\",\"mac\":\""); client.print(macToString(W5500_mac));
      client.print("\",\"link\":\""); client.print((Ethernet.linkStatus() == LinkON) ? "Connected" : "Disconnected");
      client.print("\"");
#elif defined(HAS_WIFI)
      client.print("\"mode\":\"wifi\",\"ip\":\""); client.print(IpAddress2String(WiFi.localIP()));
      client.print("\",\"subnet\":\""); client.print(IpAddress2String(WiFi.subnetMask()));
      client.print("\",\"gateway\":\""); client.print(IpAddress2String(WiFi.gatewayIP()));
      byte wMac[6]; WiFi.macAddress(wMac);
      client.print("\",\"mac\":\""); client.print(macToString(wMac));
      client.print("\",\"link\":\""); client.print((WiFi.status() == WL_CONNECTED) ? "Connected" : "Disconnected");
      client.print("\",\"ssid\":\""); client.print(WiFi.SSID());
      client.print("\",\"security\":\""); client.print(getCurrentWifiSecurity());
      client.print("\",\"rssi\":"); client.print(WiFi.RSSI());
#if defined(HAS_BATT)
      client.print(",\"battery\":"); client.print(String(readBatteryVoltage(), 2));
#endif
#endif
      client.println("}");

    // --- Route: /api/set (control commands) ---
    } else if (currentPage.startsWith("/api/set")) {
      // Parse commands: /api/set?target=72&heat=on&settings=toggle
      bool success = false;

      if (currentPage.indexOf("target=") > 0) {
        int idx = currentPage.indexOf("target=") + 7;
        int endIdx = currentPage.indexOf("&", idx);
        if (endIdx < 0) endIdx = currentPage.length();
        String val = currentPage.substring(idx, endIdx);
        targIndoorTemp = val.toInt();
        lastChangeWasUser = true;   // user adjustment -- 5s debounce
        lastHeatChange = millis();  // restart hold timer from now
        lastStateChange = millis();
        success = true;
      }

      if (currentPage.indexOf("heat=") > 0) {
        int idx = currentPage.indexOf("heat=") + 5;
        String val = currentPage.substring(idx, idx + 2);
        isHeatOn = (val == "on" || val == "1");
        lastStateChange = millis();
        success = true;
      }

      if (currentPage.indexOf("settings=") > 0) {
        int idx = currentPage.indexOf("settings=") + 9;
        String val = currentPage.substring(idx, idx + 6);
        if (displayPresent) tft.fillScreen(TFTbackgroundColor);
        settingsDrawn = false;

        if (val.startsWith("toggle")) {
#if defined(DUAL_NETWORK)
          // 3-state cycle: OFF -> LAN -> WiFi -> OFF
          if (settingsScreen == SETTINGS_OFF) {
            settingsScreen = SETTINGS_LAN;
          } else if (settingsScreen == SETTINGS_LAN) {
            settingsScreen = SETTINGS_WIFI;
          } else {
            settingsScreen = SETTINGS_OFF;
          }
#elif defined(HAS_WIFI)
          // WiFi-only: toggle OFF <-> WiFi
          settingsScreen = (settingsScreen == SETTINGS_OFF) ? SETTINGS_WIFI : SETTINGS_OFF;
#else
          // LAN-only: toggle OFF <-> LAN
          settingsScreen = (settingsScreen == SETTINGS_OFF) ? SETTINGS_LAN : SETTINGS_OFF;
#endif
        } else if (val.startsWith("on") || val == "1" || val.startsWith("lan")) {
          settingsScreen = SETTINGS_LAN;
        } else if (val.startsWith("wifi")) {
          settingsScreen = SETTINGS_WIFI;
        } else {
          // "off" or any other value
          settingsScreen = SETTINGS_OFF;
        }
        lastStateChange = millis();
        success = true;
      }

      sendJsonHeaders(client);
      client.print("{\"success\":");
      client.print(success ? "true" : "false");
      client.println("}");

    // --- Route: /outdoorjson (this unit's sensors as OUTDOOR_*) ---
    } else if (currentPage == "/outdoorjson" ||
               currentPage == "/indoorjson") {
      // /outdoorjson: this unit's sensors with OUTDOOR_ prefix
      //              (for an indoor master to consume as exterior readings)
      // /indoorjson:  this unit's sensors with INDOOR_ prefix
      //              (raw data dump for diagnostics / integration)
      sendJsonHeaders(client);
      bool asOutdoor = (currentPage == "/outdoorjson");
      sendSensorJson(client, asOutdoor);

    // --- Route: /config (network + outdoor configuration) ---
    } else if (currentPage.startsWith("/config")) {
      sendHtmlHeaders(client);
      sendConfigPage(client, currentPage);

    // --- Route: /update (OTA firmware update page) ---
    } else if (currentPage == "/update") {
      sendHtmlHeaders(client);
      sendUpdatePage(client);

    // --- Route: POST /api/update (OTA firmware upload) ---
    } else if (currentPage == "/api/update" && req.startsWith("POST")) {
      handleFirmwareUpload(client, contentLength);

    // --- CORS preflight for POST endpoints ---
    } else if (req.startsWith("OPTIONS") &&
               (currentPage == "/api/config" || currentPage == "/api/update" ||
                currentPage == "/api/sdupload" || currentPage == "/api/sddelete" ||
                currentPage == "/api/sdwrite" || currentPage == "/api/sdformat" ||
                currentPage == "/api/sdfileupload")) {
      client.println("HTTP/1.1 204 No Content");
      client.println("Access-Control-Allow-Origin: *");
      client.println("Access-Control-Allow-Methods: POST, OPTIONS");
      client.println("Access-Control-Allow-Headers: Content-Type");
      client.println("Access-Control-Max-Age: 86400");
      client.println("Connection: close");
      client.println();

    // --- Route: POST /api/sdupload (SD card firmware upload) ---
    } else if (currentPage == "/api/sdupload" && req.startsWith("POST")) {
      handleSDUpload(client, contentLength);

    // --- Route: /api/reboot (software reset) ---
    } else if (currentPage == "/api/reboot") {
      sendJsonHeaders(client);
      client.println("{\"success\":true,\"message\":\"Rebooting...\"}");
      CLIENT_CLEAR(client);
      delay(100);
      client.stop();
      Serial.println(F("Rebooting via /api/reboot..."));
      delay(500);
#if defined(ARDUINO_ARCH_SAMD)
      NVIC_SystemReset();
#elif defined(ARDUINO_ARCH_RP2040)
      rp2040.reboot();
#elif defined(ARDUINO_ARCH_ESP32)
      ESP.restart();
#else
      // AVR/other: watchdog reset
      void (*resetFunc)(void) = 0;
      resetFunc();
#endif

    // --- Route: POST /api/config (JSON configuration) ---
    } else if (currentPage == "/api/config" && req.startsWith("POST")) {
      // Read JSON body
      char jsonBuf[512];
      int toRead = min(contentLength, (long)sizeof(jsonBuf) - 1);
      int got = 0;
      unsigned long timeout = millis() + 5000;
      while (got < toRead && millis() < timeout) {
        if (client.available()) {
          jsonBuf[got++] = client.read();
        }
      }
      jsonBuf[got] = '\0';

      // Parse JSON with ArduinoJson
#if ARDUINOJSON_VERSION_MAJOR >= 7
      JSON_DOC doc;
#else
      JSON_DOC doc(512);
#endif
      DeserializationError err = deserializeJson(doc, jsonBuf);

      sendJsonHeaders(client);

      if (err) {
        client.print("{\"success\":false,\"error\":\"");
        client.print(err.c_str());
        client.println("\"}");
      } else {
        String action = doc["action"] | "";
        bool ok = false;

        if (action == "network") {
          // Save network settings - uses eth_* for Ethernet, wifi_* for WiFi
#if defined(W5500) && !defined(DUAL_NETWORK)
          // Ethernet-only mode: eth_* keys map to cfgStatic*
          const char* sip = doc["eth_cfg_ip"];
          const char* sdns = doc["eth_cfg_dns"];
          const char* ssub = doc["eth_cfg_subnet"];
          const char* sgw = doc["eth_cfg_gateway"];
          if (sip && String(sip).length() > 0 && String(sip) != "DHCP") cfgIP.fromString(sip);
          else cfgIP = IPAddress(0, 0, 0, 0);
          if (sdns && String(sdns).length() > 0) cfgDNS.fromString(sdns);
          if (ssub && String(ssub).length() > 0) cfgSubnet.fromString(ssub);
          if (sgw && String(sgw).length() > 0) cfgGateway.fromString(sgw);
#elif defined(HAS_WIFI) && !defined(DUAL_NETWORK)
          // WiFi-only mode: wifi_* keys map to cfgStatic* and cfgWifi*
          const char* sip = doc["wifi_cfg_ip"];
          const char* sdns = doc["wifi_cfg_dns"];
          const char* ssub = doc["wifi_cfg_subnet"];
          const char* sgw = doc["wifi_cfg_gateway"];
          if (sip && String(sip).length() > 0 && String(sip) != "DHCP") cfgIP.fromString(sip);
          else cfgIP = IPAddress(0, 0, 0, 0);
          if (sdns && String(sdns).length() > 0) cfgDNS.fromString(sdns);
          if (ssub && String(ssub).length() > 0) cfgSubnet.fromString(ssub);
          if (sgw && String(sgw).length() > 0) cfgGateway.fromString(sgw);
          const char* wssid = doc["wifi_ssid"];
          const char* wpass = doc["wifi_pass"];
          if (wssid) cfgWifiSSID = String(wssid);
          if (wpass) cfgWifiPass = String(wpass);
#endif
          saveDeviceConfig();
          Serial.println("Network config updated via /api/config");
          ok = true;

#if defined(DUAL_NETWORK)
        } else if (action == "dual_network") {
          // DUAL_NETWORK: Save both Ethernet and WiFi settings
          // Detect which interface the client is connected via
          IPAddress clientLocalIP = client.localIP();
          bool clientIsEthernet = (clientLocalIP == Ethernet.localIP());
          bool clientIsWiFi = (clientLocalIP == WiFi.localIP());
          
          // Parse Ethernet settings
          const char* ethSip = doc["eth_cfg_ip"];
          const char* ethDns = doc["eth_cfg_dns"];
          const char* ethSub = doc["eth_cfg_subnet"];
          const char* ethGw = doc["eth_cfg_gateway"];
          if (ethSip && String(ethSip).length() > 0 && String(ethSip) != "DHCP") 
            cfgEthIP.fromString(ethSip);
          else cfgEthIP = IPAddress(0, 0, 0, 0);
          if (ethDns && String(ethDns).length() > 0) cfgEthDNS.fromString(ethDns);
          if (ethSub && String(ethSub).length() > 0) cfgEthSubnet.fromString(ethSub);
          if (ethGw && String(ethGw).length() > 0) cfgEthGateway.fromString(ethGw);
          
          // Parse WiFi settings
          const char* wssid = doc["wifi_ssid"];
          const char* wpass = doc["wifi_pass"];
          const char* wifiSip = doc["wifi_cfg_ip"];
          const char* wifiDns = doc["wifi_cfg_dns"];
          const char* wifiSub = doc["wifi_cfg_subnet"];
          const char* wifiGw = doc["wifi_cfg_gateway"];
          if (wssid) cfgWifiSSID = String(wssid);
          if (wpass) cfgWifiPass = String(wpass);
          if (wifiSip && String(wifiSip).length() > 0 && String(wifiSip) != "DHCP")
            cfgWifiIP.fromString(wifiSip);
          else cfgWifiIP = IPAddress(0, 0, 0, 0);
          if (wifiDns && String(wifiDns).length() > 0) cfgWifiDNS.fromString(wifiDns);
          if (wifiSub && String(wifiSub).length() > 0) cfgWifiSubnet.fromString(wifiSub);
          if (wifiGw && String(wifiGw).length() > 0) cfgWifiGateway.fromString(wifiGw);
          
          // Save to device.cfg first
          saveDeviceConfig();
          
          // Apply changes to non-client interface immediately
          String applied = "";
          if (clientIsEthernet) {
            // Client is on Ethernet - safe to reconfigure WiFi now
            Serial.println("Applying WiFi config (client on Ethernet)");
            WiFi.disconnect();
            delay(100);
            const char* useSSID = (cfgWifiSSID.length() > 0) ? cfgWifiSSID.c_str() : ssid;
            const char* usePass = (cfgWifiPass.length() > 0) ? cfgWifiPass.c_str() : pass;
            WiFi.begin(useSSID, usePass);
            if (cfgWifiIP != IPAddress(0, 0, 0, 0)) {
              WiFi.config(cfgWifiIP, cfgWifiGateway, cfgWifiSubnet, cfgWifiDNS);
            }
            applied = "wifi";
          } else if (clientIsWiFi) {
            // Client is on WiFi - safe to reconfigure Ethernet now
            Serial.println("Applying Ethernet config (client on WiFi)");
            if (cfgEthIP != IPAddress(0, 0, 0, 0)) {
              // ETH.h uses config() to change IP settings (not begin())
              ETH.config(cfgEthIP, cfgEthGateway, cfgEthSubnet, cfgEthDNS);
            }
            // Note: For DHCP, ETH.h doesn't have a direct "restart DHCP" method
            // The interface was already configured at boot; user may need to reboot for DHCP
            applied = "ethernet";
          } else {
            applied = "none";
          }
          
          Serial.println("Dual network config updated via /api/config");
          
          // Send response with info about what was applied
          client.print("{\"success\":true,\"applied\":\"");
          client.print(applied);
          client.print("\",\"client_interface\":\"");
          client.print(clientIsEthernet ? "ethernet" : (clientIsWiFi ? "wifi" : "unknown"));
          client.println("\"}");
          return;  // Already sent response
#endif

        } else if (action == "outdoor") {
          // Save outdoor unit settings
          const char* ipStr = doc["ip"];
          int port = doc["port"] | 0;
          int poll = doc["poll"] | 0;

          if (ipStr && String(ipStr).length() > 0) {
            IPAddress parsed;
            if (parsed.fromString(ipStr)) {
              outdoorIP = parsed;
            }
          }
          if (port > 0) {
            outdoorPort = port;
          }
          if (poll > 0) {
            outdoorPollSecs = poll;
            if (outdoorPollSecs < 5) outdoorPollSecs = 5;
            if (outdoorPollSecs > 3600) outdoorPollSecs = 3600;
          }
          outdoorConfigured = (outdoorIP[0] != 0 || outdoorIP[1] != 0 ||
                               outdoorIP[2] != 0 || outdoorIP[3] != 0);
          outdoorLastPoll = 0;
          saveDeviceConfig();
          Serial.println("Outdoor config updated via /api/config");
          ok = true;

        } else if (action == "clear_outdoor") {
          // Clear outdoor config
          clearOutdoorConfig();
          saveDeviceConfig();
          Serial.println("Outdoor config cleared via /api/config");
          ok = true;

        } else {
          client.println("{\"success\":false,\"error\":\"unknown action\"}");
          return;
        }

        client.print("{\"success\":");
        client.print(ok ? "true" : "false");
        client.println("}");
      }

#if defined(HAS_WIFI)
    // --- Route: /api/scan (WiFi network scan) ---
    } else if (currentPage == "/api/scan") {
      sendJsonHeaders(client);
      wifiScanSorted();
      client.print("{\"maxNetworks\":");
      client.print(TPL_MAX_FOREACH_ITEMS);
      client.print(",\"count\":");
      client.print(tplScanCount);
      client.print(",\"networks\":[");
      for (int i = 0; i < tplScanCount; i++) {
        if (i > 0) client.print(",");
        client.print("{\"ssid\":\"");
        client.print(tplScanCache[i].ssid);
        client.print("\",\"rssi\":");
        client.print(tplScanCache[i].rssi);
        client.print(",\"enc\":\"");
        client.print(tplScanCache[i].enc);
        client.print("\"}");
      }
      client.println("]}");
#endif

    // --- Route: /sdfiles (SD card file manager UI) ---
    } else if (currentPage == "/sdfiles") {
      sendSDFilesPage(client);

    // --- Route: /api/sdlist (list SD card directory) ---
    } else if (currentPage.startsWith("/api/sdlist")) {
      handleSDList(client, currentPage);

    // --- Route: /api/sdread (read file for editor) ---
    } else if (currentPage.startsWith("/api/sdread")) {
      handleSDRead(client, currentPage);

    // --- Route: /api/sddownload (download file) ---
    } else if (currentPage.startsWith("/api/sddownload")) {
      handleSDDownload(client, currentPage);

    // --- Route: POST /api/sddelete (delete file/dir) ---
    } else if (currentPage == "/api/sddelete" && req.startsWith("POST")) {
      handleSDDelete(client, contentLength);

    // --- Route: POST /api/sdwrite (save file from editor) ---
    } else if (currentPage == "/api/sdwrite" && req.startsWith("POST")) {
      handleSDWrite(client, contentLength);

    // --- Route: POST /api/sdformat (format SD card) ---
    } else if (currentPage == "/api/sdformat" && req.startsWith("POST")) {
      handleSDFormat(client, contentLength);

    // --- Route: POST /api/sdfileupload (upload file to SD) ---
    } else if (currentPage == "/api/sdfileupload" && req.startsWith("POST")) {
      handleSDFileUpload(client, contentLength, req);

    // --- Route: SD card catchall / 404 ---
    } else {
      if (!serveSDFile(client, currentPage)) {
        // File not found -- try SD 404.html, else embedded error
        client.println("HTTP/1.1 404 Not Found");
        client.println("Content-Type: text/html");
        client.println("Connection: close");
        client.println();
        if (sdCardPresent) {
          file_t f404;
          if (f404.open("404.html", O_RDONLY)) {
            tplReset();
            char buf[257];
            while (f404.available()) {
              int len = f404.fgets(buf, sizeof(buf));
              if (len <= 0) break;
              String line = String(buf);
              processTemplateLine(line, client);
            }
            f404.close();
            CLIENT_CLEAR(client);
          } else {
            send404Error(client);
          }
        } else {
          send404Error(client);
        }
      }
    }

    delay(1);
    Serial.println("Disconnecting.");
    client.stop();
  }
}

// ===========================================================================
// REST API - JSON State
// ===========================================================================

// Send current state as JSON for the web interface to poll
void sendApiState(HardwareClient cl) {
#if ARDUINOJSON_VERSION_MAJOR >= 7
  JSON_DOC doc;
#else
  JSON_DOC doc(JSON_DOC_SIZE);
#endif

  doc["currentTemp"] = myround(curIndoorTemp);
  doc["targetTemp"] = targIndoorTemp;
  doc["heatEnabled"] = isHeatOn;
  doc["heatActive"] = isCurHeatOn;
  doc["lastHeatChange"] = lastHeatChange;  // millis() of last relay state change
  // Settings overlay: "off", "lan", or "wifi"
  const char* settingsStates[] = {"off", "lan", "wifi"};
  doc["settingsScreen"] = settingsStates[settingsScreen];
  doc["lastChange"] = lastStateChange;
  doc["uptime"] = millis();

  // Temperature color (CSS hex)
  doc["tempColor"] = tempColorCSS();

  // System info
  doc["board"] = curBoard;
  doc["currentTempRaw"] = curIndoorTemp;  // With decimal precision
  doc["sdCard"] = sdCardPresent;
  if (sdCardPresent) {
    doc["sdFormat"] = getSDCardFormat();
  }
  doc["displayPresent"] = displayPresent;
  doc["otaSupported"] = OTA_SUPPORTED ? true : false;
#if defined(ESP32) || defined(ESP8266)
  doc["freeRam"] = ESP.getFreeHeap();
#elif defined(__SAMD51__) || defined(__SAMD21__)
  char stackTop;
  doc["freeRam"] = (uint32_t)&stackTop - (uint32_t)sbrk(0);
#endif

  // Indoor environment data (BME sensor)
  JsonObject env = CREATE_NESTED_OBJ(doc, "environment");
  if (!bmePoweredOn) {
    env["sensor"] = "off";  // Powered off for cold protection
  } else if (bme680Present) {
    float pressInHg = bme680.pressure / 100.0 * 0.02953;
    float humidity = bme680.humidity;
    env["humidity"] = humidity;
    env["pressInhg"] = pressInHg;
    env["dewPoint"] = getDewPointF(curIndoorTemp, humidity);
    env["gasKohms"] = bme680.gas_resistance / 1000.0;
    env["sensor"] = "BME680";
    // Weather prediction using pressure, humidity, and temperature
    env["pressureCategory"] = getPressureCategory(pressInHg);
    String trend = getPressureTrendStr();
    if (trend != "") env["pressureTrend"] = trend;
    env["weatherPrediction"] = getWeatherPrediction(pressInHg, humidity, curIndoorTemp);
  } else if (bme280Present) {
    float pressInHg = bme280.readPressure() / 100.0 * 0.02953;
    float humidity = bme280.readHumidity();
    env["humidity"] = humidity;
    env["pressInhg"] = pressInHg;
    env["dewPoint"] = getDewPointF(curIndoorTemp, humidity);
    env["sensor"] = "BME280";
    // Weather prediction using pressure, humidity, and temperature
    env["pressureCategory"] = getPressureCategory(pressInHg);
    String trend = getPressureTrendStr();
    if (trend != "") env["pressureTrend"] = trend;
    env["weatherPrediction"] = getWeatherPrediction(pressInHg, humidity, curIndoorTemp);
  }

  // Outdoor unit data (from polling)
  JsonObject outdoor = CREATE_NESTED_OBJ(doc, "outdoor");
  outdoor["configured"] = outdoorConfigured;
  outdoor["status"] = outdoorStatus;
  if (outdoorConfigured) {
    outdoor["ip"] = IpAddress2String(outdoorIP);
    outdoor["port"] = outdoorPort;
    outdoor["pollSecs"] = outdoorPollSecs;
  }
  if (outdoorStatus == "Online") {
    outdoor["temp"] = outdoorTemp;
    outdoor["humidity"] = outdoorHumidity;
    outdoor["pressInhg"] = outdoorPressInhg;
    outdoor["gasKohms"] = outdoorGasKohms;
    outdoor["bmeType"] = outdoorBmeType;
    outdoor["board"] = outdoorBoard;
    outdoor["ethIP"] = outdoorEthIP;
    outdoor["wifiIP"] = outdoorWifiIP;
    outdoor["iface"] = outdoorIface;
    outdoor["batVolts"] = outdoorBatteryV;
    outdoor["hasEthernet"] = outdoorHasEth;
    outdoor["hasWifi"] = outdoorHasWifi;
  }

  // Network info
  JsonObject network = CREATE_NESTED_OBJ(doc, "network");
#if defined(DUAL_NETWORK)
  // DUAL_NETWORK: Report both interfaces
  JsonObject eth = CREATE_NESTED_OBJ(network, "ethernet");
  eth["ip"] = IpAddress2String(Ethernet.localIP());
  eth["subnet"] = IpAddress2String(Ethernet.subnetMask());
  eth["gateway"] = IpAddress2String(Ethernet.gatewayIP());
  eth["dns"] = IpAddress2String(Ethernet.dnsServerIP());
  eth["mac"] = macToString(W5500_mac);
  eth["link"] = (Ethernet.linkStatus() == LinkON) ? "Connected" : "Disconnected";

  JsonObject wifi = CREATE_NESTED_OBJ(network, "wifi");
  wifi["ip"] = IpAddress2String(WiFi.localIP());
  wifi["subnet"] = IpAddress2String(WiFi.subnetMask());
  wifi["gateway"] = IpAddress2String(WiFi.gatewayIP());
  byte wMac[6];
  WiFi.macAddress(wMac);
  wifi["mac"] = macToString(wMac);
  wifi["link"] = (WiFi.status() == WL_CONNECTED) ? "Connected" : "Disconnected";
  wifi["ssid"] = WiFi.SSID();
  wifi["rssi"] = WiFi.RSSI();
  wifi["security"] = getCurrentWifiSecurity();
  
#if defined(HAS_BATT)
  network["battery"] = serialized(String(readBatteryVoltage(), 2));
#endif
  network["mode"] = "dual";
#elif defined(W5500)
  network["ip"] = IpAddress2String(Ethernet.localIP());
  network["subnet"] = IpAddress2String(Ethernet.subnetMask());
  network["gateway"] = IpAddress2String(Ethernet.gatewayIP());
  network["mac"] = macToString(W5500_mac);
  network["link"] = (Ethernet.linkStatus() == LinkON) ? "Connected" : "Disconnected";
  network["mode"] = "ethernet";
#elif defined(WINC1500)
  network["ip"] = IpAddress2String(WiFi.localIP());
  network["subnet"] = IpAddress2String(WiFi.subnetMask());
  network["gateway"] = IpAddress2String(WiFi.gatewayIP());
  byte wMac[6];
  WiFi.macAddress(wMac);
  network["mac"] = macToString(wMac);
  network["link"] = (WiFi.status() == WL_CONNECTED) ? "Connected" : "Disconnected";
  network["ssid"] = WiFi.SSID();
  network["security"] = encTypeStr(WiFi.encryptionType());
  network["rssi"] = WiFi.RSSI();
#if defined(HAS_BATT)
  network["battery"] = serialized(String(readBatteryVoltage(), 2));
#endif
  network["mode"] = "wifi";
#elif defined(ESP_WIFI)
  network["ip"] = IpAddress2String(WiFi.localIP());
  network["subnet"] = IpAddress2String(WiFi.subnetMask());
  network["gateway"] = IpAddress2String(WiFi.gatewayIP());
  byte wMac[6];
  WiFi.macAddress(wMac);
  network["mac"] = macToString(wMac);
  network["link"] = (WiFi.status() == WL_CONNECTED) ? "Connected" : "Disconnected";
  network["ssid"] = WiFi.SSID();
  network["rssi"] = WiFi.RSSI();
#if defined(HAS_BATT)
  network["battery"] = serialized(String(readBatteryVoltage(), 2));
#endif
  network["mode"] = "wifi";
#else
  network["ip"] = "N/A";
  network["link"] = "No network";
  network["mode"] = "none";
#endif

  serializeJson(doc, cl);
}

// ===========================================================================
// TFT Display Helpers
// ===========================================================================

void TFTtext(Adafruit_ILI9341 *screen, const uint16_t x, const uint16_t y, const char *str,
  const GFXfont *font = NULL, uint8_t textsize = 1, uint16_t textcolor = ILI9341_BLACK)
{
  screen->setCursor(x, y);
  screen->setFont(font);
  screen->setTextSize(textsize);
  screen->setTextColor(textcolor);
  screen->print(str);
}

void LCDtemp(Adafruit_ILI9341 *screen, uint16_t x, uint16_t y, const char *str,
  const GFXfont *font = NULL, uint8_t textsize = 1, uint16_t textcolor = ILI9341_BLACK,
  bool rightJustify = false)
{
  uint16_t xcircle, ycircle;
  int16_t  x1, y1;
  uint16_t w, h;
  const uint16_t radius = 5;

  screen->setCursor(x, y);
  screen->setFont(font);
  screen->setTextSize(textsize);
  screen->getTextBounds(str, 0, y, &x1, &y1, &w, &h);
  xcircle = x + w + 15;
  ycircle = y1 + 7;
  if (rightJustify) {
    x = coordTouchSystem[0] + 40 - (w - 88);
    xcircle = 313;
  }
  TFTtext(screen, x, y, str, font, textsize, textcolor);
  screen->drawCircle(xcircle, ycircle, radius, textcolor);
  screen->drawCircle(xcircle, ycircle, radius - 1, textcolor);
}

// ===========================================================================
// LayoutDisplay - draws the main thermostat UI on the TFT
// ===========================================================================

unsigned long LayoutDisplay() {
  if (!displayPresent) return 0;

  unsigned long start = micros();
  int16_t  x1, y1;
  uint16_t w, h;
  const uint16_t radius = 5;
  char insidetemp[4] = "";
  uint16_t curIndoorTempColor = ILI9341_GREEN;

  tft.setFont();
  // Determine if heating should be active.
  // Two modes of comparison:
  //   User just adjusted target → simple: relay ON if inside < target
  //     (what the display shows is what the user expects)
  //   Automatic (temp drift)   → 1°F hysteresis to prevent short-cycling:
  //     relay ON when inside < target-1, OFF when inside >= target
  bool shouldHeat;
  if (lastChangeWasUser) {
    // User-initiated: straightforward comparison, no hysteresis
    shouldHeat = (curIndoorTemp < targIndoorTemp);
  } else if (isCurHeatOn) {
    shouldHeat = (curIndoorTemp < targIndoorTemp);
  } else {
    shouldHeat = (curIndoorTemp < (targIndoorTemp - 1));
  }

  // Enforce short-cycle protection.
  // Two scenarios:
  //   1) User just changed the target → 5s debounce (let them finish adjusting)
  //   2) Relay changed on its own (temp drift) → 3-min furnace protection
  if (lastHeatChange > 0) {
    unsigned long elapsed = millis() - lastHeatChange;
    unsigned long holdTime = lastChangeWasUser ? userDebounce
                           : (isCurHeatOn ? minRunTime : minOffTime);
    if (isCurHeatOn && !shouldHeat && elapsed < holdTime) {
      shouldHeat = true;   // force relay to stay ON during hold
    } else if (!isCurHeatOn && shouldHeat && elapsed < holdTime) {
      shouldHeat = false;  // force relay to stay OFF during hold
    }
  }

  // Track state changes
  if (shouldHeat != isCurHeatOn) {
    lastHeatChange = millis();
    lastChangeWasUser = false;  // relay toggled on its own → 3-min protection next time
  }
  isCurHeatOn = shouldHeat;

  // Drive relay output (if configured via device.cfg or compile-time default)
  if (cfgRelayPin >= 0) {
    digitalWrite(cfgRelayPin, isCurHeatOn ? HIGH : LOW);
  }

  TFTtext(&tft, 5, 10, "Inside", &FreeSans9pt7b, 1, ILI9341_YELLOW);

  if (curIndoorTemp > 99.0) {
    curIndoorTemp = myround(curIndoorTemp) % 100;
    curIndoorTempColor = ILI9341_RED;
  } else if (curIndoorTemp <= 0.0 and curIndoorTemp > -100.0) {
    curIndoorTemp = abs(curIndoorTemp);
    curIndoorTempColor = ILI9341_CYAN;
  } else if (curIndoorTemp <= 0 and curIndoorTemp > -100) {
    curIndoorTemp = abs(myround(curIndoorTemp)) % 100;
    curIndoorTempColor = ILI9341_PURPLE;
  } else {
    curIndoorTempColor = ILI9341_GREEN;
  }

  if (curIndoorTempOld != myround(curIndoorTemp)) {
    sprintf(insidetemp, "%02d", myround(curIndoorTempOld));
    LCDtemp(&tft, 0, 130, insidetemp, &LCD7segment72pt7b, 1, TFTbackgroundColor);
    curIndoorTempOld = myround(curIndoorTemp);
  }
  sprintf(insidetemp, "%02d", myround(curIndoorTemp));
  tft.getTextBounds(insidetemp, 0, 130, &x1, &y1, &w, &h);
  LCDtemp(&tft, 0, 130, insidetemp, &LCD7segment72pt7b, 1, curIndoorTempColor);

  // Touch area coordinates
  coordTouchLight[0] = 15;
  coordTouchLight[1] = y1 + h + 10;
  coordTouchLight[2] = 140;
  coordTouchLight[3] = tft.height() - coordTouchLight[1] + radius;
  coordTouchSystem[0] = coordTouchLight[0] + coordTouchLight[2] + 15;
  coordTouchSystem[1] = coordTouchLight[1];
  coordTouchSystem[2] = coordTouchLight[2];
  coordTouchSystem[3] = coordTouchLight[3];

  tft.drawRoundRect(coordTouchLight[0], coordTouchLight[1], coordTouchLight[2], coordTouchLight[3], radius, ILI9341_YELLOW);
  tft.drawRoundRect(coordTouchSystem[0], coordTouchSystem[1], coordTouchSystem[2], coordTouchSystem[3], radius, ILI9341_YELLOW);

  TFTtext(&tft, coordTouchLight[0] + coordTouchLight[2] / 2, coordTouchLight[1] + 18,
      "Light", &FreeSansBold9pt7b, 1, ILI9341_YELLOW);

  // coordMinus/coordPlus removed -- glyphs no longer drawn.

  tft.drawLine(coordTouchLight[0], coordTouchLight[1] + 26,
      coordTouchLight[2] + coordTouchLight[0] - 1, coordTouchLight[1] + 26, ILI9341_YELLOW);
  tft.drawLine(coordTouchSystem[0], coordTouchSystem[1] + 26,
      coordTouchSystem[2] + coordTouchSystem[0] - 1, coordTouchSystem[1] + 26, ILI9341_YELLOW);
  TFTtext(&tft, coordTouchSystem[0] + 5, coordTouchSystem[1] + 18,
      "System", &FreeSansBold9pt7b, 1, ILI9341_YELLOW);
  // +/- glyphs removed -- real FocusPro has physical buttons below
  // the LCD. Touch targets still active via coordTouchLight/coordTouchSystem boxes.

  TFTtext(&tft, coordTouchLight[0] + coordTouchLight[2] - 5, coordTouchLight[1] - 25,
      "Heat On", &FreeSans9pt7b, 1, isHeatOn && isCurHeatOn ? ILI9341_YELLOW : TFTbackgroundColor);
  TFTtext(&tft, coordTouchSystem[0] + 20, coordTouchLight[1] + 46,
      "Heat", &FreeSans9pt7b, 1, isHeatOn ? ILI9341_YELLOW : TFTbackgroundColor);
  TFTtext(&tft, coordTouchSystem[0] + 40, 27,
      "Heat", &FreeSans9pt7b, 1, isHeatOn ? ILI9341_YELLOW : TFTbackgroundColor);
  TFTtext(&tft, coordTouchSystem[0] + 55, 47,
      "Setting", &FreeSans9pt7b, 1, isHeatOn ? ILI9341_YELLOW : TFTbackgroundColor);

  if (targIndoorTempOld != targIndoorTemp) {
    sprintf(insidetemp, "%02d", targIndoorTempOld);
    LCDtemp(&tft, 0, 130, insidetemp, &LCD7segment48pt7b, 1, TFTbackgroundColor, true);
    targIndoorTempOld = targIndoorTemp;
  }
  sprintf(insidetemp, "%02d", targIndoorTemp);
  LCDtemp(&tft, 0, 130, insidetemp, &LCD7segment48pt7b, 1, isHeatOn ? ILI9341_GREEN : TFTbackgroundColor, true);

  return micros() - start;
}

// ===========================================================================
// Web Page - HTML5 Canvas mirroring TFT display
// ===========================================================================

// ---------------------------------------------------------------------------
// Template variable replacement engine for SD-hosted pages
// Replaces {$VARIABLE_NAME} tokens with live firmware values.
// ---------------------------------------------------------------------------

// Helper: format uptime as "Xd Yh Zm"
String uptimeText() {
  unsigned long sec = millis() / 1000;
  unsigned long d = sec / 86400; sec %= 86400;
  unsigned long h = sec / 3600;  sec %= 3600;
  unsigned long m = sec / 60;
  String s = "";
  if (d > 0) { s += String(d) + "d "; }
  s += String(h) + "h " + String(m) + "m";
  return s;
}

// Helper: get CSS hex color for current temperature
// Matches the TFT display color logic:
//   > 99F  = red       (ILI9341_RED)
//   0 to 99F = green   (ILI9341_GREEN)
//   0 to -99F = cyan   (ILI9341_CYAN)
//   <= -100F = purple  (ILI9341_PURPLE)
String tempColorCSS() {
  if (curIndoorTemp > 99.0)                              return "#FF0000";
  else if (curIndoorTemp <= 0.0 && curIndoorTemp > -100.0) return "#00FFFF";
  else if (curIndoorTemp <= -100.0)                      return "#800080";
  else                                                   return "#00FF00";
}

// ===========================================================================
// Smarty-Style Template Engine
// ===========================================================================
//
// Syntax (subset of Smarty / PHP Smarty template language):
//
//   {$VARIABLE}              Variable substitution
//   {if EXPR}...{/if}        Conditional block
//   {if EXPR}...{else}...{/if}
//   {if EXPR}...{elseif EXPR}...{else}...{/if}
//
// EXPR can be:
//   {$VAR}                   Truthy test (non-empty, non-"0")
//   {$VAR == "value"}        String equality
//   {$VAR != "value"}        String inequality
//   {$VAR > number}          Numeric greater-than
//   {$VAR >= number}         Numeric greater-or-equal
//   {$VAR < number}          Numeric less-than
//   {$VAR <= number}         Numeric less-or-equal
//   {$VAR == $VAR2}          Variable-to-variable comparison
//
// Nesting is supported up to 8 levels deep.
// Literal curly braces: use {ldelim} and {rdelim} if needed.
//
// Comments: {* ... *} are stripped from output and may span multiple
// lines.  Nested comments are supported: {* outer {* inner *} outer *}
// works correctly via a depth counter.
//
// Literal blocks: {literal}...{/literal} passes content through raw
// with no template processing.  Ideal for JavaScript blocks.
//
// The engine processes the template line-by-line.  Conditionals and
// comments may span multiple lines.  Each {if} increments a nesting
// depth counter and {/if} decrements it.
// ===========================================================================

// Forward declaration of {foreach} loop state (defined later, used by resolveVar)
// (TPL_MAX_FOREACH_ITEMS defined near top of file)
extern struct TplForeachState {
  bool active;        // Currently inside a {foreach} block?
  bool parentOutput;  // Was parent outputting when foreach started?
  String itemVar;     // Name of the loop variable (e.g. "net")
  int count;          // Total items in the array
  int current;        // Current iteration index (0-based)
  String bodyLines[64];
  int bodyLineCount;
  bool collecting;    // Are we collecting body lines (first pass)?
} tplForeach;

// Resolve a Smarty variable name to its current string value.
// Returns true if the variable was recognized, false otherwise.
bool resolveVar(const String &varName, String &value) {

  // --- Temperature & Control ---
  // CURRENT_TEMP matches the TFT 7-segment display: always 2 character positions.
  //   > 99F:       last 2 digits, zero-padded (e.g. 105 -> "05")     red
  //   10-99F:      as-is (e.g. 72 -> "72")                           green
  //   0-9F:        space-padded (e.g. 5 -> " 5")                     green
  //   0 to -9F:    abs value, space-padded (e.g. -3 -> " 3")         cyan
  //   -10 to -99F: abs value (e.g. -42 -> "42")                      cyan
  //   <= -100F:    last 2 digits of abs, zero-padded (e.g. -105->"05") purple
  if (varName == "CURRENT_TEMP") {
    int t = myround(curIndoorTemp);
    char buf[4];
    if (curIndoorTemp > 99.0) {
      sprintf(buf, "%02d", t % 100);
    } else if (curIndoorTemp <= -100.0) {
      sprintf(buf, "%02d", abs(t) % 100);
    } else if (curIndoorTemp <= 0.0) {
      sprintf(buf, "%2d", abs(t));
    } else {
      sprintf(buf, "%2d", t);
    }
    value = String(buf);
  }
  else if (varName == "TARGET_TEMP")       value = String(targIndoorTemp);
  else if (varName == "CURRENT_TEMP_RAW")  value = String(curIndoorTemp, 1);
  else if (varName == "TEMP_COLOR")        value = tempColorCSS();
  else if (varName == "HEAT_ENABLED")      value = isHeatOn ? "1" : "0";
  else if (varName == "HEAT_ACTIVE")       value = isCurHeatOn ? "1" : "0";
  else if (varName == "HEAT_ENABLED_TEXT") value = isHeatOn ? "On" : "Off";
  else if (varName == "HEAT_ACTIVE_TEXT")  value = isCurHeatOn ? "On" : "Off";

  // --- Environment (BME) ---
  else if (varName == "BME_PRESENT") {
    if      (!bmePoweredOn)  value = "";  // Powered off for cold protection
    else if (bme680Present)  value = "bme680";
    else if (bme280Present)  value = "bme280";
    else                     value = "";
  }
  else if (varName == "HUMIDITY") {
    if      (!bmePoweredOn)  value = "";  // Powered off for cold protection
    else if (bme680Present)  value = String(bme680.humidity, 1);
    else if (bme280Present)  value = String(bme280.readHumidity(), 1);
    else                     value = "";
  }
  else if (varName == "PRESSURE_HPA") {
    if      (!bmePoweredOn)  value = "";  // Powered off for cold protection
    else if (bme680Present)  value = String(bme680.pressure / 100.0, 2);
    else if (bme280Present)  value = String(bme280.readPressure() / 100.0, 2);
    else                     value = "";
  }
  else if (varName == "PRESSURE_INHG") {
    if      (!bmePoweredOn)  value = "";  // Powered off for cold protection
    else if (bme680Present)  value = String(bme680.pressure / 100.0 * 0.02953, 2);
    else if (bme280Present)  value = String(bme280.readPressure() / 100.0 * 0.02953, 2);
    else                     value = "";
  }
  else if (varName == "GAS_KOHMS") {
    if      (!bmePoweredOn)  value = "";  // Powered off for cold protection
    else if (bme680Present)  value = String(bme680.gas_resistance / 1000.0, 1);
    else                     value = "";
  }

  // --- Network & System ---
  else if (varName == "BOARD")          value = curBoard;
  else if (varName == "UPTIME")         value = String(millis() / 1000);
  else if (varName == "UPTIME_TEXT")    value = uptimeText();
  else if (varName == "WEBSERVER_PORT") value = String(WEBSERVER_PORT);
  else if (varName == "SD_CARD")        value = sdCardPresent ? "1" : "0";
  else if (varName == "SD_FORMAT")      value = sdCardPresent ? getSDCardFormat() : "";

  // Battery voltage (boards with LiPo monitoring)
#if defined(HAS_BATT)
  else if (varName == "BATVOLTS") value = String(readBatteryVoltage(), 2);
#endif

  // Network-specific variables - use ETH_* or WIFI_* instead of generic
#if defined(WINC1500)
  else if (varName == "WIFI_SSID")       value = WiFi.SSID();
  else if (varName == "WIFI_SECURITY") value = encTypeStr(WiFi.encryptionType());
  else if (varName == "WIFI_RSSI")       value = String(WiFi.RSSI());
#elif defined(ESP_WIFI) && !defined(DUAL_NETWORK)
  else if (varName == "WIFI_SSID")       value = WiFi.SSID();
  else if (varName == "WIFI_SECURITY") value = getCurrentWifiSecurity();
  else if (varName == "WIFI_RSSI")       value = String(WiFi.RSSI());
#endif

#if defined(DUAL_NETWORK)
  // --- Dual Network: Ethernet interface (current values) ---
  else if (varName == "ETH_IP")          value = IpAddress2String(Ethernet.localIP());
  else if (varName == "ETH_MAC")         value = macToString(W5500_mac);
  else if (varName == "ETH_SUBNET")      value = IpAddress2String(Ethernet.subnetMask());
  else if (varName == "ETH_GATEWAY")     value = IpAddress2String(Ethernet.gatewayIP());
  else if (varName == "ETH_DNS")         value = IpAddress2String(Ethernet.dnsServerIP());
  else if (varName == "ETH_LINK")        value = (Ethernet.linkStatus() == LinkON) ? "Connected" : "Disconnected";
  // --- Dual Network: Ethernet interface (configured values) ---
  else if (varName == "ETH_CFG_IP")      value = (cfgEthIP != IPAddress(0,0,0,0)) ? IpAddress2String(cfgEthIP) : "";
  else if (varName == "ETH_CFG_DNS")     value = (cfgEthDNS != IPAddress(0,0,0,0)) ? IpAddress2String(cfgEthDNS) : "";
  else if (varName == "ETH_CFG_SUBNET")  value = (cfgEthSubnet != IPAddress(255,255,255,0)) ? IpAddress2String(cfgEthSubnet) : "";
  else if (varName == "ETH_CFG_GATEWAY") value = (cfgEthGateway != IPAddress(0,0,0,0)) ? IpAddress2String(cfgEthGateway) : "";
  else if (varName == "ETH_CFG_MAC")     value = cfgMacOverride ? macToString(W5500_mac) : "";
  // --- Dual Network: WiFi interface (current values) ---
  else if (varName == "WIFI_IP")         value = IpAddress2String(WiFi.localIP());
  else if (varName == "WIFI_MAC") {
    byte wMac[6]; WiFi.macAddress(wMac);
    value = macToString(wMac);
  }
  else if (varName == "WIFI_SUBNET")     value = IpAddress2String(WiFi.subnetMask());
  else if (varName == "WIFI_GATEWAY")    value = IpAddress2String(WiFi.gatewayIP());
  else if (varName == "WIFI_DNS")        value = IpAddress2String(WiFi.dnsIP());
  else if (varName == "WIFI_LINK")       value = (WiFi.status() == WL_CONNECTED) ? "Connected" : "Disconnected";
  else if (varName == "WIFI_SSID")       value = WiFi.SSID();
  else if (varName == "WIFI_RSSI")       value = String(WiFi.RSSI());
  else if (varName == "WIFI_SECURITY")   value = getCurrentWifiSecurity();
  // --- Dual Network: WiFi interface (configured values) ---
  else if (varName == "WIFI_CFG_IP")      value = (cfgWifiIP != IPAddress(0,0,0,0)) ? IpAddress2String(cfgWifiIP) : "";
  else if (varName == "WIFI_CFG_DNS")     value = (cfgWifiDNS != IPAddress(0,0,0,0)) ? IpAddress2String(cfgWifiDNS) : "";
  else if (varName == "WIFI_CFG_SUBNET")  value = (cfgWifiSubnet != IPAddress(255,255,255,0)) ? IpAddress2String(cfgWifiSubnet) : "";
  else if (varName == "WIFI_CFG_GATEWAY") value = (cfgWifiGateway != IPAddress(0,0,0,0)) ? IpAddress2String(cfgWifiGateway) : "";
  else if (varName == "WIFI_CFG_SSID")    value = cfgWifiSSID;
  else if (varName == "WIFI_CFG_PASS")    value = cfgWifiPass;
  else if (varName == "WIFI_CFG_SECURITY") value = cfgWifiSecurity;
  // Compile-time capability flags
  else if (varName == "IS_DUAL_NETWORK") value = "1";
  else if (varName == "HAS_ETHERNET")    value = "1";
  else if (varName == "HAS_WIFI")        value = "1";
  else if (varName == "HAS_BATT")        value = "1";  // ESP32 has battery monitoring circuit
  else if (varName == "NETWORK_MODE")    value = "dual";
  // Runtime connection status
  else if (varName == "ETH_CONNECTED")   value = (Ethernet.linkStatus() == LinkON) ? "1" : "0";
  else if (varName == "WIFI_CONNECTED")  value = (WiFi.status() == WL_CONNECTED) ? "1" : "0";
#endif

  // --- Network mode flags (all builds) ---
#if !defined(DUAL_NETWORK)
  else if (varName == "IS_DUAL_NETWORK") value = "0";
#if defined(W5500)
  else if (varName == "HAS_ETHERNET")    value = "1";
  else if (varName == "HAS_WIFI")        value = "0";
  else if (varName == "HAS_BATT")        value = "0";  // Ethernet builds don't have battery
  else if (varName == "NETWORK_MODE")    value = "ethernet";
  else if (varName == "ETH_CONNECTED")   value = (Ethernet.linkStatus() == LinkON) ? "1" : "0";
  else if (varName == "WIFI_CONNECTED")  value = "0";
  // ETH_* variables for single-interface Ethernet (current values)
  else if (varName == "ETH_IP")          value = IpAddress2String(Ethernet.localIP());
  else if (varName == "ETH_MAC")         value = macToString(W5500_mac);
  else if (varName == "ETH_SUBNET")      value = IpAddress2String(Ethernet.subnetMask());
  else if (varName == "ETH_GATEWAY")     value = IpAddress2String(Ethernet.gatewayIP());
  else if (varName == "ETH_DNS")         value = IpAddress2String(Ethernet.dnsServerIP());
  else if (varName == "ETH_LINK")        value = (Ethernet.linkStatus() == LinkON) ? "Connected" : "Disconnected";
  // ETH_* variables for single-interface Ethernet (configured values)
  else if (varName == "ETH_CFG_IP")      value = (cfgIP != IPAddress(0,0,0,0)) ? IpAddress2String(cfgIP) : "";
  else if (varName == "ETH_CFG_DNS")     value = (cfgDNS != IPAddress(0,0,0,0)) ? IpAddress2String(cfgDNS) : "";
  else if (varName == "ETH_CFG_SUBNET")  value = (cfgSubnet != IPAddress(255,255,255,0)) ? IpAddress2String(cfgSubnet) : "";
  else if (varName == "ETH_CFG_GATEWAY") value = (cfgGateway != IPAddress(0,0,0,0)) ? IpAddress2String(cfgGateway) : "";
  else if (varName == "ETH_CFG_MAC")     value = cfgMacOverride ? macToString(W5500_mac) : "";
#elif defined(HAS_WIFI)
  else if (varName == "HAS_ETHERNET")    value = "0";
  else if (varName == "HAS_WIFI")        value = "1";
#if defined(HAS_BATT)
  else if (varName == "HAS_BATT")        value = "1";
#else
  else if (varName == "HAS_BATT")        value = "0";
#endif
  else if (varName == "NETWORK_MODE")    value = "wifi";
  else if (varName == "ETH_CONNECTED")   value = "0";
  else if (varName == "WIFI_CONNECTED")  value = (WiFi.status() == WL_CONNECTED) ? "1" : "0";
  // WIFI_* variables for single-interface WiFi (current values)
  else if (varName == "WIFI_IP")         value = IpAddress2String(WiFi.localIP());
  else if (varName == "WIFI_MAC") {
    byte wMac[6]; WiFi.macAddress(wMac);
    value = macToString(wMac);
  }
  else if (varName == "WIFI_SUBNET")     value = IpAddress2String(WiFi.subnetMask());
  else if (varName == "WIFI_GATEWAY")    value = IpAddress2String(WiFi.gatewayIP());
#if defined(ESP_WIFI)
  else if (varName == "WIFI_DNS")        value = IpAddress2String(WiFi.dnsIP());
#else
  // WiFi101 (WINC1500) doesn't expose current DNS - return configured value
  else if (varName == "WIFI_DNS")        value = IpAddress2String(cfgDNS);
#endif
  else if (varName == "WIFI_LINK")       value = (WiFi.status() == WL_CONNECTED) ? "Connected" : "Disconnected";
  else if (varName == "WIFI_SSID")       value = WiFi.SSID();
  else if (varName == "WIFI_RSSI")       value = String(WiFi.RSSI());
#if defined(WINC1500)
  else if (varName == "WIFI_SECURITY")   value = encTypeStr(WiFi.encryptionType());
#else
  else if (varName == "WIFI_SECURITY")   value = getCurrentWifiSecurity();
#endif
  // WIFI_* variables for single-interface WiFi (configured values)
  else if (varName == "WIFI_CFG_IP")      value = (cfgIP != IPAddress(0,0,0,0)) ? IpAddress2String(cfgIP) : "";
  else if (varName == "WIFI_CFG_DNS")     value = (cfgDNS != IPAddress(0,0,0,0)) ? IpAddress2String(cfgDNS) : "";
  else if (varName == "WIFI_CFG_SUBNET")  value = (cfgSubnet != IPAddress(255,255,255,0)) ? IpAddress2String(cfgSubnet) : "";
  else if (varName == "WIFI_CFG_GATEWAY") value = (cfgGateway != IPAddress(0,0,0,0)) ? IpAddress2String(cfgGateway) : "";
  else if (varName == "WIFI_CFG_SSID")    value = cfgWifiSSID;
  else if (varName == "WIFI_CFG_PASS")    value = cfgWifiPass;
  else if (varName == "WIFI_CFG_SECURITY") value = cfgWifiSecurity;
#else
  else if (varName == "HAS_ETHERNET")    value = "0";
  else if (varName == "HAS_WIFI")        value = "0";
  else if (varName == "HAS_BATT")        value = "0";
  else if (varName == "NETWORK_MODE")    value = "none";
  else if (varName == "ETH_CONNECTED")   value = "0";
  else if (varName == "WIFI_CONNECTED")  value = "0";
#endif
#endif

  // --- Client interface (set per-request in webserver()) ---
  else if (varName == "CLIENT_INTERFACE") value = currentClientInterface;
  else if (varName == "CLIENT_IS_ETH")  value = (currentClientInterface == "eth") ? "1" : "0";
  else if (varName == "CLIENT_IS_WIFI") value = (currentClientInterface == "wifi") ? "1" : "0";

  else if (varName == "FREE_RAM") {
#if defined(__SAMD51__) || defined(__SAMD21__)
    // SAMD: use stack pointer minus heap end
    char top;
    value = String((uint32_t)&top - (uint32_t)sbrk(0));
#elif defined(ESP32) || defined(ESP8266)
    // ESP32/ESP8266: use ESP.getFreeHeap()
    value = String(ESP.getFreeHeap());
#elif defined(__arm__)
    // Generic ARM fallback
    value = "";
#else
    value = "";
#endif
  }

  // --- Display State ---
  else if (varName == "DISPLAY_PRESENT") value = displayPresent ? "1" : "0";
  else if (varName == "OTA_SUPPORTED")   value = OTA_SUPPORTED ? "1" : "0";

  // --- FocusPro LCD Regions (all 1/0) ---
  else if (varName == "SHOW_INSIDE_LABEL")  value = "1";
  else if (varName == "SHOW_CURRENT_TEMP")  value = "1";
  else if (varName == "SHOW_DEGREE_SYMBOL") value = "1";
  else if (varName == "SHOW_HEAT_ON")       value = (isHeatOn && isCurHeatOn) ? "1" : "0";
  else if (varName == "SHOW_HEAT_SETTING")  value = isHeatOn ? "1" : "0";
  else if (varName == "SHOW_TARGET_TEMP")   value = isHeatOn ? "1" : "0";
  else if (varName == "SHOW_LIGHT_BUTTON")  value = "1";
  else if (varName == "SHOW_SYSTEM_BUTTON") value = "1";
  else if (varName == "SHOW_HEAT_LABEL")    value = isHeatOn ? "1" : "0";
  else if (varName == "SHOW_MINUS")         value = "1";
  else if (varName == "SHOW_PLUS")          value = "1";
  else if (varName == "SHOW_BME_CARD")      value = (bme280Present || bme680Present) ? "1" : "0";

  // --- Outdoor sensor data (from outdoor unit polling) ---
  else if (varName == "OUTDOOR_AVAILABLE")     value = (outdoorConfigured && outdoorStatus == "Online") ? "1" : "0";
  else if (varName == "OUTDOOR_TEMP")          value = outdoorTemp;
  else if (varName == "OUTDOOR_HUMIDITY")      value = outdoorHumidity;
  else if (varName == "OUTDOOR_PRESSURE_HPA")  value = outdoorPressHpa;
  else if (varName == "OUTDOOR_PRESSURE_INHG") value = outdoorPressInhg;
  else if (varName == "OUTDOOR_GAS_KOHMS")     value = outdoorGasKohms;
  else if (varName == "OUTDOOR_BME_TYPE")      value = outdoorBmeType;
  else if (varName == "OUTDOOR_BOARD")         value = outdoorBoard;
  else if (varName == "OUTDOOR_ETH_IP")        value = outdoorEthIP;
  else if (varName == "OUTDOOR_WIFI_IP")       value = outdoorWifiIP;
  else if (varName == "OUTDOOR_IFACE")         value = outdoorIface;
  else if (varName == "OUTDOOR_BATVOLTS") value = outdoorBatteryV;
  else if (varName == "OUTDOOR_HAS_ETHERNET")  value = outdoorHasEth ? "1" : "0";
  else if (varName == "OUTDOOR_HAS_WIFI")      value = outdoorHasWifi ? "1" : "0";

  // --- Outdoor unit connection config ---
  else if (varName == "OUTDOOR_CONFIGURED")  value = outdoorConfigured ? "1" : "0";
  else if (varName == "OUTDOOR_CFG_IP")      value = outdoorConfigured ? IpAddress2String(outdoorIP) : "";
  else if (varName == "OUTDOOR_CFG_PORT")    value = String(outdoorPort);
  else if (varName == "OUTDOOR_POLL_SECS")   value = String(outdoorPollSecs);
  else if (varName == "OUTDOOR_STATUS")      value = outdoorStatus;

  // --- Foreach loop variables: $item.field and $item@index ---
  else if (tplForeach.active && varName.startsWith(tplForeach.itemVar + ".")) {
    String field = varName.substring(tplForeach.itemVar.length() + 1);
    int idx = tplForeach.current;
#if defined(HAS_WIFI)
    if (idx >= 0 && idx < tplScanCount) {
      if      (field == "ssid") value = tplScanCache[idx].ssid;
      else if (field == "rssi") value = String(tplScanCache[idx].rssi);
      else if (field == "enc")  value = tplScanCache[idx].enc;
      else return false;
    } else return false;
#else
    return false;
#endif
  }
  else if (tplForeach.active && varName == tplForeach.itemVar + "@index") {
    value = String(tplForeach.current);
  }
  else if (tplForeach.active && varName == tplForeach.itemVar + "@count") {
    value = String(tplForeach.count);
  }
  else if (tplForeach.active && varName == tplForeach.itemVar + "@first") {
    value = (tplForeach.current == 0) ? "1" : "0";
  }
  else if (tplForeach.active && varName == tplForeach.itemVar + "@last") {
    value = (tplForeach.current == tplForeach.count - 1) ? "1" : "0";
  }

  else return false;  // Unknown variable

  return true;
}

// Parse a value token from a Smarty expression.
// Handles: $VAR (variable reference), "string", 'string', or bare number.
String parseValue(const String &token) {
  String t = token;
  t.trim();
  if (t.length() == 0) return "";

  // $VARIABLE -- resolve it
  if (t.charAt(0) == '$') {
    String val;
    if (resolveVar(t.substring(1), val)) return val;
    return "";  // Unknown var resolves to empty
  }
  // Quoted string -- strip quotes
  if ((t.charAt(0) == '"'  && t.charAt(t.length()-1) == '"') ||
      (t.charAt(0) == '\'' && t.charAt(t.length()-1) == '\'')) {
    return t.substring(1, t.length() - 1);
  }
  // Bare value (number or unquoted string)
  return t;
}

// Evaluate a Smarty {if} / {elseif} expression.
// Supports: $VAR (truthy), $VAR op VALUE, $VAR op $VAR2
// Operators: ==  !=  >  >=  <  <=  eq  ne  gt  ge  lt  le
bool evaluateExpr(const String &expr) {
  String e = expr;
  e.trim();
  if (e.length() == 0) return false;

  // Find operator by scanning for known two-char and one-char operators
  // Order matters: check two-char first (==, !=, >=, <=, eq, ne, gt, ge, lt, le)
  struct { const char *op; int len; } ops[] = {
    {"==",2}, {"!=",2}, {">=",2}, {"<=",2},
    {"eq",2}, {"ne",2}, {"gt",2}, {"ge",2}, {"lt",2}, {"le",2},
    {">",1},  {"<",1}
  };

  for (int i = 0; i < 12; i++) {
    // Search for operator surrounded by spaces (to avoid matching inside strings)
    String needle = String(" ") + ops[i].op + " ";
    int idx = e.indexOf(needle);
    if (idx < 0) continue;

    String lhs = parseValue(e.substring(0, idx));
    String rhs = parseValue(e.substring(idx + ops[i].len + 2));
    const char *op = ops[i].op;

    // Try numeric comparison first
    bool isNumeric = true;
    float lf = lhs.toFloat();
    float rf = rhs.toFloat();
    // toFloat returns 0 for non-numeric; check if it's actually "0" or empty
    if (lhs.length() == 0 && rhs.length() == 0) isNumeric = false;
    // Simple heuristic: if both start with digit, minus, or dot, treat as numeric
    if (lhs.length() > 0 && rhs.length() > 0) {
      char lc = lhs.charAt(0), rc = rhs.charAt(0);
      if (!isDigit(lc) && lc != '-' && lc != '.') isNumeric = false;
      if (!isDigit(rc) && rc != '-' && rc != '.') isNumeric = false;
    }

    if (strcmp(op, "==") == 0 || strcmp(op, "eq") == 0)
      return isNumeric ? (lf == rf) : (lhs == rhs);
    if (strcmp(op, "!=") == 0 || strcmp(op, "ne") == 0)
      return isNumeric ? (lf != rf) : (lhs != rhs);
    if (strcmp(op, ">") == 0  || strcmp(op, "gt") == 0)
      return lf > rf;
    if (strcmp(op, ">=") == 0 || strcmp(op, "ge") == 0)
      return lf >= rf;
    if (strcmp(op, "<") == 0  || strcmp(op, "lt") == 0)
      return lf < rf;
    if (strcmp(op, "<=") == 0 || strcmp(op, "le") == 0)
      return lf <= rf;
  }

  // No operator found -- simple truthy test on a single value
  String val = parseValue(e);
  return (val.length() > 0 && val != "0");
}

// ---------------------------------------------------------------------------
// Smarty template processor -- handles {$var}, {if}, {elseif}, {else}, {/if},
// {ldelim}, {rdelim}, and Smarty-style {* comments *}.
// Processes the SD file line-by-line with a nesting stack.
// ---------------------------------------------------------------------------

#define TPL_MAX_DEPTH 8

// Nesting stack state for {if}/{elseif}/{else}/{/if}
struct {
  bool outputting;    // Are we currently emitting output at this depth?
  bool anyBranchTook; // Has any branch at this depth already matched?
  bool parentOutput;  // Was the parent depth outputting?
} tplStack[TPL_MAX_DEPTH];
int tplDepth = 0;
bool tplOutputting = true;    // Master output flag
int  tplCommentDepth = 0;     // Nested {* ... *} comment depth (0 = not in comment)
bool tplLiteral = false;      // {literal} mode -- pass everything through raw

// {foreach} loop state -- one level deep (no nested foreach)
// (TPL_MAX_FOREACH_ITEMS defined earlier with forward declaration)
TplForeachState tplForeach;

// WiFi scan cache declared earlier (before webserver)

void tplReset() {
  tplDepth = 0;
  tplOutputting = true;
  tplCommentDepth = 0;
  tplLiteral = false;
  tplForeach.active = false;
  tplForeach.collecting = false;
  tplForeach.bodyLineCount = 0;
#if defined(HAS_WIFI)
  tplScanDone = false;
  tplScanCount = 0;
#endif
}

// Process a single Smarty tag (the content between { and }).
// Returns true if the tag was consumed (don't output it).
bool processSmartyTag(const String &tag) {
  String t = tag;
  t.trim();

  // {* comment *} -- consume silently
  if (t.startsWith("*") && t.endsWith("*")) return true;

  // {ldelim} / {rdelim} -- handled during substitution pass, not here
  if (t == "ldelim" || t == "rdelim") return false;

  // {if EXPR}
  if (t.startsWith("if ")) {
    if (tplDepth >= TPL_MAX_DEPTH) return true;  // Too deep, silently skip
    bool parentOut = tplOutputting;
    bool result = parentOut ? evaluateExpr(t.substring(3)) : false;
    tplStack[tplDepth].parentOutput = parentOut;
    tplStack[tplDepth].outputting = result;
    tplStack[tplDepth].anyBranchTook = result;
    tplDepth++;
    tplOutputting = result;
    return true;
  }

  // {elseif EXPR}
  if (t.startsWith("elseif ")) {
    if (tplDepth <= 0) return true;  // Mismatched, ignore
    int d = tplDepth - 1;
    if (tplStack[d].anyBranchTook || !tplStack[d].parentOutput) {
      tplOutputting = false;
    } else {
      bool result = evaluateExpr(t.substring(7));
      tplStack[d].outputting = result;
      tplStack[d].anyBranchTook = result;
      tplOutputting = result;
    }
    return true;
  }

  // {else}
  if (t == "else") {
    if (tplDepth <= 0) return true;
    int d = tplDepth - 1;
    if (tplStack[d].anyBranchTook || !tplStack[d].parentOutput) {
      tplOutputting = false;
    } else {
      tplStack[d].outputting = true;
      tplStack[d].anyBranchTook = true;
      tplOutputting = true;
    }
    return true;
  }

  // {/if}
  if (t == "/if") {
    if (tplDepth <= 0) return true;
    tplDepth--;
    // Restore parent output state
    if (tplDepth == 0) {
      tplOutputting = true;
    } else {
      // Parent level is outputting if its branch was taken
      // AND *its* parent was also outputting
      tplOutputting = tplStack[tplDepth - 1].outputting;
    }
    return true;
  }

  return false;  // Not a recognized control tag
}

// Process one line of Smarty template: handle tags, substitute variables,
// and emit output if the current conditional state allows it.
// Supports nested multi-line {* ... {* ... *} ... *} comments via depth counter.
// Supports {literal}...{/literal} for raw pass-through (e.g. JavaScript).
void processTemplateLine(const String &line, HardwareClient &cl) {
  // --- {foreach} body collection mode ---
  // When collecting foreach body lines, buffer everything until {/foreach}
  if (tplForeach.collecting) {
    String trimmed = line;
    trimmed.trim();
    // Check for {/foreach} (bare or HTML-comment-wrapped)
    if (trimmed == "{/foreach}" || trimmed == "<!-- {/foreach} -->" ||
        trimmed.indexOf("{/foreach}") >= 0) {
      // End collection -- replay buffered lines for each item
      tplForeach.collecting = false;
      if (tplForeach.parentOutput && tplForeach.count > 0) {
        for (int i = 0; i < tplForeach.count; i++) {
          tplForeach.current = i;
          tplOutputting = true;
          for (int j = 0; j < tplForeach.bodyLineCount; j++) {
            processTemplateLine(tplForeach.bodyLines[j], cl);
          }
        }
      }
      tplForeach.active = false;
      tplOutputting = tplForeach.parentOutput;
      return;
    }
    // Check for {foreachelse} -- handled inline as a tag, but if it
    // appears on its own line we need to catch it here too
    if (trimmed == "{foreachelse}" || trimmed == "<!-- {foreachelse} -->") {
      // Process it as a tag by falling through to processTemplateLine
      // (the tag handler above will handle the state change)
    } else {
      // Buffer this line (up to capacity)
      if (tplForeach.bodyLineCount < 64) {
        tplForeach.bodyLines[tplForeach.bodyLineCount++] = line;
      }
      return;
    }
  }

  // --- {foreach} foreachelse passthrough mode ---
  // After {foreachelse} with count==0: output lines until {/foreach}
  // After {foreachelse} with count>0: suppress lines until {/foreach}
  if (tplForeach.active && !tplForeach.collecting && tplForeach.bodyLineCount == -1) {
    String trimmed = line;
    trimmed.trim();
    if (trimmed == "{/foreach}" || trimmed == "<!-- {/foreach} -->" ||
        trimmed.indexOf("{/foreach}") >= 0) {
      tplForeach.active = false;
      tplOutputting = tplForeach.parentOutput;
      return;
    }
    // Output if count==0 (tplOutputting already set by foreachelse handler)
    if (tplOutputting) {
      cl.print(line);
    }
    return;
  }

  String output = "";
  int pos = 0;
  int len = line.length();

  // {literal} mode -- pass everything through raw, only look for {/literal}
  if (tplLiteral) {
    // Check for both bare and HTML-comment-wrapped variants
    int endTag = line.indexOf("{/literal}");
    int endTagHtml = line.indexOf("<!-- {/literal} -->");
    // Prefer the HTML-wrapped variant if it appears at or before bare
    if (endTagHtml >= 0 && (endTag < 0 || endTagHtml <= endTag)) {
      if (tplOutputting) output += line.substring(0, endTagHtml);
      tplLiteral = false;
      pos = endTagHtml + 19;
    } else if (endTag >= 0) {
      if (tplOutputting) output += line.substring(0, endTag);
      tplLiteral = false;
      pos = endTag + 10;
    } else {
      // Entire line is literal
      if (tplOutputting) cl.print(line);
      return;
    }
  }

  // If we're inside a (possibly nested) comment, scan for {* and *} to
  // track depth.  Only resume normal processing when depth hits 0.
  if (tplCommentDepth > 0) {
    while (pos < len && tplCommentDepth > 0) {
      int nextOpen  = line.indexOf("{*", pos);
      int nextClose = line.indexOf("*}", pos);
      // Neither found -- rest of line is inside comment
      if (nextOpen < 0 && nextClose < 0) return;
      // Determine which comes first
      bool openFirst = (nextOpen >= 0) && (nextClose < 0 || nextOpen < nextClose);
      if (openFirst) {
        tplCommentDepth++;
        pos = nextOpen + 2;
      } else {
        tplCommentDepth--;
        pos = nextClose + 2;
      }
    }
    if (tplCommentDepth > 0) return;  // Still inside comment
    // Fell out of comment -- continue processing remainder of line
  }

  while (pos < len) {
    int tagStart = line.indexOf('{', pos);
    if (tagStart < 0) {
      // No more tags -- append remainder
      if (tplOutputting) output += line.substring(pos);
      break;
    }

    // Append text before the tag
    if (tplOutputting && tagStart > pos) {
      String before = line.substring(pos, tagStart);
      // Strip trailing "<!-- " if this tag is HTML-comment-wrapped
      if (before.endsWith("<!-- ")) {
        before = before.substring(0, before.length() - 5);
      }
      output += before;
    }

    // Check if this { is preceded by "<!-- " (even if nothing before it to append)
    bool htmlWrapped = false;
    if (tagStart >= 5 && line.substring(tagStart - 5, tagStart) == "<!-- ") {
      htmlWrapped = true;
    } else if (tagStart >= 4 && line.substring(tagStart - 4, tagStart) == "<!--") {
      htmlWrapped = true;
    }

    // Check for comment open {*
    if (tagStart + 1 < len && line.charAt(tagStart + 1) == '*') {
      // Look for closing *} on this line (scanning for nested opens too)
      tplCommentDepth = 1;
      int scanPos = tagStart + 2;
      while (scanPos < len && tplCommentDepth > 0) {
        int nextOpen  = line.indexOf("{*", scanPos);
        int nextClose = line.indexOf("*}", scanPos);
        if (nextOpen < 0 && nextClose < 0) break;
        bool openFirst = (nextOpen >= 0) && (nextClose < 0 || nextOpen < nextClose);
        if (openFirst) {
          tplCommentDepth++;
          scanPos = nextOpen + 2;
        } else {
          tplCommentDepth--;
          scanPos = nextClose + 2;
        }
      }
      if (tplCommentDepth == 0) {
        // Comment fully closed on this line -- skip trailing " -->"
        String rem = line.substring(scanPos);
        if (rem.startsWith(" -->"))      scanPos += 4;
        else if (rem.startsWith("-->"))  scanPos += 3;
        pos = scanPos;
        continue;
      }
      // Comment spans to next line(s)
      break;
    }

    // Find closing brace
    int tagEnd = line.indexOf('}', tagStart + 1);
    if (tagEnd < 0) {
      // Unclosed brace -- treat rest of line as literal
      if (tplOutputting) output += line.substring(tagStart);
      break;
    }

    // If HTML-comment-wrapped, skip trailing " -->" after the closing brace
    int tagAdvance = tagEnd + 1;  // default: just past '}'
    if (htmlWrapped) {
      String after = line.substring(tagEnd + 1);
      if (after.startsWith(" -->"))      tagAdvance = tagEnd + 5;
      else if (after.startsWith("-->"))  tagAdvance = tagEnd + 4;
    }

    String tagContent = line.substring(tagStart + 1, tagEnd);
    String tc = tagContent;
    tc.trim();

    // --- {literal} -- enter raw pass-through mode ---
    if (tc == "literal") {
      tplLiteral = true;
      // Output remainder of this line raw (until {/literal} or end of line)
      pos = tagAdvance;
      int endTag = line.indexOf("{/literal}", pos);
      // Also check for <!-- {/literal} --> variant
      int endTagHtml = line.indexOf("<!-- {/literal} -->", pos);
      if (endTagHtml >= 0 && (endTag < 0 || endTagHtml <= endTag)) {
        if (tplOutputting) output += line.substring(pos, endTagHtml);
        tplLiteral = false;
        pos = endTagHtml + 19;  // length of "<!-- {/literal} -->"
        continue;
      }
      if (endTag >= 0) {
        if (tplOutputting) output += line.substring(pos, endTag);
        tplLiteral = false;
        pos = endTag + 10;
        continue;
      }
      // {/literal} not on this line -- output rest raw
      if (tplOutputting) output += line.substring(pos);
      break;
    }

    // --- Control tags: {if}, {elseif}, {else}, {/if} ---
    if (tc.startsWith("if ") || tc.startsWith("elseif ") ||
        tc == "else" || tc == "/if") {
      processSmartyTag(tc);
      pos = tagAdvance;
      continue;
    }

    // --- {foreach $ARRAY as $item} ---
    if (tc.startsWith("foreach ")) {
      // Parse: foreach $ARRAYNAME as $itemvar
      String expr = tc.substring(8);
      expr.trim();
      int asPos = expr.indexOf(" as ");
      if (asPos > 0) {
        String arrayName = expr.substring(0, asPos);
        arrayName.trim();
        if (arrayName.startsWith("$")) arrayName = arrayName.substring(1);
        String itemName = expr.substring(asPos + 4);
        itemName.trim();
        if (itemName.startsWith("$")) itemName = itemName.substring(1);

        tplForeach.parentOutput = tplOutputting;
        tplForeach.itemVar = itemName;
        tplForeach.current = 0;
        tplForeach.bodyLineCount = 0;
        tplForeach.collecting = true;
        tplForeach.active = true;
        tplForeach.count = 0;

#if defined(HAS_WIFI)
        if (arrayName == "WIFI_NETWORKS") {
          // Perform WiFi scan (cached per template render)
          if (!tplScanDone) {
            wifiScanSorted();
            tplScanDone = true;
          }
          tplForeach.count = tplScanCount;
        }
#endif
        // Suppress output while collecting body lines
        tplOutputting = false;
      }
      pos = tagAdvance;
      continue;
    }

    // --- {foreachelse} -- content shown when array is empty ---
    if (tc == "foreachelse") {
      // If we're collecting, switch: if count==0, output the else block
      // For now, just stop collecting body and note the else section
      // Implementation: stop collecting, output foreachelse content directly
      // if count is 0, then we let the remaining lines before {/foreach}
      // flow through normally
      if (tplForeach.collecting) {
        tplForeach.collecting = false;
        // If no items, output lines between {foreachelse} and {/foreach}
        if (tplForeach.count == 0) {
          tplOutputting = tplForeach.parentOutput;
          // Set a flag so {/foreach} knows to just close without replay
          tplForeach.bodyLineCount = -1;  // sentinel: foreachelse active
        } else {
          // Has items: suppress the foreachelse block, replay body now
          if (tplForeach.parentOutput) {
            for (int i = 0; i < tplForeach.count; i++) {
              tplForeach.current = i;
              tplForeach.active = true;
              tplOutputting = true;
              for (int j = 0; j < tplForeach.bodyLineCount; j++) {
                processTemplateLine(tplForeach.bodyLines[j], cl);
              }
            }
          }
          tplForeach.active = false;
          tplOutputting = false;  // suppress foreachelse content
        }
      }
      pos = tagAdvance;
      continue;
    }

    // --- {ldelim} / {rdelim} ---
    if (tc == "ldelim") {
      if (tplOutputting) output += "{";
      pos = tagAdvance;
      continue;
    }
    if (tc == "rdelim") {
      if (tplOutputting) output += "}";
      pos = tagAdvance;
      continue;
    }

    // --- Variable substitution: {$VARNAME} ---
    if (tc.startsWith("$")) {
      if (tplOutputting) {
        String val;
        if (resolveVar(tc.substring(1), val)) {
          output += val;
        } else {
          // Unknown variable -- output tag literally
          output += "{" + tagContent + "}";
        }
      }
      pos = tagAdvance;
      continue;
    }

    // --- Unrecognized tag -- pass through literally ---
    if (tplOutputting) {
      output += "{" + tagContent + "}";
    }
    pos = tagEnd + 1;
  }

  if (output.length() > 0) {
    cl.print(output);
  }
}

// ---------------------------------------------------------------------------
// Generate default index.html on SD card with Smarty template variables
// ---------------------------------------------------------------------------
// Creates a starting-point template that users can customize. Uses Smarty
// variables (processed at serve time) rather than hardcoded values, so the
// page updates live without regeneration.

void generateIndexHtml() {
  file_t f;
  if (!f.open("index.html", O_WRONLY | O_CREAT | O_TRUNC)) {
    Serial.println("generateIndexHtml: can't create index.html");
    return;
  }

  // HTML head
  f.println("<!DOCTYPE html>");
  f.println("<html><head>");
  f.println("<title>Thermostat</title>");
  f.println("<meta name='viewport' content='width=device-width,initial-scale=1'>");
  f.println("<!-- {* FocusPro Thermostat -- Smarty template (edit to customize) *} -->");
  f.println("{literal}<style>");
  f.println("body{font-family:Arial,sans-serif;background:#000;color:#eee;margin:20px;}");
  f.println(".container{max-width:420px;margin:0 auto;}");
  f.println(".card{background:#16213e;border-radius:10px;padding:15px;margin:10px 0;min-width:200px;}");
  f.println(".label{color:#888;font-size:0.9em;margin-bottom:6px;}");
  f.println(".row{display:flex;justify-content:space-between;margin:8px 0;}");
  f.println(".value{color:#60a5fa;}");
  f.println(".heat-on{color:#fbbf24;font-weight:bold;}");
  f.println("#status{display:inline-block;width:8px;height:8px;border-radius:50%;margin-right:6px;}");
  f.println(".online{background:#22c55e;}.offline{background:#ef4444;}.stale{background:#eab308;}");
  f.println("canvas{cursor:pointer;}");
  f.println("</style>{/literal}");
  f.println("</head><body>");

  // Layout: canvas + arrows row, cards below
  f.println("<div class='container'>");
  f.println("<div style='display:flex;align-items:center;gap:10px;'>");
  f.println("<canvas id='cv' width='320' height='240' style='background:#000;border-radius:4px;'></canvas>");
  // Up/Down arrows for temperature adjustment
  f.println("<div style='display:flex;flex-direction:column;gap:8px;'>");
  f.println("<button id='btnUp' onclick='adjTemp(1)' style='width:50px;height:70px;font-size:28px;background:#222;color:#0a0;border:2px solid #444;border-radius:6px;cursor:pointer;'>&#9650;</button>");
  f.println("<button id='btnDown' onclick='adjTemp(-1)' style='width:50px;height:70px;font-size:28px;background:#222;color:#0a0;border:2px solid #444;border-radius:6px;cursor:pointer;'>&#9660;</button>");
  f.println("</div>");
  f.println("</div>");
  // Status line
  f.println("<div style='margin-top:4px;font-size:11px;color:#555;'>");
  f.println("<span id='status' class='online'></span><span id='statusText'>Connected</span></div>");
  // Cards below
  f.println("<div id='cards'></div>");

  f.println("</div>"); // end container

  // Footer links
  f.println("<div style='text-align:center;margin-top:10px;'>");
  f.println("<a href='/config' style='color:#4ade80;font-size:12px;'>&#9881; Configuration</a>");
  f.println(" &middot; ");
  f.println("<a href='/sdfiles' style='color:#fbbf24;font-size:12px;'>&#128193; SD Files</a>");
  f.println(" &middot; ");
  f.println("<a href='/update' style='color:#888;font-size:12px;'>&#128295; Firmware Update</a>");
  f.println("</div>");

  // Smarty variable bridge -- inject server values into JS
  f.println("<script>");
  f.println("var seedTemp = {$CURRENT_TEMP};");
  f.println("var seedTarget = {$TARGET_TEMP};");
  f.println("var seedHeatOn = {$HEAT_ENABLED};");
  f.println("var seedHeatActive = {$HEAT_ACTIVE};");
  f.println("var seedColor = '{$TEMP_COLOR}';");
  f.println("</script>");

  // Main JavaScript -- all in {literal} block
  f.println("{literal}<script>");

  // 7-segment renderer
  f.println("var cv=document.getElementById('cv'),ctx=cv.getContext('2d');");
  f.println("var S={'0':[1,1,1,1,1,1,0],'1':[0,1,1,0,0,0,0],'2':[1,1,0,1,1,0,1],'3':[1,1,1,1,0,0,1],'4':[0,1,1,0,0,1,1],'5':[1,0,1,1,0,1,1],'6':[1,0,1,1,1,1,1],'7':[1,1,1,0,0,0,0],'8':[1,1,1,1,1,1,1],'9':[1,1,1,1,0,1,1]};");
  f.println("function d7(x,y,d,W,c){var s=S[d]||S['8'],t=Math.max(3,Math.round(W*0.15)),h=t/2,g=Math.max(1,Math.round(t*0.12)),H=2*W-t,m=y+W-h;ctx.fillStyle=c;");
  f.println("function hS(l,r,cy){ctx.beginPath();ctx.moveTo(l,cy);ctx.lineTo(l+h,cy-h);ctx.lineTo(r-h,cy-h);ctx.lineTo(r,cy);ctx.lineTo(r-h,cy+h);ctx.lineTo(l+h,cy+h);ctx.closePath();ctx.fill();}");
  f.println("function vS(cx,t,b){ctx.beginPath();ctx.moveTo(cx,t);ctx.lineTo(cx+h,t+h);ctx.lineTo(cx+h,b-h);ctx.lineTo(cx,b);ctx.lineTo(cx-h,b-h);ctx.lineTo(cx-h,t+h);ctx.closePath();ctx.fill();}");
  f.println("var aY=y+h,gY=m,dY=y+H-h,hL=x+h+g,hR=x+W-h-g,vL=x+h,vR=x+W-h,tT=y+h+g,tB=m-g,bT=m+g,bB=y+H-h-g;");
  f.println("if(s[0])hS(hL,hR,aY);if(s[1])vS(vR,tT,tB);if(s[2])vS(vR,bT,bB);if(s[3])hS(hL,hR,dY);if(s[4])vS(vL,bT,bB);if(s[5])vS(vL,tT,tB);if(s[6])hS(hL,hR,gY);");
  f.println("return W+4;}");
  f.println("function dN(x,y,n,W,c){var s=Math.abs(n).toString().padStart(2,'0');for(var i=0;i<s.length;i++)x+=d7(x,y,s[i],W,c);return x;}");
  f.println("function dD(x,y,W,c){var s=Math.round(W*0.25);ctx.strokeStyle=c;ctx.lineWidth=Math.max(1,Math.round(s*0.2));ctx.beginPath();ctx.arc(x+s/2,y+s/2,s/2-ctx.lineWidth,0,Math.PI*2);ctx.stroke();}");

  // State + draw
  f.println("var st=null,failCount=0;");
  f.println("function drawAll(s){ctx.clearRect(0,0,320,240);");
  f.println("ctx.fillStyle='#FFFF00';ctx.font='16px sans-serif';ctx.fillText('Inside',5,20);");
  f.println("var tc=s.tempColor||'#00FF00';var ex=dN(5,28,s.currentTemp,54,tc);dD(ex+2,28,54,tc);");
  f.println("if(s.heatEnabled){ctx.fillStyle='#FFFF00';ctx.font='14px sans-serif';ctx.fillText('Heat',230,35);ctx.fillText('Setting',220,52);");
  f.println("var tx=dN(218,60,s.targetTemp,36,'#00FF00');dD(tx+2,60,36,'#00FF00');}");
  f.println("if(s.heatEnabled&&s.heatActive){ctx.fillStyle='#FFFF00';ctx.font='12px sans-serif';ctx.fillText('Heat On',140,128);}");
  f.println("ctx.strokeStyle='#FFFF00';ctx.lineWidth=1;");
  f.println("ctx.beginPath();ctx.roundRect(15,145,140,90,5);ctx.stroke();");
  f.println("ctx.beginPath();ctx.roundRect(170,145,140,90,5);ctx.stroke();");
  f.println("ctx.fillStyle='#FFFF00';ctx.font='bold 14px sans-serif';ctx.fillText('Light',65,163);ctx.fillText('System',215,163);");
  f.println("ctx.beginPath();ctx.moveTo(15,170);ctx.lineTo(155,170);ctx.moveTo(170,170);ctx.lineTo(310,170);ctx.stroke();");
  // +/- glyphs removed -- real FocusPro has physical buttons below LCD
  f.println("if(s.heatEnabled){ctx.fillStyle='#FFFF00';ctx.font='14px sans-serif';ctx.fillText('Heat',225,190);}");
  f.println("}");

  // Info cards builder (sample template order)
  f.println("function buildCards(s){var h='';");
  // ENVIRONMENT card
  f.println("if(s.environment){var e=s.environment;h+=\"<div class='card'><div class='label'>ENVIRONMENT (\"+e.sensor+\")</div>\";");
  f.println("if(e.humidity!=null)h+=\"<div class='row'><span>Humidity</span><span class='value'>\"+e.humidity.toFixed(1)+\"%</span></div>\";");
  f.println("if(e.pressInhg!=null)h+=\"<div class='row'><span>Pressure</span><span class='value'>\"+e.pressInhg.toFixed(2)+\" inHg</span></div>\";");
  f.println("if(e.gasKohms!=null)h+=\"<div class='row'><span>Gas</span><span class='value'>\"+e.gasKohms.toFixed(1)+\" kOhms</span></div>\";");
  f.println("if(e.dewPoint!=null)h+=\"<div class='row'><span>Dew Point</span><span class='value'>\"+e.dewPoint.toFixed(1)+\"&deg;F</span></div>\";");
  f.println("if(e.pressureTrend){var tc=(e.pressureTrend.indexOf('Rising')>=0)?'#22c55e':(e.pressureTrend.indexOf('Falling')>=0)?'#ef4444':'#eab308';");
  f.println("h+=\"<div class='row'><span>Trend</span><span class='value' style='color:\"+tc+\"'>\"+e.pressureTrend+\"</span></div>\";}");
  f.println("if(e.weatherPrediction)h+=\"<div class='row'><span>Forecast</span><span class='value'>\"+e.weatherPrediction+\"</span></div>\";");
  f.println("h+=\"</div>\";}");
  // SYSTEM card
  f.println("h+=\"<div class='card'><div class='label'>SYSTEM</div>\";");
  f.println("if(s.board)h+=\"<div class='row'><span>Board</span><span class='value'>\"+s.board+\"</span></div>\";");
  f.println("if(s.network){var n=s.network;");
  f.println("if(n.mode==='dual'){");
  f.println("if(n.ethernet){var e=n.ethernet;");
  f.println("h+=\"<div class='row'><span>Eth IP</span><span class='value'>\"+e.ip+\"</span></div>\";");
  f.println("h+=\"<div class='row'><span>Eth MAC</span><span class='value'>\"+e.mac+\"</span></div>\";");
  f.println("var elc=e.link==='Connected'?'#22c55e':'#ef4444';");
  f.println("h+=\"<div class='row'><span>Eth Link</span><span class='value' style='color:\"+elc+\"'>\"+e.link+\"</span></div>\";}");
  f.println("if(n.wifi){var w=n.wifi;");
  f.println("h+=\"<div class='row'><span>WiFi IP</span><span class='value'>\"+w.ip+\"</span></div>\";");
  f.println("h+=\"<div class='row'><span>WiFi MAC</span><span class='value'>\"+w.mac+\"</span></div>\";");
  f.println("if(w.ssid)h+=\"<div class='row'><span>WiFi SSID</span><span class='value'>\"+w.ssid+\"</span></div>\";");
  f.println("var wlc=w.link==='Connected'?'#22c55e':'#ef4444';");
  f.println("h+=\"<div class='row'><span>WiFi Link</span><span class='value' style='color:\"+wlc+\"'>\"+w.link+\"</span></div>\";}");
  f.println("}else{");
  f.println("h+=\"<div class='row'><span>IP</span><span class='value'>\"+n.ip+\"</span></div>\";");
  f.println("if(n.mac)h+=\"<div class='row'><span>MAC</span><span class='value'>\"+n.mac+\"</span></div>\";");
  f.println("if(n.ssid)h+=\"<div class='row'><span>SSID</span><span class='value'>\"+n.ssid+\"</span></div>\";");
  f.println("var lc=n.link==='Connected'?'#22c55e':'#ef4444';");
  f.println("h+=\"<div class='row'><span>Link</span><span class='value' style='color:\"+lc+\"'>\"+n.link+\"</span></div>\";}}");
  f.println("var u=s.uptime||0,sec=Math.floor(u/1000),m=Math.floor(sec/60)%60,hr=Math.floor(sec/3600)%24,d=Math.floor(sec/86400);");
  f.println("h+=\"<div class='row'><span>Uptime</span><span class='value'>\"+d+'d '+hr+'h '+m+'m'+\"</span></div>\";");
  f.println("var hc=s.heatEnabled?'#22c55e':'#888';");
  f.println("h+=\"<div class='row'><span>Heat</span><span class='value' style='color:\"+hc+\"'>\"+(s.heatEnabled?'Enabled':'Off')+\"</span></div>\";");
  f.println("if(s.heatEnabled&&s.heatActive){h+=\"<div class='row'><span>Furnace</span><span class='value' style='color:#fbbf24;font-weight:bold'>ACTIVE</span></div>\";}");
  f.println("if(s.sdCard!=null){var sdc=s.sdCard?'#22c55e':'#888';var sdtxt=s.sdCard?(s.sdFormat||'Present'):'None';");
  f.println("h+=\"<div class='row'><span>SD Card</span><span class='value' style='color:\"+sdc+\"'>\"+sdtxt+\"</span></div>\";}");
  f.println("if(s.displayPresent!=null){var dc=s.displayPresent?'#22c55e':'#888';");
  f.println("h+=\"<div class='row'><span>Display</span><span class='value' style='color:\"+dc+\"'>\"+(s.displayPresent?'Connected':'Headless')+\"</span></div>\";}");
  f.println("if(s.otaSupported)h+=\"<div class='row'><span>OTA Update</span><span class='value' style='color:#22c55e'>Supported</span></div>\";");
  f.println("if(s.network&&s.network.battery){var bv=s.network.battery;var bc=(bv>3.5)?'#22c55e':(bv<=3.3)?'#ef4444':'#eab308';");
  f.println("h+=\"<div class='row'><span>Battery</span><span class='value' style='color:\"+bc+\"'>\"+bv.toFixed(2)+\"V</span></div>\";}");
  f.println("if(s.freeRam)h+=\"<div class='row'><span>Free RAM</span><span class='value'>\"+(s.freeRam/1024).toFixed(1)+\" KB</span></div>\";");
  f.println("h+=\"</div>\";");
  // OUTDOOR UNIT card
  f.println("if(s.outdoor&&s.outdoor.configured){var o=s.outdoor;h+=\"<div class='card'><div class='label'>OUTDOOR UNIT</div>\";");
  f.println("var sc=o.status==='Online'?'#22c55e':o.status==='Offline'?'#ef4444':'#888';");
  f.println("h+=\"<div class='row'><span>Status</span><span style='color:\"+sc+\"'>\"+o.status+\"</span></div>\";");
  f.println("h+=\"<div class='row'><span>IP</span><span class='value'>\"+o.ip+\":\"+o.port+\"</span></div>\";");
  f.println("if(o.status==='Online'){");
  f.println("if(o.temp)h+=\"<div class='row'><span>Temp</span><span class='value'>\"+o.temp+\"&deg;F</span></div>\";");
  f.println("if(o.humidity)h+=\"<div class='row'><span>Humidity</span><span class='value'>\"+o.humidity+\"%</span></div>\";");
  f.println("if(o.pressInhg)h+=\"<div class='row'><span>Pressure</span><span class='value'>\"+o.pressInhg+\" inHg</span></div>\";");
  f.println("if(o.gasKohms)h+=\"<div class='row'><span>Gas</span><span class='value'>\"+o.gasKohms+\" kOhms</span></div>\";}");
  f.println("h+=\"</div>\";}");
  f.println("document.getElementById('cards').innerHTML=h;}");

  // Fetch + controls
  f.println("function fetchState(){fetch('/api/state').then(function(r){return r.json();}).then(function(s){");
  f.println("st=s;failCount=0;drawAll(s);buildCards(s);");
  f.println("document.getElementById('status').className='online';document.getElementById('statusText').textContent='Connected';");
  f.println("setTimeout(fetchState,1000);");  // Schedule next poll after completion
  f.println("}).catch(function(e){failCount++;");
  f.println("if(failCount>=3){document.getElementById('status').className='offline';document.getElementById('statusText').textContent='Disconnected';}");
  f.println("else{document.getElementById('status').className='stale';document.getElementById('statusText').textContent='Retrying...';}");
  f.println("setTimeout(fetchState,2000);});}");  // Retry after 2s on error
  f.println("function sendCmd(q){fetch('/api/set?'+q).then(function(){fetchState();});}");
  f.println("function adjTemp(d){if(!st)return;sendCmd('target='+(st.targetTemp+d));}");
  f.println("function handleClick(e){if(!st)return;e.preventDefault();var r=cv.getBoundingClientRect();var cx=e.clientX,cy=e.clientY;");
  f.println("if(e.touches&&e.touches.length>0){cx=e.touches[0].clientX;cy=e.touches[0].clientY;}");
  f.println("var x=(cx-r.left)*(320/r.width);var y=(cy-r.top)*(240/r.height);");
  // If settings overlay is showing, any click cycles/dismisses it
  f.println("if(st.settingsScreen&&st.settingsScreen!=='off'){sendCmd('settings=toggle');return;}");
  // Hidden settings toggle: top-left area (matches physical TFT touch zone)
  f.println("if(x>=15&&x<=155&&y<145){sendCmd('settings=toggle');return;}");
  f.println("if(x>=15&&x<=155&&y>=170&&y<=235){sendCmd('target='+(st.targetTemp-1));return;}");
  f.println("if(x>=170&&x<=310&&y>=170&&y<=235){sendCmd('target='+(st.targetTemp+1));return;}");
  // Hidden heat toggle: top-right area (matches physical TFT touch zone)
  f.println("if(x>=170&&x<=310&&y<145){sendCmd('heat='+(st.heatEnabled?'off':'on'));return;}");
  // Header areas also work for convenience
  f.println("if(x>=15&&x<=155&&y>=145&&y<=170){sendCmd('settings=toggle');return;}");
  f.println("if(x>=170&&x<=310&&y>=145&&y<=170){sendCmd('heat='+(st.heatEnabled?'off':'on'));return;}}");
  f.println("cv.addEventListener('click',handleClick);cv.addEventListener('touchstart',handleClick);");

  // Seed initial state from Smarty variables and start polling
  f.println("st={currentTemp:seedTemp,targetTemp:seedTarget,heatEnabled:seedHeatOn==1,heatActive:seedHeatActive==1,tempColor:seedColor};");
  f.println("drawAll(st);fetchState();");  // Start polling (setTimeout chains from within)

  f.println("</script>{/literal}");
  f.println("</body></html>");

  f.close();
  Serial.println("generateIndexHtml: wrote index.html to SD card");
}

// Serve index.html from SD card with Smarty template processing
void sendPageFromSD(HardwareClient cl) {
  file_t f;
  if (!f.open("index.html", O_RDONLY)) {
    sendPage(cl);
    return;
  }

  tplReset();

  // Read and process line-by-line
  char buf[257];
  while (f.available()) {
    int len = f.fgets(buf, sizeof(buf));
    if (len <= 0) break;
    String line = String(buf);
    processTemplateLine(line, cl);
  }
  f.close();
  CLIENT_CLEAR(cl);
}

// ---------------------------------------------------------------------------
// Shared CSS/HTML helper functions to reduce code duplication
// ---------------------------------------------------------------------------

// Common page header with dark theme base styles
void sendHtmlHead(HardwareClient& cl, const char* title) {
  cl.println(F("<!DOCTYPE html><html><head>"));
  cl.print(F("<meta charset='UTF-8'><meta name='viewport' content='width=device-width,initial-scale=1'><title>"));
  cl.print(title);
  cl.println(F("</title><style>"));
  cl.println(F("*{margin:0;padding:0;box-sizing:border-box}"));
  cl.println(F("body{font-family:-apple-system,BlinkMacSystemFont,sans-serif;background:#000;color:#eee;min-height:100vh}"));
}

// Button styles (shared across multiple pages)
void sendButtonCSS(HardwareClient& cl) {
  cl.println(F(".btn{padding:8px 16px;font-size:13px;border:none;border-radius:6px;cursor:pointer;transition:all 0.2s;font-weight:500}"));
  cl.println(F(".btn-primary{background:#4ade80;color:#000}.btn-primary:hover{background:#22c55e}"));
  cl.println(F(".btn-danger{background:#ef4444;color:#fff}.btn-danger:hover{background:#dc2626}"));
  cl.println(F(".btn-secondary{background:#64748b;color:#fff}.btn-secondary:hover{background:#475569}"));
  cl.println(F(".btn-warning{background:#f59e0b;color:#000}.btn-warning:hover{background:#d97706}"));
  cl.println(F(".btn:disabled{opacity:0.5;cursor:not-allowed}"));
}

// Modal dialog styles (shared by SD files page and potentially others)
void sendModalCSS(HardwareClient& cl) {
  cl.println(F(".modal{display:none;position:fixed;top:0;left:0;right:0;bottom:0;background:rgba(0,0,0,0.8);justify-content:center;align-items:center;z-index:1000;padding:20px}"));
  cl.println(F(".modal.active{display:flex}"));
  cl.println(F(".modal-content{background:#1e293b;border-radius:12px;width:100%;max-width:700px;max-height:90vh;display:flex;flex-direction:column}"));
  cl.println(F(".modal-header{padding:15px 20px;border-bottom:1px solid rgba(255,255,255,0.1);display:flex;justify-content:space-between;align-items:center}"));
  cl.println(F(".modal-header h2{font-size:16px;font-weight:500}"));
  cl.println(F(".modal-close{background:none;border:none;color:#fff;font-size:24px;cursor:pointer;opacity:0.7}"));
  cl.println(F(".modal-close:hover{opacity:1}"));
  cl.println(F(".modal-body{padding:20px;flex-grow:1;overflow:auto}"));
  cl.println(F(".modal-footer{padding:15px 20px;border-top:1px solid rgba(255,255,255,0.1);display:flex;justify-content:flex-end;gap:10px}"));
}

// Form input styles (shared across config pages)
void sendFormCSS(HardwareClient& cl) {
  cl.println(F(".input-group{margin-bottom:15px}"));
  cl.println(F(".input-group label{display:block;margin-bottom:5px;font-size:13px;color:#94a3b8}"));
  cl.println(F(".input-group input{width:100%;padding:10px;background:#0f172a;border:1px solid #334155;border-radius:6px;color:#fff;font-size:14px}"));
  cl.println(F(".input-group input:focus{outline:none;border-color:#60a5fa}"));
}

// Status message styles
void sendStatusCSS(HardwareClient& cl) {
  cl.println(F(".status{margin-top:10px;padding:10px;border-radius:6px;font-size:13px}"));
  cl.println(F(".status.error{background:rgba(239,68,68,0.2);color:#fca5a5}"));
  cl.println(F(".status.success{background:rgba(74,222,128,0.2);color:#86efac}"));
}

// Card container styles
void sendCardCSS(HardwareClient& cl) {
  cl.println(F(".card{background:rgba(255,255,255,0.08);border-radius:10px;padding:15px;margin-bottom:15px}"));
}

// Progress bar styles (for uploads)
void sendProgressCSS(HardwareClient& cl) {
  cl.println(F(".progress{display:none;margin:20px 0}"));
  cl.println(F(".progress-bar{height:20px;background:#333;border-radius:10px;overflow:hidden}"));
  cl.println(F(".progress-fill{height:100%;background:#4CAF50;width:0%;transition:width 0.3s}"));
}

// Footer links (back to main page)
void sendFooterLinks(HardwareClient& cl, bool showConfig, bool showSDFiles, bool showUpdate) {
  cl.println(F("<div style='text-align:center;margin-top:15px'>"));
  cl.println(F("<a href='/' style='color:#60a5fa;font-size:12px'>&#127968; Home</a>"));
  if (showConfig) {
    cl.println(F(" &middot; <a href='/config' style='color:#4ade80;font-size:12px'>&#9881; Config</a>"));
  }
  if (showSDFiles && sdCardPresent) {
    cl.println(F(" &middot; <a href='/sdfiles' style='color:#fbbf24;font-size:12px'>&#128193; SD Files</a>"));
  }
  if (showUpdate) {
    cl.println(F(" &middot; <a href='/update' style='color:#888;font-size:12px'>&#128295; Update</a>"));
  }
  cl.println(F("</div>"));
}

// ---------------------------------------------------------------------------
// Firmware-embedded fallback page (used when SD card has no index.html)
// ---------------------------------------------------------------------------
// Live canvas -- polls /api/state every 5s, interactive buttons,
//         dynamic environment cards, outdoor unit display.
// ---------------------------------------------------------------------------

void sendPage(HardwareClient cl) {
  cl.println(F("<html><head><title>Thermostat</title>"));
  cl.println(F("<meta name='viewport' content='width=device-width,initial-scale=1'><style>"));
  cl.println(F("body{font-family:Arial,sans-serif;background:#000;color:#eee;margin:20px}"));
  cl.println(F(".container{max-width:420px;margin:0 auto}"));
  cl.println(F(".card{background:#16213e;border-radius:10px;padding:15px;margin:10px 0;min-width:200px}"));
  cl.println(F(".label{color:#888;font-size:0.9em;margin-bottom:6px}"));
  cl.println(F(".row{display:flex;justify-content:space-between;margin:8px 0}.value{color:#60a5fa}"));
  cl.println(F(".heat-on{color:#fbbf24;font-weight:bold}"));
  cl.println(F("#status{display:inline-block;width:8px;height:8px;border-radius:50%;margin-right:6px}"));
  cl.println(F(".online{background:#22c55e}.offline{background:#ef4444}.stale{background:#eab308}"));
  cl.println(F("canvas{cursor:pointer}</style></head><body>"));
  cl.println(F("<div class='container'>"));
  // Canvas + arrows row
  cl.println(F("<div style='display:flex;align-items:center;gap:10px'>"));
  cl.println(F("<canvas id='cv' width='320' height='240' style='background:#000;border-radius:4px'></canvas>"));
  cl.println(F("<div style='display:flex;flex-direction:column;gap:8px'>"));
  cl.println(F("<button id='btnUp' onclick='adjTemp(1)' style='width:50px;height:70px;font-size:28px;background:#222;color:#0a0;border:2px solid #444;border-radius:6px;cursor:pointer'>&#9650;</button>"));
  cl.println(F("<button id='btnDown' onclick='adjTemp(-1)' style='width:50px;height:70px;font-size:28px;background:#222;color:#0a0;border:2px solid #444;border-radius:6px;cursor:pointer'>&#9660;</button>"));
  cl.println(F("</div></div>"));
  // Status line
  cl.println(F("<div style='margin-top:4px;font-size:11px;color:#555'>"));
  cl.println(F("<span id='status' class='online'></span><span id='statusText'>Connected</span></div>"));
  // Environment cards (below canvas)
  cl.println(F("<div id='cards'></div></div>"));
  // Footer links
  cl.println(F("<div style='text-align:center;margin-top:10px'>"));
  cl.println(F("<a href='/config' style='color:#4ade80;font-size:12px'>&#9881; Configuration</a>"));
  if (sdCardPresent) {
    cl.println(F(" &middot; <a href='/sdfiles' style='color:#fbbf24;font-size:12px'>&#128193; SD Files</a>"));
  }
  cl.println(F(" &middot; <a href='/update' style='color:#888;font-size:12px'>&#128295; Firmware Update</a>"));
  cl.println(F("</div><script>"));
  // Shorthand: $('id') = document.getElementById('id')
  cl.println(F("function $(i){return document.getElementById(i)}"));

  // --- 7-segment renderer (minified) ---
  cl.println(F("var cv=$('cv'),ctx=cv.getContext('2d');"));
  cl.println(F("var S={'0':[1,1,1,1,1,1,0],'1':[0,1,1,0,0,0,0],'2':[1,1,0,1,1,0,1],'3':[1,1,1,1,0,0,1],'4':[0,1,1,0,0,1,1],'5':[1,0,1,1,0,1,1],'6':[1,0,1,1,1,1,1],'7':[1,1,1,0,0,0,0],'8':[1,1,1,1,1,1,1],'9':[1,1,1,1,0,1,1]};"));
  cl.println(F("function d7(x,y,d,W,c){var s=S[d]||S['8'],t=Math.max(3,Math.round(W*0.15)),h=t/2,g=Math.max(1,Math.round(t*0.12)),H=2*W-t,m=y+W-h;ctx.fillStyle=c;"));
  cl.println(F("function hS(l,r,cy){ctx.beginPath();ctx.moveTo(l,cy);ctx.lineTo(l+h,cy-h);ctx.lineTo(r-h,cy-h);ctx.lineTo(r,cy);ctx.lineTo(r-h,cy+h);ctx.lineTo(l+h,cy+h);ctx.closePath();ctx.fill();}"));
  cl.println(F("function vS(cx,t,b){ctx.beginPath();ctx.moveTo(cx,t);ctx.lineTo(cx+h,t+h);ctx.lineTo(cx+h,b-h);ctx.lineTo(cx,b);ctx.lineTo(cx-h,b-h);ctx.lineTo(cx-h,t+h);ctx.closePath();ctx.fill();}"));
  cl.println(F("var aY=y+h,gY=m,dY=y+H-h,hL=x+h+g,hR=x+W-h-g,vL=x+h,vR=x+W-h,tT=y+h+g,tB=m-g,bT=m+g,bB=y+H-h-g;if(s[0])hS(hL,hR,aY);if(s[1])vS(vR,tT,tB);if(s[2])vS(vR,bT,bB);if(s[3])hS(hL,hR,dY);if(s[4])vS(vL,bT,bB);if(s[5])vS(vL,tT,tB);if(s[6])hS(hL,hR,gY);return W+4;}"));

  cl.println(F("function dN(x,y,n,W,c){var s=Math.abs(n).toString().padStart(2,'0');for(var i=0;i<s.length;i++)x+=d7(x,y,s[i],W,c);return x;}"));
  cl.println(F("function dD(x,y,W,c){var s=Math.round(W*0.25);ctx.strokeStyle=c;ctx.lineWidth=Math.max(1,Math.round(s*0.2));ctx.beginPath();ctx.arc(x+s/2,y+s/2,s/2-ctx.lineWidth,0,Math.PI*2);ctx.stroke();}"));
  cl.println(F("var st=null,failCount=0;function drawAll(s){ctx.clearRect(0,0,320,240);"));
  cl.println(F("ctx.fillStyle='#FFFF00';ctx.font='16px sans-serif';ctx.fillText('Inside',5,20);var tc=s.tempColor||'#00FF00';var ex=dN(5,28,s.currentTemp,54,tc);dD(ex+2,28,54,tc);"));
  cl.println(F("if(s.heatEnabled){ctx.fillStyle='#FFFF00';ctx.font='14px sans-serif';ctx.fillText('Heat',230,35);ctx.fillText('Setting',220,52);var tx=dN(218,60,s.targetTemp,36,'#00FF00');dD(tx+2,60,36,'#00FF00');}"));
  cl.println(F("if(s.heatEnabled&&s.heatActive){ctx.fillStyle='#FFFF00';ctx.font='12px sans-serif';ctx.fillText('Heat On',140,128);}"));
  cl.println(F("ctx.strokeStyle='#FFFF00';ctx.lineWidth=1;ctx.beginPath();ctx.roundRect(15,145,140,90,5);ctx.stroke();ctx.beginPath();ctx.roundRect(170,145,140,90,5);ctx.stroke();"));
  cl.println(F("ctx.fillStyle='#FFFF00';ctx.font='bold 14px sans-serif';ctx.fillText('Light',65,163);ctx.fillText('System',215,163);ctx.beginPath();ctx.moveTo(15,170);ctx.lineTo(155,170);ctx.moveTo(170,170);ctx.lineTo(310,170);ctx.stroke();"));
  cl.println(F("if(s.heatEnabled){ctx.fillStyle='#FFFF00';ctx.font='14px sans-serif';ctx.fillText('Heat',225,190);}"));

  // Settings overlay - LAN=yellow, WiFi=blue matching TFT
  cl.println(F("var scr=s.settingsScreen||'off';if(scr!=='off'){var a=(scr==='lan')?'#FF0':'#60a5fa',v=80,y=46,l=16;"));
  cl.println(F("ctx.fillStyle='rgba(0,0,0,.95)';ctx.fillRect(6,3,308,234);ctx.strokeStyle=a;ctx.lineWidth=2;ctx.strokeRect(6,3,308,234);ctx.strokeRect(8,5,304,230);"));
  cl.println(F("ctx.fillStyle=a;ctx.font='bold 14px sans-serif';ctx.fillText('Network Settings',14,22);ctx.font='12px sans-serif';ctx.fillText((scr==='lan')?'Ethernet [1/2]':'WiFi [2/2]',210,20);"));
  cl.println(F("ctx.beginPath();ctx.moveTo(14,28);ctx.lineTo(306,28);ctx.stroke();if(s.network){var net=s.network,n=(net.mode==='dual')?((scr==='lan')?net.ethernet:net.wifi):net;"));
  cl.println(F("if(scr==='lan'){ctx.fillStyle='#fff';ctx.fillText('IP:',14,y);ctx.fillStyle='#0f0';ctx.fillText(n.ip||'-',v,y);y+=l;"));
  cl.println(F("ctx.fillStyle='#fff';ctx.fillText('Subnet:',14,y);ctx.fillStyle='#0f0';ctx.fillText(n.subnet||'-',v,y);y+=l;ctx.fillStyle='#fff';ctx.fillText('Gateway:',14,y);ctx.fillStyle='#0f0';ctx.fillText(n.gateway||'-',v,y);y+=l;"));
  cl.println(F("if(n.dns){ctx.fillStyle='#fff';ctx.fillText('DNS:',14,y);ctx.fillStyle='#0f0';ctx.fillText(n.dns,v,y);y+=l;}ctx.fillStyle='#fff';ctx.fillText('MAC:',14,y);ctx.fillStyle='#0f0';ctx.fillText(n.mac||'-',v,y);y+=l;"));
  cl.println(F("ctx.fillStyle='#fff';ctx.fillText('Link:',14,y);ctx.fillStyle=(n.link==='Connected')?'#0f0':'#f00';ctx.fillText(n.link||'-',v,y);y+=l;}else{"));
  // WiFi overlay - two column layout matching TFT
  cl.println(F("var c2=165,v2=220;ctx.fillStyle='#aaa';ctx.fillText('SSID:',14,y);ctx.fillStyle='#fff';ctx.fillText(n.ssid||'-',v,y);y+=l;"));
  cl.println(F("ctx.fillStyle='#aaa';ctx.fillText('IP:',14,y);ctx.fillStyle='#fff';ctx.fillText(n.ip||'-',v,y);ctx.fillStyle='#aaa';ctx.fillText('Status:',c2,y);ctx.fillStyle=(n.link==='Connected')?'#0f0':'#f00';ctx.fillText(n.link||'-',v2,y);y+=l;"));
  cl.println(F("ctx.fillStyle='#aaa';ctx.fillText('Subnet:',14,y);ctx.fillStyle='#fff';ctx.fillText(n.subnet||'-',v,y);if(net.battery){var bv=net.battery;ctx.fillStyle='#aaa';ctx.fillText('Battery:',c2,y);ctx.fillStyle=(bv>3.5)?'#0f0':(bv<=3.3)?'#f00':'#ff0';ctx.fillText(bv.toFixed(2)+'V',v2,y);}y+=l;"));
  cl.println(F("ctx.fillStyle='#aaa';ctx.fillText('Gateway:',14,y);ctx.fillStyle='#fff';ctx.fillText(n.gateway||'-',v,y);y+=l;ctx.fillStyle='#aaa';ctx.fillText('MAC:',14,y);ctx.fillStyle='#fff';ctx.fillText(n.mac||'-',v,y);y+=l;"));
  cl.println(F("if(n.security){ctx.fillStyle='#aaa';ctx.fillText('Security:',14,y);ctx.fillStyle='#fff';ctx.fillText(n.security,v,y);}if(n.rssi!=null){ctx.fillStyle='#aaa';ctx.fillText('RSSI:',c2,y);var rs=n.rssi;ctx.fillStyle=(rs>-50)?'#0f0':(rs>-70)?'#ff0':'#f00';ctx.fillText(rs+' dBm',v2,y);}y+=l;}}"));
  cl.println(F("y+=6;ctx.strokeStyle='#444';ctx.beginPath();ctx.moveTo(14,y);ctx.lineTo(306,y);ctx.stroke();y+=14;ctx.fillStyle=a;ctx.font='bold 12px sans-serif';ctx.fillText('Outdoor Unit',14,y);y+=l;ctx.font='12px sans-serif';"));
  cl.println(F("if(s.outdoor&&s.outdoor.configured){var o=s.outdoor;ctx.fillStyle='#fff';ctx.fillText('Remote:',14,y);ctx.fillStyle='#0f0';ctx.fillText(o.ip+':'+o.port,v,y);y+=l;"));
  cl.println(F("ctx.fillStyle='#fff';ctx.fillText('Status:',14,y);ctx.fillStyle=(o.status==='Online')?'#0f0':'#f00';ctx.fillText(o.status,v,y);}else{ctx.fillStyle='#888';ctx.fillText('Not configured',v,y);}"));
  cl.println(F("ctx.fillStyle=a;ctx.font='11px sans-serif';ctx.fillText((scr==='lan')?'Tap for WiFi':'Tap anywhere to close',90,228);}}")); // end showSettings + drawAll

  // R(label,value,color) helper + buildCards with helper
  cl.println(F("function R(l,v,c){return\"<div class='row'><span>\"+l+\"</span><span class='value'\"+(c?\" style='color:\"+c+\"'\":'')+\">\"+v+\"</span></div>\";}"));
  cl.println(F("function buildCards(s){var h='';if(s.environment){var e=s.environment;h+=\"<div class='card'><div class='label'>ENVIRONMENT (\"+e.sensor+\")</div>\";"));
  cl.println(F("if(e.humidity!=null)h+=R('Humidity',e.humidity.toFixed(1)+'%');if(e.pressInhg!=null)h+=R('Pressure',e.pressInhg.toFixed(2)+' inHg');"));
  cl.println(F("if(e.gasKohms!=null)h+=R('Gas',e.gasKohms.toFixed(1)+' kOhms');if(e.dewPoint!=null)h+=R('Dew Point',e.dewPoint.toFixed(1)+'&deg;F');"));
  cl.println(F("if(e.pressureTrend){var tc=e.pressureTrend.indexOf('Rising')>=0?'#22c55e':e.pressureTrend.indexOf('Falling')>=0?'#ef4444':'#eab308';h+=R('Trend',e.pressureTrend,tc);}"));
  cl.println(F("if(e.weatherPrediction)h+=R('Forecast',e.weatherPrediction);h+=\"</div>\";}"));
  cl.println(F("h+=\"<div class='card'><div class='label'>SYSTEM</div>\";if(s.board)h+=R('Board',s.board);"));
  cl.println(F("if(s.network){var n=s.network;if(n.mode==='dual'){if(n.ethernet){var e=n.ethernet;h+=R('Eth IP',e.ip)+R('Eth MAC',e.mac)+R('Eth Link',e.link,e.link==='Connected'?'#22c55e':'#ef4444');}"));
  cl.println(F("if(n.wifi){var w=n.wifi;h+=R('WiFi IP',w.ip)+R('WiFi MAC',w.mac);if(w.ssid)h+=R('WiFi SSID',w.ssid);h+=R('WiFi Link',w.link,w.link==='Connected'?'#22c55e':'#ef4444');}}else{"));
  cl.println(F("h+=R('IP',n.ip);if(n.mac)h+=R('MAC',n.mac);if(n.ssid)h+=R('SSID',n.ssid);h+=R('Link',n.link,n.link==='Connected'?'#22c55e':'#ef4444');}}"));
  cl.println(F("var u=s.uptime||0,sec=Math.floor(u/1000),m=Math.floor(sec/60)%60,hr=Math.floor(sec/3600)%24,d=Math.floor(sec/86400);h+=R('Uptime',d+'d '+hr+'h '+m+'m');"));
  cl.println(F("h+=R('Heat',s.heatEnabled?'Enabled':'Off',s.heatEnabled?'#22c55e':'#888');if(s.heatEnabled&&s.heatActive)h+=R('Furnace','ACTIVE','#fbbf24');"));
  cl.println(F("if(s.sdCard!=null)h+=R('SD Card',s.sdCard?(s.sdFormat||'Present'):'None',s.sdCard?'#22c55e':'#888');"));
  cl.println(F("if(s.displayPresent!=null)h+=R('Display',s.displayPresent?'Connected':'Headless',s.displayPresent?'#22c55e':'#888');"));
  cl.println(F("if(s.otaSupported)h+=R('OTA Update','Supported','#22c55e');"));
  cl.println(F("if(s.network&&s.network.battery){var bv=s.network.battery;h+=R('Battery',bv.toFixed(2)+'V',bv>3.5?'#22c55e':bv<=3.3?'#ef4444':'#eab308');}"));
  cl.println(F("if(s.freeRam)h+=R('Free RAM',(s.freeRam/1024).toFixed(1)+' KB');h+=\"</div>\";"));
  cl.println(F("if(s.outdoor&&s.outdoor.configured){var o=s.outdoor;h+=\"<div class='card'><div class='label'>OUTDOOR UNIT</div>\";"));
  cl.println(F("h+=R('Status',o.status,o.status==='Online'?'#22c55e':o.status==='Offline'?'#ef4444':'#888')+R('IP',o.ip+':'+o.port);"));
  cl.println(F("if(o.status==='Online'){if(o.temp)h+=R('Temp',o.temp+'&deg;F');if(o.humidity)h+=R('Humidity',o.humidity+'%');"));
  cl.println(F("if(o.pressInhg)h+=R('Pressure',o.pressInhg+' inHg');if(o.gasKohms)h+=R('Gas',o.gasKohms+' kOhms');}h+=\"</div>\";}"));
  cl.println(F("$('cards').innerHTML=h;}"));
  // --- Fetch network data (only when settings overlay shown) ---
  cl.println(F("var netData=null;function fetchNet(cb){"));
  cl.println(F("fetch('/api/network').then(r=>r.json()).then(n=>{netData=n;st.network=n;if(cb)cb();}).catch(()=>{if(cb)cb();});}"));
  // --- Initial full state fetch (populates cards, then starts polling) ---
  cl.println(F("function fetchFull(){fetch('/api/state').then(r=>r.json()).then(s=>{"));
  cl.println(F("st=s;drawAll(s);buildCards(s);$('status').className='online';$('statusText').textContent='Connected';setTimeout(poll,1000);"));
  cl.println(F("}).catch(()=>setTimeout(fetchFull,2000));}"));
  // --- Lightweight poll (just temp/heat/settings) ---
  cl.println(F("var lastScr='off';function poll(){fetch('/api/poll').then(r=>r.json()).then(p=>{failCount=0;"));
  cl.println(F("st.currentTemp=p.t;st.targetTemp=p.g;st.heatEnabled=p.h==1;st.heatActive=p.a==1;st.settingsScreen=p.s;"));
  cl.println(F("if(p.s!==lastScr){lastScr=p.s;if(p.s!=='off')fetchNet(()=>drawAll(st));else drawAll(st);}else drawAll(st);"));
  cl.println(F("$('status').className='online';$('statusText').textContent='Connected';setTimeout(poll,1000);"));
  cl.println(F("}).catch(e=>{failCount++;if(failCount>=3){$('status').className='offline';$('statusText').textContent='Disconnected';}"));
  cl.println(F("else{$('status').className='stale';$('statusText').textContent='Retrying...';}setTimeout(poll,2000);});}"));
  // --- Send command to /api/set ---
  cl.println(F("function sendCmd(q){fetch('/api/set?'+q).then(()=>poll());}"));
  // --- Adjust temperature via arrow buttons ---
  cl.println(F("function adjTemp(d){if(!st)return;sendCmd('target='+(st.targetTemp+d));}"));

  // --- Canvas click/touch handler (supports both mouse and touch) ---
  cl.println("function handleClick(e){");
  cl.println("if(!st)return;");  // Guard against null state
  cl.println("e.preventDefault();");
  cl.println("var r=cv.getBoundingClientRect();");
  cl.println("var cx=e.clientX,cy=e.clientY;");
  cl.println("if(e.touches&&e.touches.length>0){cx=e.touches[0].clientX;cy=e.touches[0].clientY;}");
  cl.println("var x=(cx-r.left)*(320/r.width);");
  cl.println("var y=(cy-r.top)*(240/r.height);");
  // If settings overlay is showing, any click cycles/dismisses it
  cl.println("if(st.settingsScreen&&st.settingsScreen!=='off'){sendCmd('settings=toggle');return;}");
  // Hidden settings toggle: top-left area (matches physical TFT touch zone)
  cl.println("if(x>=15&&x<=155&&y<145){sendCmd('settings=toggle');return;}");
  // Light minus: box (15,145)-(155,235), below header
  cl.println("if(x>=15&&x<=155&&y>=170&&y<=235){sendCmd('target='+(st.targetTemp-1));return;}");
  // System plus: box (170,145)-(310,235), below divider
  cl.println("if(x>=170&&x<=310&&y>=170&&y<=235){sendCmd('target='+(st.targetTemp+1));return;}");
  // Hidden heat toggle: top-right area (matches physical TFT touch zone)
  cl.println("if(x>=170&&x<=310&&y<145){sendCmd('heat='+(st.heatEnabled?'off':'on'));return;}");
  // Light header area (also settings toggle for convenience)
  cl.println("if(x>=15&&x<=155&&y>=145&&y<=170){sendCmd('settings=toggle');return;}");
  // System header area (also heat toggle for convenience)
  cl.println("if(x>=170&&x<=310&&y>=145&&y<=170){sendCmd('heat='+(st.heatEnabled?'off':'on'));return;}");
  cl.println("}");
  cl.println("cv.addEventListener('click',handleClick);");
  cl.println("cv.addEventListener('touchstart',handleClick);");

  // --- Seed initial state from server-side values ---
  cl.print("st={currentTemp:");
  cl.print(myround(curIndoorTemp));
  cl.print(",targetTemp:");
  cl.print(targIndoorTemp);
  cl.print(",heatEnabled:");
  cl.print(isHeatOn ? "true" : "false");
  cl.print(",heatActive:");
  cl.print(isCurHeatOn ? "true" : "false");
  cl.print(",settingsScreen:'");
  const char* settingsStatesJS[] = {"off", "lan", "wifi"};
  cl.print(settingsStatesJS[settingsScreen]);
  cl.print("',tempColor:'");
  cl.print(tempColorCSS());
  cl.println("'};");
  cl.println("drawAll(st);");

  // --- Start polling (full fetch first, then lightweight poll) ---
  cl.println(F("fetchFull();"));
  cl.println(F("</script></body></html>"));
  CLIENT_CLEAR(cl);
}

// ===========================================================================
// JSON Endpoints -- /outdoorjson and /indoorjson
// ===========================================================================
// Returns this unit's sensor data as JSON.
// /outdoorjson: OUTDOOR_ prefixed keys (for a master to consume)
// /indoorjson:  INDOOR_ prefixed keys (raw data dump / diagnostics)
// Works regardless of SD card or TFT presence.

void sendSensorJson(HardwareClient cl, bool asOutdoor) {
  const char *pfx = asOutdoor ? "OUTDOOR_" : "INDOOR_";
  String val;

  cl.print("{");

  // Temperature (raw decimal)
  cl.print("\""); cl.print(pfx); cl.print("TEMP\":\"");
  resolveVar("CURRENT_TEMP_RAW", val);
  cl.print(val);

  // Temperature color
  cl.print("\",\""); cl.print(pfx); cl.print("TEMP_COLOR\":\"");
  resolveVar("TEMP_COLOR", val);
  cl.print(val);

  // Humidity
  cl.print("\",\""); cl.print(pfx); cl.print("HUMIDITY\":\"");
  resolveVar("HUMIDITY", val);
  cl.print(val);

  // Pressure (hPa)
  cl.print("\",\""); cl.print(pfx); cl.print("PRESSURE_HPA\":\"");
  resolveVar("PRESSURE_HPA", val);
  cl.print(val);

  // Pressure (inHg)
  cl.print("\",\""); cl.print(pfx); cl.print("PRESSURE_INHG\":\"");
  resolveVar("PRESSURE_INHG", val);
  cl.print(val);

  // Gas resistance (BME680 only, "" otherwise)
  cl.print("\",\""); cl.print(pfx); cl.print("GAS_KOHMS\":\"");
  resolveVar("GAS_KOHMS", val);
  cl.print(val);

  // BME sensor type
  cl.print("\",\""); cl.print(pfx); cl.print("BME_TYPE\":\"");
  resolveVar("BME_PRESENT", val);
  cl.print(val);

  // Board identification
  cl.print("\",\""); cl.print(pfx); cl.print("BOARD\":\"");
  cl.print(curBoard);

  // Network capabilities
  cl.print("\",\""); cl.print(pfx); cl.print("HAS_ETHERNET\":\"");
#if defined(W5500) || defined(DUAL_NETWORK)
  cl.print("1");
#else
  cl.print("0");
#endif

  cl.print("\",\""); cl.print(pfx); cl.print("HAS_WIFI\":\"");
#if defined(HAS_WIFI)
  cl.print("1");
#else
  cl.print("0");
#endif

  // Network interface type
  cl.print("\",\""); cl.print(pfx); cl.print("IFACE\":\"");
#if defined(DUAL_NETWORK)
  cl.print("dual");
#elif defined(W5500)
  cl.print("ethernet");
#elif defined(HAS_WIFI)
  cl.print("wifi");
#else
  cl.print("none");
#endif

  // IP addresses (interface-specific)
#if defined(W5500) || defined(DUAL_NETWORK)
  cl.print("\",\""); cl.print(pfx); cl.print("ETH_IP\":\"");
  cl.print(IpAddress2String(Ethernet.localIP()));
#endif

#if defined(HAS_WIFI)
  cl.print("\",\""); cl.print(pfx); cl.print("WIFI_IP\":\"");
  cl.print(IpAddress2String(WiFi.localIP()));
#endif

  // Uptime (seconds)
  cl.print("\",\""); cl.print(pfx); cl.print("UPTIME\":\"");
  resolveVar("UPTIME", val);
  cl.print(val);

  // Battery voltage (boards with LiPo monitoring)
  cl.print("\",\""); cl.print(pfx); cl.print("BATVOLTS\":\"");
#if defined(HAS_BATT)
  cl.print(String(readBatteryVoltage(), 2));
#else
  cl.print("");  // Empty for Ethernet-only or Dual Network
#endif

  cl.println("\"}");
  CLIENT_CLEAR(cl);
}

// ===========================================================================
// Device Config -- SD Card (device.cfg)
// ===========================================================================
// File format:  key=value pairs, one per line.  # comments allowed.
//
// Ethernet settings (W5500 or DUAL_NETWORK):
//   ETH_CFG_IP=10.1.0.71      (or DHCP; 0.0.0.0 or omit = use DHCP)
//   ETH_DNS=10.1.0.1
//   ETH_SUBNET=255.255.255.0
//   ETH_GATEWAY=10.1.0.1
//   ETH_MAC=DE:AD:BE:EF:FE:ED  (optional; overrides EEPROM)
//
// WiFi settings (WINC1500, ESP_WIFI, or DUAL_NETWORK):
//   WIFI_SSID=MyNetwork
//   WIFI_PASS=MyPassword
//   WIFI_SECURITY=WPA2             (optional: WPA2, WPA, WEP, Open)
//   WIFI_CFG_IP=192.168.1.50   (or DHCP)
//   WIFI_DNS=192.168.1.1
//   WIFI_SUBNET=255.255.255.0
//   WIFI_GATEWAY=192.168.1.1
//
// Outdoor unit keys:
//   OUTDOOR_IP=10.1.0.72
//   OUTDOOR_PORT=80
//   OUTDOOR_POLL=30
//
// Relay output:
//   RELAY_PIN=13                  (or OFF, DISABLED, -1)

void loadDeviceConfig() {
  file_t f;
  if (!f.open("device.cfg", O_RDONLY)) {
    Serial.println("device.cfg: not found");
    return;
  }
  Serial.println("device.cfg: loading");
  char line[128];
  while (f.available()) {
    int len = f.fgets(line, sizeof(line));
    if (len <= 0) break;
    String s = String(line);
    s.trim();
    if (s.length() == 0 || s.startsWith("#")) continue;
    int eq = s.indexOf('=');
    if (eq < 0) continue;
    String key = s.substring(0, eq);
    String val = s.substring(eq + 1);
    key.trim(); val.trim();

#if defined(DUAL_NETWORK)
    // --- DUAL_NETWORK: Separate Ethernet and WiFi settings ---
    // Ethernet settings (ETH_* prefix)
    if (key == "ETH_IP") {
      String valUpper = val; valUpper.toUpperCase();
      if (valUpper == "DHCP") {
        cfgEthIP = IPAddress(0, 0, 0, 0);
        Serial.println("  Eth Static IP: DHCP");
      } else {
        cfgEthIP.fromString(val);
        Serial.print("  Eth Static IP: "); Serial.println(val);
      }
    } else if (key == "ETH_DNS") {
      cfgEthDNS.fromString(val);
      Serial.print("  Eth Static DNS: "); Serial.println(val);
    } else if (key == "ETH_SUBNET") {
      cfgEthSubnet.fromString(val);
      Serial.print("  Eth Static Subnet: "); Serial.println(val);
    } else if (key == "ETH_GATEWAY") {
      cfgEthGateway.fromString(val);
      Serial.print("  Eth Static Gateway: "); Serial.println(val);
    } else if (key == "ETH_MAC") {
      // Parse MAC address: XX:XX:XX:XX:XX:XX (hex, colon-separated)
      byte mac[6];
      bool valid = true;
      int pos = 0;
      for (int i = 0; i < 6 && valid; i++) {
        if (i > 0) {
          if (pos < (int)val.length() && val.charAt(pos) == ':') pos++;
          else valid = false;
        }
        if (valid && pos + 1 < (int)val.length()) {
          char hi = toupper(val.charAt(pos));
          char lo = toupper(val.charAt(pos + 1));
          if (isxdigit(hi) && isxdigit(lo)) {
            mac[i] = (hi <= '9' ? hi - '0' : hi - 'A' + 10) << 4
                    | (lo <= '9' ? lo - '0' : lo - 'A' + 10);
            pos += 2;
          } else valid = false;
        } else valid = false;
      }
      if (valid && pos == (int)val.length()) {
        memcpy(W5500_mac, mac, 6);
        cfgMacOverride = true;
        Serial.print("  MAC override: "); Serial.println(macToString(mac));
      } else {
        Serial.print("  ETH_MAC: invalid format: "); Serial.println(val);
      }
    }
    // WiFi settings (WIFI_* prefix)
    else if (key == "WIFI_SSID") {
      cfgWifiSSID = val;
      Serial.print("  WiFi SSID: "); Serial.println(val);
    } else if (key == "WIFI_PASS") {
      cfgWifiPass = val;
      Serial.println("  WiFi Pass: (set)");
    } else if (key == "WIFI_SECURITY") {
      String valUpper = val; valUpper.toUpperCase();
      if (valUpper == "WPA2")      cfgWifiSecurity = "WPA2";
      else if (valUpper == "WPA")  cfgWifiSecurity = "WPA";
      else if (valUpper == "WEP")  cfgWifiSecurity = "WEP";
      else if (valUpper == "OPEN") cfgWifiSecurity = "Open";
      else                         cfgWifiSecurity = val;
      Serial.print("  WiFi Encrypt: "); Serial.println(cfgWifiSecurity);
    } else if (key == "WIFI_IP") {
      String valUpper = val; valUpper.toUpperCase();
      if (valUpper == "DHCP") {
        cfgWifiIP = IPAddress(0, 0, 0, 0);
        Serial.println("  WiFi Static IP: DHCP");
      } else {
        cfgWifiIP.fromString(val);
        Serial.print("  WiFi Static IP: "); Serial.println(val);
      }
    } else if (key == "WIFI_DNS") {
      cfgWifiDNS.fromString(val);
      Serial.print("  WiFi Static DNS: "); Serial.println(val);
    } else if (key == "WIFI_SUBNET") {
      cfgWifiSubnet.fromString(val);
      Serial.print("  WiFi Static Subnet: "); Serial.println(val);
    } else if (key == "WIFI_GATEWAY") {
      cfgWifiGateway.fromString(val);
      Serial.print("  WiFi Static Gateway: "); Serial.println(val);
    }
#else
    // --- Single-interface mode ---
#if defined(W5500)
    // Ethernet settings (ETH_* keys)
    if (key == "ETH_IP") {
      String valUpper = val; valUpper.toUpperCase();
      if (valUpper == "DHCP") {
        cfgIP = IPAddress(0, 0, 0, 0);
        Serial.println("  Eth Static IP: DHCP");
      } else {
        cfgIP.fromString(val);
        Serial.print("  Eth Static IP: "); Serial.println(val);
      }
    } else if (key == "ETH_DNS") {
      cfgDNS.fromString(val);
      Serial.print("  Eth Static DNS: "); Serial.println(val);
    } else if (key == "ETH_SUBNET") {
      cfgSubnet.fromString(val);
      Serial.print("  Eth Static Subnet: "); Serial.println(val);
    } else if (key == "ETH_GATEWAY") {
      cfgGateway.fromString(val);
      Serial.print("  Eth Static Gateway: "); Serial.println(val);
    } else if (key == "ETH_MAC") {
      // Parse MAC address: XX:XX:XX:XX:XX:XX (hex, colon-separated)
      byte mac[6];
      bool valid = true;
      int pos = 0;
      for (int i = 0; i < 6 && valid; i++) {
        if (i > 0) {
          if (pos < (int)val.length() && val.charAt(pos) == ':') pos++;
          else valid = false;
        }
        if (valid && pos + 1 < (int)val.length()) {
          char hi = toupper(val.charAt(pos));
          char lo = toupper(val.charAt(pos + 1));
          if (isxdigit(hi) && isxdigit(lo)) {
            mac[i] = (hi <= '9' ? hi - '0' : hi - 'A' + 10) << 4
                    | (lo <= '9' ? lo - '0' : lo - 'A' + 10);
            pos += 2;
          } else valid = false;
        } else valid = false;
      }
      if (valid && pos == (int)val.length()) {
        memcpy(W5500_mac, mac, 6);
        cfgMacOverride = true;
        Serial.print("  MAC override: "); Serial.println(macToString(mac));
      } else {
        Serial.print("  ETH_MAC: invalid format: "); Serial.println(val);
      }
    }
#elif defined(HAS_WIFI)
    // WiFi settings (WIFI_* prefix)
    if (key == "WIFI_SSID") {
      cfgWifiSSID = val;
      Serial.print("  WiFi SSID: "); Serial.println(val);
    } else if (key == "WIFI_PASS") {
      cfgWifiPass = val;
      Serial.println("  WiFi Pass: (set)");
    } else if (key == "WIFI_SECURITY") {
      // Normalize to canonical names: WPA2, WPA, WEP, Open
      String valUpper = val;
      valUpper.toUpperCase();
      if (valUpper == "WPA2")      cfgWifiSecurity = "WPA2";
      else if (valUpper == "WPA")  cfgWifiSecurity = "WPA";
      else if (valUpper == "WEP")  cfgWifiSecurity = "WEP";
      else if (valUpper == "OPEN") cfgWifiSecurity = "Open";
      else                         cfgWifiSecurity = val;  // keep as-is
      Serial.print("  WiFi Encrypt: "); Serial.println(cfgWifiSecurity);
    } else if (key == "WIFI_IP") {
      String valUpper = val; valUpper.toUpperCase();
      if (valUpper == "DHCP") {
        cfgIP = IPAddress(0, 0, 0, 0);
        Serial.println("  WiFi Static IP: DHCP");
      } else {
        cfgIP.fromString(val);
        Serial.print("  WiFi Static IP: "); Serial.println(val);
      }
    } else if (key == "WIFI_DNS") {
      cfgDNS.fromString(val);
      Serial.print("  WiFi Static DNS: "); Serial.println(val);
    } else if (key == "WIFI_SUBNET") {
      cfgSubnet.fromString(val);
      Serial.print("  WiFi Static Subnet: "); Serial.println(val);
    } else if (key == "WIFI_GATEWAY") {
      cfgGateway.fromString(val);
      Serial.print("  WiFi Static Gateway: "); Serial.println(val);
    }
#endif
#endif  // DUAL_NETWORK

    // --- Outdoor unit settings (common to all modes) ---
    if (key == "OUTDOOR_IP") {
      if (outdoorIP.fromString(val)) {
        outdoorConfigured = (outdoorIP != IPAddress(0, 0, 0, 0));
        Serial.print("  Outdoor IP: "); Serial.println(IpAddress2String(outdoorIP));
      }
    } else if (key == "OUTDOOR_PORT") {
      outdoorPort = val.toInt();
      if (outdoorPort == 0) outdoorPort = 80;
      Serial.print("  Outdoor port: "); Serial.println(outdoorPort);
    } else if (key == "OUTDOOR_POLL") {
      outdoorPollSecs = val.toInt();
      if (outdoorPollSecs < 5) outdoorPollSecs = 5;
      Serial.print("  Outdoor poll: "); Serial.print(outdoorPollSecs); Serial.println("s");
    }
    // --- Relay output ---
    else if (key == "RELAY_PIN") {
      cfgRelayPin = val.toInt();
      // Allow -1 or "OFF"/"DISABLED" to disable
      String valUpper = val;
      valUpper.toUpperCase();
      if (valUpper == "OFF" || valUpper == "DISABLED" || valUpper == "NONE") {
        cfgRelayPin = -1;
      }
      Serial.print("  Relay pin: ");
      if (cfgRelayPin >= 0) Serial.println(cfgRelayPin);
      else Serial.println("DISABLED");
    }
  }
  f.close();
}

void saveDeviceConfig() {
  if (!sdCardPresent) {
    Serial.println("saveDeviceConfig: no SD card");
    return;
  }
  file_t f;
  if (!f.open("device.cfg", O_WRONLY | O_CREAT | O_TRUNC)) {
    Serial.println("saveDeviceConfig: can't open device.cfg for writing");
    return;
  }

#if defined(DUAL_NETWORK)
  // DUAL_NETWORK: Save both Ethernet and WiFi settings
  f.println("# Ethernet settings");
  if (cfgEthIP != IPAddress(0, 0, 0, 0)) {
    f.print("ETH_IP=");      f.println(IpAddress2String(cfgEthIP));
    f.print("ETH_DNS=");     f.println(IpAddress2String(cfgEthDNS));
    f.print("ETH_SUBNET=");  f.println(IpAddress2String(cfgEthSubnet));
    f.print("ETH_GATEWAY="); f.println(IpAddress2String(cfgEthGateway));
  } else {
    f.println("ETH_IP=DHCP");
  }
  if (cfgMacOverride) {
    f.print("ETH_MAC="); f.println(macToString(W5500_mac));
  }
  
  f.println("# WiFi settings");
  if (cfgWifiSSID.length() > 0) {
    f.print("WIFI_SSID="); f.println(cfgWifiSSID);
    f.print("WIFI_PASS="); f.println(cfgWifiPass);
  }
  if (cfgWifiSecurity.length() > 0) {
    f.print("WIFI_SECURITY="); f.println(cfgWifiSecurity);
  }
  if (cfgWifiIP != IPAddress(0, 0, 0, 0)) {
    f.print("WIFI_IP=");      f.println(IpAddress2String(cfgWifiIP));
    f.print("WIFI_DNS=");     f.println(IpAddress2String(cfgWifiDNS));
    f.print("WIFI_SUBNET=");  f.println(IpAddress2String(cfgWifiSubnet));
    f.print("WIFI_GATEWAY="); f.println(IpAddress2String(cfgWifiGateway));
  } else {
    f.println("WIFI_IP=DHCP");
  }
#else
  // Single-interface mode
  f.println("# Network settings");
#if defined(W5500)
  // Ethernet settings
  if (cfgIP != IPAddress(0, 0, 0, 0)) {
    f.print("ETH_IP=");      f.println(IpAddress2String(cfgIP));
    f.print("ETH_DNS=");     f.println(IpAddress2String(cfgDNS));
    f.print("ETH_SUBNET=");  f.println(IpAddress2String(cfgSubnet));
    f.print("ETH_GATEWAY="); f.println(IpAddress2String(cfgGateway));
  } else {
    f.println("ETH_IP=DHCP");
  }
  if (cfgMacOverride) {
    f.print("ETH_MAC="); f.println(macToString(W5500_mac));
  }
#elif defined(HAS_WIFI)
  // WiFi settings
  if (cfgWifiSSID.length() > 0) {
    f.print("WIFI_SSID="); f.println(cfgWifiSSID);
    f.print("WIFI_PASS="); f.println(cfgWifiPass);
  }
  if (cfgWifiSecurity.length() > 0) {
    f.print("WIFI_SECURITY="); f.println(cfgWifiSecurity);
  }
  if (cfgIP != IPAddress(0, 0, 0, 0)) {
    f.print("WIFI_IP=");      f.println(IpAddress2String(cfgIP));
    f.print("WIFI_DNS=");     f.println(IpAddress2String(cfgDNS));
    f.print("WIFI_SUBNET=");  f.println(IpAddress2String(cfgSubnet));
    f.print("WIFI_GATEWAY="); f.println(IpAddress2String(cfgGateway));
  } else {
    f.println("WIFI_IP=DHCP");
  }
#endif
#endif  // DUAL_NETWORK

  // Outdoor unit settings
  f.println("# Outdoor unit");
  f.print("OUTDOOR_IP=");   f.println(IpAddress2String(outdoorIP));
  f.print("OUTDOOR_PORT="); f.println(outdoorPort);
  f.print("OUTDOOR_POLL="); f.println(outdoorPollSecs);
  // Relay output
  f.println("# Relay output");
  if (cfgRelayPin >= 0) {
    f.print("RELAY_PIN="); f.println(cfgRelayPin);
  } else {
    f.println("RELAY_PIN=OFF");
  }
  f.close();
  Serial.println("saveDeviceConfig: saved");
}

// ===========================================================================
// Outdoor Polling -- HTTP GET /outdoorjson from remote unit
// ===========================================================================

void pollOutdoor() {
  if (!outdoorConfigured) return;
  if (millis() - outdoorLastPoll < (unsigned long)outdoorPollSecs * 1000UL) return;
  outdoorLastPoll = millis();

  Serial.print("pollOutdoor: connecting to ");
  Serial.print(IpAddress2String(outdoorIP));
  Serial.print(":");
  Serial.println(outdoorPort);

#if defined(W5500) || defined(WINC1500)
  HardwareClient http;
#ifdef W5500
  http.setConnectionTimeout(3000);
#endif
  if (!http.connect(outdoorIP, outdoorPort)) {
    outdoorFailCount++;
    if (outdoorFailCount >= 5) outdoorStatus = "Offline";
    Serial.println("pollOutdoor: connect failed (" +
                   String(outdoorFailCount) + " consecutive)");
    return;
  }

  // Send HTTP GET
  http.print("GET /outdoorjson HTTP/1.1\r\n");
  http.print("Host: ");
  http.print(IpAddress2String(outdoorIP));
  http.print("\r\nConnection: close\r\n\r\n");

  // Wait for response (up to 5 seconds)
  unsigned long start = millis();
  while (!http.available()) {
    if (millis() - start > 5000) {
      Serial.println("pollOutdoor: response timeout");
      http.stop();
      outdoorFailCount++;
      if (outdoorFailCount >= 5) outdoorStatus = "Offline";
      return;
    }
    delay(10);
  }

  // Read entire response
  String response = "";
  while (http.available()) {
    response += (char)http.read();
  }
  http.stop();

  // Find JSON body (after blank line in HTTP response)
  int bodyStart = response.indexOf("\r\n\r\n");
  if (bodyStart < 0) {
    outdoorFailCount++;
    if (outdoorFailCount >= 5) outdoorStatus = "Offline";
    Serial.println("pollOutdoor: no body in response");
    return;
  }
  String json = response.substring(bodyStart + 4);
  json.trim();

  // Simple JSON parser for flat {"key":"value",...} objects.
  // No ArduinoJson needed -- our response is always a single flat object.
  auto getJsonVal = [&json](const String &key) -> String {
    String search = "\"" + key + "\":\"";
    int start = json.indexOf(search);
    if (start < 0) return "";
    start += search.length();
    int end = json.indexOf("\"", start);
    if (end < 0) return "";
    return json.substring(start, end);
  };

  outdoorTemp      = getJsonVal("OUTDOOR_TEMP");
  outdoorHumidity  = getJsonVal("OUTDOOR_HUMIDITY");
  outdoorPressHpa  = getJsonVal("OUTDOOR_PRESSURE_HPA");
  outdoorPressInhg = getJsonVal("OUTDOOR_PRESSURE_INHG");
  outdoorGasKohms  = getJsonVal("OUTDOOR_GAS_KOHMS");
  outdoorBmeType   = getJsonVal("OUTDOOR_BME_TYPE");
  outdoorBoard     = getJsonVal("OUTDOOR_BOARD");
  outdoorEthIP     = getJsonVal("OUTDOOR_ETH_IP");
  outdoorWifiIP    = getJsonVal("OUTDOOR_WIFI_IP");
  outdoorIface     = getJsonVal("OUTDOOR_IFACE");
  outdoorBatteryV  = getJsonVal("OUTDOOR_BATVOLTS");
  outdoorHasEth    = (getJsonVal("OUTDOOR_HAS_ETHERNET") == "1");
  outdoorHasWifi   = (getJsonVal("OUTDOOR_HAS_WIFI") == "1");

  outdoorStatus    = "Online";
  outdoorFailCount = 0;
  outdoorLastOK    = millis();

  Serial.print("pollOutdoor: OK (outdoor temp=");
  Serial.print(outdoorTemp);
  Serial.println("F)");
#else
  // No networking -- can't poll
  outdoorStatus = "N/A";
#endif
}

// ===========================================================================
// SD Card Hot-Swap Detection
// ===========================================================================
// Re-probes the SD card slot to detect insertion or removal while running.
// Called periodically from loop() if SD_REPROBE_INTERVAL_MS > 0.

void reprobeSDCard() {
  bool checkNow = false;

#if SDDET_PIN >= 0
  // CD pin check: HIGH = card present (floating, pulled up), LOW = no card
  static bool lastCDState = HIGH;
  bool cdState = digitalRead(SDDET_PIN);
  if (cdState != lastCDState) {
    lastCDState = cdState;
    checkNow = true;
  }
#elif SD_REPROBE_INTERVAL_MS > 0
  // Polling mode: check periodically
  if (millis() - lastSDReprobe >= SD_REPROBE_INTERVAL_MS) {
    lastSDReprobe = millis();
    checkNow = true;
  }
#endif

  if (!checkNow) return;

  bool wasPresent = sdCardPresent;

  // Try to initialize the SD card
  if (sd.begin(SD_CS, SD_SCK_MHZ(4))) {
    if (!wasPresent) {
      // Card was just inserted
      sdCardPresent = true;
      Serial.println("SD card: INSERTED (hot-swap detected)");

      // Check for index.html
      file_t testFile;
      if (testFile.open("index.html", O_RDONLY)) {
        sdIndexPresent = true;
        testFile.close();
        Serial.println("SD index.html: FOUND");
      } else {
        sdIndexPresent = false;
        Serial.println("SD index.html: not found");
      }

      // Reload device config from newly inserted card
      loadDeviceConfig();
    }
    // else: card was already present, nothing to do
  } else {
    if (wasPresent) {
      // Card was just removed
      sdCardPresent = false;
      sdIndexPresent = false;
      Serial.println("SD card: REMOVED (hot-swap detected)");
    }
    // else: card was already absent, nothing to do
  }
}

// ===========================================================================
// Config Page -- /config
// ===========================================================================
// Shows network status, network settings, outdoor unit settings.
// Network settings are saved to device.cfg and applied at next boot.
//
// GET /config                              -> shows config page
// GET /config?section=network&sip=X&...    -> saves network settings
// GET /config?ip=X&port=Y&poll=Z           -> saves outdoor settings
// GET /config?clear=1                      -> clears outdoor config

void sendConfigPage(HardwareClient cl, const String &reqPage) {
  // Check for query-string parameters (form submission)
  int qm = reqPage.indexOf('?');
  if (qm > 0) {
    String qs = reqPage.substring(qm + 1);

    // Helper to extract a param value from query string
    auto getParam = [&qs](const String &name) -> String {
      String search = name + "=";
      int start = qs.indexOf(search);
      if (start < 0) return "";
      start += search.length();
      int end = qs.indexOf('&', start);
      if (end < 0) end = qs.length();
      return qs.substring(start, end);
    };

    // --- Network settings form ---
    String section = getParam("section");
    if (section == "network") {
      String sip = getParam("sip");
      String sdns = getParam("sdns");
      String ssub = getParam("ssub");
      String sgw  = getParam("sgw");
      if (sip.length() > 0)  cfgIP.fromString(sip);
      else                    cfgIP = IPAddress(0, 0, 0, 0);
      if (sdns.length() > 0) cfgDNS.fromString(sdns);
      if (ssub.length() > 0) cfgSubnet.fromString(ssub);
      if (sgw.length() > 0)  cfgGateway.fromString(sgw);
#if defined(HAS_WIFI)
      String wssid = getParam("wssid");
      String wpass = getParam("wpass");
      cfgWifiSSID = wssid;
      cfgWifiPass = wpass;
#endif
      saveDeviceConfig();
      Serial.println("Network config updated via web");
    }
    // --- Outdoor config ---
    else if (getParam("clear") == "1") {
      clearOutdoorConfig();
      saveDeviceConfig();
      Serial.println("Outdoor config cleared");
    } else if (getParam("ip").length() > 0 || getParam("port").length() > 0) {
      String ipStr = getParam("ip");
      String portStr = getParam("port");
      String pollStr = getParam("poll");

      if (ipStr.length() > 0) {
        IPAddress parsed;
        if (parsed.fromString(ipStr)) {
          outdoorIP = parsed;
        }
      }
      if (portStr.length() > 0) {
        outdoorPort = portStr.toInt();
        if (outdoorPort == 0) outdoorPort = 80;
      }
      if (pollStr.length() > 0) {
        outdoorPollSecs = pollStr.toInt();
        if (outdoorPollSecs < 5) outdoorPollSecs = 5;
        if (outdoorPollSecs > 3600) outdoorPollSecs = 3600;
      }
      outdoorConfigured = (outdoorIP[0] != 0 || outdoorIP[1] != 0 ||
                           outdoorIP[2] != 0 || outdoorIP[3] != 0);
      outdoorLastPoll = 0;
      saveDeviceConfig();
      Serial.println("Outdoor config updated via web");
    }
  }

  // Build the outdoor IP string for display
  String oipStr;
  if (outdoorConfigured) {
    oipStr = IpAddress2String(outdoorIP);
  } else {
    oipStr = "Not configured";
  }

  String val;  // for resolveVar calls

  // Render the config page - use shared head
  sendHtmlHead(cl, "Thermostat Config");
  cl.println(F("body{padding:10px;display:flex;flex-direction:column;align-items:center}"));
  cl.println(F(".card{background:#111;border:1px solid #333;border-radius:6px;padding:10px;margin:8px 0;max-width:320px;width:100%}"));
  cl.println(F(".label{color:#999;font-size:.75em;font-weight:bold;margin-bottom:6px}"));
  cl.println(F(".row{display:flex;justify-content:space-between;align-items:center;padding:3px 0;font-size:.9em}"));
  cl.println(F(".value{color:#4ade80;text-align:right}"));
  cl.println(F("input{background:#222;color:#eee;border:1px solid #555;border-radius:4px;padding:4px 6px;width:140px;font-size:.9em}"));
  cl.println(F("button{background:#333;color:#eee;border:1px solid #555;border-radius:4px;padding:6px 16px;margin:4px;cursor:pointer;font-size:.9em}"));
  cl.println(F("button:hover{background:#555}"));
  cl.println(F(".online{color:#4ade80}.offline{color:#f87171}.na{color:#999}"));
  cl.println(F("a{color:#60a5fa}"));
#if defined(HAS_WIFI)
  cl.println(F(".net-border{border-color:#60a5fa}.net-label{color:#60a5fa}"));
  cl.println(F(".scan-btn{background:#1a3a5c;color:#60a5fa;border:1px solid #60a5fa;border-radius:4px;padding:4px 10px;cursor:pointer;font-size:.78em;margin-left:6px}"));
  cl.println(F(".scan-btn:hover{background:#264a6e}.scan-btn:disabled{opacity:.5;cursor:wait}"));
  cl.println(F(".scan-list{background:#111;border:1px solid #444;border-radius:4px;margin:6px 0;max-height:160px;overflow-y:auto;font-size:.85em;display:none}"));
  cl.println(F(".scan-item{display:flex;justify-content:space-between;align-items:center;padding:5px 8px;cursor:pointer;border-bottom:1px solid #222}"));
  cl.println(F(".scan-item:last-child{border-bottom:none}.scan-item:hover{background:#1a2a3a}"));
  cl.println(F(".scan-ssid{color:#eee;flex:1;overflow:hidden;text-overflow:ellipsis;white-space:nowrap}"));
  cl.println(F(".scan-detail{color:#888;font-size:.85em;margin-left:8px;white-space:nowrap}"));
  cl.println(F(".scan-lock{color:#f87171;font-size:.7em;margin-left:4px}"));
  cl.println(F(".scan-bars{display:inline-block;margin-left:4px}"));
  cl.println(F(".scan-bar{display:inline-block;width:3px;background:#444;margin-right:1px;vertical-align:bottom;border-radius:1px 1px 0 0}"));
  cl.println(F(".scan-bar.active{background:#4ade80}"));
#else
  cl.println(F(".net-border{border-color:#fbbf24}.net-label{color:#fbbf24}"));
#endif
  cl.println(F(".hint{color:#888;font-size:.75em}"));
  cl.println(F(".tabs{display:flex;border-bottom:1px solid #333;margin-bottom:8px}"));
  cl.println(F(".tab{flex:1;padding:8px;text-align:center;cursor:pointer;background:#1a1a1a;border:1px solid #333;border-bottom:none;margin-right:-1px;font-size:.85em;color:#888}"));
  cl.println(F(".tab:first-child{border-radius:4px 0 0 0}.tab:last-child{border-radius:0 4px 0 0;margin-right:0}"));
  cl.println(F(".tab.active{background:#111;color:#4ade80;border-bottom:1px solid #111;margin-bottom:-1px}"));
  cl.println(F(".tab-spacer{flex:1}"));
  cl.println(F(".info-box{background:#1a2a1a;border:1px solid #2a4a2a;border-radius:4px;padding:8px;margin:8px 0;font-size:.8em;color:#6b8}"));
  cl.println(F("</style></head><body>"));

  // ========== Network Settings Card (Tabbed) ==========
  cl.println("<div class='card net-border'>");
  cl.println("<div class='label net-label'>NETWORK CONFIGURATION</div>");
  
  // --- Tab bar (consistent layout: Ethernet left, WiFi right) ---
  cl.println("<div class='tabs'>");
#if defined(DUAL_NETWORK)
  // Dual network: both tabs active
  cl.println("<div class='tab active' id='tabEth' onclick='switchTab(\"eth\")'>Ethernet</div>");
  cl.println("<div class='tab' id='tabWifi' onclick='switchTab(\"wifi\")'>WiFi</div>");
#elif defined(W5500)
  // Ethernet only: Ethernet tab on left, empty space on right
  cl.println("<div class='tab active' id='tabEth'>Ethernet</div>");
  cl.println("<div class='tab-spacer'></div>");
#elif defined(HAS_WIFI)
  // WiFi only: empty space on left, WiFi tab on right
  cl.println("<div class='tab-spacer'></div>");
  cl.println("<div class='tab active' id='tabWifi'>WiFi</div>");
#endif
  cl.println("</div>");
  
  // --- Form with hidden fields for all values ---
  cl.println("<form id='netForm'>");

#if defined(DUAL_NETWORK) || defined(W5500)
  // --- Ethernet Panel ---
  cl.println("<div id='panelEth'>");
  // Status
  cl.print("<div class='row'><span>IP</span><span class='value'>");
  cl.print(IpAddress2String(Ethernet.localIP()));
  cl.println("</span></div>");
  cl.print("<div class='row'><span>MAC</span><span class='value'>");
  cl.print(macToString(W5500_mac));
  cl.println("</span></div>");
  cl.print("<div class='row'><span>Link</span><span class='value ");
  cl.print((Ethernet.linkStatus() == LinkON) ? "online'>Connected" : "offline'>Disconnected");
  cl.println("</span></div>");
  // Settings
  cl.println("<div class='hint' style='margin:8px 0 4px'>Static IP (leave blank for DHCP)</div>");
  cl.print("<div class='row'><span>Static IP</span><input id='eth_sip' value='");
#if defined(DUAL_NETWORK)
  if (cfgEthIP != IPAddress(0, 0, 0, 0)) cl.print(IpAddress2String(cfgEthIP));
#else
  if (cfgIP != IPAddress(0, 0, 0, 0)) cl.print(IpAddress2String(cfgIP));
#endif
  cl.println("' placeholder='10.1.0.71'></div>");
  cl.print("<div class='row'><span>DNS</span><input id='eth_dns' value='");
#if defined(DUAL_NETWORK)
  if (cfgEthDNS != IPAddress(0, 0, 0, 0)) cl.print(IpAddress2String(cfgEthDNS));
#else
  if (cfgDNS != IPAddress(0, 0, 0, 0)) cl.print(IpAddress2String(cfgDNS));
#endif
  cl.println("' placeholder='10.1.0.1'></div>");
  cl.print("<div class='row'><span>Subnet</span><input id='eth_sub' value='");
#if defined(DUAL_NETWORK)
  if (cfgEthSubnet != IPAddress(255, 255, 255, 0)) cl.print(IpAddress2String(cfgEthSubnet));
#else
  if (cfgSubnet != IPAddress(255, 255, 255, 0)) cl.print(IpAddress2String(cfgSubnet));
#endif
  cl.println("' placeholder='255.255.255.0'></div>");
  cl.print("<div class='row'><span>Gateway</span><input id='eth_gw' value='");
#if defined(DUAL_NETWORK)
  if (cfgEthGateway != IPAddress(0, 0, 0, 0)) cl.print(IpAddress2String(cfgEthGateway));
#else
  if (cfgGateway != IPAddress(0, 0, 0, 0)) cl.print(IpAddress2String(cfgGateway));
#endif
  cl.println("' placeholder='10.1.0.1'></div>");
  cl.println("</div>");  // end panelEth
#endif

#if defined(DUAL_NETWORK) || defined(HAS_WIFI)
  // --- WiFi Panel ---
#if defined(DUAL_NETWORK)
  cl.println("<div id='panelWifi' style='display:none'>");
#else
  cl.println("<div id='panelWifi'>");
#endif
  // Status
  cl.print("<div class='row'><span>SSID</span><span class='value'>");
  cl.print(WiFi.SSID());
  cl.println("</span></div>");
  cl.print("<div class='row'><span>IP</span><span class='value'>");
  cl.print(IpAddress2String(WiFi.localIP()));
  cl.println("</span></div>");
  byte wifiMac[6]; WiFi.macAddress(wifiMac);
  cl.print("<div class='row'><span>MAC</span><span class='value'>");
  cl.print(macToString(wifiMac));
  cl.println("</span></div>");
  cl.print("<div class='row'><span>RSSI</span><span class='value'>");
  cl.print(WiFi.RSSI());
  cl.println(" dBm</span></div>");
  // Settings
  cl.print("<div class='row'><span>SSID</span><span style='display:flex;align-items:center'>");
  cl.print("<input id='wifi_ssid' value='");
  cl.print(cfgWifiSSID.length() > 0 ? cfgWifiSSID : WiFi.SSID());
  cl.println("' placeholder='MyNetwork'>");
  cl.println("<button type='button' class='scan-btn' id='scanBtn' onclick='doScan()'>Scan</button></span></div>");
  cl.println("<div class='scan-list' id='scanList'></div>");
  cl.print("<div class='row'><span>Password</span><input id='wifi_pass' type='password' value='");
  if (cfgWifiPass.length() > 0) cl.print(cfgWifiPass);
  cl.println("'></div>");
  cl.println("<div class='hint' style='margin:8px 0 4px'>Static IP (leave blank for DHCP)</div>");
  cl.print("<div class='row'><span>Static IP</span><input id='wifi_sip' value='");
#if defined(DUAL_NETWORK)
  if (cfgWifiIP != IPAddress(0, 0, 0, 0)) cl.print(IpAddress2String(cfgWifiIP));
#else
  if (cfgIP != IPAddress(0, 0, 0, 0)) cl.print(IpAddress2String(cfgIP));
#endif
  cl.println("' placeholder='192.168.1.50'></div>");
  cl.print("<div class='row'><span>DNS</span><input id='wifi_dns' value='");
#if defined(DUAL_NETWORK)
  if (cfgWifiDNS != IPAddress(0, 0, 0, 0)) cl.print(IpAddress2String(cfgWifiDNS));
#else
  if (cfgDNS != IPAddress(0, 0, 0, 0)) cl.print(IpAddress2String(cfgDNS));
#endif
  cl.println("' placeholder='192.168.1.1'></div>");
  cl.print("<div class='row'><span>Subnet</span><input id='wifi_sub' value='");
#if defined(DUAL_NETWORK)
  if (cfgWifiSubnet != IPAddress(255, 255, 255, 0)) cl.print(IpAddress2String(cfgWifiSubnet));
#else
  if (cfgSubnet != IPAddress(255, 255, 255, 0)) cl.print(IpAddress2String(cfgSubnet));
#endif
  cl.println("' placeholder='255.255.255.0'></div>");
  cl.print("<div class='row'><span>Gateway</span><input id='wifi_gw' value='");
#if defined(DUAL_NETWORK)
  if (cfgWifiGateway != IPAddress(0, 0, 0, 0)) cl.print(IpAddress2String(cfgWifiGateway));
#else
  if (cfgGateway != IPAddress(0, 0, 0, 0)) cl.print(IpAddress2String(cfgGateway));
#endif
  cl.println("' placeholder='192.168.1.1'></div>");
  cl.println("</div>");  // end panelWifi
#endif

  // --- Info box and Save button ---
#if defined(DUAL_NETWORK)
  cl.println("<div class='info-box' id='infoBox'>&#9432; Connected via Ethernet. WiFi changes apply immediately. Ethernet changes apply from WiFi or restart.</div>");
#else
  cl.println("<div class='info-box'>&#9432; Changes apply after restart.</div>");
#endif
  cl.println("<div style='text-align:center;margin-top:8px'>");
  cl.println("<button type='submit' id='saveBtn'>Save Network</button>");
  cl.println("</div></form>");
  
  // Board info
  cl.print("<div class='row' style='margin-top:8px;border-top:1px solid #333;padding-top:8px'><span>Board</span><span class='value'>");
  cl.print(curBoard);
  cl.println("</span></div>");
  cl.print("<div class='row'><span>SD Card</span><span class='value'>");
  if (sdCardPresent) {
    cl.print(getSDCardFormat());
    cl.print(" (OK)");
  } else {
    cl.print("Not detected");
  }
  cl.println("</span></div>");
  cl.println("</div>");  // end card

  // ========== Outdoor Unit Status ==========
  cl.println("<div class='card'><div class='label'>OUTDOOR UNIT STATUS</div>");
  cl.print("<div class='row'><span>Outdoor IP</span><span class='value'>");
  cl.print(oipStr);
  cl.println("</span></div>");
  cl.print("<div class='row'><span>Port</span><span class='value'>");
  cl.print(outdoorPort);
  cl.println("</span></div>");
  cl.print("<div class='row'><span>Poll interval</span><span class='value'>");
  cl.print(outdoorPollSecs);
  cl.println("s</span></div>");
  cl.print("<div class='row'><span>Status</span><span class='value ");
  if (outdoorStatus == "Online") cl.print("online");
  else if (outdoorStatus == "Offline") cl.print("offline");
  else cl.print("na");
  cl.print("'>");
  cl.print(outdoorStatus);
  cl.println("</span></div>");

  if (outdoorStatus == "Online") {
    cl.print("<div class='row'><span>Outdoor Temp</span><span class='value'>");
    cl.print(outdoorTemp);
    cl.println("&deg;F</span></div>");
    if (outdoorHumidity.length() > 0) {
      cl.print("<div class='row'><span>Humidity</span><span class='value'>");
      cl.print(outdoorHumidity);
      cl.println("%</span></div>");
    }
  }
  cl.println("</div>");

  // ========== Configure Outdoor Unit ==========
  cl.println("<div class='card'><div class='label'>CONFIGURE OUTDOOR UNIT</div>");
  cl.println("<form id='outForm' action='/config' method='get'>");
  cl.print("<div class='row'><span>Outdoor IP</span><input name='ip' value='");
  if (outdoorConfigured) cl.print(oipStr);
  cl.println("' placeholder='10.1.0.72'></div>");
  cl.print("<div class='row'><span>Port</span><input name='port' value='");
  cl.print(outdoorPort);
  cl.println("'></div>");
  cl.print("<div class='row'><span>Poll (sec)</span><input name='poll' value='");
  cl.print(outdoorPollSecs);
  cl.println("'></div>");
  cl.println("<div style='text-align:center;margin-top:8px'>");
  cl.println("<button type='submit'>Save</button>");
  cl.println("</div></form></div>");

  // Clear button
  if (outdoorConfigured) {
    cl.println("<div class='card' style='text-align:center'>");
    cl.println("<form id='clearForm' action='/config' method='get'>");
    cl.println("<input type='hidden' name='clear' value='1'>");
    cl.println("<button type='submit' style='color:#f87171'>Clear Outdoor Config</button>");
    cl.println("</form></div>");
  }

  // Quick links
  cl.println("<div class='card'>");
  cl.println("<div class='label'>ENDPOINTS</div>");
  cl.println("<div class='row'><a href='/'>Home</a></div>");
  if (sdCardPresent) {
    cl.println("<div class='row'><a href='/sdfiles'>/sdfiles</a> -- SD card file manager</div>");
  }
  cl.println("<div class='row'><a href='/outdoorjson'>/outdoorjson</a> -- this unit as outdoor</div>");
  cl.println("<div class='row'><a href='/indoorjson'>/indoorjson</a> -- this unit as indoor</div>");
  cl.println("<div class='row'><a href='/api/state'>/api/state</a> -- JSON state</div>");
  cl.println("<div class='row'><a href='/config'>/config</a> -- this page</div>");
#if defined(HAS_WIFI)
  cl.println("<div class='row'><a href='/api/scan'>/api/scan</a> -- WiFi scan (JSON)</div>");
#endif
  cl.println("</div>");

  if (!sdCardPresent) {
    cl.println("<p style='color:#f87171;font-size:.8em;text-align:center'>&#9888; No SD card -- settings won't persist across reboot.</p>");
  }

  cl.println("<p><a href='/'>&larr; Back to thermostat</a></p>");

  // Form submission via /api/config POST + WiFi scan JavaScript
  cl.println(F("<script>function $(i){return document.getElementById(i)}"));
  // --- API config submit helper ---
  cl.println(F("function apiPost(d,b){b.disabled=true;b.textContent='Saving...';"));
  cl.println(F("fetch('/api/config',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(d)})"));
  cl.println(F(".then(r=>r.json()).then(j=>{b.textContent=j.success?'Saved!':'Error';setTimeout(()=>location.reload(),1500);})"));
  cl.println(F(".catch(()=>{b.textContent='Error';b.disabled=false;});}"));
#if defined(DUAL_NETWORK)
  // --- Tab switching for dual network ---
  cl.println(F("function switchTab(t){$('panelEth').style.display=t==='eth'?'block':'none';$('panelWifi').style.display=t==='wifi'?'block':'none';"));
  cl.println(F("$('tabEth').classList.toggle('active',t==='eth');$('tabWifi').classList.toggle('active',t==='wifi');"));
  cl.println(F("$('infoBox').innerHTML=t==='eth'?'&#9432; Ethernet active. WiFi changes apply now. Eth changes need WiFi/restart.'"));
  cl.println(F(":'&#9432; WiFi active. Eth changes apply now. WiFi changes need Eth/restart.';}"));
#endif
  // --- Network form submit ---
  cl.println(F("function saveNetwork(e){e.preventDefault();var b=$('saveBtn');"));
#if defined(DUAL_NETWORK)
  cl.println(F("var d={action:'dual_network',eth_cfg_ip:$('eth_sip').value,eth_cfg_dns:$('eth_dns').value,"));
  cl.println(F("eth_cfg_subnet:$('eth_sub').value,eth_cfg_gateway:$('eth_gw').value,wifi_ssid:$('wifi_ssid').value,"));
  cl.println(F("wifi_pass:$('wifi_pass').value,wifi_cfg_ip:$('wifi_sip').value,wifi_cfg_dns:$('wifi_dns').value,"));
  cl.println(F("wifi_cfg_subnet:$('wifi_sub').value,wifi_cfg_gateway:$('wifi_gw').value};"));
#elif defined(W5500)
  cl.println(F("var d={action:'network',eth_cfg_ip:$('eth_sip').value,eth_cfg_dns:$('eth_dns').value,"));
  cl.println(F("eth_cfg_subnet:$('eth_sub').value,eth_cfg_gateway:$('eth_gw').value};"));
#elif defined(HAS_WIFI)
  cl.println(F("var d={action:'network',wifi_ssid:$('wifi_ssid').value,wifi_pass:$('wifi_pass').value,"));
  cl.println(F("wifi_cfg_ip:$('wifi_sip').value,wifi_cfg_dns:$('wifi_dns').value,"));
  cl.println(F("wifi_cfg_subnet:$('wifi_sub').value,wifi_cfg_gateway:$('wifi_gw').value};"));
#endif
  cl.println(F("apiPost(d,b);}$('netForm').addEventListener('submit',saveNetwork);"));
  // --- Outdoor form submit ---
  cl.println(F("function saveOutdoor(e){e.preventDefault();var f=e.target;"));
  cl.println(F("apiPost({action:'outdoor',ip:f.ip.value,port:parseInt(f.port.value)||80,poll:parseInt(f.poll.value)||30},f.querySelector('button[type=submit]'));}"));
  cl.println(F("$('outForm').addEventListener('submit',saveOutdoor);"));
  // --- Clear outdoor ---
  cl.println(F("function clearOutdoor(e){e.preventDefault();apiPost({action:'clear_outdoor'},e.target.querySelector('button[type=submit]'));}"));
  cl.println(F("var cf=$('clearForm');if(cf)cf.addEventListener('submit',clearOutdoor);"));
#if defined(HAS_WIFI) || defined(DUAL_NETWORK)
  // --- WiFi scan ---
  cl.println(F("function doScan(){var b=$('scanBtn'),l=$('scanList');b.disabled=true;b.textContent='Scanning...';"));
  cl.println(F("l.style.display='block';l.innerHTML='<div style=\"padding:10px;color:#888;text-align:center\">Scanning...</div>';"));
  cl.println(F("fetch('/api/scan').then(r=>r.json()).then(data=>{var nets=data.networks||data;nets.sort((a,b)=>b.rssi-a.rssi);"));
  cl.println(F("var h='';for(var i=0;i<nets.length;i++){var n=nets[i];var lock=n.enc!=='Open'?'<span class=\"scan-lock\">&#128274;</span>':'';"));
  cl.println(F("var bars='<span class=\"scan-bars\">';var s=n.rssi>=-50?4:n.rssi>=-60?3:n.rssi>=-70?2:1;"));
  cl.println(F("for(var j=1;j<=4;j++)bars+='<span class=\"scan-bar'+(j<=s?' active':'')+'\" style=\"height:'+(3+j*3)+'px\"></span>';bars+='</span>';"));
  cl.println(F("h+='<div class=\"scan-item\" onclick=\"selSsid(this.dataset.ssid)\" data-ssid=\"'+n.ssid.replace(/\"/g,'&quot;')+'\">';"));
  cl.println(F("h+='<span class=\"scan-ssid\">'+n.ssid+lock+'</span><span class=\"scan-detail\">'+n.enc+' '+n.rssi+'dBm'+bars+'</span></div>';}"));
  cl.println(F("l.innerHTML=h||'<div style=\"padding:10px;color:#888;text-align:center\">No networks</div>';b.disabled=false;b.textContent='Scan';"));
  cl.println(F("}).catch(()=>{l.innerHTML='<div style=\"padding:10px;color:#f87171;text-align:center\">Scan failed</div>';b.disabled=false;b.textContent='Scan';});}"));
  cl.println(F("function selSsid(s){$('wifi_ssid').value=s;}"));
#endif
  cl.println(F("</script></body></html>"));
  CLIENT_CLEAR(cl);
}

// ===========================================================================
// TFT Settings / Network Info Screen
// ===========================================================================
// Drawn when settingsScreen != SETTINGS_OFF.  Shows network info, outdoor unit status,
// and sensor summary.  Touch the Light button region to cycle/dismiss.

void drawSettingsScreen() {
  if (settingsDrawn) return;
  settingsDrawn = true;

  tft.fillScreen(TFTbackgroundColor);
  String val;

#if defined(DUAL_NETWORK)
  // DUAL_NETWORK: Show LAN or WiFi based on settingsScreen
  bool showEthernet = (settingsScreen == SETTINGS_LAN);
  bool showWifi = (settingsScreen == SETTINGS_WIFI);

  if (showWifi) {
    // ===================== WiFi Overlay (blue border) =====================
    uint16_t accentColor = 0x653F;  // #60a5fa blue
    int ox = 6, oy = 3, ow = 308, oh = 234;
    int p = 8, lh = 18;
    int vxL = ox + p + 60;
    int col2 = ox + 160;
    int vxR = col2 + 52;

    tft.drawRect(ox, oy, ow, oh, accentColor);
    tft.drawRect(ox + 2, oy + 2, ow - 4, oh - 4, accentColor);

    tft.setFont(&FreeSansBold9pt7b);
    tft.setTextColor(accentColor);
    tft.setCursor(ox + p, oy + 20);
    tft.print("Network Settings");
    tft.setFont(&FreeSans9pt7b);
    tft.setCursor(ox + ow - p - 85, oy + 18);
    tft.print("WiFi [2/2]");
    tft.drawFastHLine(ox + p, oy + 25, ow - 2 * p, accentColor);

    int y = oy + 30;
    tft.setFont(&FreeSans9pt7b);

    // SSID (full width)
    tft.setTextColor(0xAD55);
    tft.setCursor(ox + p, y + 10);
    tft.print("SSID:");
    tft.setTextColor(ILI9341_WHITE);
    tft.setCursor(vxL, y + 10);
    tft.print(WiFi.SSID());
    y += lh;

    // Two-column: IP/Subnet/Gateway (left) + Status/Battery (right)
    int ly = y, ry = y;

    // Left column: IP
    tft.setTextColor(0xAD55);
    tft.setCursor(ox + p, ly + 10);
    tft.print("IP:");
    tft.setTextColor(ILI9341_WHITE);
    tft.setCursor(vxL, ly + 10);
    tft.print(IpAddress2String(WiFi.localIP()));
    ly += lh;

    // Right column: Status
    tft.setTextColor(0xAD55);
    tft.setCursor(col2, ry + 10);
    tft.print("Status:");
    tft.setTextColor((WiFi.status() == WL_CONNECTED) ? ILI9341_GREEN : ILI9341_RED);
    tft.setCursor(vxR, ry + 10);
    tft.print((WiFi.status() == WL_CONNECTED) ? "Connected" : "Disconnected");
    ry += lh;

    // Left column: Subnet
    tft.setTextColor(0xAD55);
    tft.setCursor(ox + p, ly + 10);
    tft.print("Subnet:");
    tft.setTextColor(ILI9341_WHITE);
    tft.setCursor(vxL, ly + 10);
    tft.print(IpAddress2String(WiFi.subnetMask()));
    ly += lh;

#if HAS_BATT
    // Right column: Battery
    float battV = readBatteryVoltage();
    tft.setTextColor(0xAD55);
    tft.setCursor(col2, ry + 10);
    tft.print("Battery:");
    tft.setTextColor(battV > 3.5 ? ILI9341_GREEN : battV <= 3.3 ? ILI9341_RED : 0x9CC0);
    tft.setCursor(vxR, ry + 10);
    tft.print(String(battV, 2) + "V");
    ry += lh;
#endif

    // Left column: Gateway
    tft.setTextColor(0xAD55);
    tft.setCursor(ox + p, ly + 10);
    tft.print("Gateway:");
    tft.setTextColor(ILI9341_WHITE);
    tft.setCursor(vxL, ly + 10);
    tft.print(IpAddress2String(WiFi.gatewayIP()));
    ly += lh;

    y = (ly > ry) ? ly : ry;

    // MAC (full width)
    tft.setTextColor(0xAD55);
    tft.setCursor(ox + p, y + 10);
    tft.print("MAC:");
    tft.setTextColor(ILI9341_WHITE);
    byte wMac[6]; WiFi.macAddress(wMac);
    tft.setCursor(vxL, y + 10);
    tft.print(macToString(wMac));
    y += lh;

    // Security + RSSI (two columns)
    tft.setTextColor(0xAD55);
    tft.setCursor(ox + p, y + 10);
    tft.print("Security:");
    tft.setTextColor(ILI9341_WHITE);
    tft.setCursor(vxL, y + 10);
    tft.print(getCurrentWifiSecurity());
    tft.setTextColor(0xAD55);
    tft.setCursor(col2, y + 10);
    tft.print("RSSI:");
    int rssi = WiFi.RSSI();
    tft.setTextColor(rssi > -50 ? ILI9341_GREEN : rssi > -70 ? ILI9341_YELLOW : ILI9341_RED);
    tft.setCursor(vxR, y + 10);
    tft.print(String(rssi) + " dBm");
    y += lh;

    // Outdoor section divider
    y += 2;
    tft.drawFastHLine(ox + p, y, ow - 2 * p, 0x4208);
    y += 4;

    tft.setFont(&FreeSansBold9pt7b);
    tft.setTextColor(accentColor);
    tft.setCursor(ox + p, y + 12);
    tft.print("Outdoor Unit");
    y += lh;

    tft.setFont(&FreeSans9pt7b);
    if (outdoorConfigured) {
      tft.setTextColor(0xAD55);
      tft.setCursor(ox + p, y + 10);
      tft.print("Remote:");
      tft.setTextColor(ILI9341_WHITE);
      tft.setCursor(vxL, y + 10);
      tft.print(IpAddress2String(outdoorIP) + ":" + String(outdoorPort));
      y += lh;

      tft.setTextColor(0xAD55);
      tft.setCursor(ox + p, y + 10);
      tft.print("Status:");
      if (outdoorStatus == "Online") {
        tft.setTextColor(ILI9341_GREEN);
        tft.setCursor(vxL, y + 10);
        tft.print("Online  " + outdoorTemp + "\xB0""F  " + outdoorHumidity + "%");
      } else if (outdoorStatus == "Offline") {
        tft.setTextColor(ILI9341_RED);
        tft.setCursor(vxL, y + 10);
        tft.print("Offline");
      } else {
        tft.setTextColor(0x7BEF);
        tft.setCursor(vxL, y + 10);
        tft.print("N/A");
      }
    } else {
      tft.setTextColor(0x7BEF);
      tft.setCursor(ox + p, y + 10);
      tft.print("Not configured");
    }

    // Footer: Tap to close
    tft.setFont(&FreeSans9pt7b);
    tft.setTextColor(accentColor);
    String tapMsg = "Tap anywhere to close";
    int16_t tx1, ty1; uint16_t ttw, tth;
    tft.getTextBounds(tapMsg, 0, 0, &tx1, &ty1, &ttw, &tth);
    tft.setCursor(ox + (ow - (int)ttw) / 2, oy + oh - 6);
    tft.print(tapMsg);

  } else {
    // ==================== Ethernet Overlay (yellow border) ====================
    uint16_t accentColor = ILI9341_YELLOW;
    int ox = 6, oy = 3, ow = 308, oh = 234;
    int p = 8, lh = 18;
    int vx = ox + p + 70;  // Value column - moved right to avoid overlap

    tft.drawRect(ox, oy, ow, oh, accentColor);
    tft.drawRect(ox + 2, oy + 2, ow - 4, oh - 4, accentColor);

    tft.setFont(&FreeSansBold9pt7b);
    tft.setTextColor(accentColor);
    tft.setCursor(ox + p, oy + 20);
    tft.print("Network Settings");
    tft.setFont(&FreeSans9pt7b);
    tft.setCursor(ox + ow - p - 105, oy + 18);
    tft.print("Ethernet [1/2]");
    tft.drawFastHLine(ox + p, oy + 25, ow - 2 * p, accentColor);

    int y = oy + 30;
    tft.setFont(&FreeSans9pt7b);

    // IP
    tft.setTextColor(ILI9341_WHITE);
    tft.setCursor(ox + p, y + 10);
    tft.print("IP:");
    tft.setTextColor(ILI9341_GREEN);
    tft.setCursor(vx, y + 10);
    tft.print(IpAddress2String(Ethernet.localIP()));
    y += lh;

    // Subnet
    tft.setTextColor(ILI9341_WHITE);
    tft.setCursor(ox + p, y + 10);
    tft.print("Subnet:");
    tft.setTextColor(ILI9341_GREEN);
    tft.setCursor(vx, y + 10);
    tft.print(IpAddress2String(Ethernet.subnetMask()));
    y += lh;

    // Gateway
    tft.setTextColor(ILI9341_WHITE);
    tft.setCursor(ox + p, y + 10);
    tft.print("Gateway:");
    tft.setTextColor(ILI9341_GREEN);
    tft.setCursor(vx, y + 10);
    tft.print(IpAddress2String(Ethernet.gatewayIP()));
    y += lh;

    // DNS
    tft.setTextColor(ILI9341_WHITE);
    tft.setCursor(ox + p, y + 10);
    tft.print("DNS:");
    tft.setTextColor(ILI9341_GREEN);
    tft.setCursor(vx, y + 10);
    tft.print(IpAddress2String(Ethernet.dnsServerIP()));
    y += lh;

    // MAC
    tft.setTextColor(ILI9341_WHITE);
    tft.setCursor(ox + p, y + 10);
    tft.print("MAC:");
    tft.setTextColor(ILI9341_GREEN);
    tft.setCursor(vx, y + 10);
    tft.print(macToString(W5500_mac));
    y += lh;

    // Link
    tft.setTextColor(ILI9341_WHITE);
    tft.setCursor(ox + p, y + 10);
    tft.print("Link:");
    bool linkUp = (Ethernet.linkStatus() == LinkON);
    tft.setTextColor(linkUp ? ILI9341_GREEN : ILI9341_RED);
    tft.setCursor(vx, y + 10);
    tft.print(linkUp ? "Connected" : "Disconnected");
    y += lh;

    // Outdoor section divider
    y += 2;
    tft.drawFastHLine(ox + p, y, ow - 2 * p, 0x4208);
    y += 4;

    tft.setFont(&FreeSansBold9pt7b);
    tft.setTextColor(accentColor);
    tft.setCursor(ox + p, y + 12);
    tft.print("Outdoor Unit");
    y += lh;

    tft.setFont(&FreeSans9pt7b);
    if (outdoorConfigured) {
      tft.setTextColor(ILI9341_WHITE);
      tft.setCursor(ox + p, y + 10);
      tft.print("Remote:");
      tft.setTextColor(ILI9341_GREEN);
      tft.setCursor(vx, y + 10);
      tft.print(IpAddress2String(outdoorIP) + ":" + String(outdoorPort));
      y += lh;

      tft.setTextColor(ILI9341_WHITE);
      tft.setCursor(ox + p, y + 10);
      tft.print("Status:");
      if (outdoorStatus == "Online") {
        tft.setTextColor(ILI9341_GREEN);
        tft.setCursor(vx, y + 10);
        tft.print("Online  " + outdoorTemp + "\xB0""F");
      } else if (outdoorStatus == "Offline") {
        tft.setTextColor(ILI9341_RED);
        tft.setCursor(vx, y + 10);
        tft.print("Offline");
      } else {
        tft.setTextColor(0x7BEF);
        tft.setCursor(vx, y + 10);
        tft.print("N/A");
      }
    } else {
      tft.setTextColor(0x7BEF);
      tft.setCursor(ox + p + 80, y + 10);
      tft.print("Not configured");
    }

    // Footer: Tap for WiFi
    tft.setFont(&FreeSans9pt7b);
    tft.setTextColor(accentColor);
    String tapMsg = "Tap for WiFi";
    int16_t tx1, ty1; uint16_t ttw, tth;
    tft.getTextBounds(tapMsg, 0, 0, &tx1, &ty1, &ttw, &tth);
    tft.setCursor(ox + (ow - (int)ttw) / 2, oy + oh - 6);
    tft.print(tapMsg);
  }

#elif defined(HAS_WIFI)
  // ===================== WiFi Overlay (blue border) =====================
  uint16_t accentColor = 0x653F;  // #60a5fa blue
  int ox = 6, oy = 3, ow = 308, oh = 234;
  int p = 8, lh = 18;
  int vxL = ox + p + 60;   // left value column
  int col2 = ox + 160;     // right label column
  int vxR = col2 + 52;     // right value column

  // Double border
  tft.drawRect(ox, oy, ow, oh, accentColor);
  tft.drawRect(ox + 2, oy + 2, ow - 4, oh - 4, accentColor);

  // Header
  tft.setFont(&FreeSansBold9pt7b);
  tft.setTextColor(accentColor);
  tft.setCursor(ox + p, oy + 20);
  tft.print("Network Settings");
  tft.setFont(&FreeSans9pt7b);
  tft.setCursor(ox + ow - p - 58, oy + 18);
  tft.print("WiFi");
  tft.drawFastHLine(ox + p, oy + 25, ow - 2 * p, accentColor);

  int y = oy + 30;
  tft.setFont(&FreeSans9pt7b);

  // SSID (full-width)
  tft.setTextColor(0xAD55);  // #AAA grey
  tft.setCursor(ox + p, y + 10);
  tft.print("SSID:");
  tft.setTextColor(ILI9341_WHITE);
  // Truncate SSID with ellipsis if needed
  String ssidStr = WiFi.SSID();
  int maxSsidPx = ox + ow - p - vxL;
  int16_t x1, y1; uint16_t tw, th;
  tft.getTextBounds(ssidStr, 0, 0, &x1, &y1, &tw, &th);
  if ((int)tw > maxSsidPx) {
    while (ssidStr.length() > 0) {
      String test = ssidStr + "...";
      tft.getTextBounds(test, 0, 0, &x1, &y1, &tw, &th);
      if ((int)tw <= maxSsidPx) break;
      ssidStr = ssidStr.substring(0, ssidStr.length() - 1);
    }
    ssidStr += "...";
  }
  tft.setCursor(vxL, y + 10);
  tft.print(ssidStr);
  y += lh;

  // Two-column section: left = IP, Subnet, Gateway; right = Status, Battery
  int ly = y, ry = y;

  // Left column: IP
  tft.setTextColor(0xAD55);
  tft.setCursor(ox + p, ly + 10);
  tft.print("IP:");
  tft.setTextColor(ILI9341_WHITE);
  tft.setCursor(vxL, ly + 10);
  tft.print(IpAddress2String(WiFi.localIP()));
  ly += lh;

  // Right column: Status
  tft.setTextColor(0xAD55);
  tft.setCursor(col2, ry + 10);
  tft.print("Status:");
  tft.setTextColor((WiFi.status() == WL_CONNECTED) ? ILI9341_GREEN : ILI9341_RED);
  tft.setCursor(vxR, ry + 10);
  tft.print((WiFi.status() == WL_CONNECTED) ? "Connected" : "Disconnected");
  ry += lh;

  // Left column: Subnet
  tft.setTextColor(0xAD55);
  tft.setCursor(ox + p, ly + 10);
  tft.print("Subnet:");
  tft.setTextColor(ILI9341_WHITE);
  tft.setCursor(vxL, ly + 10);
  tft.print(IpAddress2String(WiFi.subnetMask()));
  ly += lh;

#if defined(HAS_BATT)
  // Right column: Battery
  float battV = readBatteryVoltage();
  tft.setTextColor(0xAD55);
  tft.setCursor(col2, ry + 10);
  tft.print("Battery:");
  tft.setTextColor(battV > 3.5 ? ILI9341_GREEN : battV <= 3.3 ? ILI9341_RED : 0x9CC0);
  tft.setCursor(vxR, ry + 10);
  tft.print(String(battV, 2) + "V");
  ry += lh;
#endif

  // Left column: Gateway
  tft.setTextColor(0xAD55);
  tft.setCursor(ox + p, ly + 10);
  tft.print("Gateway:");
  tft.setTextColor(ILI9341_WHITE);
  tft.setCursor(vxL, ly + 10);
  tft.print(IpAddress2String(WiFi.gatewayIP()));
  ly += lh;

  y = max(ly, ry);

  // MAC (full-width)
  tft.setTextColor(0xAD55);
  tft.setCursor(ox + p, y + 10);
  tft.print("MAC:");
  tft.setTextColor(ILI9341_WHITE);
  byte wMac[6]; WiFi.macAddress(wMac);
  tft.setCursor(vxL, y + 10);
  tft.print(macToString(wMac));
  y += lh;

  // Security + RSSI combined row
  tft.setTextColor(0xAD55);
  tft.setCursor(ox + p, y + 10);
  tft.print("Security:");
  tft.setTextColor(ILI9341_WHITE);
  tft.setCursor(vxL, y + 10);
  tft.print(getCurrentWifiSecurity());
  tft.setTextColor(0xAD55);
  tft.setCursor(col2, y + 10);
  tft.print("RSSI:");
  int rssi = WiFi.RSSI();
  tft.setTextColor(rssi > -50 ? ILI9341_GREEN : rssi > -70 ? ILI9341_YELLOW : ILI9341_RED);
  tft.setCursor(vxR, y + 10);
  tft.print(String(rssi) + " dBm");
  y += lh;

  // Divider
  y += 2;
  tft.drawFastHLine(ox + p, y, ow - 2 * p, 0x4208);  // dark grey
  y += 4;

  // Outdoor section
  tft.setFont(&FreeSansBold9pt7b);
  tft.setTextColor(accentColor);
  tft.setCursor(ox + p, y + 12);
  tft.print("Outdoor Unit");
  y += lh;

  tft.setFont(&FreeSans9pt7b);
  if (outdoorConfigured) {
    tft.setTextColor(0xAD55);
    tft.setCursor(ox + p, y + 10);
    tft.print("Remote:");
    tft.setTextColor(ILI9341_WHITE);
    tft.setCursor(vxL, y + 10);
    tft.print(IpAddress2String(outdoorIP) + ":" + String(outdoorPort));
    y += lh;

    tft.setTextColor(0xAD55);
    tft.setCursor(ox + p, y + 10);
    tft.print("Status:");
    if (outdoorStatus == "Online") {
      tft.setTextColor(ILI9341_GREEN);
      tft.setCursor(vxL, y + 10);
      tft.print("Online  " + outdoorTemp + "\xB0""F  " + outdoorHumidity + "%");
    } else if (outdoorStatus == "Offline") {
      tft.setTextColor(ILI9341_RED);
      tft.setCursor(vxL, y + 10);
      tft.print("Offline");
    } else {
      tft.setTextColor(0x7BEF);
      tft.setCursor(vxL, y + 10);
      tft.print("N/A");
    }
  } else {
    tft.setTextColor(0x7BEF);
    tft.setCursor(ox + p, y + 10);
    tft.print("Not configured");
  }

  // Footer
  tft.setFont(&FreeSans9pt7b);
  tft.setTextColor(accentColor);
  int16_t tx1, ty1; uint16_t ttw, tth;
  String tapMsg = "Tap anywhere to close";
  tft.getTextBounds(tapMsg, 0, 0, &tx1, &ty1, &ttw, &tth);
  tft.setCursor(ox + (ow - (int)ttw) / 2, oy + oh - 6);
  tft.print(tapMsg);

#elif defined(W5500)
  // ================== Ethernet Overlay (yellow border) ==================
  int ox = 6, oy = 3, ow = 308, oh = 234;
  int p = 8, lh = 18;
  int vx = ox + p + 60;

  // Double border
  tft.drawRect(ox, oy, ow, oh, ILI9341_YELLOW);
  tft.drawRect(ox + 2, oy + 2, ow - 4, oh - 4, ILI9341_YELLOW);

  // Header
  tft.setFont(&FreeSansBold9pt7b);
  tft.setTextColor(ILI9341_YELLOW);
  tft.setCursor(ox + p, oy + 20);
  tft.print("Network Settings");
  tft.setFont(&FreeSans9pt7b);
  tft.setCursor(ox + ow - p - 78, oy + 18);
  tft.print("Ethernet");
  tft.drawFastHLine(ox + p, oy + 25, ow - 2 * p, ILI9341_YELLOW);

  int y = oy + 30;
  tft.setFont(&FreeSans9pt7b);

  // IP
  tft.setTextColor(ILI9341_WHITE);
  tft.setCursor(ox + p, y + 10);
  tft.print("IP:");
  tft.setTextColor(ILI9341_GREEN);
  tft.setCursor(vx, y + 10);
  tft.print(IpAddress2String(Ethernet.localIP()));
  y += lh;

  // Subnet
  tft.setTextColor(ILI9341_WHITE);
  tft.setCursor(ox + p, y + 10);
  tft.print("Subnet:");
  tft.setTextColor(ILI9341_GREEN);
  tft.setCursor(vx, y + 10);
  tft.print(IpAddress2String(Ethernet.subnetMask()));
  y += lh;

  // Gateway
  tft.setTextColor(ILI9341_WHITE);
  tft.setCursor(ox + p, y + 10);
  tft.print("Gateway:");
  tft.setTextColor(ILI9341_GREEN);
  tft.setCursor(vx, y + 10);
  tft.print(IpAddress2String(Ethernet.gatewayIP()));
  y += lh;

  // DNS
  tft.setTextColor(ILI9341_WHITE);
  tft.setCursor(ox + p, y + 10);
  tft.print("DNS:");
  tft.setTextColor(ILI9341_GREEN);
  tft.setCursor(vx, y + 10);
  tft.print(IpAddress2String(Ethernet.dnsServerIP()));
  y += lh;

  // MAC
  tft.setTextColor(ILI9341_WHITE);
  tft.setCursor(ox + p, y + 10);
  tft.print("MAC:");
  tft.setTextColor(ILI9341_GREEN);
  tft.setCursor(vx, y + 10);
  tft.print(macToString(W5500_mac));
  y += lh;

  // Link
  tft.setTextColor(ILI9341_WHITE);
  tft.setCursor(ox + p, y + 10);
  tft.print("Link:");
  bool linkUp = (Ethernet.linkStatus() == LinkON);
  tft.setTextColor(linkUp ? ILI9341_GREEN : ILI9341_RED);
  tft.setCursor(vx, y + 10);
  tft.print(linkUp ? "Connected" : "Disconnected");
  y += lh;

  // Divider
  y += 2;
  tft.drawFastHLine(ox + p, y, ow - 2 * p, 0x4208);
  y += 4;

  // Outdoor section
  tft.setFont(&FreeSansBold9pt7b);
  tft.setTextColor(ILI9341_YELLOW);
  tft.setCursor(ox + p, y + 12);
  tft.print("Outdoor Unit");
  y += lh;

  tft.setFont(&FreeSans9pt7b);
  if (outdoorConfigured) {
    tft.setTextColor(ILI9341_WHITE);
    tft.setCursor(ox + p, y + 10);
    tft.print("Remote:");
    tft.setTextColor(ILI9341_GREEN);
    tft.setCursor(vx, y + 10);
    tft.print(IpAddress2String(outdoorIP) + ":" + String(outdoorPort));
    y += lh;

    tft.setTextColor(ILI9341_WHITE);
    tft.setCursor(ox + p, y + 10);
    tft.print("Status:");
    if (outdoorStatus == "Online") {
      tft.setTextColor(ILI9341_GREEN);
      tft.setCursor(vx, y + 10);
      tft.print("Online  " + outdoorTemp + "\xB0""F");
    } else if (outdoorStatus == "Offline") {
      tft.setTextColor(ILI9341_RED);
      tft.setCursor(vx, y + 10);
      tft.print("Offline");
    } else {
      tft.setTextColor(0x7BEF);
      tft.setCursor(vx, y + 10);
      tft.print("N/A");
    }
  } else {
    tft.setTextColor(0x7BEF);
    tft.setCursor(ox + p + 80, y + 10);
    tft.print("Not configured");
  }

  // Footer
  tft.setFont(&FreeSans9pt7b);
  tft.setTextColor(ILI9341_YELLOW);
  int16_t tx1, ty1; uint16_t ttw, tth;
  String tapMsg = "Tap anywhere to close";
  tft.getTextBounds(tapMsg, 0, 0, &tx1, &ty1, &ttw, &tth);
  tft.setCursor(ox + (ow - (int)ttw) / 2, oy + oh - 6);
  tft.print(tapMsg);

#else
  // ================== No Network Overlay ==================
  int ox = 6, oy = 3, ow = 308, oh = 234;
  int p = 8;

  // Double border
  tft.drawRect(ox, oy, ow, oh, ILI9341_YELLOW);
  tft.drawRect(ox + 2, oy + 2, ow - 4, oh - 4, ILI9341_YELLOW);

  // Header
  tft.setFont(&FreeSansBold9pt7b);
  tft.setTextColor(ILI9341_YELLOW);
  tft.setCursor(ox + p, oy + 20);
  tft.print("Device Settings");
  tft.drawFastHLine(ox + p, oy + 25, ow - 2 * p, ILI9341_YELLOW);

  tft.setFont(&FreeSans9pt7b);
  tft.setTextColor(0x7BEF);
  tft.setCursor(ox + p, oy + 50);
  tft.print("No network interface");
  tft.setCursor(ox + p, oy + 70);
  tft.print("configured for this board.");

  // Footer
  tft.setFont(&FreeSans9pt7b);
  tft.setTextColor(ILI9341_YELLOW);
  int16_t tx1, ty1; uint16_t ttw, tth;
  String tapMsg = "Tap anywhere to close";
  tft.getTextBounds(tapMsg, 0, 0, &tx1, &ty1, &ttw, &tth);
  tft.setCursor(ox + (ow - (int)ttw) / 2, oy + oh - 6);
  tft.print(tapMsg);
#endif
}

// ===========================================================================
// SD Card File Manager Page
// ===========================================================================

void sendSDFilesPage(HardwareClient cl) {
  sendHtmlHeaders(cl);

  if (!sdCardPresent) {
    cl.println(F("<!DOCTYPE html><html><head><title>SD Card</title></head><body style='background:#1a1a2e;color:#fff;font-family:sans-serif;padding:40px;text-align:center'>"));
    cl.println(F("<h1>&#128193; SD Card Manager</h1><p style='color:#f87171;margin:20px 0'>No SD card detected</p>"));
    cl.println(F("<a href='/' style='color:#4ade80'>&#8592; Back to Thermostat</a></body></html>"));
    return;
  }

  // Use shared CSS functions to reduce code duplication
  sendHtmlHead(cl, "SD Card Manager");
  cl.println(F("body{background:#000;padding:20px}"));
  cl.println(F(".container{max-width:800px;margin:0 auto}"));
  cl.println(F("h1{font-weight:300;margin-bottom:10px}"));
  cl.println(F(".toolbar{display:flex;gap:10px;flex-wrap:wrap;margin-bottom:15px;align-items:center}"));
  cl.println(F(".breadcrumb{background:rgba(255,255,255,0.1);padding:8px 15px;border-radius:6px;font-size:14px;flex-grow:1}"));
  cl.println(F(".breadcrumb a{color:#60a5fa;text-decoration:none}.breadcrumb a:hover{text-decoration:underline}"));
  sendButtonCSS(cl);
  sendCardCSS(cl);
  cl.println(F(".file-list{list-style:none}"));
  cl.println(F(".file-item{display:flex;align-items:center;padding:10px;border-bottom:1px solid rgba(255,255,255,0.1);gap:10px}"));
  cl.println(F(".file-item:last-child{border-bottom:none}.file-item:hover{background:rgba(255,255,255,0.05)}"));
  cl.println(F(".file-icon{font-size:20px;width:30px;text-align:center}"));
  cl.println(F(".file-name{flex-grow:1;cursor:pointer;word-break:break-all}.file-name:hover{color:#60a5fa}"));
  cl.println(F(".file-size{color:#888;font-size:12px;width:80px;text-align:right}"));
  cl.println(F(".file-actions{display:flex;gap:5px}.file-actions .btn{padding:4px 10px;font-size:12px}"));
  cl.println(F(".empty{text-align:center;color:#888;padding:40px}"));
  sendModalCSS(cl);
  cl.println(F("textarea{width:100%;height:400px;background:#0f172a;border:1px solid #334155;border-radius:6px;color:#e2e8f0;padding:12px;font-family:monospace;font-size:13px;resize:vertical}"));
  cl.println(F("textarea:focus{outline:none;border-color:#60a5fa}"));
  sendFormCSS(cl);
  sendStatusCSS(cl);
  cl.println(F(".status{display:none}"));
  cl.println(F(".info{background:rgba(96,165,250,0.1);border:1px solid #60a5fa;border-radius:6px;padding:12px;margin-bottom:15px;font-size:13px}"));
  cl.println(F(".loading{text-align:center;padding:40px;color:#888}"));
  cl.println(F("</style></head><body>"));

  cl.println(F("<div class='container'>"));
  cl.println(F("<h1>&#128193; SD Card Manager</h1>"));
  cl.println(F("<p style='color:#888;margin-bottom:20px;font-size:14px'>Manage files on SD card</p>"));

  // Toolbar
  cl.println(F("<div class='toolbar'>"));
  cl.println(F("<div class='breadcrumb' id='breadcrumb'>/</div>"));
  cl.println(F("<button class='btn btn-primary' onclick='showNewFile()'>&#10133; New</button>"));
  cl.println(F("<button class='btn btn-secondary' onclick='showUpload()'>&#8593; Upload</button>"));
  cl.println(F("<button class='btn btn-danger' onclick='confirmFormat()'>&#128465; Format</button>"));
  cl.println(F("</div>"));

  // File list card
  cl.println(F("<div class='card'>"));
  cl.println(F("<ul class='file-list' id='fileList'><li class='loading'>Loading...</li></ul>"));
  cl.println(F("</div>"));

  // Back link
  cl.println(F("<div style='text-align:center;margin-top:20px'>"));
  cl.println(F("<a href='/' style='color:#4ade80;text-decoration:none'>&#8592; Back to Thermostat</a>"));
  cl.println(F(" &nbsp;|&nbsp; "));
  cl.println(F("<a href='/config' style='color:#60a5fa;text-decoration:none'>&#9881; Configuration</a>"));
  cl.println(F(" &nbsp;|&nbsp; "));
  cl.println(F("<a href='/update' style='color:#888;text-decoration:none'>&#128295; Firmware Update</a>"));
  cl.println(F("</div>"));

  cl.println(F("</div>")); // end container

  // Editor Modal
  cl.println(F("<div class='modal' id='editorModal'>"));
  cl.println(F("<div class='modal-content'>"));
  cl.println(F("<div class='modal-header'>"));
  cl.println(F("<h2 id='editorTitle'>Edit File</h2>"));
  cl.println(F("<button class='modal-close' onclick='closeEditor()'>&times;</button>"));
  cl.println(F("</div>"));
  cl.println(F("<div class='modal-body'>"));
  cl.println(F("<textarea id='editorContent' spellcheck='false'></textarea>"));
  cl.println(F("<div class='status' id='editorStatus'></div>"));
  cl.println(F("</div>"));
  cl.println(F("<div class='modal-footer'>"));
  cl.println(F("<button class='btn btn-secondary' onclick='closeEditor()'>Cancel</button>"));
  cl.println(F("<button class='btn btn-primary' id='saveBtn' onclick='saveFile()'>Save</button>"));
  cl.println(F("</div>"));
  cl.println(F("</div></div>"));

  // New File Modal
  cl.println(F("<div class='modal' id='newFileModal'>"));
  cl.println(F("<div class='modal-content' style='max-width:400px'>"));
  cl.println(F("<div class='modal-header'>"));
  cl.println(F("<h2>New File</h2>"));
  cl.println(F("<button class='modal-close' onclick='closeNewFile()'>&times;</button>"));
  cl.println(F("</div>"));
  cl.println(F("<div class='modal-body'>"));
  cl.println(F("<div class='input-group'>"));
  cl.println(F("<label>Filename</label>"));
  cl.println(F("<input type='text' id='newFileName' placeholder='example.txt'>"));
  cl.println(F("</div>"));
  cl.println(F("<div class='status' id='newFileStatus'></div>"));
  cl.println(F("</div>"));
  cl.println(F("<div class='modal-footer'>"));
  cl.println(F("<button class='btn btn-secondary' onclick='closeNewFile()'>Cancel</button>"));
  cl.println(F("<button class='btn btn-primary' onclick='createFile()'>Create</button>"));
  cl.println(F("</div>"));
  cl.println(F("</div></div>"));

  // Format Confirmation Modal
  cl.println(F("<div class='modal' id='formatModal'>"));
  cl.println(F("<div class='modal-content' style='max-width:420px'>"));
  cl.println(F("<div class='modal-header'>"));
  cl.println(F("<h2 style='color:#ef4444'>&#9888; Format SD Card</h2>"));
  cl.println(F("<button class='modal-close' onclick='closeFormat()'>&times;</button>"));
  cl.println(F("</div>"));
  cl.println(F("<div class='modal-body'>"));
  cl.println(F("<div style='margin-bottom:15px'>"));
  cl.println(F("<label style='display:block;margin-bottom:5px;font-weight:bold'>Format Type:</label>"));
  cl.println(F("<select id='formatType' style='width:100%;padding:8px;background:#374151;color:#e5e7eb;border:1px solid #4b5563;border-radius:4px'>"));
  cl.println(F("<option value='quick'>Quick (delete files only)</option>"));
  cl.println(F("<option value='auto'>Auto (FAT32 or exFAT based on size)</option>"));
  cl.println(F("<option value='fat32'>FAT32 (for cards up to 32GB)</option>"));
  cl.println(F("<option value='exfat'>exFAT (for any size card)</option>"));
  cl.println(F("</select>"));
  cl.println(F("</div>"));
  cl.println(F("<div style='background:#1f2937;padding:10px;border-radius:4px;margin-bottom:15px;font-size:13px'>"));
  cl.println(F("<p style='margin:0 0 8px 0'><strong>Quick:</strong> Deletes files, keeps filesystem</p>"));
  cl.println(F("<p style='margin:0 0 8px 0'><strong>Auto:</strong> Full format, auto-selects best type</p>"));
  cl.println(F("<p style='margin:0 0 8px 0'><strong>FAT32:</strong> Compatible with most devices</p>"));
  cl.println(F("<p style='margin:0'><strong>exFAT:</strong> Best for large files &gt;4GB</p>"));
  cl.println(F("</div>"));
  cl.println(F("<p style='color:#fca5a5;font-size:13px;margin-bottom:10px'>&#9888; All data will be erased! This cannot be undone.</p>"));
  cl.println(F("<div class='status' id='formatStatus'></div>"));
  cl.println(F("</div>"));
  cl.println(F("<div class='modal-footer'>"));
  cl.println(F("<button class='btn btn-secondary' onclick='closeFormat()'>Cancel</button>"));
  cl.println(F("<button class='btn btn-danger' id='formatBtn' onclick='doFormat()'>Format SD Card</button>"));
  cl.println(F("</div>"));
  cl.println(F("</div></div>"));

  // Upload Modal
  cl.println(F("<div class='modal' id='uploadModal'>"));
  cl.println(F("<div class='modal-content' style='max-width:450px'>"));
  cl.println(F("<div class='modal-header'>"));
  cl.println(F("<h2>&#8593; Upload File</h2>"));
  cl.println(F("<button class='modal-close' onclick='closeUpload()'>&times;</button>"));
  cl.println(F("</div>"));
  cl.println(F("<div class='modal-body'>"));
  cl.println(F("<div class='input-group'>"));
  cl.println(F("<label>Select File</label>"));
  cl.println(F("<input type='file' id='uploadFile' onchange='fileSelected()'>"));
  cl.println(F("</div>"));
  cl.println(F("<div id='uploadInfo' style='font-size:13px;color:#888;margin-bottom:10px'></div>"));
  cl.println(F("<div style='background:#0f172a;border-radius:6px;height:8px;overflow:hidden;display:none' id='uploadProgress'>"));
  cl.println(F("<div id='uploadBar' style='background:#4ade80;height:100%;width:0%;transition:width 0.2s'></div>"));
  cl.println(F("</div>"));
  cl.println(F("<div class='status' id='uploadStatus'></div>"));
  cl.println(F("</div>"));
  cl.println(F("<div class='modal-footer'>"));
  cl.println(F("<button class='btn btn-secondary' onclick='closeUpload()'>Cancel</button>"));
  cl.println(F("<button class='btn btn-primary' id='uploadBtn' onclick='doUpload()' disabled>Upload</button>"));
  cl.println(F("</div>"));
  cl.println(F("</div></div>"));

  // JavaScript
  cl.println(F("<script>function $(i){return document.getElementById(i)}"));
  cl.println(F("var currentPath='/',currentFile=null,isNewFile=false,cardSizeMB=0;"));
  // Load directory
  cl.println(F("function loadDir(path){currentPath=path;updateBreadcrumb();$('fileList').innerHTML='<li class=\"loading\">Loading...</li>';"));
  cl.println(F("fetch('/api/sdlist?path='+encodeURIComponent(path)).then(r=>r.json()).then(data=>{"));
  cl.println(F("if(!data.success){showError(data.error);return;}if(data.cardSizeMB)cardSizeMB=data.cardSizeMB;updateAutoFormatLabel();renderFiles(data.files);"));
  cl.println(F("}).catch(e=>showError('Load failed: '+e));}"));
  // Update Auto format label based on card size
  cl.println(F("function updateAutoFormatLabel(){var sel=$('formatType');if(!sel)return;var o=sel.querySelector('option[value=\"auto\"]');if(!o)return;"));
  cl.println(F("var fmt=cardSizeMB>32768?'exFAT':cardSizeMB>2048?'FAT32':'FAT16';"));
  cl.println(F("o.textContent='Auto ('+fmt+' for '+(cardSizeMB>=1024?(cardSizeMB/1024).toFixed(1)+' GB':cardSizeMB+' MB')+' card)';}"));
  // Update breadcrumb
  cl.println(F("function updateBreadcrumb(){var bc=$('breadcrumb'),parts=currentPath.split('/').filter(p=>p);"));
  cl.println(F("var html='<a href=\"#\" onclick=\"loadDir(\\'/\\');return false\">/</a>',p='/';"));
  cl.println(F("for(var i=0;i<parts.length;i++){p+='/'+parts[i];html+=' <a href=\"#\" onclick=\"loadDir(\\''+p+'\\');return false\">'+parts[i]+'</a>/';}bc.innerHTML=html;}"));
  // Render file list
  cl.println(F("function renderFiles(files){var list=$('fileList');if(!files||files.length===0){list.innerHTML='<li class=\"empty\">No files</li>';return;}"));

  // Sort: dirs first, then by name
  cl.println(F("files.sort((a,b)=>{if(a.type!==b.type)return a.type==='dir'?-1:1;return a.name.localeCompare(b.name);});"));

  // Add parent dir if not at root
  cl.println(F("var html='';"));
  cl.println(F("if(currentPath!=='/'){"));
  cl.println(F("html+='<li class=\"file-item\" onclick=\"goUp()\"><span class=\"file-icon\">&#128193;</span><span class=\"file-name\">..</span><span class=\"file-size\">Parent</span><span class=\"file-actions\"></span></li>';"));
  cl.println(F("}"));

  cl.println(F("for(var i=0;i<files.length;i++){"));
  cl.println(F("var f=files[i];"));
  cl.println(F("var icon=f.type==='dir'?'&#128193;':'&#128196;';"));
  cl.println(F("var size=f.type==='dir'?'--':formatSize(f.size);"));
  cl.println(F("var actions='';"));

  // Actions based on file type
  cl.println(F("if(f.type==='file'){"));
  cl.println(F("if(isEditable(f.name)){actions+='<button class=\"btn btn-primary\" onclick=\"editFile(\\''+f.name+'\\');event.stopPropagation()\">Edit</button>';}"));
  cl.println(F("actions+='<button class=\"btn btn-secondary\" onclick=\"downloadFile(\\''+f.name+'\\');event.stopPropagation()\">&#8595;</button>';"));
  cl.println(F("actions+='<button class=\"btn btn-danger\" onclick=\"deleteFile(\\''+f.name+'\\');event.stopPropagation()\">&times;</button>';"));
  cl.println(F("}else{"));
  cl.println(F("actions+='<button class=\"btn btn-danger\" onclick=\"deleteFile(\\''+f.name+'\\');event.stopPropagation()\">&times;</button>';"));
  cl.println(F("}"));

  cl.println(F("html+='<li class=\"file-item\" onclick=\"itemClick(\\''+f.name+'\\',\\''+f.type+'\\')\"><span class=\"file-icon\">'+icon+'</span><span class=\"file-name\">'+f.name+'</span><span class=\"file-size\">'+size+'</span><span class=\"file-actions\">'+actions+'</span></li>';"));
  cl.println(F("}"));
  cl.println(F("list.innerHTML=html;"));
  cl.println(F("}"));

  // Go up one directory
  cl.println(F("function goUp(){"));
  cl.println(F("var parts=currentPath.split('/').filter(p=>p);"));
  cl.println(F("parts.pop();"));
  cl.println(F("loadDir('/'+parts.join('/'));"));
  cl.println(F("}"));

  // Item click handler
  cl.println(F("function itemClick(name,type){"));
  cl.println(F("if(type==='dir'){"));
  cl.println(F("var newPath=currentPath==='/'?'/'+name:currentPath+'/'+name;"));
  cl.println(F("loadDir(newPath);"));
  cl.println(F("}else if(isEditable(name)){"));
  cl.println(F("editFile(name);"));
  cl.println(F("}"));
  cl.println(F("}"));

  // Format file size
  cl.println(F("function formatSize(bytes){"));
  cl.println(F("if(bytes<1024)return bytes+' B';"));
  cl.println(F("if(bytes<1048576)return(bytes/1024).toFixed(1)+' KB';"));
  cl.println(F("return(bytes/1048576).toFixed(1)+' MB';"));
  cl.println(F("}"));

  // Check if file is editable
  cl.println(F("function isEditable(name){"));
  cl.println(F("var ext=name.toLowerCase().split('.').pop();"));
  cl.println(F("return['txt','html','htm','css','js','json','cfg','xml','md','log','csv','ini'].indexOf(ext)>=0;"));
  cl.println(F("}"));

  // Get full path for file
  cl.println(F("function getFullPath(name){"));
  cl.println(F("return currentPath==='/'?'/'+name:currentPath+'/'+name;"));
  cl.println(F("}"));

  // Edit file
  cl.println(F("function editFile(name){currentFile=name;isNewFile=false;$('editorTitle').textContent='Edit: '+name;$('editorContent').value='Loading...';"));
  cl.println(F("$('editorStatus').className='status';$('editorModal').classList.add('active');"));
  cl.println(F("fetch('/api/sdread?path='+encodeURIComponent(getFullPath(name))).then(r=>r.json()).then(data=>{"));
  cl.println(F("$('editorContent').value=data.success?data.content:'Error: '+data.error;}).catch(e=>{$('editorContent').value='Error: '+e;});}"));
  // Save file
  cl.println(F("function saveFile(){var b=$('saveBtn'),st=$('editorStatus');b.disabled=true;b.textContent='Saving...';st.className='status';"));
  cl.println(F("var path=isNewFile?'/'+currentFile:getFullPath(currentFile);"));
  cl.println(F("fetch('/api/sdwrite',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({path:path,content:$('editorContent').value})})"));
  cl.println(F(".then(r=>r.json()).then(data=>{b.disabled=false;b.textContent='Save';"));
  cl.println(F("if(data.success){st.textContent='Saved ('+data.size+' bytes)';st.className='status success';setTimeout(()=>{closeEditor();loadDir(currentPath);},1000);}"));
  cl.println(F("else{st.textContent='Error: '+data.error;st.className='status error';}"));
  cl.println(F("}).catch(e=>{b.disabled=false;b.textContent='Save';st.textContent='Error: '+e;st.className='status error';});}"));
  // Close editor
  cl.println(F("function closeEditor(){$('editorModal').classList.remove('active');currentFile=null;}"));
  // Download file
  cl.println(F("function downloadFile(name){location.href='/api/sddownload?path='+encodeURIComponent(getFullPath(name));}"));
  // Delete file
  cl.println(F("function deleteFile(name){if(!confirm('Delete \"'+name+'\"?'))return;"));
  cl.println(F("fetch('/api/sddelete',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({path:getFullPath(name)})})"));
  cl.println(F(".then(r=>r.json()).then(data=>{if(data.success)loadDir(currentPath);else alert('Delete failed: '+data.error);}).catch(e=>alert('Delete failed: '+e));}"));
  // Show new file modal
  cl.println(F("function showNewFile(){$('newFileName').value='';$('newFileStatus').className='status';$('newFileModal').classList.add('active');$('newFileName').focus();}"));
  // Close new file modal
  cl.println(F("function closeNewFile(){$('newFileModal').classList.remove('active');}"));
  // Create new file
  cl.println(F("function createFile(){var name=$('newFileName').value.trim();if(!name){$('newFileStatus').textContent='Enter a filename';$('newFileStatus').className='status error';return;}"));
  cl.println(F("closeNewFile();currentFile=name;isNewFile=true;$('editorTitle').textContent='New: '+name;$('editorContent').value='';$('editorStatus').className='status';$('editorModal').classList.add('active');}"));
  // Format SD card
  cl.println(F("function confirmFormat(){$('formatStatus').className='status';$('formatModal').classList.add('active');}"));
  cl.println(F("function closeFormat(){$('formatModal').classList.remove('active');}"));
  cl.println(F("function doFormat(){var b=$('formatBtn'),ft=$('formatType').value,st=$('formatStatus');b.disabled=true;"));
  cl.println(F("b.textContent=ft==='quick'?'Deleting files...':'Formatting...';st.className='status';st.textContent='';"));
  cl.println(F("fetch('/api/sdformat',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({confirm:true,type:ft})})"));
  cl.println(F(".then(r=>r.json()).then(data=>{b.disabled=false;b.textContent='Format SD Card';"));
  cl.println(F("if(data.success){st.textContent=data.message||'Done!';st.className='status success';setTimeout(()=>{closeFormat();loadDir('/');},2000);}"));
  cl.println(F("else{st.textContent='Error: '+data.error;st.className='status error';}"));
  cl.println(F("}).catch(e=>{b.disabled=false;b.textContent='Format SD Card';st.textContent='Error: '+e;st.className='status error';});}"));
  // Show error
  cl.println(F("function showError(msg){$('fileList').innerHTML='<li class=\"empty\" style=\"color:#fca5a5\">Error: '+msg+'</li>';}"));
  // Upload functions
  cl.println(F("function showUpload(){$('uploadFile').value='';$('uploadInfo').textContent='';$('uploadStatus').className='status';"));
  cl.println(F("$('uploadProgress').style.display='none';$('uploadBar').style.width='0%';$('uploadBtn').disabled=true;$('uploadModal').classList.add('active');}"));
  cl.println(F("function closeUpload(){$('uploadModal').classList.remove('active');}"));
  cl.println(F("function fileSelected(){var f=$('uploadFile').files[0];if(f){$('uploadInfo').textContent=f.name+' ('+formatSize(f.size)+')';$('uploadBtn').disabled=false;}"));
  cl.println(F("else{$('uploadInfo').textContent='';$('uploadBtn').disabled=true;}}"));
  cl.println(F("function doUpload(){var f=$('uploadFile').files[0];if(!f)return;var b=$('uploadBtn'),st=$('uploadStatus'),prog=$('uploadProgress'),bar=$('uploadBar');"));
  cl.println(F("b.disabled=true;b.textContent='Uploading...';prog.style.display='block';st.className='status';"));

  cl.println(F("var fd=new FormData();fd.append('path',currentPath);fd.append('file',f);var xhr=new XMLHttpRequest();"));
  cl.println(F("xhr.open('POST','/api/sdfileupload',true);xhr.upload.onprogress=function(e){if(e.lengthComputable)bar.style.width=(e.loaded/e.total*100)+'%';};"));
  cl.println(F("xhr.onload=function(){b.disabled=false;b.textContent='Upload';try{var r=JSON.parse(xhr.responseText);"));
  cl.println(F("if(r.success){st.textContent='Uploaded: '+r.filename+' ('+formatSize(r.size)+')';st.className='status success';setTimeout(()=>{closeUpload();loadDir(currentPath);},1200);}"));
  cl.println(F("else{st.textContent='Error: '+r.error;st.className='status error';}}catch(e){st.textContent='Error: '+xhr.responseText;st.className='status error';}};"));
  cl.println(F("xhr.onerror=function(){b.disabled=false;b.textContent='Upload';st.textContent='Upload failed';st.className='status error';};xhr.send(fd);}"));
  // Initial load
  cl.println(F("loadDir('/');"));

  cl.println(F("</script>"));
  cl.println(F("</body></html>"));
}

// ===========================================================================
// OTA Firmware Update Page
// ===========================================================================

void sendUpdatePage(HardwareClient cl) {
  // Use shared CSS functions for consistent styling
  sendHtmlHead(cl, "Firmware Update");
  cl.println(F("body{display:flex;justify-content:center;align-items:center;padding:20px}"));
  cl.println(F(".container{text-align:center;max-width:500px;width:100%}"));
  cl.println(F("h1{margin-bottom:20px;font-weight:300}h2{font-weight:400;margin-bottom:15px;font-size:18px}"));
  sendCardCSS(cl);
  cl.println(F(".card{padding:30px}"));
  sendButtonCSS(cl);
  cl.println(F(".btn{padding:12px 30px;font-size:15px}"));
  cl.println(F(".file-input{display:none}"));
  cl.println(F(".file-label{display:block;padding:40px 20px;border:2px dashed #666;border-radius:8px;cursor:pointer;transition:all 0.2s;margin-bottom:20px}"));
  cl.println(F(".file-label:hover{border-color:#4ade80;background:rgba(74,222,128,0.1)}"));
  cl.println(F(".file-label.selected{border-color:#4ade80;border-style:solid}"));
  sendProgressCSS(cl);
  sendStatusCSS(cl);
  cl.println(F(".warning{background:rgba(245,158,11,0.15);border:1px solid #f59e0b;border-radius:8px;padding:15px;margin-bottom:20px;text-align:left}"));
  cl.println(F(".warning h3{color:#f59e0b;margin-bottom:10px}.warning ul{margin-left:20px}"));
  cl.println(F(".divider{border-top:1px solid rgba(255,255,255,0.15);margin:25px 0}"));
  cl.println(F(".crc{font-family:monospace;font-size:13px;color:#aaa;margin-top:8px}"));
  cl.println(F("</style></head><body>"));
  cl.println(F("<div class='container'>"));
  cl.println(F("<h1>&#128295; Firmware Update</h1>"));

  // --- Warning box (always shown) ---
  cl.println(F("<div class='card'>"));
  cl.println(F("<div class='warning'>"));
  cl.println(F("<h3>&#9888;&#65039; Warning</h3>"));
  cl.println(F("<ul>"));
  cl.println(F("<li>Do not power off during update</li>"));
  cl.println(F("<li>Use only .bin files compiled for this board</li>"));
  cl.println(F("<li>Device will reboot after successful update</li>"));
  cl.println(F("</ul></div>"));

  // --- OTA section (compile-time gate) ---
#if OTA_SUPPORTED
  cl.println(F("<h2>&#128225; Network OTA Update</h2>"));
  cl.println(F("<form id='otaForm'>"));
  cl.println(F("<label class='file-label' id='otaLabel'>"));
  cl.println(F("<input type='file' class='file-input' id='otaFile' accept='.bin'>"));
  cl.println(F("<span id='otaName'>&#128193; Click to select firmware file (.bin)</span>"));
  cl.println(F("</label>"));
  cl.println(F("<button type='submit' class='btn btn-primary' id='otaBtn' disabled>Upload &amp; Flash</button>"));
  cl.println(F("</form>"));
  cl.println(F("<div class='progress' id='otaProgress'>"));
  cl.println(F("<div class='progress-bar'><div class='progress-fill' id='otaFill'></div></div>"));
  cl.println(F("<div class='status' id='otaStatus'>Uploading...</div>"));
  cl.println(F("</div>"));
#endif

  // --- SD card upload section (runtime gate) ---
  if (sdCardPresent) {
#if OTA_SUPPORTED
    cl.println(F("<div class='divider'></div>"));
#endif
    cl.println(F("<h2>&#128190; SD Card Firmware Update</h2>"));
    cl.println(F("<form id='sdForm'>"));
    cl.println(F("<label class='file-label' id='sdLabel'>"));
    cl.println(F("<input type='file' class='file-input' id='sdFile' accept='.bin'>"));
    cl.println(F("<span id='sdName'>&#128193; Click to select firmware file (.bin)</span>"));
    cl.println(F("</label>"));
    cl.println(F("<button type='submit' class='btn btn-primary' id='sdBtn' disabled>Upload to SD Card</button>"));
    cl.println(F("</form>"));
    cl.println(F("<div class='progress' id='sdProgress'>"));
    cl.println(F("<div class='progress-bar'><div class='progress-fill' id='sdFill'></div></div>"));
    cl.println(F("<div class='status' id='sdStatus'>Uploading...</div>"));
    cl.println(F("</div>"));
    cl.println(F("<div class='crc' id='sdCrc'></div>"));
  }

  // --- "not supported" only if no OTA AND no SD ---
#if !OTA_SUPPORTED
  if (!sdCardPresent) {
    cl.println(F("<p>OTA updates are not supported on this board and no SD card is present.</p>"));
    cl.println(F("<p>Please use USB to flash new firmware.</p>"));
  }
#endif

  // --- Divider before reboot ---
  cl.println(F("<div class='divider'></div>"));

  // --- Reboot button (always available) ---
  cl.println(F("<button class='btn btn-warning' id='rebootBtn' onclick='doReboot()'>&#128260; Reboot Now</button>"));
  cl.println(F("<div class='status' id='rebootStatus'></div>"));

  cl.println(F("</div>"));  // end card
  cl.println(F("<a href='/'><button class='btn btn-secondary' style='margin-top:20px'>&#8592; Back to Thermostat</button></a>"));
  cl.println(F("</div>"));  // end container

  // --- JavaScript ---
  cl.println(F("<script>"));

  // Reboot function (always available)
  cl.println(F("function doReboot() {"));
  cl.println(F("  if (!confirm('Reboot the thermostat now?')) return;"));
  cl.println(F("  document.getElementById('rebootStatus').textContent = 'Rebooting...';"));
  cl.println(F("  fetch('/api/reboot').then(() => {"));
  cl.println(F("    document.getElementById('rebootStatus').textContent = 'Rebooted. Reconnecting...';"));
  cl.println(F("    document.getElementById('rebootStatus').className = 'status success';"));
  cl.println(F("    setTimeout(() => location.href = '/', 8000);"));
  cl.println(F("  }).catch(() => {"));
  cl.println(F("    document.getElementById('rebootStatus').textContent = 'Reboot sent (connection lost -- expected).';"));
  cl.println(F("    document.getElementById('rebootStatus').className = 'status success';"));
  cl.println(F("    setTimeout(() => location.href = '/', 8000);"));
  cl.println(F("  });"));
  cl.println(F("}"));

  // Helper: setup a file picker + XHR upload
  cl.println(F("function setupUpload(fileId, labelId, nameId, btnId, formId, progressId, fillId, statusId, url, onSuccess) {"));
  cl.println(F("  var fi = document.getElementById(fileId);"));
  cl.println(F("  if (!fi) return;"));
  cl.println(F("  var lb = document.getElementById(labelId);"));
  cl.println(F("  var nm = document.getElementById(nameId);"));
  cl.println(F("  var bt = document.getElementById(btnId);"));
  cl.println(F("  var fm = document.getElementById(formId);"));
  cl.println(F("  var pg = document.getElementById(progressId);"));
  cl.println(F("  var fl = document.getElementById(fillId);"));
  cl.println(F("  var st = document.getElementById(statusId);"));
  cl.println(F("  fi.addEventListener('change', function() {"));
  cl.println(F("    if (this.files.length > 0) {"));
  cl.println(F("      var f = this.files[0];"));
  cl.println(F("      if (!f.name.endsWith('.bin')) { alert('Please select a .bin file'); this.value=''; return; }"));
  cl.println(F("      nm.textContent = '\\uD83D\\uDCC4 ' + f.name + ' (' + (f.size/1024).toFixed(1) + ' KB)';"));
  cl.println(F("      lb.classList.add('selected');"));
  cl.println(F("      bt.disabled = false;"));
  cl.println(F("    }"));
  cl.println(F("  });"));
  cl.println(F("  fm.addEventListener('submit', function(e) {"));
  cl.println(F("    e.preventDefault();"));
  cl.println(F("    var f = fi.files[0]; if (!f) return;"));
  cl.println(F("    bt.disabled = true;"));
  cl.println(F("    pg.style.display = 'block';"));
  cl.println(F("    st.textContent = 'Uploading...';"));
  cl.println(F("    st.className = 'status';"));
  cl.println(F("    var xhr = new XMLHttpRequest();"));
  cl.println(F("    xhr.open('POST', url, true);"));
  cl.println(F("    xhr.setRequestHeader('Content-Type', 'application/octet-stream');"));
  cl.println(F("    xhr.upload.onprogress = function(ev) {"));
  cl.println(F("      if (ev.lengthComputable) {"));
  cl.println(F("        var pct = (ev.loaded / ev.total) * 100;"));
  cl.println(F("        fl.style.width = pct + '%';"));
  cl.println(F("        st.textContent = 'Uploading: ' + pct.toFixed(0) + '%';"));
  cl.println(F("      }"));
  cl.println(F("    };"));
  cl.println(F("    xhr.onload = function() {"));
  cl.println(F("      if (xhr.status === 200) {"));
  cl.println(F("        try { var r = JSON.parse(xhr.responseText); } catch(x) {"));
  cl.println(F("          st.textContent = '\\u2717 Bad response'; st.className='status error'; bt.disabled=false; return; }"));
  cl.println(F("        if (r.success) { onSuccess(r, st); }"));
  cl.println(F("        else { st.textContent = '\\u2717 ' + r.error; st.className='status error'; bt.disabled=false; }"));
  cl.println(F("      } else { st.textContent = '\\u2717 HTTP ' + xhr.status; st.className='status error'; bt.disabled=false; }"));
  cl.println(F("    };"));
  cl.println(F("    xhr.onerror = function() { st.textContent = '\\u2717 Connection error'; st.className='status error'; bt.disabled=false; };"));
  cl.println(F("    xhr.send(f);"));
  cl.println(F("  });"));
  cl.println(F("}"));

  // OTA upload handler
#if OTA_SUPPORTED
  cl.println(F("setupUpload('otaFile','otaLabel','otaName','otaBtn','otaForm','otaProgress','otaFill','otaStatus',"));
  cl.println(F("  '/api/update', function(r, st) {"));
  cl.println(F("    st.textContent = '\\u2713 Update successful! Rebooting...';"));
  cl.println(F("    st.className = 'status success';"));
  cl.println(F("    setTimeout(function() { location.href = '/'; }, 5000);"));
  cl.println(F("});"));
#endif

  // SD upload handler
  if (sdCardPresent) {
    cl.println(F("setupUpload('sdFile','sdLabel','sdName','sdBtn','sdForm','sdProgress','sdFill','sdStatus',"));
    cl.println(F("  '/api/sdupload', function(r, st) {"));
    cl.println(F("    st.textContent = '\\u2713 Uploaded to SD card. Click Reboot Now to apply.';"));
    cl.println(F("    st.className = 'status success';"));
    cl.println(F("    if (r.crc32) document.getElementById('sdCrc').textContent = 'CRC32: ' + r.crc32 + '  Size: ' + r.size + ' bytes';"));
    cl.println(F("});"));
  }

  cl.println(F("</script>"));
  cl.println(F("</body></html>"));
}

// ===========================================================================
// Handle firmware upload via POST /api/update
// ===========================================================================

void handleFirmwareUpload(HardwareClient& client, long contentLength) {
#if OTA_SUPPORTED
  Serial.println(F("Firmware upload started"));
  Serial.print(F("Expected size: "));
  Serial.println(contentLength);

  if (contentLength < 1000) {
    client.println(F("HTTP/1.1 200 OK"));
    client.println(F("Content-Type: application/json"));
    client.println(F("Connection: close"));
    client.println();
    client.println(F("{\"success\":false,\"error\":\"File too small or missing\"}"));
    return;
  }

  // Open internal storage for writing
  InternalStorage.open(contentLength);

  unsigned long bytesReceived = 0;
  uint8_t buffer[256];
  bool success = true;
  String errorMsg = "";

  unsigned long timeout = millis() + 30000;  // 30 second timeout for upload

  while (bytesReceived < (unsigned long)contentLength && millis() < timeout) {
    if (client.available()) {
      size_t toRead = min((size_t)(contentLength - bytesReceived), sizeof(buffer));
      size_t len = client.read(buffer, toRead);
      if (len > 0) {
        for (size_t i = 0; i < len; i++) {
          InternalStorage.write(buffer[i]);
        }
        bytesReceived += len;
        timeout = millis() + 5000;  // Reset timeout on data received

        // Progress indicator
        if (bytesReceived % 10240 == 0) {
          Serial.print(F("Received: "));
          Serial.print(bytesReceived);
          Serial.print(F(" / "));
          Serial.println(contentLength);
        }
      }
    } else {
      delay(1);
    }
  }

  InternalStorage.close();

  Serial.print(F("Total received: "));
  Serial.print(bytesReceived);
  Serial.print(F(" / "));
  Serial.println(contentLength);

  if (bytesReceived < (unsigned long)contentLength) {
    success = false;
    errorMsg = "Incomplete upload";
  }

  // Send response
  client.println(F("HTTP/1.1 200 OK"));
  client.println(F("Content-Type: application/json"));
  client.println(F("Connection: close"));
  client.println();

  if (success) {
    client.println(F("{\"success\":true}"));
    CLIENT_CLEAR(client);
    delay(100);
    client.stop();

    Serial.println(F("Applying update and rebooting..."));
    delay(500);
    InternalStorage.apply();  // This will reboot the device
  } else {
    client.print(F("{\"success\":false,\"error\":\""));
    client.print(errorMsg);
    client.println(F("\"}"));
  }

#else
  // OTA not supported on this board
  client.println(F("HTTP/1.1 200 OK"));
  client.println(F("Content-Type: application/json"));
  client.println(F("Connection: close"));
  client.println();
  client.println(F("{\"success\":false,\"error\":\"OTA not supported on this board\"}"));
#endif
}

// ===========================================================================
// CRC32 Utility (standard polynomial 0xEDB88320, same as zlib/Ethernet)
// ===========================================================================

uint32_t crc32Stream(file_t &f) {
  uint32_t crc = 0xFFFFFFFF;
  uint8_t buf[128];
  int n;
  while ((n = f.read(buf, sizeof(buf))) > 0) {
    for (int i = 0; i < n; i++) {
      crc ^= buf[i];
      for (int b = 0; b < 8; b++)
        crc = (crc >> 1) ^ (0xEDB88320 & -(crc & 1));
    }
  }
  return crc ^ 0xFFFFFFFF;
}

// ===========================================================================
// Handle SD card firmware upload via POST /api/sdupload
// ===========================================================================
// Streams the POST body to UPDATE.bin on the SD card, then verifies the
// written file by computing CRC32 during upload and re-reading from SD.
// Also writes UPDATE.crc alongside the binary. On CRC mismatch the file
// is deleted and an error is returned.

void handleSDUpload(HardwareClient& client, long contentLength) {
  // Gate: SD card must be present
  if (!sdCardPresent) {
    client.println(F("HTTP/1.1 200 OK"));
    client.println(F("Content-Type: application/json"));
    client.println(F("Connection: close"));
    client.println();
    client.println(F("{\"success\":false,\"error\":\"No SD card detected\"}"));
    return;
  }

  // Sanity check file size
  if (contentLength < 1000) {
    client.println(F("HTTP/1.1 200 OK"));
    client.println(F("Content-Type: application/json"));
    client.println(F("Connection: close"));
    client.println();
    client.println(F("{\"success\":false,\"error\":\"File too small or missing\"}"));
    return;
  }

  Serial.println(F("SD upload started"));
  Serial.print(F("Expected size: "));
  Serial.println(contentLength);

  // Open UPDATE.bin for writing
  file_t upFile;
  if (!upFile.open("UPDATE.bin", O_WRONLY | O_CREAT | O_TRUNC)) {
    client.println(F("HTTP/1.1 200 OK"));
    client.println(F("Content-Type: application/json"));
    client.println(F("Connection: close"));
    client.println();
    client.println(F("{\"success\":false,\"error\":\"Cannot write to SD card (write-protected?)\"}"));
    return;
  }

  // Stream from HTTP body to SD, computing CRC32 as we go
  uint32_t crcUpload = 0xFFFFFFFF;
  unsigned long bytesReceived = 0;
  uint8_t buffer[256];
  bool writeError = false;
  unsigned long timeout = millis() + 30000;

  while (bytesReceived < (unsigned long)contentLength && millis() < timeout) {
    if (client.available()) {
      size_t toRead = min((size_t)(contentLength - bytesReceived), sizeof(buffer));
      size_t len = client.read(buffer, toRead);
      if (len > 0) {
        // Write to SD
        size_t written = upFile.write(buffer, len);
        if (written != len) {
          writeError = true;
          break;
        }
        // Update running CRC
        for (size_t i = 0; i < len; i++) {
          crcUpload ^= buffer[i];
          for (int b = 0; b < 8; b++)
            crcUpload = (crcUpload >> 1) ^ (0xEDB88320 & -(crcUpload & 1));
        }
        bytesReceived += len;
        timeout = millis() + 5000;

        if (bytesReceived % 10240 == 0) {
          Serial.print(F("SD upload: "));
          Serial.print(bytesReceived);
          Serial.print(F(" / "));
          Serial.println(contentLength);
        }
      }
    } else {
      delay(1);
    }
  }
  upFile.close();
  crcUpload ^= 0xFFFFFFFF;

  Serial.print(F("SD upload received: "));
  Serial.print(bytesReceived);
  Serial.print(F(" / "));
  Serial.println(contentLength);

  // Check for errors
  if (writeError || bytesReceived < (unsigned long)contentLength) {
    sd.remove("UPDATE.bin");
    client.println(F("HTTP/1.1 200 OK"));
    client.println(F("Content-Type: application/json"));
    client.println(F("Connection: close"));
    client.println();
    if (writeError) {
      client.println(F("{\"success\":false,\"error\":\"SD card write error\"}"));
    } else {
      client.println(F("{\"success\":false,\"error\":\"Incomplete upload (timeout)\"}"));
    }
    return;
  }

  // Verify: re-read UPDATE.bin from SD and compute CRC32
  file_t verifyFile;
  if (!verifyFile.open("UPDATE.bin", O_RDONLY)) {
    sd.remove("UPDATE.bin");
    client.println(F("HTTP/1.1 200 OK"));
    client.println(F("Content-Type: application/json"));
    client.println(F("Connection: close"));
    client.println();
    client.println(F("{\"success\":false,\"error\":\"Cannot re-read UPDATE.bin for verification\"}"));
    return;
  }
  uint32_t crcVerify = crc32Stream(verifyFile);
  verifyFile.close();

  if (crcUpload != crcVerify) {
    sd.remove("UPDATE.bin");
    Serial.println(F("SD upload CRC MISMATCH -- file deleted"));
    client.println(F("HTTP/1.1 200 OK"));
    client.println(F("Content-Type: application/json"));
    client.println(F("Connection: close"));
    client.println();
    client.println(F("{\"success\":false,\"error\":\"CRC32 mismatch -- file deleted\"}"));
    return;
  }

  // Write UPDATE.crc companion file
  file_t crcFile;
  if (crcFile.open("UPDATE.crc", O_WRONLY | O_CREAT | O_TRUNC)) {
    char hexBuf[9];
    snprintf(hexBuf, sizeof(hexBuf), "%08lX", (unsigned long)crcVerify);
    crcFile.print(hexBuf);
    crcFile.close();
  }

  Serial.print(F("SD upload complete, CRC32 verified: "));
  Serial.println(crcVerify, HEX);

  // Success response with CRC
  client.println(F("HTTP/1.1 200 OK"));
  client.println(F("Content-Type: application/json"));
  client.println(F("Access-Control-Allow-Origin: *"));
  client.println(F("Connection: close"));
  client.println();
  client.print(F("{\"success\":true,\"crc32\":\""));
  char hexBuf[9];
  snprintf(hexBuf, sizeof(hexBuf), "%08lX", (unsigned long)crcVerify);
  client.print(hexBuf);
  client.print(F("\",\"size\":"));
  client.print(bytesReceived);
  client.println(F("}"));
}

// ===========================================================================
// SD Card File Manager API Handlers
// ===========================================================================

// Helper: Extract query parameter value from URL
String getQueryParam(const String& url, const String& param) {
  int qPos = url.indexOf('?');
  if (qPos < 0) return "";
  String query = url.substring(qPos + 1);
  int pPos = query.indexOf(param + "=");
  if (pPos < 0) return "";
  int vStart = pPos + param.length() + 1;
  int vEnd = query.indexOf('&', vStart);
  if (vEnd < 0) vEnd = query.length();
  String value = query.substring(vStart, vEnd);
  // URL decode (basic: spaces and common chars)
  value.replace("%20", " ");
  value.replace("%2F", "/");
  value.replace("+", " ");
  return value;
}

// Helper: Check if path is safe (no traversal)
bool isPathSafe(const String& path) {
  return path.indexOf("..") < 0;
}

// Helper: JSON-escape a string
void jsonEscapePrint(Print& out, const String& str) {
  for (unsigned int i = 0; i < str.length(); i++) {
    char c = str.charAt(i);
    switch (c) {
      case '"':  out.print("\\\""); break;
      case '\\': out.print("\\\\"); break;
      case '\n': out.print("\\n"); break;
      case '\r': out.print("\\r"); break;
      case '\t': out.print("\\t"); break;
      default:
        if (c >= 0x20 && c < 0x7F) {
          out.print(c);
        } else {
          out.print("\\u00");
          if ((uint8_t)c < 16) out.print('0');
          out.print((uint8_t)c, HEX);
        }
    }
  }
}

// GET /api/sdlist?path=/
// Lists directory contents as JSON
void handleSDList(HardwareClient& cl, const String& query) {
  sendJsonHeaders(cl);

  if (!sdCardPresent) {
    cl.println(F("{\"success\":false,\"error\":\"No SD card\"}"));
    return;
  }

  String path = getQueryParam(query, "path");
  if (path.length() == 0) path = "/";

  if (!isPathSafe(path)) {
    cl.println(F("{\"success\":false,\"error\":\"Invalid path\"}"));
    return;
  }

  file_t dir;
  const char* dirPath = (path == "/") ? "/" : path.c_str();
  if (!dir.open(dirPath, O_RDONLY) || !dir.isDir()) {
    cl.println(F("{\"success\":false,\"error\":\"Cannot open directory\"}"));
    return;
  }

  // Get card size in MB for format selection UI
  uint32_t cardSizeMB = 0;
  SdCard* card = sd.card();
  if (card) {
    uint32_t sectors = card->sectorCount();
    cardSizeMB = sectors / 2048;  // sectors * 512 / 1048576
  }

  cl.print(F("{\"success\":true,\"path\":\""));
  jsonEscapePrint(cl, path);
  cl.print(F("\",\"format\":\""));
  cl.print(getSDCardFormat());
  cl.print(F("\",\"cardSizeMB\":"));
  cl.print(cardSizeMB);
  cl.print(F(",\"files\":["));

  file_t entry;
  bool first = true;
  char nameBuf[64];

  while (entry.openNext(&dir, O_RDONLY)) {
    entry.getName(nameBuf, sizeof(nameBuf));

    if (!first) cl.print(',');
    first = false;

    cl.print(F("{\"name\":\""));
    jsonEscapePrint(cl, String(nameBuf));
    cl.print(F("\",\"size\":"));
    cl.print(entry.isDir() ? 0 : entry.fileSize());
    cl.print(F(",\"type\":\""));
    cl.print(entry.isDir() ? "dir" : "file");
    cl.print(F("\"}"));

    entry.close();
  }

  dir.close();
  cl.println(F("]}"));
}

// GET /api/sdread?path=/filename.txt
// Reads file contents for editor (limited to 32KB)
void handleSDRead(HardwareClient& cl, const String& query) {
  sendJsonHeaders(cl);

  if (!sdCardPresent) {
    cl.println(F("{\"success\":false,\"error\":\"No SD card\"}"));
    return;
  }

  String path = getQueryParam(query, "path");
  if (path.length() == 0 || !isPathSafe(path)) {
    cl.println(F("{\"success\":false,\"error\":\"Invalid path\"}"));
    return;
  }

  // Strip leading slash for SdFat
  String filename = path.startsWith("/") ? path.substring(1) : path;

  file_t f;
  if (!f.open(filename.c_str(), O_RDONLY)) {
    cl.println(F("{\"success\":false,\"error\":\"Cannot open file\"}"));
    return;
  }

  if (f.isDir()) {
    f.close();
    cl.println(F("{\"success\":false,\"error\":\"Path is a directory\"}"));
    return;
  }

  uint32_t fsize = f.fileSize();
  const uint32_t MAX_EDIT_SIZE = 32768; // 32KB limit

  if (fsize > MAX_EDIT_SIZE) {
    f.close();
    cl.print(F("{\"success\":false,\"error\":\"File too large (max "));
    cl.print(MAX_EDIT_SIZE / 1024);
    cl.println(F("KB)\"}"));
    return;
  }

  cl.print(F("{\"success\":true,\"size\":"));
  cl.print(fsize);
  cl.print(F(",\"content\":\""));

  // Read and JSON-escape content
  char buf[256];
  while (f.available()) {
    int n = f.read(buf, sizeof(buf) - 1);
    if (n <= 0) break;
    buf[n] = '\0';
    jsonEscapePrint(cl, String(buf));
  }

  f.close();
  cl.println(F("\"}"));
}

// GET /api/sddownload?path=/filename.txt
// Downloads file as attachment
void handleSDDownload(HardwareClient& cl, const String& query) {
  if (!sdCardPresent) {
    cl.println(F("HTTP/1.1 503 Service Unavailable"));
    cl.println(F("Content-Type: text/plain"));
    cl.println();
    cl.println(F("No SD card"));
    return;
  }

  String path = getQueryParam(query, "path");
  if (path.length() == 0 || !isPathSafe(path)) {
    cl.println(F("HTTP/1.1 400 Bad Request"));
    cl.println(F("Content-Type: text/plain"));
    cl.println();
    cl.println(F("Invalid path"));
    return;
  }

  // Strip leading slash
  String filename = path.startsWith("/") ? path.substring(1) : path;

  file_t f;
  if (!f.open(filename.c_str(), O_RDONLY) || f.isDir()) {
    if (f.isOpen()) f.close();
    cl.println(F("HTTP/1.1 404 Not Found"));
    cl.println(F("Content-Type: text/plain"));
    cl.println();
    cl.println(F("File not found"));
    return;
  }

  // Extract just the filename for Content-Disposition
  String dispName = filename;
  int lastSlash = dispName.lastIndexOf('/');
  if (lastSlash >= 0) dispName = dispName.substring(lastSlash + 1);

  cl.println(F("HTTP/1.1 200 OK"));
  cl.print(F("Content-Disposition: attachment; filename=\""));
  cl.print(dispName);
  cl.println(F("\""));
  cl.println(F("Content-Type: application/octet-stream"));
  cl.print(F("Content-Length: "));
  cl.println(f.fileSize());
  cl.println(F("Connection: close"));
  cl.println();

  // Stream file
  uint8_t buf[256];
  while (f.available()) {
    int n = f.read(buf, sizeof(buf));
    if (n <= 0) break;
    cl.write(buf, n);
  }

  f.close();
}

// Helper: Read POST body into String (limited size)
String readPostBody(HardwareClient& cl, long contentLength, long maxLen = 33792) {
  String body;
  if (contentLength <= 0 || contentLength > maxLen) return body;
  body.reserve(contentLength);

  unsigned long timeout = millis() + 10000;
  while (body.length() < (unsigned)contentLength && millis() < timeout) {
    if (cl.available()) {
      body += (char)cl.read();
    } else {
      delay(1);
    }
  }
  return body;
}

// POST /api/sddelete
// Body: {"path":"/filename.txt"}
void handleSDDelete(HardwareClient& cl, long contentLength) {
  // Read body BEFORE sending any response
  String body = readPostBody(cl, contentLength, 512);

  sendJsonHeaders(cl);

  if (!sdCardPresent) {
    cl.println(F("{\"success\":false,\"error\":\"No SD card\"}"));
    return;
  }

  // Simple JSON parse for path
  int pathIdx = body.indexOf("\"path\"");
  if (pathIdx < 0) {
    cl.println(F("{\"success\":false,\"error\":\"Missing path\"}"));
    return;
  }

  int valStart = body.indexOf(':', pathIdx) + 1;
  while (valStart < (int)body.length() && (body[valStart] == ' ' || body[valStart] == '"')) valStart++;
  int valEnd = body.indexOf('"', valStart);
  if (valEnd < 0) valEnd = body.length();

  String path = body.substring(valStart, valEnd);

  if (path.length() == 0 || !isPathSafe(path)) {
    cl.println(F("{\"success\":false,\"error\":\"Invalid path\"}"));
    return;
  }

  // Strip leading slash
  String filename = path.startsWith("/") ? path.substring(1) : path;

  // Check if file exists
  file_t f;
  if (!f.open(filename.c_str(), O_RDONLY)) {
    cl.println(F("{\"success\":false,\"error\":\"File not found\"}"));
    return;
  }

  bool isDir = f.isDir();
  f.close();

  bool ok;
  if (isDir) {
    ok = sd.rmdir(filename.c_str());
  } else {
    ok = sd.remove(filename.c_str());
  }

  if (ok) {
    cl.println(F("{\"success\":true}"));
  } else {
    cl.println(F("{\"success\":false,\"error\":\"Delete failed (directory not empty?)\"}"));
  }
}

// POST /api/sdwrite
// Body: {"path":"/filename.txt","content":"..."}
void handleSDWrite(HardwareClient& cl, long contentLength) {
  // Read body BEFORE sending any response
  String body = readPostBody(cl, contentLength, 34816); // 34KB max (32KB content + JSON overhead)

  sendJsonHeaders(cl);

  if (!sdCardPresent) {
    cl.println(F("{\"success\":false,\"error\":\"No SD card\"}"));
    return;
  }

  // Parse path
  int pathIdx = body.indexOf("\"path\"");
  if (pathIdx < 0) {
    cl.println(F("{\"success\":false,\"error\":\"Missing path\"}"));
    return;
  }

  int pathStart = body.indexOf(':', pathIdx) + 1;
  while (pathStart < (int)body.length() && (body[pathStart] == ' ' || body[pathStart] == '"')) pathStart++;
  int pathEnd = body.indexOf('"', pathStart);

  String path = body.substring(pathStart, pathEnd);

  if (path.length() == 0 || !isPathSafe(path)) {
    cl.println(F("{\"success\":false,\"error\":\"Invalid path\"}"));
    return;
  }

  // Parse content
  int contentIdx = body.indexOf("\"content\"");
  if (contentIdx < 0) {
    cl.println(F("{\"success\":false,\"error\":\"Missing content\"}"));
    return;
  }

  int contentStart = body.indexOf(':', contentIdx) + 1;
  while (contentStart < (int)body.length() && (body[contentStart] == ' ' || body[contentStart] == '"')) contentStart++;

  // Find the closing quote (handle escaped quotes)
  int contentEnd = contentStart;
  while (contentEnd < (int)body.length()) {
    if (body[contentEnd] == '"' && (contentEnd == 0 || body[contentEnd - 1] != '\\')) break;
    contentEnd++;
  }

  String content = body.substring(contentStart, contentEnd);

  // Unescape JSON string
  content.replace("\\n", "\n");
  content.replace("\\r", "\r");
  content.replace("\\t", "\t");
  content.replace("\\\"", "\"");
  content.replace("\\\\", "\\");

  // Strip leading slash
  String filename = path.startsWith("/") ? path.substring(1) : path;

  file_t f;
  if (!f.open(filename.c_str(), O_WRONLY | O_CREAT | O_TRUNC)) {
    cl.println(F("{\"success\":false,\"error\":\"Cannot create file\"}"));
    return;
  }

  size_t written = f.print(content);
  f.close();

  cl.print(F("{\"success\":true,\"size\":"));
  cl.print(written);
  cl.println(F("}"));
}

// POST /api/sdformat
// Body: {"confirm":true}
void handleSDFormat(HardwareClient& cl, long contentLength) {
  // Read body BEFORE sending any response
  String body = readPostBody(cl, contentLength, 128);

  sendJsonHeaders(cl);

  if (!sdCardPresent) {
    cl.println(F("{\"success\":false,\"error\":\"No SD card\"}"));
    return;
  }

  // Check for confirm:true
  if (body.indexOf("\"confirm\":true") < 0 && body.indexOf("\"confirm\": true") < 0) {
    cl.println(F("{\"success\":false,\"error\":\"Confirmation required\"}"));
    return;
  }

  // Parse format type: "quick", "auto", "fat32", "exfat"
  String formatType = "quick";  // default
  int typeIdx = body.indexOf("\"type\":\"");
  if (typeIdx >= 0) {
    int start = typeIdx + 8;
    int end = body.indexOf("\"", start);
    if (end > start) {
      formatType = body.substring(start, end);
    }
  }

  bool success = false;
  String resultMsg = "";

  if (formatType == "quick") {
    // Quick format: just delete all files
    file_t root;
    if (!root.open("/", O_RDONLY)) {
      cl.println(F("{\"success\":false,\"error\":\"Cannot open root\"}"));
      return;
    }

    file_t entry;
    char nameBuf[64];
    String filesToDelete[32];
    int fileCount = 0;

    while (entry.openNext(&root, O_RDONLY) && fileCount < 32) {
      entry.getName(nameBuf, sizeof(nameBuf));
      if (nameBuf[0] != '.') {
        filesToDelete[fileCount++] = String(nameBuf);
      }
      entry.close();
    }
    root.close();

    int deleted = 0;
    for (int i = 0; i < fileCount; i++) {
      file_t f;
      if (f.open(filesToDelete[i].c_str(), O_RDONLY)) {
        bool isDir = f.isDir();
        f.close();
        if (isDir) {
          sd.rmdir(filesToDelete[i].c_str());
        } else {
          if (sd.remove(filesToDelete[i].c_str())) deleted++;
        }
      }
    }
    success = true;
    resultMsg = "Quick format done, deleted " + String(deleted) + " files";

  } else if (formatType == "auto" || formatType == "fat32" || formatType == "exfat") {
    // True filesystem format using SdFat formatters
    // Close SD and re-init card for raw access
    sd.end();

    // Sector buffer for formatting (512 bytes on stack)
    uint8_t secBuf[512] __attribute__((aligned(4)));

    // Re-init in card-only mode to get raw access
    SdSpiConfig spiConfig(SD_CS, SHARED_SPI, SD_SCK_MHZ(4));

    // Get card sector count to determine appropriate format
    if (!sd.begin(spiConfig)) {
      cl.println(F("{\"success\":false,\"error\":\"SD card init failed\"}"));
      sdCardPresent = false;
      return;
    }

    // Get card pointer for low-level formatting
    SdCard* card = sd.card();
    if (!card) {
      cl.println(F("{\"success\":false,\"error\":\"Cannot access card\"}"));
      return;
    }

    uint32_t sectorCount = card->sectorCount();
    if (sectorCount == 0) {
      cl.println(F("{\"success\":false,\"error\":\"Cannot read card size\"}"));
      return;
    }

    // Determine card size in MB for info
    uint32_t cardSizeMB = sectorCount / 2048;  // sectors * 512 / 1048576

    // Close filesystem to allow raw format, then re-init for raw card access.
    // Note: We use sd.begin() instead of card->begin() because SdCardInterface
    // doesn't have begin() on all platforms (e.g. RP2040 with PIO SDIO).
    sd.end();
    if (!sd.begin(spiConfig)) {
      cl.println(F("{\"success\":false,\"error\":\"Card reinit failed\"}"));
      sdCardPresent = false;
      return;
    }
    card = sd.card();

    bool formatOk = false;
    String formatUsed = "";

    if (formatType == "exfat") {
      // Force exFAT format
      ExFatFormatter exFmt;
      formatOk = exFmt.format(card, secBuf, &Serial);
      formatUsed = "exFAT";
    } else if (formatType == "fat32") {
      // Force FAT16/FAT32 format (auto-selects based on size)
      if (sectorCount > 67108864) {  // >32GB
        cl.println(F("{\"success\":false,\"error\":\"Card too large for FAT32, use exFAT\"}"));
        // Re-init SD
        sd.begin(SD_CS, SD_SCK_MHZ(4));
        return;
      }
      FatFormatter fatFmt;
      formatOk = fatFmt.format(card, secBuf, &Serial);
      formatUsed = (sectorCount < 4194304) ? "FAT16" : "FAT32";
    } else {
      // Auto format - let library choose
      if (sectorCount > 67108864) {
        ExFatFormatter exFmt;
        formatOk = exFmt.format(card, secBuf, &Serial);
        formatUsed = "exFAT";
      } else {
        FatFormatter fatFmt;
        formatOk = fatFmt.format(card, secBuf, &Serial);
        formatUsed = (sectorCount < 4194304) ? "FAT16" : "FAT32";
      }
    }

    // Re-initialize SD filesystem
    if (!sd.begin(SD_CS, SD_SCK_MHZ(4))) {
      cl.println(F("{\"success\":false,\"error\":\"Format done but SD reinit failed\"}"));
      sdCardPresent = false;
      return;
    }

    if (formatOk) {
      success = true;
      resultMsg = "Formatted as " + formatUsed + " (" + String(cardSizeMB) + " MB)";
    } else {
      cl.print(F("{\"success\":false,\"error\":\"Format failed for "));
      cl.print(formatUsed);
      cl.println(F("\"}"));
      return;
    }
  } else {
    cl.println(F("{\"success\":false,\"error\":\"Unknown format type\"}"));
    return;
  }

  // Regenerate essential files after any format
  generateIndexHtml();

  // Write favicon from PROGMEM
  file_t fav;
  if (fav.open("favicon.ico", O_WRONLY | O_CREAT | O_TRUNC)) {
    extern unsigned char favicon_ico[];
    extern unsigned int favicon_ico_len;
    fav.write(favicon_ico, favicon_ico_len);
    fav.close();
  }

  cl.print(F("{\"success\":true,\"message\":\""));
  cl.print(resultMsg);
  cl.print(F("\",\"format\":\""));
  cl.print(getSDCardFormat());
  cl.println(F("\"}"));
}

// POST /api/sdfileupload
// Multipart form upload to SD card
// Expects: Content-Type: multipart/form-data with "file" field and "path" field
void handleSDFileUpload(HardwareClient& cl, long contentLength, const String& req) {
  if (!sdCardPresent) {
    sendJsonHeaders(cl);
    cl.println(F("{\"success\":false,\"error\":\"No SD card\"}"));
    return;
  }

  if (contentLength <= 0 || contentLength > 65536) {
    sendJsonHeaders(cl);
    cl.println(F("{\"success\":false,\"error\":\"Invalid content length (max 64KB)\"}"));
    return;
  }

  // Find boundary from Content-Type header
  int ctIdx = req.indexOf("Content-Type: multipart/form-data");
  if (ctIdx < 0) {
    sendJsonHeaders(cl);
    cl.println(F("{\"success\":false,\"error\":\"Expected multipart/form-data\"}"));
    return;
  }

  int boundIdx = req.indexOf("boundary=", ctIdx);
  if (boundIdx < 0) {
    sendJsonHeaders(cl);
    cl.println(F("{\"success\":false,\"error\":\"Missing boundary\"}"));
    return;
  }

  int boundEnd = req.indexOf("\r\n", boundIdx);
  if (boundEnd < 0) boundEnd = req.length();
  String boundary = "--" + req.substring(boundIdx + 9, boundEnd);
  boundary.trim();

  // Read the entire body (limited to 64KB for safety)
  String body;
  body.reserve(contentLength);
  unsigned long timeout = millis() + 30000;

  while (body.length() < (unsigned)contentLength && millis() < timeout) {
    if (cl.available()) {
      body += (char)cl.read();
    } else {
      delay(1);
    }
  }

  // Parse multipart form data to find filename and content
  String filename = "";
  String destPath = "/";
  int fileStart = -1;
  int fileEnd = -1;

  // Find the file part
  int partStart = body.indexOf(boundary);
  while (partStart >= 0) {
    int partEnd = body.indexOf(boundary, partStart + boundary.length());
    if (partEnd < 0) partEnd = body.length();

    String part = body.substring(partStart, partEnd);

    // Check if this is the file part
    int dispIdx = part.indexOf("Content-Disposition:");
    if (dispIdx >= 0) {
      int filenameIdx = part.indexOf("filename=\"", dispIdx);
      if (filenameIdx >= 0) {
        int fnStart = filenameIdx + 10;
        int fnEnd = part.indexOf("\"", fnStart);
        if (fnEnd > fnStart) {
          filename = part.substring(fnStart, fnEnd);
          // Find file content (after double CRLF)
          int contentStart = part.indexOf("\r\n\r\n", dispIdx);
          if (contentStart >= 0) {
            contentStart += 4;
            // Content ends before trailing CRLF--
            int contentEnd = part.length();
            if (part.endsWith("\r\n")) contentEnd -= 2;
            fileStart = partStart + contentStart;
            fileEnd = partStart + contentEnd;
          }
        }
      }

      // Check if this is the path field
      int nameIdx = part.indexOf("name=\"path\"", dispIdx);
      if (nameIdx >= 0) {
        int valStart = part.indexOf("\r\n\r\n", nameIdx);
        if (valStart >= 0) {
          valStart += 4;
          int valEnd = part.indexOf("\r\n", valStart);
          if (valEnd > valStart) {
            destPath = part.substring(valStart, valEnd);
            destPath.trim();
          }
        }
      }
    }

    partStart = body.indexOf(boundary, partStart + boundary.length());
  }

  if (filename.length() == 0 || fileStart < 0 || fileEnd <= fileStart) {
    sendJsonHeaders(cl);
    cl.println(F("{\"success\":false,\"error\":\"No file found in upload\"}"));
    return;
  }

  // Security check
  if (filename.indexOf("..") >= 0 || destPath.indexOf("..") >= 0) {
    sendJsonHeaders(cl);
    cl.println(F("{\"success\":false,\"error\":\"Invalid path\"}"));
    return;
  }

  // Build full path
  String fullPath = destPath;
  if (!fullPath.endsWith("/")) fullPath += "/";
  fullPath += filename;
  if (fullPath.startsWith("/")) fullPath = fullPath.substring(1);

  // Write file
  file_t f;
  if (!f.open(fullPath.c_str(), O_WRONLY | O_CREAT | O_TRUNC)) {
    sendJsonHeaders(cl);
    cl.println(F("{\"success\":false,\"error\":\"Cannot create file\"}"));
    return;
  }

  size_t written = 0;
  for (int i = fileStart; i < fileEnd; i++) {
    f.write(body[i]);
    written++;
  }
  f.close();

  sendJsonHeaders(cl);
  cl.print(F("{\"success\":true,\"filename\":\""));
  jsonEscapePrint(cl, filename);
  cl.print(F("\",\"size\":"));
  cl.print(written);
  cl.println(F("}"));
}

// Attempts to serve a file from the SD card root. The path parameter is the
// URL path with leading slash (e.g. "/config-wifi.html"). Returns true if
// the file was found and served, false otherwise (caller should send 404).
//
// HTML files are processed through the template engine (Smarty variables,
// {foreach}, {if}/{else}, {literal} blocks). All other files are streamed
// as raw bytes with a Content-Type inferred from the extension.

bool serveSDFile(HardwareClient cl, const String &path) {
  if (!sdCardPresent) return false;

  // Strip leading slash to get SD filename; reject path traversal
  String filename = path.substring(1);
  if (filename.length() == 0 || filename.indexOf("..") >= 0) return false;

  file_t f;
  if (!f.open(filename.c_str(), O_RDONLY)) return false;

  // Auto-index: if path is a directory, try index.html inside it
  if (f.isDir()) {
    f.close();
    // Build index path: "subdir" -> "subdir/index.html", "subdir/" -> "subdir/index.html"
    String indexPath = filename;
    if (!indexPath.endsWith("/")) indexPath += "/";
    indexPath += "index.html";
    if (!f.open(indexPath.c_str(), O_RDONLY)) return false;
    filename = indexPath;  // update for Content-Type detection
  }

  // Determine Content-Type from extension
  String ct = "application/octet-stream";
  int dot = filename.lastIndexOf('.');
  if (dot >= 0) {
    String ext = filename.substring(dot + 1);
    ext.toLowerCase();
    if      (ext == "html" || ext == "htm") ct = "text/html";
    else if (ext == "css")   ct = "text/css";
    else if (ext == "js")    ct = "application/javascript";
    else if (ext == "json")  ct = "application/json";
    else if (ext == "txt")   ct = "text/plain";
    else if (ext == "png")   ct = "image/png";
    else if (ext == "jpg" || ext == "jpeg") ct = "image/jpeg";
    else if (ext == "gif")   ct = "image/gif";
    else if (ext == "svg")   ct = "image/svg+xml";
    else if (ext == "ico")   ct = "image/x-icon";
    else if (ext == "xml")   ct = "application/xml";
    else if (ext == "bin")   ct = "application/octet-stream";
  }

  cl.println("HTTP/1.1 200 OK");
  cl.print("Content-Type: "); cl.println(ct);
  cl.println("Connection: close");
  cl.println();

  // HTML files: run through template engine (Smarty variables, {foreach}, etc.)
  if (ct == "text/html") {
    tplReset();
    char buf[257];
    while (f.available()) {
      int len = f.fgets(buf, sizeof(buf));
      if (len <= 0) break;
      String line = String(buf);
      processTemplateLine(line, cl);
    }
  } else {
    // All other files: stream raw bytes
    uint8_t buf[128];
    int n;
    while ((n = f.read(buf, sizeof(buf))) > 0) {
      cl.write(buf, n);
    }
  }
  f.close();
  CLIENT_CLEAR(cl);
  return true;
}

// ===========================================================================
// 404 Error Page
// ===========================================================================

void send404Error(HardwareClient cl) {
  cl.println("<!DOCTYPE html>");
  cl.println("<html><head><title>404 Not Found</title></head>");
  cl.println("<body style='background:#000;color:#eee;font-family:Arial,sans-serif;'>");
  cl.println("<h1>404 Not Found</h1>");
  cl.println("<p>The requested page was not found on this server.</p>");
  cl.println("</body></html>");
}

#pragma GCC diagnostic pop
