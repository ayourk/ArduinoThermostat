# eyeSPI Thermostat SD Card Templates
# ====================================

This package contains SD card template files for the eyeSPI Thermostat
firmware.

## Quick Start

1. Copy these files to your SD card root:
   - config-sample.html
   - update-sample.html
   - index-sample.html  (optional — customizable home page)
   - sdfile-sample.html  (optional — SD card file manager)

2. Create a device.cfg file for your network settings (see below).

3. Insert SD card and power on the thermostat.

The template files work with all hardware configurations (WiFi,
Ethernet, and Dual Network).  Network-specific sections are shown
or hidden automatically using Smarty-style conditionals based on
the {$HAS_ETHERNET} and {$HAS_WIFI} template variables.


## File Descriptions

  File                  Purpose
  --------------------  ---------------------------------------------------
  config-sample.html    Configuration page for all variants (network
                        settings with Ethernet/WiFi tabs for dual mode,
                        WiFi scan, outdoor unit config).  Uses Smarty
                        conditionals to adapt to the active build.

  update-sample.html    Firmware update page (OTA upload when supported,
                        SD card upload, reboot button).  Adapts to
                        board capabilities via {$OTA_SUPPORTED} and
                        {$SD_CARD} variables.

  index-sample.html     Custom home page with HTML5 Canvas thermostat
                        display, environment card, system info, and
                        outdoor unit status.  Fetches /api/state via
                        JavaScript and dynamically renders all network
                        modes.

  sdfile-sample.html    SD card file manager with directory browsing,
                        file editing, upload, download, delete, and
                        format (quick, FAT32, exFAT).

  sample_reference.html Developer reference showing all SHOW_ template
                        variables and canvas layout coordinates.  Not
                        intended for deployment to the SD card.


## How It Works

The firmware serves these files via the SD card catchall route.  Any URL
that doesn't match a built-in route (/, /config, /update, /api/*) is
looked up on the SD card.  HTML files are processed through the Smarty-
style template engine, so variables like {$CURRENT_TEMP} and conditionals
like {if $HEAT_ENABLED == "1"}...{/if} work in your custom pages.

The built-in /config and /update routes always serve the embedded pages,
regardless of SD card contents.  To use these SD templates, link to them
by filename (e.g., /config-sample.html) rather than /config.


## Template Variables

Thermostat state:
  {$CURRENT_TEMP}       Indoor temperature (integer, °F)
  {$CURRENT_TEMP_RAW}   Indoor temperature (4 decimals, °F)
  {$TARGET_TEMP}        Target temperature (integer, °F)
  {$TEMP_COLOR}         CSS color for current temp display
  {$HEAT_ENABLED}       "1" if heat is enabled
  {$HEAT_ACTIVE}        "1" if furnace is currently firing
  {$HEAT_ENABLED_TEXT}  "On" or "Off"
  {$HEAT_ACTIVE_TEXT}   "On" or "Off"

Environment (indoor BME280/BME680):
  {$BME_PRESENT}        "1" if BME sensor detected
  {$HUMIDITY}           Relative humidity (%)
  {$PRESSURE_HPA}       Barometric pressure (hPa)
  {$PRESSURE_INHG}      Barometric pressure (inHg)
  {$GAS_KOHMS}          Gas resistance (kOhms, BME680 only)

System:
  {$BOARD}              Board name (e.g., "Feather M4")
  {$UPTIME}             Uptime in seconds
  {$UPTIME_TEXT}        Uptime as human-readable string
  {$FREE_RAM}           Free RAM in bytes
  {$SD_CARD}            "1" if SD card present
  {$SD_FORMAT}          Filesystem type (FAT32, exFAT, etc.)
  {$DISPLAY_PRESENT}    "1" if TFT display detected
  {$OTA_SUPPORTED}      "1" if OTA firmware update supported
  {$WEBSERVER_PORT}     Web server port number
  {$BATVOLTS}           Battery voltage (if available)

Network capability flags:
  {$HAS_ETHERNET}       "1" if Ethernet is available
  {$HAS_WIFI}           "1" if WiFi is available
  {$HAS_BATT}           "1" if battery monitoring available
  {$IS_DUAL_NETWORK}    "1" if dual network build
  {$NETWORK_MODE}       "dual", "ethernet", "wifi", or "none"
  {$ETH_CONNECTED}      "1" if Ethernet link is up
  {$WIFI_CONNECTED}     "1" if WiFi is connected

Client detection (set per-request):
  {$CLIENT_INTERFACE}   "eth" or "wifi"
  {$CLIENT_IS_ETH}      "1" if client connected via Ethernet
  {$CLIENT_IS_WIFI}     "1" if client connected via WiFi

Ethernet interface (W5500 or DUAL_NETWORK):
  {$ETH_IP}             Current IP address
  {$ETH_MAC}            MAC address
  {$ETH_SUBNET}         Subnet mask
  {$ETH_GATEWAY}        Gateway address
  {$ETH_DNS}            DNS server
  {$ETH_LINK}           "Connected" or "Disconnected"
  {$ETH_CFG_IP}         Configured static IP (empty = DHCP)
  {$ETH_CFG_DNS}        Configured DNS
  {$ETH_CFG_SUBNET}     Configured subnet
  {$ETH_CFG_GATEWAY}    Configured gateway
  {$ETH_CFG_MAC}        Configured MAC override

WiFi interface (WINC1500, ESP32, or DUAL_NETWORK):
  {$WIFI_IP}            Current IP address
  {$WIFI_MAC}           MAC address
  {$WIFI_SUBNET}        Subnet mask
  {$WIFI_GATEWAY}       Gateway address
  {$WIFI_DNS}           DNS server
  {$WIFI_LINK}          "Connected" or "Disconnected"
  {$WIFI_SSID}          Connected SSID
  {$WIFI_RSSI}          Signal strength in dBm
  {$WIFI_SECURITY}      Security type (WPA2, Open, etc.)
  {$WIFI_CFG_IP}        Configured static IP (empty = DHCP)
  {$WIFI_CFG_DNS}       Configured DNS
  {$WIFI_CFG_SUBNET}    Configured subnet
  {$WIFI_CFG_GATEWAY}   Configured gateway
  {$WIFI_CFG_SSID}      Configured SSID
  {$WIFI_CFG_PASS}      Configured password
  {$WIFI_CFG_SECURITY}  Configured security type

Outdoor unit:
  {$OUTDOOR_CONFIGURED} "1" if outdoor IP configured
  {$OUTDOOR_AVAILABLE}  "1" if outdoor unit responding
  {$OUTDOOR_STATUS}     "Online", "Offline", or "Unknown"
  {$OUTDOOR_CFG_IP}     Configured outdoor IP
  {$OUTDOOR_CFG_PORT}   Configured outdoor port
  {$OUTDOOR_POLL_SECS}  Poll interval in seconds
  {$OUTDOOR_TEMP}       Outdoor temperature (°F)
  {$OUTDOOR_HUMIDITY}   Outdoor humidity (%)
  {$OUTDOOR_PRESSURE_HPA}  Outdoor pressure (hPa)
  {$OUTDOOR_PRESSURE_INHG} Outdoor pressure (inHg)
  {$OUTDOOR_GAS_KOHMS}  Outdoor gas resistance (kOhms)
  {$OUTDOOR_BME_TYPE}   Outdoor BME sensor type
  {$OUTDOOR_BOARD}      Outdoor unit board name
  {$OUTDOOR_ETH_IP}     Outdoor unit Ethernet IP
  {$OUTDOOR_WIFI_IP}    Outdoor unit WiFi IP
  {$OUTDOOR_IFACE}      Outdoor unit active interface
  {$OUTDOOR_BATVOLTS}   Outdoor unit battery voltage
  {$OUTDOOR_HAS_ETHERNET} "1" if outdoor has Ethernet
  {$OUTDOOR_HAS_WIFI}   "1" if outdoor has WiFi

Display SHOW_ flags (for canvas rendering):
  {$SHOW_INSIDE_LABEL}   Always "1"
  {$SHOW_CURRENT_TEMP}   Always "1"
  {$SHOW_DEGREE_SYMBOL}  Always "1"
  {$SHOW_HEAT_ON}        "1" if heat enabled and furnace active
  {$SHOW_HEAT_SETTING}   "1" if heat enabled
  {$SHOW_TARGET_TEMP}    "1" if heat enabled
  {$SHOW_HEAT_LABEL}     "1" if heat enabled
  {$SHOW_LIGHT_BUTTON}   Always "1"
  {$SHOW_SYSTEM_BUTTON}  Always "1"
  {$SHOW_MINUS}          Always "1"
  {$SHOW_PLUS}           Always "1"
  {$SHOW_BME_CARD}       "1" if any BME sensor detected


## device.cfg Options

Ethernet settings (W5500 or DUAL_NETWORK builds):

  Key              Description                      Default
  ---------------  -------------------------------  ----------------
  ETH_IP           Static IP or "DHCP"              DHCP
  ETH_DNS          DNS server                       (from DHCP)
  ETH_SUBNET       Subnet mask                      255.255.255.0
  ETH_GATEWAY      Gateway address                  (from DHCP)
  ETH_MAC          Override MAC                     (from EEPROM)

WiFi settings (WINC1500, ESP32, or DUAL_NETWORK builds):

  Key              Description                      Default
  ---------------  -------------------------------  ----------------
  WIFI_SSID        WiFi network name                (none)
  WIFI_PASS        WiFi password                    (none)
  WIFI_IP          Static IP or "DHCP"              DHCP
  WIFI_DNS         DNS server                       (from DHCP)
  WIFI_SUBNET      Subnet mask                      255.255.255.0
  WIFI_GATEWAY     Gateway address                  (from DHCP)

Common settings (all builds):

  Key              Description                      Default
  ---------------  -------------------------------  ----------------
  OUTDOOR_IP       Remote outdoor unit IP           (none)
  OUTDOOR_PORT     Remote outdoor unit port         80
  OUTDOOR_POLL     Poll interval in seconds         30
  RELAY_PIN        Heat relay GPIO pin              13


## Firmware

These templates are designed for the eyeSPI Thermostat firmware.
Check PROJECT_DOCUMENTATION.txt for the full firmware documentation.

GitHub: https://github.com/ayourk/ArduinoThermostat
