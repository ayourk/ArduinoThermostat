# eyeSPI Thermostat SD Card Templates
# ====================================

This package contains SD card template files for the eyeSPI Thermostat
firmware.

## Quick Start

1. Copy the files for your hardware variant to your SD card root:

   **WiFi (WINC1500):**
   - config-wifi.html
   - update-wifi.html

   **Ethernet (W5500/PoE):**
   - config-eth.html
   - update-eth.html

2. Copy device.cfg.example to device.cfg and edit for your network.

3. Optionally copy index.html.example to index.html and customize.

4. Insert SD card and power on the thermostat.


## File Descriptions

  File                  Purpose
  --------------------  ---------------------------------------------------
  config-wifi.html      Configuration page for WiFi variant (network
                        settings, WiFi scan, outdoor unit config)

  config-eth.html       Configuration page for Ethernet variant (network
                        status, static IP, outdoor unit config)

  update-wifi.html      Firmware update page for WiFi variant (OTA upload
                        with WiFi status display)

  update-eth.html       Firmware update page for Ethernet variant (OTA
                        upload with link status display)

  device.cfg.example    Example device configuration file with all
                        available options documented

  index.html.example    Example custom home page showing temperature,
                        status, and links to config/update pages


## How It Works

The firmware serves these files via the SD card catchall route.  Any URL
that doesn't match a built-in route (/, /config, /update, /api/*) is
looked up on the SD card.  HTML files are processed through the Smarty-
style template engine, so variables like {$IP_ADDRESS} and conditionals
like {if $HEAT_ON}...{/if} work in your custom pages.

The built-in /config and /update routes always serve the embedded pages,
regardless of SD card contents.  To use these SD templates, link to them
by filename (e.g., /config-wifi.html) rather than /config.


## Template Variables

Common variables available in all templates:

  {$BOARD}              Board name (e.g., "Feather M4")
  {$IP_ADDRESS}         Current IP address
  {$SUBNET}             Subnet mask
  {$GATEWAY}            Gateway address
  {$MAC_ADDRESS}        MAC address
  {$SD_CARD}            "1" if SD card present

WiFi-specific (WINC1500):
  {$WIFI_SSID}          Connected SSID
  {$WIFI_ENCRYPTION}    Security type (WPA2, Open, etc.)
  {$WIFI_RSSI}          Signal strength in dBm
  {$BATTERY_VOLTAGE}    Battery voltage (if available)

Ethernet-specific (W5500):
  {$LINK_STATUS}        "Up" or "Down"

Thermostat state:
  {$INDOOR_TEMP}        Indoor temperature (°F)
  {$TARGET_TEMP}        Target temperature (°F)
  {$HEAT_ON}            "1" if heat is currently on
  {$MODE}               "Auto" or "Manual"

Outdoor unit:
  {$OUTDOOR_CONFIGURED} "1" if outdoor IP configured
  {$OUTDOOR_AVAILABLE}  "1" if outdoor unit responding
  {$OUTDOOR_TEMP}       Outdoor temperature (°F)
  {$OUTDOOR_HUMIDITY}   Outdoor humidity (%)


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

GitHub: https://github.com/ayourk/eyeSPI-FeatherWing-PCB
