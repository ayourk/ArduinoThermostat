#!/bin/bash
#
# build-all.sh -- Compile eyeSPI Thermostat firmware for all supported boards
#
# Usage: ./build-all.sh [board...]
#
# Examples:
#   ./build-all.sh              # Build all primary boards
#   ./build-all.sh m4           # Build only M4
#   ./build-all.sh m0 esp32s2   # Build M0 and ESP32-S2
#   ./build-all.sh all nrf52840 # Build all primary + nRF52840
#
# Requires: arduino-cli (https://arduino.github.io/arduino-cli/)
#
# =============================================================================
# BOARD PACKAGE INSTALLATION
# =============================================================================
#
# First, configure arduino-cli with required board manager URLs:
#
#   arduino-cli config init
#   arduino-cli config add board_manager.additional_urls \
#     https://adafruit.github.io/arduino-board-index/package_adafruit_index.json
#   arduino-cli config add board_manager.additional_urls \
#     https://github.com/earlephilhower/arduino-pico/releases/download/global/package_rp2040_index.json
#   arduino-cli config add board_manager.additional_urls \
#     https://espressif.github.io/arduino-esp32/package_esp32_index.json
#   arduino-cli config add board_manager.additional_urls \
#     https://github.com/stm32duino/BoardManagerFiles/raw/main/package_stmicroelectronics_index.json
#
# Then install the board packages:
#
#   arduino-cli core update-index
#
#   # Primary boards (required)
#   arduino-cli core install adafruit:samd        # M0, M4
#   arduino-cli core install rp2040:rp2040        # RP2040
#   arduino-cli core install esp32:esp32          # ESP32-S2, S3, V2
#
#   # Optional boards
#   arduino-cli core install adafruit:nrf52       # nRF52840 (BLE)
#   arduino-cli core install STMicroelectronics:stm32  # STM32F405
#
# =============================================================================
# LIBRARY INSTALLATION
# =============================================================================
#
#   arduino-cli lib install "Adafruit MAX31865 library"
#   git clone https://github.com/drhaney/pt100rtd.git \
#     "$(arduino-cli config get directories.user)/libraries/pt100rtd"
#   arduino-cli lib install "Adafruit GFX Library"
#   arduino-cli lib install "Adafruit ILI9341"
#   arduino-cli lib install "Adafruit FT6206 Library"
#   arduino-cli lib install "Adafruit Unified Sensor"
#   arduino-cli lib install "Adafruit BME680 Library"
#   arduino-cli lib install "Adafruit BME280 Library"
#   arduino-cli lib install "ArduinoJson"
#   arduino-cli lib install "SdFat - Adafruit Fork"
#   arduino-cli lib install "Ethernet"
#   arduino-cli lib install "WiFi101"
#   arduino-cli lib install "ArduinoOTA"
#
# Note: pt100rtd requires manual patching for ESP32. See PROJECT_DOCUMENTATION.txt
#
# =============================================================================

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
SKETCH="$SCRIPT_DIR/Thermostat.ino"
OUT="$SCRIPT_DIR/binaries"
BUILD_DIR="$SCRIPT_DIR/build"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color
VERBOSE=false  # Set to true with --verbose flag
FLASH=false    # Set to true with --flash flag
FLASH_PORT=""  # Serial port for flashing (auto-detect if empty)

# Board definitions: name|fqbn|output_extension|output_name
# Primary boards (included in "all")
PRIMARY_BOARDS=(
    "m4|adafruit:samd:adafruit_feather_m4|bin|Thermostat-M4-W5500"
    "m0|adafruit:samd:adafruit_feather_m0|bin|Thermostat-M0-WINC1500"
    "rp2040|rp2040:rp2040:adafruit_feather|uf2|Thermostat-RP2040-W5500"
    "esp32s2|esp32:esp32:adafruit_feather_esp32s2|bin|Thermostat-ESP32S2-WiFi"
    "esp32s3|esp32:esp32:adafruit_feather_esp32s3|bin|Thermostat-ESP32S3-WiFi"
    "esp32s2dual|esp32:esp32:adafruit_feather_esp32s2|bin|Thermostat-ESP32S2-DualNet"
)

# Dual network builds (ESP32 + PoE FeatherWing)
# These require #define DUAL_NETWORK at top of sketch
DUAL_NETWORK_BOARDS=(
    "esp32s2dual|esp32:esp32:adafruit_feather_esp32s2|bin|Thermostat-ESP32S2-DualNet"
    "esp32s3dual|esp32:esp32:adafruit_feather_esp32s3|bin|Thermostat-ESP32S3-DualNet"
    "esp32v2dual|esp32:esp32:adafruit_feather_esp32_v2|bin|Thermostat-ESP32V2-DualNet"
)

# Optional boards (must be explicitly named)
# Note: Some boards require additional board packages (see header for install instructions)
OPTIONAL_BOARDS=(
    # ESP32 variants (requires: esp32:esp32)
    "esp32v2|esp32:esp32:adafruit_feather_esp32_v2|bin|Thermostat-ESP32V2-WiFi"
    "esp32s2tft|esp32:esp32:adafruit_feather_esp32s2_tft|bin|Thermostat-ESP32S2TFT-WiFi"
    "esp32s3tft|esp32:esp32:adafruit_feather_esp32s3_tft|bin|Thermostat-ESP32S3TFT-WiFi"
    # SAMD variants (requires: adafruit:samd)
    "m4can|adafruit:samd:adafruit_feather_m4_can|bin|Thermostat-M4CAN-W5500"
    "m0express|adafruit:samd:adafruit_feather_m0_express|bin|Thermostat-M0Express-W5500"
    # nRF52840 variants (requires: adafruit:nrf52)
    "nrf52840|adafruit:nrf52:feather52840|zip|Thermostat-nRF52840-W5500"
    "nrf52840sense|adafruit:nrf52:feather52840sense|zip|Thermostat-nRF52840Sense-W5500"
    # STM32 (requires: STMicroelectronics:stm32)
    "stm32f405|STMicroelectronics:stm32:GenF4:pnum=FEATHER_F405|bin|Thermostat-STM32F405-W5500"
)

# Combine all boards for lookup
ALL_BOARDS=("${PRIMARY_BOARDS[@]}" "${DUAL_NETWORK_BOARDS[@]}" "${OPTIONAL_BOARDS[@]}")

# Print usage
usage() {
    echo "Usage: $0 [board...] [options]"
    echo ""
    echo "Options:"
    echo "  --help, -h      Show this help"
    echo "  --verbose, -v   Show full compiler output (also saves to build/<board>-error.log)"
    echo "  --flash         Flash firmware after building (ESP32 boards only)"
    echo "  --port PORT     Serial port for flashing (default: auto-detect)"
    echo "  --list-boards   List all available board FQBNs on this system"
    echo "  --list-ports    List available serial ports"
    echo "  --install-deps  Interactive installer for board packages and libraries"
    echo ""
    echo "Primary boards (included in './build-all.sh' with no arguments):"
    echo "  Board         FQBN                                    Package"
    echo "  -----------   --------------------------------------  --------------"
    for board_def in "${PRIMARY_BOARDS[@]}"; do
        IFS='|' read -r name fqbn ext outname <<< "$board_def"
        local pkg=$(echo "$fqbn" | cut -d: -f1-2)
        printf "  %-12s  %-38s  %s\n" "$name" "$fqbn" "$pkg"
    done
    echo ""
    echo "Dual network boards (ESP32 + PoE FeatherWing, requires #define DUAL_NETWORK):"
    echo "  Board         FQBN                                    Package"
    echo "  -----------   --------------------------------------  --------------"
    for board_def in "${DUAL_NETWORK_BOARDS[@]}"; do
        IFS='|' read -r name fqbn ext outname <<< "$board_def"
        local pkg=$(echo "$fqbn" | cut -d: -f1-2)
        printf "  %-12s  %-38s  %s\n" "$name" "$fqbn" "$pkg"
    done
    echo ""
    echo "Optional boards (must be explicitly named):"
    echo "  Board         FQBN                                    Package"
    echo "  -----------   --------------------------------------  --------------"
    for board_def in "${OPTIONAL_BOARDS[@]}"; do
        IFS='|' read -r name fqbn ext outname <<< "$board_def"
        local pkg=$(echo "$fqbn" | cut -d: -f1-2)
        printf "  %-12s  %-38s  %s\n" "$name" "$fqbn" "$pkg"
    done
    echo ""
    echo "Examples:"
    echo "  $0                    # Build all primary boards"
    echo "  $0 m4                 # Build only M4"
    echo "  $0 m0 esp32s2         # Build M0 and ESP32-S2"
    echo "  $0 nrf52840           # Build nRF52840 (requires adafruit:nrf52)"
    echo "  $0 all esp32v2        # Build all primary + ESP32 V2"
    echo ""
    echo "Flash examples:"
    echo "  $0 esp32s2 --flash                # Build and flash ESP32-S2 (auto-detect port)"
    echo "  $0 esp32s2 --flash --port /dev/ttyACM0  # Build and flash to specific port"
    echo ""
    echo "Note: --flash only works with a single ESP32 board target."
    echo "      Put the board in bootloader mode first: hold BOOT, tap RESET, release BOOT."
}

# Install dependencies with prompts
install_deps() {
    echo "=============================================="
    echo "eyeSPI Thermostat - Dependency Installer"
    echo "=============================================="
    echo ""
    
    # Check if arduino-cli exists
    if ! command -v arduino-cli &> /dev/null; then
        echo -e "${RED}Error: arduino-cli not found${NC}"
        echo ""
        echo "Install arduino-cli first:"
        echo "  curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sh"
        echo ""
        exit 1
    fi
    
    # Board manager URLs
    local URLS=(
        "https://adafruit.github.io/arduino-board-index/package_adafruit_index.json"
        "https://github.com/earlephilhower/arduino-pico/releases/download/global/package_rp2040_index.json"
        "https://espressif.github.io/arduino-esp32/package_esp32_index.json"
        "https://github.com/stm32duino/BoardManagerFiles/raw/main/package_stmicroelectronics_index.json"
    )
    
    # Primary board packages
    local PRIMARY_PACKAGES=(
        "adafruit:samd|Adafruit SAMD (M0, M4)"
        "rp2040:rp2040|Raspberry Pi RP2040"
        "esp32:esp32|Espressif ESP32"
    )
    
    # Optional board packages
    local OPTIONAL_PACKAGES=(
        "adafruit:nrf52|Adafruit nRF52 (BLE)"
        "STMicroelectronics:stm32|STM32 (STM32F405)"
    )
    
    # Required libraries (pt100rtd is installed separately from GitHub)
    local LIBRARIES=(
        "Adafruit MAX31865 library"
        "Adafruit GFX Library"
        "Adafruit ILI9341"
        "Adafruit FT6206 Library"
        "Adafruit Unified Sensor"
        "Adafruit BME680 Library"
        "Adafruit BME280 Library"
        "ArduinoJson"
        "SdFat - Adafruit Fork"
        "Ethernet"
        "WiFi101"
        "ArduinoOTA"
    )
    
    # Step 1: Configure board manager URLs
    echo -e "${YELLOW}Step 1: Configure board manager URLs${NC}"
    echo ""
    read -p "Add required board manager URLs? [Y/n] " -n 1 -r
    echo ""
    if [[ ! $REPLY =~ ^[Nn]$ ]]; then
        # Initialize config if needed
        arduino-cli config init 2>/dev/null || true
        
        for url in "${URLS[@]}"; do
            echo "  Adding: $url"
            arduino-cli config add board_manager.additional_urls "$url" 2>/dev/null || true
        done
        echo -e "${GREEN}✓ Board manager URLs configured${NC}"
    else
        echo "Skipped."
    fi
    echo ""
    
    # Step 2: Update index
    echo -e "${YELLOW}Step 2: Update board index${NC}"
    echo ""
    read -p "Update arduino-cli board index? [Y/n] " -n 1 -r
    echo ""
    if [[ ! $REPLY =~ ^[Nn]$ ]]; then
        arduino-cli core update-index
        echo -e "${GREEN}✓ Board index updated${NC}"
    else
        echo "Skipped."
    fi
    echo ""
    
    # Step 3: Install primary board packages
    echo -e "${YELLOW}Step 3: Install primary board packages${NC}"
    echo "These are required to build the main firmware variants."
    echo ""
    for pkg_def in "${PRIMARY_PACKAGES[@]}"; do
        IFS='|' read -r pkg desc <<< "$pkg_def"
        # Check if already installed
        if arduino-cli core list | grep -q "^$pkg"; then
            echo -e "  ${GREEN}✓${NC} $desc ($pkg) - already installed"
        else
            read -p "  Install $desc ($pkg)? [Y/n] " -n 1 -r
            echo ""
            if [[ ! $REPLY =~ ^[Nn]$ ]]; then
                arduino-cli core install "$pkg" && \
                    echo -e "  ${GREEN}✓ $desc installed${NC}" || \
                    echo -e "  ${RED}✗ Failed to install $desc${NC}"
            else
                echo "  Skipped."
            fi
        fi
    done
    echo ""
    
    # Step 4: Install optional board packages
    echo -e "${YELLOW}Step 4: Install optional board packages${NC}"
    echo "These enable additional board targets (nRF52840, STM32)."
    echo ""
    for pkg_def in "${OPTIONAL_PACKAGES[@]}"; do
        IFS='|' read -r pkg desc <<< "$pkg_def"
        # Check if already installed
        if arduino-cli core list | grep -q "^$pkg"; then
            echo -e "  ${GREEN}✓${NC} $desc ($pkg) - already installed"
        else
            read -p "  Install $desc ($pkg)? [y/N] " -n 1 -r
            echo ""
            if [[ $REPLY =~ ^[Yy]$ ]]; then
                arduino-cli core install "$pkg" && \
                    echo -e "  ${GREEN}✓ $desc installed${NC}" || \
                    echo -e "  ${RED}✗ Failed to install $desc${NC}"
            else
                echo "  Skipped."
            fi
        fi
    done
    echo ""
    
    # Step 5: Install libraries
    echo -e "${YELLOW}Step 5: Install required libraries${NC}"
    echo ""
    read -p "Install all required libraries? [Y/n] " -n 1 -r
    echo ""
    if [[ ! $REPLY =~ ^[Nn]$ ]]; then
        for lib in "${LIBRARIES[@]}"; do
            echo -n "  Installing: $lib... "
            if arduino-cli lib install "$lib" 2>/dev/null; then
                echo -e "${GREEN}✓${NC}"
            else
                echo -e "${YELLOW}(already installed or not found)${NC}"
            fi
        done
        echo -e "${GREEN}✓ Libraries installed${NC}"
    else
        echo "Skipped."
    fi
    echo ""

    # Step 6: Install pt100rtd from GitHub (not in Arduino Library Manager)
    echo -e "${YELLOW}Step 6: Install pt100rtd library from GitHub${NC}"
    local user_dir
    user_dir=$(arduino-cli config get directories.user 2>/dev/null)
    local pt100_path="$user_dir/libraries/pt100rtd"
    if [ -d "$pt100_path" ]; then
        echo -e "  ${GREEN}✓${NC} pt100rtd already installed at $pt100_path"
    else
        read -p "Clone pt100rtd from GitHub? [Y/n] " -n 1 -r
        echo ""
        if [[ ! $REPLY =~ ^[Nn]$ ]]; then
            git clone https://github.com/drhaney/pt100rtd.git "$pt100_path" && \
                echo -e "  ${GREEN}✓ pt100rtd installed${NC}" || \
                echo -e "  ${RED}✗ Failed to clone pt100rtd${NC}"
        else
            echo "  Skipped."
        fi
    fi
    echo ""

    # Done
    echo "=============================================="
    echo -e "${GREEN}Dependency installation complete!${NC}"
    echo "=============================================="
    echo ""
    
    # Attempt to patch pt100rtd library for ESP32 compatibility
    patch_pt100rtd
    
    echo "Run './build-all.sh' to build all primary firmware variants."
}

# Patch pt100rtd library for ESP32 compatibility
# ESP32 needs <pgmspace.h> instead of <avr/pgmspace.h>
# Also adds fallback PROGMEM defines for other platforms
patch_pt100rtd() {
    echo -e "${YELLOW}Checking pt100rtd library for ESP32 compatibility...${NC}"
    
    # Find pt100rtd library location
    local lib_path
    lib_path=$(arduino-cli config get directories.user 2>/dev/null)/libraries/pt100rtd
    
    if [ ! -d "$lib_path" ]; then
        # Try alternate location
        lib_path="$HOME/Arduino/libraries/pt100rtd"
    fi
    
    if [ ! -d "$lib_path" ]; then
        echo "  pt100rtd library not found, skipping patch"
        return
    fi
    
    local header="$lib_path/pt100rtd.h"
    if [ ! -f "$header" ]; then
        echo "  pt100rtd.h not found, skipping patch"
        return
    fi
    
    # Check if already patched (look for ESP32-specific include)
    if grep -q "defined(ESP32)" "$header"; then
        echo -e "  ${GREEN}pt100rtd already patched for ESP32${NC}"
        return
    fi
    
    echo "  Applying ESP32 pgmspace.h fix..."
    # Backup original
    cp "$header" "$header.bak"
    
    # The fix: Replace the pgmspace include block with a more comprehensive one
    # Original typically has: #if defined(__arm__) ... #include <avr/pgmspace.h>
    # ESP32/ESP8266 need <pgmspace.h> without avr/ prefix
    
    # Create the replacement block (ESP32 first, then group all avr/ users)
    local new_block='#if defined(ESP32) || defined(ESP8266)
  #include <pgmspace.h>
#elif defined(__AVR__) || defined(__arm__) || defined(__SAMD21__) || defined(__SAMD51__)
  #include <avr/pgmspace.h>
#else
  #ifndef PROGMEM
    #define PROGMEM
  #endif
  #ifndef pgm_read_word_near
    #define pgm_read_word_near(addr) (*(const uint16_t *)(addr))
  #endif
#endif'
    
    # Use awk to replace the pgmspace section
    awk -v new="$new_block" '
    /^#if defined\(__arm__\)/ { 
        in_block = 1
        print new
        next
    }
    in_block && /^#endif/ {
        in_block = 0
        next
    }
    in_block { next }
    { print }
    ' "$header" > "$header.tmp" && mv "$header.tmp" "$header"
    
    if grep -q "defined(ESP32)" "$header"; then
        echo -e "  ${GREEN}pt100rtd patched successfully${NC}"
        echo "  Backup saved to $header.bak"
    else
        echo -e "  ${RED}Patch failed${NC}"
        echo "  See PROJECT_DOCUMENTATION.txt for manual patching instructions"
        # Restore backup
        mv "$header.bak" "$header"
    fi
}

# List available boards
list_boards() {
    echo "Searching for installed Feather-compatible boards..."
    echo ""
    arduino-cli board listall | grep -iE "feather|adafruit" | sort
}

# Check if arduino-cli is installed
check_arduino_cli() {
    if ! command -v arduino-cli &> /dev/null; then
        echo -e "${RED}Error: arduino-cli not found${NC}"
        echo ""
        echo "Install arduino-cli:"
        echo "  curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sh"
        echo ""
        echo "Or via package manager:"
        echo "  brew install arduino-cli      # macOS"
        echo "  sudo apt install arduino-cli  # Debian/Ubuntu"
        exit 1
    fi
}

# Check if sketch exists
check_sketch() {
    if [ ! -f "$SKETCH" ]; then
        echo -e "${RED}Error: Sketch not found: $SKETCH${NC}"
        exit 1
    fi
}

# List available serial ports
list_ports() {
    echo "Available serial ports:"
    echo ""
    local found=false
    for port in /dev/ttyACM* /dev/ttyUSB*; do
        if [ -e "$port" ]; then
            found=true
            # Try to get device info from udevadm
            local info
            info=$(udevadm info --query=property "$port" 2>/dev/null | grep -E "ID_MODEL=|ID_VENDOR=" | head -2)
            local vendor model
            vendor=$(echo "$info" | grep "ID_VENDOR=" | cut -d= -f2)
            model=$(echo "$info" | grep "ID_MODEL=" | cut -d= -f2)
            if [ -n "$model" ]; then
                echo "  $port  ($vendor $model)"
            else
                echo "  $port"
            fi
        fi
    done
    if [ "$found" = false ]; then
        echo "  (none found)"
        echo ""
        echo "If flashing an ESP32, put it in bootloader mode first:"
        echo "  Hold BOOT, tap RESET, release BOOT"
    fi
}

# Auto-detect serial port for ESP32 flashing
detect_port() {
    local port=""

    # Prefer ttyACM (ESP32-S2/S3 native USB) over ttyUSB (UART bridges)
    for candidate in /dev/ttyACM* /dev/ttyUSB*; do
        if [ -e "$candidate" ]; then
            port="$candidate"
            break
        fi
    done

    echo "$port"
}

# Map board short name to esptool chip type
get_esp_chip() {
    local name="$1"
    case "$name" in
        esp32s2|esp32s2dual|esp32s2tft)  echo "esp32s2" ;;
        esp32s3|esp32s3dual|esp32s3tft)  echo "esp32s3" ;;
        esp32v2|esp32v2dual)             echo "esp32"   ;;
        *)                               echo ""        ;;
    esac
}

# Flash firmware to an ESP32 board
flash_board() {
    local name="$1"
    local fqbn="$2"
    local ext="$3"
    local outname="$4"
    local port="$5"

    local chip
    chip=$(get_esp_chip "$name")
    if [ -z "$chip" ]; then
        echo -e "${RED}Error: --flash is only supported for ESP32 boards${NC}"
        echo "  Board '$name' is not an ESP32 variant."
        return 1
    fi

    local bin_file="$OUT/$outname.$ext"
    local board_build="$BUILD_DIR/$name"
    if [ ! -f "$bin_file" ]; then
        echo -e "${RED}Error: Binary not found: $bin_file${NC}"
        echo "  Build the firmware first, or check for build errors above."
        return 1
    fi

    # Auto-detect port if not specified
    if [ -z "$port" ]; then
        echo "Auto-detecting serial port..."
        port=$(detect_port)
        if [ -z "$port" ]; then
            echo -e "${RED}Error: No serial port found${NC}"
            echo ""
            echo "Make sure the board is connected and in bootloader mode:"
            echo "  Hold BOOT, tap RESET, release BOOT"
            echo ""
            echo "Or specify a port with --port /dev/ttyACM0"
            return 1
        fi
        echo "  Detected: $port"
    fi

    # Verify port exists
    if [ ! -e "$port" ]; then
        echo -e "${RED}Error: Port $port does not exist${NC}"
        echo ""
        list_ports
        return 1
    fi

    echo -e "${YELLOW}Flashing: $outname.$ext -> $port ($chip)${NC}"

    # Prefer arduino-cli upload (handles flash offsets, bootloader, and partitions)
    if command -v arduino-cli &> /dev/null; then
        echo "  Using: arduino-cli upload (from build dir: $board_build)"
        local flash_output
        flash_output=$(arduino-cli upload \
            --fqbn "$fqbn" \
            --port "$port" \
            --input-dir "$board_build" 2>&1) && flash_status=0 || flash_status=$?

        if [ $flash_status -eq 0 ]; then
            echo -e "${GREEN}✓ Flash complete: $outname.$ext -> $port${NC}"
            echo "  Tap RESET to boot into new firmware."
            return 0
        else
            echo -e "${RED}✗ arduino-cli upload failed:${NC}"
            echo "$flash_output" | tail -10
            echo ""
            echo "  Falling back to esptool..."
        fi
    fi

    # Fallback: esptool.py (requires manual flash offset)
    local esptool_cmd=""
    if command -v esptool.py &> /dev/null; then
        esptool_cmd="esptool.py"
    elif command -v esptool &> /dev/null; then
        esptool_cmd="esptool"
    elif python3 -m esptool version &> /dev/null; then
        esptool_cmd="python3 -m esptool"
    fi

    if [ -z "$esptool_cmd" ]; then
        echo -e "${RED}Error: Neither arduino-cli upload nor esptool found${NC}"
        echo ""
        echo "Install esptool:"
        echo "  pip install esptool --break-system-packages"
        return 1
    fi

    echo "  Using: $esptool_cmd"

    # ESP32 app partition starts at 0x10000
    local flash_offset="0x10000"

    $esptool_cmd --chip "$chip" --port "$port" --baud 921600 \
        write_flash "$flash_offset" "$bin_file"

    if [ $? -eq 0 ]; then
        echo -e "${GREEN}✓ Flash complete: $outname.$ext -> $port${NC}"
        echo "  Tap RESET to boot into new firmware."
        return 0
    else
        echo -e "${RED}✗ Flash failed${NC}"
        return 1
    fi
}

# Build a single board
build_board() {
    local name="$1"
    local fqbn="$2"
    local ext="$3"
    local outname="$4"
    local is_dual="$5"  # Optional: "dual" for DUAL_NETWORK builds

    echo -e "${YELLOW}Building: $name ($fqbn)${NC}"
    
    # Create board-specific build directory
    local board_build="$BUILD_DIR/$name"
    mkdir -p "$board_build"
    
    # Build extra flags for DUAL_NETWORK mode
    # Use compiler.cpp.extra_flags to avoid overwriting platform's build.extra_flags
    local extra_flags=""
    if [ "$is_dual" = "dual" ]; then
        extra_flags="--build-property compiler.cpp.extra_flags=-DDUAL_NETWORK=1"
        echo "  (DUAL_NETWORK mode enabled)"
    fi
    
    # Compile (capture output and status separately)
    local compile_output
    local compile_status
    local logfile="$board_build/build-${name}.log"
    
    if [ "$VERBOSE" = true ]; then
        # Verbose mode: show output in real-time and capture it
        arduino-cli compile \
            --fqbn "$fqbn" \
            --build-path "$board_build" \
            --warnings default \
            $extra_flags \
            "$SKETCH" 2>&1 | tee "$logfile"
        compile_status=${PIPESTATUS[0]}
        compile_output=$(cat "$logfile")
    else
        # Normal mode: capture output silently
        compile_output=$(arduino-cli compile \
            --fqbn "$fqbn" \
            --build-path "$board_build" \
            --warnings default \
            $extra_flags \
            "$SKETCH" 2>&1) && compile_status=0 || compile_status=$?
        # Always save build log
        echo "$compile_output" > "$logfile"
    fi
    
    if [ $compile_status -eq 0 ]; then
        # Find and copy output file
        local src_file
        if [ "$ext" = "uf2" ]; then
            src_file=$(find "$board_build" -name "*.uf2" -type f 2>/dev/null | head -1)
        elif [ "$ext" = "zip" ]; then
            src_file=$(find "$board_build" -name "*.zip" -type f 2>/dev/null | head -1)
        else
            # For bin files, look for the main binary (not bootloader, partitions, etc.)
            src_file=$(find "$board_build" -name "Thermostat.ino.bin" -type f 2>/dev/null | head -1)
            if [ -z "$src_file" ]; then
                src_file=$(find "$board_build" -name "*.bin" -type f ! -name "*bootloader*" ! -name "*partitions*" 2>/dev/null | head -1)
            fi
        fi
        
        if [ -n "$src_file" ] && [ -f "$src_file" ]; then
            cp "$src_file" "$OUT/$outname.$ext"
            # Cross-platform file size (Linux uses -c, macOS uses -f)
            local size
            size=$(stat -c%s "$src_file" 2>/dev/null || stat -f%z "$src_file" 2>/dev/null || echo "unknown")
            echo -e "${GREEN}✓ $name: $OUT/$outname.$ext ($size bytes)${NC}"
            echo -e "  ${YELLOW}Build log: $logfile${NC}"
            return 0
        else
            echo -e "${RED}✗ $name: Build succeeded but output file not found${NC}"
            echo "  Build directory contents:"
            ls -la "$board_build"/*.{bin,uf2,zip} 2>/dev/null || echo "  (no bin/uf2/zip files found)"
            echo -e "  ${YELLOW}Build log: $logfile${NC}"
            return 1
        fi
    else
        echo -e "${RED}✗ $name: Compilation failed${NC}"
        # Show last 20 lines in terminal
        echo "$compile_output" | tail -20
        echo -e "  ${YELLOW}Full build log: $logfile${NC}"
        return 1
    fi
}

# Main
main() {
    # Handle special options first
    local args=()
    local skip_next=false
    for i in $(seq 1 $#); do
        local arg="${!i}"
        if [ "$skip_next" = true ]; then
            skip_next=false
            continue
        fi
        case "$arg" in
            -h|--help)
                usage
                exit 0
                ;;
            --verbose|-v)
                VERBOSE=true
                ;;
            --flash)
                FLASH=true
                ;;
            --port)
                local next=$((i + 1))
                FLASH_PORT="${!next}"
                skip_next=true
                if [ -z "$FLASH_PORT" ]; then
                    echo -e "${RED}Error: --port requires a device path (e.g. /dev/ttyACM0)${NC}"
                    exit 1
                fi
                ;;
            --install-deps)
                install_deps
                exit 0
                ;;
            --list-boards)
                check_arduino_cli
                list_boards
                exit 0
                ;;
            --list-ports)
                list_ports
                exit 0
                ;;
            *)
                args+=("$arg")
                ;;
        esac
    done
    
    check_arduino_cli
    check_sketch
    
    mkdir -p "$OUT"
    
    # Determine which boards to build
    local boards_to_build=()
    
    if [ ${#args[@]} -eq 0 ]; then
        # No board args: build all primary (but not if --flash with no board specified)
        if [ "$FLASH" = true ]; then
            echo -e "${RED}Error: --flash requires specifying a single board target${NC}"
            echo ""
            echo "Example: $0 esp32s2 --flash"
            exit 1
        fi
        for board_def in "${PRIMARY_BOARDS[@]}"; do
            boards_to_build+=("$board_def")
        done
    else
        # Build specified boards
        for arg in "${args[@]}"; do
            # Special keyword "all" adds all primary boards
            if [ "$arg" = "all" ]; then
                for board_def in "${PRIMARY_BOARDS[@]}"; do
                    # Avoid duplicates
                    local already_added=false
                    for existing in "${boards_to_build[@]}"; do
                        if [ "$existing" = "$board_def" ]; then
                            already_added=true
                            break
                        fi
                    done
                    if [ "$already_added" = false ]; then
                        boards_to_build+=("$board_def")
                    fi
                done
                continue
            fi
            
            local found=false
            for board_def in "${ALL_BOARDS[@]}"; do
                IFS='|' read -r name fqbn ext outname <<< "$board_def"
                if [ "$arg" = "$name" ]; then
                    boards_to_build+=("$board_def")
                    found=true
                    break
                fi
            done
            
            if [ "$found" = false ]; then
                echo -e "${RED}Unknown board: $arg${NC}"
                echo ""
                usage
                exit 1
            fi
        done
    fi

    # Validate --flash usage
    if [ "$FLASH" = true ] && [ ${#boards_to_build[@]} -gt 1 ]; then
        echo -e "${RED}Error: --flash only works with a single board target${NC}"
        echo ""
        echo "Specify one board to build and flash:"
        echo "  $0 esp32s2 --flash"
        exit 1
    fi
    
    echo "=============================================="
    echo "eyeSPI Thermostat Firmware Builder"
    echo "=============================================="
    echo "Sketch: $SKETCH"
    echo "Output: $OUT"
    echo ""
    
    local success=0
    local failed=0
    
    for board_def in "${boards_to_build[@]}"; do
        IFS='|' read -r name fqbn ext outname <<< "$board_def"
        echo ""
        
        # Check if this is a dual network board
        local is_dual=""
        for dual_def in "${DUAL_NETWORK_BOARDS[@]}"; do
            IFS='|' read -r dual_name dual_fqbn dual_ext dual_outname <<< "$dual_def"
            if [ "$name" = "$dual_name" ]; then
                is_dual="dual"
                break
            fi
        done
        
        if build_board "$name" "$fqbn" "$ext" "$outname" "$is_dual"; then
            ((success++))
        else
            ((failed++))
        fi
    done
    
    echo ""
    echo "=============================================="
    echo -e "Complete: ${GREEN}$success succeeded${NC}, ${RED}$failed failed${NC}"
    echo "=============================================="
    
    # Flash step (only if --flash and build succeeded)
    if [ "$FLASH" = true ]; then
        if [ $failed -gt 0 ]; then
            echo ""
            echo -e "${RED}Skipping flash due to build failure${NC}"
            exit 1
        fi

        echo ""
        echo "=============================================="
        echo "Flashing firmware"
        echo "=============================================="

        local board_def="${boards_to_build[0]}"
        IFS='|' read -r name fqbn ext outname <<< "$board_def"

        echo ""
        if flash_board "$name" "$fqbn" "$ext" "$outname" "$FLASH_PORT"; then
            exit 0
        else
            exit 1
        fi
    fi
    
    if [ $failed -gt 0 ]; then
        exit 1
    fi
}

main "$@"
