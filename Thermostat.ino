
//////
//
//  Parts to build: 
// Controller
//      https://www.tindie.com/products/silicognition/poe-featherwing/
//      https://www.tindie.com/products/silicognition/m4-shim/
//  or
//      https://www.adafruit.com/product/3061
// Screen      
//      https://www.adafruit.com/product/1947
//  or
//      https://www.adafruit.com/product/1651
//  or
//      https://www.adafruit.com/product/1480
// TempSensor
//      https://www.adafruit.com/product/5046
//
//
//
//
//
////

#pragma GCC diagnostic push
//#pragma GCC diagnostic ignored "-Wc++17-extensions"
#pragma GCC diagnostic ignored "-Wstrict-aliasing"
#pragma GCC diagnostic ignored "-Wstrict-overflow"
//#pragma GCC diagnostic ignored "-Wstrict-prototypes"
#pragma GCC diagnostic ignored "-Wstrict-null-sentinel"


#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <map>
#include <Adafruit_MAX31865.h> // PT100
#include <pt100rtd.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <Adafruit_FT6206.h>
#include "favicon_ico.h"
#include <Fonts/FreeSans9pt7b.h>
#include <Fonts/FreeSansBold9pt7b.h>
#include <Fonts/FreeSans24pt7b.h>
#include "LCD7segment48pt7b.h"  // 7 segment digital monospace font, 48pt
#include "LCD7segment72pt7b.h"  // 7 segment digital monospace font, 72pt
#include <Adafruit_Sensor.h>
#include <ArduinoJson.h>
#include <SdFat.h>  // Better than the standard SD.h; Allows for exFAT!

const int NUM_FONTS = 5;
// Create a map of font names to GFXfont objects
std::map<String, const GFXfont*> GFXfonts = {
    {"FreeSans24pt7b", &FreeSans24pt7b},
    {"FreeSans9pt7b", &FreeSans9pt7b},
    {"FreeSansBold9pt7b", &FreeSansBold9pt7b},
    {"LCD7segment48pt7b", &LCD7segment48pt7b},
    {"LCD7segment72pt7b", &LCD7segment72pt7b}
};

typedef struct {
  int first;
  int second;
} pair;

// Celsius to Fahrenheit conversion macro
#define C2F(c) ((9 * c / 5) + 32)

//#warning Can be useful to emit messages during compile time.

#define BAUDRATE 9600
#define WEBSERVER_PORT 80

// Uncomment *one* of the following for what networking to use (for webserver):
//#define W5500
//#define WINC1500

// What temp sensor to use:
//#define PT100
//#define BME688

// Ethernet variables;
#ifdef W5500
#define HardwareNet Ethernet
#define HardwareServer EthernetServer
#define HardwareClient EthernetClient
#include <Ethernet.h>
#define W5500_MAC_I2C 0x50
#define W5500_CS_PIN 10  // Pin for CS
// Enter a MAC address for your controller below.
// Newer Ethernet shields have a MAC address printed on a sticker on the shield
byte W5500_mac[] = { 0x0E, 0xAD, 0xBE, 0xEF, 0xFE, 0xE0 };
// Enter an IP address for your controller below.
// You can use the same address as the host PC you're using to view the output
// if you're using a router
// Set the static IP address to use if the DHCP fails to assign
IPAddress eth_ip(10, 1, 0, 71);
IPAddress eth_dns(10, 1, 0, 1);

byte readRegister(byte r)
{
  unsigned char v;
  Wire.beginTransmission(W5500_MAC_I2C);
  Wire.write(r);  // Register to read
  Wire.endTransmission();

  Wire.requestFrom(W5500_MAC_I2C, 1); // Read a byte
  while(!Wire.available());  // Wait
  v = Wire.read();
  return v;
}


// WiFi
#elif defined(WINC1500)
#define HardwareNet WiFi
#define HardwareServer WiFiServer
#define HardwareClient WiFiClient
#include <WiFi101.h>
#include "arduino_secrets.h" 
///////please enter your sensitive data in the Secret tab/arduino_secrets.h
char ssid[] = SECRET_SSID;    // your network SSID (name)
char pass[] = SECRET_PASS;    // your network password (use for WPA, or use as key for WEP)
int keyIndex = 0;             // your network key Index number (needed only for WEP)
int status = WL_IDLE_STATUS;
#else
// Set up some dummy skeleton classes in case neither WiFi nor Ethernet are available.
// This is only here to make it compile cleanly.
// These could be reimplemented for Bluetooth networking if someone really wanted.
class HardwareClient {  // Must be defined before HArdwareServer
public:
  HardwareClient() {}
  HardwareClient(bool value) {}
  HardwareClient(const HardwareClient& other) { }  // Copy constructor
  HardwareClient& operator=(bool value) { return *this; }
  bool connected() { return false; }
  bool available() { return false; }
  String readStringUntil(char c) { return ""; }
  void stop() {}
  void flush() {}
  void write(uint8_t *b, int s) {}
  void print(int s) {}
  void print(String s="") {}
  void println(int s) {}
  void println(String s="") {}
  operator bool() const { return false; }
};

class HardwareServer {
public:
  HardwareServer() {}
  HardwareServer(int value) {}
  void begin() {}
  void write(char c) {}
  HardwareClient available() { return HardwareClient(false); }
  char read() { return 0; }
  bool connected() { return false; }
};
#endif

HardwareServer server(WEBSERVER_PORT);

#ifdef IPAddress
String IpAddress2String(const IPAddress& ipAddress)
{
  return String(ipAddress[0]) + String(".") +\
  String(ipAddress[1]) + String(".") +\
  String(ipAddress[2]) + String(".") +\
  String(ipAddress[3]); 
}
#endif

class Canvas_GFX : public Adafruit_GFX {
private:
  const GFXfont  * font = NULL;
  const char *fontName = NULL;
  uint8_t  * bitmaps = NULL;
  GFXglyph * glyphs = NULL;
  int16_t firstGlyph = -1;
  int16_t lastGlyph = -1;
  HardwareClient* cl = NULL;
  String canvas_id = "canvas", bgcolor = "#000000";
  int16_t max_x = 0, max_y = 0;
public:
  // Constructor that takes the width and height of the screen
  Canvas_GFX(int16_t w, int16_t h) : Adafruit_GFX(w, h), font(NULL), bitmaps(NULL), glyphs(NULL), firstGlyph(-1), lastGlyph(-1) {}
  Canvas_GFX() : Adafruit_GFX(0, 0), font(NULL), bitmaps(NULL), glyphs(NULL), firstGlyph(-1), lastGlyph(-1) {}

  void setFont(const GFXfont *font = NULL) {
    this->font = font;
    setFontName(this->font);
    if (font != NULL) {
      bitmaps = (uint8_t  *)font->bitmap;
      glyphs = (GFXglyph *)font->glyph;
      firstGlyph = font->first;
      lastGlyph = font->last;
    } else { // Font not found
      this->font = NULL;
      fontName = NULL;
      bitmaps = NULL;
      glyphs = NULL;
      firstGlyph = -1;
      lastGlyph = -1;
    }
  }

  void setFont(char *fontName = NULL) {
    // Create a String object with the font name
    String fontNameObject = String(fontName);
    if (fontNameObject == "FreeSans24pt7b") {
      setFont(&FreeSans24pt7b);
    } else if (fontNameObject == "FreeSans9pt7b") {
      setFont(&FreeSans9pt7b);
    } else if (fontNameObject == "FreeSansBold9pt7b") {
      setFont(&FreeSansBold9pt7b);
    } else if (fontNameObject == "LCD7segment48pt7b") {
      setFont(&LCD7segment48pt7b);
    } else if (fontNameObject == "LCD7segment72pt7b") {
      setFont(&LCD7segment72pt7b);
    } else { // Font not found
      setFont((GFXfont *)NULL);
    }
  }


  // Returns the first glyph in the font
  int16_t firstChar() {
    return firstGlyph;
  }

  // Returns the last glyph in the font
  int16_t lastChar() {
    return lastGlyph;
  }

  // Returns the name of the font currently set for this object.
  // Returns NULL if no font is set.
  const char *getFontName() {
    return this->fontName;
  }

  void setFontName(const GFXfont *font) {
    if (font == &FreeSans24pt7b) {
      fontName = "FreeSans24pt7b";
    } else if (font == &FreeSans9pt7b) {
      fontName = "FreeSans9pt7b";
    } else if (font == &FreeSansBold9pt7b) {
      fontName = "FreeSansBold9pt7b";
    } else if (font == &LCD7segment48pt7b) {
      fontName = "LCD7segment48pt7b";
    } else if (font == &LCD7segment72pt7b) {
      fontName = "LCD7segment72pt7b";
    } else { // Font not found
      fontName = NULL;
    }
  }

  // Returns the width of the specified character, or 0 if the character
  // is not in the font or if the glyph is an empty placeholder
  int16_t getCharWidth(int16_t c) {
    GFXglyph *glyph = getGlyph(c);
    if (glyph != NULL) {
      return glyph->xAdvance;
    } else {
      return 0;
    }
  }

  // Returns the height of the specified character, or 0 if the character
  // is not in the font or if the glyph is an empty placeholder
  int16_t getCharHeight(int16_t c) {
    GFXglyph *glyph = getGlyph(c);
    if (glyph != NULL) {
      return glyph->height;
    } else {
      return 0;
    }
  }

  // Returns the glyph for the specified character, or NULL if the character
  // is not in the font or if the glyph is an empty placeholder
  GFXglyph* getGlyph(int16_t c) {
    if (c >= firstGlyph && c <= lastGlyph &&
        glyphs[c - firstGlyph].bitmapOffset != 0 &&
        glyphs[c - firstGlyph].width != 0 &&
        glyphs[c - firstGlyph].height != 0 &&
        glyphs[c - firstGlyph].xAdvance != 0 &&
        glyphs[c - firstGlyph].xOffset != 0 &&
        glyphs[c - firstGlyph].yOffset != 0) {
      return &glyphs[c - firstGlyph];
    } else {
      return NULL;
    }
  }

  // Function to set the HardwareClient and the dimensions of the HTML5 canvas
  void setCanvas(HardwareClient& cl, int16_t x, int16_t y) {
    // Store the HardwareClient in the private variable
    this->cl = &cl;
    // Set the dimensions of the HTML5 canvas
    max_x = x;
    max_y = y;
  }

  // Function to return the height of the HTML5 canvas
  int16_t height() {
    return max_y;
  }

  // Function to return the width of the HTML5 canvas
  int16_t width() {
    return max_x;
  }

  // Function to send the HTML5 canvas element to the web client with the default canvas ID and background color
  void sendCanvas() {
    // Call the sendCanvas function with the default canvas ID and background color
    sendCanvas("canvas", "#000000");
  }

  // Function to send the HTML5 canvas element to the web client with a specified canvas ID and default background color
  void sendCanvas(String id) {
    // Call the sendCanvas function with the specified canvas ID and default background color
    sendCanvas(id, "#000000");
  }

  // Function to send the HTML5 canvas element to the web client with a specified canvas ID and background color
  void sendCanvas(String id, String bgcolor) {
    // Store the canvas ID and background color in the private variables
    canvas_id = id;
    this->bgcolor = bgcolor;
    // Use the HardwareClient to send the HTML code for the canvas element to the web client
    cl->println("<canvas id='" + canvas_id + "' width='" + String(max_x) + "' height='" + String(max_y) + "' style='background-color: " + bgcolor + ";'></canvas>");
    cl->println("<script>");
    cl->println("var canvas = document.getElementById('" + canvas_id + "');");
    cl->println("var " + canvas_id + "_ctx = canvas.getContext('2d');");
    cl->println("</script>");
  }

  // Function to set the background color of the HTML5 canvas on the web client
  void setBackgroundColor(String bgcolor) {
    // Store the canvas background color in the private variable
    this->bgcolor = bgcolor;
    // Use the HardwareClient to send a JavaScript command to the web client
    cl->println("document.getElementById('" + canvas_id + "').style.backgroundColor = '" + bgcolor + "';");
  }

  // Override the drawRoundRect function in Adafruit_GFX
  void drawRoundRect(int16_t x, int16_t y, int16_t w, int16_t h, int16_t r, uint16_t color) {
    // Use the HardwareClient to send a JavaScript command to the web client
    cl->println(canvas_id + "_ctx.strokeStyle = '#" + String(color, HEX) + "';");
    cl->println(canvas_id + "_ctx.lineWidth = 1;");
    cl->println(canvas_id + "_ctx.beginPath();");
    cl->println(canvas_id + "_ctx.moveTo(" + String(x + r) + ", " + String(y) + ");");
    cl->println(canvas_id + "_ctx.lineTo(" + String(x + w - r) + ", " + String(y) + ");");
    cl->println(canvas_id + "_ctx.arcTo(" + String(x + w) + ", " + String(y) + ", " + String(x + w) + ", " + String(y + r) + ", " + String(r) + ");");
    cl->println(canvas_id + "_ctx.lineTo(" + String(x + w) + ", " + String(y + h - r) + ");");
    cl->println(canvas_id + "_ctx.arcTo(" + String(x + w) + ", " + String(y + h) + ", " + String(x + w - r) + ", " + String(y + h) + ", " + String(r) + ");");
    cl->println(canvas_id + "_ctx.lineTo(" + String(x + r) + ", " + String(y + h) + ");");
    cl->println(canvas_id + "_ctx.arcTo(" + String(x) + ", " + String(y + h) + ", " + String(x) + ", " + String(y + h - r) + ", " + String(r) + ");");
    cl->println(canvas_id + "_ctx.lineTo(" + String(x) + ", " + String(y + r) + ");");
    cl->println(canvas_id + "_ctx.arcTo(" + String(x) + ", " + String(y) + ", " + String(x + r) + ", " + String(y) + ", " + String(r) + ");");
    cl->println(canvas_id + "_ctx.stroke();");
    cl->println(canvas_id + "_ctx.closePath();");
  }

  // Forward calls to the base class functions
  void drawPixel(int16_t x, int16_t y, uint16_t color) override {
    // Use the HardwareClient to send a JavaScript command to the web client
    cl->println(canvas_id + "_ctx.fillStyle = '#" + String(color, HEX) + "';");
    cl->println(canvas_id + "_ctx.fillRect(" + String(x) + ", " + String(y) + ", 1, 1);");
  }

/*
  void startWrite(void) override {
    Adafruit_GFX::startWrite();
  }
  void writePixel(int16_t x, int16_t y, uint16_t color) override {
    Adafruit_GFX::writePixel(x, y, color);
  }
  void writeFillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color) override {
    Adafruit_GFX::writeFillRect(x, y, w, h, color);
  }
  void writeFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color) override {
    Adafruit_GFX::writeFastVLine(x, y, h, color);
  }
  void writeFastHLine(int16_t x, int16_t y, int16_t w, uint16_t color) override {
    Adafruit_GFX::writeFastHLine(x, y, w, color);
  }
  void writeLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color) override {
    Adafruit_GFX::writeLine(x0, y0, x1, y1, color);
  }
  void endWrite(void) override {
    Adafruit_GFX::endWrite();
  }
  void setRotation(uint8_t r) override {
    Adafruit_GFX::setRotation(r);
  }
  void invertDisplay(bool i) override {
    Adafruit_GFX::invertDisplay(i);
  }
  void drawFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color) override {
    Adafruit_GFX::drawFastVLine(x, y, h, color);
  }
  void drawFastHLine(int16_t x, int16_t y, int16_t w, uint16_t color) override {
    Adafruit_GFX::drawFastHLine(x, y, w, color);
  }
  void fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color) override {
    Adafruit_GFX::fillRect(x, y, w, h, color);
  }
  void fillScreen(uint16_t color) override {
    Adafruit_GFX::fillScreen(color);
  }
  void drawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color) override {
    Adafruit_GFX::drawLine(x0, y0, x1, y1, color);
  }
  void drawRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color) override {
    Adafruit_GFX::drawRect(x, y, w, h, color);
  }*/

};

#ifdef ARDUINO_SAMD_ZERO
//#warning "ARDUINO_SAMD_ZERO defined"
#endif
#ifdef ARDUINO_SAMD_FEATHER_M0
//#warning "ARDUINO_SAMD_FEATHER_M0 defined"
#endif
#ifdef ARDUINO_SAMD_ADAFRUIT
//#warning "ARDUINO_SAMD_ADAFRUIT defined"
#endif
#ifdef ARDUINO_SAMD_ZERO
//#warning "ARDUINO_SAMD_ZERO defined"
#endif
#ifdef ARDUINO_SAMD_ZERO
//#warning "ARDUINO_SAMD_ZERO defined"
#endif

#if defined(__AVR_ATmega2560__)
const String curBoard = "Arduino MEGA 2560";
// For the Adafruit shield, these are the default.
#define TFT_DC 9
#define TFT_CS 6  // 10 is taken by the W5500 PoE Ethernet Wing
#define SD_CS  4  // SD card select pin
#elif defined(ARDUINO_SAMD_ZERO)
const String curBoard = "Adafruit Feather M0";
#define TFT_DC 9
#define TFT_CS 6  // 10 is taken by the W5500 PoE Ethernet Wing
#define SD_CS  5  // SD card select pin
#elif defined(ARDUINO_SAMD_ADAFRUIT)
const String curBoard = "Adafruit Feather M4";
#define TFT_DC 9
#define TFT_CS 6  // 10 is taken by the W5500 PoE Ethernet Wing
#define SD_CS  5  // SD card select pin
#else
const String curLine = String(__LINE__);
const String curBoard = "Unknown board on line " + curLine;
#define TFT_DC 9
#define TFT_CS 6  // 10 is taken by the W5500 PoE Ethernet Wing
#define SD_CS  5 // SD card select pin (currently not used)
#endif

// Use hardware SPI (on Uno, #13, #12, #11) and the above for CS/DC
Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC);
// If using the breakout, change pins as desired
//Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_MOSI, TFT_CLK, TFT_RST, TFT_MISO);

// The FT6206 uses hardware I2C (SCL/SDA)
Adafruit_FT6206 ts = Adafruit_FT6206();

// Begin SDFat SD Card
//------------------------------------------------------------------------------
// This example was designed for exFAT but will support FAT16/FAT32.
// Note: Uno will not support SD_FAT_TYPE = 3.
// SD_FAT_TYPE = 0 for SdFat/File as defined in SdFatConfig.h,
// 1 for FAT16/FAT32, 2 for exFAT, 3 for FAT16/FAT32 and exFAT.
#define SD_FAT_TYPE 2
//------------------------------------------------------------------------------
// Interval between data records in microseconds.
// Try 250 with Teensy 3.6, Due, or STM32.
// Try 2000 with AVR boards.
// Try 4000 with SAMD Zero boards.
const uint32_t LOG_INTERVAL_USEC = 2000;

// Set USE_RTC nonzero for file timestamps.
// RAM use will be marginal on Uno with RTClib.
// 0 - RTC not used
// 1 - DS1307
// 2 - DS3231
// 3 - PCF8523
#define USE_RTC 0
#if USE_RTC
#include "RTClib.h"
#endif  // USE_RTC

// LED to light if overruns occur.
#define ERROR_LED_PIN -1

// FIFO SIZE - 512 byte sectors.  Modify for your board.
#ifdef __AVR_ATmega328P__
// Use 512 bytes for 328 boards.
#define FIFO_SIZE_SECTORS 1
#elif defined(__AVR__)
// Use 2 KiB for other AVR boards.
#define FIFO_SIZE_SECTORS 4
#else  // __AVR_ATmega328P__
// Use 8 KiB for non-AVR boards.
#define FIFO_SIZE_SECTORS 16
#endif  // __AVR_ATmega328P__
//const uint64_t PREALLOCATE_SIZE  =  (uint64_t)PREALLOCATE_SIZE_MiB << 20;
// Max length of file name including zero byte.
#define FILE_NAME_DIM 40
// Max number of records to buffer while SD is busy.
//const size_t FIFO_DIM = 512*FIFO_SIZE_SECTORS/sizeof(data_t);

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
#else  // SD_FAT_TYPE
#error Invalid SD_FAT_TYPE
#endif  // SD_FAT_TYPE

sd_t sd;

// End SDFat SD Card

#ifdef PT100
// Use software SPI: CS, DI, DO, CLK
//Adafruit_MAX31865 thermo = Adafruit_MAX31865(10, 11, 12, 13);
// use hardware SPI, just pass in the CS pin
Adafruit_MAX31865 thermo = Adafruit_MAX31865(12);
pt100rtd PT100lt = pt100rtd();

// The value of the Rref resistor. Use 430.0 for PT100 and 4300.0 for PT1000
#define RREF      429.83  // Actual value from REF resistor on board.
#define RREF2     428.985   // 429.83
// The 'nominal' 0-degrees-C resistance of the sensor
// 100.0 for PT100, 1000.0 for PT1000
#define RNOMINAL  100.0
#endif

#ifdef BME688
#include <bsec.h> // BME688
#include <Adafruit_BME680.h> // BME688
#define BME_SCK 13
#define BME_MISO 12
#define BME_MOSI 11
#define BME_CS 10

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME680 bme; // I2C
//Adafruit_BME680 bme(BME_CS); // hardware SPI (conflicts with W5500 & TFT)
//Adafruit_BME680 bme(BME_CS, BME_MOSI, BME_MISO,  BME_SCK);

// Create an object of the class Bsec
Bsec iaqSensor;
void checkIaqSensorStatus(void);
void errLeds(void);
#endif

// Helper functions declarations
unsigned long LayoutDisplay();

int myround(double mydecimal) {
#ifdef BME688
  return((int)(mydecimal+0.5));
#else // PT100
  return((int)(mydecimal));
#endif
}

int TFTbackgroundColor = ILI9341_BLACK;
int coordTouchLight[] = {0, 0, 0, 0, 0};
int coordTouchSystem[] = {0, 0, 0, 0, 0};
int coordMinus[] = {0, 0};
int coordPlus[] = {0, 0};
bool isHeatOn = true;
bool isCurHeatOn = false;
double curIndoorTemp = 71.0;
int curIndoorTempOld = 71;
int targIndoorTemp = 71;
int targIndoorTempOld = 71;
bool headerPrinted = true;
bool doSettings = false;

/*
#define ILI9341_RDMODE 0x0A     ///< Read Display Power Mode
#define ILI9341_RDMADCTL 0x0B   ///< Read Display MADCTL
#define ILI9341_RDPIXFMT 0x0C   ///< Read Display Pixel Format
#define ILI9341_RDIMGFMT 0x0D   ///< Read Display Image Format
#define ILI9341_RDSELFDIAG 0x0F ///< Read Display Self-Diagnostic Result
uint8_t Adafruit_ILI9341::readcommand8(uint8_t commandByte, uint8_t index);

// Page Address Set(2B) = Row; Column Address Set(2A) = Column

*/

void setup() {
  // put your setup code here, to run once:
  char printbuf[256] = "";

  Serial.begin(BAUDRATE);
  unsigned long timeout = millis() + 5000; // 30 second timeout;
  while (!Serial && millis() < timeout); // wait for serial port to connect. Needed for native USB port only

  // Begin TFT display
  Serial.print("Thermostat ILI9341 Test! on "); 
  Serial.println(curBoard); 
  tft.begin();

  // read diagnostics (optional but can help debug problems)
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
  
  Serial.println(F("Done!"));
  tft.fillScreen(TFTbackgroundColor);
  // origin = left,top landscape (USB left upper)
  tft.setRotation(1);
  tft.setFont();
  tft.setTextSize(1);
  tft.setTextWrap(false);
  sprintf(printbuf, "Max X: %d, Max Y: %d", tft.width(), tft.height());
  Serial.println(printbuf);
  // maxX = tft.width();
  // maxY = tft.height(); 
  // End TFT display

  // Begin SD Card
//  if (!SD.begin(SD_CS)) {
//    Serial.println("SD Card initialization failed!");
//  } else {
//    pass;
//  }
  // End SD Card

  // Begin FT6206 Touch Screen
  if (!ts.begin(40)) { 
    Serial.println("Unable to start touchscreen.");
  } 
  else { 
    Serial.println("Touchscreen started."); 
  }
  // End FT6206 Touch Screen

#ifdef PT100
  thermo.begin(MAX31865_3WIRE);  // set to 2WIRE or 4WIRE as necessary
#endif

#ifdef BME688
  // Begin Temp/Environ Sensor
  Serial.println(F("BME680 test"));

  if (!bme.begin()) {
    Serial.println("Could not find a valid BME680 sensor, check wiring!");
    while (!bme.begin());
  } else {

    // Set up oversampling and filter initialization
    bme.setTemperatureOversampling(BME680_OS_8X);
    bme.setHumidityOversampling(BME680_OS_2X);
    bme.setPressureOversampling(BME680_OS_4X);
    bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
    bme.setGasHeater(0, 150); // 320*C for 150 ms
  }
  // End Temp/Environ Sensor
#endif

  // Ethernet
  #ifdef W5500
    Serial.println("Starting W5500");
    Wire.begin();  // Start I2C bus
    // Read the MAC programmed in the 24AA02E48 chip
    W5500_mac[0] = readRegister(0xFA);
    W5500_mac[1] = readRegister(0xFB);
    W5500_mac[2] = readRegister(0xFC);
    W5500_mac[3] = readRegister(0xFD);
    W5500_mac[4] = readRegister(0xFE);
    W5500_mac[5] = readRegister(0xFF);
    int DHCP_status = 0;

    // You can use Ethernet.init(pin) to configure the CS pin
    Ethernet.init(W5500_CS_PIN);  // Most Arduino shields use pin 10
    // start the Ethernet connection:
    Serial.println("Initialize Ethernet with DHCP:");
    DHCP_status = Ethernet.begin(W5500_mac);
    if (Ethernet.hardwareStatus() == EthernetNoHardware) {
      Serial.println("Ethernet shield was not found.");
    } else if (Ethernet.linkStatus() == LinkOFF) {
      Serial.println("Ethernet cable is not connected.");
    } else {
      if (DHCP_status == 0) {
        Serial.println("Failed to configure Ethernet using DHCP");
        // Check for Ethernet hardware present
        if (Ethernet.linkStatus() == LinkOFF) {
          Serial.println("Ethernet cable is not connected.");
        }
        // try to congifure using IP address instead of DHCP:
        Ethernet.begin(W5500_mac, eth_ip, eth_dns);
        Serial.print("My IP address: ");
        Serial.println(Ethernet.localIP());
      } else {
        Serial.print("  DHCP assigned IP ");
        Serial.println(Ethernet.localIP());
      }
    }

  // WiFi
  #elif defined(WINC1500)
    //Configure pins for Adafruit ATWINC1500 Feather
    WiFi.setPins(8,7,4,2);
    // check for the presence of the shield:
    if (WiFi.status() == WL_NO_SHIELD) {
      Serial.println("WiFi shield not present");
    }

    int linelen = 25;
    WiFi.begin(ssid, pass);
    Serial.print("Trying to connect WiFi...");
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print("."); linelen++;
      if (linelen == 80) {
        Serial.println(); linelen=0;
      }
    }
    if (linelen > 65)
      Serial.println();
    Serial.println("WiFi connected");

    // Print the IP address
    Serial.print("server is at ");
    Serial.println(IpAddress2String(WiFi.localIP()));
  #else
    // code for no shield goes here
    Serial.println("No server capable networking shield detected!");
  #endif
  server.begin();  // start the web server on port WEBSERVER_PORT
  Serial.println("Webserver instance started");

  Serial.println("Setup complete");
}

void loop() {
  //char printbuf[256] = "";

#ifdef PT100
  uint16_t ohmsx100, rtd = thermo.readRTD();
  uint32_t dummy;
  float ohms, Tlut, Tcvd, Tcube, Tpoly, Trpoly;
  float ratio = rtd;

  ratio /= 32768;
//#ifdef PRINT_TEMPS
  Serial.println();
  Serial.print("RTD value: "); Serial.println(rtd);
  Serial.print("Lower Thresh: "); Serial.println(thermo.getLowerThreshold());
  Serial.print("Upper Thresh: "); Serial.println(thermo.getUpperThreshold());
  Serial.print("Ratio = "); Serial.println(ratio,8);
  Serial.print("Resistance = "); Serial.println(RREF*ratio,8);
  curIndoorTemp = C2F(thermo.temperature(RNOMINAL, RREF2));
  Serial.print("Temperature = "); Serial.println(curIndoorTemp);
  curIndoorTemp = C2F(thermo.calculateTemperature(rtd, RNOMINAL, RREF2));
  Serial.print("CalcTemperature = "); Serial.println(curIndoorTemp);
  Serial.println();

  // Use uint16_t (ohms * 100) since it matches data type in lookup table.
  dummy = ((uint32_t)(rtd << 1)) * 100 * ((uint32_t) floor(RREF));
  dummy >>= 16;
  ohmsx100 = (uint16_t) (dummy & 0xFFFF);

  // or use exact ohms floating point value.
  ohms = (float)(ohmsx100 / 100) + ((float)(ohmsx100 % 100) / 100.0);

  Serial.print("rtd: 0x") ; Serial.print(rtd,HEX);
  Serial.print(", ohms: ") ; Serial.println(ohms,2);

  Tlut   = PT100lt.celsius(ohmsx100) ;     // NoobNote: LUT== LookUp Table
  Tcvd   = PT100lt.celsius_cvd(ohms) ;         // Callendar-Van Dusen calc
  Tcube  = PT100lt.celsius_cubic(ohms) ;       // Cubic eqn calc
  Tpoly  = PT100lt.celsius_polynomial(ohms) ;        // 5th order polynomial
  Trpoly = PT100lt.celsius_rationalpolynomial(ohms) ;  // ugly rational polynomial quotient

  // report temperatures at 0.001C resolution to highlight methodological differences
//  Serial.print("Tlut   = ") ; Serial.print(Tlut  ,3) ; Serial.println(" C (exact)") ;
//  Serial.print("Tcvd   = ") ; Serial.print(Tcvd  ,3) ; Serial.println(" C") ;
//  Serial.print("Tcube  = ") ; Serial.print(Tcube ,3) ; Serial.println(" C") ;
//  Serial.print("Tpoly  = ") ; Serial.print(Tpoly ,3) ; Serial.println(" C") ;
//  Serial.print("Trpoly = ") ; Serial.print(Trpoly,3) ; Serial.println(" C") ;

  // report temperatures at 0.001C resolution to highlight methodological differences
  Serial.print("Tlut   = ") ; Serial.print(C2F(Tlut  )) ; Serial.println(" F (exact)") ;
  Serial.print("Tcvd   = ") ; Serial.print(C2F(Tcvd  ),3) ; Serial.println(" F") ;
  Serial.print("Tcube  = ") ; Serial.print(C2F(Tcube ),3) ; Serial.println(" F") ;
  Serial.print("Tpoly  = ") ; Serial.print(C2F(Tpoly ),3) ; Serial.println(" F") ;
  Serial.print("Trpoly = ") ; Serial.print(C2F(Trpoly),3) ; Serial.println(" F") ;

  curIndoorTemp = C2F(Tlut);

  delay(1000);
//#endif

  // Check and print any faults
  uint8_t fault = thermo.readFault();
  if (fault) {
    Serial.print("Fault 0x"); Serial.println(fault, HEX);
    if (fault & MAX31865_FAULT_HIGHTHRESH) {
      Serial.println("RTD High Threshold"); 
    }
    if (fault & MAX31865_FAULT_LOWTHRESH) {
      Serial.println("RTD Low Threshold"); 
    }
    if (fault & MAX31865_FAULT_REFINLOW) {
      Serial.println("REFIN- > 0.85 x Bias"); 
    }
    if (fault & MAX31865_FAULT_REFINHIGH) {
      Serial.println("REFIN- < 0.85 x Bias - FORCE- open"); 
    }
    if (fault & MAX31865_FAULT_RTDINLOW) {
      Serial.println("RTDIN- < 0.85 x Bias - FORCE- open"); 
    }
    if (fault & MAX31865_FAULT_OVUV) {
      Serial.println("Under/Over voltage"); 
    }
    thermo.clearFault();
    Serial.print("Temperature = "); Serial.println(curIndoorTemp);
    Serial.println();
  }
#endif

#ifdef BME688
  // Gather the temperature and other environmental values
  if (! bme.performReading()) {
    Serial.println("Failed to perform BME688 reading :(");
    return;
  }
  curIndoorTemp = (bme.temperature * 9/5) + 32;
#ifdef PRINT_TEMPS
  Serial.print("Temperature = ");
  Serial.print(curIndoorTemp);
  Serial.println(" °F");
  Serial.print("Temperature = ");
  Serial.print(bme.temperature);
  Serial.println(" °C");
/*
 // The following values will be exposed via MQTT
  Serial.print("Pressure = ");  Serial.print(bme.pressure / 100.0);  Serial.println(" hPa");
  Serial.print("Humidity = ");  Serial.print(bme.humidity);  Serial.println(" %");
  Serial.print("Gas = ");  Serial.print(bme.gas_resistance / 1.0);  Serial.println(" Ohms");
  Serial.print("Approx. Altitude = ");  Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));  Serial.println(" m");
*/
  Serial.println();
#endif
#endif
//  delay(2000);

  webserver();

  // Display the results
  if (!doSettings) {
    LayoutDisplay();
//  } else {
//    pass;
  }
  if (headerPrinted) {
//    sprintf(printbuf, "Light x=%d, y=%d, w=%d, h=%d", coordTouchLight[0], coordTouchLight[1], coordTouchLight[2], coordTouchLight[3]);
//    Serial.println(printbuf);
//    sprintf(printbuf, "System x=%d, y=%d, w=%d, h=%d", coordTouchSystem[0], coordTouchSystem[1], coordTouchSystem[2], coordTouchSystem[3]);
//    Serial.println(printbuf);
    headerPrinted = false;
  }
  // See if there's any touch data for us
  if (ts.touched())
  {   
    // Retrieve a point  
    TS_Point p = ts.getPoint(); 
    // rotate coordinate system
    // flip it around to match the screen.
    p.x = map(p.x, 0, 240, 240, 0);
    p.y = map(p.y, 0, 320, 320, 0);
    int y = tft.height() - p.x;
    int x = p.y;

    if (!doSettings) { // Buttons only active outside of Settings
      if ((x > coordTouchLight[0]) && (x < (coordTouchLight[0] + coordTouchLight[2]))) {
        if ((y > coordTouchLight[1]) && (y <= (coordTouchLight[1] + coordTouchLight[3]))) {
          Serial.print("Light button hit ");
          if (isHeatOn) {
            targIndoorTemp--;
//            tft.fillScreen(TFTbackgroundColor);
          }
          Serial.print(targIndoorTemp);
          Serial.println();
        }
      }
      if ((x > coordTouchSystem[0]) && (x < (coordTouchSystem[0] + coordTouchSystem[2]))) {
        if ((y > coordTouchSystem[1]) && (y <= (coordTouchSystem[1] + coordTouchSystem[3]))) {
//          isHeatOn = !isHeatOn;
          Serial.print("System button hit ");
//          Serial.print(isHeatOn);
          if (isHeatOn) {
            targIndoorTemp++;
//            tft.fillScreen(TFTbackgroundColor);
          }
          Serial.print(targIndoorTemp);
          Serial.println();
        }
      }
      // 3rd hidden button - Toggle whether to control temperature (hot/cold)
      if ((x > coordTouchSystem[0]) && (x < (coordTouchSystem[0] + coordTouchSystem[2]))) {
        if ((y < coordTouchSystem[1])) {
          isHeatOn = !isHeatOn;
          Serial.print("Heating status toggled ");
          Serial.print(isHeatOn);
          Serial.println();
        }
      }
    }
    // 4th hidden button - Settings
    if ((x > coordTouchLight[0]) && (x < (coordTouchLight[0] + coordTouchLight[2]))) {
      if ((y < coordTouchSystem[1])) {
        Serial.print("Settings button hit ");
        tft.fillScreen(TFTbackgroundColor);
        Serial.print(doSettings);
        doSettings = !doSettings;
        Serial.print(" ");
        Serial.print(doSettings);
        Serial.println();
      }
    }
    /*
    // Print out the remapped (rotated) coordinates
    Serial.print("Touched at point: ("); Serial.print(p.x);
    Serial.print(", "); Serial.print(p.y);
    Serial.println(")");
    */

  }
  // Begin M4 PoE FeatherWing
//#ifdef W5500
//  if (Ethernet.linkStatus() != LinkOFF) {
//    webListen();
//  }
//#endif
}

void webserver() {
  // Check for incoming connections from clients
  HardwareClient client = server.available();
  if (!client) {
    return;
  }

  // Wait for a client to send a request
  Serial.println("Client connected");
  while (!client.available()) {
    delay(1);
  }

  // Read the first line of the request
  String req = client.readStringUntil('\r');
  Serial.println(req);

  // Match the request
  if (req.indexOf("/") != -1) {  // Could be any arbitrairy location.
    // Find the first ' ' character in the request string
    int startIndex = req.indexOf("/");
    int endIndex = req.substring(startIndex).indexOf(' ') + startIndex;
    if (endIndex == -1) {
      return;
    }

    // Extract the current page and directory from the request string
    String currentPage = req.substring(startIndex, endIndex);
    if (currentPage == "/") {
      currentPage += "index.html";
    }
    Serial.println("CurrentPage: " + currentPage);

    if (currentPage == "/favicon.ico") {
      client.println("HTTP/1.1 200 OK");
      client.println("Content-Type: image/ico");
      client.println("Connection: close");
      client.println();
      //client.print((const __FlashStringHelper*) favIconHeader);  // don't use println() or you'll add extra CRLF into the binary stream
      const byte bufferSize = 70;  // 70 * 157 = 10990 (orig 48)
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
    } else {
      // Send a text response back to the client
      client.println("HTTP/1.1 200 OK");
      client.println("Content-Type: text/html");
      client.println("Connection: close");
      client.println();
    }

    // Output the HTTP response
    if (currentPage == "/index.html") {  // Main page should be first!
      sendPage(client);
    } else if (currentPage == "/dir/index.html") {
      sendPage(client);  // default to the font page
    } else {
      send404Error(client);  // page not found
    }

    delay(1);
    Serial.println("Disconnecting.");
    client.stop();
  }
}

void TFTtext(Adafruit_ILI9341 *screen, const uint16_t x, const uint16_t y, const char *str,
  const GFXfont *font = NULL, uint8_t textsize=1, uint16_t textcolor=ILI9341_BLACK)
{
  screen->setCursor(x, y);
  screen->setFont(font);
  screen->setTextSize(textsize);
  screen->setTextColor(textcolor);
  screen->print(str);
}

void LCDtemp(Adafruit_ILI9341 *screen, uint16_t x, uint16_t y, const char *str, 
  const GFXfont *font = NULL, uint8_t textsize=1, uint16_t textcolor=ILI9341_BLACK, 
  bool rightJustify=false)
{
  uint16_t xcircle, ycircle;
  int16_t  x1, y1;
  uint16_t w, h;
  const uint16_t radius=5;
  //char insidetemp[4] = "";
  //char printbuf[256] = "";

  screen->setCursor(x, y);
  screen->setFont(font);  // Proper font needs to be set to find text bounds
  screen->setTextSize(textsize);
  screen->getTextBounds(str, 0, y, &x1, &y1, &w, &h);
  xcircle = x+w+15;
  ycircle = y1 + 7;
  if (rightJustify) {
    x = coordTouchSystem[0]+40 - (w-88); // Right justify this text
    xcircle = 313;
  }
  TFTtext(screen, x, y, str, font, textsize, textcolor);
  screen->drawCircle(xcircle, ycircle, radius, textcolor);
  screen->drawCircle(xcircle, ycircle, radius-1, textcolor);
}


unsigned long LayoutDisplay() {
  //tft.fillScreen(TFTbackgroundColor);
  unsigned long start = micros();
//  int16_t  x, y;
  int16_t  x1, y1;
  uint16_t w, h;
  const uint16_t radius=5;
  char insidetemp[4] = "";
  //char printbuf[256] = "";
  uint16_t curIndoorTempColor = ILI9341_GREEN; //, targIndoorTempColor = ILI9341_GREEN;

  tft.setFont();  // Need to set the font or else wierd shifts happen to setCursor()
  // Time to turn the heat on:
  if (curIndoorTemp < targIndoorTemp) {
    isCurHeatOn = true;
    // Turn on the relay to fire up the boiler via GPIO;
  } else  // Maybe add a timer() before turning this false when curIndoorTemp == targIndoorTemp;
    isCurHeatOn = false;

  TFTtext(&tft, 5, 10, "Inside", &FreeSans9pt7b, 1, ILI9341_YELLOW);

  if (curIndoorTemp > 99.0) {
    curIndoorTemp = myround(curIndoorTemp) % 100;  // For any temp over 100 degrees, just show the last 2 digits;
    curIndoorTempColor = ILI9341_RED;
  } else if (curIndoorTemp <= 0.0 and curIndoorTemp > -100.0) {
    curIndoorTemp = abs(curIndoorTemp);  // For any negative temp greater than -100, just show the last 2 digits;
    curIndoorTempColor = ILI9341_CYAN;
  } else if (curIndoorTemp <= 0 and curIndoorTemp > -100) {
    curIndoorTemp = abs(myround(curIndoorTemp))%100;  // For any negative temp smaller than -100, just show the last 2 digits;
    curIndoorTempColor = ILI9341_PURPLE;
  } else
    curIndoorTempColor = ILI9341_GREEN;
  if (curIndoorTempOld != myround(curIndoorTemp)) {
    // Erase the old values from the screen to make way for the new values
    sprintf(insidetemp, "%02d", myround(curIndoorTempOld));
    LCDtemp(&tft, 0, 130, insidetemp, &LCD7segment72pt7b, 1, TFTbackgroundColor);
    curIndoorTempOld = myround(curIndoorTemp);
  } // Degrees = °
  sprintf(insidetemp, "%02d", myround(curIndoorTemp));
  tft.getTextBounds(insidetemp, 0, 130, &x1, &y1, &w, &h);
  LCDtemp(&tft, 0, 130, insidetemp, &LCD7segment72pt7b, 1, curIndoorTempColor);

  // Light touch area:
  coordTouchLight[0] = 15;
  coordTouchLight[1] = y1+h + 10;
  coordTouchLight[2] = 140;
  coordTouchLight[3] = tft.height()-coordTouchLight[1] + radius; // Bottom boundary line should NOT be visible!
  coordTouchSystem[0] = (coordTouchLight[0] + coordTouchLight[2] + 15);  
  coordTouchSystem[1] = coordTouchLight[1];
  coordTouchSystem[2] = coordTouchLight[2];
  coordTouchSystem[3] = coordTouchLight[3];
  tft.drawRoundRect(coordTouchLight[0], coordTouchLight[1], coordTouchLight[2], coordTouchLight[3], radius, ILI9341_YELLOW);
  tft.drawRoundRect(coordTouchSystem[0], coordTouchSystem[1], coordTouchSystem[2], coordTouchSystem[3], radius, ILI9341_YELLOW);

//  x=coordTouchLight[0] + coordTouchLight[2]/2; y=coordTouchLight[1]+18;
  TFTtext(&tft, coordTouchLight[0] + coordTouchLight[2]/2, coordTouchLight[1]+18, 
      "Light", &FreeSansBold9pt7b, 1, ILI9341_YELLOW);

  coordMinus[0] = coordTouchLight[0] + coordTouchLight[2]/2-20;
  coordMinus[1] = coordTouchLight[1]+98;
  coordPlus[0] = coordTouchSystem[0]+coordTouchLight[2]/2-40;
  coordPlus[1] = coordTouchLight[1]+93;

  tft.drawLine(coordTouchLight[0], coordTouchLight[1]+26,
      coordTouchLight[2] + coordTouchLight[0]-1, coordTouchLight[1]+26, ILI9341_YELLOW);
  tft.drawLine(coordTouchSystem[0], coordTouchSystem[1]+26,
      coordTouchSystem[2] + coordTouchSystem[0]-1, coordTouchSystem[1]+26, ILI9341_YELLOW);
  TFTtext(&tft, coordTouchSystem[0] + 5, coordTouchSystem[1]+18, 
      "System", &FreeSansBold9pt7b, 1, ILI9341_YELLOW);
  TFTtext(&tft, coordMinus[0], coordMinus[1], 
      "-", &FreeSans24pt7b, 3, ILI9341_GREEN);
  TFTtext(&tft, coordPlus[0], coordPlus[1],
      "+", &FreeSans24pt7b, 3, ILI9341_GREEN);

  // Have to blank out the text with TFTbackgroundColor when isHeatOn is false;
  TFTtext(&tft, coordTouchLight[0]+coordTouchLight[2]-5, coordTouchLight[1]-25, 
      "Heat On", &FreeSans9pt7b, 1, isHeatOn && isCurHeatOn ? ILI9341_YELLOW: TFTbackgroundColor);
  TFTtext(&tft, coordTouchSystem[0]+20, coordTouchLight[1]+46, 
      "Heat", &FreeSans9pt7b, 1, isHeatOn ? ILI9341_YELLOW: TFTbackgroundColor);
  TFTtext(&tft, coordTouchSystem[0]+40, 27, 
      "Heat", &FreeSans9pt7b, 1, isHeatOn ? ILI9341_YELLOW: TFTbackgroundColor);
  TFTtext(&tft, coordTouchSystem[0]+55, 47, 
      "Setting", &FreeSans9pt7b, 1, isHeatOn ? ILI9341_YELLOW: TFTbackgroundColor);

  if (targIndoorTempOld != targIndoorTemp) {
    // Erase the old values from the screen to make way for the new values
    sprintf(insidetemp, "%02d", targIndoorTempOld);
    LCDtemp(&tft, 0, 130, insidetemp, &LCD7segment48pt7b, 1, TFTbackgroundColor, true);
    targIndoorTempOld = targIndoorTemp;
  }
  sprintf(insidetemp, "%02d", targIndoorTemp);
  LCDtemp(&tft, 0, 130, insidetemp, &LCD7segment48pt7b, 1, isHeatOn ? ILI9341_GREEN: TFTbackgroundColor, true);


//  sprintf(printbuf, "%d, %d, %d, %d", coordTouchSystem[0], coordTouchSystem[1], coordTouchSystem[2], coordTouchSystem[3]);
//  Serial.println(printbuf); 

  return micros() - start;
}


#if !defined(__INT_MAX__) || (__INT_MAX__ > 0xFFFF)
#define pgm_read_pointer(addr) ((void *)pgm_read_dword(addr))
#else
#define pgm_read_pointer(addr) ((void *)pgm_read_word(addr))
#endif

inline GFXglyph *pgm_read_glyph_ptr(const GFXfont *gfxFont, uint8_t c) {
#ifdef __AVR__
  return &(((GFXglyph *)pgm_read_pointer(&gfxFont->glyph))[c]);
#else
  // expression in __AVR__ section may generate "dereferencing type-punned
  // pointer will break strict-aliasing rules" warning In fact, on other
  // platforms (such as STM32) there is no need to do this pointer magic as
  // program memory may be read in a usual way So expression may be simplified
  return gfxFont->glyph + c;
#endif //__AVR__
}

inline uint8_t *pgm_read_bitmap_ptr(const GFXfont *gfxFont) {
#ifdef __AVR__
  return (uint8_t *)pgm_read_pointer(&gfxFont->bitmap);
#else
  // expression in __AVR__ section generates "dereferencing type-punned pointer
  // will break strict-aliasing rules" warning In fact, on other platforms (such
  // as STM32) there is no need to do this pointer magic as program memory may
  // be read in a usual way So expression may be simplified
  return gfxFont->bitmap;
#endif //__AVR__
}

void drawGlyph(HardwareClient cl, int x, int y, const GFXfont *gfxFont, uint8_t glyph, uint8_t scaleFactor = 1) {
  glyph -= (uint8_t)pgm_read_byte(&gfxFont->first);
  GFXglyph *glyphMetadata = pgm_read_glyph_ptr(gfxFont, glyph);
  uint8_t *bitmap = pgm_read_bitmap_ptr(gfxFont);

  uint16_t bitmapOffset = pgm_read_word(&glyphMetadata->bitmapOffset);
  uint8_t width = pgm_read_byte(&glyphMetadata->width), height = pgm_read_byte(&glyphMetadata->height);
  int8_t xOffset = pgm_read_byte(&glyphMetadata->xOffset), yOffset = pgm_read_byte(&glyphMetadata->yOffset);
  uint8_t xIndex, yIndex, bits = 0, bit = 0;
  int16_t scaledXOffset = 0, scaledYOffset = 0;

  if (scaleFactor > 1) {
    scaledXOffset = xOffset;
    scaledYOffset = yOffset;
  }
  for (yIndex = 0; yIndex < height; yIndex++) {
    for (xIndex = 0; xIndex < width; xIndex++) {
      if (!(bit++ & 7)) {
        bits = pgm_read_byte(&bitmap[bitmapOffset++]);
      }
      if (bits & 0x80) {
        // Output the JavaScript code to draw a filled rectangle at the correct position
        cl.print("ctx.fillRect(");
        if (scaleFactor == 1) {
          cl.print(x + xOffset + xIndex);
          cl.print(", ");
          cl.print(y + yOffset + yIndex);
          cl.println(", 1, 1);");
        } else {
          cl.print(x + (scaledXOffset + xIndex) * scaleFactor);
          cl.print(", ");
          cl.print(y + (scaledYOffset + yIndex) * scaleFactor);
          cl.print(", ");
          cl.print(scaleFactor);
          cl.print(", ");
          cl.print(scaleFactor);
          cl.println(");");
        }
      }
      bits <<= 1;
    }
  }
  cl.flush();
}

pair canvasPrintf(HardwareClient cl, int x, int y, int canvasWidth, int canvasHeight, const GFXfont *gfxFont, uint8_t scaleFactor, const char *format, ...) {
  char buffer[100] = {0}; // Initialize buffer to all zeros
  va_list args;
  va_start(args, format);
  vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);

  for (int i = 0; i < strlen(buffer); i++) {
    if (buffer[i] != '\r' && buffer[i] != '\n') {
      uint8_t first = pgm_read_byte(&gfxFont->first);
      if ((buffer[i] >= first) && (buffer[i] <= (uint8_t)pgm_read_byte(&gfxFont->last))) {
        GFXglyph *glyphMetadata = pgm_read_glyph_ptr(gfxFont, buffer[i] - first);
        if ((pgm_read_byte(&glyphMetadata->width) > 0) && (pgm_read_byte(&glyphMetadata->height) > 0)) {
          drawGlyph(cl, x, y, gfxFont, buffer[i], scaleFactor);
          x += pgm_read_byte(&glyphMetadata->xAdvance) * scaleFactor;
        }
      }
    }
#ifdef notdef
    // Check if the next character will go off the canvas and wrap to the next line if necessary
    if (x + (pgm_read_byte(&glyphMetadata->width) * scaleFactor) > canvasWidth) {
      x = 0;
      y += (pgm_read_byte(&glyphMetadata->height)) * scaleFactor;
    }
#endif
  }
  // Return the (x, y) coordinate of where the next glyph was to be drawn
  pair nextPos;
  nextPos.first = x;
  nextPos.second = y;
  return nextPos;
}

void sendPage(HardwareClient cl) {
  int pixelsDrawn = -1;
  // Send the HTML code for the web page
  cl.println("<html>");
  cl.println("<head>");
  cl.println("<title>My Web Page</title>");
  cl.println("</head>");
  cl.println("<body>");

  // Create the canvas element where the font will be drawn
  cl.println("<canvas id='canvas' width='320' height='240' style='background-color: black;'></canvas>");

  // Send the JavaScript code that will draw the font on the canvas
  cl.println("<script>");
  cl.println("var canvas = document.getElementById('canvas');");
  cl.println("var ctx = canvas.getContext('2d');");

  // Set the fill style to white (for the font pixels)
  cl.println("ctx.fillStyle = 'yellow';");
  pair nextPos = canvasPrintf(cl, 5,15,320,240, &FreeSans9pt7b, 1, "Inside");
//  const char curIndoorTemp[] = "°";
  const char curIndoorTemp[] = "88°";
  cl.println("ctx.fillStyle = 'lime';");  // Green
//  drawGlyph(cl, 0, 130, &LCD7segment72pt7b, 0xB0, 1);
  nextPos = canvasPrintf(cl, 0,130,320,240, &LCD7segment72pt7b, 1, curIndoorTemp);
  cl.println("ctx.fillStyle = 'red';");


  Serial.print("Last X: " + String(nextPos.first));
  Serial.println(" Last Y: " + String(nextPos.second));

  cl.println("</script>");

  cl.println("</body>");
  cl.println("</html>");

  // Make sure all of the data has been sent
  cl.flush();
}

void send404Error(HardwareClient cl) {
  cl.println("<!DOCTYPE html>");
  cl.println("<html>");
  cl.println("<head>");
  cl.println("<title>404 Not Found</title>");
  cl.println("</head>");
  cl.println("<body>");
  cl.println("<h1>404 Not Found</h1>");
  cl.println("<p>The requested page was not found on this server.</p>");
  cl.println("</body>");
  cl.println("</html>");
}

#pragma GCC diagnostic pop
