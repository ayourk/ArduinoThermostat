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

//#include <math.h>  // float/double round()ing or (int)(float+0.5)
#include <Wire.h>
#include <SPI.h>
#include "bsec.h"
#include "Adafruit_GFX.h"
#include "Adafruit_ILI9341.h"
#include <Adafruit_FT6206.h>
#include <Fonts/FreeSans9pt7b.h>
#include <Fonts/FreeSansBold9pt7b.h>
#include <Fonts/FreeSans24pt7b.h>
#include "LCD7segment48pt7b.h"  // 7 segment digital monospace
#include "LCD7segment72pt7b.h"  // 7 segment digital monospace
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"

// For the Adafruit shield, these are the default.
#define TFT_DC 9
#define TFT_CS 10

// Use hardware SPI (on Uno, #13, #12, #11) and the above for CS/DC
Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC);
// If using the breakout, change pins as desired
//Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_MOSI, TFT_CLK, TFT_RST, TFT_MISO);

// The FT6206 uses hardware I2C (SCL/SDA)
Adafruit_FT6206 ts = Adafruit_FT6206();

#define BME_SCK 13
#define BME_MISO 12
#define BME_MOSI 11
#define BME_CS 10

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME680 bme; // I2C
//Adafruit_BME680 bme(BME_CS); // hardware SPI
//Adafruit_BME680 bme(BME_CS, BME_MOSI, BME_MISO,  BME_SCK);

// Helper functions declarations
void checkIaqSensorStatus(void);
void errLeds(void);

int myround(double mydecimal) {
  return((int)(mydecimal+0.5));
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

// Create an object of the class Bsec
Bsec iaqSensor;

void setup() {
  // put your setup code here, to run once:
  char printbuf[256] = "";

  Serial.begin(9600);
  while (!Serial);

  // Begin TFT display
  Serial.println("ILI9341 Test!"); 
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

  // Begin FT6206 Touch Screen
  if (!ts.begin(40)) { 
    Serial.println("Unable to start touchscreen.");
  } 
  else { 
    Serial.println("Touchscreen started."); 
  }
  // End FT6206 Touch Screen

  // Begin Temp/Environ Sensor
  Serial.println(F("BME680 test"));

  if (!bme.begin()) {
    Serial.println("Could not find a valid BME680 sensor, check wiring!");
 //   while (1);
  } else {

    // Set up oversampling and filter initialization
    bme.setTemperatureOversampling(BME680_OS_8X);
    bme.setHumidityOversampling(BME680_OS_2X);
    bme.setPressureOversampling(BME680_OS_4X);
    bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
    bme.setGasHeater(320, 150); // 320*C for 150 ms
  }
  // End Temp/Environ Sensor
}

void loop() {
  char printbuf[256] = "";

  // Gather the temperature and other environmental values
  if (! bme.performReading()) {
    Serial.println("Failed to perform reading :(");
    return;
  }
  curIndoorTemp = (bme.temperature * 9/5) + 32;
  Serial.print("Temperature = ");
  Serial.print(curIndoorTemp);
  Serial.println(" *F");
  Serial.print("Temperature = ");
  Serial.print(bme.temperature);
  Serial.println(" *C");
/*
  Serial.print("Pressure = ");
  Serial.print(bme.pressure / 100.0);
  Serial.println(" hPa");

  Serial.print("Humidity = ");
  Serial.print(bme.humidity);
  Serial.println(" %");

  Serial.print("Gas = ");
  Serial.print(bme.gas_resistance / 1.0);
  Serial.println(" Ohms");

  Serial.print("Approx. Altitude = ");
  Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
  Serial.println(" m");
*/
  Serial.println();
//  delay(2000);

  // Display the results
  if (!doSettings) {
    LayoutDisplay();
//  } else {
    ;
  }
  if (headerPrinted) {
//    sprintf(printbuf, "Light x=%d, y=%d, w=%d, h=%d", coordTouchLight[0], coordTouchLight[1], coordTouchLight[2], coordTouchLight[3]);
//    Serial.println(printbuf);
//    sprintf(printbuf, "System x=%d, y=%d, w=%d, h=%d", coordTouchSystem[0], coordTouchSystem[1], coordTouchSystem[2], coordTouchSystem[3]);
//    Serial.println(printbuf);
    headerPrinted = false;
  }
  // See if there's any  touch data for us
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
      // 3rd hidden button - Toggle whether to control furnace
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
  char insidetemp[4] = "";
  char printbuf[256] = "";

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
  char printbuf[256] = "";
  uint16_t curIndoorTempColor = ILI9341_GREEN, targIndoorTempColor = ILI9341_GREEN;

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
    sprintf(insidetemp, "%-02d", targIndoorTempOld);
    LCDtemp(&tft, 0, 130, insidetemp, &LCD7segment48pt7b, 1, TFTbackgroundColor, true);
    targIndoorTempOld = targIndoorTemp;
  }
  sprintf(insidetemp, "%-02d", targIndoorTemp);
  LCDtemp(&tft, 0, 130, insidetemp, &LCD7segment48pt7b, 1, isHeatOn ? ILI9341_GREEN: TFTbackgroundColor, true);


//  sprintf(printbuf, "%d, %d, %d, %d", coordTouchSystem[0], coordTouchSystem[1], coordTouchSystem[2], coordTouchSystem[3]);
//  Serial.println(printbuf); 

  return micros() - start;
}
