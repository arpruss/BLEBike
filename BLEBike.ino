/*
Multi BLE Sensor - Richard Hedderly 2019

Based on heart sensor code by Andreas Spiess which was based on a Neil
Kolban example.

Based on Neil Kolban example for IDF:
https://github.com/nkolban/esp32-snippets/blob/master/cpp_utils/tests/BLE%20Tests/SampleServer.cpp
Ported to Arduino ESP32 by Evandro Copercini
updates by chegewara
heavily modified by Alexander Pruss
*/

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h> 
#include <ArduinoNvs.h>
#include "debounce.h"

#define POWER
#define CADENCE
//#define LCD5110
#define LCD20X4
#define LIBRARY_HD44780
//#define TEST

const uint32_t rotationDetectPin = 23;

#ifdef LCD20X4
#define LCD

#ifdef LIBRARY_HD44780
# include <hd44780.h>
# include <hd44780ioClass/hd44780_pinIO.h> // Arduino pin i/o class header
#else
# include <LiquidCrystal.h>
#endif

const int rs = 12, en = 14, d4 = 27, d5 = 26, d6 = 33, d7 = 32;
#define BACKLIGHT 25
// 1-16 on LCD board
// Left / Right on ESP32 with USB port at bottom
// 1 = GND -- Left from bottom 6
// 2 = VCC -- 5V = Left from bottom 1 (perhaps via diode?)
// 3 -- contrast pot wiper (right goes to + and left goes to GND)
// 4 = RS -- GPIO12 = Left from bottom 7, with pulldown resistor to GND (4.7k works)
// 5 = RW -- GND = Right from top 7
// 6 = EN -- GPIO14 = Left from bottom 8
// 11 = D4 -- GPIO27 = Left from bottom 9
// 12 = D5 -- GPIO26 = Left from bottom 10
// 13 = D6 -- GPIO33 = Left from bottom 12
// 14 = D7 -- GPIO32 = Left from bottom 13
// 15 = GPIO25 -- Left from bottom 11
// 16 = GND -- Left from bottom 6
// Bike GND -- GND = Right from top 1
// Bike Detect -- GPIO23 = Right from top 2
// GND - switch - GPIO0 = Right from bottom 6
// GND - switch - GPIO4 = Right from bottom 7

#ifdef LIBRARY_HD44780
hd44780_pinIO
#else
LiquidCrystal 
#endif
lcd(rs, en, d4, d5, d6, d7);

byte bluetooth[8] = {
  B00110, //..XX.
  B10101, //X.X.X
  B01110, //.XXX.
  B00100, //..X..
  B01110, //.XXX.
  B10101, //X.X.X
  B00110, //..XX.
};
#endif

#ifdef LCD5110
#include <Adafruit_GFX.h>
#include <Adafruit_PCD8544.h>

#define BACKLIGHT 25
#define LCD
// int8_t SCLK, int8_t DIN, int8_t DC, int8_t CS, int8_t RST
static Adafruit_PCD8544 lcd(14,13,27,15,26); 
#endif

const uint32_t ledPin = 2;
const uint32_t incPin = 0;
const uint32_t decPin = 4; 
Debounce incButton(incPin, LOW);
Debounce decButton(decPin, LOW);
bool ignoreIncRelease = false;
bool ignoreDecRelease = false;
const uint8_t defaultRotationMarkerValue = 1; // this is what is there most of the time
uint8_t cleanRotationMarkerState = defaultRotationMarkerValue; // most likely
const uint32_t rotationMarkerDebounceTime = 20;
const uint32_t minimumUpdateTime = 250;
const uint32_t idleTime = 4000; 
bool incState = false;
bool decState = false;

uint32_t prevRotationMarker = 0;
uint32_t rotationMarkers = 0;
uint32_t lastRotationDuration = 0;
uint32_t lastPower = 0;
uint32_t pedalStartTime = 0;
bool detectedRotation = false;
uint32_t lastBounce = 0;

uint64_t millijoules = 0;
uint32_t pedalledTime = 0;

uint32_t lastReportedRotationTime = 0;
uint16_t lastCrankRevolution = 0; // in 1024ths of a second!
uint16_t crankRevolution = 0;

uint32_t lastUpdateTime = 0;

#define NUM_RESISTANCES 8
// resistance model: force = - resistanceCoeffRots * rotationsPerTime - mechanicalFriction
const uint32_t resistanceCoeffRotsX10[NUM_RESISTANCES] = {441,733,1036,1344,1726,2050,2264,2433};
const uint32_t mechanicalFrictionX10 = 78;

#define RADIUSX1000 145 // radius of crank in meters * 1000 (= radius of crank in mm)

const uint32_t flashPauseDuration = 200;
const uint32_t flashPlayDuration = 200;
const uint32_t flashDelayDuration = 2000;

char* flashPattern = NULL;
uint32_t flashStartTime = 0;

char flashPatterns[][NUM_RESISTANCES*2+2] = { "10D", "1010D", "101010D", "10101010D", "1010101010D", "101010101010D", "10101010101010D", "1010101010101010D" };

byte resistanceValue = 0;
byte savedResistanceValue = 0;
byte brightnessValue = 208;
byte savedBrightnessValue = 208;
byte cscMeasurement[5] = { 2 };
byte powerMeasurement[6] = { 0x20 }; // include crank revolution data
byte cscFeature = 2;
byte powerFeature = 8; // crank revolution
byte powerlocation = 6; // right crank

bool bleConnected = false;

#define ID(x) (BLEUUID((uint16_t)(x)))

#ifdef CADENCE
#define CADENCE_UUID ID(0x1816)
BLECharacteristic cscMeasurementCharacteristics(ID(0x2A5B), BLECharacteristic::PROPERTY_NOTIFY);
BLECharacteristic cscFeatureCharacteristics(ID(0x2A5C), BLECharacteristic::PROPERTY_READ);

BLEDescriptor cscFeatureDescriptor(ID(0x2901));
#endif

#ifdef POWER
#define POWER_UUID ID(0x1818)
BLECharacteristic powerMeasurementCharacteristics(ID(0x2A63), BLECharacteristic::PROPERTY_NOTIFY);
BLECharacteristic powerFeatureCharacteristics(ID(0x2A65), BLECharacteristic::PROPERTY_READ);
BLECharacteristic powerSensorLocationCharacteristics(ID(0x2A5D), BLECharacteristic::PROPERTY_READ);

BLEDescriptor powerFeatureDescriptor(ID(0x2901));
BLEDescriptor powerSensorLocationDescriptor(ID(0x2901));
#endif

class MyServerCallbacks:public BLEServerCallbacks
{
  void onConnect(BLEServer* pServer)
  {
    Serial.println("connected");
    bleConnected = true;
  };

  void onDisconnect(BLEServer* pServer)
  {
    Serial.println("disconnected");
    bleConnected = false;
  }
};

void InitBLE()
{
  BLEDevice::init("Exercise Bike Sensor");
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();

#ifdef CADENCE
  BLEService *pCadence = pServer->createService(CADENCE_UUID);

  pCadence->addCharacteristic(&cscMeasurementCharacteristics);
  cscMeasurementCharacteristics.addDescriptor(new BLE2902());

  pCadence->addCharacteristic(&cscFeatureCharacteristics);
  cscFeatureDescriptor.setValue("CSC Feature");
  cscFeatureCharacteristics.addDescriptor(&cscFeatureDescriptor);
  cscFeatureCharacteristics.setValue(&cscFeature, 1);

  pAdvertising->addServiceUUID(CADENCE_UUID);

  pCadence->start();
#endif

#ifdef POWER
  BLEService *pPower = pServer->createService(POWER_UUID);

  pPower->addCharacteristic(&powerMeasurementCharacteristics);
  powerMeasurementCharacteristics.addDescriptor(new BLE2902());
  
  pPower->addCharacteristic(&powerFeatureCharacteristics);
  powerFeatureCharacteristics.setValue(&powerFeature, 1);
  
  pPower->addCharacteristic(&powerSensorLocationCharacteristics);
  powerSensorLocationCharacteristics.setValue(&powerlocation, 1);  

  powerFeatureDescriptor.setValue("Power Feature");
  powerFeatureCharacteristics.addDescriptor(&powerFeatureDescriptor);

  powerSensorLocationDescriptor.setValue("Power Sensor Location");
  powerSensorLocationCharacteristics.addDescriptor(&powerSensorLocationDescriptor);

  pAdvertising->addServiceUUID(POWER_UUID);

  pPower->start();
#endif

  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);  // functions that help with iPhone connections issue
  pAdvertising->setMinPreferred(0x12);
  pAdvertising->start();
}

void setResistance(uint32_t value) {  
    resistanceValue = value;
    flashPattern = flashPatterns[value];
    flashStartTime = millis();
}

void setBrightness(uint8_t value) {
    brightnessValue = value;
    dacWrite(BACKLIGHT, brightnessValue);
}

void flashPlay() {
  if (flashPattern == NULL) {
    digitalWrite(ledPin, 0);
    return;
  }
  char* p = flashPattern;
  uint32_t currentTime = millis() - flashStartTime;
  bool active = false;
  uint32_t duration = 0;
  uint32_t t = 0;
  while(*p) {
    switch(*p) {
      case '0':
        active = false;
        duration = flashPauseDuration;
        break;
      case '1':
        active = true;
        duration = flashPlayDuration;
        break;
      case 'D':
        active = false;
        duration = flashDelayDuration;
        break;
      default:
        duration = 0;
        active = false;
    }
    if (currentTime < t + duration) {
      digitalWrite(ledPin, active);
      return;
    }
    t += duration;
    p++;
  }  
  flashStartTime = millis(); // rewind
}

uint32_t calculatePower(uint32_t revTimeMillis) {
  if (revTimeMillis == 0)
    return 0;
    // https://www.instructables.com/Measure-Exercise-Bike-Powercalorie-Usage/
  return (uint32_t) ((2 * PI) * RADIUSX1000 * 100 + 0.5) * resistanceCoeffRotsX10[resistanceValue] / revTimeMillis / revTimeMillis +
         (uint32_t) ((2 * PI) * RADIUSX1000 + 0.5) * mechanicalFrictionX10 / 10000;
}

inline uint16_t getTime1024ths(uint32_t ms) 
{
  // TODO: there will be a glitch every 4.66 hours
  ms &= 0x00FFFFFFul;
  return ms * 128/125;
}




void IRAM_ATTR rotationISR() {
  if (digitalRead(rotationDetectPin) == cleanRotationMarkerState)
    return;

  uint32_t t = millis();

  if (t < lastBounce + rotationMarkerDebounceTime) {
    lastBounce = t;
    return;
  }

  lastBounce = t;

  cleanRotationMarkerState = ! cleanRotationMarkerState;

  if (cleanRotationMarkerState != defaultRotationMarkerValue)
    return; 
  // trigger on end of marker
  // TODO: figure out if the debounce algorithm is solid
      
  if (rotationMarkers > 0) {
    lastRotationDuration = t - prevRotationMarker;
    lastPower = calculatePower(lastRotationDuration);
    millijoules += lastPower * lastRotationDuration;
  }
  else {
    pedalStartTime = t;
  }
  rotationMarkers++;
  prevRotationMarker = t;
  detectedRotation = true;
}

void setup()
{
#ifdef LCD5110  
  pinMode(BACKLIGHT, OUTPUT);
  digitalWrite(BACKLIGHT, 0);
  lcd.begin(60);
  lcd.clearDisplay();
  lcd.setCursor(1,0);
  lcd.setTextSize(2);
  lcd.print("BLE\nBike");
  lcd.display();
  lcd.setTextSize(1);
#endif
#ifdef LCD20X4
//  pinMode(BACKLIGHT, OUTPUT);
//  digitalWrite(BACKLIGHT, 1);
//  ledcSetup(0,5000,8);
//  ledcAttachPin(BACKLIGHT,0);
//  ledcWrite(64,0);
  dacWrite(BACKLIGHT,brightnessValue);
  lcd.createChar(1, bluetooth);
  lcd.begin(20,4);
  lcd.print("BLEBike");
#endif
  pinMode(incPin, INPUT_PULLUP);
  pinMode(decPin, INPUT_PULLUP);
  Serial.begin(115200);
  Serial.println("BLEBike start");
  InitBLE();
#ifndef TEST  
  pinMode(rotationDetectPin, INPUT_PULLUP); 
  // it would be better to trigger on FALLING or on RISING, but the debounce would be tricky,
  // and neither FALLING or RISING trigger seems reliable: https://github.com/espressif/arduino-esp32/issues/1111
  attachInterrupt(rotationDetectPin, rotationISR, CHANGE);
#endif  
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, 0);
  pinMode(0, INPUT);
  cleanRotationMarkerState = digitalRead(rotationDetectPin);

  NVS.begin();
  setResistance(NVS.getInt("res", 0));
  savedResistanceValue = resistanceValue;
#ifdef LCD20X4
  setBrightness(NVS.getInt("bright", 208));
  savedBrightnessValue = brightnessValue;
#endif  
}

#ifdef LCD20X4
void printdigits(unsigned n, unsigned x, bool leftAlign=false) {
  const unsigned maxDigits = 10;
  char buffer[maxDigits+1];
  if (n>maxDigits)
    n=maxDigits;
  if (n==0)
    return;
  char*p = buffer+maxDigits;
  *p = 0;
  do {
    *--p = '0' + (x % 10);
    x /= 10;
    n--;
  } while(n>0 && x);
  if (x) {
    char *q = p;
    while(*q) {
      *q++ = '#';
    }
  }
  if (leftAlign) {
    lcd.print(p);
    while(n--)
      lcd.write(' ');
    return;
  }
  while(n>0) {
    *--p = ' ';
    n--;
  }
  lcd.print(p);
}
#endif

void show(uint32_t crankRevolution,uint32_t power,uint32_t joules,uint32_t pedalledTime, uint32_t resistance, uint32_t rpm)
{
#ifdef LCD
  char t[32];
  uint32_t orgTime = pedalledTime;
  pedalledTime /= 1000;
  unsigned sec = pedalledTime % 60;
  pedalledTime /= 60;
  unsigned min = pedalledTime % 60;
  pedalledTime /= 60;
  sprintf(t, "%u:%02u:%02u",(unsigned)pedalledTime,min,sec); 

#ifdef LCD20X4
// 01234567890123456789
// xxxxW xxxrpm xxxxcal
// #xxxxx Rx B 00:00:00
  //lcd.begin(20,(millis()%2000)<1000?1:4);
  lcd.home();
  printdigits(4,joules/1000);
  lcd.print("cal ");
  printdigits(4, power);
  lcd.print("W ");
  printdigits(3, rpm);
  lcd.print("rpm ");
  lcd.setCursor(0,3);
  lcd.print("#");
  printdigits(5,crankRevolution,true);
  lcd.print(" R");
  printdigits(1,resistance);
  lcd.print(bleConnected ? " \x01 " : "   ");
  if (pedalledTime < 10)
    lcd.write(' ');
  lcd.print(t);
  //lcd.clearToEndOfLine(); 
  orgTime /= 1000;
  if (orgTime == 0) {
    //lcd.setCursor(0,2);
    //lcd.clearToEndOfLine();
    return;
  }
  lcd.setCursor(0,1);
  lcd.print("   avg: ");
  printdigits(4, (joules+orgTime/2) / orgTime);
  lcd.print("W ");
  printdigits(3, crankRevolution * 60 / orgTime); 
  lcd.print("rpm");
  //lcd.clearToEndOfLine();
/*  lcd.setCursor(1,0);
  lcd.print("l1");
  lcd.setCursor(3,0);
  lcd.print("l3"); */
#endif

#ifdef LCD5110   
  static bool initialized = false;

  if (! initialized) {
    lcd.setCursor(0,0);
    lcd.println("Rev");
    lcd.println("Power");
    lcd.println("KCal");
    lcd.println("Time");
    lcd.println("RPM");
    lcd.println("Resis");
  }
  uint32_t y = 0;
  const uint32_t x = 40-6;
  lcd.setCursor(x,y);
  lcd.print(crankRevolution);
  lcd.setCursor(x,y+=8);
  lcd.print(String(power)+"W");
  lcd.setCursor(x,y+=8);
  lcd.println(String(joules/1000));
  lcd.setCursor(x,y+=8);
  lcd.println(t);
  lcd.setCursor(x,y+=8);
  lcd.println(String(rpm)+"    ");
  lcd.setCursor(x,y+=8);
  lcd.println(String(resistance)+" ");
  lcd.display();
#endif  
#endif
lcd.home();
}

void checkSave() {
  static uint32_t lastSavedTime = 0;
  if (resistanceValue == savedResistanceValue && brightnessValue == savedBrightnessValue)
    return;
  if ((millis() - lastSavedTime) < 4000)
    return; // reduce wear by not saving right away
  if (resistanceValue != savedResistanceValue) {
    NVS.setInt("res", resistanceValue);
    savedResistanceValue = resistanceValue;
  }
  if (brightnessValue != savedBrightnessValue) {
    NVS.setInt("bright", brightnessValue);
    savedBrightnessValue = brightnessValue;
  }
  lastSavedTime = millis();
}

void changeResistance(int32_t delta) {
  setResistance((resistanceValue + delta + NUM_RESISTANCES) % NUM_RESISTANCES);
}

void changeBrightness(int32_t delta) {
  int32_t nb = (int32_t)brightnessValue + delta;
  if (nb < 0)
    setBrightness(0);
  else if (nb > 255)
    setBrightness(255);
  else
    setBrightness(nb);
}

void loop ()
{
  switch(incButton.getEvent()) {
    case DEBOUNCE_RELEASED:
      incState = false;
      if (ignoreIncRelease)
        break;
      if (!decState) {
        changeResistance(1);
      }
      else {
        changeBrightness(8);
        ignoreDecRelease = true;
      }
      break;
    case DEBOUNCE_PRESSED:
      incState = true;
      ignoreIncRelease = false;
      break;
  }

  switch(decButton.getEvent()) {
    case DEBOUNCE_RELEASED:
      decState = false;
      if (ignoreDecRelease)
        break;
      if (!incState) {
        changeResistance(-1);
      }
      else {
        changeBrightness(-8);
        ignoreIncRelease = true;
      }
      break;
    case DEBOUNCE_PRESSED:
      decState = true;
      ignoreDecRelease = false;
      break;
  }

  checkSave();

  uint32_t t = millis();

  if (! detectedRotation && lastUpdateTime && t < lastUpdateTime + minimumUpdateTime)
    return;

  noInterrupts();
  uint32_t rev = rotationMarkers-1;
  uint32_t _lastPower = lastPower;
  uint32_t _millijoules = millijoules;
  uint32_t _prevRotationMarker = prevRotationMarker;
  if (_prevRotationMarker + idleTime <= t && rev && !detectedRotation) {
      pedalStartTime += t - lastUpdateTime;
  }
  detectedRotation = false;
  uint32_t _pedalStartTime = pedalStartTime;
  uint32_t _lastRotationDuration = lastRotationDuration;
  interrupts();

  uint32_t rpm;
  uint32_t _power = _lastPower;
  if (!rev || _lastRotationDuration == 0)
    rpm = 0;
  else if (_lastRotationDuration < t - prevRotationMarker) {
    rpm = 60000 / (t - prevRotationMarker);
    _power = calculatePower(t - prevRotationMarker);
  }
  else 
    rpm = 60000 / _lastRotationDuration;

  show(rev, _power, _millijoules/1000, _pedalStartTime ? t - _pedalStartTime : 0, resistanceValue+1, rpm);

  cscMeasurement[1] = rev;
  cscMeasurement[2] = rev >> 8;
  
  uint32_t lastCrankRevolution = getTime1024ths(_prevRotationMarker);

  cscMeasurement[3] = lastCrankRevolution;
  cscMeasurement[4] = lastCrankRevolution >> 8;

#ifdef CADENCE
  cscMeasurementCharacteristics.setValue(cscMeasurement, sizeof(cscMeasurement));
  cscMeasurementCharacteristics.notify();
#endif  

  powerMeasurement[1] = 0; 
  powerMeasurement[2] = _lastPower;
  powerMeasurement[3] = _lastPower >> 8;
  powerMeasurement[4] = rev;
  powerMeasurement[5] = rev >> 8;

#ifdef POWER
  powerMeasurementCharacteristics.setValue(powerMeasurement, sizeof(powerMeasurement));
  powerMeasurementCharacteristics.notify();
#endif  

  lastUpdateTime = t;
}

