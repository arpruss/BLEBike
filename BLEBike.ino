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
#define AVERAGING_ROTATIONS 0
//#define TEST

#ifndef TEST
const uint32_t rotationDetectPin = 23;
#endif

#ifdef LCD20X4

#define LCD
#ifdef LIBRARY_HD44780
# include <hd44780.h>
# include <hd44780ioClass/hd44780_pinIO.h> // Arduino pin i/o class header
#else
# include <LiquidCrystal.h>
#endif

const int rs = 12, en = 14, d4 = 27, d5 = 26, d6 = 25, d7 = 33;
// 1-16 on LCD board
// Left / Right on ESP32 with USB port at bottom
// 1 = GND = Right Top
// 2 = VCC = Left Top
// 3 = resistor to GND
// 4 = RS = GPIO12 = Left from bottom 7
// 5 = RW = GND = Right from top 7
// 6 = EN = GPIO14 = Left from bottom 8
// 11 = D4 = GPIO27 = Left from bottom 9
// 12 = D5 = GPIO26 = Left from bottom 10
// 13 = D6 = GPIO25 = Left from bottom 11
// 14 = D7 = GPIO33 = Left from bottom 12
// 15 = 5V = Left from bottom 1
// 16 = GND = Left from bottom 6
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
Debounce incButton(incPin, LOW);
const uint8_t defaultRotationValue = 1;
const uint32_t rotationDebounceTime = 100;
const uint32_t minimumUpdateTime = 250;
const uint32_t idleTime = 4000;
uint32_t lastRotationDuration = 0;
uint32_t prevPowerTime = 0;
#if AVERAGING_ROTATIONS > 0
struct WindowData {
    uint64_t millijoules;
    uint32_t time;
} windowData[AVERAGING_ROTATIONS];
uint32_t windowIndex = 0;
#endif

uint64_t millijoules = 0;
uint32_t pedalledTime = 0;

uint32_t lastReportedRotationTime = 0;
uint16_t lastCrankRevolution = 0; // in 1024ths of a second!
uint16_t crankRevolution = 0;

uint16_t prevRotationDetect = 0;
uint32_t lastRotationDetectTime = 0;
uint32_t lastUpdateTime = 0;

#define NUM_RESISTANCES 8
// resistance model: force = -resistanceCoeff * angularVelocity
const uint32_t resistanceCoeffX10[] =  {103,157,199,240,333,471,503,574};
//{ 236, 278, 301, 329, 404, 523, 553, 619 };
const uint32_t stopRampingResistanceAtRPM=60; // if you're getting RPM above this, you're breaking records or something is wrong; more likely, the latter
#define RADIUSX1000 145 // radius of crank in meters * 1000 (= radius of crank in mm)

const uint32_t flashPauseDuration = 200;
const uint32_t flashPlayDuration = 200;
const uint32_t flashDelayDuration = 2000;

char* flashPattern = NULL;
uint32_t flashStartTime = 0;

char flashPatterns[][NUM_RESISTANCES*2+2] = { "10D", "1010D", "101010D", "10101010D", "1010101010D", "101010101010D", "10101010101010D", "1010101010101010D" };

byte resistanceValue = 0;
byte savedResistanceValue = 0;
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

void setup()
{
#ifdef LCD5110  
  pinMode(BACKLIGHT, OUTPUT);
  digitalWrite(BACKLIGHT, 1);
  lcd.begin(60);
  lcd.clearDisplay();
  lcd.setCursor(1,0);
  lcd.setTextSize(2);
  lcd.print("BLE\nBike");
  lcd.display();
  lcd.setTextSize(1);
#endif
#ifdef LCD20X4
  lcd.createChar(1, bluetooth);
  lcd.begin(20,4);
#endif
  Serial.begin(115200);
  Serial.println("BLEBike start");
  InitBLE();
#ifndef TEST  
  pinMode(rotationDetectPin, INPUT_PULLUP); 
#endif  
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, 1);
  pinMode(0, INPUT);

  NVS.begin();
  setResistance(NVS.getInt("res", 0));
  savedResistanceValue = resistanceValue;
}

uint32_t calculatePower(uint32_t revTimeMillis) {
  if (revTimeMillis == 0)
    return 0;
// linear resistance:
  // angularVelocity = 2 * pi / revTime
  // distance = angularVelocity * r * dt
  // force = angularVelocity * resistanceCoeff
  // power = force * distance / dt
  //       = angularVelocity * resistanceCoeff * distance / dt
  //       = (2 * pi)^2 * resistanceCoeff * r / revTime^2 
  // power = (uint32_t)( (2 * PI) * (2 * PI) * RADIUSX1000 + 0.5) * resistanceCoeffX10[resistanceValue] / 10000 * 1000^2 / revTimeMillis^2

  return (uint32_t)( (2 * PI) * (2 * PI) * RADIUSX1000 * 100 + 0.5) * resistanceCoeffX10[resistanceValue] / revTimeMillis / max(revTimeMillis, 60000 / stopRampingResistanceAtRPM);
}

inline uint16_t getTime1024ths(uint32_t ms) 
{
  // TODO: there will be a glitch every 4.66 hours
  ms &= 0x00FFFFFFul;
  return ms * 128/125;
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
  lcd.setCursor(0,0);
  printdigits(4, power);
  lcd.print("W ");
  printdigits(3, rpm);
  lcd.print("rpm ");
  printdigits(4,joules/1000);
  lcd.print("cal#");
  printdigits(5,crankRevolution,true);
  lcd.print(" R");
  printdigits(1,resistance);
  lcd.print(bleConnected ? " \x01 " : "   ");
  if (pedalledTime < 10)
    lcd.write(' ');
  lcd.print(t); 
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
}

void checkSaveResistance() {
  static uint32_t lastSavedTime = 0;
  if (resistanceValue == savedResistanceValue)
    return;
  if ((millis() - lastSavedTime) < 4000)
    return; // reduce wear by not saving right away
  NVS.setInt("res", resistanceValue);
  savedResistanceValue = resistanceValue;
  lastSavedTime = millis();
}

void loop ()
{
  uint8_t rotationDetect;
  uint32_t ms;
  bool needUpdate;
  bool rotationDetected;
  uint32_t fromLastRotation;

  ms = millis();

#ifdef TEST
  rotationDetect = (ms % 1000) < 200;
#else  
  rotationDetect = digitalRead(rotationDetectPin) ^ defaultRotationValue;
#endif
  if (incButton.getEvent() == DEBOUNCE_PRESSED) {
    setResistance((resistanceValue + 1) % NUM_RESISTANCES);
  }
#ifdef LCD
  digitalWrite(ledPin, rotationDetect);
#else  
  flashPlay();
#endif  

  checkSaveResistance();

  uint32_t curRotationDuration = ms - lastReportedRotationTime;
  fromLastRotation = ms - lastRotationDetectTime;

  if (rotationDetect && ! prevRotationDetect && fromLastRotation >= rotationDebounceTime) {
    Serial.println("rotation detected at "+String(fromLastRotation));
    lastRotationDuration = curRotationDuration;
    if (lastReportedRotationTime>0) {
      crankRevolution++;
    }
    rotationDetected = true;
    lastReportedRotationTime = ms;
    lastCrankRevolution = getTime1024ths(ms);
    needUpdate = true;
  }
  else {
    needUpdate = false;
    rotationDetected = false;
    if (lastRotationDuration < curRotationDuration)
      lastRotationDuration = curRotationDuration;
  }
  
  if (rotationDetect)
    lastRotationDetectTime = ms;

  prevRotationDetect = rotationDetect;

  uint32_t power = crankRevolution >= 2 ? calculatePower(lastRotationDuration) : 0;

  if (prevPowerTime > 0 && crankRevolution >= 1) {
    millijoules += power * (ms - prevPowerTime);
    if (lastRotationDuration < idleTime) {
      pedalledTime += ms - prevPowerTime;
    }
  }

#if AVERAGING_ROTATIONS > 0
  if (rotationDetected) {
    struct WindowData* wd = windowData + windowIndex;
    wd->millijoules = millijoules;
    wd->time = ms;
    windowIndex = (windowIndex + 1) % AVERAGING_ROTATIONS;
  }
#endif

  prevPowerTime = ms;

  if (ms - lastUpdateTime >= minimumUpdateTime) 
    needUpdate = 1;

  if (! needUpdate)
    return;

#if AVERAGING_ROTATIONS > 0
  uint32_t rpmInWindow;
  uint32_t powerInWindow;
  
  if (crankRevolution < 2) {
    rpmInWindow = 0;
    powerInWindow = 0;
  }
  else {
    uint32_t inWindow = crankRevolution >= AVERAGING_ROTATIONS + 1 ? AVERAGING_ROTATIONS : crankRevolution - 1;
    struct WindowData* wd = windowData + (windowIndex - inWindow + AVERAGING_ROTATIONS) % AVERAGING_ROTATIONS;
    uint32_t dt = ms-wd->time;
    if (dt == 0) {
      powerInWindow = 0;
      rpmInWindow = 0;
    }
    else {
      powerInWindow = (millijoules-wd->millijoules)/dt; // factors of 1000 cancel out from the MILLIjoules and the MILLIseconds
      rpmInWindow = 60000 * inWindow / (dt - curRotationDuration + lastRotationDuration); // ??
    }
  }
  show(crankRevolution-1,powerInWindow,millijoules/1000,pedalledTime,resistanceValue+1,rpmInWindow);
#else
  show(crankRevolution-1,power,millijoules/1000,pedalledTime,resistanceValue+1,crankRevolution>=2 ? (60000/lastRotationDuration) : 0);
#endif  

  if (power > 0x7FFF)
    power = 0x7FFF;

  if (!bleConnected)
    return;

  lastUpdateTime = ms;

  cscMeasurement[1] = crankRevolution;
  cscMeasurement[2] = crankRevolution >> 8;

  cscMeasurement[3] = lastCrankRevolution;
  cscMeasurement[4] = lastCrankRevolution >> 8;

#ifdef CADENCE
  cscMeasurementCharacteristics.setValue(cscMeasurement, sizeof(cscMeasurement));
  cscMeasurementCharacteristics.notify();
#endif  

  powerMeasurement[1] = 0; 
  powerMeasurement[2] = power;
  powerMeasurement[3] = power >> 8;
  powerMeasurement[4] = crankRevolution;
  powerMeasurement[5] = crankRevolution >> 8;

#ifdef POWER
  powerMeasurementCharacteristics.setValue(powerMeasurement, sizeof(powerMeasurement));
  powerMeasurementCharacteristics.notify();
#endif  
}

