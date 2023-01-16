/*
Multi BLE Sensor - Richard Hedderly 2019

Based on heart sensor code by Andreas Spiess which was based on a Neil
Kolban example.

Based on Neil Kolban example for IDF:
https://github.com/nkolban/esp32-snippets/blob/master/cpp_utils/tests/BLE%20Tests/SampleServer.cpp
Ported to Arduino ESP32 by Evandro Copercini
updates by chegewarax`
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
//#define WHEEL
#define LIBRARY_HD44780
#define PULLUP_ON_ROTATION_DETECT

const uint32_t rotationDetectPin = 23;

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
// Positions of pins may differ on your board.
// GND = Left from bottom 6, Right from top 7, Right from top 1
// 1 = GND -- Left from bottom 6
// 2 = VCC -- 5V = Left from bottom 1 (perhaps via diode?)
// 3 -- contrast pot wiper: right goes to +5V and left goes to GND
// 4 = RS -- GPIO12 = Left from bottom 7 -- 4.7k -- GND
// 5 = RW -- GND
// 6 = EN -- GPIO14 = Left from bottom 8
// 11 = D4 -- GPIO27 = Left from bottom 9
// 12 = D5 -- GPIO26 = Left from bottom 10
// 13 = D6 -- GPIO33 = Left from bottom 12
// 14 = D7 -- GPIO32 = Left from bottom 13
// 15 = GPIO25 -- Left from bottom 11
// 16 = GND
// Bike GND -- GND = Right from top 1
// Bike Detect -- GPIO23 = Right from top 2
// GND -- switch -- GPIO0 = Right from bottom 6
// GND -- switch -- GPIO4 = Right from bottom 7

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
  B00000, //.....
};

const uint32_t ledPin = 2;
const uint32_t incPin = 0;
const uint32_t decPin = 4; 
Debounce incButton(incPin, LOW);
Debounce decButton(decPin, LOW);
bool ignoreIncRelease = false;
bool ignoreDecRelease = false;
bool needToClear = true;
const uint8_t defaultRotationMarkerValue = 1; // this is what is there most of the time
uint8_t cleanRotationMarkerState = defaultRotationMarkerValue; // most likely
const uint32_t rotationMarkerDebounceTime = 20;
const uint32_t minimumUpdateTime = 250;
const uint32_t idleTime = 4000; 
bool incState = false;
bool decState = false;
bool cadenceEnabled = true;
bool powerEnabled = true;

uint32_t prevRotationMarker = 0;
uint32_t rotationMarkers = 0;
uint32_t lastRotationDuration = 0;
uint32_t lastPower = 0;
uint32_t pedalStartTime = 0;
bool detectedRotation = false;
uint32_t lastBounce = 0;

uint64_t millijoules = 0;
uint32_t pedalledTime = 0;

uint32_t lastUpdateTime = 0;

#ifdef WHEEL
const uint32_t gearRatio1 = 1; // gearRatio1:gearRatio2 = wheel rotations:crank rotations
const uint32_t gearRatio2 = 2; 
#endif

#define NUM_RESISTANCES 8
#define RADIUSX1000 149 // radius of crank in meters * 1000 (= radius of crank in mm)
#define MECHANICAL_FRICTION (9.8 * 0.8) // mechanical friction as measured
// resistance model: force = - resistanceCoeffRots * rotationsPerTime - mechanicalFriction
const uint32_t resistanceCoeffRotsX10[NUM_RESISTANCES] = {285,544,802,1052,1393,1671,1873,1969};
const uint32_t _2_pi_r_100000 = (uint32_t) (2 * PI * RADIUSX1000 * 100 + 0.5);
const uint32_t mechanicalPart1000 = (uint32_t)(2 * PI * RADIUSX1000 * MECHANICAL_FRICTION + 0.5);

byte resistanceValue = 0;
byte savedResistanceValue = 0;
byte brightnessValue = 208;
byte savedBrightnessValue = 208;
// https://github.com/sputnikdev/bluetooth-gatt-parser/blob/master/src/main/resources/gatt/characteristic/org.bluetooth.characteristic.csc_measurement.xml

struct CSCMeasurement_t {
  uint8_t flags; // bit 0: wheel data present; bit 1: crank data present
#ifdef WHEEL  
  uint32_t wheelRevolutions;
  uint16_t lastWheelEvent;
#endif  
  uint16_t crankRevolutions;
  uint16_t lastCrankEvent;
} __packed;
CSCMeasurement_t cscMeasurement = { .flags = (1<<1) // crank data
#ifdef WHEEL
  |(1<<0)
#endif
};
struct PowerMeasurement_t {
  uint16_t flags; // bit 4: wheel data present; bit 5: crank data present
  int16_t instantaneousPower;
#ifdef WHEEL  
  uint32_t wheelRevolutions;
  uint16_t lastWheelEvent;
#endif  
  uint16_t crankRevolutions;
  uint16_t lastCrankEvent;
} __packed;
PowerMeasurement_t powerMeasurement = { .flags = 
  (1<<5) // crank data
#ifdef WHEEL
  |(1<<4) // wheel data
#endif  
}; 
byte cscFeature = 2 // crank revolution
#ifdef WHEEL
 |1
#endif
; 
byte powerFeature = 8 // crank revolution
#ifdef WHEEL
 |4
#endif
;
byte powerlocation = 6; // right crank

bool bleConnected = false;
BLEServer *pServer;

#define ID(x) (BLEUUID((uint16_t)(x)))

#ifdef CADENCE
#define CADENCE_UUID ID(0x1816)
BLECharacteristic cscMeasurementCharacteristics(ID(0x2A5B), BLECharacteristic::PROPERTY_NOTIFY);
BLE2902 cscMeasurementDescriptor; //Client Characteristic Descriptor
BLECharacteristic cscFeatureCharacteristics(ID(0x2A5C), BLECharacteristic::PROPERTY_READ);
//BLEDescriptor cscFeatureDescriptor(ID(0x2901));
#endif

#ifdef POWER
#define POWER_UUID ID(0x1818)
BLECharacteristic powerMeasurementCharacteristics(ID(0x2A63), BLECharacteristic::PROPERTY_NOTIFY);
BLE2902 powerMeasurementDescriptor; // (ID(0x2902)); //Client Characteristic Descriptor
BLECharacteristic powerFeatureCharacteristics(ID(0x2A65), BLECharacteristic::PROPERTY_READ);
BLECharacteristic powerSensorLocationCharacteristics(ID(0x2A5D), BLECharacteristic::PROPERTY_READ);

//BLEDescriptor powerFeatureDescriptor(ID(0x2901));
//BLEDescriptor powerSensorLocationDescriptor(ID(0x2901));
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
    pServer->startAdvertising();
    bleConnected = false;
  }
};

void InitBLE()
{
  if (!powerEnabled && !cadenceEnabled)
    return;
  
  BLEDevice::init("OmegaCentauri BLEBike");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();

#ifdef CADENCE
  if (cadenceEnabled) {
    BLEService *pCadence = pServer->createService(CADENCE_UUID);
  
    pCadence->addCharacteristic(&cscMeasurementCharacteristics);
//    cscMeasurementDescriptor.setValue("CSC Measurement");
    cscMeasurementDescriptor.setNotifications(true);
    cscMeasurementCharacteristics.addDescriptor(&cscMeasurementDescriptor);
  
    pCadence->addCharacteristic(&cscFeatureCharacteristics);
//    cscFeatureDescriptor.setValue("CSC Feature");
//    cscFeatureCharacteristics.addDescriptor(&cscFeatureDescriptor);
    cscFeatureCharacteristics.setValue(&cscFeature, 1);
  
    pAdvertising->addServiceUUID(CADENCE_UUID);
  
    pCadence->start();
  }
#endif

#ifdef POWER
  if (powerEnabled) {
    BLEService *pPower = pServer->createService(POWER_UUID);
  
    pPower->addCharacteristic(&powerMeasurementCharacteristics);
//    powerMeasurementDescriptor.setValue("Power Measurement");
    powerMeasurementDescriptor.setNotifications(true);
    powerMeasurementCharacteristics.addDescriptor(&powerMeasurementDescriptor);
    
    pPower->addCharacteristic(&powerFeatureCharacteristics);
    powerFeatureCharacteristics.setValue(&powerFeature, 1);
    
    pPower->addCharacteristic(&powerSensorLocationCharacteristics);
    powerSensorLocationCharacteristics.setValue(&powerlocation, 1);  
  
//    powerFeatureDescriptor.setValue("Power Feature");
//    powerFeatureCharacteristics.addDescriptor(&powerFeatureDescriptor);
  
//    powerSensorLocationDescriptor.setValue("Power Sensor Location");
//    powerSensorLocationCharacteristics.addDescriptor(&powerSensorLocationDescriptor);
  
    pAdvertising->addServiceUUID(POWER_UUID);
  
    pPower->start();
  }
#endif

  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);  // functions that help with iPhone connections issue
  pAdvertising->setMinPreferred(0x12);
  pAdvertising->start();
}

void setResistance(uint32_t value) {  
    resistanceValue = value;
}

void setBrightness(uint8_t value) {
    brightnessValue = value;
    dacWrite(BACKLIGHT, brightnessValue);
}

uint32_t calculatePower(uint32_t revTimeMillis) {
  if (revTimeMillis == 0)
    return 0;
    // https://www.instructables.com/Measure-Exercise-Bike-Powercalorie-Usage/
  return (_2_pi_r_100000 * resistanceCoeffRotsX10[resistanceValue] / revTimeMillis + mechanicalPart1000 + revTimeMillis/2) / revTimeMillis;
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
  pinMode(BACKLIGHT, OUTPUT);
  dacWrite(BACKLIGHT,brightnessValue);
  lcd.begin(20,4);
  lcd.createChar(1, bluetooth);
  lcd.print("BLEBike");
  lcd.setCursor(0,1);
  lcd.print("Omega Centauri Soft");
  pinMode(incPin, INPUT_PULLUP);
  pinMode(decPin, INPUT_PULLUP);
//  if (!digitalRead(incPin))
//    cadenceEnabled = false;
  if (!digitalRead(decPin))
    powerEnabled = false;
  Serial.begin(115200);
  Serial.println("BLEBike start");
  InitBLE();
#ifdef PULLUP_ON_ROTATION_DETECT  
  pinMode(rotationDetectPin, INPUT_PULLUP); 
#else
  pinMode(rotationDetectPin, INPUT); 
#endif  
  // it would be better to trigger on FALLING or on RISING, but the debounce would be tricky,
  // and neither FALLING or RISING trigger seems reliable: https://github.com/espressif/arduino-esp32/issues/1111
  attachInterrupt(rotationDetectPin, rotationISR, CHANGE);
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, 0);
  cleanRotationMarkerState = digitalRead(rotationDetectPin);

  NVS.begin();
  setResistance(NVS.getInt("res", 0));
  savedResistanceValue = resistanceValue;
  setBrightness(NVS.getInt("bright", 208));
  savedBrightnessValue = brightnessValue;
}

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

void show(uint32_t crankRevolution,uint32_t power,uint32_t joules,uint32_t pedalledTime, uint32_t resistance, uint32_t rpm)
{
  if (needToClear) {
    lcd.clear();
    needToClear = false;
  }
  
  char t[32];
  uint32_t orgTime = pedalledTime;
  pedalledTime /= 1000;
  unsigned sec = pedalledTime % 60;
  pedalledTime /= 60;
  unsigned min = pedalledTime % 60;
  pedalledTime /= 60;
  sprintf(t, "%u:%02u:%02u",(unsigned)pedalledTime,min,sec); 

// 01234567890123456789
// xxxxW xxxrpm xxxxcal
// #xxxxx Rx B 00:00:00
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
  orgTime /= 1000;
  if (orgTime == 0 || crankRevolution < 2) {
    return;
  }
  lcd.setCursor(0,1);
  lcd.print("   avg: ");
  printdigits(4, (joules+orgTime/2) / orgTime);
  lcd.print("W ");
  printdigits(3, crankRevolution * 60 / orgTime); 
  lcd.print("rpm");
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
        changeBrightness(5);
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
        changeBrightness(-5);
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
  bool updateCadence = detectedRotation;
  uint32_t rev = rotationMarkers ? rotationMarkers-1 : 0;
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

  uint32_t lastCrankRevolution = getTime1024ths(_prevRotationMarker);

  powerMeasurement.instantaneousPower = _lastPower;
  powerMeasurement.crankRevolutions = rev;
  powerMeasurement.lastCrankEvent = lastCrankRevolution;
#ifdef WHEEL
  uint32_t wheelRevolutions = rev * gearRatio1 / gearRatio2;
  powerMeasurement.wheelRevolutions = wheelRevolutions;
  powerMeasurement.lastWheelEvent = lastCrankRevolution; // TODO: make better
#endif      

#ifdef POWER
  if (powerEnabled) {
    powerMeasurementCharacteristics.setValue((uint8_t*)&powerMeasurement, sizeof(powerMeasurement));
    powerMeasurementCharacteristics.notify();
  }
#endif  

  cscMeasurement.crankRevolutions = rev;
  cscMeasurement.lastCrankEvent = lastCrankRevolution;
#ifdef WHEEL  
  cscMeasurement.wheelRevolutions = wheelRevolutions;
  cscMeasurement.lastWheelEvent = lastCrankRevolution; // TODO: make better
#endif  

#ifdef CADENCE
  if (cadenceEnabled) {
    cscMeasurementCharacteristics.setValue((uint8_t*)&cscMeasurement, sizeof(cscMeasurement));
    cscMeasurementCharacteristics.notify();
  }
#endif  

  lastUpdateTime = t;
}

