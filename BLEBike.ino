/*
 * MIT License
*/

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h> 
#include <ArduinoNvs.h> // https://github.com/rpolitex/ArduinoNvs
#include "debounce.h"

#define POWER
#define CADENCE
//#define HEART
//#define WHEEL // Adds WHEEL to cadence; not supported in power as that would need a control point
#define LIBRARY_HD44780
//#define PULLUP_ON_ROTATION_DETECT // handy for debugging

#define DEVICE_NAME "BLEBike"
const uint32_t rotationDetectPin = 23;
const uint32_t heartSensorPin = 19;

#ifdef LIBRARY_HD44780
# include <hd44780.h>
# include <hd44780ioClass/hd44780_pinIO.h> // Arduino pin i/o class header
#else
# include <LiquidCrystal.h>
#endif

#define COLS 20
#define ROWS 4

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
unsigned curX = 0;
unsigned curY = 0;
const uint8_t defaultRotationMarkerValue = 1; // this is what is there most of the time
uint8_t cleanRotationMarkerState = defaultRotationMarkerValue; // most likely
const uint8_t defaultHeartSensorValue = 1;
uint8_t cleanHeartSensorState = defaultHeartSensorValue;
const uint32_t rotationMarkerDebounceTime = 20;
const uint32_t heartSensorDebounceTime = 2;
const uint32_t minimumUpdateTime = 250;
const uint32_t idleTime = 4000; 
const uint32_t longPressTime = 2000;
bool cadenceEnabled = true;
bool powerEnabled = true;
bool heartEnabled = true;
bool showPeleton = true;
bool updateBluetooth = false;

uint32_t prevRotationMarker = 0;
uint32_t rotationMarkers = 0;
uint32_t lastRotationDuration = 0;
uint32_t lastPower = 0;
uint32_t pedalStartTime = 0;
uint64_t millijoules = 0;
uint32_t pedalledTime = 0;
uint32_t heartBeats = 0;
uint32_t lastHeartBeatDuration = 0;
uint32_t prevHeartBeat = 0;

bool detectedRotation = false;


uint32_t lastUpdateTime = 0;

#ifdef WHEEL
const uint32_t gearRatio1 = 1; // gearRatio1:gearRatio2 = wheel rotations:crank rotations
const uint32_t gearRatio2 = 2; 
#endif

#define GROSS_EFFICIENCYx1000 195 // 195 recumbent, 206 upright, at max power output: https://doi.org/10.3389/fspor.2021.667564
#define MJ_TO_KCALx1000 239
#define NUM_RESISTANCES 8
#define RADIUSX1000 149 // radius of crank in meters * 1000 (= radius of crank in mm)

#define FV(x) (uint32_t)(2 * PI * RADIUSX1000 * (x) + 0.5)

// compile-time calculations are down with floating point, but run-time calculation are all integer
//#define MECHANICAL_FRICTION_BIKE
#ifdef MECHANICAL_FRICTION_BIKE
// resistance model: force = - mechanicalFriction
const uint32_t mechanicalPartsX1000[NUM_RESISTANCES] = 
   { FV(7.5),FV(15.5),FV(20.5),FV(35.5),FV(50.5),FV(70.4),FV(70.9),FV(90.3) };
   // replace the numbers by your mechanical friction values in Newtons
#define WATTS(rpm,i) ( mechanicalPartsX1000[(i)] / 1000. * (rpm)/60. )
#else
// resistance model: force = - resistanceCoeffRots * rotationsPerTime - mechanicalFriction
// The following are the k values from https://www.instructables.com/Measure-Exercise-Bike-Powercalorie-Usage/
// multiplied by a factor of 10.
const uint32_t resistanceCoeffRotsX10[NUM_RESISTANCES] = {285,544,802,1052,1393,1671,1873,1969}; 
const uint32_t _2_pi_r_100000 = (uint32_t) (2 * PI * RADIUSX1000 * 100 + 0.5);
const uint32_t mechanicalPartX1000 = FV( 7.84 );   // mechanical friction measured at 7.84 Newtons
#define WATTS(rpm,i) ( (2 * PI * (RADIUSX1000/1000.) * (rpm)/60. * ( resistanceCoeffRotsX10[(i)]/10.*(rpm)/60.)) + mechanicalPartX1000/1000. * (rpm)/60.)
#endif
#define PEQ(i) (uint32_t)( ( (86.6+WATTS(60,(i)))/3.98 + (183.1+WATTS(80,(i)))/7.78 + (279.1+WATTS(100,(i)))/11.73) / 3 + 0.5 )
const uint32_t peletonResistances[NUM_RESISTANCES] = {PEQ(0),PEQ(1),PEQ(2),PEQ(3),PEQ(4),PEQ(5),PEQ(6),PEQ(7)};

byte resistanceValue = 0;
byte savedResistanceValue = 0;
const byte defaultBrightnessValue = 205;
byte brightnessValue;
byte savedBrightnessValue;

// references: https://github.com/sputnikdev/bluetooth-gatt-parser/blob/master/src/main/resources/gatt/characteristic/

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
// WHEEL would require the unsupported control point  
//#ifdef WHEEL  
//  uint32_t wheelRevolutions;
//  uint16_t lastWheelEvent;
//#endif  
  uint16_t crankRevolutions;
  uint16_t lastCrankEvent;
} __packed;
PowerMeasurement_t powerMeasurement = { .flags = 
  (1<<5) // crank data
//#ifdef WHEEL
//  |(1<<4) // wheel data
//#endif  
}; 
byte cscFeature = 2 // crank revolution
#ifdef WHEEL
 |1
#endif
; 
uint32_t powerFeature = 8 // crank revolution
//#ifdef WHEEL
// |4
//#endif
;
byte powerlocation = 6; // right crank

struct HeartMeasurement8_t {
  uint8_t flags;
  uint8_t heartRate;
} __packed;

HeartMeasurement8_t heartMeasurement8 = { .flags = 0 };

struct HeartMeasurement16_t {
  uint8_t flags;
  uint16_t heartRate;
} __packed;

HeartMeasurement8_t heartMeasurement16 = { .flags = 1 };

bool bleConnected = false;

#define ID(x) (BLEUUID((uint16_t)(x)))

#ifdef CADENCE
#define CADENCE_UUID ID(0x1816)
BLECharacteristic cscMeasurementCharacteristics(ID(0x2A5B), BLECharacteristic::PROPERTY_NOTIFY);
BLE2902 cscMeasurementDescriptor; //Client Characteristic Descriptor
BLECharacteristic cscFeatureCharacteristics(ID(0x2A5C), BLECharacteristic::PROPERTY_READ);
#endif

#ifdef POWER
#define POWER_UUID ID(0x1818)
BLECharacteristic powerMeasurementCharacteristics(ID(0x2A63), BLECharacteristic::PROPERTY_NOTIFY);
BLE2902 powerMeasurementDescriptor; //Client Characteristic Descriptor
BLECharacteristic powerFeatureCharacteristics(ID(0x2A65), BLECharacteristic::PROPERTY_READ);
BLECharacteristic powerSensorLocationCharacteristics(ID(0x2A5D), BLECharacteristic::PROPERTY_READ);
#endif

#ifdef HEART
#define HEART_UUID ID(0x180D)
BLECharacteristic heartMeasurementCharacteristics(ID(0x2A37), BLECharacteristic::PROPERTY_NOTIFY);
BLE2902 heartMeasurementDescriptor; //Client Characteristic Descriptor
#endif

#define MAX_SUBOPTIONS 2

typedef struct {
  unsigned key;
  const char* heading;
  const char* subOptions[MAX_SUBOPTIONS+1];
} MenuEntry_t;

enum {
  SHOW_PELETON,
  SEND_CADENCE,
  SEND_POWER,
  SEND_HEART,
  CLEAR,
  RESUME
};

MenuEntry_t menuData[] = {
  { SHOW_PELETON, "Show Pel equiv", {"No", "Yes"} },
#ifdef POWER  
  { SEND_POWER, "Send power", {"No", "Yes" } },
#endif
#ifdef CADENCE  
  { SEND_CADENCE, "Send cadence", { "No", "Yes" } },
#endif
#ifdef HEART
  { SEND_HEART, "Send heart", { "No", "Yes" } },
#endif  
  { RESUME, "Resume", {NULL} },
  { CLEAR, "Clear data", {NULL} },
};

#define NUM_OPTIONS (sizeof menuData / sizeof *menuData)

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
    BLEDevice::startAdvertising();
    bleConnected = false;
  }
};

void InitBLE()
{
  if (!powerEnabled && !cadenceEnabled)
    return;
  
  BLEDevice::init("Omega Centauri " DEVICE_NAME);
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();

#ifdef CADENCE
  if (cadenceEnabled) {
    Serial.println("cadence data");
    BLEService *pCadence = pServer->createService(CADENCE_UUID);
  
    pCadence->addCharacteristic(&cscMeasurementCharacteristics);
    cscMeasurementCharacteristics.addDescriptor(&cscMeasurementDescriptor);
  
    pCadence->addCharacteristic(&cscFeatureCharacteristics);
    cscFeatureCharacteristics.setValue(&cscFeature, 1);
  
    pAdvertising->addServiceUUID(CADENCE_UUID);
  
    pCadence->start();
  }
#endif

#ifdef POWER
  if (powerEnabled) {
    Serial.println("power data");
    BLEService *pPower = pServer->createService(POWER_UUID);
  
    powerMeasurementCharacteristics.addDescriptor(&powerMeasurementDescriptor);
    pPower->addCharacteristic(&powerMeasurementCharacteristics);
    
    pPower->addCharacteristic(&powerFeatureCharacteristics);
    powerFeatureCharacteristics.setValue((uint8_t*)&powerFeature, sizeof(powerFeature));
    
    pPower->addCharacteristic(&powerSensorLocationCharacteristics);
    powerSensorLocationCharacteristics.setValue(&powerlocation, 1);  
  
    pAdvertising->addServiceUUID(POWER_UUID);
  
    pPower->start();
  }
#endif

#ifdef HEART
  if (heartEnabled) {
    Serial.println("heart data");
    BLEService *pHeart = pServer->createService(HEART_UUID);
  
    heartMeasurementCharacteristics.addDescriptor(&heartMeasurementDescriptor);
    pHeart->addCharacteristic(&heartMeasurementCharacteristics);
    
    pAdvertising->addServiceUUID(HEART_UUID);
  
    pHeart->start();
  }
#endif

  BLEAdvertisementData data;
  data.setManufacturerData("\xE5\x02Omega Centauri"); // use Espressif's manufacturer code
  data.setName(DEVICE_NAME);

  pAdvertising->setScanResponseData(data);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);  // functions that allegedly help with iPhone connections issue
  pAdvertising->setMaxPreferred(0x12);
  BLEDevice::startAdvertising();
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
#ifdef MECHANICAL_RESISTANCE_BIKE
  return (mechanicalPartsX1000[resistanceValue] + revTimeMillis/2) / revTimeMillis;
#else    
  return (_2_pi_r_100000 * resistanceCoeffRotsX10[resistanceValue] / revTimeMillis + mechanicalPartX1000 + revTimeMillis/2) / revTimeMillis;
#endif  
}

inline uint16_t getTime1024ths(uint32_t ms) 
{
  // TODO: there will be a glitch every 4.66 hours
  ms &= 0x00FFFFFFul;
  return ms * 128/125;
}

void IRAM_ATTR rotationISR() {
  static uint32_t lastBounce = 0;

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

void IRAM_ATTR heartISR() {
  static uint32_t lastBounce = 0;

  if (digitalRead(heartSensorPin) == cleanHeartSensorState)
    return;

  uint32_t t = millis();

  if (t < lastBounce + heartSensorDebounceTime) {
    lastBounce = t;
    return;
  }

  lastBounce = t;

  cleanHeartSensorState = ! cleanHeartSensorState;

  if (cleanHeartSensorState != defaultHeartSensorValue)
    return; 
  // trigger on end of sensor value
  // TODO: figure out if the debounce algorithm is solid
      
  if (heartBeats > 0) {
    lastHeartBeatDuration = t - prevHeartBeat;
  }
  heartBeats++;
  prevHeartBeat = t;
}

void print(const char* text) 
{
  while (*text && curX < COLS) {
    lcd.write(*text++);
    curX++;
  }
}

void clearToEOL() 
{
  while(curX < COLS) {
    lcd.write(' ');
    curX++;
  }
}

void setCursor(unsigned x, unsigned y) 
{
  lcd.setCursor(x,y);
  curX = x;
  curY = y;
}

void clear()
{
  lcd.clear();
  needToClear = false;
  setCursor(0,0);
}

void setup()
{
  pinMode(BACKLIGHT, OUTPUT);
  dacWrite(BACKLIGHT,brightnessValue);
  lcd.begin(COLS,ROWS);
  lcd.createChar(1, bluetooth);
  clear();
  print("BLEBike");
  setCursor(0,1);
  print("Omega Centauri Soft");
  pinMode(incPin, INPUT_PULLUP);
  pinMode(decPin, INPUT_PULLUP);
  Serial.begin(115200);
  Serial.println("BLEBike start");
#ifdef PULLUP_ON_ROTATION_DETECT  
  pinMode(rotationDetectPin, INPUT_PULLUP); 
#else
  pinMode(rotationDetectPin, INPUT); 
#endif  
#ifdef HEART
  pinMode(heartSensorPin, INPUT);
#endif
  // it would be better to trigger on FALLING or on RISING, but the debounce would be tricky,
  // and neither FALLING or RISING trigger seems reliable: https://github.com/espressif/arduino-esp32/issues/1111
  attachInterrupt(rotationDetectPin, rotationISR, CHANGE);
  attachInterrupt(heartSensorPin, heartISR, CHANGE);
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, 0);
  cleanRotationMarkerState = digitalRead(rotationDetectPin);

  NVS.begin();
  setResistance(NVS.getInt("res", 0));
  savedResistanceValue = resistanceValue;
  setBrightness(NVS.getInt("bright", defaultBrightnessValue));
  savedBrightnessValue = brightnessValue;
  powerEnabled = (bool)NVS.getInt("power", true);
  cadenceEnabled = (bool)NVS.getInt("cadence", true);
  showPeleton = (bool)NVS.getInt("peleton", false);
  needToClear = true;
  
  InitBLE();
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
    print(p);
    while(n--)
      print(" ");
    return;
  }
  while(n>0) {
    *--p = ' ';
    n--;
  }
  print(p);
}

unsigned getSuboption(unsigned option) {
  switch(menuData[option].key) {
    case SEND_CADENCE:
      return cadenceEnabled ? 1 : 0;
    case SEND_POWER:
      return powerEnabled ? 1 : 0;
    case SEND_HEART:
      return heartEnabled ? 1 : 0;
    case SHOW_PELETON:
      return showPeleton ? 1 : 0;
    //case CLEAR:
    default:
      return 0;
  }
}

void setSuboption(unsigned option, unsigned subOption) {
  switch(menuData[option].key) {
    case SEND_CADENCE:
      if ( cadenceEnabled != ( subOption != 0 ) ) {
        cadenceEnabled = (subOption != 0);
        NVS.setInt("cadence", (int)cadenceEnabled);
        updateBluetooth = true;
      }
      break;
    case SEND_POWER:
      if ( powerEnabled != ( subOption != 0 ) ) {
        powerEnabled = (subOption != 0);
        NVS.setInt("power", (int)powerEnabled);
        updateBluetooth = true;
      }
      break;
    case SEND_HEART:
      if ( heartEnabled != ( subOption != 0 ) ) {
        heartEnabled = (subOption != 0);
        NVS.setInt("heart", (int)heartEnabled);
        updateBluetooth = true;
      }
      break;
    case SHOW_PELETON:
      if ( showPeleton != ( subOption != 0 ) ) {
        showPeleton = (subOption != 0);
        NVS.setInt("peleton", (int)showPeleton);
      }
      break;
    case CLEAR:
      prevRotationMarker = 0;
      rotationMarkers = 0;
      lastRotationDuration = 0;
      lastPower = 0;
      pedalStartTime = 0;
      millijoules = 0;
      pedalledTime = 0;
      break;
  }
}

bool menuActive = false;

void runMenu(bool dec, bool inc, bool resetMenu) {
  static unsigned option;
  static unsigned subOption;
  static unsigned top;
  static unsigned originalSubOption;
  
  bool redraw = false;
  
  if (resetMenu) {
    option = 0;
    top = 0;
    subOption = getSuboption(0);
    originalSubOption = subOption;
    redraw = true;
  }
  if (inc) {
    if (menuData[option].subOptions[0] == NULL) {
      setSuboption(option, 0);
      menuActive = false;
      needToClear = true;
    }
    else {
      redraw = true;
      subOption++;
      if (menuData[option].subOptions[subOption] == NULL)
        subOption = 0;
    }
  }
  if (dec) {
    redraw = true;
    if (originalSubOption != subOption) {
      setSuboption(option, subOption);
      originalSubOption = subOption;
    }
    else {
      option++;
      if (option >= NUM_OPTIONS)
        option = 0;
      subOption = getSuboption(option);
      originalSubOption = subOption;
    }
  }

  if (!redraw)
    return;

  if (option < top)
    top = option;
  else if (option >= top + ROWS)
    top = option - ROWS + 1;
    
  for (unsigned i = 0 ; i < ROWS ; i++) {
    setCursor(0,i);
    unsigned o = i + top;
    if (o < NUM_OPTIONS) {
      unsigned s;
      if (o == option) {
        s = subOption;
        if (subOption != originalSubOption)
          print("*");
        else
          print(">");
      }
      else {
        s = getSuboption(o);
        print(" ");
      }
      print(menuData[o].heading);
      if (menuData[o].subOptions[0] != NULL) {
        print(": ");
        print(menuData[o].subOptions[s]);      
      }
    }
    clearToEOL();
  }
}

// returns true if menu is supposed to be showing
bool menu(DebounceEvent* decP, DebounceEvent* incP) {
  static bool exitingMenu = false;
  static bool ignoreNextDecRelease = false;
  static uint32_t decPressedTime = (uint32_t)(-1);
  bool inc;
  bool dec;
  bool resetMenu = false;

  if (incButton.getLastState())
    decPressedTime = (uint32_t)(-1);

  if (*decP == DEBOUNCE_PRESSED && !menuActive) {
    decPressedTime = millis();
  }
  else {
    if (*decP == DEBOUNCE_RELEASED)
      decPressedTime = -1;
      
    if (decPressedTime != (uint32_t)(-1) && millis() >= decPressedTime + longPressTime ) {
      ignoreNextDecRelease = true;
      if (!menuActive) {
        menuActive = true;
        resetMenu = true;
        decPressedTime = (uint32_t)(-1);
      }
    }
  }

  inc = (*incP == DEBOUNCE_RELEASED);

  if (*decP == DEBOUNCE_RELEASED) {
    if (ignoreNextDecRelease) {
      ignoreNextDecRelease = false;
      *decP = DEBOUNCE_NONE;
    }
  }

  dec = (*decP == DEBOUNCE_RELEASED);

  if (menuActive) {
    *decP = DEBOUNCE_NONE;
    *incP = DEBOUNCE_NONE;
    
    runMenu(dec,inc,resetMenu);

    return true;
  }

  if (updateBluetooth) {
    ESP.restart(); // TODO: avoid this?
  }

  return false;
}

void show(uint32_t crankRevolution,uint32_t power,uint32_t joules,uint32_t pedalledTime, uint32_t resistance, uint32_t rpm)
{
  if (needToClear) {
    clear();
  }
  
  char t[32];
  uint32_t orgTime = pedalledTime;
  pedalledTime /= 1000;
  unsigned sec = pedalledTime % 60;
  pedalledTime /= 60;
  unsigned min = pedalledTime % 60;
  pedalledTime /= 60;
  sprintf(t, "%u:%02u:%02u",(unsigned)pedalledTime,min,sec); 

  setCursor(0,0);
  printdigits(4,( joules * MJ_TO_KCALx1000 + 1000 * GROSS_EFFICIENCYx1000 / 2 ) / (1000 * GROSS_EFFICIENCYx1000) );
  print("cal ");
  printdigits(4, power);
  print("W ");
  printdigits(3, rpm);
  print("rpm ");

  if (showPeleton) {
    setCursor(0,2);
    print("R");
    printdigits(3,peletonResistances[resistance],true);
  }
  
  setCursor(0,3);
  print("L");
  printdigits(NUM_RESISTANCES<10 ? 1 : 2,resistance+1,true);
  print(bleConnected ? " \x01 " : "   ");
  print("#");
  printdigits(NUM_RESISTANCES<10 ? 6 : 5,crankRevolution,true);
  if (pedalledTime < 10)
    print(" ");
  print(t);
  orgTime /= 1000;
  if (orgTime == 0 || crankRevolution < 2) {
    return;
  }
  setCursor(0,1);
  print("   avg: ");
  printdigits(4, (joules+orgTime/2) / orgTime);
  print("W ");
  printdigits(3, crankRevolution * 60 / orgTime); 
  print("rpm");
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
  DebounceEvent incEvent = incButton.getEvent();
  DebounceEvent decEvent = decButton.getEvent();
  
  bool showedMenu = menu(&decEvent, &incEvent);
  
  switch(incEvent) {
    case DEBOUNCE_RELEASED:
      if (ignoreIncRelease)
        break;
      if (!decButton.getLastState()) {
        changeResistance(1);
      }
      else {
        changeBrightness(5);
        ignoreDecRelease = true;
      }
      break;
    case DEBOUNCE_PRESSED:
      ignoreIncRelease = false;
      break;
  }

  switch(decEvent) {
    case DEBOUNCE_RELEASED:
      if (ignoreDecRelease)
        break;
      if (!incButton.getLastState()) {
        changeResistance(-1);
      }
      else {
        changeBrightness(-5);
        ignoreIncRelease = true;
      }
      break;
    case DEBOUNCE_PRESSED:
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

  if (!showedMenu) 
    show(rev, _power, _millijoules/1000, _pedalStartTime ? t - _pedalStartTime : 0, resistanceValue, rpm);

  lastUpdateTime = t;

  if (!bleConnected || updateBluetooth)
    return;

  uint32_t lastCrankRevolution = getTime1024ths(_prevRotationMarker);

  powerMeasurement.instantaneousPower = _lastPower;
  powerMeasurement.crankRevolutions = rev;
  powerMeasurement.lastCrankEvent = lastCrankRevolution;
#ifdef WHEEL  
  uint32_t wheelRevolutions = rev * gearRatio1 / gearRatio2;
//  powerMeasurement.wheelRevolutions = wheelRevolutions;
//  powerMeasurement.lastWheelEvent = lastCrankRevolution; // TODO: make better
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

  static uint32_t lastHeartRateUpdate = 0;

#ifdef HEART
  if (heartEnabled && millis() >= lastHeartRateUpdate + 1000 && lastHeartBeatDuration > 0 && millis() <= prevHeartBeat + 4000) {
    uint16_t heartRate = (1000 + lastHeartBeatDuration/2) / lastHeartBeatDuration;
    
    if (heartRate <= 255) {
      heartMeasurement8.heartRate = heartRate;
      heartMeasurementCharacteristics.setValue((uint8_t*)&heartMeasurement8, sizeof(HeartMeasurement8_t));
    }
    else {
      heartMeasurement8.heartRate = heartRate;
      heartMeasurementCharacteristics.setValue((uint8_t*)&heartMeasurement16, sizeof(HeartMeasurement16_t));
    }
    heartMeasurementCharacteristics.notify();
    lastHeartRateUpdate = millis();
  }
#endif  

}
