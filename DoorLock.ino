///// TEST #defines
//#define TEST
#ifdef TEST
  #define TEST_SEQUENCE
  #ifndef TEST_SEQUENCE
    #define TEST_SERVO
    #ifndef TEST_SERVO
      #define TEST_SERVO_LINEAR
    #endif
  #endif
#endif

#define RING_ENABLED
//#define BUZZER_ENABLED
//#define IR_ENABLED
//#define LCD_ENABLED
#if !defined(TEST_SERVO) && !defined(TEST_SERVO_LINEAR) // only define BUTTON_ENABLED only if TEST_SERVO_LINEAR is not defined so as not to interrupt the tests.
#define BUTTON_ENABLED
#endif
#define SERVO_ENABLED
#define SERVO_LINEAR_ENABLED
#define RFID_ENABLED
#define BLE_ENABLED

////////////////////////////////////////////// All required libraries
#ifdef RFID_ENABLED
  #include <EEPROM.h>     // We are going to read and write PICC's UIDs from/to EEPROM
#endif

#ifdef BLE_ENABLED
#include <SoftwareSerial.h>
#endif

#ifdef BUZZER_ENABLED
  #include <TimerFreeTone.h>
#endif

#ifdef RING_ENABLED
  #include <Adafruit_NeoPixel.h>
#endif

#ifdef IR_ENABLED
  #include <IRremote.h>
#endif

#if defined(SERVO_ENABLED) || defined(SERVO_LINEAR_ENABLED)
  #include <Servo.h>
  #include <Wire.h>
#endif

#ifdef LCD_ENABLED
  #include <LiquidCrystal_I2C.h>
#endif
////////////////////////////////////////////// End required libraries

unsigned long currTime;

#define LOCKED 1
#define UNLOCKED 0
#define UNKNOWN -1

#define SEQUENCE_IDLE -1
#define SEQUENCE_INIT 0
#define SEQUENCE_RESET_ARM 1
#define SEQUENCE_ENGAGE 2
#define SEQUENCE_ACTION 3
#define SEQUENCE_DISENGAGE 4
#define SEQUENCE_END SEQUENCE_DISENGAGE+1
const int SEQUENCE_WAIT[] = {500, 300, 500, 1000, 500, 100};
unsigned long nextSequenceTimeStart = 0;
int8_t currSequenceStage = SEQUENCE_IDLE;
int8_t nextSequenceStage = SEQUENCE_IDLE;

uint8_t currState = UNKNOWN;
uint8_t intentState = UNKNOWN;

#ifdef BUZZER_ENABLED
  #define HIGH_SOUND          3001
  #define LOW_SOUND           1000
  #define SOUND_STEP          500
  #define SOUND_MS            20
  uint8_t soundToneTarget         = 0;
  uint8_t soundToneCurr           = 0;
  unsigned long soundToneLastTime;
#endif

//PWM
//3, 5, 6, 9, 10, and 11 
#define PIN_SERVO1            2
#ifdef RING_ENABLED
#define PIN_RING_LED          6
#endif
#define PIN_SERVO_LINEAR1     4
#define PIN_TILT              13
#define PIN_LOCK_BUTTON       3
#ifdef BUZZER_ENABLED
#define PIN_BUZZER            7
#endif
#ifdef IR_ENABLED
#define PIN_IR_RECEIVER1      8
#endif
#ifdef BLE_ENABLED
const uint8_t PIN_BLE_RXD                           = 10;
const uint8_t PIN_BLE_TXD                           = 11;
#endif

#define PRESSED_COOLDOWN_MS 5000
unsigned long pressedCooldownLastTime;
bool buttonCooldown = false;
#ifdef BUTTON_ENABLED
  #define BUTTON_MS           100
  unsigned long buttonLastTime;
#endif

#ifdef TEST_SEQUENCE
  int testSequenceIntent = UNKNOWN;
  int testSequenceCount = -1;
  unsigned long testSequenceNextTime = 0;
#endif

#ifdef SERVO_ENABLED
  // Main Arm Servo
  #define SERVO_LOCKED_DEG    30
  #define SERVO_UNLOCKED_DEG  155
  #define SERVO_STEP          2
  #define SERVO_MS            10
  Servo servoArm1;
  uint16_t servoArmTarget = SERVO_UNLOCKED_DEG;
  uint16_t servoArmCurr = SERVO_UNLOCKED_DEG;
  unsigned long servoLastTime = 0;
  
  // Servo tests
  #ifdef TEST_SERVO
  uint32_t servoTestCount = -1; //For keeping track of how many times the servo has been moved/tested. For battery capacity testing.
  #endif
#endif

#ifdef SERVO_LINEAR_ENABLED
  // Linear Servo
  #define SERVO_LINEAR_ENGAGED_DEG            68
  #define SERVO_LINEAR_DISENGAGED_DEG         32 //25
  #define SERVO_LINEAR_STEP                   2
  #define SERVO_LINEAR_MS                     10
  Servo servoLinearArm1;
  uint16_t servoLinearArmTarget               = SERVO_LINEAR_DISENGAGED_DEG;
  uint16_t servoLinearArmCurr                 = SERVO_LINEAR_DISENGAGED_DEG;
  unsigned long servoLinearLastTime           = 0;
  
  // Linear Servo tests
  #ifdef TEST_SERVO_LINEAR
  int servoLinearTestCount = -1; //For keeping track of how many times the servo has been moved/tested. For battery capacity testing.
  #endif
#endif

#ifdef LCD_ENABLED
  LiquidCrystal_I2C lcd(0x27, 16, 2);

  #define LCD_MS 100
  unsigned long lcdLastTime;
  int lcdState = UNKNOWN;
#endif

#ifdef IR_ENABLED
  IRrecv irrecv1(PIN_IR_RECEIVER1);
  decode_results irResults1;
  #define IR_LOCK_CODE 0xFFE21D
  #define IR_UNLOCK_CODE 0xFFA25D
  #define IR_MS 10
  unsigned long irLastTime;
  unsigned long lastIRValue;
#endif

#define RECONCILE_MS 10
#define RECONCILE_CERTAINTY 3
unsigned long reconcileLastTime;
int reconcileCertainty = 0;

#ifdef RING_ENABLED
  #define RING_NUMPIXELS                  24 // Popular NeoPixel ring size
  #define RING_SLEEP_TIMEOUT_MS           5000
  Adafruit_NeoPixel pixels(RING_NUMPIXELS, PIN_RING_LED, NEO_GRB + NEO_KHZ800);
  #define RING_MS                         200
  #define RING_STEP                       50
  #define RING_BRIGHTNESS_LOW             0
  #define RING_BRIGHTNESS_SLEEP           0
  #define RING_PULSE_HIGH                 30
  #define RING_PULSE_LOW                  5
  #define RING_BRIGHTNESS_HIGH            255
  unsigned long ringLastTime;
  unsigned long ringSleepTime;
  int ringBrightnessCurr                  = RING_BRIGHTNESS_LOW;
  int ringBrightnessTarget                = RING_BRIGHTNESS_HIGH;
  const int RING_LOCK_COLOR[]             = {200, 0, 0};
  const int RING_UNLOCK_COLOR[]           = {0, 0, 200};
  int* ringCurrColor                      = RING_LOCK_COLOR;
#endif

#ifdef RFID_ENABLED
  #define RFID_MS                               250
  unsigned long rfidLastTime                    = 0;
  #define RFID_EEPROM_MASTER_CARD_DEFINED       1
  #define RFID_EEPROM_MASTER_CARD_OFFSET        RFID_EEPROM_MASTER_CARD_DEFINED + 1
  #define RFID_EEPROM_SLAVE_CARD_NEXT_FREE_SLOT 6
  #define RFID_EEPROM_SLAVE_CARD_START          7     // Each slot is 4 bytes data
  #define RFID_EEPROM_SLAVE_CARD_SLOT_OFFSET    4     // Offset between each slot
  #define RFID_EEPROM_SLAVE_CARD_SLOTS          8
  
  #define MASTER_CARD_DEFINED_CODE              0x8F
  uint8_t cardAttemptsCount                     = 0;
  #define CARD_ATTEMPT_MAX_COUNT                3
  byte cardCandidates[3][4];
  unsigned long registerSlaveCardTimeout        = 0;
  #define RFID_SLAVE_CARD_REGISTRATION_TIMEOUT  10000
  
  #define RFID_MODE_IDLE                        0
  #define RFID_MODE_REGISTERING_MASTER          1
  #define RFID_MODE_REGISTERING_SLAVE           2
  uint8_t rfidMode                              = RFID_MODE_IDLE;
    
  byte readCard[4];     // Stores scanned ID read from RFID Module
  byte masterCard[4];   // Stores master card's ID read from EEPROM
  
  const uint8_t ZEROS[] = {0x0, 0x0, 0x0, 0x0};
  const uint8_t ONES[] = {0xFF, 0xFF, 0xFF, 0xFF};
#endif

#ifdef BLE_ENABLED
SoftwareSerial btSerial(PIN_BLE_RXD, PIN_BLE_TXD); // RX | TX
#endif

#ifdef BUZZER_ENABLED
void lockSound() {
  soundToneCurr = HIGH_SOUND;
  soundToneTarget = LOW_SOUND;
}

void unlockSound() {
  soundToneCurr = LOW_SOUND;
  soundToneTarget = HIGH_SOUND;
}

void soundLoop() {
  if (currTime - soundToneLastTime > SOUND_MS) {
    soundToneLastTime = currTime;
    if (soundToneCurr != soundToneTarget) {
      TimerFreeTone(PIN_BUZZER, soundToneCurr, SOUND_MS);
      if (soundToneCurr < soundToneTarget) {
        if (soundToneTarget - soundToneCurr <= SOUND_STEP) {
          soundToneCurr = soundToneTarget;
        }
        else {
          soundToneCurr+=SOUND_STEP;
        }
      }
      else if (soundToneCurr > soundToneTarget) {
        if (soundToneCurr - soundToneTarget <= SOUND_STEP) {
          soundToneCurr = soundToneTarget;
        }
        else {
          soundToneCurr-=SOUND_STEP;
        }
      }
    }
  }
}
#endif

void startButtonCooldown() {
  buttonCooldown = true;
  pressedCooldownLastTime = currTime;
}

/**
 * Processes the lock/unlock state. This will start a sequence to lock/unlock (processed by other loop functions).
 * WARNING: Do not call this at every loop! Should only call it when an intent is triggered!
 * 
 * int intent - Set to true if we want to lock. Set to false if we want to unlock.
 */
void processLockIntent(bool intent) {  

  if (currTime - pressedCooldownLastTime <= PRESSED_COOLDOWN_MS) {
    return;
  }
  
  int lock = intent ? HIGH : LOW;
  int unlock = intent ? LOW : HIGH;

  if (lock == HIGH && intentState != LOCKED) {
    Serial.println(F("LOCK"));
    startButtonCooldown();
    intentState = LOCKED;
    nextSequenceStage = SEQUENCE_INIT;
    nextSequenceTimeStart = currTime; // interrupt
  }
  else if (unlock == HIGH && intentState != UNLOCKED) {
    Serial.println(F("UNLOCK"));
    startButtonCooldown();
    intentState = UNLOCKED;
    nextSequenceStage = SEQUENCE_INIT;
    nextSequenceTimeStart = currTime; // interrupt
  }
  else {
    Serial.print(F("Command ignored: "));
    Serial.println(intent);
  }
}

#ifdef BUTTON_ENABLED
void buttonLoop() {

  if (currTime - buttonLastTime < BUTTON_MS) {
    return;
  }

  if (currSequenceStage != SEQUENCE_IDLE) {
    return;
  }

  buttonLastTime = currTime;
  if (buttonCooldown) {
    if (currTime - pressedCooldownLastTime > PRESSED_COOLDOWN_MS) {
      Serial.println(F("Cooldown expired, activated"));
      buttonCooldown = false;
    }
  }
  else {
    int lock = digitalRead (PIN_LOCK_BUTTON);
    if (lock == HIGH) {
      if (currState == LOCKED) {
        Serial.println(F("Button pressed, processing UNLOCK"));
        processLockIntent(false);
      }
      else {
        Serial.println(F("Button pressed, processing LOCK"));
        processLockIntent(true);
      }
    }
  }
}
#endif

#ifdef SERVO_ENABLED
void moveServo() {
  if (currTime - servoLastTime < SERVO_MS) {
    return;
  }
  
  servoLastTime = currTime;
  if (servoArmCurr < servoArmTarget) {
    if (servoArmTarget - servoArmCurr <= SERVO_STEP) {
      servoArmCurr = servoArmTarget;
    }
    else {
      servoArmCurr+=SERVO_STEP; 
      servoArm1.write(servoArmCurr);
    }
  }
  else {
    if (servoArmCurr - servoArmTarget <= SERVO_STEP) {
      servoArmCurr = servoArmTarget;
    }
    else {
      servoArmCurr-=SERVO_STEP;
      servoArm1.write(servoArmCurr);
    }
  }
#ifdef LCD_ENABLED
  lcd.setCursor(0, 1);
  lcd.print("S: ");
  lcd.print(servoArmCurr);
  lcd.print(" ");
#endif
}
#endif

#ifdef SERVO_LINEAR_ENABLED
void moveServoLinear() {
  if (currTime - servoLinearLastTime < SERVO_LINEAR_MS) {
    return;
  }
  
  servoLinearLastTime = currTime;
  
  if (servoLinearArmCurr < servoLinearArmTarget) {
    if (servoLinearArmTarget - servoLinearArmCurr <= SERVO_LINEAR_STEP) {
      servoLinearArmCurr = servoLinearArmTarget;
      servoLinearArm1.write(servoLinearArmCurr);
    }
    else {
      servoLinearArmCurr+=SERVO_LINEAR_STEP; 
      servoLinearArm1.write(servoLinearArmCurr);
    }
  }
  else {
    if (servoLinearArmCurr - servoLinearArmTarget <= SERVO_LINEAR_STEP) {
      servoLinearArmCurr = servoLinearArmTarget;
      servoLinearArm1.write(servoLinearArmCurr);
    }
    else {
      servoLinearArmCurr-=SERVO_LINEAR_STEP;
      servoLinearArm1.write(servoLinearArmCurr);
    }
  }
//  Serial.println(servoLinearArmCurr);
#ifdef LCD_ENABLED
  lcd.setCursor(8, 1);
  lcd.print("L: ");
  lcd.print(servoLinearArmCurr);
  lcd.print("  ");
#endif
}
#endif

#if defined(SERVO_ENABLED) || defined(SERVO_LINEAR_ENABLED)
void servoLoop () {
#ifdef TEST_SERVO
  if (servoArmCurr == servoArmTarget) {
    ++servoTestCount;
    Serial.print(F("Servo Test Count - ");
    Serial.println(servoTestCount);
    delay(100);
    if (servoArmTarget == SERVO_UNLOCKED_DEG) {
      servoArmTarget = SERVO_LOCKED_DEG;
    }
    else {
      servoArmTarget = SERVO_UNLOCKED_DEG;
    }
  }
#endif
#ifdef TEST_SERVO_LINEAR
  if (servoLinearArmCurr == servoLinearArmTarget) {
    ++servoLinearTestCount;
    Serial.print(F("Servo Linear Test Count - ");
    Serial.println(servoLinearTestCount);
    delay(100);
    if (servoLinearArmTarget == SERVO_LINEAR_DISENGAGED_DEG) {
      servoLinearArmTarget = SERVO_LINEAR_ENGAGED_DEG;
    }
    else {
      servoLinearArmTarget = SERVO_LINEAR_DISENGAGED_DEG;
    }
  }
#endif

#ifdef SERVO_ENABLED
  moveServo();
#endif
#ifdef SERVO_LINEAR_ENABLED
  moveServoLinear();
#endif
}
#endif

#ifdef LCD_ENABLED
void lcdPrintLoop() {
  if (currTime - lcdLastTime < LCD_MS) {
    return;
  }
  lcdLastTime = currTime;

  if (lcdState != currState) {
    switch(currState) {
      case LOCKED: {
        lcd.clear();
        lcd.print("Door LOCKED");
        lcd.setCursor(0, 1);
        break;
      }
      default: {
        lcd.clear();
        lcd.print("Door UNLOCKED");
        lcd.setCursor(0, 1);
        break;
      }
    }
    lcdState = currState;
  }
#ifdef IR_ENABLED
  lcd.setCursor(0, 2);
  lcd.print(lastIRValue, HEX);
  lcd.print("     ");
#endif
}
#endif

#ifdef IR_ENABLED
void irLoop() {
  if (currTime - irLastTime < IR_MS) {
    return;
  }
  irLastTime = currTime;
  if (irrecv1.decode(&irResults1)) {
    unsigned long irValue = irResults1.value;
    irrecv1.resume();
    Serial.print(F("R1 received signal - "));
    Serial.println(irValue, HEX);
    if (irValue == 0xFFFFFFFF) {
      return;
    }
    if (irValue != IR_LOCK_CODE && irValue != IR_UNLOCK_CODE) {
      return;
    }
    Serial.print(F("R1 received signal - "));
    Serial.println(irValue, HEX);
    lastIRValue = irValue;

    if (irValue == IR_LOCK_CODE) {
      Serial.println(F(" IR LOCK----------------"));
      processLockIntent(true);
    }
    else if (irValue == IR_UNLOCK_CODE) {
      Serial.println(F(" IR UNLOCK----------------"));
      processLockIntent(false);
    }
  }
}
#endif

#ifdef RING_ENABLED

void processRing() {
  
  if (currTime >= ringSleepTime) {
    sleepRing();
  }

  if (ringBrightnessCurr == ringBrightnessTarget) {
    return;
  }
  else {
    if (ringBrightnessCurr > ringBrightnessTarget) {
      if (ringBrightnessCurr - RING_STEP <= ringBrightnessTarget) {
        ringBrightnessCurr = ringBrightnessTarget;
      }
      else {
        ringBrightnessCurr-=RING_STEP;
      }
    }
    else if (ringBrightnessCurr < ringBrightnessTarget) {
      if (ringBrightnessCurr + RING_STEP >= ringBrightnessTarget) {
        ringBrightnessCurr = ringBrightnessTarget;
      }
      else {
        ringBrightnessCurr+=RING_STEP;
      }
    }
//    Serial.println(ringBrightnessCurr);
    
    pixels.clear();
    for (int i = 0; i < RING_NUMPIXELS; ++i) {
      pixels.setPixelColor(i, pixels.Color(ringCurrColor[0], ringCurrColor[1], ringCurrColor[2]));
    }
    pixels.setBrightness(ringBrightnessCurr);
    pixels.show();
  }
}

void ringLoop() {
  if (currTime - ringLastTime < RING_MS) {
    return;
  }
  ringLastTime = currTime;

  processRing();
}

void lockRing() {
  ringCurrColor = RING_LOCK_COLOR;
  ringBrightnessCurr = RING_BRIGHTNESS_LOW;
  ringBrightnessTarget = RING_BRIGHTNESS_HIGH;
  ringSleepTime = currTime + RING_SLEEP_TIMEOUT_MS;
}

void unlockRing() {
  ringCurrColor = RING_UNLOCK_COLOR;
  ringBrightnessCurr = RING_BRIGHTNESS_LOW;
  ringBrightnessTarget = RING_BRIGHTNESS_HIGH;
  ringSleepTime = currTime + RING_SLEEP_TIMEOUT_MS;
}

void sleepRing() {
  ringBrightnessTarget = RING_BRIGHTNESS_SLEEP;
}
#endif

void _reconcileState(bool force=false) {
  int tilt = digitalRead(PIN_TILT);
//  Serial.println(tilt);
  if (tilt == HIGH) {
//    Serial.print("TILT HIGH vs ");
//    Serial.println(currState);
    if (currState != LOCKED) {
      if (reconcileCertainty < RECONCILE_CERTAINTY && !force) {
        ++reconcileCertainty;
        return;
      }
      Serial.print("LOCK Reconcile Certainty: ");
      Serial.println(reconcileCertainty);
      reconcileCertainty = 0;
      Serial.print("LOCK Reconcile Certainty (AFTER): ");
      Serial.println(reconcileCertainty);
      Serial.println(F("************** LOCKED! **************"));
      currState = LOCKED;
#ifdef RING_ENABLED
      lockRing();
#endif
#ifdef BUZZER_ENABLED
      lockSound();
#endif
    }
    else {
      // reset in case of failed attempt
      reconcileCertainty = 0;
    }
  }
  else { // tilt == LOW
//    Serial.print("TILT LOW vs ");
//    Serial.println(currState);
    if (currState != UNLOCKED) {
      if (reconcileCertainty < RECONCILE_CERTAINTY && !force) {
        ++reconcileCertainty;
        return;
      }
      Serial.print("UNLOCK Reconcile Certainty (BEFORE): ");
      Serial.println(reconcileCertainty);
      reconcileCertainty = 0;
      Serial.println(F("************** UNLOCKED **************"));
      currState = UNLOCKED;
#ifdef RING_ENABLED
      unlockRing();
#endif
#ifdef BUZZER_ENABLED
      unlockSound();
#endif
    }
    else {
      // reset in case of failed attempt
      reconcileCertainty = 0;
    }
  }
}

void executeSequence() { 
  if (nextSequenceStage == SEQUENCE_IDLE) {
#ifdef TEST_SEQUENCE
    if (currTime >= testSequenceNextTime) {
      ++testSequenceCount;
      Serial.print(F("Sequence Test Count - ");
      Serial.println(testSequenceCount);
      delay(500);
      if (testSequenceIntent == LOCKED) {
        testSequenceIntent = UNLOCKED;
        processLockIntent(false);
      }
      else {
        testSequenceIntent = LOCKED;
        processLockIntent(true);
      }
    }
#else
    return;
#endif
  }

  if (currTime < nextSequenceTimeStart) {
    return;
  }

  int nextWaitTime = SEQUENCE_WAIT[nextSequenceStage];
  nextSequenceTimeStart = currTime + nextWaitTime;
  currSequenceStage = nextSequenceStage;
  switch (nextSequenceStage) {
    case SEQUENCE_INIT: {
      Serial.println(F("START --------------------------"));
      Serial.println(F("SEQUENCE_INIT"));
#ifdef SERVO_LINEAR_ENABLED
      // Disengage the main servo      
#ifndef TEST_SERVO_LINEAR
      servoLinearArm1.attach(PIN_SERVO_LINEAR1, 530, 2600);
#endif
      servoLinearArmTarget = SERVO_LINEAR_DISENGAGED_DEG;
#endif
      nextSequenceStage = SEQUENCE_RESET_ARM;
      break;
    }
    case SEQUENCE_RESET_ARM: {
#ifdef SERVO_ENABLED
      Serial.println(F("SEQUENCE_RESET_ARM"));
      servoArm1.attach(PIN_SERVO1, 530, 2600);
#ifndef TEST_SEQUENCE
      // Reset arm according to the current state
      switch(currState) {
        case LOCKED: {
          Serial.println(F("Resetting arm to LOCKED"));
          servoArmTarget = SERVO_LOCKED_DEG;
          servoArmCurr = SERVO_LOCKED_DEG;
          servoArm1.write(SERVO_LOCKED_DEG);
          break;
        }
        case UNLOCKED: {
          Serial.println(F("Resetting arm to UNLOCKED"));
          servoArmTarget = SERVO_UNLOCKED_DEG;
          servoArmCurr = SERVO_UNLOCKED_DEG;
          servoArm1.write(SERVO_UNLOCKED_DEG);
          break;
        }
      }
#endif
#endif
      
      nextSequenceStage = SEQUENCE_ENGAGE;
      break;
    }
    case SEQUENCE_ENGAGE: {
      Serial.println(F("SEQUENCE_ENGAGE"));
      
#ifdef SERVO_LINEAR_ENABLED
      // Engage the main servo
      servoLinearArmTarget = SERVO_LINEAR_ENGAGED_DEG;
#endif

      nextSequenceStage = SEQUENCE_ACTION;
      break;
    }
    case SEQUENCE_ACTION: {
      Serial.print(F("SEQUENCE_ACTION. Intent: "));
      Serial.println(intentState);
#ifdef SERVO_ENABLED
      servoArmTarget = (intentState == LOCKED ? SERVO_LOCKED_DEG : SERVO_UNLOCKED_DEG);
#endif
      nextSequenceStage = SEQUENCE_DISENGAGE;
      break;
    }
    case SEQUENCE_DISENGAGE: {
      Serial.println(F("SEQUENCE_DISENGAGE"));
#ifdef SERVO_LINEAR_ENABLED
      // Disengage main servo
      servoLinearArmTarget = SERVO_LINEAR_DISENGAGED_DEG;
#endif

      nextSequenceStage = SEQUENCE_END;

      //Force a reconcile
//      _reconcileState(true);
      break;
    }
    case SEQUENCE_END: {
      Serial.println(F("SEQUENCE_END"));
      Serial.println(F("DONE --------------------------"));

#ifdef SERVO_LINEAR_ENABLED
#ifndef TEST_SERVO_LINEAR
      servoLinearArm1.detach();
#endif
#endif
#ifdef SERVO_ENABLED
      servoArm1.detach();
#endif
      
      nextSequenceStage = SEQUENCE_IDLE;
      currSequenceStage = SEQUENCE_IDLE;
      intentState = UNKNOWN;

#ifdef TEST_SEQUENCE
      testSequenceNextTime = currTime + 1000;
#endif

      break;
    }
  }
}

void reconcileCurrState () {
  if (currTime - reconcileLastTime < RECONCILE_MS) {
    return;
  }
  reconcileLastTime = currTime;
  
  // Lock is busy, later
  if (currSequenceStage != SEQUENCE_IDLE) {
    return;
  }

  _reconcileState();
}

#ifdef LCD_ENABLED
void initI2C() {

  byte error, address;
  int nDevices;

  Serial.println(F("Scanning I2C devices...");
  nDevices = 0;
  for(address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0) {
      Serial.print(F("I2C device found at address 0x");
      if (address < 16) {
        Serial.print(F("0");
      }
      Serial.print(address, HEX);
      Serial.println(F(" !");
  
      nDevices++;
    }
    else if (error == 4) {
      Serial.print(F("Unknown error at address 0x");
      if (address < 16) {
        Serial.print(F("0");
      }
      Serial.println(address, HEX);
    }
  }

  if (nDevices == 0) {
    Serial.println(F("No I2C devices found");
  }
  else {
    Serial.println(F("Scanning done\n");
  }
  
  lcd.begin();
}
#endif

#ifdef RFID_ENABLED
void printBytes( byte toPrint[] ) {
  Serial.print(F("0x"));
  for ( uint8_t i = 0; i < 4; i++ ) {
    Serial.print(toPrint[i], HEX);
  }
  Serial.println(F(""));
}

void showCardDetails() {
  Serial.print(F("Master Card ---- "));
  for ( uint8_t i = 0; i < 4; i++ ) {          // Read Master Card's UID from EEPROM
    masterCard[i] = EEPROM.read(i + RFID_EEPROM_MASTER_CARD_OFFSET);    // Write it to masterCard
  }
  printBytes(masterCard);
  Serial.println(F("Slave cards >> "));
  for (uint8_t slotIndex = 0; slotIndex < RFID_EEPROM_SLAVE_CARD_SLOTS; ++slotIndex) {
    byte thisSlave[4];
    for(uint8_t i = 0; i < 4; ++i) {
      thisSlave[i] = EEPROM.read(RFID_EEPROM_SLAVE_CARD_START + (slotIndex * RFID_EEPROM_SLAVE_CARD_SLOT_OFFSET) + i);
    }
    printBytes(thisSlave);
  }
}

bool checkTwo ( byte a[], byte b[] ) {   
  for ( uint8_t k = 0; k < 4; k++ ) {   // Loop 4 times
    if ( a[k] != b[k] ) {     // IF a != b then false, because: one fails, all fail
       return false;
    }
  }
  return true;  
}

void copyBytes( byte* src, byte* dst ) {
  for ( uint8_t k = 0; k < 4; k++ ) {   // Loop 4 times
    dst[k] = src[k];
  }
}

void readCardFromEEPROM( uint8_t address, byte* dst) {
  for ( uint8_t i = 0; i < 4; i++ ) {          // Read Master Card's UID from EEPROM
    dst[i] = EEPROM.read(i + address);    // Write it to masterCard
  }
}

bool isMaster( byte test[] ) {
  return checkTwo(test, masterCard) && !checkTwo(test, ZEROS) && !checkTwo(test, ONES);
}

bool isSlave( byte test[] ) {
  if (checkTwo(test, ZEROS) || checkTwo(test, ONES)) {
    return false;
  }
  for (uint8_t slotIndex = 0; slotIndex < RFID_EEPROM_SLAVE_CARD_SLOTS; ++slotIndex) {
    byte thisSlave[4];
    for(uint8_t i = 0; i < 4; ++i) {
      thisSlave[i] = EEPROM.read(RFID_EEPROM_SLAVE_CARD_START + (slotIndex * RFID_EEPROM_SLAVE_CARD_SLOT_OFFSET) + i);
    }
    if (checkTwo(thisSlave, test)) {
      return true;
    }
  }
  return false;
}

void eraseEEPROM() {
  for (uint16_t x = 0; x < EEPROM.length(); x = x + 1) {    //Loop end of EEPROM address
    if (EEPROM.read(x) == 0) {              //If EEPROM address 0
      // do nothing, already clear, go to the next address in order to save time and reduce writes to EEPROM
    }
    else {
      EEPROM.write(x, 0);       // if not write 0 to clear, it takes 3.3mS
    }
  }
  rfidMode = RFID_MODE_REGISTERING_MASTER;
}

void initRFID() {
  showCardDetails();
//  eraseEEPROM();

  if (EEPROM.read(1) != MASTER_CARD_DEFINED_CODE) {
    Serial.println(F("No Master Card Defined, scan a PICC to define as Master Card"));
    rfidMode = RFID_MODE_REGISTERING_MASTER;
    eraseEEPROM();
  }
  else {
    rfidMode = RFID_MODE_IDLE;
  }
}

void tryRegisterMasterCard() {  
  if (cardAttemptsCount == 0) {
    Serial.println(F("Card detected. Please scan 2 more times to confirm card as master."));
    copyBytes(readCard, cardCandidates[0]);
    Serial.println(F("--- Master Card Registration first attempt "));
    ++cardAttemptsCount;
  }
  else {
    Serial.println(F("Comparing cards"));
    printBytes(readCard);
    printBytes(cardCandidates[0]);
    if(checkTwo(readCard, cardCandidates[0])) { // check against the first entry
      copyBytes(readCard, cardCandidates[cardAttemptsCount]);
      
      ++cardAttemptsCount;
      Serial.print(F("--- Master Card Registration attempt "));
      Serial.println(cardAttemptsCount);
    }
    else {
      Serial.println(F("Different card detected. Please scan 2 more times to confirm card as master."));
      // not the same card as previous
      copyBytes(readCard, cardCandidates[0]);
    }
  }
  
  if (cardAttemptsCount >= CARD_ATTEMPT_MAX_COUNT) {
    // 3 consecutive attempts
    Serial.print(F("Confirmed Master Card: "));
    printBytes(readCard);
  
    // Register this card as master
    for ( uint8_t j = 0; j < 4; j++ ) {        // Loop 4 times
      EEPROM.write( j + RFID_EEPROM_MASTER_CARD_OFFSET, readCard[j] );  // Write scanned PICC's UID to EEPROM, start from address 3
    }
    EEPROM.write(1, 0x8F);                  // Write to EEPROM we defined Master Card.
    readCardFromEEPROM(RFID_EEPROM_MASTER_CARD_OFFSET, masterCard);
    rfidMode = RFID_MODE_IDLE;
    showCardDetails();
  }
}

void tryRegisterSlaveCard() {
  if (isMaster(readCard)) {
    Serial.println(F("Master Card detected. Slave Card registration cancelled."));
    cardAttemptsCount = 0;
    rfidMode = RFID_MODE_IDLE;
    return;
  }
  
  if (cardAttemptsCount == 0) {
    Serial.println(F("Card detected. Please scan 2 more times to confirm card as slave."));
    copyBytes(readCard, cardCandidates[0]);
    Serial.println(F("--- Slave Card Registration first attempt "));
    ++cardAttemptsCount;
    registerSlaveCardTimeout = currTime + RFID_SLAVE_CARD_REGISTRATION_TIMEOUT;
  }
  else {
    if(checkTwo(readCard, cardCandidates[0])) { // check against the first entry
      copyBytes(readCard, cardCandidates[cardAttemptsCount]);
      
      ++cardAttemptsCount;
      Serial.print(F("--- Slave Card Registration attempt "));
      Serial.println(cardAttemptsCount);
      registerSlaveCardTimeout = currTime + RFID_SLAVE_CARD_REGISTRATION_TIMEOUT;
    }
    else {
      Serial.println(F("Different card detected. Scan the master card to restart process."));
      cardAttemptsCount = 0;
      rfidMode = RFID_MODE_IDLE;
    }
  }
  
  if (cardAttemptsCount >= CARD_ATTEMPT_MAX_COUNT) {
    // 3 consecutive attempts
    rfidMode = RFID_MODE_IDLE;
    int slotIndex = EEPROM.read(RFID_EEPROM_SLAVE_CARD_NEXT_FREE_SLOT);

    for (int i = 0 ; i < 4; ++i) {
      EEPROM.write(RFID_EEPROM_SLAVE_CARD_START + (slotIndex * RFID_EEPROM_SLAVE_CARD_SLOT_OFFSET) + i, cardCandidates[0][i]);
    }
    Serial.print(F("Saved in slot "));
    Serial.println(slotIndex + 1);

    // Save the next free slot
    int nextSlotIndex = 0;
    nextSlotIndex = slotIndex + 1 >= RFID_EEPROM_SLAVE_CARD_SLOTS ? 0 : slotIndex + 1;
    EEPROM.write(RFID_EEPROM_SLAVE_CARD_NEXT_FREE_SLOT, nextSlotIndex);

    showCardDetails();
  }
  
}

bool tryReceiveFromRFIDUnit() {
  
#ifdef BLE_ENABLED
  int i = 0;
  if (btSerial.available()) {
    byte cardId[4];
    btSerial.readBytes(readCard, 4);
    printBytes(readCard);
    return true;
  }
  else {
    return false; 
  }
#else
  return false;
#endif

}

void rfidLoop() {
  if (currTime - rfidLastTime < RFID_MS) {
    return;
  }

  if (currSequenceStage != SEQUENCE_IDLE) {
    return;
  }

  // Timeout for slave registration
  if (rfidMode == RFID_MODE_REGISTERING_SLAVE) {
    if (currTime >= registerSlaveCardTimeout) {
      Serial.println(F("Slave card registration timed-out. Please scan the master card to try again."));
      cardAttemptsCount = 0;
      rfidMode = RFID_MODE_IDLE;
    }
  }

  if (tryReceiveFromRFIDUnit()) {
    switch(rfidMode) {
      case RFID_MODE_REGISTERING_MASTER: {
        tryRegisterMasterCard();
        return;
      }
      case RFID_MODE_REGISTERING_SLAVE: {
        tryRegisterSlaveCard();
        return;
      }
      case RFID_MODE_IDLE: 
      default: {
        if (isMaster(readCard)) {
          Serial.println(F("Master card detected! Switch mode to Slave card registration."));
          cardAttemptsCount = 0;
          rfidMode = RFID_MODE_REGISTERING_SLAVE;
          registerSlaveCardTimeout = currTime + RFID_SLAVE_CARD_REGISTRATION_TIMEOUT;
        }
        else if (isSlave(readCard)) {
          Serial.print(F("This is a slave? "));
          printBytes(readCard);
          if (currState == LOCKED) {
            Serial.println(F("RFID authorized, processing UNLOCK"));
            processLockIntent(false);
          }
          else {
            Serial.println(F("RFID authorized, processing LOCK"));
            processLockIntent(true);
          }
        }
        break;
      }
    }
  }
}
#endif

#ifdef BLE_ENABLED
void initBLE() {
  btSerial.begin(9600);
}
#endif

// the setup function runs once when you press reset or power the board
void setup() {

#if defined(SERVO_ENABLED) || defined(SERVO_LINEAR_ENABLED)
  Wire.begin();
#endif
  
  Serial.begin(9600);
  while(!Serial) {
    Serial.println(F("I2C Scanner"));
  }
  Serial.println(F("Serial READY!\n"));

#ifdef LCD_ENABLED
  initI2C();
  lcdState = UNKNOWN;
#endif

  pinMode(PIN_LOCK_BUTTON, INPUT);
  pinMode(PIN_TILT, INPUT);
#ifdef BUZZER_ENABLED
  pinMode(PIN_BUZZER, OUTPUT);
#endif

#ifdef SERVO_LINEAR_ENABLED
  Serial.println(F("Init Linear Servo"));
  servoLinearArm1.attach(PIN_SERVO_LINEAR1, 530, 2600);
  servoLinearArm1.write(SERVO_LINEAR_DISENGAGED_DEG);
  delay(1000);
#ifndef TEST_SERVO_LINEAR
  servoLinearArm1.detach();
#endif
#endif

#ifdef SERVO_ENABLED
  Serial.println(F("Init Main Servo"));
  servoArm1.attach(PIN_SERVO1, 530, 2600);
  servoArm1.write(SERVO_UNLOCKED_DEG);
  delay(1000);
#ifndef TEST_SERVO
  servoArm1.detach();
#endif
#endif

#ifdef IR_ENABLED
  irrecv1.enableIRIn();
  irrecv1.blink13(true);
#endif

#ifdef RING_ENABLED
  pixels.begin(); // INITIALIZE NeoPixel strip object (REQUIRED)
  pixels.setBrightness(ringBrightnessCurr);
  pixels.show();
#endif

  currTime = millis();
#ifdef RING_ENABLED
  ringSleepTime = currTime + RING_SLEEP_TIMEOUT_MS;
#endif

#ifdef RFID_ENABLED
  initRFID();
#endif

#ifdef BLE_ENABLED
  initBLE();
#endif
}

// the loop function runs over and over again forever
void loop() {
  currTime = millis();
  
  reconcileCurrState(); // reconcile the current status using reed

#ifdef BUTTON_ENABLED
  buttonLoop();
#endif
#ifdef RFID_ENABLED
  rfidLoop();
#endif
#ifdef IR_ENABLED
  irLoop();
#endif
  executeSequence();
#if defined(SERVO_ENABLED) || defined(SERVO_LINEAR_ENABLED)
  servoLoop();
#endif
#ifdef LCD_ENABLED
  lcdPrintLoop();
#endif
#ifdef RING_ENABLED
  ringLoop();
#endif
#ifdef BUZZER_ENABLED
  soundLoop();
#endif
}
