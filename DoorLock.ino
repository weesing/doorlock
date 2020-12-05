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

#define LOCKED 1
#define UNLOCKED 0
#define UNKNOWN -1
#ifdef TEST_SEQUENCE
int testSequenceIntent = UNKNOWN;
int testSequenceCount = -1;
unsigned long testSequenceNextTime = 0;
#endif

//#define RING_ENABLED
//#define BUZZER_ENABLED
//#define IR_ENABLED
#if !defined(TEST_SERVO) && !defined(TEST_SERVO_LINEAR) // only define BUTTON_ENABLED only if TEST_SERVO_LINEAR is not defined so as not to interrupt the tests.
#define BUTTON_ENABLED
#endif

////////////////////////////////////////////// All required libraries
#include <EEPROM.h> // We are going to read and write PICC's UIDs from/to EEPROM
#include <SoftwareSerial.h>
#include <Servo.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#ifdef BUZZER_ENABLED
#include <TimerFreeTone.h>
#endif
#ifdef RING_ENABLED
#include <Adafruit_NeoPixel.h>
#endif
#ifdef IR_ENABLED
#include <IRremote.h>
#endif

////////////////////////////////////////////// End required libraries

//PWM
//3, 5, 6, 9, 10, and 11
#define PIN_SERVO_MAIN 2 // MG996
#define PIN_LOCK_BUTTON 3
#define PIN_SERVO_LINEAR 4 // SG90
#define PIN_ADXL A0        // GY-61 ADXL335
#define PIN_BLE_RXD 10
#define PIN_BLE_TXD 11
#ifdef RING_ENABLED
#define PIN_RING_LED 6
#endif
#ifdef BUZZER_ENABLED
#define PIN_BUZZER 7
#endif
#ifdef IR_ENABLED
#define PIN_IR_RECEIVER1 8
#endif

///////////////////////////////// BUTTON
#define PRESSED_COOLDOWN_MS 2000
unsigned long pressedCooldownLastTime;
bool buttonCoolingDown = false;
#define BUTTON_MS 100
unsigned long buttonLastTime;

//////////////////////////////// MAIN SERVO
#define SERVO_UNLOCK_FREQ 1800
#define SERVO_IDLE_FREQ 1500
#define SERVO_LOCK_FREQ 1200
Servo servoRotateArm;
// Servo tests
#ifdef TEST_SERVO
uint32_t servoTestCount = -1; //For keeping track of how many times the servo has been moved/tested. For battery capacity testing.
#endif

///////////////////////// ADXL
#define LOCKED_MIN_ANGLE 580
#define UNLOCKED_MAX_ANGLE 436

//////////////////////////////// LINEAR SERVO
#define SERVO_LINEAR_ENGAGED_DEG 82
#define SERVO_LINEAR_DISENGAGED_DEG 50
#define SERVO_LINEAR_STEP 4
#define SERVO_LINEAR_MS 10
Servo servoLinearArm;
uint16_t servoLinearArmTarget = SERVO_LINEAR_DISENGAGED_DEG;
uint16_t servoLinearArmCurr = SERVO_LINEAR_DISENGAGED_DEG;
unsigned long servoLinearLastTime = 0;
// Linear Servo tests
#ifdef TEST_SERVO_LINEAR
int servoLinearTestCount = -1; //For keeping track of how many times the servo has been moved/tested. For battery capacity testing.
#endif

///////////////////////// RFID
#define RFID_MS 250
unsigned long rfidLastTime = 0;
#define RFID_EEPROM_MASTER_CARD_DEFINED 1
#define RFID_EEPROM_MASTER_CARD_OFFSET RFID_EEPROM_MASTER_CARD_DEFINED + 1
#define RFID_EEPROM_SLAVE_CARD_NEXT_FREE_SLOT 6
#define RFID_EEPROM_SLAVE_CARD_START 7       // Each slot is 4 bytes data
#define RFID_EEPROM_SLAVE_CARD_SLOT_OFFSET 4 // Offset between each slot
#define RFID_EEPROM_SLAVE_CARD_SLOTS 8

#define MASTER_CARD_DEFINED_CODE 0x8F
uint8_t cardAttemptsCount = 0;
#define CARD_ATTEMPT_MAX_COUNT 3
byte cardCandidates[3][4];
unsigned long registerSlaveCardTimeout = 0;
#define RFID_SLAVE_CARD_REGISTRATION_TIMEOUT 10000

#define RFID_MODE_IDLE 0
#define RFID_MODE_REGISTERING_MASTER 1
#define RFID_MODE_REGISTERING_SLAVE 2
uint8_t rfidMode = RFID_MODE_IDLE;

byte readCard[4];   // Stores scanned ID read from RFID Module
byte masterCard[4]; // Stores master card's ID read from EEPROM

const uint8_t ZEROS[] = {0x0, 0x0, 0x0, 0x0};
const uint8_t ONES[] = {0xFF, 0xFF, 0xFF, 0xFF};

///////////////////////// BLE
SoftwareSerial btSerial(PIN_BLE_RXD, PIN_BLE_TXD); // RX | TX

///////////////////////// OLED Display
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET 4 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define ICON_BMP_HEIGHT 22
#define ICON_BMP_WIDTH 16

static const unsigned char PROGMEM lock_bmp[] =
    {B00000000, B00000000,
     B00000000, B00000000,
     B00000000, B00000000,
     B00000000, B00000000,
     B00000111, B11100000,
     B00001100, B00110000,
     B00001000, B00010000,
     B00001000, B00010000,
     B00001000, B00010000,
     B00001000, B00010000,
     B00111111, B11111100,
     B01111111, B11111110,
     B01111111, B11111110,
     B01111100, B00111110,
     B01111100, B00111110,
     B01111110, B01111110,
     B01111110, B01111110,
     B01111110, B01111110,
     B01111111, B11111110,
     B01111111, B11111110,
     B01111111, B11111110,
     B00111111, B11111100};

static const unsigned char PROGMEM unlock_bmp[] =
    {B00000111, B11100000,
     B00001100, B00110000,
     B00001000, B00010000,
     B00001000, B00010000,
     B00001000, B00010000,
     B00001000, B00010000,
     B00000000, B00010000,
     B00000000, B00010000,
     B00000000, B00010000,
     B00000000, B00010000,
     B00111111, B11111100,
     B01111111, B11111110,
     B01111111, B11111110,
     B01111100, B00111110,
     B01111100, B00111110,
     B01111110, B01111110,
     B01111110, B01111110,
     B01111110, B01111110,
     B01111111, B11111110,
     B01111111, B11111110,
     B01111111, B11111110,
     B00111111, B11111100};

#ifdef IR_ENABLED
IRrecv irrecv1(PIN_IR_RECEIVER1);
decode_results irResults1;
#define IR_LOCK_CODE 0xFFE21D
#define IR_UNLOCK_CODE 0xFFA25D
#define IR_MS 60
unsigned long irLastTime;
unsigned long lastIRValue;

void irLoop()
{
  if (g_currTime - irLastTime < IR_MS)
  {
    return;
  }
  irLastTime = g_currTime;
  if (irrecv1.decode(&irResults1))
  {
    unsigned long irValue = irResults1.value;
    irrecv1.resume();
    Serial.print(F("R1 received signal - "));
    Serial.println(irValue, HEX);
    if (irValue == 0xFFFFFFFF)
    {
      return;
    }
    if (irValue != IR_LOCK_CODE && irValue != IR_UNLOCK_CODE)
    {
      return;
    }
    Serial.print(F("R1 received signal - "));
    Serial.println(irValue, HEX);
    lastIRValue = irValue;

    if (irValue == IR_LOCK_CODE)
    {
      Serial.println(F(" IR LOCK----------------"));
      applyLockIntent(LOCKED);
    }
    else if (irValue == IR_UNLOCK_CODE)
    {
      Serial.println(F(" IR UNLOCK----------------"));
      applyLockIntent(UNLOCKED);
    }
  }
}
#endif
#ifdef RING_ENABLED
#define RING_NUMPIXELS 24 // Popular NeoPixel ring size
#define RING_SLEEP_TIMEOUT_MS 5000
Adafruit_NeoPixel pixels(RING_NUMPIXELS, PIN_RING_LED, NEO_GRB + NEO_KHZ800);
#define RING_MS 200
#define RING_STEP 50
#define RING_BRIGHTNESS_LOW 0
#define RING_BRIGHTNESS_SLEEP 0
#define RING_PULSE_HIGH 30
#define RING_PULSE_LOW 5
#define RING_BRIGHTNESS_HIGH 255
unsigned long ringLastTime;
unsigned long ringSleepTime;
int ringBrightnessCurr = RING_BRIGHTNESS_LOW;
int ringBrightnessTarget = RING_BRIGHTNESS_HIGH;
const int RING_LOCK_COLOR[] = {200, 0, 0};
const int RING_UNLOCK_COLOR[] = {0, 0, 200};
int *ringCurrColor = RING_LOCK_COLOR;

void processRing()
{
  if (g_currTime >= ringSleepTime)
  {
    sleepRing();
  }

  if (ringBrightnessCurr == ringBrightnessTarget)
  {
    return;
  }
  else
  {
    if (ringBrightnessCurr > ringBrightnessTarget)
    {
      if (ringBrightnessCurr - RING_STEP <= ringBrightnessTarget)
      {
        ringBrightnessCurr = ringBrightnessTarget;
      }
      else
      {
        ringBrightnessCurr -= RING_STEP;
      }
    }
    else if (ringBrightnessCurr < ringBrightnessTarget)
    {
      if (ringBrightnessCurr + RING_STEP >= ringBrightnessTarget)
      {
        ringBrightnessCurr = ringBrightnessTarget;
      }
      else
      {
        ringBrightnessCurr += RING_STEP;
      }
    }
    //    Serial.println(ringBrightnessCurr);

    pixels.clear();
    for (int i = 0; i < RING_NUMPIXELS; ++i)
    {
      pixels.setPixelColor(i, pixels.Color(ringCurrColor[0], ringCurrColor[1], ringCurrColor[2]));
    }
    pixels.setBrightness(ringBrightnessCurr);
    pixels.show();
  }
}

void ringLoop()
{
  if (g_currTime - ringLastTime < RING_MS)
  {
    return;
  }
  ringLastTime = g_currTime;

  processRing();
}

void lockRing()
{
  ringCurrColor = RING_LOCK_COLOR;
  ringBrightnessCurr = RING_BRIGHTNESS_LOW;
  ringBrightnessTarget = RING_BRIGHTNESS_HIGH;
  ringSleepTime = g_currTime + RING_SLEEP_TIMEOUT_MS;
}

void unlockRing()
{
  ringCurrColor = RING_UNLOCK_COLOR;
  ringBrightnessCurr = RING_BRIGHTNESS_LOW;
  ringBrightnessTarget = RING_BRIGHTNESS_HIGH;
  ringSleepTime = g_currTime + RING_SLEEP_TIMEOUT_MS;
}

void sleepRing()
{
  ringBrightnessTarget = RING_BRIGHTNESS_SLEEP;
}
#endif
#ifdef BUZZER_ENABLED
void lockSound()
{
  soundToneCurr = HIGH_SOUND;
  soundToneTarget = LOW_SOUND;
}

void unlockSound()
{
  soundToneCurr = LOW_SOUND;
  soundToneTarget = HIGH_SOUND;
}

void soundLoop()
{
  if (g_currTime - soundToneLastTime > SOUND_MS)
  {
    soundToneLastTime = g_currTime;
    if (soundToneCurr != soundToneTarget)
    {
      TimerFreeTone(PIN_BUZZER, soundToneCurr, SOUND_MS);
      if (soundToneCurr < soundToneTarget)
      {
        if (soundToneTarget - soundToneCurr <= SOUND_STEP)
        {
          soundToneCurr = soundToneTarget;
        }
        else
        {
          soundToneCurr += SOUND_STEP;
        }
      }
      else if (soundToneCurr > soundToneTarget)
      {
        if (soundToneCurr - soundToneTarget <= SOUND_STEP)
        {
          soundToneCurr = soundToneTarget;
        }
        else
        {
          soundToneCurr -= SOUND_STEP;
        }
      }
    }
  }
}
#endif

unsigned long g_currTime;

#define SEQUENCE_IDLE -1
#define SEQUENCE_INIT 0
#define SEQUENCE_RESET_ARM 1
#define SEQUENCE_ENGAGE 2
#define SEQUENCE_START_ACTION 3
#define SEQUENCE_ACTION 4
#define SEQUENCE_DISENGAGE 5
#define SEQUENCE_END 6

int8_t g_currSequenceStage = SEQUENCE_IDLE;

uint8_t g_currState = UNKNOWN;
uint8_t g_intentState = UNKNOWN;
uint32_t g_currKnobAngle = -1;

#ifdef BUZZER_ENABLED
#define HIGH_SOUND 3001
#define LOW_SOUND 1000
#define SOUND_STEP 500
#define SOUND_MS 20
uint8_t soundToneTarget = 0;
uint8_t soundToneCurr = 0;
unsigned long soundToneLastTime;
#endif

void applyLockIntent(int intent)
{
  Serial.println(F("Applying lock intent"));
  g_intentState = intent;
  if (g_currState == intent)
  {
    Serial.println(F("Already at current locked state, ignoring intent."));
    return;
  }

  switch (intent)
  {
  case LOCKED:
    Serial.println(F("LOCKING"));
    g_intentState = LOCKED;
    g_currSequenceStage = SEQUENCE_INIT;
    break;
  case UNLOCKED:
    Serial.println(F("UNLOCKING"));
    g_intentState = UNLOCKED;
    g_currSequenceStage = SEQUENCE_INIT;
    break;
  }
}

void buttonLoop()
{
  if (g_currTime - buttonLastTime < BUTTON_MS)
  {
    return;
  }

  if (g_currSequenceStage != SEQUENCE_IDLE)
  {
    return;
  }

  buttonLastTime = g_currTime;
  if (buttonCoolingDown)
  {
    if (g_currTime - pressedCooldownLastTime > PRESSED_COOLDOWN_MS)
    {
      Serial.println(F("Cooldown expired, activated"));
      buttonCoolingDown = false;
    }
    else
    {
      return;
    }
  }
  else
  {
    int btnPressed = digitalRead(PIN_LOCK_BUTTON);
    if (btnPressed == HIGH)
    {
      buttonCoolingDown = true;
      pressedCooldownLastTime = g_currTime;
      if (g_currState == LOCKED)
      {
        Serial.println(F("Button pressed, processing UNLOCK"));
        applyLockIntent(UNLOCKED);
      }
      else
      {
        Serial.println(F("Button pressed, processing LOCK"));
        applyLockIntent(LOCKED);
      }
    }
  }
}

bool moveLinearServo()
{
#ifdef TEST_SERVO_LINEAR
  if (servoLinearArmCurr == servoLinearArmTarget)
  {
    ++servoLinearTestCount;
    Serial.print(F("Servo Linear Test Count - "));
    Serial.println(servoLinearTestCount);
    delay(1000);
    if (servoLinearArmTarget == SERVO_LINEAR_DISENGAGED_DEG)
    {
      servoLinearArmTarget = SERVO_LINEAR_ENGAGED_DEG;
    }
    else
    {
      servoLinearArmTarget = SERVO_LINEAR_DISENGAGED_DEG;
    }
  }
#endif
  if (g_currTime - servoLinearLastTime < SERVO_LINEAR_MS)
  {
    return false;
  }
  servoLinearLastTime = g_currTime;
  if (servoLinearArmCurr < servoLinearArmTarget)
  {
    if (servoLinearArmTarget - servoLinearArmCurr <= SERVO_LINEAR_STEP)
    {
      servoLinearArmCurr = servoLinearArmTarget;
    }
    else
    {
      servoLinearArmCurr += SERVO_LINEAR_STEP;
    }
    servoLinearArm.write(servoLinearArmCurr);
  }
  else
  {
    if (servoLinearArmCurr - servoLinearArmTarget <= SERVO_LINEAR_STEP)
    {
      servoLinearArmCurr = servoLinearArmTarget;
    }
    else
    {
      servoLinearArmCurr -= SERVO_LINEAR_STEP;
    }
    servoLinearArm.write(servoLinearArmCurr);
  }
  return servoLinearArmCurr == servoLinearArmTarget;
}

int readADXL()
{
  analogReference(EXTERNAL);
  int yRot = analogRead(PIN_ADXL);
  // yRot = map(yRot, 0, 1023, 0, 255);
  // Serial.println(yRot);
  return yRot;
}

void performSequenceActions()
{
  switch (g_currSequenceStage)
  {
  case SEQUENCE_IDLE:
    break;
  case SEQUENCE_INIT:
    servoLinearArmTarget = SERVO_LINEAR_ENGAGED_DEG;
    g_currSequenceStage = SEQUENCE_ENGAGE; // go to next stage immediately on next loop.
    servoLinearArm.attach(PIN_SERVO_LINEAR, 530, 2600);
    servoRotateArm.attach(PIN_SERVO_MAIN, 530, 2600);
    Serial.println(F("SEQUENCE_ENGAGE"));
    break;
  case SEQUENCE_ENGAGE:
    if (moveLinearServo())
    {
      g_currSequenceStage = SEQUENCE_START_ACTION;
      Serial.println(F("SEQUENCE_START_ACTION"));
    }
    break;
  case SEQUENCE_START_ACTION:
    if (g_intentState == LOCKED)
    {
      servoRotateArm.writeMicroseconds(SERVO_LOCK_FREQ);
    }
    else if (g_intentState == UNLOCKED)
    {
      servoRotateArm.writeMicroseconds(SERVO_UNLOCK_FREQ);
    }
    g_currSequenceStage = SEQUENCE_ACTION;
    Serial.println(F("SEQUENCE_ACTION"));
    Serial.println(F("Waiting for action to complete."));
    break;
  case SEQUENCE_ACTION:
    if (g_currState == g_intentState)
    {
      int yRot = readADXL();
      Serial.println(yRot);
      Serial.println(F("Main servo action completed. Stopping rotation."));
      servoRotateArm.writeMicroseconds(SERVO_IDLE_FREQ);
      g_currSequenceStage = SEQUENCE_DISENGAGE;
      Serial.println(F("SEQUENCE_DISENGAGE"));
      servoLinearArmTarget = SERVO_LINEAR_DISENGAGED_DEG;
    }
    break;
  case SEQUENCE_DISENGAGE:
    if (moveLinearServo())
    {
      g_currSequenceStage = SEQUENCE_END;
      Serial.println(F("SEQUENCE_END"));
      servoLinearArm.detach();
      servoRotateArm.detach();
    }
    break;
  case SEQUENCE_END:
    g_currSequenceStage = SEQUENCE_IDLE;
    Serial.println(F("SEQUENCE_IDLE"));
    break;
  }
}

void reconcileLockState()
{
  int yRot = readADXL();
  g_currKnobAngle = yRot;
  if (g_currState == UNLOCKED && g_currKnobAngle >= LOCKED_MIN_ANGLE)
  {
#ifdef RING_ENABLED
    lockRing();
#endif
#ifdef BUZZER_ENABLED
    lockSound();
#endif
    Serial.println("State changed to LOCKED.");
    g_currState = LOCKED;
  }
  else if (g_currState == LOCKED && g_currKnobAngle <= UNLOCKED_MAX_ANGLE)
  {
#ifdef RING_ENABLED
    unlockRing();
#endif
#ifdef BUZZER_ENABLED
    unlockSound();
#endif
    Serial.println("State changed to UNLOCKED.");
    g_currState = UNLOCKED;
  }
}

void reconcileOLEDDisplay()
{
  int logoX = 10;
  int textX = 28;
  int textY = 13;

  display.clearDisplay();

  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(textX, 0);
  display.println(F("Knob"));
  display.setCursor(80, 0);
  display.println(g_currKnobAngle);

  unsigned char *bmp;
  String lockStateString;
  if (g_currState == LOCKED)
  {
    display.drawBitmap(
        logoX - (ICON_BMP_WIDTH) / 2,
        (display.height() - ICON_BMP_HEIGHT) / 2,
        lock_bmp, ICON_BMP_WIDTH, ICON_BMP_HEIGHT, 1);
    lockStateString = "LOCKED";
  }
  else
  {
    display.drawBitmap(
        logoX - (ICON_BMP_WIDTH) / 2,
        (display.height() - ICON_BMP_HEIGHT) / 2,
        unlock_bmp, ICON_BMP_WIDTH, ICON_BMP_HEIGHT, 1);
    lockStateString = "Unlocked";
  }
  display.setTextSize(2);
  display.setCursor(textX, textY);
  display.println(lockStateString);
  display.dim(false);

  display.display();
}

void printBytes(byte toPrint[])
{
  Serial.print(F("0x"));
  for (uint8_t i = 0; i < 4; i++)
  {
    Serial.print(toPrint[i], HEX);
  }
  Serial.println(F(""));
}

void showCardDetails()
{
  Serial.print(F("Master Card ---- "));
  for (uint8_t i = 0; i < 4; i++)
  {                                                                  // Read Master Card's UID from EEPROM
    masterCard[i] = EEPROM.read(i + RFID_EEPROM_MASTER_CARD_OFFSET); // Write it to masterCard
  }
  printBytes(masterCard);
  Serial.println(F("Slave cards >> "));
  for (uint8_t slotIndex = 0; slotIndex < RFID_EEPROM_SLAVE_CARD_SLOTS; ++slotIndex)
  {
    byte thisSlave[4];
    for (uint8_t i = 0; i < 4; ++i)
    {
      thisSlave[i] = EEPROM.read(RFID_EEPROM_SLAVE_CARD_START + (slotIndex * RFID_EEPROM_SLAVE_CARD_SLOT_OFFSET) + i);
    }
    printBytes(thisSlave);
  }
}

bool bufferCmp(byte a[], byte b[])
{
  for (uint8_t k = 0; k < 4; k++)
  { // Loop 4 times
    if (a[k] != b[k])
    { // IF a != b then false, because: one fails, all fail
      return false;
    }
  }
  return true;
}

void copyBytes(byte *src, byte *dst)
{
  for (uint8_t k = 0; k < 4; k++)
  { // Loop 4 times
    dst[k] = src[k];
  }
}

void readCardFromEEPROM(uint8_t address, byte *dst)
{
  for (uint8_t i = 0; i < 4; i++)
  {                                    // Read Master Card's UID from EEPROM
    dst[i] = EEPROM.read(i + address); // Write it to masterCard
  }
}

bool isMaster(byte test[])
{
  return bufferCmp(test, masterCard) &&
         !bufferCmp(test, ZEROS) &&
         !bufferCmp(test, ONES);
}

bool isSlave(byte test[])
{
  if (bufferCmp(test, ZEROS) || bufferCmp(test, ONES))
  {
    return false;
  }
  for (uint8_t slotIndex = 0; slotIndex < RFID_EEPROM_SLAVE_CARD_SLOTS; ++slotIndex)
  {
    byte thisSlave[4];
    for (uint8_t i = 0; i < 4; ++i)
    {
      thisSlave[i] = EEPROM.read(RFID_EEPROM_SLAVE_CARD_START + (slotIndex * RFID_EEPROM_SLAVE_CARD_SLOT_OFFSET) + i);
    }
    if (bufferCmp(thisSlave, test))
    {
      return true;
    }
  }
  return false;
}

void eraseEEPROM()
{
  for (uint16_t x = 0; x < EEPROM.length(); x = x + 1)
  { //Loop end of EEPROM address
    if (EEPROM.read(x) == 0)
    { //If EEPROM address 0
      // do nothing, already clear, go to the next address in order to save time and reduce writes to EEPROM
    }
    else
    {
      EEPROM.write(x, 0); // if not write 0 to clear, it takes 3.3mS
    }
  }
  rfidMode = RFID_MODE_REGISTERING_MASTER;
}

void retrieveRFIDData()
{
  Serial.println(F("Retrieving RFID data"));
  // eraseEEPROM();
  showCardDetails();

  if (EEPROM.read(1) != MASTER_CARD_DEFINED_CODE)
  {
    Serial.println(F("No Master Card Defined, scan a PICC to define as Master Card"));
    rfidMode = RFID_MODE_REGISTERING_MASTER;
    eraseEEPROM();
  }
  else
  {
    Serial.println(F("RIFD_MODE_IDLE"));
    rfidMode = RFID_MODE_IDLE;
  }
}

void tryRegisterMasterCard()
{
  if (cardAttemptsCount == 0)
  {
    Serial.println(F("Card detected. Please scan 2 more times to confirm card as master."));
    copyBytes(readCard, cardCandidates[0]);
    Serial.println(F("--- Master Card Registration first attempt "));
    ++cardAttemptsCount;
  }
  else
  {
    Serial.println(F("Comparing cards"));
    printBytes(readCard);
    printBytes(cardCandidates[0]);
    if (bufferCmp(readCard, cardCandidates[0]))
    { // check against the first entry
      copyBytes(readCard, cardCandidates[cardAttemptsCount]);

      ++cardAttemptsCount;
      Serial.print(F("--- Master Card Registration attempt "));
      Serial.println(cardAttemptsCount);
    }
    else
    {
      Serial.println(F("Different card detected. Please scan 2 more times to confirm card as master."));
      // not the same card as previous
      copyBytes(readCard, cardCandidates[0]);
    }
  }

  if (cardAttemptsCount >= CARD_ATTEMPT_MAX_COUNT)
  {
    // 3 consecutive attempts
    Serial.print(F("Confirmed Master Card: "));
    printBytes(readCard);

    // Register this card as master
    for (uint8_t j = 0; j < 4; j++)
    {                                                                // Loop 4 times
      EEPROM.write(j + RFID_EEPROM_MASTER_CARD_OFFSET, readCard[j]); // Write scanned PICC's UID to EEPROM, start from address 3
    }
    EEPROM.write(1, 0x8F); // Write to EEPROM we defined Master Card.
    readCardFromEEPROM(RFID_EEPROM_MASTER_CARD_OFFSET, masterCard);
    rfidMode = RFID_MODE_IDLE;
    showCardDetails();
  }
}

void tryRegisterSlaveCard()
{
  if (isMaster(readCard))
  {
    Serial.println(F("Master Card detected. Slave Card registration cancelled."));
    cardAttemptsCount = 0;
    rfidMode = RFID_MODE_IDLE;
    return;
  }

  if (cardAttemptsCount == 0)
  {
    Serial.println(F("Card detected. Please scan 2 more times to confirm card as slave."));
    copyBytes(readCard, cardCandidates[0]);
    Serial.println(F("--- Slave Card Registration first attempt "));
    ++cardAttemptsCount;
    registerSlaveCardTimeout = g_currTime + RFID_SLAVE_CARD_REGISTRATION_TIMEOUT;
  }
  else
  {
    if (bufferCmp(readCard, cardCandidates[0]))
    { // check against the first entry
      copyBytes(readCard, cardCandidates[cardAttemptsCount]);

      ++cardAttemptsCount;
      Serial.print(F("--- Slave Card Registration attempt "));
      Serial.println(cardAttemptsCount);
      registerSlaveCardTimeout = g_currTime + RFID_SLAVE_CARD_REGISTRATION_TIMEOUT;
    }
    else
    {
      Serial.println(F("Different card detected. Scan the master card to restart process."));
      cardAttemptsCount = 0;
      rfidMode = RFID_MODE_IDLE;
    }
  }

  if (cardAttemptsCount >= CARD_ATTEMPT_MAX_COUNT)
  {
    // 3 consecutive attempts
    rfidMode = RFID_MODE_IDLE;
    int slotIndex = EEPROM.read(RFID_EEPROM_SLAVE_CARD_NEXT_FREE_SLOT);

    for (int i = 0; i < 4; ++i)
    {
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

bool peekBLESerial()
{
  if (btSerial.available())
  {
    btSerial.readBytes(readCard, 4);
    printBytes(readCard);
    return true;
  }
  else if (Serial.available())
  {
    byte tempReadCard[8];
    Serial.readBytes(tempReadCard, 8);
    bool isCard = true;
    for (int i = 0; i < 4; ++i)
    {
      if (tempReadCard[i] != 0xf7)
      {
        isCard = false;
      }
    }
    if (!isCard)
    {
      return false;
    }
    Serial.println(F("Card scan detected"));
    for (int i = 4; i < 8; ++i)
    {
      readCard[i - 4] = tempReadCard[i];
    }
    printBytes(readCard);
    return true;
  }
  else
  {
    return false;
  }
}

void bleLoop()
{
  if (g_currTime - rfidLastTime < RFID_MS)
  {
    return;
  }

  if (g_currSequenceStage != SEQUENCE_IDLE)
  {
    return;
  }

  // Timeout for slave registration
  if (rfidMode == RFID_MODE_REGISTERING_SLAVE)
  {
    if (g_currTime >= registerSlaveCardTimeout)
    {
      Serial.println(F("Slave card registration timed-out. Please scan the master card to try again."));
      cardAttemptsCount = 0;
      rfidMode = RFID_MODE_IDLE;
    }
  }

  if (peekBLESerial())
  {
    switch (rfidMode)
    {
    case RFID_MODE_REGISTERING_MASTER:
    {
      tryRegisterMasterCard();
      return;
    }
    case RFID_MODE_REGISTERING_SLAVE:
    {
      tryRegisterSlaveCard();
      return;
    }
    case RFID_MODE_IDLE:
    default:
    {
      if (isMaster(readCard))
      {
        Serial.println(F("Master card detected! Switch mode to Slave card registration."));
        cardAttemptsCount = 0;
        rfidMode = RFID_MODE_REGISTERING_SLAVE;
        registerSlaveCardTimeout = g_currTime + RFID_SLAVE_CARD_REGISTRATION_TIMEOUT;
      }
      else if (isSlave(readCard))
      {
        Serial.print(F("This is a slave? "));
        printBytes(readCard);
        if (g_currState == LOCKED)
        {
          Serial.println(F("RFID authorized, processing UNLOCK"));
          applyLockIntent(UNLOCKED);
        }
        else
        {
          Serial.println(F("RFID authorized, processing LOCK"));
          applyLockIntent(LOCKED);
        }
      }
      break;
    }
    }
  }
}

// the loop function runs over and over again forever
void loop()
{
  g_currTime = millis();
  reconcileLockState();
  reconcileOLEDDisplay();
  if (g_currSequenceStage == SEQUENCE_IDLE)
  {
    buttonLoop();
    bleLoop();
  }
  else
  {
    performSequenceActions();
  }
#ifdef IR_ENABLED
  irLoop();
#endif
#ifdef RING_ENABLED
  ringLoop();
#endif
#ifdef BUZZER_ENABLED
  soundLoop();
#endif
}

void initBLE()
{
  Serial.println(F("initializing BLE"));

  btSerial.begin(9600);
}

void initADXL()
{
  pinMode(PIN_ADXL, INPUT);
  int knobAngle = readADXL();
  if (knobAngle >= LOCKED_MIN_ANGLE)
  {
    g_currState = LOCKED;
    g_intentState = LOCKED;
    Serial.println(F("Initial state: LOCKED"));
  }
  else
  {
    g_currState = UNLOCKED;
    g_intentState = UNLOCKED;
    Serial.println(F("Intial state: UNLOCKED"));
  }
}

void initOLED()
{
  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C))
  { // Address 0x3C for 128x32
    Serial.println(F("SSD1306 allocation failed"));
  }
}

// the setup function runs once when you press reset or power the board
void setup()
{
  Wire.begin();

  Serial.begin(9600);
  while (!Serial)
  {
    Serial.println(F("I2C Scanner"));
  }
  Serial.println(F("Serial READY!\n"));

  pinMode(PIN_LOCK_BUTTON, INPUT);

  initADXL();

  Serial.println(F("Init Linear Servo"));
  servoLinearArm.attach(PIN_SERVO_LINEAR, 530, 2600);
  servoLinearArm.write(SERVO_LINEAR_DISENGAGED_DEG);
  delay(1000);
#ifndef TEST_SERVO_LINEAR
  servoLinearArm.detach();
#endif

  Serial.println(F("Init Main Servo"));
  servoRotateArm.attach(PIN_SERVO_MAIN, 530, 2600);
  servoRotateArm.writeMicroseconds(SERVO_IDLE_FREQ);
  delay(1000);
#ifndef TEST_SERVO
  servoLinearArm.detach();
#endif
  retrieveRFIDData();
  initBLE();
  initOLED();

#ifdef BUZZER_ENABLED
  pinMode(PIN_BUZZER, OUTPUT);
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

#ifdef RING_ENABLED
  ringSleepTime = g_currTime + RING_SLEEP_TIMEOUT_MS;
#endif
}
