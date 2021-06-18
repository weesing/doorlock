////////////////////////////////////////////// All required libraries
#include <EEPROM.h> // We are going to read and write PICC's UIDs from/to EEPROM
#include <SoftwareSerial.h>
#include <Servo.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

////////////////////////////////////////////// End required libraries

// Lock status
#define LOCKED 1
#define UNLOCKED 0
#define UNKNOWN -1

//PWM
//3, 5, 6, 9, 10, and 11
#define PIN_SERVO_MAIN 2 // MG996
#define PIN_LOCK_BUTTON 3
#define PIN_SERVO_LINEAR 4 // SG90
#define PIN_ADXL A0        // GY-61 ADXL335

///////////////////////////////// BUTTON
#define PRESSED_COOLDOWN_MS 2000
unsigned long pressedCooldownLastTime;
bool buttonCoolingDown = false;
#define BUTTON_MS 100
unsigned long buttonLastTime;

//////////////////////////////// MAIN SERVO
int g_SERVO_UNLOCK_FREQ     = 1800;
int g_SERVO_IDLE_FREQ       = 1500;
int g_SERVO_LOCK_FREQ       = 1200;
Servo servoRotateArm;

///////////////////////// ADXL
#define LOCKED_MIN_ANGLE 580
#define UNLOCKED_MAX_ANGLE 436
#define ADXL_READ_COUNT 50

//////////////////////////////// LINEAR SERVO
#define SERVO_LINEAR_ENGAGED_DEG 77
#define SERVO_LINEAR_DISENGAGED_DEG 45
#define SERVO_LINEAR_STEP 4
#define SERVO_LINEAR_MS 10
Servo servoLinearArm;
uint16_t servoLinearArmTarget = SERVO_LINEAR_DISENGAGED_DEG;
uint16_t servoLinearArmCurr = SERVO_LINEAR_DISENGAGED_DEG;
unsigned long servoLinearLastTime = 0;

///////////////////////// BLE
#include "secret_defines.h"

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
  uint8_t readCount = 0;
  int total = 0;
  for (readCount = 0; readCount < ADXL_READ_COUNT; ++readCount)
  {
    int curr = analogRead(PIN_ADXL);
    total += curr;
  }
  int yRot = total / ADXL_READ_COUNT;
  // yRot = map(yRot, 0, 1023, 0, 255);
//   Serial.println(yRot);
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
      servoRotateArm.writeMicroseconds(g_SERVO_LOCK_FREQ);
    }
    else if (g_intentState == UNLOCKED)
    {
      servoRotateArm.writeMicroseconds(g_SERVO_UNLOCK_FREQ);
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
      servoRotateArm.writeMicroseconds(g_SERVO_IDLE_FREQ);
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
    Serial.println("State changed to LOCKED.");
    g_currState = LOCKED;
  }
  else if (g_currState == LOCKED && g_currKnobAngle <= UNLOCKED_MAX_ANGLE)
  {
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

void printSettingsCommand(const String command, const String payload) {
  Serial.print("SET ");
  Serial.print(command);
  Serial.print(" = ");
  Serial.println(payload);
}

void bleLoop()
{
  while (Serial.available())
  {
    byte b[1];
    Serial.readBytes(b, 1);
    char c = (char)(b[0]);
    if (c != '<') {
      Serial.print(c);
    }
    else {
      // ########## Read start tag ##########
      String command = Serial.readStringUntil('>');
      
      // ########## Read payload ##########
      String payload = Serial.readStringUntil(';');

      // settings commands
      if (command.equals("m_xlk")) {
        // main servo / unlock frequency
        printSettingsCommand(command, payload);
      }
      else if (command.equals("m_lk")) {
        // main servo / lock frequency
        printSettingsCommand(command, payload);
      }
      else if (command.equals("m_idl")) {
        // main servo / idle frequency
        printSettingsCommand(command, payload);
      }
      else if (command.equals("l_en")) {
        // linear servo / engaged angle
        printSettingsCommand(command, payload);
      }
      else if (command.equals("l_xen")) {
        // linear servo / disengage angle
        printSettingsCommand(command, payload);
        break;
      }
      else if (command.equals("l_step")) {
        // linear servo / steps
        printSettingsCommand(command, payload);
      }
      else if (command.equals("l_ms")) {
        // linear servo / ms
        printSettingsCommand(command, payload);
      }
      else if (command.equals("a_rdct")) {
        // ADXL read sample count
        printSettingsCommand(command, payload);
      }
      else if (command.equals("a_lk")) {
        // ADXL / lock angle
        printSettingsCommand(command, payload);
      }
      else if (command.equals("a_xlk")) {
        // ADXL / unlock angle
        printSettingsCommand(command, payload);
      }
      else if (command.equals("lock")) {
        Serial.print("LOCK ");
        if (payload.equals(SECRET_KEY)) {
          if (g_currSequenceStage == SEQUENCE_IDLE) {
            Serial.println("Authorized");
          }
          else {
            Serial.println("Authorized, but device busy");
          }
        }
        else {
          Serial.print(payload);
          Serial.println(" <-- Unauthorized");
        }
      }
    }
  }
}

// the loop function runs over and over again forever
void loop()
{
  g_currTime = millis();
  bleLoop();
  reconcileLockState();
  reconcileOLEDDisplay();
  if (g_currSequenceStage == SEQUENCE_IDLE)
  {
    buttonLoop();
  }
  else
  {
    performSequenceActions();
  }
}

void initBLE() {
  Serial.println("<request_data>");
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

  initBLE();
  initADXL();

  Serial.println(F("Init Linear Servo"));
  Serial.print(SERVO_LINEAR_DISENGAGED_DEG);
  Serial.print(F("--"));
  Serial.println(SERVO_LINEAR_ENGAGED_DEG);
  servoLinearArm.attach(PIN_SERVO_LINEAR, 530, 2600);
  servoLinearArm.write(SERVO_LINEAR_DISENGAGED_DEG);
  delay(1000);
  
  servoLinearArm.detach();

  Serial.println(F("Init Main Servo"));
  servoRotateArm.attach(PIN_SERVO_MAIN, 530, 2600);
  servoRotateArm.writeMicroseconds(g_SERVO_IDLE_FREQ);
  delay(1000);
  
  servoRotateArm.detach();
  
  initOLED();
}
