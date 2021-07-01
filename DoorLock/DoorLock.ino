////////////////////////////////////////////// All required libraries
#include <Servo.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include "defines.h"
#include "secret_defines.h"

////////////////////////////////////////////// End required libraries

void applyLockIntent(int intent)
{
  g_intentState = intent;
  if (g_currState == intent)
  {
    Serial.println(F("LCK_IGNORE"));
    return;
  }

  switch (intent)
  {
  case LOCKED:
    Serial.println(F("locking"));
    g_intentState = LOCKED;
    g_currSeqStage = SEQUENCE_INIT;
    break;
  case UNLOCKED:
    Serial.println(F("unlocking"));
    g_intentState = UNLOCKED;
    g_currSeqStage = SEQUENCE_INIT;
    break;
  }
}

void buttonLoop() {
  if (g_currTime - buttonLastTime < BUTTON_MS) {
    return;
  }

  if (g_currSeqStage != SEQUENCE_IDLE) {
    return;
  }

  buttonLastTime = g_currTime;
  if (buttonCoolingDown) {
    if (g_currTime - pressedCooldownLastTime > PRESSED_COOLDOWN_MS) {
      Serial.println(F("BTN_CD_EXP"));
      buttonCoolingDown = false;
    }
    else {
      return;
    }
  }
  else {
    int btnPressed = digitalRead(PIN_LOCK_BUTTON);
    if (btnPressed == HIGH) {
      buttonCoolingDown = true;
      pressedCooldownLastTime = g_currTime;
      if (g_currState == LOCKED) {
        Serial.println(F("Btn unlock"));
        applyLockIntent(UNLOCKED);
      }
      else {
        Serial.println(F("Btn lock"));
        applyLockIntent(LOCKED);
      }
    }
  }
}

bool moveLinearServo()
{
  if (g_currTime - servoLinearLastTime < g_SERVO_LINEAR_MS) {
    return false;
  }
  
  servoLinearLastTime = g_currTime;
  if (servoLinearArmCurr < servoLinearArmTarget) {
    if (servoLinearArmTarget - servoLinearArmCurr <= g_SERVO_LINEAR_STEP) {
      servoLinearArmCurr = servoLinearArmTarget;
    }
    else {
      servoLinearArmCurr += g_SERVO_LINEAR_STEP;
    }
    servoLinearArm.write(servoLinearArmCurr);
  }
  else {
    if (servoLinearArmCurr - servoLinearArmTarget <= g_SERVO_LINEAR_STEP) {
      servoLinearArmCurr = servoLinearArmTarget;
    }
    else {
      servoLinearArmCurr -= g_SERVO_LINEAR_STEP;
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
  for (readCount = 0; readCount < g_ADXL_READ_COUNT; ++readCount) {
    int curr = analogRead(PIN_ADXL);
    total += curr;
  }
  return total / g_ADXL_READ_COUNT;
}

void performSequenceActions()
{
  switch (g_currSeqStage)
  {
  case SEQUENCE_IDLE:
    break;
  case SEQUENCE_INIT:
    servoLinearArmTarget = g_SERVO_LINEAR_ENGAGED_DEG;
    g_currSeqStage = SEQUENCE_ENGAGE; // go to next stage immediately on next loop.
    servoLinearArm.attach(PIN_SERVO_LINEAR, 530, 2600);
    servoRotateArm.attach(PIN_SERVO_MAIN, 530, 2600);
    Serial.println(F("ENGAGE"));
    break;
  case SEQUENCE_ENGAGE:
    if (moveLinearServo())
    {
      g_currSeqStage = SEQUENCE_START_ACTION;
      Serial.println(F("START_ACTION"));
    }
    break;
  case SEQUENCE_START_ACTION:
    if (g_intentState == LOCKED) {
      gServoCurrFreq = g_SERVO_LOCK_FREQ;
      servoRotateArm.writeMicroseconds(g_SERVO_LOCK_FREQ);
    }
    else if (g_intentState == UNLOCKED) {
      gServoCurrFreq = g_SERVO_UNLOCK_FREQ;
      servoRotateArm.writeMicroseconds(g_SERVO_UNLOCK_FREQ);
    }
    g_currSeqStage = SEQUENCE_ACTION;
    Serial.println(F("ACTION"));
    break;
  case SEQUENCE_ACTION:
    if (g_currState == g_intentState) {
      gServoCurrFreq = g_SERVO_IDLE_FREQ;
      servoRotateArm.writeMicroseconds(g_SERVO_IDLE_FREQ);
      delay(100);
      servoRotateArm.detach();
      g_currSeqStage = SEQUENCE_DISENGAGE;
      Serial.println(F("DISENGAGE"));
      servoLinearArmTarget = g_SERVO_LINEAR_DISENGAGED_DEG;
      // delay to let linear arm have more current
      delay(g_SERVO_END_DELAY);
    }
    break;
  case SEQUENCE_DISENGAGE:
    if (moveLinearServo())
    {
      g_currSeqStage = SEQUENCE_END;
      Serial.println(F("END"));
      // delay a bit to let linear arm settle.
      delay(g_SERVO_LINEAR_END_DELAY);
      servoLinearArm.detach();
    }
    break;
  case SEQUENCE_END:
    g_currSeqStage = SEQUENCE_IDLE;
    Serial.println(F("IDLE"));
    break;
  }
}

void reconcileLockState()
{
  int yRot = readADXL();
  g_currKnobAngle = yRot;
  if (g_currState == UNLOCKED && g_currKnobAngle >= g_LOCKED_MIN_ANGLE)
  {
    Serial.println(F("CHG_LCK"));
    g_currState = LOCKED;
    servoRotateArm.writeMicroseconds(g_SERVO_IDLE_FREQ);
  }
  else if (g_currState == LOCKED && g_currKnobAngle <= g_UNLOCKED_MAX_ANGLE)
  {
    Serial.println(F("CHG_XLCK"));
    g_currState = UNLOCKED;
    servoRotateArm.writeMicroseconds(g_SERVO_IDLE_FREQ);
  }
}

void reconcileOLEDDisplay()
{
  int logoX = 8;
  int logoY = 15;
  int textX = 28;
  int textY = logoY - 5;
  int locX = 0;
  int locY = logoY - (ICON_BMP_HEIGHT / 2);

  display.clearDisplay();

  display.setTextSize(1);
  display.setTextColor(WHITE);
  
  unsigned char *bmp;
  String lockStateString;
    
  if (g_DEBUG_DISPLAY != 0) {
    locX = logoX - (ICON_BMP_WIDTH / 2);
  } else {
    locX = (SCREEN_WIDTH / 2) - (ICON_BMP_WIDTH / 2);
    locY += 4;
    textX = 20;
    textY = locY + 38;
  }

  if (g_currState == LOCKED)
  {
    if (g_DEBUG_DISPLAY == 0) {
      display.drawCircle(SCREEN_WIDTH / 2 - 1, logoY + 6, ICON_BMP_WIDTH, SSD1306_WHITE);
      textX = 32;
    }
    display.drawBitmap(
        locX,
        locY,
        lock_bmp, ICON_BMP_WIDTH, ICON_BMP_HEIGHT, 1);
    lockStateString = "LOCKED";
  }
  else
  {
    display.drawBitmap(
        locX,
        locY,
        unlock_bmp, ICON_BMP_WIDTH, ICON_BMP_HEIGHT, 1);
    lockStateString = "Unlocked";
  }
  display.setTextSize(2);
  display.setCursor(textX, textY);
  display.println(lockStateString);

  if (g_DEBUG_DISPLAY != 0) {
    int yRot = readADXL();
    display.setTextSize(1);
    display.setCursor(0, logoY + 14);
    display.print(F("States "));
    display.print(g_currState);
    display.print(F(" - "));
    display.println(g_intentState);
    display.print(F("Stage "));
    display.print(g_currSeqStage);
    display.print(F("  Knob "));
    display.println(g_currKnobAngle);
    display.print(F("Linear "));
    display.print(servoLinearArmCurr);
    display.print(F(" to "));
    display.println(servoLinearArmTarget);
    display.print(F("Servo "));
    display.println(gServoCurrFreq);
  }
  
  display.dim(false);

  display.display();
}

void printSettingsCommand(const String command, const String payload) {
  Serial.print(F("SET "));
  Serial.print(command);
  Serial.print(F(" = "));
  Serial.println(payload);
}

void sendValue(String key, String args) {
  Serial.print("<");
  Serial.print(key);
  Serial.print(">");
  if (key.equals("m_xlk")) {
    Serial.println(g_SERVO_UNLOCK_FREQ);
  }
  else if (key.equals("m_lk")) {
    Serial.println(g_SERVO_LOCK_FREQ);
  }
  else if (key.equals("m_idl")) {
    Serial.println(g_SERVO_IDLE_FREQ);
  }
  else if (key.equals("m_edel")) {
    Serial.println(g_SERVO_END_DELAY);
  }
  else if (key == "l_en") {
    Serial.println(g_SERVO_LINEAR_ENGAGED_DEG);
  }
  else if (key == "l_xen") {
    Serial.println(g_SERVO_LINEAR_DISENGAGED_DEG);
  }
  else if (key == "l_step") {
    Serial.println(g_SERVO_LINEAR_STEP);
  }
  else if (key == "l_ms") {
    Serial.println(g_SERVO_LINEAR_MS);
  }
  else if (key == "l_edel") {
    Serial.println(g_SERVO_LINEAR_END_DELAY);
  }
  else if (key == "a_rdct") {
    Serial.println(g_ADXL_READ_COUNT);
  }
  else if (key == "a_lk") {
    Serial.println(g_LOCKED_MIN_ANGLE);
  }
  else if (key == "a_xlk") {
    Serial.println(g_UNLOCKED_MAX_ANGLE);
  }
  else if (key == "o_dbg") {
    Serial.println(g_DEBUG_DISPLAY);
  }
}

void(* rebootFunc)(void) = 0; //reboot function @ address 0

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
      if (command.startsWith("get_")) {
        command.replace("get_", "");
        sendValue(command, payload);
      }
      else if (command.equals("m_xlk")) {
        // main servo / unlock frequency
        printSettingsCommand(command, payload);
        g_SERVO_UNLOCK_FREQ = payload.toInt();
      }
      else if (command.equals("m_lk")) {
        // main servo / lock frequency
        printSettingsCommand(command, payload);
        g_SERVO_LOCK_FREQ = payload.toInt();
      }
      else if (command.equals("m_idl")) {
        // main servo / idle frequency
        printSettingsCommand(command, payload);
        g_SERVO_IDLE_FREQ = payload.toInt();
      }
      else if (command.equals("m_edel")) {
        printSettingsCommand(command, payload);
        g_SERVO_END_DELAY = payload.toInt();
      }
      else if (command.equals("l_en")) {
        // linear servo / engaged angle
        printSettingsCommand(command, payload);
        g_SERVO_LINEAR_ENGAGED_DEG = payload.toInt();
      }
      else if (command.equals("l_xen")) {
        // linear servo / disengage angle
        printSettingsCommand(command, payload);
        g_SERVO_LINEAR_DISENGAGED_DEG = payload.toInt();
      }
      else if (command.equals("l_step")) {
        // linear servo / steps
        printSettingsCommand(command, payload);
        g_SERVO_LINEAR_STEP = payload.toInt();
      }
      else if (command.equals("l_ms")) {
        // linear servo / ms
        printSettingsCommand(command, payload);
        g_SERVO_LINEAR_MS = payload.toInt();
      }
      else if (command.equals("l_edel")) {
        printSettingsCommand(command, payload);
        g_SERVO_LINEAR_END_DELAY = payload.toInt();
      }
      else if (command.equals("a_rdct")) {
        // ADXL read sample count
        printSettingsCommand(command, payload);
        g_ADXL_READ_COUNT = payload.toInt();
      }
      else if (command.equals("a_lk")) {
        // ADXL / lock angle
        printSettingsCommand(command, payload);
        g_LOCKED_MIN_ANGLE = payload.toInt();
      }
      else if (command.equals("a_xlk")) {
        // ADXL / unlock angle
        printSettingsCommand(command, payload);
        g_UNLOCKED_MAX_ANGLE = payload.toInt();
      }
      else if (command.equals("o_dbg")) {
        printSettingsCommand(command, payload);
        g_DEBUG_DISPLAY = payload.toInt();
      }
      else if (command.equals("reboot")) {
        rebootFunc();
      }
      else if (command.equals("hb")) {
//        Serial.println(F("HB ping"));
      }
      else if (command.equals("lock")) {
        Serial.print(F("LOCK "));
        if (payload.equals(SECRET_KEY)) {
          if (g_currSeqStage == SEQUENCE_IDLE) {
            Serial.println(F("Authorized"));
            // Toggle the lock
            if (g_currState == LOCKED)
            {
              applyLockIntent(UNLOCKED);
              Serial.println("<status>0");
            }
            else
            {
              applyLockIntent(LOCKED);
              Serial.println("<status>1");
            }
          }
          else {
            Serial.println(F("Device busy"));
          }
        }
        else {
          Serial.print(payload);
          Serial.println(F("Unauthorized"));
        }
      }
      else if (command.equals("status")) {
        if (g_currState == LOCKED) {
          Serial.println("<status>1");
        }
        else {
          Serial.println("<status>0");
        }
      }
    }
  }
}

// the loop function runs over and over again forever
void loop()
{
  g_currTime = millis();
  
  reconcileLockState();
  if (g_currSeqStage == SEQUENCE_IDLE) {
    buttonLoop();
  }
  else {
    performSequenceActions();
  }
  
  bleLoop();
  reconcileOLEDDisplay();
}

void requestSettings() {
  Serial.println(F("Init BLE"));
  delay(500);
  Serial.println(F("<req_lock_data>"));
  delay(500);
}

void initADXL()
{
  Serial.println(F("Init ADXL"));
  pinMode(PIN_ADXL, INPUT);
  g_currKnobAngle = readADXL();
  if (g_currKnobAngle >= g_LOCKED_MIN_ANGLE)
  {
    g_currState = LOCKED;
    g_intentState = LOCKED;
    Serial.println(F("curr-LOCKED"));
  }
  else
  {
    g_currState = UNLOCKED;
    g_intentState = UNLOCKED;
    Serial.println(F("curr-UNLOCKED"));
  }
}

void initOLED()
{
  Serial.println(F("Init OLED"));
  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }

  // Clear the buffer
  display.clearDisplay();

  // Show the display buffer on the screen. You MUST call display() after
  // drawing commands to make them visible on screen!
  display.display();
  delay(2000);
    
  display.clearDisplay();

  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  display.println(F("Initializing..."));

  display.dim(false);

  display.display();
  
  Serial.println(F("OLED init"));
}

void setup()
{
  Serial.begin(9600);
  
  initOLED();
  
  while (!Serial) { }
  Serial.println(F("Ser READY!"));

  delay(100);
  pinMode(PIN_LOCK_BUTTON, INPUT);
  initADXL();
  delay(100);

  Serial.println(F("Init LServo"));
  servoLinearArm.attach(PIN_SERVO_LINEAR, 530, 2600);
  servoLinearArm.write(g_SERVO_LINEAR_DISENGAGED_DEG);
  delay(1000);
  
  servoLinearArm.detach();

  Serial.println(F("Init MServo"));
  servoRotateArm.attach(PIN_SERVO_MAIN, 530, 2600);
  servoRotateArm.writeMicroseconds(g_SERVO_IDLE_FREQ);
  delay(1000);
  
  servoRotateArm.detach();

  delay(1000);
  requestSettings();

  Serial.println(F("All done"));
}
