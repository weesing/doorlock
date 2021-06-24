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
int gServoCurrFreq          = g_SERVO_IDLE_FREQ;
Servo servoRotateArm;

///////////////////////// ADXL
int g_LOCKED_MIN_ANGLE      = 580;
int g_UNLOCKED_MAX_ANGLE    = 436;
int g_ADXL_READ_COUNT = 50;
uint32_t g_currKnobAngle = -1;

//////////////////////////////// LINEAR SERVO
int g_SERVO_LINEAR_ENGAGED_DEG      = 77;
int g_SERVO_LINEAR_DISENGAGED_DEG   = 45;
int g_SERVO_LINEAR_STEP             = 4;
int g_SERVO_LINEAR_MS               = 10;
Servo servoLinearArm;
uint16_t servoLinearArmTarget       = g_SERVO_LINEAR_DISENGAGED_DEG;
uint16_t servoLinearArmCurr         = g_SERVO_LINEAR_DISENGAGED_DEG;
unsigned long servoLinearLastTime = 0;

///////////////////////// BLE

///////////////////////// OLED Display
#define SCREEN_WIDTH    128
#define SCREEN_HEIGHT   64
uint8_t g_DEBUG_DISPLAY = 0;

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET      -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS  0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
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

int8_t g_currSeqStage = SEQUENCE_IDLE;

uint8_t g_currState = UNKNOWN;
uint8_t g_intentState = UNKNOWN;
