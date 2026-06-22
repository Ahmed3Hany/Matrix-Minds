#define BLYNK_TEMPLATE_ID "TMPL2TqOtArVz"
#define BLYNK_TEMPLATE_NAME "Matrix Minds"
#define BLYNK_AUTH_TOKEN "ebDTvON3KAjl5FToo8J9jhXjkUOy1AS5"

#define BLYNK_PRINT Serial

#include <Wire.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
#include <Adafruit_PWMServoDriver.h>

// ============== بيانات الـ WiFi ==============
char ssid[] = "POCO X3 NFC";  // Replace with your WiFi SSID
char pass[] = "poahmedco3";   // Replace with your WiFi Password

// ============== Car Motor Driver L298N ==============
#define ENA 23
#define ENB 32
#define IN1 16
#define IN2 17
#define IN3 18
#define IN4 19
const int Car_freq = 1000;
const int resolution = 8;

// ============== Color Sensor TCS3200 ==============
#define TCS_S0 14
#define TCS_S1 27
#define TCS_S2 25
#define TCS_S3 26
#define TCS_OUT 33
// ============== SETTINGS

// Frequency scaling = 20%
#define NUM_SAMPLES 15         // Number of samples for averaging
#define READ_DELAY 3           // Small delay between readings
#define PULSE_TIMEOUT 50000UL  // pulseIn timeout (microseconds)

// Enable / Disable serial debug
#define DEBUG_MODE true

// ============== RGB STRUCT
struct RGBData {
  uint32_t r;
  uint32_t g;
  uint32_t b;
  uint32_t clear;
};

// ========================================================== Robot Arm Config
// ============== أبناس I2C بتاعة الـ PCA9685 ==============
const uint8_t I2C_SDA_PIN = 21;
const uint8_t I2C_SCL_PIN = 22;

// عنوان الـ PCA9685 الافتراضي (لو غيرته بالـ solder jumpers على
// البورد، عدّل هنا. الافتراضي غالبًا 0x40)
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

// تردد الـ PWM للسيرفوهات (سيرفوهات عادية بتشتغل على 50Hz)
const int SERVO_FREQ = 50;

// ============== عدد السيرفوهات ==============
const uint8_t NUM_SERVOS = 6;

// ============== قنوات الـ PCA9685 (Channel 0 -> 15) ==============
const uint8_t servoChannel[NUM_SERVOS] = { 0, 2, 4, 6, 8, 10 };

// ============== زوايا البداية (Home Position) ==============
const int homeAngles[NUM_SERVOS] = { 90, 90, 90, 90, 90, 30 };

// ============== حدود كل سيرفو (Min / Max) بالدرجات ==============
const int minAngles[NUM_SERVOS] = { 0, 0, 0, 0, 0, 25 };
const int maxAngles[NUM_SERVOS] = { 180, 180, 180, 180, 180, 100 };

// ============== معايرة النبضة (Pulse) لكل سيرفو ==============
const int SERVO_MIN_PULSE[NUM_SERVOS] = { 102, 102, 102, 102, 102, 102 };  // ~0.5ms
const int SERVO_MAX_PULSE[NUM_SERVOS] = { 512, 512, 512, 512, 512, 512 };  // ~2.5ms

// ============== إعدادات الحركة ==============
const float STEP_DEGREE = 0.6;  // خطوة الحركة طول ما الزرار مضغوط
const int LOOP_DELAY_MS = 8;    // تأخير بين كل خطوة أثناء الإمساك بالزرار

const int HOME_STEP_DEGREE = 1;
const int HOME_STEP_DELAY_MS = 12;

// الزاوية الحالية الفعلية لكل سيرفو
float currentAngle[NUM_SERVOS];

// holdDirection[i]: +1 (زيادة) / -1 (نقصان) / 0 (واقف)
int8_t holdDirection[NUM_SERVOS] = { 0, 0, 0, 0, 0, 0 };

// حالة كل زرار: [i][0]=زرار الناقص, [i][1]=زرار الزيادة
bool buttonState[NUM_SERVOS][2] = { { false, false }, { false, false }, { false, false }, { false, false }, { false, false }, { false, false } };

unsigned long lastStepTime = 0;

// ================================================================================================================ نهاية التوصيلات

// =================================================== Car Controls ===================================================
BLYNK_WRITE(V16) {  // زرار يتحرك قدام
  int state = param.asInt();

  if (state == 1) {  // لما تضغط
    MV_Forward();
  } else {
    Stop_Car();
  }
}

BLYNK_WRITE(V17) {  // زرار يرجع ورا
  int state = param.asInt();

  if (state == 1) {  // لما تضغط
    MV_Backward();
  } else {
    Stop_Car();
  }
}

BLYNK_WRITE(V19) {  // زرار يلف يمين
  int state = param.asInt();

  if (state == 1) {  // لما تضغط
    T_Right();
  } else {
    Stop_Car();
  }
}

BLYNK_WRITE(V18) {  // زرار يلف شمال
  int state = param.asInt();

  if (state == 1) {  // لما تضغط
    T_Left();
  } else {
    Stop_Car();
  }
}

int car_speed = 255;
BLYNK_WRITE(V15) {  // التحكم في السرعة
  car_speed = param.asInt();
  ledcWrite(ENA, car_speed);
  ledcWrite(ENB, car_speed);
}

void MV_Forward() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void MV_Backward() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void T_Right() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void T_Left() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void Stop_Car() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

// =================================================== Color Sensor ===================================================
uint32_t readChannel(bool s2, bool s3) {

  digitalWrite(TCS_S2, s2);
  digitalWrite(TCS_S3, s3);

  delay(READ_DELAY);

  uint32_t total = 0;
  uint8_t validReads = 0;

  for (int i = 0; i < NUM_SAMPLES; i++) {

    uint32_t pulse = pulseIn(TCS_OUT, LOW, PULSE_TIMEOUT);

    // Ignore timeout / invalid reads
    if (pulse > 0) {
      total += pulse;
      validReads++;
    }

    delay(1);
  }

  // Prevent division by zero
  if (validReads == 0) {
    return 0;
  }

  return total / validReads;
}


/* ===================== FUNCTION: READ RGB ===================== */
/*
 * Reads all color channels
 */
RGBData readRGB() {

  RGBData color;

  // RED filter
  color.r = readChannel(LOW, LOW);

  // BLUE filter
  color.b = readChannel(LOW, HIGH);

  // GREEN filter
  color.g = readChannel(HIGH, HIGH);

  // CLEAR filter
  color.clear = readChannel(HIGH, LOW);

  return color;
}


/* ===================== FUNCTION: NORMALIZE ===================== */
/*
 * Convert raw frequency values into normalized RGB
 * Lower pulse value = stronger color
 */
void normalizeRGB(RGBData &c, int &r, int &g, int &b) {

  // Prevent divide-by-zero
  if (c.r == 0 || c.g == 0 || c.b == 0) {
    r = g = b = 0;
    return;
  }

  // Invert values because TCS3200 works inversely
  float rf = 1000.0 / c.r;
  float gf = 1000.0 / c.g;
  float bf = 1000.0 / c.b;

  // Find max channel
  float maxVal = max(rf, max(gf, bf));

  // Normalize to 0–255
  r = (rf / maxVal) * 255;
  g = (gf / maxVal) * 255;
  b = (bf / maxVal) * 255;
}


/* ===================== FUNCTION: DETECT COLOR ===================== */
/*
 * Smart detection using:
 * - Relative RGB comparison
 * - Brightness checking
 * - Noise filtering
 */
String detectColor(int r, int g, int b, uint32_t clearVal) {

  int brightness = (r + g + b) / 3;

  // WHITE
  if (brightness > 170 && abs(r - g) < 35 && abs(g - b) < 35 && abs(r - b) < 35) {
    return "WHITE";
  }

  // ================= RED =================
  if (r > g * 1.4 && r > b * 1.4 && r > 100) {
    return "RED";
  }

  // ================= GREEN =================
  if (g > r + 15 && g > b + 15 && g > 90) {
    return "GREEN";
  }

  // ================= BLUE =================
  if (b > r * 1.3 && b > g * 1.3 && b > 100) {
    return "BLUE";
  }

  // ================= UNKNOWN =================
  return "UNKNOWN";
}

bool sensorEnabled = false;

BLYNK_WRITE(V13)
{
    sensorEnabled = param.asInt();
}

// ============== Robot Arm ==============
// ---------------------------------------------------------
// تحويل زاوية (0-180) لقيمة Pulse تفهمها PCA9685، وكتابتها
// مباشرة على القناة بتاعة السيرفو ده
// ---------------------------------------------------------
void writeServoAngle(uint8_t servoIndex, float angle) {
  int pulse = map((int)round(angle), 0, 180,
                  SERVO_MIN_PULSE[servoIndex], SERVO_MAX_PULSE[servoIndex]);
  pwm.setPWM(servoChannel[servoIndex], 0, pulse);
}

// ---------------------------------------------------------
// دالة مساعدة: التأكد ان الزاوية جوه الحدود المسموح بيها
// ---------------------------------------------------------
float clampAngle(float value, int mn, int mx) {
  if (value < mn) return mn;
  if (value > mx) return mx;
  return value;
}

// ---------------------------------------------------------
// تحديث اتجاه الحركة لكل سيرفو بناءً على حالة الأزرار
// ---------------------------------------------------------
void updateHoldDirections() {
  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    bool minusPressed = buttonState[i][0];
    bool plusPressed = buttonState[i][1];

    if (minusPressed && !plusPressed) {
      holdDirection[i] = -1;
    } else if (plusPressed && !minusPressed) {
      holdDirection[i] = +1;
    } else {
      holdDirection[i] = 0;
    }
  }
}

// ---------------------------------------------------------
// تحريك أي سيرفو زرراره مضغوط خطوة صغيرة في الاتجاه المطلوب
// ---------------------------------------------------------
void applyHeldMovement() {
  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    if (holdDirection[i] == 0) continue;

    float newAngle = currentAngle[i] + (holdDirection[i] * STEP_DEGREE);
    newAngle = clampAngle(newAngle, minAngles[i], maxAngles[i]);

    if (newAngle != currentAngle[i]) {
      currentAngle[i] = newAngle;
      writeServoAngle(i, currentAngle[i]);
    }
  }
}

// ---------------------------------------------------------
// تحريك الدراع بالكامل لوضع معين (Home) بشكل ناعم ومتزامن
// ---------------------------------------------------------
void moveArmSmoothTo(const int targetSet[NUM_SERVOS]) {
  bool allReached = false;

  while (!allReached) {
    allReached = true;

    for (uint8_t i = 0; i < NUM_SERVOS; i++) {
      float diff = targetSet[i] - currentAngle[i];

      if (fabs(diff) < 0.5) {
        currentAngle[i] = targetSet[i];
        continue;
      }

      allReached = false;
      if (diff > 0) {
        currentAngle[i] += HOME_STEP_DEGREE;
        if (currentAngle[i] > targetSet[i]) currentAngle[i] = targetSet[i];
      } else {
        currentAngle[i] -= HOME_STEP_DEGREE;
        if (currentAngle[i] < targetSet[i]) currentAngle[i] = targetSet[i];
      }
      writeServoAngle(i, currentAngle[i]);
    }

    delay(HOME_STEP_DELAY_MS);
    Blynk.run();
  }
}

// ===========================================================
//          Blynk Virtual Pin Handlers (الأزرار الـ 12)
// ===========================================================
BLYNK_WRITE(V0) {
  buttonState[0][1] = param.asInt();
}  // Base +
BLYNK_WRITE(V1) {
  buttonState[0][0] = param.asInt();
}  // Base -

BLYNK_WRITE(V2) {
  buttonState[3][1] = param.asInt();
}  // Wrist Pitch +
BLYNK_WRITE(V3) {
  buttonState[3][0] = param.asInt();
}  // Wrist Pitch -

BLYNK_WRITE(V4) {
  buttonState[4][1] = param.asInt();
}  // Wrist Roll +
BLYNK_WRITE(V5) {
  buttonState[4][0] = param.asInt();
}  // Wrist Roll -

BLYNK_WRITE(V6) {
  buttonState[1][1] = param.asInt();
}  // Shoulder +
BLYNK_WRITE(V7) {
  buttonState[1][0] = param.asInt();
}  // Shoulder -

BLYNK_WRITE(V8) {
  buttonState[2][0] = param.asInt();
}  // Elbow -
BLYNK_WRITE(V9) {
  buttonState[2][1] = param.asInt();
}  // Elbow +

BLYNK_WRITE(V10) {
  buttonState[5][1] = param.asInt();
}  // Gripper +
BLYNK_WRITE(V11) {
  buttonState[5][0] = param.asInt();
}  // Gripper -

// زرار الـ Home (مجرد ضغطة عادية، مش Hold)
BLYNK_WRITE(V12) {
  int pressed = param.asInt();
  if (pressed == 1) {
    moveArmSmoothTo(homeAngles);
  }
}

// ===========================================================
//                          SETUP
// ===========================================================
void setup() {
  Serial.begin(115200);

  // ================= Car Motor Driver L298N PinModes ====================
  ledcAttach(ENA, Car_freq, resolution);  //Control Speed of Motor A
  ledcAttach(ENB, Car_freq, resolution);  //Control Speed of Motor B
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  Blynk.virtualWrite(V13, 255);

  // ================= TCS3200 Color Sensor ====================
  pinMode(TCS_S0, OUTPUT);
  pinMode(TCS_S1, OUTPUT);
  pinMode(TCS_S2, OUTPUT);
  pinMode(TCS_S3, OUTPUT);
  pinMode(TCS_OUT, INPUT);
  digitalWrite(TCS_S0, HIGH);
  digitalWrite(TCS_S1, LOW);  // 20% frequency scaling

  // ================= PCA9685 Robot Arm Driver ====================
  // بدء اتصال I2C على بنات 21 (SDA) و 22 (SCL)
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);

  pwm.begin();
  pwm.setOscillatorFrequency(27000000);  // الافتراضي لمعظم بوردات PCA9685
  pwm.setPWMFreq(SERVO_FREQ);
  delay(10);

  // وضع كل سيرفو على زاوية البداية مباشرة (قبل أي حركة ناعمة)
  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    currentAngle[i] = homeAngles[i];
    writeServoAngle(i, currentAngle[i]);
  }

  // ================= Blynk Mobile App Config ====================
  // اتصال بالـ WiFi و Blynk
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);

  Serial.println("Connecting to Blynk...");
  while (!Blynk.connected()) {
    Blynk.run();
    delay(100);
  }
  Serial.println("Connected!");

  // -------------------------------------------------------
  // أول ما الدراع يشتغل، يروح لزاوية البداية بشكل ناعم.
  // -------------------------------------------------------
  moveArmSmoothTo(homeAngles);

  Serial.println("Arm is at HOME position. Ready for joystick control.");
}

// ===========================================================
//                           LOOP
// ===========================================================
void loop() {
  Blynk.run();

  // خطوات حركة الذراع
  unsigned long now = millis();
  if (now - lastStepTime >= LOOP_DELAY_MS) {
    lastStepTime = now;
    updateHoldDirections();
    applyHeldMovement();
  }

  // تشغيل و ايقاف السنسور
  static unsigned long lastTime = 0;

    if (sensorEnabled && millis() - lastTime >= 1000)
    {
        lastTime = millis();

        RGBData raw = readRGB();

        int r, g, b;
        normalizeRGB(raw, r, g, b);

        String detected = detectColor(r, g, b, raw.clear);

        Blynk.virtualWrite(V14, detected);
    }
}
