#define BLYNK_TEMPLATE_ID "TMPL2TqOtArVz"
#define BLYNK_TEMPLATE_NAME "Matrix Minds"
#define BLYNK_AUTH_TOKEN "ebDTvON3KAjl5FToo8J9jhXjkUOy1AS5"

#define BLYNK_PRINT Serial

#include <Wire.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
#include <Adafruit_PWMServoDriver.h>
#include <Ultrasonic.h>

// ============== بيانات الـ WiFi ==============
char ssid[] = "POCO X3 NFC";  // Replace with your WiFi SSID
char pass[] = "poahmedco3";   // Replace with your WiFi Password

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
const int homeAngles[NUM_SERVOS] = { 100, 180, 160, 90, 40, 50 };

// ============== زوايا المسك (Pick Position) ==============
const int pickAngles[NUM_SERVOS] = { 100, 40, 150, 90, 80, 100 };
const int pickAnglesClose[NUM_SERVOS] = { 100, 40, 150, 90, 80, 50 };

// ============== زوايا المسك (Release Red Position) ==============
const int TurnReleaseAnglesRed[NUM_SERVOS] = { 180, 180, 160, 90, 40, 50 };
const int ReleaseAnglesRed[NUM_SERVOS] = { 180, 40, 150, 90, 80, 50 };
const int releaseAnglesRedOpen[NUM_SERVOS] = { 180, 40, 150, 90, 80, 100 };

// ============== زوايا المسك (Release Green Position) ==============
const int TurnReleaseAnglesGreen[NUM_SERVOS] = { 130, 180, 160, 90, 40, 50 };
const int ReleaseAnglesGreen[NUM_SERVOS] = { 130, 40, 150, 90, 80, 50 };
const int releaseAnglesGreenOpen[NUM_SERVOS] = { 130, 40, 150, 90, 80, 100 };

// ============== زوايا المسك (Release Blue Position) ==============
const int TurnReleaseAnglesBlue[NUM_SERVOS] = { 50, 180, 160, 90, 40, 50 };
const int ReleaseAnglesBlue[NUM_SERVOS] = { 50, 40, 150, 90, 80, 50 };
const int releaseAnglesBlueOpen[NUM_SERVOS] = { 50, 40, 150, 90, 80, 100 };

// ============== زوايا المسك (Release Other Color Position) ==============
const int TurnReleaseAnglesOther[NUM_SERVOS] = { 0, 180, 160, 90, 40, 50 };
const int ReleaseAnglesOther[NUM_SERVOS] = { 0, 40, 150, 90, 80, 50 };
const int releaseAnglesOtherOpen[NUM_SERVOS] = { 0, 40, 150, 90, 80, 100 };

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

// ================================================================================ Robot Arm ===============================================================================
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

//================================================================ Blynk Virtual Pin Handlers (الأزرار الـ 12)===================================================================
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

BLYNK_WRITE(V21){
  int pressed = param.asInt();
  if (pressed == 1 && sensorEnabled == true) {
    
    RGBData raw = readRGB();
    int r, g, b;
    normalizeRGB(raw, r, g, b);
    String detected = detectColor(r, g, b, raw.clear);

    moveArmSmoothTo(pickAngles);
    delay(500);
    moveArmSmoothTo(pickAnglesClose);
    delay(500);
    moveArmSmoothTo(homeAngles);
    delay(500);

    if(detected == "RED"){
      moveArmSmoothTo(TurnReleaseAnglesRed);
      delay(500);
      moveArmSmoothTo(ReleaseAnglesRed);
      delay(500);
      moveArmSmoothTo(releaseAnglesRedOpen);
      delay(500);
    }else if(detected == "GREEN"){
      moveArmSmoothTo(TurnReleaseAnglesGreen);
      delay(500);
      moveArmSmoothTo(ReleaseAnglesGreen);
      delay(500);
      moveArmSmoothTo(releaseAnglesGreenOpen);
      delay(500);
    }else if(detected == "BLUE"){
      moveArmSmoothTo(TurnReleaseAnglesBlue);
      delay(500);
      moveArmSmoothTo(ReleaseAnglesBlue);
      delay(500);
      moveArmSmoothTo(releaseAnglesBlueOpen);
      delay(500);
    }else{
      moveArmSmoothTo(TurnReleaseAnglesOther);
      delay(500);
      moveArmSmoothTo(ReleaseAnglesOther);
      delay(500);
      moveArmSmoothTo(releaseAnglesOtherOpen);
      delay(500);
    }

    moveArmSmoothTo(homeAngles);

  }
}

//===================================================================================================== SETUP ===================================================================

void setup() {
  Serial.begin(115200);

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
  
  // -------------------------------------------------------
  // أول ما الدراع يشتغل، يروح لزاوية البداية بشكل ناعم.
  // -------------------------------------------------------
  moveArmSmoothTo(homeAngles);


  // ================= Blynk Mobile App Config ====================
  // اتصال بالـ WiFi و Blynk
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);

  Serial.println("Connecting to Blynk...");
  while (!Blynk.connected()) {
    Blynk.run();
    delay(100);
  }
  Serial.println("Connected!");
}

void loop() {
  Blynk.run();
  timer.run();

  // خطوات حركة الذراع
  unsigned long now = millis();
  if (now - lastStepTime >= LOOP_DELAY_MS) {
    lastStepTime = now;
    updateHoldDirections();
    applyHeldMovement();
  }
}