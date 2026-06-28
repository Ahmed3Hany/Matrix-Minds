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

// ============== Car Motor Driver L298N ==============
#define ENA 23
#define ENB 32
#define IN1 16
#define IN2 17
#define IN3 18
#define IN4 19
const int Car_freq = 1000;
int car_speed = 180;
const int resolution = 8;

// ============== Ultrasonic Sensor HC-SR04 ==============
Ultrasonic ultrasonic(12, 13);
bool goingForward = false;
int distance = 0;
BlynkTimer timer;

// ================================================================================== Ultrasonic Function ======================================================================

void ReadDistance() {
  distance = ultrasonic.read();
  Blynk.virtualWrite(V20, distance);

  // لو العربية ماشية قدام وفيه عائق → وقف
  if (goingForward && distance <= 20) {
    Stop_Car();
  }
  // لو العائق اتشال والزرار لسه مضغوط → امشي
  else if (goingForward && distance > 20) {
    MV_Forward();
  }
}

// ==================================================================================== Car Controls ===========================================================================
// متغير بيحفظ حالة زرار الأمام

BLYNK_WRITE(V16) {
  goingForward = param.asInt(); // 1 = مضغوط، 0 = متلقيش
  if (!goingForward) Stop_Car();
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

BLYNK_WRITE(V15) {  // التحكم في السرعة
  car_speed = param.asInt();
  ledcWrite(ENA, car_speed);
  ledcWrite(ENB, car_speed);
}

void MV_Forward() {
  ledcWrite(ENA, car_speed);
  ledcWrite(ENB, car_speed);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void MV_Backward() {
  ledcWrite(ENA, car_speed);
  ledcWrite(ENB, car_speed);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void T_Right() {
  ledcWrite(ENA, car_speed);
  ledcWrite(ENB, car_speed);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void T_Left() {
  ledcWrite(ENA, car_speed);
  ledcWrite(ENB, car_speed);
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

void setup() {
  Serial.begin(115200);

  // ================= Car Motor Driver L298N PinModes ====================
  ledcAttach(ENA, Car_freq, resolution);  //Control Speed of Motor A
  ledcAttach(ENB, Car_freq, resolution);  //Control Speed of Motor B
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  ledcWrite(ENA, car_speed);
  ledcWrite(ENB, car_speed);
  // Ultrasonic
  timer.setInterval(50L ,ReadDistance); //الدالة هتشتغل كل 50 ميلي ثانية 

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
}