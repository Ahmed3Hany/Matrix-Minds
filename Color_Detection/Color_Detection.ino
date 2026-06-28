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

// ================================================================================ Color Sensor ========================================================================
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

void setup() {
  Serial.begin(115200);

  // ================= TCS3200 Color Sensor ====================
  pinMode(TCS_S0, OUTPUT);
  pinMode(TCS_S1, OUTPUT);
  pinMode(TCS_S2, OUTPUT);
  pinMode(TCS_S3, OUTPUT);
  pinMode(TCS_OUT, INPUT);
  digitalWrite(TCS_S0, HIGH);
  digitalWrite(TCS_S1, LOW);  // 20% frequency scaling

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
  
  // تشغيل و ايقاف السنسور
  static unsigned long lastTime = 0;
  static bool prevSensorEnabled = true;

  if (sensorEnabled && millis() - lastTime >= 1000)
  {
    lastTime = millis();

    RGBData raw = readRGB();

    int r, g, b;
    normalizeRGB(raw, r, g, b);

    String detected = detectColor(r, g, b, raw.clear);

    Blynk.virtualWrite(V14, detected);
  } else if (!sensorEnabled && prevSensorEnabled != sensorEnabled) {
    Blynk.virtualWrite(V14, "Sensor Off");
  }
  prevSensorEnabled = sensorEnabled;
}