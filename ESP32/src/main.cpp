#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include "RTClib.h"
#include <LiquidCrystal_I2C.h>
#include <ESP32Servo.h>

// WiFi + MQTT config
#define WIFI_SSID       "YOUR_WIFI_NAME"
#define WIFI_PASSWORD   "YOUR_WIFI_PASSWORD"
#define TOKEN           "YOUR_ACCESS_TOKEN"
#define THINGSBOARD_SERVER "demo.thingsboard.io"
#define THINGSBOARD_PORT 1883

WiFiClient espClient;
PubSubClient client(espClient);

// MQ2, Buzzer
#define MQ2_PIN         34
#define BUZZER_PIN      25
#define GAS_THRESHOLD   2

// Ultrasonic + Servo
#define TRIG_IN         5
#define ECHO_IN         18
#define SERVO_IN_PIN    19
#define TRIG_OUT        17
#define ECHO_OUT        16
#define SERVO_OUT_PIN   4

// UART
#define RX_PIN          26
#define TX_PIN          27
HardwareSerial MySerial(2);

// RTC, LCD, Servo
RTC_DS3231 rtc;
LiquidCrystal_I2C lcd(0x27, 20, 4);
Servo servoright;
Servo servoleft;

bool Gateleft = false, Gateright = false;
int car = 0, motor = 0;
bool fullcar = false, fullmotor = false;
bool blinkcar = false, blinkmotor = false;

// ISR flags
volatile bool flag_isr12 = false, flag_isr13 = false, flag_isr14 = false, flag_isr23 = false;
unsigned long last_isr12_time = 0, last_isr13_time = 0, last_isr14_time = 0, last_isr23_time = 0;

// === ISR debounce ===
void IRAM_ATTR isr12() {
  if (millis() - last_isr12_time > 300) { flag_isr12 = true; last_isr12_time = millis(); }
}
void IRAM_ATTR isr13() {
  if (millis() - last_isr13_time > 300) { flag_isr13 = true; last_isr13_time = millis(); }
}
void IRAM_ATTR isr14() {
  if (millis() - last_isr14_time > 300) { flag_isr14 = true; last_isr14_time = millis(); }
}
void IRAM_ATTR isr23() {
  if (millis() - last_isr23_time > 300) { flag_isr23 = true; last_isr23_time = millis(); }
}

// === WiFi + MQTT Setup ===
void setupWiFi() {
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500); Serial.print(".");
  }
  Serial.println("WiFi connected");
  client.setServer(THINGSBOARD_SERVER, THINGSBOARD_PORT);
}

void reconnectMQTT() {
  while (!client.connected()) {
    if (client.connect("ESP32Client", TOKEN, nullptr)) {
      Serial.println("Connected to ThingsBoard");
    } else {
      delay(2000);
    }
  }
}

// === GAS đọc PPM ===
float readGasPPM() {
  int adc = analogRead(MQ2_PIN);
  float voltage = adc * (3.3 / 4095.0);
  float ratio = voltage / 1.4;
  return 1000.0 * pow(10, ((log10(ratio) - 1.0278) / 0.6629));
}

// === Gửi Telemetry ===
void publishTelemetry(float gasPPM, const char* status) {
  StaticJsonDocument<256> doc;
  doc["gas"] = gasPPM;
  doc["status"] = status;
  doc["car_count"] = car;
  doc["motorbike_count"] = motor;

  char buffer[256];
  serializeJson(doc, buffer);
  client.publish("v1/devices/me/telemetry", buffer);
}

// === Đọc khoảng cách ===
long readDistanceCM(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW); delayMicroseconds(2);
  digitalWrite(trigPin, HIGH); delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  return pulseIn(echoPin, HIGH, 30000) * 0.034 / 2;
}

// === Setup ===
void setup() {
  Serial.begin(9600);
  Serial2.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);

  setupWiFi();

  pinMode(BUZZER_PIN, OUTPUT); digitalWrite(BUZZER_PIN, LOW);
  pinMode(TRIG_IN, OUTPUT); pinMode(ECHO_IN, INPUT);
  pinMode(TRIG_OUT, OUTPUT); pinMode(ECHO_OUT, INPUT);

  servoright.attach(SERVO_IN_PIN);
  servoleft.attach(SERVO_OUT_PIN);
  servoright.write(0); servoleft.write(0);

  Wire.begin();
  lcd.init(); lcd.backlight();
  lcd.setCursor(0, 0); lcd.print("Khoi dong...");
  rtc.begin();
  if (rtc.lostPower()) rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));

  // ISR config
  pinMode(12, INPUT_PULLDOWN); attachInterrupt(12, isr12, RISING);
  pinMode(13, INPUT_PULLDOWN); attachInterrupt(13, isr13, RISING);
  pinMode(14, INPUT_PULLDOWN); attachInterrupt(14, isr14, RISING);
  pinMode(23, INPUT_PULLDOWN); attachInterrupt(23, isr23, RISING);

  lcd.clear(); lcd.print("He thong san sang");
}

// === LOOP ===
unsigned long lastPublish = 0;
const unsigned long publishInterval = 2000;

void loop() {
  if (!client.connected()) reconnectMQTT();
  client.loop();

  if (flag_isr12) { flag_isr12 = false; if (motor > 0) motor--; fullmotor = false; }
  if (flag_isr13) { flag_isr13 = false; if (car < 20) car++; if (car == 20) fullcar = true; }
  if (flag_isr14) { flag_isr14 = false; if (car > 0) car--; fullcar = false; }
  if (flag_isr23) { flag_isr23 = false; if (motor < 50) motor++; if (motor == 50) fullmotor = true; }

  float gasPPM = readGasPPM();
  digitalWrite(BUZZER_PIN, gasPPM > GAS_THRESHOLD ? HIGH : LOW);

  unsigned long now = millis();
  if (now - lastPublish >= publishInterval) {
    publishTelemetry(gasPPM, gasPPM > GAS_THRESHOLD ? "Gas leak!" : "Safe");
    lastPublish = now;
  }

  long dist_in = readDistanceCM(TRIG_IN, ECHO_IN);
  long dist_out = readDistanceCM(TRIG_OUT, ECHO_OUT);

  if (dist_out <= 4 && !Gateleft) { servoleft.write(0); Gateleft = true; }
  if (dist_out > 4 && dist_out <= 10 && Gateright) { servoright.write(90); Gateright = false; }

  if (dist_in <= 4 && Gateleft) { servoleft.write(90); Serial2.write("DETECT"); Gateleft = false; }
  if (dist_in > 4 && dist_in <= 10 && !Gateright) { servoright.write(0); Serial2.write("DETECT"); Gateright = true; }

  // LCD hiển thị
  DateTime t = rtc.now();
  lcd.setCursor(0, 0);
  lcd.printf("Time:%02d:%02d:%02d", t.hour(), t.minute(), t.second());
  lcd.setCursor(0, 1); lcd.printf("So xe may: %2d", motor);
  if (fullmotor) { lcd.setCursor(15, 1); lcd.print(blinkmotor ? "!FULL" : "     "); blinkmotor = !blinkmotor; }

  lcd.setCursor(0, 2); lcd.printf("So xe oto: %2d", car);
  if (fullcar) { lcd.setCursor(15, 2); lcd.print(blinkcar ? "!FULL" : "     "); blinkcar = !blinkcar; }

  if (Serial2.available()) { uint8_t cmd = Serial2.read(); Serial2.write(0x22); }

  delay(500);
}
