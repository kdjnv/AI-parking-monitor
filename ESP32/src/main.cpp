#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include "RTClib.h"
#include <LiquidCrystal_I2C.h>
#include <ESP32Servo.h>

// WiFi + MQTT config
#define WIFI_SSID       "Cam Va Quyt"
#define WIFI_PASSWORD   "phongdz003"
#define TOKEN           "5cJxzxh2jbnYmobbRUIO"
#define THINGSBOARD_SERVER "thingsboard.cloud"
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

bool Gateleft = false;
bool Gateright = false;

int car = 0;
int motor = 0;
bool fullcar = false;
bool fullmotor = false;
bool blinkmotor = false;
bool blinkcar = false;
// 
bool parkingLocked = false; // Mặc định bãi xe mở

// --- ISR debounce flags ---
volatile bool flag_isr12 = false;
volatile bool flag_isr13 = false;
volatile bool flag_isr14 = false;
volatile bool flag_isr23 = false;

unsigned long last_isr12_time = 0;
unsigned long last_isr13_time = 0;
unsigned long last_isr14_time = 0;
unsigned long last_isr23_time = 0;

// === ISR debounce ===
void IRAM_ATTR isr12() {
  unsigned long now = millis();
  if (now - last_isr12_time > 300) {
    flag_isr12 = true;
    last_isr12_time = now;
  }
}
void IRAM_ATTR isr13() {
  unsigned long now = millis();
  if (now - last_isr13_time > 300) {
    flag_isr13 = true;
    last_isr13_time = now;
  }
}
void IRAM_ATTR isr14() {
  unsigned long now = millis();
  if (now - last_isr14_time > 300) {
    flag_isr14 = true;
    last_isr14_time = now;
  }
}
void IRAM_ATTR isr23() {
  unsigned long now = millis();
  if (now - last_isr23_time > 300) {
    flag_isr23 = true;
    last_isr23_time = now;
  }
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
  // Wait until we're connected
  while (!client.connected()) {
    if (client.connect("ESP32Client", TOKEN, nullptr)) {
      Serial.println("Connected to ThingsBoard");
    } else {
      delay(2000);
    }
  }
  client.subscribe("v1/devices/me/rpc/request/+");
}

// === GAS đọc PPM ===
float readGasPPM() {
  int adcValue = analogRead(MQ2_PIN);
  float voltage = adcValue * (3.3 / 4095.0);
  float ratio = voltage / 1.4;
  float ppm = 1000.0 * pow(10, ((log10(ratio) - 1.0278) / 0.6629));
  return ppm;
}

// === Gửi Telemetry ===
void publishTelemetry(float gasPPM, const char* status) {
  StaticJsonDocument<256> doc;
  doc["gas"] = gasPPM;
  doc["status"] = status;
  doc["car_count"] = car;
  doc["motorbike_count"] = motor;
  doc["locked"] = parkingLocked;
  doc["time"] = rtc.now().timestamp();

  char buffer[256];
  serializeJson(doc, buffer);
  client.publish("v1/devices/me/telemetry", buffer);
}

// === Đọc khoảng cách ===
long readDistanceCM(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH, 30000);
  return duration * 0.034 / 2;

}

// === MQTT Callback ===
void rpcCallback(char* topic, byte* payload, unsigned int length) {
  StaticJsonDocument<128> doc;
  deserializeJson(doc, payload, length);

  const char* method = doc["method"];
  if (strcmp(method, "lock_parking") == 0) {
    parkingLocked = doc["params"];  // nhận true hoặc false
    Serial.print("Parking locked: ");
    Serial.println(parkingLocked ? "ON" : "OFF");
  }
}

// === Setup ===
void setup() {
  Serial.begin(9600);
  Serial2.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);

  client.setCallback(rpcCallback);

  setupWiFi();

  pinMode(BUZZER_PIN, OUTPUT); 
  digitalWrite(BUZZER_PIN, LOW);

  // ISR config
  pinMode(12, INPUT_PULLDOWN);
  attachInterrupt(digitalPinToInterrupt(12), isr12, RISING);

  pinMode(13, INPUT_PULLDOWN);
  attachInterrupt(digitalPinToInterrupt(13), isr13, RISING);

  pinMode(14, INPUT_PULLDOWN);
  attachInterrupt(digitalPinToInterrupt(14), isr14, RISING);

  pinMode(23, INPUT_PULLDOWN);
  attachInterrupt(digitalPinToInterrupt(23), isr23, RISING);

  pinMode(TRIG_IN, OUTPUT); 
  pinMode(ECHO_IN, INPUT);
  pinMode(TRIG_OUT, OUTPUT); 
  pinMode(ECHO_OUT, INPUT);

  servoright.attach(SERVO_IN_PIN);
  servoleft.attach(SERVO_OUT_PIN);
  servoright.write(0);
  servoleft.write(0);

  Wire.begin();
  lcd.init(); lcd.backlight();
  lcd.setCursor(0, 0); lcd.print("Khoi dong...");

  if (!rtc.begin()) {
    lcd.setCursor(0, 0);
    lcd.print("Loi ket noi RTC");
    while (1);
  }
  if (rtc.lostPower()) {
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }
  lcd.clear();
  lcd.print("Doc thoi gian");
  servoleft.write(90);
  servoright.write(90);
  delay(1000);
}

// === LOOP ===
unsigned long lastPublish = 0;
const unsigned long publishInterval = 2000;

void loop() {
  if (!client.connected()) reconnectMQTT();
  client.loop();

  // Xử lý ISR debounce
  if (flag_isr12) {
    flag_isr12 = false;
    if (motor > 0) motor--;
    fullmotor = false;
  }
  if (flag_isr13) {
    flag_isr13 = false;
    if (car < 20) car++;
    if (car == 20) fullcar = true;
  }
  if (flag_isr14) {
    flag_isr14 = false;
    if (car > 0) car--;
    fullcar = false;
  }
  if (flag_isr23) {
    flag_isr23 = false;
    if (motor < 50) motor++;
    if (motor == 50) fullmotor = true;
  }

  // Gas
  unsigned long currentMillis = millis();
  if (currentMillis - lastPublish >= publishInterval) {
    float gasPPM = readGasPPM();
    digitalWrite(BUZZER_PIN, gasPPM > GAS_THRESHOLD ? HIGH : LOW);
    publishTelemetry(gasPPM, gasPPM > GAS_THRESHOLD ? "Gas leak!" : "Safe");
    lastPublish = currentMillis;
  }

  // Servo mở/đóng theo button và cảm biến khoảng cách
if (!parkingLocked) {
  long dist_in = readDistanceCM(TRIG_IN, ECHO_IN);
  long dist_out = readDistanceCM(TRIG_OUT, ECHO_OUT);

  // 🚗 Xe ĐI RA: khoảng cách ở OUT gần
  if (dist_out >= 0 && dist_out <= 4) {
    if (!Gateleft) {
      servoleft.write(0); // Mở cổng trái
      Gateleft = true;
    }
  } else if (dist_out > 4 && dist_out <= 10) {
    if (Gateright) {
      servoright.write(90); // Đóng cổng phải
      Gateright = false;
    }
  }

  // 🚗 Xe ĐI VÀO: khoảng cách ở IN gần
  if (dist_in >= 0 && dist_in <= 4) {
    if (Gateleft) {
      servoleft.write(90); // Đóng cổng trái
      Serial2.print("DETECT");
      Gateleft = false;
    }
  } else if (dist_in >= 5 && dist_in <= 10) {
    if (!Gateright) {
      servoright.write(0); // Mở cổng phải
      Serial2.print("DETECT");
      Gateright = true;
    }
  }
}



  //LCD
  DateTime now = rtc.now();
  lcd.setCursor(0, 0);
  lcd.print("Time:");
  if (now.hour() < 10) lcd.print("0");
  lcd.print(now.hour()); lcd.print(":");
  if (now.minute() < 10) lcd.print("0");
  lcd.print(now.minute()); lcd.print(":");
  if (now.second() < 10) lcd.print("0");
  lcd.print(now.second());

  lcd.setCursor(0, 1);
  lcd.print("So xe may: "); lcd.print(motor);
  if (fullmotor) {
    if (blinkmotor) lcd.setCursor(15, 1), lcd.print("!FULL");
    else lcd.setCursor(15, 1), lcd.print("     ");
    blinkmotor = !blinkmotor;
  }

  lcd.setCursor(0, 2);
  lcd.print("So xe oto: "); lcd.print(car);
  if (fullcar) {
    if (blinkcar) lcd.setCursor(15, 2), lcd.print("!FULL");
    else lcd.setCursor(15, 2), lcd.print("     ");
    blinkcar = !blinkcar;
  }

  delay(500);
}
