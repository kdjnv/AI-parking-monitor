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

bool ONSYS = true;
bool onAI = true;

// --- ISR debounce flags ---
volatile bool flag_isr12 = false;
volatile bool flag_isr13 = false;
volatile bool flag_isr14 = false;
volatile bool flag_isr23 = false;

unsigned long last_isr12_time = 0;
unsigned long last_isr13_time = 0;
unsigned long last_isr14_time = 0;
unsigned long last_isr23_time = 0;

void rpcCallback(char* topic, byte* payload, unsigned int length);

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
  while (!client.connected()) {
    if (client.connect("ESP32Client", TOKEN, nullptr)) {
      Serial.println("Connected to ThingsBoard");

      client.setCallback(rpcCallback);
      client.subscribe("v1/devices/me/rpc/request/+");
    } else {
      delay(2000);
    }
  }
}

// === GAS PPM ===
float readGasPPM() {
  int adcValue = analogRead(MQ2_PIN);
  float voltage = adcValue * (3.3 / 4095.0);
  float ratio = voltage / 1.4;
  float ppm = 1000.0 * pow(10, ((log10(ratio) - 1.0278) / 0.6629));
  return ppm;
}

// === Telemetry ===
void publishTelemetry(float gasPPM, const char* status) {
  StaticJsonDocument<256> doc;
  doc["gas"] = gasPPM;
  doc["status"] = status;
  doc["car_count"] = car;
  doc["motorbike_count"] = motor;

  DateTime now = rtc.now();
  char timestr[9]; // "HH:MM:SS" + null
  snprintf(timestr, sizeof(timestr), "%02d:%02d:%02d", now.hour(), now.minute(), now.second());
  doc["time"] = timestr;

  char buffer[256];
  serializeJson(doc, buffer);
  client.publish("v1/devices/me/telemetry", buffer);
}

// === Doc khoang cach ===
long readDistanceCM(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH, 30000);
  return duration * 0.034 / 2;
}

// === Setup ===
void setup() {
  Serial.begin(9600);
  Serial2.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);

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

  client.setCallback(rpcCallback);
  client.subscribe("v1/devices/me/rpc/request/+");

  delay(1000);
}

// === LOOP ===
unsigned long lastPublish = 0;
const unsigned long publishInterval = 2000;
String uartBuffer = "";
bool detectcar = false;
bool detectmotor = false;

void loop() {
  if (!client.connected()) reconnectMQTT();
  client.loop();

  // ISR debounce
  if (flag_isr12 && !onAI) {
    flag_isr12 = false;
    if (motor > 0) motor--;
    fullmotor = false;
  }
  if (flag_isr13 && !onAI && ONSYS) {
    flag_isr13 = false;
    if (car < 20) car++;
    if (car == 20) fullcar = true;
  }
  if (flag_isr14 && !onAI) {
    flag_isr14 = false;
    if (car > 0) car--;
    fullcar = false;
  }
  if (flag_isr23 && !onAI && ONSYS) {
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

  // Servo hoat dong theo khoang cach
  long dist_in = readDistanceCM(TRIG_IN, ECHO_IN);
  long dist_out = readDistanceCM(TRIG_OUT, ECHO_OUT);

  if (dist_out >= 0 && dist_out <= 4 && ONSYS) {
    if (!Gateleft) {
      if (servoleft.read() != 0) servoleft.write(0);
      Gateleft = true;
    }
  } else if (dist_out > 4 && dist_out <= 10) {
    if (Gateright) {
      if (servoright.read() != 90) servoright.write(90);
      Gateright = false;
    }
  }

  if (dist_in >= 0 && dist_in <= 4) {
    if (Gateleft) {
      if (servoleft.read() != 90) servoleft.write(90);
      Serial2.print("DETECT");
      Gateleft = false;
      while (!detectcar && !detectmotor) {
        if (Serial2.available()){
          char c = Serial2.read();
          if (c == '\n') {
            uartBuffer.trim(); // loai bo ki tu thua
            if (uartBuffer == "OTO") {
              detectcar = true;
              Serial.println("Nhan duoc oto tu UART");
            } else if (uartBuffer == "XEMAY") {
              detectmotor = true;
              Serial.println("Nhan duoc xe may tu UART");
            }
            uartBuffer = ""; // reset buffer
          } else {
            uartBuffer += c;
          }
        }
      }
    }
    if (detectcar){
      detectcar = false;
      if (car < 20) car++;
    }
    if (detectmotor){
      detectmotor = false;
      if (motor < 50) motor++;
    }
  } else if (dist_in >= 5 && dist_in <= 10) {
    if (!Gateright) {
      Serial2.print("DETECT");
      Gateright = true;
      while (!detectcar && !detectmotor) {
        if (Serial2.available()){
          char c = Serial2.read();
          if (c == '\n') {
            uartBuffer.trim(); 
            if (uartBuffer == "OTO") {
              detectcar = true;
              Serial.println("Nhan duoc oto tu UART");
            } else if (uartBuffer == "XEMAY") {
              detectmotor = true;
              Serial.println("Nhan duoc xe may tu UART");
            }
            uartBuffer = ""; // reset buffer
          } else {
            uartBuffer += c;
          }
        }
      }
      if (servoright.read() != 0) servoright.write(0);
    }
    if (detectcar){
      detectcar = false;
      if (car != 0) car--;
    }
    if (detectmotor){
      detectmotor = false;
      if (motor != 0) motor--;
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

  lcd.setCursor(0,3);
  if (ONSYS) lcd.print("ON ");
  else lcd.print("OFF");

  lcd.setCursor(10,3);
  if (onAI) lcd.print("ON ");
  else lcd.print("OFF");

  delay(500);
}

void rpcCallback(char* topic, byte* payload, unsigned int length) {
  StaticJsonDocument<128> doc;
  DeserializationError error = deserializeJson(doc, payload, length);
  if (error) {
    Serial.println("deserializeJson() failed");
    return;
  }

  const char* method = doc["method"];
  if (strcmp(method, "set_abcdxyz") == 0) {
    ONSYS = doc["params"]; //true or false
    Serial.print("ONSYS: ");
    Serial.println(ONSYS ? "ON" : "OFF");
  }

  if (strcmp(method, "on/offAI") == 0) {
    onAI = doc["params"]; //true or false
    Serial.print("AI: ");
    Serial.println(onAI ? "ON" : "OFF");
  }
}
