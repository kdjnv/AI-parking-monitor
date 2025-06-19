#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

#include <Wire.h>
#include "RTClib.h"
#include <LiquidCrystal_I2C.h>

#include <ESP32Servo.h>

// Pin configuration mq2 buzzer
#define MQ2_PIN         34
#define BUZZER_PIN      25
#define GAS_THRESHOLD   2

// pin hc-sr04 servo
#define TRIG_IN 5
#define ECHO_IN 18
#define SERVO_IN_PIN 19

#define TRIG_OUT 17
#define ECHO_OUT 16
#define SERVO_OUT_PIN 4

// UART
#define RX_PIN 26
#define TX_PIN 27
HardwareSerial MySerial(2);

// Khởi tạo đối tượng
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

// --- ISR debounce flags ---
volatile bool flag_isr12 = false;
volatile bool flag_isr13 = false;
volatile bool flag_isr14 = false;
volatile bool flag_isr23 = false;

unsigned long last_isr12_time = 0;
unsigned long last_isr13_time = 0;
unsigned long last_isr14_time = 0;
unsigned long last_isr23_time = 0;

// ISR với debounce
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

float readGasPPM() {
  int adcValue = analogRead(MQ2_PIN);
  float voltage = adcValue * (3.3 / 4095.0);
  float ratio = voltage / 1.4;
  float ppm = 1000.0 * pow(10, ((log10(ratio) - 1.0278) / 0.6629));
  return ppm;
}

void publishTelemetry(float gasPPM, const char* status) {
  StaticJsonDocument<200> doc;
  doc["gas"] = gasPPM;
  doc["status"] = status;
  char buffer[256];
  serializeJson(doc, buffer);
}

long readDistanceCM(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH, 30000);
  return duration * 0.034 / 2;
}

void setup() {
  Serial.begin(9600);
  Serial2.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);

  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

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
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Khoi dong...");

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

unsigned long lastPublish = 0;
const unsigned long publishInterval = 2000;

void loop() {
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

  // // Gas
  // unsigned long currentMillis = millis();
  // if (currentMillis - lastPublish >= publishInterval) {
  //   float gasPPM = readGasPPM();
  //   digitalWrite(BUZZER_PIN, gasPPM > GAS_THRESHOLD ? HIGH : LOW);
  //   publishTelemetry(gasPPM, gasPPM > GAS_THRESHOLD ? "Gas leak!" : "Safe");
  //   lastPublish = currentMillis;
  // }

  // Servo mở/đóng theo cảm biến khoảng cách
  long dist_in = readDistanceCM(TRIG_IN, ECHO_IN);
  long dist_out = readDistanceCM(TRIG_OUT, ECHO_OUT);

  if (dist_out >= 0 && dist_out <= 4) {
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
      Serial2.write("DETECT");
      Gateleft = false;
    }
  } else if (dist_in >= 5 && dist_in <= 10) {
    if (!Gateright) {
      if (servoright.read() != 0) servoright.write(0);
      Serial2.write("DETECT");
      Gateright = true;
    }
  }

  // LCD đồng hồ & hiển thị
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

  // UART
  if (Serial2.available()) {
    uint8_t command = Serial2.read();
    Serial2.write(0x22);
  }

  delay(500); // Delay nhỏ để tránh quá tải LCD và servo
}
