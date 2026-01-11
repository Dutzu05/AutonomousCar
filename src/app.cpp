#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <ESP32Servo.h>

// ================= DATA STRUCTURE =================
typedef struct {
  int value;          // ADC value (0–4095)
  uint8_t forward;   // 1 = pressed
  uint8_t backward;  // 1 = pressed
} packet_t;

packet_t data;

// ================= CONFIGURATION =================
uint8_t receiverMacAddress[] = {0xDC, 0xB4, 0xD9, 0x8B, 0xD0, 0xFC};

#if !defined(SENDER) && !defined(RECEIVER)
#error "Define either SENDER or RECEIVER in build_flags"
#endif

// ================= SENDER =================
#if defined(SENDER)
#define POT_PIN       1
#define FORWARD_BTN   4
#define BACKWARD_BTN  5
#endif

// ================= RECEIVER =================
#if defined(RECEIVER)

// ---------- Ultrasonic ----------
#define TRIG_PIN 7
#define ECHO_PIN 9

volatile float g_distance_cm = 0.0;
volatile bool g_obstacle = false;

// ---------- Servo ----------
#define SERVO_PIN     8
#define SERVO_MIN_US  500
#define SERVO_MAX_US  2400

// ---------- Motor ----------
#define CW_PIN   5
#define CCW_PIN  4

Servo servo;

// ---------- RX buffer ----------
volatile packet_t rxData;
volatile bool newPacket = false;

#endif

// ================= CALLBACKS =================
void onDataSent(const uint8_t *, esp_now_send_status_t status) {
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "SEND OK" : "SEND FAIL");
}

void onDataRecv(const uint8_t *, const uint8_t *incomingData, int len) {
#if defined(RECEIVER)
  if (len != sizeof(packet_t)) return;
  memcpy((void*)&rxData, incomingData, sizeof(packet_t));
  newPacket = true;
#endif
}

// ================= ULTRASONIC TASK =================
#if defined(RECEIVER)
void ultrasonicTask(void *pvParameters)
{
  static unsigned long lastPrint = 0;
  for (;;)
  {
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    unsigned long duration = pulseIn(ECHO_PIN, HIGH, 25000);

    float d = 0.0;
    if (duration > 0)
      d = (duration * 0.0343f) / 2.0f;

    g_distance_cm = d;
    g_obstacle = (d > 0 && d < 10.0f);
    if (millis() - lastPrint > 300)   // ~3 Hz
    {
      lastPrint = millis();
      Serial.print("[US] Distance: ");
      Serial.print(g_distance_cm);
      Serial.println(" cm");
    }
    vTaskDelay(pdMS_TO_TICKS(50)); // 20 Hz
  }
}
#endif

// ================= SETUP =================
void setup() {
  Serial.begin(115200);
  delay(1000);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_max_tx_power(78);

  Serial.print("MAC: ");
  Serial.println(WiFi.macAddress());

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW INIT FAILED");
    while (true);
  }

#if defined(SENDER)
  analogReadResolution(12);
  analogSetPinAttenuation(POT_PIN, ADC_11db);

  pinMode(FORWARD_BTN, INPUT_PULLDOWN);
  pinMode(BACKWARD_BTN, INPUT_PULLDOWN);

  esp_now_register_send_cb(onDataSent);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, receiverMacAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("FAILED TO ADD PEER");
    while (true);
  }
#endif

#if defined(RECEIVER)
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  servo.setPeriodHertz(50);
  servo.attach(SERVO_PIN, SERVO_MIN_US, SERVO_MAX_US);

  pinMode(CW_PIN, OUTPUT);
  pinMode(CCW_PIN, OUTPUT);

  digitalWrite(CW_PIN, LOW);
  digitalWrite(CCW_PIN, LOW);

  esp_now_register_recv_cb(onDataRecv);

  // --- Create ultrasonic task ---
  xTaskCreatePinnedToCore(
    ultrasonicTask,
    "Ultrasonic",
    2048,
    NULL,
    2,
    NULL,
    1
  );
#endif

  Serial.println("SETUP COMPLETE");
}

// ================= LOOP =================
void loop() {

#if defined(SENDER)
  data.value = analogRead(POT_PIN);
  data.forward = digitalRead(FORWARD_BTN) == HIGH;
  data.backward = digitalRead(BACKWARD_BTN) == HIGH;

  esp_now_send(receiverMacAddress, (uint8_t *)&data, sizeof(data));

  delay(20);
#endif

#if defined(RECEIVER)
  if (newPacket) {
    newPacket = false;

    packet_t localData;
    noInterrupts();
    memcpy(&localData, (void*)&rxData, sizeof(packet_t));
    interrupts();

    // ----- SERVO (always allowed) -----
    int pulseUs = map(localData.value, 0, 4095, SERVO_MIN_US, SERVO_MAX_US);
    pulseUs = constrain(pulseUs, SERVO_MIN_US, SERVO_MAX_US);
    servo.writeMicroseconds(pulseUs);

    // ----- MOTOR (urgent stop override) -----
    if (g_obstacle)
    {
      digitalWrite(CW_PIN, LOW);
      digitalWrite(CCW_PIN, LOW);
      Serial.println("OBJECT AHEAD → MOTOR STOP");
    }
    else
    {
      if (localData.forward && !localData.backward) {
        digitalWrite(CW_PIN, HIGH);
        digitalWrite(CCW_PIN, LOW);
      }
      else if (localData.backward && !localData.forward) {
        digitalWrite(CW_PIN, LOW);
        digitalWrite(CCW_PIN, HIGH);
      }
      else {
        digitalWrite(CW_PIN, LOW);
        digitalWrite(CCW_PIN, LOW);
      }
    }
  }

  delay(5);
#endif
}
