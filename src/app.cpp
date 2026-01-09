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
#define SERVO_PIN     8
#define SERVO_MIN_US  500
#define SERVO_MAX_US  2400

#define CW_PIN   5
#define CCW_PIN  4
//#define EN_PIN   6

/*const int pwmChannel = 0;
const int pwmFreq = 20000;
const int pwmResolution = 8;

int duty = 0;*/

Servo servo;

// RX buffer (NO HARDWARE CONTROL IN CALLBACK)
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
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  servo.setPeriodHertz(50);
  servo.attach(SERVO_PIN, SERVO_MIN_US, SERVO_MAX_US);

  pinMode(CW_PIN, OUTPUT);
  pinMode(CCW_PIN, OUTPUT);
  //pinMode(EN_PIN, OUTPUT);

  digitalWrite(CW_PIN, LOW);
  digitalWrite(CCW_PIN, LOW);
  //digitalWrite(EN_PIN, LOW);  // Try LOW - some drivers enable with LOW

  esp_now_register_recv_cb(onDataRecv);
#endif

  Serial.println("SETUP COMPLETE");
}

// ================= LOOP =================
void loop() {

#if defined(SENDER)
  data.value = analogRead(POT_PIN);

  // Buttons connected to 3.3V, so HIGH = pressed
  data.forward = (digitalRead(FORWARD_BTN) == HIGH) ? 1 : 0;
  data.backward = (digitalRead(BACKWARD_BTN) == HIGH) ? 1 : 0;

  esp_now_send(
    receiverMacAddress,
    (uint8_t *)&data,
    sizeof(data)
  );

  Serial.printf(
    "TX ADC=%d FWD=%d BWD=%d\n",
    data.value,
    data.forward,
    data.backward
  );

  delay(20);
#endif

#if defined(RECEIVER)
  if (newPacket) {
    newPacket = false;

    // Copy volatile data to local variables
    packet_t localData;
    noInterrupts();
    memcpy(&localData, (void*)&rxData, sizeof(packet_t));
    interrupts();

    int pulseUs = map(
      localData.value,
      0, 4095,
      SERVO_MIN_US, SERVO_MAX_US
    );
    pulseUs = constrain(pulseUs, SERVO_MIN_US, SERVO_MAX_US);
    servo.writeMicroseconds(pulseUs);

    if (localData.forward && !localData.backward) {
      digitalWrite(CW_PIN, HIGH);
      digitalWrite(CCW_PIN, LOW);
      Serial.println("MOTOR: FORWARD");
    }
    else if (localData.backward && !localData.forward) {
      digitalWrite(CW_PIN, LOW);
      digitalWrite(CCW_PIN, HIGH);
      Serial.println("MOTOR: BACKWARD");
    }
    else {
      digitalWrite(CW_PIN, LOW);
      digitalWrite(CCW_PIN, LOW);
      Serial.println("MOTOR: STOP");
    }

    Serial.printf(
      "RX ADC=%d FWD=%d BWD=%d\n",
      localData.value,
      localData.forward,
      localData.backward
    );
  }

  delay(5);
#endif
}
