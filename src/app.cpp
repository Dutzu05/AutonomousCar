#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <ESP32Servo.h>

// ================= DATA STRUCTURE =================
typedef struct
{
  int value;          // steering pot (0–4095)
  uint8_t forward;    // 1 = pressed
  uint8_t backward;   // 1 = pressed
  uint8_t speed_up;   // 1 = pressed
  uint8_t speed_down; // 1 = pressed
} packet_t;

packet_t data;

// ================= CONFIGURATION =================
uint8_t receiverMacAddress[] = {0xDC, 0xB4, 0xD9, 0x8B, 0xD0, 0xFC};

#if !defined(SENDER) && !defined(RECEIVER)
#error "Define either SENDER or RECEIVER in build_flags"
#endif

// ================= SENDER =================
#if defined(SENDER)

// IMPORTANT: you demanded speed_up on GPIO1, so POT cannot also be GPIO1.
// Rewire potentiometer wiper to GPIO2.
#define POT_PIN 1

#define FORWARD_BTN 4
#define BACKWARD_BTN 5

#define SPEED_DOWN_BTN 0
#define SPEED_UP_BTN 2

#endif

// ================= RECEIVER =================
#if defined(RECEIVER)

// ---------- Motion override state ----------
enum MotionState
{
  NORMAL,
  WAIT_OBSTACLE,
  MOVE_LEFT,
  MOVE_RIGHT,
  RESTORE
};

volatile MotionState motionState = NORMAL;

unsigned long obstacleStartTime = 0;
unsigned long stateStartTime = 0;

int savedServoUs = 0;
int savedMotorDir = 0;

// ---------- Ultrasonic ----------
#define TRIG_PIN 7
#define ECHO_PIN 9

volatile float g_distance_cm = 0.0;
volatile bool g_obstacle = false;

// ---------- Servo ----------
#define SERVO_PIN 8
#define SERVO_MIN_US 500
#define SERVO_MAX_US 2400

int lastManualServoUs = (SERVO_MIN_US + SERVO_MAX_US) / 2;

#define SERVO_LEFT_OFFSET 250
#define SERVO_RIGHT_OFFSET 250
#define AVOIDANCE_LIMIT_US 300

// ---------- Motor ----------
#define CW_PIN 5
#define CCW_PIN 4
#define EN_PIN 6 // PWM ENABLE

#define MOTOR_PWM_CH_CW 2
#define MOTOR_PWM_CH_CCW 3
#define MOTOR_PWM_FREQ 20000
#define MOTOR_PWM_RES 8 // 0–255

int userSpeed = 50;       // % (user-controlled)
const int baseSpeed = 50; // % (used during avoidance)

Servo servo;

// ---------- RX buffer ----------
volatile packet_t rxData;
volatile bool newPacket = false;

#endif

// ================= HELPERS =================
#if defined(RECEIVER)

int clampServo(int us)
{
  return constrain(us, SERVO_MIN_US, SERVO_MAX_US);
}

void setMotor(int dir, int speedPercent)
{
  speedPercent = constrain(speedPercent, 0, 100);
  int duty = map(speedPercent, 0, 100, 0, 255);

  if (dir == 1)
  {
    ledcWrite(MOTOR_PWM_CH_CCW, 0);
    ledcWrite(MOTOR_PWM_CH_CW, duty);
  }
  else if (dir == -1)
  {
    ledcWrite(MOTOR_PWM_CH_CW, 0);
    ledcWrite(MOTOR_PWM_CH_CCW, duty);
  }
  else
  {
    ledcWrite(MOTOR_PWM_CH_CW, 0);
    ledcWrite(MOTOR_PWM_CH_CCW, 0);
  }
}

#endif

// ================= CALLBACKS =================
void onDataSent(const uint8_t *, esp_now_send_status_t status)
{
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "SEND OK" : "SEND FAIL");
}

void onDataRecv(const uint8_t *, const uint8_t *incomingData, int len)
{
#if defined(RECEIVER)

  if (len != sizeof(packet_t))
    return;

  memcpy((void *)&rxData, incomingData, sizeof(packet_t));
  newPacket = true;
#endif
}

// ================= ULTRASONIC TASK =================
#if defined(RECEIVER)
void ultrasonicTask(void *pvParameters)
{
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

    vTaskDelay(pdMS_TO_TICKS(50));
  }
}
#endif

// ================= SETUP =================
void setup()
{
  Serial.begin(115200);
  delay(1000);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_max_tx_power(78);

  if (esp_now_init() != ESP_OK)
  {
    Serial.println("ESP-NOW INIT FAILED");
    while (true)
    {
    }
  }

#if defined(SENDER)

  analogReadResolution(12);
  analogSetPinAttenuation(POT_PIN, ADC_11db);

  pinMode(FORWARD_BTN, INPUT_PULLDOWN);
  pinMode(BACKWARD_BTN, INPUT_PULLDOWN);

  // Buttons on GPIO0 / GPIO1 as requested
  pinMode(SPEED_UP_BTN, INPUT_PULLUP);
  pinMode(SPEED_DOWN_BTN, INPUT_PULLUP);

  esp_now_register_send_cb(onDataSent);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, receiverMacAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK)
  {
    Serial.println("FAILED TO ADD PEER");
    while (true)
    {
    }
  }

#endif

#if defined(RECEIVER)

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // RESTORED: servo timer allocation (your previous working setup had this)
  ESP32PWM::allocateTimer(0);

  servo.setPeriodHertz(50);
  servo.attach(SERVO_PIN, SERVO_MIN_US, SERVO_MAX_US);

  pinMode(CW_PIN, OUTPUT);
  pinMode(CCW_PIN, OUTPUT);
  pinMode(EN_PIN, OUTPUT);
  digitalWrite(EN_PIN, HIGH);

  ledcSetup(MOTOR_PWM_CH_CW, MOTOR_PWM_FREQ, MOTOR_PWM_RES);
  ledcSetup(MOTOR_PWM_CH_CCW, MOTOR_PWM_FREQ, MOTOR_PWM_RES);

  ledcAttachPin(CW_PIN, MOTOR_PWM_CH_CW);
  ledcAttachPin(CCW_PIN, MOTOR_PWM_CH_CCW);

  ledcWrite(MOTOR_PWM_CH_CW, 0);
  ledcWrite(MOTOR_PWM_CH_CCW, 0);

  esp_now_register_recv_cb(onDataRecv);

  xTaskCreatePinnedToCore(
      ultrasonicTask,
      "Ultrasonic",
      2048,
      NULL,
      2,
      NULL,
      1);

#endif

  Serial.println("SETUP COMPLETE");
}

// ================= LOOP =================
void loop()
{
#if defined(SENDER)

  data.value = analogRead(POT_PIN);
  data.forward = (digitalRead(FORWARD_BTN) == HIGH);
  data.backward = (digitalRead(BACKWARD_BTN) == HIGH);

  data.speed_up = (digitalRead(SPEED_UP_BTN) == LOW);
  data.speed_down = (digitalRead(SPEED_DOWN_BTN) == LOW);

  esp_now_send(receiverMacAddress, (uint8_t *)&data, sizeof(data));
  static uint32_t t = 0;
  if (millis() - t > 200)
  {
    t = millis();
    Serial.printf("rawUp=%d rawDown=%d\n",
                  digitalRead(SPEED_UP_BTN),
                  digitalRead(SPEED_DOWN_BTN));
  }
  delay(20);

#endif

#if defined(RECEIVER)

  if (!newPacket)
  {
    delay(1);
    return;
  }

  newPacket = false;

  packet_t localData;
  noInterrupts();
  memcpy(&localData, (void *)&rxData, sizeof(packet_t));
  interrupts();

  // ---- SPEED CONTROL (NORMAL ONLY, EDGE-TRIGGERED) ----
  static uint8_t prevUp = 0;
  static uint8_t prevDown = 0;

  if (motionState == NORMAL)
  {
    if (localData.speed_up && !prevUp)
      userSpeed = min(userSpeed + 10, 100);

    if (localData.speed_down && !prevDown)
      userSpeed = max(userSpeed - 10, 0);
  }

  prevUp = localData.speed_up;
  prevDown = localData.speed_down;

  // ---- SERVO MANUAL (NORMAL ONLY) ----
  if (motionState == NORMAL)
  {
    int pulseUs = map(localData.value, 0, 4095, SERVO_MIN_US, SERVO_MAX_US);
    pulseUs = constrain(pulseUs, SERVO_MIN_US, SERVO_MAX_US);
    lastManualServoUs = pulseUs;
    servo.writeMicroseconds(pulseUs);
  }

  unsigned long now = millis();

  int desiredDir = 0;
  if (localData.forward && !localData.backward)
    desiredDir = 1;
  else if (!localData.forward && localData.backward)
    desiredDir = -1;

  switch (motionState)
  {
  case NORMAL:
    if (g_obstacle)
    {
      obstacleStartTime = now;
      setMotor(0, 0);
      motionState = WAIT_OBSTACLE;
      Serial.println("OBJECT DETECTED ... WAITING");
    }
    else
    {
      setMotor(desiredDir, userSpeed);
    }
    break;

  case WAIT_OBSTACLE:
    setMotor(0, 0);

    if (!g_obstacle)
    {
      motionState = NORMAL;
      break;
    }

    if (now - obstacleStartTime > 5000)
    {
      savedServoUs = lastManualServoUs;
      savedMotorDir = desiredDir;

      servo.writeMicroseconds(
          clampServo(constrain(
              savedServoUs - SERVO_LEFT_OFFSET,
              lastManualServoUs - AVOIDANCE_LIMIT_US,
              lastManualServoUs + AVOIDANCE_LIMIT_US)));

      stateStartTime = now;
      motionState = MOVE_LEFT;
      Serial.println("START AVOIDANCE");
    }
    break;

  case MOVE_LEFT:
    setMotor(savedMotorDir, baseSpeed);

    if (now - stateStartTime > 4000)
    {
      servo.writeMicroseconds(clampServo(savedServoUs + SERVO_RIGHT_OFFSET));
      stateStartTime = now;
      motionState = MOVE_RIGHT;
      Serial.println("MOVING RIGHT NOW");
    }
    break;

  case MOVE_RIGHT:
    setMotor(savedMotorDir, baseSpeed);

    if (now - stateStartTime > 4000)
    {
      servo.writeMicroseconds(savedServoUs);
      setMotor(0, 0);
      stateStartTime = now;
      motionState = RESTORE;
      Serial.println("RESTORING THE POSITION");
    }
    break;

  case RESTORE:
    setMotor(0, 0);
    if (now - stateStartTime > 1500)
    {
      motionState = NORMAL;
      Serial.println("AVOIDANCE COMPLETE");
    }
    break;

  default:
    motionState = NORMAL;
    break;
  }

  delay(5);

#endif
}
