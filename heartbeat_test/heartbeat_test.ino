#include <Arduino.h>

#define MS_TICKS(x) ((x) / portTICK_PERIOD_MS)

// Watchdog Heartbeat Pins
const int HEARTBEAT_1_PIN = 14; 
const int HEARTBEAT_2_PIN = 27;

// Watchdog States
volatile bool isAlive1 = true;
volatile bool isAlive2 = true;


// ---------------------------------------------------------
// TASK: Watchdog Heartbeat L1 (every 15ms, timeout ~30ms)
// ---------------------------------------------------------
void HeartbeatTask1(void *pvParameters) {
  Serial.println(">>> HB1 task started");
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = 15 / portTICK_PERIOD_MS;
  uint32_t count = 0;

  while (true) {
    if (isAlive1) {
      digitalWrite(HEARTBEAT_1_PIN, HIGH);
      delayMicroseconds(1000);
      digitalWrite(HEARTBEAT_1_PIN, LOW);
    }
    if (++count % 67 == 0) {  // print roughly once per second
      Serial.print("HB1 tick #"); Serial.println(count);
    }
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

// ---------------------------------------------------------
// TASK: Watchdog Heartbeat L2 (every 50ms, timeout ~150ms)
// ---------------------------------------------------------
void HeartbeatTask2(void *pvParameters) {
  Serial.println(">>> HB2 task started");
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = 50 / portTICK_PERIOD_MS;
  uint32_t count = 0;

  while (true) {
    if (isAlive2) {
      digitalWrite(HEARTBEAT_2_PIN, HIGH);
      delayMicroseconds(1000);
      digitalWrite(HEARTBEAT_2_PIN, LOW);
    }
    if (++count % 20 == 0) {  // print roughly once per second
      Serial.print("HB2 tick #"); Serial.println(count);
    }
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}


void setup() {
  Serial.begin(115200);
  while (!Serial);

  // Initialize Heartbeat Pins
  pinMode(HEARTBEAT_1_PIN, OUTPUT);
  pinMode(HEARTBEAT_2_PIN, OUTPUT);
  digitalWrite(HEARTBEAT_1_PIN, LOW);
  digitalWrite(HEARTBEAT_2_PIN, LOW);

  Serial.println("--- System Boot ---");
  Serial.println("Send 'q' to stop heartbeat Level 1");
  Serial.println("Send 'w' to stop heartbeat Level 2");
  Serial.println("Send 'r' to restart heartbeats");

  // Create Heartbeat Tasks on Core 0
  xTaskCreatePinnedToCore((void(*)(void*))&HeartbeatTask1, "HeartbeatTask1", 2048, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore((void(*)(void*))&HeartbeatTask2, "HeartbeatTask2", 2048, NULL, 2, NULL, 0);
}

void loop() {
  if (Serial.available() > 0) {
    char c = Serial.read();
    
    if (c == 'q') {
      isAlive1 = false;
      Serial.println("\n>>> Heartbeat 1 (Level 1) STOPPED");
    } 
    else if (c == 'w') {
      isAlive2 = false;
      Serial.println("\n>>> Heartbeat 2 (Level 2) STOPPED");
    }
    else if (c == 'r') { 
      isAlive1 = true;
      isAlive2 = true;
      Serial.println("\n>>> Heartbeats RESTARTED");
    }
  }
  
  delay(50);
}