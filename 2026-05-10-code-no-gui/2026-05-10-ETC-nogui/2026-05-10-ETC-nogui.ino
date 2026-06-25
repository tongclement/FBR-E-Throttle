#include <Arduino.h>
#include <TMC5160.h>
#include <TMC5160_registers.h>

#define MS_TICKS(x) ((x) / portTICK_PERIOD_MS)

// SPI and Stepper Pins
const uint8_t SPI_CS = 5;

// Sensor Pins (Updated from your snippet)
const int PEDAL_PIN = 34;      // APPS 1 sensor pin (Controls Motor)
const int PEDAL_2_PIN = 32;    // APPS 2 sensor pin (Telemetry Only)
const int THROTTLE_PIN = 33;   // TPS sensor pin (Telemetry Only)

// Watchdog Heartbeat Pins
const int HEARTBEAT_1_PIN = 14; 
const int HEARTBEAT_2_PIN = 27;

// Watchdog States
volatile bool isAlive1 = true;
volatile bool isAlive2 = true;

TMC5160_SPI motor = TMC5160_SPI(SPI_CS);

// ---------------------------------------------------------
// TASK 1: Stepper Motor Control (Runs every 50ms)
// ---------------------------------------------------------
void StepperTask(void *pvParameters){
  Serial.println("Hello from Stepper RTOS Task");
  
  TMC5160::PowerStageParameters powerStageParams; 
  TMC5160::MotorParameters motorParams;

  SPI.begin();
  motor.begin(powerStageParams, motorParams, TMC5160::NORMAL_MOTOR_DIRECTION);
  
  // Set positioning mode based on your working snippet
  motor.setRampMode(TMC5160::POSITIONING_MODE);
  motor.setMaxSpeed(200);
  motor.setAcceleration(300);
  
  motor.writeRegister(TMC5160_Reg::GCONF, 0x00000004);
  motor.writeRegister(TMC5160_Reg::GLOBAL_SCALER, 200);
  motor.writeRegister(TMC5160_Reg::IHOLD_IRUN, 0x00011F14);

  Serial.println("Stepper initialized, waiting 1s for tuning...");
  vTaskDelay(MS_TICKS(1000)); 

  TickType_t xTaskDelayTick = xTaskGetTickCount();
  
  while (true){
    // 1. Read the driving sensor (APPS 1)
    int raw_apps_1 = analogRead(PEDAL_PIN);
    float target = map(raw_apps_1, 0, 2500, 0, 200); 

    // 2. Read the other sensors strictly for telemetry/bench verification
    int raw_apps_2 = analogRead(PEDAL_2_PIN);
    int raw_throttle = analogRead(THROTTLE_PIN);

    // 3. Command the motor to the absolute step position
    motor.setTargetPosition(target);

    // 4. Print all data
    Serial.print("Target Step: ");
    Serial.print(target);
    Serial.print(" | Actual Step: ");
    Serial.print(motor.getCurrentPosition());
    Serial.print(" | APPS1: ");
    Serial.print(raw_apps_1);
    Serial.print(" | APPS2: ");
    Serial.print(raw_apps_2);
    Serial.print(" | TPS: ");
    Serial.println(raw_throttle);

    // Run task at ~20Hz
    vTaskDelayUntil(&xTaskDelayTick, 50 / portTICK_PERIOD_MS);
  }
}

// ---------------------------------------------------------
// TASK 2: Watchdog Heartbeat (Runs every 15ms)
// ---------------------------------------------------------
void HeartbeatTask(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = 15 / portTICK_PERIOD_MS; 

  while (true) {
    // Force a crisp rising edge for L2 and falling edge for L1
    if (isAlive1) digitalWrite(HEARTBEAT_1_PIN, HIGH);
    if (isAlive2) digitalWrite(HEARTBEAT_2_PIN, HIGH);

    // Hold the pulse for exactly 1 millisecond
    delayMicroseconds(1000); 

    // Pull back to ground
    digitalWrite(HEARTBEAT_1_PIN, LOW);
    digitalWrite(HEARTBEAT_2_PIN, LOW);

    // Sleep until the next 15ms cycle
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

  // Create Stepper Task on Core 1
  xTaskCreatePinnedToCore((void(*)(void*))&StepperTask, "StepperTask", 8192, NULL, 1, NULL, 1);
  
  // Create Heartbeat Task on Core 0 (Highest Priority to avoid jitter)
  xTaskCreatePinnedToCore((void(*)(void*))&HeartbeatTask, "HeartbeatTask", 2048, NULL, 2, NULL, 0);
}

void loop() {
  // Main loop handles non-blocking Serial inputs for testing watchdogs
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