#include <ArxTypeTraits.h>
#include <Bitfield.h>
#include <TMC5160.h>
#include <TMC5160_registers.h>
#include <Arduino.h>

#define MS_TICKS(x) ((x) / portTICK_PERIOD_MS)

// SPI and Stepper Pins
const uint8_t SPI_CS = 5;

// Sensor Pins
const int PEDAL_PIN = 34;      // APPS sensor pin
const int PEDAL_2_PIN = 33;    // APPS2 sensor pin
const int THROTTLE_PIN = 32;   // TPPS sensor pin 

// Watchdog Heartbeat Pins
const int HEARTBEAT_1_PIN = 14; 
const int HEARTBEAT_2_PIN = 27;

// Watchdog States (Volatile because they are modified in loop and read in the RTOS task)
volatile bool isAlive1 = true;
volatile bool isAlive2 = true;

float Kp = 10.0;

TMC5160_SPI motor = TMC5160_SPI(SPI_CS);

// ---------------------------------------------------------
// TASK 1: Stepper Motor Control (Runs every 100ms)
// ---------------------------------------------------------
void StepperTask(void *pvParameters){
  Serial.println("Hello from Stepper RTOS Task");
  
  TMC5160::PowerStageParameters powerStageParams; 
  TMC5160::MotorParameters motorParams;

  SPI.begin();
  motor.begin(powerStageParams, motorParams, TMC5160::NORMAL_MOTOR_DIRECTION);
  
  motorParams.globalScaler = 98;
  motorParams.irun = 31;
  motorParams.ihold = 16;
  
  motor.writeRegister(TMC5160_Reg::GCONF, 0x00000004);
  motor.writeRegister(TMC5160_Reg::GLOBAL_SCALER, 200);
  motor.writeRegister(TMC5160_Reg::IHOLD_IRUN, 0x00011F14);
  motor.writeRegister(TMC5160_Reg::RAMPMODE, 1);

  // Ramp definition
  motor.setRampMode(TMC5160::VELOCITY_MODE);
  motor.setMaxSpeed(1000);
  motor.setAcceleration(2000);

  Serial.println("Stepper initialized, waiting 1s for tuning...");
  vTaskDelay(MS_TICKS(1000)); // Standstill for automatic tuning

  TickType_t xTaskDelayTick = xTaskGetTickCount();
  
  while (true){
    int raw = analogRead(PEDAL_PIN);
    float target = map(raw, 0, 2500, 0, 200); 
    
    int raw_throttle = analogRead(THROTTLE_PIN);
    float scaled_throttle = map(raw_throttle, 0, 4095, 0, 200);
    
    float error = target - scaled_throttle;
    float velocity = Kp * error;

    int vel_int = constrain(abs(velocity), 0, 5000);
    
    if (velocity > 0) {
        motor.writeRegister(TMC5160_Reg::RAMPMODE, 1);
        motor.writeRegister(TMC5160_Reg::VMAX, vel_int);
    } else {
        motor.writeRegister(TMC5160_Reg::RAMPMODE, 2);
        motor.writeRegister(TMC5160_Reg::VMAX, vel_int);
    }

    Serial.print("Target: ");
    Serial.print(target);
    Serial.print(" | Error: ");
    Serial.print(error);
    Serial.print(" | Velocity: ");
    Serial.print(vel_int);
    Serial.print(" | APPS: ");
    Serial.print(raw);
    Serial.print(" | TPS: ");
    Serial.println(raw_throttle);

    // Runs at 10Hz (every 100ms)
    vTaskDelayUntil(&xTaskDelayTick, 100 / portTICK_PERIOD_MS);
  }
}

// ---------------------------------------------------------
// TASK 2: Watchdog Heartbeat (Runs every 10ms)
// ---------------------------------------------------------
void HeartbeatTask(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = 10 / portTICK_PERIOD_MS; // 10ms loop
  bool toggleState = false;

  while (true) {
    toggleState = !toggleState;

    // Toggle pin 1 if alive, otherwise force LOW
    if (isAlive1) {
      digitalWrite(HEARTBEAT_1_PIN, toggleState);
    } else {
      digitalWrite(HEARTBEAT_1_PIN, LOW);
    }

    // Toggle pin 2 if alive, otherwise force LOW
    if (isAlive2) {
      digitalWrite(HEARTBEAT_2_PIN, toggleState);
    } else {
      digitalWrite(HEARTBEAT_2_PIN, LOW);
    }

    // Wait exactly 10ms
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
      Serial.println(">>> Heartbeat 1 (Level 1) STOPPED");
    } 
    else if (c == 'w') {
      isAlive2 = false;
      Serial.println(">>> Heartbeat 2 (Level 2) STOPPED");
    }
    else if (c == 'r') { // Added a reset key just in case
      isAlive1 = true;
      isAlive2 = true;
      Serial.println(">>> Heartbeats RESTARTED");
    }
  }
  
  delay(50); // Small delay to prevent UTD polling
}