#include <Arduino.h>
#include <SPI.h>

#include <ArxTypeTraits.h>
#include <Bitfield.h>
#include <TMC5160.h>
#include <TMC5160_registers.h>

// Watchdog heartbeat pins.
constexpr int HEARTBEAT_1_PIN = 14;
constexpr int HEARTBEAT_2_PIN = 27;

// TMC5160 SPI chip-select pin.
constexpr uint8_t SPI_CS = 5;

// Set this to the ESP32 pin connected to TMC5160 DRV_ENN/EN, if used.
// The enable pin is active-low. Leave as -1 if the driver enable is tied low.
constexpr int SPI_DRV_ENN = -1;

// Single-APPS bench mode.
// APPS plausibility checks and TPS feedback are intentionally disabled for now.
constexpr int PEDAL_PIN = 34;
constexpr int APPS_ADC_CLOSED = 0;
constexpr int APPS_ADC_OPEN = 2500;
constexpr float THROTTLE_CLOSED_STEPS = 0.0f;
constexpr float THROTTLE_OPEN_STEPS = 200.0f;
constexpr uint32_t APPS_CONTROL_PERIOD_MS = 20;
constexpr uint32_t TELEMETRY_PERIOD_MS = 100;

volatile bool isAlive1 = true;
volatile bool isAlive2 = true;

TMC5160_SPI motor(SPI_CS);

// Watchdog heartbeat L1: every 15 ms, timeout roughly 30 ms.
void HeartbeatTask1(void *pvParameters) {
  (void)pvParameters;

  Serial.println(">>> HB1 task started");
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(15);
  uint32_t count = 0;

  while (true) {
    if (isAlive1) {
      digitalWrite(HEARTBEAT_1_PIN, HIGH);
      delayMicroseconds(1000);
      digitalWrite(HEARTBEAT_1_PIN, LOW);
    }

    if (++count % 67 == 0) {
      Serial.print("HB1 tick #");
      Serial.println(count);
    }

    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

// Watchdog heartbeat L2: every 50 ms, timeout roughly 150 ms.
void HeartbeatTask2(void *pvParameters) {
  (void)pvParameters;

  Serial.println(">>> HB2 task started");
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(50);
  uint32_t count = 0;

  while (true) {
    if (isAlive2) {
      digitalWrite(HEARTBEAT_2_PIN, HIGH);
      delayMicroseconds(1000);
      digitalWrite(HEARTBEAT_2_PIN, LOW);
    }

    if (++count % 20 == 0) {
      Serial.print("HB2 tick #");
      Serial.println(count);
    }

    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

static void handleSerialCommands() {
  if (Serial.available() <= 0) {
    return;
  }

  const char c = Serial.read();

  if (c == 'q') {
    isAlive1 = false;
    Serial.println("\n>>> Heartbeat 1 (Level 1) STOPPED");
  } else if (c == 'w') {
    isAlive2 = false;
    Serial.println("\n>>> Heartbeat 2 (Level 2) STOPPED");
  } else if (c == 'r') {
    isAlive1 = true;
    isAlive2 = true;
    Serial.println("\n>>> Heartbeats RESTARTED");
  }
}

static void printHex32(uint32_t value) {
  char buffer[11];
  snprintf(buffer, sizeof(buffer), "0x%08lX", static_cast<unsigned long>(value));
  Serial.print(buffer);
}

static TMC5160_Reg::IOIN_Register readTmcIoin() {
  TMC5160_Reg::IOIN_Register ioin = {0};
  ioin.value = motor.readRegister(TMC5160_Reg::IO_INPUT_OUTPUT);
  return ioin;
}

static float mapFloat(float value, float inMin, float inMax, float outMin, float outMax) {
  return (value - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}

static float targetStepsFromApps(int rawApps) {
  const int constrainedRaw = constrain(rawApps, APPS_ADC_CLOSED, APPS_ADC_OPEN);
  return mapFloat(constrainedRaw,
                  APPS_ADC_CLOSED,
                  APPS_ADC_OPEN,
                  THROTTLE_CLOSED_STEPS,
                  THROTTLE_OPEN_STEPS);
}

static void printTmcDiagnostics() {
  const TMC5160_Reg::IOIN_Register ioin = readTmcIoin();
  const uint8_t version = static_cast<uint8_t>(ioin.version);
  const bool driverDisabled = static_cast<bool>(ioin.drv_enn);
  const uint32_t gstat = motor.readRegister(TMC5160_Reg::GSTAT);
  const uint32_t drvStatus = motor.readRegister(TMC5160_Reg::DRV_STATUS);
  const uint32_t xactual = motor.readRegister(TMC5160_Reg::XACTUAL);
  const uint32_t xtarget = motor.readRegister(TMC5160_Reg::XTARGET);

  Serial.print("TMC IOIN=");
  printHex32(ioin.value);
  Serial.print(" version=0x");
  Serial.print(version, HEX);
  Serial.print(" drv_enn=");
  Serial.print(driverDisabled ? 1 : 0);
  Serial.print(" GSTAT=");
  printHex32(gstat);
  Serial.print(" DRV_STATUS=");
  printHex32(drvStatus);
  Serial.print(" XACTUAL=");
  printHex32(xactual);
  Serial.print(" XTARGET=");
  printHex32(xtarget);

  if (ioin.value == 0 || ioin.value == 0xFFFFFFFF) {
    Serial.print("  <-- no useful SPI reply");
  } else if (version != motor.IC_VERSION) {
    Serial.print("  <-- unexpected TMC version");
  }

  Serial.println();
}

static void configureMotor() {
  if (SPI_DRV_ENN >= 0) {
    pinMode(SPI_DRV_ENN, OUTPUT);
    digitalWrite(SPI_DRV_ENN, LOW);
  }

  SPI.begin();

  const TMC5160_Reg::IOIN_Register ioinBeforeBegin = readTmcIoin();
  const uint8_t versionBeforeBegin = static_cast<uint8_t>(ioinBeforeBegin.version);
  Serial.print("TMC5160 pre-init IOIN=");
  printHex32(ioinBeforeBegin.value);
  Serial.print(" version=0x");
  Serial.println(versionBeforeBegin, HEX);

  TMC5160::PowerStageParameters powerStageParams;
  TMC5160::MotorParameters motorParams;
  motorParams.globalScaler = 255;
  motorParams.irun = 18;
  motorParams.ihold = 12;

  const bool motorBeginOk = motor.begin(powerStageParams, motorParams, TMC5160::NORMAL_MOTOR_DIRECTION);
  Serial.print("TMC5160 begin: ");
  Serial.println(motorBeginOk ? "ok" : "failed");

  // setTargetPosition() requires positioning mode.
  motor.setRampMode(TMC5160::POSITIONING_MODE);
  motor.setMaxSpeed(400);
  motor.setAcceleration(500);

  printTmcDiagnostics();
}

void setup() {
  Serial.begin(115200);
  delay(200);

  analogReadResolution(12);
  analogSetPinAttenuation(PEDAL_PIN, ADC_11db);
  pinMode(PEDAL_PIN, INPUT);

  pinMode(HEARTBEAT_1_PIN, OUTPUT);
  pinMode(HEARTBEAT_2_PIN, OUTPUT);
  digitalWrite(HEARTBEAT_1_PIN, LOW);
  digitalWrite(HEARTBEAT_2_PIN, LOW);

  Serial.println("--- System Boot ---");
  Serial.println("Send 'q' to stop heartbeat Level 1");
  Serial.println("Send 'w' to stop heartbeat Level 2");
  Serial.println("Send 'r' to restart heartbeats");
  Serial.println("Single APPS bench mode: APPS2/TPS plausibility checks disabled");

  xTaskCreatePinnedToCore(HeartbeatTask1, "HeartbeatTask1", 2048, nullptr, 2, nullptr, 0);
  xTaskCreatePinnedToCore(HeartbeatTask2, "HeartbeatTask2", 2048, nullptr, 2, nullptr, 0);

  configureMotor();

  Serial.println("starting up");

  delay(1000);
}

void loop() {
  handleSerialCommands();

  const uint32_t now = millis();

  static uint32_t tControl = 0;
  static uint32_t tTelemetry = 0;
  static uint32_t tDiagnostics = 0;
  static int rawApps = 0;
  static float target = THROTTLE_CLOSED_STEPS;

  if (now - tControl >= APPS_CONTROL_PERIOD_MS) {
    tControl = now;
    rawApps = analogRead(PEDAL_PIN);
    target = targetStepsFromApps(rawApps);
    motor.setTargetPosition(target);
  }

  if (now - tTelemetry >= TELEMETRY_PERIOD_MS) {
    tTelemetry = now;

    const float xactual = motor.getCurrentPosition();
    const float vactual = motor.getCurrentSpeed();

    Serial.print("APPS raw : ");
    Serial.print(rawApps);
    Serial.print("\ttarget position : ");
    Serial.print(target);
    Serial.print("\tcurrent position : ");
    Serial.print(xactual);
    Serial.print("\tcurrent speed : ");
    Serial.println(vactual);
  }

  if (now - tDiagnostics >= 1000) {
    tDiagnostics = now;
    printTmcDiagnostics();
  }
}
