#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <TMC5160.h>
#include <TMC5160_registers.h>

// Safe placement: Define these AFTER all system headers are loaded
#ifndef min
#define min(a,b) ((a)<(b)?(a):(b))
#endif
#ifndef max
#define max(a,b) ((a)>(b)?(a):(b))
#endif

#define MS_TICKS(x) ((x) / portTICK_PERIOD_MS)

// --- WiFi Credentials ---
const char* ssid = "ESP32_Throttle_Tuner";
const char* password = "password123"; // Make sure your phone/laptop uses this to connect

// --- Web Server & Socket ---
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// --- Hardware Pins ---
const uint8_t SPI_CS = 5;
const int PEDAL_PIN = 34;      // APPS sensor pin
const int PEDAL_2_PIN = 33;    // APPS2 sensor pin (Currently unused in logic)
const int THROTTLE_PIN = 32;   // TPPS sensor pin
const int HEARTBEAT_1_PIN = 14; 
const int HEARTBEAT_2_PIN = 27;

// --- Global State & Telemetry Variables ---
volatile bool isAlive1 = true;
volatile bool isAlive2 = true;
float Kp = 10.0;

// Shared variables for the web dashboard (Volatile because modified by Core 1, read by Core 0)
volatile float tel_target = 0;
volatile float tel_throttle = 0;
volatile float tel_error = 0;
volatile int tel_velocity = 0;
volatile int tel_apps = 0;
volatile int tel_tps = 0;

TMC5160_SPI motor = TMC5160_SPI(SPI_CS);

// --- HTML Dashboard (Stored in Program Memory) ---
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html>
<head>
  <title>Drive-By-Wire Tuner</title>
  <style>
    body { font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif; background-color: #0f111a; color: #fff; text-align: center; margin: 0; padding: 20px;}
    .grid { display: flex; flex-wrap: wrap; justify-content: center; gap: 15px; margin-bottom: 20px;}
    .card { background-color: #1e2130; padding: 15px; border-radius: 8px; width: 160px; box-shadow: 0 4px 6px rgba(0,0,0,0.3); border-top: 3px solid #3b4252;}
    .card h3 { margin: 0 0 10px 0; font-size: 1rem; color: #8892b0; text-transform: uppercase; letter-spacing: 1px;}
    .card h2 { margin: 0; font-size: 2rem; color: #64ffda; }
    .danger { color: #ff5555; }
    .chart-container { background-color: #1e2130; padding: 20px; border-radius: 8px; display: inline-block; box-shadow: 0 4px 6px rgba(0,0,0,0.3); }
    canvas { background-color: #0f111a; border-radius: 4px; border: 1px solid #3b4252; }
    .legend { display: flex; justify-content: center; gap: 20px; margin-top: 10px; }
    .legend-item { display: flex; align-items: center; gap: 5px; font-weight: bold;}
    .dot-target { height: 12px; width: 12px; background-color: #64ffda; border-radius: 50%; }
    .dot-throttle { height: 12px; width: 12px; background-color: #ffb86c; border-radius: 50%; }
  </style>
</head>
<body>
  <h1 style="color: #e6e9f0; margin-bottom: 5px;">Drive-By-Wire Telemetry</h1>
  <h3 class="danger">Kp Gain: <span id="kp">10.0</span></h3>

  <div class="grid">
    <div class="card"><h3>APPS Raw</h3><h2 id="apps">0</h2></div>
    <div class="card"><h3>TPS Raw</h3><h2 id="tps">0</h2></div>
    <div class="card"><h3>Velocity</h3><h2 id="velocity">0</h2></div>
    <div class="card"><h3>Error</h3><h2 id="error" style="color: #ff5555;">0.0</h2></div>
  </div>

  <div class="chart-container">
    <canvas id="liveChart" width="800" height="300"></canvas>
    <div class="legend">
      <div class="legend-item"><div class="dot-target"></div> Target Position</div>
      <div class="legend-item"><div class="dot-throttle"></div> Actual Throttle</div>
    </div>
  </div>

  <script>
    var gateway = `ws://${window.location.hostname}/ws`;
    var websocket;
    
    // Graphing Variables (100 points at 10Hz = 10 seconds of data)
    const MAX_POINTS = 100;
    let targetHistory = new Array(MAX_POINTS).fill(0);
    let throttleHistory = new Array(MAX_POINTS).fill(0);
    const canvas = document.getElementById('liveChart');
    const ctx = canvas.getContext('2d');

    function drawGraph() {
      ctx.clearRect(0, 0, canvas.width, canvas.height);
      
      // Draw grid lines
      ctx.strokeStyle = '#2a2e3f';
      ctx.lineWidth = 1;
      for(let i=0; i<5; i++) {
        let y = (canvas.height / 4) * i;
        ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(canvas.width, y); ctx.stroke();
      }

      function plotLine(dataArray, color) {
        ctx.beginPath();
        ctx.strokeStyle = color;
        ctx.lineWidth = 3;
        ctx.lineJoin = 'round';
        for(let i=0; i<dataArray.length; i++) {
          let x = (i / (MAX_POINTS - 1)) * canvas.width;
          // Scale 0-200 to canvas height (inverted for standard graph orientation)
          let y = canvas.height - ((dataArray[i] / 200) * canvas.height);
          if(i === 0) ctx.moveTo(x, y);
          else ctx.lineTo(x, y);
        }
        ctx.stroke();
      }

      plotLine(targetHistory, '#64ffda'); // Cyan line for Target
      plotLine(throttleHistory, '#ffb86c'); // Orange line for Throttle
    }

    function initWebSocket() {
      websocket = new WebSocket(gateway);
      websocket.onmessage = function(event) {
        var data = JSON.parse(event.data);
        
        // Update text cards
        document.getElementById('apps').innerText = data.apps;
        document.getElementById('tps').innerText = data.tps;
        document.getElementById('velocity').innerText = data.velocity;
        document.getElementById('error').innerText = data.error.toFixed(2);
        document.getElementById('kp').innerText = data.kp.toFixed(2);

        // Update arrays and redraw graph
        targetHistory.push(data.target);
        throttleHistory.push(data.throttle);
        targetHistory.shift();
        throttleHistory.shift();
        
        requestAnimationFrame(drawGraph);
      };
    }
    
    drawGraph();
    window.addEventListener('load', initWebSocket);
  </script>
</body>
</html>
)rawliteral";

// ---------------------------------------------------------
// TASK 1: Stepper Motor Control (Core 1, Priority 1)
// ---------------------------------------------------------
void StepperTask(void *pvParameters){
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

  motor.setRampMode(TMC5160::VELOCITY_MODE);
  motor.setMaxSpeed(1000);
  motor.setAcceleration(2000);

  vTaskDelay(MS_TICKS(1000)); 

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

    // Update global variables for the WebSocket to read safely
    tel_apps = raw;
    tel_tps = raw_throttle;
    tel_target = target;
    tel_throttle = scaled_throttle;
    tel_error = error;
    tel_velocity = vel_int;

    vTaskDelayUntil(&xTaskDelayTick, 100 / portTICK_PERIOD_MS);
  }
}

// ---------------------------------------------------------
// TASK 2: Watchdog Heartbeat (Core 1, Priority 3 - HIGHEST)
// ---------------------------------------------------------
void HeartbeatTask(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = 10 / portTICK_PERIOD_MS; 
  bool toggleState = false;

  while (true) {
    toggleState = !toggleState;
    digitalWrite(HEARTBEAT_1_PIN, isAlive1 ? toggleState : LOW);
    digitalWrite(HEARTBEAT_2_PIN, isAlive2 ? toggleState : LOW);
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

// ---------------------------------------------------------
// TASK 3: WebSocket Broadcaster (Core 0, Priority 1)
// ---------------------------------------------------------
void WebTelemetryTask(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  while(true) {
    // Only compile and send JSON if someone is actually looking at the webpage
    if(ws.count() > 0) {
      char payload[256];
      snprintf(payload, sizeof(payload), 
        "{\"apps\":%d, \"tps\":%d, \"target\":%.2f, \"throttle\":%.2f, \"error\":%.2f, \"velocity\":%d, \"kp\":%.2f}",
        tel_apps, tel_tps, tel_target, tel_throttle, tel_error, tel_velocity, Kp);
      
      ws.textAll(payload);
    }
    // Broadcast at ~10Hz
    vTaskDelayUntil(&xLastWakeTime, 100 / portTICK_PERIOD_MS);
  }
}

void setup() {
  Serial.begin(115200);
  
  pinMode(HEARTBEAT_1_PIN, OUTPUT);
  pinMode(HEARTBEAT_2_PIN, OUTPUT);
  digitalWrite(HEARTBEAT_1_PIN, LOW);
  digitalWrite(HEARTBEAT_2_PIN, LOW);

  // Set up ESP32 as a WiFi Access Point
  Serial.println("\n--- System Boot ---");
  Serial.println("Starting WiFi AP...");
  WiFi.softAP(ssid, password);
  Serial.print("Connect your device to WiFi network 'ESP32_Throttle_Tuner'");
  Serial.print("\nThen navigate your browser to: http://");
  Serial.println(WiFi.softAPIP());

  // Set up Web Server
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html", index_html);
  });
  server.addHandler(&ws);
  server.begin();

  Serial.println("--- Serial Commands Available ---");
  Serial.println(" 'q' = Kill Heartbeat 1");
  Serial.println(" 'w' = Kill Heartbeat 2");
  Serial.println(" 'r' = Restart both Heartbeats");
  Serial.println(" '+' = Increase Kp by 1.0");
  Serial.println(" '-' = Decrease Kp by 1.0");

  // Create Tasks
  // BOTH control tasks go to Core 1. Heartbeat gets priority 3, Stepper gets 1.
  xTaskCreatePinnedToCore((void(*)(void*))&HeartbeatTask, "Heartbeat", 2048, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore((void(*)(void*))&StepperTask, "Stepper", 8192, NULL, 1, NULL, 1);
  
  // The Web Broadcaster goes to Core 0, alongside the WiFi stack.
  xTaskCreatePinnedToCore((void(*)(void*))&WebTelemetryTask, "WebTel", 4096, NULL, 1, NULL, 0);
}

void loop() {
  // Serial debugging and live PID tuning remains intact
  if (Serial.available() > 0) {
    char c = Serial.read();
    if (c == 'q') { isAlive1 = false; Serial.println(">>> Heartbeat 1 STOPPED"); } 
    else if (c == 'w') { isAlive2 = false; Serial.println(">>> Heartbeat 2 STOPPED"); }
    else if (c == 'r') { isAlive1 = true; isAlive2 = true; Serial.println(">>> Heartbeats RESTARTED"); }
    else if (c == '+') { Kp += 1.0; Serial.print(">>> New Kp: "); Serial.println(Kp); }
    else if (c == '-') { Kp -= 1.0; Serial.print(">>> New Kp: "); Serial.println(Kp); }
  }
  delay(50);
}