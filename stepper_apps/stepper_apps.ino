
#include <ArxTypeTraits.h>

#include <Bitfield.h>
#include <TMC5160.h>
#include <TMC5160_registers.h>

#include <Arduino.h>

#define MS_TICKS(x) ((x) / portTICK_PERIOD_MS)

//interupt 
hw_timer_t *Timer0_Cfg = NULL;
const uint8_t SPI_CS = 5; // CS pin in SPI mode
//const uint8_t SPI_DRV_ENN = 8;  // DRV_ENN pin in SPI mode

const int PEDAL_PIN = 34; //APPS sensor pin
const int PEDAL_2_PIN = 33; //APPS2 sensor pin
const int THROTTLE_PIN = 32; //TPPS sensor pin - note 35 is not allowed????

float Kp = 10.0;


TMC5160_SPI motor = TMC5160_SPI(SPI_CS); //Use default SPI peripheral and SPI settings.

/*void IRAM_ATTR Timer0_ISR()
{
    //digitalWrite(LED, !digitalRead(LED));
    Serial.println("HELLO FROM THE TIMER :)))");
}*/

void StepperTask(){
  Serial.println("Hello from Stepper RTOS Task");
  //This sets the motor & driver parameters /!\ run the configWizard for your driver and motor for fine tuning !
    // This sets the motor & driver parameters /!\ run the configWizard for your driver and motor for fine tuning !

  TMC5160::PowerStageParameters powerStageParams; // defaults.
  TMC5160::MotorParameters motorParams;


  SPI.begin();
  motor.begin(powerStageParams, motorParams, TMC5160::NORMAL_MOTOR_DIRECTION);
  motorParams.globalScaler = 98; // Adapt to your driver and motor (check TMC5160 datasheet - "Selecting sense resistors")
  motorParams.irun = 31;
  motorParams.ihold = 16;
  motor.writeRegister(TMC5160_Reg::GCONF, 0x00000004);
  motor.writeRegister(TMC5160_Reg::GLOBAL_SCALER, 200);
  motor.writeRegister(TMC5160_Reg::IHOLD_IRUN, 0x00011F14);
  motor.writeRegister(TMC5160_Reg::RAMPMODE, 1);
  


  // ramp definition
  motor.setRampMode(TMC5160::VELOCITY_MODE);
  motor.setMaxSpeed(1000);
  motor.setAcceleration(2000);

  Serial.println("starting up");

  /*Timer0_Cfg = timerBegin(cpufreqmhz); //cpufreqmhz should be 240 (Mhz), but not hard coded - prescaler causes the timer to run at 1Mhz (our website says 0, cpufreqmhz, true but that is wrong if u see "see definition" )
  timerAttachInterrupt(Timer0_Cfg, &Timer0_ISR); //our website says ,,true but again it must be wrong 
  timerAlarm(Timer0_Cfg, 2000, true,0); //1Mhz/2000 = 500Hz (every 2ms)
  timerStart(Timer0_Cfg); */

  vTaskDelay(MS_TICKS(1000)); // Standstill for automatic tuning

  TickType_t xTaskDelayTick = xTaskGetTickCount();

  while (true){ //alternative for(;;)
    // uint32_t now = millis();

    bool dir = true;

    int raw = analogRead(PEDAL_PIN);

    float target = map(raw, 0, 2500, 0, 200); //2500 is max sensor reading after going through potential divider, 200 steps per revolution (1.8 deg per step)
    int raw_throttle = analogRead(THROTTLE_PIN);
    float error = 0;
    float scaled_throttle = 0;
    scaled_throttle = map(raw_throttle, 0, 4095, 0, 200);
    error = target - scaled_throttle;


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
    //Serial.print(" | Current position: ");
    //Serial.print(motor.getCurrentPosition());
    Serial.print(" | Error: ");
    Serial.print(error);
    Serial.print(" | Velocity: ");
    Serial.print(vel_int);
    Serial.print(" | APPS Signal: ");
    Serial.print(raw);

    //int raw_apps_2 = analogRead(PEDAL_2_PIN);
    //Serial.print(",");
    //Serial.print(raw_apps_2);
    

    //Throttle Position Sensor
    
    Serial.print(" | T: ");
    Serial.println(raw_throttle);

    vTaskDelayUntil(&xTaskDelayTick, 100 / portTICK_PERIOD_MS); //2
  }
}

void setup()
{
  // USB/debug serial coms
  Serial.begin(115200);
  while (!Serial)
    ;

  xTaskCreatePinnedToCore((void(*)(void*))&StepperTask,"StepperTask",8192,NULL,1,NULL,ARDUINO_RUNNING_CORE);
  

}

void loop()
{/*/
  // Serial input for adjusting PID Gains (might not work - code wont run maybe if it doesnt have input yet?)
  if (Serial.available()) {
    Kp = Serial.parseFloat();
    Serial.print("New Kp: ");
    Serial.println(Kp);
  }
  delay(100);
  */
}