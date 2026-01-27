
#include <ArxTypeTraits.h>

#include <Bitfield.h>
#include <TMC5160.h>
#include <TMC5160_registers.h>

#include <Arduino.h>


const uint8_t SPI_CS = 5; // CS pin in SPI mode
//const uint8_t SPI_DRV_ENN = 8;  // DRV_ENN pin in SPI mode

const int PEDAL_PIN = 34; //APPS sensor pin

TMC5160_SPI motor = TMC5160_SPI(SPI_CS); //Use default SPI peripheral and SPI settings.


void setup()
{
  // USB/debug serial coms
  Serial.begin(115200);

  //pinMode(SPI_DRV_ENN, OUTPUT); 
  //digitalWrite(SPI_DRV_ENN, LOW); // Active low

  // This sets the motor & driver parameters /!\ run the configWizard for your driver and motor for fine tuning !
  TMC5160::PowerStageParameters powerStageParams; // defaults.
  TMC5160::MotorParameters motorParams;
  motorParams.globalScaler = 98; // Adapt to your driver and motor (check TMC5160 datasheet - "Selecting sense resistors")
  motorParams.irun = 31;
  motorParams.ihold = 16;

  SPI.begin();
  motor.begin(powerStageParams, motorParams, TMC5160::NORMAL_MOTOR_DIRECTION);

  // ramp definition
  motor.setRampMode(TMC5160::POSITIONING_MODE);
  motor.setMaxSpeed(200);
  motor.setAcceleration(300);

  Serial.println("starting up");

  delay(1000); // Standstill for automatic tuning
}

void loop()
{
  uint32_t now = millis();
  bool dir = true;

  int raw = analogRead(PEDAL_PIN);

  float target = map(raw, 0, 2500, 0, 200); //2500 is max sensor reading after going through potential divider, 200 steps per revolution

  Serial.print("Target: ");
  Serial.print(target);

  motor.setTargetPosition(dir ? target : 0);

  Serial.print(" | Current position: ");
  Serial.print(motor.getCurrentPosition());
  Serial.print(" | Sensor Signal: ");
  Serial.println(raw);

  delay(1);
  
}