#include <ModbusRTUSlave.h>                         //  
#define MODBUS_SERIAL Serial1                       //  ███╗░░░███╗░█████╗░██████╗░██████╗░██╗░░░██╗░██████╗
#define MODBUS_BAUD 115200                          //  ████╗░████║██╔══██╗██╔══██╗██╔══██╗██║░░░██║██╔════╝
#define MODBUS_CONFIG SERIAL_8N1                    //  ██╔████╔██║██║░░██║██║░░██║██████╦╝██║░░░██║╚█████╗░
#define MODBUS_UNIT_ID 1                            //  ██║╚██╔╝██║██║░░██║██║░░██║██╔══██╗██║░░░██║░╚═══██╗
const int16_t dePin = 10;                           //  ██║░╚═╝░██║╚█████╔╝██████╔╝██████╦╝╚██████╔╝██████╔╝
ModbusRTUSlave modbus(MODBUS_SERIAL, dePin);        //  ╚═╝░░░░░╚═╝░╚════╝░╚═════╝░╚═════╝░░╚═════╝░╚═════╝░
const uint8_t numHoldingRegisters = 6;              //
uint16_t holdingRegisters[numHoldingRegisters];     //

#include <TMCStepper.h>                             //  ▀▀█▀▀ █▀▄▀█ █▀▀ 
#define SERIAL_PORT Serial2                         //  ░░█░░ █░▀░█ █░░ 
#define R_SENSE 0.11f                               //  ░░▀░░ ▀░░░▀ ▀▀▀
TMC2208Stepper driver(&SERIAL_PORT, R_SENSE);       //    

#include <AccelStepper.h>                                                 //
#define EN_PIN           52                                               //  █▀▀█ █▀▀ █▀▀ █▀▀ █░░ 
#define DIR_PIN          50                                               //  █▄▄█ █░░ █░░ █▀▀ █░░ 
#define STEP_PIN         48                                               //  ▀░░▀ ▀▀▀ ▀▀▀ ▀▀▀ ▀▀▀
AccelStepper stepper = AccelStepper(stepper.DRIVER, STEP_PIN, DIR_PIN);   // 
constexpr uint32_t steps_per_mm = 16;                                     // 


void setup() {
  modbus.configureHoldingRegisters(holdingRegisters, numHoldingRegisters);  //  █▀▄▀█ █▀▀█ █▀▀▄ █▀▀▄ █░░█ █▀▀ 
  MODBUS_SERIAL.begin(MODBUS_BAUD, MODBUS_CONFIG);                          //  █░▀░█ █░░█ █░░█ █▀▀▄ █░░█ ▀▀█ 
  modbus.begin(MODBUS_UNIT_ID, MODBUS_BAUD, MODBUS_CONFIG);                 //  ▀░░░▀ ▀▀▀▀ ▀▀▀░ ▀▀▀░ ░▀▀▀ ▀▀▀
  holdingRegisters[0] = 500;                                                //

  SERIAL_PORT.begin(115200);      // HW UART drivers                            ▀▀█▀▀ █▀▄▀█ █▀▀ 
  driver.toff(5);                 // Enables driver in software                 ░░█░░ █░▀░█ █░░ 
  driver.rms_current(700);        // Set motor RMS current                      ░░▀░░ ▀░░░▀ ▀▀▀               
  driver.microsteps(16);          // Set microsteps to 1/16th                   
  driver.pwm_autoscale(true);     // Needed for stealthChop

  stepper.setMaxSpeed(50*steps_per_mm);                                      // █▀▀█ █▀▀ █▀▀ █▀▀ █░░ 
  stepper.setAcceleration(1000*steps_per_mm);                                // █▄▄█ █░░ █░░ █▀▀ █░░ 
  stepper.setEnablePin(EN_PIN);                                              // ▀░░▀ ▀▀▀ ▀▀▀ ▀▀▀ ▀▀▀
  stepper.setPinsInverted(false, false, true);                               //
}


void loop() {
  
  holdingRegisters[0] = stepper.currentPosition() / steps_per_mm;
  holdingRegisters[1] = stepper.distanceToGo() / steps_per_mm;
  holdingRegisters[3] = driver.rms_current();

  if (holdingRegisters[2] != stepper.targetPosition() / steps_per_mm) {
    stepper.moveTo(holdingRegisters[2] * steps_per_mm); // Move 100mm
    stepper.enableOutputs();
  }

  if (stepper.run() != true) {
    stepper.disableOutputs();
  }

  modbus.poll();
  //analogWrite(3, holdingRegisters[1]);      // отправляем на мосфет
  delay(4);  
}
