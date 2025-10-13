//  Карта Modbus-регистров
//
//  Регистр | Значение
//     0    | Уставка тока драйвера
//     1    | Текущий ток драйвера
//     2    | Максимальная скорость
//     3    | Максимальное ускорение
//    10    | Текущаяя позиция
//    11    | Дистанция до цели
//    12    | Цель

#include <ModbusRTUSlave.h>                         //  
#define MODBUS_SERIAL Serial1                       //  ███╗░░░███╗░█████╗░██████╗░██████╗░██╗░░░██╗░██████╗
#define MODBUS_BAUD 115200                          //  ████╗░████║██╔══██╗██╔══██╗██╔══██╗██║░░░██║██╔════╝
#define MODBUS_CONFIG SERIAL_8N1                    //  ██╔████╔██║██║░░██║██║░░██║██████╦╝██║░░░██║╚█████╗░
#define MODBUS_UNIT_ID 1                            //  ██║╚██╔╝██║██║░░██║██║░░██║██╔══██╗██║░░░██║░╚═══██╗
const int16_t dePin = 10;                           //  ██║░╚═╝░██║╚█████╔╝██████╔╝██████╦╝╚██████╔╝██████╔╝
ModbusRTUSlave modbus(MODBUS_SERIAL, dePin);        //  ╚═╝░░░░░╚═╝░╚════╝░╚═════╝░╚═════╝░░╚═════╝░╚═════╝░
const uint8_t numHoldingRegisters = 14;             //
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
  holdingRegisters[0] = 300;                                                //

  SERIAL_PORT.begin(115200);                      // HW UART drivers                            ▀▀█▀▀ █▀▄▀█ █▀▀ 
  driver.toff(5);                                 // Enables driver in software                 ░░█░░ █░▀░█ █░░ 
  driver.rms_current(holdingRegisters[0]);        // Set motor RMS current                      ░░▀░░ ▀░░░▀ ▀▀▀               
  driver.microsteps(16);                          // Set microsteps to 1/16th                   
  driver.pwm_autoscale(true);                     // Needed for stealthChop

  stepper.setMaxSpeed(50*steps_per_mm);                                      // █▀▀█ █▀▀ █▀▀ █▀▀ █░░ 
  stepper.setAcceleration(1000*steps_per_mm);                                // █▄▄█ █░░ █░░ █▀▀ █░░ 
  stepper.setEnablePin(EN_PIN);                                              // ▀░░▀ ▀▀▀ ▀▀▀ ▀▀▀ ▀▀▀
  stepper.setPinsInverted(false, false, true);                               //
}

  int16_t razn;

void loop() {
  holdingRegisters[10] = stepper.currentPosition() / steps_per_mm;
  holdingRegisters[11] = stepper.distanceToGo() / steps_per_mm;
  holdingRegisters[1] = driver.rms_current();

  razn = holdingRegisters[0] - holdingRegisters[1];
  razn = abs(razn);
  holdingRegisters[4] = razn;
  if (razn > 50) {
    driver.rms_current(holdingRegisters[0]);        // Set motor RMS current
  }

  if (holdingRegisters[2] != stepper.targetPosition() / steps_per_mm) {
    stepper.moveTo(holdingRegisters[12] * steps_per_mm); // Move 100mm
    stepper.enableOutputs();
  }

  if (stepper.run() != true) {
    stepper.disableOutputs();
  }

  modbus.poll();
  //analogWrite(3, holdingRegisters[1]);      // отправляем на мосфет
  delay(4);  
}
