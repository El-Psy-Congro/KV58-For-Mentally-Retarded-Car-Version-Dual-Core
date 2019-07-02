#ifndef _PMU_H_
#define _PMU_H_

extern uint16_t
  voltageMotor,
  voltageServo;

bool IsMotorVoltage();
bool IsServoVoltage();

#endif
