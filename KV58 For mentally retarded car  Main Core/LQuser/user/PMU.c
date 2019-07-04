#include "include.h"

uint16_t
  voltageMotor = 0,
  voltageServo = 0;


/*
 * 检测电机电源开关是否打开（是否供电）
 */
bool IsMotorVoltage(){
  voltageMotor = ADC0_Ave(ADC0_SE8,ADC_12bit,10)*22/52;
  if(voltageMotor<70){
    PIDMotorLeft.sumError = 0;
    PIDMotorRight.sumError = 0;        //当电机电源开关关闭时 消除电机PID 积分累计
    return false;
  }
  return true;
}


/*
 * 检测舵机供电是否充足
 */
bool IsServoVoltage(){
  voltageServo = ADC0_Ave(ADC0_DP0,ADC_12bit,10)*22/52;
  if(voltageServo < 600){
    return false;
  }
  return true;
}
