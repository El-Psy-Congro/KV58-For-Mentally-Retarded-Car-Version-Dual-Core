#include "include.h"

uint16_t
  voltageMotor = 0,
  voltageServo = 0;


/*
 * �������Դ�����Ƿ�򿪣��Ƿ񹩵磩
 */
bool IsMotorVoltage(){
  voltageMotor = ADC0_Ave(ADC0_SE8,ADC_12bit,10)*22/52;
  if(voltageMotor<70){
    PIDMotorLeft.sumError = 0;
    PIDMotorRight.sumError = 0;        //�������Դ���عر�ʱ �������PID �����ۼ�
    return false;
  }
  return true;
}


/*
 * ����������Ƿ����
 */
bool IsServoVoltage(){
  voltageServo = ADC0_Ave(ADC0_DP0,ADC_12bit,10)*22/52;
  if(voltageServo < 600){
    return false;
  }
  return true;
}
