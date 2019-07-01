#include "include.h"

int gear;
int coding;

/*
 * 编码开关
 * ftmn 相应的编码器通道     FTM1/FTM2
 * threshold    阈值   调节编码器转动的灵活度
 *
 * PTC6 为拨码开关引脚
 */
int CodingSwitch(FTM_Type * ftmn, u8 threshold){
  gear = 0;
  if(!IsMotorVoltage()){
    PIDMotorLeft.sumError = 0;
    PIDMotorRight.sumError = 0;        //当电机电源开关关闭时 消除电机PID 积分累计
    
    
    coding = FTM_AB_Get(ftmn);
    if(coding > 0){
      while(coding > threshold){
         gear++;
         coding -= threshold;
      }
    }else{

      while(coding < -1*threshold){
        gear--;
        coding += threshold;
      }
    }

  }

  time_delay_ms(20);
  return gear;
}




