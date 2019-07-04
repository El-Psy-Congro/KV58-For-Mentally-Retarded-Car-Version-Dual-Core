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
int CodingSwitch(int channelSelection, u8 threshold){
  gear = 0;
  if(!IsMotorVoltage() && !GPIO_Get(PTB20)){


    coding = channelSelection;
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

  time_delay_ms(1);
  return gear;
}




