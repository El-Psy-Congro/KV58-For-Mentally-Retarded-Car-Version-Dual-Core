#include "include.h"

int gear;
int coding;

/*
 * ���뿪��
 * ftmn ��Ӧ�ı�����ͨ��     FTM1/FTM2
 * threshold    ��ֵ   ���ڱ�����ת��������
 *
 * PTC6 Ϊ���뿪������
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




