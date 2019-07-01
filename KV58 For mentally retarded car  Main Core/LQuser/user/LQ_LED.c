/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
��ƽ    ̨������KV58F24���ܳ�VDĸ��
����    д��CHIUSIR
��E-mail  ��chiusir@163.com
������汾��V1.0
�������¡�2017��12��15��
�������Ϣ�ο����е�ַ��
����    վ��http://www.lqist.cn
���Ա����̡�http://shop36265907.taobao.com
------------------------------------------------
��dev.env.��IAR7.80.4
��Target  ��MKV58F1M0VLQ24
��Crystal �� 50.000Mhz
��busclock��137.500MHz
��pllclock��275.000MHz
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/

#include "include.h"

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
����  �ߡ�CHIUSIR
������˵������ʼ��LED����IO��
������汾��V1.0
�������¡�2017��11��24�� 
����������
������ֵ����
������ֵ����
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
void LED_Init(void)
{  
  //-----�˿ڳ�ʼ��----//
  GPIO_Init(GPIOD,15,GPO,1);
  GPIO_Init(GPIOC,16,GPO,1);
  GPIO_Init(GPIOD,7,GPO,1);
  GPIO_Init(GPIOC,8,GPO,1);

}
/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
����  �ߡ�CHIUSIR
������˵��������IO����ߵ͵�ƽ
������汾��V1.0
�������¡�2017��11��24�� 
����������
������ֵ����
������ֵ��
LEDn_e ledno, ���
LEDs_e sta ״̬������
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
void LED_Ctrl(LEDn_e ledno, LEDs_e sta)
{
  switch(ledno) 
  {
  case LED0:
    if(sta==ON)       PTD15_OUT=0; //Turn on target LED1
    else if(sta==OFF) PTD15_OUT=1; //Turn off target LED1
    else if(sta==RVS) GPIO_Reverse (GPIOD, 15);//Toggle on target LED1
    break;
  case LED1:
    if(sta==ON)       PTC16_OUT=0; //Turn on target LED3
    else if(sta==OFF) PTC16_OUT=1; //Turn off target LED3
    else if(sta==RVS) GPIO_Reverse (GPIOC, 16);//Toggle on target LED3
    break;   
  case LEDALL:
    if(sta==ON) 
    {       
      PTD15_OUT=0;
      PTC16_OUT=0; //Turn on target LED1

    }
    else if(sta==OFF)
    { 
      PTB15_OUT=1;
      PTB16_OUT=1;   //Turn off target LED1
    }
    else if(sta==RVS)
    {       
      GPIO_Reverse (GPIOD, 15); //Toggle on target LED1
      GPIO_Reverse (GPIOC, 16); //Toggle on target LED2
    }
    break;
  default:
    break;    
  }   
}
/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
����  �ߡ�CHIUSIR
������˵��������LED������ˮ��
������汾��V1.0
�������¡�2017��11��24�� 
����������
������ֵ����
������ֵ����
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
void Test_LED(void)
{
  u8 i=0;
  //Init output LED GPIO. //
  LED_Init();
  LED_Ctrl(LEDALL, OFF);    
  while (1)
  {          
    for(i=0;i<10;i++)
    {
      LED_Ctrl(LED1, RVS); 
      time_delay_ms(50*i);
      LED_Ctrl(LED1, OFF); 
      LED_Ctrl(LED2, RVS); 
      time_delay_ms(50*i);
      LED_Ctrl(LED2, OFF); 
      LED_Ctrl(LED3, RVS); 
      time_delay_ms(50*i);
      LED_Ctrl(LED3, OFF); 
      LED_Ctrl(LED0, RVS); 
      time_delay_ms(50*i);
      LED_Ctrl(LEDALL, OFF); 
      time_delay_ms(500);
    }
    LED_Ctrl(LEDALL, OFF); 
    time_delay_ms(500);
    LED_Ctrl(LEDALL, ON); 
    time_delay_ms(500);
  }
}

