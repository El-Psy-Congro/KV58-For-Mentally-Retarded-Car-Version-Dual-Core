/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
【平    台】龙邱KV58F24智能车VD母板
【编    写】CHIUSIR
【E-mail  】chiusir@163.com
【软件版本】V1.0
【最后更新】2017年12月15日
【相关信息参考下列地址】
【网    站】http://www.lqist.cn
【淘宝店铺】http://shop36265907.taobao.com
------------------------------------------------
【dev.env.】IAR7.80.4
【Target  】MKV58F1M0VLQ24
【Crystal 】 50.000Mhz
【busclock】137.500MHz
【pllclock】275.000MHz
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/

#include "include.h"

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
【作  者】CHIUSIR
【功能说明】初始化LED所用IO口
【软件版本】V1.0
【最后更新】2017年11月24日 
【函数名】
【返回值】无
【参数值】无
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
void LED_Init(void)
{  
  //-----端口初始化----//
  /*
   * led
   */
  GPIO_Init(GPIOD,15,GPO,1);
  GPIO_Init(GPIOC,16,GPO,1);


  GPIO_Init(GPIOD,7,GPO,1);
  GPIO_Init(GPIOC,8,GPO,1);


  /*
   * Ultrasonic
   */
  GPIO_Init(GPIOC,14,GPO,0);
  GPIO_Init(GPIOC,15,GPI,0);
}
/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
【作  者】CHIUSIR
【功能说明】控制IO输出高低电平
【软件版本】V1.0
【最后更新】2017年11月24日 
【函数名】
【返回值】无
【参数值】
LEDn_e ledno, 编号
LEDs_e sta 状态，亮灭
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
【作  者】CHIUSIR
【功能说明】测试LED亮灭，流水灯
【软件版本】V1.0
【最后更新】2017年11月24日 
【函数名】
【返回值】无
【参数值】无
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

