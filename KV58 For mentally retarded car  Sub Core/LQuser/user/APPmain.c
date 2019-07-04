/*******************************************************************************
【平    台】龙邱KV58F24智能车VD母板
【编    写】CHIUSIR
【E-mail  】chiusir@163.com
【软件版本】V1.0
【最后更新】2018年3月28日
【相关信息参考下列地址】
【网    站】http://www.lqist.cn
【淘宝店铺】http://shop36265907.taobao.com
------------------------------------------------
【dev.env.】IAR7.80.4及以上版本
【Target  】MKV58F1M0VLQ24
【Crystal 】 50.000Mhz
【busclock】137.500MHz
【pllclock】275.000MHz
------------------------------------------------
没有完美的代码，只有不断的奉献，大家一起努力；
赠人玫瑰手留余香，欢迎大家反馈bug，将开源进行到底；
=============================================================
接口定义：
摄像头          单片机接口    
VCC             3.3V 
GND             GND
Y0-7            PTD0-7共8位并口      
HREF            PTD13
VSY             PTD14
PCK             PTD12
-------------------------------------------------------------
TSL1401模块     单片机接口
VCC             5V
GND             GND
SI              PTE27
SCK             PTE28
ADC             ADC0_SE8
ADC             ADC0_SE4B

SI              PTE2
SCK             PTE3
ADC             ADC0DP0
ADC             ADC0_SE5B
===============================================================
多路电感模块    单片机接口
VCC             3.3V
GND             GND
ADC通道         管脚关系     
ADC0_SE10       PTE5
ADC0_DP2        PTE4
ADC0_SE4A       PTE6
ADC0_DP3        PTE11
ADC0_SE11       PTE12
ADC0_DP1        PTE16
ADC0_SE9        PTE17
ADC0_SE5A       PTE18
-------------------------------------------------------------
电源监控或者它用
ADC0_SE7A       HSADC0CH6  
-------------------------------------------------------------
九轴模块        单片机接口   FLEXCOMM8
VCC             3V3
GND             GND
SDA1            PTD8
SCL1            PTD9
//////////////////////////////////////////////////////////通用部分功能//////////
电机驱动        单片机接口   
VCC             3.3V
FTM0_CH0  PTC1，电机驱动接口
FTM0_CH1  PTC2，电机驱动接口
FTM0_CH2  PTC3，电机驱动接口
FTM0_CH3  PTC4，电机驱动接口
-------------------------------------------------------------
舵机接口        单片机接口
VADJ            可调电源
GND             GND
FTM3_CH6        PTC10，舵机接口
FTM3_CH7        PTC11，舵机接口
-------------------------------------------------------------
龙邱512编码器   单片机接口     
VCC             3V3
GND             GND      
LSB/A           PTA12   
DIR/B           PTA13
LSB/A           PTB18
DIR/B           PTB19
-------------------------------------------------------------
OLED模块        单片机接口  核心板用的是PTC16-19
VCC             3.3V        用户自行修改初始化和管脚定义
GND             GND
OLED接口定义：
CLK --PTC16
SDA --PTC15
RST --PTC14
DC  --PTC13
TFT1.8接口定义：
RST --PTC12
DC  --PTC13
SDA --PTC14
CLK --PTC15
CS  --PTC16
-------------------------------------------------------------
蓝牙/USBTTL     单片机接口   
GND             GND
TX              PTE24   UART4_TX 接蓝牙的RX
RX              PTE25   UART4_RX 接蓝牙的TX
-------------------------------------------------------------
LED接口定义：
LED1            PTA17 核心板
LED2            PTC0  核心板
LED3            PTD15 核心板   
LED4            PTE29 核心板
LEDD1           PTC18 母板
LEDD2           PTC19 母板
-------------------------------------------------------------
KEY接口定义：
KEY0            PTB20  
KEY1            PTB21
KEY2            PTB22   //可中断触发
-------------------------------------------------------------
修改历史：
20180208-中断处理函数在startup_MKV58F24.s中，名字必须对应才能触发中断；
20180327-已经修改神眼摄像头的采集模式为DMA，分辨率为188*120，100db HDR；
20181120-修改九轴，6050的IIC通信问题，完善九轴测试。添加模拟IIC，可直接修改IIC使用引脚。
********************************************************************************/
#include "include.h"

//==========================================================================================  
//测试函数名称        测试内部模块及功能           智能车及应用开发               完成状况
//----------------------------------------------------------------------------------------- 
//Test_LED();         //测试GPIO输出口              LED显示及外设控制              已完成
//Test_OLED();       //测试模拟SPI功能              OLED模块功能                   已完成
//Test_GPIO_KEY();   //测试GPIO输入                 按键检测功能                   已完成 
//Test_GPIO_EXINT(); //测试GPIO输入及外部中断       按键、中断检测功能             已完成     
//Test_UART();       //测试UART及中断               蓝牙、USB转TTL调试及通信       已完成
//Test_ADC0();       //测试ADC采集功能              电磁传感器及电源电压监控       部分完成,HSADC不能通过
//Test_Servo();      //测试用FTM-PWM功能            数字舵机驱动控制               已完成
//Test_Motor();      //测试用FTM-PWM功能            电机驱动控制                   已完成
//Test_AB_Pulse_Cnt();//测试用FTM正交解码功能       编码器测速,电机舵机控制        已完成 
//Test_LPTMR_delay();//测试LPTMR功能                延时功能                       已完成
//Test_LPTMR_Counter();//测试LPTMR功能              计数功能                       已完成
//Test_DMA_Counter();//测试DMA计数功能              计数功能                       已完成
//TestOLED_TSL1401();//测试OLED和TSL1401功能        OLED显示屏及线阵摄像头         已完成
//TestOLED_MT9V034();//测试OLED和MT9V034功能        OLED显示屏及面阵摄像头动图     已完成
//Test_WDG();        //测试看门狗功能               程序监控                       已完成
//========================================================================================== 

void main(void)
{   
  PLL_Init(PLL250);         //设置内核及总线频率等
//  KEY_Init();               //按键及输入口初始化  
  LED_Init();               //LED初始化
  UART_Init(UART_5,115200); //串口初始化
  TFTSPI_Init();             //LCD初始化
  OV7725_Init();             //摄像头初始化
  PIT_Init(PIT0, 30);
  PIT_Init(PIT1, 100);

  
  TFTSPI_CLS(u16BLACK);
  EnableInterrupts
  LED_Ctrl(LEDALL, OFF);   
  BEE_OFF;
  //-----------------------------------------------------------------------------------------  
  //  测试函数都是死循环，每次只能开启一个
  //-----------------------------------------------------------------------------------------
  //Test_ADC0();         //测试ADC采集功能               电磁传感器及电源电压监控 
  //Test_UART();         //测试UART及中断                蓝牙、USB转TTL调试及通信
  // Test_LED();         //测试GPIO输出口                LED显示及外设控制        
  //Test_OLED();         //O测试模拟SPI功能              OLED模块功能   
  //Test_GPIO_KEY();     //测试GPIO输入                  按键检测功能              
  //Test_GPIO_EXINT();   //测试GPIO输入及外部中断        按键、中断检测功能  
  //Test_Servo();        //数字舵机测试
  //Test_Motor();        //直流电机驱动测试，            用龙邱全桥驱动板
  //Test_9AX();          //测试I2C及龙邱九轴
  //Test_MPU6050();      //测试I2C及6轴陀螺仪功能
 // Test_OV7725();         //TFT1.8寸彩色显示屏显示彩色图像  //已完成
  //Test_PIT();          //测试PIT定时中断功能 
  //Test_AB_Pulse_Cnt(); //测试编码器正交解码功能
  //Test_LPTMR_delay();  //测试LPTMR延时功能                
  //Test_LPTMR_Counter();//测试LPTMR计数功能   
  //Test_DMA_Counter();  //测试DMA计数功能   
  //TFTSPI_Test();          //测试龙邱TFT1.8吋SPI彩屏
  //Test_OLED();  
  //-----------------------------------------------------------------------------------------
  //LCD_Show_LQLogo();
  while(1)
    {   
      if(Field_Over_Flag)    //完成一场图像采集后显示并发送数据到上位机
    {
      //串口发送数据非常慢，注释掉OLED刷新很快
      //UARTSendPicture(Image_Data);     //发送数据到上位机，注意协议格式，不同的上位机看原函数对应修改
//      UARTSendPicture2(Image_Data);
        TFTSPI_Show_Pic3(20, 27, 125, 75, Image_Data);
//      Get_Use_Image();     //采集图像数据存放数组
//      Get_01_Value();      //二值化图像数据
//                  
//      Threshold = GetOSTU(Image_Data);   //OSTU大津法 获取全局阈值
//      BinaryImage(Image_Data,Threshold); //二值化图像数据
//      
//      Draw_Road();         //龙邱OLED模块显示动态图像
      Field_Over_Flag= 0;       
    }
     // TFTSPI_P8X8NUM(uint8_t x, uint8_t y, uint16_t num, uint8_t num_bit,uint16_t word_color,uint16_t back_color)
     /* LED_Ctrl(LED0, RVS);//反转   
      LED_Ctrl(LED1, RVS);//反转 
      LED_Ctrl(LED2, RVS);//反转 
      LED_Ctrl(LED3, RVS);//反转 
      time_delay_ms(500); //延时  */         
    }
} 

/*******************************************************************************
* EOF
******************************************************************************/
