#include "include.h"

#define NUMBER_OF_MENUS 7
#define NUMBER_OF_SPEED 2
#define NUMBER_OF_PID 4
#define NUMBER_OF_MOTOR 4
#define NUMBER_OF_ERECT 4
#define NUMBER_OF_ADC 8
#define NUMBER_OF_GRAPH_SPREAD 2
#define VARIATION_SPEED_PID 0.1
#define VARIATION_GRAPH_PID 0.1
#define VARIATION_Electromagnetism_PID 0.01
#define VARIATION_ERECT_PID 0.001
#define VARIATION_SPEED 10
#define VARIATION_ERECT 20
#define VARIATION_SERVO_MEDIA 10

#define THRESHOLDOFPAGE 10
#define THRESHOLDOFADJUST 7


/*
 * 菜单切换变量
 */
short menuSelection = 0;
short menuSwitch = 0;
short temp;
int adjust;


/*
 * 菜单链表基础变量
 */
menu *head = NULL, *menus;
monitor monitorSelection;
short menuPages = 0;
char txt[16];





/*菜单初始化函数
 *作用：
 *  将写好的菜单页函数存储到链表中，然后使用按键进行切换
 *使用方法：
 *  将写好的菜单页函数使用MenuPageAdd()添加；
 */
void MenuInit(){
  if(monitorSelection == OLED){
    //OLED的菜单页放这里
    MenuPageAdd(OLEDMenuOfCameraImage);
    MenuPageAdd(OLEDMenuOfCameraImageplus);
    MenuPageAdd(OLEDMenuOfMotor);
    MenuPageAdd(OLEDMenuOfUltrasonic);

    MenuPageAdd(OLEDMenuOfGraphPID);
    MenuPageAdd(OLEDMenuOfElectromagnetismPID);

//    MenuPageAdd(OLEDMenuOfMotorLeft);
//    MenuPageAdd(OLEDMenuOfMotorRight);
//    MenuPageAdd(OLEDMenuOfERECT);
    MenuPageAdd(OLEDMenuOfVoltage);
    MenuPageAdd(OLEDMenuOfADCshow);


  }else if(monitorSelection == TFT){
    //TFT1.8的菜单页放这里
    MenuPageAdd(TFTMenuOfMT9V034);
    TFTSPI_CLS(u16WHITE);
  }
//  MenuPageAdd(MenuOfADCMedia);
//  MenuPageAdd(menuOfSpeedMeasure);
  menus = head;
}


/*
 * 菜单页切换函数
 */
void Menu(){
  /*
   * 编码开关
   */
  if(CodingSwitch(speedRightGet, THRESHOLDOFPAGE) < 0){
    menus = menus->last;
    LCD_CLS();
  }else if(CodingSwitch(speedRightGet, THRESHOLDOFPAGE) > 0){
    menus = menus->next;
    LCD_CLS();
  }



    (*menus->page)();
}


/*
 * 菜单页添加函数
 */
void MenuPageAdd(void (*aPage)(void)) {
  menu *node = NULL;
  node = (menu *)malloc(sizeof(menu));
  if(!head){
    head = node;
  }

  menus = head;
  node->next = head;
  node->page = aPage;

  while(!(menus->next == head)){
    menus = menus->next;
  }
  node->last = menus;
  menus->next = node;
  head->last = node;
}


void TFTMenuOfMT9V034(){

  for (int i = GRAPH_HIGHT - 1; i > 0; i--){
    for (int j = 87 - 1; j > 7; j--){
      TFTSPI_Draw_Dot(2*j - 14, 2*i, Image_Use[i][j]);
      TFTSPI_Draw_Dot(2*j - 14+ 1, 2*i + 1, Image_Use[i][j]);
      TFTSPI_Draw_Dot(2*j - 14+ 1, 2*i, Image_Use[i][j]);
      TFTSPI_Draw_Dot(2*j - 14, 2*i + 1, Image_Use[i][j]);

    }
  }

}

/*
 * 超声波测距显示
 */
void OLEDMenuOfUltrasonic(){
  sprintf(txt, "distance=%07d", len);
  LCD_P8x16Str(0, 3, (u8*) txt);

  sprintf(txt, "Ultrasonic", len);
  LCD_P8x16Str(0, 0, (u8*) txt);
}

/*
 * 摄像头图像显示
 */
void OLEDMenuOfCameraImage(){
  LCD_Show_Frame100();
  Draw_Road();
  LCD_P8x16Str(0,0,"Graph");
  sprintf(txt,"%03d",thresholdOfGraph);
  LCD_P6x8Str(100,1,(u8*)txt);
}
void OLEDMenuOfCameraImageplus(){
  LCD_Show_Frame100();
  DrawRoad();
  LCD_P8x16Str(0,0,"GraphAdd");
  sprintf(txt,"%03d",thresholdOfGraph);
  LCD_P6x8Str(100,1,(u8*)txt);
}

/*
 * 电压观察
 */
void OLEDMenuOfVoltage(){
  IsMotorVoltage();
  IsServoVoltage();
  sprintf(txt,"%04d",voltageMotor);
  LCD_P6x8Str(40,0,(u8*)txt);

  sprintf(txt,"%04d",voltageServo);
  LCD_P6x8Str(40,4,(u8*)txt);
}


/*
 * 直立PID参数调整
 */
void OLEDMenuOfERECT(){
  if (!KEYRead()) {
    time_delay_ms(100);
    if (!KEYRead()) {
      menuSwitch++;
      LCD_CLS();
    }
  }


  menuSwitch = menuSwitch % NUMBER_OF_ERECT;

/****
编码
**/
  adjust = CodingSwitch(speedLeftGet, THRESHOLDOFADJUST);
  if(adjust){
    if(menuSwitch == 0){
      PIDErect.proportion += adjust*VARIATION_ERECT;
    }else if(menuSwitch == 1){
      PIDMotor.integral += adjust*VARIATION_ERECT;
    }else if(menuSwitch == 2){
      PIDErect.derivative += adjust*VARIATION_ERECT;
    }else if(menuSwitch == 3){
      PIDErect.setPoint += adjust*VARIATION_ERECT;
    }
  }

  temp = (int) (PIDErect.proportion * (1/VARIATION_ERECT_PID));
  sprintf(txt, "P:%04d", temp);
  LCD_P8x16Str(10, 2, (u8*) txt);

  temp = (int) (PIDErect.integral * (1/VARIATION_ERECT_PID));
  sprintf(txt, "I:%04d", temp);
  LCD_P8x16Str(10, 4, (u8*) txt);

  temp = (int) (PIDErect.derivative * (1/VARIATION_ERECT_PID));
  sprintf(txt, "D:%04d", temp);
  LCD_P8x16Str(10, 6, (u8*) txt);


  sprintf(txt, "%04d", PIDErect.setPoint);
  LCD_P8x16Str(80, 4, (u8*) txt);

  sprintf(txt, "%04d", speedRightGet);
  LCD_P8x16Str(80, 6, (u8*) txt);

  LCD_P8x16Str(0, 0, "Erect");
  LCD_P8x16Str(70, 2, "Bary");
  if ((menuSwitch + 1) * 2 < 8) {
    LCD_P8x16Str(0, (menuSwitch + 1) * 2, ">");
  } else {
    LCD_P8x16Str(70, (menuSwitch + 1) * 2 - 4, ">");
  }


}

/*
 * 电机参数调整
 */
void OLEDMenuOfMotor(){
  if (!KEYRead()) {
    time_delay_ms(100);
    if (!KEYRead()) {
      menuSwitch++;
      LCD_CLS();
    }
  }
  menuSwitch = menuSwitch % NUMBER_OF_MOTOR;

/***
编码
**/
  adjust = CodingSwitch(speedLeftGet, THRESHOLDOFADJUST);
  if(adjust){
    if(menuSwitch == 0){
      PIDMotor.proportion += adjust*VARIATION_SPEED_PID;
    }else if(menuSwitch == 1){
      PIDMotor.integral += adjust*VARIATION_SPEED_PID;
    }else if(menuSwitch == 2){
      PIDMotor.derivative += adjust*VARIATION_SPEED_PID;
    }else if(menuSwitch == 3){
      PIDMotor.setPoint += adjust*VARIATION_SPEED;
    }
  }

  temp = (int) (PIDMotor.proportion * (1/VARIATION_SPEED_PID));
  sprintf(txt, "P:%04d", temp);
  LCD_P8x16Str(10, 2, (u8*) txt);

  temp = (int) (PIDMotor.integral * (1/VARIATION_SPEED_PID));
  sprintf(txt, "I:%04d", temp);
  LCD_P8x16Str(10, 4, (u8*) txt);

  temp = (int) (PIDMotor.derivative * (1/VARIATION_SPEED_PID));
  sprintf(txt, "D:%04d", temp);
  LCD_P8x16Str(10, 6, (u8*) txt);


  sprintf(txt, "%04d", PIDMotor.setPoint);
  LCD_P8x16Str(80, 4, (u8*) txt);

  sprintf(txt, "%04d", (speedRightGet + speedLeftGet)/2);
  LCD_P8x16Str(80, 6, (u8*) txt);

  LCD_P8x16Str(0, 0, "Motor adjust");
  LCD_P8x16Str(80, 2, "Speed");
  if ((menuSwitch + 1) * 2 < 8) {
    LCD_P8x16Str(0, (menuSwitch + 1) * 2, ">");
  } else {
    LCD_P8x16Str(70, (menuSwitch + 1) * 2 - 4, ">");
  }

  PIDMotorRight.setPoint = PIDMotorLeft.setPoint =   PIDMotor.setPoint;
  PIDMotorRight.proportion = PIDMotorLeft.proportion =   PIDMotor.proportion;
  PIDMotorRight.integral = PIDMotorLeft.integral =   PIDMotor.integral;
  PIDMotorRight.derivative = PIDMotorLeft.derivative =   PIDMotor.integral;

}

/*
 * 右电机参数调整
 */
void OLEDMenuOfMotorRight(){
  if (!KEYRead()) {
    time_delay_ms(100);
    if (!KEYRead()) {
      menuSwitch++;
      LCD_CLS();
    }
  }
  menuSwitch = menuSwitch % NUMBER_OF_MOTOR;

/**
编码
**/
  adjust = CodingSwitch(speedLeftGet, THRESHOLDOFADJUST);
  if(adjust){
    if(menuSwitch == 0){
      PIDMotorRight.proportion += adjust*VARIATION_SPEED_PID;
    }else if(menuSwitch == 1){
      PIDMotorRight.integral += adjust*VARIATION_SPEED_PID;
    }else if(menuSwitch == 2){
      PIDMotorRight.derivative += adjust*VARIATION_SPEED_PID;
    }else if(menuSwitch == 3){
      PIDMotorRight.setPoint += adjust*VARIATION_SPEED;
    }
  }

  temp = (int) (PIDMotorRight.proportion * (1/VARIATION_SPEED_PID));
  sprintf(txt, "P:%04d", temp);
  LCD_P8x16Str(10, 2, (u8*) txt);

  temp = (int) (PIDMotorRight.integral * (1/VARIATION_SPEED_PID));
  sprintf(txt, "I:%04d", temp);
  LCD_P8x16Str(10, 4, (u8*) txt);

  temp = (int) (PIDMotorRight.derivative * (1/VARIATION_SPEED_PID));
  sprintf(txt, "D:%04d", temp);
  LCD_P8x16Str(10, 6, (u8*) txt);


  sprintf(txt, "%04d", PIDMotorRight.setPoint);
  LCD_P8x16Str(80, 4, (u8*) txt);

  sprintf(txt, "%04d", speedRightGet);
  LCD_P8x16Str(80, 6, (u8*) txt);

  LCD_P8x16Str(0, 0, "Right Motor");
  LCD_P8x16Str(80, 2, "Speed");
  if ((menuSwitch + 1) * 2 < 8) {
    LCD_P8x16Str(0, (menuSwitch + 1) * 2, ">");
  } else {
    LCD_P8x16Str(70, (menuSwitch + 1) * 2 - 4, ">");
  }

}

/*
 * 左电机参数调整
 */
void OLEDMenuOfMotorLeft(){
  if (!KEYRead()) {
    time_delay_ms(100);
    if (!KEYRead()) {
      menuSwitch++;
      LCD_CLS();
    }
  }
  menuSwitch = menuSwitch % NUMBER_OF_MOTOR;

/***
/**编码*
**/
  adjust = CodingSwitch(speedLeftGet, THRESHOLDOFADJUST);
  if(adjust){
    if(menuSwitch == 0){
      PIDMotorLeft.proportion += adjust*VARIATION_SPEED_PID;
    }else if(menuSwitch == 1){
      PIDMotorLeft.integral += adjust*VARIATION_SPEED_PID;
    }else if(menuSwitch == 2){
      PIDMotorLeft.derivative += adjust*VARIATION_SPEED_PID;
    }else if(menuSwitch == 3){
      PIDMotorLeft.setPoint += adjust*VARIATION_SPEED;
    }
  }


  temp = (int) (PIDMotorLeft.proportion * (1/VARIATION_SPEED_PID));
  sprintf(txt, "P:%04d", temp);
  LCD_P8x16Str(10, 2, (u8*) txt);

  temp = (int) (PIDMotorLeft.integral * (1/VARIATION_SPEED_PID));
  sprintf(txt, "I:%04d", temp);
  LCD_P8x16Str(10, 4, (u8*) txt);

  temp = (int) (PIDMotorLeft.derivative * (1/VARIATION_SPEED_PID));
  sprintf(txt, "D:%04d", temp);
  LCD_P8x16Str(10, 6, (u8*) txt);


  sprintf(txt, "%04d", PIDMotorLeft.setPoint);
  LCD_P8x16Str(80, 4, (u8*) txt);

  sprintf(txt, "%04d", speedLeftGet);
  LCD_P8x16Str(80, 6, (u8*) txt);

  LCD_P8x16Str(0, 0, "Left Motor");
  LCD_P8x16Str(80, 2, "Speed");
  if ((menuSwitch + 1) * 2 < 8) {
    LCD_P8x16Str(0, (menuSwitch + 1) * 2, ">");
  } else {
    LCD_P8x16Str(70, (menuSwitch + 1) * 2 - 4, ">");
  }

}

void OLEDMenuOfGraphPID(){
  if(!KEYRead()){
    time_delay_ms(100);
    if(!KEYRead()){
      menuSwitch++;
      LCD_CLS();
    }
  }
  menuSwitch = menuSwitch%NUMBER_OF_PID;

  /*
   * 编码开关
   */
  adjust = CodingSwitch(speedLeftGet, THRESHOLDOFADJUST);
  if(adjust){
    if(menuSwitch == 0){
      PIDServoOfGraph.proportion += adjust*VARIATION_GRAPH_PID;
    }else if(menuSwitch == 1){
      PIDServoOfGraph.integral += adjust*VARIATION_GRAPH_PID;
    }else if(menuSwitch == 2){
      PIDServoOfGraph.derivative += adjust*VARIATION_GRAPH_PID;
    }else if(menuSwitch == 3){
      servoMedian += adjust*VARIATION_SERVO_MEDIA;
    }
  }



  temp = (int)(PIDServoOfGraph.proportion * (1/VARIATION_GRAPH_PID));
  sprintf(txt,"P:%04d",temp);
  LCD_P8x16Str(20,2,(u8*)txt);

  temp = (int)(PIDServoOfGraph.integral * (1/VARIATION_GRAPH_PID));
  sprintf(txt,"I:%04d",temp);
  LCD_P8x16Str(20,4,(u8*)txt);

  temp = (int)(PIDServoOfGraph.derivative * (1/VARIATION_GRAPH_PID));
  sprintf(txt,"D:%04d",temp);
  LCD_P8x16Str(20,6,(u8*)txt);

  sprintf(txt, "MEDIAN");
  LCD_P8x16Str(80, 2, (u8*) txt);

  sprintf(txt, "%04d", servoMedian);
  LCD_P8x16Str(80, 4, (u8*) txt);

  sprintf(txt, "%04d", servo);
  LCD_P8x16Str(80, 6, (u8*) txt);


  LCD_P8x16Str(0,0,"Graph PID");

  if(menuSwitch == 3){
  LCD_P8x16Str(70,4,">");
  }else{
    LCD_P8x16Str(0,(menuSwitch+1)*2,"->");
  }


}


/*
 * 电磁PID调整
 */
void OLEDMenuOfElectromagnetismPID(){
  if(!KEYRead()){
    time_delay_ms(100);
    if(!KEYRead()){
      menuSwitch++;
      LCD_CLS();
    }
  }
  menuSwitch = menuSwitch%NUMBER_OF_PID;

/***
*编码
*/
  adjust = CodingSwitch(speedLeftGet, THRESHOLDOFADJUST);
  if(adjust){
    if(menuSwitch == 0){
      PIDServoOfElectromagnetism.proportion += adjust*VARIATION_Electromagnetism_PID;
    }else if(menuSwitch == 1){
      PIDServoOfElectromagnetism.integral += adjust*VARIATION_Electromagnetism_PID;
    }else if(menuSwitch == 2){
      PIDServoOfElectromagnetism.derivative += adjust*VARIATION_Electromagnetism_PID;
    }else if(menuSwitch == 3){
      servoMedian += adjust*VARIATION_SERVO_MEDIA;
    }
  }



  temp = (int)(PIDServoOfElectromagnetism.proportion * (1/VARIATION_Electromagnetism_PID));
  sprintf(txt,"P:%04d",temp);
  LCD_P8x16Str(20,2,(u8*)txt);

  temp = (int)(PIDServoOfElectromagnetism.integral * (1/VARIATION_Electromagnetism_PID));
  sprintf(txt,"I:%04d",temp);
  LCD_P8x16Str(20,4,(u8*)txt);

  temp = (int)(PIDServoOfElectromagnetism.derivative * (1/VARIATION_Electromagnetism_PID));
  sprintf(txt,"D:%04d",temp);
  LCD_P8x16Str(20,6,(u8*)txt);

  sprintf(txt, "MEDIAN");
  LCD_P8x16Str(80, 2, (u8*) txt);

  sprintf(txt, "%04d", servoMedian);
  LCD_P8x16Str(80, 4, (u8*) txt);

  sprintf(txt, "%04d", servo);
  LCD_P8x16Str(80, 6, (u8*) txt);


  LCD_P8x16Str(0,0,"EM PID");

  if(menuSwitch == 3){
  LCD_P8x16Str(70,4,">");
  }else{
    LCD_P8x16Str(0,(menuSwitch+1)*2,"->");
  }


}


/*
 *
 */
void OLEDMenuOfGyro(){
//  u16 tem=0;
//  float fv=0.01;
//  char  txt[16]="X:";
//  Update9AX();
//  if (GYRO_X.MYBYTE.BYTEH > 0x7F)            //判断加速度X轴正负,此处为负值
//  {
//    tem = (~(GYRO_X.MYWORD >> 2) + 1) & 0X3FFF;  //把补码数值转换为有效值
//  } else                                    //正数情况处理
//  {
//    tem = (GYRO_X.MYWORD >> 2) & 0X3FFF;          //转换为有效值
//  }
//  sprintf(txt, "DJ0:%04d", GYRO_Y.MYWORD);             //数值转换为字符串
//  LCD_P8x16Str(10, 0, (uint8*) txt);          //OLED屏显示转向数值
//
//  //转换为加速度数值
//  Cvt_14bit_Str(txt, ACC_X);                //加速度数值专为固定格式字符串，方便屏幕显示
//  LCD_P6x8Str(10, 4, (uint8*) txt);           //OLED屏显示数值
//
//  //温度检测
//  if (LQ9AX_DAT[18] > 0x7F)                   //温度位正数0--125°
//  {
//    LCD_P6x8Str(10, 6, (uint8*) "low temp  ");    //OLED屏显示数值
//  } else {
//    sprintf(txt, "temp:+%03d ", LQ9AX_DAT[18]);  //转换为字符串
//    LCD_P6x8Str(10, 6, (uint8*) txt);             //OLED屏显示数值
//  }
//  time_delay_ms(100);

    sprintf(txt, "DJ0:%04d", angle);             //数值转换为字符串
    LCD_P8x16Str(10, 0, (uint8*) txt);
}


/*
 * 电感数据显示
 */
void OLEDMenuOfADCshow(){
//  ADC0_Ch_e ADCRemawp[] = {ADC0_SE5a, ADC0_SE9, ADC0_DP1, ADC0_SE11, ADC0_DP3, ADC0_SE4a, ADC0_SE10, ADC0_DP2};
  ADC0_Ch_e ADCRemawp[] = {ADC0_DP1, ADC0_SE5a, ADC0_DP2, ADC0_DP3,  ADC0_SE11, ADC0_SE9, ADC0_SE4a, ADC0_SE10};
//  if(!KEY_Read(Up)){
//    time_delay_ms(100);
//    if(!KEY_Read(Up)){
//      menuSwitch++;
//    }
//  }else if(!KEY_Read(Down)){
//    time_delay_ms(100);
//    if(!KEY_Read(Down)){
//      menuSwitch--;
//      if(menuSwitch<0){
//       menuSwitch = NUMBEROFADC-1;
//      }
//  }
//
//  menuSwitch = menuSwitch % NUMBEROFADC;


//  if(!KEY_Read(up)){
//    time_delay_ms(100);
//    if(!KEY_Read(up)){
//      menuSwitch += 1;
//    }
//  }else if(!KEY_Read(down)){
//    time_delay_ms(100);
//    if(!KEY_Read(down)){
//      menuSwitch -= 1;
//      if(menuSwitch<0){
//        menuSwitch = NUMBER_OF_ADC;
//      }
//    }
//  }

  LCD_CLS();

  sprintf(txt,"%d|",ADC0_Ave(L_0,ADC_16bit,10));
  LCD_P8x16Str(0,0,(u8*)txt);

  sprintf(txt,"%d|",ADC0_Ave(L_1,ADC_16bit,10));
  LCD_P8x16Str(0,2,(u8*)txt);

  sprintf(txt,"%d|",ADC0_Ave(L_2,ADC_16bit,10));
  LCD_P8x16Str(0,4,(u8*)txt);

  sprintf(txt,"%d|",ADC0_Ave(L_3,ADC_16bit,10));
  LCD_P8x16Str(0,6,(u8*)txt);


  sprintf(txt,"%d|",ADC0_Ave(L_4,ADC_16bit,10));
  LCD_P8x16Str(80,0,(u8*)txt);


  sprintf(txt,"%d|",ADC0_Ave(L_5,ADC_16bit,10));
  LCD_P8x16Str(80,2,(u8*)txt);


  sprintf(txt,"%d|",ADC0_Ave(L_6,ADC_16bit,10));
  LCD_P8x16Str(80,4,(u8*)txt);


  sprintf(txt,"%d|",ADC0_Ave(L_7,ADC_16bit,10));
  LCD_P8x16Str(80,6,(u8*)txt);





  time_delay_ms(10);

}

void OLEDMenuOfADCMedia(){
  ADC0_Ch_e ADCRemawp[] = {ADC0_DP1, ADC0_SE5a, ADC0_DP2, ADC0_DP3,  ADC0_SE11, ADC0_SE9, ADC0_SE4a, ADC0_SE10};
  sprintf(txt,"ADC:%d",(int)(ADC0_Ave(ADCRemawp[3],ADC_16bit,10)-ADC0_Ave(ADCRemawp[0],ADC_16bit,10)+ADC0_Ave(ADCRemawp[2],ADC_16bit,10)-ADC0_Ave(ADCRemawp[1],ADC_16bit,10))*10000
                           /(ADC0_Ave(ADCRemawp[3],ADC_16bit,10)+ADC0_Ave(ADCRemawp[0],ADC_16bit,10)+ADC0_Ave(ADCRemawp[2],ADC_16bit,10)+ADC0_Ave(ADCRemawp[1],ADC_16bit,10)));
  LCD_P8x16Str(0,3,(u8*)txt);
}


void MenuOfPrecisionAdjustment(){

}


void OLEDMenuOfSpeedMeasure(){

}

