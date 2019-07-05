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
 * �˵��л�����
 */
short menuSelection = 0;
short menuSwitch = 0;
short temp;
int adjust;


/*
 * �˵������������
 */
menu *head = NULL, *menus;
monitor monitorSelection;
short menuPages = 0;
char txt[16];





/*�˵���ʼ������
 *���ã�
 *  ��д�õĲ˵�ҳ�����洢�������У�Ȼ��ʹ�ð��������л�
 *ʹ�÷�����
 *  ��д�õĲ˵�ҳ����ʹ��MenuPageAdd()��ӣ�
 */
void MenuInit(){
  if(monitorSelection == OLED){
    //OLED�Ĳ˵�ҳ������
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
    //TFT1.8�Ĳ˵�ҳ������
    MenuPageAdd(TFTMenuOfMT9V034);
    TFTSPI_CLS(u16WHITE);
  }
//  MenuPageAdd(MenuOfADCMedia);
//  MenuPageAdd(menuOfSpeedMeasure);
  menus = head;
}


/*
 * �˵�ҳ�л�����
 */
void Menu(){
  /*
   * ���뿪��
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
 * �˵�ҳ��Ӻ���
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
 * �����������ʾ
 */
void OLEDMenuOfUltrasonic(){
  sprintf(txt, "distance=%07d", len);
  LCD_P8x16Str(0, 3, (u8*) txt);

  sprintf(txt, "Ultrasonic", len);
  LCD_P8x16Str(0, 0, (u8*) txt);
}

/*
 * ����ͷͼ����ʾ
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
 * ��ѹ�۲�
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
 * ֱ��PID��������
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
����
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
 * �����������
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
����
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
 * �ҵ����������
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
����
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
 * ������������
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
/**����*
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
   * ���뿪��
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
 * ���PID����
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
*����
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
//  if (GYRO_X.MYBYTE.BYTEH > 0x7F)            //�жϼ��ٶ�X������,�˴�Ϊ��ֵ
//  {
//    tem = (~(GYRO_X.MYWORD >> 2) + 1) & 0X3FFF;  //�Ѳ�����ֵת��Ϊ��Чֵ
//  } else                                    //�����������
//  {
//    tem = (GYRO_X.MYWORD >> 2) & 0X3FFF;          //ת��Ϊ��Чֵ
//  }
//  sprintf(txt, "DJ0:%04d", GYRO_Y.MYWORD);             //��ֵת��Ϊ�ַ���
//  LCD_P8x16Str(10, 0, (uint8*) txt);          //OLED����ʾת����ֵ
//
//  //ת��Ϊ���ٶ���ֵ
//  Cvt_14bit_Str(txt, ACC_X);                //���ٶ���ֵרΪ�̶���ʽ�ַ�����������Ļ��ʾ
//  LCD_P6x8Str(10, 4, (uint8*) txt);           //OLED����ʾ��ֵ
//
//  //�¶ȼ��
//  if (LQ9AX_DAT[18] > 0x7F)                   //�¶�λ����0--125��
//  {
//    LCD_P6x8Str(10, 6, (uint8*) "low temp  ");    //OLED����ʾ��ֵ
//  } else {
//    sprintf(txt, "temp:+%03d ", LQ9AX_DAT[18]);  //ת��Ϊ�ַ���
//    LCD_P6x8Str(10, 6, (uint8*) txt);             //OLED����ʾ��ֵ
//  }
//  time_delay_ms(100);

    sprintf(txt, "DJ0:%04d", angle);             //��ֵת��Ϊ�ַ���
    LCD_P8x16Str(10, 0, (uint8*) txt);
}


/*
 * ���������ʾ
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

