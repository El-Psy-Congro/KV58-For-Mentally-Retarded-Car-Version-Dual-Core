#include "include.h"

#define NL  4.0
#define NM  2.0
#define NS  1.0
#define ZO  0.0
#define PS  1.0
#define PM  2.0
#define PL  4.0

#define INDEXMIN 0
#define INDEXMAX 6




static const float
  fuzzyRuleKp[7][7] = {
    PL, PL, PM, PM, PS, ZO, ZO,
    PL, PL, PM, PS, PS, ZO, ZO,
    PM, PM, PM, PS, ZO, NS, NS,
    PM, PM, PS, ZO, NS, NM, NM,
    PS, PS, ZO, NS, NS, NM, NM,
    PS, ZO, NS, NM, NM, NM, NL,
    ZO, ZO, NM, NM, NM, NL, NL
  },

  fuzzyRuleKd[7][7] = {
    PS, NS, NL, NL, NL, NM, PS,
    PS, NS, NL, NM, NM, NS, ZO,
    ZO, NS, NM, NM, NS, NS, ZO,
    ZO, NS, NS, NS, NS, NS, ZO,
    ZO, ZO, ZO, ZO, ZO, ZO, ZO,
    PL, NS, PS, PS, PS, PS, PL,
    PL, PM, PM, PM, PS, PS, PL
  },

  fuzzyRuleKi[7][7] = {
    NL, NL, NM, NM, NS, ZO, ZO,
    NL, NL, NM, NS, NS, ZO, ZO,
    NL, NM, NS, NS, ZO, PS, PS,
    NM, NM, NS, ZO, PS, PM, PM,
    NM, NS, ZO, PS, PS, PM, PL,
    ZO, ZO, PS, PS, PM, PL, PL,
    ZO, ZO, PS, PM, PM, PL, PL
  };

PID
  PIDServoOfGraph,
  PIDServoOfElectromagnetism,
  PIDMotorLeft, PIDMotorRight,
  PIDErect,
  PIDMotor,
  fuzzyPIDServo;

deviation graphic, inductance;                                    //�������ߵ�ƫ��
int speedSet = 0;

void PIDInit(){
  PIDServoOfGraph.setPoint = 0;
  PIDServoOfGraph.proportion = 9;  //0.27
  PIDServoOfGraph.integral = 0;
  PIDServoOfGraph.derivative = 0;
  PIDServoOfGraph.isDeviation = false;

  PIDServoOfElectromagnetism.setPoint = 0;
  PIDServoOfElectromagnetism.proportion = 0.08;  //0.27
  PIDServoOfElectromagnetism.integral = 0;
  PIDServoOfElectromagnetism.derivative = 0.20;
  PIDServoOfElectromagnetism.isDeviation = false;

  PIDMotor.setPoint = 140;
  PIDMotor.proportion = 0.100;
  PIDMotor.integral = 0.200;
  PIDMotor.derivative = 0;
  PIDMotor.isDeviation = true;

  PIDMotorLeft.setPoint = 140;
  PIDMotorLeft.proportion = 0.200;
  PIDMotorLeft.integral = 0.200;
  PIDMotorLeft.derivative = 0;
  PIDMotorLeft.isDeviation = true;

  PIDMotorRight.setPoint = 140;
  PIDMotorRight.proportion = 0.200;
  PIDMotorRight.integral = 0.200;
  PIDMotorRight.derivative = 0;
  PIDMotorRight.isDeviation = true;

  PIDErect.setPoint = 2100;
  PIDErect.proportion =0.8;
  PIDErect.integral = 0.000;
  PIDErect.derivative = 0;
  PIDErect.isDeviation = false;

  graphic.interval = 5;
  inductance.interval = 6;

}


/*λ��ʽPID
 * NextPoint: ����ֵ
 * *aPID    :�����ƶ���Ĳ���
 * ���ؼ�����
 * �������PD����
 * �������PI����
 */
int PIDPositional(int NextPoint, PID *aPID){
  int
    currentError,           //��ǰ���
    proportionVariable,     //�����������˵ı���
    integralVariable,
    derivativeVariable;
      
  currentError = aPID->setPoint - NextPoint;
  aPID->sumError += currentError;

  proportionVariable = currentError;
  integralVariable = aPID->sumError;
  derivativeVariable = currentError - aPID->lastError;

  aPID->prevError = aPID->lastError;
  aPID->lastError = currentError;

  if(aPID->isDeviation){
    return (int)( aPID->proportion * proportionVariable
                + aPID->integral * integralVariable
                + aPID->derivative * derivativeVariable);
  }else{
    return (int)(aPID->setPoint
               + aPID->proportion * proportionVariable
               + aPID->integral * integralVariable
               + aPID->derivative * derivativeVariable);
  }
}


/*����ʽPID
 * NextPoint: ����ֵ
 * *aPID    :�����ƶ���Ĳ���
 * ���ؼ�����
 * �������PD����
 * �������PI����
 */
int PIDIncremental(int NextPoint, PID *aPID){
  int
    currentError,           //��ǰ���
    proportionVariable,     //�����������˵ı���
    integralVariable,       //����ֳ�����˵ı���
    derivativeVariable;     //��΢�ֳ�����˵ı���
  
  currentError = aPID->setPoint - NextPoint;
  aPID->sumError += currentError;

  proportionVariable = currentError - aPID->lastError;
  integralVariable = currentError;
  derivativeVariable = currentError - 2*aPID->lastError + aPID->prevError;

  aPID->prevError = aPID->lastError;
  aPID->lastError = currentError;
  
  
  if(aPID->isDeviation){
    return (int)( aPID->proportion * proportionVariable
                + aPID->integral * integralVariable
                + aPID->derivative * derivativeVariable);
  }else{
    return (int)(aPID->setPoint
               + aPID->proportion * proportionVariable
               + aPID->integral * integralVariable
               + aPID->derivative * derivativeVariable);
  }
}

/*
 * ģ��PID
 * �ο� CSDN����     ��˼�������ܳ�----ģ��PID�㷨ͨ�׽�           https://blog.csdn.net/weixin_36340979/article/details/79168052
 * �ο� CSDN����     ģ������ӦPID�㷨��������                                      https://blog.csdn.net/a841771798/article/details/79323118
 */
int PIDFuzzy(deviation *aDeviation, PID *aPID){
  u8 txt[16];
  float
    affiliationCurrent,                //������ֵ��������
    affiliationCurrentOneMinus,        // = 1 - affiliationCurrent
    affiliationBias,                   //ƫ������������                   ƫ������ = |deviationNow - deviationLast|
    affiliationBiasOneMinus;           // = 1 - AffiliationBias

  int
    indexCurrent = 3,
    indexBias = 3,
    currentError,           //��ǰ���
    proportionVariable,     //�����������˵ı���
    integralVariable,       //����ֳ�����˵ı���
    derivativeVariable;     //��΢�ֳ�����˵ı���

  deviation temp;
  PID *tempPID = NULL;

 /**********************ģ���Ƶ�*****************************************/
  temp.deviationNow = aDeviation->deviationNow;
  temp.deviationLast = ABS(aDeviation->deviationLast - aDeviation->deviationNow);

  if(temp.deviationNow > 0){
    indexCurrent ++;
    while(temp.deviationNow > aDeviation->interval){
      temp.deviationNow -= aDeviation->interval;
      indexCurrent ++;
    }
    LimitingAmplitude(&indexCurrent, INDEXMIN, INDEXMAX);
    affiliationCurrent = (float)temp.deviationNow/(float)aDeviation->interval;
    affiliationCurrentOneMinus = ((float)aDeviation->interval - (float)temp.deviationNow)/(float)aDeviation->interval;  // = 1.0 - affiliationCurrent  ��������ڵ�Ƭ��������д��float����־��ȶ�ʧ������

  }else{
    temp.deviationNow = ABS(temp.deviationNow);
    while(temp.deviationNow > aDeviation->interval){
      temp.deviationNow -= aDeviation->interval;
      indexCurrent --;
    }
    LimitingAmplitude(&indexCurrent, INDEXMIN, INDEXMAX);
    affiliationCurrent = ((float)aDeviation->interval - (float)temp.deviationNow)/(float)aDeviation->interval;
    affiliationCurrentOneMinus = (float)temp.deviationNow/(float)aDeviation->interval;                                  // = 1.0 - affiliationCurrent  ��������ڵ�Ƭ��������д��float����־��ȶ�ʧ������

  }


  if(temp.deviationLast > 0){
    indexBias ++;
    while(temp.deviationLast > aDeviation->interval){
      temp.deviationLast -= aDeviation->interval;
      indexBias ++;
    }
    LimitingAmplitude(&indexBias, INDEXMIN, INDEXMAX);
    affiliationBias = (float)temp.deviationLast/(float)aDeviation->interval;
    affiliationBiasOneMinus = ((float)aDeviation->interval - (float)temp.deviationLast)/(float)aDeviation->interval;
  }else{
    temp.deviationLast = ABS(temp.deviationLast);
    while(temp.deviationLast > aDeviation->interval){
      temp.deviationLast -= aDeviation->interval;
      indexBias --;
    }
    LimitingAmplitude(&indexBias, INDEXMIN, INDEXMAX);
    affiliationBias = ((float)aDeviation->interval - (float)temp.deviationLast)/(float)aDeviation->interval;
    affiliationBiasOneMinus = (float)temp.deviationLast/(float)aDeviation->interval;
  }








/*********************������****************************************************/
  tempPID->proportion = aPID->proportion
                   + affiliationBias * affiliationCurrent * fuzzyRuleKp[indexBias][indexCurrent]
                   + affiliationBias * affiliationCurrentOneMinus * fuzzyRuleKp[indexBias][Z(indexCurrent-1)]                   //Z()  ����ı�������Ǹ���������
                   + affiliationBiasOneMinus * affiliationCurrent * fuzzyRuleKp[Z(indexBias-1)][indexCurrent]
                   + affiliationBiasOneMinus * affiliationCurrentOneMinus * fuzzyRuleKp[Z(indexBias-1)][Z(indexCurrent-1)];


//  tempPID->integral = aPID->integral
//                   + affiliationBias * affiliationCurrent * fuzzyRuleKi[indexBias][indexCurrent]
//                   + affiliationBias * affiliationCurrentOneMinus * fuzzyRuleKi[indexBias][Z(indexCurrent-1)]
//                   + affiliationBiasOneMinus * affiliationCurrent * fuzzyRuleKi[Z(indexBias-1)][indexCurrent]
//                   + affiliationBiasOneMinus * affiliationCurrentOneMinus * fuzzyRuleKi[Z(indexBias-1)][Z(indexCurrent-1)];

  tempPID->derivative = aPID->derivative
                   + affiliationBias * affiliationCurrent * fuzzyRuleKd[indexBias][indexCurrent]
                   + affiliationBias * affiliationCurrentOneMinus * fuzzyRuleKd[indexBias][Z(indexCurrent-1)]
                   + affiliationBiasOneMinus * affiliationCurrent * fuzzyRuleKd[Z(indexBias-1)][indexCurrent]
                   + affiliationBiasOneMinus * affiliationCurrentOneMinus * fuzzyRuleKd[Z(indexBias-1)][Z(indexCurrent-1)];

//  sprintf(txt, "%08d", (int)(tempPID->proportion*1000));
//  LCD_P8x16Str(0, 0, (uint8*) txt);
//  sprintf(txt, "%04d", (int)(tempPID->derivative * 1000));
//  LCD_P8x16Str(0, 2, (uint8*) txt);
//  sprintf(txt, "%04d", aDeviation->deviationNow);             //��ֵת��Ϊ�ַ���
//  LCD_P8x16Str(0, 4, (uint8*) txt);
//  sprintf(txt, "%04d", indexCurrent);
//  LCD_P8x16Str(0, 6, (uint8*) txt);




  currentError = aPID->setPoint - aDeviation->deviationNow;
  aPID->sumError += currentError;

  proportionVariable = currentError;
  integralVariable = aPID->sumError;
  derivativeVariable = currentError - aPID->lastError;

  aPID->prevError = aPID->lastError;
  aPID->lastError = currentError;

  if(aPID->isDeviation){
    return (int)( tempPID->proportion * proportionVariable
                + aPID->integral * integralVariable
                + aPID->derivative * derivativeVariable);
  }else{
    return (int)( aPID->setPoint
               + tempPID->proportion * proportionVariable
               + aPID->integral * integralVariable
               + aPID->derivative * derivativeVariable);
  }


}



//int PositionalPID(int NextPoint, PID aPID){
//    int  iError,dError;
//    char txt[16];
//
//    iError = aPID->SetPoint - NextPoint;       //ƫ��
//    aPID->SumError += iError;       //����
//    dError = iError - aPID->LastError;     //΢��
//    aPID->LastError = iError;
//
//    return (int)(aPID->SetPoint + aPID->Proportion * iError            //������
//           + aPID->Integral * aPID->SumError   //������
//           + aPID->Derivative * dError
//           );
//}
//
//
//void MotorLeftLocPIDCalc(int NextPoint){
//    int  iError,dError;
//    char txt[16];
//
//    iError = PIDMotorLeft.SetPoint - NextPoint;       //ƫ��
//    PIDMotorLeft.SumError += iError;       //����
//    dError = iError - PIDMotorLeft.LastError;     //΢��
//    PIDMotorLeft.LastError = iError;
//
//    speedLeft = (int)(PIDMotorLeft.SetPoint + PIDMotorLeft.Proportion * iError            //������
//           + PIDMotorLeft.Integral * PIDMotorLeft.SumError   //������
//           + PIDMotorLeft.Derivative * dError
//           );
//    Motor_Duty(MotL,speedLeft);
//
//}
//
//void MotorRightLocPIDCalc(int NextPoint){
//    int  iError,dError;
//    char txt[16];
//
//    iError = PIDMotorRight.SetPoint - NextPoint;       //ƫ��
//    PIDMotorRight.SumError += iError;       //����
//    dError = iError - PIDMotorRight.LastError;     //΢��
//    PIDMotorRight.LastError = iError;
//
//    speedRight = (int)( PIDMotorRight.SetPoint + PIDMotorRight.Proportion * iError            //������
//           + PIDMotorRight.Integral * PIDMotorRight.SumError   //������
//           + PIDMotorRight.Derivative * dError
//           );
//    Motor_Duty(MotR,speedRight);
//}
//
//
//void ErectLocPIDCalc(int NextPoint){
//    int  iError,dError;
//    char txt[16];
//
//    iError = PIDErect.SetPoint - NextPoint;       //ƫ��
//    PIDErect.SumError += iError;       //����
//    dError = iError - PIDErect.LastError;     //΢��
//    PIDErect.LastError = iError;
//
//    angle = (int)(PIDErect.Proportion * iError            //������
//           + PIDErect.Integral * PIDErect.SumError   //������
//           + PIDErect.Derivative * dError
//           );
//
//    PIDMotorLeft.SetPoint = angle;
//    PIDMotorRight.SetPoint = angle;
//}


