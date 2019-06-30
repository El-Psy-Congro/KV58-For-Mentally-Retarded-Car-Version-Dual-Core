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

deviation graphic, inductance;                                    //基于中线的偏差
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


/*位置式PID
 * NextPoint: 期望值
 * *aPID    :被控制对象的参数
 * 返回计算结果
 * 舵机是用PD控制
 * 电机是用PI控制
 */
int PIDPositional(int NextPoint, PID *aPID){
  int
    currentError,           //当前误差
    proportionVariable,     //与比例常数相乘的变量
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


/*增量式PID
 * NextPoint: 期望值
 * *aPID    :被控制对象的参数
 * 返回计算结果
 * 舵机是用PD控制
 * 电机是用PI控制
 */
int PIDIncremental(int NextPoint, PID *aPID){
  int
    currentError,           //当前误差
    proportionVariable,     //与比例常数相乘的变量
    integralVariable,       //与积分常数相乘的变量
    derivativeVariable;     //与微分常数相乘的变量
  
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
 * 模糊PID
 * 参考 CSDN博客     飞思卡尔智能车----模糊PID算法通俗讲           https://blog.csdn.net/weixin_36340979/article/details/79168052
 * 参考 CSDN博客     模糊自适应PID算法及其运用                                      https://blog.csdn.net/a841771798/article/details/79323118
 */
int PIDFuzzy(deviation *aDeviation, PID *aPID){
  u8 txt[16];
  float
    affiliationCurrent,                //现在中值的隶属度
    affiliationCurrentOneMinus,        // = 1 - affiliationCurrent
    affiliationBias,                   //偏差数据右隶属                   偏差数据 = |deviationNow - deviationLast|
    affiliationBiasOneMinus;           // = 1 - AffiliationBias

  int
    indexCurrent = 3,
    indexBias = 3,
    currentError,           //当前误差
    proportionVariable,     //与比例常数相乘的变量
    integralVariable,       //与积分常数相乘的变量
    derivativeVariable;     //与微分常数相乘的变量

  deviation temp;
  PID *tempPID = NULL;

 /**********************模糊推导*****************************************/
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
    affiliationCurrentOneMinus = ((float)aDeviation->interval - (float)temp.deviationNow)/(float)aDeviation->interval;  // = 1.0 - affiliationCurrent  但是如果在单片机上这样写会float会出现精度丢失的问题

  }else{
    temp.deviationNow = ABS(temp.deviationNow);
    while(temp.deviationNow > aDeviation->interval){
      temp.deviationNow -= aDeviation->interval;
      indexCurrent --;
    }
    LimitingAmplitude(&indexCurrent, INDEXMIN, INDEXMAX);
    affiliationCurrent = ((float)aDeviation->interval - (float)temp.deviationNow)/(float)aDeviation->interval;
    affiliationCurrentOneMinus = (float)temp.deviationNow/(float)aDeviation->interval;                                  // = 1.0 - affiliationCurrent  但是如果在单片机上这样写会float会出现精度丢失的问题

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








/*********************清晰化****************************************************/
  tempPID->proportion = aPID->proportion
                   + affiliationBias * affiliationCurrent * fuzzyRuleKp[indexBias][indexCurrent]
                   + affiliationBias * affiliationCurrentOneMinus * fuzzyRuleKp[indexBias][Z(indexCurrent-1)]                   //Z()  输入的变量如果是负数返回零
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
//  sprintf(txt, "%04d", aDeviation->deviationNow);             //数值转换为字符串
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
//    iError = aPID->SetPoint - NextPoint;       //偏差
//    aPID->SumError += iError;       //积分
//    dError = iError - aPID->LastError;     //微分
//    aPID->LastError = iError;
//
//    return (int)(aPID->SetPoint + aPID->Proportion * iError            //比例项
//           + aPID->Integral * aPID->SumError   //积分项
//           + aPID->Derivative * dError
//           );
//}
//
//
//void MotorLeftLocPIDCalc(int NextPoint){
//    int  iError,dError;
//    char txt[16];
//
//    iError = PIDMotorLeft.SetPoint - NextPoint;       //偏差
//    PIDMotorLeft.SumError += iError;       //积分
//    dError = iError - PIDMotorLeft.LastError;     //微分
//    PIDMotorLeft.LastError = iError;
//
//    speedLeft = (int)(PIDMotorLeft.SetPoint + PIDMotorLeft.Proportion * iError            //比例项
//           + PIDMotorLeft.Integral * PIDMotorLeft.SumError   //积分项
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
//    iError = PIDMotorRight.SetPoint - NextPoint;       //偏差
//    PIDMotorRight.SumError += iError;       //积分
//    dError = iError - PIDMotorRight.LastError;     //微分
//    PIDMotorRight.LastError = iError;
//
//    speedRight = (int)( PIDMotorRight.SetPoint + PIDMotorRight.Proportion * iError            //比例项
//           + PIDMotorRight.Integral * PIDMotorRight.SumError   //积分项
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
//    iError = PIDErect.SetPoint - NextPoint;       //偏差
//    PIDErect.SumError += iError;       //积分
//    dError = iError - PIDErect.LastError;     //微分
//    PIDErect.LastError = iError;
//
//    angle = (int)(PIDErect.Proportion * iError            //比例项
//           + PIDErect.Integral * PIDErect.SumError   //积分项
//           + PIDErect.Derivative * dError
//           );
//
//    PIDMotorLeft.SetPoint = angle;
//    PIDMotorRight.SetPoint = angle;
//}


