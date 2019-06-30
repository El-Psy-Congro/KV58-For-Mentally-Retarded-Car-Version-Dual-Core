#ifndef __PID_H__
#define __PID_H__

typedef struct{
    int  setPoint;              //设定目标 Desired Value
    long sumError;              //误差累计
    float  proportion;         //比例常数 Proportional Cons
    float  integral;           //积分常数 Integral Const
    float  derivative;         //微分常数 Derivative Const
    int lastError;              //Error[-1]
    int prevError;              //Error[-2]
    bool  isDeviation;              //返回值是否加上设定值     1为不加，   2为加上设定值
} PID;

typedef struct{
  float proportion;
  float  integral;
  float  derivative;
}PIDParameter;

typedef struct{
  int deviationNow;             //现在与中线的偏差
  int deviationLast;            //上一次与这次的偏差的差值  |deviationNow - deviationLast|
  int interval;
}deviation;

void PIDInit(void);
int PIDPositional(int NextPoint, PID *aPID);
int PIDIncremental(int NextPoint, PID *aPID);
int PIDFuzzy(deviation *aDeviation, PID *aPID);
int PIDFuzzy(deviation *aDeviation, PID *aPID);
//void ServoLocPIDCalc(int NextPoint);
//void MotorLeftLocPIDCalc(int NextPoint);
//void MotorRightLocPIDCalc(int NextPoint);
//void ErectLocPIDCalc(int NextPoint);

extern PID PIDServoOfGraph,PIDServoOfElectromagnetism, PIDMotorLeft, PIDMotorRight, PIDErect, PIDMotor;
extern deviation graphic, inductance;

#endif 
