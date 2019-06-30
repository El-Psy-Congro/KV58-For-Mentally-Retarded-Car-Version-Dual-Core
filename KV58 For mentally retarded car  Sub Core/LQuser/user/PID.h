#ifndef __PID_H__
#define __PID_H__

typedef struct{
    int  setPoint;              //�趨Ŀ�� Desired Value
    long sumError;              //����ۼ�
    float  proportion;         //�������� Proportional Cons
    float  integral;           //���ֳ��� Integral Const
    float  derivative;         //΢�ֳ��� Derivative Const
    int lastError;              //Error[-1]
    int prevError;              //Error[-2]
    bool  isDeviation;              //����ֵ�Ƿ�����趨ֵ     1Ϊ���ӣ�   2Ϊ�����趨ֵ
} PID;

typedef struct{
  float proportion;
  float  integral;
  float  derivative;
}PIDParameter;

typedef struct{
  int deviationNow;             //���������ߵ�ƫ��
  int deviationLast;            //��һ������ε�ƫ��Ĳ�ֵ  |deviationNow - deviationLast|
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
