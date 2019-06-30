#ifndef _PROCESSING_H
#define _PROCESSING_H

#define AD_Data0  ADC0_Ave(ADC0_SE9,ADC_16bit,10)
#define AD_Data1  ADC0_Ave(ADC0_SE5a,ADC_16bit,10)
#define AD_Data2  ADC0_Ave(ADC0_DP2,ADC_16bit,10)
#define AD_Data3  ADC0_Ave(ADC0_SE10,ADC_16bit,10)
#define AD_Data4  ADC0_Ave(ADC0_DP1,ADC_16bit,10)

extern uint16_t ADvalue[6],AD_Data[6];
extern int AD_cha;
extern int AD_sum;

#define L_0 ADC0_DP1
#define L_3 ADC0_SE5a
#define L_4 ADC0_DP2
#define L_5 ADC0_DP3
#define L_2 ADC0_SE11
#define L_1 ADC0_SE9
#define L_7 ADC0_SE4a
#define L_6 ADC0_SE10


extern int32 servoMedian, servo, angle, gyroLast;
extern float angleFromGyro, angleFromAcceleration;
extern s8 kernelImage[GRAPH_HIGHT][GRAPH_WIDTH], gaussianImage[GRAPH_HIGHT][GRAPH_WIDTH];
extern bool
isIslandLeft,
isIslandRight;



int DataFusion();

int GraphProcessing();
int GraphProcessingOfEdgeFluctuation();
void GraphProcessingOfLineEdgeFluctuation();
void GraphProcessingOfLineScanFromMedian(int i);                //边缘扫描
void GraphProcessingOfLineScanFromSettingPoint(int i, int m);   //边缘扫描
void GraphProcessingOfLineScanFromCentralDiffusion(int i);      //边缘扫描
s8 GraphProcessingOfLineScanFromEdge(int i);                    //边缘扫描
s8 GraphProcessingOfLineScanFromQuarters(int i);                //边缘扫描
int GraphProcessingOfLineWhitePointCounting(int i, int n, int m);       //行白点计算
int GraphProcessingOfSquareAreaWhitePointCounting(int x, int y, int area);  //区白点计数
bool IsDisconnectRoad();                                                    //断路判断
bool IsStaightLine(s8 line[], bool isExistence[], u8 initial, u8 half, u8 allowableError, float MIissing);      //直线判断
bool GraphProcessingOfEnteringIslandforLeft();                              //左环岛
bool GraphProcessingOfEnteringIslandforRight();                             //右环岛
bool IsStraightLane();                                                      //直道判断
bool IsGraphProcessingOfFinishLine();                                       //终点线判断
void GraphProcessingOfEnteringStraightLaneAccelerate();                     //直道加速
int GraphProcessingOfCannyEdgeDetection();                                  //canny直线检测                 未完成 且计算量大后面应该不会去写了

void DifferentialSpeed();                                                   //差速


int ElectromagnetismProcessing();
int ElectromagnetismProcessingOfBasics();
bool ElectromagnetismProcessingOfIsland();
bool ElectromagnetismProcessingOfLoseDataForStop();

void GyroAngleProcessing();

#endif
