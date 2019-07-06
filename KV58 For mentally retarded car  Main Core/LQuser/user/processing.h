 #ifndef _PROCESSING_H
#define _PROCESSING_H

#define AD_Data0  ADC0_Ave(ADC0_SE9,ADC_16bit,10)
#define AD_Data1  ADC0_Ave(ADC0_SE11,ADC_16bit,10)
#define AD_Data2  ADC0_Ave(ADC0_SE5a,ADC_16bit,10)
#define AD_Data3  ADC0_Ave(ADC0_DP2,ADC_16bit,10)
#define AD_Data4  ADC0_Ave(ADC0_DP3,ADC_16bit,10)
#define AD_Data5  ADC0_Ave(ADC0_DP1,ADC_16bit,10)
#define AD_Data6  ADC0_Ave(ADC0_SE10,ADC_16bit,10)


extern uint16_t ADvalue[7],AD_Data[7];
extern int AD_cha;
extern int AD_sum;
extern int flag4;
extern int len;

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
extern s16 kernelImage[GRAPH_HIGHT][GRAPH_WIDTH], gaussianImage[GRAPH_HIGHT][GRAPH_WIDTH];
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
s16 GraphProcessingOfLineScanFromEdge(int i);                    //边缘扫描
s16 GraphProcessingOfLineScanFromQuarters(int i);                //边缘扫描
int GraphProcessingOfLineWhitePointCounting(int i, int n, int m);       //行白点计算
int GraphProcessingOfSquareAreaWhitePointCounting(int x, int y, int area);  //区白点计数
bool IsDisconnectRoad();                                                    //断路判断
bool IsStaightLine(s16 line[], bool isExistence[], u8 initial, u8 half, u8 allowableError, float MIissing);      //直线判断
bool GraphProcessingOfEnteringIslandforLeftPreconditions();                 //左环岛前条件
bool GraphProcessingOfEnteringIslandforLeft();                              //左环岛
bool GraphProcessingOfEnteringIslandforRight();                             //右环岛
bool GraphProcessingOfEnteringIslandofElectromagnetism();                             //环岛摄像头中采用电磁
bool IsStraightLane();                                                      //直道判断
bool IsGraphProcessingOfFinishLine();                                       //终点线判断
bool IsModeSwitch();
void GraphProcessingOfEnteringStraightLaneAccelerate();                     //直道加速
int GraphProcessingOfCannyEdgeDetection();                                  //canny直线检测                 未完成 且计算量大后面应该不会去写了
void GraphProcessingOfProspectadjustment(u16 aline);                        //摄像头前瞻调整

void DifferentialSpeed();                                                   //差速


int ElectromagnetismProcessing();
int ElectromagnetismProcessingOfBasics();                                   //电磁处理  四电感
int ElectromagnetismProcessingOfBasicsPlus();                                  //电磁处理 六电感
bool ElectromagnetismProcessingOfIsland();                                  //电磁环岛
bool ElectromagnetismProcessingOfLoseDataForStop();                         //电磁完全丢失停车

void Ultrasonic();                                                          //超声波测距

void GyroAngleProcessing();

#endif
