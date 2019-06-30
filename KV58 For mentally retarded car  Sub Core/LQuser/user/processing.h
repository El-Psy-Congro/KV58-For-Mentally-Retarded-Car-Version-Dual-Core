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
void GraphProcessingOfLineScanFromMedian(int i);                //��Եɨ��
void GraphProcessingOfLineScanFromSettingPoint(int i, int m);   //��Եɨ��
void GraphProcessingOfLineScanFromCentralDiffusion(int i);      //��Եɨ��
s8 GraphProcessingOfLineScanFromEdge(int i);                    //��Եɨ��
s8 GraphProcessingOfLineScanFromQuarters(int i);                //��Եɨ��
int GraphProcessingOfLineWhitePointCounting(int i, int n, int m);       //�а׵����
int GraphProcessingOfSquareAreaWhitePointCounting(int x, int y, int area);  //���׵����
bool IsDisconnectRoad();                                                    //��·�ж�
bool IsStaightLine(s8 line[], bool isExistence[], u8 initial, u8 half, u8 allowableError, float MIissing);      //ֱ���ж�
bool GraphProcessingOfEnteringIslandforLeft();                              //�󻷵�
bool GraphProcessingOfEnteringIslandforRight();                             //�һ���
bool IsStraightLane();                                                      //ֱ���ж�
bool IsGraphProcessingOfFinishLine();                                       //�յ����ж�
void GraphProcessingOfEnteringStraightLaneAccelerate();                     //ֱ������
int GraphProcessingOfCannyEdgeDetection();                                  //cannyֱ�߼��                 δ��� �Ҽ����������Ӧ�ò���ȥд��

void DifferentialSpeed();                                                   //����


int ElectromagnetismProcessing();
int ElectromagnetismProcessingOfBasics();
bool ElectromagnetismProcessingOfIsland();
bool ElectromagnetismProcessingOfLoseDataForStop();

void GyroAngleProcessing();

#endif
