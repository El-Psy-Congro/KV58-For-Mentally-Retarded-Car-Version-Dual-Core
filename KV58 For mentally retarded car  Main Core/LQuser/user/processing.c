#include "include.h"

/*****************************Graph*******************************/
#define LINE_INITIAL (GRAPH_HIGHT - 8)    //��LINE_INITIAL�п�ʼɨ��(�н���Զ)
#define LINE_TERMINATION 10               //��LINE_TERMINATION�н���
#define LINE_MEDIAN (LINE_INITIAL + LINE_TERMINATION)/2
#define LINE_QUARTERS 24                  //�е��ķ�֮һ��
#define LINE_EDGE_LEFT 4                  //��߽�ɨ���յ�
#define LINE_EDGE_RIGHT 92                //�ұ߽�ɨ���յ�
#define HEI 1
#define BAI 0
#define LINE 40                           //��ǰʹ����
/*****************************************************************/

int servoMedian = 5250, graphMedian = 0, servo = 0, angle = 0, gyroLast = 0;
float angleFromGyro = 0, angleFromAcceleration = 0;
/*****************************************************************/
int dRpm = 0, M = 0;
float turnAngle = 0.08*3.1415926/180.0, A = 0.05;
double steeringAngle;


/*****************************Electromagnetism************************************/
uint16_t ADvalue[6]={0,0,0,0,0,0};
uint16_t AD_Data[6]={0,0,0,0,0,0};
int admax[6]={35000,35000,35000,35000,35000,35000}, admin[6]={2,2,2,2,2,2};
int ia;
int AD_sum=0;
int AD_cha=0;
int flag1=0;
int flag2=0;
int flag3=0;
/*****************************************************************/

/*****************************Graph*******************************/
s8
  edgeLeft[GRAPH_HIGHT],
  edgeRight[GRAPH_HIGHT],               //���ұ߽�
  graphicMedian[GRAPH_HIGHT] = 47;       //ÿһ��ͼ����ֵ
  
s8 laneWidth[60] = {10,10,10,10,10,10,10,10,11,13,13,15,        //��·���     Ӧ�����α����Կ�Ȳ�һ
                    16,17,19,19,20,22,23,24,25,26,27,28,
                    30,30,32,33,34,35,37,38,38,40,41,42,
                    43,45,45,47,48,48,50,51,52,53,55,55,
                    57,58,59,60,61,62,64,64,65,67,67,69};



bool
  isIslandLeft = false,
  isIslandRight = false,
  isEdgeLeft[GRAPH_HIGHT],
  isEdgeRight[GRAPH_HIGHT],             //���ұ߽���ڱ�־      ����Ϊtrue ������Ϊfalse
  isEdgeMedian[GRAPH_HIGHT];            //���ߴ��ڱ�־               ����Ϊtrue ������Ϊfalse  ���ұ߽綼������  ���ܵ��� ���߲�����


s8                                      //�����
  kernelSobelX[3][3] = {{-1, -2, -1}, {0, 0, 0}, {1, 2, 1}},
  kernelSobelY[3][3] = {{-1, 0, 1}, {-2, 0, 2}, {-1, 0, 1}},
  kernelGaussian[5][5] = {{2,4,5,4,2},{4,9,12,9,4},{5,12,15,12,5},{4,9,12,9,4},{2,4,5,4,2}};

s8
  kernelImageX[GRAPH_HIGHT][GRAPH_WIDTH],
  kernelImageY[GRAPH_HIGHT][GRAPH_WIDTH],
  kernelImage[GRAPH_HIGHT][GRAPH_WIDTH],
  gaussianImage[GRAPH_HIGHT][GRAPH_WIDTH];

/*****************************************************************/

/*************************Speed**********************************/
bool
  isStop = false;

/*****************************************************************/


int DataFusion(){
  GetUseImage();                       //�ɼ�ͼ�����ݴ������  �Ƶ���IsDisconnectRoad();
  thresholdOfGraph = GetOSTU(imageData);      //OSTU��� ��ȡȫ����ֵ
  GetBinarizationValue();              //��ֵ��ͼ������
  GraphProcessing();


  if(0){

    servo = servoMedian - PIDPositional(ElectromagnetismProcessingOfIsland(), &PIDServoOfElectromagnetism);
  }else if(0){
    servo = servoMedian - PIDPositional(ElectromagnetismProcessingOfIsland(), &PIDServoOfElectromagnetism);

  }else if(!isEdgeLeft[LINE] && !isEdgeLeft[LINE]){
    servo = servoMedian - PIDPositional(ElectromagnetismProcessingOfBasics(), &PIDServoOfElectromagnetism);
    BEE_ON;
  }else{
    servo = servoMedian + PIDFuzzy(&graphic, &PIDServoOfGraph);
    BEE_OFF;
  }
  if(IsGraphProcessingOfFinishLine()){
    isStop = true;
  }
    
  GraphProcessingOfEnteringStraightLaneAccelerate();

  DifferentialSpeed();
  

}
/*
 * ���ټ���
 */
void DifferentialSpeed(){
  steeringAngle = ((servo - servoMedian)*turnAngle);
  dRpm = (PIDMotor.setPoint*(M+A*tan(steeringAngle)));

  LimitingAmplitude(&dRpm, -PIDMotor.setPoint*(5/4), PIDMotor.setPoint*(5/4));

  PIDMotorRight.setPoint = PIDMotor.setPoint + dRpm;
  PIDMotorLeft.setPoint = PIDMotor.setPoint - dRpm;
}

int GraphProcessing(){
  if(fieldOverFlag){
    graphic.deviationLast = graphic.deviationNow;
    graphic.deviationNow =  GraphProcessingOfEdgeFluctuation() - 47;
    fieldOverFlag = 0;

  }
  return graphMedian;
}



int GraphProcessingOfCannyEdgeDetection(){
  for(int i = GRAPH_HIGHT - 3; i > 2; i--){
    for (int j = GRAPH_WIDTH - 3; j > 2; j--){


      gaussianImage[i][j] = (Image_Use[i-2][j-2]*kernelGaussian[0][0]
                                   +Image_Use[i-2][j-1]*kernelGaussian[0][1]
                                   +Image_Use[i-2][j]*kernelGaussian[0][2]
                                   +Image_Use[i-2][j+1]*kernelGaussian[0][3]
                                   +Image_Use[i-2][j+2]*kernelGaussian[0][4]
                                   +Image_Use[i-1][j-2]*kernelGaussian[1][0]
                                   +Image_Use[i-1][j-1]*kernelGaussian[1][1]
                                   +Image_Use[i-1][j]*kernelGaussian[1][2]
                                   +Image_Use[i-1][j+1]*kernelGaussian[1][3]
                                   +Image_Use[i-1][j+2]*kernelGaussian[1][4]
                                   +Image_Use[i][j-2]*kernelGaussian[2][0]
                                   +Image_Use[i][j-1]*kernelGaussian[2][1]
                                   +Image_Use[i][j]*kernelGaussian[2][2]
                                   +Image_Use[i][j+1]*kernelGaussian[2][3]
                                   +Image_Use[i][j+2]*kernelGaussian[2][4]
                                   +Image_Use[i+1][j-2]*kernelGaussian[3][0]
                                   +Image_Use[i+1][j-1]*kernelGaussian[3][1]
                                   +Image_Use[i+1][j]*kernelGaussian[3][2]
                                   +Image_Use[i+1][j+1]*kernelGaussian[3][3]
                                   +Image_Use[i+1][j+2]*kernelGaussian[3][4]
                                   +Image_Use[i+2][j-2]*kernelGaussian[4][0]
                                   +Image_Use[i+2][j-1]*kernelGaussian[4][1]
                                   +Image_Use[i+2][j]*kernelGaussian[4][2]
                                   +Image_Use[i+2][j+1]*kernelGaussian[4][3]
                                   +Image_Use[i+2][j-2]*kernelGaussian[4][4])/159;



      kernelImageX[i][j] = ABS(gaussianImage[i-1][j-1]*kernelSobelX[0][0]
                                   +gaussianImage[i-1][j]*kernelSobelX[0][1]
                                   +gaussianImage[i][j-1]*kernelSobelX[1][0]
                                   +gaussianImage[i][j]*kernelSobelX[1][1]
                                   +gaussianImage[i][j+1]*kernelSobelX[1][2]
                                   +gaussianImage[i+1][j]*kernelSobelX[2][1]
                                   +gaussianImage[i+1][j+1]*kernelSobelX[2][2]
                                   +gaussianImage[i-1][j+1]*kernelSobelX[0][2]
                                   +gaussianImage[i+1][j-1]*kernelSobelX[2][0]);
      if(kernelImageX[i][j] < 0){
        kernelImageX[i][j] = 0;
      }else if(kernelImageX[i][j] > 255){
        kernelImageX[i][j] = 255;
      }
      kernelImageY[i][j] = ABS(gaussianImage[i-1][j-1]*kernelSobelY[0][0]
                                                                       +gaussianImage[i-1][j]*kernelSobelY[0][1]
                                                                       +gaussianImage[i][j-1]*kernelSobelY[1][0]
                                                                       +gaussianImage[i][j]*kernelSobelY[1][1]
                                                                       +gaussianImage[i][j+1]*kernelSobelY[1][2]
                                                                       +gaussianImage[i+1][j]*kernelSobelY[2][1]
                                                                       +gaussianImage[i+1][j+1]*kernelSobelY[2][2]
                                                                       +gaussianImage[i-1][j+1]*kernelSobelY[0][2]
                                                                       +gaussianImage[i+1][j-1]*kernelSobelY[2][0]);

      kernelImage[i][j] = sqrt(square(kernelImageX[i][j]) + square(kernelImageY[i][j]));
      if(kernelImage[i][j] < 0){
        kernelImage[i][j] = 0;
      }else if(kernelImage[i][j] > 255){
        kernelImage[i][j] = 255;
      }

    }
  }
}

int GraphProcessingOfEdgeFluctuation(){
//    graph[GRAPH_HIGHT][LCHW]
  //  GraphProcessingOfLineScanFromEdge(LINE_INITIAL);
    for(int i = LINE_INITIAL; i > LINE_TERMINATION; i--){
      isEdgeLeft[i] = false;
      isEdgeRight[i] = false;
      isEdgeMedian[i] = false;


      if(!graph[i][graphicMedian[i+1]]){
        if(i == LINE_INITIAL){
          GraphProcessingOfLineScanFromCentralDiffusion(i);
          graphicMedian[i] = (edgeLeft[i] + edgeRight[i])/2;
        }else{
          GraphProcessingOfLineScanFromCentralDiffusion(i);
        graphicMedian[i] = (edgeLeft[i] + edgeRight[i])/2;
      }
    }else{
      GraphProcessingOfLineScanFromSettingPoint(i, graphicMedian[i+1]);
      graphicMedian[i] = (edgeLeft[i] + edgeRight[i])/2;
    }
  }

  
/***********************�����ж�******************************/
  if(GraphProcessingOfEnteringIslandforLeft()){
    isIslandLeft = true;
  }else if(GraphProcessingOfEnteringIslandforRight()){
    isIslandRight = true;
  }
/************************************************************/
//  IsStaightLine(edgeRight, isEdgeRight, 50, 30, 3, 0.5);

  for(int i = GRAPH_WIDTH; i > 0; i--){
    graph[LINE][i] = 0;
    graph[LINE_MEDIAN][i] = 0;
    graph[LINE_INITIAL][i] = 0;
    graph[LINE_TERMINATION][i] = 0;
    graph[LINE_TERMINATION + 10][i] = 0;
  }

//  for(int i = 14; i<40; i++){
//    if(isEdgeLeft[i] || isEdgeRight[i]){
//      return graphicMedian[i];
//    }
//  }

  if(isEdgeLeft[LINE] && !isEdgeRight[LINE]){
    graph[LINE][LimitingAmplitudeVersionReturn(edgeLeft[LINE] + laneWidth[LINE], 0, 93)] = 0;             //oled��ʾ�۲��
    return LimitingAmplitudeVersionReturn(edgeLeft[LINE] + laneWidth[LINE], 0, 93);
  }else if(!isEdgeLeft[LINE] && isEdgeRight[LINE]){
    graph[LINE][LimitingAmplitudeVersionReturn(edgeRight[LINE] - laneWidth[LINE], 0, 93)] = 0;
    return LimitingAmplitudeVersionReturn(edgeRight[LINE] - laneWidth[LINE], 0, 93);
  }
//  for(int i = ){
//  }
  graph[LINE][graphicMedian[LINE]] = 0;
  return graphicMedian[LINE];
}

void GraphProcessingOfLineScanFromMedian(int i){
  for(int j = graphicMedian[i+1]; j>LINE_EDGE_LEFT; j--){
    if(!graph[i][j] && !graph[i][j-1] && graph[i][j+1] && graph[i][j+2]){
      isEdgeLeft[i] = true;
      edgeLeft[i] = j;
      break;
    }
  }

  for(int j = graphicMedian[i+1]; j < LINE_EDGE_RIGHT; j++){
    if(!graph[i][j] && !graph[i][j+1] && graph[i][j-1] && graph[j-2]){
      isEdgeRight[i] = true;
      edgeRight[i] = j;
      break;
    }
  }
}

//���ض���������ɨ��߽磻 iΪָ���У� mΪ�ض���
//��GraphProcessingOfLineScanFromSettingPoint(30, 40)
//��ʾΪ��ͼ��ĵ�30�еĵ�40��ʼ������ɨ��ͻ��
void GraphProcessingOfLineScanFromSettingPoint(int i, int m){
  for(int j = m; j > LINE_EDGE_LEFT; j--){
    if(!graph[i][j] && !graph[i][j-1] && graph[i][j+1] && graph[i][j+2]){
      isEdgeLeft[i] = true;
      edgeLeft[i] = j;
      break;
    }
  }
  if(!isEdgeLeft[i]){
    edgeLeft[i] = 0;
  }

  for(int j = m; j < LINE_EDGE_RIGHT; j++){
    if(!graph[i][j] && !graph[i][j+1] && graph[i][j-1] && graph[j-2]){
      isEdgeRight[i] = true;
      edgeRight[i] = j;
      break;
    }
  }

  if(!isEdgeRight[i]){
    edgeRight[i] = GRAPH_WIDTH - 1;
  }
}
/*
 * ɨ���i�еı߽�
 * ɨ���ص�
 *    ��x��������ɨ��
 *    xΪ���м��������ƶ������������İ׵�
 */
void GraphProcessingOfLineScanFromCentralDiffusion(int i){
  for(int j = GRAPH_WIDTH/2; j > 0; j--){
    if(graph[i][j] && graph[i][j+1] && graph[i][j-1] && graph[i][j-2]){
      GraphProcessingOfLineScanFromSettingPoint(i, j);
      break;
    }

    if(graph[i][GRAPH_HIGHT-j] && graph[i][GRAPH_HIGHT-j-1] && graph[i][GRAPH_HIGHT-j+1] && graph[i][GRAPH_HIGHT-j+2]){
      GraphProcessingOfLineScanFromSettingPoint(i, GRAPH_HIGHT-j);
      break;
    }
  }
}

/*
 * ͳ�Ƶ�i�е�n�У�m��֮��İ׵���
 * ���ذ׵���Ŀ
 * i Ҫɨ�����
 * n ��i�е�n�㿪ʼ
 * m ��i�е�m�����
 */
int GraphProcessingOfLineWhitePointCounting(int i, int n, int m){
  int count = 0;
  for(int j = n; j < m; j++){
    if(graph[i][j]){
      count++;
    }
  }
  return count;
}


/*
 * ͳ��ĳ���������ڵİ׵���
 * ���ذ׵���Ŀ
 * x Ҫɨ��ķ�����������½�x����
 * y Ҫɨ��ķ�����������½�y����
 * area Ҫɨ��ķ�������ı߳�
 * ɨ������ԣ�x�� y��Ϊ���½������  area * area ����
 */
int GraphProcessingOfSquareAreaWhitePointCounting(int x, int y, int area){
  int count = 0;
  for(int i = x; i > x - area; i--){
    for(int j = y; j < y + area; j++){
      if(graph[i][j]){
        count ++;
      }
    }
  }
  return count;
}

/*
 * �������жϡ�����
 * ����bool   true Ϊ�ǻ���           falseΪ���ǻ���
 *
 * �ó���ļ�ⷽ��
 *      �ȼ��һ���Ƿ�Ϊֱ��
 *      �ڼ�⻷���ļ��v
 *          �ȼ���Ƿ�������沿�ִ��ڱ߽磬���沿�ֲ����ڱ߽�
 *          �ڼ���������Ƿ��д����ڵ㣬��������Ƿ񼸺�ȫ��Ϊ��ɫ
 */
bool GraphProcessingOfEnteringIslandforLeft(){
  if(IsStaightLine(edgeRight, isEdgeRight, 45, 35, 2, 0.5)){
    for(int i = LINE_TERMINATION; i < LINE_INITIAL - 7; i++){
      if(isEdgeLeft[i] && isEdgeLeft[i+1] && isEdgeLeft[i+2] && !isEdgeLeft[i+3] && !isEdgeLeft[i+4] && !isEdgeLeft[i+5]){
        if(edgeLeft[i+2] < 11 || !isEdgeRight[i+2] || !isEdgeRight[i+3] || !isEdgeRight[i+5] || !isEdgeRight[i+6]){
          continue;
        }
        if(GraphProcessingOfSquareAreaWhitePointCounting(i+2, edgeLeft[i+2]-5, 11) < 50
        && GraphProcessingOfSquareAreaWhitePointCounting(i+2, edgeLeft[i+2]-5, 11) > 7
        && GraphProcessingOfSquareAreaWhitePointCounting(i+2+12, edgeLeft[i+2]-5, 11) > 119){

          graph[i+2][edgeLeft[i+2]-5] = 0;
          graph[i+2][edgeLeft[i+2]-4] = 0;
          graph[i+2][edgeLeft[i+2]-3] = 0;
          graph[i+2][edgeLeft[i+2]-2] = 0;
          graph[i+2][edgeLeft[i+2]-1] = 0;
          graph[i+2][edgeLeft[i+2]-1] = 0;
          graph[i+2][edgeLeft[i+2]+1] = 0;
          graph[i+2][edgeLeft[i+2]+2] = 0;
          graph[i+2][edgeLeft[i+2]+3] = 0;
          graph[i+2][edgeLeft[i+2]+4] = 0;


          graph[i+2-11][edgeLeft[i+2]-5] = 0;
          graph[i+2-11][edgeLeft[i+2]-4] = 0;
          graph[i+2-11][edgeLeft[i+2]-3] = 0;
          graph[i+2-11][edgeLeft[i+2]-2] = 0;
          graph[i+2-11][edgeLeft[i+2]-1] = 0;
          graph[i+2-11][edgeLeft[i+2]-1] = 0;
          graph[i+2-11][edgeLeft[i+2]+1] = 0;
          graph[i+2-11][edgeLeft[i+2]+2] = 0;
          graph[i+2-11][edgeLeft[i+2]+3] = 0;
          graph[i+2-11][edgeLeft[i+2]+4] = 0;

          graph[i+2-11][edgeLeft[i+2]-5] = 0;
          graph[i+2-1][edgeLeft[i+2]-5] = 0;
          graph[i+2-2][edgeLeft[i+2]-5] = 0;
          graph[i+2-3][edgeLeft[i+2]-5] = 0;
          graph[i+2-4][edgeLeft[i+2]-5] = 0;
          graph[i+2-5][edgeLeft[i+2]-5] = 0;
          graph[i+2-6][edgeLeft[i+2]-5] = 0;
          graph[i+2-7][edgeLeft[i+2]-5] = 0;
          graph[i+2-8][edgeLeft[i+2]-5] = 0;
          graph[i+2-9][edgeLeft[i+2]-5] = 0;
          graph[i+2-10][edgeLeft[i+2]-5] = 0;

          graph[i+2-11][edgeLeft[i+2]+5] = 0;
          graph[i+2-1][edgeLeft[i+2]+5] = 0;
          graph[i+2-2][edgeLeft[i+2]+5] = 0;
          graph[i+2-3][edgeLeft[i+2]+5] = 0;
          graph[i+2-4][edgeLeft[i+2]+5] = 0;
          graph[i+2-5][edgeLeft[i+2]+5] = 0;
          graph[i+2-6][edgeLeft[i+2]+5] = 0;
          graph[i+2-7][edgeLeft[i+2]+5] = 0;
          graph[i+2-8][edgeLeft[i+2]+5] = 0;
          graph[i+2-9][edgeLeft[i+2]+5] = 0;
          graph[i+2-10][edgeLeft[i+2]+5] = 0;
          return true;
        }else{

        }

      }
    }
  }

  return false;
}


/*
 * �������жϡ�����
 * ����bool   true Ϊ�ǻ���           falseΪ���ǻ���
 *
 * �ó���ļ�ⷽ��
 *      �ȼ��һ���Ƿ�Ϊֱ��
 *      �ڼ�⻷���ļ��v
 *          �ȼ���Ƿ�������沿�ִ��ڱ߽磬���沿�ֲ����ڱ߽�
 *          �ڼ���������Ƿ��д����ڵ㣬��������Ƿ񼸺�ȫ��Ϊ��ɫ
 */
bool GraphProcessingOfEnteringIslandforRight(){
  if(IsStaightLine(edgeLeft, isEdgeLeft, 45, 35, 2, 0.5)){
    for(int i = LINE_TERMINATION; i < LINE_INITIAL - 7; i++){
      if(isEdgeRight[i] && isEdgeRight[i+1] && isEdgeRight[i+2] && !isEdgeRight[i+3] && !isEdgeRight[i+4] && !isEdgeRight[i+5]){
        if(edgeRight[i+2] > 72 || !isEdgeLeft[i+2] || !isEdgeLeft[i+3] || !isEdgeLeft[i+5] || !isEdgeLeft[i+6]){
          continue;
        }
        if(GraphProcessingOfSquareAreaWhitePointCounting(i+2, edgeRight[i+2]-5, 11) < 50
        && GraphProcessingOfSquareAreaWhitePointCounting(i+2, edgeRight[i+2]-5, 11) > 7
        && GraphProcessingOfSquareAreaWhitePointCounting(i+2+12, edgeRight[i+2]-5, 11) > 119){

          graph[i+2][edgeRight[i+2]-5] = 0;
          graph[i+2][edgeRight[i+2]-4] = 0;
          graph[i+2][edgeRight[i+2]-3] = 0;
          graph[i+2][edgeRight[i+2]-2] = 0;
          graph[i+2][edgeRight[i+2]-1] = 0;
          graph[i+2][edgeRight[i+2]-1] = 0;
          graph[i+2][edgeRight[i+2]+1] = 0;
          graph[i+2][edgeRight[i+2]+2] = 0;
          graph[i+2][edgeRight[i+2]+3] = 0;
          graph[i+2][edgeRight[i+2]+4] = 0;


          graph[i+2-11][edgeRight[i+2]-5] = 0;
          graph[i+2-11][edgeRight[i+2]-4] = 0;
          graph[i+2-11][edgeRight[i+2]-3] = 0;
          graph[i+2-11][edgeRight[i+2]-2] = 0;
          graph[i+2-11][edgeRight[i+2]-1] = 0;
          graph[i+2-11][edgeRight[i+2]-1] = 0;
          graph[i+2-11][edgeRight[i+2]+1] = 0;
          graph[i+2-11][edgeRight[i+2]+2] = 0;
          graph[i+2-11][edgeRight[i+2]+3] = 0;
          graph[i+2-11][edgeRight[i+2]+4] = 0;

          graph[i+2-11][edgeRight[i+2]-5] = 0;
          graph[i+2-1][edgeRight[i+2]-5] = 0;
          graph[i+2-2][edgeRight[i+2]-5] = 0;
          graph[i+2-3][edgeRight[i+2]-5] = 0;
          graph[i+2-4][edgeRight[i+2]-5] = 0;
          graph[i+2-5][edgeRight[i+2]-5] = 0;
          graph[i+2-6][edgeRight[i+2]-5] = 0;
          graph[i+2-7][edgeRight[i+2]-5] = 0;
          graph[i+2-8][edgeRight[i+2]-5] = 0;
          graph[i+2-9][edgeRight[i+2]-5] = 0;
          graph[i+2-10][edgeRight[i+2]-5] = 0;

          graph[i+2-11][edgeRight[i+2]+5] = 0;
          graph[i+2-1][edgeRight[i+2]+5] = 0;
          graph[i+2-2][edgeRight[i+2]+5] = 0;
          graph[i+2-3][edgeRight[i+2]+5] = 0;
          graph[i+2-4][edgeRight[i+2]+5] = 0;
          graph[i+2-5][edgeRight[i+2]+5] = 0;
          graph[i+2-6][edgeRight[i+2]+5] = 0;
          graph[i+2-7][edgeRight[i+2]+5] = 0;
          graph[i+2-8][edgeRight[i+2]+5] = 0;
          graph[i+2-9][edgeRight[i+2]+5] = 0;
          graph[i+2-10][edgeRight[i+2]+5] = 0;
          return true;
        }else{

        }

      }
    }
  }

  return false;
}


s8 GraphProcessingOfLineScanFromQuarters(int i){
//  if(graph[i][LINE_EDGE_LEFT] && !graph[i][LINE_EDGE_RIGHT]){
//
//  }else if(!graph[i][LINE_EDGE_LEFT] && graph[i][LINE_EDGE_RIGHT]){
//
//  }else
  if(graph[i][LINE_QUARTERS]){
    GraphProcessingOfLineScanFromSettingPoint(i, LINE_QUARTERS);
  }else if(graph[i][LINE_QUARTERS*3]){
    GraphProcessingOfLineScanFromSettingPoint(i, LINE_QUARTERS*3);
  }

}


/*
 * ֱ������
 */
void GraphProcessingOfEnteringStraightLaneAccelerate(){

  if(isStop){
    PIDMotor.setPoint = 0;
    BEE_ON;
  }else if(IsStraightLane()){
//   BEE_ON;
    PIDMotor.setPoint = 170;
    PIDServoOfGraph.proportion = 12;  //0.27
  }else if(!ElectromagnetismProcessingOfLoseDataForStop()){
//    BEE_OFF;
    PIDMotor.setPoint = 100;
    PIDServoOfGraph.proportion = 9;  //0.27
  }else{
    PIDMotor.setPoint = 0;
  }
}



/*
 * ��·���
 */
bool IsDisconnectRoad(){
  if(GraphProcessingOfLineWhitePointCounting(LINE_INITIAL - 7, 0, GRAPH_WIDTH-1) < 15 ||
     GraphProcessingOfLineWhitePointCounting(LINE_INITIAL - 5, 0, GRAPH_WIDTH-1) < 15 ||
     GraphProcessingOfLineWhitePointCounting(LINE_INITIAL - 3, 0, GRAPH_WIDTH-1) < 15 ||
     GraphProcessingOfLineWhitePointCounting(LINE_INITIAL - 25, 0, GRAPH_WIDTH-1) < 15 &&
     GraphProcessingOfLineWhitePointCounting(LINE_INITIAL - 26, 0, GRAPH_WIDTH-1) < 15 &&
     GraphProcessingOfLineWhitePointCounting(LINE_INITIAL - 27, 0, GRAPH_WIDTH-1) < 15){

    return false;
  }else{

    return true;
  }

}

bool IsSharpBend(){

}

/*
*�յ����ж�
*����ֵ bool 
*true ��ʾ��⵽�յ���
*/

bool IsGraphProcessingOfFinishLine(){
     u8 count[2] = 0, m[2];
     
     
     m[0] = graph[41][LINE_EDGE_LEFT];
     m[1] = graph[44][LINE_EDGE_LEFT];
     for(int j = LINE_EDGE_LEFT;j < LINE_EDGE_RIGHT;j+=2){
       
       if(graph[41][j] != m[0]){
         count[0]++;
         m[0] = graph[41][j];
       }
       
       if(graph[44][j] != m[1]){
         count[1]++;
         m[1] = graph[44][j];
       }
       
       
     }
     if(count[0] >10 && count[1] > 10){
     return true;
     }
     
     return false;
  }

/*
 * ֱ�߼��
 * ���ܣ� ���ָ�����ȱ��Ƿ�Ϊֱ��
 * ����ֵbool
 *
 * �������
 *   line[] Ҫ���ı�
 *   isExistence[] Ҫ���ıߵĴ��ڱ�־
 *   initial    �ӵ�initial��ʼ���
 *   half       Ҫ��ⳤ�ȵ��е�
 *      ����initial�м�鵽2*half-initail����
 *
 *   allowbleError  �������
 *      Խ��Խ�������У�ԽСԽ���ж�
 *
 *   Missing  ��߽���ڸ������
 *      0~1 ֮��
 *      Խ��Ҫ��ߵĴ��ڸ���Խ��
 */
bool IsStaightLine(s8 line[], bool isExistence[], u8 initial, u8 half, u8 allowableError, float MIissing){
//  int accumulator[180][200];
//  int radius;
//  for(int i = LINE_TERMINATION; i > GRAPH_HIGHT/2; i--){
//    if(1){
//      for(int j = 0; j < 180; j++){
//        radius = (int)(i * sin(j) + line[i] * cos(j));
//        accumulator[j][radius]++;
//        if(accumulator[j][radius] > 2){
//          BEE_ON;
//          return true;
//
//        }
//      }
//    }
//  }
//  BEE_OFF;
//  return false;

//  u8 txt[16];
//  int k = 0, slopeDeviation = 0;
//  int j = 0, existence = 0;
//  for(int i = LINE_INITIAL; i > LINE_MEDIAN; i--){
//    j = i - LINE_MEDIAN + 10;
//    if(isExistence[i] && isExistence[j]){
//      if(!k){
//        k = (line[j] - line[i]);
//      }else{
//        slopeDeviation +=  ABS((line[j] - line[i]) - k);
//      }
//      existence++;
//    }
//  }
//
////  sprintf(txt, "%04d", k);
////  LCD_P8x16Str(0, 0, (u8*) txt);
////  sprintf(txt, "%04d", existence);
////  LCD_P8x16Str(0, 2, (u8*) txt);
////  sprintf(txt, "%04d", slopeDeviation);
////  LCD_P8x16Str(0, 4, (u8*) txt);
//
//  if(slopeDeviation < 3*existence && existence > 7){
//    return true;
//  }else{
//    return false;
//  }

//  u8 txt[16];
  int k = 0, slopeDeviation = 0;
  int j = 0, existence = 0;
  for(int i = initial; i > half; i--){
    j = i - (initial - half);
//    graph[j][50] = 0;
//    graph[j][51] = 0;
//    graph[j][52] = 0;
//    graph[j][53] = 0;
//    graph[i][54] = 0;
//    graph[i][55] = 0;
//    graph[i][56] = 0;
//    graph[i][57] = 0;
    if(isExistence[i] && isExistence[j]){
      if(!k){
        k = (line[j] - line[i]);
      }else{
        slopeDeviation +=  ABS((line[j] - line[i]) - k);
      }
      existence++;
    }
  }

//  sprintf(txt, "%04d", k);
//  LCD_P8x16Str(0, 0, (u8*) txt);
//  sprintf(txt, "%04d", existence);
//  LCD_P8x16Str(0, 2, (u8*) txt);
//  sprintf(txt, "%04d", slopeDeviation);
//  LCD_P8x16Str(0, 4, (u8*) txt);

  if(slopeDeviation < allowableError*existence && existence > MIissing*(initial -half)){

    return true;
  }else{

    return false;
  }

}

bool IsStraightLane(){
  if(IsStaightLine(edgeLeft, isEdgeLeft, 45, 30, 2, 0.5) 
     && IsStaightLine(edgeRight, isEdgeRight, 45, 30, 2, 0.5)){
    return true;
  }else{
    return false;
  }
}


int ElectromagnetismProcessing(){
  return ElectromagnetismProcessingOfBasics();
}

int ElectromagnetismProcessingOfBasics(){

  inductance.deviationLast = inductance.deviationNow;
  //inductance.deviationNow = (int)(ADC0_Ave(L_1,ADC_16bit,10)-ADC0_Ave(L_6,ADC_16bit,10)+ADC0_Ave(L_3,ADC_16bit,10)-ADC0_Ave(L_4,ADC_16bit,10))*10000
    //(ADC0_Ave(L_1,ADC_16bit,10)+ADC0_Ave(L_6,ADC_16bit,10)+ADC0_Ave(L_3,ADC_16bit,10)+ADC0_Ave(L_4,ADC_16bit,10));
     AD_Data[0]=AD_Data0;
     AD_Data[1]=AD_Data1;
     AD_Data[2]=AD_Data2;
     AD_Data[3]=AD_Data3;
     AD_Data[4]=AD_Data4;

  for(ia=0;ia<6;ia++)
  {
                if (AD_Data[ia]>admax[ia])
                    AD_Data[ia]=admax[ia];
                if (AD_Data[ia]<admin[ia])
        AD_Data[ia]=admin[ia];
        ADvalue[ia]=(float)(400*(AD_Data[ia]-admin[ia])/(admax[ia]-admin[ia]));
    }



  if(ElectromagnetismProcessingOfIsland()){
    return -(35000-AD_Data4)/4;
  }



    //ad04=(float)((ADvalue[0]-ADvalue[3])/(ADvalue[0]+ADvalue[3]));
    //ad13=(float)((ADvalue[1]-ADvalue[2])/(ADvalue[1]+ADvalue[2]));

    AD_cha=ADvalue[2]+ADvalue[3]-ADvalue[0]-ADvalue[1];
    //ADo��
    AD_sum=ADvalue[0]+ADvalue[1]+ADvalue[3]+ADvalue[4];

       inductance.deviationNow=-((AD_cha*19000)/AD_sum);



  return inductance.deviationNow;


}

bool ElectromagnetismProcessingOfIsland(){
  if(AD_Data4>5000&&AD_Data0>22000)// ʶ���԰
  {  flag1++;

  }

  if(flag1>0&&AD_Data3>15000)
  {
    flag2++;
  }

  if(flag2>0&&AD_Data0>26000&&AD_Data3>20000&&AD_Data4<3000)
  {
    flag3++;
  }




  if(flag3>0&&AD_Data0>24000&&AD_Data3>24000)
  {
      BEE_ON;
      return true;

  }else{
    BEE_OFF;
  }

  if(AD_Data4<4000&&AD_Data0<20000)
  {
    flag1=0;
    flag2=0;
    flag3=0;
  }

  return false;

}


bool ElectromagnetismProcessingOfLoseDataForStop(){
  if(AD_Data0 < 3000 &&
    AD_Data0 < 3000 &&
    AD_Data0 < 3000 &&
    AD_Data0 < 3000 ){
    return true;
  }else{
    return false;
  }
}

void GyroAngleProcessing(){
  angle = angleFromGyro - PIDErect.setPoint;

   PIDMotorRight.setPoint = PIDMotorLeft.setPoint = -angle;



}
