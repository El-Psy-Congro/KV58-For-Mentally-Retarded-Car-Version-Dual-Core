#include "include.h"


void LimitingAmplitude(int *value, int min, int max){
  if(max > min){                //Ϊ��ֹ������Ӧ��Ϊ���ص���max<min
    if(*value < min){
      *value = min;
    }else if(*value > max){
      *value = max;
    }
  }else{
    if(*value < max){
      *value = max;
    }else if(*value > min){
      *value = min;
    }
  }
}

int LimitingAmplitudeVersionReturn(int value, int min, int max){
  if(max > min){                //Ϊ��ֹ������Ӧ��Ϊ���ص���max<min
    if(value < min){
      value = min;
    }else if(value > max){
      value = max;
    }
  }else{
    if(value < max){
      value = max;
    }else if(value > min){
      value = min;
    }
  }
  
  return value;
}

float Average(){

}

float Normalized(int value, int multiple, int min, int max){
  return (value-min)*multiple/(max-min);
}


