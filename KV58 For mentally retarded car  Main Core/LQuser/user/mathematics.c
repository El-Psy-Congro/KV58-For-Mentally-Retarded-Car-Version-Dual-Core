#include "include.h"


void LimitingAmplitude(int *value, int min, int max){
  if(max > min){                //为防止输入是应人为因素导致max<min
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
  if(max > min){                //为防止输入是应人为因素导致max<min
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


void Calculate_cicular(circular acircular, point px1, point px2, point px3)
{
    int x1, y1, x2, y2, x3, y3;
    int a, b, c, g, e, f;
    x1 = px1.x;
    y1 = px1.y;
    x2 = px2.x;
    y2 = px2.y;
    x3 = px3.x;
    y3 = px3.y;
    e = 2 * (x2 - x1);
    f = 2 * (y2 - y1);
    g = x2*x2 - x1*x1 + y2*y2 - y1*y1;
    a = 2 * (x3 - x2);
    b = 2 * (y3 - y2);
    c = x3*x3 - x2*x2 + y3*y3 - y2*y2;
    acircular.Center.x = (g*b - c*f) / (e*b - a*f);
    acircular.Center.y = (a*g - c*e) / (a*f - b*e);
    acircular.radius = sqrt((acircular.Center.x-x1)*(acircular.Center.x-x1)+(acircular.Center.y-y1)*(acircular.Center.y-y1));

}

