#ifndef _MATHEMATICS_H_
#define _MATHEMATICS_H_

#define ABS(x) ((x)>0?(x):-(x))
#define square(x) (x)*(x)
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MAX(a, b) ((a) > (b) ? (a) : (b))
#define Z(x) ((x)<0?0:(x))

void LimitingAmplitude(int *value, int min, int max);
int LimitingAmplitudeVersionReturn(int value, int min, int max);

#endif
