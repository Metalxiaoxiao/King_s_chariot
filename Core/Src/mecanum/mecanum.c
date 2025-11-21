#include "mecanum.h"
#include <math.h>
#define L 0.12f
#define W 0.10f
#define MAX_PWM 900
void Mecanum_Calc(float vx,float vy,float omega,int16_t out_pwm[4]){
 float k=L+W;
 float w0=vx -vy -omega*k;
 float w1=vx +vy +omega*k;
 float w2=vx +vy -omega*k;
 float w3=vx -vy +omega*k;
 float arr[4]={w0,w1,w2,w3},maxv=1;
 for(int i=0;i<4;i++)if(fabsf(arr[i])>maxv)maxv=fabsf(arr[i]);
 float scale=(maxv>MAX_PWM)?(MAX_PWM/maxv):1.0f;
 for(int i=0;i<4;i++){
  int16_t v=(int16_t)(arr[i]*scale);
  if(v>MAX_PWM)v=MAX_PWM;if(v<-MAX_PWM)v=-MAX_PWM;
  out_pwm[i]=v;
 }
}
