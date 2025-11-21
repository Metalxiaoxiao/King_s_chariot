#ifndef MECANUM_H
#define MECANUM_H
#include <stdint.h>
void Mecanum_Calc(float vx,float vy,float omega,int16_t out_pwm[4]);
#endif
