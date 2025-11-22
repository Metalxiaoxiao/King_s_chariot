#include "mecanum/mecanum.h"
#include <math.h>
#define L       0.10f        // 前后轮距一半 (m)
#define W       0.10f        // 左右轮距一半 (m)
#define R       0.029f       // 轮子半径 25mm
#define MAX_PWM 999



void Mecanum_Calc(float vx, float vy, float omega, int16_t out_pwm[4])
{
    float k = (L + W);

    // ====== 正确的麦克纳姆轮逆运动学 =======
    float w1 = ( vy - vx - omega * k ) / R;   
    float w0 = ( vy + vx + omega * k ) / R;   
    float w2 = ( vy + vx - omega * k ) / R;   
    float w3 = ( vy - vx + omega * k ) / R;   

    float arr[4] = {w0, w1, w2, w3};

    // ========= 比例控制逻辑 (Proportional Control) ==========
    // 设定参考最大速度 (对应满 PWM)，只有超过时才限制，否则按比例输出。
    // 假设 vx = 30.0f 时达到满速 (参考 main.c 中的系数)
    float ref_max_speed = 30.0f; 
    float ref_max_w = ref_max_speed / R;

    // 计算固定的比例系数
    float scale = (float)MAX_PWM / ref_max_w;

    // 找出当前计算出的最大轮速
    float maxv = 0;
    for(int i=0;i<4;i++){
        if(fabsf(arr[i]) > maxv)
            maxv = fabsf(arr[i]);
    }

    // 如果当前最大轮速超过了参考最大值（即会导致 PWM > MAX_PWM），
    // 则需要缩小比例，以保持运动方向（防饱和）。
    if (maxv * scale > (float)MAX_PWM) {
        scale = (float)MAX_PWM / maxv;
    }

    // ========= 输出四个 PWM =============
    for(int i=0;i<4;i++){
        int16_t pwm = (int16_t)(arr[i] * scale);

        if(pwm > MAX_PWM)  pwm = MAX_PWM;
        if(pwm < -MAX_PWM) pwm = -MAX_PWM;

        out_pwm[i] = pwm;
    }
}
