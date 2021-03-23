/*
 * @Description: Data process logic and algorithms
 * @Company: Optimus Prime, Inc.
 * @Author: Qijian Lu
 * @Date: 2021-03-18 14:54:46
 * @LastEditors: Qijian Lu
 * @LastEditTime: 2021-03-23 15:51:17
 */

#include "data.h"

uint32_t ADC_Buf[ADC_CHANNEL_CNT];
uint32_t ADC_Cnt = 0;
double PWM_Compare_Percentage = 0.0;
uint16_t Compare = 0;
uint32_t Band_Max_Time = 2500;
uint32_t Band_Min_Time = 500;
uint32_t Band_Time = 2000;
uint32_t Step_Time = 2;

/* Kalman filters for knob value */
KalmanFilterStructTypedef KalmanFilter_KnobValue = {
    .ResourceData = 0.0,
    .ProcessNoise_Q = KALMAN_Q,
    .MeasureNoise_R = KALMAN_R,
    .X_Last = 0.0,
    .X_Mid = 0.0,
    .X_Now = 0.0,
    .P_Last = 0.0,
    .P_Mid = 0.0,
    .P_Now = 0.0,
    .KG = 0.0};

/* Kalman filter */
double KalmanFilter(double rscData, KalmanFilterStructTypedef *filter)
{
    filter->ResourceData = rscData;
    filter->X_Mid = filter->X_Last;                          // x_last=x(k-1|k-1),x_mid=x(k|k-1)
    filter->P_Mid = filter->P_Last + filter->ProcessNoise_Q; // p_mid=p(k|k-1),p_last=p(k-1|k-1), Q=Noise

    /* Five formulas of Kalman filter */
    filter->KG = filter->P_Mid / (filter->P_Mid + filter->MeasureNoise_R);
    filter->X_Now = filter->X_Mid + filter->KG * (filter->ResourceData - filter->X_Mid);
    filter->P_Now = (1 - filter->KG) * filter->P_Mid;
    filter->P_Last = filter->P_Now;
    filter->X_Last = filter->X_Now;

    return filter->X_Now;
}
