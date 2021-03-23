/*
 * @Description: 
 * @Company: Optimus Prime, Inc.
 * @Author: Qijian Lu
 * @Date: 2021-03-18 14:55:38
 * @LastEditors: Qijian Lu
 * @LastEditTime: 2021-03-18 16:07:13
 */

#ifndef __DATA_H
#define __DATA_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "main.h"

    /* Kalman filter data struction */
    typedef struct
    {
        double ResourceData;
        double ProcessNoise_Q;
        double MeasureNoise_R;
        double X_Last;
        double X_Mid;
        double X_Now;
        double P_Last;
        double P_Mid;
        double P_Now;
        double KG;
    } KalmanFilterStructTypedef;

/* For Kalman filter */
#define KALMAN_Q 0.05 // 0.0001
#define KALMAN_R 1.0  // 100.0000
#define ADC_CHANNEL_CNT 1
    extern uint32_t ADC_Buf[ADC_CHANNEL_CNT];
    extern uint32_t ADC_Cnt;
    extern double PWM_Compare_Percentage;
    extern uint16_t Compare;
    extern uint32_t Band_Time;
    extern uint32_t Band_Max_Time;
    extern uint32_t Band_Min_Time;
    extern uint32_t Step_Time;

    extern KalmanFilterStructTypedef KalmanFilter_KnobValue;

    /* Kalman filter */
    double KalmanFilter(double rscData, KalmanFilterStructTypedef *filter);

#ifdef __cplusplus
}
#endif

#endif // !__DATA_H
