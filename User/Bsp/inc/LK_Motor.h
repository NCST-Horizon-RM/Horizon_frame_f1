/*
 * @Descripttion: 
 * @version: 
 * @Author: Eugene
 * @Date: 2024-07-04 16:02:33
 * @LastEditors: Andy
 * @LastEditTime: 2024-07-05 15:45:32
 */
#ifndef __LK_MOTOR_H
#define __LK_MOTOR_H

#include "MY_define.h"
#include "can_bsp.h"
#include "controller.h"
#include "DJI_Motor.h"

typedef struct
{
    int8_t ONLINE_JUDGE_TIME;
    int16_t Angle_last; // 上一个角度值
    int16_t Angle_now;  // 现在的角度值
    int16_t Speed_last; // 上一个速度值
    int16_t Speed_now;  // 现在的速度值
    int16_t current;
    int8_t temperature;
    int32_t Angle_Infinite;
    int64_t Stuck_Time;
    uint16_t Stuck_Flag[2];
    int16_t Laps;
    float Error;
    float Aim;
    float Aim_last;
    float dt;
}LK_MOTOR_DATA_Typedef;

typedef struct
{
    uint8_t PID_INIT;
    LK_MOTOR_DATA_Typedef DATA;
    PID_t PID_P;
    PID_t PID_S;
}LK_MOTOR_Typedef;

typedef struct 
{
    DJI_MOTOR_Typedef M3508[4];
    DJI_MOTOR_Typedef M6020[2];
    LK_MOTOR_Typedef M6020_lk[2];
}MOTOR_Typedef;


void LKMF_iq_ctrl(CAN_HandleTypeDef* hcan, uint16_t motor_id, int32_t iqControl);
void LKMF_Data_Read(CAN_HandleTypeDef* hcan, uint16_t motor_id);
void LK_MotorResolve(DJI_MOTOR_Typedef *motor,uint8_t *RxMessage);

#endif
