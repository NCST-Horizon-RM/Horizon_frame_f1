#include "Gimbal_Task.h"
#include "controller.h"
#include "All_init.h"
#include "MY_Define.h"
#include "LK_Motor.h"
#include "VOFA.h"

uint8_t MOTOR_PID_Gimbal_INIT(MOTOR_Typedef *motor)
{
    float PID_P_Pitch[3] = {80.0f, 0.001f, 0.0f};
    float PID_S_Pitch[3] = {1.5f, 0.0f, 0.0f};

    PID_Init(&motor->M6020_lk[PITCH].PID_P, 400.0f, 10.0f, 
              PID_P_Pitch, 1000.0f, 1000.0f, 
              0.7f, 0.7f, 2, 
              Integral_Limit|OutputFilter|ErrorHandle|
              Trapezoid_Intergral|ChangingIntegrationRate|
              Derivative_On_Measurement|DerivativeFilter);
    PID_Init(&motor->M6020_lk[PITCH].PID_S, 800.0f, 50.0f, 
              PID_S_Pitch, 1000.0f, 1000.0f, 
              0.7f, 0.7f, 2, 
              Integral_Limit|OutputFilter|ErrorHandle|
              Trapezoid_Intergral|ChangingIntegrationRate|
              Derivative_On_Measurement|DerivativeFilter);
    
    float PID_P_Yaw[3] = {80.0f, 0.003f, 1.0f};
    float PID_S_Yaw[3] = {2.5f, 0.0f, 0.0f};

    PID_Init(&motor->M6020_lk[YAW].PID_P, 400.0f, 10.0f, 
              PID_P_Yaw, 1000.0f, 1000.0f, 
              0.7f, 0.7f, 2, 
              Integral_Limit|OutputFilter|ErrorHandle|
              Trapezoid_Intergral|ChangingIntegrationRate|
              Derivative_On_Measurement|DerivativeFilter);
    PID_Init(&motor->M6020_lk[YAW].PID_S, 2000.0f, 50.0f, 
              PID_S_Yaw, 1000.0f, 1000.0f, 
              0.7f, 0.7f, 2, 
              Integral_Limit|OutputFilter|ErrorHandle|
              Trapezoid_Intergral|ChangingIntegrationRate|
              Derivative_On_Measurement|DerivativeFilter);
    return 0;
}

uint8_t Gimbal_AIM_INIT()
{

    return 0;
}
#define LK_MOTOR 1
// #define IDENTIFY_TEST 1
#define LQR_CONTROL 1

uint8_t gimbal_task(CONTAL_Typedef *CONTAL, MOTOR_Typedef *MOTOR)
{
    MOTOR->M6020_lk[PITCH].DATA.Aim = CONTAL->HEAD.Pitch;
    MOTOR->M6020_lk[YAW].DATA.Aim   = CONTAL->HEAD.Yaw;

    #ifdef IDENTIFY_TEST
    float yaw_chirp = MotorIdentify_Chirp(&yaw_id);
    float pitch_chirp = MotorIdentify_Chirp(&pitch_id);
    #endif

    #ifdef LK_MOTOR
    // LKMF_Data_Read(&hcan,2);
    // LKMF_Data_Read(&hcan,1);
    #endif

    PID_Calculate(&MOTOR->M6020_lk[PITCH].PID_P, 
                   MOTOR->M6020_lk[PITCH].DATA.Angle_Infinite,
                   MOTOR->M6020_lk[PITCH].DATA.Aim);
    PID_Calculate(&MOTOR->M6020_lk[PITCH].PID_S, 
                   MOTOR->M6020_lk[PITCH].DATA.Speed_now,
                   MOTOR->M6020_lk[PITCH].PID_P.Output);

    PID_Calculate(&MOTOR->M6020_lk[YAW].PID_P,
                   MOTOR->M6020_lk[YAW].DATA.Angle_Infinite,
                   MOTOR->M6020_lk[YAW].DATA.Aim);
    PID_Calculate(&MOTOR->M6020_lk[YAW].PID_S,
                   MOTOR->M6020_lk[YAW].DATA.Speed_now,
                   MOTOR->M6020_lk[YAW].PID_P.Output);
    #ifndef LK_MOTOR
    // DJI_Current_Ctrl(&hcan, 0x1ff, 
    //                  (int16_t)MOTOR->M6020[YAW].PID_S.Output,
    //                  (int16_t)MOTOR->M6020[PITCH].PID_S.Output,
    //                  0,
    //                  0);
    #endif
    

    #ifdef LK_MOTOR
    // LKMF_iq_ctrl(&hcan,1,MOTOR->M6020_lk[YAW].PID_S.Output);
    LKMF_iq_ctrl(&hcan,2,MOTOR->M6020_lk[PITCH].PID_S.Output);
    
    // LKMF_iq_ctrl(&hcan,1,0);
    // LKMF_iq_ctrl(&hcan,2,0);
    #endif

    #ifdef IDENTIFY_TEST
    MotorIdentify_Update(
       &yaw_id,
       yaw_chirp,
       MOTOR->M6020_lk[YAW].DATA.Angle_Infinite
    );
    MotorIdentify_Update(
       &pitch_id,
       pitch_chirp,
       MOTOR->M6020_lk[PITCH].DATA.Angle_Infinite
    );
    LKMF_iq_ctrl(&hcan,1,( int16_t)yaw_chirp);
    LKMF_iq_ctrl(&hcan,2,(int16_t) 0);
    VOFA_justfloat(yaw_id.phi[0], yaw_id.phi[1], yaw_id.phi[2], yaw_id.phi[3], 
                    yaw_id.y[0],0,0,0,0,0);
    #endif

    #ifdef LQR_CONTROL
    float yaw_out;

    yaw_out = LQR_Update(
        &yaw_lqr,
        MOTOR->M6020_lk[YAW].DATA.Aim,
        MOTOR->M6020_lk[YAW].DATA.Angle_Infinite,
        MOTOR->M6020_lk[YAW].DATA.Speed_now / 6.0f
    );

    LKMF_iq_ctrl(
        &hcan,
        1,
        yaw_out
    );
    #endif
    return 0;
}



