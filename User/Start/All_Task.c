#include "All_Task.h"
#include "main.h"
#include "cmsis_os.h"
#include "VOFA.h"
#include "All_init.h"
#include "can_bsp.h"
#include "Robot.h"
#include "bsp_dwt.h"

void StartDefaultTask(void)
{
    All_Init();
    for(;;)
    {
        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_12);
        osDelay(500);
    }
}

void StartGimbalTask(void)
{
    for(;;)
    {
        ALL_CONTAL.DWT_TIME.Gimbal_dt = DWT_GetDeltaT(&ALL_CONTAL.DWT_TIME.Gimbal_Count);
        ALL_CONTAL.DWT_TIME.Gimbal_time = DWT_GetTimeline_ms();
        // VOFA_justfloat((float)DBUS_V_DATA.Remote.CH0_int16, 
        //                 (float)ALL_MOTOR.M6020[YAW].DATA.Angle_Infinite,
        //                 (float)ALL_MOTOR.M6020[PITCH].DATA.Angle_Infinite,
        //                 (float)ALL_MOTOR.M6020[PITCH].DATA.Speed_now/60.0f*360.0f,
        //                 (float)ALL_MOTOR.M6020[YAW].DATA.Speed_now/60.0f*360.0f,
        //                 ALL_CONTAL.DWT_TIME.Gimbal_dt,
        //                 ALL_CONTAL.DWT_TIME.Monitor_dt,
        //                 (float) VisionRxData.Data.isOnline,
        //                 0,
        //                 ALL_CONTAL.DWT_TIME.Gimbal_time);
        VOFA_justfloat((float)DBUS_V_DATA.Remote.CH0_int16, 
                        (float)ALL_MOTOR.M6020[YAW].DATA.Angle_Infinite / 8192.0f * 360.0f,
                        (float)ALL_MOTOR.M6020[PITCH].DATA.Angle_Infinite / 8192.0f * 360.0f,
                        (float)ALL_MOTOR.M6020[PITCH].DATA.Speed_now/60.0f*360.0f,
                        (float)ALL_MOTOR.M6020[YAW].DATA.Speed_now/60.0f*360.0f,
                        (float)VisionRxData.Data.PitchAngle,
                        (float)VisionRxData.Data.YawAngle,
                        ALL_CONTAL.DWT_TIME.Monitor_dt,
                        (float) VisionRxData.Data.isOnline,
                        0);
        Vision_Tx_Data(&ALL_MOTOR);
        gimbal_task(&ALL_CONTAL, &ALL_MOTOR);
        RobotTask(3, &DBUS_V_DATA, &ALL_CONTAL);  // 3 自瞄 2 遥控
        osDelay(1);
    }
}

void StartMonitorTask(void)
{
    //
    for(;;)
    {
        ALL_CONTAL.DWT_TIME.Monitor_dt = DWT_GetDeltaT(&ALL_CONTAL.DWT_TIME.Monitor_Count);
        // RobotTask(3, &DBUS_V_DATA, &ALL_CONTAL);  // 3 自瞄 2 遥控
        DBUS_OFFLINE_Check(&DBUS_V_DATA);           // can发送在这里，应该写到task里的，懒了
        Vision_Monitor(&VisionRxData);
        osDelay(1);
    }
}
