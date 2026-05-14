#include "All_Init.h"
#include "cmsis_os.h"

//遥控相关变量
DBUS_UNION_Typdef DBUS_V_UNION = {0};
DBUS_Typedef DBUS_V_DATA = { 0 };

//电机
MOTOR_Typedef ALL_MOTOR;

//视觉
VisionRxDataUnion VisionRxData = {0};

CONTAL_Typedef ALL_CONTAL = {0};

MotorIdentify_t yaw_id;
MotorIdentify_t pitch_id;

void All_Init(void)
{   
    DWT_Init(72);
    CAN_Filter_Init();

    HAL_TIM_Base_Start_IT(&htim2);

	__HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);//遥控串口
    HAL_UART_Receive_DMA(&huart3, (uint8_t *)DBUS_V_UNION.GetData, 19);

    __HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);//视觉串口
    HAL_UART_Receive_DMA(&huart2, (uint8_t *)VisionRxData.OriginData, sizeof(VisionRxData.OriginData));

    LKMF_iq_ctrl(&hcan,1,0);
    LKMF_iq_ctrl(&hcan,2,0);
    // while (ALL_MOTOR.M6020_lk[PITCH].DATA.Angle_now == 0) {osDelay(1);}
    osDelay(100);
    ALL_CONTAL.HEAD.Pitch_MAX =  30.0f;
    ALL_CONTAL.HEAD.Pitch_MIN = -30.0f;
    ALL_CONTAL.HEAD.Yaw_Init  = (float)ALL_MOTOR.M6020_lk[YAW].DATA.Angle_Infinite;
    ALL_MOTOR.M6020_lk[PITCH].DATA.Angle_init = -18057;
    
    if (ALL_MOTOR.M6020_lk[PITCH].DATA.Angle_now < 32767 && ALL_MOTOR.M6020_lk[PITCH].DATA.Angle_now > (32768-18057))
    {
        ALL_MOTOR.M6020_lk[PITCH].DATA.Laps = -1;
    }
    
    ALL_MOTOR.M6020_lk[YAW].DATA.Angle_init = -22442;
    
    MOTOR_PID_Gimbal_INIT(&ALL_MOTOR);

    MotorIdentify_Init(&yaw_id, 0.001f);
    MotorIdentify_Init(&pitch_id, 0.001f);
    MotorIdentify_SetChirp(&yaw_id, 120.0f, 0.5f, 3.0f, 20.0f);
    MotorIdentify_SetChirp(&pitch_id, 80.0f, 0.5f, 6.0f, 20.0f);
}