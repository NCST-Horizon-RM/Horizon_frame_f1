#include "All_Init.h"

//遥控相关变量
DBUS_UNION_Typdef DBUS_V_UNION = {0};
DBUS_Typedef DBUS_V_DATA = { 0 };

//电机
MOTOR_Typedef ALL_MOTOR;

//视觉
VisionRxDataUnion VisionRxData = {0};

CONTAL_Typedef ALL_CONTAL = {0};

MotorIdentify_t yaw_id = {0};
MotorIdentify_t pitch_id = {0};

LQR_t yaw_lqr = {0};
LQR_t pitch_lqr = {0};

void All_Init(void)
{   
    DWT_Init(72);
    CAN_Filter_Init();

    HAL_TIM_Base_Start_IT(&htim2);

	__HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);//遥控串口
    HAL_UART_Receive_DMA(&huart3, (uint8_t *)DBUS_V_UNION.GetData, 19);

    __HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);//视觉串口
    HAL_UART_Receive_DMA(&huart2, (uint8_t *)VisionRxData.OriginData, sizeof(VisionRxData.OriginData));

    ALL_CONTAL.HEAD.Pitch_MAX =  800.0f;
    ALL_CONTAL.HEAD.Pitch_MIN = -800.0f;
    ALL_CONTAL.HEAD.Yaw_Init  = (float)ALL_MOTOR.M6020[YAW].DATA.Angle_Infinite;
    MOTOR_PID_Gimbal_INIT(&ALL_MOTOR);

    MotorIdentify_Init(&yaw_id, 0.001f);
    MotorIdentify_Init(&pitch_id, 0.001f);
    MotorIdentify_SetChirp(&yaw_id, 2500.0f, 1.5f, 1.5f, 20.0f);
    MotorIdentify_SetChirp(&pitch_id, 1250.0f, 1.5f, 1.5f, 20.0f);
    LQR_Init(
        &yaw_lqr,
          4.96509592, -1.64296105, -0.51295266,
        10000.0f
    );
}