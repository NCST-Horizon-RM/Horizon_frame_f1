#ifndef __ALL_INIT_H
#define __ALL_INIT_H

#include "main.h"
#include "tim.h"
#include "usart.h"

#include "RUI_DBUS.h"
#include "Vision.h"
#include "DJI_Motor.h"
#include "bsp_dwt.h"
#include "can_bsp.h"
#include "Gimbal_Task.h"
#include "LK_Motor.h"
#include "MotorIdentify.h"
#include "lqr_controller.h"

extern DBUS_Typedef DBUS_V_DATA;
extern DBUS_UNION_Typdef DBUS_V_UNION;
extern MOTOR_Typedef ALL_MOTOR;
extern VisionRxDataUnion VisionRxData;
extern CONTAL_Typedef ALL_CONTAL;
extern MotorIdentify_t yaw_id;
extern MotorIdentify_t pitch_id;
extern LQR_t yaw_lqr;
extern LQR_t pitch_lqr;

void All_Init(void);

#endif // !__ALL_INIT_H
