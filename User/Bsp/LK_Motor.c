#include "LK_Motor.h"


void LKMF_iq_ctrl(CAN_HandleTypeDef* hcan, uint16_t motor_id, int32_t iqControl)
{
	CAN_TxHeaderTypeDef tx;
	uint32_t mailbox = 0;
	uint8_t data[8];
		
	
	tx.StdId = 0x140 + motor_id;
	tx.IDE = CAN_ID_STD;
	tx.RTR = CAN_RTR_DATA;
	tx.DLC = 0x08;
	
	data[0] = 0xA1;
	data[1] = 0x00;
	data[2] = 0x00;
	data[3] = 0x00;
	data[4] = *(uint8_t *)(&iqControl);
	data[5] = *((uint8_t *)(&iqControl)+1);
	data[6] = 0x00;
	data[7] = 0x00;
	
	 HAL_CAN_AddTxMessage(hcan, &tx, data, &mailbox);
}

void LKMF_Data_Read(CAN_HandleTypeDef* hcan, uint16_t motor_id)
{
	CAN_TxHeaderTypeDef tx;
	uint32_t mailbox = 0;
	uint8_t data[8];	
	
	tx.StdId = 0x140 + motor_id;
	tx.IDE = CAN_ID_STD;
	tx.RTR = CAN_RTR_DATA;
	tx.DLC = 0x08;
	
	data[0] = 0x9c;
	data[1] = 0x00;
	data[2] = 0x00;
	data[3] = 0x00;
	data[4] = 0x00;
	data[5] = 0x00;
	data[6] = 0x00;
	data[7] = 0x00;
	
	 HAL_CAN_AddTxMessage(hcan, &tx, data, &mailbox);
}
void LK_MotorResolve(DJI_MOTOR_Typedef *motor,uint8_t *RxMessage)
{

		motor->DATA.temperature = RxMessage[1];
	
		motor->DATA.current = ((uint16_t)RxMessage[3] << 8 | (uint16_t)RxMessage[2]);
	
		motor->DATA.Speed_last = motor->DATA.Speed_now;
		motor->DATA.Speed_now     = ((uint16_t)RxMessage[5] << 8 | (uint16_t)RxMessage[4]);
		
		motor->DATA.Angle_last = motor->DATA.Angle_now;
		motor->DATA.Angle_now     = ((uint16_t)RxMessage[7] << 8 | (uint16_t)RxMessage[6]);
		
//		if(motor->DATA.State)
//		{
			if (motor->DATA.Angle_now - motor->DATA.Angle_last < -40000) 
			{
				motor->DATA.Laps++;
			}
			else if (motor->DATA.Angle_now - motor->DATA.Angle_last > 40000) 
			{
				motor->DATA.Laps--;
			}
//			motor->data.lastConEncode = motor->data.conEncode;
//			motor->data.conEncode     = (float) motor->DATA.Laps* 360 + (float) motor->data.rawEncode * 360 / 65536;	
//		}
//		else
//		{
//			motor->data.conEncode=(float)motor->data.rawEncode * 360 / 65536;	
//		} 
	
}