/*
 * Motor.h
 *
 *  Created on: 08.03.2019
 *      Author: cc-isn
 */

#ifndef SOURCES_MOTOR_H_
#define SOURCES_MOTOR_H_
#include <PE_Types.h>

#define Current_to_Vout 	2.4f //Ripropi/Aipropi --> 2400Ohm / 1000ua/A
#define Current_TO_DUTY_INTEGER								(1241)// 4096/3.3V
typedef struct Motor_{

	struct{
		bool  DRV1_FAULT;	/* Input	: */
		bool  DRV1_IN1;	/* Output ENABLE	EN:  */
		bool  DRV1_IN2;	/* Output DIRECTION	PH: */
		bool  DRV1_PMODE;	/* Output LOW for EN/PH MODE | High for PWM Mode	: */
		uint16_t DRV1_IPROI;	/* Input	: */
		uint16_t DRV1_VREF;	/* Output	: */
	}right;

	struct{
		bool  DRV2_FAULT;	/* Input	: */
		bool  DRV2_IN1;	/* Output ENABLE	EN:  */
		bool  DRV2_IN2;	/* Output DIRECTION	PH: */
		bool  DRV2_PMODE;	/* Output LOW for EN/PH MODE | High for PWM Mode	: */
		uint16_t DRV2_IPROI;	/* Input	: */
		uint16_t DRV2_VREF;	/* Output	: */
	}left;

}Motor_t;

void initMotors(void);
void deinitMotors(void);
void setAllParameter(Motor_t parameter);
void MotorDirection(bool motor_Right,bool motor_Left);
void MotorEnable(bool motor_Right,bool motor_Left);
void MotorMode(bool motor_Right,bool motor_Left);
void set_IPROI(uint16_t i_Right, uint16_t i_Left);
void set_VREF(float vREF_Right, float vREF_Left);
Motor_t get_MotorStruct(void);



#endif /* SOURCES_MOTOR_H_ */
