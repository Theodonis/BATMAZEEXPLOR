/*
 * Motor.c
 *
 *  Created on: 08.03.2019
 *      Author: cc-isn
 */


#include "Motor.h"
#include "DRV1_FAULT.h"
#include "DRV2_FAULT.h"
#include "DRV1_EN.h"
#include "DRV2_EN.h"
#include "DRV1_PH.h"
#include "DRV2_PH.h"
#include "DRV1_PMODE.h"
#include "DRV2_PMODE.h"
#include "I_Motor_L.h"
#include "I_Motor_R.h"
#include "SLEEP_L.h"
#include "SLEEP_R.h"
#include "WAIT1.h"

Motor_t motor;

void initMotors(void){
	SLEEP_R_ClrVal();
	DRV1_PMODE_ClrVal();
	WAIT1_Waitms(10);
	SLEEP_R_SetVal();
	DRV1_EN_SetVal();
	DRV1_PH_SetVal();

	SLEEP_L_ClrVal();
	DRV2_PMODE_ClrVal();
	WAIT1_Waitms(10);
	SLEEP_L_SetVal();
	DRV2_EN_SetVal();
	DRV2_PH_SetVal();
}

void deinitMotors(void){

	DRV1_EN_ClrVal();
	DRV1_PH_ClrVal();


	DRV2_EN_ClrVal();
	DRV2_PH_ClrVal();
}

void setAllParameter(Motor_t parameter){
	DRV1_EN_PutVal(parameter.right.DRV1_IN1);
	DRV1_PH_PutVal(parameter.right.DRV1_IN2);
	DRV1_PMODE_PutVal(parameter.right.DRV1_PMODE);
	I_Motor_R_SetValue(&parameter.right.DRV1_VREF);

	DRV2_EN_PutVal(parameter.left.DRV2_IN1);
	DRV2_PH_PutVal(parameter.left.DRV2_IN2);
	DRV2_PMODE_PutVal(parameter.left.DRV2_PMODE);
	I_Motor_L_SetValue(&parameter.left.DRV2_VREF);
}

void MotorEnable(bool motor_Right,bool motor_Left){
	motor.right.DRV1_IN1 = motor_Right;
	motor.left.DRV2_IN1 = motor_Left;
	DRV1_EN_PutVal(motor.right.DRV1_IN1);
	DRV2_EN_PutVal(motor.left.DRV2_IN1);

}


void MotorDirection(bool motor_Right,bool motor_Left){
	motor.right.DRV1_IN2 = motor_Right;
	motor.left.DRV2_IN2 = motor_Left;
	DRV1_PH_PutVal(motor.right.DRV1_IN2);
	DRV2_PH_PutVal(motor.left.DRV2_IN2);
}

void MotorMode(bool motor_Right,bool motor_Left){
	motor.right.DRV1_PMODE = motor_Right;
	motor.left.DRV2_PMODE = motor_Left;
	DRV1_PMODE_PutVal(motor.right.DRV1_PMODE);
	DRV2_PMODE_PutVal(motor.left.DRV2_PMODE);
}

void set_IPROI(uint16_t i_Right, uint16_t i_Left){
	motor.right.DRV1_IPROI = i_Right;
	motor.left.DRV2_IPROI = i_Left;
}

void set_VREF(float vREF_Right, float vREF_Left){
	if(vREF_Right<0){
		if(vREF_Right<-3.3){
			vREF_Right = -1*-3.3;
		}
		else{
			vREF_Right = -1*vREF_Right;
		}
		DRV1_PH_ClrVal();
	}
	else{
		DRV1_PH_SetVal();
	}


	if(vREF_Left<0){
		if(vREF_Left<-3.3){
			vREF_Left = -1*-3.3;
		}
		else{
			vREF_Left = -1*vREF_Left;
		}
		DRV2_PH_SetVal();
	}
	else{
		DRV2_PH_ClrVal();
	}


	if(vREF_Right>3.3){
		vREF_Right = 3.3;
	}
	if(vREF_Left>3.3){
		vREF_Left = 3.3;
	}

		motor.right.DRV1_VREF = (uint16_t)(vREF_Right * Current_TO_DUTY_INTEGER); //version 3: wires connected the other way around!!!!
		motor.left.DRV2_VREF = (uint16_t)(vREF_Left * Current_TO_DUTY_INTEGER);
		I_Motor_R_SetValue(&motor.right.DRV1_VREF);
		I_Motor_L_SetValue(&motor.left.DRV2_VREF);
}



Motor_t get_MotorStruct(void){
	return motor;
}
