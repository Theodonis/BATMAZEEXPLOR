
#include "Application.h"
#include "Motor.h"
#include "PID.h"
#include "FSM.h"
#include "ADC.h"
#include "IMU.h"
#include "Encoder.h"
#include "WAIT1.h"
#include <stdlib.h>

 PID_data_t pidData_R, pidData_L, pidData_UC;
 ADC_data_t adcData;
 Encoder_data_t encData;
 IMU_data_t imuData;
 uint16_t detect_counter;
 bool newField_Flag;
 float angle;
 void PID_Init(void){

 	/*RIGHT Control Variables*/
 	  pidData_R.signals.w = 0;

 	  pidData_R.parameter.K		= 0.1;//0.00005;//0.015;//0.002;//0.2
 	  pidData_R.parameter.Ti 	= 1;//0.0001
 	  pidData_R.parameter.Td 	= 1000000;//2.5//5000;//5000000
 	  pidData_R.parameter.Tt 	= 1;
 	  pidData_R.parameter.N 	= 1;
 	  pidData_R.parameter.b 	= 1;
 	  pidData_R.parameter.h 	= 0.5;//ms
 	  pidData_R.parameter.ulow 	= -3.25;
 	  pidData_R.parameter.uhigh = 3.25;
 	  pidData_R.parameter.ARW = TRUE;
 	  pidData_R.states.v_motor_right = 0.4;
 	  pidData_R.states.arw = 500;

 	  PID_addDerivatedParameters(&pidData_R);



 	  /*LEFT Control Variables*/
 	  pidData_L.signals.w = 7015;

 	  pidData_L.parameter.K		= 0.005;
 	  pidData_L.parameter.Ti 	= 1000000;
 	  pidData_L.parameter.Td 	= 0.001;
 	  pidData_L.parameter.Tt 	= 0.5;
 	  pidData_L.parameter.N 	= 1;
 	  pidData_L.parameter.b 	= 1;
 	  pidData_L.parameter.h 	= 0.0005;
 	  pidData_L.parameter.ulow 	= -3.3;
 	  pidData_L.parameter.uhigh = 3.3;
 	  pidData_L.parameter.ARW = TRUE;
 	  pidData_L.states.v_motor_left = 0.5;

 	  PID_addDerivatedParameters(&pidData_L);
 }

 void drive()
 {
 	set_VREF(pidData_R.states.v_motor_right,0.4);
 }

 void stopp()
 {
	 set_VREF(0,0);
 }
 void turn_left(){
	 stopp();
	 calcIMU_data(&imuData);
 	angle = imuData.unit_Values.z_Axis_Gyro_angle;
 	set_VREF(1,-1);
 	while((imuData.unit_Values.z_Axis_Gyro_angle-angle) > -90){
 		calcIMU_data(&imuData);
 		WAIT1_Waitus(250);
 	}
 	stopp();
 	WAIT1_Waitms(1);
 }

 void turn_right()
 {
	 stopp();
	 calcIMU_data(&imuData);
	 	angle = imuData.unit_Values.z_Axis_Gyro_angle;
	 	set_VREF(-1.1,1.1);
 	while((imuData.unit_Values.z_Axis_Gyro_angle-angle) < 90){
 		calcIMU_data(&imuData);
 		WAIT1_Waitus(250);
 	}
     stopp();
     WAIT1_Waitms(10);

 }

 void driveToTurn(float dist)
 {
 	//WAIT1_Waitms(1);
	  calcENC_data(&encData);
 	  float posMeter = (encData.uint_Values.m_posL + encData.uint_Values.m_posR)*0.5;
	  FTM1_CNT =0;
	  FTM2_CNT =0;
 	 set_VREF(0.5,0.5);
 	  while((((encData.uint_Values.m_posL + encData.uint_Values.m_posR)*0.5) - posMeter)<dist){
 		  calcENC_data(&encData);

 	  }
 	  stopp();

 }

 void turn_l(void){
 	  turn_left();
 	  pidData_R.states.v_motor_right = 0.4;
 }

 void turn_r(uint8_t state){

 	switch(state) {
 		case 1:
 		  	  //driveToTurn(0.05);
 			  turn_right();
 			  stopp();
 			  pidData_R.states.v_motor_right = 0.4;

 		break;

 		case 2:
 				driveToTurn(0.09);
 			  turn_right();
 		  	  driveToTurn(0.09);
 			  pidData_R.states.v_motor_right = 0.4;

 		break;
 		default: stopp(); break;
 	}

 }


 /**
  * @brief  Read data from LSM6DSL Gyroscope
  * @param  pData the pointer where the gyroscope data are stored
  * @retval LSM6DSL_STATUS_OK in case of success, an error code otherwise
  */
 void getSensors(void)
 {
	  	// stateMachine();
		calcADC_data(&adcData);
		calcENC_data(&encData);
		calcIMU_data(&imuData);

 		if(detect_counter == 0){
 			adcData.raw_Values.raw_Right_old =adcData.raw_Values.raw_Right;
 		}
 		detect_counter++;

 		if(detect_counter == 10){
 			if(adcData.raw_Values.raw_Right_old!= 0 && ((abs(adcData.raw_Values.raw_Right-adcData.raw_Values.raw_Right_old) > 300) && (abs(adcData.raw_Values.raw_Right-adcData.raw_Values.raw_Right_old) < 3000))){
 				EVNT_SetEvent(EVNT_New_Field);
 				newField_Flag = TRUE;
 			}
 			else{
 				detect_counter = 0;
 			}

 		}

 		if(!newField_Flag){
 			if(adcData.mm_Values.mm_Right>80){
 				EVNT_SetEvent(EVNT_Turn_Right);
// 				set_VREF(-2,-2);
// 				WAIT1_Waitms(10);
// 				set_VREF(0,0);
 			}
 			else if(adcData.mm_Values.mm_MiddleL<90){
 				EVNT_SetEvent(EVNT_Turn_Left);
// 				set_VREF(-2,-2);
// 				WAIT1_Waitms(10);
// 				set_VREF(0,0);
 			}
 		}
 		newField_Flag = FALSE;
 }



void Run_PID(void){
	  int8_t diffPosL;
	  int8_t diffPosR;
	  double errorPos;
	  double errorDist;

		errorDist = (double)(55-adcData.mm_Values.mm_Right);///(60-adcData.mm_Values.mm_Right);//(adcData.mm_Values.mm_Left-adcData.mm_Values.mm_Right);
		errorPos = 0;//(double)(-1*((posR-posR_old)+(posL-posL_old)));
		pidData_R.signals.y = (20*errorPos)+(errorDist);//+errorDist; 20, 0.001--> Gewichtung Distaz
		PID_CalculateOutput(&pidData_R);

		if((pidData_R.states.v_motor_right+(-1*pidData_R.signals.u))>0.65){
			pidData_R.states.v_motor_right = 0.65;
		}
		else if((pidData_R.states.v_motor_right+(-1*pidData_R.signals.u))<(0.15)){
			pidData_R.states.v_motor_right = 0.15;
		}
		else {
			pidData_R.states.v_motor_right += (-1*pidData_R.signals.u);
		}
		drive();
}


