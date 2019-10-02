/*
 * PID.c
 *
 *  Created on: Mar 12, 2018
 *      Author: cc-isn
 */

#include "PID.h"
#include "IMU.h"


//void PID_Init(PID_data_t *pidData_R, PID_data_t *pidData_L){
//	/*RIGHT Control Variables*/
//	  pidData_R->signals.w = 0;
//
//	  pidData_R->parameter.K= 5;//0.015;//0.002;//0.2
//	  pidData_R->parameter.Ti 	= 1;//0.0001
//	  pidData_R->parameter.Td 	= 10;//2.5//5000;//5000000
//	  pidData_R->parameter.Tt 	= 0.5;
//	  pidData_R->parameter.N 	= 0.005;
//	  pidData_R->parameter.b 	= 1;
//	  pidData_R->parameter.h 	= SAMPLETIME;
//	  pidData_R->parameter.ulow 	= -10000;
//	  pidData_R->parameter.uhigh = 10000;
//	  pidData_R->parameter.ARW = TRUE;
//	  pidData_R->states.v_motor_right = 1.5;
//	  pidData_R->states.v_motor_left = 1.5;
//	  pidData_R->states.arw = 500;
//
//	  PID_addDerivatedParameters(pidData_R);
//
//
//
//	  /*LEFT Control Variables*/
//	  pidData_L->signals.w = 0;
//
//	  pidData_L->parameter.K		= 0.005;
//	  pidData_L->parameter.Ti 	= 1000000;
//	  pidData_L->parameter.Td 	= 0.001;
//	  pidData_L->parameter.Tt 	= 0.5;
//	  pidData_L->parameter.N 	= 1;
//	  pidData_L->parameter.b 	= 1;
//	  pidData_L->parameter.h 	= SAMPLETIME;
//	  pidData_L->parameter.ulow 	= -2000;
//	  pidData_L->parameter.uhigh = 2000;
//	  pidData_L->parameter.ARW = TRUE;
//	  pidData_L->states.v_motor_left = 1.5;
//
//	  PID_addDerivatedParameters(pidData_L);
//}

void PID_CalculateOutput(PID_data_t *data){

	/*PROPORTIONAL*/
	data->states.P =
			data->parameter.K *
			(data->parameter.b * data->signals.w - data->signals.y);

	/*DERIVATIVE*/
	data->states.D =
			data->parameter.ad * data->states.D -
			data->parameter.bd * (data->signals.y - data->states.yold);

	/*CONTROL SIGNAL*/

	if(data->states.I > data->states.arw){
		data->states.I = 0;
	}
	else if (data->states.I < -data->states.arw){
		data->states.I = 0;
	}

	data->signals.v = data->states.P + data->states.D; //+ data->states.I;// + data->states.D;//

	/*Anti Reset Windup*/
//	if(data->parameter.ARW){
//		data->states.e_arw = data->signals.u - data->signals.v;
//		data->states.arw = data->states.arw + data->states.e_arw - data->parameter.ar;
//	}
//	else{
//		data->states.arw = 0;
//	}




	/*ACTUATOR LIMITATIONS*/
	if(data->signals.v < data->parameter.ulow){
		/*Below low limit*/
		data->signals.u = data->parameter.ulow;
	} else if (data->signals.v > data->parameter.uhigh){
		/*Above high limit*/
		data->signals.u = data->parameter.uhigh;
	}else{
		/*In between limits*/
		data->signals.u = data->signals.v;
	}



	/*UPDATE STATES*/
	/*INTEGRAL*/
	data->states.I =
			data->states.I +
			data->parameter.bi* (data->signals.w - data->signals.y); +
			data->parameter.ar* (data->signals.u - data->signals.v);

//			data->states.I =  data->states.arw + data->states.I;
	/*MEASURED VARIABLE*/
	data->states.yold = data->signals.y;
}

void PID_addDerivatedParameters(PID_data_t *data){
	data->parameter.bi =
			data->parameter.K * data->parameter.h /
			data->parameter.Ti;

	data->parameter.ar =
			data->parameter.h /
			data->parameter.Tt;

	data->parameter.bd =
			data->parameter.K * data->parameter.N * data->parameter.Td /
			(data->parameter.Td + data->parameter.N * data->parameter.h);

	data->parameter.ad =
			data->parameter.Td /
			(data->parameter.Td + data->parameter.N *  data->parameter.h);

	data->states.P = 0;
	data->states.I = 0;
	data->states.D = 0;
	data->states.yold = 0;
}




//uint8_t PID_ParseCommand(const unsigned char *cmd, bool *handled, const CLS1_StdIOType *io){
//	  uint8_t res = ERR_OK;
//	  int32_t val;
//	  const unsigned char *p;
//
//
//	  return 0;
//}
