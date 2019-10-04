/*
 * Logging.c
 *
 *  Created on: 17.01.2019
 *      Author: cc-isn
 */

#include "Logging.h"
#include "UTIL1.h"
#include "AS_BLE.h"
#include "WAIT1.h"
#include "ADC.h"
#include "I_LED_L.h"
#include "I_LED_R.h"


//#include "QuadSmaple.h"
//#include "Distance_INT.h"

IMU_data_t iimuData;
ADC_data_t aadcData;
Encoder_data_t eencData;
int16_t ccounter;
uint16_t speedR;
uint16_t speedL;

float raw_dataFloat[14][LOGGING_LENGTH];
//float raw_dataFloat[11][LOGGING_LENGTH];
//float raw_dataAcc[1][LOGGING_LENGTH];
//uint16_t raw_dataInt[2][LOGGING_LENGTH];
//int16_t raw_dataSen[4][7500];
//nt16_t raw_dataSen[1][6100];
//int16_t raw_dataSen[3][10000];
//uint8_t lookUP[32768];
uint16_t ji;
uint8_t overstepflag;

static void putStream(uint8_t *cmd) {
  uint16_t snt;
  AS_BLE_SendBlock(cmd, strlen((char*)cmd), &snt);
}

#if 0
void saveControllerData(float v_ref[2],float v_est[2],float U_mot[2],float M[2])
{

						//FC1_Reset();
	overstepflag++;
						if(ji < 1900){// && overstepflag >=2 ){
						raw_dataFloat[0][ji] = v_ref[0];
						raw_dataFloat[1][ji] = v_ref[1];
						raw_dataFloat[2][ji] = v_est[0];
						raw_dataFloat[3][ji] = v_est[1];
						raw_dataFloat[4][ji] = U_mot[0];
						raw_dataFloat[5][ji] = U_mot[1];
						raw_dataFloat[6][ji] = M[0];
						raw_dataFloat[7][ji] = M[1];
						ji++;
						overstepflag = 0;
						}

}

void printSaveControllerData(void)
{
	uint8_t ccc[4];
	QuadSmaple_DisableEvent();
	ccc[0] = '\0';
	UTIL1_chcat(ccc, sizeof(ccc), '\n');
	uint16_t j = 0;
	  for(j; j<ji; j++){
		   uint8_t testt[147];
			testt[0] = '\0';
			UTIL1_chcat(testt, sizeof(testt), '\n');

		  			  		  UTIL1_strcatNumFloat(testt, sizeof(testt),(raw_dataFloat[0][j]),8); //enc right
		  			  		  UTIL1_strcat(testt, sizeof(testt)," ");
		  			  		  UTIL1_chcat(testt, sizeof(testt), '\t');

		  			  		  UTIL1_strcatNumFloat(testt, sizeof(testt),(raw_dataFloat[1][j]),8); //enc left
		  			  		  UTIL1_strcat(testt, sizeof(testt)," ");
		  			  		  UTIL1_chcat(testt, sizeof(testt), '\t');

		  			  		  UTIL1_strcatNumFloat(testt, sizeof(testt),(raw_dataFloat[2][j]),8); //gyro dps
		  			  		  UTIL1_strcat(testt, sizeof(testt)," ");
		  			  		  UTIL1_chcat(testt, sizeof(testt), '\t');

		  			  		  UTIL1_strcatNumFloat(testt, sizeof(testt),(raw_dataFloat[3][j]),8); //acc m/s2
		  			  		  UTIL1_strcat(testt, sizeof(testt)," ");
		  			  		  UTIL1_chcat(testt, sizeof(testt), '\t');

		  			  		  UTIL1_strcatNumFloat(testt, sizeof(testt),(raw_dataFloat[4][j]),8); //enc right
		  			  		  UTIL1_strcat(testt, sizeof(testt)," ");
		  			  		  UTIL1_chcat(testt, sizeof(testt), '\t');

		  			  		  UTIL1_strcatNumFloat(testt, sizeof(testt),(raw_dataFloat[5][j]),8); //enc left
		  			  		  UTIL1_strcat(testt, sizeof(testt)," ");
		  			  		  UTIL1_chcat(testt, sizeof(testt), '\t');

		  			  		  UTIL1_strcatNumFloat(testt, sizeof(testt),(raw_dataFloat[6][j]),8); //gyro dps
		  			  		  UTIL1_strcat(testt, sizeof(testt)," ");
		  			  		  UTIL1_chcat(testt, sizeof(testt), '\t');

		  			  		  UTIL1_strcatNumFloat(testt, sizeof(testt),(raw_dataFloat[7][j]),8); //acc m/s2
		  			  		  UTIL1_strcat(testt, sizeof(testt)," ");
		  			  		  UTIL1_chcat(testt, sizeof(testt), '\t');



		  putStream((uint8_t*)testt);
		  memset(&testt[0], 0, sizeof(testt));
		  WAIT1_Waitms(20);
	  }
}


#endif

#if 1
void saveData(float *wallCenterDivergence, float weightDistanceSensor, float v_r[2],  float v_est[2], float q_r[3], float q[3],
		Distance_sensors_HP_filtered *distance_HP, float x_IMU_dot_test, float x_enc_testfloat, float I_mot_ist[2],
	Distance_sensors_LP_filtered *distanceLP, float d_hp_filtered, bool SegmentFinished,
	float gyroXY[2], Distance_Bandpass_t *distanceBandpass, Wall_availability_state *wallState,
	float vc_logging[2], float I_mot[2],float u_bat_test){
	if(ji < LOGGING_LENGTH){
		raw_dataFloat[0][ji] = 0;//v_r[0];
		raw_dataFloat[1][ji] = 0;//v_r[1];
		raw_dataFloat[2][ji] = 0;//v_est[0];
		raw_dataFloat[3][ji] = 0;//v_est[1];
//		raw_dataFloat[2][ji] = vc_logging[0];
//		raw_dataFloat[3][ji] = vc_logging[1];
		raw_dataFloat[4][ji] = 0;//q_r[0];
		raw_dataFloat[5][ji] = 0;//q_r[1];
		raw_dataFloat[6][ji] = 0;//I_mot_ist[0];
		raw_dataFloat[7][ji] = 0;//I_mot_ist[1];
		raw_dataFloat[8][ji] = 0;//u_bat_test;
		raw_dataFloat[8][ji] = 0;//q[1];
		raw_dataFloat[9][ji] = 0;//q[2];
//		raw_dataFloat[4][ji] = I_mot[0];
//		raw_dataFloat[5][ji] = I_mot[1];

		/* Walls */
		raw_dataFloat[10][ji] = distanceLP->Right;
		raw_dataFloat[11][ji] = distanceLP->Left;
//		raw_dataFloat[12][ji] = *wallCenterDivergence;
//		raw_dataFloat[13][ji] = weightDistanceSensor;
//		raw_dataFloat[10][ji] = vc_logging[0];
//		raw_dataFloat[11][ji] = vc_logging[1];
		raw_dataFloat[12][ji] =0;// I_mot[0];
		raw_dataFloat[13][ji] =0;// I_mot[1];

		//raw_dataFloat[15][ji] = v_r[0];

		ji++;
		}
}

void saveIMU(IMU_data_t *imuData)
{
	if(ji < LOGGING_LENGTH){
		//raw_dataAcc[0][ji] = imuData->unit_Values.x_Axis_Acc_ms2;
	}
	ji++;
}
void saveADC(ADC_data_t *adcData)
{
	if(ji < LOGGING_LENGTH){

						//raw_dataInt[0][ji] = adcData->mm_Values.mm_Right;
						//raw_dataInt[1][ji] = adcData->mm_Values.mm_Left;
	}
}


void printSaveData(void)
{
	uint8_t ccc[4];
	ccc[0] = '\0';
	UTIL1_chcat(ccc, sizeof(ccc), '\n');
	uint16_t j = 0;
	  for(j; j<ji; j++){
		   uint8_t testt[220]; //147, 180 works fine
			testt[0] = '\0';
			UTIL1_chcat(testt, sizeof(testt), '\n');

//		  			  		  UTIL1_strcatNumFloat(testt, sizeof(testt),(raw_dataInt[0][j]),1); //Dist right
//		  			  		  UTIL1_strcat(testt, sizeof(testt)," ");
//		  			  		  UTIL1_chcat(testt, sizeof(testt), '\t');
//
//		  			  		  UTIL1_strcatNumFloat(testt, sizeof(testt),(raw_dataInt[1][j]),1); //Dist left
//		  			  		  //UTIL1_Num16uToStr(testt, sizeof(testt),raw_dataSen[1][j]);
//		  			  		  UTIL1_strcat(testt, sizeof(testt)," ");
//		  			  		  UTIL1_chcat(testt, sizeof(testt), '\t');

		  			  		  UTIL1_strcatNumFloat(testt, sizeof(testt),(raw_dataFloat[0][j]),8);
		  			  		  UTIL1_strcat(testt, sizeof(testt)," ");
		  			  		  UTIL1_chcat(testt, sizeof(testt), '\t');

		  			  		  UTIL1_strcatNumFloat(testt, sizeof(testt),(raw_dataFloat[1][j]),8);
		  			  		  UTIL1_strcat(testt, sizeof(testt)," ");
		  			  		  UTIL1_chcat(testt, sizeof(testt), '\t');

		  			  		  UTIL1_strcatNumFloat(testt, sizeof(testt),(raw_dataFloat[2][j]),8);
		  			  		  UTIL1_strcat(testt, sizeof(testt)," ");
		  			  		  UTIL1_chcat(testt, sizeof(testt), '\t');

		  			  		  UTIL1_strcatNumFloat(testt, sizeof(testt),(raw_dataFloat[3][j]),8);
		  			  		  UTIL1_strcat(testt, sizeof(testt)," ");
		  			  		  UTIL1_chcat(testt, sizeof(testt), '\t');

		  			  		  UTIL1_strcatNumFloat(testt, sizeof(testt),(raw_dataFloat[4][j]),8);
		  			  		  UTIL1_strcat(testt, sizeof(testt)," ");
		  			  		  UTIL1_chcat(testt, sizeof(testt), '\t');

		  			  		  UTIL1_strcatNumFloat(testt, sizeof(testt),(raw_dataFloat[5][j]),8);
		  			  		  UTIL1_strcat(testt, sizeof(testt)," ");
		  			  		  UTIL1_chcat(testt, sizeof(testt), '\t');

		  			  		  UTIL1_strcatNumFloat(testt, sizeof(testt),(raw_dataFloat[6][j]),8);
		  			  		  UTIL1_strcat(testt, sizeof(testt)," ");
		  			  		  UTIL1_chcat(testt, sizeof(testt), '\t');

		  			  		  UTIL1_strcatNumFloat(testt, sizeof(testt),(raw_dataFloat[7][j]),8);
		  			  		  UTIL1_strcat(testt, sizeof(testt)," ");
		  			  		  UTIL1_chcat(testt, sizeof(testt), '\t');

		  			  		  UTIL1_strcatNumFloat(testt, sizeof(testt),(raw_dataFloat[8][j]),8);
		  			  		  UTIL1_strcat(testt, sizeof(testt)," ");
		  			  		  UTIL1_chcat(testt, sizeof(testt), '\t');

		  			  		  UTIL1_strcatNumFloat(testt, sizeof(testt),(raw_dataFloat[9][j]),8);
		  			  		  UTIL1_strcat(testt, sizeof(testt)," ");
		  			  		  UTIL1_chcat(testt, sizeof(testt), '\t');

		  			  		  UTIL1_strcatNumFloat(testt, sizeof(testt),(raw_dataFloat[10][j]),8);
		  			  		  UTIL1_strcat(testt, sizeof(testt)," ");
		  			  		  UTIL1_chcat(testt, sizeof(testt), '\t');

		  			  		  UTIL1_strcatNumFloat(testt, sizeof(testt),(raw_dataFloat[11][j]),8);
		  			  		  UTIL1_strcat(testt, sizeof(testt)," ");
		  			  		  UTIL1_chcat(testt, sizeof(testt), '\t');

		  			  		  UTIL1_strcatNumFloat(testt, sizeof(testt),(raw_dataFloat[12][j]),8);
		  			  		  UTIL1_strcat(testt, sizeof(testt)," ");
		  			  		  UTIL1_chcat(testt, sizeof(testt), '\t');

		  			  		  UTIL1_strcatNumFloat(testt, sizeof(testt),(raw_dataFloat[13][j]),8);
		  			  		  UTIL1_strcat(testt, sizeof(testt)," ");
		  			  		  UTIL1_chcat(testt, sizeof(testt), '\t');

//		  			  		  UTIL1_strcatNumFloat(testt, sizeof(testt),(raw_dataFloat[14][j]),8);
//		  			  		  UTIL1_strcat(testt, sizeof(testt)," ");
//		  			  		  UTIL1_chcat(testt, sizeof(testt), '\t');

//		  			  		  UTIL1_strcatNumFloat(testt, sizeof(testt),(raw_dataFloat[15][j]),8);
//		  			  		  UTIL1_strcat(testt, sizeof(testt)," ");
//		  			  		  UTIL1_chcat(testt, sizeof(testt), '\t');
//
//		  			  		  UTIL1_strcatNumFloat(testt, sizeof(testt),(raw_dataFloat[16][j]),8);
//		  			  		  UTIL1_strcat(testt, sizeof(testt)," ");
//		  			  		  UTIL1_chcat(testt, sizeof(testt), '\t');
//
//		  			  		  UTIL1_strcatNumFloat(testt, sizeof(testt),(raw_dataFloat[17][j]),8);
//		  			  		  UTIL1_strcat(testt, sizeof(testt)," ");
//		  			  		  UTIL1_chcat(testt, sizeof(testt), '\t');
		  putStream((uint8_t*)testt);
		  memset(&testt[0], 0, sizeof(testt));
		  WAIT1_Waitms(50); //20, 40
	  }
}
#endif


void PIDoverBLE(PID_data_t *pData)
{
			   uint8_t a_xbuf[100];
			  		  a_xbuf[0] = '\0';

			  		  UTIL1_strcatNumFloat(a_xbuf, sizeof(a_xbuf),(pData->states.P),8);//),8);//-biasACC_X)/1000,8);
			  		  UTIL1_strcat(a_xbuf, sizeof(a_xbuf)," ");
			  		  UTIL1_chcat(a_xbuf, sizeof(a_xbuf), '\t');
			  		//putStream((uint8_t*)a_xbuf);
			  		  UTIL1_strcatNumFloat(a_xbuf, sizeof(a_xbuf),(pData->states.D),8);//,8);//-biasGyro_Y)/1000,8);
			  		  UTIL1_strcat(a_xbuf, sizeof(a_xbuf)," ");
			  		  UTIL1_chcat(a_xbuf, sizeof(a_xbuf), '\t');
			  		putStream((uint8_t*)a_xbuf);
			  		  UTIL1_Num16uToStr(a_xbuf, sizeof(a_xbuf),pData->states.v_motor_right);
			  		  UTIL1_strcat(a_xbuf, sizeof(a_xbuf)," ");
			  		  UTIL1_chcat(a_xbuf, sizeof(a_xbuf), '\n');

			  		  putStream((uint8_t*)a_xbuf);
}


void FloatOverBLE(float pData)
{
	uint8_t a_xbuf[30];
	a_xbuf[0] = '\0';
	UTIL1_strcatNumFloat(a_xbuf, sizeof(a_xbuf),(pData),6);
	UTIL1_strcat(a_xbuf, sizeof(a_xbuf)," ");
	UTIL1_chcat(a_xbuf, sizeof(a_xbuf), '\t');
	putStream((uint8_t*)a_xbuf);
}

void IntOverBLE(int pData)
{
	uint8_t a_xbuf[30];
	a_xbuf[0] = '\0';
	UTIL1_Num32uToStr(a_xbuf, sizeof(a_xbuf),(pData));
	UTIL1_strcat(a_xbuf, sizeof(a_xbuf)," ");
	UTIL1_chcat(a_xbuf, sizeof(a_xbuf), '\t');
	putStream((uint8_t*)a_xbuf);
}

void NewLine(void)
{
	uint8_t a_xbuf[5];
	a_xbuf[0] = '\0';
	UTIL1_chcat(a_xbuf, sizeof(a_xbuf), '\n');
	putStream((uint8_t*)a_xbuf);
}


/**
 * @brief  Enc data over BLE
 * @param
 * @retval
 */
void ENCoverBLE(Encoder_data_t *pData)
{

			  static uint8_t a_xbuf[100];

			 // calcENC_data(pData);
			  		  a_xbuf[0] = '\0';

			  		  UTIL1_strcatNumFloat(a_xbuf, sizeof(a_xbuf),(pData->uint_Values.m_posR),8);//),8);//-biasACC_X)/1000,8);
			  		  UTIL1_strcat(a_xbuf, sizeof(a_xbuf)," ");
			  		  UTIL1_chcat(a_xbuf, sizeof(a_xbuf), '\t');

			  		  UTIL1_strcatNumFloat(a_xbuf, sizeof(a_xbuf),(pData->uint_Values.m_posL),8);//,8);//-biasGyro_Y)/1000,8);
			  		  UTIL1_strcat(a_xbuf, sizeof(a_xbuf)," ");
			  		  UTIL1_chcat(a_xbuf, sizeof(a_xbuf), '\n');


			  		  putStream((uint8_t*)a_xbuf);

}


/**
 * @brief  IMU data over BLE
 * @param
 * @retval
 */
void IMUoverBLE(IMU_data_t *pData)
{

			  static uint8_t a_xbuf[100];

			  calcIMU_data(pData);
			  		  a_xbuf[0] = '\0';

			  		  UTIL1_strcatNumFloat(a_xbuf, sizeof(a_xbuf),(pData->unit_Values.x_Axis_Acc_ms2),8);//),8);//-biasACC_X)/1000,8);
			  		  UTIL1_strcat(a_xbuf, sizeof(a_xbuf)," ");
			  		  UTIL1_chcat(a_xbuf, sizeof(a_xbuf), '\t');

			  		  UTIL1_strcatNumFloat(a_xbuf, sizeof(a_xbuf),(pData->unit_Values.z_Axis_Gyro_dps),8);//,8);//-biasGyro_Y)/1000,8);
			  		  UTIL1_strcat(a_xbuf, sizeof(a_xbuf)," ");
			  		  UTIL1_chcat(a_xbuf, sizeof(a_xbuf), '\n');


			  		  putStream((uint8_t*)a_xbuf);

}





void IRoverBLE(ADC_data_t *pData)
{
	static uint8_t a_xbuf[30];
	a_xbuf[0] = '\0';

	//UTIL1_chcat(a_xbuf, sizeof(a_xbuf), '\n');
	UTIL1_NumFloatToStr(a_xbuf, sizeof(a_xbuf),pData->mm_Values.mm_Right,1);
	UTIL1_strcat(a_xbuf, sizeof(a_xbuf)," ");
	  UTIL1_chcat(a_xbuf, sizeof(a_xbuf), '\t');
	  putStream((uint8_t*)a_xbuf);
	  a_xbuf[0] = '\0';
		UTIL1_NumFloatToStr(a_xbuf, sizeof(a_xbuf),pData->mm_Values.mm_Left,1);
	  UTIL1_strcat(a_xbuf, sizeof(a_xbuf)," ");
	  UTIL1_chcat(a_xbuf, sizeof(a_xbuf), '\t');
	  putStream((uint8_t*)a_xbuf);
	  a_xbuf[0] = '\0';
		UTIL1_NumFloatToStr(a_xbuf, sizeof(a_xbuf),pData->mm_Values.mm_MiddleL,1);
	  UTIL1_strcat(a_xbuf, sizeof(a_xbuf)," ");
	  UTIL1_chcat(a_xbuf, sizeof(a_xbuf), '\t');
	  UTIL1_chcat(a_xbuf, sizeof(a_xbuf), '\n');
	  putStream((uint8_t*)a_xbuf);

}




void Print_ADCs_100M(ADC_data_t *pData)
{
	uint8_t dist[100];
	uint8_t newline[4];
	dist[0] = '\0';
	newline[0] = '\0';
		  int i = 0;
		  UTIL1_chcat(newline, sizeof(newline), '\n');
		  putStream((uint8_t*)newline);
			  for(i; i<100; i++){
				  WAIT1_Waitms(20);
				  UTIL1_NumFloatToStr(dist, sizeof(dist),pData->raw_Values.raw_MiddleL,1);
				  UTIL1_strcat(dist, sizeof(dist)," ");
				  UTIL1_chcat(dist, sizeof(dist), '\t');
				  putStream((uint8_t*)dist);
		  }
	  //}
}

void Print_ADCs_100L(ADC_data_t *pData){
	uint8_t dist[100];
	uint8_t newline[4];
	dist[0] = '\0';
	newline[0] = '\0';
		  int i = 0;
		  UTIL1_chcat(newline, sizeof(newline), '\n');
		  putStream((uint8_t*)newline);
			  for(i; i<500; i++){
				  I_LED_R_SetVal();I_LED_L_SetVal(); // turn IR leds off
				  WAIT1_Waitus(500);
				  calcADC_data(pData);
				  I_LED_R_ClrVal();I_LED_L_ClrVal(); // turn IR leds off
				  dist[0] = '\0';
				  UTIL1_strcatNumFloat(dist, sizeof(dist),pData->raw_Values.raw_Left,1);
				  UTIL1_strcat(dist, sizeof(dist)," ");
				  UTIL1_chcat(dist, sizeof(dist), '\t');
				  UTIL1_strcatNumFloat(dist, sizeof(dist),pData->raw_Values.raw_Right,1);
				  UTIL1_strcat(dist, sizeof(dist)," ");
				  UTIL1_chcat(dist, sizeof(dist), '\n');
				  putStream((uint8_t*)dist);
				  WAIT1_Waitms(10); //40

		  }

}

void Print_ADCs_100R(ADC_data_t *pData){
	uint8_t dist[100];
	uint8_t newline[4];
	dist[0] = '\0';
	newline[0] = '\0';
		  int i = 0;
		  UTIL1_chcat(newline, sizeof(newline), '\n');
		  putStream((uint8_t*)newline);
			  for(i; i<100; i++){

				  WAIT1_Waitms(20);
				  UTIL1_NumFloatToStr(dist, sizeof(dist),pData->raw_Values.raw_Right,1);
				  UTIL1_strcat(dist, sizeof(dist)," ");
				  UTIL1_chcat(dist, sizeof(dist), '\t');
				  putStream((uint8_t*)dist);
		  }
}

void Print_ADCs_100R45(ADC_data_t *pData){
	uint8_t dist[100];
	uint8_t newline[4];
	dist[0] = '\0';
	newline[0] = '\0';
		  int i = 0;
		  UTIL1_chcat(newline, sizeof(newline), '\n');
		  putStream((uint8_t*)newline);
			  for(i; i<100; i++){

				  WAIT1_Waitms(20);
				  UTIL1_NumFloatToStr(dist, sizeof(dist),pData->raw_Values.raw_45Right,1);
				  UTIL1_strcat(dist, sizeof(dist)," ");
				  UTIL1_chcat(dist, sizeof(dist), '\t');
				  putStream((uint8_t*)dist);
		  }
}


void Print_ADCs_100L45(ADC_data_t *pData){
	uint8_t dist[100];
	uint8_t newline[4];
	dist[0] = '\0';
	newline[0] = '\0';
		  int i = 0;
		  UTIL1_chcat(newline, sizeof(newline), '\n');
		  putStream((uint8_t*)newline);
			  for(i; i<100; i++){

				  WAIT1_Waitms(20);
				  UTIL1_NumFloatToStr(dist, sizeof(dist),pData->raw_Values.raw_45Left,1);
				  UTIL1_strcat(dist, sizeof(dist)," ");
				  UTIL1_chcat(dist, sizeof(dist), '\t');
				  putStream((uint8_t*)dist);
		  }
}


void Print_Maze (uint8_t y, uint8_t x, uint8_t maze[y][x]){
	uint8_t data[50];
	data[0] = '\0';
	uint8_t newline[2];
	newline[0] = '\0';
	int j =0;
	  for(j; j<y; j++){
		  int i = 0;
		  UTIL1_chcat(newline, sizeof(newline), '\n');
		  putStream((uint8_t*)newline);
			  for(i; i<x; i++){
				  WAIT1_Waitms(1);
				  UTIL1_Num8uToStr(data, sizeof(data),maze[j][i]);
				  UTIL1_strcat(data, sizeof(data)," ");
				  UTIL1_chcat(data, sizeof(data), '\t');
				  putStream((uint8_t*)data);
		  }
	  }
	  putStream((uint8_t*)newline);
	  putStream((uint8_t*)newline);
}
