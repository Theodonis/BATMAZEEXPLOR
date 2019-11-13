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
#include "Explore.h"


//#include "QuadSmaple.h"
//#include "Distance_INT.h"

IMU_data_t iimuData;
ADC_data_t aadcData;
Encoder_data_t eencData;
int16_t ccounter;
uint16_t speedR;
uint16_t speedL;




uint8_t header[LOGGING_NUMBER_OF_VALUES][50];
float raw_dataFloat[LOGGING_NUMBER_OF_VALUES][LOGGING_LENGTH];
//float raw_dataFloat[11][LOGGING_LENGTH];
//float raw_dataAcc[1][LOGGING_LENGTH];
//uint16_t raw_dataInt[2][LOGGING_LENGTH];
//int16_t raw_dataSen[4][7500];
//nt16_t raw_dataSen[1][6100];
//int16_t raw_dataSen[3][10000];
//uint8_t lookUP[32768];
uint16_t saveLinePointer = 0;
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
						if(saveLinePointer < 1900){// && overstepflag >=2 ){
						raw_dataFloat[0][saveLinePointer] = v_ref[0];
						raw_dataFloat[1][saveLinePointer] = v_ref[1];
						raw_dataFloat[2][saveLinePointer] = v_est[0];
						raw_dataFloat[3][saveLinePointer] = v_est[1];
						raw_dataFloat[4][saveLinePointer] = U_mot[0];
						raw_dataFloat[5][saveLinePointer] = U_mot[1];
						raw_dataFloat[6][saveLinePointer] = M[0];
						raw_dataFloat[7][saveLinePointer] = M[1];
						saveLinePointer++;
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
	  for(j; j<saveLinePointer; j++){
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
	float vc_logging[2], float I_mot[2],float u_bat_test,ADC_data_t *adcData, uint16_t* p_dirstCnt){
	if(saveLinePointer < LOGGING_LENGTH){
		raw_dataFloat[0][saveLinePointer] = (float) *p_dirstCnt;// (float)adcData->raw_Values.raw_Right;//0;//v_r[0];
		raw_dataFloat[1][saveLinePointer] = (float) *(p_dirstCnt+1);// (float)adcData->raw_Values.raw_Left;//0;//v_r[1];
		raw_dataFloat[2][saveLinePointer] = (float) *(p_dirstCnt+2);// adcData->mm_Values.mm_Right;//0;//v_est[0];
		raw_dataFloat[3][saveLinePointer] = 0;// adcData->raw_Values.raw_MiddleR;//0;//v_est[1];
//		raw_dataFloat[2][saveLinePointer] = vc_logging[0];
//		raw_dataFloat[3][saveLinePointer] = vc_logging[1];
		raw_dataFloat[4][saveLinePointer] = 0;// distanceLP->Right;//0;//q_r[0];
		raw_dataFloat[5][saveLinePointer] = 0;// distanceLP->Left;//0;//q_r[1];
		raw_dataFloat[6][saveLinePointer] = 0;// p_ADC_BIAS->raw_Right;//0;//I_mot_ist[0];
		raw_dataFloat[7][saveLinePointer] = 0;// p_ADC_BIAS->raw_Left;//0;//I_mot_ist[1];
		raw_dataFloat[8][saveLinePointer] = 0;
//		raw_dataFloat[8][saveLinePointer] = 0;//q[1];
		raw_dataFloat[9][saveLinePointer] = q[0];// 0;//q[2]; /* x-Position of MC-Car in m */
//		raw_dataFloat[4][saveLinePointer] = I_mot[0];
//		raw_dataFloat[5][saveLinePointer] = I_mot[1];

		/* Walls */
		raw_dataFloat[10][saveLinePointer] = 0;// v_est[0];//(float)adcData->mm_Values.mm_Right;
		raw_dataFloat[11][saveLinePointer] = 0;//  v_est[1];// (float)adcData->mm_Values.mm_Left;
//		raw_dataFloat[12][saveLinePointer] = *wallCenterDivergence;
//		raw_dataFloat[13][saveLinePointer] = weightDistanceSensor;
//		raw_dataFloat[10][saveLinePointer] = vc_logging[0];
//		raw_dataFloat[11][saveLinePointer] = vc_logging[1];
		raw_dataFloat[12][saveLinePointer] = q[1];//I_mot[0]; 0;///* y-Position of MC-Car in m */
		raw_dataFloat[13][saveLinePointer] = q[2];//0;// I_mot[1]; /* theat-Angle of MC-Car in rad */

		//raw_dataFloat[15][saveLinePointer] = v_r[0];

		saveLinePointer++;
		}
}

void saveIMU(IMU_data_t *imuData)
{
	if(saveLinePointer < LOGGING_LENGTH){
		//raw_dataAcc[0][saveLinePointer] = imuData->unit_Values.x_Axis_Acc_ms2;
	}
	saveLinePointer++;
}
void saveADC(ADC_data_t *adcData)
{
	if(saveLinePointer < LOGGING_LENGTH){

						//raw_dataInt[0][saveLinePointer] = adcData->mm_Values.mm_Right;
						//raw_dataInt[1][saveLinePointer] = adcData->mm_Values.mm_Left;
	}
}

void setExplorationDataHeader(uint8_t varName[], uint8_t savepos){
	if(savepos<LOGGING_NUMBER_OF_VALUES){
		UTIL1_strcpy(header[savepos], sizeof(header[savepos]),varName);
		UTIL1_chcat(header[savepos], sizeof(header[savepos]), '\t');
	}
}


void saveExplorationValue(float value, uint8_t varName[], uint8_t savepos){
	if(saveLinePointer==0){
 		setExplorationDataHeader(varName,savepos);
	}
	if(saveLinePointer < LOGGING_LENGTH){
		raw_dataFloat[savepos][saveLinePointer] = value;
	}
}

void incrmentSaveLinePointer(void){
	saveLinePointer++;
}

void resetSaveLinePointer(void){
	saveLinePointer = 0;
}

void saveExplorationData(t_explore_log *data){
	if(saveLinePointer==0){
 		setExplorationDataHeader(" ",1);
	}
	if(saveLinePointer < LOGGING_LENGTH){
		raw_dataFloat[0][saveLinePointer] = data->logADC.mm_Values.mm_MiddleR;


		saveLinePointer++;
	}
}


void printSaveData(void)
{
	uint8_t ccc[4];
	ccc[0] = '\0';
	UTIL1_chcat(ccc, sizeof(ccc), '\n');
	uint16_t j = 0;
	uint8_t headOut[LOGGING_NUMBER_OF_VALUES*50];
	for(j;j<LOGGING_NUMBER_OF_VALUES; j++){
		 UTIL1_strcat(headOut, sizeof(headOut), header[j]);

	}
	putStream((uint8_t*)headOut);
	memset(&headOut[0], 0, sizeof(headOut));
	WAIT1_Waitms(50); //20, 40


	j = 0;
	for(j; j<saveLinePointer; j++){
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
