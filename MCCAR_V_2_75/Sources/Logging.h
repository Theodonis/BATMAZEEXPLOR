


/*
 * Logging.h
 *
 *  Created on: 17.01.2019
 *      Author: cc-isn
 */

#ifndef SOURCES_LOGGING_H_
#define SOURCES_LOGGING_H_

#include "ADC.h"
#include "IMU.h"
#include "PID.h"
#include "Encoder.h"
#include "Driving.h"

#define LOGGING_LENGTH 2500

void RawDataOverBLE(void);
void IMUoverBLE(IMU_data_t *pData);
void IRoverBLE(ADC_data_t *pData);
void Print_ADCs_100M(ADC_data_t *pData);
void Print_ADCs_100L(ADC_data_t *pData);
void Print_ADCs_100R(ADC_data_t *pData);
void Print_ADCs_100R45(ADC_data_t *pData);
void Print_ADCs_100L45(ADC_data_t *pData);
void Print_Maze (uint8_t y, uint8_t x, uint8_t maze[y][x]);
void PIDoverBLE(PID_data_t *pData);
//void saveData(IMU_data_t *imuData, ADC_data_t *adcData, Encoder_data_t *encData, int16_t *counter,  float v_est[2] , float f_value ,float f_value2 ,float f_array[3],float f_array2[3]);

/*
 * Note bug when converting negative whole-number float values to string.
 * Would convert negative number to positive if the following correction is not done:
 * in: UTIL1_strcatNumFloat - >UTIL1_NumFloatToStr
 * correct to >= : if (isNeg && fractional>***=***0 && nofFracDigits>0) {
 */
void printSaveData(void);

void FloatOverBLE(float pData);
void NewLine(void);
void saveADC(ADC_data_t *adcData);
void saveIMU(IMU_data_t *imuData);
void IntOverBLE(int pData);
void saveData(float *wallCenterDivergence, float weightDistanceSensor, float v_r[2],  float v_est[2], float q_r[3], float q[3],
		Distance_sensors_HP_filtered *distance_HP, float x_IMU_dot_test, float x_enc_test, float I_mot_ist[2],
		Distance_sensors_LP_filtered *distanceLP, float d_hp_filtered, bool SegmentFinished, float gyroXY[2],
		Distance_Bandpass_t *distanceBandpass, Wall_availability_state *wallState,
		float vc_logging[2], float I_mot[2] ,float u_bat_test);
void saveControllerData(float v_ref[2],float v_est[2],float U_mot[2],float M[2]);
void printSaveControllerData(void);
void ENCoverBLE(Encoder_data_t *pData);
#endif /* SOURCES_LOGGING_H_ */
