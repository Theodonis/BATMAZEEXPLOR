/*
 * IMU.h
 *
 *  Created on: 16.01.2019
 *      Author: cc-isn
 */

#ifndef SOURCES_IMU_H_
#define SOURCES_IMU_H_

#include "Application.h"


#define SAMPLETIME 0.0005f
#define imuAddress 0x6A //IMU Address when SDO/SA0 pin is connected to ground, the LSb value is ‘0’ (address 1101010b)
#define DEGREE_TO_RADIANT 0.01745329251994329576923690768489f


typedef struct IMU_data_{

	struct{
		float  x_Axis_Acc_mg;	/* Input	: */
		float z_Axis_Gyro_mdps;	/* Input	:  */
		float x_Axis_Gyro_mdps;	/* Input	:  */
		float y_Axis_Gyro_mdps;	/* Input	:  */
	}raw_Values;
	//
	struct{
		 float  bias_x_Axis_Acc_mg;	/* Input	: */

		 float  bias_z_Axis_Gyro_mdps;	/* Input	:  */
		 float  bias_x_Axis_Gyro_mdps;	/* Input	:  */
		 float  bias_y_Axis_Gyro_mdps;	/* Input	:  */


	} bias_Values;
	//
	struct{
		float  x_Axis_Acc_ms2;	/* Input	: */
		float  z_Axis_Gyro_dps;	/* Input	:  */
		float  z_Axis_Gyro_rps;	/* Input	:  */
		float  x_Axis_Acc_velocity;;	/* Input	:  */
		float  z_Axis_Gyro_angle;

		float  x_Axis_Gyro_dps;	/* Input	:  */
		float  x_Axis_Gyro_rps;	/* Input	:  */
		float  x_Axis_Gyro_angle;

		float  y_Axis_Gyro_dps;	/* Input	:  */
		float  y_Axis_Gyro_rps;	/* Input	:  */
		float  y_Axis_Gyro_angle;
	} unit_Values;
	//

}IMU_data_t;



bool initIMU(void);
void calcIMU_data(IMU_data_t *pData);
void biasCalc(void);
float getGyroBias_X(void);
float getGyroBias_Y(void);
float getGyroBias_Z(void);
float getAccBias(void);

#endif /* SOURCES_IMU_H_ */
