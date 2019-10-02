/*
 * Encoder.h
 *
 *  Created on: 17.01.2019
 *      Author: cc-isn
 */

#ifndef SOURCES_ENCODER_H_
#define SOURCES_ENCODER_H_

#include "Application.h"
#include "PlatformConfiguration.h"

#define STEPS_TO_METER 			(2 * PI * RADIUS_WHEELS * RATIO_SINGLE_TURN_TO_ENCODER_STEPS)//0.000403571429f //(1/140)*0.0565f //140= 11.67 (untersetzungsverhältnis) * 12 (steps/umdrehung) // 0.0565= Umfang Reifen
#define STEPS_TO_RPM 			500 //Auflösung rpm // (60*1000ms)/20ms = 3000 --> 3000/12 = 250

typedef struct Encoder_data_{
	struct{
		int16_t posR;
		int16_t posL;
		int16_t posR_old;
		int16_t posL_old;
	}raw_Values;
	struct{
		float m_posR;
		float m_posL;
		float rpm_posR;
		float rpm_posL;
	}uint_Values;
	//
}Encoder_data_t;



void calcENC_data(Encoder_data_t *pData);


void resetPosition(void);


#endif /* SOURCES_ENCODER_H_ */
