/*
 * Encoder.c
 *
 *  Created on: 17.01.2019
 *      Author: cc-isn
 */

#include "Encoder.h"



int16_t rpm_R_array[12];
int16_t rpm_L_array[12];
int array_counter;


/**
 *
 */
void calcENC_data(Encoder_data_t *pData)
{

	pData->raw_Values.posR = FTM1_CNT;
	pData->raw_Values.posL = -1*FTM2_CNT;
	pData->raw_Values.posR =(int16_t)(pData->raw_Values.posR*0.25);
	pData->raw_Values.posL = (int16_t)(pData->raw_Values.posL*0.25);
	if((pData->raw_Values.posR > 7500) || (pData->raw_Values.posL > 7500)){
		FTM1_CNT=0;
		FTM2_CNT=0;
		pData->uint_Values.m_posR = pData->uint_Values.m_posR+(pData->raw_Values.posR-pData->raw_Values.posR_old) * STEPS_TO_METER;
		pData->uint_Values.m_posL = pData->uint_Values.m_posL+(pData->raw_Values.posL-pData->raw_Values.posL_old) * STEPS_TO_METER;
		pData->raw_Values.posR_old = 0;
		pData->raw_Values.posL_old = 0;
		array_counter = 0;
	}
	else{
		pData->uint_Values.m_posR = pData->uint_Values.m_posR+(pData->raw_Values.posR-pData->raw_Values.posR_old) * STEPS_TO_METER;
		pData->uint_Values.m_posL = pData->uint_Values.m_posL+(pData->raw_Values.posL-pData->raw_Values.posL_old) * STEPS_TO_METER;
		pData->raw_Values.posR_old = pData->raw_Values.posR;
		pData->raw_Values.posL_old = pData->raw_Values.posL;

		rpm_R_array[array_counter] = pData->raw_Values.posR;
		rpm_L_array[array_counter] = pData->raw_Values.posL;

		if(array_counter==11){
			pData->uint_Values.rpm_posR = (rpm_R_array[11]-rpm_R_array[1])*STEPS_TO_RPM;
			pData->uint_Values.rpm_posL = (rpm_R_array[11]-rpm_R_array[1])*STEPS_TO_RPM;
			array_counter = 0;

		}
	}

	array_counter++;
}



void resetPosition(void)
{
//	HS_MOT_R_SetPos(0);
//	HS_MOT_L_SetPos(0);
}
