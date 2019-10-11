/*
 * Explore.c
 *
 *  Created on: 11.10.2019
 *      Author: Theo
 */

#include "stdbool.h"
#include "ADC.h"
#include "Explore.h"

volatile static segEndDet_state_t segEndDetState = FIRST_CALL;

bool segEndDetection(ADC_data_t *adcData,bool *segmentEnd) {

	static float prev_2_dist[2] 	= { 0.0, 0.0};
	static float prev_1_dist[2] 	= { 0.0, 0.0};
	static float dist[2] 			= { 0.0, 0.0};



	switch (segEndDetState){
		case FIRST_CALL:
			prev_2_dist[0] 	= adcData->mm_Values.mm_Right;
			prev_2_dist[1] 	= adcData->mm_Values.mm_Left;
			prev_1_dist[0] 	= adcData->mm_Values.mm_Right;
			prev_1_dist[1] 	= adcData->mm_Values.mm_Left;
			dist[0] 		= adcData->mm_Values.mm_Right;
			dist[1] 		= adcData->mm_Values.mm_Left;
			segEndDetState  = SECOND_CALL;
			break;
		case SECOND_CALL:
			prev_1_dist[0] 	= adcData->mm_Values.mm_Right;
			prev_1_dist[1] 	= adcData->mm_Values.mm_Left;
			dist[0] 		= adcData->mm_Values.mm_Right;
			dist[1] 		= adcData->mm_Values.mm_Left;
			segEndDetState 	= LATER_CALL;
			break;
		case LATER_CALL:
			prev_2_dist[0] 	= prev_1_dist[0];
			prev_2_dist[1] 	= prev_1_dist[1];
			prev_1_dist[0] 	= dist[0];
			prev_1_dist[1] 	= dist[1];
			dist[0] 		= adcData->mm_Values.mm_Right;
			dist[1] 		= adcData->mm_Values.mm_Left;
			break;
	}

	*segmentEnd =FALSE;
	return TRUE;


}


void reinit_Explore(void){
	segEndDetState = FIRST_CALL;
}
