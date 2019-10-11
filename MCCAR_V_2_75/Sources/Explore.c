/*
 * Explore.c
 *
 *  Created on: 11.10.2019
 *      Author: Theo
 */

#include "stdbool.h"
#include "ADC.h"
#include "Explore.h"
#define SEGMENT_END_TRESHOLD 0.08

volatile static segEndDet_state_t segEndDetState = FIRST_CALL;

bool segEndDetection(ADC_data_t *adcData,bool *segmentEnd) {

	static float prev_2_dist[2] 	= { 0.0, 0.0};
	static float prev_1_dist[2] 	= { 0.0, 0.0};
	static float dist[2] 			= { 0.0, 0.0};
	float prevVal;
	float actualVal;
	float d_Val;
	bool segEnd = FALSE;



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
	/* Calculation for rightDistance */
	prevVal  	= prev_2_dist[0] + prev_1_dist[0];
	actualVal 	= prev_1_dist[0] + dist[0];

	d_Val 		= actualVal - prevVal;
	d_Val 		= d_Val*d_Val;

	if (d_Val>SEGMENT_END_TRESHOLD){
		segEnd = TRUE;
	}
	/* Calculation for leftDistance */
	prevVal  	= prev_2_dist[1] + prev_1_dist[1];
	actualVal 	= prev_1_dist[1] + dist[1];

	d_Val 		= actualVal - prevVal;
	d_Val 		= d_Val*d_Val;

	/* Decide if segmentend detected */
	if (d_Val>SEGMENT_END_TRESHOLD && segEnd){
		segEnd = TRUE;
	}else{
		segEnd = FALSE;
	}

	*segmentEnd =segEnd;
	return TRUE;


}


void reinit_Explore(void){
	segEndDetState = FIRST_CALL;
}
