/*
 * Explore.c
 *
 *  Created on: 11.10.2019
 *      Author: Theo
 */

#include "stdbool.h"
#include "ADC.h"
#include "Explore.h"

bool segEndDetection(ADC_data_t *adcData,bool *segmentEnd) {

	static float LP_dist[2] 			= { 0.0, 0.0};
	static float prev_LP_dist[2] 		= { 0.0, 0.0};
	static float dist_dot_square[2] 	= { 0.0, 0.0};
	static float LP_dist_dot_square[2] 	= { 0.0, 0.0};

	bool segEnd = FALSE;

	/* init at first call */
	if (LP_dist[0] == 0.0){
		LP_dist[0]		= (float)adcData->raw_Values.raw_Right;
		LP_dist[1]		= (float)adcData->raw_Values.raw_Left;
		prev_LP_dist[0]	= LP_dist[0];
		prev_LP_dist[1]	= LP_dist[1];
	}

	/* LowPass for rightDistance */
	LP_dist[0] 	= LP_dist[0] * SEG_END_DET_DISTANCE_LP_LAST_VAL_FACTOR  + (float)adcData->raw_Values.raw_Right * (1.0-SEG_END_DET_DISTANCE_LP_LAST_VAL_FACTOR);
	/* LowPass for leftDistance */
	LP_dist[1] 	= LP_dist[1] * SEG_END_DET_DISTANCE_LP_LAST_VAL_FACTOR  + (float)adcData->raw_Values.raw_Left * (1.0-SEG_END_DET_DISTANCE_LP_LAST_VAL_FACTOR);


	/* Derivation */

	dist_dot_square[0] = LP_dist[0]-prev_LP_dist[0];	//Right side derivation
	dist_dot_square[1] = LP_dist[0]-prev_LP_dist[1];	//Left side derivation

	/* Derivation Square */

	dist_dot_square[0] = dist_dot_square[0]*dist_dot_square[0]; //Square of right side derivation
	dist_dot_square[1] = dist_dot_square[1]*dist_dot_square[1]; //Square of left side derivation

	/* LowPass for rightDistanceDerivation */
	LP_dist_dot_square[0] = LP_dist_dot_square[0] * SEG_END_DET_DERIVATION_LP_LAST_VAL_FACTOR + dist_dot_square[0] * (1.0-SEG_END_DET_DERIVATION_LP_LAST_VAL_FACTOR);
	/* LowPass for leftDistanceDerivation */
	LP_dist_dot_square[1] = LP_dist_dot_square[1] * SEG_END_DET_DERIVATION_LP_LAST_VAL_FACTOR + dist_dot_square[1] * (1.0-SEG_END_DET_DERIVATION_LP_LAST_VAL_FACTOR);


	/* if filtered square of derivation is to high -> its a segment end */
	if(LP_dist_dot_square[0]> 350){ // 350 is only a Test value to save up multiplications (instead of lowlow pass to compare) 70 ->expectation x 5 =350
		*segmentEnd =segEnd;
	}
	if(LP_dist_dot_square[1]> 600){ // 600 is only a Test value to save up multiplications (instead of lowlow pass to compare) 120 ->expectation x 5 =600
		*segmentEnd =segEnd;
	}

	return TRUE;
}


void reinit_Explore(void){
}
