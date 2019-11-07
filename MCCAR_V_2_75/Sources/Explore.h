/*
 * Explore.h
 *
 *  Created on: 11.10.2019
 *      Author: Theo
 */

#ifndef SOURCES_EXPLORE_H_
#define SOURCES_EXPLORE_H_
#include <stdint.h>
#include "ADC.h"
#include "PE_Error.h"

typedef enum PosState{
	initState,
	driveToFrontWall,
	FrontWallDetected,
	turnState,
	driveToLeftBranch,
	leftBranchDetected,
	stopped,
	stateNumbers /*mustz be last one*/
}t_PosState;


/* Filter of Distance for SegmentEndDetection */
#define SEG_END_DET_DISTANCE_LP_LAST_VAL_FACTOR 0.95
#define SEG_END_DET_DERIVATION_LP_LAST_VAL_FACTOR 0.93

byte TargetPosStateMaschine(void);
bool segEndDetection(ADC_data_t *adcData,bool *segmentEnd);
void reinit_Explore(void);




#endif /* SOURCES_EXPLORE_H_ */
