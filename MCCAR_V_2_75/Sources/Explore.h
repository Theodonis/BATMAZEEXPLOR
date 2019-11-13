/*
 * Explore.h
 *
 *  Created on: 11.10.2019
 *      Author: Theo
 */

#ifndef SOURCES_EXPLORE_H_
#define SOURCES_EXPLORE_H_
#include "ExploreConfig.h"
#include <stdint.h>
#include "ADC.h"
#include "PE_Error.h"
#include "DrivingExplore_Interface.h"



typedef enum PosState{
	initState,
	driveToFrontWall,
	FrontWallDetected,
	turnState,
	driveToLeftBranch,
	leftBranchDetected,
	stopped,
	turnAngleCalibration,
	initTurnAngleCalibration,
	stateNumbers /*must be last one*/
}t_PosState;

typedef enum fieldstate{
	fieldinitState,
	firstQuarterOfField,
	secondQuarterOfField,
	thirdQuarterOfField,
	fourthQuarterOfField

}t_fieldState;

typedef enum directons{
	north,
	east,
	south,
	west

}t_directions;

typedef enum exploreInformation{
	ex_unknew,
	ex_true,
	ex_false

}t_exploreInformation;

typedef struct posibleDirections{
	t_exploreInformation north;
	t_exploreInformation east;
	t_exploreInformation south;
	t_exploreInformation west;

}t_possibleDirections;

typedef struct mazeFieldData{ // Field means one little square in built with four S�ulen?
	bool exploredFlag; // gets true when when field is explored
	t_possibleDirections posibDirections;

}t_mazeFieldData;


/* Not used any more*/
typedef struct explore_log{
	t_PosState logPosState;
	ADC_data_t logADC;
	t_data_for_exploration logPosSpeed;
}t_explore_log;

/* Filter of Distance for SegmentEndDetection */
#define SEG_END_DET_DISTANCE_LP_LAST_VAL_FACTOR 0.95
#define SEG_END_DET_DERIVATION_LP_LAST_VAL_FACTOR 0.93

byte TargetPosStateMaschine(void);
bool segEndDetection(ADC_data_t *adcData,bool *segmentEnd);
void reinit_Explore(void);




#endif /* SOURCES_EXPLORE_H_ */
