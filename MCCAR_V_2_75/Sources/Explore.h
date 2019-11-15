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
	turn90State,
	driveToEnd,
	stopped,
	turnAngleCalibration,
	initTurnAngleCalibration,
	stateNumbers /*must be last one*/
}t_PosState;



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

typedef struct mazeFieldData{ // Field means one little square in built with four Säulen?
	bool exploredFlag; // gets true when when field is explored
	t_possibleDirections posibDirections;

}t_mazeFieldData;


/* Not used any more*/
typedef struct explore_log{
	t_PosState logPosState;
	ADC_data_t logADC;
	t_data_for_exploration logPosSpeed;
}t_explore_log;




/*
** ===================================================================
**     Method      :  TargetPosStateMaschine (void)
**
**
**     @brief
**     		State machine to drive to a wall in front and return to a left branch
**     		has to be called with cycle frequency -> defined in PlatformConfig.h
**
**     @param
**
**     @return
**                         - Error code, possible codes:
**                           ERR_OK - State machine finished
**                           ERR_FAILED - Wall or branch to drive until not detected
**                           ERR_BUSY - State machine is driving
**
*/
/* ===================================================================*/
byte TargetPosStateMaschine(void);
//bool segEndDetection(ADC_data_t *adcData,bool *segmentEnd);
void reinit_Explore(void);




#endif /* SOURCES_EXPLORE_H_ */
