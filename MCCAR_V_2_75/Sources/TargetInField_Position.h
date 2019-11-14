/*
 * TargetInField_Position.h
 *
 *  Created on: 14.11.2019
 *      Author: Theo
 */

#ifndef SOURCES_TARGETINFIELD_POSITION_H_
#define SOURCES_TARGETINFIELD_POSITION_H_

#include "Explore.h"
#include "ADC.h"
#include <stdbool.h>

#define MAZE_FIELD_LENGTH 				(0.17)
#define HALF_OF_MAZE_FIELD_LENGTH 		(0.085)
#define QUARTER_OF_MAZE_FIELD_LENGTH 	(0.0425)
#define INIT_POS_INFIELD 				(0.074)
#define POS_INFIELD_AFTER_TURN_180 		(0.075)
#define POS_INFIELD_AFTER_TURN_90 		(0.0425)

typedef enum fieldstate{
	fieldinitState,
	firstQuarterOfField,
	secondQuarterOfField,
	thirdQuarterOfField,
	fourthQuarterOfField,
	detectWalls,
	targetHasTurned

}t_fieldState;



/* Filter of Distance for SegmentEndDetection */
#define SEG_END_DET_DISTANCE_LP_LAST_VAL_FACTOR 0.95
#define SEG_END_DET_DERIVATION_LP_LAST_VAL_FACTOR 0.93



/*
** ===================================================================
**     Method      :  t_fieldState fieldPositioner(t_PosEstimation pos, uint8_t* xPos,uint8_t* yPos,t_mazeFieldData* maze)
**
**
**     @brief
**     		State machine to handle the position in current maze field
**
**     @param
**     			- pos: Estimation from driving() of x, y distance and theta angle of target
**     			- xPos: Pointer to current field x-index of maze data matrix
**				- yPos: Pointer to current field y-index of maze data matrix
**
**     @return
**         		- fieldState: the current state in field -> allows to do measurement in midle of the field
**
*/
/* ===================================================================*/
t_fieldState fieldPositioner(t_PosEstimation pos,uint8_t* xPos,uint8_t* yPos, t_directions targetOrientation);



#endif /* SOURCES_TARGETINFIELD_POSITION_H_ */
