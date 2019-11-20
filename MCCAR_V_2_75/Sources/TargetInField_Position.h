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
										//calib //Physmeasured
#define MAZE_FIELD_LENGTH 				(0.19)	//(0.17)
#define HALF_OF_MAZE_FIELD_LENGTH 		(0.095)	//(0.085)
#define QUARTER_OF_MAZE_FIELD_LENGTH 	(0.0475)//(0.0425)
#define INIT_POS_INFIELD 				(0.078)		//(0.074)
#define POS_INFIELD_AFTER_TURN_180 		(0.075)//not yet done...		//(0.075)
#define POS_INFIELD_AFTER_TURN_90 		(0.0425)//not yet done...

typedef enum fieldstate{
	fieldinitState,
	firstQuarterOfField,
	secondQuarterOfField,
	thirdQuarterOfField,
	fourthQuarterOfField,
	detectSideWalls,
	targetHasTurned

}t_fieldState;



/* Filter of Distance for SegmentEndDetection */
#define SEG_END_DET_DISTANCE_LP_LAST_VAL_FACTOR 0.95
#define SEG_END_DET_DERIVATION_LP_LAST_VAL_FACTOR 0.93




t_fieldState fieldPositioner(t_PosEstimation pos,uint8_t* xPos,uint8_t* yPos, t_directions targetOrientation);



#endif /* SOURCES_TARGETINFIELD_POSITION_H_ */
