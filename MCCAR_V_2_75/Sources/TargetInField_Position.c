/*
 * TargetInFiled_Position.c
 *
 *  Created on: 13.11.2019
 *      Author: Theo
 */

#include "TargetInField_Position.h"


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
t_fieldState fieldPositioner(t_PosEstimation pos,uint8_t* xPos,uint8_t* yPos, t_directions targetOrientation){
	static float fieldpos 	= 0; /*pos in curent field */
	static float initpos 	= 0;
	static t_fieldState fieldState = fieldinitState;
	static t_directions prevTargetOrientation=north;

	/* new position in current field */
	if(targetOrientation==north || targetOrientation==south){
		fieldpos = initpos - pos.xPos;
	}else{
		fieldpos = pos.yPos + initpos;
	}

	if(prevTargetOrientation!=targetOrientation){
		fieldState = targetHasTurned; //turn flag to update direction
	}

	switch(fieldState){
		case fieldinitState:
			if(pos.xPos>1.4){ //value after init of Driving
				fieldpos = INIT_POS_INFIELD;
				fieldState = firstQuarterOfField;
				if(targetOrientation==north || targetOrientation==south){
					initpos = pos.xPos + INIT_POS_INFIELD;
				}else{
					initpos = -pos.yPos; //error?
				}
			}

			break;
		case firstQuarterOfField:
			if(fieldpos>QUARTER_OF_MAZE_FIELD_LENGTH){
				fieldState = secondQuarterOfField;
			}else if(fieldpos<0-MAZE_FIELD_LENGTH ){
				fieldState = saveFrontwall;
				if(targetOrientation==south){
					(*xPos)--;
					initpos = pos.xPos;
				}else if(targetOrientation==west){
					(*yPos)--;
					initpos = -pos.yPos;
				}
			}
			break;
		case secondQuarterOfField:
			if(fieldpos>HALF_OF_MAZE_FIELD_LENGTH){
				fieldState = detectSideWalls;
			}else if(fieldpos<0-QUARTER_OF_MAZE_FIELD_LENGTH-HALF_OF_MAZE_FIELD_LENGTH){
				fieldState = firstQuarterOfField;
			}
			break;
		case detectSideWalls: //one tick in middle of field: do Measure in Explore...
			switch(targetOrientation){
				case north:
				case east:
					/*is forwards running */
					fieldState=thirdQuarterOfField;
					break;
				case south:
				case west:
					/*is backwards running */
					fieldState=secondQuarterOfField;
					break;
			}
			break;
		case thirdQuarterOfField:
			if(fieldpos>QUARTER_OF_MAZE_FIELD_LENGTH+HALF_OF_MAZE_FIELD_LENGTH){
				fieldState = fourthQuarterOfField;
			}else if(fieldpos<0-HALF_OF_MAZE_FIELD_LENGTH){
				fieldState = detectSideWalls;
			}
			break;
		case fourthQuarterOfField:
			if(fieldpos>MAZE_FIELD_LENGTH){
				fieldState = saveFrontwall;
				if(targetOrientation==north){
					(*xPos)++;
					initpos = pos.xPos;
				}else if(targetOrientation==east){
					(*yPos)++;
					initpos = -pos.yPos;
				}
			}else if(fieldpos<0-QUARTER_OF_MAZE_FIELD_LENGTH){
				fieldState = thirdQuarterOfField;
			}
			break;

		case saveFrontwall:
			switch(targetOrientation){
				case north:
				case east:
					/*is forwards running */
					fieldState = firstQuarterOfField;
					break;
				case south:
				case west:
					fieldState = fourthQuarterOfField;
					/*is backwards running */
					break;
			}

			break;
		case targetHasTurned:
			if((prevTargetOrientation-targetOrientation)%2){//is a 90° turn
				switch(targetOrientation){
					case north:
						initpos 	= pos.xPos + POS_INFIELD_AFTER_TURN_90;
						fieldpos 	= initpos - pos.xPos;
						fieldState 	= firstQuarterOfField;
						break;
					case east:
						initpos 	= POS_INFIELD_AFTER_TURN_90 - pos.yPos  ;
						fieldpos 	= initpos + pos.yPos;
						fieldState 	= firstQuarterOfField;
						break;
					case south:
						initpos 	= pos.xPos - POS_INFIELD_AFTER_TURN_90;
						fieldpos 	= initpos - pos.xPos;
						fieldState 	= fourthQuarterOfField;
						break;
					case west:
						initpos	 	= -pos.yPos - POS_INFIELD_AFTER_TURN_90;
						fieldpos	= initpos + pos.yPos;
						fieldState 	= fourthQuarterOfField;
						break;
				}
			}else{
				switch(targetOrientation){
					case north:
						initpos 	= pos.xPos + POS_INFIELD_AFTER_TURN_180;
						fieldpos 	= initpos - pos.xPos;
						fieldState 	= firstQuarterOfField;
						break;
					case east:
						initpos 	= POS_INFIELD_AFTER_TURN_180 - pos.yPos;
						fieldpos 	= initpos + pos.yPos;
						fieldState 	= firstQuarterOfField;
						break;
					case south:
						initpos 	= pos.xPos - POS_INFIELD_AFTER_TURN_180;
						fieldpos 	= initpos - pos.xPos;
						fieldState 	= fourthQuarterOfField;
						break;
					case west:
						initpos 	= -pos.yPos - POS_INFIELD_AFTER_TURN_180;
						fieldpos 	= initpos + pos.yPos;
						fieldState 	= fourthQuarterOfField;
						break;
				}
			}

			break;
	}

	prevTargetOrientation = targetOrientation;
	return fieldState;
}



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
