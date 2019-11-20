/*
 * MazeHndl.c
 *
 *  Created on: 19.11.2019
 *      Author: Theo
 */

#include "Explore.h"
#include "ExploreConfig.h"
#include "MazeHndl.h"
#include "stdbool.h"


/*
** ===================================================================
**     Method      :  initMaze(t_mazeFieldData* MazePointer)
**
**     @brief	Set every walls in maze data matrix to unknown and each field
**     			to unexplored.
**
**     @param	MazePointer: Pointer to first field of maze data
**     			matrix
**
**
*/
void initMaze(t_mazeFieldData* MazePointer){
	t_mazeFieldData maze;
	maze.exploredFlag= FALSE;
	maze.posibDirections.north = ex_unknew;
	maze.posibDirections.east = ex_unknew;
	maze.posibDirections.south = ex_unknew;
	maze.posibDirections.west = ex_unknew;

	for(uint8_t index =0; index<MAZE_FIELDS_LENGTH_EAST_DIRECTION*MAZE_FIELDS_WIDTH_NORTH_DIRECTION; index++){
		*(MazePointer+index) = maze;
	}
}


/*
** ===================================================================
**     	Method      :  byte doMazeMeasurement(ADC_data_t* adc_data, t_mazeFieldData* currentField)
**
**     	@brief	Measure side walls in each middle of Field
**
**     	@param	adc_data: Pointer to current adc values
**     			currentField: Pointer to currentField to write data in
**
**		@return Error code, possible codes:
**                           ERR_OK - measurement don
**                           ERR_FAILED - Wall or branch to drive until not detected
*/
byte doMazeMeasurement(ADC_data_t* adc_data, t_mazeFieldData* currentField, t_directions  currentTargetOrientation){
	switch(currentTargetOrientation){
		case north:
			if(adc_data->mm_Values.mm_Right>MAX_DIST_TO_WALL_MM){/* no wall right */
				currentField->posibDirections.east = ex_true;
			}else{
				currentField->posibDirections.east = ex_false;
			}
			if(adc_data->mm_Values.mm_Left>MAX_DIST_TO_WALL_MM){/* no wall left */
				currentField->posibDirections.west = ex_true;
			}else{
				currentField->posibDirections.west = ex_false;
			}
			break;
		case east:
			if(adc_data->mm_Values.mm_Right>MAX_DIST_TO_WALL_MM){/* no wall right */
				currentField->posibDirections.south = ex_true;
			}else{
				currentField->posibDirections.south = ex_false;
			}
			if(adc_data->mm_Values.mm_Left>MAX_DIST_TO_WALL_MM){/* no wall left */
				currentField->posibDirections.north = ex_true;
			}else{
				currentField->posibDirections.north = ex_false;
			}
			break;
		case south:
			if(adc_data->mm_Values.mm_Right>MAX_DIST_TO_WALL_MM){/* no wall right */
				currentField->posibDirections.west = ex_true;
			}else{
				currentField->posibDirections.west = ex_false;
			}
			if(adc_data->mm_Values.mm_Left>MAX_DIST_TO_WALL_MM){/* no wall left */
				currentField->posibDirections.east = ex_true;
			}else{
				currentField->posibDirections.east = ex_false;
			}
			break;
		case west:
			if(adc_data->mm_Values.mm_Right>MAX_DIST_TO_WALL_MM){/* no wall right */
				currentField->posibDirections.north = ex_true;
			}else{
				currentField->posibDirections.north = ex_false;
			}
			if(adc_data->mm_Values.mm_Left>MAX_DIST_TO_WALL_MM){/* no wall left */
				currentField->posibDirections.south = ex_true;
			}else{
				currentField->posibDirections.south = ex_false;
			}
			break;
	}
	currentField->exploredFlag = true;
	return ERR_OK;
}
