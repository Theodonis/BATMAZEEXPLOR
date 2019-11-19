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
**     @brief
**     		Set every walls in maze data matrix to unknown and each field
**     		to unexplored.
**
**     @param
**						- MazePointer: Pointer to first field of maze data
**						  matrix
**
**     @return
**
**
*/
/* ===================================================================*/
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



byte doMazeMeasurement(ADC_data_t* adc_data, t_mazeFieldData* currentField){

	currentField->exploredFlag = true;


	return ERR_OK;
}
