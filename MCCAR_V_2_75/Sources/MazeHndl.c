/*
 * MazeHndl.c
 *
 *  Created on: 19.11.2019
 *      Author: Theo
 */

#include "Explore.h"
#include "ExploreConfig.h"
#include "stdbool.h"
#include "TargetInField_Position.h"
#include "MazeHndl.h"


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
	maze.enterDirection = unknown;

	/*init start Field */
	MazePointer->exploredFlag=TRUE;
	MazePointer->posibDirections.north = ex_true;
	MazePointer->posibDirections.east = ex_false;
	MazePointer->posibDirections.south = ex_false;
	MazePointer->posibDirections.west = ex_false;
	MazePointer->hasUnexploredBranchFlag = FALSE;
	MazePointer->enterDirection = north;
	/*init other fields as unknew (start at secondffield)*/
	for(uint8_t index = 1; index<MAZE_FIELDS_LENGTH_EAST_DIRECTION*MAZE_FIELDS_WIDTH_NORTH_DIRECTION; index++){
		*(MazePointer+index) = maze;
	}
}


/*
** ===================================================================
**     	Method      :  byte sideBranchMeasurement(ADC_data_t* adc_data, t_mazeFieldData* currentField)
**
**     	@brief	Measure side walls in each middle of Field
**
**     	@param	adc_data: Pointer to current adc values
**     			currentField: Pointer to currentField to write data in
**
**		@return Error code, possible codes:
**                           ERR_OK - measurement done
**                           ERR_FAILED - not yet used
*/
byte sideBranchMeasurement(ADC_data_t* adc_data, t_mazeFieldData* currentField, t_directions  currentTargetOrientation){

		if(adc_data->mm_Values.mm_Right>MAX_SIDEDIST_TO_WALL_MM){
			setWallInfo(currentField,get_wallOrientation(currentTargetOrientation,right),ex_true);
		}else{
			setWallInfo(currentField,get_wallOrientation(currentTargetOrientation,right),ex_false);
		}

		if(adc_data->mm_Values.mm_Left>MAX_SIDEDIST_TO_WALL_MM){
			setWallInfo(currentField,get_wallOrientation(currentTargetOrientation,left),ex_true);
		}else{
			setWallInfo(currentField,get_wallOrientation(currentTargetOrientation,left),ex_false);
		}


		currentField->exploredFlag = true;
		return ERR_OK;

		/* Old implemetation: */
//	switch(currentTargetOrientation){
//		case north:
//			if(adc_data->mm_Values.mm_Right>MAX_DIST_TO_WALL_MM){/* no wall right */
//				currentField->posibDirections.east = ex_true;
//			}else{
//				currentField->posibDirections.east = ex_false;
//			}
//			if(adc_data->mm_Values.mm_Left>MAX_DIST_TO_WALL_MM){/* no wall left */
//				currentField->posibDirections.west = ex_true;
//			}else{
//				currentField->posibDirections.west = ex_false;
//			}
//			break;
//		case east:
//			if(adc_data->mm_Values.mm_Right>MAX_DIST_TO_WALL_MM){/* no wall right */
//				currentField->posibDirections.south = ex_true;
//			}else{
//				currentField->posibDirections.south = ex_false;
//			}
//			if(adc_data->mm_Values.mm_Left>MAX_DIST_TO_WALL_MM){/* no wall left */
//				currentField->posibDirections.north = ex_true;
//			}else{
//				currentField->posibDirections.north = ex_false;
//			}
//			break;
//		case south:
//			if(adc_data->mm_Values.mm_Right>MAX_DIST_TO_WALL_MM){/* no wall right */
//				currentField->posibDirections.west = ex_true;
//			}else{
//				currentField->posibDirections.west = ex_false;
//			}
//			if(adc_data->mm_Values.mm_Left>MAX_DIST_TO_WALL_MM){/* no wall left */
//				currentField->posibDirections.east = ex_true;
//			}else{
//				currentField->posibDirections.east = ex_false;
//			}
//			break;
//		case west:
//			if(adc_data->mm_Values.mm_Right>MAX_DIST_TO_WALL_MM){/* no wall right */
//				currentField->posibDirections.north = ex_true;
//			}else{
//				currentField->posibDirections.north = ex_false;
//			}
//			if(adc_data->mm_Values.mm_Left>MAX_DIST_TO_WALL_MM){/* no wall left */
//				currentField->posibDirections.south = ex_true;
//			}else{
//				currentField->posibDirections.south = ex_false;
//			}
//			break;
//	}
}

/*
** ===================================================================
**     	Method      :  byte unexploredBranchSet(t_mazeFieldData* currentField)
**
**     	@brief	set Flag if over given Field has an unexplored way after leaving
**     			(call only if setting for field is finished)
**
**     	@param	currentField: Pointer to currentField to write data in
**
**		@return Error code, possible codes:
**                           ERR_OK - measurement done
**                           ERR_FAILED - not yet used
*/
byte unexploredBranchSet(t_mazeFieldData* currentField, t_directions currentTargetOrientation){
	currentField->hasUnexploredBranchFlag=false;

	if(currentField->posibDirections.north==ex_true && currentTargetOrientation != north){
		if((currentField+MAZE_FIELDS_LENGTH_EAST_DIRECTION)->exploredFlag==false){
			currentField->hasUnexploredBranchFlag=true;
		}
	}
	if(currentField->posibDirections.east==ex_true && currentTargetOrientation != east){
		if((currentField+1)->exploredFlag==false){ /*ToDo: Check this!!*/
			currentField->hasUnexploredBranchFlag=true;
		}
	}
	if(currentField->posibDirections.south==ex_true && currentTargetOrientation != south){
		if((currentField-MAZE_FIELDS_LENGTH_EAST_DIRECTION)->exploredFlag==false){
			currentField->hasUnexploredBranchFlag=true;
		}
	}
	if(currentField->posibDirections.west==ex_true && currentTargetOrientation != west){
		if((currentField-1)->exploredFlag==false){
			currentField->hasUnexploredBranchFlag=true;
		}
	}


	return ERR_OK;
}

/*
** ===================================================================
**     	Method      :  byte setDriveDirectionWallInfo(t_mazeFieldData* currentField, t_directions  currentTargetOrientation, t_exploreInformation frontIsOpen){
**
**     	@brief	To set wall info of passed way (current and next filed bevore leaving) in maze
**
**     	@param	currentField: Pointer to currentField to write data in
**     			currentTargetOrientation: current orientation of MC-Car
**
**		@return Error code, possible codes:
**                           ERR_OK - value set
**                           ERR_FAILED - not yet used
*/
byte setDriveDirectionWallInfo(t_mazeFieldData* currentField, t_directions  currentTargetOrientation){
	switch(currentTargetOrientation){
		case north:
			(void) setWallInfo((currentField),north,ex_true);/*Front wall of current field */
			(void) setWallInfo(currentField+MAZE_FIELDS_LENGTH_EAST_DIRECTION,south,ex_true);/*Back wall of next field*/
			(currentField+MAZE_FIELDS_LENGTH_EAST_DIRECTION)->enterDirection = currentTargetOrientation; /*Set enterDirection  of next field*/
			break;
		case east:
			(void) setWallInfo((currentField),east,ex_true);/*Front wall of current field */
			(void) setWallInfo(currentField+1,west,ex_true);/*Back wall of next field*/
			(currentField+1)->enterDirection = currentTargetOrientation; /*Set enterDirection  of next field*/
			break;

		case south:
			(void) setWallInfo((currentField),south,ex_true);/*Front wall of current field */
			(void) setWallInfo(currentField-MAZE_FIELDS_LENGTH_EAST_DIRECTION,north,ex_true);/*Back wall of next field*/
			(currentField-MAZE_FIELDS_LENGTH_EAST_DIRECTION)->enterDirection = currentTargetOrientation; /*Set enterDirection  of next field*/
			break;

		case west:
			(void) setWallInfo((currentField),west,ex_true);/*Front wall of current field */
			(void) setWallInfo(currentField-1,east,ex_true);/*Back wall of next field*/
			(currentField-1)->enterDirection = currentTargetOrientation; /*Set enterDirection  of next field*/
			break;
	}
	return ERR_OK;
}


/*
** ===================================================================
**     	Method      : byte setWallInfo(t_mazeFieldData* currentField, t_directions  wallOrientation, t_exploreInformation wallIsOpen)
**
**     	@brief	To set wall info of an maze filed
**
**     	@param	currentField: Pointer to currentField to write data in
**     			currentTargetOrientation: current orientation of MC-Car
**     			frontIsOpen: the value, should be written in in MazeFieldinfo
**     				->ex_true: possible to drive strait, ex_false: not possible
**
**		@return Error code, possible codes:
**                           ERR_OK - value set
**                           ERR_FAILED - not yet used
**
**
** ===================================================================
*/
byte setWallInfo(t_mazeFieldData* currentField, t_directions  wallOrientation, t_exploreInformation wallIsOpen){
	switch(wallOrientation){
		case north:
			currentField->posibDirections.north = wallIsOpen;
			break;
		case east:
			currentField->posibDirections.east = wallIsOpen;
			break;
		case south:
			currentField->posibDirections.south = wallIsOpen;
			break;
		case west:
			currentField->posibDirections.west = wallIsOpen;
			break;
	}
	return ERR_OK;
}

/*
** ===================================================================
**     	Method      : bool get_isUnexploredBranch(t_mazeFieldData* currentField, t_directions currentTargetOrientation, t_dir infoDirection)
**
**     	@brief	decide if in relative direction to MC-Car orientation is an
**     			an unexplored branch
**
**     	@param	currentField: Pointer to currentField
**     			currentTargetOrientation: current orientation of MC-Car
**     			infoDirection: direction relative to MC-Car orientation to get information about
**
**		@return True: 	if in infoDirection is an unexplored branch
**				False: 	else
**
**
** ===================================================================
*/
bool get_isUnexploredBranch(t_mazeFieldData* currentField, t_directions currentTargetOrientation, t_dir infoDirection){
	switch(get_wallOrientation(currentTargetOrientation,infoDirection)){
		case north:
			if(currentField->posibDirections.north == ex_true){/*no wall in infoDirection*/
				if(!(currentField+MAZE_FIELDS_LENGTH_EAST_DIRECTION)->exploredFlag){/*is an unexplored branch in infoDirection*/
					return true;
				}
			}
			break;
		case east:
			if(currentField->posibDirections.east == ex_true){/*no wall in infoDirection*/
				if(!(currentField+1)->exploredFlag){/*is an unexplored branch in infoDirection*/
					return true;
				}
			}
			break;
		case south:
			if(currentField->posibDirections.south == ex_true){/*no wall in infoDirection*/
				if(!(currentField-MAZE_FIELDS_LENGTH_EAST_DIRECTION)->exploredFlag){/*is an unexplored branch in infoDirection*/
					return true;
				}
			}
			break;
		case west:
			if(currentField->posibDirections.west == ex_true){/*no wall in infoDirection*/
				if(!(currentField-1)->exploredFlag){/*is an unexplored branch in infoDirection*/
					return true;
				}
			}
			break;
	}
	return false;
}



/*
** ===================================================================
**     	bool get_isExploredFieldInFront(t_mazeFieldData* p_currField, t_directions curOrient, t_dir infoDirection, uint8_t xPos, uint8_t yPos)
**
**     	@brief	decide if next field in driving direction was even explored
**
**     	@param	p_currField:	Pointer to currentField
**     			curOrient: 		current orientation of MC-Car
**				xPos: 			current X-Index in MazaData-Matrix
**				yPos: 			current Y-Index in MazaData-Matrix
**
**		@return True: 	if in front is an explored field
**				False: 	else
**
**
** ===================================================================
*/
bool get_isExploredFieldInFront(t_mazeFieldData* p_currField, t_directions curOrient, uint8_t xPos, uint8_t yPos){
	switch(curOrient){
		case north:
			if(xPos<MAZE_FIELDS_WIDTH_NORTH_DIRECTION-1){
				/* not in last field of Matrix*/
				return (p_currField+MAZE_FIELDS_LENGTH_EAST_DIRECTION)->exploredFlag;
			}
			break;
		case east:
			if(yPos<MAZE_FIELDS_LENGTH_EAST_DIRECTION-1){
				/* not in last field of Matrix*/
				return (p_currField+1)->exploredFlag;
			}
			break;
		case south:
			if(xPos>0){
				/* not in first field of Matrix*/
				return (p_currField+MAZE_FIELDS_LENGTH_EAST_DIRECTION)->exploredFlag;
			}
			break;
		case west:
			if(yPos>0){
				/* not in last field of Matrix*/
				return (p_currField-1)->exploredFlag;
			}
			break;
	}
	return false;
}
