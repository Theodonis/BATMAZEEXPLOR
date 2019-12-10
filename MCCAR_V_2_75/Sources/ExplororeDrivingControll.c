/*
 * ExplororeDrivingControll.c
 *
 *  Created on: 15.11.2019
 *      Author: Theo
 */


#include "ExplororeDrivingControll.h"
#include "Driving.h"
#include "DrivingExplore_Interface.h"
#include "ExploreConfig.h"
#include "TargetInField_Position.h"
#include "stdbool.h"


#include "I_LED_R.h"
#include "I_LED_L.h"
#include "I_LED_MR.h"
#include "I_LED_ML.h"

#if ENABLE_EXPLORE_DATALOG
	#include "Logging.h"
#endif

#if ENABLE_TIMING_CONROLL
	#include "FC1.h"
#endif



/*
** ===================================================================
**     	Method      : bool exploreDriving(Maze_segments MazeSegmentsToBeDriven, ADC_data_t* p_adc_data)
**
**     	@brief	wrapper for Driving(). Add timing log and returns the newest adc values
**
**     	@param	MazeSegmentsToBeDriven: Segment to drive (turn or strait or ...)
**     			p_adc_data: Pointer to adc_data element to write newest adc data in it
**
**		@return True: 	if driving segments is finished or stop after stop flag
**				False: 	if driving is still on work
**
**
** ===================================================================
*/
bool exploreDriving(Maze_segments MazeSegmentsToBeDriven, ADC_data_t* p_adc_data){
	bool drivingFinishedFlag;
	drivingFinishedFlag = Driving(MazeSegmentsToBeDriven);
	(*p_adc_data) =	*get_latest_ADC_data();/* get newest ADC after Driving*/
	#if ENABLE_EXPLORE_DATALOG
	#if ENABLE_TIMING_CONROLL
		uint16_t ticksAfterDriving;
		FC1_GetCounterValue(&ticksAfterDriving);
		saveExplorationValue((float)ticksAfterDriving, "ticksAfterDriving", 1);//(*logValCnt)++);
	#endif
	#endif
	return drivingFinishedFlag;
}


/*
** ===================================================================
**     	byte driveToFrontWall(uint8_t* p_segNumb,ADC_data_t* p_adc_data, t_directions curOrient, t_mazeFieldData* p_currField, uint8_t xPos, uint8_t yPos)
**
**
**     	@brief	Driving till a front wall is detected or the next Field was even explored. Stop before the wall.
**
**     	@param	p_segNumb: 		Pointer to segmentNumber counter. Used to set always the
**     							correct Maze_seg.numberOfSegments to drive
**     			adc_data: 		Pointer to adc_data element to write newest adc data in it
**     			curOrient: 		current orientation of MC-Car
**     			p_currField: 	Pointer to currentField to decide if its a branch or a turn in history
**				xPos: 			current X-Index in MazaData-Matrix
**				yPos: 			current Y-Index in MazaData-Matrix
**
**		@return Error code, possible codes:
**                           ERR_OK - stopped before wall or explored Field
**                           ERR_FAILED - no wall reached while Driving 10 Fields
**                           ERR_BUSY - still driving -> call again in next cycle
**
**
** ===================================================================
*/
byte driveToFrontWall(uint8_t* p_segNumb,ADC_data_t* p_adc_data, t_directions curOrient, t_mazeFieldData* p_currField, uint8_t xPos, uint8_t yPos){
	static t_genericState state_toWall = gen_initState;
	static Maze_segments Maze_seg;
	static uint16_t waitTicksCnt = 1;

	switch(state_toWall){
		case gen_initState: /* Set up segments to drive */
			Maze_seg.segments[(*p_segNumb)].SingleSegment = 10;
			Maze_seg.numberOfSegments = ++(*p_segNumb);
			state_toWall = gen_runnigState;
			if(exploreDriving(Maze_seg, p_adc_data)){
				return ERR_FAILED;
			}
			break;
		case gen_runnigState: /*Drive till wall -> error if segments driven without Walldetection*/
			if(exploreDriving(Maze_seg, p_adc_data)){
				state_toWall=gen_initState;
				return ERR_FAILED;
			}else if(p_adc_data->mm_Values.mm_MiddleL < MAX_FRONTDIST_TO_WALL_MM){
				state_toWall=gen_deinitState;
				setStopFlag();
			}else if(get_isExploredFieldInFront(p_currField, curOrient, xPos, yPos)){
				/* next Field was even explored*/
				state_toWall=gen_waitState; // continue driving till front of wall
			}

			break;
		case gen_deinitState: /* Finish driving nd return Ok if finished*/
			if(exploreDriving(Maze_seg, p_adc_data)){
				state_toWall = gen_initState;
				return ERR_OK;
			}
			break;
		case gen_waitState:  /* drive some more steps to stop in front of wall (if exist or not)*/
			if(exploreDriving(Maze_seg, p_adc_data)){
				waitTicksCnt = 1;
				state_toWall = gen_initState;
				return ERR_FAILED;
			}else if(waitTicksCnt*DT>=EXPLOR_DRIVE_TIME_TO_STOPP_BEFORE_EXPLORED_FIELD){
				state_toWall 	= gen_deinitState;
				waitTicksCnt 	= 1;/*set to 1 because Driving was even called since set to waitState */
				setStopFlag();
			}else if(p_adc_data->mm_Values.mm_MiddleL < MAX_FRONTDIST_TO_WALL_MM){ /*still watch out for e frontWall to don't crash */
				state_toWall 	= gen_deinitState;
				waitTicksCnt 	= 1;
				setStopFlag();
			}else{
				state_toWall 	= gen_waitState;
				waitTicksCnt++;
			}

			break;
		case gen_ErrorState:/* not used in this case*/
		default:
			state_toWall = gen_initState;
			break;
	}
	return ERR_BUSY;
}


/*
** ===================================================================
**     	byte driveToUnexpBranch(uint8_t* p_segNumb,ADC_data_t* p_adc_data, t_directions* p_curOrient, t_mazeFieldData* p_currField))
**
**
**     	@brief	Drive back until the last unexplored field or to start
**
**
**     	@param	p_segNumb: 		Pointer to segmentNumber counter. Used to set always the
**     							correct Maze_seg.numberOfSegments to drive
**     			p_adc_data:		Pointer to adc_data element to write newest adc data in it
**     			p_curOrient: 	Pointer to current target orientation
**     			p_currField: 	Pointer to currentField to decide if its a branch or a turn in history
**
**		@return Error code, possible codes:
**                           ERR_OK - stopped at unexplored Branch or at startfield
**                           ERR_FAILED - Failed in here or in lower Function
**                           ERR_BUSY - still driving -> call again in next cycle
**
**
** ===================================================================
*/
byte driveToUnexpBranch(uint8_t* p_segNumb,ADC_data_t* p_adc_data, t_directions* p_curOrient, t_mazeFieldData* p_currField){
	static t_returToBranchState state_toUnexp = retBra_initState;
	static t_dir dir_toggel = left; // to toggle 180 turn direction -> theta angle should not be to high
	static uint16_t waitTicksCnt = 1;

	switch(state_toUnexp){

	case retBra_initState:
			#if BLE_DRIVETOUNEXPLOREDBRANCH_LOG
				IntOverBLE(state_toUnexp);
			#endif
			state_toUnexp = retBra_calcNextStep;
			static uint16_t waitTicksCnt = 1;
			break;

		case retBra_calcNextStep:

			#if BLE_DRIVETOUNEXPLOREDBRANCH_LOG
				IntOverBLE(state_toUnexp);
			#endif
			if(p_currField->hasUnexploredBranchFlag){
				state_toUnexp=retBra_initState;
				return ERR_OK;
			}
			if(p_currField->enterDirection == get_wallOrientation(*p_curOrient,left)){
					/*to return must driving against wayHist */
				state_toUnexp = retBra_turnRight;

			}else if(p_currField->enterDirection == get_wallOrientation(*p_curOrient,right)){
					/*to return must driving against wayHist */
				state_toUnexp = retBra_turnLeft;

			}else if(p_currField->enterDirection == get_wallOrientation(*p_curOrient,behind)){
						/*to return must driving against wayHist */
					state_toUnexp = retBra_runnigState;
				#if BLE_DRIVETOUNEXPLOREDBRANCH_LOG
					IntOverBLE(state_toUnexp);
				#endif

			}else if(p_currField->enterDirection == *p_curOrient){
						/*to return must driving against wayHist */
				state_toUnexp = retBra_turnAround;

				#if BLE_DRIVETOUNEXPLOREDBRANCH_LOG
					IntOverBLE(state_toUnexp);
				#endif

			}else{
					state_toUnexp = retBra_initState;
					return ERR_FAILED;
			}
			break;

		case retBra_turnLeft:
			/*turn left 90° */
			switch(turn90(p_segNumb, p_curOrient, left)){
				case ERR_BUSY:
					state_toUnexp = retBra_turnLeft;
					break;
				case ERR_FAILED:
					state_toUnexp =  retBra_initState;
					return ERR_FAILED;
				case ERR_OK:
					#if BLE_DRIVETOUNEXPLOREDBRANCH_LOG
						IntOverBLE(state_toUnexp);
					#endif
					state_toUnexp= retBra_calcNextStep;
					break;
			}
			break;

		case retBra_turnRight:
			/*turn right 90° */
			switch(turn90(p_segNumb, p_curOrient, right)){
				case ERR_BUSY:
					state_toUnexp = retBra_turnRight;
					break;
				case ERR_FAILED:
					state_toUnexp =  retBra_initState;
					return ERR_FAILED;
				case ERR_OK:
					#if BLE_DRIVETOUNEXPLOREDBRANCH_LOG
						IntOverBLE(state_toUnexp);
					# endif
					state_toUnexp= retBra_calcNextStep;
					break;
			}
			break;


		case retBra_turnAround:

			/*turn left 180° */
			switch(turn180(p_segNumb, p_curOrient, dir_toggel)){
				case ERR_BUSY:
					state_toUnexp = retBra_turnAround;
					break;
				case ERR_FAILED:
					#if BLE_DRIVETOUNEXPLOREDBRANCH_LOG
						IntOverBLE(state_toUnexp);
						IntOverBLE(0xFF);
					#endif
					state_toUnexp =  retBra_initState;
					return ERR_FAILED;
				case ERR_OK:
					#if BLE_DRIVETOUNEXPLOREDBRANCH_LOG
						IntOverBLE(state_toUnexp);
					#endif
					state_toUnexp= retBra_calcNextStep;
					dir_toggel++;
					if(dir_toggel>1){
						dir_toggel = left;
					}
					break;
			}
			break;

		case retBra_runnigState:
			/*Drive till unexplored branch or wall -> error if segments driven without Frontwalldetection*/
			switch(driveToBranch(p_segNumb,p_adc_data,p_curOrient,p_currField)){
				case ERR_OK:
					#if BLE_DRIVETOUNEXPLOREDBRANCH_LOG
						IntOverBLE(state_toUnexp);
					#endif
					state_toUnexp = retBra_calcNextStep;
					break;
				case ERR_FAILED:
					state_toUnexp = retBra_initState;
					return ERR_FAILED;
				case ERR_BUSY:
					#if BLE_DRIVETOUNEXPLOREDBRANCH_LOG
						IntOverBLE(20);
					#endif
					state_toUnexp = retBra_runnigState;
					break;
			}
			break;

		case retBra_ErrorState: /*not used till now*/
			state_toUnexp = retBra_initState;
			return ERR_FAILED;
		case retBra_waitState: /*not used till now*/
		case retBra_deinitState: /*not used till now*/
		default:
			state_toUnexp = retBra_initState;
			break;
	}
	return ERR_BUSY;
}


/*
** ===================================================================
**     	byte driveToBranch(uint8_t* p_segNumb, ADC_data_t* p_adc_data, t_mazeFieldData* p_currField, t_directions* p_curOrient)
**
**
**     	@brief	For returning in way history:
**     			Driving strait till a curve in way history,
**     			a front wall is detected or a unexplored branch is reached
**
**     	@param	p_segNumb: 	 Pointer to segmentNumber counter. Used to set always the
**     						 correct Maze_seg.numberOfSegments to drive
**     			p_adc_data:	 Pointer to adc_data element to write newest adc data in it
**     			p_currField: Pointer to currentField to decide if its a branch or a turn in history
**     			p_curOrient: pointer to current target orientation
**
**		@return Error code, possible codes:
**                           ERR_OK - 		stopped at unexplored Branch or curv
**                           ERR_FAILED - 	unexpected wall in front,
										 	no wall branch or curve reached while Driving 10 Fields
**                           ERR_BUSY - 	still driving -> call again in next cycle
**
**
** ===================================================================
*/
byte driveToBranch(uint8_t* p_segNumb, ADC_data_t* p_adc_data, t_directions* p_curOrient, t_mazeFieldData* p_currField){
	static t_genericState state_toBranch = gen_initState;
	static Maze_segments Maze_seg;
	static uint16_t waitTicksCnt = 1;

	switch(state_toBranch){
		case gen_initState: /* Set up segments to drive */
			Maze_seg.segments[(*p_segNumb)].SingleSegment = 10;
			Maze_seg.numberOfSegments = ++(*p_segNumb);
			#if BLE_DRIVETOBRANCH_LOG
				IntOverBLE(state_toBranch);
			#endif
			state_toBranch = gen_runnigState;

			waitTicksCnt = 1;

			if(exploreDriving(Maze_seg, p_adc_data)){
				setStopFlag();
				return ERR_FAILED;
			}
			break;
		case gen_runnigState: /*Drive till branch*/
			if(exploreDriving(Maze_seg, p_adc_data)){
				state_toBranch=gen_initState;
				return ERR_FAILED;
			}else if(p_adc_data->mm_Values.mm_MiddleL < MAX_FRONTDIST_TO_WALL_MM){
				/*Error but Driving must be finished */
				#if BLE_DRIVETOBRANCH_LOG
					IntOverBLE(state_toBranch);
				#endif

				setStopFlag();
				state_toBranch=gen_ErrorState;

			}else if(p_currField->hasUnexploredBranchFlag){
				/* Branch detected Finish Driving */
				state_toBranch = gen_waitState; //driving till middle of field

			}else if(*p_curOrient != get_wallOrientation(p_currField->enterDirection, behind)){
				/* no more driving against way history */
				state_toBranch = gen_waitState; //driving till middle of field
			}

			break;
		case gen_waitState:  /* drive some more steps to stop in middle of field*/
			if(exploreDriving(Maze_seg, p_adc_data)){
				waitTicksCnt = 1;
				state_toBranch = gen_initState;
				return ERR_FAILED;
			}else if(waitTicksCnt*DT>=EXPLOR_DRIVE_TIME_IN_KNOWN_FIELD_TO_STOPP_MIDDLED){
				state_toBranch 	= gen_deinitState;
				waitTicksCnt 	= 1;/*set to 1 because Driving was even called since set to waitState */
				setStopFlag();
			}else if(p_adc_data->mm_Values.mm_MiddleL < MAX_FRONTDIST_TO_WALL_MM){ /*still watch out for e frontWall to don't crash */
				state_toBranch 	= gen_deinitState;
				waitTicksCnt 	= 1;
				setStopFlag();
			}else{
				state_toBranch 	= gen_waitState;
				waitTicksCnt++;
			}

			break;
		case gen_deinitState: /* Finish driving nd return Ok if finished*/
			if(exploreDriving(Maze_seg, p_adc_data)){
				state_toBranch = gen_initState;
				return ERR_OK;
			}
			#if BLE_DRIVETOBRANCH_LOG
				IntOverBLE(state_toBranch);
			#endif

			break;
		case gen_ErrorState: /* finish driving and return Error*/
			if(exploreDriving(Maze_seg, p_adc_data)){
				state_toBranch = gen_initState;
				return ERR_FAILED;
			}
			break;
		default:
			state_toBranch = gen_initState;
			break;

	}
	return ERR_BUSY;
}


/*
** ===================================================================
**     	byte turn90(uint8_t* p_segNumb, t_directions* p_curOrient, t_dir dir)
**
**
**     	@brief	turn on spot 90 degree with and reinit driving at begin
**
**     	@param	p_segNumb: 			Pointer to segmentNumber counter. Used to set always the
**     								correct Maze_seg.numberOfSegments to drive
**     			p_curOrient: 		Pointer to current target orientation to update after finished
**     			dir:				turn direction
**
**
**		@return Error code, possible codes:
**                           ERR_OK - 		finished turning
**                           ERR_FAILED - 	unexpected dir value, driving finished before satrter
										 	no wall branch or curve reached while Driving 10 Fields
**                           ERR_BUSY - 	still driving -> call again in next cycle
**
**
** ===================================================================
*/
byte turn90(uint8_t* p_segNumb, t_directions* p_curOrient, t_dir dir){
	static t_genericState state_turn90 = gen_initState;
	static Maze_segments Maze_seg;
	ADC_data_t adc_data;

	switch (state_turn90) {
		case gen_initState:
			#if REINIT_DRIVING_BEFORE_TURN
				reinit_Drving(true);

				Maze_seg.numberOfSegments = 2;
				(*p_segNumb) = 2;
				if(dir == left){
					Maze_seg.segments[0].SingleSegment = 900;
				}else if(dir == right){
					Maze_seg.segments[0].SingleSegment = -900;
				}else{
					state_turn90 = gen_initState;
					return ERR_FAILED;
				}
				/*to init Driving() for strait drive after turn*/
				Maze_seg.segments[1].SingleSegment = 10;
			#else
				if(dir == left){
					Maze_seg.segments[*segmentNumber].SingleSegment = 900;
				}else if(dir == right){
					Maze_seg.segments[*segmentNumber].SingleSegment = -900;
				}else{
					state_turn90 = gen_initState;
					return ERR_FAILED;
				}
				Maze_seg.numberOfSegments = ++(*segmentNumber);
			#endif

			if(exploreDriving(Maze_seg, &adc_data)){
				state_turn90 = gen_initState;
				return ERR_FAILED;
			}
			Maze_seg.numberOfSegments = 1;
			(*p_segNumb) = 1;

			state_turn90 = gen_runnigState;
			break;

		case gen_runnigState:
			if(exploreDriving(Maze_seg, &adc_data)){
				state_turn90= gen_deinitState;
			}
			break;

		case gen_deinitState:
			/*ubdate current target oreintation */
			*p_curOrient = get_wallOrientation(*p_curOrient,dir);
			state_turn90 = gen_initState;
			return ERR_OK;
			break;
		case gen_waitState:  /* not used in this case*/
		case gen_ErrorState: /* not used in this case*/
		default:
			state_turn90 = gen_initState;
			break;
	}
	return ERR_BUSY;
}


/*
** ===================================================================
**     	byte turn180(uint8_t* p_segNumb, t_directions* p_curOrient, t_dir dir)
**
**
**     	@brief	turn on spot 180 degree with reinit driving at begin
**
**     	@param	p_segNumb: 		Pointer to segmentNumber counter. Used to set always the
**     								correct Maze_seg.numberOfSegments to drive
**     			p_curOrient: 		Pointer to current target orientation to update after finished
**     			dir:				turn direction
**
**
**		@return Error code, possible codes:
**                           ERR_OK - 		finished turning
**                           ERR_FAILED - 	unexpected dir value, driving finished before satrter
										 	no wall branch or curve reached while Driving 10 Fields
**                           ERR_BUSY - 	still driving -> call again in next cycle
**
**
** ===================================================================
*/
byte turn180(uint8_t* p_segNumb, t_directions* p_curOrient, t_dir dir){
	static t_genericState state_turn180 = gen_initState;
	static Maze_segments Maze_seg;
	ADC_data_t adc_data;

	bool reinit = true;
	switch (state_turn180) {
		case gen_initState:
			#if REINIT_DRIVING_BEFORE_TURN
				reinit_Drving(true);

				Maze_seg.numberOfSegments = 3;
				(*p_segNumb) = 3;
				if(dir == left){
					Maze_seg.segments[0].SingleSegment = 900;
					Maze_seg.segments[1].SingleSegment = 900;
				}else if(dir == right){
					Maze_seg.segments[0].SingleSegment = -900;
					Maze_seg.segments[1].SingleSegment = -900;
				}else{
					/*ERROR: unexpected parameters*/
					state_turn180 = gen_initState;
					return ERR_FAILED;
				}
				/*to init Driving() for strait segment after turn*/
				Maze_seg.segments[2].SingleSegment = 10;
			#else
				if(dir == left){
					Maze_seg.segments[*p_segNumb].SingleSegment = 900;
					Maze_seg.segments[++(*p_segNumb)].SingleSegment = 900;
				}else if(dir == right){
					Maze_seg.segments[*p_segNumb].SingleSegment = -900;
					Maze_seg.segments[++(*p_segNumb)].SingleSegment = -900;
				}else{
					/*ERROR: unexpected parameters*/
					state_turn180 = gen_initState;
					return ERR_FAILED;
				}
				Maze_seg.numberOfSegments = ++(*p_segNumb);
			#endif

			if(exploreDriving(Maze_seg, &adc_data)){
				/*ERROR: finished bevore started...*/
				state_turn180 = gen_initState;
				return ERR_FAILED;
			}
			Maze_seg.numberOfSegments = 2;
			(*p_segNumb) = 2;
			state_turn180 = gen_runnigState;
			break;

		case gen_runnigState:
			if(exploreDriving(Maze_seg, &adc_data)){
				state_turn180= gen_deinitState;
			}
			break;

		case gen_deinitState:
			(*p_curOrient)=get_wallOrientation(*p_curOrient,behind);
			state_turn180 = gen_initState;
			return ERR_OK;
			break;
		case gen_waitState:  /* not used in this case*/
		case gen_ErrorState: /* not used in this case*/
		default:
			state_turn180 = gen_initState;
			break;
	}
	return ERR_BUSY;
}




