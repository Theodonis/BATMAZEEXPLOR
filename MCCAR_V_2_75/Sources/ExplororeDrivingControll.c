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
**     	Method      : bool exploreDriving(Maze_segments MazeSegmentsToBeDriven, ADC_data_t* adc_data)
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
**     	byte driveToFrontWall(uint8_t* p_segNumb,ADC_data_t* p_adc_data)
**
**
**     	@brief	Driving till a front wall is detected. Stop before the wall.
**
**     	@param	p_segNumb: 		Pointer to segmentNumber counter. Used to set always the
**     							correct Maze_seg.numberOfSegments to drive
**     			adc_data: 		Pointer to adc_data element to write newest adc data in it
**
**		@return Error code, possible codes:
**                           ERR_OK - stopped before wall
**                           ERR_FAILED - no wall reached while Driving 10 Fields
**                           ERR_BUSY - still driving -> call again in next cycle
**
**
** ===================================================================
*/
byte driveToFrontWall(uint8_t* p_segNumb,ADC_data_t* p_adc_data){
	static t_genericState state_toWall = gen_initState;
	static Maze_segments Maze_seg;
//	ADC_data_t adc_data;

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
			}
			break;
		case gen_deinitState: /* Finish driving nd return Ok if finished*/
			if(exploreDriving(Maze_seg, p_adc_data)){
				state_toWall = gen_initState;
				return ERR_OK;
			}
			break;
		case gen_waitState:  /* not used in this case*/
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
**     			p_currField: 	Pointer to currentField to decide if its a branch or a turn in history
**     			p_curOrient: 	Pointer to current target orientation
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

		case retBra_runnigState: /*Drive till unexplored branch or wall -> error if segments driven without Frontwalldetection*/
			switch(driveToBranch(p_segNumb,p_adc_data,p_currField,p_curOrient)){
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

//			if(exploreDriving(Maze_seg, adc_data)){
//				state_toUnexp = retBra_initState;
//				return ERR_FAILED;
//			}else if(adc_data->raw_Values.raw_MiddleL < MAX_FRONTDIST_TO_WALL_RAW){
//					state_toUnexp=retBra_ErrorState;
//					setStopFlag();
//			}else if(currentMazeFieldData.hasUnexploredBranchFlag){
//				//setStopFlag();/* back at last Branch*/
//				state_toUnexp = retBra_unexploredBranchState;
//			}else if(*currentTargetOrientation != get_wallOrientation(currentMazeFieldData.enterDirection,behind)){
//				//setStopFlag();/*curve in wayHist*/
//				state_toUnexp = retBra_waitState;
//			 /*Error: in wayHist was no wall!*/
//			}
			break;

		  /* drive some more time to start curve in middle of field*/
//			if(exploreDriving(Maze_seg, adc_data)){
//				waitTicksCnt = 1; /*set to 1 because Driving was even called since set to waitState */
//				state_toUnexp = retBra_initState;
//				return ERR_FAILED;
//			}else if(waitTicksCnt*DT>=EXPLOR_DRIVE_TIME_IN_KNOWN_FIELD_TO_STOPP_MIDDLED){
//				state_toUnexp 	= retBra_curvState;
//				waitTicksCnt 	= 1;
//				setStopFlag();
//			}else if(adc_data->raw_Values.raw_MiddleL < MAX_FRONTDIST_TO_WALL_RAW){ /*still watch out for e frontwall to don't crash */
//				state_toUnexp 	= retBra_curvState;
//				waitTicksCnt 	= 1;
//				setStopFlag();
//			}else{
//				state_toUnexp 	= retBra_waitState;
//				waitTicksCnt++;
//			}
//			break;
//		case retBra_curvState: /* Finish driving nd return Ok if finished*/
//			if(exploreDriving(Maze_seg, adc_data)){
//				if(get_wallOrientation(*currentTargetOrientation,left)==get_wallOrientation(currentMazeFieldData.enterDirection,behind)){
//					state_toUnexp = retBra_turnLeft;
//				}else if(get_wallOrientation(*currentTargetOrientation,right)==get_wallOrientation(currentMazeFieldData.enterDirection,behind)){
//					state_toUnexp = retBra_turnRight;
//				}else{
//					state_toUnexp = retBra_initState;
//					return ERR_FAILED;
//				}
//			}
//
//			break;
//
//		case retBra_unexploredBranchState:
//			  /* drive some more time to start curve in middle of field*/
//			if(exploreDriving(Maze_seg, adc_data)){
//				waitTicksCnt = 1; /*set to 1 because Driving was even called since set to waitState */
//				state_toUnexp = retBra_initState;
//				return ERR_FAILED;
//			}else if(waitTicksCnt*DT>=EXPLOR_DRIVE_TIME_IN_KNOWN_FIELD_TO_STOPP_MIDDLED){
//				state_toUnexp 	= retBra_deinitState;
//				waitTicksCnt 	= 1;
//				setStopFlag();
//			}else if(adc_data->raw_Values.raw_MiddleL < MAX_FRONTDIST_TO_WALL_RAW){ /*still watch out for e frontwall to don't crash */
//				state_toUnexp 	= retBra_deinitState;
//				waitTicksCnt 	= 1;
//				setStopFlag();
//			}else{
//				state_toUnexp 	= retBra_unexploredBranchState;
//				waitTicksCnt++;
//			}
//			break;
//
//
//		case retBra_deinitState: /* stop in driving return ok -> Back on a field with unexplored branch */
//			if(exploreDriving(Maze_seg, adc_data)){
//				state_toUnexp = retBra_initState;
//				return ERR_OK;
//			}
//			break;
//		case retBra_ErrorState: /* finish driving and return error*/
//			if(exploreDriving(Maze_seg, adc_data)){
//				state_toUnexp = retBra_initState;
//				return ERR_FAILED;
//			}
//			break;

		case retBra_ErrorState: /*not used till now*/
			state_toUnexp = retBra_initState;
			return ERR_FAILED;
		case retBra_waitState: /*not used till now*/
		case retBra_deinitState: /*not used till now*/
		default:
			state_toUnexp = retBra_initState;
			break;
	}

//	saveExplorationValue(state_toUnexp,"Returnstate", 10);
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
byte driveToBranch(uint8_t* p_segNumb, ADC_data_t* p_adc_data, t_mazeFieldData* p_currField, t_directions* p_curOrient){
	static t_genericState state_toBranch = gen_initState;
	static Maze_segments Maze_seg;
	static uint16_t waitTicksCnt = 1;

//	ADC_data_t adc_data;

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

			}else if(*p_curOrient != get_wallOrientation(p_currentField->enterDirection, behind)){
				/* no more driving against way history */
				state_toBranch = gen_waitState; //driving till middle of field
			}
#if BLE_DRIVETOBRANCH_LOG
//					IntOverBLE(state_toBranch);
//			IntOverBLE(*segmentNumber);
//			IntOverBLE(Maze_seg.segments[(*segmentNumber)-1].SingleSegment);
#endif

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
//					IntOverBLE(state_toBranch);
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
**     	byte turn90Intern(uint8_t* segmentNumber, t_directions* currentOrientation, t_dir dir, bool reinit)
**
**
**     	@brief	turn on spot 90 degree with decision if driving should be reinit at beginning
**     			(only for internal use in ExploreDriving.c)
**
**     	@param	segmentNumber: 				Pointer to segmentNumber counter. Used to set always the
**     										correct Maze_seg.numberOfSegments to drive
**     			currentOrientation: 		Pointer to current target orientation to update after finished
**     			dir:					 	turn direction
**     			reinit:						reinit driving at begining if ture
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
//ToDo: is an internal Wrapper function-> used to deside if reinit or not... -> should be fixed
byte turn90Intern(uint8_t* segmentNumber, t_directions* currentOrientation, t_dir dir, bool reinit){
	static t_genericState state_turn90 = gen_initState;
	static Maze_segments Maze_seg;
	ADC_data_t adc_data;

	switch (state_turn90) {
		case gen_initState:
			if(reinit){
				reinit_Drving(true);

				Maze_seg.numberOfSegments = 2;
				(*segmentNumber) = 2;
				if(dir == left){
					Maze_seg.segments[0].SingleSegment = 900;
				}else if(dir == right){
					Maze_seg.segments[0].SingleSegment = -900;
				}else{
					state_turn90 = gen_initState;
					return ERR_FAILED;
				}
				Maze_seg.segments[1].SingleSegment = 10; /* stop after this */
			}else{
				if(dir == left){
					Maze_seg.segments[*segmentNumber].SingleSegment = 900;
				}else if(dir == right){
					Maze_seg.segments[*segmentNumber].SingleSegment = -900;
				}else{
					state_turn90 = gen_initState;
					return ERR_FAILED;
				}
				Maze_seg.numberOfSegments = ++(*segmentNumber);
			}
			if(exploreDriving(Maze_seg, &adc_data)){
				state_turn90 = gen_initState;
				return ERR_FAILED;
			}
			Maze_seg.numberOfSegments = 1;
			(*segmentNumber) = 1;

			state_turn90 = gen_runnigState;
			break;

		case gen_runnigState:
			if(exploreDriving(Maze_seg, &adc_data)){
				state_turn90= gen_deinitState;
			}
			break;

		case gen_deinitState:
//			if(exploreDriving(Maze_seg, &adc_data)){
				/* be sure is terminated, v ref=0 and Segmenttime at end*/
//			}
			/*ubdate current target oreintation */
			*currentOrientation = get_wallOrientation(*currentOrientation,dir);

//			if(dir==left){
//				if(*currentOrientation==north){
//					(*currentOrientation) = west;
//				}else{
//					(*currentOrientation)--;
//				} /* ubdate orientation */
//			}else if(dir==right){
//				if(*currentOrientation==west){
//					(*currentOrientation) = north;
//				}else{
//					(*currentOrientation)++;
//				}/* ubdate orientation */
//			}
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
**     	byte turn90(uint8_t* p_segNumb, t_directions* p_curOrient, t_dir dir)
**
**
**     	@brief	turn on spot 90 degree with and reinit driving at begin
**
**     	@param	p_segNumb: 				Pointer to segmentNumber counter. Used to set always the
**     										correct Maze_seg.numberOfSegments to drive
**     			p_curOrient: 		Pointer to current target orientation to update after finished
**     			dir:					 	turn direction
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
	return turn90Intern(p_segNumb, p_curOrient, dir, true);
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
	static uint16_t waitTicksCnt = 1;

	bool reinit = true;
	switch (state_turn180) {
		case gen_initState:
			if(reinit){
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
				/*to init Driving() for strait drive after turn*/
				Maze_seg.segments[2].SingleSegment = 10;
			}else{
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
			}

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
			if(waitTicksCnt*DT>=EXPLOR_DRIVE_TIME_IN_KNOWN_FIELD_TO_STOPP_MIDDLED){
				state_turn180 	= gen_deinitState;
				waitTicksCnt 	= 1;/*set to 1 because Driving was even called since set to waitState */
			}else{
				state_turn180 	= gen_waitState;
				waitTicksCnt++;
			}
			break;
		case gen_ErrorState: /* not used in this case*/
		default:
			state_turn180 = gen_initState;
			break;
	}
	return ERR_BUSY;

//	switch(state_turn180){
//	 	 case gen_initState: /* is not used as init -> do ing 90° turn */
//			 switch(turn90Intern(segmentNumber, currentOrientation, dir, true)){
//				 case ERR_BUSY:
//					 state_turn180 = gen_initState;
//					 break;
//				 case ERR_FAILED:
//					 state_turn180 = gen_initState;
//					 return ERR_FAILED;
//				 case ERR_OK:
//					 state_turn180 = gen_runnigState;
//					 break;
//			 }
//			 break;
//		 case gen_runnigState: /* turn again 90° */
//			 switch(turn90Intern(segmentNumber, currentOrientation, dir, true)){
//				 case ERR_BUSY:
//					 state_turn180 = gen_runnigState;
//					 break;
//				 case ERR_FAILED:
//					 state_turn180 = gen_initState;
//					 return ERR_FAILED;
//				 case ERR_OK:
//					 state_turn180 = gen_deinitState;
//					 break;
//			 }
//			 break;
//		 case gen_deinitState:
//			 state_turn180 = gen_initState;
//			 //reinit_Drving(true);
//			 //(*segmentNumber)=0;
//			 return ERR_OK;
//		 case gen_waitState:  /* not used in this case*/
//		 case gen_ErrorState: /* not used in this case*/
//		 default:
//			 state_turn180 = gen_initState;
//			 break;
//	}
//	return ERR_BUSY;
}




