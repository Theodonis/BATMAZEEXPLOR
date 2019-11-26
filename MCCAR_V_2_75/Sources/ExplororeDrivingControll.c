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


bool exploreDriving(Maze_segments MazeSegmentsToBeDriven, ADC_data_t* adc_data){
	bool drivingFinishedFlag;
//	#if ENABLE_TIMING_CONROLL
//			uint16_t beforDriving;
//			FC1_GetCounterValue(&beforDriving);
//			saveExplorationValue((float)beforDriving, "beforDriving", (*logValCnt)++);
//	#endif
	drivingFinishedFlag = Driving(MazeSegmentsToBeDriven);
	(*adc_data) =	*get_latest_ADC_data();/* get newest ADC after Driving*/
	#if ENABLE_EXPLORE_DATALOG
	#if ENABLE_TIMING_CONROLL
		uint16_t ticksAfterDriving;
		FC1_GetCounterValue(&ticksAfterDriving);
		saveExplorationValue((float)ticksAfterDriving, "ticksAfterDriving", 1);//(*logValCnt)++);
	#endif
	#endif
	return drivingFinishedFlag;
}



byte driveToFrontWall(uint8_t* segmentNumber,ADC_data_t* adc_data){
	static t_genericState state_toWall = gen_initState;
	static Maze_segments Maze_seg;
//	ADC_data_t adc_data;

	switch(state_toWall){
		case gen_initState: /* Set up segments to drive */
			Maze_seg.segments[(*segmentNumber)].SingleSegment = 10;
			Maze_seg.numberOfSegments = ++(*segmentNumber);
			state_toWall = gen_runnigState;
			if(exploreDriving(Maze_seg, adc_data)){
				return ERR_FAILED;
			}
			break;
		case gen_runnigState: /*Drive till wall -> error if segments driven without Walldetection*/
			if(exploreDriving(Maze_seg, adc_data)){
				state_toWall=gen_initState;
				return ERR_FAILED;
			}else if(adc_data->raw_Values.raw_MiddleL < 55000){
				state_toWall=gen_deinitState;
				setStopFlag();
			}
			break;
		case gen_deinitState: /* Finish driving nd return Ok if finished*/
			if(exploreDriving(Maze_seg, adc_data)){
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

byte driveToUnexpBranch(uint8_t* segmentNumber,ADC_data_t* adc_data, t_directions* currentTargetOrientation, t_directions* wayHist, t_mazeFieldData currentMazeFieldData){
	static t_returToBranchState state_toUnexp = retBra_initState;
	static Maze_segments Maze_seg;
	static uint16_t waitTicksCnt = 1;

	switch(state_toUnexp){
		case retBra_initState: /* Set up segments to drive */
			Maze_seg.segments[(*segmentNumber)].SingleSegment = 10;
			Maze_seg.numberOfSegments = ++(*segmentNumber);
			state_toUnexp = retBra_runnigState;
			if(exploreDriving(Maze_seg, adc_data)){
				return ERR_FAILED;
			}
			break;
		case retBra_runnigState: /*Drive till unexplored branch or wall -> error if segments driven without Walldetection*/
			if(exploreDriving(Maze_seg, adc_data)){
				state_toUnexp = retBra_initState;
				return ERR_FAILED;
			}else if(*currentTargetOrientation != get_wallOrientation(*wayHist,behind)){
				setStopFlag();/*curve in wayHist*/
				state_toUnexp = retBra_curvState;
			}else if(adc_data->raw_Values.raw_MiddleL < 55000){
				setStopFlag();
				state_toUnexp=retBra_ErrorState; /*Error: in wayHist was no wall!*/
			}else if(currentMazeFieldData.hasUnexploredBranchFlag){
				setStopFlag();/* back at last Branch*/
				state_toUnexp = retBra_unexploredBranchState;
			}
			break;

		case retBra_waitState:  /* drive some more time to start curve in middle of field*/
			if(exploreDriving(Maze_seg, adc_data)){
				waitTicksCnt = 1; /*set to 1 because Driving was even called since set to waitState */
				state_toUnexp = retBra_initState;
				return ERR_FAILED;
			}else if(waitTicksCnt*DT>=EXPLOR_DRIVE_TIME_IN_KNOWN_FIELD_TO_STOPP_MIDDLED){
				state_toUnexp 	= retBra_curvState;
				waitTicksCnt 	= 1;
				setStopFlag();
			}else if(adc_data->raw_Values.raw_MiddleL < 55000){ /*still watch out for e frontwall to don't crash */
				state_toUnexp 	= retBra_curvState;
				waitTicksCnt 	= 1;
				setStopFlag();
			}else{
				state_toUnexp 	= retBra_waitState;
				waitTicksCnt++;
			}
			break;
		case retBra_curvState: /* Finish driving nd return Ok if finished*/
			if(exploreDriving(Maze_seg, adc_data)){
				if(get_wallOrientation(*currentTargetOrientation,left)==get_wallOrientation(*wayHist,behind)){
					state_toUnexp = retBra_turnLeft;
				}else if(get_wallOrientation(*currentTargetOrientation,right)==get_wallOrientation(*wayHist,behind)){
					state_toUnexp = retBra_turnRight;
				}else{
					state_toUnexp = retBra_initState;
					return ERR_FAILED;
				}
			}

			break;
		case retBra_turnLeft:
			/*turn left 90� */
			switch(turn90(segmentNumber, currentTargetOrientation, left)){
				case ERR_BUSY:
					state_toUnexp = retBra_turnLeft;
					break;
				case ERR_FAILED:
					state_toUnexp =  retBra_initState;
					return ERR_FAILED;
				case ERR_OK:
					state_toUnexp= retBra_runnigState;
					break;
			}
			break;

		case retBra_turnRight:
			/*turn right 90� */
			switch(turn90(segmentNumber, currentTargetOrientation, right)){
				case ERR_BUSY:
					state_toUnexp = retBra_turnRight;
					break;
				case ERR_FAILED:
					state_toUnexp =  retBra_initState;
					return ERR_FAILED;
				case ERR_OK:
					state_toUnexp= retBra_runnigState;
					break;
			}
			break;

		case retBra_unexploredBranchState:
			  /* drive some more time to start curve in middle of field*/
			if(exploreDriving(Maze_seg, adc_data)){
				waitTicksCnt = 1; /*set to 1 because Driving was even called since set to waitState */
				state_toUnexp = retBra_initState;
				return ERR_FAILED;
			}else if(waitTicksCnt*DT>=EXPLOR_DRIVE_TIME_IN_KNOWN_FIELD_TO_STOPP_MIDDLED){
				state_toUnexp 	= retBra_deinitState;
				waitTicksCnt 	= 1;
				setStopFlag();
			}else if(adc_data->raw_Values.raw_MiddleL < 55000){ /*still watch out for e frontwall to don't crash */
				state_toUnexp 	= retBra_deinitState;
				waitTicksCnt 	= 1;
				setStopFlag();
			}else{
				state_toUnexp 	= retBra_unexploredBranchState;
				waitTicksCnt++;
			}
			break;


		case retBra_deinitState: /* stop in driving return ok -> Back on a field with unexplored branch */
			if(exploreDriving(Maze_seg, adc_data)){
				state_toUnexp = retBra_initState;
				return ERR_OK;
			}
			break;
		case retBra_ErrorState: /* finish driving and return error*/
			if(exploreDriving(Maze_seg, adc_data)){
				state_toUnexp = retBra_initState;
				return ERR_FAILED;
			}
			break;
		default:
			state_toUnexp = retBra_initState;
			break;
	}
	return ERR_BUSY;
}

byte driveToBranch(uint8_t* segmentNumber, t_dir dir,ADC_data_t* adc_data){
	static t_genericState state_toBranch = gen_initState;
	static Maze_segments Maze_seg;
	static uint16_t waitTicksCnt = 1;

//	ADC_data_t adc_data;

	switch(state_toBranch){
		case gen_initState: /* Set up segments to drive */
			Maze_seg.segments[(*segmentNumber)].SingleSegment = 10;
			Maze_seg.numberOfSegments = ++(*segmentNumber);
			state_toBranch = gen_runnigState;

			waitTicksCnt = 1;

			/*put on I_LED's to measure distance correct at frist ADC_Cal in Driving*/
			//I_LED_R_SetVal();I_LED_L_SetVal();I_LED_ML_SetVal();
			//->should be done from Application

			//don't call yet to because of I_LED's not read -> no more problem because use of newest adc in function
			if(exploreDriving(Maze_seg, adc_data)){
				return ERR_FAILED;
			}
			break;
		case gen_runnigState: /*Drive till branch
		 -> error if segments finished driven without branch detected
		 -> error if front wall bevore branch*/
			if(exploreDriving(Maze_seg, adc_data)){
				state_toBranch=gen_initState;
				return ERR_FAILED;
			}else if(adc_data->raw_Values.raw_MiddleL < 55000){
				/*Error but Driving must be finished */
				state_toBranch=gen_ErrorState;
				setStopFlag();
			}else if(dir==left&&(adc_data->mm_Values.mm_Left>90)){
				/* Branch detected Finish Driving */
				state_toBranch = gen_waitState;//gen_deinitState; // ev. go to wait state for some cycles to be in midle of branch
//				setStopFlag();
			}else if(dir==right&&(adc_data->mm_Values.mm_Right>90)){
				/* Branch detected Finish Driving */
				state_toBranch = gen_waitState;//gen_deinitState; // ev. go to wait state for some cycles to be in midle of branch
//				setStopFlag();
			}

			break;
		case gen_waitState:  /* drive some more steps to stop in middle of Branch*/
			if(exploreDriving(Maze_seg, adc_data)){
				waitTicksCnt = 1; /*set to 1 because Driving was even called since set to waitstait */
				state_toBranch = gen_initState;
				return ERR_FAILED;
			}else if(waitTicksCnt*DT>=EXPLOR_DRIVE_TIME_AFTER_BRANCHDETECT){
				state_toBranch 	= gen_deinitState;
				waitTicksCnt 	= 1;
				setStopFlag();
			}else if(adc_data->raw_Values.raw_MiddleL < 55000){ /*still watch out for e frontwall to don't crash */
				state_toBranch 	= gen_deinitState;
				waitTicksCnt 	= 1;
				setStopFlag();
			}else{
				state_toBranch 	= gen_waitState;
				waitTicksCnt++;
			}
			break;
		case gen_deinitState: /* Finish driving nd return Ok if finished*/
			if(exploreDriving(Maze_seg, adc_data)){
				state_toBranch = gen_initState;
				return ERR_OK;
			}
			break;
		case gen_ErrorState: /* finish driving and return Error*/
			if(exploreDriving(Maze_seg, adc_data)){
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


//ToDo: is an internal Wrapper function-> used to deside if reinit or not... -> should be fixed
byte turn90Intern(uint8_t* segmentNumber, t_directions* currentOrientation, t_dir dir, bool reinit){
	static t_genericState state_turn90 = gen_initState;
	static Maze_segments Maze_seg;
	ADC_data_t adc_data;

	switch (state_turn90) {
		case gen_initState:
			if(reinit){
				reinit_Drving(true);

				Maze_seg.numberOfSegments = 1;
				(*segmentNumber) = 1;
				if(dir == left){
					Maze_seg.segments[0].SingleSegment = 900;
				}else if(dir == right){
					Maze_seg.segments[0].SingleSegment = -900;
				}else{
					state_turn90 = gen_initState;
					return ERR_FAILED;
				}
				Maze_seg.segments[1].SingleSegment = 0; /* stop after this */
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
			if(dir==left){
				if(*currentOrientation==north){
					(*currentOrientation) = west;
				}else{
					(*currentOrientation)--;
				} /* ubdate orientation */
			}else if(dir==right){
				if(*currentOrientation==west){
					(*currentOrientation) = north;
				}else{
					(*currentOrientation)++;
				}/* ubdate orientation */
			}
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

byte turn90(uint8_t* segmentNumber, t_directions* currentOrientation, t_dir dir){
	return turn90Intern(segmentNumber, currentOrientation, dir, true);
}


byte turn180(uint8_t* segmentNumber, t_directions* currentOrientation, t_dir dir){
	static t_genericState state_turn180 = gen_initState;
	static Maze_segments Maze_seg;
	ADC_data_t adc_data;

	bool reinit = true;
	switch (state_turn180) {
		case gen_initState:
			if(reinit){
				reinit_Drving(true);

				Maze_seg.numberOfSegments = 2;
				(*segmentNumber) = 2;
				if(dir == left){
					Maze_seg.segments[0].SingleSegment = 900;
					Maze_seg.segments[1].SingleSegment = 900;
				}else if(dir == right){
					Maze_seg.segments[0].SingleSegment = -900;
					Maze_seg.segments[1].SingleSegment = -900;
				}else{
					state_turn180 = gen_initState;
					return ERR_FAILED;
				}
				Maze_seg.segments[2].SingleSegment = 0; /* stop after this */
			}else{
				if(dir == left){
					Maze_seg.segments[*segmentNumber].SingleSegment = 900;
					Maze_seg.segments[++(*segmentNumber)].SingleSegment = 900;
				}else if(dir == right){
					Maze_seg.segments[*segmentNumber].SingleSegment = -900;
					Maze_seg.segments[++(*segmentNumber)].SingleSegment = -900;
				}else{
					state_turn180 = gen_initState;
					return ERR_FAILED;
				}
				Maze_seg.numberOfSegments = ++(*segmentNumber);
			}
			if(exploreDriving(Maze_seg, &adc_data)){
				state_turn180 = gen_initState;
				return ERR_FAILED;
			}

			state_turn180 = gen_runnigState;
			break;

		case gen_runnigState:
			if(exploreDriving(Maze_seg, &adc_data)){
				state_turn180= gen_deinitState;
			}
			break;

		case gen_deinitState:
			(*currentOrientation)=get_wallOrientation(*currentOrientation,behind);
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

//	switch(state_turn180){
//	 	 case gen_initState: /* is not used as init -> do ing 90� turn */
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
//		 case gen_runnigState: /* turn again 90� */
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




