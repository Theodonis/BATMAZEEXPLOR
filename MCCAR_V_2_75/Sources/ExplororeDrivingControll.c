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
#include "stdbool.h"


#include "I_LED_R.h"
#include "I_LED_L.h"
#include "I_LED_MR.h"
#include "I_LED_ML.h"

#ifdef ENABLE_DATALOG
	#include "Logging.h"
#endif

#ifdef ENABLE_TIMING_CONROLL
	#include "FC1.h"
#endif


bool exploreDriving(Maze_segments MazeSegmentsToBeDriven, ADC_data_t* adc_data){
	bool drivingFinishedFlag;
//	#ifdef ENABLE_TIMING_CONROLL
//			uint16_t beforDriving;
//			FC1_GetCounterValue(&beforDriving);
//			saveExplorationValue((float)beforDriving, "beforDriving", (*logValCnt)++);
//	#endif
	drivingFinishedFlag = Driving(MazeSegmentsToBeDriven);
	(*adc_data) =	*get_latest_ADC_data();/* get newest ADC after Driving*/
	#ifdef ENABLE_DATALOG
	#ifdef ENABLE_TIMING_CONROLL
		uint16_t ticksAfterDriving;
		FC1_GetCounterValue(&ticksAfterDriving);
		saveExplorationValue((float)ticksAfterDriving, "ticksAfterDriving", 1);//(*logValCnt)++);
	#endif
	#endif
	return drivingFinishedFlag;
}



byte driveToFrontWall(uint8_t* segmentNumber, uint16_t frontDistance){
	static t_genericState state_toWall = gen_initState;
	static Maze_segments Maze_seg;
	ADC_data_t adc_data;

	switch(state_toWall){
		case gen_initState: /* Set up segments to drive */
			Maze_seg.segments[(*segmentNumber)].SingleSegment = 10;
			Maze_seg.numberOfSegments = ++(*segmentNumber);
			state_toWall = gen_runnigState;
			if(exploreDriving(Maze_seg, &adc_data)){
				return ERR_FAILED;
			}
			break;
		case gen_runnigState: /*Drive till wall -> error if segments driven without Walldetection*/
			if(exploreDriving(Maze_seg, &adc_data)){
				state_toWall=gen_initState;
				return ERR_FAILED;
			}else if(adc_data.raw_Values.raw_MiddleL < 55000){
				state_toWall=gen_deinitState;
				setStopFlag();
			}
			break;
		case gen_deinitState: /* Finish driving nd return Ok if finished*/
			if(exploreDriving(Maze_seg, &adc_data)){
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

byte driveToBranch(uint8_t* segmentNumber, uint16_t frontDistance, float sideDist){
	static t_genericState state_toBranch = gen_initState;
	static Maze_segments Maze_seg;

	ADC_data_t adc_data;

	switch(state_toBranch){
		case gen_initState: /* Set up segments to drive */
			Maze_seg.segments[(*segmentNumber)].SingleSegment = 10;
			Maze_seg.numberOfSegments = ++(*segmentNumber);
			state_toBranch = gen_runnigState;

			/*put on I_LED's to measure distance correct at frist ADC_Cal in Driving*/
			I_LED_R_SetVal();I_LED_L_SetVal();I_LED_ML_SetVal();

			//don't call yet to because of I_LED's not read
//			if(exploreDriving(Maze_seg, &adc_data)){
//				return ERR_FAILED;
//			}
			break;
		case gen_runnigState: /*Drive till branch
		 -> error if segments finished driven without branch detected
		 -> error if front wall bevore branch*/
			if(exploreDriving(Maze_seg, &adc_data)){
				state_toBranch=gen_initState;
				return ERR_FAILED;
			}else if(adc_data.raw_Values.raw_MiddleL < 55000){
				/*Error but Driving must be finished */
				state_toBranch=gen_ErrorState;
				setStopFlag();
			}else if(adc_data.mm_Values.mm_Left>90){
				/* Branch detected Finish Driving */
				state_toBranch = gen_deinitState; // ev. go to wait state for some cycles to be in midle of branch
				setStopFlag();
			}
			break;
		case gen_deinitState: /* Finish driving nd return Ok if finished*/
			if(exploreDriving(Maze_seg, &adc_data)){
				state_toBranch = gen_initState;
				return ERR_OK;
			}
			break;
		case gen_ErrorState: /* finish driving and return Error*/
			if(exploreDriving(Maze_seg, &adc_data)){
				state_toBranch = gen_initState;
				return ERR_FAILED;
			}
			break;
		case gen_waitState:  /* not used in this case*/
		default:
			state_toBranch = gen_initState;
			break;

	}
	return ERR_BUSY;
}

byte turn90(uint8_t* segmentNumber, t_directions* currentOrientation, t_dir dir){
	static t_genericState state_turn90 = gen_initState;
	static Maze_segments Maze_seg;
	ADC_data_t adc_data;

	switch (state_turn90) {
		case gen_initState:
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
			if(exploreDriving(Maze_seg, &adc_data)){
				/* be sure is terminated, v ref=0 and Segmenttime at end*/
				state_turn90 = gen_initState;
				return ERR_OK;

			}
		case gen_waitState:  /* not used in this case*/
		case gen_ErrorState: /* not used in this case*/
		default:
			state_turn90 = gen_initState;
			break;
	}
	return ERR_BUSY;

}




