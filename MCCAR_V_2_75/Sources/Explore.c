/*
 * Explore.c
 *
 *  Created on: 11.10.2019
 *      Author: Theo
 */

#include "stdbool.h"
#include "ADC.h"
#include "Explore.h"
#include "Driving.h"
#include "DrivingExplore_Interface.h"

/*
** ===================================================================
**     Method      :  TargetPosStateMaschine (
*/
/*!
**     @brief
**     		State machine to drive to a wall in front and return to a left branch
**     		has to be called every 500us
**
**     @param
**
**     @return
**                         - Error code, possible codes:
**                           ERR_OK - State machine finished
**                           ERR_FAILED - Wall or branch to drive until not detected
**                           ERR_BUSY - State machine is driving
**
*/
/* ===================================================================*/
byte TargetPosStateMaschine(void){
	static t_PosState posState;
	static Maze_segments MazeSegmentsToBeDriven;
	ADC_data_t adc_data;
 	adc_data =	*get_latest_ADC_data();
	t_data_for_exploration driving_data;
	getDataForExplore(&driving_data);

	switch(posState){
		case initState:
			posState = driveToFrontWall;
			MazeSegmentsToBeDriven.numberOfSegments = 1;
			MazeSegmentsToBeDriven.segments[0].SingleSegment = 	10;
			break;
		case driveToFrontWall:
			if(Driving(MazeSegmentsToBeDriven)){
				return ERR_FAILED;
			}else if(adc_data.raw_Values.raw_MiddleR < 55000.0){
				posState = FrontWallDetected;
				setStopFlag();
			}
		case FrontWallDetected:
			if(Driving(MazeSegmentsToBeDriven)){
				posState = turnState;
				MazeSegmentsToBeDriven.numberOfSegments++;
				MazeSegmentsToBeDriven.segments[MazeSegmentsToBeDriven.numberOfSegments].SingleSegment = 	900;
				MazeSegmentsToBeDriven.numberOfSegments++;
				MazeSegmentsToBeDriven.segments[MazeSegmentsToBeDriven.numberOfSegments].SingleSegment = 	900;
			}
			break;
		case turnState:
			if(Driving(MazeSegmentsToBeDriven)){
				posState = driveToLeftBranch;
				MazeSegmentsToBeDriven.numberOfSegments++;
				MazeSegmentsToBeDriven.segments[MazeSegmentsToBeDriven.numberOfSegments].SingleSegment = 	10;
			}
			break;
		case driveToLeftBranch:
			if(Driving(MazeSegmentsToBeDriven)){
				return ERR_FAILED;
			}else if(adc_data.mm_Values.mm_Left > 90.0){
				posState = leftBranchDetected;
				setStopFlag();
			}
			break;
		case leftBranchDetected:
			if(Driving(MazeSegmentsToBeDriven)){
				posState = stopped;
			}
			break;
		case stopped:
			return ERR_OK;
	}
	return ERR_BUSY;




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
