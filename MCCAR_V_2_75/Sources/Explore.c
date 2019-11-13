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
#include "Logging.h"

#ifdef ENABLE_TIMING_CONROLL
	#include "FC1.h"
#endif


bool exploreDriving(Maze_segments MazeSegmentsToBeDriven, uint8_t* logValCnt){
	bool drivingFinishedFlag;
//	#ifdef ENABLE_TIMING_CONROLL
//			uint16_t beforDriving;
//			FC1_GetCounterValue(&beforDriving);
//			saveExplorationValue((float)beforDriving, "beforDriving", (*logValCnt)++);
//		#endif
	drivingFinishedFlag = Driving(MazeSegmentsToBeDriven);
	#ifdef ENABLE_DATALOG
	#ifdef ENABLE_TIMING_CONROLL
		uint16_t ticksAfterDriving;
		FC1_GetCounterValue(&ticksAfterDriving);
		saveExplorationValue((float)ticksAfterDriving, "ticksAfterDriving", 1);//(*logValCnt)++);
	#endif
	#endif
	return drivingFinishedFlag;
}
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

t_directions calcDrivingDirection(float theataAngle){

}


byte fieldPositioner(t_PosEstimation pos,uint8_t* xPos,uint8_t* yPos,t_mazeFieldData* maze){
	static float fieldpos 	= 0; /*pos in curent field */
	static float initpos 	= 0;
	static bool firstCall 	= false;
	static t_fieldState fieldState = initState;

	switch(fieldState){
		case initState:
			fieldpos = 0;
			initpos = pos.xPos;
			break;
		case firstQuarterOfField:
			if(fieldpos>QUARTER_OF_MAZE_FIELD_LENGTH){
				fieldState = secondQuarterOfField;
			}
			break;
		case secondQuarterOfField:
			if(fieldpos>HALF_OF_MAZE_FIELD_LENGTH){
				fieldState = thirdQuarterOfField;
			}
			break;
		case thirdQuarterOfField:
			if(fieldpos>QUARTER_OF_MAZE_FIELD_LENGTH+HALF_OF_MAZE_FIELD_LENGTH){
				fieldState = fourthQuarterOfField;
			}
			break;
		case fourthQuarterOfField:
			if(fieldpos>EXPLORE_MAZE_SIZE/2){
				fieldState = firstQuarterOfField;
//				switch(pos.thetaAngle){
//				case PI:
//					break;
//				}
			}
			break;
	}



//	if(!firstCall){
//		initpos = pos.xPos;
//		firstCall = true;
//	}
//
//	fieldpos = initpos - pos.xPos;
//	if(fieldpos>EXPLORE_MAZE_SIZE){
//		initpos = pos.xPos;
//		fieldpos = 0;
//		(*xPos)++;
//	}

}




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
	static t_mazeFieldData MazeData[MAZE_FIELDS_WIDTH_NORTH_DIRECTION][MAZE_FIELDS_LENGTH_EAST_DIRECTION];
	static uint8_t xPos =0 ,yPos =0;

	static Maze_segments MazeSegmentsToBeDriven;
	static uint8_t saveDataCnt = 0; // for calculation of logging sample period
	uint8_t logValCnt = 0;
	/* Data Log */
	#ifdef ENABLE_DATALOG
		#ifdef ENABLE_TIMING_CONROLL
			uint16_t callPeriod;
			FC1_GetCounterValue(&callPeriod);
			FC1_Reset();

			saveExplorationValue((float)callPeriod, "callPeriod", 0);//logValCnt++);
		#endif
	#endif



	ADC_data_t adc_data =	*get_latest_ADC_data();
	t_data_for_exploration driving_data;
	getDataForExplore(&driving_data);
	#ifdef ENABLE_DATALOG
	//saveExplorationValue((float)adc_data.mm_Values.mm_MiddleR, "adc_data.mm_Values.mm_MiddleR", logValCnt++);
	#endif



	switch(posState){
		case initState:
			posState = driveToFrontWall;//initTurnAngleCalibration;//
			MazeSegmentsToBeDriven.numberOfSegments = 1;
			MazeSegmentsToBeDriven.segments[0].SingleSegment = 	10;
			initMaze(&MazeData[0][0]);
			xPos =0 ,yPos =0;
			resetSaveLinePointer();
			return ERR_BUSY;
			break;
		case driveToFrontWall:
			if(exploreDriving(MazeSegmentsToBeDriven,&logValCnt)){
				return ERR_FAILED;
			}else if(adc_data.raw_Values.raw_MiddleR < 55000.0){
				posState = FrontWallDetected;
				setStopFlag();
			}
			saveExplorationValue(1,"state",3);
			break;
		case FrontWallDetected:
			if(exploreDriving(MazeSegmentsToBeDriven,&logValCnt)){

				reinit_Drving(true);
				posState = turnState;
				MazeSegmentsToBeDriven.segments[0].SingleSegment = 	900;
				MazeSegmentsToBeDriven.segments[1].SingleSegment = 	900;
				MazeSegmentsToBeDriven.numberOfSegments=2;
//				MazeSegmentsToBeDriven.segments[MazeSegmentsToBeDriven.numberOfSegments].SingleSegment = 	900;
//				MazeSegmentsToBeDriven.numberOfSegments++;
//				MazeSegmentsToBeDriven.segments[MazeSegmentsToBeDriven.numberOfSegments].SingleSegment = 	900;
//				MazeSegmentsToBeDriven.numberOfSegments++;
			}
			saveExplorationValue(2,"state",3);
			break;
		case turnState:
			if(exploreDriving(MazeSegmentsToBeDriven,&logValCnt)){
				posState = driveToLeftBranch;
				MazeSegmentsToBeDriven.segments[MazeSegmentsToBeDriven.numberOfSegments].SingleSegment = 	10;
				MazeSegmentsToBeDriven.numberOfSegments++;
			}
			saveExplorationValue(3,"state",3);
			break;
		case driveToLeftBranch:
			if(exploreDriving(MazeSegmentsToBeDriven,&logValCnt)){
				return ERR_FAILED;
			}else if(adc_data.mm_Values.mm_Left > 90.0){
				posState = leftBranchDetected;
				setStopFlag();
			}
			saveExplorationValue(4,"state",3);
			break;
		case leftBranchDetected:
			if(exploreDriving(MazeSegmentsToBeDriven,&logValCnt)){
				posState = stopped;
			}
			saveExplorationValue(5,"state",3);
			break;
		case stopped:
			return ERR_OK;
			break;
		case initTurnAngleCalibration:
			posState = turnAngleCalibration;
			for(uint8_t numOfTurns = 0; numOfTurns<2; numOfTurns++){
				MazeSegmentsToBeDriven.segments[numOfTurns].SingleSegment = 	900;
				MazeSegmentsToBeDriven.numberOfSegments=numOfTurns+1;
			}
			return ERR_BUSY;
		case turnAngleCalibration:
			if(exploreDriving(MazeSegmentsToBeDriven,&logValCnt)){//Driving(MazeSegmentsToBeDriven)){//
				posState=stopped;
			}
			break;

	}

	(void)fieldPositioner(driving_data.posEstimation,&xPos,&yPos,&MazeData[0][0]);


	#ifdef ENABLE_DATALOG

//		saveExplorationValue(adc_data.raw_Values.raw_Right,"rawRight", logValCnt++);
		saveExplorationValue(driving_data.posEstimation.xPos,"xPos", 4);//logValCnt++);
//		saveExplorationValue(driving_data.velocityEstimation.forwardVeloc,"Velocity", logValCnt++);
		saveExplorationValue(driving_data.posEstimation.thetaAngle,"thetaAngle", 5);//logValCnt++);
//		saveExplorationValue(xPos,"X", logValCnt++);

#ifdef ENABLE_TIMING_CONROLL
		uint16_t ticksAfterExplore;
		FC1_GetCounterValue(&ticksAfterExplore);
		saveExplorationValue((float)ticksAfterExplore, varNameToString(ticksAfterExplore), 2);//logValCnt++);
	#endif

		if(saveDataCnt>=0){  //to set sample period (0 => 0,5ms)
			incrmentSaveLinePointer(); //all sample values are overwritten until its incremented
			saveDataCnt=0;
		}else{
			saveDataCnt++;
		}
	#endif

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
