/*
 * Explore.c
 *
 *  Created on: 11.10.2019
 *      Author: Theo
 */

#include "stdbool.h"
#include "ADC.h"
#include "Explore.h"
#include "DrivingExplore_Interface.h"
#include "TargetInField_Position.h"
#include "ExplororeDrivingControll.h"


#ifdef ENABLE_DATALOG
	#include "Logging.h"
#endif

#ifdef ENABLE_TIMING_CONROLL
	#include "FC1.h"
#endif


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

/*
** ===================================================================
**     Method      :  exploreDriving(Maze_segments MazeSegmentsToBeDriven, uint8_t* logValCnt)
**
**     @brief
**     		Adaptation from Driving(MazeSegmentsToBeDriven) -> with this function instead of
**     		call driving directly is the time after the call also logged.
**
**     @param
**						- MazeSegmentsToBeDriven: way to drive -> Driving() in Driving.c
**						- logValCnt: pointer to Log variable number
**     @return
**                      - Error code, possible codes:
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
	static t_directions  currentTargetOrientation = north;


	static uint8_t  segmentNumber = 0; /* to Handle the current SegNumb for all Driving calls*/

	static uint8_t saveDataCnt = 0; // for calculation of logging sample period

	/* Data Log */
	#ifdef ENABLE_DATALOG
		#ifdef ENABLE_TIMING_CONROLL
			uint16_t callPeriod;
			FC1_GetCounterValue(&callPeriod);
			FC1_Reset();

			saveExplorationValue((float)callPeriod, "callPeriod", 0);//logValCnt++);
		#endif
	#endif

	ADC_data_t adc_data =	*get_latest_ADC_data();/* is always one step behind newest data because driving will be called use*/
	t_data_for_exploration driving_data;
	getDataForExplore(&driving_data);
	#ifdef ENABLE_DATALOG
	//saveExplorationValue((float)adc_data.mm_Values.mm_MiddleR, "adc_data.mm_Values.mm_MiddleR", logValCnt++);
	#endif



	switch(posState){
		case initState:
			posState = driveToLeftBranch;//initTurnAngleCalibration;//
			initMaze(&MazeData[0][0]);
			calcADC_data(&adc_data); /* if driving() isn't called, also the adc isn't called...*/
			xPos =0 ,yPos =0;
			#ifdef ENABLE_DATALOG
				resetSaveLinePointer();
			#endif
			return ERR_BUSY;
			break;
		case driveToFront:
			switch(driveToFrontWall(&segmentNumber, adc_data.raw_Values.raw_MiddleL)){
				case ERR_BUSY:
					posState = driveToFront;
					break;
				case ERR_FAILED:
					posState =  initState;
					return ERR_FAILED;
				case ERR_OK:
					posState= stopped;
					break;
			}

			#ifdef ENABLE_DATALOG
				saveExplorationValue(fieldPositioner(driving_data.posEstimation,&xPos,&yPos,currentTargetOrientation),"fieldState",7);
			#endif
			break;
		case FrontWallDetected:


			break;
		case turnState:
			switch(turn90(&segmentNumber, &currentTargetOrientation, left)){
				case ERR_BUSY:
					posState = turnState;
					break;
				case ERR_FAILED:
					posState =  initState;
					return ERR_FAILED;
				case ERR_OK:
					posState= driveToFront;
					break;

			}
			break;
		case driveToLeftBranch:
			switch(driveToBranch(&segmentNumber, adc_data.raw_Values.raw_MiddleL, adc_data.mm_Values.mm_Left)){
				case ERR_BUSY:
					posState = driveToLeftBranch;
					break;
				case ERR_FAILED:
					posState =  initState;
					return ERR_FAILED;
				case ERR_OK:
					posState= turnState;
					break;

			}
			break;
		case leftBranchDetected:
//			if(exploreDriving(MazeSegmentsToBeDriven,&logValCnt)){
//				reinit_Drving(true);
//				posState = turn90State;
//				MazeSegmentsToBeDriven.segments[0].SingleSegment = 	900;
//				MazeSegmentsToBeDriven.numberOfSegments=1;
//				currentTargetOrientation--;
//			}else{
//
//				#ifdef ENABLE_DATALOG
//					saveExplorationValue(fieldPositioner(driving_data.posEstimation,&xPos,&yPos,currentTargetOrientation),"fieldState",7);
//				#endif
//			}
			break;

		case turn90State:
//			if(exploreDriving(MazeSegmentsToBeDriven,&logValCnt)){
//				MazeSegmentsToBeDriven.segments[MazeSegmentsToBeDriven.numberOfSegments].SingleSegment = 	10;
//				MazeSegmentsToBeDriven.numberOfSegments++;
//				posState = driveToFrontWall;
//			}
			break;
		case driveToEnd:
//			if(exploreDriving(MazeSegmentsToBeDriven,&logValCnt)){
//				posState = stopped;
//			}
//			#ifdef ENABLE_DATALOG
//				saveExplorationValue(fieldPositioner(driving_data.posEstimation,&xPos,&yPos,currentTargetOrientation),"fieldState",7);
//			#endif
			break;
		case stopped:
			return ERR_OK;
			break;


		case initTurnAngleCalibration:
//			posState = turnAngleCalibration;
//			for(uint8_t numOfTurns = 0; numOfTurns<2; numOfTurns++){
//				MazeSegmentsToBeDriven.segments[numOfTurns].SingleSegment = 	900;
//				MazeSegmentsToBeDriven.numberOfSegments=numOfTurns+1;
//			}
			return ERR_BUSY;
		case turnAngleCalibration:
//			if(exploreDriving(MazeSegmentsToBeDriven,&logValCnt)){//Driving(MazeSegmentsToBeDriven)){//
//				posState=stopped;
//			}
//			break;
		case errorStop:
//			if(exploreDriving(MazeSegmentsToBeDriven,&logValCnt)){
//				posState=stopped;
//				return ERR_FAILED;
//			}
			break;


	}




	#ifdef ENABLE_DATALOG

//		saveExplorationValue(adc_data.raw_Values.raw_Right,"rawRight", logValCnt++);
		saveExplorationValue(driving_data.posEstimation.xPos,"xPos", 4);//logValCnt++);
		saveExplorationValue(posState,"state",3);
		saveExplorationValue(driving_data.posEstimation.thetaAngle,"thetaAngle", 5);//logValCnt++);
		saveExplorationValue(xPos,"x-Position (index)", 6);
		saveExplorationValue(currentTargetOrientation,"TargetOrintation", 8);
		saveExplorationValue(adc_data.voltage_Values.v_Bat,"Bateriespannung", 9);


		saveExplorationValue(yPos,"y-Position (index)", 11);
		saveExplorationValue(driving_data.posEstimation.yPos,"yPos", 10);



#ifdef ENABLE_TIMING_CONROLL
		uint16_t ticksAfterExplore;
		FC1_GetCounterValue(&ticksAfterExplore);
		saveExplorationValue((float)ticksAfterExplore, varNameToString(ticksAfterExplore), 2);//logValCnt++);
	#endif

		if(saveDataCnt>=4){  //to set sample period (0 => 0,5ms)
			incrmentSaveLinePointer(); //all sample values are overwritten until its incremented
			saveDataCnt=0;
		}else{
			saveDataCnt++;
		}
	#endif

		return ERR_BUSY;

}



