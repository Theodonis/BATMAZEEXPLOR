/*
 * Explore.c
 *
 *  Created on: 11.10.2019
 *      Author: Theo
 */

#include "stdbool.h"
#include "Explore.h"
#include "DrivingExplore_Interface.h"
#include "TargetInField_Position.h"
#include "ExplororeDrivingControll.h"
#include "MazeHndl.h"


#if ENABLE_EXPLORE_DATALOG
	#include "Logging.h"
#endif

#if ENABLE_TIMING_CONROLL
	#include "FC1.h"
#endif




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
byte TargetPosStateMaschine(void){
	static t_PosState posState;
	static t_mazeFieldData MazeData[MAZE_FIELDS_WIDTH_NORTH_DIRECTION][MAZE_FIELDS_LENGTH_EAST_DIRECTION];
	static uint8_t xPos =0 ,yPos =0;
	static t_directions  currentTargetOrientation = north;

	t_fieldState currentFieldState = 0;


	static uint8_t  segmentNumber = 0; /* to Handle the current SegNumb for all Driving calls*/

	static uint8_t saveDataCnt = 0; // for calculation of logging sample period

	/* Data Log */
	#if ENABLE_EXPLORE_DATALOG
		#if ENABLE_TIMING_CONROLL
			uint16_t callPeriod;
			FC1_GetCounterValue(&callPeriod);
			FC1_Reset();

			saveExplorationValue((float)callPeriod, "callPeriod", 0);//logValCnt++);
		#endif
	#endif

	ADC_data_t adc_data =	*getLatestADC();/* is always one step behind newest data because driving will be called use*/
	t_data_for_exploration driving_data;
	getDataForExplore(&driving_data);
	#if ENABLE_EXPLORE_DATALOG
	//saveExplorationValue((float)adc_data.mm_Values.mm_MiddleR, "adc_data.mm_Values.mm_MiddleR", logValCnt++);
	#endif



	switch(posState){
		case initState:
			posState = driveToLeftBranch;//initTurnAngleCalibration;//
			initMaze(&MazeData[0][0]);
			calcADC_data(&adc_data); /* if driving() isn't called, also the adc isn't called...*/
			xPos =0 ,yPos =0;
			#if ENABLE_EXPLORE_DATALOG
				resetSaveLinePointer();
			#endif
			return ERR_BUSY;
			break;
		case driveToFront:
			switch(driveToFrontWall(&segmentNumber)){
				case ERR_BUSY:
					posState = driveToFront;
					break;
				case ERR_FAILED:
					posState =  initState;
					return ERR_FAILED;
				case ERR_OK:
					posState= turnState;
					break;
			}
			currentFieldState = fieldPositioner(driving_data.posEstimation,&xPos,&yPos,currentTargetOrientation);
			if(currentFieldState == detectWalls){
				(void) doMazeMeasurement(&adc_data, &MazeData[xPos][yPos],currentTargetOrientation);
			}
			#if ENABLE_EXPLORE_DATALOG
				saveExplorationValue(currentFieldState,"fieldState",7);
			#endif
			break;
		case FrontWallDetected:


			break;
		case turnState:
			switch(turn180(&segmentNumber, &currentTargetOrientation, left)){
				case ERR_BUSY:
					posState = turnState;
					break;
				case ERR_FAILED:
					posState =  initState;
					return ERR_FAILED;
				case ERR_OK:
					posState= stopped;
					break;
			}
			break;
		case driveToLeftBranch:
			switch(driveToBranch(&segmentNumber,right)){
				case ERR_BUSY:
					posState = driveToLeftBranch;
					break;
				case ERR_FAILED:
					posState =  initState;
					return ERR_FAILED;
				case ERR_OK:
					posState= turn90State;
					break;
			}
			#if ENABLE_EXPLORE_DATALOG
				saveExplorationValue(fieldPositioner(driving_data.posEstimation,&xPos,&yPos,currentTargetOrientation),"fieldState",7);
			#endif
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
//				#if ENABLE_EXPLORE_DATALOG
//					saveExplorationValue(fieldPositioner(driving_data.posEstimation,&xPos,&yPos,currentTargetOrientation),"fieldState",7);
//				#endif
//			}
			break;

		case turn90State:
			switch(turn90(&segmentNumber, &currentTargetOrientation, right)){
				case ERR_BUSY:
					posState = turn90State;
					break;
				case ERR_FAILED:
					posState =  initState;
					return ERR_FAILED;
				case ERR_OK:
					posState= driveToFront;
					break;
			}
			break;
		case driveToEnd:
//			if(exploreDriving(MazeSegmentsToBeDriven,&logValCnt)){
//				posState = stopped;
//			}
//			#if ENABLE_EXPLORE_DATALOG
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




	#if ENABLE_EXPLORE_DATALOG

//		saveExplorationValue(adc_data.raw_Values.raw_Right,"rawRight", logValCnt++);
		saveExplorationValue(posState,"state",3);
		saveExplorationValue(driving_data.posEstimation.xPos,"xPos", 4);//logValCnt++);
		saveExplorationValue(driving_data.posEstimation.thetaAngle,"thetaAngle", 5);//logValCnt++);
		saveExplorationValue(xPos,"x-Position (index)", 6);
		saveExplorationValue(currentTargetOrientation,"TargetOrintation", 8);
		saveExplorationValue(adc_data.voltage_Values.v_Bat,"battery voltage", 9);

		saveExplorationValue(driving_data.posEstimation.yPos,"yPos", 10);
		saveExplorationValue(yPos,"y-Position (index)", 11);
		saveExplorationValue(adc_data.mm_Values.mm_Left,"mm_Left", 12);



	#if ENABLE_TIMING_CONROLL
		uint16_t ticksAfterExplore;
		FC1_GetCounterValue(&ticksAfterExplore);
		saveExplorationValue((float)ticksAfterExplore, varNameToString(ticksAfterExplore), 2);//logValCnt++);
	#endif

		if(saveDataCnt>=4){  //to set sample period (0 => 0,7ms)
			incrmentSaveLinePointer(); //all sample values are overwritten until its incremented
			saveDataCnt=0;
		}else{
			saveDataCnt++;
		}
	#endif

	return ERR_BUSY;

}



