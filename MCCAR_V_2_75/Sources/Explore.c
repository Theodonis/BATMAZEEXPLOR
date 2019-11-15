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
#include "TargetInField_Position.h"

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
bool exploreDriving(Maze_segments MazeSegmentsToBeDriven, uint8_t* logValCnt){
	bool drivingFinishedFlag;
//	#ifdef ENABLE_TIMING_CONROLL
//			uint16_t beforDriving;
//			FC1_GetCounterValue(&beforDriving);
//			saveExplorationValue((float)beforDriving, "beforDriving", (*logValCnt)++);
//	#endif
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


byte TargetPosStateMaschine(void){
	static t_PosState posState;
	static t_mazeFieldData MazeData[MAZE_FIELDS_WIDTH_NORTH_DIRECTION][MAZE_FIELDS_LENGTH_EAST_DIRECTION];
	static uint8_t xPos =0 ,yPos =0;
	static t_directions  currentTargetOrientation = north;

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
			#ifdef ENABLE_DATALOG
				resetSaveLinePointer();
			#endif
			return ERR_BUSY;
			break;
		case driveToFrontWall:
			if(exploreDriving(MazeSegmentsToBeDriven,&logValCnt)){
				return ERR_FAILED;
			}else if(adc_data.raw_Values.raw_MiddleR < 55000.0){
				posState = FrontWallDetected;
				setStopFlag();
			}
			#ifdef ENABLE_DATALOG
				saveExplorationValue(fieldPositioner(driving_data.posEstimation,&xPos,&yPos,currentTargetOrientation),"fieldState",7);
			#endif
			break;
		case FrontWallDetected:
			if(exploreDriving(MazeSegmentsToBeDriven,&logValCnt)){
				if(currentTargetOrientation==north|currentTargetOrientation==east){
					reinit_Drving(true);
					posState = turnState;
					MazeSegmentsToBeDriven.segments[0].SingleSegment = 	900;
					currentTargetOrientation++;
					MazeSegmentsToBeDriven.segments[1].SingleSegment = 	900;
					currentTargetOrientation++;
					MazeSegmentsToBeDriven.numberOfSegments=2;
	//				MazeSegmentsToBeDriven.segments[MazeSegmentsToBeDriven.numberOfSegments].SingleSegment = 	900;
	//				MazeSegmentsToBeDriven.numberOfSegments++;
	//				MazeSegmentsToBeDriven.segments[MazeSegmentsToBeDriven.numberOfSegments].SingleSegment = 	900;
	//				MazeSegmentsToBeDriven.numberOfSegments++;
				}else if(currentTargetOrientation == west){
					posState=stopped;
				}
			}else{
			#ifdef ENABLE_DATALOG
				saveExplorationValue(fieldPositioner(driving_data.posEstimation,&xPos,&yPos,currentTargetOrientation),"fieldState",7);
			#endif
			}


			break;
		case turnState:
			if(exploreDriving(MazeSegmentsToBeDriven,&logValCnt)){
//				if(adc_data.voltage_Values.v_Bat < 3.2){
//					return ERR_FAILED;
//				}
				if(adc_data.mm_Values.mm_Left > 90.0){
					posState = stopped;
					MazeSegmentsToBeDriven.segments[0].SingleSegment = 	0;
					MazeSegmentsToBeDriven.numberOfSegments=0;
				}else{
					if(currentTargetOrientation==south){
						posState = driveToLeftBranch;
					}else if(currentTargetOrientation==west|currentTargetOrientation==east){
						posState = driveToFrontWall;
					}
					MazeSegmentsToBeDriven.segments[MazeSegmentsToBeDriven.numberOfSegments].SingleSegment = 	10;
					MazeSegmentsToBeDriven.numberOfSegments++;
				}
				//reinit_Drving(true);
			}

			break;
		case driveToLeftBranch:
			if(exploreDriving(MazeSegmentsToBeDriven,&logValCnt)){
				return ERR_FAILED;
			}else if(adc_data.mm_Values.mm_Left > 90.0){
				posState = leftBranchDetected;
				setStopFlag();
			}
			#ifdef ENABLE_DATALOG
				saveExplorationValue(fieldPositioner(driving_data.posEstimation,&xPos,&yPos,currentTargetOrientation),"fieldState",7);
			#endif
			break;
		case leftBranchDetected:
			if(exploreDriving(MazeSegmentsToBeDriven,&logValCnt)){
				reinit_Drving(true);
				posState = turn90State;
				MazeSegmentsToBeDriven.segments[0].SingleSegment = 	900;
				MazeSegmentsToBeDriven.numberOfSegments=1;
				currentTargetOrientation--;
			}else{

				#ifdef ENABLE_DATALOG
					saveExplorationValue(fieldPositioner(driving_data.posEstimation,&xPos,&yPos,currentTargetOrientation),"fieldState",7);
				#endif
			}
			break;

		case turn90State:
			if(exploreDriving(MazeSegmentsToBeDriven,&logValCnt)){
				MazeSegmentsToBeDriven.segments[MazeSegmentsToBeDriven.numberOfSegments].SingleSegment = 	10;
				MazeSegmentsToBeDriven.numberOfSegments++;
				posState = driveToFrontWall;
			}
			break;
		case driveToEnd:
			if(exploreDriving(MazeSegmentsToBeDriven,&logValCnt)){
				posState = stopped;
			}
			#ifdef ENABLE_DATALOG
				saveExplorationValue(fieldPositioner(driving_data.posEstimation,&xPos,&yPos,currentTargetOrientation),"fieldState",7);
			#endif
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



