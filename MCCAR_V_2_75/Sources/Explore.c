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
#include "UTIL1.h"


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
**     		Main FSM to handle exploration
**
**     @param

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
	//Todo: deside if north or south is bigger..
	static t_mazeFieldData init_MazeData[MAZE_FIELDS_WIDTH_NORTH_DIRECTION];/* save the date till startorientation is clear -> start north or east warts*/

	static uint8_t xPos =0 ,yPos =0;
	static t_directions  currentTargetOrientation = north;
	static uint8_t numberofTurns = 0;

	static bool drivenStraitWihtStopp =false;

	#if LOG_BLE_ENABLE
		static t_bleLogState ble_Log_State = BLE_idleState;
		static uint8_t ble_wait_cycle_cnt = 0;
		static t_mazeFieldData lastField;
	#endif
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



	switch(posState){
		case initState:
			posState = explore;//initTurnAngleCalibration;//turnAngleCalibration;//
			initMaze(&MazeData[0][0]);
			calcADC_data(&adc_data); /* if driving() isn't called, also the adc isn't called...*/
			xPos =0 ,yPos =0;
			#if ENABLE_EXPLORE_DATALOG
				resetSaveLinePointer();
			#endif
			return ERR_BUSY;
			break;

		case explore:
			/*drive strait until e wall and detect all field info */
			switch(driveToFrontWall(&segmentNumber,&adc_data,currentTargetOrientation,&MazeData[xPos][yPos],xPos,yPos)){
				case ERR_BUSY:
					posState = explore;
					if(drivenStraitWihtStopp){ /* its a strait way after a strait way -> there are steps in posdata...*/
						getDataForExplore(&driving_data);
						currentFieldState = fieldPositioner(driving_data.posEstimation,&xPos,&yPos,currentTargetOrientation,true);
						drivenStraitWihtStopp = false;
					}
					break;
				case ERR_FAILED:
					posState =  initState;
					return ERR_FAILED;
				case ERR_OK:
					setWallInfo(&MazeData[xPos][yPos],currentTargetOrientation,ex_false); /*set wall info of wall in front*/
					(void) sideBranchMeasurement(&adc_data, &MazeData[xPos][yPos],currentTargetOrientation);
					(void) unexploredBranchSet(&MazeData[xPos][yPos],currentTargetOrientation,true); /*update if unexplored branch before change state*/
					posState= calcNextStep;//stopped;//turnRight;//
					break;
			}
//			IntOverBLE(xPos);
			/* do measurement */
			currentFieldState = fieldPositioner(driving_data.posEstimation,&xPos,&yPos,currentTargetOrientation,false);
			switch(currentFieldState){
				case detectSideWalls:
					/*measure side walls in Middle of Field and set Field to explored*/
					(void) sideBranchMeasurement(&adc_data, &MazeData[xPos][yPos],currentTargetOrientation);

					#if LOG_BLE_ENABLE
						ble_Log_State = sendXIndex; //start bluetooth send
					#endif
					break;
				case saveFrontwall:
					setDriveDirectionWallInfo(&MazeData[xPos][yPos],currentTargetOrientation); /* update Wall info in leaving direction before leaving*/
					unexploredBranchSet(&MazeData[xPos][yPos],currentTargetOrientation,true); /*update if unexplored branch before leaving*/

					#if LOG_DEPENDING_MAZE_POS

						saveExplorationValue(posState,"state", 2);
						saveExplorationValue(adc_data.raw_Values.raw_MiddleL,"DistanzFrontL",3);
						saveExplorationValue(driving_data.posEstimation.xPos,"X-Pos", 4);
						saveExplorationValue(driving_data.posEstimation.yPos,"Y-Pos",5);
						saveExplorationValue(xPos,"x-Position (index)", 6);
						saveExplorationValue(yPos,"y-Position (index)", 7);
						saveExplorationValue(adc_data.raw_Values.raw_Left,"LeftDist", 8);
						saveExplorationValue(adc_data.raw_Values.raw_Right,"RightDist", 9);
//						saveExplorationValue(MazeData[xPos][yPos].hasUnexploredBranchFlag,"BranchFlag", 10);
						saveExplorationValue(MazeData[xPos][yPos].posibDirections.south,"Feld Info S�d", 11);
						saveExplorationValue(MazeData[xPos][yPos].posibDirections.north,"Feld Info Nord", 12);
						saveExplorationValue(MazeData[xPos][yPos].posibDirections.east,"Feld Info Ost", 13);

						incrmentSaveLinePointer(); /*only log at enter of new field*/
					#endif
					break;

			}
			break;
		case calcNextStep:
			if(MazeData[xPos][yPos].hasUnexploredBranchFlag){
				if(get_isUnexploredBranch(&MazeData[xPos][yPos],currentTargetOrientation,left)){
					/* it's an unexplored branch on the left!*/
					posState= turnLeft;
				}else if(get_isUnexploredBranch(&MazeData[xPos][yPos],currentTargetOrientation,right)){
					/* it's an unexplored branch on the right!*/
					posState= turnRight;
				}else if(get_isUnexploredBranch(&MazeData[xPos][yPos],currentTargetOrientation,front)){
					/* it's an unexplored branch in front!*/
					posState= explore;
					/*need to reinit fieldpositioner*/
					drivenStraitWihtStopp = true;
				}
			}else{
				/*it's a dead end*/
				posState = returnToBranch;
			}
			break;

		case turnLeft:
			/*turn left 90� */
			switch(turn90(&segmentNumber, &currentTargetOrientation, left)){
				case ERR_BUSY:
					posState = turnLeft;
					break;
				case ERR_FAILED:
					posState =  initState;
					return ERR_FAILED;
				case ERR_OK:
					posState= explore;//stopped;//
					break;
			}
			break;

		case turnRight:
			/*turn right 90� */
			switch(turn90(&segmentNumber, &currentTargetOrientation, right)){
				case ERR_BUSY:
					posState = turnRight;
					break;
				case ERR_FAILED:
					posState =  initState;
					return ERR_FAILED;
				case ERR_OK:
					posState= explore;//stopped;//
					break;
			}
			break;


		case returnToBranch:
			/*call second level FSM to return to last unexplored branch */
			switch(driveToUnexpBranch(&segmentNumber, &adc_data, &currentTargetOrientation, &MazeData[xPos][yPos])){
				static uint8_t callCNT = 0;
				case ERR_BUSY:
					if(callCNT<4){
						callCNT++;
					}
					if(drivenStraitWihtStopp&&callCNT>=3){ /* its a strait way after a strait way -> there are steps in posdata...*/
						getDataForExplore(&driving_data);
						currentFieldState = fieldPositioner(driving_data.posEstimation,&xPos,&yPos,currentTargetOrientation,true);
						drivenStraitWihtStopp = false;
					}
					posState = returnToBranch;
					break;
				case ERR_FAILED:
					posState =  initState;
					callCNT = 0;
					return ERR_FAILED;
				case ERR_OK:
					if(MazeData[xPos][yPos].hasUnexploredBranchFlag){
						/*check if branch is still unexplored */
						(void) unexploredBranchSet(&MazeData[xPos][yPos],currentTargetOrientation,false); /*update if unexplored branch before change state*/
						if(MazeData[xPos][yPos].hasUnexploredBranchFlag){
							posState = calcNextStep;
						}else{
							if(MazeData[xPos][yPos].enterDirection==get_wallOrientation(currentTargetOrientation,behind)){
								drivenStraitWihtStopp = true;
								IntOverBLE(0XFF);
							}
							posState = calcNextStep;

						}
					}
					callCNT = 0;
					break;
			}
			/* do measurement */
			currentFieldState = fieldPositioner(driving_data.posEstimation,&xPos,&yPos,currentTargetOrientation,false);
			switch(currentFieldState){
				case detectSideWalls:
					#if LOG_BLE_ENABLE
						ble_Log_State = sendXIndex; //start bluetooth send
					#endif
					/*measurement not used while returning on explored path*/
//					(void) sideBranchMeasurement(&adc_data, &MazeData[xPos][yPos],currentTargetOrientation);
					break;
				case saveFrontwall:

					/*measurement not used while returning on explored path*/
//					setDriveDirectionWallInfo(&MazeData[xPos][yPos],currentTargetOrientation); /* update Wall info in leaving direction before leaving*/
//					unexploredBranchSet(&MazeData[xPos][yPos],currentTargetOrientation); /*update if unexplored branch before leaving*/



					#if LOG_DEPENDING_MAZE_POS
						saveExplorationValue(posState,"state", 2);
						saveExplorationValue(adc_data.raw_Values.raw_MiddleL,"DistanzFrontL",3);
						saveExplorationValue(driving_data.posEstimation.xPos,"X-Pos", 4);
						saveExplorationValue(driving_data.posEstimation.yPos,"Y-Pos",5);
						saveExplorationValue(xPos,"x-Position (index)", 6);
						saveExplorationValue(yPos,"y-Position (index)", 7);
						saveExplorationValue(adc_data.raw_Values.raw_Left,"LeftDist", 8);
						saveExplorationValue(adc_data.raw_Values.raw_Right,"RightDist", 9);
//						saveExplorationValue(MazeData[xPos][yPos].hasUnexploredBranchFlag,"BranchFlag", 10);
						saveExplorationValue(MazeData[xPos][yPos].posibDirections.south,"Feld Info S�d", 11);
						saveExplorationValue(MazeData[xPos][yPos].posibDirections.north,"Feld Info Nord", 12);
						saveExplorationValue(MazeData[xPos][yPos].posibDirections.east,"Feld Info Ost", 13);
						incrmentSaveLinePointer(); /*only log at enter of new field*/
					#endif
					break;

			}
			break;




		/* old states */
		case driveToFront:
			/*Driving*/
			switch(driveToFrontWall(&segmentNumber,&adc_data,currentTargetOrientation,&MazeData[xPos][yPos],xPos,yPos)){
				case ERR_BUSY:
					posState = driveToFront;
					break;
				case ERR_FAILED:
					posState =  initState;
					return ERR_FAILED;
				case ERR_OK:
					posState= turnState;
					(void) sideBranchMeasurement(&adc_data, &MazeData[xPos][yPos],currentTargetOrientation); /* do an extra sideMasurement before turn*/
					(void) setWallInfo(&MazeData[xPos][yPos],currentTargetOrientation,ex_false);/*set front wall false if detected frontwall */
					unexploredBranchSet(&MazeData[xPos][yPos],currentTargetOrientation,true); /*update if field has unexplored branch before turn*/
					break;
			}

			/*Field info -> measure and log...*/
			currentFieldState = fieldPositioner(driving_data.posEstimation,&xPos,&yPos,currentTargetOrientation,false);
			switch(currentFieldState){
				 case detectSideWalls:
					(void) sideBranchMeasurement(&adc_data, &MazeData[xPos][yPos],currentTargetOrientation);
					break;
				 case saveFrontwall:
						setDriveDirectionWallInfo(&MazeData[xPos][yPos],currentTargetOrientation); /* update Wall info in leaving direction before leaving*/
						unexploredBranchSet(&MazeData[xPos][yPos],currentTargetOrientation,true); /*update if field has unexplored branch before leaving*/
						saveExplorationValue(MazeData[xPos][yPos].posibDirections.north,"Nordwand", 2);
						saveExplorationValue(MazeData[xPos][yPos].posibDirections.east,"Ostwand", 5);
						saveExplorationValue(MazeData[xPos][yPos].posibDirections.south,"S�dwand", 9);
						saveExplorationValue(MazeData[xPos][yPos].posibDirections.west,"Westwand", 13);
						/*old Way used always previews field*/
//					 switch(currentTargetOrientation){
//						 case north:
//							 (void) setWallInfo(&MazeData[xPos-1][yPos],north,ex_true);/*Front wall of old field */
//							 (void) setWallInfo(&MazeData[xPos][yPos],south,ex_true);/*Back wall of new field*/
//
//
//							 (void)unexploredBranchSet(&MazeData[xPos-1][yPos]);/* set if leaved field had en unexplored branch */
//							 /*log info of old Field */
//							 saveExplorationValue(MazeData[xPos-1][yPos].posibDirections.north,"Nordwand", 2);
//							 saveExplorationValue(MazeData[xPos-1][yPos].posibDirections.east,"Ostwand", 5);
//							 saveExplorationValue(MazeData[xPos-1][yPos].posibDirections.south,"S�dwand", 9);
//							 saveExplorationValue(MazeData[xPos-1][yPos].posibDirections.west,"Westwand", 13);
//							 saveExplorationValue(MazeData[xPos-1][yPos].hasUnexploredBranchFlag,"Unexplored Branch", 10);
//							 break;
//						 case east:
//							 (void) setWallInfo(&MazeData[xPos][yPos-1],east,ex_true);/*Front wall of old field */
//							 (void) setWallInfo(&MazeData[xPos][yPos],west,ex_true);/*Back wall of new field*/
//
//							 (void)unexploredBranchSet(&MazeData[xPos][yPos-1]);/* set if leaved field had en unexplored branch */
//							 /*log info of old Field */
//							 saveExplorationValue(MazeData[xPos][yPos-1].posibDirections.north,"Nordwand", 2);
//							 saveExplorationValue(MazeData[xPos][yPos-1].posibDirections.east,"Ostwand", 5);
//							 saveExplorationValue(MazeData[xPos][yPos-1].posibDirections.south,"S�dwand", 9);
//							 saveExplorationValue(MazeData[xPos][yPos-1].posibDirections.west,"Westwand", 13);
//							 saveExplorationValue(MazeData[xPos][yPos-1].hasUnexploredBranchFlag,"Unexplored Branch", 10);
//							 break;
//						 case south:
//							 (void) setWallInfo(&MazeData[xPos+1][yPos],south,ex_true);/*Front wall of old field */
//							 (void) setWallInfo(&MazeData[xPos][yPos],north,ex_true);/*Back wall of new field*/
//
//							 (void)unexploredBranchSet(&MazeData[xPos+1][yPos]);/* set if leaved field had en unexplored branch */
//							 /*log info of old Field */
//							 saveExplorationValue(MazeData[xPos+1][yPos].posibDirections.north,"Nordwand", 2);
//							 saveExplorationValue(MazeData[xPos+1][yPos].posibDirections.east,"Ostwand", 5);
//							 saveExplorationValue(MazeData[xPos+1][yPos].posibDirections.south,"S�dwand", 9);
//							 saveExplorationValue(MazeData[xPos+1][yPos].posibDirections.west,"Westwand", 13);
//							 saveExplorationValue(MazeData[xPos+1][yPos].hasUnexploredBranchFlag,"Unexplored Branch", 10);
//							 break;
//						 case west:
//							 (void) setWallInfo(&MazeData[xPos][yPos+1],west,ex_true);/*Front wall of old field */
//							 (void) setWallInfo(&MazeData[xPos][yPos],east,ex_true);/*Back wall of new field*/
//
//							 (void)unexploredBranchSet(&MazeData[xPos][yPos+1]);/* set if leaved field had en unexplored branch */
//							 /*log info of old Field */
//							 saveExplorationValue(MazeData[xPos][yPos+1].posibDirections.north,"Nordwand", 2);
//							 saveExplorationValue(MazeData[xPos][yPos+1].posibDirections.east,"Ostwand", 5);
//							 saveExplorationValue(MazeData[xPos][yPos+1].posibDirections.south,"S�dwand", 9);
//							 saveExplorationValue(MazeData[xPos][yPos+1].posibDirections.west,"Westwand", 13);
//							 saveExplorationValue(MazeData[xPos][yPos+1].hasUnexploredBranchFlag,"Unexplored Branch", 10);
//							 break;
//					 }

					 saveExplorationValue(yPos,"y-Position (index)", 11);
					 saveExplorationValue(xPos,"x-Position (index)", 6);
					 incrmentSaveLinePointer(); /*only log at enter of new field*/
					 break;
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
//			switch(driveToBranch(&segmentNumber,right,&adc_data)){
//				case ERR_BUSY:
//					posState = driveToLeftBranch;
//					break;
//				case ERR_FAILED:
//					posState =  initState;
//					return ERR_FAILED;
//				case ERR_OK:
//					posState= turn90State;
//					(void) sideBranchMeasurement(&adc_data, &MazeData[xPos][yPos],currentTargetOrientation); /* do an extra sideMasurement before turn*/
//					if(adc_data.mm_Values.mm_MiddleL < 110){/*Set front wall before turn */
//						 (void) setWallInfo(&MazeData[xPos][yPos],currentTargetOrientation,ex_false);
//					}else{
//						 (void) setWallInfo(&MazeData[xPos][yPos],currentTargetOrientation,ex_true);/*Front wall of old field */
//					}
//					break;
//			}
			currentFieldState = fieldPositioner(driving_data.posEstimation,&xPos,&yPos,currentTargetOrientation,false);
			switch(currentFieldState){
			 	 case detectSideWalls:
					(void) sideBranchMeasurement(&adc_data, &MazeData[xPos][yPos],currentTargetOrientation);
					break;
				 case saveFrontwall:
					setDriveDirectionWallInfo(&MazeData[xPos][yPos],currentTargetOrientation); /* update Wall info in leaving direction before leaving*/
					unexploredBranchSet(&MazeData[xPos][yPos],currentTargetOrientation,true); /*update if field has unexplored branch before leaving*/
					saveExplorationValue(MazeData[xPos][yPos].posibDirections.north,"Nordwand", 2);
					saveExplorationValue(MazeData[xPos][yPos].posibDirections.east,"Ostwand", 5);
					saveExplorationValue(MazeData[xPos][yPos].posibDirections.south,"S�dwand", 9);
					saveExplorationValue(MazeData[xPos][yPos].posibDirections.west,"Westwand", 13);
//					 switch(currentTargetOrientation){
//						 case north:
////							 (void) setWallInfo(&MazeData[xPos-1][yPos],north,ex_true);/*Front wall of old field */
////							 (void) setWallInfo(&MazeData[xPos][yPos],south,ex_true);/*Back wall of new field*/
//							 (void)unexploredBranchSet(&MazeData[xPos-1][yPos]); /* set if leaved field had en unexplored branch */
//							 /*log info of old Field */
//							 saveExplorationValue(MazeData[xPos-1][yPos].posibDirections.north,"Nordwand", 2);
//							 saveExplorationValue(MazeData[xPos-1][yPos].posibDirections.east,"Ostwand", 5);
//							 saveExplorationValue(MazeData[xPos-1][yPos].posibDirections.south,"S�dwand", 9);
//							 saveExplorationValue(MazeData[xPos-1][yPos].posibDirections.west,"Westwand", 13);
//							 saveExplorationValue(MazeData[xPos-1][yPos].hasUnexploredBranchFlag,"Unexplored Branch", 10);
//							 break;
//						 case east:
////							 (void) setWallInfo(&MazeData[xPos][yPos-1],east,ex_true);/*Front wall of old field */
////							 (void) setWallInfo(&MazeData[xPos][yPos],west,ex_true);/*Back wall of new field*/
//							 (void)unexploredBranchSet(&MazeData[xPos][yPos-1]);/* set if leaved field had en unexplored branch */
//							 /*log info of old Field */
//							 saveExplorationValue(MazeData[xPos][yPos-1].posibDirections.north,"Nordwand", 2);
//							 saveExplorationValue(MazeData[xPos][yPos-1].posibDirections.east,"Ostwand", 5);
//							 saveExplorationValue(MazeData[xPos][yPos-1].posibDirections.south,"S�dwand", 9);
//							 saveExplorationValue(MazeData[xPos][yPos-1].hasUnexploredBranchFlag,"Unexplored Branch", 10);
//							 saveExplorationValue(MazeData[xPos][yPos-1].posibDirections.west,"Westwand", 13);
//							 break;
//						 case south:
////							 (void) setWallInfo(&MazeData[xPos+1][yPos],south,ex_true);/*Front wall of old field */
////							 (void) setWallInfo(&MazeData[xPos][yPos],north,ex_true);/*Back wall of new field*/
//							 (void)unexploredBranchSet(&MazeData[xPos+1][yPos]);/* set if leaved field had en unexplored branch */
//							 /*log info of old Field */
//							 saveExplorationValue(MazeData[xPos+1][yPos].posibDirections.north,"Nordwand", 2);
//							 saveExplorationValue(MazeData[xPos+1][yPos].posibDirections.east,"Ostwand", 5);
//							 saveExplorationValue(MazeData[xPos+1][yPos].posibDirections.south,"S�dwand", 9);
//							 saveExplorationValue(MazeData[xPos+1][yPos].hasUnexploredBranchFlag,"Unexplored Branch", 10);
//							 saveExplorationValue(MazeData[xPos+1][yPos].posibDirections.west,"Westwand", 13);
//							 break;
//						 case west:
////							 (void) setWallInfo(&MazeData[xPos][yPos+1],west,ex_true);/*Front wall of old field */
////							 (void) setWallInfo(&MazeData[xPos][yPos],east,ex_true);/*Back wall of new field*/
//							 (void)unexploredBranchSet(&MazeData[xPos][yPos+1]);/* set if leaved field had en unexplored branch */
//							 /*log info of old Field */
//							 saveExplorationValue(MazeData[xPos][yPos+1].posibDirections.north,"Nordwand", 2);
//							 saveExplorationValue(MazeData[xPos][yPos+1].posibDirections.east,"Ostwand", 5);
//							 saveExplorationValue(MazeData[xPos][yPos+1].posibDirections.south,"S�dwand", 9);
//							 saveExplorationValue(MazeData[xPos][yPos+1].hasUnexploredBranchFlag,"Unexplored Branch", 10);
//							 saveExplorationValue(MazeData[xPos][yPos+1].posibDirections.west,"Westwand", 13);
//							 break;
//					 }


					 saveExplorationValue(yPos,"y-Position (index)", 11);
					 saveExplorationValue(xPos,"x-Position (index)", 6);
					 incrmentSaveLinePointer(); /*only log at enter of new field->means all other values are from one call before*/
					 break;
			}
			#if ENABLE_EXPLORE_DATALOG
				saveExplorationValue(currentFieldState,"fieldState",7);
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
					posState= stopped;
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
//			return ERR_BUSY;
		case turnAngleCalibration:
			switch(turn180(&segmentNumber,&currentTargetOrientation,left)){//Driving(MazeSegmentsToBeDriven)){//
			case ERR_OK:
				if(numberofTurns>=0){
					posState=stopped;
					numberofTurns = 0;
				}else{
					posState=turnAngleCalibration;
					numberofTurns++;
				}
				break;
			case ERR_FAILED:
				posState = initState;
				return ERR_FAILED;
			case ERR_BUSY:
				posState=turnAngleCalibration;
				break;
			}
//			FloatOverBLE(driving_data.posEstimation.thetaAngle);
			break;
		case errorStop:
			posState=stopped;
			return ERR_FAILED;
//			if(exploreDriving(MazeSegmentsToBeDriven,&logValCnt)){
//				posState=stopped;
//				return ERR_FAILED;
//			}
			break;


	}

	#if LOG_BLE_ENABLE
		switch(ble_Log_State){
			case sendXIndex:
					ExploreDataOverBLE(0, xPos);
					ble_Log_State = sendYIndex;
				break;
			case sendYIndex:
				ExploreDataOverBLE(1, yPos);
				ble_Log_State =sendWallinfo;
				break;
			case sendWallinfo:
				if(currentFieldState==saveFrontwall){
					uint8_t wallMerge = (MazeData[xPos][yPos].posibDirections.north<<6)|(MazeData[xPos][yPos].posibDirections.east<<4)|(MazeData[xPos][yPos].posibDirections.south<<2)|MazeData[xPos][yPos].posibDirections.west;
					ExploreDataOverBLE(2, wallMerge);
					lastField = MazeData[xPos][yPos];
					ble_Log_State =SendFieldend;
				}
				break;

			case sendFieldInfo:
				{
					uint8_t fieldinfoMerge = (lastField.hasUnexploredBranchFlag<<4)|(lastField.exploredFlag<<3)|lastField.enterDirection;
					ExploreDataOverBLE(3, fieldinfoMerge);
					ble_Log_State =SendFieldend;
				}

				break;
			case SendFieldend:
				ExploreDataOverBLE(9,99);
				ble_Log_State =BLE_idleState;
				break;
			case BLE_idleState:
				/*its an Idle state*/
				ble_Log_State =BLE_idleState;
				break;
		}

	#endif
	#if LOG_BLE_ENABLE

	#endif


	#if LOG_DEPENDING_ON_CYCLE

		saveExplorationValue(currentFieldState,"Fieldstate", 2);
		saveExplorationValue(driving_data.posEstimation.thetaAngle,"Theta",3);
//		saveExplorationValue(posState,"State",3);
		saveExplorationValue(driving_data.posEstimation.xPos,"X-Pos", 4);
		saveExplorationValue(driving_data.posEstimation.yPos,"Y-Pos",5);
		saveExplorationValue(xPos,"x-Position (index)", 6);
		saveExplorationValue(yPos,"y-Position (index)", 7);
		saveExplorationValue(adc_data.mm_Values.mm_Left,"LeftDist", 8);
//		saveExplorationValue(MazeData[xPos][yPos].hasUnexploredBranchFlag,"unexpFlag", 8);
		saveExplorationValue(adc_data.mm_Values.mm_Right,"RightDist", 9);
		saveExplorationValue(MazeData[xPos][yPos].posibDirections.north,"Feld Info Nord", 10);
		saveExplorationValue(MazeData[xPos][yPos].posibDirections.east,"Feld Info Ost", 11);
		saveExplorationValue(MazeData[xPos][yPos].posibDirections.south,"Feld Info S�d", 12);
		saveExplorationValue(MazeData[xPos][yPos].posibDirections.west,"Feld Info West", 13);


	#if ENABLE_TIMING_CONROLL
//		uint16_t ticksAfterExplore;
//		FC1_GetCounterValue(&ticksAfterExplore);
//		saveExplorationValue((float)ticksAfterExplore, varNameToString(ticksAfterExplore), 2);//logValCnt++);
	#endif
		if((xPos>0&&xPos<5&&yPos==8&&currentTargetOrientation==north)){//||(xPos<3&&yPos==1&&currentTargetOrientation==south)){/*start logging at the fourth field*/
			if(saveDataCnt>=0){  //to set sample period (0 => DT)
				incrmentSaveLinePointer(); //all sample values are overwritten until its incremented
				saveDataCnt=0;
			}else{
				saveDataCnt++;
			}
		}
	#endif

	return ERR_BUSY;


}


//byte exploreLog(){
//
//}


