/*
 * FSM.c
 *
 *  Created on: 14.11.2018
 *      Author: cc-isn
 */


/*
 * FSM.c
 *
 *  Created on: 09.03.2018
 *      Author: Cyrill95
 */
#include "Application.h"
#include "FSM.h"
#include "Driving.h"
//#include "Event_timer.h"
#include "Distance_INT.h"
#include "WAIT1.h"
#include "FC1.h"
#include "IMU.h"
#include "ADC.h"
#include "Motor.h"
#include "I_LED_L.h"
#include "I_LED_R.h"
#include "I_LED_ML.h"
#include "I_LED_MR.h"
#include "LED_GREEN_F_L.h"
#include "LED_RED_F_L.h"
#include "LED_GREEN_F_R.h"
#include "LED_RED_F_R.h"
#include "Exploration_Drive.h"



// Defines


// Variablen
volatile static state_t actMainState = INIT;
volatile static state_t oldMainState = INIT;

Maze_segments MazeSegmentsToBeDriven;
static MAZE_data_t mazeData;
static ADC_data_t *adcData;
static MAZE_track_t mazeTrack[100];


uint16_t distances[3];//¨[0]links  [1]rechts  [2] mitte


void stateMachine(void) {

	switch (actMainState) {
		case INIT:
			mazeData.maze_Values.maze[mazeData.maze_Parameter.maze_y][mazeData.maze_Parameter.maze_x] = mazeData.maze_Parameter.maze_counter;
			actMainState = North;
			MazeSegmentsToBeDriven.numberOfSegments = 1;
//			  QuadSmaple_EnableEvent();
//			  Distance_INT_EnableEvent();
//			  FC1_Reset();
			break;

		case North:
			MazeSegmentsToBeDriven.segments[0].SingleSegment = 1;

//			reinit_Drving();
//			while(!driveSegment(1));
			oldMainState = actMainState;
			Run_PID();
//			EVNT_SetEvent(EVNT_New_Field);

			break;

		case East:
			MazeSegmentsToBeDriven.segments[0].SingleSegment = 1;

//			reinit_Drving();
//			while(!driveSegment(1));
			oldMainState = actMainState;
			Run_PID();
//			EVNT_SetEvent(EVNT_New_Field);
			break;

		case South:
			MazeSegmentsToBeDriven.segments[0].SingleSegment = 1;

//			reinit_Drving();
//			while(!driveSegment(1));
			oldMainState = actMainState;
			Run_PID();
//			EVNT_SetEvent(EVNT_New_Field);
			break;

		case West:
			MazeSegmentsToBeDriven.segments[0].SingleSegment = 1;

//			reinit_Drving();
//			while(!driveSegment(1));
			oldMainState = actMainState;
			Run_PID();
//			EVNT_SetEvent(EVNT_New_Field);
			break;

		case Turn_Right:
			MazeSegmentsToBeDriven.segments[0].SingleSegment = -900;
//			Distance_INT_EnableEvent();
//			reinit_Drving();
//			while(!driveSegment(1));
			MazeSegmentsToBeDriven.segments[0].SingleSegment = 1;
//			Distance_INT_EnableEvent();
//			reinit_Drving();
//			while(!driveSegment(1));
			Distance_INT_DisableEvent();
			turn_r(1);
			mazeData.maze_Parameter.maze_counter ++;
			switch (oldMainState) {
							case North:
								actMainState = East;
								decreaseValue(&mazeData.maze_Parameter.maze_x);
								break;

							case East:
								actMainState = South;
								decreaseValue(&mazeData.maze_Parameter.maze_y);
								break;

							case South:
								actMainState = West;
								increaseValue(&mazeData.maze_Parameter.maze_x);
								break;

							case West:
								actMainState = North;
								increaseValue(&mazeData.maze_Parameter.maze_y);
								break;
						}

			if(mazeData.maze_Values.maze[mazeData.maze_Parameter.maze_y][mazeData.maze_Parameter.maze_x] != 0){
				mazeData.maze_Parameter.maze_counter = mazeData.maze_Values.maze[mazeData.maze_Parameter.maze_y][mazeData.maze_Parameter.maze_x];
			}
			else{
				mazeData.maze_Values.maze[mazeData.maze_Parameter.maze_y][mazeData.maze_Parameter.maze_x] = mazeData.maze_Parameter.maze_counter;
			}

			oldMainState = actMainState;

			if(adcData->mm_Values.mm_Right>95){
				MazeSegmentsToBeDriven.segments[0].SingleSegment = -900;
//				Distance_INT_EnableEvent();
//				reinit_Drving();
//				while(!driveSegment(1));
				MazeSegmentsToBeDriven.segments[0].SingleSegment = 1;
//				Distance_INT_EnableEvent();
//				reinit_Drving();
//				while(!driveSegment(1));
				turn_r(2);
				mazeData.maze_Parameter.maze_counter ++;
							switch (oldMainState) {
								case North:
									actMainState = East;
									decreaseValue(&mazeData.maze_Parameter.maze_x);
									break;

								case East:
									actMainState = South;
									decreaseValue(&mazeData.maze_Parameter.maze_y);
									break;

								case South:
									actMainState = West;
									increaseValue(&mazeData.maze_Parameter.maze_x);
									break;

								case West:
									actMainState = North;
									increaseValue(&mazeData.maze_Parameter.maze_y);
									break;
							}
							oldMainState = actMainState;

							if(mazeData.maze_Values.maze[mazeData.maze_Parameter.maze_y][mazeData.maze_Parameter.maze_x] != 0){
								mazeData.maze_Parameter.maze_counter = mazeData.maze_Values.maze[mazeData.maze_Parameter.maze_y][mazeData.maze_Parameter.maze_x];
							}
							else{
								mazeData.maze_Values.maze[mazeData.maze_Parameter.maze_y][mazeData.maze_Parameter.maze_x] = mazeData.maze_Parameter.maze_counter;
							}
			}
			EVNT_ClearEvent(EVNT_Turn_Right);
			Distance_INT_EnableEvent();
			break;

		case Turn_Left:
			MazeSegmentsToBeDriven.segments[0].SingleSegment = 900;
			Distance_INT_DisableEvent();
//			Distance_INT_EnableEvent();
//			reinit_Drving();
//			while(!driveSegment(1));
			turn_l();
			switch (oldMainState) {
							case North:
								actMainState = West;
								break;

							case East:
								actMainState = North;
								break;

							case South:
								actMainState = East;
								break;

							case West:
								actMainState = South;
								break;
						}
			oldMainState = actMainState;
			EVNT_ClearEvent(EVNT_Turn_Left);
			Distance_INT_EnableEvent();
			if(adcData->mm_Values.mm_MiddleR <80){
				EVNT_SetEvent(Turn_Left);
			}
			break;

		case New_Field:
			LED_GREEN_F_R_Neg();
			mazeData.maze_Parameter.maze_counter ++;
			if(actMainState != oldMainState){
				switch (oldMainState) {
					case North:
						increaseValue(&mazeData.maze_Parameter.maze_y);
						break;

					case East:
						decreaseValue(&mazeData.maze_Parameter.maze_x);
						break;

					case South:
						decreaseValue(&mazeData.maze_Parameter.maze_y);
						break;

					case West:
						increaseValue(&mazeData.maze_Parameter.maze_x);
						break;
				}
				if(mazeData.maze_Values.maze[mazeData.maze_Parameter.maze_y][mazeData.maze_Parameter.maze_x] != 0){
						mazeData.maze_Parameter.maze_counter = mazeData.maze_Values.maze[mazeData.maze_Parameter.maze_y][mazeData.maze_Parameter.maze_x];
					}
					else{
						mazeData.maze_Values.maze[mazeData.maze_Parameter.maze_y][mazeData.maze_Parameter.maze_x] = mazeData.maze_Parameter.maze_counter;
				}
				actMainState = oldMainState;
			}
			EVNT_ClearEvent(EVNT_New_Field);

			if(adcData->mm_Values.mm_Right >95){
				EVNT_SetEvent(EVNT_Turn_Right);
			}else if(adcData->mm_Values.mm_MiddleR <80){
				EVNT_SetEvent(EVNT_Turn_Left);
			}

			break;

		case NOT_AUS:
			break;
	}
}


void increaseValue (uint8_t *value){
	*value = *value +1;
}

void decreaseValue (uint8_t *value){
	*value = *value -1;
}

void setState(state_t state){
	actMainState = state;
}

state_t get_actState(void){
	return actMainState;
}

MAZE_data_t get_Maze(void){
	return mazeData;
}


void fillMazeData(uint16_t field){


	if(adcData->mm_Values.mm_MiddleL > 95){
		mazeTrack[field].wall_front = FALSE;
	}
	else{
		mazeTrack[field].wall_front = TRUE;
	}


	if(adcData->mm_Values.mm_Right > 95){
		mazeTrack[field].wall_right = FALSE;
	}
	else{
		mazeTrack[field].wall_right = TRUE;
	}


	if(adcData->mm_Values.mm_Left > 95){
		mazeTrack[field].wall_left = FALSE;
	}
	else{
		mazeTrack[field].wall_left = TRUE;
	}
}

bool driveSegment(int Segment){
	if(getMsFlag()){

		if(Driving(MazeSegmentsToBeDriven)){
			Distance_INT_DisableEvent();
			set_VREF(0,0);
			//deinitMotors();
			LED_GREEN_F_R_Off();
			LED_GREEN_F_L_Off();
			LED_RED_F_R_Off();
			LED_RED_F_L_Off();
			clearMsFlag();
			I_LED_R_ClrVal();I_LED_L_ClrVal();I_LED_MR_ClrVal();I_LED_ML_ClrVal(); // turn IR leds off
			adcData = get_latest_ADC_data();

			return TRUE;
		}
		clearMsFlag();
		return FALSE;
	}
	return FALSE;
}
