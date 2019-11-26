/*
 * ExplororeDrivingControll.h
 *
 *  Created on: 15.11.2019
 *      Author: Theo
 */

#ifndef SOURCES_EXPLOROREDRIVINGCONTROLL_H_
#define SOURCES_EXPLOROREDRIVINGCONTROLL_H_



#include "PE_Error.h"
#include "stdint.h"
#include "PE_Types.h"
#include "Explore.h"

typedef enum genericState{
	gen_initState,
	gen_runnigState,
	gen_deinitState,
	gen_waitState,
	gen_ErrorState
}t_genericState;

typedef enum returToBranchState{
	retBra_initState,
	retBra_runnigState,
	retBra_curvState,
	retBra_waitState,
	retBra_unexploredBranchState,
	retBra_turnLeft,
	retBra_turnRight,
	retBra_deinitState,
	retBra_ErrorState
}t_returToBranchState;



#define  getLatestADC() (get_latest_ADC_data()) /* wrapper to us function in Explore.c*/

byte driveToUnexpBranch(uint8_t* segmentNumber,ADC_data_t* adc_data, t_directions* currentTargetOrientation, t_directions* wayHist, t_mazeFieldData currentMazeFieldData);

byte driveToFrontWall(uint8_t* segmentNumber,ADC_data_t* adc_data);
byte driveToBranch(uint8_t* segmentNumber, t_dir dir,ADC_data_t* adc_data);
byte turn90(uint8_t* segmentNumber, t_directions* currentOrientation, t_dir dir);
byte turn180(uint8_t* segmentNumber, t_directions* currentOrientation, t_dir dir);

#endif /* SOURCES_EXPLOROREDRIVINGCONTROLL_H_ */
