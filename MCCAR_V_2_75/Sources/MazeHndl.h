/*
 * MazeHndl.h
 *
 *  Created on: 19.11.2019
 *      Author: Theo
 */

#ifndef SOURCES_MAZEHNDL_H_
#define SOURCES_MAZEHNDL_H_


void initMaze(t_mazeFieldData* MazePointer);

byte sideBranchMeasurement(ADC_data_t* adc_data, t_mazeFieldData* currentField, t_directions  currentTargetOrientation);
byte setDriveDirectionWallInfo(t_mazeFieldData* currentField, t_directions  currentTargetOrientation);


byte setWallInfo(t_mazeFieldData* currentField, t_directions  wallOrientation, t_exploreInformation wallIsOpen);

byte unexploredBranchSet(t_mazeFieldData* currentField, t_directions  currentTargetOrientation);

#endif /* SOURCES_MAZEHNDL_H_ */
