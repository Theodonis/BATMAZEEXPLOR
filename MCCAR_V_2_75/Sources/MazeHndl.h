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

byte unexploredBranchSet(t_mazeFieldData* currentField, t_directions currentTargetOrientation);

bool get_isUnexploredBranch(t_mazeFieldData* currentField,t_directions currentTargetOrientation, t_dir infoDirection);
bool get_isExploredFieldInFront(t_mazeFieldData* p_currField, t_directions curOrient, uint8_t xPos, uint8_t yPos);


#endif /* SOURCES_MAZEHNDL_H_ */
