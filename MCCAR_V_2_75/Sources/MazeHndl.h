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

byte saveFrontWallInfo(t_mazeFieldData* currentField, t_directions  currentTargetOrientation, t_exploreInformation frontIsOpen);

#endif /* SOURCES_MAZEHNDL_H_ */
