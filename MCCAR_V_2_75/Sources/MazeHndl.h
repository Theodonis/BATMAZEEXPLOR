/*
 * MazeHndl.h
 *
 *  Created on: 19.11.2019
 *      Author: Theo
 */

#ifndef SOURCES_MAZEHNDL_H_
#define SOURCES_MAZEHNDL_H_



void initMaze(t_mazeFieldData* MazePointer);
byte doMazeMeasurement(ADC_data_t* adc_data, t_mazeFieldData* currentField);

#endif /* SOURCES_MAZEHNDL_H_ */
