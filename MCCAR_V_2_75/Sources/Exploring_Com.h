/*
 * Exploring_Com.h
 *
 *  Created on: 01.05.2019
 *      Author: cc-isn
 */

#ifndef SOURCES_EXPLORING_COM_H_
#define SOURCES_EXPLORING_COM_H_
#include "Driving.h"

void getNumberOfMazeSegments(Maze_segments *MazeSegments,const char *buffer);
char* getMazeSegment(Maze_segments *MazeSegments, char *buffer, uint8_t numberOfSegments);
void testUART(void);
Maze_segments getReferenceOfMazesegment(void);
#define CLS1_CMD_HELP   "help"
#define CLS1_CMD_STATUS "status"
#define CLS1_CMD_MSG_START "$"

#endif /* SOURCES_EXPLORING_COM_H_ */
