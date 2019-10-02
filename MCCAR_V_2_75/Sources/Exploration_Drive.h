/*
 * Exploration_Drive.h
 *
 *  Created on: 14.08.2019
 *      Author: cc-isn
 */

#ifndef SOURCES_EXPLORATION_DRIVE_H_
#define SOURCES_EXPLORATION_DRIVE_H_


void getSensors(void);
void Run_PID(void);
void turn_r(uint8_t state);
void turn_l(void);
void PID_Init(void);
void driveToTurn(float dist);
void stopp();

#endif /* SOURCES_EXPLORATION_DRIVE_H_ */
