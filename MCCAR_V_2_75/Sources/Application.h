/**
 * \file
 * \brief Main application interface
 * \author Erich Styger, erich.styger@hslu.ch
 *
 * This provides the main application entry point.
 */

#ifndef __APPLICATION_H_
#define __APPLICATION_H_

#include "Event.h"
#include "Driving.h"
//static bool ms_Flag;
void APP_Start(void);
void Init_Maze(void);
void DrivingControl(void);
void APP_EventHandler(EVNT_Handle event);
bool getMsFlag(void);
void clearMsFlag(void);
#endif /* APPLICATION_H_ */
