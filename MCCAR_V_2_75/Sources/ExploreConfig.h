/*
 * ExploreConfig.h
 *
 *  Created on: 08.11.2019
 *      Author: Theo
 */

#ifndef EXPLORECONFIG_H
#define EXPLORECONFIG_H

#include "PlatformConfiguration.h"


#if (!DRIVING_LOG_ENABLE)
#define ENABLE_EXPLORE_DATALOG 				(1)
#define ENABLE_TIMING_CONROLL				(1&&ENABLE_EXPLORE_DATALOG)
#endif


#define EXPLOR_DRIVE_TIME_AFTER_BRANCHDETECT 	(0.01) //time in s ~>additional driven m if velocity 1m/sS

#define MAZE_FIELDS_WIDTH_NORTH_DIRECTION (9)
#define MAZE_FIELDS_LENGTH_EAST_DIRECTION (9)




#endif
