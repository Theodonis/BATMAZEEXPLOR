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

#define LOG_DEPENDING_ON_CYCLE 				(1&&ENABLE_EXPLORE_DATALOG)
#define LOG_DEPENDING_MAZE_POS 				(1&&ENABLE_EXPLORE_DATALOG&&!LOG_DEPENDING_ON_CYCLE)

#define LOG_BLE_ENABLE 						(0) // enable send data over bluetooth wille explore
#define BLE_DRIVETOUNEXPLOREDBRANCH_LOG		(0&&!LOG_BLE_ENABLE)
#define BLE_DRIVETOBRANCH_LOG				(0&&!LOG_BLE_ENABLE&&!BLE_DRIVETOUNEXPLOREDBRANCH_LOG)

#define EXPLOR_DRIVE_TIME_AFTER_BRANCHDETECT 				(0.03) 	//time in s ~>additional driven m if velocity 1m/s
#define EXPLOR_DRIVE_TIME_IN_KNOWN_FIELD_TO_STOPP_MIDDLED 	(0.085)//(0.095) //time in s ~>additional driven m if velocity 1m/s
#define MAX_SIDEDIST_TO_WALL_MM								(75) 	// if side wards distance is bigger ,means no wall there


#define MAX_FRONTDIST_TO_WALL_RAW							(62000) 	// if front distance is smaler ->stop driving

#define MAX_FRONTDIST_TO_WALL_MM							(82) 	// if front distance is smaler ->stop driving

#define MAZE_FIELDS_WIDTH_NORTH_DIRECTION (9)
#define MAZE_FIELDS_LENGTH_EAST_DIRECTION (9)

#define SIZE_OF_WAY_HIST (81)




#endif
