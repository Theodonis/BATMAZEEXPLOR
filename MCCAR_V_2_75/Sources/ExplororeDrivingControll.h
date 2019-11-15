/*
 * ExplororeDrivingControll.h
 *
 *  Created on: 15.11.2019
 *      Author: Theo
 */

#ifndef SOURCES_EXPLOROREDRIVINGCONTROLL_H_
#define SOURCES_EXPLOROREDRIVINGCONTROLL_H_

typedef enum genericState{
	gen_initState,
	gen_runnigState,
	gen_deinitState,
	gen_waitState,
	gen_ErrorState
}t_genericState;

typedef enum dir{
	left,
	right
} t_dir;

#endif /* SOURCES_EXPLOROREDRIVINGCONTROLL_H_ */
