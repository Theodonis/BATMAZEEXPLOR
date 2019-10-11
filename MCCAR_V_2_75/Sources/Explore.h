/*
 * Explore.h
 *
 *  Created on: 11.10.2019
 *      Author: Theo
 */

#ifndef SOURCES_EXPLORE_H_
#define SOURCES_EXPLORE_H_
#include <stdint.h>
#include "ADC.h"

typedef enum segEndDet_state_ {
	FIRST_CALL = 1,
	SECOND_CALL,
	LATER_CALL
} segEndDet_state_t;

bool segEndDetection(ADC_data_t *adcData,bool *segmentEnd);
void reinit_Explore(void);




#endif /* SOURCES_EXPLORE_H_ */
