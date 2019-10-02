/*
 * PID.h
 *
 *  Created on: Mar 12, 2018
 *      Author: cc-isn
 */

#ifndef SOURCES_PID_H_
#define SOURCES_PID_H_

#include "MCUC1.h"


typedef struct PID_data_{
	struct{
		double  w;	/* Input	: Wish / Set point */
		double y;	/* Input	: Measured Variable */
		double u;	/* Output	: Limited Controller Output */
		double v;	/* Output	: Unlimited Controller Output*/
	}signals;
	//
	struct{
		double P;		/* Proportional Part*/
		double I;		/* Integral Part */
		double D;		/* Derivative Part */
		double yold; 	/* Delayed measurement Variable */
		double arw;
		double e_arw;
		float v_motor_right;
		float v_motor_left;
	} states;
	//
	struct{
		double h;		/* Sampling period */
		double K;		/* Controller Gain */
		double Ti;		/* Integral Time */
		double Td;		/* Derivative Time*/
		double Tt;		/* Reset Time */
		double N;		/* Maximum Derivate Time*/
		double b;		/* Setpoint Fraction */
		double ulow;	/* Low output limit */
		double uhigh;	/* High output limit */
		bool ARW;
		double bi,ar,bd,ad;
	}parameter;
}PID_data_t;


void PID_CalculateOutput(PID_data_t *data);

void PID_addDerivatedParameters(PID_data_t *data);


//void PID_Init(PID_data_t *pidData_R, PID_data_t *pidData_L);


#if PL_CONFIG_HAS_SHELL
#include "CLS1.h"
/*!
 * \brief Shell command line parser.
 * \param[in] cmd Pointer to command string
 * \param[out] handled If command is handled by the parser
 * \param[in] io Std I/O handler of shell
 */
uint8_t PID_ParseCommand(const unsigned char *cmd, bool *handled, const CLS1_StdIOType *io);
#endif /* PL_CONFIG_HAS_SHELL */


#endif /* SOURCES_PID_H_ */
