/*
 * ADC.c
 *
 *  Created on: 17.01.2019
 *      Author: cc-isn
 */
#include "ADC.h"
#include <math.h>

#ifdef DISTANCE_BIASED_ENABLE
 	 static raw_Values_t ADC_BIAS;
#endif

/**
 *
 */
void calcADC_data(ADC_data_t *pData){
	static float left_raw;
	static float right_raw;
	static float middleL_raw;
	static float middleR_raw;
	uint16_t values_ADC0_raw[5];
	uint16_t values_ADC1_raw[5];
	ADC_0_Measure(TRUE);
	ADC_0_GetValue16(values_ADC0_raw);
	ADC_1_Measure(TRUE);
	ADC_1_GetValue16(values_ADC1_raw);
	#ifdef DISTANCE_BIASED_ENABLE
		pData->raw_Values.raw_Right 	= values_ADC1_raw[2]+ADC_BIAS.raw_Right;
		pData->raw_Values.raw_45Right 	= values_ADC0_raw[1]+ADC_BIAS.raw_45Right;
		pData->raw_Values.raw_MiddleR 	= values_ADC1_raw[1]+ADC_BIAS.raw_MiddleR;
		pData->raw_Values.raw_MiddleL 	= values_ADC0_raw[2]+ADC_BIAS.raw_MiddleL;
		pData->raw_Values.raw_45Left 	= values_ADC0_raw[0]+ADC_BIAS.raw_45Left;
		pData->raw_Values.raw_Left 		= values_ADC1_raw[0]+ADC_BIAS.raw_Left;
	#else
		pData->raw_Values.raw_Right= values_ADC1_raw[2];
		pData->raw_Values.raw_45Right = values_ADC0_raw[1];
		pData->raw_Values.raw_MiddleR = values_ADC1_raw[1];
		pData->raw_Values.raw_MiddleL = values_ADC0_raw[2];
		pData->raw_Values.raw_45Left = values_ADC0_raw[0];
		pData->raw_Values.raw_Left = values_ADC1_raw[0];
	#endif

	pData->raw_Values.raw_DRV1_IPROI = values_ADC1_raw[3];
	pData->raw_Values.raw_DRV2_IPROI = values_ADC1_raw[4];
	pData->raw_Values.raw_HALF_U_BAT = values_ADC0_raw[3];
	pData->raw_Values.raw_I_TOT = values_ADC0_raw[4];

	pData->voltage_Values.v_Bat = (2*ADC_3_3V_Constant*values_ADC0_raw[3]);
	pData->voltage_Values.I_Mot_L = (ADC_3_3V_Constant*values_ADC1_raw[3])/(TORQUE_TO_VREF*1000);//PullDown 2.4k
	pData->voltage_Values.I_Mot_R = (ADC_3_3V_Constant*values_ADC1_raw[4])/(TORQUE_TO_VREF*1000);//PullDown 2.4k

	left_raw = (float)(0.001*pData->raw_Values.raw_Left);
	right_raw = (float)(0.001*pData->raw_Values.raw_Right);
	middleL_raw =(float)(0.001*pData->raw_Values.raw_MiddleL);
	middleR_raw =(float)(0.001*pData->raw_Values.raw_MiddleR);

	if(left_raw < MIN_RAW_VALUE){
		pData->mm_Values.mm_Left = 5;
	}else if(left_raw < EXP_FIT){
		pData->mm_Values.mm_Left = fitExp(left_raw);
	}else if(left_raw < MIDDLE_FIT){
		pData->mm_Values.mm_Left = fitMiddle(left_raw);
	}else if(left_raw < FAR_FIT){
		pData->mm_Values.mm_Left = fitFar(left_raw);
	}else{
		pData->mm_Values.mm_Left = 119.6; // max. distance
	}

	if(right_raw < MIN_RAW_VALUE){
		pData->mm_Values.mm_Right = 5;
	}else if(right_raw < EXP_FIT){
		pData->mm_Values.mm_Right = fitExp(right_raw);
	}else if(right_raw < MIDDLE_FIT){
		pData->mm_Values.mm_Right = fitMiddle(right_raw);
	}else if(right_raw < FAR_FIT){
		pData->mm_Values.mm_Right = fitFar(right_raw);
	}else{
		pData->mm_Values.mm_Right = 119.6; // max. distance
	}

	if(middleL_raw < MIN_RAW_VALUE){
		pData->mm_Values.mm_MiddleL = 5;
	}else if(middleL_raw < EXP_FIT){
		pData->mm_Values.mm_MiddleL = fitExp(middleL_raw);
	}else if(middleL_raw < MIDDLE_FIT){
		pData->mm_Values.mm_MiddleL = fitMiddle(middleL_raw);
	}else if(middleL_raw < FAR_FIT){
		pData->mm_Values.mm_MiddleL = fitFar(middleL_raw);
	}else{
		pData->mm_Values.mm_MiddleL = 119.6; // max. distance
	}

	if(middleR_raw < MIN_RAW_VALUE){
		pData->mm_Values.mm_MiddleR = 5;
	}else if(middleR_raw < EXP_FIT){
		pData->mm_Values.mm_MiddleR = fitExp(middleR_raw);
	}else if(middleR_raw < MIDDLE_FIT){
		pData->mm_Values.mm_MiddleR = fitMiddle(middleR_raw);
	}else if(middleR_raw < FAR_FIT){
		pData->mm_Values.mm_MiddleR = fitFar(middleR_raw);
	}else{
		pData->mm_Values.mm_MiddleR = 119.6; // max. distance
	}

//	pData->mm_Values.mm_Left = lookUpLeft[(uint16_t)(0.01*pData->raw_Values.raw_Left)];
//	pData->mm_Values.mm_Right = lookUpRight[(uint16_t)(0.01*pData->raw_Values.raw_Right)];
//	pData->mm_Values.mm_MiddleL = lookUpMiddle[(uint16_t)(0.01*pData->raw_Values.raw_MiddleL)];
//	pData->mm_Values.mm_45Left = lookUpMiddle[(uint16_t)(0.01*pData->raw_Values.raw_45Left)];
//	pData->mm_Values.mm_45Right = lookUpMiddle[(uint16_t)(0.01*pData->raw_Values.raw_45Right)];
}

float fitMiddle(float raw){
	float yfit;
	float x2 = raw*raw;

	yfit = MIDDLE_FIT_COEF_1*x2 + MIDDLE_FIT_COEF_2*raw + MIDDLE_FIT_COEF_3;

	return yfit;
}

float fitExp(float raw){
	float yfit;

	yfit = EXP_FIT_COEF_A*exp(EXP_FIT_COEF_B*raw) + EXP_FIT_COEF_C*exp(EXP_FIT_COEF_D*raw);

	return yfit;
}

float fitFar(float raw){
	float yfit;
	float x2 = raw*raw;
	float x3 = x2*raw;
	float x4 = x3*raw;

	yfit = FAR_FIT_COEF_1*x4 + FAR_FIT_COEF_2*x3 + FAR_FIT_COEF_3*x2 +
			FAR_FIT_COEF_4*raw + FAR_FIT_COEF_5;

	return yfit;
}

float get_half_U_Bat(){
	uint16_t values_ADC0_raw[5];
	ADC_0_Measure(TRUE);
	ADC_0_GetValue16(values_ADC0_raw);
	return (float)(2*values_ADC0_raw[3]*0.000050354004);
}
#ifdef DISTANCE_BIASED_ENABLE
	void set_dist_Bias(void){
		uint16_t values_ADC0_raw[5];
		uint16_t values_ADC1_raw[5];
		ADC_0_Measure(TRUE);
		ADC_0_GetValue16(values_ADC0_raw);
		ADC_1_Measure(TRUE);
		ADC_1_GetValue16(values_ADC1_raw);

		ADC_BIAS.raw_Right 		= (uint16_t)0xFFFF-values_ADC1_raw[2];
		ADC_BIAS.raw_45Right 	= (uint16_t)0xFFFF-values_ADC0_raw[1];
		ADC_BIAS.raw_MiddleR 	= (uint16_t)0xFFFF-values_ADC1_raw[1];
		ADC_BIAS.raw_MiddleL 	= (uint16_t)0xFFFF-values_ADC0_raw[2];
		ADC_BIAS.raw_45Left 	= (uint16_t)0xFFFF-values_ADC0_raw[0];
		ADC_BIAS.raw_Left 		= (uint16_t)0xFFFF-values_ADC1_raw[0];
	}
#endif

void get_dist_Bias(raw_Values_t *p_ADC_BIAS){
	#ifdef DISTANCE_BIASED_ENABLE
		*p_ADC_BIAS 		= ADC_BIAS;
	#else//Visibility in log
		p_ADC_BIAS->raw_Right 	= 0;
		p_ADC_BIAS->raw_45Right = 0;
		p_ADC_BIAS->raw_MiddleR = 0;
		p_ADC_BIAS->raw_MiddleL = 0;
		p_ADC_BIAS->raw_45Left 	= 0;
		p_ADC_BIAS->raw_Left 	= 0;
	#endif
}


