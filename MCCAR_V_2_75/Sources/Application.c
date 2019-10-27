/**
 * \file
 * \brief Main application file
 * \author Erich Styger, erich.styger@hslu.ch
 *
 * This provides the main application entry point.
 */


/* User includes (#include below this line is not maintained by Processor Expert) */

#include "stdbool.h"
#include "Cpu.h"
#include "Events.h"
#include "LEDpin1.h"
#include "BitIoLdd1.h"
#include "MCUC1.h"
#include "LEDpin2.h"
#include "BitIoLdd2.h"
#include "LEDpin3.h"
#include "BitIoLdd3.h"
#include "LEDpin4.h"
#include "BitIoLdd4.h"
#include "LEDpin5.h"
#include "BitIoLdd5.h"
#include "LEDpin6.h"
#include "BitIoLdd6.h"
#include "GI2C1.h"
#include "CI2C1.h"
#include "BitIoLdd14.h"
#include "BitIoLdd15.h"
#include "JOY_UP.h"
#include "BitIoLdd16.h"
#include "JOY_DOWN.h"
#include "BitIoLdd17.h"
#include "JOY_LEFT.h"
#include "BitIoLdd18.h"
#include "JOY_RIGHT.h"
#include "BitIoLdd19.h"
#include "JOY_PUSH.h"
#include "BitIoLdd20.h"
#include "AS_BLE.h"
#include "ASerialLdd1.h"
#include "CLS_BLE.h"
#include "UTIL1.h"
#include "XF1.h"
#include "CS1.h"
#include "AS_PRG.h"
#include "ASerialLdd2.h"
#include "DacLdd1.h"
#include "Kill.h"
#include "BitIoLdd27.h"
#include "PWR_BTN.h"
#include "BitIoLdd28.h"
#include "Platform.h"
#include "BLE_Mode.h"
#include "BitIoLdd31.h"
#include "AdcLdd2.h"
#include <GPIO_PDD.h>
#include <PORT_PDD.h>
#include "PE_Types.h"
#include "PE_Error.h"
#include "PE_Const.h"
#include "IO_Map.h"
#include "WAIT1.h"
#include <stdlib.h>
#include <math.h>
#include "Application.h"
#include "PID.h"
#include "FSM.h"
#include "Event.h"
#include "IMU.h"
#include "ADC.h"
#include "Logging.h"
#include "Encoder.h"
#include "Driving.h"
#include "Motor.h"
#include "I_Motor_L.h"
#include "I_Motor_R.h"
#include "Exploring_Com.h"
#include "Exploration_Drive.h"
#include "Explore.h"

 PID_data_t pidData_R, pidData_L, pidData_UC;
 ADC_data_t adcData;
 Encoder_data_t encData;
 IMU_data_t imuData;
 Maze_segments MazeSegmentsToBeDriven;

static bool ms_Flag;
static bool exploFinishFlag = TRUE;
static char dataBuffer[400] = "$6;#;1;-180;#;2;6;0,0;0,0;0,0;0,0;0,0;0,0;#;3;3;1,1;1,1;1,1;#;4;-90;#;5;2;0,0;0,0;#;6;180;$255;";
static bool segmentEnd = FALSE;

uint16_t encCounter = 0;

FC1_TTimerValue temp;

//uint16_t motor_vRight1 = 0;
//uint16_t motor_vRight2 = 400;
//uint16_t motor_vRight3 = 1000;
//uint16_t motor_vRight4 = 2000;
//uint16_t motor_vRight5 = 3000;
//uint16_t motor_vRight6 = 4095;



void APP_EventHandler(EVNT_Handle event) {
  /*! \todo handle events */
  switch(event) {
  case EVNT_STARTUP:
	  setState(INIT);
    break;

  case EVNT_Turn_Right:
	  LED_RED_F_R_Neg();
	  Distance_INT_DisableEvent();
	  setState(Turn_Right);
	  stopp();
	  turn_r(1);
	  MotorEnable(0,0);

	  LED_RED_F_R_Neg();
	  //Distance_INT_EnableEvent();
	  EVNT_SetEvent(EVNT_New_Field);
    break;

  case EVNT_Turn_Left:
	  setState(Turn_Left);
	  LED_GREEN_F_R_Neg();
	  Distance_INT_DisableEvent();
	  turn_l();
	  stopp();
	  LED_GREEN_F_R_Neg();
	  //Distance_INT_EnableEvent();
	  //EVNT_ClearEvent(EVNT_Turn_Left);
    break;

  case EVNT_New_Field:
	  setState(New_Field);LED_BLUE_F_L_Neg();
	  //EVNT_ClearEvent(EVNT_New_Field);
    break;

  case EVNT_None:

    break;

    default:
      break;
   } /* switch */
}

void testJoystick()
{
	if(JOY_UP_GetVal() == 0) {
		LED_GREEN_F_R_Off();
		LED_GREEN_F_L_Off();
		LED_BLUE_F_R_Off();
		LED_BLUE_F_L_Off();
		LED_RED_F_R_On();
		LED_RED_F_L_On();
		testUART();
		MazeSegmentsToBeDriven = getReferenceOfMazesegment();
		LED_RED_F_R_Off();
		LED_RED_F_L_Off();
		LED_BLUE_F_R_On();
		LED_BLUE_F_L_On();

	} else {

	}
	if(JOY_RIGHT_GetVal() == 0) {

		WAIT1_Waitms(1000);

		/* IMU */
		LED_GREEN_F_R_Off();
		LED_GREEN_F_L_Off();
		LED_BLUE_F_R_Off();
		LED_BLUE_F_L_Off();
		LED_RED_F_R_On();
		LED_RED_F_L_On();
		biasCalc();
		setBias();
		LED_RED_F_R_Off();
		LED_RED_F_L_Off();
		calcADC_data(&adcData);
		calcENC_data(&encData);
		calcIMU_data(&imuData);
		set_dist_Bias();
		I_LED_MR_SetVal();I_LED_ML_SetVal();
		I_LED_R_SetVal();I_LED_L_SetVal();
		/* Initial measurement, as first measurement seemed to be corrupted for some reason.*/
		WAIT1_Waitms(1000);
		calcADC_data(&adcData);
		calcIMU_data(&imuData);
		WAIT1_Waitms(1000);
//		LED_GREEN_F_R_On();
//		LED_GREEN_F_L_On();
		if(get_half_U_Bat()> 3.7){
		Distance_INT_EnableEvent();
		initMotors();

		}else{
			 BAT_LOW_ClrVal();
			set_VREF(0,0);
			deinitMotors();

		}


	} else {

	}
	if(JOY_LEFT_GetVal() == 0) {
		//exploration
		/* comend out existing trys */
//		WAIT1_Waitms(1000);
//
//		/* IMU */
//		LED_GREEN_F_R_Off();
//		LED_GREEN_F_L_Off();
//		LED_BLUE_F_R_Off();
//		LED_BLUE_F_L_Off();
//		LED_RED_F_R_On();
//		LED_RED_F_L_On();
//		biasCalc();
//		setBias();
//		LED_RED_F_R_Off();
//		LED_RED_F_L_Off();
//
//		I_LED_R_SetVal();I_LED_L_SetVal();I_LED_ML_SetVal();I_LED_MR_SetVal();
//		/* Initial measurement, as first measurement seemed to be corrupted for some reason.*/
//		WAIT1_Waitms(1000);
//		calcADC_data(&adcData);
//		calcIMU_data(&imuData);
//		WAIT1_Waitms(1000);
////		LED_GREEN_F_R_On();
////		LED_GREEN_F_L_On();
//		if(get_half_U_Bat()> 3.7){
//		PID_Init();
//		exploFinishFlag = FALSE;
//		//initMotors();
////		driveToTurn(0.05);
////		stopp();
//		Distance_INT_EnableEvent();
//		}else{
//			 BAT_LOW_ClrVal();
//			set_VREF(0,0);
//			deinitMotors();
//		}
		/* New Trys */

		LED_GREEN_F_R_Off();
		LED_GREEN_F_L_Off();
		LED_BLUE_F_L_On();
		LED_BLUE_F_R_On();
		reinit_Drving(true);
		//reinit_Explore();
		WAIT1_Waitms(1000);
//		LED_GREEN_F_R_On();
//		LED_GREEN_F_L_On();
		LED_BLUE_F_L_Off();
		LED_BLUE_F_R_Off();

		exploFinishFlag = FALSE;
		WAIT1_Waitms(1000);
		calcADC_data(&adcData);
		calcIMU_data(&imuData);
		WAIT1_Waitms(1000);

		if(get_half_U_Bat()> 3.7){
			MazeSegmentsToBeDriven.segments[0].SingleSegment = 1;
			MazeSegmentsToBeDriven.numberOfSegments = 1;


			Distance_INT_EnableEvent();
			initMotors();
		}else{
			set_VREF(0,0);
			deinitMotors();
			LED_RED_F_R_On();
			WAIT1_Waitms(500);
			BAT_LOW_ClrVal();
		}

	} else {

	}
	if(JOY_DOWN_GetVal() == 0) {
		Init_Maze();
		LED_GREEN_F_R_Off();
		LED_GREEN_F_L_Off();
		LED_BLUE_F_L_On();
		LED_BLUE_F_R_On();
	} else {

	}
	if(JOY_PUSH_GetVal() == 0) {
		LED_GREEN_F_R_On();
		LED_GREEN_F_L_On();
		printSaveData();



#if 0
		printSaveData();
		//printSaveControllerData();
//		calcENC_data(&encData);
//		resetPosition();
#endif
	}
}


void Init_Hardware_Driver(void){
	initIMU();
	PL_Init();
//	HS_MOT_R_Init();
//	HS_MOT_L_Init();
	initMotors();

	CLS_BLE_Init();

}

void Timer_1ms_Int(void){

	ms_Flag = TRUE;
	if(PWR_SW_GetVal()){
		SYSOFF_SetVal();
	}

}

bool getMsFlag(void){
	return ms_Flag;
}

void clearMsFlag(void){
	ms_Flag = FALSE;
}

void Init_GPIOs(void){
	SYSOFF_ClrVal();
	BLE_RST_SetVal();
}

void Init_Maze(void){
#if 0
	MazeSegmentsToBeDriven.segments[0].SingleSegment = 1;
	MazeSegmentsToBeDriven.segments[1].SingleSegment = -90;
	MazeSegmentsToBeDriven.segments[2].SingleSegment = 1;
	MazeSegmentsToBeDriven.segments[3].SingleSegment = 180;
	MazeSegmentsToBeDriven.segments[4].SingleSegment = 1;
	MazeSegmentsToBeDriven.segments[5].SingleSegment = -90;
	MazeSegmentsToBeDriven.segments[6].SingleSegment = 2;
	MazeSegmentsToBeDriven.segments[7].SingleSegment = -180;
	MazeSegmentsToBeDriven.segments[8].SingleSegment = 1;
	MazeSegmentsToBeDriven.segments[9].SingleSegment = 180;
	MazeSegmentsToBeDriven.segments[10].SingleSegment = 3;
	MazeSegmentsToBeDriven.segments[11].SingleSegment = -90;
	MazeSegmentsToBeDriven.segments[12].SingleSegment = 4;
	MazeSegmentsToBeDriven.segments[13].SingleSegment = -90;
	MazeSegmentsToBeDriven.segments[14].SingleSegment = 6;
	MazeSegmentsToBeDriven.segments[15].SingleSegment = -90;
	MazeSegmentsToBeDriven.segments[16].SingleSegment = 3;
	MazeSegmentsToBeDriven.segments[17].SingleSegment = -90;
	MazeSegmentsToBeDriven.segments[18].SingleSegment = 1;
	MazeSegmentsToBeDriven.segments[19].SingleSegment = -90;
	MazeSegmentsToBeDriven.segments[20].SingleSegment = 1;
	MazeSegmentsToBeDriven.segments[21].SingleSegment = 90;
	MazeSegmentsToBeDriven.segments[22].SingleSegment = 1;
	MazeSegmentsToBeDriven.segments[23].SingleSegment = 90;
	MazeSegmentsToBeDriven.segments[24].SingleSegment = 1;
	MazeSegmentsToBeDriven.segments[25].SingleSegment = 180;
	MazeSegmentsToBeDriven.segments[26].SingleSegment = 1;
	MazeSegmentsToBeDriven.segments[27].SingleSegment = -90;

	MazeSegmentsToBeDriven.numberOfSegments = 25;


#else
	MazeSegmentsToBeDriven.segments[0].SingleSegment = 6;
	MazeSegmentsToBeDriven.segments[1].SingleSegment = 90;
	MazeSegmentsToBeDriven.segments[2].SingleSegment = 1;
	MazeSegmentsToBeDriven.segments[3].SingleSegment = 90;
	MazeSegmentsToBeDriven.segments[4].SingleSegment = 7;
	MazeSegmentsToBeDriven.segments[5].SingleSegment = 90;
	MazeSegmentsToBeDriven.segments[6].SingleSegment = 6;
	MazeSegmentsToBeDriven.segments[7].SingleSegment = 90;
	MazeSegmentsToBeDriven.segments[8].SingleSegment = 6;
	MazeSegmentsToBeDriven.segments[9].SingleSegment = 90;
	MazeSegmentsToBeDriven.segments[10].SingleSegment = 5;
	MazeSegmentsToBeDriven.segments[11].SingleSegment = 90;
	MazeSegmentsToBeDriven.segments[12].SingleSegment = 5;
	MazeSegmentsToBeDriven.segments[13].SingleSegment = 90;
	MazeSegmentsToBeDriven.segments[14].SingleSegment = 4;
	MazeSegmentsToBeDriven.segments[15].SingleSegment = 90;
	MazeSegmentsToBeDriven.segments[16].SingleSegment = 4;
	MazeSegmentsToBeDriven.segments[17].SingleSegment = 90;
	MazeSegmentsToBeDriven.segments[18].SingleSegment = 3;
	MazeSegmentsToBeDriven.segments[19].SingleSegment = 90;
	MazeSegmentsToBeDriven.segments[20].SingleSegment = 3;
	MazeSegmentsToBeDriven.segments[21].SingleSegment = 90;
	MazeSegmentsToBeDriven.segments[22].SingleSegment = 2;
	MazeSegmentsToBeDriven.segments[23].SingleSegment = 90;
	MazeSegmentsToBeDriven.segments[24].SingleSegment = 2;
	MazeSegmentsToBeDriven.segments[25].SingleSegment = 90;
	MazeSegmentsToBeDriven.segments[26].SingleSegment = 1;
	MazeSegmentsToBeDriven.segments[27].SingleSegment = 90;
	MazeSegmentsToBeDriven.segments[28].SingleSegment = 1;

	MazeSegmentsToBeDriven.numberOfSegments = 1;
	#endif
}



void quadratureDecoder(void)
{
	//using FTM1 as quadrature module, enable FTM0 and FTM1 clock
	SIM_SCGC6|= SIM_SCGC6_FTM1_MASK; //enable FTM0 and FTM1 module clock
	FTM1_SC=0x00; //initialize the FTM1 module as quadrature mode
	FTM1_MODE|=0x05; //enable FTM1 module
	FTM1_SYNC |=FTM_SYNC_SWSYNC_MASK;
	FTM1_CONF |= FTM_SYNCONF_CNTINC_MASK;
	FTM1_SC |= FTM_SC_PS(0);
	FTM1_CONF |= (FTM_CONF_BDMMODE(3));
	FTM1_CONF &= ~FTM_CONF_GTBEEN_MASK;
	FTM1_OUTINIT = 0;
	FTM1_POL = 0;
	FTM1_EXTTRIG = 0;
	FTM1_CNTIN=0x00;
	FTM1_CNT = 0;
	FTM1_MOD=65535;
	FTM1_CNT = 0;
	FTM1_QDCTRL &=~(FTM_QDCTRL_QUADMODE_MASK | FTM_QDCTRL_PHBPOL_MASK);
	FTM1_QDCTRL |=(FTM_QDCTRL_PHAPOL_MASK);
	FTM1_QDCTRL |=FTM_QDCTRL_QUADEN_MASK; //enable quadrature mode
	FTM1_SC |= FTM_SC_TOIE_MASK; //enable counter overflow interrupt
	  //NVICIP43 = NVIC_IP_PRI43(32);
	  //NVICISER1 |= NVIC_ISER_SETENA((1<<11));
	  FTM1_SC |= FTM_SC_CLKS(1);


	SIM_SCGC6|= SIM_SCGC6_FTM2_MASK; //enable FTM0 and FTM1 module clock
	FTM2_SC=0x00; //initialize the FTM1 module as quadrature mode
	FTM2_MODE|=0x05; //enable FTM1 module
	FTM2_SYNC |=FTM_SYNC_SWSYNC_MASK;
	FTM2_CONF |= FTM_SYNCONF_CNTINC_MASK;
	FTM2_SC |= FTM_SC_PS(0);
	FTM2_CONF |= (FTM_CONF_BDMMODE(3));
	FTM2_CONF &= ~FTM_CONF_GTBEEN_MASK;
	FTM2_OUTINIT = 0;
	FTM2_POL = 0;
	FTM2_EXTTRIG = 0;
	FTM2_CNTIN=0x00;
	FTM2_CNT = 0;
	FTM2_MOD=65535;
	FTM2_CNT = 0;
	FTM2_QDCTRL &=~(FTM_QDCTRL_QUADMODE_MASK | FTM_QDCTRL_PHBPOL_MASK);
	FTM2_QDCTRL |=(FTM_QDCTRL_PHAPOL_MASK);
	FTM2_QDCTRL |=FTM_QDCTRL_QUADEN_MASK; //enable quadrature mode
	FTM2_SC |= FTM_SC_TOIE_MASK; //enable counter overflow interrupt
	//NVICIP44 = NVIC_IP_PRI44(0x40);
	//NVICISER1 |= NVIC_ISER_SETENA((1<<12));
	FTM2_SC |= FTM_SC_CLKS(1);
	FTM1_CNT =0;
	FTM2_CNT =0;
}

void APP_Start(void) {
	static float Pos[3];
	Init_GPIOs();
	WAIT1_Waitms(500);
	for(;;){
		if(initIMU()){
			quadratureDecoder();
			LED_RED_F_R_Off();
			LED_RED_F_L_Off();
			LED_GREEN_F_R_On();
			LED_GREEN_F_L_On();
		  if(get_half_U_Bat()> 3.7){
			  for(;;) {
				if(!PWR_SW_GetVal()){
					testJoystick();
						if(ms_Flag){
							if(exploFinishFlag){
								if(Driving(MazeSegmentsToBeDriven)){
									Distance_INT_DisableEvent();
									set_VREF(0,0);
									deinitMotors();
									LED_GREEN_F_R_Off();
									LED_GREEN_F_L_Off();
									LED_RED_F_R_Off();
									LED_RED_F_L_Off();
									ms_Flag = FALSE;
									I_LED_R_ClrVal();I_LED_L_ClrVal();I_LED_MR_ClrVal();I_LED_ML_ClrVal(); // turn IR leds off
								}
									ms_Flag = FALSE;
							}
							else{//call exploration-fsm here
								if(Driving(MazeSegmentsToBeDriven)){
									static int i=0;
									/* Stop comand */
//									if(i>4){
										Distance_INT_DisableEvent();
										set_VREF(0,0);
										deinitMotors();
										ms_Flag = FALSE;
										I_LED_R_ClrVal();I_LED_L_ClrVal();I_LED_MR_ClrVal();I_LED_ML_ClrVal(); // turn IR leds off

										LED_GREEN_F_R_On();
										LED_GREEN_F_L_On();
										LED_RED_F_R_Off();
										LED_RED_F_L_Off();
//										i=0;
//									}else{
//										i++;
//										MazeSegmentsToBeDriven.numberOfSegments = MazeSegmentsToBeDriven.numberOfSegments+1;
//										MazeSegmentsToBeDriven.segments[MazeSegmentsToBeDriven.numberOfSegments].SingleSegment = 1;
//									}
								}else{
									static float d=1.40;
									if(getPosition(&Pos[0])){
										;//error;
									}
									if(Pos[0]<d){
//										Distance_INT_DisableEvent();
										set_VREF(-0.01,-0.01);
//										MazeSegmentsToBeDriven.segments[0].SingleSegment = 0;
//										MazeSegmentsToBeDriven.numberOfSegments = 0;
										reinit_Drving(true);
										LED_RED_F_L_On();
										d=d-0.1;
										set_VREF(0,0);
										deinitMotors();
										//segmentEnd=true;
//										if(!segEndDetection(get_latest_ADC_data(),&segmentEnd)){
//											//Error;
//										}
									}
								}


								ms_Flag = FALSE;
							}
						}
						EVNT_HandleEvent(APP_EventHandler,TRUE);
				 }else{
						SYSOFF_SetVal();
//						LED_RED_F_R_On();
//						LED_RED_F_L_On();
//						LED_GREEN_F_R_On();
//						LED_GREEN_F_L_On();
				}
			}
		  }else{

			set_VREF(0,0);
			deinitMotors();
			LED_RED_F_R_On();
			WAIT1_Waitms(500);
			BAT_LOW_ClrVal();

		  }
		}
		else{
			LED_RED_F_R_On();
			LED_RED_F_L_On();
		}
	}
}


