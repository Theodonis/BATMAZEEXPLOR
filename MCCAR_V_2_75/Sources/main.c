/* ###################################################################
**     Filename    : main.c
**     Project     : MCCAR_V_tests
**     Processor   : MK24FN1M0VDC12
**     Version     : Driver 01.01
**     Compiler    : GNU C Compiler
**     Date/Time   : 2018-01-25, 16:41, # CodeGen: 0
**     Abstract    :
**         Main module.
**         This module contains user's application code.
**     Settings    :
**     Contents    :
**         No public methods
**
** ###################################################################*/
/*!
** @file main.c
** @version 01.01
** @brief
**         Main module.
**         This module contains user's application code.
*/         
/*!
**  @addtogroup main_module main module documentation
**  @{
*/         
/* MODULE main */



/* Including needed modules to compile this module/procedure */
#include "Cpu.h"
#include "Events.h"
#include "LED_BLUE_F_L.h"
#include "LEDpin1.h"
#include "BitIoLdd1.h"
#include "MCUC1.h"
#include "LED_RED_F_L.h"
#include "LEDpin2.h"
#include "BitIoLdd2.h"
#include "LED_GREEN_F_L.h"
#include "LEDpin3.h"
#include "BitIoLdd3.h"
#include "LED_BLUE_F_R.h"
#include "LEDpin4.h"
#include "BitIoLdd4.h"
#include "LED_RED_F_R.h"
#include "LEDpin5.h"
#include "BitIoLdd5.h"
#include "LED_GREEN_F_R.h"
#include "LEDpin6.h"
#include "BitIoLdd6.h"
#include "GI2C1.h"
#include "CI2C1.h"
#include "BitIoLdd14.h"
#include "DRV2_FAULT.h"
#include "BitIoLdd15.h"
#include "DRV2_EN.h"
#include "BitIoLdd21.h"
#include "DRV2_PH.h"
#include "BitIoLdd22.h"
#include "DRV2_PMODE.h"
#include "BitIoLdd23.h"
#include "I_Motor_L.h"
#include "DacLdd2.h"
#include "IMU_INT.h"
#include "BitIoLdd24.h"
#include "BLE_CONNECTED.h"
#include "BitIoLdd25.h"
#include "PWR_SW.h"
#include "BitIoLdd26.h"
#include "BAT_LOW.h"
#include "BitIoLdd29.h"
#include "CHARGE_STATUS.h"
#include "BitIoLdd30.h"
#include "I_LED_SL.h"
#include "BitIoLdd32.h"
#include "SM1.h"
#include "SMasterLdd1.h"
#include "SLEEP_R.h"
#include "BitIoLdd33.h"
#include "SLEEP_L.h"
#include "BitIoLdd34.h"
#include "Pins1.h"
#include "I_LED_L.h"
#include "BitIoLdd35.h"
#include "I_LED_ML.h"
#include "BitIoLdd36.h"
#include "I_LED_MR.h"
#include "BitIoLdd37.h"
#include "I_LED_R.h"
#include "BitIoLdd38.h"
#include "I_LED_SR.h"
#include "BitIoLdd39.h"
#include "BLE_RST.h"
#include "BitIoLdd40.h"
#include "SYSOFF.h"
#include "BitIoLdd41.h"
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
#include "I_Motor_R.h"
#include "DacLdd1.h"
#include "Kill.h"
#include "BitIoLdd27.h"
#include "PWR_BTN.h"
#include "BitIoLdd28.h"

#include "BLE_Mode.h"
#include "BitIoLdd31.h"
#include "ADC_0.h"
#include "AdcLdd2.h"
#include "TU5.h"
#include "Distance_INT.h"
#include "TimerIntLdd1.h"
#include "TU3.h"
#include "FC1.h"
#include "FreeCntrLdd1.h"
#include "ADC_1.h"
#include "AdcLdd3.h"
#include "DRV1_FAULT.h"
#include "BitIoLdd7.h"
#include "DRV1_PH.h"
#include "BitIoLdd8.h"
#include "DRV1_EN.h"
#include "BitIoLdd10.h"
#include "DRV1_PMODE.h"
#include "PE_Types.h"
#include "PE_Error.h"
#include "PE_Const.h"
#include "IO_Map.h"
#include "PDD_Includes.h"
#include "Init_Config.h"

#include "WAIT1.h"

#include "Application.h"



/*lint -save  -e970 Disable MISRA rule (6.3) checking. */
int main(void)
/*lint -restore Enable MISRA rule (6.3) checking. */
{
  /* Write your local variable definition here */

  /*** Processor Expert internal initialization. DON'T REMOVE THIS CODE!!! ***/


	PE_low_level_init();
  /*** End of Processor Expert internal initialization.                    ***/

	APP_Start();

  /*** Don't write any code pass this line, or it will be deleted during code generation. ***/
  /*** RTOS startup code. Macro PEX_RTOS_START is defined by the RTOS component. DON'T MODIFY THIS CODE!!! ***/
  #ifdef PEX_RTOS_START
    PEX_RTOS_START();                  /* Startup of the selected RTOS. Macro is defined by the RTOS component. */
  #endif
  /*** End of RTOS startup code.  ***/
  /*** Processor Expert end of main routine. DON'T MODIFY THIS CODE!!! ***/
  for(;;){}
  /*** Processor Expert end of main routine. DON'T WRITE CODE BELOW!!! ***/
} /*** End of main routine. DO NOT MODIFY THIS TEXT!!! ***/

/* END main */
/*!
** @}
*/
/*
** ###################################################################
**
**     This file was created by Processor Expert 10.5 [05.21]
**     for the Freescale Kinetis series of microcontrollers.
**
** ###################################################################
*/
