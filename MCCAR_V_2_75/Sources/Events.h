/* ###################################################################
**     Filename    : Events.h
**     Project     : MCCAR_V_tests
**     Processor   : MK24FN1M0VDC12
**     Component   : Events
**     Version     : Driver 01.00
**     Compiler    : GNU C Compiler
**     Date/Time   : 2018-01-25, 16:41, # CodeGen: 0
**     Abstract    :
**         This is user's event module.
**         Put your event handler code here.
**     Contents    :
**         Cpu_OnNMI - void Cpu_OnNMI(void);
**
** ###################################################################*/
/*!
** @file Events.h
** @version 01.00
** @brief
**         This is user's event module.
**         Put your event handler code here.
*/         
/*!
**  @addtogroup Events_module Events module documentation
**  @{
*/         

#ifndef __Events_H
#define __Events_H
/* MODULE Events */

#include "PE_Types.h"
#include "PE_Error.h"
#include "PE_Const.h"
#include "IO_Map.h"
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
#include "WAIT1.h"

#ifdef __cplusplus
extern "C" {
#endif 

extern void Timer_1ms_Int(void);
extern void Get_G_Axes_Int(void);
void blabla1(void);
void blabla2(void);
//extern void Run_PID(void);


/*
** ===================================================================
**     Event       :  Cap_IR_RC_T_OnCapture (module Events)
**
**     Component   :  Cap_IR_RC_T [Capture]
**     Description :
**         This event is called on capturing of Timer/Counter actual
**         value (only when the component is enabled - <Enable> and the
**         events are enabled - <EnableEvent>.This event is available
**         only if a <interrupt service/event> is enabled.
**     Parameters  : None
**     Returns     : Nothing
** ===================================================================
*/
void Cap_IR_RC_T_OnCapture(void);

/*
** ===================================================================
**     Event       :  Cap_IR_DS_R_OnCapture (module Events)
**
**     Component   :  Cap_IR_DS_R [Capture]
**     Description :
**         This event is called on capturing of Timer/Counter actual
**         value (only when the component is enabled - <Enable> and the
**         events are enabled - <EnableEvent>.This event is available
**         only if a <interrupt service/event> is enabled.
**     Parameters  : None
**     Returns     : Nothing
** ===================================================================
*/
void Cap_IR_DS_R_OnCapture(void);

/*
** ===================================================================
**     Event       :  Cap_IR_DS_F_OnCapture (module Events)
**
**     Component   :  Cap_IR_DS_F [Capture]
**     Description :
**         This event is called on capturing of Timer/Counter actual
**         value (only when the component is enabled - <Enable> and the
**         events are enabled - <EnableEvent>.This event is available
**         only if a <interrupt service/event> is enabled.
**     Parameters  : None
**     Returns     : Nothing
** ===================================================================
*/
void Cap_IR_DS_F_OnCapture(void);

/*
** ===================================================================
**     Event       :  QuadSmaple_OnInterrupt (module Events)
**
**     Component   :  QuadSmaple [TimerInt]
**     Description :
**         When a timer interrupt occurs this event is called (only
**         when the component is enabled - <Enable> and the events are
**         enabled - <EnableEvent>). This event is enabled only if a
**         <interrupt service/event> is enabled.
**     Parameters  : None
**     Returns     : Nothing
** ===================================================================
*/
void QuadSmaple_OnInterrupt(void);






void EVNT1_AppHandleEvent(uint8_t event);
/*
** ===================================================================
**     Event       :  EVNT1_AppHandleEvent (module Events)
**
**     Component   :  EVNT1 [SimpleEvents]
**     Description :
**         
**     Parameters  :
**         NAME            - DESCRIPTION
**         event           - Event (event number) to be processed.
**     Returns     : Nothing
** ===================================================================
*/

/*
** ===================================================================
**     Event       :  Distance_INT_OnInterrupt (module Events)
**
**     Component   :  Distance_INT [TimerInt]
**     Description :
**         When a timer interrupt occurs this event is called (only
**         when the component is enabled - <Enable> and the events are
**         enabled - <EnableEvent>). This event is enabled only if a
**         <interrupt service/event> is enabled.
**     Parameters  : None
**     Returns     : Nothing
** ===================================================================
*/
void Distance_INT_OnInterrupt(void);

/*
** ===================================================================
**     Event       :  Cpu_OnNMI (module Events)
**
**     Component   :  Cpu [MK24FN1M0LQ12]
*/
/*!
**     @brief
**         This event is called when the Non maskable interrupt had
**         occurred. This event is automatically enabled when the [NMI
**         interrupt] property is set to 'Enabled'.
*/
/* ===================================================================*/
void Cpu_OnNMI(void);

void SM1_OnRxChar(void);
/*
** ===================================================================
**     Event       :  SM1_OnRxChar (module Events)
**
**     Component   :  SM1 [SynchroMaster]
**     Description :
**         This event is called after a correct character is received.
**         The event is available only when the <Interrupt
**         service/event> property is enabled.
**     Parameters  : None
**     Returns     : Nothing
** ===================================================================
*/

void SM1_OnTxChar(void);
/*
** ===================================================================
**     Event       :  SM1_OnTxChar (module Events)
**
**     Component   :  SM1 [SynchroMaster]
**     Description :
**         This event is called after a character is transmitted.
**     Parameters  : None
**     Returns     : Nothing
** ===================================================================
*/

void SM1_OnError(void);
/*
** ===================================================================
**     Event       :  SM1_OnError (module Events)
**
**     Component   :  SM1 [SynchroMaster]
**     Description :
**         This event is called when a channel error (not the error
**         returned by a given method) occurs. The errors can be read
**         using <GetError> method.
**         The event is available only when the <Interrupt
**         service/event> property is enabled.
**     Parameters  : None
**     Returns     : Nothing
** ===================================================================
*/

void SM1_OnFullRxBuf(void);
/*
** ===================================================================
**     Event       :  SM1_OnFullRxBuf (module Events)
**
**     Component   :  SM1 [SynchroMaster]
**     Description :
**         This event is called when the input buffer is full, i.e.
**         after reception of the last character that was successfully
**         placed into input buffer.
**         This event is available only when the <Interrupt
**         service/event> property is enabled and the <Input buffer
**         size> property is set to non-zero value.
**     Parameters  : None
**     Returns     : Nothing
** ===================================================================
*/

void SM1_OnFreeTxBuf(void);
/*
** ===================================================================
**     Event       :  SM1_OnFreeTxBuf (module Events)
**
**     Component   :  SM1 [SynchroMaster]
**     Description :
**         This event is called after the last character in output
**         buffer is transmitted.
**         This event is available only when the <Interrupt
**         service/event> property is enabled and the <Output buffer
**         size> property is set to non-zero value.
**     Parameters  : None
**     Returns     : Nothing
** ===================================================================
*/

/* END Events */

#ifdef __cplusplus
}  /* extern "C" */
#endif 

#endif 
/* ifndef __Events_H*/
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
