/* ###################################################################
**     Filename    : Events.c
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
** @file Events.c
** @version 01.00
** @brief
**         This is user's event module.
**         Put your event handler code here.
*/         
/*!
**  @addtogroup Events_module Events module documentation
**  @{
*/         
/* MODULE Events */

#include "Cpu.h"
#include "Events.h"
#include "Init_Config.h"
#include "PDD_Includes.h"
#include "Event.h"
#include "Logging.h"
//#include "stdint.h"


#ifdef __cplusplus
extern "C" {
#endif 

uint16_t kFTM_TimeOverflowFlag = (1U << 9);
/* User includes (#include below this line is not maintained by Processor Expert) */


#include "Application.h"



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
void Cap_IR_RC_T_OnCapture(void)
{

}

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
void Cap_IR_DS_R_OnCapture(void)
{
  /* Write your code here ... */

}

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
void Cap_IR_DS_F_OnCapture(void)
{
  /* Write your code here ... */

}

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
void QuadSmaple_OnInterrupt(void)
{
  /* Write your code here ... */
//	HS_MOT_R_Sample();
//	HS_MOT_L_Sample();
	//Get_G_Axes_Int();
}

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
void EVNT1_AppHandleEvent(uint8_t event)
{
  (void)event; /* only to avoid compiler warning about unused variable */
  /* Write your code here ... */
}

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
void Distance_INT_OnInterrupt(void)
{
	//DrivingControl();//saveData(NULL, NULL, NULL, NULL);
	Timer_1ms_Int();/* Write your code here ... */
}

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
void Cpu_OnNMI(void)
{
  /* Write your code here ... */
}

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
void SM1_OnRxChar(void)
{
  /* Write your code here ... */
}

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
void SM1_OnTxChar(void)
{
  /* Write your code here ... */
}

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
void SM1_OnError(void)
{
  /* Write your code here ... */
}

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
void SM1_OnFullRxBuf(void)
{
  /* Write your code here ... */
}

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
void SM1_OnFreeTxBuf(void)
{
  /* Write your code here ... */
}





PE_ISR( blabla1 )
{
	//GPIOC_PTOR=0x80;
	FTM1_SC &= ~FTM_SC_TOF_MASK; //clear overflow flag
	FTM1_STATUS &= ~(kFTM_TimeOverflowFlag & 0xFFU);
	//encCounter++;
	//asm("nop");
}


PE_ISR( blabla2 )
{
	//GPIOC_PTOR=0x80;
	FTM2_SC&= ~FTM_SC_TOF_MASK; //clear overflow flag
	FTM2_STATUS &= ~(kFTM_TimeOverflowFlag & 0xFFU);

	//encCounter++;
	//asm("nop");
}

/* END Events */

#ifdef __cplusplus
}  /* extern "C" */
#endif 

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
