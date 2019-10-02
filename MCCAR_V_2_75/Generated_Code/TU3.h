/* ###################################################################
**     This component module is generated by Processor Expert. Do not modify it.
**     Filename    : TU3.h
**     Project     : MCCAR_V_2_75
**     Processor   : MK24FN1M0VDC12
**     Component   : TimerUnit_LDD
**     Version     : Component 01.164, Driver 01.11, CPU db: 3.00.000
**     Repository  : Kinetis
**     Compiler    : GNU C Compiler
**     Date/Time   : 2019-07-22, 14:22, # CodeGen: 163
**     Abstract    :
**          This TimerUnit component provides a low level API for unified hardware access across
**          various timer devices using the Prescaler-Counter-Compare-Capture timer structure.
**     Settings    :
**          Component name                                 : TU3
**          Module name                                    : FTM0
**          Counter                                        : FTM0_CNT
**          Counter direction                              : Up
**          Counter width                                  : 16 bits
**          Value type                                     : uint16_t
**          Input clock source                             : Internal
**            Counter frequency                            : 468.75 kHz
**          Counter restart                                : On-overrun
**            Overrun period                               : Auto select
**            Interrupt                                    : Disabled
**          Channel list                                   : 1
**            Channel 0                                    : 
**              Mode                                       : Compare
**                Compare                                  : FTM0_C0V
**                Offset                                   : 500 �s
**                Output on compare                        : Disconnect
**                Interrupt                                : Enabled
**                  Interrupt                              : INT_FTM0
**                  Interrupt priority                     : medium priority
**          Initialization                                 : 
**            Enabled in init. code                        : yes
**            Auto initialization                          : no
**            Event mask                                   : 
**              OnCounterRestart                           : Disabled
**              OnChannel0                                 : Enabled
**              OnChannel1                                 : Disabled
**              OnChannel2                                 : Disabled
**              OnChannel3                                 : Disabled
**              OnChannel4                                 : Disabled
**              OnChannel5                                 : Disabled
**              OnChannel6                                 : Disabled
**              OnChannel7                                 : Disabled
**          CPU clock/configuration selection              : 
**            Clock configuration 0                        : This component enabled
**            Clock configuration 1                        : This component disabled
**            Clock configuration 2                        : This component disabled
**            Clock configuration 3                        : This component disabled
**            Clock configuration 4                        : This component disabled
**            Clock configuration 5                        : This component disabled
**            Clock configuration 6                        : This component disabled
**            Clock configuration 7                        : This component disabled
**     Contents    :
**         Init            - LDD_TDeviceData* TU3_Init(LDD_TUserData *UserDataPtr);
**         SetEventMask    - LDD_TError TU3_SetEventMask(LDD_TDeviceData *DeviceDataPtr, LDD_TEventMask...
**         GetEventMask    - LDD_TEventMask TU3_GetEventMask(LDD_TDeviceData *DeviceDataPtr);
**         GetCounterValue - TU3_TValueType TU3_GetCounterValue(LDD_TDeviceData *DeviceDataPtr);
**         SetOffsetTicks  - LDD_TError TU3_SetOffsetTicks(LDD_TDeviceData *DeviceDataPtr, uint8_t...
**         GetOffsetTicks  - LDD_TError TU3_GetOffsetTicks(LDD_TDeviceData *DeviceDataPtr, uint8_t...
**
**     Copyright : 1997 - 2015 Freescale Semiconductor, Inc. 
**     All Rights Reserved.
**     
**     Redistribution and use in source and binary forms, with or without modification,
**     are permitted provided that the following conditions are met:
**     
**     o Redistributions of source code must retain the above copyright notice, this list
**       of conditions and the following disclaimer.
**     
**     o Redistributions in binary form must reproduce the above copyright notice, this
**       list of conditions and the following disclaimer in the documentation and/or
**       other materials provided with the distribution.
**     
**     o Neither the name of Freescale Semiconductor, Inc. nor the names of its
**       contributors may be used to endorse or promote products derived from this
**       software without specific prior written permission.
**     
**     THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
**     ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
**     WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
**     DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
**     ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
**     (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
**     LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
**     ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
**     (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
**     SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
**     
**     http: www.freescale.com
**     mail: support@freescale.com
** ###################################################################*/
/*!
** @file TU3.h
** @version 01.11
** @brief
**          This TimerUnit component provides a low level API for unified hardware access across
**          various timer devices using the Prescaler-Counter-Compare-Capture timer structure.
*/         
/*!
**  @addtogroup TU3_module TU3 module documentation
**  @{
*/         

#ifndef __TU3_H
#define __TU3_H

/* MODULE TU3. */

/* Include shared modules, which are used for whole project */
#include "PE_Types.h"
#include "PE_Error.h"
#include "PE_Const.h"
#include "IO_Map.h"
/* Include inherited beans */

#include "FTM_PDD.h"
#include "Cpu.h"

#ifdef __cplusplus
extern "C" {
#endif 


#ifndef __BWUserType_TU3_TValueType
#define __BWUserType_TU3_TValueType
  typedef uint16_t TU3_TValueType ;    /* Type for data parameters of methods */
#endif
#define TU3_CNT_INP_FREQ_U_0 0x0007270EUL /* Counter input frequency in Hz */
#define TU3_CNT_INP_FREQ_R_0 468750.07324219897F /* Counter input frequency in Hz */
#define TU3_CNT_INP_FREQ_COUNT 0U      /* Count of predefined counter input frequencies */
#define TU3_PERIOD_TICKS   0x00010000UL /* Initialization value of period in 'counter ticks' */
#define TU3_NUMBER_OF_CHANNELS 0x01U   /* Count of predefined channels */
#define TU3_COUNTER_WIDTH  0x10U       /* Counter width in bits  */
#define TU3_COUNTER_DIR    DIR_UP      /* Direction of counting */
#define TU3_OFFSET_0_TICKS 0xEAul      /* Initialization value of offset as 'counter ticks' for channel 0 */
/*! Peripheral base address of a device allocated by the component. This constant can be used directly in PDD macros. */
#define TU3_PRPH_BASE_ADDRESS  0x40038000U
  
/* Methods configuration constants - generated for all enabled component's methods */
#define TU3_Init_METHOD_ENABLED        /*!< Init method of the component TU3 is enabled (generated) */
#define TU3_SetEventMask_METHOD_ENABLED /*!< SetEventMask method of the component TU3 is enabled (generated) */
#define TU3_GetEventMask_METHOD_ENABLED /*!< GetEventMask method of the component TU3 is enabled (generated) */
#define TU3_GetCounterValue_METHOD_ENABLED /*!< GetCounterValue method of the component TU3 is enabled (generated) */
#define TU3_SetOffsetTicks_METHOD_ENABLED /*!< SetOffsetTicks method of the component TU3 is enabled (generated) */
#define TU3_GetOffsetTicks_METHOD_ENABLED /*!< GetOffsetTicks method of the component TU3 is enabled (generated) */

/* Events configuration constants - generated for all enabled component's events */
#define TU3_OnChannel0_EVENT_ENABLED   /*!< OnChannel0 event of the component TU3 is enabled (generated) */



/*
** ===================================================================
**     Method      :  TU3_Init (component TimerUnit_LDD)
*/
/*!
**     @brief
**         Initializes the device. Allocates memory for the device data
**         structure, allocates interrupt vectors and sets interrupt
**         priority, sets pin routing, sets timing, etc. If the
**         property ["Enable in init. code"] is set to "yes" value then
**         the device is also enabled (see the description of the
**         [Enable] method). In this case the [Enable] method is not
**         necessary and needn't to be generated. This method can be
**         called only once. Before the second call of Init the [Deinit]
**         must be called first.
**     @param
**         UserDataPtr     - Pointer to the user or
**                           RTOS specific data. This pointer will be
**                           passed as an event or callback parameter.
**     @return
**                         - Pointer to the dynamically allocated private
**                           structure or NULL if there was an error.
*/
/* ===================================================================*/
LDD_TDeviceData* TU3_Init(LDD_TUserData *UserDataPtr);

/*
** ===================================================================
**     Method      :  TU3_SetEventMask (component TimerUnit_LDD)
*/
/*!
**     @brief
**         Enables/disables event(s). The events contained within the
**         mask are enabled. Events not contained within the mask are
**         disabled. The component event masks are defined in the
**         PE_Types.h file. Note: Event that are not generated (See the
**         "Events" tab in the Component inspector) are not handled by
**         this method. In this case the method returns ERR_PARAM_MASK
**         error code. See also method [GetEventMask].
**     @param
**         DeviceDataPtr   - Device data structure
**                           pointer returned by [Init] method.
**     @param
**         EventMask       - Event mask
**     @return
**                         - Error code, possible codes:
**                           ERR_OK - OK
**                           ERR_SPEED - The component does not work in
**                           the active clock configuration
**                           ERR_PARAM_MASK - Event mask is not valid
*/
/* ===================================================================*/
LDD_TError TU3_SetEventMask(LDD_TDeviceData *DeviceDataPtr, LDD_TEventMask EventMask);

/*
** ===================================================================
**     Method      :  TU3_GetEventMask (component TimerUnit_LDD)
*/
/*!
**     @brief
**         Returns current events mask. Note: Event that are not
**         generated (See the "Events" tab in the Component inspector)
**         are not handled by this method. See also method
**         [SetEventMask].
**     @param
**         DeviceDataPtr   - Device data structure
**                           pointer returned by [Init] method.
**     @return
**                         - Current EventMask.
**                           The component event masks are defined in
**                           the PE_Types.h file.
*/
/* ===================================================================*/
LDD_TEventMask TU3_GetEventMask(LDD_TDeviceData *DeviceDataPtr);

/*
** ===================================================================
**     Method      :  TU3_GetCounterValue (component TimerUnit_LDD)
*/
/*!
**     @brief
**         Returns the content of counter register. This method can be
**         used both if counter is enabled and if counter is disabled.
**         The method is not available if HW doesn't allow reading of
**         the counter.
**     @param
**         DeviceDataPtr   - Device data structure
**                           pointer returned by [Init] method.
**     @return
**                         - Counter value (number of counted ticks).
*/
/* ===================================================================*/
TU3_TValueType TU3_GetCounterValue(LDD_TDeviceData *DeviceDataPtr);

/*
** ===================================================================
**     Method      :  TU3_SetOffsetTicks (component TimerUnit_LDD)
*/
/*!
**     @brief
**         Sets the new offset value to channel specified by the
**         parameter ChannelIdx. It is user responsibility to use value
**         below selected period. This method is available when at
**         least one channel is configured.
**     @param
**         DeviceDataPtr   - Device data structure
**                           pointer returned by [Init] method.
**     @param
**         ChannelIdx      - Index of the component
**                           channel.
**     @param
**         Ticks           - Number of counter ticks to compare
**                           match.
**     @return
**                         - Error code, possible codes:
**                           ERR_OK - OK 
**                           ERR_PARAM_INDEX - ChannelIdx parameter is
**                           out of possible range.
**                           ERR_NOTAVAIL -  The compare mode is not
**                           selected for selected channel
**                           ERR_PARAM_TICKS - Ticks parameter is out of
**                           possible range.
**                           ERR_SPEED - The component does not work in
**                           the active clock configuration
*/
/* ===================================================================*/
LDD_TError TU3_SetOffsetTicks(LDD_TDeviceData *DeviceDataPtr, uint8_t ChannelIdx, TU3_TValueType Ticks);

/*
** ===================================================================
**     Method      :  TU3_GetOffsetTicks (component TimerUnit_LDD)
*/
/*!
**     @brief
**         Returns the number of counter ticks to compare match channel
**         specified by the parameter ChannelIdx. See also method
**         [SetOffsetTicks]. This method is available when at least one
**         channel is configured.
**     @param
**         DeviceDataPtr   - Device data structure
**                           pointer returned by [Init] method.
**     @param
**         ChannelIdx      - Index of the component
**                           channel.
**     @param
**         TicksPtr        - Pointer to return value of the
**                           number of counter ticks to compare match.
**     @return
**                         - Error code, possible codes:
**                           ERR_OK - OK 
**                           ERR_PARAM_INDEX - ChannelIdx parameter is
**                           out of possible range.
**                           ERR_NOTAVAIL -  The compare mode is not
**                           selected for selected channel.
**                           ERR_SPEED - The component does not work in
**                           the active clock configuration
*/
/* ===================================================================*/
LDD_TError TU3_GetOffsetTicks(LDD_TDeviceData *DeviceDataPtr, uint8_t ChannelIdx, TU3_TValueType *TicksPtr);

/*
** ===================================================================
**     Method      :  TU3_Interrupt (component TimerUnit_LDD)
**
**     Description :
**         The method services the interrupt of the selected peripheral(s)
**         and eventually invokes event(s) of the component.
**         This method is internal. It is used by Processor Expert only.
** ===================================================================
*/
/* {Default RTOS Adapter} ISR function prototype */
PE_ISR(TU3_Interrupt);

/* END TU3. */

#ifdef __cplusplus
}  /* extern "C" */
#endif 

#endif
/* ifndef __TU3_H */
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
