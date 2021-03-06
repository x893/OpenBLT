/************************************************************************************//**
* \file         Demo\ARMCM3_EFM32_Olimex_EM32G880F128STK_Crossworks\Prog\timer.c
* \brief        Timer driver source file.
* \ingroup      Prog_ARMCM3_EFM32_Olimex_EM32G880F128STK_Crossworks
* \internal
*----------------------------------------------------------------------------------------
*                          C O P Y R I G H T
*----------------------------------------------------------------------------------------
*   Copyright (c) 2012  by Feaser    http://www.feaser.com    All rights reserved
*
*----------------------------------------------------------------------------------------
*                            L I C E N S E
*----------------------------------------------------------------------------------------
* This file is part of OpenBLT. OpenBLT is free software: you can redistribute it and/or
* modify it under the terms of the GNU General Public License as published by the Free
* Software Foundation, either version 3 of the License, or (at your option) any later
* version.
*
* OpenBLT is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
* without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
* PURPOSE. See the GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with OpenBLT.
* If not, see <http://www.gnu.org/licenses/>.
*
* A special exception to the GPL is included to allow you to distribute a combined work 
* that includes OpenBLT without being obliged to provide the source code for any 
* proprietary components. The exception text is included at the bottom of the license
* file <license.html>.
* 
* \endinternal
****************************************************************************************/

/****************************************************************************************
* Include files
****************************************************************************************/
#include "header.h"                                    /* generic header               */


/****************************************************************************************
* Local data declarations
****************************************************************************************/
/** \brief Local variable for storing the number of milliseconds that have elapsed since
 *         startup.
 */
static unsigned long millisecond_counter;


/************************************************************************************//**
** \brief     Initializes the timer.
** \return    none.
**
****************************************************************************************/
void TimerInit(void)
{
  /* configure the SysTick timer for 1 ms period */
  SysTick_Config(CMU_ClockFreqGet(cmuClock_CORE) / 1000);
  /* reset the millisecond counter */
  TimerSet(0);
} /*** end of TimerInit ***/


/************************************************************************************//**
** \brief     Stops and disables the timer.
** \return    none.
**
****************************************************************************************/
void TimerDeinit(void)
{
  SysTick->CTRL = 0;
} /*** end of TimerDeinit ***/


/************************************************************************************//**
** \brief     Sets the initial counter value of the millisecond timer.
** \param     timer_value initialize value of the millisecond timer.
** \return    none.
**
****************************************************************************************/
void TimerSet(unsigned long timer_value)
{
  /* set the millisecond counter */
  millisecond_counter = timer_value;
} /*** end of TimerSet ***/


/************************************************************************************//**
** \brief     Obtains the counter value of the millisecond timer.
** \return    Current value of the millisecond timer.
**
****************************************************************************************/
unsigned long TimerGet(void)
{
  /* read and return the millisecond counter value */
  return millisecond_counter;
} /*** end of TimerGet ***/


/************************************************************************************//**
** \brief     Interrupt service routine of the timer.
** \return    none.
**
****************************************************************************************/
void TimerISRHandler(void)
{
  /* increment the millisecond counter */
  millisecond_counter++;
} /*** end of TimerISRHandler ***/


/*********************************** end of timer.c ************************************/
