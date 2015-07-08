#include "boot.h"                                /* bootloader generic header          */


/****************************************************************************************
* Macro definitions
****************************************************************************************/
/** \brief Pointer to the user program's reset vector. */
#define CPU_USER_PROGRAM_STARTADDR_PTR    ((uint32_t)(FlashGetUserProgBaseAddress() + 0x00000004))
/** \brief Pointer to the user program's vector table. */
#define CPU_USER_PROGRAM_VECTABLE_OFFSET  ((uint32_t)FlashGetUserProgBaseAddress())


/****************************************************************************************
* Register definitions
****************************************************************************************/
/** \brief Vector table offset register. */
#define SCB_VTOR    (*((volatile uint32_t *) 0xE000ED08))


/****************************************************************************************
* Hook functions
****************************************************************************************/
#if (BOOT_CPU_USER_PROGRAM_START_HOOK > 0)
	extern bool CpuUserProgramStartHook(void);
#endif


/****************************************************************************************
* External functions
****************************************************************************************/
extern void reset_handler(void);	/* implemented in cstart.s	*/


/************************************************************************************//**
** \brief	Starts the user program, if one is present. In this case this function
**			does not return. 
** \return	none.
**
****************************************************************************************/
void CpuStartUserProgram(void)
{
	void (*pProgResetHandler)(void);

	/* check if a user program is present by verifying the checksum */
	if (NvmVerifyChecksum() == false)
	{
		/* not a valid user program so it cannot be started */
		return;
	}

#if (BOOT_CPU_USER_PROGRAM_START_HOOK > 0)
	/* invoke callback */
	if (CpuUserProgramStartHook() == false)
	{
		/* callback requests the user program to not be started */
		return;
	}
#endif

#if (BOOT_COM_ENABLE > 0)
	/* release the communication interface */
	ComFree();
#endif

	/* reset the timer */
	TimerReset();
	/* remap user program's vector table */
	SCB_VTOR = CPU_USER_PROGRAM_VECTABLE_OFFSET & (uint32_t)0x1FFFFF80;
	/* set the address where the bootloader needs to jump to. this is the address of
	 * the 2nd entry in the user program's vector table. this address points to the
	 * user program's reset handler.
	 */
	pProgResetHandler = (void(*)(void))(*((uint32_t*)CPU_USER_PROGRAM_STARTADDR_PTR));
	/* start the user program by activating its reset interrupt service routine */
	pProgResetHandler();
} /*** end of CpuStartUserProgram ***/


/************************************************************************************//**
** \brief	Copies data from the source to the destination address.
** \param	dest Destination address for the data.
** \param	src  Source address of the data.
** \param	len  length of the data in bytes. 
** \return	none.
**
****************************************************************************************/
void CpuMemCopy(uint32_t dest, uint32_t src, uint16_t len)
{
	uint8_t *from, *to;

	/* set casted pointers */
	from = (uint8_t *)src;
	to = (uint8_t *)dest;

	/* copy all bytes from source address to destination address */
	while(len-- > 0)
	{
		*to++ = *from++;	/* store byte value from source to destination */
		CopService();		/* keep the watchdog happy */
	}
} /*** end of CpuMemCopy ***/


/************************************************************************************//**
** \brief	Perform a soft reset of the microcontroller by starting from the reset ISR.
** \return	none.
**
****************************************************************************************/
void CpuReset(void)
{
	/* perform a software reset by calling the reset ISR routine */
	reset_handler();
} /*** end of CpuReset ***/


/*********************************** end of cpu.c **************************************/
/************************************************************************************//**
* \file         Source\ARMCM3_STM32\cpu.c
* \brief        Bootloader cpu module source file.
* \ingroup      Target_ARMCM3_STM32
* \internal
*----------------------------------------------------------------------------------------
*                          C O P Y R I G H T
*----------------------------------------------------------------------------------------
*   Copyright (c) 2011  by Feaser    http://www.feaser.com    All rights reserved
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
