#include "boot.h"	/* bootloader generic header	*/

#if (BOOT_COM_CAN_ENABLE > 0)

#include "stm32f10x.h"

/****************************************************************************************
* Register definitions
****************************************************************************************/
/** \brief Macro for accessing CAN controller registers. */
#if (BOOT_COM_CAN_CHANNEL_INDEX == 0)
	#define CANx		CAN1
#elif (BOOT_COM_CAN_CHANNEL_INDEX == 1)
	#define CANx		CAN2
#else
	/** \brief Set CAN base address to CAN1 by default. */
	#define CANx		CAN1
#endif

/****************************************************************************************
* Type definitions
****************************************************************************************/
/** \brief Structure type for grouping CAN bus timing related information. */
typedef struct t_can_bus_timing
{
	uint8_t tseg1;	/**< CAN time segment 1	*/
	uint8_t tseg2;	/**< CAN time segment 2	*/
} tCanBusTiming;

typedef struct t_can_bus_bitrate
{
	uint16_t prescale;
	tCanBusTiming tseg;
} tCanBusPrescale;

/****************************************************************************************
* Local constant declarations
****************************************************************************************/
/** \brief CAN bittiming table for dynamically calculating the bittiming settings.
 *  \details According to the CAN protocol 1 bit-time can be made up of between 8..25 
 *           time quanta (TQ). The total TQ in a bit is SYNC + TSEG1 + TSEG2 with SYNC 
 *           always being 1. The sample point is (SYNC + TSEG1) / (SYNC + TSEG1 + SEG2) * 
 *           100%. This array contains possible and valid time quanta configurations with
 *           a sample point between 68..78%.
 */
static const tCanBusTiming canTiming[] =
{						/*  TQ | TSEG1 | TSEG2 | SP  */
						/* ------------------------- */
	{  5, 2 },	/*   8 |   5   |   2   | 75% */
	{  6, 2 },	/*   9 |   6   |   2   | 78% */
	{  6, 3 },	/*  10 |   6   |   3   | 70% */
	{  7, 3 },	/*  11 |   7   |   3   | 73% */
	{  8, 3 },	/*  12 |   8   |   3   | 75% */
	{  9, 3 },	/*  13 |   9   |   3   | 77% */
	{  9, 4 },	/*  14 |   9   |   4   | 71% */
	{ 10, 4 },	/*  15 |  10   |   4   | 73% */
	{ 11, 4 },	/*  16 |  11   |   4   | 75% */
	{ 12, 4 },	/*  17 |  12   |   4   | 76% */
	{ 12, 5 },	/*  18 |  12   |   5   | 72% */
	{ 13, 5 },	/*  19 |  13   |   5   | 74% */
	{ 14, 5 },	/*  20 |  14   |   5   | 75% */
	{ 15, 5 },	/*  21 |  15   |   5   | 76% */
	{ 15, 6 },	/*  22 |  15   |   6   | 73% */
	{ 16, 6 },	/*  23 |  16   |   6   | 74% */
	{ 16, 7 },	/*  24 |  16   |   7   | 71% */
	{ 16, 8 }	/*  25 |  16   |   8   | 68% */
};


/************************************************************************************//**
** \brief     Search algorithm to match the desired baudrate to a possible bus 
**            timing configuration.
** \param     baud The desired baudrate in kbps. Valid values are 10..1000.
** \param     prescaler Pointer to where the value for the prescaler will be stored.
** \param     tseg1 Pointer to where the value for TSEG2 will be stored.
** \param     tseg2 Pointer to where the value for TSEG2 will be stored.
** \return    true if the CAN bustiming register values were found, false 
**            otherwise.
**
****************************************************************************************/
static bool CanGetSpeedConfig(uint16_t baud, tCanBusPrescale *scale)
{
	uint8_t  cnt;

	/* loop through all possible time quanta configurations to find a match */
	for (cnt = 0; cnt < sizeof(canTiming) / sizeof(canTiming[0]); cnt++)
	{
		if (((BOOT_CPU_SYSTEM_SPEED_KHZ / 2) % (baud * (canTiming[cnt].tseg1 + canTiming[cnt].tseg2 + 1))) == 0)
		{
			/* compute the prescaler that goes with this TQ configuration */
			scale->prescale = (BOOT_CPU_SYSTEM_SPEED_KHZ / 2) / (baud * (canTiming[cnt].tseg1 + canTiming[cnt].tseg2 + 1));

			/* make sure the prescaler is valid */
			if ( (scale->prescale > 0) && (scale->prescale <= 1024) )
			{
				/* store the bustiming configuration */
				scale->tseg.tseg1 = canTiming[cnt].tseg1;
				scale->tseg.tseg2 = canTiming[cnt].tseg2;
				/* found a good bus timing configuration */
				return true;
			}
		}
	}
	/* could not find a good bus timing configuration */
	return false;
}


/* Time out for CAN operations */
#define CAN_INAK_TIMEOUT	((uint32_t)0x0000FFFF)
#define CAN_RESET_TIMEOUT	((uint32_t)0x0000FFFF)
#define CAN_TXRQ_TIMEOUT	((uint32_t)0x0000FFFF)

/************************************************************************************//**
** \brief     Initializes the CAN controller and synchronizes it to the CAN bus.
** \return    none.
**
****************************************************************************************/
bool CanInit(void)
{
	tCanBusPrescale scale;
	bool	result;
	uint32_t	wait_ack;
	CAN_TypeDef *can = CANx;

	/* the current implementation supports CAN1. throw an assertion error in case a 
	 * different CAN channel is configured.  
	 */
	ASSERT_CT(BOOT_COM_CAN_CHANNEL_INDEX == 0);

	/* obtain bittiming configuration information */
	result = CanGetSpeedConfig(BOOT_COM_CAN_BAUDRATE / 1000, &scale);
	ASSERT_RT(result == true);
	if (result != true)
		return false;

	/* set request to reset the can controller */
	can->MCR |= CAN_MCR_RESET ;
	/* wait for acknowledge that the can controller was reset */
	wait_ack = CAN_RESET_TIMEOUT;
	while ((can->MCR & CAN_MCR_RESET) != 0 && wait_ack != 0)
	{
		--wait_ack;
		CopService();	/* keep the watchdog happy */
	}
	if (wait_ack == 0)
		return false;

	/* exit from sleep mode, which is the default mode after reset */
	can->MCR &= ~CAN_MCR_SLEEP;
	/* set request to enter initialisation mode */
	can->MCR |= CAN_MCR_INRQ ;
	/* wait for acknowledge that initialization mode was entered */
	wait_ack = CAN_INAK_TIMEOUT;
	while ((can->MSR & CAN_MSR_INAK) == 0 && wait_ack != 0)
	{
		--wait_ack;
		CopService();	/* keep the watchdog happy */
	}
	if (wait_ack == 0)
		return false;

	/* Set the no automatic retransmission */
	can->MCR &= ~(	(uint32_t)CAN_MCR_TTCM |	/* Set the time triggered communication mode */
					(uint32_t)CAN_MCR_ABOM |	/* Set the automatic bus-off management */
					(uint32_t)CAN_MCR_AWUM |	/* Set the automatic wake-up mode */
					(uint32_t)CAN_MCR_RFLM |	/* Set the receive FIFO locked mode */
					(uint32_t)CAN_MCR_TXFP |	/* Set the transmit FIFO priority */
					0);
	can->MCR |= (	(uint32_t)CAN_MCR_NART |	/* Set the no automatic retransmission */
					(uint32_t)0x00010000   |	/* Enable Debug Freeze  */
					0);

	/* configure the bittming */
	can->BTR =	(0x01 << 30) |	/* Loopback mode */
				// (0x00 << 30) |	/* Normal mode */
				(0x00 << 24) |	/* SJW */
				(uint32_t)((uint32_t)(scale.tseg.tseg1 - 1) << 16) |
				(uint32_t)((uint32_t)(scale.tseg.tseg2 - 1) << 20) |
				(uint32_t)(scale.prescale - 1);
	/* set request to leave initialisation mode */
	can->MCR &= ~CAN_MCR_INRQ;
	/* wait for acknowledge that initialization mode was exited */
	wait_ack = CAN_INAK_TIMEOUT;
	while ((can->MSR & CAN_MSR_INAK) != 0 && wait_ack != 0)
	{
		--wait_ack;
		/* keep the watchdog happy */
		CopService();
	}
	if (wait_ack == 0)
		return false;

	/* enter initialisation mode for the acceptance filter */
	can->FMR |= CAN_FMR_FINIT;
	/* deactivate filter 0 */
	can->FA1R &= ~CAN_FA1R_FACT0;
	/* 32-bit scale for the filter */
	can->FS1R |= CAN_FA1R_FACT0;
	/* open up the acceptance filter to receive all messages */
	can->sFilterRegister[0].FR1 = 0; 
	can->sFilterRegister[0].FR2 = 0; 
	/* select id/mask mode for the filter */
	can->FM1R &= ~CAN_FA1R_FACT0;
	/* FIFO 0 assignation for the filter */
	can->FFA1R &= ~CAN_FA1R_FACT0;
	/* filter activation */
	can->FA1R |= CAN_FA1R_FACT0;
	/* leave initialisation mode for the acceptance filter */
	can->FMR &= ~CAN_FMR_FINIT;

	return true;
}


/************************************************************************************//**
** \brief     Transmits a packet formatted for the communication interface.
** \param     data Pointer to byte array with data that it to be transmitted.
** \param     len  Number of bytes that are to be transmitted.
** \return    none.
**
****************************************************************************************/
bool CanTransmitPacket(uint8_t *data, uint8_t len)
{
	uint32_t	wait_ack;
	CAN_TypeDef *can = CANx;

	/* make sure that transmit mailbox 0 is available */
	ASSERT_RT((can->TSR & CAN_TSR_TME0) == CAN_TSR_TME0);

	/* store the 11-bit message identifier */
	can->sTxMailBox[0].TIR &= CAN_TI0R_TXRQ;
	can->sTxMailBox[0].TIR |= ((uint32_t)BOOT_COM_CAN_TX_MSG_ID << 21);

	/* store the message date length code (DLC) */
	can->sTxMailBox[0].TDTR = len;

	/* store the message data bytes */
	can->sTxMailBox[0].TDLR = *(((uint32_t *)(data + 0)));
	can->sTxMailBox[0].TDHR = *(((uint32_t *)(data + 4)));

	/* request the start of message transmission */
	can->sTxMailBox[0].TIR |= CAN_TI0R_TXRQ;
	/* wait for transmit completion with timeout */
	wait_ack = CAN_TXRQ_TIMEOUT;
	while ((can->TSR & CAN_TSR_TME0) == 0 && wait_ack != 0)
	{
		--wait_ack;
		CopService();	/* keep the watchdog happy */
	}
	if (wait_ack == 0)
		return false;

	return true;
}


/************************************************************************************//**
** \brief     Receives a communication interface packet if one is present.
** \param     data Pointer to byte array where the data is to be stored.
** \return    true is a packet was received, false otherwise.
**
****************************************************************************************/
bool CanReceivePacket(uint8_t *data)
{
	uint32_t rxMsgId;
	uint8_t dlc;
	bool   result = false;
	CAN_TypeDef *can = CANx;

	// check if a new message was received (only FIFO 0 used
	if ((can->RF0R & CAN_RF0R_FMP0) != 0)
	{
		// read out the message identifier
		if ((CANx->sFIFOMailBox[0].RIR & CAN_RI0R_IDE) == 0) // CAN_Id_Standard
			rxMsgId = (can->sFIFOMailBox[0].RIR >> 21) & (uint32_t)0x000007FF;
		else
			rxMsgId = (can->sFIFOMailBox[0].RIR >> 3) & (uint32_t)0x1FFFFFFF;

		// is this the packet identifier
		if (rxMsgId == BOOT_COM_CAN_RX_MSG_ID)
		{
			result = true;
			dlc = can->sFIFOMailBox[0].RDTR & (uint8_t)0x0F;
			*(((uint32_t *)(data + 0))) = (uint32_t)can->sFIFOMailBox[0].RDLR;
			*(((uint32_t *)(data + 4))) = (uint32_t)can->sFIFOMailBox[0].RDHR;
		}
		can->RF0R |= CAN_RF0R_RFOM0;	// release FIFO0
	}
	return result;
}
#endif /* BOOT_COM_CAN_ENABLE > 0 */


/*********************************** end of can.c **************************************/
/************************************************************************************//**
* \file         Source\ARMCM3_STM32\can.c
* \brief        Bootloader CAN communication interface source file.
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
