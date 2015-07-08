#include "boot.h"	/* bootloader generic header	*/

#if (BOOT_COM_UART_ENABLE > 0)

#include "stm32f10x.h"

/****************************************************************************************
* Register definitions
****************************************************************************************/
#if (BOOT_COM_UART_CHANNEL_INDEX == 0)
	/** \brief Set UART base address to USART1. */
	#define UARTx		USART1
#elif (BOOT_COM_UART_CHANNEL_INDEX == 1)
	/** \brief Set UART base address to USART2. */
	#define UARTx		USART2
#else
	/** \brief Set UART base address to USART1 by default. */
	#define UARTx		USART1
#endif


/****************************************************************************************
* Function prototypes
****************************************************************************************/
static bool UartReceiveByte(uint8_t *data);
static bool UartTransmitByte(uint8_t data);


/************************************************************************************//**
** \brief	Initializes the UART communication interface.
** \return	none.
**
****************************************************************************************/
void UartInit(void)
{
	/* the current implementation supports USART1 and USART2. throw an assertion error in 
	* case a different UART channel is configured.  
	*/
	ASSERT_CT((BOOT_COM_UART_CHANNEL_INDEX == 0) || (BOOT_COM_UART_CHANNEL_INDEX == 1)); 
	/* first reset the UART configuration. note that this already configures the UART
	* for 1 stopbit, 8 databits and no parity.
	*/
	UARTx->BRR = 0;
	UARTx->CR1 = 0;
	UARTx->CR2 = 0;
	UARTx->CR3 = 0;
	UARTx->GTPR = 0;
	/* configure the baudrate, knowing that PCLKx is configured to be half of
	* BOOT_CPU_SYSTEM_SPEED_KHZ.
	*/
	UARTx->BRR = ((BOOT_CPU_SYSTEM_SPEED_KHZ / 2) * 1000) / BOOT_COM_UART_BAUDRATE;
	/* enable the UART including the transmitter and the receiver */
	UARTx->CR1 |= (USART_CR1_UE | USART_CR1_TE | USART_SR_NE);
} /*** end of UartInit ***/


/************************************************************************************//**
** \brief	Transmits a packet formatted for the communication interface.
** \param	data Pointer to byte array with data that it to be transmitted.
** \param	len  Number of bytes that are to be transmitted.
** \return	none.
**
****************************************************************************************/
bool UartTransmitPacket(uint8_t *data, uint8_t len)
{
	uint16_t data_index;
	bool result;

	/* verify validity of the len-paramenter */
	ASSERT_RT(len <= BOOT_COM_UART_TX_MAX_DATA);

	/* first transmit the length of the packet */  
	result = UartTransmitByte(len);
	ASSERT_RT(result == true);  
	if (result)
	{
		/* transmit all the packet bytes one-by-one */
		for (data_index = 0; data_index < len; data_index++)
		{
			CopService();	/* keep the watchdog happy */
			result = UartTransmitByte(data[data_index]);
			ASSERT_RT(result == true);
			if (result != true)
				break;
		}
	}
	return result;
}

typedef struct {
	uint8_t xcpCtoRxLength;
	bool  xcpCtoRxInProgress;
	uint8_t xcpCtoReqPacket[BOOT_COM_UART_RX_MAX_DATA + 1];  // one extra for length
} UartReqPacket_t;
UartReqPacket_t UartReqPacket;

/************************************************************************************//**
** \brief	Receives a communication interface packet if one is present.
** \param	data Pointer to byte array where the data is to be stored.
** \return	true if a packet was received, false otherwise.
**
****************************************************************************************/
bool UartReceivePacket(uint8_t *data)
{
	UartReqPacket_t *req = &UartReqPacket;
	// start of cto packet received ?
	if ( ! req->xcpCtoRxInProgress)
	{	// store the message length when received
		if (UartReceiveByte(&(req->xcpCtoReqPacket[0])))
		{
			if (req->xcpCtoReqPacket[0] > 0)
			{	// indicate that a cto packet is being received
				req->xcpCtoRxInProgress = true;
				// reset packet data count
				req->xcpCtoRxLength = 0;
			}
		}
	}
	else
	{	// store the next packet byte
		if (UartReceiveByte(&(req->xcpCtoReqPacket[req->xcpCtoRxLength + 1])))
		{	// increment the packet data count
			req->xcpCtoRxLength++;

			// check to see if the entire packet was received
			if (req->xcpCtoRxLength == req->xcpCtoReqPacket[0])
			{	// copy the packet data
				CpuMemCopy((uint32_t)data, (uint32_t)&req->xcpCtoReqPacket[1], req->xcpCtoRxLength);
				// done with cto packet reception
				req->xcpCtoRxInProgress = false;

				// packet reception complete
				return true;
			}
		}
	}
	// packet reception not yet complete
	return false;
}


/************************************************************************************//**
** \brief	Receives a communication interface byte if one is present.
** \param	data Pointer to byte where the data is to be stored.
** \return	true if a byte was received, false otherwise.
**
****************************************************************************************/
static bool UartReceiveByte(uint8_t *data)
{
	/* check if a new byte was received by means of the RDR-bit */
	if ((UARTx->SR & USART_SR_RXNE) != 0)
	{
		/* store the received byte */
		data[0] = UARTx->DR;
		/* inform caller of the newly received byte */
		return true;
	}
	/* inform caller that no new data was received */
	return false;
} /*** end of UartReceiveByte ***/


/************************************************************************************//**
** \brief	Transmits a communication interface byte.
** \param	data Value of byte that is to be transmitted.
** \return	true if the byte was transmitted, false otherwise.
**
****************************************************************************************/
static bool UartTransmitByte(uint8_t data)
{
	/* check if tx holding register can accept new data */
	if ((UARTx->SR & USART_SR_TXE) == 0)
	{
		/* UART not ready. should not happen */
		return false;
	}
	/* write byte to transmit holding register */
	UARTx->DR = data;
	/* wait for tx holding register to be empty */
	while((UARTx->SR & USART_SR_TXE) == 0) 
	{ 
		/* keep the watchdog happy */
		CopService();
	}
	/* byte transmitted */
	return true;
} /*** end of UartTransmitByte ***/

#endif /* BOOT_COM_UART_ENABLE > 0 */

/*********************************** end of uart.c *************************************/
/************************************************************************************//**
* \file         Source\ARMCM3_STM32\uart.c
* \brief        Bootloader UART communication interface source file.
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
