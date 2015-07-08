#include "boot.h"                                /* bootloader generic header          */

#if (BOOT_COM_USB_ENABLE > 0)

#include "usb_lib.h"                             /* USB library driver header          */
#include "usb_desc.h"                            /* USB descriptor header              */
#include "usb_pwr.h"                             /* USB power management header        */
#include "usb_istr.h"                            /* USB interrupt routine header       */


/****************************************************************************************
* Macro definitions
****************************************************************************************/
/** \brief Total number of fifo buffers. */
#define FIFO_MAX_BUFFERS         (2)
/** \brief Invalid value for a fifo buffer handle. */
#define FIFO_ERR_INVALID_HANDLE  (255)
/** \brief Number of bytes that fit in the fifo pipe. */
#define FIFO_PIPE_SIZE           (64)


/****************************************************************************************
* Type definitions
****************************************************************************************/
/** \brief Structure type for fifo control. */
typedef struct t_fifo_ctrl {
	uint8_t *startptr;	/**< pointer to start of buffer     */
	uint8_t *endptr;	/**< pointer to end of buffer       */
	uint8_t *readptr;	/**< pointer to next read location  */
	uint8_t *writeptr;	/**< pointer to next free location  */
	uint8_t length;		/**< number of buffer elements      */
	uint8_t entries;	/**< # of full buffer elements      */
	uint8_t handle;		/**< handle of the buffer           */ 
	struct  t_fifo_ctrl	*fifoctrlptr;	/**< pointer to free buffer control */
} tFifoCtrl;

/** \brief Structure type for a fifo pipe. */
typedef struct
{
	uint8_t handle;					/**< fifo handle                    */
	uint8_t data[FIFO_PIPE_SIZE];	/**< fifo data buffer               */
} tFifoPipe;						/**< USB pipe fifo type             */


/****************************************************************************************
* Hook functions
****************************************************************************************/
extern void UsbEnterLowPowerModeHook(void);
extern void UsbLeaveLowPowerModeHook(void);
extern void UsbConnectHook(bool connect);


/****************************************************************************************
* Function prototypes
****************************************************************************************/
static bool  UsbReceiveByte(uint8_t *data);
static bool  UsbTransmitByte(uint8_t data);
static void      UsbFifoMgrInit(void);
static uint8_t UsbFifoMgrCreate(uint8_t *buffer, uint8_t length);
static bool  UsbFifoMgrWrite(uint8_t handle, uint8_t data);
static bool  UsbFifoMgrRead(uint8_t handle, uint8_t *data);
static uint8_t UsbFifoMgrScan(uint8_t handle);


/****************************************************************************************
* Local data declarations
****************************************************************************************/
/** \brief Local variable that holds the fifo control structures. */
static tFifoCtrl  fifoCtrl[FIFO_MAX_BUFFERS];
/** \brief Local pointer that points to the next free fifo control structure. */
static tFifoCtrl *fifoCtrlFree;
/** \brief Fifo pipe used for the bulk in endpoint. */
static tFifoPipe  fifoPipeBulkIN;
/** \brief Fifo pipe used for the bulk out endpoint. */
static tFifoPipe  fifoPipeBulkOUT;


/************************************************************************************//**
** \brief     Initializes the USB communication interface.
** \return    none.
**
****************************************************************************************/
void UsbInit(void)
{
	/* initialize the FIFO manager */
	UsbFifoMgrInit();
	/* place 2 buffers under FIFO management */
	fifoPipeBulkIN.handle  = UsbFifoMgrCreate(fifoPipeBulkIN.data,  FIFO_PIPE_SIZE);
	fifoPipeBulkOUT.handle = UsbFifoMgrCreate(fifoPipeBulkOUT.data, FIFO_PIPE_SIZE);
	/* validate fifo handles */
	ASSERT_RT( (fifoPipeBulkIN.handle  != FIFO_ERR_INVALID_HANDLE) && (fifoPipeBulkOUT.handle != FIFO_ERR_INVALID_HANDLE) );
	/* initialize the low level USB driver */
	USB_Init();
} /*** end of UsbInit ***/


/************************************************************************************//**
** \brief     Releases the USB communication interface. 
** \return    none.
**
****************************************************************************************/
void UsbFree(void)
{
	/* disconnect the USB device from the USB host */
	UsbConnectHook(false);
} /*** end of UsbFree ***/


/************************************************************************************//**
** \brief     Transmits a packet formatted for the communication interface.
** \param	data Pointer to byte array with data that it to be transmitted.
** \param	len  Number of bytes that are to be transmitted.
** \return    none.
**
****************************************************************************************/
bool UsbTransmitPacket(uint8_t *data, uint8_t len)
{
	uint16_t data_index;
	bool result;

	/* verify validity of the len-paramenter */
	ASSERT_RT(len <= BOOT_COM_USB_TX_MAX_DATA);  

	/* first transmit the length of the packet */  
	result = UsbTransmitByte(len);
	ASSERT_RT(result == true);
	if (result)
	{
		/* transmit all the packet bytes one-by-one */
		for (data_index = 0; data_index < len; data_index++)
		{
			CopService();	/* keep the watchdog happy */
			/* write byte */
			result = UsbTransmitByte(data[data_index]);
			ASSERT_RT(result == true);
			if (!result)
				break;
		}
	}
	return result;
} /*** end of UsbTransmitPacket ***/


/************************************************************************************//**
** \brief     Receives a communication interface packet if one is present.
** \param	data Pointer to byte array where the data is to be stored.
** \return    true if a packet was received, false otherwise.
**
****************************************************************************************/
bool UsbReceivePacket(uint8_t *data)
{
	static uint8_t xcpCtoReqPacket[BOOT_COM_USB_RX_MAX_DATA + 1];  // one extra for length
	static uint8_t xcpCtoRxLength;
	static bool  xcpCtoRxInProgress = false;

	// poll USB interrupt flags to process USB related events
	USB_Istr();

	// start of cto packet received ?
	if ( ! xcpCtoRxInProgress)
	{	// store the message length when received
		if (UsbReceiveByte(&xcpCtoReqPacket[0]))
		{
			if (xcpCtoReqPacket[0] > 0)
			{	// indicate that a cto packet is being received
				xcpCtoRxInProgress = true;
				// reset packet data count
				xcpCtoRxLength = 0;
			}
		}
	}
	else
	{
		// store the next packet byte
		if (UsbReceiveByte(&xcpCtoReqPacket[xcpCtoRxLength + 1]))
		{
			// increment the packet data count
			xcpCtoRxLength++;

			// check to see if the entire packet was received
			if (xcpCtoRxLength == xcpCtoReqPacket[0])
			{
				// copy the packet data
				CpuMemCopy((uint32_t)data, (uint32_t)&xcpCtoReqPacket[1], xcpCtoRxLength);        
				xcpCtoRxInProgress = false;	// done with cto packet reception
				return true;				// packet reception complete
			}
		}
	}
	return false;	// packet reception not yet complete
}


/************************************************************************************//**
** \brief     Receives a communication interface byte if one is present.
** \param	data Pointer to byte where the data is to be stored.
** \return    true if a byte was received, false otherwise.
**
****************************************************************************************/
static bool UsbReceiveByte(uint8_t *data)
{
  bool result;
 
  // obtain data from the fifo
  result = UsbFifoMgrRead(fifoPipeBulkOUT.handle, data);
  return result;
}


/************************************************************************************//**
** \brief     Transmits a communication interface byte.
** \param	data Value of byte that is to be transmitted.
** \return    true if the byte was transmitted, false otherwise.
**
****************************************************************************************/
static bool UsbTransmitByte(uint8_t data)
{
  bool result;
 
  /* write data from to fifo */
  result = UsbFifoMgrWrite(fifoPipeBulkIN.handle, data);
  return result;
} /*** end of UsbTransmitByte ***/


/************************************************************************************//**
** \brief     Power-off system clocks and power while entering suspend mode.
** \return    none.
**
****************************************************************************************/
void UsbEnterLowPowerMode(void)
{
  /* Set the device state to suspend */
  bDeviceState = SUSPENDED;
  /* power-off system clocks and power */
  UsbEnterLowPowerModeHook();
} /*** end of UsbEnterLowPowerMode ***/


/************************************************************************************//**
** \brief     Restores system clocks and power while exiting suspend mode.
** \return    none.
**
****************************************************************************************/
void UsbLeaveLowPowerMode(void)
{
	DEVICE_INFO *pInfo = &Device_Info;

	// restore power and system clocks
	UsbLeaveLowPowerModeHook();
	// Set the device state to the correct state
	if (pInfo->Current_Configuration != 0)
	{
		// Device configured
		bDeviceState = CONFIGURED;
	}
	else
	{
		bDeviceState = ATTACHED;
	}
}


/************************************************************************************//**
** \brief     Checks if there is still data left to transmit and if so submits it
**			for transmission with the USB endpoint.
** \return    none.
**
****************************************************************************************/
void UsbTransmitPipeBulkIN(void)
{
	/* USB_Tx_Buffer is static for run-time optimalization */
	static uint8_t USB_Tx_Buffer[BULK_DATA_SIZE];
	uint8_t nr_of_bytes_for_tx_endpoint;
	uint8_t byte_counter;
	uint8_t byte_value;
	bool  result;

	/* read how many bytes should be transmitted */
	nr_of_bytes_for_tx_endpoint = UsbFifoMgrScan(fifoPipeBulkIN.handle);
	/* only continue if there is actually data left to transmit */
	if (nr_of_bytes_for_tx_endpoint == 0)
	{
		return;
	}
	/* make sure to not transmit more than the USB endpoint can handle */
	if (nr_of_bytes_for_tx_endpoint > BULK_DATA_SIZE)
	{
		nr_of_bytes_for_tx_endpoint = BULK_DATA_SIZE;
	}
	/* copy the transmit data to the transmit buffer */
	for (byte_counter=0; byte_counter < nr_of_bytes_for_tx_endpoint; byte_counter++)
	{
		/* obtain data from the fifo */
		result = UsbFifoMgrRead(fifoPipeBulkIN.handle, &byte_value);
		ASSERT_RT(result == true);
		if (!result)
			return;
		/* store it in the endpoint's RAM */
		USB_Tx_Buffer[byte_counter] = byte_value; 
	}
	/* store it in the endpoint's RAM */
	UserToPMABufferCopy(&USB_Tx_Buffer[0], ENDP1_TXADDR, nr_of_bytes_for_tx_endpoint);
	/* set the number of bytes that need to be transmitted from this endpoint */
	SetEPTxCount(ENDP1, nr_of_bytes_for_tx_endpoint);
	/* inform the endpoint that it can start its transmission because the data is valid */
	SetEPTxValid(ENDP1); 
} /*** end of UsbTransmitPipeBulkIN ***/


/************************************************************************************//**
** \brief     Stores data that was received on the Bulk OUT pipe in the fifo.
** \return    none.
**
****************************************************************************************/
void UsbReceivePipeBulkOUT(void)
{
	/* USB_Rx_Buffer is static for run-time optimalization */
	static uint8_t USB_Rx_Buffer[BULK_DATA_SIZE];
	uint16_t USB_Rx_Cnt;
	uint16_t byte_counter;
	bool result;
  
	/* Get the received data buffer and update the counter */
	USB_Rx_Cnt = USB_SIL_Read(EP1_OUT, USB_Rx_Buffer);
  
	/* USB data will be immediately processed, this allow next USB traffic being 
	 * NAKed till the end of the USART Xfer 
	 */
	for (byte_counter=0; byte_counter<USB_Rx_Cnt; byte_counter++)
	{
		/* add the data to the fifo */
		result = UsbFifoMgrWrite(fifoPipeBulkOUT.handle, USB_Rx_Buffer[byte_counter]);
		/* verify that the fifo wasn't full */
		ASSERT_RT(result == true);
		if (!result)
			return;
	}
	/* Enable the reception of data on EP1 */
	SetEPRxValid(ENDP1);
} /*** end of UsbReceivePipeBulkOUT ***/


/************************************************************************************//**
** \brief     Converts Hex 32Bits value into char.
** \param	value The hexadecimal value to convert.
** \param	pbuf  Pointer to where the resulting string should be stored.
** \param	len   Number of characters to convert.
** \return    none.
**
****************************************************************************************/
static void IntToUnicode (uint32_t value , uint8_t *pbuf , uint8_t len)
{
	uint8_t idx = 0;
  
	for ( idx = 0 ; idx < len ; idx ++)
	{
		if ( ((value >> 28)) < 0xA )
		{
			pbuf[ 2* idx] = (value >> 28) + '0';
		}
		else
		{
			pbuf[2* idx] = (value >> 28) + 'A' - 10; 
		}
    
		value = value << 4;
    
		pbuf[ 2* idx + 1] = 0;
	}
} /*** end of IntToUnicode ***/


/************************************************************************************//**
** \brief     Creates the serial number string descriptor.
** \return    none.
**
****************************************************************************************/
void UsbGetSerialNum(void)
{
	uint32_t Device_Serial0, Device_Serial1, Device_Serial2;

	Device_Serial0 = *(volatile uint32_t*)(0x1FFFF7E8);
	Device_Serial1 = *(volatile uint32_t*)(0x1FFFF7EC);
	Device_Serial2 = *(volatile uint32_t*)(0x1FFFF7F0);

	Device_Serial0 += Device_Serial2;

	if (Device_Serial0 != 0)
	{
		IntToUnicode(Device_Serial0, &Bulk_StringSerial[2] , 8);
		IntToUnicode(Device_Serial1, &Bulk_StringSerial[18], 4);
	}
} /*** end of UsbGetSerialNum ***/


/************************************************************************************//**
** \brief     Initializes the fifo manager. Each controlled fifo is assigned a
**			unique handle, which is the same as its index into fifoCtrl[]. Each
**			controlled fifo holds a pointer to the next free fifo control.
**			For the last fifo in fifoCtrl[] this one is set to a null-pointer as
**			an out of fifo control indicator. Function should be called once
**			before any of the other fifo management functions are called.
** \return    none.
**
****************************************************************************************/
static void UsbFifoMgrInit(void)
{
	uint8_t i;
	tFifoCtrl *pbc1, *pbc2;

	pbc1 = &fifoCtrl[0];
	pbc2 = &fifoCtrl[1];
	/* assign fifo handles and pointer to next free fifo */
	for (i = 0; i < (FIFO_MAX_BUFFERS - 1); i++)
	{
		pbc1->handle = i;
		pbc1->fifoctrlptr = pbc2;
		pbc1++;
		pbc2++;
	}
	/* initialize handle for the last one and use null-pointer for the next free fifo  */
	pbc1->handle = i;
	pbc1->fifoctrlptr = (tFifoCtrl *)0;
	fifoCtrlFree = &fifoCtrl[0];
} /*** end of UsbFifoMgrInit ***/


/************************************************************************************//**
** \brief     Places a data storage array under fifo management control. A handle
**			for identifying the fifo in subsequent fifo management function
**			calls is returned, if successful.
** \param	buffer Pointer to the first element in the data storage fifo.
** \param	length Maximum number of data elements that can be stored in the fifo.
** \return    Fifo handle if successfull, or FIFO_ERR_INVALID_HANDLE.
**
****************************************************************************************/
static uint8_t UsbFifoMgrCreate(uint8_t *buffer, uint8_t length)
{
	tFifoCtrl *pbc;

	/* first determine if these is still a free fifo control available */
	if (fifoCtrlFree == (tFifoCtrl *)0)
	{
		return FIFO_ERR_INVALID_HANDLE;
	}
	/* store pointer to free fifo and update pointer to next free one */
	pbc = fifoCtrlFree;
	fifoCtrlFree = pbc->fifoctrlptr;

	/* initialize the buffer control */
	pbc->length = length;
	pbc->readptr = buffer;
	pbc->writeptr = buffer;
	pbc->entries = 0;
	pbc->startptr = buffer;
	pbc->endptr = (uint8_t*)(buffer + length - 1);

	/* return the handle to the successfully created fifo control */
	return pbc->handle;
} /*** end of UsbFifoMgrCreate ***/


/************************************************************************************//**
** \brief     Stores data in the fifo.
** \param	handle Identifies the fifo to write data to.
** \param	data   Pointer to the data that is to be written to the fifo.
** \return    true if the data was successfully stored in the fifo, false
**			otherwise.
**
****************************************************************************************/
static bool UsbFifoMgrWrite(uint8_t handle, uint8_t data)
{
	/* check the validity of the handle parameter */
	ASSERT_RT(handle < FIFO_MAX_BUFFERS);
	/* check if fifo is full */
	if (fifoCtrl[handle].entries == fifoCtrl[handle].length)
	{
		return false;
	}
	/* copy data to fifo */
	*fifoCtrl[handle].writeptr = data;
	/* data written so update number of entries */
	fifoCtrl[handle].entries++;
	/* update write pointer */
	fifoCtrl[handle].writeptr++;
	/* check end of fifo */
	if (fifoCtrl[handle].writeptr > fifoCtrl[handle].endptr)
	{
		/* set write pointer to start of the cyclic fifo */
		fifoCtrl[handle].writeptr = fifoCtrl[handle].startptr;
	}
	/* still here so all is okay */	
	return true;
} /*** end of UsbFifoMgrWrite ***/


/************************************************************************************//**
** \brief     Retrieves data from the fifo.
** \param	handle Identifies the fifo to read data from.
** \param	data   Pointer to where the read data is to be stored.
** \return    true if the data was successfully read from the fifo, false
**			otherwise.
**
****************************************************************************************/
static bool UsbFifoMgrRead(uint8_t handle, uint8_t *data)
{
	/* check the validity of the handle parameter */
	ASSERT_RT(handle < FIFO_MAX_BUFFERS);
	/* check if fifo is empty */
	if (fifoCtrl[handle].entries == 0)
	{
		return false;
	}
	/* read the data */
	*data = *fifoCtrl[handle].readptr;
	/* data read so update number of entries */
	fifoCtrl[handle].entries--;
	/* update read pointer */
	fifoCtrl[handle].readptr++;
	/* check end of fifo */
	if (fifoCtrl[handle].readptr > fifoCtrl[handle].endptr)
	{
		/* set read pointer to start of the cyclic fifo */
		fifoCtrl[handle].readptr = fifoCtrl[handle].startptr;
	}
	/* still here so all is good */
	return true;
} /*** end of UsbFifoMgrRead ***/


/************************************************************************************//**
** \brief     Returns the number of data entries currently present in the fifo.
** \param	handle Identifies the fifo that is to be scanned.
** \return    Number of data entries in the fifo if successful, otherwise 0.
**
****************************************************************************************/
static uint8_t UsbFifoMgrScan(uint8_t handle)
{
	/* check the validity of the handle parameter */
	ASSERT_RT(handle < FIFO_MAX_BUFFERS);
	/* read and return the number of data entries */
	return fifoCtrl[handle].entries;
} /*** end of UsbFifoMgrScan ***/

#endif /* BOOT_COM_USB_ENABLE > 0 */

/*********************************** end of usb.c **************************************/
/************************************************************************************//**
* \file         Source\ARMCM3_STM32\usb.c
* \brief        Bootloader USB communication interface source file.
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
