#include "boot.h"	/* bootloader generic header	*/


/****************************************************************************************
* Defines
****************************************************************************************/
/** \brief XCP protocol layer version number (16-bit). */
#define XCP_VERSION_PROTOCOL_LAYER	(0x0100)

/** \brief XCP transport layer version number (16-bit). */
#define XCP_VERSION_TRANSPORT_LAYER	(0x0100)

/* XCP packet identifiers */
/** \brief Command response packet identifier. */
#define XCP_PID_RES					(0xFF)
/** \brief Error packet identifier. */            
#define XCP_PID_ERR					(0xFE)

/* XCP error codes */
typedef enum {
	XCP_ERR_CMD_SYNCH		= (0x00),	/** \brief Cmd processor synchronization error code. */
	XCP_ERR_CMD_BUSY		= (0x10),	/** \brief Command was not executed error code. */
	XCP_ERR_CMD_UNKNOWN		= (0x20),	/** \brief Unknown or unsupported command error code. */
	XCP_ERR_OUT_OF_RANGE	= (0x22),	/** \brief Parameter out of range error code. */
	XCP_ERR_ACCESS_LOCKED	= (0x25),	/** \brief Protected error code. Seed/key required. */
	XCP_ERR_PAGE_NOT_VALID	= (0x26),	/** \brief Cal page not valid error code. */
	XCP_ERR_SEQUENCE		= (0x29),	/** \brief Sequence error code. */
	XCP_ERR_GENERIC			= (0x31),	/** \brief Generic error code. */
} XCPError_t;

/* XCP command codes */
typedef enum {
	XCP_CMD_CONNECT			= (0xFF),	/** \brief CONNECT command code. */
	XCP_CMD_DISCONNECT		= (0xFE),	/** \brief DISCONNECT command code. */
	XCP_CMD_GET_STATUS		= (0xFD),	/** \brief GET_STATUS command code. */
	XCP_CMD_SYNCH			= (0xfC),	/** \brief SYNCH command code. */
	XCP_CMD_GET_ID			= (0xFA),	/** \brief GET_ID command code. */
	XCP_CMD_GET_SEED		= (0xf8),	/** \brief GET_SEED command code. */
	XCP_CMD_UNLOCK			= (0xf7),	/** \brief UNLOCK command code. */
	XCP_CMD_SET_MTA			= (0xf6),	/** \brief SET_MTA command code. */
	XCP_CMD_UPLOAD			= (0xf5),	/** \brief UPLOAD command code. */
	XCP_CMD_SHORT_UPLOAD	= (0xf4),	/** \brief SHORT_UPLOAD command code. */
	XCP_CMD_BUILD_CHECKSUM	= (0xf3),	/** \brief BUILD_CHECKSUM command code. */
	XCP_CMD_DOWNLOAD		= (0xf0),	/** \brief DOWNLOAD command code. */
	XCP_CMD_DOWLOAD_MAX		= (0xee),	/** \brief DOWNLOAD_MAX command code. */
	XCP_CMD_SET_CAL_PAGE	= (0xeb),	/** \brief SET_CALPAGE command code. */
	XCP_CMD_GET_CAL_PAGE	= (0xea),	/** \brief GET_CALPAGE command code. */
	XCP_CMD_PROGRAM_START	= (0xd2),	/** \brief PROGRAM_START command code. */
	XCP_CMD_PROGRAM_CLEAR	= (0xd1),	/** \brief PROGRAM_CLEAR command code. */
	XCP_CMD_PROGRAM			= (0xd0),	/** \brief PROGRAM command code. */
	XCP_CMD_PROGRAM_RESET   = (0xcf),	/** \brief PROGRAM_RESET command code. */
	XCP_CMD_PROGRAM_PREPARE	= (0xcc),	/** \brief PROGRAM_PREPARE command code. */
	XCP_CMD_PROGRAM_MAX		= (0xc9),	/** \brief PROGRAM_MAX command code. */
} XCPCommand_t;

/****************************************************************************************
* Type definitions
****************************************************************************************/
/** \brief Struture type for grouping XCP internal module information. */
typedef struct {
	bool  		connected;						/**< connection established			*/
	bool		protection;						/**< protection state				*/
	uint8_t		s_n_k_resource;					/**< for seed/key sequence			*/
	bool		ctoPending;						/**< cto transmission pending flag	*/
	int16_t		ctoLen;							/**< cto current packet length		*/
	uint32_t	mta;							/**< memory transfer address		*/
	uint8_t		ctoData[BOOT_COM_RX_MAX_DATA];	/**< cto packet data buffer			*/
} XcpInfo_t;


/****************************************************************************************
* Function prototypes
****************************************************************************************/
/* transport layer specific functions */
static void      XcpTransmitPacket(uint8_t *data, int16_t len);

/* application specific functions */
static uint8_t XcpComputeChecksum(uint32_t address, uint32_t length, uint32_t *checksum);

#if (XCP_SEED_KEY_PROTECTION_EN == 1)
	static uint8_t XcpGetSeed(uint8_t resource, uint8_t *seed);
	static uint8_t XcpVerifyKey(uint8_t resource, uint8_t *key, uint8_t len);
#endif

/* general utility functions */
static void XcpProtectResources(void);
static void XcpSetCtoError(XCPError_t error);

/* XCP command processors */
static void XcpCmdConnect(uint8_t *data);
static void XcpCmdDisconnect(uint8_t *data);
static void XcpCmdGetStatus(uint8_t *data);
static void XcpCmdSynch(uint8_t *data);
static void XcpCmdGetId(uint8_t *data);
static void XcpCmdSetMta(uint8_t *data);
static void XcpCmdUpload(uint8_t *data);
static void XcpCmdShortUpload(uint8_t *data);
static void XcpCmdBuildCheckSum(uint8_t *data);

#if (XCP_SEED_KEY_PROTECTION_EN == 1)
	static void XcpCmdGetSeed(uint8_t *data);
	static void XcpCmdUnlock(uint8_t *data);
#endif

#if (XCP_RES_CALIBRATION_EN == 1)
	static void XcpCmdDownload(uint8_t *data);
	static void XcpCmdDownloadMax(uint8_t *data);
#endif

#if (XCP_RES_PAGING_EN == 1)
	static void XcpCmdSetCalPage(uint8_t *data);
	static void XcpCmdGetCalPage(uint8_t *data);
#endif

#if (XCP_RES_PROGRAMMING_EN == 1)
	static void XcpCmdProgramMax(uint8_t *data);
	static void XcpCmdProgram(uint8_t *data);
	static void XcpCmdProgramStart(uint8_t *data);
	static void XcpCmdProgramClear(uint8_t *data);
	static void XcpCmdProgramReset(uint8_t *data);
	static void XcpCmdProgramPrepare(uint8_t *data);
#endif


/****************************************************************************************
* Hook functions
****************************************************************************************/
#if (XCP_RES_PAGING_EN == 1)
	extern uint8_t XcpCalSetPageHook(uint8_t segment, uint8_t page);
	extern uint8_t XcpCalGetPageHook(uint8_t segment);
#endif

#if (XCP_CONNECT_MODE_HOOK_EN == 1)
	extern bool XcpConnectModeHook(uint8_t mode);
#endif

#if (XCP_SEED_KEY_PROTECTION_EN == 1)
	extern uint8_t XcpGetSeedHook(uint8_t resource, uint8_t *seed);
	extern uint8_t XcpVerifyKeyHook(uint8_t resource, uint8_t *key, uint8_t len);
#endif



/****************************************************************************************
* External functions
****************************************************************************************/
#if (BOOT_COM_ENABLE == 0)
	/* in case no internally supported communication interface is used, a custom 
	 * communication module can be added. In order to use the XCP protocol in the custom
	 * communication module, this hook function needs to be implemented. In the XCP protocol
	 * is not needed, then simply remove the xcp.c source from the project.
	 */
	extern void XcpTransmitPacketHook(uint8_t *data, uint16_t len);
#endif


/****************************************************************************************
* Local constants
****************************************************************************************/
/** \brief String buffer with station id. */
static const int8_t xcpStationId[] = XCP_STATION_ID_STRING;


/****************************************************************************************
* Local data definitions
****************************************************************************************/
/** \brief Local variable for storing XCP internal module info. */
static XcpInfo_t xcpInfo;


/************************************************************************************//**
** \brief	Initializes the XCP driver. Should be called once upon system startup.
** \return	none
**
****************************************************************************************/
void XcpInit(void)
{
	XcpInfo_t *xcp = &xcpInfo;

	/* reset xcp module info */
	xcp->connected = false;
	xcp->mta = 0;
	xcp->ctoPending = false;
	xcp->ctoLen = 0;
	xcp->s_n_k_resource = 0;
	xcp->protection = false;
}


/************************************************************************************//**
** \brief	Obtains information about the XCP connection state.
** \return	true is an XCP connection is established, false otherwise.
**
****************************************************************************************/
bool XcpIsConnected(void)
{
	return xcpInfo.connected;
}


/************************************************************************************//**
** \brief	Informs the core that a pending packet transmission was completed by 
**			the transport layer.
** \return	none
**
****************************************************************************************/
void XcpPacketTransmitted(void)
{
	/* reset packet transmission pending flag */
	xcpInfo.ctoPending = false;
}


/************************************************************************************//**
** \brief	Informs the core that a new packet was received by the transport layer.
** \param	data Pointer to byte buffer with packet data.
** \return	none
**
****************************************************************************************/
void XcpPacketReceived(uint8_t *data)
{
	/* was this a connect command? */
	if (data[0] == XCP_CMD_CONNECT)
	{
		/* process the connect command */
		XcpCmdConnect(data);
	}
	/* only continue if connected */
	else if ( ! xcpInfo.connected)
	{
		/* return to make sure response packet is not send because we are not connected */
		return;
	}
	else
	{
		switch (data[0])
		{
			case XCP_CMD_UPLOAD:
				XcpCmdUpload(data);
				break;
			case XCP_CMD_SHORT_UPLOAD:
				XcpCmdShortUpload(data);
				break;
			case XCP_CMD_SET_MTA:
				XcpCmdSetMta(data);
				break;
			case XCP_CMD_BUILD_CHECKSUM:
				XcpCmdBuildCheckSum(data);
				break;
			case XCP_CMD_GET_ID:
				XcpCmdGetId(data);
				break;
			case XCP_CMD_SYNCH:
				XcpCmdSynch(data);
				break;
			case XCP_CMD_GET_STATUS:
				XcpCmdGetStatus(data);
				break;
			case XCP_CMD_DISCONNECT:
				XcpCmdDisconnect(data);
				break;
#if (XCP_RES_CALIBRATION_EN == 1)
			case XCP_CMD_DOWNLOAD:
				XcpCmdDownload(data);
				break;
			case XCP_CMD_DOWLOAD_MAX:
				XcpCmdDownloadMax(data);
				break;
#endif
#if (XCP_RES_PROGRAMMING_EN == 1)
			case XCP_CMD_PROGRAM_MAX:
				XcpCmdProgramMax(data);
				break;
			case XCP_CMD_PROGRAM:
				XcpCmdProgram(data);
				break;
			case XCP_CMD_PROGRAM_START:
				XcpCmdProgramStart(data);
				break;
			case XCP_CMD_PROGRAM_CLEAR:
				XcpCmdProgramClear(data);
				break;
			case XCP_CMD_PROGRAM_RESET:
				XcpCmdProgramReset(data);
				break;
			case XCP_CMD_PROGRAM_PREPARE:
				XcpCmdProgramPrepare(data);
				break;
#endif
#if (XCP_SEED_KEY_PROTECTION_EN == 1)
			case XCP_CMD_GET_SEED:
				XcpCmdGetSeed(data);
				break;
			case XCP_CMD_UNLOCK:
				XcpCmdUnlock(data);
				break;
#endif
#if (XCP_RES_PAGING_EN == 1)
			case XCP_CMD_SET_CAL_PAGE:
				XcpCmdSetCalPage(data);
				break;
			case XCP_CMD_GET_CAL_PAGE:
				XcpCmdGetCalPage(data);
				break;
#endif
			default:
				XcpSetCtoError(XCP_ERR_CMD_UNKNOWN);
				break;
		}
	}

	/* make sure the previous command was completed */
	if (xcpInfo.ctoPending)
	{
		/* command overrun occurred */
		XcpSetCtoError(XCP_ERR_CMD_BUSY);
	}

	/* send the response if it contains something */
	if (xcpInfo.ctoLen > 0)
	{
		/* set cto packet transmission pending flag */
		xcpInfo.ctoPending = true;

		/* transmit the cto response packet */
		XcpTransmitPacket(xcpInfo.ctoData, xcpInfo.ctoLen);
	}
}


/************************************************************************************//**
** \brief	Transmits the packet using the xcp transport layer.
** \param	data Pointer to the byte buffer with packet data.
** \param	len  Number of data bytes that need to be transmitted.
** \return	none
**
****************************************************************************************/
static void XcpTransmitPacket(uint8_t *data, int16_t len)
{
	/* submit packet to the communication interface for transmission */
#if (BOOT_COM_ENABLE == 0)
	XcpTransmitPacketHook(data, len);
#else
	ComTransmitPacket(data, len);
#endif
}


/************************************************************************************//**
** \brief	Called by the BUILD_CHECKSUM command to perform a checksum calculation
**			over the specified memory region.
** \param	address   The start address of the memory region.
** \param	length    Length of the memory region in bytes.
** \param	checksum  Pointer to where the calculated checksum is to be stored.
** \return	Checksum type that was used during the checksum calculation.
**
****************************************************************************************/
static uint8_t XcpComputeChecksum(uint32_t address, uint32_t length, uint32_t *checksum)
{
	uint8_t cs = 0;

	/* this example computes the checksum using the add byte to byte algorithm */
	while (length-- > 0)
	{
		cs += *((uint8_t*)(uint32_t)address);
		address++;
	}

	*checksum = cs;
	return XCP_CS_ADD11;
}


#if (XCP_SEED_KEY_PROTECTION_EN == 1)
/************************************************************************************//**
** \brief	Provides a seed to the XCP master that will be used for the key 
**			generation when the master attempts to unlock the specified resource. 
**			Called by the GET_SEED command.
** \param	resource  Resource that the seed if requested for (XCP_RES_XXX).
** \param	seed      Pointer to byte buffer wher the seed will be stored.
** \return	Length of the seed in bytes.
**
****************************************************************************************/
static uint8_t XcpGetSeed(uint8_t resource, uint8_t *seed)
{
	/* pass request on to the application through a hook function */
	return XcpGetSeedHook(resource, seed);
}


/************************************************************************************//**
** \brief	Called by the UNLOCK command and checks if the key to unlock the 
**			specified resource was correct. If so, then the resource protection 
**			will be removed.
** \param	resource  resource to unlock (XCP_RES_XXX).
** \param	key       pointer to the byte buffer holding the key.
** \param	len       length of the key in bytes.
** \return	1 if the key was correct, 0 otherwise.
**
****************************************************************************************/
static uint8_t XcpVerifyKey(uint8_t resource, uint8_t *key, uint8_t len)
{
	/* pass request on to the application through a hook function */
	return XcpVerifyKeyHook(resource, key, len);
}
#endif /* XCP_SEED_KEY_PROTECTION_EN == 1 */


/************************************************************************************//**
** \brief	Utility function to protects all the available resources.
** \return	none
**
****************************************************************************************/
static void XcpProtectResources(void)
{
	xcpInfo.protection = false;

#if (XCP_SEED_KEY_PROTECTION_EN == 1)

	#if (XCP_RES_CALIBRATION_EN == 1)
	xcpInfo.protection |= XCP_RES_CALPAG;
	#endif

	#if (XCP_RES_PAGING_EN == 1)
	xcpInfo.protection |= XCP_RES_CALPAG;
	#endif

	#if (XCP_RES_PROGRAMMING_EN == 1)
	xcpInfo.protection |= XCP_RES_PGM;
	#endif

	#if (XCP_RES_DATA_ACQUISITION_EN == 1)
	xcpInfo.protection |= XCP_RES_DAQ;
	#endif

	#if (XCP_RES_DATA_STIMULATION_EN == 1)
	xcpInfo.protection |= XCP_RES_STIM;
	#endif
#endif /* XCP_SEED_KEY_PROTECTION_EN == 1 */
}


/************************************************************************************//**
** \brief	Prepares the cto packet data for the specified error.
** \param	error XCP error code (XCP_ERR_XXX).
** \return	none
**
****************************************************************************************/
static void XcpSetCtoError(XCPError_t error)
{
	/* prepare the error packet */
	xcpInfo.ctoData[0] = XCP_PID_ERR;
	xcpInfo.ctoData[1] = error;
	xcpInfo.ctoLen = 2;
}


/************************************************************************************//**
** \brief	XCP command processor function which handles the CONNECT command as
**			defined by the protocol.
** \param	data Pointer to a byte buffer with the packet data.
** \return	none
**
****************************************************************************************/
static void XcpCmdConnect(uint8_t *data)
{
	/* suppress compiler warning for unused parameter */
	data = data;
	uint8_t xb;

#if (BOOT_FILE_SYS_ENABLE > 0)
	/* reject the connection if the file module is not idle. this means that a firmware
	 * update from the locally attached storage is in progress
	 */
	if (FileIsIdle() == false)
	{
		/* command not processed because we are busy */
		XcpSetCtoError(XCP_ERR_CMD_BUSY);
		return;
	}
#endif
  
#if (XCP_CONNECT_MODE_HOOK_EN == 1)
	/* pass on the mode to a application specific hook function. This function can determine
	 * is the mode is supported or not. A return value of false causes the CONNECT command
	 * to be ignored. Note that this mode could potentially be used to specify a node ID in a
	 * multi XCP slave system.
	 */
	if (XcpConnectModeHook(data[1]) == false)
	{
		/* set the response length to 0 to suppress it */
		xcpInfo.ctoLen = 0;
		return;
	}
#endif

	/* enable resource protection */
	XcpProtectResources();

	/* indicate that the connection is established */
	xcpInfo.connected = true;

	/* set packet id to command response packet */
	xcpInfo.ctoData[0] = XCP_PID_RES;

	/* report available resources */
	xb = 0;

#if (XCP_RES_CALIBRATION_EN == 1)
	xb |= XCP_RES_CALPAG;
#endif

#if (XCP_RES_PAGING_EN == 1)
	xb |= XCP_RES_CALPAG;
#endif

#if (XCP_RES_PROGRAMMING_EN == 1)
	xb |= XCP_RES_PGM;
#endif

#if (XCP_RES_DATA_ACQUISITION_EN == 1)
	xb |= XCP_RES_DAQ;
#endif

#if (XCP_RES_DATA_STIMULATION_EN == 1)
	xb |= XCP_RES_STIM;
#endif
	xcpInfo.ctoData[1] = xb;

	/* report communication mode info. only byte granularity is supported */
	/* configure for motorola or intel byte ordering */
	xcpInfo.ctoData[2] = XCP_MOTOROLA_FORMAT;

	/* report max cto data length */
	xcpInfo.ctoData[3] = (uint8_t)XCP_CTO_PACKET_LEN;

	/* report max dto data length */
#if (XCP_MOTOROLA_FORMAT == 0)
	xcpInfo.ctoData[4] = (uint8_t)XCP_DTO_PACKET_LEN;
	xcpInfo.ctoData[5] = (uint8_t)(XCP_DTO_PACKET_LEN >> 8);
#else
	xcpInfo.ctoData[4] = (uint8_t)(XCP_DTO_PACKET_LEN >> 8);
	xcpInfo.ctoData[5] = (uint8_t)XCP_DTO_PACKET_LEN;
#endif

	/* report msb of protocol layer version number */
	xcpInfo.ctoData[6] = XCP_VERSION_PROTOCOL_LAYER >> 8;

	/* report msb of transport layer version number */
	xcpInfo.ctoData[7] = XCP_VERSION_TRANSPORT_LAYER >> 8;

	/* set packet length */
	xcpInfo.ctoLen = 8;

}


/************************************************************************************//**
** \brief	XCP command processor function which handles the DISCONNECT command as
**			defined by the protocol.
** \param	data Pointer to a byte buffer with the packet data.
** \return	none
**
****************************************************************************************/
static void XcpCmdDisconnect(uint8_t *data)
{
	/* indicate that the xcp connection is disconnected */
	xcpInfo.connected = 0;

	/* enable resource protection */
	XcpProtectResources();

	/* set packet id to command response packet */
	xcpInfo.ctoData[0] = XCP_PID_RES;

	/* set packet length */
	xcpInfo.ctoLen = 1;
}


/************************************************************************************//**
** \brief	XCP command processor function which handles the GET_STATUS command as
**			defined by the protocol.
** \param	data Pointer to a byte buffer with the packet data.
** \return	none
**
****************************************************************************************/
static void XcpCmdGetStatus(uint8_t *data)
{
	/* set packet id to command response packet */
	xcpInfo.ctoData[0] = XCP_PID_RES;

	/* report session status */
	xcpInfo.ctoData[1] = 0;

	/* report current resource protection status */
	xcpInfo.ctoData[2] = xcpInfo.protection;

	/* reset reserved and session configuration id values */
	xcpInfo.ctoData[3] = 0;
	xcpInfo.ctoData[4] = 0;
	xcpInfo.ctoData[5] = 0;

	/* set packet length */
	xcpInfo.ctoLen = 6;
}


/************************************************************************************//**
** \brief	XCP command processor function which handles the SYNCH command as
**			defined by the protocol.
** \param	data Pointer to a byte buffer with the packet data.
** \return	none
**
****************************************************************************************/
static void XcpCmdSynch(uint8_t *data)
{
	/* synch requires a negative response */
	XcpSetCtoError(XCP_ERR_CMD_SYNCH);
}


/************************************************************************************//**
** \brief	XCP command processor function which handles the GET_ID command as
**			defined by the protocol.
** \param	data Pointer to a byte buffer with the packet data.
** \return	none
**
****************************************************************************************/
static void XcpCmdGetId(uint8_t *data)
{
	/* set packet id to command response packet */
	xcpInfo.ctoData[0] = XCP_PID_RES;

	/* point mta to start of station id string */
	xcpInfo.mta = (uint32_t)&xcpStationId[0];

	/* set station id mode to 0 */
	xcpInfo.ctoData[1] = 0;

	/* reset reserved values */
	xcpInfo.ctoData[2] = 0;
	xcpInfo.ctoData[3] = 0;

	/* store station id length (excl. null termination) for response packet */
	*(uint32_t*)&xcpInfo.ctoData[4] = (sizeof(xcpStationId)/sizeof(xcpStationId[0])) - 1;

	/* set packet length */
	xcpInfo.ctoLen = 8;
}


/************************************************************************************//**
** \brief	XCP command processor function which handles the SET_MTA command as
**			defined by the protocol.
** \param	data Pointer to a byte buffer with the packet data.
** \return	none
**
****************************************************************************************/
static void XcpCmdSetMta(uint8_t *data)
{
	/* set packet id to command response packet */
	xcpInfo.ctoData[0] = XCP_PID_RES;

	/* update mta. current implementation ignores address extension */
	xcpInfo.mta = *(uint32_t*)&data[4];

	/* set packet length */
	xcpInfo.ctoLen = 1;
}


/************************************************************************************//**
** \brief	XCP command processor function which handles the UPLOAD command as
**			defined by the protocol.
** \param	data Pointer to a byte buffer with the packet data.
** \return	none
**
****************************************************************************************/
static void XcpCmdUpload(uint8_t *data)
{
	/* validate length of upload request */
	if (data[1] > (XCP_CTO_PACKET_LEN-1))
	{
		/* requested data length is too long */
		XcpSetCtoError(XCP_ERR_OUT_OF_RANGE);
		return;
	}

	/* copy the data from memory to the data packet */
	CpuMemCopy(((uint32_t)(uint32_t)&xcpInfo.ctoData[1]),(uint32_t)xcpInfo.mta, data[1]);

	/* set packet id to command response packet */
	xcpInfo.ctoData[0] = XCP_PID_RES;

	/* post increment the mta */
	xcpInfo.mta += data[1];

	/* set packet length */
	xcpInfo.ctoLen = data[1]+1;
}


/************************************************************************************//**
** \brief	XCP command processor function which handles the SHORT_UPLOAD command as
**			defined by the protocol.
** \param	data Pointer to a byte buffer with the packet data.
** \return	none
**
****************************************************************************************/
static void XcpCmdShortUpload(uint8_t *data)
{
	/* validate length of upload request */
	if (data[1] > (XCP_CTO_PACKET_LEN-1))
	{
		/* requested data length is too long */
		XcpSetCtoError(XCP_ERR_OUT_OF_RANGE);
		return;
	}

	/* update mta. current implementation ignores address extension */
	xcpInfo.mta = *(uint32_t*)&data[4];

	/* copy the data from memory to the data packet */
	CpuMemCopy((uint32_t)((uint32_t)&xcpInfo.ctoData[1]),(uint32_t)xcpInfo.mta, data[1]);
	/* set packet id to command response packet */
	xcpInfo.ctoData[0] = XCP_PID_RES;

	/* post increment the mta */
	xcpInfo.mta += data[1];

	/* set packet length */
	xcpInfo.ctoLen = data[1]+1;
}


#if (XCP_RES_CALIBRATION_EN == 1)
/************************************************************************************//**
** \brief	XCP command processor function which handles the DOWNLOAD command as
**			defined by the protocol.
** \param	data Pointer to a byte buffer with the packet data.
** \return	none
**
****************************************************************************************/
static void XcpCmdDownload(uint8_t *data)
{
#if (XCP_SEED_KEY_PROTECTION_EN == 1)
	/* check if CAL_PAG resource is unlocked */
	if ((xcpInfo.protection & XCP_RES_CALPAG) != 0)
	{
		/* resource is locked. use seed/key sequence to unlock */
		XcpSetCtoError(XCP_ERR_ACCESS_LOCKED);
		return;
	}
#endif

	/* validate length of download request */
	if (data[1] > (XCP_CTO_PACKET_LEN-2))
	{
		/* requested data length is too long */
		XcpSetCtoError(XCP_ERR_OUT_OF_RANGE);
		return;
	}

	/* copy the data from the data packet to memory */
	CpuMemCopy((uint32_t)xcpInfo.mta, (uint32_t)((uint32_t)&data[2]), data[1]);
	/* set packet id to command response packet */
	xcpInfo.ctoData[0] = XCP_PID_RES;

	/* post increment the mta */
	xcpInfo.mta += data[1];

	/* set packet length */
	xcpInfo.ctoLen = 1;
}


/************************************************************************************//**
** \brief	XCP command processor function which handles the DOWNLOAD_MAX command as
**			defined by the protocol.
** \param	data Pointer to a byte buffer with the packet data.
** \return	none
**
****************************************************************************************/
static void XcpCmdDownloadMax(uint8_t *data)
{
#if (XCP_SEED_KEY_PROTECTION_EN == 1)
	/* check if CAL_PAG resource is unlocked */
	if ((xcpInfo.protection & XCP_RES_CALPAG) != 0)
	{
		/* resource is locked. use seed/key sequence to unlock */
		XcpSetCtoError(XCP_ERR_ACCESS_LOCKED);
		return;
	}
#endif

	/* copy the data from the data packet to memory */
	CpuMemCopy((uint32_t)xcpInfo.mta, (uint32_t)((uint32_t)&data[1]), XCP_CTO_PACKET_LEN - 1);

	/* set packet id to command response packet */
	xcpInfo.ctoData[0] = XCP_PID_RES;

	/* post increment the mta */
	xcpInfo.mta += XCP_CTO_PACKET_LEN-1;

	/* set packet length */
	xcpInfo.ctoLen = 1;
}
#endif /* XCP_RES_CALIBRATION_EN == 1 */


/************************************************************************************//**
** \brief	XCP command processor function which handles the BUILD_CHECKSUM command as
**			defined by the protocol.
** \param	data Pointer to a byte buffer with the packet data.
** \return	none
**
****************************************************************************************/
static void XcpCmdBuildCheckSum(uint8_t *data)
{
	/* set packet id to command response packet */
	xcpInfo.ctoData[0] = XCP_PID_RES;

	/* obtain checksum and checksum type */
	xcpInfo.ctoData[1] = XcpComputeChecksum(xcpInfo.mta, *(uint32_t *)&data[4], (uint32_t *)&xcpInfo.ctoData[4]);

	/* initialize reserved parameters */
	xcpInfo.ctoData[2] = 0;
	xcpInfo.ctoData[3] = 0;

	/* set packet length */
	xcpInfo.ctoLen = 8;
}


#if (XCP_SEED_KEY_PROTECTION_EN == 1)
/************************************************************************************//**
** \brief	XCP command processor function which handles the GET_SEED command as
**			defined by the protocol.
** \param	data Pointer to a byte buffer with the packet data.
** \return	none
**
****************************************************************************************/
static void XcpCmdGetSeed(uint8_t *data)
{
	uint8_t resourceOK;

	/* init resource check variable as if an illegal resource is requested */
	resourceOK = 0;

	/* check if calibration/paging resource is requested for seed/key and make
	 * sure this is the only requested resource
	 */
	if (((data[2] & XCP_RES_CALPAG) > 0) && ((data[2] & ~XCP_RES_CALPAG) == 0))
	{
		resourceOK = 1;
	}

	/* check if programming resource is requested for seed/key and make
	 * sure this is the only requested resource
	 */
	if (((data[2] & XCP_RES_PGM) > 0) && ((data[2] & ~XCP_RES_PGM) == 0))
	{
		resourceOK = 1;
	}

	/* check if data acquisition resource is requested for seed/key and make
	 * sure this is the only requested resource
	 */
	if (((data[2] & XCP_RES_DAQ) > 0) && ((data[2] & ~XCP_RES_DAQ) == 0))
	{
		resourceOK = 1;
	}

	/* check if data stimulation resource is requested for seed/key and make
	 * sure this is the only requested resource
	 */
	if (((data[2] & XCP_RES_STIM) > 0) && ((data[2] & ~XCP_RES_STIM) == 0))
	{
		resourceOK = 1;
	}

	/* now process the resource validation */
	if (resourceOK == 0)
	{
		XcpSetCtoError(XCP_ERR_OUT_OF_RANGE);
		return;
	}

	/* store resource for which the seed/key sequence is started */
	xcpInfo.s_n_k_resource = data[2];

	/* set packet id to command response packet */
	xcpInfo.ctoData[0] = XCP_PID_RES;

	/* request the seed from the application */
	xcpInfo.ctoData[1] = XcpGetSeed(xcpInfo.s_n_k_resource, &xcpInfo.ctoData[2]);

	/* seed cannot be longer than XCP_CTO_PACKET_LEN-2 */
	if (xcpInfo.ctoData[1] > (XCP_CTO_PACKET_LEN - 2))
	{
		/* seed length length is too long */
		XcpSetCtoError(XCP_ERR_OUT_OF_RANGE);
		return;
	}

	/* set packet length */
	xcpInfo.ctoLen = xcpInfo.ctoData[1] + 2;
}


/************************************************************************************//**
** \brief	XCP command processor function which handles the UNLOCK command as
**			defined by the protocol.
** \param	data Pointer to a byte buffer with the packet data.
** \return	none
**
****************************************************************************************/
static void XcpCmdUnlock(uint8_t *data)
{
	/* key cannot be longer than XCP_CTO_PACKET_LEN-2 */
	if (data[1] > (XCP_CTO_PACKET_LEN-2))
	{
		/* key is too long incorrect */
		XcpSetCtoError(XCP_ERR_SEQUENCE);
		return;
	}

	/* verify the key */
	if (XcpVerifyKey(xcpInfo.s_n_k_resource, &data[2], data[1]) == 0)
	{
		/* invalid key so inform the master and do a disconnect */
		XcpSetCtoError(XCP_ERR_ACCESS_LOCKED);

		/* indicate that the xcp connection is disconnected */
		xcpInfo.connected = 0;

		/* enable resource protection */
		XcpProtectResources();

		return;
	}

	/* key correct so unlock the resource */
	xcpInfo.protection &= ~xcpInfo.s_n_k_resource;

	/* reset seed/key resource variable for possible next unlock */
	xcpInfo.s_n_k_resource = 0;

	/* set packet id to command response packet */
	xcpInfo.ctoData[0] = XCP_PID_RES;

	/* report the current resource protection */
	xcpInfo.ctoData[1] = xcpInfo.protection;

	/* set packet length */
	xcpInfo.ctoLen = 2;
}
#endif /* XCP_SEED_KEY_PROTECTION_EN == 1 */


#if (XCP_RES_PAGING_EN == 1)
/************************************************************************************//**
** \brief	XCP command processor function which handles the SET_CAL_PAGE command as
**			defined by the protocol.
** \param	data Pointer to a byte buffer with the packet data.
** \return	none
**
****************************************************************************************/
static void XcpCmdSetCalPage(uint8_t *data)
{
#if (XCP_SEED_KEY_PROTECTION_EN == 1)
	/* check if CAL_PAG resource is unlocked */
	if ((xcpInfo.protection & XCP_RES_CALPAG) == XCP_RES_CALPAG)
	{
		/* resource is locked. use seed/key sequence to unlock */
		XcpSetCtoError(XCP_ERR_ACCESS_LOCKED);
		return;
	}
#endif

	/* select the page. note that the mode parameter is ignored */
	if (XcpCalSetPageHook(data[2], data[3]) == 0)
	{
		/* calibration page could not be selected */
		XcpSetCtoError(XCP_ERR_PAGE_NOT_VALID);
		return;
	}

	/* set packet id to command response packet */
	xcpInfo.ctoData[0] = XCP_PID_RES;

	/* set packet length */
	xcpInfo.ctoLen = 1;
}


/************************************************************************************//**
** \brief	XCP command processor function which handles the GET_CAL_PAGE command as
**			defined by the protocol.
** \param	data Pointer to a byte buffer with the packet data.
** \return	none
**
****************************************************************************************/
static void XcpCmdGetCalPage(uint8_t *data)
{
#if (XCP_SEED_KEY_PROTECTION_EN == 1)
	/* check if CAL_PAG resource is unlocked */
	if ((xcpInfo.protection & XCP_RES_CALPAG) == XCP_RES_CALPAG)
	{
		/* resource is locked. use seed/key sequence to unlock */
		XcpSetCtoError(XCP_ERR_ACCESS_LOCKED);
		return;
	}
#endif

	/* set packet id to command response packet */
	xcpInfo.ctoData[0] = XCP_PID_RES;

	/* initialize reserved parameters */
	xcpInfo.ctoData[1] = 0;
	xcpInfo.ctoData[2] = 0;

	/* store the calibration page */
	xcpInfo.ctoData[3] = XcpCalGetPageHook(data[2]);

	/* set packet length */
	xcpInfo.ctoLen = 4;
}
#endif /* XCP_RES_PAGING_EN == 1 */


#if (XCP_RES_PROGRAMMING_EN == 1)
/************************************************************************************//**
** \brief	XCP command processor function which handles the PROGRAM_START command as
**			defined by the protocol.
** \param	data Pointer to a byte buffer with the packet data.
** \return	none
**
****************************************************************************************/
static void XcpCmdProgramStart(uint8_t *data)
{

#if (XCP_SEED_KEY_PROTECTION_EN == 1)
	/* check if PGM resource is unlocked */
	if ((xcpInfo.protection & XCP_RES_PGM) == XCP_RES_PGM)
	{
		/* resource is locked. use seed/key sequence to unlock */
		XcpSetCtoError(XCP_ERR_ACCESS_LOCKED);
		return;
	}
#endif

	/* set packet id to command response packet */
	xcpInfo.ctoData[0] = XCP_PID_RES;

	/* initialize reserved parameter */
	xcpInfo.ctoData[1] = 0;

	/* no special communication mode supported during programming */
	xcpInfo.ctoData[2] = 0;

	/* cto packet length stays the same during programming */
	xcpInfo.ctoData[3] = (uint8_t)XCP_CTO_PACKET_LEN;

	/* no block size, st-min time, or queue size supported */
	xcpInfo.ctoData[4] = 0;
	xcpInfo.ctoData[5] = 0;
	xcpInfo.ctoData[6] = 0;

	/* set packet length */
	xcpInfo.ctoLen = 7;
}


/************************************************************************************//**
** \brief	XCP command processor function which handles the PROGRAM_MAX command as
**			defined by the protocol.
** \param	data Pointer to a byte buffer with the packet data.
** \return	none
**
****************************************************************************************/
static void XcpCmdProgramMax(uint8_t *data)
{
#if (XCP_SEED_KEY_PROTECTION_EN == 1)
	/* check if PGM resource is unlocked */
	if ((xcpInfo.protection & XCP_RES_PGM) == XCP_RES_PGM)
	{
		/* resource is locked. use seed/key sequence to unlock */
		XcpSetCtoError(XCP_ERR_ACCESS_LOCKED);
		return;
	}
#endif

	/* program the data */
	if (NvmWrite((uint32_t)xcpInfo.mta, XCP_CTO_PACKET_LEN-1, &data[1]) == 0)
	{
		/* error occurred during programming */
		XcpSetCtoError(XCP_ERR_GENERIC);
		return;
	}

	/* set packet id to command response packet */
	xcpInfo.ctoData[0] = XCP_PID_RES;

	/* post increment the mta */
	xcpInfo.mta += XCP_CTO_PACKET_LEN-1;

	/* set packet length */
	xcpInfo.ctoLen = 1;
}


/************************************************************************************//**
** \brief	XCP command processor function which handles the PROGRAM command as
**			defined by the protocol.
** \param	data Pointer to a byte buffer with the packet data.
** \return	none
**
****************************************************************************************/
static void XcpCmdProgram(uint8_t *data)
{
#if (XCP_SEED_KEY_PROTECTION_EN == 1)
	/* check if PGM resource is unlocked */
	if ((xcpInfo.protection & XCP_RES_PGM) == XCP_RES_PGM)
	{
		/* resource is locked. use seed/key sequence to unlock */
		XcpSetCtoError(XCP_ERR_ACCESS_LOCKED);
		return;
	}
#endif

	/* validate length of download request */
	if (data[1] > (XCP_CTO_PACKET_LEN-2))
	{
		/* requested data length is too long */
		XcpSetCtoError(XCP_ERR_OUT_OF_RANGE);
		return;
	}

	/* set packet id to command response packet */
	xcpInfo.ctoData[0] = XCP_PID_RES;

	/* set packet length */
	xcpInfo.ctoLen = 1;

	/* end of programming sequence (datasize is 0)? */
	if (data[1] == 0)
	{
		/* call erase/programming cleanup routine */
		if (NvmDone() == false)
		{
			/* error occurred while finishing up programming */
			XcpSetCtoError(XCP_ERR_GENERIC);
		}
		return;
	}
	/* program the data */
	if (NvmWrite((uint32_t)xcpInfo.mta, data[1], &data[2]) == 0)
	{
		/* error occurred during programming */
		XcpSetCtoError(XCP_ERR_GENERIC);
		return;
	}

	/* post increment the mta */
	xcpInfo.mta += data[1];
}


/************************************************************************************//**
** \brief	XCP command processor function which handles the PROGRAM_CLEAR command as
**			defined by the protocol.
** \param	data Pointer to a byte buffer with the packet data.
** \return	none
**
****************************************************************************************/
static void XcpCmdProgramClear(uint8_t *data)
{
#if (XCP_SEED_KEY_PROTECTION_EN == 1)
	/* check if PGM resource is unlocked */
	if ((xcpInfo.protection & XCP_RES_PGM) == XCP_RES_PGM)
	{
		/* resource is locked. use seed/key sequence to unlock */
		XcpSetCtoError(XCP_ERR_ACCESS_LOCKED);
		return;
	}
#endif

	/* erase the memory */
	if (NvmErase((uint32_t)xcpInfo.mta, *(uint32_t*)&data[4]) == 0)
	{
		/* error occurred during erasure */
		XcpSetCtoError(XCP_ERR_GENERIC);
		return;
	}

	/* set packet id to command response packet */
	xcpInfo.ctoData[0] = XCP_PID_RES;

	/* set packet length */
	xcpInfo.ctoLen = 1;
}


/************************************************************************************//**
** \brief	XCP command processor function which handles the PROGRAM_RESET command as
**			defined by the protocol.
** \param	data Pointer to a byte buffer with the packet data.
** \return	none
**
****************************************************************************************/
static void XcpCmdProgramReset(uint8_t *data)
{

#if (XCP_SEED_KEY_PROTECTION_EN == 1)
	/* check if PGM resource is unlocked */
	if ((xcpInfo.protection & XCP_RES_PGM) == XCP_RES_PGM)
	{
		/* resource is locked. use seed/key sequence to unlock */
		XcpSetCtoError(XCP_ERR_ACCESS_LOCKED);
		return;
	}
#endif

	/* reset the ecu after programming is done. so basically, just start the newly programmed
	 * firmware. it is okay if the code does not return here. if CpuReset() is used here, then
	 * the bootloader is first activated again, including the backdoor timer which is not
	 * desired.
	 */
	CpuStartUserProgram();

	/* set packet id to command response packet */
	xcpInfo.ctoData[0] = XCP_PID_RES;

	/* set packet length */
	xcpInfo.ctoLen = 1;
}


/************************************************************************************//**
** \brief	XCP command processor function which handles the PROGRAM_PREPARE command as
**			defined by the protocol.
** \param	data Pointer to a byte buffer with the packet data.
** \return	none
**
****************************************************************************************/
static void XcpCmdProgramPrepare(uint8_t *data)
{
#if (XCP_SEED_KEY_PROTECTION_EN == 1)
	/* check if PGM resource is unlocked */
	if ((xcpInfo.protection & XCP_RES_PGM) == XCP_RES_PGM)
	{
		/* resource is locked. use seed/key sequence to unlock */
		XcpSetCtoError(XCP_ERR_ACCESS_LOCKED);
		return;
	}
#endif

	/* programming with kernel currently not needed and therefore not supported */
	XcpSetCtoError(XCP_ERR_GENERIC);
}
#endif /* XCP_RES_PROGRAMMING_EN == 1 */

/******************************** end of xcp.c *****************************************/
/************************************************************************************//**
* \file         Source\xcp.c
* \brief        XCP 1.0 protocol core source file.
* \ingroup      Core
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
