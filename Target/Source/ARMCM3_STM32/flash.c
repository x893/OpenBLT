#include "boot.h"	/* bootloader generic header	*/
#include "stm32f10x.h"


/****************************************************************************************
* Macro definitions
****************************************************************************************/
/** \brief Value for an invalid flash sector. */
#define FLASH_INVALID_SECTOR		(0xFF)

/** \brief Value for an invalid flash address. */
#define FLASH_INVALID_ADDRESS		(0xFFFFFFFF)

/** \brief Standard size of a flash block for writing. */
#define FLASH_WRITE_BLOCK_SIZE		(512)

/** \brief Total numbers of sectors in array flashLayout[]. */
#define FLASH_TOTAL_SECTORS			(sizeof(flashLayout) / sizeof(flashLayout[0]))

#if (BOOT_NVM_SIZE_KB > 128)
	/** \brief Number of bytes to erase per erase operation. */
	#define FLASH_ERASE_BLOCK_SIZE	(0x800)
#else
	/** \brief Number of bytes to erase per erase operation. */
	#define FLASH_ERASE_BLOCK_SIZE	(0x400)
#endif

/** \brief Offset into the user program's vector table where the checksum is located. */
extern uint32_t OpenBLT_Checksum;
#define FLASH_VECTOR_TABLE_CS_OFFSET	((uint32_t)&OpenBLT_Checksum - FLASH_BASE)

#define FLASH_KEY1			((uint32_t)0x45670123)
#define FLASH_KEY2			((uint32_t)0xCDEF89AB)
#define FLASH_LOCK_BIT		((uint32_t)0x00000080)
#define FLASH_EOP_BIT		((uint32_t)0x00000020)
#define FLASH_PGERR_BIT		((uint32_t)0x00000004)
#define FLASH_WRPRTERR_BIT	((uint32_t)0x00000010)
#define FLASH_BSY_BIT		((uint32_t)0x00000001)
#define FLASH_PER_BIT		((uint32_t)0x00000002)
#define FLASH_STRT_BIT		((uint32_t)0x00000040)
#define FLASH_PG_BIT		((uint32_t)0x00000001)


/****************************************************************************************
* Type definitions
****************************************************************************************/
/** \brief Flash sector descriptor type. */
typedef struct {
	uint32_t sector_start;	/**< sector start address	*/
	uint32_t sector_size;	/**< sector size in bytes	*/
	uint8_t  sector_num;	/**< sector number			*/
} FlashSector_t;

/** \brief    Structure type for grouping flash block information.
 *  \details  Programming is done per block of max FLASH_WRITE_BLOCK_SIZE. for this a 
 *            flash block manager is implemented in this driver. this flash block manager
 *            depends on this flash block info structure. It holds the base address of 
 *            the flash block and the data that should be programmed into the flash 
 *            block. The .base_addr must be a multiple of FLASH_WRITE_BLOCK_SIZE.
 */
typedef struct {
	uint32_t base_addr;
	uint8_t	 data[FLASH_WRITE_BLOCK_SIZE];
} FlashBlockInfo_t;

/****************************************************************************************
* Function prototypes
****************************************************************************************/
static bool FlashInitBlock(FlashBlockInfo_t *block, uint32_t address);
static FlashBlockInfo_t *FlashSwitchBlock(FlashBlockInfo_t *block, uint32_t base_addr);
static bool  FlashAddToBlock(FlashBlockInfo_t *block, uint32_t address, uint8_t *data, uint32_t len);
static bool  FlashWriteBlock(FlashBlockInfo_t *block);
static bool  FlashEraseSectors(uint8_t first_sector, uint8_t last_sector);
static void      FlashUnlock(void);
static void      FlashLock(void);
static uint8_t FlashGetSector(uint32_t address);
static uint32_t  FlashGetSectorBaseAddr(uint8_t sector);
static uint32_t  FlashGetSectorSize(uint8_t sector);


/****************************************************************************************
* Local constant declarations
****************************************************************************************/
/** \brief   Array wit the layout of the flash memory.
 *  \details Also controls what part of the flash memory is reserved for the bootloader. 
 *           If the bootloader size changes, the reserved sectors for the bootloader 
 *           might need adjustment to make sure the bootloader doesn't get overwritten.
 *           The current flash layout does not reflect the minimum sector size of the
 *           physical flash (1 - 2kb), because this would make the table quit long and
 *           a waste of ROM. The minimum sector size is only really needed when erasing
 *           the flash. This can still be done in combination with macro 
 *           FLASH_ERASE_BLOCK_SIZE.
 */
static const FlashSector_t flashLayout[] =
{
	/* space is reserved for a bootloader configuration with all supported communication
	 * interfaces enabled. when for example only UART is needed, than the space required
	 * for the bootloader can be made a lot smaller here.
	 */
	/* { FLASH_BASE, 0x02000,  0},           flash sector  0 - reserved for bootloader   */
#if (BOOT_SIZE_KB <= 8)
	{ FLASH_BASE + 0x02000,	0x02000,	1},		/* flash sector  1 - can reserved for bootloader   */
#endif
#if (BOOT_SIZE_KB <= 16)
	{ FLASH_BASE + 0x04000,	0x02000,	2},		/* flash sector  2 - can reserved for bootloader   */
#endif

	{ FLASH_BASE + 0x06000,	0x02000,	3},		/* flash sector  3 - 8kb	*/
#if (BOOT_NVM_SIZE_KB > 32)
	{ FLASH_BASE + 0x08000,	0x02000,	4},		/* flash sector  4 - 8kb	*/
	{ FLASH_BASE + 0x0A000,	0x02000,	5},		/* flash sector  5 - 8kb	*/
	{ FLASH_BASE + 0x0C000,	0x02000,	6},		/* flash sector  6 - 8kb	*/
	{ FLASH_BASE + 0x0E000,	0x02000,	7},		/* flash sector  7 - 8kb	*/
#endif
#if (BOOT_NVM_SIZE_KB > 64)
	{ FLASH_BASE + 0x10000, 0x02000,	8},		/* flash sector  8 - 8kb	*/
	{ FLASH_BASE + 0x12000,	0x02000,	9},		/* flash sector  9 - 8kb	*/
	{ FLASH_BASE + 0x14000,	0x02000,	10},	/* flash sector 10 - 8kb	*/
	{ FLASH_BASE + 0x16000,	0x02000,	11},	/* flash sector 11 - 8kb	*/
	{ FLASH_BASE + 0x18000,	0x02000,	12},	/* flash sector 12 - 8kb	*/
	{ FLASH_BASE + 0x1A000, 0x02000, 13},	/* flash sector 13 - 8kb	*/
	{ FLASH_BASE + 0x1C000, 0x02000, 14},	/* flash sector 14 - 8kb	*/
	{ FLASH_BASE + 0x1E000, 0x02000, 15},	/* flash sector 15 - 8kb	*/
#endif
#if (BOOT_NVM_SIZE_KB > 128)
	{ FLASH_BASE + 0x20000, 0x08000, 16},	/* flash sector 16 - 32kb	*/
	{ FLASH_BASE + 0x28000, 0x08000, 17},	/* flash sector 17 - 32kb	*/
	{ FLASH_BASE + 0x30000, 0x08000, 18},	/* flash sector 18 - 32kb	*/
	{ FLASH_BASE + 0x38000, 0x08000, 19},	/* flash sector 19 - 32kb	*/
#endif
#if (BOOT_NVM_SIZE_KB > 256)
	{ FLASH_BASE + 0x40000, 0x08000, 20},	/* flash sector 20 - 32kb	*/
	{ FLASH_BASE + 0x48000, 0x08000, 21},	/* flash sector 21 - 32kb	*/
	{ FLASH_BASE + 0x50000, 0x08000, 22},	/* flash sector 22 - 32kb	*/
	{ FLASH_BASE + 0x58000, 0x08000, 23},	/* flash sector 23 - 32kb	*/
	{ FLASH_BASE + 0x60000, 0x08000, 24},	/* flash sector 24 - 32kb	*/
	{ FLASH_BASE + 0x68000, 0x08000, 25},	/* flash sector 25 - 32kb	*/
	{ FLASH_BASE + 0x70000, 0x08000, 26},	/* flash sector 26 - 32kb	*/
	{ FLASH_BASE + 0x78000, 0x08000, 27},	/* flash sector 27 - 32kb	*/
#endif

#if (BOOT_NVM_SIZE_KB > 512)
	#error "BOOT_NVM_SIZE_KB > 512 is currently not supported."
#endif
};


/****************************************************************************************
* Local data declarations
****************************************************************************************/
/** \brief   Local variable with information about the flash block that is currently
 *           being operated on.
 *  \details The smallest amount of flash that can be programmed is 
 *           FLASH_WRITE_BLOCK_SIZE. A flash block manager is implemented in this driver
 *           and stores info in this variable. Whenever new data should be flashed, it
 *           is first added to a RAM buffer, which is part of this variable. Whenever
 *           the RAM buffer, which has the size of a flash block, is full or  data needs
 *           to be written to a different block, the contents of the RAM buffer are 
 *           programmed to flash. The flash block manager requires some software 
 *           overhead, yet results is faster flash programming because data is first 
 *           harvested, ideally until there is enough to program an entire flash block, 
 *           before the flash device is actually operated on.
 */
static FlashBlockInfo_t blockInfo;

/** \brief   Local variable with information about the flash boot block.
 *  \details The first block of the user program holds the vector table, which on the 
 *           STM32 is also the where the checksum is written to. Is it likely that 
 *           the vector table is first flashed and then, at the end of the programming
 *           sequence, the checksum. This means that this flash block need to be written
 *           to twice. Normally this is not a problem with flash memory, as long as you
 *           write the same values to those bytes that are not supposed to be changed 
 *           and the locations where you do write to are still in the erased 0xFF state.
 *           Unfortunately, writing twice to flash this way, does not work reliably on 
 *           all micros. This is why we need to have an extra block, the bootblock,
 *           placed under the management of the block manager. This way is it possible 
 *           to implement functionality so that the bootblock is only written to once
 *           at the end of the programming sequence.
 */
static FlashBlockInfo_t bootBlockInfo;


/************************************************************************************//**
** \brief	Initializes the flash driver. 
** \return	none.
**
****************************************************************************************/
void FlashInit(void)
{
	// init the flash block info structs by setting the address to an invalid address
	blockInfo.base_addr = FLASH_INVALID_ADDRESS;
	bootBlockInfo.base_addr = FLASH_INVALID_ADDRESS;
}


/************************************************************************************//**
** \brief	Writes the data to flash through a flash block manager. Note that this
**			function also checks that no data is programmed outside the flash 
**			memory region, so the bootloader can never be overwritten.
** \param	addr Start address.
** \param	len  Length in bytes.
** \param	data Pointer to the data buffer.
** \return	true if successful, false otherwise. 
**
****************************************************************************************/
bool FlashWrite(uint32_t addr, uint32_t len, uint8_t *data)
{
	uint32_t base_addr;

	// make sure the addresses are within the flash device
	if ((FlashGetSector(addr) == FLASH_INVALID_SECTOR)
	||	(FlashGetSector(addr+len-1) == FLASH_INVALID_SECTOR)
		)
	{
		return false;       
	}

	// if this is the bootblock, then let the boot block manager handle it
	base_addr = (addr/FLASH_WRITE_BLOCK_SIZE)*FLASH_WRITE_BLOCK_SIZE;
	if (base_addr == flashLayout[0].sector_start)
	{	// let the boot block manager handle it
		return FlashAddToBlock(&bootBlockInfo, addr, data, len);
	}
	// let the block manager handle it
	return FlashAddToBlock(&blockInfo, addr, data, len);
}


/************************************************************************************//**
** \brief	Erases the flash memory. Note that this function also checks that no 
**			data is erased outside the flash memory region, so the bootloader can 
**			never be erased.
** \param	addr Start address.
** \param	len  Length in bytes.
** \return	true if successful, false otherwise. 
**
****************************************************************************************/
bool FlashErase(uint32_t addr, uint32_t len)
{
	uint8_t first_sector;
	uint8_t last_sector;

	// obtain the first and last sector number
	first_sector = FlashGetSector(addr);
	last_sector  = FlashGetSector(addr + len - 1);
	// check them
	if ( (first_sector == FLASH_INVALID_SECTOR) || (last_sector == FLASH_INVALID_SECTOR) )
		return false;

	// erase the sectors
	return FlashEraseSectors(first_sector, last_sector);
}


/************************************************************************************//**
** \brief	Writes a checksum of the user program to non-volatile memory. This is
**			performed once the entire user program has been programmed. Through
**			the checksum, the bootloader can check if the programming session
**			was completed, which indicates that a valid user programming is
**			present and can be started.
** \return	true if successful, false otherwise. 
**
****************************************************************************************/
bool FlashWriteChecksum(void)
{
	uint32_t signature_checksum = 0;
	uint32_t * addr = (uint32_t *)flashLayout[0].sector_start;

	/* for the STM32 target we defined the checksum as the Two's complement value of the
	 * sum of the first 7 exception addresses.
	 *
	 * Layout of the vector table:
	 *    0x08000000 Initial stack pointer 
	 *    0x08000004 Reset Handler
	 *    0x08000008 NMI Handler
	 *    0x0800000C Hard Fault Handler
	 *    0x08000010 MPU Fault Handler 
	 *    0x08000014 Bus Fault Handler
	 *    0x08000018 Usage Fault Handler
	 *
	 *    signature_checksum = Two's complement of (SUM(exception address values))
	 *   
	 *    the bootloader writes this 32-bit checksum value right after the vector table
	 *    of the user program. note that this means one extra dummy entry must be added
	 *    at the end of the user program's vector table to reserve storage space for the
	 *    checksum.
	 */

	/* first check that the bootblock contains valid data. if not, this means the
	 * bootblock is not part of the reprogramming this time and therefore no
	 * new checksum needs to be written
	 */
	if (bootBlockInfo.base_addr == FLASH_INVALID_ADDRESS)
		return true;

	/* compute the checksum. note that the user program's vectors are not yet written
	 * to flash but are present in the bootblock data structure at this point.
	 */
	signature_checksum += *addr++;
	signature_checksum += *addr++;
	signature_checksum += *addr++;
	signature_checksum += *addr++;
	signature_checksum += *addr++;
	signature_checksum += *addr++;
	signature_checksum += *addr++;
	signature_checksum  = ~signature_checksum;	// one's complement
	signature_checksum += 1;					// two's complement

	// write the checksum
	return FlashWrite(
		flashLayout[0].sector_start + FLASH_VECTOR_TABLE_CS_OFFSET,
		sizeof(uint32_t),
		(uint8_t *)&signature_checksum
		);
}


/************************************************************************************//**
** \brief	Verifies the checksum, which indicates that a valid user program is
**			present and can be started.
** \return	true if successful, false otherwise.
**
****************************************************************************************/
bool FlashVerifyChecksum(void)
{
	uint32_t signature_checksum = 0;
	uint32_t * addr = (uint32_t *)flashLayout[0].sector_start;

	/* verify the checksum based on how it was written by CpuWriteChecksum() */
	signature_checksum += *addr++;
	signature_checksum += *addr++;
	signature_checksum += *addr++;
	signature_checksum += *addr++;
	signature_checksum += *addr++;
	signature_checksum += *addr++;
	signature_checksum += *addr++;
	signature_checksum += *((uint32_t *)(flashLayout[0].sector_start + FLASH_VECTOR_TABLE_CS_OFFSET));

	// sum should add up to an unsigned 32-bit value of 0
	if (signature_checksum == 0)
		return true;	// checksum okay

	return false;		// checksum incorrect
}


/************************************************************************************//**
** \brief	Finalizes the flash driver operations. There could still be data in
**			the currently active block that needs to be flashed.
** \return	true if successful, false otherwise. 
**
****************************************************************************************/
bool FlashDone(void)
{
	// check if there is still data waiting to be programmed in the boot block
	if (bootBlockInfo.base_addr != FLASH_INVALID_ADDRESS
	&&	(! FlashWriteBlock(&bootBlockInfo))
		)
		return false;

	/* check if there is still data waiting to be programmed */
	if (blockInfo.base_addr != FLASH_INVALID_ADDRESS
	&&	(! FlashWriteBlock(&blockInfo))
		)
	{
		return false;
	}

	/* still here so all is okay */  
	return true;
}


/************************************************************************************//**
** \brief	Obtains the base address of the flash memory available to the user program.
**			This is basically the first address in the flashLayout table.
** \return	Base address.
**
****************************************************************************************/
uint32_t FlashGetUserProgBaseAddress(void)
{
	return flashLayout[0].sector_start;
}


/************************************************************************************//**
** \brief	Copies data currently in flash to the block->data and sets the 
**			base address.
** \param	block   Pointer to flash block info structure to operate on.
** \param	address Base address of the block data.
** \return	true if successful, false otherwise. 
**
****************************************************************************************/
static bool FlashInitBlock(FlashBlockInfo_t *block, uint32_t address)
{
	// check address alignment
	if ((address % FLASH_WRITE_BLOCK_SIZE) != 0)
		return false;

	// make sure that we are initializing a new block and not the same one
	if (block->base_addr == address)
	{	// block already initialized, so nothing to do
		return true;
	}
	// set the base address and copies the current data from flash
	block->base_addr = address;  
	CpuMemCopy((uint32_t)block->data, address, FLASH_WRITE_BLOCK_SIZE);
	return true;
}


/************************************************************************************//**
** \brief	Switches blocks by programming the current one and initializing the
**			next.
** \param	block   Pointer to flash block info structure to operate on.
** \param	base_addr Base address of the next block.
** \return	The pointer of the block info struct that is no being used, or a NULL
**			pointer in case of error.
**
****************************************************************************************/
static FlashBlockInfo_t *FlashSwitchBlock(FlashBlockInfo_t *block, uint32_t base_addr)
{
	/* check if a switch needs to be made away from the boot block. in this case the boot
	 * block shouldn't be written yet, because this is done at the end of the programming
	 * session by FlashDone(), this is right after the checksum was written. 
	 */
	if (block == &bootBlockInfo)
	{	// switch from the boot block to the generic block info structure
		block = &blockInfo;
	}
	/* check if a switch back into the bootblock is needed. in this case the generic block 
	 * doesn't need to be written here yet.
	 */
	else if (base_addr == flashLayout[0].sector_start)
	{	// switch from the generic block to the boot block info structure
		block = &bootBlockInfo;
		base_addr = flashLayout[0].sector_start;
	}
	else
	{	// need to switch to a new block, so program the current one and init the next
		if ( ! FlashWriteBlock(block))
			return NULL;
	}

	// initialize tne new block when necessary 
	if ( ! FlashInitBlock(block, base_addr))
		return NULL;
	return block;	// still here to all is okay
}


/************************************************************************************//**
** \brief	Programming is done per block. This function adds data to the block
**			that is currently collecting data to be written to flash. If the
**			address is outside of the current block, the current block is written
**			to flash an a new block is initialized.
** \param	block   Pointer to flash block info structure to operate on.
** \param	address Flash destination address.
** \param	data    Pointer to the byte array with data.
** \param	len     Number of bytes to add to the block.
** \return	true if successful, false otherwise.
**
****************************************************************************************/
static bool FlashAddToBlock(FlashBlockInfo_t *block, uint32_t address, uint8_t *data, uint32_t len)
{
	uint32_t   current_base_addr;
	uint8_t  *dst;
	uint8_t  *src;
  
	// determine the current base address
	current_base_addr = (address/FLASH_WRITE_BLOCK_SIZE)*FLASH_WRITE_BLOCK_SIZE;

	// make sure the blockInfo is not uninitialized
	if (block->base_addr == FLASH_INVALID_ADDRESS)
	{	// initialize the blockInfo struct for the current block
		if ( ! FlashInitBlock(block, current_base_addr))
			return false;
	}

	// check if the new data fits in the current block
	if (block->base_addr != current_base_addr)
	{	// need to switch to a new block, so program the current one and init the next
		block = FlashSwitchBlock(block, current_base_addr);
		if (block == NULL)
			return false;
	}

	// add the data to the current block, but check for block overflow
	dst = &(block->data[address - block->base_addr]);
	src = data;
	do
	{
		CopService();	// keep the watchdog happy
		// buffer overflow ?
		if ((uint32_t)(dst-&(block->data[0])) >= FLASH_WRITE_BLOCK_SIZE)
		{	// need to switch to a new block, so program the current one and init the next
			block = FlashSwitchBlock(block, current_base_addr+FLASH_WRITE_BLOCK_SIZE);
			if (block == NULL)
				return false;
			// reset destination pointer
			dst = &(block->data[0]);
		}

		*dst++ = *src++;	// write the data to the buffer
		len--;				// decrement byte counter
	} while (len > 0);

	return true;	// still here so all is good
}


/************************************************************************************//**
** \brief	Programs FLASH_WRITE_BLOCK_SIZE bytes to flash from the block->data
**			array.
** \param	block   Pointer to flash block info structure to operate on.
** \return	true if successful, false otherwise.
**
****************************************************************************************/
static bool FlashWriteBlock(FlashBlockInfo_t *block)
{
	uint8_t  sector_num;
	bool   result = true;
	uint32_t   prog_addr;
	uint32_t prog_data;
	uint32_t word_cnt;

	// check that address is actually within flash
	sector_num = FlashGetSector(block->base_addr);
	if (sector_num == FLASH_INVALID_SECTOR)
	{
		return false;
	}

	// unlock the flash array
	FlashUnlock();

	// check that the flash peripheral is not busy
	if ((FLASH->SR & FLASH_BSY_BIT) == FLASH_BSY_BIT)
	{
		FlashLock();	// lock the flash array again
		return false;	// could not perform erase operation
	}

	FLASH->CR |= FLASH_PG_BIT;	// set the program bit to indicate that we are about to program data

	// program all words in the block one by one
	for (word_cnt = 0; word_cnt < (FLASH_WRITE_BLOCK_SIZE / sizeof(uint32_t)); word_cnt++)
	{
		prog_addr = block->base_addr + (word_cnt * sizeof(uint32_t));
		prog_data = *(volatile uint32_t*)(&block->data[word_cnt * sizeof(uint32_t)]);

		// program the first half word
		*(volatile uint16_t*)prog_addr = (uint16_t)prog_data;
		// wait for the program operation to complete
		while ((FLASH->SR & FLASH_BSY_BIT) == FLASH_BSY_BIT)
		{	// keep the watchdog happy
			CopService();
		}
		// program the second half word
		*(volatile uint16_t*)(prog_addr + 2) = (uint16_t)(prog_data >> 16);
		// wait for the program operation to complete
		while ((FLASH->SR & FLASH_BSY_BIT) == FLASH_BSY_BIT)
		{	// keep the watchdog happy
			CopService();
		}

		// verify that the written data is actually there
		if (*(volatile uint32_t*)prog_addr != prog_data)
		{
			result = false;
			break;
		}
	}

	FLASH->CR &= ~FLASH_PG_BIT;	// reset the program bit to indicate that we are done programming data
	FlashLock();				// lock the flash array
	return result;				// still here so all is okay
}


/************************************************************************************//**
** \brief	Erases the flash sectors from first_sector up until last_sector.
** \param	first_sector First flash sector number.
** \param	last_sector  Last flash sector number.
** \return	true if successful, false otherwise.
**
****************************************************************************************/
static bool FlashEraseSectors(uint8_t first_sector, uint8_t last_sector)
{
	uint16_t nr_of_blocks;
	uint16_t block_cnt;
	uint32_t   start_addr;
	uint32_t   end_addr;

	// validate the sector numbers
	if (first_sector > last_sector)
	{
		return false;
	}
	if ((first_sector < flashLayout[0].sector_num)
	||	(last_sector > flashLayout[FLASH_TOTAL_SECTORS-1].sector_num)
		)
	{
		return false;
	}
	// unlock the flash array
	FlashUnlock();
	// check that the flash peripheral is not busy
	if ((FLASH->SR & FLASH_BSY_BIT) == FLASH_BSY_BIT)
	{
		// lock the flash array again
		FlashLock();
		// could not perform erase operation
		return false;
	}
	// set the page erase bit to indicate that we are about to erase a block
	FLASH->CR |= FLASH_PER_BIT;

	// determine how many blocks need to be erased
	start_addr = FlashGetSectorBaseAddr(first_sector);
	end_addr = FlashGetSectorBaseAddr(last_sector) + FlashGetSectorSize(last_sector) - 1;
	nr_of_blocks = (end_addr - start_addr + 1) / FLASH_ERASE_BLOCK_SIZE;

	// erase all blocks one by one
	for (block_cnt=0; block_cnt<nr_of_blocks; block_cnt++)
	{
		// store an address of the block that is to be erased to select the block
		FLASH->AR = start_addr + (block_cnt * FLASH_ERASE_BLOCK_SIZE);
		// start the block erase operation
		FLASH->CR |= FLASH_STRT_BIT;
		// wait for the erase operation to complete
		while ((FLASH->SR & FLASH_BSY_BIT) == FLASH_BSY_BIT)
		{	
			CopService();	// keep the watchdog happy
		}
	}
	// reset the page erase bit because we're all done erasing
	FLASH->CR &= ~FLASH_PER_BIT;
	// lock the flash array
	FlashLock();
	// still here so all went okay
	return true;
}


/************************************************************************************//**
** \brief	Unlocks the flash array so that erase and program operations can be
**			performed.
** \return	none.
**
****************************************************************************************/
static void FlashUnlock(void)
{
	// authorize the FPEC to access bank 1
	FLASH->KEYR = FLASH_KEY1;
	FLASH->KEYR = FLASH_KEY2;
	// clear all possibly pending status flags
	FLASH->SR = (FLASH_EOP_BIT | FLASH_PGERR_BIT | FLASH_WRPRTERR_BIT);
}


/************************************************************************************//**
** \brief	Locks the flash array so that erase and program operations can no
**			longer be performed.
** \return	none.
**
****************************************************************************************/
static void FlashLock(void)
{
	// set the lock bit to lock the FPEC
	FLASH->CR |= FLASH_LOCK_BIT;
}


/************************************************************************************//**
** \brief	Determines the flash sector the address is in.
** \param	address Address in the flash sector.
** \return	Flash sector number or FLASH_INVALID_SECTOR.
**
****************************************************************************************/
static uint8_t FlashGetSector(uint32_t address)
{
	uint8_t sectorIdx;
  
	// search through the sectors to find the right one
	for (sectorIdx = 0; sectorIdx < FLASH_TOTAL_SECTORS; sectorIdx++)
	{
		// keep the watchdog happy
		CopService();
		// is the address in this sector ?
		if ((address >= flashLayout[sectorIdx].sector_start)
		&&	(address < (flashLayout[sectorIdx].sector_start + flashLayout[sectorIdx].sector_size))
			)
		{	// return the sector number
			return flashLayout[sectorIdx].sector_num;
		}
	}
	// still here so no valid sector found
	return FLASH_INVALID_SECTOR;
}


/************************************************************************************//**
** \brief	Determines the flash sector base address.
** \param	sector Sector to get the base address of.
** \return	Flash sector base address or FLASH_INVALID_ADDRESS.
**
****************************************************************************************/
static uint32_t FlashGetSectorBaseAddr(uint8_t sector)
{
	uint8_t sectorIdx;

	// search through the sectors to find the right one
	for (sectorIdx = 0; sectorIdx < FLASH_TOTAL_SECTORS; sectorIdx++)
	{	// keep the watchdog happy
		CopService();
		if (flashLayout[sectorIdx].sector_num == sector)
			return flashLayout[sectorIdx].sector_start;
	}
	// still here so no valid sector found
	return FLASH_INVALID_ADDRESS;
}


/************************************************************************************//**
** \brief	Determines the flash sector size.
** \param	sector Sector to get the size of.
** \return	Flash sector size or 0.
**
****************************************************************************************/
static uint32_t FlashGetSectorSize(uint8_t sector)
{
	uint8_t sectorIdx;

	// search through the sectors to find the right one
	for (sectorIdx = 0; sectorIdx < FLASH_TOTAL_SECTORS; sectorIdx++)
	{	// keep the watchdog happy
		CopService();
		if (flashLayout[sectorIdx].sector_num == sector)
			return flashLayout[sectorIdx].sector_size;
	}
	// still here so no valid sector found
	return 0;
}

/*********************************** end of flash.c ************************************/
/************************************************************************************//**
* \file         Source\ARMCM3_STM32\flash.c
* \brief        Bootloader flash driver source file.
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
