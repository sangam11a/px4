/****************************************************************************
 * drivers/mtd/mt25ql.c
 *
*/

/* Driver for SPI-based MT25QL (2Gbit, 1Gbit, 512Mbit, 256MBit, 64Mbit) FLASH
 * (and compatible)
 */
/****************************************************************************
 * Included Files
 ****************************************************************************/
#include <nuttx/config.h>

#include <sys/types.h>
#include <inttypes.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/signal.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/spi/spi.h>
#include <nuttx/mtd/mtd.h>

#ifndef MT25QL_H_
#define MT25QL_H_

#ifdef __cplusplus
extern "C" {
#endif


/***************************************************************************
 * Pre-Processor Definitions
****************************************************************************/
#ifndef CONFIG_MT25QL_SPIMODE
#define CONFIG_MT25QL_SPIMODE	SPIDEV_MODE0
#endif

#ifndef CONFIG_MT25QL_SPIFREQUENCY
#define CONFIG_MT25QL_SPIFREQUENCY	20000000
#endif

#ifndef CONFIG_MT25QL_MANUFACTURER
#define CONFIG_MT25QL_MANUFACTURER	0x20
#endif

#ifndef CONFIG_MT25QL_MEMORY_TYPE
#define CONFIG_MT25QL_MEMORY_TYPE
#endif


/* MT25QL Registers ********************************************************/

/* Identification Registers values */

#define MT25QL_MANFACTURER		CONFIG_MT25QL_MANUFACTURER	// Micron Manufactrer ID assigned by JEDEC
#define MT25Q_MEMORY_TYPE         	CONFIG_MT25QL_MEMORY_TYPE
#define MT25QL_CAPACITY_2GB		0x22	// 2Gb
#define MT25QL_CAPACITY_1GB		0x21	// 1Gb
#define MT25QL_CAPACITY_512		0x20	// 512Mb
#define MT25QL_CAPACITY_256		0x19	// 256Mb
#define MT25QL_CAPACITY_128		0x18	// 128Mb
#define MT25QL_CAPACITY_64		0x17	// 64Mb

/* SPI Commands ***********************************************************/

/* Basic configration operations */

#define MT25QL_RSTEN		0x66	/* Reset Enable */
#define MT25QL_RSTMEM		0x99	/* Reset Memory */
#define MT25QL_WRTEN		0x06	/* Write Enable */
#define MT25QL_WRTDIS		0x04	/* Write Disable */

#define MT25QL_E4BYTMD		0xB7	/* Enter 4-Byte Addressing Mode */
#define MT25QL_X4BYTMD		0xE9	/* Exit 4-Byte Addressing Mode */
#define MT25QL_EDEPPWRDWN	0xB9	/* Enter Deep Power Down */
#define MT25QL_XDEPPWRDWN	0xAB	/* Exit Deep Power Down */

#define MT25QL_PRGERSSUS	0x75	/* Program Erase Suspend Operation */
#define MT25QL_PRGERRES		0x7A	/* Program Erase Resume Operation */

#define MT25QL_UNLCKPWD		0x29	/* Unlock Password*/

/* Read Operations */

#define MT25QL_RDDEVID			0x9F	/* READ device id */
#define MT25QL_RDSRFFLSPARAM		0x5A 	/* Read serial flash discovery parameter */
#define MT25QL_RDSECTPRCT		0x2D	/* Sector protection bits read */
#define MT25QL_RDVOLLCKBIT		0xE8	/* Volatile lock bits read */
#define MT25QL_RDNVOLLCKBIT		0xE2	/* Non-Volatile lock bits read */
#define MT25QL_RDGBLFRZBIT		0xA7	/* Global Freeze bits read */
#define MT25QL_RDPWD			0x27	/* Read password */
#define MT25QL_RDOTPARRY		0x4B	/* Read OTP Array*/

#define MT25QL_RDSTATREG		0x05	/* Read status register */
#define MT25QL_RDFLGSTATREG		0x70	/* Read flag status register */
#define MT25QL_RDNVOLCONREG		0xB5	/* Read Non-Volatile Configuration Register */
#define MT25QL_RDVOLCONREG		0x85	/* Read Volatile Configuration Register */
#define MT25QL_RDEVOLCONREG		0x65	/* Read Enchanced Configuration Register */
#define MT25QL_RDEXTADDRREG		0xC8	/* Read Extended Address Register */
#define MT25QL_RDGENPUPRDREG		0x96	/* Read General Purpose Read Register */


/* Write Operations */

#define MT25QL_WRTSTATREG		0x01	/* Write Status Register */
#define MT25QL_WRTNVOLCONREG		0xB1	/* Write Non-Volatile Configuration Register */
#define MT25QL_WRTVOLCONREG		0x81	/* Write Volatile Configuration Register */
#define MT25QL_WRTENVOLCONREG		0x61	/* Write Enhanced Volatile Configuration Register */
#define MT25QL_WRTEXTADDRREG		0xC5	/* Write Extended Address Register */

#define MT25QL_WRTSECTPRCT		0x2c	/* Program sector protection register */
#define MT25QL_WRTVOLLCKBIT		0xE5	/* Write Volatile Lock Bits */
#define MT25QL_WRTNVOLLCKBIT		0xE3	/* Write Non-Volatile Lock Bits */
#define MT25QL_WRTGBLFRZBIT		0xA6	/* Write Global Freeze Bits */
#define MT25QL_WRTPWD			0x28	/* Write Password */
#define MT25QL_WRTOTPARRY		0xE9	/* Program OTP Array */

/* Erase Operations */
#define MT25QL_CLRFLGSTATREG		0x50	/* Clear Flag Status Register */
#define MT25QL_CHIPERASE		0xC4	/* Chip Erase, all sectors */
#define MT25QL_ERSNVOLLCKBIT		0xE4	/* Eraser Non-Volatile Lock Bits */

/* 3-byte addressing mode commands*/

/* 3-Byte Read Commands */
#define MT25QL_3BMRD			0x03	/* Read memory 3-byte addressing Mode */
#define MT25QL_3BMFSTRD			0x0B	/* Fast Read Memory 3-Byte Addressing Mode */

/* 3-Byte Program Commands */

#define MT25QL_3BMPRGPG			0x02	/* Page Program 3-Byte addressing Mode */

/* 3-Byte Erase Commands */
#define MT25QL_3BMSECTERS		0xD8	/* 64-KB Sector Erase 3-Byte addressing Mode */
#define MT25QL_3BM32KSUBSECTERS		0x52	/* 32-KB Sub-Sector Erase 3-Byte addressing Mode */
#define MT25QL_3BM4KSUBSECTERS		0x20	/* 4-KB Sub-Sector Erase 3-Byte addressing Mode */


/* 4-Byte Addressing Mode Commands */

/* 4-Byte Read Commands */
#define MT25QL_4BMRD			0x13	/* Read Memory 4-Byte Addressing Mode */
#define MT25QL_4BMFSTRD			0x0C	/* Fast Read Memory 4-Byte Addressing Mode */

/* 4-Byte Program Commands */

#define MT25QL_4BMPRGPG			0x12	/* Page Program 3-Byte Addressing Mode */

/* 4-Byte Erase Commands */

#define MT25QL_4BMSECTERS		0xDC	/* 64-KB Sector Erase 4-Byte Addressing Mode */
#define MT25QL_4BM32KSUBSECTERS		0x5C	/* 32-KB Sub-Sector Erase 4-Byte Addressing Mode */
#define MT25QL_4BM4KSUBSECTERS		0x21	/* 4-KB Sub-Sector Erase 4-Byte Addressing Mode */

#define MT25QL_DUMMY			0xA5

/* Status Register Bit Definations [Default] */
#define MT25QL_SRWEN		(1 << 7)		/* Bit 7: Write Enable -> 0 | Write Disable -> 1 [used with W# Line]*/
#define MT25QL_SRTP		(1 << 5)		/* Bit 5: Block protection from TOP/BOT [TOP -> 0 | BOT -> 1]*/
#define MT25QL_SRBP		((1<< 6) | (7 << 2))	/* Bit 6, 4:2 : Block Protection bits against PRG/ERS*/
#define MT25QL_SRWENL		(1 << 1)		/* Bit 1: Write Enable latch [Write Disable -> 0 | Write_Enable -> 1]*/
#define MT25QL_SRRDY		(1 << 0) 		/* Bit 0: Ready -> 0 | Busy -> 1 */

/* Status Flag Register Bit Definations */
#define MT25QL_SFRRDY		(1 << 7)		/* Bit 7: Ready / Not Busy */
#define MT25QL_SFRERSUS		(1 << 6)		/* Bit 6: Erase Suspend Clear */
#define MT25QL_SFRERSTAT	(1 << 5)		/* Bit 5: Erase Success */
#define MT25QL_SFRPRGSTAT	(1 << 4)		/* Bit 4: Program / CRC check success */
#define MT25QL_SFRPRGSUS	(1 << 2)		/* Bit 2: Program Suspend Clear */
#define MT25QL_SFRPRCTTRIG	(1 << 1)		/* Bit 1: Block Protection Tigger Clear */
#define MT25QL_SFRADDRMD	(1 << 0)		/* Bit 0: 4-Byte Addressing Mode Enabled  [3-Byte Mode default]*/

/*  MT25QL 64 capacity is 8,388,608 bytes:
 *  (128 sectors) * (65,536 bytes per sector)
 *  (32768 pages) * (256 bytes per page)
 */

#define MT25Q64_SECTOR_SHIFT  16    /* Sector size 1 << 16 = 65,536 */
#define MT25Q64_NSECTORS      128
#define MT25Q64_PAGE_SHIFT    8     /* Page size 1 << 8 = 256 */
#define MT25Q64_NPAGES        32768
#define MT25Q64_SUBSECT_SHIFT 12    /* Sub-Sector size 1 << 12 = 4,096 */

/*  MT25QL 128 capacity is 16,777,216 bytes:
 *  (256 sectors) * (65,536 bytes per sector)
 *  (65536 pages) * (256 bytes per page)
 */

#define MT25Q128_SECTOR_SHIFT  16    /* Sector size 1 << 16 = 65,536 */
#define MT25Q128_NSECTORS      256
#define MT25Q128_PAGE_SHIFT    8     /* Page size 1 << 8 = 256 */
#define MT25Q128_NPAGES        65536
#define MT25Q128_SUBSECT_SHIFT 12    /* Sub-Sector size 1 << 12 = 4,096 */

/*  MT25QL 256 capacity is 33,554,432 bytes:
 *  (512 sectors) * (65,536 bytes per sector)
 *  (131072 pages) * (256 bytes per page)
 */

#define MT25Q256_SECTOR_SHIFT  16    /* Sector size 1 << 16 = 65,536 */
#define MT25Q256_NSECTORS      512
#define MT25Q256_PAGE_SHIFT    8     /* Page size 1 << 8 = 256 */
#define MT25Q256_NPAGES        131072
#define MT25Q256_SUBSECT_SHIFT 12    /* Sub-Sector size 1 << 12 = 4,096 */

/*  MT25QL 512 capacity is 67,108,864 bytes:
 *  (1024 sectors) * (65,536 bytes per sector)
 *  (262144 pages) * (256 bytes per page)
 */

#define MT25Q512_SECTOR_SHIFT  16    /* Sector size 1 << 16 = 65,536 */
#define MT25Q512_NSECTORS      1024
#define MT25Q512_PAGE_SHIFT    8     /* Page size 1 << 8 = 256 */
#define MT25Q512_NPAGES        262144
#define MT25Q512_SUBSECT_SHIFT 12    /* Sub-Sector size 1 << 12 = 4,096 */

/*  MT25QL 1G capacity is 134,217,728  bytes:
 *  (2048 sectors) * (65,536 bytes per sector)
 *  (524288 pages) * (256 bytes per page)
 */

#define MT25Q1G_SECTOR_SHIFT  16    /* Sector size 1 << 16 = 65,536 */
#define MT25Q1G_NSECTORS      2048
#define MT25Q1G_PAGE_SHIFT    8     /* Page size 1 << 8 = 256 */
#define MT25Q1G_NPAGES        524288
#define MT25Q1G_SUBSECT_SHIFT 12    /* Sub-Sector size 1 << 12 = 4,096 */

/*  MT25QL 2G capacity is 268,435,436  bytes:
 *  (4096 sectors) * (65,536 bytes per sector)
 *  (1048576 pages) * (256 bytes per page)
 */

#define MT25Q2G_SECTOR_SHIFT  16    /* Sector size 1 << 16 = 65,536 */
#define MT25Q2G_NSECTORS      4096
#define MT25Q2G_PAGE_SHIFT    8     /* Page size 1 << 8 = 256 */
#define MT25Q2G_NPAGES        1048576
#define MT25Q2G_SUBSECT_SHIFT 12    /* Sub-Sector size 1 << 12 = 4,096 */
/***************************************************************************************
 * Private Types
 *
 * *************************************************************************************
*/

/* This type represents the state of the MTD Device. The Struct mtd_dev_s
 * must appear at the beginning of the defination so that switching between
 * mtd_dev_s strutcture and mt25QL_dev_s stucture will be smooth.
*/

struct mt25ql_dev_s
{
	struct mtd_dev_s mtd;			/* MTD Interface */
	FAR struct spi_dev_s *dev;		/* saved SPI interface instance */
	uint8_t sectorshift;
	uint8_t pageshift;			/* log2 of the page size*/
	uint8_t nsectors;
	uint32_t npages;			/* Number of pages in the devices*/
	#ifdef CONFIG_MT25QL_SUBSECTOR_ERASE
	uint8_t subsectorshift;
	#endif
};


/***************************************************************************************
 * Private Function Prototpyes
 * *************************************************************************************
*/

/* Lock and per-transaction configuration */
/* This function is used for locking the SPI lines when using the connected device
 * used when multiple devices are connected on the same bus
*/
static void mt25ql_lock(FAR struct spi_dev_s *dev);
static inline void mt25ql_unlock(FAR struct spi_dev_s *dev); /* used for unlocking the  spi bus */

/* use this prototype to define write disable the flash */
/* use this prototpye to write enable the flash */

/* Power Management Modules */


/* mt25ql low level helpers */

static inline void mt25ql_resetenable(struct mt25ql_dev_s *priv); /* Enables software reset of memory */
static inline void mt25ql_resetflash(struct mt25ql_dev_s *priv); /* Reset the specified flash memory */
static void mt25ql_writeenable(struct mt251ql_dev_s *priv); /* Write enable for flash */
static void mt25ql_writedisable(struct mt25ql_dev_s * priv); /* Write disable for flash */

static inline int mt25ql_readid(struct mt25ql_dev_s *priv);	/* Read device id */


static uint8_t mt25ql_readsr(struct mt25ql_dev_s *priv); /* Read status register */
static uint8_t mt25ql_readsrflag(struct mt25ql_dev_s *priv);	/* Read the flag status register */
static uint16_t mt25ql_readnvconreg(struct mt25ql_dev_s *priv);	/* Reads the non-volatile status register of flash */
static uint8_t mt25ql_readvconreg(struct mt25ql_dev_s *priv); /* Reads volatile status register of the flash */

static uint8_t mt25ql_writesr(struct mt25ql_dev_s *priv,
				FAR const uint8_t val); 	/* write to the status register */
// static uint8_t mt25ql_writenvconreg(struct mt25ql_dev_s *priv,
// 				FAR const uint8_t val);		/* write to non-volatile status register */
// static uint8_t mt25ql_writevconreg(struct mt25ql_dev_s *priv,
// 				FAR const uint8_t val);		/* Write to volatile status register */
// static uint8_t mt25ql_writeenvconreg(struct mt25ql_dev_s *priv,
// 				FAR const uint8_t val);		/* Write to Enahanced status register */
// static uint8_t mt25ql_writeextaddreg(struct mt25ql_dev_s *priv,
// 				FAR const uint8_t val);		/* Write to extended status register*/

static void mt25ql_clearsrflag(struct mt25ql_dev_s *priv); /* Clear the status register flag */

static uint8_t mt25ql_waitbusy(struct mt25ql_dev_s *priv); /* Wait while operation on flash is going on */

#ifdef MT25QL_4_BYTE_MODE
static void mt25ql_4byteaddress(struct mt25ql_dev_s *priv);	/* Change the address mode to 4-byte */
#endif //MT25QL_4_BYTE_MODE

static ssize_t mt25ql_read(struct mt25ql_dev_s *priv,
				off_t offset, size_t bytes,
				FAR uint8_t *buffer);	/* reads data from flash*/
static inline int mt25ql_sectorerase(struct mt25ql_dev_s *priv,
					off_t offset,
					uint8_t type);	/* Erase the sector from the specified offset */
static inline int mt25ql_chiperase(struct mt25ql_dev_s *priv); /* Erase the full flash completely */

static inline void mt25ql_pagewrite(struct mt25ql_dev_s *priv,
					FAR const uint8_t *buffer,
					off_t offset); /* write data to the flash memory */
#ifdef MT25QL_BYTE_WRITE
static void mt25ql_bytewrite(struct mt25ql_dev_s *priv,
				FAR const uint8_t *buffer,
				size_t nbytes, off_t offset); /* Write data to the flash in byte wise mode */
#endif //MT25QL_BYTE_WRITE

/* MTD Driver Methods */

static int mt25ql_erase(FAR struct mtd_dev_s *dev,
			off_t startblock,
			size_t nblocks);
static ssize_t mt25ql_bread(FAR struct mtd_dev_s *dev,
			off_t start_block,
			size_t nblocks,
			FAR const uint8_t *buf);
static ssize_t mt25ql_bwrite(FAR struct mtd_dev_s *dev,
				off_t startblock,
				size_t nblocks,
				FAR uint8_t *buf);
static size_t mt25ql_read(FAR struct mtd_dev_s *dev,
				off_t offset, size_t nbytes,
				FAR uint8_t *buffer);
#ifdef CONFIG_MTD_BYTE_WRITE
static ssize_t mt25ql_write(FAR struct mtd_dev_s *dev,
				off_t offset,
				size_t nbytes,
				FAR const uint8_t *buffer);
#endif //CONFIG_MTD_BYTE_WRITE

static int mt25ql_ioctl(FAR struct mtd_dev_s *dev,
			int cmd, unsigned long arg);


/******************************************************************************************
 * Private Functions
 * ****************************************************************************************
*/
/******************************************************************************************
 * Name: mt25ql_lock
*******************************************************************************************/

static void mt25ql_lock(FAR struct spi_dev_s * dev)
{
	/* On SPI buses where there are multiple devices, it will be necessary to
   	* lock SPI to have exclusive access to the buses for a sequence of
   	* transfers.  The bus should be locked before the chip is selected.
  	 *
	* This is a blocking call and will not return until we have exclusive
	* access to the SPI bus.  We will retain that exclusive access until the
	* bus is unlocked.
	*/

	SPI_LOCK(dev, true);

	/* After locking the SPI bus, the we also need call the setfrequency,
	* setbits, and setmode methods to make sure that the SPI is properly
	* configured for the device.
	* If the SPI bus is being shared, then it may have been left in an
	* incompatible state.
	*/

	SPI_SETMODE(dev, SPIDEV_MODE0);
	SPI_SETBITS(dev, 8);
	SPI_HWFEATURES(dev, 0);
	SPI_SETFREQUENCY(dev, CONFIG_MT25QL_SPIFREQUENCY);
}

/****************************************************************************
 * Name: mt25ql_unlock
 ****************************************************************************/

static inline void mt25ql_unlock(FAR struct spi_dev_s *dev)
{
  SPI_LOCK(dev, false);
}


/****************************************************************************
 * Name: mt25ql_resetenable
 ****************************************************************************/
static inline void mt25ql_resetenable(struct mt25ql_dev_s *priv)
{
	/* Select the flash*/
	SPI_SELECT(priv->dev, SPIDEV_FLASH(0), true);
	nxsig_usleep(20);
	/* Send reset enable command */
	SPI_SEND(priv->dev, MT25QL_RSTEN);
	SPI_SELECT(priv->dev, SPIDEV_FLASH(0), false);
	nxsig_usleep(1000);
	finfo("Reset Enabled\n");
	return;
}


/****************************************************************************
 * Name: mt25ql_resetflash
 ****************************************************************************/
static inline void mt25ql_resetflash(struct mt25ql_dev_s *priv)
{
	/* lock the spi bus */
	mt25ql_lock(priv->dev);
	mt25ql_resetenable(priv); /* enabling software reset of flash */
	SPI_SELECT(priv->dev, SPIDEV_FLASH(0), true);
	SPI_SEND(priv->dev, MT25QL_RSTMEM);
	SPI_SELECT(priv->div, SPIDEV_FLASH(0), false);
	nxsig_usleep(2000);
	mt25ql_unlock(priv->dev);
	finfo("Reset Done\n");
	return;
}


/****************************************************************************
 * Name: mt25ql_writeenable
 ****************************************************************************/
static void mt25ql_writeenable(struct mt25ql_dev_s *priv)
{
	/* spi bus should be locked by caller*/
	SPI_SELECT(priv->dev, SPIDEV_FLASH(0), true);
	SPI_SEND(priv->dev, MT25QL_WRTEN);
	SPI_SELECT(priv->dev, SPIDEV_FLASH(0), false);
	nxsig_usleep(1000);
	finfo("Enabled\n");
}

/****************************************************************************
 * Name: mt25ql_writedisable
 ****************************************************************************/
static void mt25ql_writedisable(struct mt25ql_dev_s *priv)
{
	/* spi bus should be locked by caller*/
	SPI_SELECT(priv->dev, SPIDEV_FLASH(0), true);
	SPI_SEND(priv->dev, MT25QL_WRTDIS);
	SPI_SELECT(priv->dev, SPIDEV_FLASH(0), false);
	nxsig_usleep(1000);
	finfo("Disabled\n");
}

/****************************************************************************
 * Name: mt25ql_rdid
 ****************************************************************************/

static inline int mt25ql_readid(FAR struct mt25ql_dev_s *priv)
{
	uint8_t manufacturer;
	uint8_t memory;
	uint8_t capacity;
	uint8_t devid[20];

	finfo("priv: %p\n", priv);

	/**
	 * configurng the bus, selecting the flash device. (caller needs to have already
	 * locked the bus for exclusive access)
	 *
	*/
	mt25ql_lock(priv->dev);

	SPI_SELECT(priv->dev, SPIDEV_FLASH(0), true);

	/**
	 * send the read id command and read 20 bytes id info from the flash
	*/

	SPI_SEND(priv->dev, MT25QL_RDDEVID);	// sending the command
	SPI_RECVBLOCK(priv->dev, devid, 20);

	/* deselecting the flash */
	SPI_SELECT(priv->dev, SPIDEV_FLASH(0), false);
	mt25ql_unlock(dev);

	finfo("Read ID: ");
	for(int i=0; i<20; i++){
		finfo("%02x ", devid[i]);
	}
	finfo("\n");
	manufacturer = devid[0];
	memory = devid[1];
	capacity = devid[2];

	/* checking for id validation */
	if(manufacturer == MT25QL_MANFACTURER && memory == MT25QL_MEMORY_TYPE)
	{
		/* checknig if the connected falsh memory is valid */

		/* checking for the understandable flash capacity*/
		switch(capacity)
		{
			case MT25QL_CAPACITY_2GB:
			/* save the flash geometry for the 2Gbit flash */
				priv->sectorshift = MT25QL2G_SECTOR_SHIFT;
				priv->pageshift	  = MT25QL2G_PAGE_SHIFT;
				priv->nsectors	  = MT25QL2G_NSECTORS;
				priv->npages	  = MT25QL2G_NPAGES;
				#ifdef CONFIG_MT25QL_SUBSECTOR_ERASE
				priv->subsectorshift = MT25QL2G_SUBSECT_SHIFT;
				#endif //CONFIG_MT25QL_SUBSECTOR_ERASE
				return OK;

			case MT25QL_CAPACITY_1GB:
			/* save the flash geometry for 1Gbit flash */
				priv->sectorshift = MT25QL1G_SECTOR_SHIFT;
				priv->pageshift	  = MT25QL1G_PAGE_SHIFT;
				priv->nsectors	  = MT25QL1G_NSECTORS;
				priv->npages	  = MT25QL1G_NPAGES;
				#ifdef CONFIG_MT25QL_SUBSECTOR_ERASE
				priv->subsectorshift = MT25QL1G_SUBSECT_SHIFT;
				#endif //CONFIG_MT25QL_SUBSECTOR_ERASE
				return OK;
			case MT25QL_CAPACITY_512:
			/* save the flash geometry for 512Mbit flash */
				priv->sectorshift = MT25QL512_SECTOR_SHIFT;
				priv->pageshift	  = MT25QL512_PAGE_SHIFT;
				priv->nsectors	  = MT25QL512_NSECTORS;
				priv->npages	  = MT25QL512_NPAGES;
				#ifdef CONFIG_MT25QL_SUBSECTOR_ERASE
				priv->subsectorshift = MT25QL512_SUBSECT_SHIFT;
				#endif //CONFIG_MT25QL_SUBSECTOR_ERASE
				return OK;
			case MT25QL_CAPACITY_256:
			/* save the flash geometry for 256Mbit flash */
				priv->sectorshift = MT25QL256_SECTOR_SHIFT;
				priv->pageshift	  = MT25QL256_PAGE_SHIFT;
				priv->nsectors	  = MT25QL256_NSECTORS;
				priv->npages	  = MT25QL256_NPAGES;
				#ifdef CONFIG_MT25QL_SUBSECTOR_ERASE
				priv->subsectorshift = MT25QL256_SUBSECT_SHIFT;
				#endif //CONFIG_MT25QL_SUBSECTOR_ERASE
				return OK;
			case MT25QL_CAPACITY_128:
			/* save the flash geometry for 128Mbit flash */
				priv->sectorshift = MT25QL128_SECTOR_SHIFT;
				priv->pageshift	  = MT25QL128_PAGE_SHIFT;
				priv->nsectors	  = MT25QL128_NSECTORS;
				priv->npages	  = MT25QL128_NPAGES;
				#ifdef CONFIG_MT25QL_SUBSECTOR_ERASE
				priv->subsectorshift = MT25QL128_SUBSECT_SHIFT;
				#endif //CONFIG_MT25QL_SUBSECTOR_ERASE
				return OK;
			case MT25QL_CAPACITY_64:
			/* save the flash geometry for 64Mbit flash */
				priv->sectorshift = MT25QL64_SECTOR_SHIFT;
				priv->pageshift	  = MT25QL64_PAGE_SHIFT;
				priv->nsectors	  = MT25QL64_NSECTORS;
				priv->npages	  = MT25QL64_NPAGES;
				#ifdef CONFIG_MT25QL_SUBSECTOR_ERASE
				priv->subsectorshift = MT25QL64_SUBSECT_SHIFT;
				#endif //CONFIG_MT25QL_SUBSECTOR_ERASE
				return OK;
			default:
				return -ENODEV;

		}
	}
	return -ENODEV;
}

/****************************************************************************
 * Name: mt25ql_readsr
 ****************************************************************************/
static uint8_t mt25ql_readsr(struct mt25ql_dev_s *priv)
{
	uint8_t status_byte;
	/* caller should lock the spi bus */
	SPI_SELECT(priv->dev, SPIDEV_FLASH(0), true);
	SPI_SEND(priv->dev, MT25QL_RDSTATREG);
	status_byte = SPI_SEND(priv->dev, MT25QL_DUMMY);
	SPI_SELECT(priv->dev, SPI_DEV_FLASH(0), false);
	finfo("status: %02x", status_byte);
	return status_byte;
}

/****************************************************************************
 * Name: mt25ql_readsrflag
 ****************************************************************************/
static uint8_t mt25ql_readsrflag(struct mt25ql_dev_s *priv)
{
	uint8_t status_flag_byte;
	/* caller needs to lock the spi bus */
	SPI_SELECT(priv->dev, SPIDEV_FLASH(0), true);
	SPI_SEND(priv->dev, MT25QL_RDFLGSTATREG);
	status_flag_byte=SPI_SEND(priv->dev, MT25QL_DUMMY);
	SPI_SELECT(priv->dev, SPIDEV_FLASH(0), false);
	finfo("status flag: %02x\n", status_flag_byte);
	return status_flag_byte;
}

/****************************************************************************
 * Name: mt25ql_readnvconreg
 ****************************************************************************/
static uint8_t mt25ql_readnvconreg(struct mt25ql_dev_s *priv)
{
	uint8_t nvconreg_byte;
	/* caller needs to lock the spi bus */
	SPI_SELECT(priv->dev, SPIDEV_FLASH(0), true);
	SPI_SEND(priv->dev, MT25QL_RDNVOLCONREG);
	nvconreg_byte=SPI_SEND(priv->dev, MT25QL_DUMMY);
	SPI_SELECT(priv->dev, SPIDEV_FLASH(0), false);
	finfo("NV Con reg: %02x\n", nvconreg_byte);
	return nvconreg_byte;
}
/****************************************************************************
 * Name: mt25ql_readvconreg
 ****************************************************************************/
static uint8_t mt25ql_readvconreg(struct mt25ql_dev_s *priv)
{
	uint8_t vconreg_byte;
	/* caller needs to lock the spi bus */
	SPI_SELECT(priv->dev, SPIDEV_FLASH(0), true);
	SPI_SEND(priv->dev, MT25QL_RDVOLCONREG);
	vconreg_byte=SPI_SEND(priv->dev, MT25QL_DUMMY);
	SPI_SELECT(priv->dev, SPIDEV_FLASH(0), false);
	finfo("V Con reg: %02x\n", vconreg_byte);
	return vconreg_byte;
}

/****************************************************************************
 * Name: mt25ql_writesr
 ****************************************************************************/
static uint8_t mt25ql_writesr(struct mt25ql_dev_s *priv,
				FAR const uint8_t val)
{
	/* caller needs to lock the spi bus */
	SPI_SELECT(priv->dev, SPIDEV_FLASH(0), true);
	SPI_SEND(priv->dev, MT25QL_WRRSTATRsEG);
	SPI_SEND(priv->dev, val);
	SPI_SELECT(priv->dev, SPIDEV_FLASH(0), false);
	finfo("Stats reg updated");
	return;
}





#ifdef __cplusplus
	}
#endif
#endif //MT25QL_H_
