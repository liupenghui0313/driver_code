/******************************************************************************
*    Copyright (c) 2009-2010 by czy.
*    All rights reserved.
* ***
*
******************************************************************************/

#ifndef SPI_FLASH_H_
#define SPI_FLASH_H_

/*****************************************************************************/

#define _1K		(0x400)
#define _2K		(0x800)

#define _4K		(0x1000)
#define _8K		(0x2000)
#define _16K		(0x4000)
#define _32K		(0x8000)

#define _64K		(0x10000)
#define _128K		(0x20000)
#define _256K		(0x40000)
#define _512K		(0x80000)

#define _1M		(0x100000)
#define _2M		(0x200000)
#define _4M		(0x400000)
#define _8M		(0x800000)

#define _16M		(0x1000000)
#define _32M		(0x2000000)
#define _64M		(0x4000000)

#define INFINITE	(0xFFFFFFFF)
/*****************************************************************************/

#define SPI_IF_READ_STD			(0x01)
#define SPI_IF_READ_FAST		(0x02)
#define SPI_IF_READ_DUAL		(0x04)
#define SPI_IF_READ_DUAL_ADDR		(0x08)
#define SPI_IF_READ_QUAD		(0x10)
#define SPI_IF_READ_QUAD_ADDR		(0x20)

#define SPI_IF_WRITE_STD		(0x01)
#define SPI_IF_WRITE_DUAL		(0x02)
#define SPI_IF_WRITE_DUAL_ADDR		(0x04)
#define SPI_IF_WRITE_QUAD		(0x08)
#define SPI_IF_WRITE_QUAD_ADDR		(0x10)

#define SPI_IF_ERASE_SECTOR		(0x01)	/* sector erase, 64K */
#define SPI_IF_ERASE_CHIP		(0x02)	/* chip erase */
#define SPI_IF_ERASE_4K			(0x04)	/* 4K */
#define SPI_IF_ERASE_8K			(0x08)	/* 8K */

#define SPI_IF_ERASE_SECTOR_4K		(0x01)	/* 4K */
#define SPI_IF_ERASE_SECTOR_32K		(0x02)	/* 32K */
#define SPI_IF_ERASE_SECTOR_64K		(0x04)	/* 64K */
#define SPI_IF_ERASE_SECTOR_256K	(0x08)	/* 256K */
/*****************************************************************************/
#define SPI_CMD_BRWR           (0x17)  /*write value to BAR*/
#define SPI_EN4B_VALUE         (0x80)  /*the enable 4Byte addr len value*/
#define SPI_EX4B_VALUE         (0x00)  /*the disable 4Byte addr len value*/
#define SPI_4BYTE_ADDR_LEN     (4)     /*address len 4Byte*/
/*****************************************************************************/

#define SPI_CMD_WREN			0x06	/* Write Enable */
/*****************************************************************************/
#define SPI_CMD_SE_4K			0x20	/* 4KB sector Erase */
#define SPI_CMD_SE_32K			0x52	/* 32KB sector Erase */

#define SPI_CMD_SE_64K			0xD8	/* 64KB sector Erase */
#define SPI_CMD_SE_256K			0xD8	/* 256KB sector Erase */

#define SPI_CMD_SE			0xD8	/* 64KB Sector Erase */
#define SPI_CMD_BE			0xC7	/* chip erase */
/*****************************************************************************/
#define SPI_CMD_WRSR			0x01	/* Write Status Register */
#define SPI_CMD_RDSR2			0x35	/* Read Status Register-2 */

#define SPI_CMD_RDSR			0x05	/* Read Status Register */
#define SPI_CMD_RDID			0x9F	/* Read Identification */
/*****************************************************************************/
#define SPI_CMD_PP			0x02	/* Page Programming */
#define SPI_CMD_WRITE_DUAL		0xA2	/* fast program dual input */
#define SPI_CMD_WRITE_QUAD		0x32	/* fast program quad input */
#define SPI_CMD_WRITE_DUAL_ADDR		0xD2	/* Dual I/O High Performance
							Write */
#define SPI_CMD_WRITE_QUAD_ADDR		0x12	/* Quad I/O High Performance
							Write */
/*****************************************************************************/
#define SPI_CMD_READ			0x03	/* Read Data bytes */
#define SPI_CMD_FAST_READ		0x0B	/* Read Data Bytes at Higher
							Speed */
#define SPI_CMD_READ_DUAL		0x3B	/* fast read dual output */
#define SPI_CMD_READ_QUAD		0x6B	/* fast read quad output */
#define SPI_CMD_READ_DUAL_ADDR		0xBB	/* Dual I/O High Performance
							Read */
#define SPI_CMD_READ_QUAD_ADDR		0xEB	/* Quad I/O High Performance
							Read */
/*****************************************************************************/
#define SPI_CMD_SR_WIP			1	/* Write in Progress */
#define SPI_CMD_SR_WEL			2	/* Write Enable Latch */

#define SPI_CMD_SR_QE			(0x1<<9)	/* quad enable */
#define SPI_CMD_SR_XQE			(0x0<<9)	/* quad disable */
/*****************************************************************************/
#define SPI_CMD_EN4B			0xB7	/* enter to 4 bytes mode and
							set 4 byte bit as '1' */
#define SPI_CMD_EX4B			0xE9	/* exit 4 bytes mode and
						clear 4 byte bit as '0' */

/*****************************************************************************/
#define OPER_READ  0
#define OPER_WRITE 1
#define OPER_ERASE 2


struct spi_operation {
    unsigned char	cmd;
    unsigned char	dummy;
    unsigned short msecs;//operation's time in milisecond
    unsigned int	freq;   //clock frequency in Hz
};

struct flash_info {
    struct spi_device *spi;
    unsigned int cs;
    char *name;    
    unsigned int	id;    
    unsigned int  addrcached;//address of sector cached;
    unsigned char *bufcached;//sector cache for writing
    unsigned int pagesize;
    unsigned int sectorsize;
    unsigned int sectornums;
    unsigned int	chipsize;
    unsigned int	addrcycle;
    
    struct spi_operation opers[3]; //0-read, 1-write, 2-erase
};

struct flash_info* detect_jedec_spiflash(struct spi_device *spi);
void free_spiflash(struct flash_info*);

ssize_t read_spiflash(struct flash_info *flash, 
            char *buf, size_t count, unsigned int address);
ssize_t write_spiflash(struct flash_info *flash, 
            const char *buf, size_t count, unsigned int address);
/******************************************************************************/
#endif /* SPI_FLASH */
