/*
 * Driver for mb85rc64 FRAM
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
 
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/jiffies.h>
#include <linux/delay.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/miscdevice.h>
#include <linux/mutex.h>
#include <linux/log2.h>
#include "spi_flash.h"
#include "spi_host.h"

#define JEDEC_WINBOND   0xEF
#define JEDEC_W25Q32BV  0xEF4016
#define JEDEC_W25Q64FV  0xEF4017
#define JEDEC_W25Q128FV 0xEF4018

static int get_flash_status(struct spi_hostdev *spi)
{
    unsigned char recv[1];
    char cmd[1] = {SPI_CMD_RDSR};
    int ret = spi->transmit(spi, cmd, sizeof(cmd), recv, 0, sizeof(recv));
    if (ret >= 1) {
        ret = (recv[0] & 0x00FF);
    }
    else
        ret = -1;
    return ret;
}

static int write_flash_enable(struct spi_hostdev *spi)
{
    char cmd[1] = {SPI_CMD_WREN};
    //printk("write_flash_enable...\n");
    spi->transmit(spi, cmd, sizeof(cmd), NULL, 0, 0);
    return 0;
}

static int wait_flash_idle(struct flash_info *flash, unsigned int msecs)
{
    unsigned long timeout, read_time;
    struct spi_hostdev *spi = flash->spi;
    
    read_time = jiffies;
    timeout = read_time + msecs_to_jiffies(msecs);
    if (read_time == timeout)
        timeout = read_time + 1;
    do {
        if ((get_flash_status(spi) & 0x01) == 0)
            return 0;
    }while (time_before(read_time, timeout));
    return -ETIMEDOUT;
}

static size_t prepare_command(struct flash_info *flash, unsigned char *cmd, 
                                unsigned int address, unsigned int type)
{    
    struct spi_operation* oper = &flash->opers[type];
    cmd[0] = oper->cmd;
    if (flash->addrcycle == 4) {
        cmd[1] = (unsigned char)(address >> 24);
        cmd[2] = (unsigned char)(address >> 16);
        cmd[3] = (unsigned char)(address >> 8);
        cmd[4] = (unsigned char)(address);
        type = 5;
    } else {
        cmd[1] = (unsigned char)(address >> 16);
        cmd[2] = (unsigned char)(address >> 8);
        cmd[3] = (unsigned char)(address);
        type = 4;
    }
    return type + oper->dummy;
}

static int read_flash(struct flash_info *flash, unsigned int address, char *buf, size_t count)
{
    unsigned char cmd[16];
    size_t cmdlen = prepare_command(flash, cmd, address, OPER_READ);
    return flash->spi->transmit(flash->spi, cmd, cmdlen, buf, 0, count);
}

static int write_page(struct flash_info *flash, unsigned int address, char *buf, size_t count)
{
    unsigned char cmd[16];
    size_t cmdlen = address & ((flash->pagesize-1));
    if (cmdlen + count > flash->pagesize)
        count = flash->pagesize - cmdlen;
    for(cmdlen=0; cmdlen<count; cmdlen++) {
        if (buf[cmdlen] != 0xFF) {
            int ret;
            //printk("write_page...\n");
            write_flash_enable(flash->spi);
            cmdlen = prepare_command(flash, cmd, address, OPER_WRITE);
            ret = flash->spi->transmit(flash->spi, cmd, cmdlen, buf, count, 0);
            wait_flash_idle(flash, flash->opers[OPER_ERASE].msecs);            
            return ret;
        }
    }
    return count;
}

static int erase_sector(struct flash_info *flash, unsigned int address)
{
    unsigned char cmd[16];
    size_t cmdlen = prepare_command(flash, cmd, address & (~(flash->sectorsize-1)), OPER_ERASE);
    //printk("erase sector...\n");
    write_flash_enable(flash->spi);
    flash->spi->transmit(flash->spi, cmd, cmdlen, NULL, 0, 0);
    wait_flash_idle(flash, flash->opers[OPER_ERASE].msecs);
    return 0;
}

static int write_sector(struct flash_info *flash, unsigned int address, const char *buf, size_t count)
{
    int ret;
    unsigned int i, addrsector, need_erase = 0;
    unsigned int offset = address & ((flash->sectorsize-1));
    
    //printk("write sector: %08X, %d\n", address, count);
    
    if (count > flash->sectorsize - offset)        
        count = flash->sectorsize - offset; 
    if (memcmp(&flash->bufcached[offset], buf, count)) {
        //check if need do erasing
        addrsector = offset + count;
        for(i=offset; i<addrsector; i++) {
            if (flash->bufcached[i] != 0xFF && 
                flash->bufcached[i] != buf[i-offset]) {
                printk("write sector need erase...\n");
                need_erase = 1;
                break;
            }
        }
        memcpy(&flash->bufcached[offset], buf, count);
        addrsector = flash->addrcached;
        if (need_erase) {            
            erase_sector(flash, addrsector);
            for (i=addrsector; i<addrsector+flash->sectorsize; i += flash->pagesize) {
                write_page(flash, i, &flash->bufcached[i-addrsector], flash->pagesize);
            }
            ret = count;
        } else {
            for (i=address; count;) {
                unsigned int wlen = flash->pagesize - (i&(flash->pagesize-1));
                if (count < wlen)
                    wlen = count;                
                write_page(flash, i, &flash->bufcached[i-addrsector], wlen);
                ret += wlen;
                i += wlen;
                count -= wlen;
            }
        }
    } else {
        ret = count;
    }
    return ret;
}

static int inline address_is_cached(struct flash_info *flash, unsigned int address)
{
    if (address >= flash->addrcached && address < flash->addrcached + flash->sectorsize)
        return 1;
    return 0;
}
//=========================================================================================
static int wait_buf_idle(struct flash_info *flash, int msecs)
{
    struct spi_hostdev *spi = flash->spi;
    int ret = spi->wait_ready(spi, msecs);
    if (ret == 0)
        spi->select_bus(spi, flash->cs);
    return ret;
}
//=========================================================================================
struct flash_info* detect_jedec_spiflash(struct spi_hostdev *spi, unsigned int cs)
{
    int ret;
    unsigned char buf[3];
    char cmd[] = {SPI_CMD_RDID};
    struct flash_info *flash = NULL;
    printk("detect_jedec_spiflash...\n");
    spi->select_bus(spi, cs);
    ret = spi->transmit(spi, cmd, sizeof(cmd), buf, 0, sizeof(buf));
    if (ret == 3) {
        ret = buf[0]<<16|buf[1]<<8|buf[2];
        printk("found flash: %02X, %02X%02X\n", buf[0],buf[1],buf[2]);
        if (ret == JEDEC_W25Q32BV) {
            flash = kzalloc(sizeof(struct flash_info), GFP_KERNEL);
            if (flash) {
                flash->spi = spi;
                flash->cs = cs;
                flash->name = "W25Q32BV"; 
                flash->id = ret;
                flash->pagesize = 256;
                flash->sectorsize = 4096;
                flash->sectornums = 1024;
                flash->chipsize = 4096*1024;
                flash->addrcycle = 3;
                flash->addrcached = INFINITE;
                flash->bufcached = kmalloc(flash->sectorsize, GFP_KERNEL);
                flash->opers[OPER_READ].cmd = SPI_CMD_FAST_READ;
                flash->opers[OPER_READ].dummy = 1;
                flash->opers[OPER_READ].msecs = 0;
                flash->opers[OPER_READ].freq = 104*1000*1000;
                
                flash->opers[OPER_WRITE].cmd = SPI_CMD_PP;
                flash->opers[OPER_WRITE].dummy = 0;
                flash->opers[OPER_WRITE].msecs = 3;
                flash->opers[OPER_WRITE].freq = 104*1000*1000;
                
                flash->opers[OPER_ERASE].cmd = SPI_CMD_SE_4K;
                flash->opers[OPER_ERASE].dummy = 0;
                flash->opers[OPER_ERASE].msecs = 400;
                flash->opers[OPER_ERASE].freq = 104*1000*1000;
                
                if (flash->bufcached == NULL) {
                    kfree(flash);
                    flash = NULL;
                }
            }
        } else if (ret == JEDEC_W25Q64FV) {
            flash = kzalloc(sizeof(struct flash_info), GFP_KERNEL);
            if (flash) {
                flash->spi = spi;
                flash->cs = cs;
                flash->name = "W25Q64FV"; 
                flash->id = ret;
                flash->pagesize = 256;
                flash->sectorsize = 4096;
                flash->sectornums = 2048;
                flash->chipsize = 4096*2049;
                flash->addrcycle = 3;
                flash->addrcached = INFINITE;
                flash->bufcached = kmalloc(flash->sectorsize, GFP_KERNEL);
                flash->opers[OPER_READ].cmd = SPI_CMD_FAST_READ;
                flash->opers[OPER_READ].dummy = 1;
                flash->opers[OPER_READ].msecs = 0;
                flash->opers[OPER_READ].freq = 104*1000*1000;
                
                flash->opers[OPER_WRITE].cmd = SPI_CMD_PP;
                flash->opers[OPER_WRITE].dummy = 0;
                flash->opers[OPER_WRITE].msecs = 3;
                flash->opers[OPER_WRITE].freq = 104*1000*1000;
                
                flash->opers[OPER_ERASE].cmd = SPI_CMD_SE_4K;
                flash->opers[OPER_ERASE].dummy = 0;
                flash->opers[OPER_ERASE].msecs = 400;
                flash->opers[OPER_ERASE].freq = 104*1000*1000;
                
                if (flash->bufcached == NULL) {
                    kfree(flash);
                    flash = NULL;
                }
            }
        } else if (ret == JEDEC_W25Q128FV) {
            flash = kzalloc(sizeof(struct flash_info), GFP_KERNEL);
            if (flash) {
                flash->spi = spi;
                flash->cs = cs;
                flash->name = "W25Q128FV";
                flash->id = ret;
                flash->pagesize = 256;
                flash->sectorsize = 4096;
                flash->sectornums = 4096;
                flash->chipsize = 4096*4096;
                flash->addrcycle = 3;
                flash->addrcached = INFINITE;
                flash->bufcached = kmalloc(flash->sectorsize, GFP_KERNEL);
                flash->opers[OPER_READ].cmd = SPI_CMD_FAST_READ;
                flash->opers[OPER_READ].dummy = 1;
                flash->opers[OPER_READ].msecs = 0;
                flash->opers[OPER_READ].freq = 104*1000*1000;
                
                flash->opers[OPER_WRITE].cmd = SPI_CMD_PP;
                flash->opers[OPER_WRITE].dummy = 0;
                flash->opers[OPER_WRITE].msecs = 3;
                flash->opers[OPER_WRITE].freq = 104*1000*1000;
                
                flash->opers[OPER_ERASE].cmd = SPI_CMD_SE_4K;
                flash->opers[OPER_ERASE].dummy = 0;
                flash->opers[OPER_ERASE].msecs = 400;
                flash->opers[OPER_ERASE].freq = 104*1000*1000;
                
                if (flash->bufcached == NULL) {
                    kfree(flash);
                    flash = NULL;
                }
            }
        }
    }
    return flash;
}

void free_spiflash(struct flash_info* flash)
{
    if (flash) {
        struct spi_hostdev *spi = flash->spi;
        if (flash->bufcached)
            kfree(flash->bufcached);
        kfree(flash);
        spi_host_deinit(spi);
    }
}

ssize_t read_spiflash(struct flash_info *flash, 
            char *buf, size_t count, unsigned int address)
{
    ssize_t readed = 0;
    //char *buf1 = buf;
    //try to read from cached buffer
    if (address_is_cached(flash, address)) {
        unsigned int offset = address & (flash->sectorsize-1);            
        unsigned int cplen = flash->sectorsize - offset;
        cplen = cplen < count ? cplen : count;
        memcpy(buf, flash->bufcached+offset, cplen);
        readed += cplen;
        count -= cplen;
        buf += cplen;
        address += cplen;
    }
    //read from spi flash
    if (count) {
        ssize_t ret = wait_buf_idle(flash, 50);
        if (ret == 0) {
            ret = wait_flash_idle(flash, 50);
            if (ret == 0) {
                ret = read_flash(flash, address, buf, count);
                if (ret > 0)
                    readed += ret;
            }
        }
        if (readed == 0)
            readed = ret; //return error code
    }
/*    for (count=0; count<readed; count+=16) {
        printk("%08X:%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x\n", 
            count, 
            buf1[count+0],buf1[count+1],buf1[count+2],buf1[count+3],
            buf1[count+4],buf1[count+5],buf1[count+6],buf1[count+7],
            buf1[count+8],buf1[count+9],buf1[count+10],buf1[count+11],
            buf1[count+12],buf1[count+13],buf1[count+14],buf1[count+15]);    
    }*/
    return readed;    
}

ssize_t write_spiflash(struct flash_info *flash, 
            const char *buf, size_t count, unsigned int address)
{
    unsigned int addrsector;
    ssize_t ret, written = wait_buf_idle(flash, 50);
    if (written) 
        return written;
    
    while (count) {
        //first, cache sector;
        if (!address_is_cached(flash, address)) {
            //printk("caching spi flash: %08X\n", address);
            addrsector = address & (~(flash->sectorsize-1));
            ret = read_flash(flash, addrsector, flash->bufcached, flash->sectorsize);
            if (ret != flash->sectorsize) {
                flash->addrcached = INFINITE;
                printk("caching spi flash failed\n");
                break;
            }
            flash->addrcached = addrsector;
        }
        //second, write sector
        addrsector = flash->sectorsize - (address & (flash->sectorsize-1));
        if (addrsector > count)
            addrsector = count;
        ret = write_sector(flash, address, buf, addrsector);
        if (ret < 0 || ret != addrsector)
            break;
        written += addrsector;
        buf += addrsector;
        address += addrsector;
        count -= addrsector;
    }   
    return written;
}