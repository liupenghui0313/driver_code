/*  extdrv/interface/ssp/hi_ssp.c
 *
 * Copyright (c) 2006 Hisilicon Co., Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program;
 *
 * History:
 *      21-April-2006 create this file
 */

#include <linux/module.h>
//#include <linux/config.h>
#include <linux/errno.h>
#include <linux/miscdevice.h>
#include <linux/fcntl.h>
#include <linux/jiffies.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/proc_fs.h>
//#include <linux/workqueue.h>

#include <asm/uaccess.h>
#include <asm/system.h>
#include <asm/io.h>

#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/types.h>
#include <linux/mm.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include "spi_host.h"

#define SSP_NUMS 1

#define ssp_readw(addr,ret)     (ret =(*(volatile unsigned int *)(addr)))
#define ssp_writew(addr,val)    ((*(volatile unsigned int *)(addr)) = (val))

#define HI_REG_READ(addr,ret)   (ret =(*(volatile unsigned int *)(addr)))
#define HI_REG_WRITE(addr,val)  ((*(volatile unsigned int *)(addr)) = (val))

#define IO_ADDRESS_VERIFY(x) (hispi->reg_ssp_base_va + ((x)-(SSP_BASE)))
#define IO_VA(x)             (reg_base_va + (x))
#define IO_ADDRESS_2(x)      (hispi->reg_gpio_cs_va + (x))

#ifdef HI3520D
#pragma message("Building SPI flash driver for HI3520DV200")

#define SSP_CPSDVR      2
#define SSP_BASE        0x200C0000
#define SSP_SIZE        0x10000

#define MUXCTRL_BASE    0x200F0000
#define MUXCTRL_SIZE    0x10000
#define SPI_CLK         IO_VA(0x30)
#define SPI_SDO         IO_VA(0x34)
#define SPI_SDI         IO_VA(0x38)
#define SPI_CS0         IO_VA(0x3C)  //GPIO8_3

#define GPIO_CS_BASE    0x201D0000
#define GPIO_CS_SIZE    0x10000
/* gpio cs reg */
#define GPIO_CS         IO_ADDRESS_2(0x20)
#define GPIO_0_DIR      IO_ADDRESS_2(0x400)
#define SSP_CS          (1 << 3)

#elif defined(HI3521A)
#pragma message("Building SPI flash driver for HI3520DV300 or HI3521A")

#define SSP_CPSDVR      4
#define SSP_BASE        0x120D0000
#define SSP_SIZE        0x10000

#define MUXCTRL_BASE    0x120F0000
#define MUXCTRL_SIZE    0x10000
#define SPI_CLK         IO_VA(0xC4)
#define SPI_SDO         IO_VA(0xC8)
#define SPI_SDI         IO_VA(0xCC)
#define SPI_CS0         IO_VA(0xD0)  //GPIO5_3

#define GPIO_CS_BASE    0x121A0000
#define GPIO_CS_SIZE    0x10000
/* gpio cs reg */
#define GPIO_CS         IO_ADDRESS_2(0x20)
#define GPIO_0_DIR      IO_ADDRESS_2(0x400)
#define SSP_CS          (1 << 3)

#else
#error "Platform not defined: -DHI3520D or -DHI3521A in makefile"
#endif

/* SSP register definition .*/
#define SSP_CR0              IO_ADDRESS_VERIFY(SSP_BASE + 0x00)
#define SSP_CR1              IO_ADDRESS_VERIFY(SSP_BASE + 0x04)
#define SSP_DR               IO_ADDRESS_VERIFY(SSP_BASE + 0x08)
#define SSP_SR               IO_ADDRESS_VERIFY(SSP_BASE + 0x0C)
#define SSP_CPSR             IO_ADDRESS_VERIFY(SSP_BASE + 0x10)
#define SSP_IMSC             IO_ADDRESS_VERIFY(SSP_BASE + 0x14)
#define SSP_RIS              IO_ADDRESS_VERIFY(SSP_BASE + 0x18)
#define SSP_MIS              IO_ADDRESS_VERIFY(SSP_BASE + 0x1C)
#define SSP_ICR              IO_ADDRESS_VERIFY(SSP_BASE + 0x20)
#define SSP_DMACR            IO_ADDRESS_VERIFY(SSP_BASE + 0x24)

#define SSP_USE_GPIO_DO_CS

struct hi_spi_host {
    struct spi_hostdev host;    
    void __iomem *reg_ssp_base_va;
    void __iomem *reg_gpio_cs_va;
};

static struct hi_spi_host spihosts[SSP_NUMS];

#ifdef SSP_USE_GPIO_DO_CS
void gpio_cs_init(struct hi_spi_host *hispi)
{
    unsigned int reg;    
    HI_REG_READ(GPIO_0_DIR, reg);
    reg |= SSP_CS; // output
    HI_REG_WRITE(GPIO_0_DIR, reg);    
}

void gpio_cs_level(struct hi_spi_host *hispi, int high)
{
    unsigned int reg = 0;
    if (high)
        reg = SSP_CS;
    HI_REG_WRITE(GPIO_CS, reg);     
}
#endif

/*
 * enable SSP routine.
 *
 */
static void hi_ssp_enable(struct hi_spi_host *hispi)
{
    unsigned int ret = 0x42;
    ssp_readw(SSP_CR1,ret);
    ret = ret & (~0x34);
    ret = ret | 0x02;
    ssp_writew(SSP_CR1,ret);
}

/*
 * disable SSP routine.
 *
 */
static void hi_ssp_disable(struct hi_spi_host *hispi)
{
    unsigned int ret = 0;
    ssp_readw(SSP_CR1,ret);
    ret = ret & (~(0x1 << 1));
    ssp_writew(SSP_CR1,ret);
}

static void inline hi_ssp_cs(struct hi_spi_host *hispi, int high)
{
    unsigned int ret = 0;
    ssp_readw(SSP_CR1,ret);
    if (high)
        ret = ret & (~(0x1 << 1));
    else
        ret = ret | (0x01 << 1);
    ssp_writew(SSP_CR1,ret);
 #ifdef SSP_USE_GPIO_DO_CS
    gpio_cs_level(hispi, high);
 #endif
}

/*
 * set SSP frame form routine.
 *
 * @param framemode: frame form
 * 00: Motorola SPI frame form.
 * when set the mode,need set SSPCLKOUT phase and SSPCLKOUT voltage level.
 * 01: TI synchronous serial frame form
 * 10: National Microwire frame form
 * 11: reserved
 * @param sphvalue: SSPCLKOUT phase (0/1)
 * @param sp0: SSPCLKOUT voltage level (0/1)
 * @param datavalue: data bit
 * 0000: reserved    0001: reserved    0010: reserved    0011: 4bit data
 * 0100: 5bit data   0101: 6bit data   0110:7bit data    0111: 8bit data
 * 1000: 9bit data   1001: 10bit data  1010:11bit data   1011: 12bit data
 * 1100: 13bit data  1101: 14bit data  1110:15bit data   1111: 16bit data
 *
 * @return value: 0--success; -1--error.
 *
 */

static int hi_ssp_set_frameform(struct hi_spi_host *hispi, unsigned char framemode,
                    unsigned char spo, unsigned char sph, unsigned char datawidth)
{
    unsigned int ret = 0;
    ssp_readw(SSP_CR0,ret);
    //printk("hi_ssp_set_frameform, read CR0=%08X\n", ret);
    if(framemode > 3) {
        printk("set frame parameter err.\n");
        return -1;
    }
    ret = (ret & 0xFFCF) | (framemode << 4);
    if((ret & 0x30) == 0) {
        if(spo > 1) {
            printk("set spo parameter err.\n");
            return -1;
        }
        if(sph > 1) {
            printk("set sph parameter err.\n");
            return -1;
        }
        ret = (ret & 0xFF3F) | (sph << 7) | (spo << 6);
    }
    if((datawidth > 16) || (datawidth < 4)) {
        printk("set datawidth parameter err.\n");
        return -1;
    }
    ret = (ret & 0xFFF0) | (datawidth -1);
    ssp_writew(SSP_CR0,ret);
    //printk("hi_ssp_set_frameform, write CR0=%08X\n", ret);
    return 0;
}

/*
 * set SSP serial clock rate routine.
 *
 * @param scr: scr value.(0-255,usually it is 0)
 * @param cpsdvsr: Clock prescale divisor.(2-254 even)
 *
 * @return value: 0--success; -1--error.
 *
 */

static int hi_ssp_set_serialclock(struct hi_spi_host *hispi, unsigned char scr, unsigned char cpsdvsr)
{
    unsigned int ret = 0;
    ssp_readw(SSP_CR0,ret);
    //printk("hi_ssp_set_serialclock, read CR0=%08X\n", ret);
    ret = (ret & 0xFF) | (scr << 8);
    ssp_writew(SSP_CR0,ret);
    //printk("hi_ssp_set_serialclock, write CR0=%08X\n", ret);
    if((cpsdvsr & 0x1)) {
        printk("set cpsdvsr parameter err.\n");
        return -1;
    }
    //printk("hi_ssp_set_serialclock, write CPSR=%08X\n", cpsdvsr);
    ssp_writew(SSP_CPSR,cpsdvsr);
    return 0;
}

static int hi_ssp_alt_mode_set(struct hi_spi_host *hispi, int enable)
{
    unsigned int ret = 0;    
    ssp_readw(SSP_CR1, ret);
    //printk("hi_ssp_alt_mode_set, read CR1=%08X\n", ret);
    if (enable) {
        ret = ret & (~0x40);
    }
    else {
        ret = (ret & 0xFF) | 0x40;
    }
    ssp_writew(SSP_CR1, ret);
    //printk("hi_ssp_alt_mode_set, write CR1=%08X\n", ret);
    return 0;
}

static void hi_ssp_wait_buf_fifo_ok(struct hi_spi_host *hispi)
{
    unsigned int ret;
    do {
        ssp_readw(SSP_DR, ret);
        ssp_readw(SSP_SR, ret);
        //printk("hi_ssp_wait_buf_fifo_ok, read SR=%08X\n", ret);
    }while((ret & 0x15) != 0x01);
}

static size_t hi_ssp_recv(struct hi_spi_host *hispi, void *buf, size_t count)
{
    unsigned char *p;
    unsigned int ret, dummy = 0xFF;
    unsigned long timeout, read_time;    
    p = (unsigned char*)buf;
    
    timeout = jiffies + msecs_to_jiffies(hispi->host.msecs);
    ssp_writew(SSP_DR, dummy);
    do {
        ssp_readw(SSP_SR, ret);
        //printk("hi_ssp_recv, read SR=%08X\n", ret);
        if (ret & 0x04) {
            ssp_readw(SSP_DR, ret);
            //printk("hi_ssp_recv, read DR=%08X\n", ret);
            *p++ = (unsigned char)ret;
            count--;
            if (count)
                ssp_writew(SSP_DR, dummy);
            timeout = jiffies + msecs_to_jiffies(hispi->host.msecs);     
        }
        read_time = jiffies;
    } while (time_before(read_time, timeout) && count);
    return p - (unsigned char*)buf;
}

static size_t hi_ssp_send(struct hi_spi_host *hispi, const void *buf, size_t count)
{
    unsigned char *p;
    unsigned int ret, recv = count;
    unsigned long timeout, read_time;    
    p = (unsigned char*)buf;
    
    timeout = jiffies + msecs_to_jiffies(hispi->host.msecs);
    do {
        ssp_readw(SSP_SR, ret);
        //printk("hi_ssp_send, SR=%08X\n", ret);
        if ((ret & 0x02) && count) {
            ret = *p++;
            ssp_writew(SSP_DR, ret);
            count--;
            timeout = jiffies + msecs_to_jiffies(hispi->host.msecs);
        } else if(ret & 0x4) {
            //clear recv fifo
            ssp_readw(SSP_DR, ret);
            recv--;
        }
        read_time = jiffies;
    } while (time_before(read_time, timeout) && (count || recv));
    return p - (unsigned char*)buf;
}


static int hi_ssp_init_defcfg(struct hi_spi_host *hispi)
{
    unsigned char spo = 1;
    unsigned char sph = 1;
    unsigned char scr = 1;
    unsigned char cpsdvsr = SSP_CPSDVR;
    printk("hi_ssp_init_defcfg...\n");
    hi_ssp_disable(hispi);
    hi_ssp_set_frameform(hispi, 0, spo, sph, 8);    
    hi_ssp_set_serialclock(hispi, scr, cpsdvsr);    
    // altasens mode
    hi_ssp_alt_mode_set(hispi, 1);
    //close interupt
    ssp_writew(SSP_IMSC, 0x00);
    //close DMA mode
    ssp_writew(SSP_DMACR, 0x00);
    hi_ssp_enable(hispi);
    return 0;
}

static int hi_ssp_select_bus(struct spi_hostdev *spi, unsigned int cs)
{
    return 0;
}

static int hi_ssp_transmit(struct spi_hostdev *spi, const void *cmd, size_t len, void *buf, size_t send, size_t recv)
{
    //unsigned long start = jiffies;
    size_t xmit;
    struct hi_spi_host *hispi = container_of(spi, struct hi_spi_host, host);
    
    hi_ssp_wait_buf_fifo_ok(hispi);
    hi_ssp_cs(hispi, 0);
    xmit = hi_ssp_send(hispi, cmd, len);
    //printk("hi_ssp_transmit, sent0: %02X, %d\n", ((char*)cmd)[0], xmit);
    if (xmit == len) {
        if (send) {
            xmit = hi_ssp_send(hispi, buf, send);
            //printk("hi_ssp_transmit, sent1: %d, %ld\n", xmit, jiffies-start);
        } else if (recv) {
            xmit = hi_ssp_recv(hispi, buf, recv);
            //printk("hi_ssp_transmit, recv: %d, %ld\n", xmit, jiffies-start);
        }
    } else {
        xmit = -ETIMEDOUT;
    }
    hi_ssp_cs(hispi, 1);
    return xmit;
}

static int hi_ssp_set_clock(struct spi_hostdev *spi, unsigned int Hz)
{
    return 0;
}

static int hi_ssp_set_mode(struct spi_hostdev *spi, int spo, int sph)
{
    struct hi_spi_host *hispi = container_of(spi, struct hi_spi_host, host);
    hi_ssp_set_frameform(hispi, 0, spo, sph, 8);
    return 0;
}

static int hi_ssp_wait_ready(struct spi_hostdev *spi, int msecs)
{
    unsigned int ret;
    struct hi_spi_host *hispi = container_of(spi, struct hi_spi_host, host);
    if (msecs < 0) {
         do {
            ssp_readw(SSP_SR, ret);
            if ((ret & 0x11) == 0x01)
                break;
            udelay(5);
        } while (1);
        return 0;
    } else {
        unsigned long timeout, read_time;
        read_time = jiffies;
        timeout = read_time + msecs_to_jiffies(msecs);
        do {
            ssp_readw(SSP_SR, ret);
            if ((ret & 0x11) == 0x01)
                return 0;
            udelay(5);
        }while (time_before(read_time, timeout));
        return -ETIMEDOUT;
    }	
}

static int ssp_io_config(void)
{
    void __iomem *reg_base_va = ioremap_nocache(MUXCTRL_BASE, MUXCTRL_SIZE);
    printk("ssp_io_config...\n");
    if (reg_base_va) {
        HI_REG_WRITE(SPI_CLK, 0x01);
        HI_REG_WRITE(SPI_SDO, 0x01);
        HI_REG_WRITE(SPI_SDI, 0x01);
#ifndef SSP_USE_GPIO_DO_CS
        HI_REG_WRITE(SPI_CS0, 0x01);
#else
        HI_REG_WRITE(SPI_CS0, 0x00);
#endif
        iounmap((void*)reg_base_va);
        return 0;
    } else {
        printk("Kernel: ioremap ssp base failed!\n");
        return -ENOMEM;        
    }
}

int spi_host_init(unsigned int msecs)
{
    int ret;
    struct hi_spi_host *hispi = spihosts;
    
    ret = ssp_io_config();
    if (ret) 
        return ret;        
    hispi->reg_ssp_base_va = ioremap_nocache((unsigned long)SSP_BASE, (unsigned long)SSP_SIZE);
    if (!hispi->reg_ssp_base_va) {
        printk("Kernel: ioremap ssp base failed!\n");
        return -ENOMEM;
    }
#ifdef SSP_USE_GPIO_DO_CS
    hispi->reg_gpio_cs_va = ioremap_nocache((unsigned long)GPIO_CS_BASE, (unsigned long)GPIO_CS_SIZE);
    if (!hispi->reg_gpio_cs_va) {
        printk("Kernel: ioremap gpio base failed!\n");
        iounmap((void*)hispi->reg_ssp_base_va);
        return -ENOMEM;
    }
    gpio_cs_level(hispi, 1);
    gpio_cs_init(hispi);
#endif
    hispi->host.msecs = msecs;
    hispi->host.iftype = SPI_IF_STD;
    hispi->host.csnums = 1;
    ret = hi_ssp_init_defcfg(hispi);
    if (ret) {
        printk("Kernel: init ssp base failed: %d!\n", ret);
        return ret;        
    }
    //map functions
    hispi->host.select_bus = hi_ssp_select_bus;
    hispi->host.transmit = hi_ssp_transmit;
    hispi->host.set_clock = hi_ssp_set_clock;
    hispi->host.set_mode = hi_ssp_set_mode;
    hispi->host.wait_ready = hi_ssp_wait_ready;
    hispi->host.entry_4addr = NULL;
    hispi->host.qe_enable = NULL;
    //register spi host to bus
    ret = spi_host_register(&hispi->host);
    return ret;
}

void spi_host_deinit(struct spi_hostdev *spi)
{
    struct hi_spi_host *hispi = container_of(spi, struct hi_spi_host, host);
    
    //shun down SPI
    hi_ssp_disable(hispi);
#ifdef SSP_USE_GPIO_DO_CS
    gpio_cs_level(hispi, 1);
    iounmap((void*)hispi->reg_gpio_cs_va);
    hispi->reg_gpio_cs_va = NULL;
#endif
    //free iomem resource
    iounmap((void*)hispi->reg_ssp_base_va);
    hispi->reg_ssp_base_va = NULL;
}