/*
 * Driver for mb85rc64 FRAM
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
 
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/spi/spi.h>
#include <linux/device.h>
#include <linux/kobject.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/jiffies.h>
#include <linux/delay.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/log2.h>
#include "spi_flash.h"
#include "spi_host.h"

#define DEV_NAME    "dfl1"
#define MAX_FLASHS  5

struct spiflash_device {
    struct mutex lock;
    struct flash_info *flash;
};

static struct spiflash_device dev = {
    .flash      = NULL,
};

/*
 * Specs often allow 5 msec for a page write, sometimes 20 msec;
 * it's important to recover from write timeouts.
 */
static unsigned int oper_timeout = 50;
module_param(oper_timeout, uint, S_IRUGO);
MODULE_PARM_DESC(write_timeout, "Time (in ms) to wait of one operation (default 50)");

/*-------------------------------------------------------------------------*/
static int spiflash_open(struct inode *inode, struct file *filp)
{
    int err = -EBUSY;
    if (unlikely(!dev.flash))
        return err;
    filp->private_data = &dev;
    return 0;
}

static int spiflash_release(struct inode *inode, struct file *filp)
{
    struct spiflash_device *pdev = (struct spiflash_device*)filp->private_data;
    if (pdev) {
        filp->private_data = NULL;
    }
    return 0;
}

static ssize_t spiflash_read(struct file *filp, char *buf, size_t count, 
            loff_t *offset)
{
    ssize_t ret;
    unsigned char *kbuf;
    struct spiflash_device *pdev = (struct spiflash_device*)filp->private_data;
    // printk("read from spiflash: %zu, %zu\n", (size_t)*offset, count);
    if (unlikely(*offset < 0))
        return -EFAULT;
    if (unlikely(!count))
        return 0;
    if (*offset >= pdev->flash->chipsize)
        return 0;
    if (*offset + count > pdev->flash->chipsize)
        count = pdev->flash->chipsize - *offset;

    kbuf = kmalloc(count, GFP_KERNEL);
    if (kbuf) {
        ret = mutex_lock_interruptible(&pdev->lock);
        if (ret == 0) {
            ret = read_spiflash(pdev->flash, kbuf, count, *offset);
            mutex_unlock(&pdev->lock);
            if (ret > 0) {
                ret = ret - copy_to_user(buf, kbuf, ret);
                *offset += ret;
            }
        } else{
            ret = -EINTR;
        }
        kfree(kbuf);
    } else {
        printk("spiflash read malloc error!\n");
        ret = -ENOMEM;
    }
    return ret;
}

static ssize_t spiflash_write(struct file *filp, const char *buf, size_t count,
            loff_t *offset)
{
    ssize_t ret;
    unsigned char *kbuf;
    struct spiflash_device *pdev = (struct spiflash_device*)filp->private_data;
    //printk("write to spi flash: %d, %d\n", (size_t)*offset, count);
    if (unlikely(*offset < 0))
        return -EFAULT;
    if (unlikely(!count))
        return 0;
    if (*offset >= pdev->flash->chipsize)
        return -EFAULT;		   
    if (*offset + count > pdev->flash->chipsize)
        count = pdev->flash->chipsize - *offset;
    
    kbuf = kmalloc(count, GFP_KERNEL);
    if (kbuf) {
        if (copy_from_user(kbuf, buf, count) == 0) {
            ret = mutex_lock_interruptible(&pdev->lock);
            if (ret == 0) {
                ret = write_spiflash(pdev->flash, kbuf, count, *offset);
                if (ret > 0)
                    *offset += ret;
                mutex_unlock(&pdev->lock);
            } else {
                ret = -EINTR;
            }
        } else {
            ret = -EFAULT;
        }
        kfree(kbuf);
    } else {
        printk("spiflash write malloc error!\n");
        ret = -ENOMEM;
    }
    return ret;
}

static loff_t spiflash_llseek(struct file *filp, loff_t offset, int whence)
{
    loff_t new_offset = -EINVAL;
    struct spiflash_device *pdev = (struct spiflash_device*)filp->private_data;
    switch(whence) {
    case 0: //SEEK_SET
        new_offset = offset;
        break;        
    case 1: //SEEK_CUR
        new_offset = filp->f_pos + offset;
        break;        
    case 2: //SEEK_END
        new_offset = pdev->flash->chipsize + offset;
        break;
    };
    if (new_offset < 0)
        return -EINVAL;
    if (new_offset < pdev->flash->chipsize)
        filp->f_pos = new_offset;
    else 
        filp->f_pos = new_offset - pdev->flash->chipsize;
    return new_offset;
}

static long spiflash_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    return -ENOTTY;
}

static struct file_operations spiflash_fops = {
    .owner  = THIS_MODULE,
    .read   = spiflash_read,
    .write  = spiflash_write,
    .open   = spiflash_open,
    .release = spiflash_release,
    .llseek = spiflash_llseek,
    .unlocked_ioctl = spiflash_ioctl,
};
/*-------------------------------------------------------------------------*/
static struct miscdevice spiflash_miscdev = {
    .minor  = MISC_DYNAMIC_MINOR,
    .name   = DEV_NAME,
    .fops   = &spiflash_fops,
};
/*-------------------------------------------------------------------------*/
static int spiflash_probe(struct spi_device *spi)
{
    mutex_init(&dev.lock);
    //if (dev.flash == NULL) 
    {
        dev.flash = detect_jedec_spiflash(spi);
        if (dev.flash)
            misc_register(&spiflash_miscdev);
    }
    return dev.flash ? 0 : -1;
}

static int spiflash_remove(struct spi_device *spi)
{
    if (spi) {
        misc_deregister(&spiflash_miscdev);
        free_spiflash(dev.flash);
        dev.flash = NULL;
    }
    return 0;
}

static const struct of_device_id spi_w25flash_of_match[]= {
    {.compatible = "winbond,w25q32", },
    { }
};


static struct spi_driver spi_w25flash_driver = {
    .driver = {
        .name           = "w25q32",
        .of_match_table = spi_w25flash_of_match,
    },
    .probe  = spiflash_probe,
    .remove = spiflash_remove,
};

/**
 * module_spi_driver() - Helper macro for registering a SPI driver
 * @__spi_driver: spi_driver struct
 *
 * Helper macro for SPI drivers which do not do anything special in module
 * init/exit. This eliminates a lot of boilerplate. Each module may only
 * use this macro once, and calling it replaces module_init() and module_exit()
 */

module_spi_driver(spi_w25flash_driver);

MODULE_DESCRIPTION("Driver for SPI FLASH");
MODULE_LICENSE("GPL");