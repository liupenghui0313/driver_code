
ifeq ($(PARAM_FILE), )
		PARAM_FILE:=../../Makefile.param
		include $(PARAM_FILE)
endif

obj-m := spiflash.o 
spiflash-y += hi_ssp.o spi_flash.o storage.o

EXTRA_CFLAGS += -DHI3520D

EXTRA_CFLAGS += -Wall -O2 -I$(PWD)/
 
default:	
	@make -C $(LINUX_ROOT) M=$(PWD) modules
	rm *.o modules.* *.symvers *.mod.c
clean:
	@make -C $(LINUX_ROOT) M=$(PWD) clean
