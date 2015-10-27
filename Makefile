LINUX_ROOT?=/home/yujingtao/Hi3516A_SDK_V1.0.4.0/linux-3.4.y
CC=arm-hisiv300-linux-gcc

obj-m := nfc.o

EXTRA_CFLAGS += 

default:	
	$(CC) -g -Wall -o nfc_test nfc_test.c
	@$(MAKE) ARCH=arm CROSS_COMPILE=arm-hisiv300-linux- -C $(LINUX_ROOT) M=$(PWD) modules 
	@rm -f *.o modules.* *.symvers *.mod.c

clean:
	@make -C $(LINUX_ROOT) M=$(PWD) clean 
	@rm -f nfc_test *.bak *.o *.ko *.cmd

