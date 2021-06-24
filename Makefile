TARGET=rtc-rx8804.ko

obj-m := rtc-rx8804.o
module-objs := rtc-rx8804.o

KDIR :=../linux/
INCLUDE += -I./ 
PWD := $(shell pwd)

default:
	CROSS_COMPILE=arm-linux-gnueabihf- ARCH=arm $(MAKE) -C $(KDIR) SUBDIRS=$(PWD) modules
clean:
	rm -rf *.o 
	rm -rf *.ko 
	rm -rf *.mod.* 
	rm -rf .*.cmd
	rm -rf *.order
	rm -rf *.symvers 
	rm -rf .tmp_versions
distclean: clean
