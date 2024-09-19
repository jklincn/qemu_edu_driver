obj-m	:= qemu_edu_driver.o
KERNELDIR ?= /lib/modules/$(shell uname -r)/build
CFLAGS=-Wall

modules:
	make -C $(KERNELDIR) M=$(PWD) modules

test:
	gcc user_test.c -o user_test
	sudo ./user_test

clean:
	make -C $(KERNELDIR) M=$(PWD) clean

.PHONY: modules test clean