NAM_PROJ = imrsim
KERNVER  = $(shell uname -r)
D_KERNEL_SOURCE = /usr/src/linux-$(KERNVER)

ccflags-y :=-g -I$(PWD) -I$(D_KERNEL_SOURCE)/drivers/md/
obj-m           += dm-$(NAM_PROJ).o

all:
	make -C /lib/modules/$(KERNVER)/build M=$(PWD) modules
clean:
	make -C /lib/modules/$(KERNVER)/build M=$(PWD) clean
modules_install:
	make INSTALL_MOD_DIR=kernel/drivers/md -C /lib/modules/$(KERNVER)/build M=$(PWD) modules_install
