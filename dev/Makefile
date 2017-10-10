ifneq ($(KERNELRELEASE),)
obj-m := ltc6802.o
else
KDIR := $(HOME)/linux-kernel-labs/src/linux
all:
	$(MAKE) -C $(KDIR) M=$$PWD
endif
