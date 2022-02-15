# Makefile2.6
KERNELDIR := /home/darius/nvidia/Tegra186_Linux_R32.6.1/Linux_for_Tegra/source/public/kernel/kernel-4.9/kernel_out/

ifneq ($(KERNELRELEASE),)
#kbuild syntax. dependency relationshsip of files and target modules are listed here.

mymodule-objs := imx283.o
obj-m := imx283.o   

else
PWD  := $(shell pwd)

KVER ?= $(shell uname -r)

#KDIR目录其实是链接到上面那个Makefile中的那个/usr/src/linux-source-2.6.15/*中
KDIR := /lib/modules/$(KVER)/build 
all:
#KERNELDIR或者KDIR要按照当前的环境进行选择，KERNELDIR是在交叉编译使用，使用过程中要定义ＡＲＣＨ＝arm64，KDIR在目标设备使用。
	$(MAKE) -C $(KERNELDIR) M=$(PWD) modules

clean:
	rm -rf .*.cmd *.o *.mod.c *.ko .tmp_versions

endif
