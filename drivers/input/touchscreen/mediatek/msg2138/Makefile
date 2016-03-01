include $(srctree)/drivers/misc/mediatek/Makefile.custom

# Linux driver folder
ccflags-y += -I$(srctree)/drivers/misc/mediatek/mach/$(MTK_PLATFORM)/$(ARCH_MTK_PROJECT)/touchpanel/msg2138/
ccflags-y += -I$(srctree)/arch/arm/mach-$(MTK_PLATFORM)/$(ARCH_MTK_PROJECT)/touchpanel/msg2138/
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/msg2138/
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/

obj-y	+=  msg2138.o

