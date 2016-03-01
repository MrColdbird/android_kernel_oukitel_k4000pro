include $(srctree)/drivers/misc/mediatek/Makefile.custom

# Linux driver folder
ccflags-y += -I$(srctree)/drivers/misc/mediatek/mach/$(MTK_PLATFORM)/$(ARCH_MTK_PROJECT)/touchpanel/gsl1688/
ccflags-y += -I$(srctree)/arch/arm/mach-$(MTK_PLATFORM)/$(ARCH_MTK_PROJECT)/touchpanel/gsl1688/
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/gsl1688/
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/

obj-y	+=  gsl_ts_driver.o
