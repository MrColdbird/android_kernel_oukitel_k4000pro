include $(srctree)/drivers/misc/mediatek/Makefile.custom

# Linux driver folder
# ccflags-y += -I$(srctree)/drivers/misc/mediatek/mach/$(MTK_PLATFORM)/$(ARCH_MTK_PROJECT)/touchpanel/ft_allinone/
# ccflags-y += -I$(srctree)/arch/arm/mach-$(MTK_PLATFORM)/$(ARCH_MTK_PROJECT)/touchpanel/ft_allinone/
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/ili2115/
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/

obj-y	+=  ili2115_driver_funtion1.o
obj-y +=  gesture_function1.o
$(obj)/gesture_function1.o: $(srctree)/$(obj)/gesture_function1.a
	cp $(srctree)/$(obj)/gesture_function1.a  $(obj)/gesture_function1.o
#obj-y	+=  gesture_function1.a
# obj-y   +=  gesture_function1.a
