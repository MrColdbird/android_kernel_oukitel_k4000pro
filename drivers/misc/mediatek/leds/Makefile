include $(srctree)/drivers/misc/mediatek/Makefile.custom

obj-$(CONFIG_MTK_LEDS) += leds_drv.o

ifeq ($(CONFIG_HCT_LED_SGM3785),y)
obj-y += sgm3785_drv.o
endif

obj-y += $(subst ",,$(CONFIG_MTK_PLATFORM))/

