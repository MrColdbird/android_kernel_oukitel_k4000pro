include $(srctree)/drivers/misc/mediatek/Makefile.custom


ifeq ($(CONFIG_FINGERPRINT_ET300),y)
obj-y	+=  et300/
endif

ifeq ($(CONFIG_FINGERPRINT_SILEADE),y)
obj-y	+=  sileade_sensor/
endif

ifeq ($(CONFIG_FINGERPRINT_NEWSCHIP),y)
obj-y	+= newschip_sensor/
endif

ifeq ($(CONFIG_FINGERPRINT_JMT103),y)
obj-y	+= jmt103/
endif
