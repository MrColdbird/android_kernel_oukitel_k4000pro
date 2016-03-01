#ifndef __SPI_SENSOR_HUB_H__
#   define __SPI_SENSOR_HUB_H__

#   define SPI_SH_DEV_NAME "spi_sh"

#define IMPLEMENT_SPIDEV_INTDETECT 1

enum SUPPORTED_PLATFORMS {
	SUPPORTED_PLATFORMS_VCD,
};

struct spi_sh_platform_data {
	int hostInterruptGpio;		/* GPIO used by sensor hub wakeup */
	int fpgaCresetGpio;		    /* GPIO used by FPGA CRESET */
	int fpgaSsGpio;		        /* GPIO used by FPGA SS */
	enum SUPPORTED_PLATFORMS activePlatform;
	
#   ifdef CONFIG_SENSORS_SPI_SH_SPI_BUS
	/*
	 * delay in usec between SPI commands to allow command processing
	 * time to the slave 
	 */
	unsigned long interCommandDelayUsecs;
#   endif
};


#define SPI_IOC_WR_VD_WORK_MODE		_IOW(SPI_IOC_MAGIC, 99, __u32)
#define SPI_IOC_RD_VD_WORK_MODE		_IOR(SPI_IOC_MAGIC, 99, __u32)


#endif /* __SPI_SENSOR_HUB_H__ */
