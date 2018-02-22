/*
	PULUROBOT RN1-HOST Computer-on-RobotBoard main software

	(c) 2017-2018 Pulu Robotics and other contributors
	Maintainer: Antti Alhonen <antti.alhonen@iki.fi>

	This program is free software; you can redistribute it and/or modify
	it under the terms of the GNU General Public License version 2, as 
	published by the Free Software Foundation.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	GNU General Public License version 2 is supplied in file LICENSING.



	Driver for SPI-connected PULUTOF 3D Time-of-Flight add-on

	For Raspberry Pi 3, make sure that:
	dtparam=spi=on     is in /boot/config.txt uncommented
	/dev/spidev0.0 should exist

	If needed:
	/boot/cmdline.txt:  spidev.bufsiz=xxxxx


*/

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>

#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <unistd.h>


#define PULUTOF_SPI_DEVICE "/dev/spidev0.0"


static int spi_fd;

static const unsigned char spi_mode = SPI_MODE_0;
static const unsigned char spi_bits_per_word = 8;
static unsigned int spi_speed = 1000000; // Hz

static int init_spi()
{
	spi_fd = open(PULUTOF_SPI_DEVICE, O_RDWR);

	if(spi_fd < 0)
	{
		printf("ERROR: Opening PULUTOF SPI device %s failed: %d (%s).\n", PULUTOF_SPI_DEVICE, errno, strerror(errno));
		return -1;
	}

	/*
		SPI_MODE_0 CPOL = 0, CPHA = 0, Clock idle low, data is clocked in on rising edge, output data (change) on falling edge
		SPI_MODE_1 CPOL = 0, CPHA = 1, Clock idle low, data is clocked in on falling edge, output data (change) on rising edge
		SPI_MODE_2 CPOL = 1, CPHA = 0, Clock idle high, data is clocked in on falling edge, output data (change) on rising edge
		SPI_MODE_3 CPOL = 1, CPHA = 1, Clock idle high, data is clocked in on rising, edge output data (change) on falling edge
	*/

	/*
		Several code examples, (for example, http://www.raspberry-projects.com/pi/programming-in-c/spi/using-the-spi-interface),
		have totally misunderstood what SPI_IOC_WR_* and SPI_IOC_RD_* mean. They are not different settings for SPI RX/TX respectively
		(that wouldn't make any sense: SPI by definition has synchronous RX and TX so clearly the settings are always the same). Instead,
		SPI_IOC_WR_* and SPI_IOC_RD_* write and read the driver settings, respectively, as explained in spidev documentation:
		https://www.kernel.org/doc/Documentation/spi/spidev

		Here, we just set what we need.
	*/

	if(ioctl(spi_fd, SPI_IOC_WR_MODE, &spi_mode) < 0)
	{
		printf("ERROR: Opening PULUTOF SPI devide: ioctl SPI_IOC_WR_MODE failed: %d (%s).\n", errno, strerror(errno));
		return -2;
	}

	if(ioctl(spi_fd, SPI_IOC_WR_BITS_PER_WORD, &spi_bits_per_word) < 0)
	{
		printf("ERROR: Opening PULUTOF SPI devide: ioctl SPI_IOC_WR_BITS_PER_WORD failed: %d (%s).\n", errno, strerror(errno));
		return -2;
	}

	if(ioctl(spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &spi_speed) < 0)
	{
		printf("ERROR: Opening PULUTOF SPI devide: ioctl SPI_IOC_WR_MAX_SPEED_HZ failed: %d (%s).\n", errno, strerror(errno));
		return -2;
	}

	return 0;
}

/*
Alternative:
static int init_spi()
{
	
	if(!bcm2835_init() || !bcm2835_spi_begin())
	{
		printf("ERROR: Starting PULUTOF SPI device failed. If not running as root, try that.\n");
		return -1;
	}

	bcm2835_spi_setBitOrder(BCM2835_SPI_BIT_ORDER_MSBFIRST);
	bcm2835_spi_setDataMode(BCM2835_SPI_MODE0);
	bcm2835_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_65536);
	bcm2835_spi_chipSelect(BCM2835_SPI_CS0);
	bcm2835_spi_setChipSelectPolarity(BCM2835_SPI_CS0, LOW);

	return 0;
}
*/

static int deinit_spi()
{
	if(close(spi_fd) < 0)
	{
		printf("WARNING: Closing PULUTOF SPI devide failed: %d (%s).\n", errno, strerror(errno));
		return -1;
	}

	return 0;
}


static uint8_t txbuf[20000];
static uint8_t rxbuf[20000];
static uint8_t expected_rxbuf[20000];

int main(int argc, char** argv)
{


	int len = -1;
	int speed = -1;
	if(argc >= 2)
		len = atoi(argv[1]);

	if(argc >= 3)
		speed = atoi(argv[2]);

	if(len < 0 || len > 10000)
	{
		len=1024;
		printf("using len=1024\n");
	}

	if(speed < 1 || speed > 50)
	{
		speed=2;
		printf("using speed=2\n");
	}

	spi_speed = speed*1000000;
	init_spi();

	uint8_t c = 0x42;
	for(int i=0; i<len; i++)
	{
		expected_rxbuf[i] = c++;
	}


	int check_len = (len>10000)?10000:len;
//	while(1)
//	{
		struct spi_ioc_transfer xfer;
		uint8_t response;

		memset(&xfer, 0, sizeof(xfer)); // unused fields need to be initialized zero.
		xfer.tx_buf = (uint32_t)txbuf;
		xfer.rx_buf = (uint32_t)rxbuf;
		xfer.len = len;
		xfer.cs_change = 1; // deassert chip select after the transfer

		if(ioctl(spi_fd, SPI_IOC_MESSAGE(1), &xfer) < 0)
		{
			printf("ERROR: spi ioctl transfer operation failed: %d (%s)\n", errno, strerror(errno));
			return -1;
		}

		printf("RX buffer:\n");
		int misses = 0;
		for(int i = 0; i < len; i++)
		{
			if(i%4 == 0) printf(" ");
			if(i%32 == 0) printf("\n");
			printf("%02x ", rxbuf[i]);
		}


		for(int i = 0; i < check_len; i++)
		{
			if(rxbuf[i] != expected_rxbuf[i]) misses++;
		}

		printf("\n");
		if(misses)
			printf("WARN: %d mismatches\n", misses);

//	}

	return 0;
}

