#define _BSD_SOURCE // for usleep
#include <inttypes.h>
#include <termios.h>
#include <unistd.h>
#include <string.h>
#include <stdio.h>
#include <stdio_ext.h>
#include <stdlib.h>
#include <fcntl.h>
#include <stropts.h>
#include <sys/select.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <math.h>

int set_uart_attribs(int fd, int speed)
{
	struct termios tty;
	memset(&tty, 0, sizeof(tty));
	if(tcgetattr(fd, &tty) != 0)
	{
		printf("error %d from tcgetattr\n", errno);
		return -1;
	}

	cfmakeraw(&tty);
	tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;

	tty.c_cflag |= (CLOCAL | CREAD);
	tty.c_cflag &= ~(PARENB | PARODD);
	tty.c_cflag &= ~CSTOPB;

	tty.c_cc[VMIN] = 255; // Return from read() if 255 characters received, or...
	tty.c_cc[VTIME] = 10; // if 1s elapsed from the first character

	cfsetospeed(&tty, speed);
	cfsetispeed(&tty, speed);

	if(tcsetattr(fd, TCSANOW, &tty) != 0)
	{
		printf("error %d from tcsetattr\n", errno);
		return -1;
	}
	return 0;
}

int uart;

#define NUM_SECTORS 10
const int sector_lens[NUM_SECTORS] =
{
16*1024,
16*1024,
16*1024,
16*1024,
64*1024,
128*1024,
128*1024,
128*1024,
128*1024,
128*1024
};

int main(int argc, char** argv)
{
	int i;
	uint32_t addr = 0x08000000;
	uint8_t file[640*1024];
	uint8_t check[640*1024];
	uint8_t buf[16];
	uint8_t rxbuf[16];

	if(argc != 3)
	{
		printf("Usage: prog /dev/ttyUSB0 main.bin\n");
		return 1;
	}

	FILE* bin = fopen(argv[2], "rb");

	if(!bin)
	{
		printf("Error opening %s.\n", argv[2]);
		return 1;
	}

	int size = fread(file, 1, 640*1024, bin);

	if(size < 10 || size >= 640*1024-1)
	{
		printf("File length mismatch (%u)\n", size);
		return 1;
	}


	uart = open(argv[1], O_RDWR | O_NOCTTY);

	if(uart < 0)
	{
		printf("error %d opening %s: %s\n", errno, argv[1], strerror(errno));
		return 1;
	}



	set_uart_attribs(uart, B115200);

	printf("Entering flasher...\n");

	buf[0] = '6';
	buf[1] = '7';
	buf[2] = '8';
	buf[3] = '9';
	write(uart, buf, 4);
	usleep(500000);
	tcflush(uart, TCIFLUSH);

	int cur_s;
	int cumul_bytes = 0;
	for(cur_s = 0; cur_s < NUM_SECTORS; cur_s++)
	{
		printf("Erasing sector %u\n", cur_s);

		buf[0] = 100;
		buf[1] = cur_s;
		write(uart, buf, 2);

		rxbuf[0] = 2;
		read(uart, rxbuf, 1);

		if(rxbuf[0] == 0)
		{
			printf("OK\n");
		}
		else
		{
			printf("Failure %u\n", rxbuf[0]);
			goto FAIL;
		}

		cumul_bytes += sector_lens[cur_s];
		if(cumul_bytes >= size)
		{
			break;
		}
	}

	printf("Writing %u bytes starting from 0x%08x...\n", size, addr);
	buf[0] = 101;
	buf[1] = (addr&0xff000000)>>24;
	buf[2] = (addr&0xff0000)>>16;
	buf[3] = (addr&0xff00)>>8;
	buf[4] = (addr&0xff);
	buf[5] = (size&0xff000000)>>24;
	buf[6] = (size&0xff0000)>>16;
	buf[7] = (size&0xff00)>>8;
	buf[8] = (size&0xff);

	if(write(uart, buf, 9) != 9)
	{
		printf("UART write fail (write header)\n");
		goto FAIL;
	}

	if(write(uart, file, size) != size)
	{
		printf("UART write fail (write bin data)\n");
		goto FAIL;
	}

	usleep(500000);

	rxbuf[0] = 2;
	read(uart, rxbuf, 1);

	if(rxbuf[0] == 0)
	{
		printf("OK\n");
	}
	else
	{
		printf("Failure %u\n", rxbuf[0]);
		goto FAIL;
	}


	printf("Reading %u bytes starting from 0x%08x...\n", size, addr);

	buf[0] = 102;
	// Addr, size still the same.

	if(write(uart, buf, 9) != 9)
	{
		printf("UART write fail (read header)\n");
		goto FAIL;
	}


	rxbuf[0] = 2;
	read(uart, rxbuf, 1);

	if(rxbuf[0] == 0)
	{
		printf("OK\n");
	}
	else
	{
		printf("Failure %u\n", rxbuf[0]);
		goto FAIL;
	}


	int nbytes = 0;
	uint8_t *p_check = check;
	while(nbytes < size)
	{
		int bytes_got = read(uart, p_check, size-nbytes);
		nbytes+=bytes_got;
		p_check += bytes_got;
//		printf("Got %u bytes\n", bytes_got);
	}

	for(i = 0; i < size; i++)
	{
		if(check[i] != file[i])
		{
			printf("Verification error at %u (0x%02x != 0x%02x)!\n", i, check[i], file[i]);
			goto FAIL;
		}
	}

	printf("Hard-resetting...\n");

	buf[0] = 150;
	write(uart, buf, 1);

	usleep(500000);

FAIL:
	close(uart);
	fclose(bin);

	return 0;
}
