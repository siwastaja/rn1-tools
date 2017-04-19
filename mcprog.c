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

#define NUM_PAGES 30
#define PAGE_LEN 1024

int main(int argc, char** argv)
{
	int i;
	uint32_t addr = 0x08000000;
	uint8_t file[30*1024];
	uint8_t check[30*1024];
	uint8_t buf[16];
	uint8_t rxbuf[16];

	if(argc != 4)
	{
		printf("Usage: mcprog <serial dev> <bin file> <motor controlled num>\n");
		printf("e.g. /dev/ttyUSB0 mc.bin 4\n");
		printf("Programs a motor controller mcu through the brain mcu.\n");
		printf("Through UART to rn1-brain, from there through SPI to the motcon MCU.\n");
		return 1;
	}

	FILE* bin = fopen(argv[2], "rb");

	if(!bin)
	{
		printf("Error opening %s.\n", argv[2]);
		return 1;
	}

	int mcnum = argv[3][0]-'0';
	if(mcnum < 1 || mcnum > 4)
	{
		printf("motor controller num must be between 1 and 4\n");
		return 1;
	}

	int size = fread(file, 1, 30*1024-1, bin);

	if(size < 10 || size >= 30*1024-1)
	{
		printf("File length mismatch (%u)\n", size);
		return 1;
	}

	if(size&1)
	{
		printf("INFO: Padded uneven file size for full 16-bit words.\n");
		file[size] = 0xff;
		size++;
	}

	uart = open(argv[1], O_RDWR | O_NOCTTY);

	if(uart < 0)
	{
		printf("error %d opening %s: %s\n", errno, argv[1], strerror(errno));
		return 1;
	}



	set_uart_attribs(uart, B115200);

	usleep(200000);
	tcflush(uart, TCOFLUSH);

	printf("Entering flasher on MC%u...\n", mcnum);

	buf[0] = 0xfe;
	buf[1] = 0x42;
	buf[2] = 0x11;
	buf[3] = 0x7a;
	buf[4] = 0x53;
	buf[5] = mcnum;
	buf[6] = 0xff;
	if(write(uart, buf, 7) != 7)
	{
		printf("UART write fail (entering flasher)\n");
		goto FAIL;
	}
	usleep(500000);
	tcflush(uart, TCIFLUSH);

	int pages = size/PAGE_LEN + 1;
	printf("Erasing %u pages\n", pages);

	buf[0] = 100;
	buf[1] = pages;
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

	printf("Writing %u bytes...\n", size);
	buf[0] = 101;
	buf[1] = (size&0xff00)>>8;
	buf[2] = (size&0xff);

	if(write(uart, buf, 3) != 3)
	{
		printf("UART write fail (write header)\n");
		goto FAIL;
	}

	if(write(uart, file, size) != size)
	{
		printf("UART write fail (write bin data)\n");
		goto FAIL;
	}

	usleep(200000);

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


	printf("Reading %u bytes\n", size);

	buf[0] = 102;
	// size still the same.

	if(write(uart, buf, 3) != 3)
	{
		printf("UART write fail (read header)\n");
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

	printf("Soft-resetting...\n");
	buf[0] = 151;

	write(uart, buf, 1);

	usleep(200000);

FAIL:
	close(uart);
	fclose(bin);

	return 0;
}
