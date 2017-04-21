#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <termios.h>
#include <string.h>
#include <fcntl.h>
#include <stropts.h>
#include <sys/select.h>
#include <sys/ioctl.h>

#define MAXBUF 1024

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

	// nonblocking
	tty.c_cc[VMIN] = 0;
	tty.c_cc[VTIME] = 0;

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

#define BUFLEN 2048
 
int main (int argc, char** argv)
{
	uint8_t uart_rx_buf[MAXBUF];
	uint8_t socket_rx_buf[MAXBUF];
	uint8_t socket_tx_buf[MAXBUF];

	if(argc != 3)
	{
		printf("Usage: udpserver <serial device> <port>\ne.g. udpserver /dev/ttyUSB0 22334\n");
		return 1;
	}

	uart = open(argv[1], O_RDWR | O_NOCTTY | O_NONBLOCK);

	if(uart < 0)
	{
		printf("error %d opening %s: %s\n", errno, argv[1], strerror(errno));
		return 1;
	}

	int port = atoi(argv[2]);
	if(port < 1 || port > 65535)
	{
		printf("Invalid port number %u\n", port);
		return 1;
	}

	set_uart_attribs(uart, B115200);

	/* Create the socket and set it up to accept connections. */
	struct sockaddr_in si_me, si_other, si_subscriber;
	 
	int udpsock, i, slen = sizeof(si_other) , recv_len;
	uint8_t buf[BUFLEN];
	 
	//create a UDP socket
	if ((udpsock=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1)
	{
		printf("Error opening UDP socket.\n");
		return 1;
	}
	 
	// zero out the structure
	memset((char *) &si_me, 0, sizeof(si_me));
	 
	si_me.sin_family = AF_INET;
	si_me.sin_port = htons(port);
	si_me.sin_addr.s_addr = htonl(INADDR_ANY);
	 
	//bind socket to port
	if( bind(udpsock , (struct sockaddr*)&si_me, sizeof(si_me) ) == -1)
	{
		printf("Error binding the port to the socket.\n");
		return 1;
	}
	 

	fd_set fds;

	int fds_size = udpsock;
	if(uart > fds_size) fds_size = uart;
	fds_size+=1;

	int rxloc = 0;

	int subscriber = 0;
	int subs_addr_len = 0;

	while(1)
	{
		FD_ZERO(&fds);
		FD_SET(udpsock, &fds);
		FD_SET(uart, &fds);
		if (select(fds_size, &fds, NULL, NULL, NULL) < 0)
		{
			printf("select() failed");
			return 1;
		}

		if(FD_ISSET(udpsock, &fds))
		{
//			printf("UDP!!!\n");
			if ((recv_len = recvfrom(udpsock, buf, BUFLEN, 0, (struct sockaddr *) &si_other, &slen)) == -1)
			{
				printf("recvfrom() failed");
				return 1;
			}
			else
			{
//				printf("LEN = %d  data = %u  %u!!!\n", recv_len, buf[0], buf[1]);
				if(recv_len >= 2 && buf[0] == 123 && buf[1] == 0xaa)
				{
					printf("Subscribed!\n");
					subscriber=1;
					memcpy(&si_subscriber, &si_other, sizeof(si_other));
					subs_addr_len = slen;
				}
				else
				{
//					printf("uart write\n");
					if(write(uart, &buf[1], recv_len-1) != recv_len-1)
					{
						printf("uart write error\n");
					}
				}
			}

		}

		if(FD_ISSET(uart, &fds))
		{
//			printf("UART!!!\n");
			uint8_t byte;
			if(read(uart, &byte, 1) < 0)
			{
				printf("read() (uart) failed");
				return 1;
			}

			if(rxloc > 1000)
				rxloc = 0;

			if(byte > 127)
			{
				if(subscriber)
				{
					if(sendto(udpsock, uart_rx_buf, rxloc, 0, (struct sockaddr*) &si_subscriber, subs_addr_len) == -1)
					{
						printf("sendto() failed");
						return 1;
					}
				}

				uart_rx_buf[0] = byte;
				rxloc = 1;
				break;
			}

			uart_rx_buf[rxloc] = byte;
			rxloc++;
		}

	}

}

