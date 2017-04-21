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

int build_socket(uint16_t port)
{
	int sock;
	struct sockaddr_in name;

	sock = socket (PF_INET, SOCK_STREAM, 0);
	if (sock < 0)
	{
		perror ("socket");
		exit (EXIT_FAILURE);
	}

	name.sin_family = AF_INET;
	name.sin_port = htons (port);
	name.sin_addr.s_addr = htonl (INADDR_ANY);
	if (bind(sock, (struct sockaddr *) &name, sizeof (name)) < 0)
	{
		perror ("bind");
		exit (EXIT_FAILURE);
	}

	return sock;
}

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

int read_from_client (int filedes)
{
	char buffer[MAXBUF];
	int nbytes;

	nbytes = read (filedes, buffer, MAXBUF);
	if (nbytes < 0)
	{
		/* Read error. */
		perror ("read");
		exit (EXIT_FAILURE);
	}
	else if (nbytes == 0)
		/* End-of-file. */
		return -1;
	else
	{
		/* Data read. */
		fprintf (stderr, "Server: got message: `%s'\n", buffer);
		return 0;
	}
}

int main (int argc, char** argv)
{
	uint8_t uart_rx_buf[MAXBUF];
	uint8_t socket_rx_buf[MAXBUF];
	uint8_t socket_tx_buf[MAXBUF];

	int sock;
	fd_set active_fd_set, read_fd_set;
	int i;
	struct sockaddr_in clientname;
	size_t size;

	if(argc != 3)
	{
		printf("Usage: tcpserver <serial device> <port>\ne.g. tcpserver /dev/ttyUSB0 22334\n");
		return 1;
	}

	uart = open(argv[1], O_RDWR | O_NOCTTY);

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
	sock = build_socket(port);
	if (listen (sock, 1) < 0)
	{
		perror ("listen");
		exit(EXIT_FAILURE);
	}

	/* Initialize the set of active sockets. */
	FD_ZERO(&active_fd_set);
	FD_SET(sock, &active_fd_set);
	FD_SET(uart, &active_fd_set);

	int latest = 0;
	int rxloc = 0;

	while(1)
	{
		/* Block until input arrives on one or more active sockets. */
		read_fd_set = active_fd_set;
		if (select (FD_SETSIZE, &read_fd_set, NULL, NULL, NULL) < 0)
		{
			perror ("select");
			exit (EXIT_FAILURE);
		}

		/* Service all the sockets with input pending. */
		for (i = 0; i < FD_SETSIZE; ++i)
		{
			if (FD_ISSET (i, &read_fd_set))
			{
				if(i == uart)
				{
					uint8_t byte;
					if(read(uart, &byte, 1) < 0)
					{
						perror ("uart_read");
						exit (EXIT_FAILURE);
					}

					if(rxloc > 1000)
						rxloc = 0;

					if(byte > 127)
					{
						if(latest == 0)
						{
							printf("Data received on uart - not relaying until TCP connection is established.\n");
						}
						else
						{
							memcpy(socket_tx_buf, uart_rx_buf, rxloc);

							int n = 0;
							while(n < rxloc)
							{
								int nn = write(latest, socket_tx_buf, rxloc);
								if (nn < 0) 
									error("ERROR writing to socket");
								n += nn;
							}
						}
						uart_rx_buf[0] = byte;
						rxloc = 1;
						break;
					}

					uart_rx_buf[rxloc] = byte;
					rxloc++;
				}
				else if (i == sock)
				{
					/* Connection request on original socket. */
					int new;
					size = sizeof (clientname);
					new = accept (sock, (struct sockaddr *) &clientname, &size);
					if (new < 0)
					{
						perror ("accept");
						exit (EXIT_FAILURE);
					}
					latest = new;
					fprintf (stderr, "Server: connect from host %u, port %hd.\n", clientname.sin_addr.s_addr,
					        ntohs(clientname.sin_port));

					FD_SET (new, &active_fd_set);
				}
				else
				{
					/* Data arriving on an already-connected socket. */
					if (read_from_client (i) < 0)
					{
						close (i);
						FD_CLR (i, &active_fd_set);
					}
				}
			}
		}
	}
}

