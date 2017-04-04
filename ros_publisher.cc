#include <stdint.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <stropts.h>
#include <sys/select.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <cstdio>
#include <cstring>
#include <cmath>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>



#define I14_I16(msb,lsb) ((int16_t)( ( ((uint16_t)(msb)<<9) | ((uint16_t)(lsb)<<2) ) ))

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

	tty.c_cc[VMIN] = 255;
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

#define PUBLISHER_BUFFER_LEN 1024  // Must be able to include the longest message possible, plus the start byte.

const int message_lens[128] =
{
7,	// 0x80 MSG_GYRO
7,	// 0x81 MSG_XCEL
7,	// 0x82 MSG_COMPASS
6,	// 0x83 MSG_OPTFLOW
721,	// 0x84 MSG_LIDAR
15,	// 0x85 MSG_SONAR
0,	// 0x86
0,	// 0x87
0,	// 0x88
0,	// 0x89
0,	// 0x8a
0,	// 0x8b
0,	// 0x8c
0,	// 0x8d
0,	// 0x8e
0,	// 0x8f

0,	// 0x90
0,	// 0x91
0,	// 0x92
0,	// 0x93
0,	// 0x94
0,	// 0x95
0,	// 0x96
0,	// 0x97
0,	// 0x98
0,	// 0x99
0,	// 0x9a
0,	// 0x9b
0,	// 0x9c
0,	// 0x9d
0,	// 0x9e
0,	// 0x9f

0,	// 0xa0
0,	// 0xa1
0,	// 0xa2
0,	// 0xa3
0,	// 0xa4
0,	// 0xa5
0,	// 0xa6
0,	// 0xa7
0,	// 0xa8
0,	// 0xa9
0,	// 0xaa
0,	// 0xab
0,	// 0xac
0,	// 0xad
0,	// 0xae
0,	// 0xaf

0,	// 0xb0
0,	// 0xb1
0,	// 0xb2
0,	// 0xb3
0,	// 0xb4
0,	// 0xb5
0,	// 0xb6
0,	// 0xb7
0,	// 0xb8
0,	// 0xb9
0,	// 0xba
0,	// 0xbb
0,	// 0xbc
0,	// 0xbd
0,	// 0xbe
0,	// 0xbf

0,	// 0xc0
0,	// 0xc1
0,	// 0xc2
0,	// 0xc3
0,	// 0xc4
0,	// 0xc5
0,	// 0xc6
0,	// 0xc7
0,	// 0xc8
0,	// 0xc9
0,	// 0xca
0,	// 0xcb
0,	// 0xcc
0,	// 0xcd
0,	// 0xce
0,	// 0xcf

0,	// 0xd0
0,	// 0xd1
0,	// 0xd2
0,	// 0xd3
0,	// 0xd4
0,	// 0xd5
0,	// 0xd6
0,	// 0xd7
0,	// 0xd8
0,	// 0xd9
0,	// 0xda
0,	// 0xdb
0,	// 0xdc
0,	// 0xdd
0,	// 0xde
0,	// 0xdf

0,	// 0xe0
0,	// 0xe1
0,	// 0xe2
0,	// 0xe3
0,	// 0xe4
0,	// 0xe5
0,	// 0xe6
0,	// 0xe7
0,	// 0xe8
0,	// 0xe9
0,	// 0xea
0,	// 0xeb
0,	// 0xec
0,	// 0xed
0,	// 0xee
0,	// 0xef

0,	// 0xf0
0,	// 0xf1
0,	// 0xf2
0,	// 0xf3
0,	// 0xf4
0,	// 0xf5
0,	// 0xf6
0,	// 0xf7
0,	// 0xf8
0,	// 0xf9
0,	// 0xfa
0,	// 0xfb
0,	// 0xfc
0,	// 0xfd
0,	// 0xfe
0	// 0xff
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "rn1_publisher");
	uint8_t buf[PUBLISHER_BUFFER_LEN];
	if(argc != 2)
	{
		printf("Usage: ros_publisher /dev/ttyUSB0\n");
		return 1;
	}

	uart = open(argv[1], O_RDWR | O_NOCTTY);

	if(uart < 0)
	{
		printf("error %d opening %s: %s\n", errno, argv[1], strerror(errno));
		return 1;
	}

	set_uart_attribs(uart, B115200);

	tcflush(uart, TCIFLUSH);

	bool running = true;

	ros::NodeHandle n;
	ros::Publisher lidar_pub = n.advertise<sensor_msgs::LaserScan>("lidar", 50);

	while(running)
	{

		ros::Time rx_time;
		/*
			Read the stream one byte at time until we hit a start delimiter.
			After the start delimiter, which works as a message type identifier, we
			know the upcoming frame length, so we can use the right read() nbytes.
			Subsequent 1-long reads will return the next start delimiter right away.

			In error cases, the 1-long read loop reads and ignores any data until the
			next start byte, resyncing the stream.
		*/

		uint8_t byte;
		do {read(uart, &byte, 1);} while(byte<0x80);  // Blocking 1-byte reads until start frame is received.
		rx_time = ros::Time::now();
		uint8_t msgid = byte&0x7f;

		// In most cases, read() should block until all data has arrived (especially for small frames),
		// but it may return before nbytes are received (and it will do that with frames > 255 bytes),
		// so we append to the buffer as long as needed.

		int nbytes = 0;
		do
		{
			nbytes += read(uart, buf+nbytes, message_lens[msgid]-nbytes);
		} while(nbytes < message_lens[msgid]);

		// Now we have a full frame to be parsed.

		switch(msgid)
		{
			case 0x00:
			gyro_x = (double)I14_I16(parsebuf[3], parsebuf[2]);
			gyro_y = (double)I14_I16(parsebuf[5], parsebuf[4]);
			gyro_z = (double)I14_I16(parsebuf[7], parsebuf[6]);
			break;

			case 129:
			xcel_x = (double)I14_I16(parsebuf[3], parsebuf[2]);
			xcel_y = (double)I14_I16(parsebuf[5], parsebuf[4]);
			xcel_z = (double)I14_I16(parsebuf[7], parsebuf[6]);
			break;

			case 0x04:
			sensor_msgs::LaserScan scan;
			scan.header.stamp = rx_time;
			scan.header.frame_id = "lidar";
			scan.angle_min = -M_PI;
			scan.angle_max = M_PI;
			scan.angle_increment = 2.0*M_PI/360.0;
			scan.time_increment = 1.0/(5.0*360.0);
			scan.range_min = 0.1;
			scan.range_max = 10.0;
			scan.ranges.resize(360);
			for(int i = 0; i < 360; i++)
			{
				scan.ranges[i] = (float)((buf[1+2*i+1]<<7) | (buf[1+2*i]))/1000.0;
			}

			lidar_pub.publish(scan);
			break;

			default:
			break;
		}

	}


	return 0;
}
