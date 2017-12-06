#define _BSD_SOURCE  // glibc backwards incompatibility workaround to bring usleep back.
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <stropts.h>
#include <sys/select.h>
#include <sys/ioctl.h>
#include <sys/wait.h>
#include <errno.h>
#include <cstdio>
#include <cmath>
#include <cstring>
#include <termios.h>
#include <sys/select.h>
#include <sys/ioctl.h>
#include <cstdlib>
#include <linux/serial.h>
#include <SFML/Graphics.hpp>


#define EPC_XS 160
#define EPC_YS 60

#define sq(x) ((x)*(x))

#define DATA_LOW 65534
#define DATA_OVEREXP 65535


#define USE_UART


/* Open serial port in raw mode, with custom baudrate if necessary */
int serial_open(const char *device, int rate)
{
	struct termios options;
	int fd;

	/* Open and configure serial port */
	if ((fd = open(device,O_RDWR|O_NOCTTY|O_NONBLOCK)) == -1)
		return -1;


#ifdef CUSTOM_BAUD
	struct serial_struct serinfo;
	/* Custom divisor */
	serinfo.reserved_char[0] = 0;
	if (ioctl(fd, TIOCGSERIAL, &serinfo) < 0)
		return -1;
	serinfo.flags &= ~ASYNC_SPD_MASK;
	serinfo.flags |= ASYNC_SPD_CUST;
	serinfo.custom_divisor = (serinfo.baud_base + (rate / 2)) / rate;
	if (serinfo.custom_divisor < 1) 
		serinfo.custom_divisor = 1;
	if (ioctl(fd, TIOCSSERIAL, &serinfo) < 0)
		return -1;
	if (ioctl(fd, TIOCGSERIAL, &serinfo) < 0)
		return -1;
	if (serinfo.custom_divisor * rate != serinfo.baud_base) {
		printf("actual baudrate is %d / %d = %f\n",
		      serinfo.baud_base, serinfo.custom_divisor,
		      (float)serinfo.baud_base / serinfo.custom_divisor);
	}
#endif

	fcntl(fd, F_SETFL, 0);
	tcgetattr(fd, &options);

#ifdef CUSTOM_BAUD
	cfsetispeed(&options, B38400);
	cfsetospeed(&options, B38400);
#else
	cfsetispeed(&options, B921600);
	cfsetospeed(&options, B921600);
#endif
	cfmakeraw(&options);
	options.c_cflag = (options.c_cflag & ~CSIZE) | CS8;
	options.c_cflag |= (CLOCAL | CREAD);
	options.c_cflag &= ~(PARENB | PARODD);
	options.c_cflag &= ~CSTOPB;
	options.c_cflag &= ~CRTSCTS;

	// nonblocking
	options.c_cc[VMIN] = 0;
	options.c_cc[VTIME] = 0;

	if (tcsetattr(fd, TCSANOW, &options) != 0)
		return -1;

	return fd;
}

int uart;

const char serial_dev[] = "/dev/ttyUSB0";

int init_uart()
{
//	uart = open(serial_dev, O_RDWR | O_NOCTTY | O_NONBLOCK);

	uart = serial_open(serial_dev, 460800);
	if(uart < 0)
	{
		printf("error %d opening %s: %s\n", errno, serial_dev, strerror(errno));
		return 1;
	}



	printf("UART init OK\n");
	return 0;

}

int send_uart(uint8_t* buf, int len)
{
	int timeout = 100;
	while(len > 0)
	{
		int ret = write(uart, buf, len);
		if(ret == 0)
		{
			usleep(1000);
			timeout--;
			if(timeout == 0)
			{
				fprintf(stderr, "uart write error: timeout, write() returns 0 for 100 ms\n");
				return 1;
			}
		}
		else if(ret == -1)
		{
			fprintf(stderr, "uart write error %d: %s\n", errno, strerror(errno));
			return 2;
		}
		else
		{
			len -= ret;
			buf += ret;
		}
	}

	return 0;
}

typedef struct __attribute__((packed))
{
	// *4, then -1 to form the register values:
	uint16_t bw_int_len; 
	uint16_t dist_int_len;

	uint8_t clk_div; // 1 = 20MHz LED, 2 = 10 MHz...
} config_t;

config_t config =  // active configuration
{
	.bw_int_len = 1000,
	.dist_int_len = 1000,
	.clk_div = 1
};


void send_hw_settings()
{
	send_uart((uint8_t*)&config, sizeof(config_t));
}


#define UART_RX_BUF_SIZE 65536
uint8_t uart_rx_buf[UART_RX_BUF_SIZE];
int uart_rx_wr;
int uart_rx_rd;

/*
UART packet structure

1 byte: Packet type header
2 bytes: Packet payload size (little endian)
Payload 1 to 65536 bytes
1 byte: CRC8 over the payload


resynchronization sequence:
0xaa (header) 0x08 0x00 (payload size) 0xff,0xff,0xff,0xff,0xff,0xff,0x12,0xab (payload)  + CRC8 (should be 0xd6)
*/


typedef enum
{
	S_UART_RESYNC	= 0,
	S_UART_HEADER	= 1,
	S_UART_PAYLOAD	= 2,
	S_UART_CHECKSUM = 3
} uart_state_t;

uart_state_t cur_uart_state;

uint8_t uart_payload[65536];


#define CRC_INITIAL_REMAINDER 0x00
#define CRC_POLYNOMIAL 0x07 // As per CRC-8-CCITT

#define CALC_CRC(remainder) \
	for(int crc__bit = 8; crc__bit > 0; --crc__bit) \
	{ \
		if((remainder) & 0b10000000) \
		{ \
			(remainder) = ((remainder) << 1) ^ CRC_POLYNOMIAL; \
		} \
		else \
		{ \
			(remainder) = ((remainder) << 1); \
		} \
	}

int parse_uart_msg(uint8_t* buf, int msgid, int len);
// Call this when there is data in the uart read buffer (using select, for example)
void handle_uart()
{
	static uint8_t header[3];
	static int header_left;
	static int msg_id, msg_size, payload_left;
	static uint8_t checksum_accum;

	switch(cur_uart_state)
	{
		case S_UART_RESYNC:
		{
			static int resync_cnt = 0;
			static const uint8_t expected_resync[12] = {0xaa,0x08,0x00, 0xff,0xff,0xff,0xff,0xff,0xff,0x12,0xab, 0xd6};

			uint8_t byte;
			int ret;

			while( (ret=read(uart, &byte, 1)) != 0)
			{
				if(ret < 0)
				{
					printf("ERROR: read() (uart) failed in RESYNC, errno=%d\n", errno);
					resync_cnt = 0;
					break;
				}
				else if(ret == 1)
				{
					//printf("0x%02x  cnt=%d\n", byte, resync_cnt);
					if(byte == expected_resync[resync_cnt])
					{
						resync_cnt++;
						if(resync_cnt == 12)
						{
							resync_cnt = 0;
							cur_uart_state = S_UART_HEADER;
							header_left = 3;
							printf("INFO: UART RESYNC OK.\n");
							break;
						}
					}
					else
					{
						resync_cnt = 0;
					}
				}
				else
				{
					printf("ERROR: read() (uart) returned %d in RESYNC\n", ret);
				}
			}
		}
		break;

		case S_UART_HEADER:
		{
			int ret = read(uart, &header[3-header_left], header_left);
			if(ret < 0)
			{
				printf("ERROR: read() (uart) failed in S_UART_HEADER, errno=%d\n", errno);
				cur_uart_state = S_UART_RESYNC;
				break;
			}

			header_left -= ret;

			if(header_left == 0)
			{
				msg_id = header[0];
				msg_size = ((int)header[2])<<8 | ((int)header[1]);

				payload_left = msg_size;
				checksum_accum = CRC_INITIAL_REMAINDER;
				cur_uart_state = S_UART_PAYLOAD;
				//printf("INFO: Got header, msgid=%d, len=%d\n", msg_id, msg_size);

			}
		}
		break;

		case S_UART_PAYLOAD:
		{
			int ret = read(uart, &uart_payload[msg_size-payload_left], payload_left);
			//printf("PAYLOAD: read() msg_size=%d, payload_left=%d, idx=%d, data = ", msg_size, payload_left, msg_size-payload_left);
			//for(int i = 0; i < ret; i++)
			//	printf("%02x ", uart_payload[msg_size-payload_left+i]);
			//printf("\n");
			

			if(ret < 0)
			{
				printf("ERROR: read() (uart) failed in S_UART_PAYLOAD, errno=%d\n", errno);
				cur_uart_state = S_UART_RESYNC;
				break;
			}

			for(int i = 0; i < ret; i++)
			{
				checksum_accum ^= uart_payload[msg_size-payload_left+i];
				CALC_CRC(checksum_accum);
			}

			//printf("INFO: READ %d BYTES OF PAYLOAD, %d left (1st: %d)\n", ret, payload_left, uart_payload[msg_size-payload_left]);

			payload_left -= ret;

			if(payload_left == 0)
			{
				cur_uart_state = S_UART_CHECKSUM;
			}

		}
		break;

		case S_UART_CHECKSUM:
		{
			uint8_t chk;
			int ret = read(uart, &chk, 1);
			if(ret < 0)
			{
				printf("ERROR: read() (uart) failed in S_UART_CHECKSUM, errno=%d\n", errno);
				cur_uart_state = S_UART_RESYNC;
				break;
			}

			if(ret == 1)
			{
				if(chk != checksum_accum)
				{
					printf("WARN: UART checksum mismatch (msgid=%d len=%d chk_received=%02x calculated=%02x), ignoring message & resyncing.\n", msg_id, msg_size, chk, checksum_accum);

					/*for(int i = 0; i < msg_size; i++)
					{
						printf("%02x ", uart_payload[i]);
						if(i%16==15) printf("\n");
					}
					printf("\n");*/
					cur_uart_state = S_UART_RESYNC;
					break;
				}

				//printf("INFO: Parsing UART MESSAGE (msgid=%d len=%d chksum=%02x)\n", msg_id, msg_size, chk);
				parse_uart_msg(uart_payload, msg_id, msg_size);
				cur_uart_state = S_UART_HEADER;
				header_left = 3;

			}
		}
		break;

		default:
		break;

	}
}


int screen_x=640,screen_y=480;
sf::Font arial;
sf::Font monospace_regular;
sf::Font monospace_bold;

int mir_x = 0;
int mir_y = 0;
int rotated = 0;
int mono_offset = 0;
int mono_div = 8;
int dcs_offset = 2048;
int dcs_div = 16;

void draw_mono(sf::RenderWindow& win, float* img, int x_on_screen, int y_on_screen, float scale)
{
	static uint8_t pix[EPC_XS*EPC_YS*4];

	int i = 0;
	int yy = (mir_y?(EPC_YS-1):0);
	while(1)
	{
		int xx = (mir_x?(EPC_XS-1):0);
		while(1)
		{
			pix[4*(yy*EPC_XS+xx)+0] =
			pix[4*(yy*EPC_XS+xx)+1] =
			pix[4*(yy*EPC_XS+xx)+2] = (img[i]+mono_offset)/mono_div;
			pix[4*(yy*EPC_XS+xx)+3] = 255;
			i++;
			if( (mir_x && --xx<0) || (!mir_x && ++xx>=EPC_XS) ) break;
		}
		if( (mir_y && --yy<0) || (!mir_y && ++yy>=EPC_YS) ) break;
	}

	sf::Texture t;
	t.create(EPC_XS, EPC_YS);
	t.setSmooth(false);
	t.update(pix);
	sf::Sprite sprite;
	sprite.setTexture(t);
	sprite.setPosition((float)x_on_screen, (float)y_on_screen);
	sprite.setScale(sf::Vector2f(scale, scale));
	if(rotated)
	{
		sprite.setRotation(90.0);
		sprite.setPosition(x_on_screen+EPC_YS*scale, y_on_screen);
	}
	else
	{
		sprite.setPosition(x_on_screen, y_on_screen);
	}
	win.draw(sprite);
}


void draw_dist(sf::RenderWindow& win, float* img, int x_on_screen, int y_on_screen, float scale)
{
	static uint8_t pix[EPC_XS*EPC_YS*4];

	int i = 0;
	int yy = (mir_y?(EPC_YS-1):0);
	while(1)
	{
		int xx = (mir_x?(EPC_XS-1):0);
		while(1)
		{
			if(img[i] == DATA_LOW)
			{
				pix[4*(yy*EPC_XS+xx)+0] = 0;
				pix[4*(yy*EPC_XS+xx)+1] = 0;
				pix[4*(yy*EPC_XS+xx)+2] = 128;
			}
			else if(img[i] == DATA_OVEREXP)
			{
				pix[4*(yy*EPC_XS+xx)+0] = 160;
				pix[4*(yy*EPC_XS+xx)+1] = 0;
				pix[4*(yy*EPC_XS+xx)+2] = 160;
			}
			else
			{
				float percolor = 1000;

				float mm = img[i];
				float f_r = 1.0 - fabs(mm-0*percolor)/percolor;
				float f_g = 1.0 - fabs(mm-1*percolor)/percolor;
				float f_b = 1.0 - fabs(mm-2*percolor)/percolor;

				int r = f_r*256.0; if(r<0) r=0; else if(r>255) r=255;
				int g = f_g*256.0; if(g<0) g=0; else if(g>255) g=255;
				int b = f_b*256.0; if(b<0) b=0; else if(b>255) b=255;


				pix[4*(yy*EPC_XS+xx)+0] = r;
				pix[4*(yy*EPC_XS+xx)+1] = g;
				pix[4*(yy*EPC_XS+xx)+2] = b;
			}
			pix[4*(yy*EPC_XS+xx)+3] = 255;
			i++;
			if( (mir_x && --xx<0) || (!mir_x && ++xx>=EPC_XS) ) break;
		}
		if( (mir_y && --yy<0) || (!mir_y && ++yy>=EPC_YS) ) break;
	}

	sf::Texture t;
	t.create(EPC_XS, EPC_YS);
	t.setSmooth(false);
	t.update(pix);
	sf::Sprite sprite;
	sprite.setTexture(t);
	sprite.setPosition((float)x_on_screen, (float)y_on_screen);
	sprite.setScale(sf::Vector2f(scale, scale));
	if(rotated)
	{
		sprite.setRotation(90.0);
		sprite.setPosition(x_on_screen+EPC_YS*scale, y_on_screen);
	}
	else
	{
		sprite.setPosition(x_on_screen, y_on_screen);
	}
	win.draw(sprite);
}

void draw_dcs(sf::RenderWindow& win, float* img, int x_on_screen, int y_on_screen, float scale)
{
	static uint8_t pix[EPC_XS*EPC_YS*4];

	int i = 0;
	int yy = (mir_y?(EPC_YS-1):0);
	while(1)
	{
		int xx = (mir_x?(EPC_XS-1):0);
		while(1)
		{
			pix[4*(yy*EPC_XS+xx)+0] =
			pix[4*(yy*EPC_XS+xx)+1] =
			pix[4*(yy*EPC_XS+xx)+2] = ((int16_t)img[i]+dcs_offset)/dcs_div;
			pix[4*(yy*EPC_XS+xx)+3] = 255;
			i++;
			if( (mir_x && --xx<0) || (!mir_x && ++xx>=EPC_XS) ) break;
		}
		if( (mir_y && --yy<0) || (!mir_y && ++yy>=EPC_YS) ) break;
	}

	sf::Texture t;
	t.create(EPC_XS, EPC_YS);
	t.setSmooth(false);
	t.update(pix);
	sf::Sprite sprite;
	sprite.setTexture(t);
	sprite.setPosition((float)x_on_screen, (float)y_on_screen);
	sprite.setScale(sf::Vector2f(scale, scale));
	if(rotated)
	{
		sprite.setRotation(90.0);
		sprite.setPosition(x_on_screen+EPC_YS*scale, y_on_screen);
	}
	else
	{
		sprite.setPosition(x_on_screen, y_on_screen);
	}
	win.draw(sprite);
}


#define N_AT_POINTS 9

int sel_at_point = 0;

int at_points[N_AT_POINTS][2] =
{
	{2*EPC_XS/4, 2*EPC_YS/4},
	{2*EPC_XS/4, 1*EPC_YS/4},
	{2*EPC_XS/4, 3*EPC_YS/4},
	{1*EPC_XS/4, 2*EPC_YS/4},
	{1*EPC_XS/4, 1*EPC_YS/4},
	{1*EPC_XS/4, 3*EPC_YS/4},
	{3*EPC_XS/4, 2*EPC_YS/4},
	{3*EPC_XS/4, 1*EPC_YS/4},
	{3*EPC_XS/4, 3*EPC_YS/4}
};


typedef struct
{
	float min;
	float avg;
	float max;

	float val_at[N_AT_POINTS];
	float avg9_at[N_AT_POINTS];
	float avg49_at[N_AT_POINTS];
	float noise49_at[N_AT_POINTS];
} spatial_analysis_t;

void draw_analysis(sf::RenderWindow& win, spatial_analysis_t *spat, int x_on_screen, int y_on_screen, float scale)
{
	// Draw at_points on the image

	for(int i=0; i<N_AT_POINTS; i++)
	{
		if(at_points[i][0] < 0 || at_points[i][0] >= EPC_XS || at_points[i][1] < 0 || at_points[i][1] >= EPC_YS)
			continue;

		float xprepre = at_points[i][0]+0.5;
		float yprepre = at_points[i][1]+0.5;

		float xpre = scale*(mir_x?(EPC_XS-xprepre):xprepre);
		float ypre = scale*(mir_y?(EPC_YS-yprepre):yprepre);
		float x,y;
		if(rotated)
		{
			x = scale*EPC_YS-ypre;
			y = xpre;
		}
		else
		{
			x = xpre;
			y = ypre;
		}

		bool sel = i==sel_at_point;

		sf::CircleShape circ(scale*3.5);
		circ.setOrigin(scale*3.5, scale*3.5);
		circ.setFillColor(sf::Color::Transparent);
		circ.setOutlineThickness(1.0);
		circ.setOutlineColor(sf::Color(255,0,0,sel?255:128));
		circ.setPosition(x_on_screen+x, y_on_screen+y);
		win.draw(circ);

		sf::CircleShape circ2(scale*1.5);
		circ2.setOrigin(scale*1.5, scale*1.5);
		circ2.setFillColor(sf::Color::Transparent);
		circ2.setOutlineThickness(1.0);
		circ2.setOutlineColor(sf::Color(255,0,0,sel?255:128));
		circ2.setPosition(x_on_screen+x, y_on_screen+y);
		win.draw(circ2);

		sf::Text t;
		t.setFont(arial);
		char label[2];
		label[0] = '1'+i;
		label[1] = '\0';
		t.setString(label);
		t.setCharacterSize(12);
		t.setFillColor(sf::Color(255,0,0,sel?255:128));
		t.setPosition(x_on_screen+x+3.5*scale-2, y_on_screen+y+3.5*scale-2);
		win.draw(t);
	}

	sf::Text t;
	t.setFont(monospace_regular);
	char buf[256];
	t.setCharacterSize(14);
	t.setFillColor(sf::Color(255,255,255));

	sprintf(buf, "min %4.0f  avg %6.1f  max %4.0f", spat->min, spat->avg, spat->max);
	t.setString(buf);
	t.setPosition(x_on_screen+2, y_on_screen+scale*EPC_YS+1*15);
	win.draw(t);

	sprintf(buf, "Point %d: val %4.0f", sel_at_point+1, spat->val_at[sel_at_point]);
	t.setString(buf);
	t.setPosition(x_on_screen+2, y_on_screen+scale*EPC_YS+2*15);
	win.draw(t);

	sprintf(buf, "  avg9 %6.1f  avg49 %6.1f  spatnoise49 %6.1f", spat->avg9_at[sel_at_point], spat->avg49_at[sel_at_point], spat->noise49_at[sel_at_point]);
	t.setString(buf);
	t.setPosition(x_on_screen+2, y_on_screen+scale*EPC_YS+3*15);
	win.draw(t);

}

#define sq(x) ((x)*(x))

void analyze_mono(float* img, spatial_analysis_t *spat)
{
	double min = 999999999.0;
	double max = -9999999999.0;
	double accum = 0.0;
	int accum_cnt = 0;
	for(int yy=0; yy<EPC_YS; yy++)
	{
		for(int xx=0; xx<EPC_XS; xx++)
		{
			double val = img[yy*EPC_XS+xx];
			accum += val;
			accum_cnt++;
			if(val > max) max = val;
			if(val < min) min = val; 
		}
	}
	spat->min = min;
	spat->max = max;
	spat->avg = accum/accum_cnt;

	for(int i=0; i<N_AT_POINTS; i++)
	{
		if(at_points[i][0] < 0 || at_points[i][0] >= EPC_XS || at_points[i][1] < 0 || at_points[i][1] >= EPC_YS)
			continue;

		// things ok with 1 pixel
		spat->val_at[i] = img[at_points[i][1]*EPC_XS+at_points[i][0]];


		if(at_points[i][0] < 1 || at_points[i][0] >= EPC_XS-1 || at_points[i][1] < 1 || at_points[i][1] >= EPC_YS-1)
			continue;

		// things requiring 9 pixel area:

		double accum9 = 0.0;
		for(int yy = at_points[i][1]-1; yy < at_points[i][1]+1; yy++)
			for(int xx = at_points[i][0]-1; xx < at_points[i][0]+1; xx++)
				accum9 += img[yy*EPC_XS+xx];

		spat->avg9_at[i] = accum9/9.0;
		
		if(at_points[i][0] < 3 || at_points[i][0] >= EPC_XS-3 || at_points[i][1] < 3 || at_points[i][1] >= EPC_YS-3)
			continue;

		// things requiring 49 pixel area:

		double accum49 = 0.0;
		for(int yy = at_points[i][1]-3; yy < at_points[i][1]+3; yy++)
		{
			for(int xx = at_points[i][0]-3; xx < at_points[i][0]+3; xx++)
			{
				accum49 += img[yy*EPC_XS+xx];
			}
		}
		double avg49 = accum9/49.0;
		spat->avg49_at[i] = avg49;

		double deviation_accum = 0.0;
		for(int yy = at_points[i][1]-3; yy < at_points[i][1]+3; yy++)
		{
			for(int xx = at_points[i][0]-3; xx < at_points[i][0]+3; xx++)
			{
				deviation_accum += sq(avg49-img[yy*EPC_XS+xx]);
			}
		}
		spat->noise49_at[i] = sqrt(deviation_accum/49.0);
	}

}

typedef struct
{
	char title[256];
	sf::Color title_color;
	float data[EPC_XS*EPC_YS];
	spatial_analysis_t spatial_analysis;
	void (*draw_func)(sf::RenderWindow&, float*, int, int, float);
	void (*analysis_func)(float*, spatial_analysis_t*);	
	void (*draw_analysis_func)(sf::RenderWindow&, spatial_analysis_t*, int, int, float);
} screen_img_t;

screen_img_t mono = {"Ambient light",sf::Color(255,255,255,255), {0}, {0}, &draw_mono, &analyze_mono, &draw_analysis};
screen_img_t dcs[4] = {
	{"DCS0",sf::Color(255,255,255,255), {0}, {0}, &draw_dcs, &analyze_mono, &draw_analysis},
	{"DCS1",sf::Color(255,255,255,255), {0}, {0}, &draw_dcs, &analyze_mono, &draw_analysis},
	{"DCS2",sf::Color(255,255,255,255), {0}, {0}, &draw_dcs, &analyze_mono, &draw_analysis},
	{"DCS3",sf::Color(255,255,255,255), {0}, {0}, &draw_dcs, &analyze_mono, &draw_analysis}};

screen_img_t dist_sw = {"Software distance",sf::Color(255,255,255,255), {0}, {0}, &draw_dist, &analyze_mono, &draw_analysis};

void calc_dist()
{
	const float fled = 20000000.0;
	const float mult = 299792458.0/2.0*1.0/(2.0*M_PI*fled);

	for(int i=0; i<EPC_XS*EPC_YS; i++)
	{
		float dcs3 = dcs[3].data[i];
		float dcs2 = dcs[2].data[i];
		float dcs1 = dcs[1].data[i];
		float dcs0 = dcs[0].data[i];

		float dcs31 = dcs3-dcs1;
		float dcs20 = dcs2-dcs0;

		float dist = (mult * (M_PI + atan2(dcs31,dcs20)))-2.500;
		if(dist < 0.0) dist = 0.0;

		float ampl = sqrt(sq(dcs20)+sq(dcs31))/2.0;


		if(ampl < 75)
			dist_sw.data[i] = DATA_LOW;
		else if(dcs0 < -2046.0 || dcs1 < -2046.0 || dcs2 < -2046.0 || dcs3 < -2046.0 || dcs0 > 2045.0 || dcs1 > 2045.0 || dcs2 > 2045.0 || dcs3 > 2045.0)
			dist_sw.data[i] = DATA_OVEREXP;
		else
			dist_sw.data[i] = dist*1000.0;
	}
}

void gen_test_img()
{
	mono.data[2*160+2] = 700;
	mono.data[30*160+80] = 1200;
	mono.data[58*160+158] = 2000;

	mono.analysis_func(mono.data, &mono.spatial_analysis);
}

void draw_screen_img(sf::RenderWindow& win, screen_img_t* img, int x, int y, float scale)
{
	img->draw_func(win, img->data, x, y, scale);
	img->draw_analysis_func(win, &img->spatial_analysis, x, y, scale);

	sf::Text t;
	t.setFont(arial);
	t.setString(img->title);
	t.setCharacterSize(14);
	t.setFillColor(img->title_color);
	t.setPosition(x+2, y+(rotated?EPC_XS:EPC_YS)*scale);
	win.draw(t);
}

void draw_horiz(sf::RenderWindow& win)
{
	float scale = 4.0;

	draw_screen_img(win, &mono, 10, 10, scale);
	draw_screen_img(win, &dist_sw, 10+scale*EPC_XS+20, 10, scale);
	draw_screen_img(win, &dcs[0], 10, 10+scale*EPC_YS+100, scale);
	draw_screen_img(win, &dcs[1], 10+scale*EPC_XS+20, 10+scale*EPC_YS+100, scale);
	draw_screen_img(win, &dcs[2], 10, 10+2.0*scale*EPC_YS+200, scale);
	draw_screen_img(win, &dcs[3], 10+scale*EPC_XS+20, 10+2.0*scale*EPC_YS+200, scale);
}

void draw_vert(sf::RenderWindow& win)
{
	float scale = 4.0;

	draw_screen_img(win, &mono,   10, 10, scale);
//	draw_screen_img(win, &dcs[0], 10+1.0*scale*EPC_YS+20, 10, scale);
//	draw_screen_img(win, &dcs[1], 10+2.0*scale*EPC_YS+40, 10, scale);
//	draw_screen_img(win, &dcs[2], 10+3.0*scale*EPC_YS+60, 10, scale);
//	draw_screen_img(win, &dcs[3], 10+4.0*scale*EPC_YS+80, 10, scale);
}

#define BW_MSG_SIZE ((EPC_XS)*(EPC_YS)*3/2)

int parse_uart_msg(uint8_t* buf, int msgid, int len)
{
	switch(msgid)
	{
		case 0x10:
		case 0x11:
		case 0x12:
		case 0x13:
		case 0x01:
		{
			if(BW_MSG_SIZE != len)
			{
				printf("Warning: ignoring image of wrong size (expected: %d, got: %d)\n", BW_MSG_SIZE, len);
				break;
			}

			screen_img_t *dest;

			if(msgid == 0x01) dest = &mono;
			else dest = &dcs[msgid-0x10];


			int i = 0;
			for(int yy=0; yy < 60; yy++)
			{
				for(int xx=0; xx < 160; xx+=2)
				{
					int val1 = ((uint16_t)buf[i])<<4 | ((uint16_t)buf[i+1]&0xf0)>>4;
					int val2 = ((uint16_t)buf[i+1]&0xf)<<8 | ((uint16_t)buf[i+2]);

					val1 -= 2048; val2 -= 2048;
					//if(val1 < 0) val1 = 0;
					//if(val2 < 0) val2 = 0;

					dest->data[yy*EPC_XS+xx] = val1;
					dest->data[yy*EPC_XS+xx+1] = val2;
					i+=3;
				}
			}

			dest->analysis_func(dest->data, &dest->spatial_analysis);

			if(msgid == 0x13)
			{
				calc_dist();
				dist_sw.analysis_func(dist_sw.data, &dist_sw.spatial_analysis);
			}

		}
		break;

		case 0xaa:
		break;

		default:
		{
			printf("unhandled msgid=%d len=%d\n", msgid, len);
		}
		break;

	}

	return 0;
}


int main(int argc, char** argv)
{
	gen_test_img();
	int focus = 1;
	#ifdef USE_UART
	if(init_uart())
	{
		fprintf(stderr, "uart initialization failed.\n");
		return 1;
	}
	#endif

/*
	while(1)
	{
		uint8_t buf[2] = {123, 55};
		send_uart(buf, 2);
		sleep(1);
	}
*/
	if(!arial.loadFromFile("arial.ttf"))
	{
	    return 1;
	}

	if(!monospace_regular.loadFromFile("liberationmono-regular.ttf"))
	{
	    return 1;
	}

	if(!monospace_bold.loadFromFile("liberationmono-bold.ttf"))
	{
	    return 1;
	}

	sf::ContextSettings sets;
	sets.antialiasingLevel = 8;
	sf::RenderWindow win(sf::VideoMode(screen_x,screen_y), "PULUROBOTICS 3DTOFFEE development system", sf::Style::Default, sets);
	win.setFramerateLimit(30);


	while(win.isOpen())
	{
		#ifdef USE_UART
		// Calculate fd_set size (biggest fd+1)
		int fds_size = uart+1;
		fd_set fds;
		FD_ZERO(&fds);
		FD_SET(uart, &fds);

		struct timeval select_time = {0, 200};

		if(select(fds_size, &fds, NULL, NULL, &select_time) < 0)
		{
			fprintf(stderr, "select() error %d", errno);
			return 1;
		}

		if(FD_ISSET(uart, &fds))
		{
			handle_uart();
		}
		#endif

		sf::Event event;
		while (win.pollEvent(event))
		{
			if(event.type == sf::Event::Closed)
				win.close();
			if(event.type == sf::Event::Resized)
			{
				sf::Vector2u size = win.getSize();
				screen_x = size.x;
				screen_y = size.y;
				sf::FloatRect visibleArea(0, 0, screen_x, screen_y);
				win.setView(sf::View(visibleArea));
			}
			if(event.type == sf::Event::LostFocus)
				focus = 0;

			if(event.type == sf::Event::GainedFocus)
				focus = 1;

			if(event.type == sf::Event::KeyPressed)
			{
				if(event.key.code == sf::Keyboard::X)
					mir_x = !mir_x;
				else if(event.key.code == sf::Keyboard::Y)
					mir_y = !mir_y;
				else if(event.key.code == sf::Keyboard::R)
					rotated = !rotated;
				else if(event.key.code == sf::Keyboard::Num1) sel_at_point = 0;
				else if(event.key.code == sf::Keyboard::Num2) sel_at_point = 1;
				else if(event.key.code == sf::Keyboard::Num3) sel_at_point = 2;
				else if(event.key.code == sf::Keyboard::Num4) sel_at_point = 3;
				else if(event.key.code == sf::Keyboard::Num5) sel_at_point = 4;
				else if(event.key.code == sf::Keyboard::Num6) sel_at_point = 5;
				else if(event.key.code == sf::Keyboard::Num7) sel_at_point = 6;
				else if(event.key.code == sf::Keyboard::Num8) sel_at_point = 7;
				else if(event.key.code == sf::Keyboard::Num9) sel_at_point = 8;
				else if(event.key.code == sf::Keyboard::Up)
				{
					int t = ((float)config.bw_int_len*1.41);
					if(t < 10) t = 10;
					else if(t > 16384) t = 16384;
					config.bw_int_len = t;
					send_hw_settings();
				}
				else if(event.key.code == sf::Keyboard::Down)
				{
					int t = ((float)config.bw_int_len/1.41);
					if(t < 10) t = 10;
					else if(t > 16384) t = 16384;
					config.bw_int_len = t;
					send_hw_settings();
				}
				else if(event.key.code == sf::Keyboard::Right)
				{
					int t = ((float)config.dist_int_len*1.41);
					if(t < 10) t = 10;
					else if(t > 16384) t = 16384;
					config.dist_int_len = t;
					send_hw_settings();
				}
				else if(event.key.code == sf::Keyboard::Left)
				{
					int t = ((float)config.dist_int_len/1.41);
					if(t < 10) t = 10;
					else if(t > 16384) t = 16384;
					config.dist_int_len = t;
					send_hw_settings();
				}
			}

		}


		//if(sf::Keyboard::isKeyPressed(sf::Keyboard::X))


		win.clear(sf::Color(128,128,128));
		if(rotated)
			draw_vert(win);
		else
			draw_horiz(win);



		sf::Text t;
		t.setFont(monospace_regular);
		char buf[256];
		t.setCharacterSize(14);
		t.setFillColor(sf::Color(255,255,255));

		sprintf(buf, "bw_int_len=%5d dist_int_len=%5d clk_div=%d\n", config.bw_int_len, config.dist_int_len, config.clk_div);
		t.setString(buf);
		t.setPosition(5, screen_y-15);
		win.draw(t);

		win.display();


	}


	return 0;
}

