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

bool show_software_dist = false;

#define USE_UART

bool saving = false;
int save_idx = 0;
int save_samples = 0;

FILE *csv;

float ampl_mask_lvl = 10.0;

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

float scaling = 4.0;

typedef struct __attribute__((packed))
{
	// *4, then -1 to form the register values:
	uint16_t bw_int_len; 
	uint16_t dist_int_len;

	uint8_t clk_div; // 1 = 20MHz LED, 2 = 10 MHz...
	uint8_t pll_shift; // delay in 12.5 ns/step , 0-12
	uint8_t dll_shift; // delay in approx 2ns/step, 0-49

	int16_t offsets[7];

} config_t;

config_t config =  // active configuration
{
	.bw_int_len = 1000,
	.dist_int_len = 1000,
	.clk_div = 1,
	.pll_shift = 0,
	.dll_shift = 0
};

// Software calculation offsets:
float offsets[7] =
{
	0,
	0, //-2.500,
	0, //-2.500-0.660,
	0, //-2.500-1.020,
	0, //-2.500-1.060,
	0,
	0, //-2.500-0.900
};


void save_settings();
void send_hw_settings()
{
	// conv SW offsets to HW offsets:
	for(int i=0; i<7; i++)
		config.offsets[i] = offsets[i]*1000.0;

	send_uart((uint8_t*)&config, sizeof(config_t));
	save_settings();

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

float amplitude_offset = 0.0;
float amplitude_div = 1.0;

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

			if(img[i] == DATA_OVEREXP)
			{
				pix[4*(yy*EPC_XS+xx)+0] = 160;
				pix[4*(yy*EPC_XS+xx)+1] = 0;
				pix[4*(yy*EPC_XS+xx)+2] = 160;
			}
			else
			{
				pix[4*(yy*EPC_XS+xx)+0] =
				pix[4*(yy*EPC_XS+xx)+1] =
				pix[4*(yy*EPC_XS+xx)+2] = (img[i]+mono_offset)/mono_div;
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

void draw_amplitude(sf::RenderWindow& win, float* img, int x_on_screen, int y_on_screen, float scale)
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
			pix[4*(yy*EPC_XS+xx)+2] = (img[i]+amplitude_offset)/amplitude_div;

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


float blue_dist = 3000.0;

void draw_dist(sf::RenderWindow& win, float* img, int x_on_screen, int y_on_screen, float scale);

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

			if(img[i] == DATA_OVEREXP)
			{
				pix[4*(yy*EPC_XS+xx)+0] = 160;
				pix[4*(yy*EPC_XS+xx)+1] = 0;
				pix[4*(yy*EPC_XS+xx)+2] = 160;
			}
			else
			{
				pix[4*(yy*EPC_XS+xx)+0] =
				pix[4*(yy*EPC_XS+xx)+1] =
				pix[4*(yy*EPC_XS+xx)+2] = ((int16_t)img[i]+dcs_offset)/dcs_div;
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
	int nvalid;

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


	if(!rotated)
	{
		sprintf(buf, "valids %4d   min %4.0f   avg %6.1f   max %4.0f", spat->nvalid, spat->min, spat->avg, spat->max);
		t.setString(buf);
		t.setPosition(x_on_screen+2, y_on_screen+scale*EPC_YS+0*15);
		win.draw(t);

		sprintf(buf, "P%d: val %4.0f  avg9 %6.1f  avg49 %6.1f  spatnoise49 %6.1f", sel_at_point+1, spat->val_at[sel_at_point], spat->avg9_at[sel_at_point], spat->avg49_at[sel_at_point], spat->noise49_at[sel_at_point]);
		t.setString(buf);
		t.setPosition(x_on_screen+2, y_on_screen+scale*EPC_YS+1*15);
		win.draw(t);
	}
	else
	{
		sprintf(buf, "valids %4d\nmin %4.0f\navg %6.1f\nmax %4.0f", spat->nvalid, spat->min, spat->avg, spat->max);
		t.setString(buf);
		t.setPosition(x_on_screen+2, y_on_screen+scale*EPC_XS+0*15);
		win.draw(t);

		sprintf(buf, "P%d: val %4.0f\navg9 %6.1f\navg49 %6.1f\nspatnoise49 %6.1f", sel_at_point+1, spat->val_at[sel_at_point], spat->avg9_at[sel_at_point], spat->avg49_at[sel_at_point], spat->noise49_at[sel_at_point]);
		t.setString(buf);
		t.setPosition(x_on_screen+2, y_on_screen+scale*EPC_XS+5*15);
		win.draw(t);

	}
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

			if(val == DATA_LOW || val == DATA_OVEREXP)
				continue;
			accum += val;
			accum_cnt++;
			if(val > max) max = val;
			if(val < min) min = val; 
		}
	}
	spat->nvalid = accum_cnt;
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
		accum_cnt = 0;
		for(int yy = at_points[i][1]-1; yy <= at_points[i][1]+1; yy++)
		{
			for(int xx = at_points[i][0]-1; xx <= at_points[i][0]+1; xx++)
			{
				double val = img[yy*EPC_XS+xx];
				if(val == DATA_LOW || val == DATA_OVEREXP)
					continue;
				accum_cnt++;
				accum9 += val;

			}
		}
		spat->avg9_at[i] = accum9/accum_cnt;
		
		if(at_points[i][0] < 3 || at_points[i][0] >= EPC_XS-3 || at_points[i][1] < 3 || at_points[i][1] >= EPC_YS-3)
			continue;

		// things requiring 49 pixel area:

		double accum49 = 0.0;
		accum_cnt = 0;
		for(int yy = at_points[i][1]-3; yy <= at_points[i][1]+3; yy++)
		{
			for(int xx = at_points[i][0]-3; xx <= at_points[i][0]+3; xx++)
			{
				double val = img[yy*EPC_XS+xx];
				if(val == DATA_LOW || val == DATA_OVEREXP)
					continue;
				accum_cnt++;
				accum49 += img[yy*EPC_XS+xx];
			}
		}
		double avg49 = accum49/accum_cnt;
		spat->avg49_at[i] = avg49;

		double deviation_accum = 0.0;
		accum_cnt = 0;
		for(int yy = at_points[i][1]-3; yy <= at_points[i][1]+3; yy++)
		{
			for(int xx = at_points[i][0]-3; xx <= at_points[i][0]+3; xx++)
			{
				double val = img[yy*EPC_XS+xx];
				if(val == DATA_LOW || val == DATA_OVEREXP)
					continue;
				accum_cnt++;
				deviation_accum += sq(avg49-img[yy*EPC_XS+xx]);
			}
		}
		spat->noise49_at[i] = sqrt(deviation_accum/accum_cnt);
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
	int x_on_screen;
	int y_on_screen;
	float scale;
} screen_img_t;

screen_img_t mono = {"Ambient light",sf::Color(255,255,255,255), {0}, {0}, &draw_mono, &analyze_mono, &draw_analysis};
screen_img_t dcs[4] = {
	{"DCS0",sf::Color(255,255,255,255), {0}, {0}, &draw_dcs, &analyze_mono, &draw_analysis},
	{"DCS1",sf::Color(255,255,255,255), {0}, {0}, &draw_dcs, &analyze_mono, &draw_analysis},
	{"DCS2",sf::Color(255,255,255,255), {0}, {0}, &draw_dcs, &analyze_mono, &draw_analysis},
	{"DCS3",sf::Color(255,255,255,255), {0}, {0}, &draw_dcs, &analyze_mono, &draw_analysis}};

screen_img_t dist_sw = {"Hardware distance",sf::Color(255,255,255,255), {0}, {0}, &draw_dist, &analyze_mono, &draw_analysis};
screen_img_t ampl_sw = {"Hardware amplitude",sf::Color(255,255,255,255), {0}, {0}, &draw_amplitude, &analyze_mono, &draw_analysis};

void save_analysis_header()
{
	if(!csv) return;

	fprintf(csv, "\nsave_idx;%d\nbw_int_len;%5d\ndist_int_len;%5d\nclk_div;%d;%5.2f\npll_shift;%2d;%5.2f\ndll_shift;%2d;%5.2f;\nblue_dist;%5.0f\noffset;%6d\nampl_mask;%5.1f\n",
		save_idx,
		config.bw_int_len, config.dist_int_len,
		config.clk_div, 20.0/config.clk_div,
		config.pll_shift, config.pll_shift*0.299792458*12.5,
		config.dll_shift, config.dll_shift*0.299792458*2.0,
		blue_dist, config.offsets[config.clk_div], ampl_mask_lvl);


	for(int j=0; j<2; j++)
	{
		static const char names[2][5] = {"DIST", "AMPL"};

		fprintf(csv, ";%s;nvalid;min;avg;max", names[j]);
		for(int i=0; i<N_AT_POINTS; i++)
		{
			if(at_points[i][0] < 0 || at_points[i][0] >= EPC_XS || at_points[i][1] < 0 || at_points[i][1] >= EPC_YS)
				continue;
			fprintf(csv, ";P%d;val;avg9;avg49;noise49", i);
		}
	}

	fprintf(csv, "\n");
}

void save_analysis_line()
{
	if(!csv) return;

	for(int j=0; j<2; j++)
	{
		spatial_analysis_t *a;
		if(j==0)
			a = &dist_sw.spatial_analysis;
		else
			a = &ampl_sw.spatial_analysis;

		static const char names[2][5] = {"DIST", "AMPL"};

		fprintf(csv, ";%s;%d;%.0f;%.1f;%.0f", names[j], a->nvalid, a->min, a->avg, a->max);
		for(int i=0; i<N_AT_POINTS; i++)
		{
			if(at_points[i][0] < 0 || at_points[i][0] >= EPC_XS || at_points[i][1] < 0 || at_points[i][1] >= EPC_YS)
				continue;
			fprintf(csv, ";P%d;%.0f;%.1f;%.1f;%.1f", i, a->val_at[i], a->avg9_at[i], a->avg49_at[i], a->noise49_at[i]);
		}
	}
	fprintf(csv, "\n");
	save_samples++;
}

#define NUM_SCREEN_IMGS 6
screen_img_t *screen_imgs[NUM_SCREEN_IMGS] =
{
	&mono,
	&dist_sw,
	&dcs[0],
	&ampl_sw, //&dcs[1],
	&dcs[2],
	&dcs[3],
};

bool use_ampl_img = false;
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
			if((use_ampl_img && (ampl_sw.data[i] == 0)) || (!use_ampl_img && img[i] == DATA_OVEREXP))
			{
				pix[4*(yy*EPC_XS+xx)+0] = 160;
				pix[4*(yy*EPC_XS+xx)+1] = 0;
				pix[4*(yy*EPC_XS+xx)+2] = 160;
			}
			else if((use_ampl_img && ampl_sw.data[i] < ampl_mask_lvl) || (!use_ampl_img && img[i] == DATA_LOW))
			{
				pix[4*(yy*EPC_XS+xx)+0] = 0;
				pix[4*(yy*EPC_XS+xx)+1] = 0;
				pix[4*(yy*EPC_XS+xx)+2] = 128;
			}
			else
			{
				float percolor = blue_dist/3.0;

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

int check_mouse_hit_mark(int ms, int my)
{
	for(int img_idx=0; img_idx < NUM_SCREEN_IMGS; img_idx++)
	{
		screen_img_t *img = screen_imgs[img_idx];
		for(int i=0; i<N_AT_POINTS; i++)
		{
			if(at_points[i][0] < 0 || at_points[i][0] >= EPC_XS || at_points[i][1] < 0 || at_points[i][1] >= EPC_YS)
				continue;

			float xprepre = at_points[i][0]+0.5;
			float yprepre = at_points[i][1]+0.5;

			float xpre = img->scale*(mir_x?(EPC_XS-xprepre):xprepre);
			float ypre = img->scale*(mir_y?(EPC_YS-yprepre):yprepre);
			float x,y;
			if(rotated)
			{
				x = img->scale*EPC_YS-ypre;
				y = xpre;
			}
			else
			{
				x = xpre;
				y = ypre;
			}

			if(ms > (img->x_on_screen+x)-5 && ms < (img->x_on_screen+x)+5 &&
			   my > (img->y_on_screen+y)-5 && my < (img->y_on_screen+y)+5)
			{
				printf("Click in imgidx=%d on point %d\n", img_idx, i);
				return i;
			}
		}
	}

	return -1;
}


void calc_dist()
{
	if(config.clk_div < 1 || config.clk_div > 6)
	{
		printf("Illegal clk_div\n");
		return;
	}

	float fled = 20000000.0 / config.clk_div;
	float mult = 299792458.0/2.0*1.0/(2.0*M_PI*fled); 
	float unamb = 299792458.0/2.0*1.0/(fled); 

	for(int i=0; i<EPC_XS*EPC_YS; i++)
	{
		float dcs3 = dcs[3].data[i];
		float dcs2 = dcs[2].data[i];
		float dcs1 = dcs[1].data[i];
		float dcs0 = dcs[0].data[i];

		float dcs31 = dcs3-dcs1;
		float dcs20 = dcs2-dcs0;

		float dist = (mult * (/*M_PI +*/ atan2(dcs31,dcs20)))+offsets[config.clk_div];

		if(dist < 0.0) dist += unamb;
		else if(dist > unamb) dist -= unamb;

		float ampl = sqrt(sq(dcs20)+sq(dcs31))/2.0;


		if(dcs0 < -2046.0 || dcs1 < -2046.0 || dcs2 < -2046.0 || dcs3 < -2046.0 || dcs0 > 2045.0 || dcs1 > 2045.0 || dcs2 > 2045.0 || dcs3 > 2045.0)
			dist_sw.data[i] = DATA_OVEREXP;
		else if(ampl < 75)
			dist_sw.data[i] = DATA_LOW;
		else
		{
			dist_sw.data[i] = dist*1000.0;
		}
	}
}

void addj_offset(float amount)
{
	if(config.clk_div < 1 || config.clk_div > 6)
	{
		printf("Illegal clk_div\n");
		return;
	}

	offsets[config.clk_div] += amount;

	calc_dist();
	dist_sw.analysis_func(dist_sw.data, &dist_sw.spatial_analysis);

	send_hw_settings();

}

void gen_test_img()
{
	mono.data[2*160+2] = 700;
	mono.data[30*160+80] = 1200;
	mono.data[58*160+158] = 2000;

	mono.analysis_func(mono.data, &mono.spatial_analysis);
}

void draw_screen_img(sf::RenderWindow& win, screen_img_t* img)
{
	img->draw_func(win, img->data, img->x_on_screen, img->y_on_screen, img->scale);
	img->draw_analysis_func(win, &img->spatial_analysis, img->x_on_screen, img->y_on_screen, img->scale);

	sf::Text t;
	t.setFont(arial);
	t.setString(img->title);
	t.setCharacterSize(14);
	t.setFillColor(img->title_color);
	t.setPosition(img->x_on_screen+2, img->y_on_screen+(rotated?EPC_XS:EPC_YS)*img->scale-15);
	win.draw(t);
}

void draw_horiz(sf::RenderWindow& win)
{
	float scale = scaling;

//	screen_imgs[0]->x_on_screen = screen_imgs[2]->x_on_screen = screen_imgs[4]->x_on_screen = 10;
//	screen_imgs[1]->x_on_screen = screen_imgs[3]->x_on_screen = screen_imgs[5]->x_on_screen = 10+scale*EPC_XS+20;

//	mono.y_on_screen = dist_sw.y_on_screen = 10;
//	dcs[0].y_on_screen = dcs[1].y_on_screen = 10+scale*EPC_YS+70;
//	dcs[2].y_on_screen = dcs[3].y_on_screen = 10+2.0*(scale*EPC_YS+70);

//	mono.scale = dist_sw.scale = dcs[0].scale = dcs[1].scale = dcs[2].scale = dcs[3].scale = scale;


	for(int img_idx=0; img_idx < NUM_SCREEN_IMGS; img_idx++)
	{
		screen_imgs[img_idx]->x_on_screen = 10 + (img_idx%2)*(scale*EPC_XS+20);
		screen_imgs[img_idx]->y_on_screen = 10 + (img_idx/2)*(scale*EPC_YS+70);
		screen_imgs[img_idx]->scale = scale;
		draw_screen_img(win, screen_imgs[img_idx]);
	}
}

void draw_vert(sf::RenderWindow& win)
{
	float scale = scaling;

	for(int img_idx=0; img_idx < NUM_SCREEN_IMGS; img_idx++)
	{
		screen_imgs[img_idx]->x_on_screen = 10+img_idx*(scale*EPC_YS+15);
		screen_imgs[img_idx]->y_on_screen = 10;
		screen_imgs[img_idx]->scale = scale;

		draw_screen_img(win, screen_imgs[img_idx]);
	}


/*
	draw_screen_img(win, &mono, 10, 10, scale);
	draw_screen_img(win, &dist_sw, 10+1*(scale*EPC_YS+15), 10, scale);
	draw_screen_img(win, &dcs[0],  10+2*(scale*EPC_YS+15), 10, scale);
	draw_screen_img(win, &dcs[1],  10+3*(scale*EPC_YS+15), 10, scale);
	draw_screen_img(win, &dcs[2],  10+4*(scale*EPC_YS+15), 10, scale);
	draw_screen_img(win, &dcs[3],  10+5*(scale*EPC_YS+15), 10, scale);
*/

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

					if(val1 < -2047 || val1 > 2046) val1 = DATA_OVEREXP;
					if(val2 < -2047 || val2 > 2046) val2 = DATA_OVEREXP;

					dest->data[yy*EPC_XS+xx] = val1;
					dest->data[yy*EPC_XS+xx+1] = val2;
					i+=3;
				}
			}

			dest->analysis_func(dest->data, &dest->spatial_analysis);

			if(msgid == 0x13)
			{
				if(!show_software_dist) break;

				calc_dist();
				dist_sw.analysis_func(dist_sw.data, &dist_sw.spatial_analysis);
			}

		}
		break;

		case 0x80:
		{
			if(show_software_dist) break;

			screen_img_t *dest = &dist_sw;

			int i = 0;
			for(int yy=0; yy < 60; yy++)
			{
				for(int xx=0; xx < 160; xx++)
				{
					int val = ((uint16_t)buf[i+1])<<8 | ((uint16_t)buf[i]);
					dest->data[yy*EPC_XS+xx] = val;
					i+=2;
				}
			}

			printf("Calc time = %.1f ms\n", (float)(((uint16_t)buf[i+1])<<8 | ((uint16_t)buf[i]))*0.1);
			i+=2;

			dest->analysis_func(dest->data, &dest->spatial_analysis);

		}
		break;

		case 0x81:
		{
			if(show_software_dist) break;

			use_ampl_img = true; // start using it if received once.

			screen_img_t *dest = &ampl_sw;

			int i = 0;
			for(int yy=0; yy < 60; yy++)
			{
				for(int xx=0; xx < 160; xx++)
				{
					dest->data[yy*EPC_XS+xx] = buf[i];
					i++;
				}
			}

			dest->analysis_func(dest->data, &dest->spatial_analysis);

			if(saving) save_analysis_line();


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

void save_settings()
{
	FILE* f_save = fopen("saved_settings", "w");
	if(f_save)
	{
		fwrite(offsets, sizeof(offsets), 1, f_save);
		fwrite(&config, sizeof(config), 1, f_save);
		fwrite(&blue_dist, sizeof(blue_dist), 1, f_save);
		fwrite(&ampl_mask_lvl, sizeof(ampl_mask_lvl), 1, f_save);
		fwrite(&at_points, sizeof(at_points), 1, f_save);
		fclose(f_save);
	}
}

void restore_settings()
{
	FILE* f_save = fopen("saved_settings", "r");
	if(f_save)
	{
		fread(offsets, sizeof(offsets), 1, f_save);
		fread(&config, sizeof(config), 1, f_save);
		fread(&blue_dist, sizeof(blue_dist), 1, f_save);
		fread(&ampl_mask_lvl, sizeof(ampl_mask_lvl), 1, f_save);
		fread(&at_points, sizeof(at_points), 1, f_save);
		fclose(f_save);
	}
}

int main(int argc, char** argv)
{

	csv = fopen("measurements.csv", "a");

	if(!csv)
	{
		printf("Warn: couldn't open measurements.csv for write, cannot save measurements\n");
	}

	restore_settings();

	gen_test_img();
//	int focus = 1;
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
/*			if(event.type == sf::Event::LostFocus)
				focus = 0;

			if(event.type == sf::Event::GainedFocus)
				focus = 1;
*/
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
					if(sf::Keyboard::isKeyPressed(sf::Keyboard::LShift) || sf::Keyboard::isKeyPressed(sf::Keyboard::RShift) )
					{
						int t = ((float)config.bw_int_len*1.189207115);
						if(t < 10) t = 10;
						else if(t > 16384) t = 16384;
						config.bw_int_len = t;
					}
					else
					{
						int t = ((float)config.dist_int_len*1.189207115);
						if(t < 10) t = 10;
						else if(t > 16384) t = 16384;
						config.dist_int_len = t;
					}
					send_hw_settings();
				}
				else if(event.key.code == sf::Keyboard::Down)
				{
					if(sf::Keyboard::isKeyPressed(sf::Keyboard::LShift) || sf::Keyboard::isKeyPressed(sf::Keyboard::RShift) )
					{
						int t = ((float)config.bw_int_len/1.189207115);
						if(t < 10) t = 10;
						else if(t > 16384) t = 16384;
						config.bw_int_len = t;
					}
					else
					{
						//int t = ((float)config.dist_int_len/1.189207115); // 1/4 stops
						int t = ((float)config.dist_int_len/1.090507733); // 1/8 stops
						if(t < 10) t = 10;
						else if(t > 16384) t = 16384;
						config.dist_int_len = t;
					}
					send_hw_settings();
				}
				else if(event.key.code == sf::Keyboard::F11)
				{
					addj_offset((sf::Keyboard::isKeyPressed(sf::Keyboard::LShift) || sf::Keyboard::isKeyPressed(sf::Keyboard::RShift))?+0.02:+0.2);
					save_settings();

				}
				else if(event.key.code == sf::Keyboard::F10)
				{
					addj_offset((sf::Keyboard::isKeyPressed(sf::Keyboard::LShift) || sf::Keyboard::isKeyPressed(sf::Keyboard::RShift))?-0.02:-0.2);
					save_settings();
				}
				else if(event.key.code == sf::Keyboard::Right)
				{
					ampl_mask_lvl*=1.41;
				}
				else if(event.key.code == sf::Keyboard::Left)
				{
					ampl_mask_lvl/=1.41;
				}
				else if(event.key.code == sf::Keyboard::F1)
				{
					config.clk_div = 1;
					send_hw_settings();
				}
				else if(event.key.code == sf::Keyboard::F2)
				{
					config.clk_div = 2;
					send_hw_settings();
				}
				else if(event.key.code == sf::Keyboard::F3)
				{
					config.clk_div = 3;
					send_hw_settings();
				}
				else if(event.key.code == sf::Keyboard::F4)
				{
					config.clk_div = 4;
					send_hw_settings();
				}
				else if(event.key.code == sf::Keyboard::F5)
				{
					config.clk_div = 6;
					send_hw_settings();
				}
				else if(event.key.code == sf::Keyboard::PageUp)
				{
					if(sf::Keyboard::isKeyPressed(sf::Keyboard::LShift) || sf::Keyboard::isKeyPressed(sf::Keyboard::RShift) )
						scaling *= 1.025;
					else
						blue_dist += 500.0;
				}
				else if(event.key.code == sf::Keyboard::PageDown)
				{
					if(sf::Keyboard::isKeyPressed(sf::Keyboard::LShift) || sf::Keyboard::isKeyPressed(sf::Keyboard::RShift) )
						scaling /= 1.025;
					else
						blue_dist -= 500.0;
				}
				else if(event.key.code == sf::Keyboard::Comma)
				{
					if(sf::Keyboard::isKeyPressed(sf::Keyboard::LShift) || sf::Keyboard::isKeyPressed(sf::Keyboard::RShift) )
					{ if(config.dll_shift > 0) config.dll_shift--; }
					else
					{ if(config.pll_shift > 0) config.pll_shift--; }
					send_hw_settings();

				}
				else if(event.key.code == sf::Keyboard::Period)
				{
					if(sf::Keyboard::isKeyPressed(sf::Keyboard::LShift) || sf::Keyboard::isKeyPressed(sf::Keyboard::RShift) )
					{ if(config.dll_shift < 49) config.dll_shift++; }
					else
					{ if(config.pll_shift < 12) config.pll_shift++; }
					send_hw_settings();
				}
				else if(event.key.code == sf::Keyboard::BackSpace)
				{
					at_points[sel_at_point][0] = EPC_XS/2;
					at_points[sel_at_point][1] = EPC_YS/2;
				}
				else if(event.key.code == sf::Keyboard::F12)
				{
					if(show_software_dist)
					{
						show_software_dist = false;
						strcpy(dist_sw.title, "Hardware distance");
					}
					else
					{
						show_software_dist = true;
						strcpy(dist_sw.title, "Software distance");
					}
				}
				else if(event.key.code == sf::Keyboard::F9)
				{
					if(!saving && csv)
					{
						save_idx++;
						save_samples = 0;
						saving = true;
						save_analysis_header();
					}
					else
					{
						saving = false;
						fprintf(csv, "\n\n");
					}
				}

			}

		}


		sf::Vector2i mouse_pos = sf::Mouse::getPosition(win);

		static bool mouse_was_pressed = false;
		static int drag_idx = -1;
		static int drag_start_x, drag_start_y;
		if(sf::Mouse::isButtonPressed(sf::Mouse::Left))
		{
			if(!mouse_was_pressed)
			{
				if( (drag_idx = check_mouse_hit_mark(mouse_pos.x, mouse_pos.y)) >= 0 )
				{
					drag_start_x = mouse_pos.x;
					drag_start_y = mouse_pos.y;
				}
			}
			else
			{

				if(drag_idx >= 0)
				{

					int dx = mouse_pos.x - drag_start_x;
					int dy = mouse_pos.y - drag_start_y;

					if(rotated)
					{
						int tmp = -1*dx;
						dx = dy;
						dy = tmp;
					}


					at_points[drag_idx][0] += (mir_x?-1:1)*dx;
					at_points[drag_idx][1] += (mir_y?-1:1)*dy;

					drag_start_x = mouse_pos.x;
					drag_start_y = mouse_pos.y;
				
				}

			}
			mouse_was_pressed = true;
		}
		else
		{
			mouse_was_pressed = false;

			drag_idx = -1;
		}

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

		sprintf(buf, "bw_int_len=%5d dist_int_len=%5d clk_div=%d (%5.2fMHz) pll_shift=%2d(%5.2fm) dll_shift=%2d(~%5.2fm) blue_dist=%5.0f offset=%6d  ampl_mask=%5.1f", 
			config.bw_int_len, config.dist_int_len,
			config.clk_div, 20.0/config.clk_div,
			config.pll_shift, config.pll_shift*0.299792458*12.5,
			config.dll_shift, config.dll_shift*0.299792458*2.0,
			blue_dist, config.offsets[config.clk_div], ampl_mask_lvl);
		t.setString(buf);
		t.setPosition(5, screen_y-17);
		win.draw(t);


		if(saving)
		{
			t.setCharacterSize(16);
			t.setFillColor(sf::Color(255,0,0));

			sprintf(buf, "saving with idx=%d, samples=%d", save_idx, save_samples);
			t.setString(buf);
			t.setPosition(5, screen_y-33);
			win.draw(t);
		}

		win.display();


	}

	save_settings();

	if(csv) fclose(csv);
	return 0;
}

