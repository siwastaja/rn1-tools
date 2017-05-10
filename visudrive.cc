#include <stdint.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <stropts.h>
#include <sys/select.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <cstdio>
#include <cmath>
#include <cstring>
#include <SFML/Graphics.hpp>
#include <SFML/Network.hpp>

int dev_apply = 0;

//#define LOG_RX

#ifndef M_PI
#define M_PI 3.14159265358
#endif

#include "../rn1-brain/comm.h"

#define I14x2_I16(msb,lsb) ((int16_t)( ( ((uint16_t)(msb)<<9) | ((uint16_t)(lsb)<<2) ) ))

sf::Font arial;

int manual_control = 1;
int control_on = 0;

double mm_per_pixel = 10.0;

//int screen_x = 1024;
//int screen_y = 768;
int screen_x = 1200;
int screen_y = 900;

double origin_x = ((double)screen_x/2.0)*mm_per_pixel;
double origin_y = ((double)screen_y/2.0)*mm_per_pixel;

double robot_xs = 480.0;
double robot_ys = 524.0;
double robot_xs2 = 470.0;
double robot_ys2 = 510.0;
double lidar_xoffs = 120.0;
double lidar_yoffs = 0.0;

double cur_x = 0.0;
double cur_y = 0.0;
double cur_angle = 0.0;
double cur_x_at_lidar_update;
double cur_y_at_lidar_update;
double cur_angle_at_lidar_update;
double cur_compass;
double north_corr = 0.0;

double bat_voltage = 0.0;

const double lidar_line_thick = 2.0;
const double old_lidar_line_thick = 1.0;
const double sonar_line_thick = 4.0;

int speed = 0;
int angle_cmd = 0;

double int_x, int_y, opt_q;

int d1;

int32_t dbg[10];

int auto_angle = 0;
int auto_fwd = 0;

int lidar_status = 0;

int charging = 0;
int charge_finished = 0;

typedef struct
{
	int32_t angle;
	int32_t x;
	int32_t y;
	int scan[360];
} lidar_scan_t;

lidar_scan_t dev_lidar_bef_in;
lidar_scan_t dev_lidar_aft_in;
lidar_scan_t dev_lidar_aft_out;
lidar_scan_t dev_lidar_aft_out_alt;
int dev_show_lidars;

void read_lidar(FILE* f, lidar_scan_t* l)
{
	fscanf(f, "%d %d %d", &l->angle, &l->x, &l->y);
	for(int i = 0; i < 360; i++)
	{
		int t;
		fscanf(f, "%d", &t);
		l->scan[i] = t;
	}
}


void write_lidar(FILE* f, lidar_scan_t* l)
{
	fprintf(f, "%d %d %d ", l->angle, l->x, l->y);
	for(int i = 0; i < 360; i++)
	{
		fprintf(f, "%d ", l->scan[i]);
	}	
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
sf::IpAddress serv_ip;
unsigned short serv_port;
unsigned short our_port;
sf::UdpSocket udpsock;

uint8_t internal_txbuf[129];
uint8_t* txbuf;

void snd(int len)
{
	if(len < 3 || len > 128)
	{
		printf("Illegal len\n");
		return;
	}

	if(uart)
	{
		if(write(uart, txbuf, len) != len)
		{
			printf("uart write error\n");
		}
	}
	else // udp
	{
		if(udpsock.send(internal_txbuf, len+1, serv_ip, serv_port) != sf::Socket::Done)
		{
			printf("udp send error\n");
		}
	}
}

void draw_mini_robot(sf::RenderWindow& win, sf::Color color, double ang, double x, double y)
{
	sf::ConvexShape r(5);
	r.setPoint(0, sf::Vector2f(0,0));
	r.setPoint(1, sf::Vector2f(0,0.2*robot_ys/mm_per_pixel));
	r.setPoint(2, sf::Vector2f(0.2*robot_xs/mm_per_pixel,0.2*robot_ys/mm_per_pixel));
	r.setPoint(3, sf::Vector2f(0.2*1.3*robot_xs/mm_per_pixel,0.2*0.5*robot_ys/mm_per_pixel));
	r.setPoint(4, sf::Vector2f(0.2*robot_xs/mm_per_pixel,0));

	r.setFillColor(color);
	r.setOrigin(0.2*0.5*robot_xs/mm_per_pixel,0.2*0.5*robot_ys/mm_per_pixel);

	r.setRotation(ang-north_corr);
	r.setPosition((x+origin_x)/mm_per_pixel,(y+origin_y)/mm_per_pixel);

	win.draw(r);
}

void draw_robot(sf::RenderWindow& win)
{
	sf::ConvexShape r(5);
	r.setPoint(0, sf::Vector2f(0,0));
	r.setPoint(1, sf::Vector2f(0,robot_ys/mm_per_pixel));
	r.setPoint(2, sf::Vector2f(robot_xs/mm_per_pixel,robot_ys/mm_per_pixel));
	r.setPoint(3, sf::Vector2f(1.3*robot_xs/mm_per_pixel,0.5*robot_ys/mm_per_pixel));
	r.setPoint(4, sf::Vector2f(robot_xs/mm_per_pixel,0));

	r.setFillColor(sf::Color(200,135,135));
	r.setOrigin((0.5*robot_xs+lidar_xoffs)/mm_per_pixel,(0.5*robot_ys+lidar_yoffs)/mm_per_pixel);

//	r.setRotation(cur_compass-north_corr);
//	r.setPosition((cur_x+origin_x)/mm_per_pixel,(cur_y+origin_y)/mm_per_pixel);

//	win.draw(r);

	r.setFillColor(sf::Color(180,100,70));

	r.setRotation(cur_angle-north_corr);
	r.setPosition((cur_x+origin_x)/mm_per_pixel,(cur_y+origin_y)/mm_per_pixel);

	win.draw(r);

/*	sf::ConvexShape r2(4);
	r2.setPoint(0, sf::Vector2f(0,0));
	r2.setPoint(1, sf::Vector2f(0,robot_ys2/mm_per_pixel));
	r2.setPoint(2, sf::Vector2f(robot_xs2/mm_per_pixel,robot_ys2/mm_per_pixel));
	r2.setPoint(3, sf::Vector2f(robot_xs2/mm_per_pixel,0));

	r2.setFillColor(sf::Color(230,70,50));
	r.setOrigin((0.5*robot_xs2+lidar_xoffs)/mm_per_pixel,(0.5*robot_ys2+lidar_yoffs)/mm_per_pixel);

	r2.setRotation(cur_angle-north_corr);
	r2.setPosition((cur_x+origin_x)/mm_per_pixel,(cur_y+origin_y)/mm_per_pixel);

	win.draw(r2);
*/

	if(!manual_control)
	{
		if(auto_fwd != 0 || auto_angle != 0)
		{
			// Draw robot at destination

			r.setFillColor(sf::Color(0,255,0,128));

			r.setRotation(cur_angle-north_corr+auto_angle);

			double xmove = cos(M_PI*((double)(cur_angle-north_corr+auto_angle))/180.0) * (double)auto_fwd;
			double ymove = sin(M_PI*((double)(cur_angle-north_corr+auto_angle))/180.0) * (double)auto_fwd;

			r.setPosition((cur_x+origin_x+xmove)/mm_per_pixel,(cur_y+origin_y+ymove)/mm_per_pixel);

			win.draw(r);

			// Draw line to dest

			double x1 = (cur_x+origin_x)/mm_per_pixel;
			double y1 = (cur_y+origin_y)/mm_per_pixel;
			double x2 = (cur_x+origin_x+xmove)/mm_per_pixel;
			double y2 = (cur_y+origin_y+ymove)/mm_per_pixel;
			sf::RectangleShape rect(sf::Vector2f( sqrt(pow(x2-x1,2)+pow(y2-y1,2)), 10.0));
			rect.setOrigin(0, 5.0);
			rect.setPosition(x1, y1);
			rect.setRotation(atan2(y2-y1,x2-x1)*180.0/M_PI);
			rect.setFillColor(sf::Color(0,0,0,90));

			win.draw(rect);

		}

		sf::Text t;
		t.setFont(arial);
		char buf[128];
		sprintf(buf, "%d", auto_angle);
		t.setString(buf);
		t.setCharacterSize(14);
		t.setColor(sf::Color(0,0,0));
		t.setPosition((cur_x+origin_x)/mm_per_pixel-10.0,((cur_y+origin_y)/mm_per_pixel)+20.0);
		win.draw(t);

		sprintf(buf, "%d", auto_fwd);
		t.setString(buf);
		t.setCharacterSize(16);
		t.setColor(sf::Color(0,0,0));
		t.setPosition((cur_x+origin_x)/mm_per_pixel-10.0,((cur_y+origin_y)/mm_per_pixel)-20.0);
		win.draw(t);

	}

}

#define NUM_LIDAR_SNAPSHOTS 3

int lidar_scan[360];
int lidar_snapshots[NUM_LIDAR_SNAPSHOTS][360];
double ang_at_snapshot[NUM_LIDAR_SNAPSHOTS];
double x_at_snapshot[NUM_LIDAR_SNAPSHOTS];
double y_at_snapshot[NUM_LIDAR_SNAPSHOTS];
int special_idx_at_snapshot[NUM_LIDAR_SNAPSHOTS];

static const sf::Color special_lidar_colors[4] =
{
	sf::Color(0, 0, 0, 100),
	sf::Color(255, 0, 0, 150),
	sf::Color(0, 255, 0, 170),
	sf::Color(0, 0, 255, 70)
};

int sonars[3] = {1000,1000,1000};
int sonar_angles[3] = {-8, 0, 8};

void take_lidar_snapshot(int special_idx)
{
	static int ring_idx = 0;

	special_idx_at_snapshot[ring_idx] = special_idx;
	memcpy(lidar_snapshots[ring_idx], lidar_scan, 360*sizeof(int));
	ang_at_snapshot[ring_idx] = cur_angle_at_lidar_update;
	x_at_snapshot[ring_idx] = cur_x_at_lidar_update;
	y_at_snapshot[ring_idx] = cur_y_at_lidar_update;
	if(++ring_idx >= NUM_LIDAR_SNAPSHOTS) ring_idx = 0;
}

void draw_lidar(sf::RenderWindow& win, int* lid_data, sf::Color color, float thick, double ang, double x, double y)
{
	for(int i = 0; i < 360; i++)
	{
		int ip = (i==359)?0:(i+1);
		int first = lid_data[i];
		int second = lid_data[ip];

		if(first == 0 && second == 0)
			continue;

		if(first == 0) first = second;
		else if(second == 0) second = first;

		if(abs(first-second) > 200)
			continue;

		double x1 = (x+origin_x+cos(M_PI*(ang-north_corr+(double)i)/180.0) * first)/mm_per_pixel;
		double y1 = (y+origin_y+sin(M_PI*(ang-north_corr+(double)i)/180.0) * first)/mm_per_pixel;
		double x2 = (x+origin_x+cos(M_PI*(ang-north_corr+(double)ip)/180.0) * second)/mm_per_pixel;
		double y2 = (y+origin_y+sin(M_PI*(ang-north_corr+(double)ip)/180.0) * second)/mm_per_pixel;
		sf::RectangleShape rect(sf::Vector2f( sqrt(pow(x2-x1,2)+pow(y2-y1,2)), thick));
		rect.setOrigin(0, thick/2.0);
		rect.setPosition(x1, y1);
		rect.setRotation(atan2(y2-y1,x2-x1)*180.0/M_PI);
		rect.setFillColor(color);

		win.draw(rect);
	}
}

void draw_lidar_quick(sf::RenderWindow& win, int* lid_data, sf::Color color, double ang, double x, double y)
{
	for(int i = 4; i < 354; i+=4)
	{
		int ip = (i+4);
		int first = lid_data[i];
		int second = lid_data[ip];

		if(first == 0 && second == 0)
			continue;

		if(first == 0) first = second;
		else if(second == 0) second = first;

		if(abs(first-second) > 200)
			continue;

		double x1 = (x+origin_x+cos(M_PI*(ang-north_corr+(double)i)/180.0) * first)/mm_per_pixel;
		double y1 = (y+origin_y+sin(M_PI*(ang-north_corr+(double)i)/180.0) * first)/mm_per_pixel;
		double x2 = (x+origin_x+cos(M_PI*(ang-north_corr+(double)ip)/180.0) * second)/mm_per_pixel;
		double y2 = (y+origin_y+sin(M_PI*(ang-north_corr+(double)ip)/180.0) * second)/mm_per_pixel;
		sf::RectangleShape rect(sf::Vector2f( sqrt(pow(x2-x1,2)+pow(y2-y1,2)), old_lidar_line_thick));
		rect.setOrigin(0, old_lidar_line_thick/2.0);
		rect.setPosition(x1, y1);
		rect.setRotation(atan2(y2-y1,x2-x1)*180.0/M_PI);
		rect.setFillColor(color);

		win.draw(rect);
	}
}


void draw_sonars(sf::RenderWindow& win)
{
	for(int i = 0; i < 360; i++)
	{
		for(int so = 0; so < 3; so++)
		{
			if(!sonars[so])
				continue;

			int angle_comp = (int)cur_angle + sonar_angles[so];
//			printf("i=%d  cur_angle=%f  angle_comp=%d\n", i, cur_angle, angle_comp);

			if(angle_comp < 0) angle_comp += 360;
			if(angle_comp > 359) angle_comp -= 360;
			if(i == angle_comp)
			{
				double a = angle_comp;
				int in2 = i-4;
				int ip2 = i+4;
				if(in2<0) in2+=360;
				if(ip2>359) ip2-=360;

				double x1 = (cur_x+origin_x+cos(M_PI*(/*a*/-north_corr+(double)in2)/180.0) * (sonars[so]+(int)robot_ys/2))/mm_per_pixel;
				double y1 = (cur_y+origin_y+sin(M_PI*(/*a*/-north_corr+(double)in2)/180.0) * (sonars[so]+(int)robot_ys/2))/mm_per_pixel;
				double x2 = (cur_x+origin_x+cos(M_PI*(/*a*/-north_corr+(double)ip2)/180.0) * (sonars[so]+(int)robot_ys/2))/mm_per_pixel;
				double y2 = (cur_y+origin_y+sin(M_PI*(/*a*/-north_corr+(double)ip2)/180.0) * (sonars[so]+(int)robot_ys/2))/mm_per_pixel;
				sf::RectangleShape son1(sf::Vector2f( sqrt(pow(x2-x1,2)+pow(y2-y1,2)), sonar_line_thick));
				son1.setOrigin(0, sonar_line_thick/2.0);
				son1.setPosition(x1, y1);
				son1.setRotation(atan2(y2-y1,x2-x1)*180.0/M_PI);
				son1.setFillColor(sf::Color(10,10,128));

				win.draw(son1);

			}
		}

	}
}

double gyro_x, gyro_y, gyro_z, xcel_x, xcel_y, xcel_z, compass_x, compass_y, compass_z;

void draw_texts(sf::RenderWindow& win)
{
	sf::RectangleShape rect(sf::Vector2f(4,4));
	rect.setOrigin(2,2);
	rect.setPosition(origin_x/mm_per_pixel, origin_y/mm_per_pixel);
	rect.setFillColor(sf::Color(0,0,255));
	win.draw(rect);


	sf::Text t;
	t.setFont(arial);
	char buf[500];
	sprintf(buf, "Gyro x=%f  y=%f  z=%f", gyro_x, gyro_y, gyro_z);
	t.setString(buf);
	t.setCharacterSize(16);
	t.setColor(sf::Color(0,0,0));
	t.setPosition(10,10);
	win.draw(t);

	sprintf(buf, "Xcel x=%f  y=%f  z=%f", xcel_x, xcel_y, xcel_z);
	t.setString(buf);
	t.setCharacterSize(16);
	t.setColor(sf::Color(0,0,0));
	t.setPosition(10,10+22);
	win.draw(t);

	sprintf(buf, "Compass x=%f  y=%f  z=%f", compass_x, compass_y, compass_z);
	t.setString(buf);
	t.setCharacterSize(16);
	t.setColor(sf::Color(0,0,0));
	t.setPosition(10,10+2*22);
	win.draw(t);


	sprintf(buf, "speed_cmd = %d  angle_cmd = %d", speed, angle_cmd);
	t.setString(buf);
	t.setCharacterSize(16);
	t.setColor(sf::Color(0,0,0));
	t.setPosition(10,10+3*22);
	win.draw(t);


	sprintf(buf, "angle = %.0f, north_corr = %.0f", cur_angle, north_corr);
	t.setString(buf);
	t.setCharacterSize(16);
	t.setColor(sf::Color(0,0,0));
	t.setPosition(10,10+4*22);
	win.draw(t);

	sprintf(buf, "opt flow x = %.0f, y = %.0f,  opt_quality=%.0f", int_x, int_y, opt_q);
	t.setString(buf);
	t.setCharacterSize(16);
	t.setColor(sf::Color(0,0,0));
	t.setPosition(10,10+5*22);
	win.draw(t);

	sprintf(buf, "d1 = %d", d1);
	t.setString(buf);
	t.setCharacterSize(16);
	t.setColor(sf::Color(0,0,0));
	t.setPosition(10,10+6*22);
	win.draw(t);

	t.setCharacterSize(12);
	t.setColor(sf::Color(0,0,0));
	for(int i = 0; i<10; i++)
	{
//		if(i==2 || i==3) continue;
		sprintf(buf, "dbg[%2i] = %11d (%08x)", i, dbg[i], dbg[i]);
		t.setString(buf);
		t.setPosition(10,screen_y-170 + 15*i);
		win.draw(t);
	}


	sprintf(buf, "son1 = %d  son2 = %d  son3 = %d  lidar_status=%02xh", sonars[0], sonars[1], sonars[2], lidar_status);
	t.setString(buf);
	t.setCharacterSize(14);
	t.setColor(sf::Color(0,0,0));
	t.setPosition(10,10+8*22);
	win.draw(t);

	sprintf(buf, "X %5.0f  Y %5.0f  ang %5.1f", cur_x, cur_y, cur_angle);
	t.setString(buf);
	t.setCharacterSize(16);
	t.setColor(sf::Color(0,100,0));
	t.setPosition(screen_x-250,40);
	win.draw(t);

	if(manual_control)
	{
		sprintf(buf, control_on?"MANUAL CONTROL":"CONTROL OFF");
		t.setCharacterSize(20);
		t.setColor(sf::Color(220,40,20));
	}
	else
	{
		sprintf(buf, "AUTO CONTROL");
		t.setCharacterSize(22);
		t.setColor(sf::Color(40,220,20));
	}
	t.setString(buf);
	t.setPosition(screen_x-250,10);
	win.draw(t);


	double vlevel = ((bat_voltage-3.1*5.0)/5.0)*(4.2-3.1); // 1 = full, 0 = empty

	sprintf(buf, "BATT %2.2f V (%.0f%%)", bat_voltage, vlevel*100.0);
	t.setString(buf);
	t.setCharacterSize(18);
	int r = (1.0-vlevel)*250.0;
	int g = vlevel*250.0;
	if(r > 250) r = 250; if(r<0) r=0;
	if(g > 250) g = 250; if(g<0) g=0;
	t.setColor(sf::Color(r,g,0));
	t.setPosition(screen_x-180,screen_y-40);
	win.draw(t);

	if(charging)
	{
		t.setString("charging");
		t.setCharacterSize(16);
		t.setColor(sf::Color(200,100,0));
		t.setPosition(screen_x-180,screen_y-80);
		win.draw(t);
	}

	if(charge_finished)
	{
		t.setString("chrg ended");
		t.setCharacterSize(16);
		t.setColor(sf::Color(200,100,0));
		t.setPosition(screen_x-180,screen_y-60);
		win.draw(t);
	}

}

int main(int argc, char** argv)
{
	int return_pressed = 0;
	int mouse_pressed = 0;
	int save_lidars = 0;
	int f9_pressed = 0;
	int c_pressed = 0;
	uint8_t rxbuf[1024];
	uint8_t parsebuf[1024];
	int do_parse = 0;
	int rxloc = 0;
	int focus = 1;

	internal_txbuf[0] = 0; // Start all UDP packets with 0; others are messages meant to the udpserver only.
	txbuf = &internal_txbuf[1];

	if(argc != 2 && argc != 4)
	{
		printf("Usage: visudrive /dev/ttyUSB0\n");
		printf("   or: visudrive serverip serverport ourport\n");
		return 1;
	}

	uart = 0;
	if(argc == 2)
	{	
		uart = open(argv[1], O_RDWR | O_NOCTTY);

		if(uart < 0)
		{
			printf("error %d opening %s: %s\n", errno, argv[1], strerror(errno));
			return 1;
		}

		set_uart_attribs(uart, B115200);
	}
	else
	{
		serv_ip = argv[1];
		serv_port = atoi(argv[2]);
		our_port = atoi(argv[3]);

		udpsock.setBlocking(false);
		if(udpsock.bind(our_port) != sf::Socket::Done)
		{
			printf("Error binding to udp socket.\n");
			return 1;
		}

		uint8_t sub[2] = {123, 0xaa};
		if(udpsock.send(sub, 2, serv_ip, serv_port) != sf::Socket::Done)
		{
			printf("Error sending subscription UDP packet.\n");
			return 1;
		}

	}


	if (!arial.loadFromFile("arial.ttf"))
	{
	    return 1;
	}

#ifdef LOG_RX
	FILE* rxlog = fopen("rxlog.txt", "w");
	if(!rxlog) {printf("Error opening rxlog\n"); return -1;}
#endif
	sf::ContextSettings sets;
	sets.antialiasingLevel = 8;
	sf::RenderWindow win(sf::VideoMode(screen_x,screen_y), "RN#1 Visual Drive Hommeli Pommeli", sf::Style::Default, sets);
	win.setFramerateLimit(60);

	for(int i = 0; i < 360; i++)
	{
		lidar_scan[i] = 1000+i;
	}

	tcflush(uart, TCIFLUSH);

	int cnt = 0;
	int cnt2 = 0;
	int auto_subscribe_timeout = 0;
	while(win.isOpen())
	{
		auto_subscribe_timeout++;

		if(!uart && auto_subscribe_timeout > 100)
		{
			auto_subscribe_timeout = 0;
			//printf("Auto subscribe\n");
			uint8_t sub[2] = {123, 0xaa};
			if(udpsock.send(sub, 2, serv_ip, serv_port) != sf::Socket::Done)
			{
				printf("UDP send error.\n");
			}

		}

		sf::Event event;
		while (win.pollEvent(event))
		{
			if(event.type == sf::Event::Closed)
				win.close();

			if(event.type == sf::Event::LostFocus)
				focus = 0;

			if(event.type == sf::Event::GainedFocus)
				focus = 1;

			if(event.type == sf::Event::Resized)
			{
				sf::Vector2u size = win.getSize();
				screen_x = size.x;
				screen_y = size.y;
			}
			
		}

		if(focus)
		{
			if(sf::Keyboard::isKeyPressed(sf::Keyboard::PageUp))
			{
				mm_per_pixel *= 1.05;
				origin_x *= 1.05;
				origin_y *= 1.05;
	//			origin_x = ((double)screen_x/2.0)*mm_per_pixel;
	//			origin_y = ((double)screen_y/2.0)*mm_per_pixel;
			}
			if(sf::Keyboard::isKeyPressed(sf::Keyboard::PageDown))
			{
				mm_per_pixel *= 0.95;
				origin_x *= 0.95;
				origin_y *= 0.95;
	//			origin_x = ((double)screen_x/2.0)*mm_per_pixel;
	//			origin_y = ((double)screen_y/2.0)*mm_per_pixel;
			}

			if(sf::Keyboard::isKeyPressed(sf::Keyboard::F5))
			{
				uint8_t sub[2] = {123, sf::Keyboard::isKeyPressed(sf::Keyboard::LShift)?(uint8_t)0xbb:(uint8_t)0xaa};
				if(udpsock.send(sub, 2, serv_ip, serv_port) != sf::Socket::Done)
				{
					printf("UDP send error.\n");
				}
			}

			if(sf::Keyboard::isKeyPressed(sf::Keyboard::F6))
			{
				txbuf[0] = sf::Keyboard::isKeyPressed(sf::Keyboard::LShift)?0xd2:0xd1;
				txbuf[1] = 0;
				txbuf[2] = 0xff;
				snd(3);
			}

			if(manual_control)
			{
				if(sf::Keyboard::isKeyPressed(sf::Keyboard::A))
				{
					angle_cmd += 1;
					if(angle_cmd > 3)
						angle_cmd = 3;
				}
				else if(sf::Keyboard::isKeyPressed(sf::Keyboard::Left))
				{
					angle_cmd += 2;
					if(angle_cmd > 15)
						angle_cmd = 15;
				}
				else
				{
					if(angle_cmd > 0) angle_cmd--;
				}
				if(sf::Keyboard::isKeyPressed(sf::Keyboard::D))
				{
					angle_cmd -= 1;
					if(angle_cmd < -3)
						angle_cmd = -3;
				}
				else if(sf::Keyboard::isKeyPressed(sf::Keyboard::Right))
				{
					angle_cmd -= 2;
					if(angle_cmd < -15)
						angle_cmd = -15;
				}
				else
				{
					if(angle_cmd < 0) angle_cmd++;
				}
				if(sf::Keyboard::isKeyPressed(sf::Keyboard::Up))
				{
					speed += 2;
					if(sf::Keyboard::isKeyPressed(sf::Keyboard::LShift))
					{ if(speed > 63) speed = 63; } else { if(speed > 40) speed = 40; }
				}
				if(sf::Keyboard::isKeyPressed(sf::Keyboard::Down))
				{
					speed -= 2;
					if(speed < -40) speed = -40;
				}
				if(sf::Keyboard::isKeyPressed(sf::Keyboard::W))
				{
					speed += 2;
					if(speed > 15) speed = 15;
				}
				if(sf::Keyboard::isKeyPressed(sf::Keyboard::S))
				{
					speed -= 2;
					if(speed < -15) speed = -15;
				}


			}
			else // automatic control
			{
				sf::Vector2i localPosition = sf::Mouse::getPosition(win);
				double click_x = (localPosition.x * mm_per_pixel) - origin_x - cur_x;
				double click_y = (localPosition.y * mm_per_pixel) - origin_y - cur_y;
				if(sf::Mouse::isButtonPressed(sf::Mouse::Left))
				{
						auto_angle = -1* ((360.0/(2*M_PI)*atan2(click_x, click_y)) + cur_angle - 90 - north_corr);
						auto_fwd = sqrt(click_x*click_x + click_y*click_y);
						if(auto_angle < -180) auto_angle += 360;
						else if(auto_angle > 180) auto_angle -= 360;
						if(auto_angle < -180) auto_angle += 360;
						else if(auto_angle > 180) auto_angle -= 360;
				}
				else if(sf::Mouse::isButtonPressed(sf::Mouse::Right))
				{
						auto_angle = -1* ((360.0/(2*M_PI)*atan2(click_x, click_y)) + cur_angle + 90 - north_corr);
						auto_fwd = -1 * sqrt(click_x*click_x + click_y*click_y);
						if(auto_angle < -180) auto_angle += 360;
						else if(auto_angle > 180) auto_angle -= 360;
						if(auto_angle < -180) auto_angle += 360;
						else if(auto_angle > 180) auto_angle -= 360;
				}


				if(sf::Keyboard::isKeyPressed(sf::Keyboard::Left))
				{
					if(sf::Keyboard::isKeyPressed(sf::Keyboard::RShift))
					{
						double fauto = -90/360.0*65536.0;
						int iauto = fauto;
						txbuf[0] = 0x81;
						txbuf[1] = I16_MS(iauto);
						txbuf[2] = I16_LS(iauto);
						txbuf[3] = 0;
						txbuf[4] = 0;
						txbuf[5] = 0xff;
						snd(6);
					}
					else
						origin_x -= 20.0;
				}
				if(sf::Keyboard::isKeyPressed(sf::Keyboard::Right))
				{
					if(sf::Keyboard::isKeyPressed(sf::Keyboard::RShift))
					{
						double fauto = 90/360.0*65536.0;
						int iauto = fauto;
						txbuf[0] = 0x81;
						txbuf[1] = I16_MS(iauto);
						txbuf[2] = I16_LS(iauto);
						txbuf[3] = 0;
						txbuf[4] = 0;
						txbuf[5] = 0xff;
						snd(6);

					}
					else
						origin_x += 20.0;
				}
				if(sf::Keyboard::isKeyPressed(sf::Keyboard::Up))
				{
					origin_y -= 20.0;
				}
				if(sf::Keyboard::isKeyPressed(sf::Keyboard::Down))
				{
					origin_y += 20.0;
				}
				if(sf::Keyboard::isKeyPressed(sf::Keyboard::Return))
				{
					if(!return_pressed)
					{
						double fauto = auto_angle/360.0*65536.0;
						int iauto = fauto;
						return_pressed = 1;
						txbuf[0] = 0x81;
						txbuf[1] = I16_MS(iauto);
						txbuf[2] = I16_LS(iauto);
						txbuf[3] = I16_MS(auto_fwd);
						txbuf[4] = I16_LS(auto_fwd);
						txbuf[5] = 0xff;
						snd(6);
						auto_angle = 0;
						auto_fwd = 0;
					}
				}
				else
				{
					return_pressed = 0;
				}

				if(sf::Keyboard::isKeyPressed(sf::Keyboard::C))
				{
					if(!c_pressed)
					{
						if(sf::Keyboard::isKeyPressed(sf::Keyboard::LShift))
							txbuf[0] = 0x92;
						else
							txbuf[0] = 0x91;
						txbuf[1] = 0;
						txbuf[2] = 0xff;
						snd(3);
					}
				}
				else
				{
					c_pressed = 0;
				}


			}


			if(sf::Keyboard::isKeyPressed(sf::Keyboard::F1))
			{
				north_corr-=1.0;
			}
			if(sf::Keyboard::isKeyPressed(sf::Keyboard::F2))
			{
				north_corr+=1.0;
			}
			if(sf::Keyboard::isKeyPressed(sf::Keyboard::F3))
			{
				if(sf::Keyboard::isKeyPressed(sf::Keyboard::LShift))
					txbuf[0] = 0xd3;
				else
					txbuf[0] = 0xd1;
				txbuf[1] = 0x01;
				txbuf[2] = 0xff;
				snd(3);
			}
			if(sf::Keyboard::isKeyPressed(sf::Keyboard::F4))
			{
				if(sf::Keyboard::isKeyPressed(sf::Keyboard::LShift))
					txbuf[0] = 0xd4;
				else
					txbuf[0] = 0xd2;
				txbuf[1] = 0x01;
				txbuf[2] = 0xff;
				snd(3);
			}
			if(sf::Keyboard::isKeyPressed(sf::Keyboard::F10))
			{
				manual_control = 1;
				control_on = 0;
			}
			if(sf::Keyboard::isKeyPressed(sf::Keyboard::F11))
			{
				manual_control = 1;
				control_on = 1;
			}
			if(sf::Keyboard::isKeyPressed(sf::Keyboard::F12))
			{
				manual_control = 0;
				control_on = 1;
			}
			if(sf::Keyboard::isKeyPressed(sf::Keyboard::F8))
			{
				dev_apply = 0;
//				save_lidars = 2;
			}
			if(sf::Keyboard::isKeyPressed(sf::Keyboard::F9))
			{
				dev_apply = 1;

/*				if(!f9_pressed)
				{
					f9_pressed = 1;
					FILE *f1 = fopen("lidar_before_in", "r");
					FILE *f2 = fopen("lidar_after_in", "r");
					FILE *f3 = fopen("lidar_after_out", "r");
					FILE *f4 = fopen("lidar_after_out_alt", "r");
					dev_show_lidars = 1;
					if(!f1) {printf("Can't open f1\n"); dev_show_lidars = 0;} else {read_lidar(f1, &dev_lidar_bef_in);}
					if(!f2) {printf("Can't open f2\n"); dev_show_lidars = 0;} else {read_lidar(f2, &dev_lidar_aft_in);}
					if(!f3) {printf("Can't open f3\n"); dev_show_lidars = 0;} else {read_lidar(f3, &dev_lidar_aft_out);}
					if(!f4) {printf("Can't open f4\n"); dev_show_lidars = 0;} else {read_lidar(f4, &dev_lidar_aft_out_alt);}
					if(f1) fclose(f1); if(f2) fclose(f2); if(f3) fclose(f3); if(f4) fclose(f4);
				}

				*/
			}
//			else
//				f9_pressed = 0;
		} // end if(focus)

		win.clear(sf::Color(180,220,255));

		uint8_t byte;

		if(uart)
		{
			while(read(uart, &byte, 1))
			{
				if(rxloc > 1000)
					rxloc = 0;

				if(byte > 127)
				{
					memcpy(parsebuf, rxbuf, rxloc);
					do_parse = 1;
					rxbuf[0] = byte;
					rxloc = 1;
					break;
				}

				rxbuf[rxloc] = byte;
				rxloc++;
			}
		}
		else // UDP
		{
			sf::IpAddress sender;
			unsigned short sender_port;
			std::size_t len = rxloc;
			if(udpsock.receive(rxbuf, 1000, len, sender, sender_port) == sf::Socket::Done)
			{
//				printf("RECEIVED! len=%d  data = %u  %u  %u\n", len, rxbuf[0], rxbuf[1], rxbuf[2]);
				memcpy(parsebuf, rxbuf, len);

#ifdef LOG_RX
				fprintf(rxlog, "len=%4d    ", len);
				for(int i = 0; i < len; i++)
				{
					fprintf(rxlog, "%02x ", parsebuf[i]);
				}
				fprintf(rxlog, "\n");
#endif
				do_parse = 1;
			}
		}

		if(do_parse)
		{

			auto_subscribe_timeout = 0;

//			printf("Got %02x\n", parsebuf[0]);

			switch(parsebuf[0])
			{
				case 0x80:
				gyro_x = (double)I14x2_I16(parsebuf[3], parsebuf[2]);
				gyro_y = (double)I14x2_I16(parsebuf[5], parsebuf[4]);
				gyro_z = (double)I14x2_I16(parsebuf[7], parsebuf[6]);
				break;

				case 0x81:
				xcel_x = (double)I14x2_I16(parsebuf[3], parsebuf[2]);
				xcel_y = (double)I14x2_I16(parsebuf[5], parsebuf[4]);
				xcel_z = (double)I14x2_I16(parsebuf[7], parsebuf[6]);
				break;

				case 0x82:
				compass_x = (double)I14x2_I16(parsebuf[3], parsebuf[2]);
				compass_y = (double)I14x2_I16(parsebuf[5], parsebuf[4]);
				compass_z = (double)I14x2_I16(parsebuf[7], parsebuf[6]);

//				cur_angle = 360.0/(2*M_PI)*atan2(compass_x, compass_y);
				break;

				case 0x84:
				{
					lidar_status = parsebuf[1];
					for(int i = 0; i < 360; i++)
					{
						lidar_scan[359-i] = parsebuf[14+2*i+1]<<7 | parsebuf[14+2*i];
					}
					double a = (double)(I14x2_I16(parsebuf[2], parsebuf[3]));
					a = a * 360.0 / 65536.0;
					cur_angle_at_lidar_update = a;
					cur_x_at_lidar_update = I7x5_I32(parsebuf[4],parsebuf[5],parsebuf[6],parsebuf[7],parsebuf[8]);
					cur_y_at_lidar_update = I7x5_I32(parsebuf[9],parsebuf[10],parsebuf[11],parsebuf[12],parsebuf[13]);

					int speziel_idx = (lidar_status&0b1100)>>2;
					if(speziel_idx)
					{
						take_lidar_snapshot(speziel_idx);

						if(save_lidars)
						{
							lidar_scan_t hommel;
							for(int i = 0; i < 360; i++)
								hommel.scan[i] = lidar_scan[i];
							hommel.angle = ((int32_t)(I14x2_I16(parsebuf[2], parsebuf[3])))<<16;
							hommel.x = cur_x_at_lidar_update;
							hommel.y = cur_y_at_lidar_update;
							FILE* f_scanout = fopen((save_lidars==2)?"lidar_before_in":"lidar_after_in", "w");
							write_lidar(f_scanout, &hommel);
							fclose(f_scanout);
							save_lidars--;
						}
					}
				}
				break;

				case 0x85:
				sonars[0] = (I14x2_I16(parsebuf[3], parsebuf[2]))>>2;
				sonars[1] = (I14x2_I16(parsebuf[5], parsebuf[4]))>>2;
				sonars[2] = (I14x2_I16(parsebuf[7], parsebuf[6]))>>2;

				break;

				case 0xa0:
				{
					double new_angle = (double)(I14x2_I16(parsebuf[2], parsebuf[3]));
					new_angle = new_angle * 360.0 / 65536.0;
/*					if(fabs(gyro_z) > 500.0)
						cur_angle = (new_angle + 1.0*cur_angle)/2.0;
					else if(fabs(gyro_z) > 250.0)
						cur_angle = (new_angle + 3.0*cur_angle)/4.0;
					else
						cur_angle = (new_angle + 10.0*cur_angle)/11.0;
*/
					cur_angle = new_angle;
					double new_compass = (double)(I14x2_I16(parsebuf[6], parsebuf[7]));
					new_compass = new_compass * 360.0 / 65536.0;
					cur_compass = new_compass;

					cur_x = I7x5_I32(parsebuf[8],parsebuf[9],parsebuf[10],parsebuf[11],parsebuf[12]);
					cur_y = I7x5_I32(parsebuf[13],parsebuf[14],parsebuf[15],parsebuf[16],parsebuf[17]);

				}
				break;

				case 0xa1:
				{
					int_x = (double)(I14x2_I16(parsebuf[2], parsebuf[3]))/4.0;
					int_y = (double)(I14x2_I16(parsebuf[4], parsebuf[5]))/4.0;
					opt_q = (double)parsebuf[6];
				}
				break;

				case 0xa2:
				{
					charging = parsebuf[1]&1;
					charge_finished = parsebuf[1]&2;
					bat_voltage = (double)(I14x2_I16(parsebuf[2], parsebuf[3]))/22200.0 * 17.73; // 17.73V = 22200
				}
				break;
				case 0xd1:
				{
					d1 = (((int)parsebuf[1])<<9) | (((int)parsebuf[2])<<2)>>2;
				}
				break;

				case 0xd2:
				{
					for(int i=0; i<10; i++)
					{
						dbg[i] = I7x5_I32(parsebuf[i*5+1],parsebuf[i*5+2],parsebuf[i*5+3],parsebuf[i*5+4],parsebuf[i*5+5]);
					}
				}
				break;

				default:
				break;
			}
			do_parse = 0;
		}


		cnt++;

		if(speed > 0) speed--;
		else if(speed < 0) speed++;


		if(control_on)
		{
			if(manual_control)
			{
				txbuf[0] = 0x80;
				txbuf[1] = ( (uint8_t)(speed<<1) ) >> 1;
				txbuf[2] = ( (uint8_t)(angle_cmd<<1) ) >> 1;
				txbuf[3] = 0xff;
				snd(4);
			}
			else
			{
				if((cnt&0b11) == 3) // Send but don't flood "I'm alive" messages.
				{
					txbuf[0] = 0x8f;
					txbuf[1] = 0;
					txbuf[2] = 0xff;
					snd(3);
				}
			}
		}

		if(dev_show_lidars)
		{
			draw_lidar(win, dev_lidar_bef_in.scan, sf::Color(0,0,0), 5.0, ((double)dev_lidar_bef_in.angle/4294967296.0)*360.0, dev_lidar_bef_in.x, dev_lidar_bef_in.y);
			draw_mini_robot(win, sf::Color(0,0,0), ((double)dev_lidar_bef_in.angle/4294967296.0)*360.0, dev_lidar_bef_in.x, dev_lidar_bef_in.y);
			draw_lidar(win, dev_lidar_aft_in.scan, sf::Color(90,90,255), 3.0, ((double)dev_lidar_aft_in.angle/4294967296.0)*360.0, dev_lidar_aft_in.x, dev_lidar_aft_in.y);
			draw_mini_robot(win, sf::Color(90,90,255), ((double)dev_lidar_aft_in.angle/4294967296.0)*360.0, dev_lidar_aft_in.x, dev_lidar_aft_in.y);
			draw_lidar(win, dev_lidar_aft_out.scan, sf::Color(255,0,0), 1.0, ((double)dev_lidar_aft_out.angle/4294967296.0)*360.0, dev_lidar_aft_out.x, dev_lidar_aft_out.y);
			draw_lidar(win, dev_lidar_aft_out_alt.scan, sf::Color(0,255,0), 1.0, ((double)dev_lidar_aft_out_alt.angle/4294967296.0)*360.0, 
				dev_lidar_aft_out_alt.x, dev_lidar_aft_out_alt.y);
		}
		else
		{
		draw_robot(win);
		for(int l = 0; l < NUM_LIDAR_SNAPSHOTS; l++)
		{
			if(dev_apply && special_idx_at_snapshot[l] == 2)
			{
				draw_lidar(win, lidar_snapshots[l], special_lidar_colors[special_idx_at_snapshot[l]], lidar_line_thick,
					ang_at_snapshot[l]-(360*(double)dbg[3]/4294967296.0), x_at_snapshot[l]-dbg[4], y_at_snapshot[l]-dbg[5]);

			}
			else
			{
				draw_lidar(win, lidar_snapshots[l], special_lidar_colors[special_idx_at_snapshot[l]], lidar_line_thick,
					ang_at_snapshot[l], x_at_snapshot[l], y_at_snapshot[l]);
			}

		}

		draw_lidar(win, lidar_scan, sf::Color(0,0,0,100), lidar_line_thick, cur_angle_at_lidar_update, cur_x_at_lidar_update, cur_y_at_lidar_update);

		draw_sonars(win);
		draw_texts(win);
		}



		win.display();

		usleep(100);

	}
	return 0;
}
