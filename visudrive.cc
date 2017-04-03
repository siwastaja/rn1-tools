#include <cstdio>
#include <cmath>
#include <SFML/Graphics.hpp>

#ifndef M_PI
#define M_PI 3.14159265358
#endif

double mm_per_pixel = 10.0;

int screen_x = 1024;
int screen_y = 768;

double origin_x = ((double)screen_x/2.0)*mm_per_pixel;
double origin_y = ((double)screen_y/2.0)*mm_per_pixel;

double cur_x = 0.0;
double cur_y = 0.0;
double cur_angle = 5.0;

const double lidar_line_thick = 2.0;

void draw_robot(sf::RenderWindow& win)
{
	sf::ConvexShape r(5);
	r.setPoint(0, sf::Vector2f(0,0));
	r.setPoint(1, sf::Vector2f(0,12));
	r.setPoint(2, sf::Vector2f(15,12));
	r.setPoint(3, sf::Vector2f(20,6));
	r.setPoint(4, sf::Vector2f(15,0));

	r.setFillColor(sf::Color(200,70,50));
	r.setOrigin(10,6);

	r.setRotation(cur_angle);
	r.setPosition((cur_x+origin_x)/mm_per_pixel,(cur_y+origin_y)/mm_per_pixel);

	win.draw(r);
}

int lidar_scan[360];

void draw_lidar(sf::RenderWindow& win)
{
	for(int i = 0; i < 360; i++)
	{
		int ip = (i==359)?0:(i+1);
		int first = lidar_scan[i];
		int second = lidar_scan[ip];

		if(first == 0 && second == 0)
			continue;

		if(first == 0) first = second;
		else if(second == 0) second = first;

		double x1 = (cur_x+origin_x+cos(M_PI*(cur_angle+(double)i)/180.0) * first)/mm_per_pixel;
		double y1 = (cur_y+origin_y+sin(M_PI*(cur_angle+(double)i)/180.0) * first)/mm_per_pixel;
		double x2 = (cur_x+origin_x+cos(M_PI*(cur_angle+(double)ip)/180.0) * second)/mm_per_pixel;
		double y2 = (cur_y+origin_y+sin(M_PI*(cur_angle+(double)ip)/180.0) * second)/mm_per_pixel;
		sf::RectangleShape rect(sf::Vector2f( sqrt(pow(x2-x1,2)+pow(y2-y1,2)), lidar_line_thick));
		rect.setOrigin(0, lidar_line_thick/2.0);
		rect.setPosition(x1, y1);
		rect.setRotation(atan2(y2-y1,x2-x1)*180.0/M_PI);
		rect.setFillColor(sf::Color(220,30,30));

		win.draw(rect);
	}
}

int main(int argc, char** argv)
{
	sf::Font arial;
	if (!arial.loadFromFile("arial.ttf"))
	{
	    return 1;
	}

	sf::ContextSettings sets;
	sets.antialiasingLevel = 8;
	sf::RenderWindow win(sf::VideoMode(screen_x,screen_y), "RN#1 Visual Drive Hommeli Pommeli", sf::Style::Default, sets);

	for(int i = 0; i < 360; i++)
	{
		lidar_scan[i] = 1000+i;
	}

	lidar_scan[20] = 0;
	lidar_scan[50] = 0;
	lidar_scan[53] = 0;
	lidar_scan[56] = 0;

	lidar_scan[80] = 0;
	lidar_scan[81] = 0;

	lidar_scan[195] = 0;
	lidar_scan[196] = 0;
	lidar_scan[197] = 0;
	lidar_scan[198] = 0;
	lidar_scan[199] = 0;

	while(win.isOpen())
	{
		sf::Event event;
		while (win.pollEvent(event))
		{
			if(event.type == sf::Event::Closed)
				win.close();
		}

		if(sf::Keyboard::isKeyPressed(sf::Keyboard::Left))
		{
			cur_angle -= 1.0;
		}
		if(sf::Keyboard::isKeyPressed(sf::Keyboard::Right))
		{
			cur_angle += 1.0;
		}
		if(sf::Keyboard::isKeyPressed(sf::Keyboard::Up))
		{
			cur_x += cos(M_PI*cur_angle/180.0);
			cur_y += sin(M_PI*cur_angle/180.0);
		}
		if(sf::Keyboard::isKeyPressed(sf::Keyboard::Down))
		{
			cur_x -= cos(M_PI*cur_angle/180.0);
			cur_y -= sin(M_PI*cur_angle/180.0);
		}

		win.clear(sf::Color(180,220,255));

		draw_robot(win);
		draw_lidar(win);

		win.display();
	}
	return 0;
}
