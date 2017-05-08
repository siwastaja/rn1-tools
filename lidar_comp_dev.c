/*

LIDAR based location/angular error correction

*/

#include <stdio.h>
#include <math.h>
#include <stdint.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.141592653589793238
#endif

typedef struct
{
	int32_t angle;
	int32_t x;
	int32_t y;
	int16_t scan[360];
} lidar_scan_t;

typedef struct
{
	int valid;
	int32_t x;
	int32_t y;
} point_t;


lidar_scan_t bef;
lidar_scan_t aft;

point_t bef2d[360];
point_t aft2d[360];

#define LIDAR_RANGE 4500

#define sq(x) ((x)*(x))

void scan_to_2d(lidar_scan_t* in, point_t* out)
{
	for(int i = 0; i < 360; i++)
	{
		if(in->scan[i] != 0)
		{
			out[i].valid = 1;
			out[i].x = in->x + cos(2.0*M_PI*(   (((double)in->angle)/4294967296.0)  +  ((double)i/360.0)   )) * in->scan[i];
			out[i].y = in->y + sin(2.0*M_PI*(   (((double)in->angle)/4294967296.0)  +  ((double)i/360.0)   )) * in->scan[i];
		}
		else
			out[i].valid = 0;
	}
}

int num_points(point_t* img)
{
	int n = 0;
	for(int i = 0; i < 360; i++)
	{
		if(img[i].valid) n++;
	}
	return n;
}

int kakka_max = 0;
int kakka_sum = 0;
int kakka_cnt = 0;

double calc_match_lvl(point_t* img1, point_t* img2, int img1_points)
{
	/*
	For each point in the first image, search the nearest point in the second image; any valid point will do.

	Use square root to scale the distance so that small distances are more meaningfull than large distances;
	this is to ignore objects that are really different between the images, trying to account for objects common
	for both images.

	Sum up the distances.

	Finally, divide by the number of points to get comparable value regardless of num of valid points.
	*/

	int i_at_smallest;
	int o_at_smallest;

	double dist_sum = 0.0;
	for(int i = 0; i < 360; i++)
	{
		if(!img1[i].valid) continue;

		double smallest = 999999999999999.9;
		for(int o = 0; o < 360; o++)
		{
			if(!img2[o].valid) continue;
			double dist = ( sq((double)(img2[o].x - img1[i].x)) + sq((double)(img2[o].y - img1[i].y)) );
			if(dist < smallest)
			{
				smallest = dist;
				i_at_smallest = i;
				o_at_smallest = o;
			}
		}

		int kukka = i_at_smallest - o_at_smallest;
		if(kukka < -180) kukka += 360;
		if(kukka > 180) kukka -= 360;
		if(kukka > kakka_max) kakka_max = kukka;
		kakka_cnt++;
		kakka_sum += kukka;

		double dist_scaled = sqrt(sqrt(smallest));
		dist_sum += dist_scaled;
	}

	dist_sum /= img1_points;
	return dist_sum;
}

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

int main(int argc, char** argv)
{

	FILE* f_bef_in     = fopen("lidar_before_in", "r");
	FILE* f_aft_in     = fopen("lidar_after_in", "r");
	FILE* f_csv        = fopen("log.csv", "w");
	FILE* f_aft_out    = fopen("lidar_after_out", "w");

	if(!f_bef_in) {printf("Error opening lidar_before_in for read\n"); return 1;}
	if(!f_aft_in) {printf("Error opening lidar_after_in for read\n"); return 1;}
	if(!f_aft_out) {printf("Error opening lidar_after_out for write\n"); return 1;}
	if(!f_csv) {printf("Error opening log.csv for write\n"); return 1;}

	read_lidar(f_bef_in, &bef);
	read_lidar(f_aft_in, &aft);

	/*
	Step 1:
	Convert lidar scans to absolute x,y coordinates on the same map, assuming that the original
	coordinates are right.
	*/

	scan_to_2d(&bef, bef2d);
	scan_to_2d(&aft, aft2d);

	/*
	Step 2:
	Process both lidar scans to remove any points that are out of visible range seen from the other scan.
	LIDAR_RANGE must be slightly smaller than in real life, since we are assuming zero error in robot coordinates.
	*/

	for(int i = 0; i < 360; i++)
	{
		if(sqrt(sq((double)(bef2d[i].x - aft.x)) + sq((double)(bef2d[i].y - aft.y))) > LIDAR_RANGE)
		{
			bef2d[i].valid = 0;
			bef.scan[i] = 0;
		}
		if(sqrt(sq((double)(aft2d[i].x - bef.x)) + sq((double)(aft2d[i].y - bef.y))) > LIDAR_RANGE)
		{
			aft2d[i].valid = 0;
			aft.scan[i] = 0;
		}
	}


	int points1 = num_points(bef2d);
	int points2 = num_points(aft2d);
	printf("num_points=%d, %d\n", points1, points2);


	if(points1 < 20 || points2 < 20)
	{
		printf("Not enough valid points (img_before = %d, img_after = %d)\n", points1, points2);
		return 1;
	}
	/*
	Step 3:
	Test different corrections and calculate match levels
	*/

	lidar_scan_t aft_corr;
	memcpy(&aft_corr, &aft, sizeof(lidar_scan_t));

	point_t aft_corr_2d[360];

	double smallest_lvl = 99999999999999999.9;

	double best_a;
	int best_x;
	int best_y;

	fprintf(f_csv, "a,x,y,match,i-o avg, i-o max\n");
	for(double a_corr = -6.0; a_corr < 6.0; a_corr += 0.5)
//	for(double a_corr = 0.0; a_corr < 0.2; a_corr += 0.5)
	{
		printf("a_corr = %.2f\r", a_corr); fflush(stdout);
		int a_corr_i = a_corr/360.0 * 4294967296.0;
		aft_corr.angle = aft.angle + a_corr_i;
		for(int x_corr = -200; x_corr < 200; x_corr += 20)
		{
			aft_corr.x = aft.x + x_corr;
			for(int y_corr = -200; y_corr < 200; y_corr += 20)
			{
				aft_corr.y = aft.y + y_corr;
				scan_to_2d(&aft_corr, aft_corr_2d);
				kakka_max = 0;
				kakka_sum = 0;
				kakka_cnt = 0;
				double lvl = calc_match_lvl(bef2d, aft_corr_2d, points1);
				fprintf(f_csv, "%.2f,%d,%d,%.3f,%.2f,%d\n", a_corr, x_corr, y_corr, lvl, (double)kakka_sum/(double)kakka_cnt, kakka_max);
				if(lvl < smallest_lvl)
				{
					smallest_lvl = lvl;
					best_a = a_corr;
					best_x = x_corr;
					best_y = y_corr;
				}
			}
		}
	}

	printf("\nBest correction: a = %.2f, x = %d, y = %d\n", best_a, best_x, best_y);

	lidar_scan_t best;
	memcpy(&best, &aft, sizeof(lidar_scan_t));
	best.angle += best_a/360.0 * 4294967296.0;
	best.x += best_x;
	best.y += best_y;

	write_lidar(f_aft_out, &best);

	return 0;
}
