/*
	Generates arctan table for optimized 3DTOF calculation and verifies the table, bruteforcing every input condition to find maximum error
	compared to double precision floating point reference design.

	Unit: mm
*/


#include <stdio.h>
#include <stdint.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.141592653589793238
#endif

#define TOF_TBL_LEN 512

int16_t tof_tbl[TOF_TBL_LEN]; // spans pi/4 (45 degs) of the full atan2 circle

int TOF_TBL_PERIOD, TOF_TBL_HALF_PERIOD, TOF_TBL_QUART_PERIOD; // will be #defines in the generated .h file - used here for verification

#define SENSOR_MAX 2048

int16_t get_tof_tbl(int16_t y, int16_t x)
{
	int yy, xx;

	if(y<0)
		yy = -y;
	else
		yy = y;

	if(x<0)
		xx = -x;
	else
		xx = x;

	int swapped = 0;
	if(xx<yy)
	{
		swapped = 1;
		int16_t tmp = xx;
		xx = yy;
		yy = tmp;
	}

	if(xx == 0) return 0;

	int idx = (yy*(TOF_TBL_LEN-1))/xx;

	if(idx < 0 || idx > TOF_TBL_LEN-1)
	{
		printf("Err: OOR idx, y=%d  x=%d,  yy=%d, xx=%d,  idx=%d\n", y,x,yy,xx,idx);
		return 0;
	}

	int16_t res = tof_tbl[idx];
	if(swapped) res = TOF_TBL_QUART_PERIOD - res;
	if(x<0) res = TOF_TBL_HALF_PERIOD - res;
	if(y<0) res = -res;

	return res;
}

int main(int argc, char** argv)
{

	double fled = 20000000.0;
	double mult = 299792458.0/2.0*1.0/(2.0*M_PI*fled); 
	double unamb = 299792458.0/2.0*1.0/(fled);

	TOF_TBL_PERIOD = unamb*1000;
	TOF_TBL_HALF_PERIOD = unamb*1000.0/2.0;
	TOF_TBL_QUART_PERIOD = unamb*1000.0/4.0;

	FILE* tblh = fopen("tof_table.h", "w");
	if(!tblh)
	{
		printf("Error opening tof_table.h for write\n");
		return 1;
	}
	FILE* tblc = fopen("tof_table.c", "w");
	if(!tblc)
	{
		printf("Error opening tof_table.c for write\n");
		return 1;
	}

	fprintf(tblh, "#ifndef TOF_TABLE_H\n");
	fprintf(tblh, "#define TOF_TABLE_H\n\n");
	fprintf(tblh, "#include <stdint.h>\n\n");
	fprintf(tblh, "#define TOF_TBL_LEN %u\n\n", TOF_TBL_LEN);
	fprintf(tblh, "#define TOF_TBL_PERIOD %u\n\n", TOF_TBL_PERIOD);
	fprintf(tblh, "#define TOF_TBL_HALF_PERIOD %u\n\n", TOF_TBL_HALF_PERIOD);
	fprintf(tblh, "#define TOF_TBL_QUART_PERIOD %u\n\n", TOF_TBL_QUART_PERIOD);
	fprintf(tblh, "extern const int16_t tof_tbl[TOF_TBL_LEN] __attribute__((section(\".tof_tbl\")));\n\n");
	fprintf(tblh, "#endif\n");
	fclose(tblh);

	fprintf(tblc, "#include <stdint.h>\n");
	fprintf(tblc, "#include \"tof_table.h\"\n\n");
	fprintf(tblc, "const int16_t tof_tbl[TOF_TBL_LEN] __attribute__((section(\".tof_tbl\"))) =\n");
	fprintf(tblc, "{\n");

	for(int i=0; i<TOF_TBL_LEN; i++)
	{
		tof_tbl[i] = (unamb*1000.0*(atan((double)i/(double)TOF_TBL_LEN))/(2*M_PI))+0.5;
		fprintf(tblc, "/*i=%4u*/ %5d%s\n", i, tof_tbl[i], (i==TOF_TBL_LEN-1)?"":",");
	}

	fprintf(tblc, "};\n");


	// Check all possible inputs to the "real" formula to find max error:

	int max_err = -9999999;
	int min_err = 9999999;

	for(int y=-SENSOR_MAX; y<SENSOR_MAX; y++)
	{
		for(int x=-SENSOR_MAX; x<SENSOR_MAX; x++)
		{

			double actual = (mult * (/*M_PI +*/ atan2(y,x)));

			if(actual < 0.0) actual += unamb;
			else if(actual > unamb) actual -= unamb;

			int actual_i = 1000.0*actual;

			int from_table = get_tof_tbl(y, x);

			if(from_table < 0) from_table += TOF_TBL_PERIOD;

			int err = actual_i - from_table;
			if(err > TOF_TBL_PERIOD/2) err -= TOF_TBL_PERIOD;
			if(err < -TOF_TBL_PERIOD/2) err += TOF_TBL_PERIOD;

			//printf("y=%5d  x=%5d  actual=%5u  table=%5u  err=%5d\n", y, x, actual_i, from_table, err);

			if(err > max_err)
			{ 
				max_err = err;
				printf("max_err at y=%5d  x=%5d  actual=%5u  table=%5u  err=%5d\n", y, x, actual_i, from_table, err);
			}
			if(err < min_err)
			{ 
				min_err = err;
				printf("min_err at y=%5d  x=%5d  actual=%5u  table=%5u  err=%5d\n", y, x, actual_i, from_table, err);
			}

		}
	}

	printf("Memory required for the table: %lu bytes\n", sizeof(tof_tbl));
	return 0;
}
