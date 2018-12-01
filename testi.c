#include <stdio.h>
#include <stdint.h>

#define VOX_SEG_XS 10
#define VOX_SEG_YS 10
#define VOX_HIRES_UNIT 2 // mm
#define VOX_LORES_UNIT 4 // mm


typedef struct
{
	int xmin;
	int xmax;
	int ymin;
	int ymax;
	int reso;
} seg_limits_t;

const seg_limits_t seg_lims[12] =
{
	{	// Seg 0
		0, VOX_SEG_XS*VOX_HIRES_UNIT-1,
		0, VOX_SEG_YS*VOX_HIRES_UNIT-1,
		VOX_HIRES_UNIT
	},

	{	// Seg 1
		-VOX_SEG_XS*VOX_HIRES_UNIT, -1,
		0, VOX_SEG_YS*VOX_HIRES_UNIT-1,
		VOX_HIRES_UNIT
	},

	{	// Seg 2
		-VOX_SEG_XS*VOX_HIRES_UNIT, -1,
		-VOX_SEG_YS*VOX_HIRES_UNIT, -1,
		VOX_HIRES_UNIT
	},

	{	// Seg 3
		0, VOX_SEG_XS*VOX_HIRES_UNIT-1,
		-VOX_SEG_YS*VOX_HIRES_UNIT, -1,
		VOX_HIRES_UNIT
	},

	{	// Seg 4
		VOX_SEG_XS*VOX_HIRES_UNIT, VOX_SEG_XS*VOX_HIRES_UNIT + VOX_SEG_XS*VOX_LORES_UNIT-1,
		-VOX_SEG_YS*VOX_HIRES_UNIT, VOX_SEG_YS*VOX_HIRES_UNIT-1,
		VOX_LORES_UNIT
	},

	{	// Seg 5
		VOX_SEG_XS*VOX_HIRES_UNIT, VOX_SEG_XS*VOX_HIRES_UNIT + VOX_SEG_XS*VOX_LORES_UNIT-1,
		VOX_SEG_YS*VOX_HIRES_UNIT, VOX_SEG_YS*VOX_HIRES_UNIT + VOX_SEG_YS*VOX_LORES_UNIT-1,
		VOX_LORES_UNIT
	},

	{	// Seg 6
		-VOX_SEG_XS*VOX_HIRES_UNIT, VOX_SEG_XS*VOX_HIRES_UNIT-1,
		VOX_SEG_YS*VOX_HIRES_UNIT, VOX_SEG_YS*VOX_HIRES_UNIT + VOX_SEG_YS*VOX_LORES_UNIT-1,
		VOX_LORES_UNIT
	},

	{	// Seg 7
		-VOX_SEG_XS*VOX_HIRES_UNIT - VOX_SEG_XS*VOX_LORES_UNIT, -VOX_SEG_XS*VOX_HIRES_UNIT-1,
		VOX_SEG_YS*VOX_HIRES_UNIT, VOX_SEG_YS*VOX_HIRES_UNIT + VOX_SEG_YS*VOX_LORES_UNIT-1,
		VOX_LORES_UNIT
	},

	{	// Seg 8
		-VOX_SEG_XS*VOX_HIRES_UNIT - VOX_SEG_XS*VOX_LORES_UNIT, -VOX_SEG_XS*VOX_HIRES_UNIT-1,
		-VOX_SEG_YS*VOX_HIRES_UNIT, VOX_SEG_YS*VOX_HIRES_UNIT-1,
		VOX_LORES_UNIT
	},

	{	// Seg 9
		-VOX_SEG_XS*VOX_HIRES_UNIT - VOX_SEG_XS*VOX_LORES_UNIT, -VOX_SEG_XS*VOX_HIRES_UNIT-1,
		-VOX_SEG_YS*VOX_HIRES_UNIT-VOX_SEG_YS*VOX_LORES_UNIT, -VOX_SEG_YS*VOX_HIRES_UNIT-1,
		VOX_LORES_UNIT
	},

	{	// Seg 10
		-VOX_SEG_XS*VOX_HIRES_UNIT, VOX_SEG_XS*VOX_HIRES_UNIT-1,
		-VOX_SEG_YS*VOX_HIRES_UNIT-VOX_SEG_YS*VOX_LORES_UNIT, -VOX_SEG_YS*VOX_HIRES_UNIT-1,
		VOX_LORES_UNIT
	},

	{	// Seg 11
		VOX_SEG_XS*VOX_HIRES_UNIT, VOX_SEG_XS*VOX_HIRES_UNIT + VOX_SEG_XS*VOX_LORES_UNIT-1,
		-VOX_SEG_YS*VOX_HIRES_UNIT-VOX_SEG_YS*VOX_LORES_UNIT, -VOX_SEG_YS*VOX_HIRES_UNIT-1,
		VOX_LORES_UNIT
	}

};

int main()
{
	printf("       %+05d                %+05d                %+05d       \n", seg_lims[7].ymax, seg_lims[6].ymax, seg_lims[5].ymax);
	printf("                                                   \n");
	printf("                                                   \n");
	printf("  %+05d       %+05d    %+05d       %+05d    %+05d       %+05d \n", seg_lims[7].xmin, seg_lims[7].xmax, seg_lims[6].xmin, seg_lims[6].xmax, seg_lims[5].xmin, seg_lims[5].xmax);
	printf("                                                   \n");
	printf("                                                   \n");
	printf("       %+05d                %+05d                %+05d       \n", seg_lims[7].ymin, seg_lims[6].ymin, seg_lims[5].ymin);
	printf("       %+05d          %+05d       %+05d          %+05d       \n", seg_lims[8].ymax, seg_lims[1].ymax, seg_lims[0].ymax, seg_lims[4].ymax);
	printf("                  %+05d %+05d  %+05d %+05d                  \n", seg_lims[1].xmin, seg_lims[1].xmax, seg_lims[0].xmin, seg_lims[0].xmax);
	printf("                      %+05d       %+05d                     \n", seg_lims[1].ymin, seg_lims[0].ymin);
	printf("  %+05d       %+05d                     %+05d       %+05d  \n", seg_lims[8].xmin, seg_lims[8].xmax,      seg_lims[4].xmin, seg_lims[4].xmax);
	printf("                      %+05d       %+05d                     \n", seg_lims[2].ymax, seg_lims[3].ymax);
	printf("                  %+05d %+05d  %+05d %+05d                  \n", seg_lims[2].xmin, seg_lims[2].xmax, seg_lims[3].xmin, seg_lims[3].xmax);
	printf("       %+05d          %+05d       %+05d             %+05d       \n", seg_lims[8].ymin, seg_lims[2].ymin, seg_lims[3].ymin, seg_lims[4].ymin);
	printf("       %+05d                 %+05d                 %+05d       \n", seg_lims[9].ymax, seg_lims[10].ymax, seg_lims[11].ymax);
	printf("                                                   \n");
	printf("                                                   \n");
	printf("  %+05d       %+05d    %+05d       %+05d    %+05d       %+05d  \n", seg_lims[9].xmin, seg_lims[9].xmax, seg_lims[10].xmin, seg_lims[10].xmax, seg_lims[11].xmin, seg_lims[11].xmax);
	printf("                                                   \n");
	printf("                                                   \n");
	printf("       %+05d                 %+05d                 %+05d       \n", seg_lims[9].ymin, seg_lims[10].ymin, seg_lims[11].ymin);
	return 0;
}

