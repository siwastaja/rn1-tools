#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#define _BSD_SOURCE  // glibc backwards incompatibility workaround to bring usleep back.
#define _POSIX_C_SOURCE 200809L
#include <unistd.h>

#define TMPVOX_XS 20
#define TMPVOX_YS 20
#define TMPVOX_XMID (TMPVOX_XS/2)
#define TMPVOX_YMID (TMPVOX_XS/2)

#define RADTODEG(x) ((x)*(360.0/(2.0*M_PI)))
#define DEGTORAD(x) ((x)*((2.0*M_PI)/360.0))

void rotate_tmpvox(uint16_t* restrict out, uint16_t* restrict in, double ang_rad)
{
	static uint16_t tmp_skewed[TMPVOX_XS*TMPVOX_YS];

	memset(tmp_skewed, 0, sizeof tmp_skewed);

	if(ang_rad > DEGTORAD(15) || ang_rad < DEGTORAD(-15))
	{
		printf("Error: out-of-range ang_rad in rotate_tmpvox\n");
		return;
	}
	double step = tan(ang_rad);

	// Shift everything X direction first, creating a skewed copy

	for(int yy = 0; yy < TMPVOX_YS; yy++)
	{
		double fshift = ((double)(TMPVOX_YMID - yy)*step); // Bottom line (yy=0) shifts right with positive ang.
		int shift = (fshift>=0.0)?(fshift+0.5):(fshift-0.5);
		printf("Line yy=%d shifts by %d\n", yy, shift);

		if(shift >= 0)
		{
			for(int xx=0; xx<TMPVOX_XS-shift; xx++)
			{
				tmp_skewed[yy*TMPVOX_XS+xx+shift] = in[yy*TMPVOX_XS+xx];
			}
		}
		else
		{
			for(int xx=-1*shift; xx<TMPVOX_XS; xx++)
			{
				tmp_skewed[yy*TMPVOX_XS+xx+shift] = in[yy*TMPVOX_XS+xx];
			}
		}
	}

	// Shift in Y direction

	for(int xx = 0; xx < TMPVOX_XS; xx++)
	{
		double fshift = ((double)(xx - TMPVOX_XMID)*step); // Left column (xx=0) shifts down with positive ang.
		int shift = (fshift>=0.0)?(fshift+0.5):(fshift-0.5);
		printf("Column xx=%d shifts by %d\n", xx, shift);

		if(shift >= 0)
		{
			for(int yy=0; yy<TMPVOX_YS-shift; yy++)
			{
				out[(yy+shift)*TMPVOX_XS+xx] = tmp_skewed[yy*TMPVOX_XS+xx];
			}
		}
		else
		{
			for(int yy=-1*shift; yy<TMPVOX_YS; yy++)
			{
				out[(yy+shift)*TMPVOX_XS+xx] = tmp_skewed[yy*TMPVOX_XS+xx];
			}
		}

	}
}

void main()
{
	static uint16_t in[20*20] =
	{
		'.','#','.','.','#','.','.','.','.','.','.','#','.','.','#','.','.','.','.','.',
		'.','#','.','.','#','.','.','.','.','.','.','#','.','.','#','.','.','.','.','.',
		'.','#','.','.','#','.','.','.','.','.','.','#','.','.','#','.','.','.','.','.',
		'.','#','.','.','#','.','.','.','.','.','.','#','.','.','#','.','.','.','.','.',
		'.','#','.','.','#','.','.','.','.','.','.','#','.','.','#','.','.','.','.','.',
		'.','#','.','.','#','.','.','.','.','.','.','#','.','.','#','.','.','.','.','.',
		'.','#','.','.','#','.','.','.','.','.','.','#','.','.','#','.','.','.','.','.',
		'.','#','.','.','#','.','.','.','.','.','.','#','.','.','#','.','.','.','.','.',
		'.','#','.','.','#','.','.','.','.','.','.','#','.','.','#','.','.','.','.','.',
		'#','#','#','#','#','#','#','#','#','#','#','#','#','#','#','#','#','#','#','#',
		'.','#','.','.','#','.','.','.','.','.','.','#','.','.','#','.','.','.','.','.',
		'.','#','.','.','#','.','.','.','.','.','.','#','.','.','#','.','.','.','.','.',
		'.','#','.','.','#','.','.','.','.','.','.','#','.','.','#','.','.','.','.','.',
		'.','#','.','.','#','.','.','.','.','.','.','#','.','.','#','.','.','.','.','.',
		'.','#','.','.','#','.','.','.','.','.','.','#','.','.','#','.','.','.','.','.',
		'.','#','.','.','#','.','1','2','3','4','.','#','.','.','#','.','.','.','.','.',
		'.','#','.','.','#','.','.','.','.','.','.','#','.','.','#','.','.','.','.','.',
		'.','#','.','.','#','.','.','.','.','.','.','#','.','.','#','.','.','.','.','.',
		'.','#','.','.','#','.','.','.','.','.','.','#','.','.','#','.','.','.','@','.',
		'.','#','.','.','#','.','.','.','.','.','.','#','.','.','#','.','.','.','.','.',
	};

	double ang = 0.0;
	int dir = 0;
	while(1)
	{
		uint16_t out[20*20] = {'.'};
		rotate_tmpvox(out, in, DEGTORAD(ang));

		for(int yy=19; yy>=0; yy--)
		{
			for(int xx=0; xx<20; xx++)
			{
				if(out[yy*20+xx] == 0)
					putchar('.');
				else
					putchar(out[yy*20+xx]);
				putchar(' ');
			}
			putchar('\n');
		}
		printf("ang = %.1f\n", ang);
		usleep(50000);

		if(dir == 0)
		{
			ang += 0.5;
			if(ang > 14.0)
			{
				dir = 1;
			}
		}
		else
		{
			ang -= 0.5;
			if(ang < -14.0)
			{
				dir = 0;
			}

		}
	}
}

