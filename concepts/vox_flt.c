#include <stdio.h>
#include <stdint.h>
#include <string.h>

#define XS 20
#define YS 25

#define ORIG_X 9
#define ORIG_Y 1

#define VC(x_, y_) ((y_)*XS+(x_))

#define O 0
#define I 0xffffffff
uint32_t range_template[XS*YS] =
{
O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,
O,O,O,O,O,O,O,O,O,I,O,O,O,O,O,O,O,O,O,O,
O,O,O,O,O,O,O,O,I,I,I,O,O,O,O,O,O,O,O,O,
O,O,O,O,O,O,O,O,I,I,I,O,O,O,O,O,O,O,O,O,
O,O,O,O,O,O,O,I,I,I,I,I,O,O,O,O,O,O,O,O,
O,O,O,O,O,O,O,I,I,I,I,I,O,O,O,O,O,O,O,O,
O,O,O,O,O,O,I,I,I,I,I,I,I,O,O,O,O,O,O,O,
O,O,O,O,O,O,I,I,I,I,I,I,I,O,O,O,O,O,O,O,
O,O,O,O,O,I,I,I,I,I,I,I,I,I,O,O,O,O,O,O,
O,O,O,O,O,I,I,I,I,I,I,I,I,I,O,O,O,O,O,O,
O,O,O,O,I,I,I,I,I,I,I,I,I,I,I,O,O,O,O,O,
O,O,O,O,I,I,I,I,I,I,I,I,I,I,I,O,O,O,O,O,
O,O,O,I,I,I,I,I,I,I,I,I,I,I,I,I,O,O,O,O,
O,O,O,I,I,I,I,I,I,I,I,I,I,I,I,I,O,O,O,O,
O,O,I,I,I,I,I,I,I,I,I,I,I,I,I,I,I,O,O,O,
O,O,I,I,I,I,I,I,I,I,I,I,I,I,I,I,I,O,O,O,
O,I,I,I,I,I,I,I,I,I,I,I,I,I,I,I,I,I,O,O,
O,I,I,I,I,I,I,I,I,I,I,I,I,I,I,I,I,I,O,O,
O,O,I,I,I,I,I,I,I,I,I,I,I,I,I,I,I,O,O,O,
O,O,O,I,I,I,I,I,I,I,I,I,I,I,I,I,O,O,O,O,
O,O,O,O,I,I,I,I,I,I,I,I,I,I,I,O,O,O,O,O,
O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,
O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,
O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,
O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O
};


void process_sensor(uint32_t* sensor, uint32_t* occ, uint32_t* fre)
{

	// Apply the range template:
	memset(occ, 0, XS*YS*sizeof(uint32_t));
	memcpy(fre, range_template, XS*YS*sizeof(uint32_t));


	for(int yy=0; yy<YS; yy++)
	{
		for(int xx=0; xx<XS; xx++)
		{
			if(sensor[VC(xx,yy)])
			{

				// Before the obstacle:

			//	occ[VC(xx-1,yy-1)] = I;
			//	fre[VC(xx-1,yy-1)] = I;

				occ[VC(xx,yy-1)] = I;
				fre[VC(xx,yy-1)] = I;

			//	occ[VC(xx+1,yy-1)] = I;
			//	fre[VC(xx+1,yy-1)] = I;

				// Obstacle itself:
				occ[VC(xx,yy)] = I;
				fre[VC(xx,yy)] = O;

				// Ray trace '00' after the obstacle
				double dx = ORIG_X-xx;
				double dy = ORIG_Y-yy;
				double dx_per_dy = dx/dy;

				double f_ray_x = xx;
				for(int ray_y=yy+1; ray_y < YS; ray_y++)
				{
					int x = f_ray_x+0.5;
					fre[VC(x, ray_y)] = O;
					if(x>0 && f_ray_x < (double)x)
						fre[VC(x-1, ray_y)] = O;
					if(x<XS-1 && f_ray_x > (double)x)
						fre[VC(x+1, ray_y)] = O;
					f_ray_x += dx_per_dy;
				}
			}
		}
	}

}

void print_occfre(uint32_t* occ, uint32_t* fre)
{
	for(int yy=0; yy<YS; yy++)
	{
		for(int xx=0; xx<XS; xx++)
		{
			if(occ[VC(xx,yy)] && fre[VC(xx,yy)])
				putchar('@');
			else if(occ[VC(xx,yy)])
				putchar('#');
			else if(fre[VC(xx,yy)])
				putchar('-');
			else
				putchar(' ');

			putchar(' ');
		}
		printf("\n");
	}
	printf("\n");
}

int main()
{

	#define N 4

	uint32_t sensor[N][XS*YS] =
	{
		{
		O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,
		O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,
		O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,
		O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,
		O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,
		O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,
		O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,
		O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,
		O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,
		O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,
		O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,
		O,O,O,O,O,O,O,I,O,O,O,O,O,O,O,O,O,O,O,O,
		O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,
		O,O,O,O,O,O,O,O,O,O,O,I,I,I,I,I,O,O,O,O,
		O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,
		O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,
		O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,
		O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,
		O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,
		O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,
		O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,
		O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,
		O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,
		O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,
		O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O
		},
		{
		O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,
		O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,
		O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,
		O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,
		O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,
		O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,
		O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,
		O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,
		O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,
		O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,
		O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,
		O,O,O,O,O,O,O,I,O,O,O,O,O,O,O,O,O,O,O,O,
		O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,
		O,O,O,O,O,O,O,O,O,O,O,I,I,I,I,I,O,O,O,O,
		O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,
		O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,
		O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,
		O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,
		O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,
		O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,
		O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,
		O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,
		O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,
		O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,
		O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O
		},
		{
		O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,
		O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,
		O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,
		O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,
		O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,
		O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,
		O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,
		O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,
		O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,
		O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,
		O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,
		O,O,O,O,O,O,O,O,I,O,O,O,O,O,O,O,O,O,O,O,
		O,O,O,O,O,O,O,O,O,O,O,O,O,O,I,O,O,O,O,O,
		O,O,O,O,O,O,O,O,O,O,O,I,I,I,O,I,O,O,O,O,
		O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,
		O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,
		O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,
		O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,
		O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,
		O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,
		O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,
		O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,
		O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,
		O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,
		O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O
		},
		{
		O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,
		O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,
		O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,
		O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,
		O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,
		O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,
		O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,
		O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,
		O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,
		O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,
		O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,
		O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,
		O,O,O,O,O,O,O,O,I,O,O,O,O,O,O,O,O,O,O,O,  // 8,12
		O,O,O,O,O,O,O,O,O,O,O,I,I,I,I,I,O,O,O,O,
		O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,
		O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,
		O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,
		O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,
		O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,
		O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,
		O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,
		O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,
		O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,
		O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,
		O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O,O
		}



	};

	uint32_t occ[N][XS*YS];
	uint32_t fre[N][XS*YS];


	for(int i=0; i<N; i++)
	{
		process_sensor(sensor[i], occ[i], fre[i]);
		print_occfre(occ[i], fre[i]);
	}

	// Combine magic:
	uint32_t occ_comb[XS*YS];
	uint32_t fre_comb[XS*YS];

	for(int yy=0; yy<YS; yy++)
	{
		for(int xx=0; xx<XS; xx++)
		{
			int n_free = 0;
			int n_occupied = 0;
			int n_unseen = 0;
			int n_unsure = 0;
			for(int n=0; n<N; n++)
			{
				if(!occ[n][VC(xx,yy)] && fre[n][VC(xx,yy)])
					n_free++;
				else if(occ[n][VC(xx,yy)] && !fre[n][VC(xx,yy)])
					n_occupied++;
				else if(!occ[n][VC(xx,yy)] && !fre[n][VC(xx,yy)])
					n_unseen++;
				else
					n_unsure++;

			}

			//if(xx==8 && yy==12)
			//	printf("n free %d  occu %d  unseen %d  unsure %d\n", n_free, n_occupied, n_unseen, n_unsure);
			if(n_free > 0)
			{
				occ_comb[VC(xx,yy)] = O;
				fre_comb[VC(xx,yy)] = I;
			}
			else if(n_occupied > 0)
			{
				occ_comb[VC(xx,yy)] = I;
				fre_comb[VC(xx,yy)] = O;
			}
			else if(n_unseen == N)
			{
				occ_comb[VC(xx,yy)] = O;
				fre_comb[VC(xx,yy)] = O;
			}
			else
			{
				occ_comb[VC(xx,yy)] = I;
				fre_comb[VC(xx,yy)] = I;
			}

		}
	}

	print_occfre(occ_comb, fre_comb);

	return 0;
}
