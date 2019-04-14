#include <stdio.h>
#include <math.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <raylib.h>


#define DEFINE_API_VARIABLES
#include "../robotsoft/api_board_to_soft.h"
#undef DEFINE_API_VARIABLES

#include "sin_lut.c"
#include "geotables.h"
#include "../robotsoft/b2s_prints.c"

#define N_SENSORS 10

#define RADTODEG(x) ((x)*(360.0/(2.0*M_PI)))
#define DEGTORAD(x) ((x)*((2.0*M_PI)/360.0))
#define DEGTOANG16(x)  ((uint16_t)((float)(x)/(360.0)*65536.0))

#define DIST_UNDEREXP 0

typedef struct
{
	int32_t mount_mode;             // mount position 1,2,3 or 4
	int32_t x_rel_robot;          // zero = robot origin. Positive = robot front (forward)
	int32_t y_rel_robot;          // zero = robot origin. Positive = to the right of the robot
	uint16_t ang_rel_robot;        // zero = robot forward direction. positive = ccw
	uint16_t vert_ang_rel_ground;  // zero = looks directly forward. positive = looks up. negative = looks down
	int32_t z_rel_ground;         // sensor height from the ground	
} sensor_mount_t;


/*
	Sensor mount position 1:
	 _ _
	| | |
	| |L|
	|O|L|
	| |L|
	|_|_|  (front view)

	Sensor mount position 2:
	 _ _
	| | |
	|L| |
	|L|O|
	|L| |
	|_|_|  (front view)

	Sensor mount position 3:

	-------------
	|  L  L  L  |
	-------------
	|     O     |
	-------------

	Sensor mount position 4:

	-------------
	|     O     |
	-------------
	|  L  L  L  |
	-------------
*/



// const
sensor_mount_t sensor_mounts[N_SENSORS] =
{          //      mountmode    x     y       hor ang           ver ang      height    
 /*0:                */ { 0,     0,     0, DEGTOANG16(       0), DEGTOANG16( 2),         300 },

 /*1:                */ { 1,   130,   103, DEGTOANG16(    24.4), DEGTOANG16( 4.4),       310  }, // -1
 /*2:                */ { 2,  -235,   215, DEGTOANG16(    66.4), DEGTOANG16( 1.4),       310  }, // -1
 /*3:                */ { 2,  -415,   215, DEGTOANG16(    93.5), DEGTOANG16( 1.9),       310  }, // -1
 /*4:                */ { 2,  -522,   103, DEGTOANG16(   157.4), DEGTOANG16( 3.9),       280  }, // -1
 /*5:                */ { 2,  -522,   -35, DEGTOANG16(   176.0), DEGTOANG16( 4.9),       290  }, // -1
 /*6:                */ { 1,  -522,  -103, DEGTOANG16(   206.0), DEGTOANG16( 4.4),       290  }, // -1
 /*7:                */ { 1,  -415,  -215, DEGTOANG16(   271.5), DEGTOANG16( 2.4),       280  }, // -1
 /*8:                */ { 1,  -235,  -215, DEGTOANG16(   294.9), DEGTOANG16( 4.4),       300  }, // -1
 /*9:                */ { 2,   130,  -103, DEGTOANG16(   334.9), DEGTOANG16( -0.9),      320  }  // 0
};

#undef Z_STEP
#undef BASE_Z
#undef MAX_Z


#define XS 2048
#define YS 2048
//#define XS 1024
//#define YS 1024
#define ZS 64
#define XY_STEP 64
#define Z_STEP 64
#define SUBMESH_RATIO 4


#define BASE_Z -200
#define MAX_Z ((Z_STEP*ZS + BASE_Z)-1)

#define sq(x) ((x)*(x))
#define abso(x) (((x)<0)?(-(x)):(x))

uint8_t voxmap[XS*YS][ZS];

void clear_voxmap()
{
	memset(voxmap, 0, sizeof(voxmap));
}

void tof_to_voxmap(int is_narrow, uint16_t* ampldist, hw_pose_t* pose, int sidx, int32_t ref_x, int32_t ref_y)
{
	if(sidx < 0 || sidx >= N_SENSORS)
	{
		printf("Invalid sidx\n");
		return;
	}

	int32_t robot_x = pose->x;
	int32_t robot_y = pose->y;

	uint16_t robot_ang = pose->ang>>16;

	static int32_t prev_x, prev_y;
	static uint16_t prev_ang;
	static int ignore = 0;

	if(sidx == 1)
	{
		int dx = robot_x - prev_x;
		int dy = robot_y - prev_y;
		int da = (int16_t)((uint16_t)robot_ang - (uint16_t)prev_ang);


		if(abso(dx) < 40 && abso(dy) < 40 && abso(da) < 700)
		{
			printf("Not much movement, ignoring until next sidx=1\n");
			ignore = 1;
		}
		else
		{
			ignore = 0;

			prev_x = robot_x;
			prev_y = robot_y;
			prev_ang = robot_ang;
		}

	}

	if(ignore)
		return;

	// Rotation: xr = x*cos(a) + y*sin(a)
	//           yr = -x*sin(a) + y*cos(a)
	// It seems to me this widely touted formula has inverted y axis, don't understand why, so it should be:
	// Rotation: xr = x*cos(a) - y*sin(a)
	//           yr = x*sin(a) + y*cos(a)


	uint16_t global_sensor_hor_ang = sensor_mounts[sidx].ang_rel_robot + robot_ang;
//	uint16_t global_sensor_ver_ang = sensor_mounts[sidx].vert_ang_rel_ground;

	int16_t pitch_ang = pose->pitch>>16;
	int16_t roll_ang = pose->roll>>16;

	uint16_t global_sensor_ver_ang = 
		(int32_t)((int16_t)sensor_mounts[sidx].vert_ang_rel_ground) +
		((lut_cos_from_u16(sensor_mounts[sidx].ang_rel_robot)*pitch_ang)>>SIN_LUT_RESULT_SHIFT) +
		((lut_sin_from_u16(sensor_mounts[sidx].ang_rel_robot)*roll_ang)>>SIN_LUT_RESULT_SHIFT);


	uint16_t local_sensor_hor_ang = sensor_mounts[sidx].ang_rel_robot;
	uint16_t local_sensor_ver_ang = sensor_mounts[sidx].vert_ang_rel_ground;

	int32_t  global_sensor_x = robot_x - ref_x +
			((lut_cos_from_u16(robot_ang)*sensor_mounts[sidx].x_rel_robot)>>SIN_LUT_RESULT_SHIFT) +
			((lut_sin_from_u16(robot_ang)*-1*sensor_mounts[sidx].y_rel_robot)>>SIN_LUT_RESULT_SHIFT);

	int32_t  global_sensor_y = robot_y - ref_y + 
			((lut_sin_from_u16(robot_ang)*sensor_mounts[sidx].x_rel_robot)>>SIN_LUT_RESULT_SHIFT) +
			((lut_cos_from_u16(robot_ang)*sensor_mounts[sidx].y_rel_robot)>>SIN_LUT_RESULT_SHIFT);
	int32_t  global_sensor_z = sensor_mounts[sidx].z_rel_ground;


	int32_t  local_sensor_x = sensor_mounts[sidx].x_rel_robot;
	int32_t  local_sensor_y = sensor_mounts[sidx].y_rel_robot;
	int32_t  local_sensor_z = sensor_mounts[sidx].z_rel_ground;


	int insertion_cnt = 0;
	for(int py=1; py<TOF_YS-1; py++)
//	for(int py=29; py<32; py++)
	{
		for(int px=1; px<TOF_XS-1; px++)
//		for(int px=75; px<85; px++)
//		for(int px=79; px<82; px++)
		{
			int32_t avg = 0;
			int n_conform = 0;
			if(is_narrow)
			{
				int npy, npx;
				npx=px-TOF_NARROW_X_START;
				npy=py-TOF_NARROW_Y_START;
				if(npx < 1 || npy < 1 || npx >= TOF_XS_NARROW-1 || npx >= TOF_YS_NARROW-1)
					continue;

//						int32_t dist = (ampldist[(npy+iy)*TOF_XS_NARROW+(npx+ix)]&DIST_MASK)<<DIST_SHIFT;
				return; // todo: implement

			}
			else
			{
				int32_t refdist = ampldist[(py+0)*TOF_XS+(px+0)]&DIST_MASK;
				if(refdist == DIST_UNDEREXP)
					continue;

				for(int iy=-1; iy<=1; iy++)
				{
					for(int ix=-1; ix<=1; ix++)
					{
						int32_t dist = ampldist[(py+iy)*TOF_XS+(px+ix)]&DIST_MASK;
						if(dist != DIST_UNDEREXP && dist > refdist-100 && dist < refdist+100)
						{
							avg+=dist;
							n_conform++;
						}
					
					}
				}
			}

			avg <<= DIST_SHIFT;
			avg /= n_conform;

			if(n_conform >= 7)
			{
				int32_t d = avg;

				uint16_t hor_ang, ver_ang;


				// TODO: This optimizes out once we have sensor-by-sensor geometric tables;
				// they can be pre-built to the actual mount_mode.
				switch(sensor_mounts[sidx].mount_mode)
				{
					case 1: 
					hor_ang = -1*geocoords[py*TOF_XS+px].yang;
					ver_ang = geocoords[py*TOF_XS+px].xang;
					break;

					case 2: 
					hor_ang = geocoords[py*TOF_XS+px].yang;
					ver_ang = -1*geocoords[py*TOF_XS+px].xang;
					break;

					case 3:
					hor_ang = -1*geocoords[py*TOF_XS+px].xang;
					ver_ang = geocoords[py*TOF_XS+px].yang;
					break;

					case 4:
					hor_ang = geocoords[py*TOF_XS+px].xang;
					ver_ang = -1*geocoords[py*TOF_XS+px].yang;
					break;

					default: return;
				}

				uint16_t comb_hor_ang = hor_ang + global_sensor_hor_ang;
				uint16_t comb_ver_ang = ver_ang + global_sensor_ver_ang;

				int32_t x = (((int64_t)d * (int64_t)lut_cos_from_u16(comb_ver_ang) * (int64_t)lut_cos_from_u16(comb_hor_ang))>>(2*SIN_LUT_RESULT_SHIFT)) + global_sensor_x;
				int32_t y = (((int64_t)d * (int64_t)lut_cos_from_u16(comb_ver_ang) * (int64_t)lut_sin_from_u16(comb_hor_ang))>>(2*SIN_LUT_RESULT_SHIFT)) + global_sensor_y;
				int32_t z = (((int64_t)d * (int64_t)lut_sin_from_u16(comb_ver_ang))>>SIN_LUT_RESULT_SHIFT) + global_sensor_z;

				uint16_t local_comb_hor_ang = hor_ang + local_sensor_hor_ang;
				uint16_t local_comb_ver_ang = ver_ang + local_sensor_ver_ang;


				int32_t local_x = (((int64_t)d * (int64_t)lut_cos_from_u16(local_comb_ver_ang) * (int64_t)lut_cos_from_u16(local_comb_hor_ang))>>(2*SIN_LUT_RESULT_SHIFT)) + local_sensor_x;
				int32_t local_y = (((int64_t)d * (int64_t)lut_cos_from_u16(local_comb_ver_ang) * (int64_t)lut_sin_from_u16(local_comb_hor_ang))>>(2*SIN_LUT_RESULT_SHIFT)) + local_sensor_y;
				int32_t local_z = (((int64_t)d * (int64_t)lut_sin_from_u16(local_comb_ver_ang))>>SIN_LUT_RESULT_SHIFT) + local_sensor_z;

				// VACUUM APP: Ignore the nozzle
				#define NOZZLE_WIDTH 760
				if(local_z < 200 && local_x < 520 && local_x > 120 && local_y > -(NOZZLE_WIDTH/2) && local_y < (NOZZLE_WIDTH/2))
					continue;

				// Completely ignore nozzle area obstacles for mapping, but give the floor if visible!
				if(local_z > 100 && local_x < 520 && local_x > 120 && local_y > -(NOZZLE_WIDTH/2) && local_y < (NOZZLE_WIDTH/2))
					continue;


				int z_coord = ((z-BASE_Z)/Z_STEP);
				
				x /= XY_STEP;
				y /= XY_STEP;

				x += XS/2;
				y += YS/2;

				if(x < 0 || x >= XS || y < 0 || y >= YS || z_coord < 0 || z_coord >= ZS)
				{
				//	printf("Ignore OOR point x=%d, y=%d, z=%d\n", x, y, z_coord);
					continue;

				}

				int oldval = voxmap[y*XS+x][z_coord];
				int increment = d/1000;
				if(increment < 1) increment = 1;
				else if(increment > 10) increment = 10;
				int newval = oldval + increment;
				if(newval > 255)
					newval = 255;
				voxmap[y*XS+x][z_coord] = newval;
			}
		}
	}

}


int process_file(char* fname)
{
	FILE* fil = fopen(fname, "rb");
	if(!fil)
	{
		printf("Error opening file %s for read\n", fname);
	}

	static uint8_t buf[B2S_MAX_LEN];

	int n_bytes_read = fread(buf, 1, B2S_MAX_LEN, fil);

	fclose(fil);

//	printf("Read %d bytes\n", n_bytes_read);
	if(n_bytes_read < B2S_TOTAL_OVERHEAD_WITHOUT_CRC)
	{
		printf("ERROR: File is too short.\n");
		return -2;
	}

	int expected_size = ((b2s_header_t*)buf)->payload_len + B2S_TOTAL_OVERHEAD_WITHOUT_CRC;

	if(n_bytes_read != expected_size)
	{
		printf("ERROR: File size vs. header information mismatch, bytes_read=%d, expected_size=%d\n", n_bytes_read, expected_size);
		return -3;
	}


	uint8_t* p_data = buf;

//	printf("Got something! Messages:\n");

	static int first = 1;
	static int base_x, base_y;

	int offs = sizeof(b2s_header_t);
	for(int i=0; i<B2S_SUBS_U64_ITEMS; i++)
	{
		uint64_t t = ((b2s_header_t*)p_data)->subs[i];
		for(int s=i*64; s<(i+1)*64; s++)
		{
			if(t & 1)
			{
				// id #s is enabled
//				printf("msgid=%u  name=%s  comment=%s\n", s, b2s_msgs[s].name, b2s_msgs[s].comment);
//				if(b2s_msgs[s].p_print)
//					b2s_msgs[s].p_print(&p_data[offs]);

				if(s==14)
				{
					tof_slam_set_t* tss = (tof_slam_set_t*)&p_data[offs];
					if(first)
					{
						base_x = tss->sets[0].pose.x;
						base_y = tss->sets[0].pose.y;
						first = 0;
					}
					tof_to_voxmap(0, tss->sets[0].ampldist, &tss->sets[0].pose, tss->sidx, base_x, base_y);

//					if(tss->flags & TOF_SLAM_SET_FLAG_SET1_WIDE)
//						tof_to_voxmap(0, tss->sets[1].ampldist, &tss->sets[1].pose, tss->sidx, base_x, base_y);
//					if(tss->flags & TOF_SLAM_SET_FLAG_SET1_NARROW)
//						tof_to_voxmap(1, tss->sets[1].ampldist, &tss->sets[1].pose, tss->sidx, base_x, base_y);


				}

				offs += b2s_msgs[s].size;

			}
			t >>= 1;
		}
	}

	return 0;

}

#include "../raylib/src/rlgl.h"

void OmaCube(Vector3 position, float width, float height, float length, Color color)
{
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;

    if (rlCheckBufferLimit(36)) rlglDraw();

    rlPushMatrix();
        // NOTE: Transformation is applied in inverse order (scale -> rotate -> translate)
        rlTranslatef(position.x, position.y, position.z);
        //rlRotatef(45, 0, 1, 0);
        //rlScalef(1.0f, 1.0f, 1.0f);   // NOTE: Vertices are directly scaled on definition

        rlBegin(RL_TRIANGLES);
            rlColor4ub(color.r, color.g, color.b, color.a);

            // Front face
            rlVertex3f(x - width/2, y - height/2, z + length/2);  // Bottom Left
            rlVertex3f(x + width/2, y - height/2, z + length/2);  // Bottom Right
            rlVertex3f(x - width/2, y + height/2, z + length/2);  // Top Left

            rlVertex3f(x + width/2, y + height/2, z + length/2);  // Top Right
            rlVertex3f(x - width/2, y + height/2, z + length/2);  // Top Left
            rlVertex3f(x + width/2, y - height/2, z + length/2);  // Bottom Right

            // Back face
            rlVertex3f(x - width/2, y - height/2, z - length/2);  // Bottom Left
            rlVertex3f(x - width/2, y + height/2, z - length/2);  // Top Left
            rlVertex3f(x + width/2, y - height/2, z - length/2);  // Bottom Right

            rlVertex3f(x + width/2, y + height/2, z - length/2);  // Top Right
            rlVertex3f(x + width/2, y - height/2, z - length/2);  // Bottom Right
            rlVertex3f(x - width/2, y + height/2, z - length/2);  // Top Left

            // Top face
            rlVertex3f(x - width/2, y + height/2, z - length/2);  // Top Left
            rlVertex3f(x - width/2, y + height/2, z + length/2);  // Bottom Left
            rlVertex3f(x + width/2, y + height/2, z + length/2);  // Bottom Right

            rlVertex3f(x + width/2, y + height/2, z - length/2);  // Top Right
            rlVertex3f(x - width/2, y + height/2, z - length/2);  // Top Left
            rlVertex3f(x + width/2, y + height/2, z + length/2);  // Bottom Right

            // Bottom face
            rlVertex3f(x - width/2, y - height/2, z - length/2);  // Top Left
            rlVertex3f(x + width/2, y - height/2, z + length/2);  // Bottom Right
            rlVertex3f(x - width/2, y - height/2, z + length/2);  // Bottom Left

            rlVertex3f(x + width/2, y - height/2, z - length/2);  // Top Right
            rlVertex3f(x + width/2, y - height/2, z + length/2);  // Bottom Right
            rlVertex3f(x - width/2, y - height/2, z - length/2);  // Top Left

            // Right face
            rlVertex3f(x + width/2, y - height/2, z - length/2);  // Bottom Right
            rlVertex3f(x + width/2, y + height/2, z - length/2);  // Top Right
            rlVertex3f(x + width/2, y + height/2, z + length/2);  // Top Left

            rlVertex3f(x + width/2, y - height/2, z + length/2);  // Bottom Left
            rlVertex3f(x + width/2, y - height/2, z - length/2);  // Bottom Right
            rlVertex3f(x + width/2, y + height/2, z + length/2);  // Top Left

            // Left face
            rlVertex3f(x - width/2, y - height/2, z - length/2);  // Bottom Right
            rlVertex3f(x - width/2, y + height/2, z + length/2);  // Top Left
            rlVertex3f(x - width/2, y + height/2, z - length/2);  // Top Right

            rlVertex3f(x - width/2, y - height/2, z + length/2);  // Bottom Left
            rlVertex3f(x - width/2, y + height/2, z + length/2);  // Top Left
            rlVertex3f(x - width/2, y - height/2, z - length/2);  // Bottom Right

        rlEnd();
    rlPopMatrix();
}

/*
Mesh GenMeshCube(float width, float height, float length)
{
    Mesh mesh = { 0 };

    float vertices[] = {
        -width/2, -height/2, length/2,
        width/2, -height/2, length/2,
        width/2, height/2, length/2,
        -width/2, height/2, length/2,
        -width/2, -height/2, -length/2,
        -width/2, height/2, -length/2,
        width/2, height/2, -length/2,
        width/2, -height/2, -length/2,
        -width/2, height/2, -length/2,
        -width/2, height/2, length/2,
        width/2, height/2, length/2,
        width/2, height/2, -length/2,
        -width/2, -height/2, -length/2,
        width/2, -height/2, -length/2,
        width/2, -height/2, length/2,
        -width/2, -height/2, length/2,
        width/2, -height/2, -length/2,
        width/2, height/2, -length/2,
        width/2, height/2, length/2,
        width/2, -height/2, length/2,
        -width/2, -height/2, -length/2,
        -width/2, -height/2, length/2,
        -width/2, height/2, length/2,
        -width/2, height/2, -length/2
    };

    float texcoords[] = {
        0.0f, 0.0f,
        1.0f, 0.0f,
        1.0f, 1.0f,
        0.0f, 1.0f,
        1.0f, 0.0f,
        1.0f, 1.0f,
        0.0f, 1.0f,
        0.0f, 0.0f,
        0.0f, 1.0f,
        0.0f, 0.0f,
        1.0f, 0.0f,
        1.0f, 1.0f,
        1.0f, 1.0f,
        0.0f, 1.0f,
        0.0f, 0.0f,
        1.0f, 0.0f,
        1.0f, 0.0f,
        1.0f, 1.0f,
        0.0f, 1.0f,
        0.0f, 0.0f,
        0.0f, 0.0f,
        1.0f, 0.0f,
        1.0f, 1.0f,
        0.0f, 1.0f
    };

    float normals[] = {
        0.0f, 0.0f, 1.0f,
        0.0f, 0.0f, 1.0f,
        0.0f, 0.0f, 1.0f,
        0.0f, 0.0f, 1.0f,
        0.0f, 0.0f,-1.0f,
        0.0f, 0.0f,-1.0f,
        0.0f, 0.0f,-1.0f,
        0.0f, 0.0f,-1.0f,
        0.0f, 1.0f, 0.0f,
        0.0f, 1.0f, 0.0f,
        0.0f, 1.0f, 0.0f,
        0.0f, 1.0f, 0.0f,
        0.0f,-1.0f, 0.0f,
        0.0f,-1.0f, 0.0f,
        0.0f,-1.0f, 0.0f,
        0.0f,-1.0f, 0.0f,
        1.0f, 0.0f, 0.0f,
        1.0f, 0.0f, 0.0f,
        1.0f, 0.0f, 0.0f,
        1.0f, 0.0f, 0.0f,
        -1.0f, 0.0f, 0.0f,
        -1.0f, 0.0f, 0.0f,
        -1.0f, 0.0f, 0.0f,
        -1.0f, 0.0f, 0.0f
    };

    mesh.vertices = (float *)malloc(24*3*sizeof(float));
    memcpy(mesh.vertices, vertices, 24*3*sizeof(float));

    mesh.texcoords = (float *)malloc(24*2*sizeof(float));
    memcpy(mesh.texcoords, texcoords, 24*2*sizeof(float));

    mesh.normals = (float *)malloc(24*3*sizeof(float));
    memcpy(mesh.normals, normals, 24*3*sizeof(float));

    mesh.indices = (unsigned short *)malloc(36*sizeof(unsigned short));

    int k = 0;

    // Indices can be initialized right now
    for (int i = 0; i < 36; i+=6)
    {
        mesh.indices[i] = 4*k;
        mesh.indices[i+1] = 4*k+1;
        mesh.indices[i+2] = 4*k+2;
        mesh.indices[i+3] = 4*k;
        mesh.indices[i+4] = 4*k+2;
        mesh.indices[i+5] = 4*k+3;

        k++;
    }

    mesh.vertexCount = 24;
    mesh.triangleCount = 12;

    // Upload vertex data to GPU (static mesh)
    rlLoadMesh(&mesh, false);

    return mesh;
}
*/

//  466018 KB ---> 108235 KB by just hiding faces

//#define TEST_VOXMAP(x_, y_, z_) (((z_) < 32 && voxmap[(y_)*XS+(x_)][(z_)] > 40) || ((z_) >= 32 && voxmap[(y_)*XS+(x_)][(z_)] > 10))
#define TEST_VOXMAP(x_, y_, z_) (((z_) < 32 && voxmap[(y_)*XS+(x_)][(z_)] > 0) || ((z_) >= 32 && voxmap[(y_)*XS+(x_)][(z_)] > 0))
Mesh gen_subarea_mesh(int x_start, int y_start, int xs, int ys)
{
	static int32_t totmem = 0;
	Mesh bigmesh = {0};

	int n_cubes = 0;

	int y_end = y_start+ys;
	if(y_end > YS) y_end = YS;

	int x_end = x_start+xs;
	if(x_end > XS) x_end = XS;

	for(int yy=y_start; yy<y_end; yy++)
	{
		for(int xx=x_start; xx<x_end; xx++)
		{
			for(int zz=0; zz<ZS; zz++)
			{
				if(TEST_VOXMAP(xx,yy,zz))
				{
					n_cubes++;
				}
			}
		}
	}

	int n_triangles = n_cubes*12;
	int n_vertices = n_triangles*2;
	int n_indices = n_cubes*36;


	if(n_indices > 65535)
	{
		printf("ERROR generation, too many indices\n");
		return bigmesh;
	}


	if(n_vertices > 0)
		printf("Voxel cube mesh generation: n_cubes=%d, n_triangles = %d, n_vertices=%d, n_indices=%d\n", n_cubes, n_triangles, n_vertices, n_indices);
	else
	{
		return bigmesh;
	}

	bigmesh.vertices = (float *)malloc(n_vertices*3*sizeof(float));
//	bigmesh.texcoords = (float *)malloc(n_vertices*2*sizeof(float));
	bigmesh.normals = (float *)malloc(n_vertices*3*sizeof(float));
	bigmesh.indices = (unsigned short *)malloc(n_indices*sizeof(unsigned short));
	bigmesh.colors = (unsigned char *)malloc(n_vertices*4*sizeof(unsigned char));

	if(!bigmesh.vertices || !bigmesh.normals || !bigmesh.indices || !bigmesh.colors)
	{
		printf("Out of memory\n");
		return bigmesh;
	}
	
	const float width=1.0;
	const float height=1.0;
	const float length=1.0;

    const float vertices[] = {
        -width/2, -height/2, length/2,
        width/2, -height/2, length/2,
        width/2, height/2, length/2,
        -width/2, height/2, length/2,
        -width/2, -height/2, -length/2,
        -width/2, height/2, -length/2,
        width/2, height/2, -length/2,
        width/2, -height/2, -length/2,
        -width/2, height/2, -length/2,
        -width/2, height/2, length/2,
        width/2, height/2, length/2,
        width/2, height/2, -length/2,
        -width/2, -height/2, -length/2,
        width/2, -height/2, -length/2,
        width/2, -height/2, length/2,
        -width/2, -height/2, length/2,
        width/2, -height/2, -length/2,
        width/2, height/2, -length/2,
        width/2, height/2, length/2,
        width/2, -height/2, length/2,
        -width/2, -height/2, -length/2,
        -width/2, -height/2, length/2,
        -width/2, height/2, length/2,
        -width/2, height/2, -length/2
    };

    const float normals[] = {
        0.0f, 0.0f, 1.0f,
        0.0f, 0.0f, 1.0f,
        0.0f, 0.0f, 1.0f,
        0.0f, 0.0f, 1.0f,
        0.0f, 0.0f,-1.0f,
        0.0f, 0.0f,-1.0f,
        0.0f, 0.0f,-1.0f,
        0.0f, 0.0f,-1.0f,
        0.0f, 1.0f, 0.0f,
        0.0f, 1.0f, 0.0f,
        0.0f, 1.0f, 0.0f,
        0.0f, 1.0f, 0.0f,
        0.0f,-1.0f, 0.0f,
        0.0f,-1.0f, 0.0f,
        0.0f,-1.0f, 0.0f,
        0.0f,-1.0f, 0.0f,
        1.0f, 0.0f, 0.0f,
        1.0f, 0.0f, 0.0f,
        1.0f, 0.0f, 0.0f,
        1.0f, 0.0f, 0.0f,
        -1.0f, 0.0f, 0.0f,
        -1.0f, 0.0f, 0.0f,
        -1.0f, 0.0f, 0.0f,
        -1.0f, 0.0f, 0.0f
    };

/*
	for(int i=0; i<n_cubes; i++)
	{
		memcpy(&bigmesh.normals[24*3*i], normals, 24*3*sizeof(float));
	}
*/

	int v = 0;
	int norm = 0;
	int index = 0;
	int col = 0;
	int n_tri = 0;
	for(int yy=y_start; yy<y_end; yy++)
	{
		for(int xx=x_start; xx<x_end; xx++)
		{
			for(int zz=0; zz<ZS; zz++)
			{
				if(TEST_VOXMAP(xx,yy,zz))
				{

					/*
						Faces:
						0 = neg vox_y side
						1 = pos vox_y side
						2 = top
						3 = bottom
						4 = pos vox_x side
						5 = neg vox_x side
						
					*/
					int ignore_faces[6] = {0};

					if(xx > 0 && TEST_VOXMAP(xx-1, yy, zz))
						ignore_faces[5] = 1;

					if(xx < XS-1 && TEST_VOXMAP(xx+1, yy, zz))
						ignore_faces[4] = 1;

					if(yy > 0 && TEST_VOXMAP(xx, yy-1, zz))
						ignore_faces[0] = 1;

					if(yy < YS-1 && TEST_VOXMAP(xx, yy+1, zz))
						ignore_faces[1] = 1;

					if(zz > 0 && TEST_VOXMAP(xx, yy, zz-1))
						ignore_faces[3] = 1;

					if(zz < ZS-1 && TEST_VOXMAP(xx, yy, zz+1))
						ignore_faces[2] = 1;

					int r, g, b;
					r = (ZS-2*zz)*(256/ZS);
					if(r < 0) r=0; else if(r>255) r=255;

					b = -1*(ZS-2*zz)*(256/ZS);
					if(b < 0) b=0; else if(b>255) b=255;

					g = 256 - r - b;
					if(g < 0) g=0; else if(g>255) g=255;

					int alpha = 255;

					float x = xx-XS/2;
					float y = zz;
					float z = -1*(yy-YS/2);

					//x = 0.0;
					//y = 0.0;
					//z = 0.0;


					for(int face=0; face<6; face++)
					{
						if(ignore_faces[face])
							continue;

						n_tri += 2;
						int v_first=v/3;

						for(int i=face*4; i<(face+1)*4; i++)
						{
							bigmesh.vertices[v++] = vertices[i*3] + x;
							bigmesh.vertices[v++] = vertices[i*3+1] + y;
							bigmesh.vertices[v++] = vertices[i*3+2] + z;

							bigmesh.normals[norm++] = normals[i*3];
							bigmesh.normals[norm++] = normals[i*3+1];
							bigmesh.normals[norm++] = normals[i*3+2];

							bigmesh.colors[col++] = r;
							bigmesh.colors[col++] = g;
							bigmesh.colors[col++] = b;
							bigmesh.colors[col++] = alpha;

						}

//						for(int i = face*6; i < (face+1)*6; i+=6)
						{
							bigmesh.indices[index++] = v_first;  //+4*k;
							bigmesh.indices[index++] = v_first+1; //+4*k+1;
							bigmesh.indices[index++] = v_first+2; // +4*k+2;
							bigmesh.indices[index++] = v_first;   //+4*k;
							bigmesh.indices[index++] = v_first+2; //+4*k+2;
							bigmesh.indices[index++] = v_first+3; //+4*k+3;
						}

					}

					//goto JEES2;

				}
				
			}
		}
	}

	bigmesh.vertexCount = v;
	bigmesh.triangleCount = n_tri;

	int32_t mem = (v*sizeof(float)+norm*sizeof(float)+index*sizeof(unsigned short)+col*sizeof(unsigned char));
	totmem += mem;
	printf("Mesh generation done. v=%d norm=%d index=%d col=%d. Total memory: %d bytes, %d KB so far\n", v, norm, index, col, mem, totmem/1024);


#if 0
	for(int i=0; i<2*24*3; i++)
	{
		printf("%d  vertex = %.1f\n", i, bigmesh.vertices[i]);
	}

	for(int i=0; i<2*36; i++)
	{
		printf("%d  index = %d\n", i, bigmesh.indices[i]);
	}

	for(int i=0; i<2*24*3; i++)
	{
		printf("%d  normal = %.1f\n", i, bigmesh.normals[i]);
	}
#endif

	rlLoadMesh(&bigmesh, false);

	free(bigmesh.vertices);
	free(bigmesh.normals);
	free(bigmesh.indices);
	free(bigmesh.colors);

	return bigmesh;
}

// Draw a model with extended parameters
void OmaDrawModelEx(Model model, Vector3 position, Vector3 rotationAxis, float rotationAngle, Vector3 scale, Color tint)
{
    // Calculate transformation matrix from function parameters
    // Get transform matrix (rotation -> scale -> translation)
    Matrix matScale = MatrixScale(scale.x, scale.y, scale.z);
    Matrix matRotation = MatrixRotate(rotationAxis, rotationAngle*DEG2RAD);
    Matrix matTranslation = MatrixTranslate(position.x, position.y, position.z);

    Matrix matTransform = MatrixMultiply(MatrixMultiply(matScale, matRotation), matTranslation);

    // Combine model transformation matrix (model.transform) with matrix generated by function parameters (matTransform)
    //Matrix matModel = MatrixMultiply(model.transform, matTransform);    // Transform to world-space coordinates

    model.transform = MatrixMultiply(model.transform, matTransform);

//        model.materials[model.meshMaterial[i]].maps[MAP_DIFFUSE].color = tint;
    rlDrawMesh(model.mesh, model.material, model.transform);
}

void OmaDrawModel(Model model, Vector3 position, float scale, Color tint)
{
    Vector3 vScale = { scale, scale, scale };
    Vector3 rotationAxis = { 0.0f, 0.0f, 0.0f };

    OmaDrawModelEx(model, position, rotationAxis, 0.0f, vScale, tint);
}


int main()
{
	int screenWidth = 1920;
	int screenHeight = 1080;

	InitWindow(screenWidth, screenHeight, "Nakkikoivu");
	ToggleFullscreen();
	
	Camera camera = { 0 };
	camera.position = (Vector3){ 4.0f, 30.0f, 4.0f };
	camera.target = (Vector3){ 10.0f, 30.0f, 10.0f };
	camera.up = (Vector3){ 0.0f, 1.0f, 0.0f };
	camera.fovy = 90.0f;
	camera.type = CAMERA_PERSPECTIVE;

	
	
	SetCameraMode(camera, CAMERA_CUSTOM);

	SetTargetFPS(30);


	int camrota = 0;
	Vector2 camrota_start;
	double camrota_start_yaw;
	double camrota_start_vertang;

	double camera_yaw = 0.0;
	double camera_vertang = 0.0;


	clear_voxmap();
//	for(int i=13000; i<27000; i++)
	for(int i=500; i<2000; i++)
	{
		char fname[1024];
		sprintf(fname, "/home/hrst/robotsoft/tsellari/trace%08d.rb2", i);
		process_file(fname);
	}

/*
	voxmap[1024*2048+1024][32] = 50;
	voxmap[1024*2048+1024][36] = 50;
	voxmap[1024*2048+1024][40] = 50;
	voxmap[1024*2048+1034][32] = 50;
	voxmap[1024*2048+1034][36] = 50;
	voxmap[1024*2048+1034][40] = 50;
*/

	static Mesh subarea_meshes[(XS/SUBMESH_RATIO)*(YS/SUBMESH_RATIO)];
	static Model subarea_models[(XS/SUBMESH_RATIO)*(YS/SUBMESH_RATIO)] = {0};

	for(int subyy=0; subyy<YS/SUBMESH_RATIO; subyy++)
	{
		for(int subxx=0; subxx<XS/SUBMESH_RATIO; subxx++)
		{
			subarea_meshes[subyy*(XS/SUBMESH_RATIO)+subxx] = gen_subarea_mesh(subxx*SUBMESH_RATIO, subyy*SUBMESH_RATIO, SUBMESH_RATIO, SUBMESH_RATIO);

			if(subarea_meshes[subyy*(XS/SUBMESH_RATIO)+subxx].vertexCount > 0)
			{
				subarea_models[subyy*(XS/SUBMESH_RATIO)+subxx] = LoadModelFromMesh(subarea_meshes[subyy*(XS/SUBMESH_RATIO)+subxx]);
    //material.maps[MAP_NORMAL].texture;         // NOTE: By default, not set
    //material.maps[MAP_SPECULAR].texture;       // NOTE: By default, not set

    				subarea_models[subyy*(XS/SUBMESH_RATIO)+subxx].material.maps[MAP_NORMAL].color = WHITE;
    				subarea_models[subyy*(XS/SUBMESH_RATIO)+subxx].material.maps[MAP_DIFFUSE].color = WHITE;
    				subarea_models[subyy*(XS/SUBMESH_RATIO)+subxx].material.maps[MAP_SPECULAR].color = WHITE;

			}
		}
	}

	int crap = 1;
	while (!WindowShouldClose())
	{

		if(IsFileDropped())
		{
			printf("Got file(s):\n");
			int n_files = 0;
			char** files = GetDroppedFiles(&n_files);

			clear_voxmap();

			for(int f=0; f<n_files; f++)
			{
				printf("%s\n", files[f]);
				process_file(files[f]);
			}

			ClearDroppedFiles();
		}

		if(IsMouseButtonDown(MOUSE_LEFT_BUTTON))
		{
			if(camrota == 0)
			{
				camrota = 1;
				camrota_start = GetMousePosition();
				camrota_start_yaw = camera_yaw;
				camrota_start_vertang = camera_vertang;
			}
			else
			{
				Vector2 cur_mouse = GetMousePosition();
				//printf("Mouse dx=%.1f, dy=%.1f\n", camrota_start.x-cur_mouse.x, camrota_start.y-cur_mouse.y);

				float mouse_dx = camrota_start.x-cur_mouse.x;
				float mouse_dy = camrota_start.y-cur_mouse.y;
				camera_yaw = camrota_start_yaw + mouse_dx*-0.002;
				camera_vertang = camrota_start_vertang + mouse_dy*-0.002;
			}

		}
		else
		{
			camrota = 0;
		}

		if(IsMouseButtonDown(MOUSE_RIGHT_BUTTON))
		{
		}

		double speed = 2.0;

		if(IsKeyDown(KEY_LEFT_SHIFT) || IsKeyDown(KEY_RIGHT_SHIFT))
		{
			speed *= 2.0;
		}


		if(IsKeyDown(KEY_W))
		{
			camera.position.z += speed*cos(camera_yaw);
			camera.position.x += speed*sin(camera_yaw);
		}
		if(IsKeyDown(KEY_S))
		{
			camera.position.z += -speed*cos(camera_yaw);
			camera.position.x += -speed*sin(camera_yaw);
		}
		if(IsKeyDown(KEY_A))
		{
			camera.position.z += speed*cos(camera_yaw+(M_PI/2.0));
			camera.position.x += speed*sin(camera_yaw+(M_PI/2.0));
		}
		if(IsKeyDown(KEY_D))
		{
			camera.position.z += speed*cos(camera_yaw-(M_PI/2.0));
			camera.position.x += speed*sin(camera_yaw-(M_PI/2.0));
		}

		if(IsKeyDown(KEY_F))
		{
			camera.position.y += speed;
		}
		if(IsKeyDown(KEY_V))
		{
			camera.position.y += -speed;
		}

		if(IsKeyDown(KEY_UP))
		{
			camera_vertang += 0.02*speed;
			if(camera_vertang > 1.5) camera_vertang = 1.5;

		}
		if(IsKeyDown(KEY_DOWN))
		{
			camera_vertang -= 0.02*speed;
			if(camera_vertang < -1.5) camera_vertang = -1.5;

		}

		if(IsKeyDown(KEY_LEFT))
		{
			camera_yaw += 0.05*speed;
		}
		if(IsKeyDown(KEY_RIGHT))
		{
			camera_yaw -= 0.05*speed;
		}



		camera.target.z = camera.position.z + 50.0*cos(camera_yaw);
		camera.target.x = camera.position.x + 50.0*sin(camera_yaw);
		camera.target.y = camera.position.y + 50.0*sin(camera_vertang);


		BeginDrawing();

		ClearBackground(BLACK);


		BeginMode3D(camera);


		int cur_x = camera.position.x + XS/2;
		int cur_y = -1*camera.position.z + YS/2;

		for(int subyy=0; subyy<YS/SUBMESH_RATIO; subyy++)
		{
			for(int subxx=0; subxx<XS/SUBMESH_RATIO; subxx++)
			{
				if(subarea_meshes[subyy*(XS/SUBMESH_RATIO)+subxx].vertexCount > 0)
				{
					OmaDrawModel(subarea_models[subyy*(XS/SUBMESH_RATIO)+subxx],(Vector3){0,0,0}, 1.0, (Color){128, 128, 128,255});
				}
			
			}
		}


//		DrawPlane((Vector3){ 0.0f, 0.0f, 0.0f }, (Vector2){ 200.0f, 200.0f }, LIGHTGRAY); // Draw ground
		
/*
		int cur_start = 0;
		while(1)
		{
			int cur_end = cur_start+256;
			if(cur_end > n_cubes)
				cur_end = n_cubes;

			for(int i=cur_start; i<cur_end; i++)
				DrawCube(positions[i], 0.5f, heights[i], 0.5f, (Vector4){128,128,128,128});
			

			EndMode3D();
			if(cur_end >= n_cubes)
				break;
			BeginMode3D(camera);
			cur_start = cur_end;
		}
*/

//		OmaCube((Vector3){5.0, 5.0, 5.0}, 1.0, 1.0, 1.0, (Color){128,0,128,128});

/*
		int drawn=0;
		for(int yy=0; yy<YS; yy+=crap)
		{
			for(int xx=0; xx<XS; xx+=crap)
			{
				for(int zz=0; zz<64; zz+=crap)
				{
					if((zz < 32 && voxmap[yy*XS+xx][zz] > 20) || (zz >= 32 && voxmap[yy*XS+xx][zz] > 4))
					{
						int r, g, b;
						r = (32-zz)*8;
						if(r < 0) r=0; else if(r>255) r=255;

						b = (zz-31)*8;
						if(b < 0) b=0; else if(b>255) b=255;

						g = 256 - r - b;
						if(g < 0) g=0; else if(g>255) g=255;


						DrawCube((Vector3){xx-XS/2, zz, -1*(yy-YS/2)}, crap, crap, crap, (Color){r, g, b,255});
						drawn++;
						if(drawn > 256)
						{
							EndMode3D();
							BeginMode3D(camera);
							drawn = 0;
						}
					}
					
				}
			}
		}
*/


		EndMode3D();

		
		DrawRectangle( 10, 10, 220, 70, Fade(SKYBLUE, 0.5f));
		DrawRectangleLines( 10, 10, 220, 70, BLUE);

		char coords[1024];
		sprintf(coords, "cur_x=%d, cur_y=%d", cur_x, cur_y);
		DrawText(coords, 20, 20, 10, BLACK);

		EndDrawing();

	}

	CloseWindow();		// Close window and OpenGL context

	return 0;
} 
