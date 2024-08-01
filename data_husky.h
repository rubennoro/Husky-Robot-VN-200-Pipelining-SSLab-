#ifndef CUSTOM_PDO_NAME_H
#define CUSTOM_PDO_NAME_H

//-------------------------------------------------------------------//
//                                                                   //
//     This file has been created by the Easy Configurator tool      //
//                                                                   //
//     Easy Configurator project data_husky.prj
//                                                                   //
//-------------------------------------------------------------------//


#define CUST_BYTE_NUM_OUT	0
#define CUST_BYTE_NUM_IN	62
#define TOT_BYTE_NUM_ROUND_OUT	0
#define TOT_BYTE_NUM_ROUND_IN	64


typedef union												//---- output buffer ----
{
	uint8_t  Byte [TOT_BYTE_NUM_ROUND_OUT];
	struct
	{
	}Cust;
} PROCBUFFER_OUT;


typedef union												//---- input buffer ----
{
	uint8_t  Byte [TOT_BYTE_NUM_ROUND_IN];
	struct
	{
		int32_t     bApp_yaw;
		int32_t     bApp_pitch;
		int32_t     bApp_roll;
		int16_t     q1;
		int16_t     q2;
		int16_t     q3;
		int16_t     q4;
		int16_t     x1;
		int16_t     y1;
		int16_t     z1;
		int16_t     x2;
		int16_t     y2;
		int16_t     z2;
		int16_t     x3;
		int16_t     y3;
		int16_t     z3;
		int16_t     x4;
		int16_t     y4;
		int16_t     z4;
		int16_t     x;
		int16_t     y;
		int16_t     z;
		int16_t     bApp_ax;
		int16_t     bApp_ay;
		int16_t     bApp_az;
		int16_t     bApp_gx;
		int16_t     bApp_gy;
		int16_t     bApp_gz;
	}Cust;
} PROCBUFFER_IN;

#endif