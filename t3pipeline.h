#ifndef CUSTOM_PDO_NAME_H
#define CUSTOM_PDO_NAME_H

//-------------------------------------------------------------------//
//                                                                   //
//     This file has been created by the Easy Configurator tool      //
//                                                                   //
//     Easy Configurator project t3pipeline.prj
//                                                                   //
//-------------------------------------------------------------------//


#define CUST_BYTE_NUM_OUT	0
#define CUST_BYTE_NUM_IN	68
#define TOT_BYTE_NUM_ROUND_OUT	0
#define TOT_BYTE_NUM_ROUND_IN	68


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
		float       posx;
		float       posy;
		float       posz;
		float       q1;
		float       q2;
		float       q3;
		float       q4;
		float       rs1;
		float       rs2;
		float       rs3;
		float       rs4;
		int32_t     bApp_yaw;
		int32_t     bApp_pitch;
		int32_t     bApp_roll;
		int16_t     bApp_ax;
		int16_t     bApp_ay;
		int16_t     bApp_az;
		int16_t     bApp_gx;
		int16_t     bApp_gy;
		int16_t     bApp_gz;
	}Cust;
} PROCBUFFER_IN;

#endif