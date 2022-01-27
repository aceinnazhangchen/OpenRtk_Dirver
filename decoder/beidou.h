#pragma once
#include <stdint.h>

#define MAX_beidou_PACKET_TYPES 5

#pragma pack(push, 1)
namespace beidou_Tool {

	typedef struct {
		uint8_t nmea_flag;
		uint8_t flag;
		uint8_t header_len;
		uint8_t header[4];
		uint32_t nbyte;
		uint8_t buff[256];
		uint32_t nmeabyte;
		uint8_t nmea[256];
		uint8_t ntype;
	} usrRaw;

	typedef struct
	{
		uint32_t  week;
		double    timeOfWeek;
		float     accel_g[3];
		float     rate_dps[3];
	} beidou_s1_t;

	struct beidou_gN_t
	{
		uint32_t week;
		double   timeOfWeek;
		uint8_t  positionMode;
		int32_t  latitude;
		int32_t  longitude;
		float    height;
		uint8_t  numberOfSVs;
		float    hdop;
		uint16_t diffage;
		int16_t  north_vel;
		int16_t  east_vel;
		int16_t  up_vel;
		int16_t  latitude_std;
		int16_t  longitude_std;
		int16_t  height_std;
	};

	typedef struct
	{
		uint32_t week;
		double   timeOfWeek;
		uint8_t  insStatus;
		uint8_t  insPositionType;
		int32_t  latitude;
		int32_t  longitude;
		float    height;
		int16_t  north_vel;
		int16_t  east_vel;
		int16_t  up_vel;
		int16_t  roll;
		int16_t  pitch;
		int16_t  heading;
	} beidou_iN_t;

	typedef struct
	{
		uint16_t    gps_week;               // GPS Week number
		uint32_t    gps_millisecs;          // Milliseconds into week
		uint8_t     mode;
		double      speed;
		uint8_t     fwd;
		uint64_t    wheel_tick;
	} beidou_o1_t;


	typedef struct  heading
	{
		uint16_t    gps_week;          // GPS Week number
		uint32_t    gps_millisecs;     // Milliseconds into week
		float       length;
		float       heading;
		float       pitch;
		float       hdgstddev;
		float       ptchstddev;
	}beidou_hG_t;

	typedef enum {
		beidou_OUT_NONE = 0,
		beidou_OUT_SCALED1,
		beidou_OUT_INSPVA,
		beidou_OUT_GNSS,
		beidou_OUT_ODO,
		beidou_OUT_HEADING
	} beidouOutPacketType;

#pragma pack(pop)

	extern void init_beidou_data();
	extern void set_output_beidou_file(int output);
	extern void set_base_beidou_file_name(char* file_name);
	extern void write_beidou_kml_files();
	extern void close_beidou_all_log_file();

	extern int input_beidou_raw(uint8_t data);

}