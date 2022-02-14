#pragma once
#include <stdint.h>

#define MAX_PACKET_TYPES 5

namespace OpenRTK330LI_Tool {
#pragma pack(push, 1)
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
		uint16_t GPS_Week;
		uint32_t GPS_TimeOfWeek;
		float x_accel;
		float y_accel;
		float z_accel;
		float x_gyro;
		float y_gyro;
		float z_gyro;
	} user_s1_t;

	typedef struct
	{
		uint16_t GPS_Week;
		uint32_t GPS_TimeOfWeek;
		uint8_t  position_type;
		double   latitude;
		double   longitude;
		double   height;
		float    latitude_standard_deviation;
		float    longitude_standard_deviation;
		float    height_standard_deviation;
		uint8_t  number_of_satellites;
		uint8_t  number_of_satellites_in_solution;
		float    hdop;
		float    diffage;
		float    north_vel;
		float    east_vel;
		float    up_vel;
		float    north_vel_standard_deviation;
		float    east_vel_standard_deviation;
		float    up_vel_standard_deviation;
	} user_g1_t;

	typedef struct
	{
		uint16_t GPS_Week;
		uint32_t GPS_TimeOfWeek;
		uint8_t  ins_status;
		uint8_t  ins_position_type;
		double   latitude;
		double   longitude;
		double   height;
		double   north_velocity;
		double   east_velocity;
		double   up_velocity;
		double   roll;
		double   pitch;
		double   heading;
		float    latitude_std;
		float    longitude_std;
		float    height_std;
		float    north_velocity_std;
		float    east_velocity_std;
		float    up_velocity_std;
		float    roll_std;
		float    pitch_std;
		float    heading_std;
	} user_i1_t;

	typedef struct
	{
		uint16_t GPS_Week;
		uint32_t GPS_TimeOfWeek;
		uint8_t  mode;
		double   speed;
		uint8_t  fwd;
		uint64_t wheel_tick;
	} user_o1_t;

	typedef struct
	{
		uint16_t GPS_Week;
		uint32_t GPS_TimeOfWeek;
		uint8_t  satelliteId;
		uint8_t  systemId;
		uint8_t  antennaId;
		uint8_t  l1cn0;
		uint8_t  l2cn0;
		float    azimuth;
		float    elevation;
	}user_y1_t;

	typedef enum {
		USR_OUT_NONE = 0,
		USR_OUT_RAWIMU,
		USR_OUT_BESTGNSS,
		USR_OUT_INSPVAX,
		USR_OUT_ODO,
		USR_OUT_SATELLITES,
		USR_OUT_MAX
	} UserOutPacketType;

#pragma pack(pop)

	extern void init_user_data();
	extern void set_save_bin(int save);
	extern void set_output_user_file(int output);
	extern void set_base_user_file_name(char* file_name);
	extern void write_kml_files();
	extern void close_user_all_log_file();

	extern int input_user_raw(uint8_t data);
	extern uint8_t get_current_type();
	user_i1_t * get_ins_sol();
}