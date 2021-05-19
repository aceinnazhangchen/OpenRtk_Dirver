#pragma once
#include <stdint.h>
#ifdef __cplusplus
extern "C"
{
#endif

#pragma pack(push, 1)

	typedef struct {
		uint8_t nmea_flag;
		uint8_t flag;
		uint8_t header_len;
		uint8_t header[4];
		uint32_t nbyte;
		uint8_t buff[256];
		uint32_t nmeabyte;
		uint8_t nmea[128];
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
		USR_OUT_INCEPTIO_SCALED1,
		USR_OUT_INCEPTIO_INSPVA,
		USR_OUT_INCEPTIO_STD1,
		USR_OUT_INCEPTIO_GNSS,
		USR_OUT_INCEPTIO_STATUS,
		USR_OUT_MAX
	} UserOutPacketType;

	typedef struct
	{
		uint32_t GPS_Week;
		double	 GPS_TimeOfWeek;
		float	 x_accel;
		float	 y_accel;
		float	 z_accel;
		float	 x_gyro;
		float	 y_gyro;
		float	 z_gyro;
	} inceptio_s1_t;

	typedef struct
	{
		uint32_t GPS_Week;
		double	 GPS_TimeOfWeek;
		uint8_t	 positionMode;
		uint32_t latitude;
		uint32_t longitude;
		float	 height;
		uint8_t  numberOfSVs;
		float    hdop;
		uint16_t diffage;
		int16_t	 velocityNorth;
		int16_t  velocityEast;
		int16_t  velocityUp;
	} inceptio_gN_t;

	typedef struct
	{
		uint32_t GPS_Week;
		double	 GPS_TimeOfWeek;
		uint8_t  insStatus;
		uint8_t  insPositionType;
		int32_t	 latitude;
		int32_t	 longitude;
		float	 height;
		int16_t	 velocityNorth;
		int16_t  velocityEast;
		int16_t  velocityUp;
		int16_t  roll;
		int16_t  pitch;
		int16_t  heading;
	} inceptio_iN_t;

	typedef struct
	{
		uint32_t GPS_Week;
		double	 GPS_TimeOfWeek;
		int16_t	 latitude_std;
		int16_t	 longitude_std;
		int16_t	 height_std;
		int16_t	 north_vel_std;
		int16_t  east_vel_std;
		int16_t  up_vel_std;
		int16_t  roll_std;
		int16_t  pitch_std;
		int16_t  heading_std;
	} inceptio_d1_t;

	typedef struct
	{
		uint32_t GPS_Week;
		double	 GPS_TimeOfWeek;
		uint16_t year;
		uint8_t	 mouth;
		uint8_t	 day;
		uint8_t	 hour;
		uint8_t  min;
		uint8_t  sec;
		uint32_t imu_status;
		float  imu_temperature;
		float  mcu_temperature;
	} inceptio_sT_t;

#pragma pack(pop)

	extern uint16_t calc_crc(uint8_t* buff, uint32_t nbyte);

	extern void set_save_bin(int save);
	extern void set_output_file(int output);
	extern void set_base_user_file_name(char* file_name);
	extern void close_user_all_log_file();
	extern int get_packet_type();
	extern user_s1_t* get_packet_s1();
	extern user_g1_t* get_packet_g1();

	extern int input_user_raw(uint8_t data);

#ifdef __cplusplus
}
#endif