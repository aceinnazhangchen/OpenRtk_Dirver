#pragma once
#include <stdio.h>
#include <stdint.h>
#include <string>
#include <map>
#include "kml.h"

namespace NPOS122_Tool {
#define NPOS122_BUFFER_LEN 2048
#pragma pack(push, 1)
	typedef struct {
		uint8_t flag;				//header同步字是否满足包的条件 0:未满足, 1:满足
		uint8_t header_len;
		uint8_t header[4];
		uint32_t nbyte;
		uint32_t msg_len;
		uint8_t buff[NPOS122_BUFFER_LEN];
	} raw_t;

	typedef struct {
		uint8_t flag;
		uint32_t nbyte;
		uint8_t nmea[256];
	} raw_nmea_t;

	typedef struct {
		uint16_t msg_id;
		uint16_t msg_len;
		uint16_t time_status;
		uint16_t gps_week;
		uint32_t gps_millisecs;	//毫秒
		uint32_t reserved;
	} msg_header_t;

	typedef struct {
		double		gps_second;
		double		accel_x;
		double		accel_y;
		double		accel_z;
		double		gyro_x;
		double		gyro_y;
		double		gyro_z;
		uint32_t	status;
	}imu_t;

	typedef struct {
		uint32_t b_0 : 1;
		uint32_t b_1 : 1;
		uint32_t b_2 : 1;
		uint32_t b_3 : 1;
		uint32_t b_4 : 1;
		uint32_t b_5 : 1;
		uint32_t b_6 : 1;
		uint32_t b_7 : 1;
		uint32_t b_8 : 1;
		uint32_t b_9 : 1;
		uint32_t b_10 : 1;
		uint32_t b_11 : 1;
		uint32_t b_12 : 1;
		uint32_t b_13 : 1;
		uint32_t b_14 : 1;
		uint32_t b_15 : 1;
		uint32_t b_16 : 1;
		uint32_t b_17 : 1;
		uint32_t b_18 : 2;
		uint32_t b_20 : 1;
		uint32_t b_21 : 1;
		uint32_t b_22 : 2;
		uint32_t b_24 : 1;
		uint32_t b_25 : 1;
		uint32_t b_26 : 3;
		uint32_t b_29 : 3;
	}sol_status_bit;

	typedef struct {
		uint32_t	ins_status;
		uint32_t	pos_type;
		double		latitude;
		double		longitude;
		double		height;
		float		undulation;
		double		vel_x;
		double		vel_y;
		double		vel_z;
		double		roll;
		double		pitch;
		double		azimuth;
		float		latitude_std;
		float		longitude_std;
		float		height_std;
		float		vel_x_std;
		float		vel_y_std;
		float		vel_z_std;
		float		roll_std;
		float		pitch_std;
		float		azimuth_std;
		sol_status_bit	sol_status;
		uint16_t	last_update_time;
	}pva_t;
#pragma pack(pop)
	enum emMSG_ID {
		MSG_PVA = 2379,
		MSG_IMU = 2389,
	};
	typedef std::map<std::string, FILE*> FilesMap;
	class NPOS122_decoder
	{
	public:
		NPOS122_decoder();
		~NPOS122_decoder();
	protected:
		void close_all_files();
		void create_file(FILE * &file, const char * suffix, const char * title);
		FILE * get_file(std::string file_suffix, const char * title);
		void append_ins_kml();
		void output_pva();
		void output_ins_txt();
		void output_imu();
		void parse_msg_header();
		void parse_packet_payload();
	public:
		void init();
		void set_base_file_name(char* file_name);
		int8_t parse_nmea(uint8_t data);
		int8_t input_data(uint8_t data);
		void finish();
		static int get_normal_pos_type(int pos_type);
	private:
		int pack_num;
		int crc_right_num;
		int crc_error_num;
		char base_file_name[256];
		raw_t raw;
		raw_nmea_t raw_nmea;
		msg_header_t msg_header;
		FilesMap output_file_map;			//把文件指针都保存到map中
		imu_t imu;
		pva_t pva;
		kml_ins_t ins_kml;
	};

}