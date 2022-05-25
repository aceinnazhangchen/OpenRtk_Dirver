#pragma once
#include <stdint.h>
#include <map>
#include <string>
#include "openrtk_inceptio.h"
#include "kml.h"


namespace RTK330LA_Tool {
#define MAX_OUTPUT_MSG_LEN		1024
	enum emPackageType {
		em_s1 = 0x3173,
		em_s2 = 0x3273,
		em_gN = 0x4E67,
		em_iN = 0x4E69,
		em_d1 = 0x3164,
		em_d2 = 0x3264,
		em_sT = 0x5473,
		em_o1 = 0x316F,
		em_fM = 0x4D66,
		em_rt = 0x7472,
		em_sP = 0x5073,
		em_sV = 0x5673,
		em_r1 = 0x3172,
		em_w1 = 0x3177,
		em_gI = 0x4967,
		em_iI = 0x4969,
		em_g1 = 0x3167,
		em_RM = 0xde01, //RUNSTATUS_MONITOR
	};
	typedef std::map<std::string, FILE*> FilesMap;
	class Rtk330la_decoder
	{
	public:
		Rtk330la_decoder();
		~Rtk330la_decoder();
	private:
		int data_version;
		char base_file_name[256];
		char output_msg[MAX_OUTPUT_MSG_LEN];
		usrRaw raw;
		inceptio_s1_t pak_s1;
		inceptio_s1_t pak_s2;
		inceptio_gN_early_t pak_gN_early;
		inceptio_gN_t pak_gN;
		inceptio_iN_t pak_iN;
		inceptio_d1_t pak_d1;
		inceptio_d2_t pak_d2;
		inceptio_sT_t pak_sT;
		inceptio_o1_t pak_o1;
		rtk_debug1_t rtk_debug1;
		gnss_integ_t gnss_integ;
		ins_integ_t ins_integ;
		runstatus_monitor_t monitor;
		kml_gnss_t gnss_kml;
		kml_ins_t ins_kml;
		bool show_format_time;
		int pack_num;
		int crc_right_num;
		int crc_error_num;
		std::map<uint16_t, int> all_type_pack_num;
		std::map<uint16_t, int> all_type_file_output;
		FilesMap output_file_map;			//现在输出文件不断增加，把文件指针都保存到map中
		bool m_isOutputFile;
	private:
		void close_all_files();
		void create_file(FILE * &file, const char * suffix, const char * title, bool format_time);
		FILE* get_file(std::string suffix, std::string title, bool format_time);
		void parse_packet_payload();
		int parse_nmea(uint8_t data);
		void append_early_gnss_kml();
		void append_gnss_kml();
		void append_ins_kml();
		void output_s1();
		void output_s2();
		void output_gN_early();
		void output_gN();
		void output_iN();
		void output_d1();
		void output_d2();
		void output_sT();
		void output_debug1();
		void output_o1();
		void output_gnss_integ();
		void output_ins_integ();
		void output_ins_and_integ();
		void output_g1();
		void output_runstatus_monitor();
	public:
		void init();
		void set_base_file_name(char * file_name);
		int input_raw(uint8_t data);
		void finish();
	public:
		int get_current_type();
		inceptio_s1_t * get_imu_raw();
		inceptio_gN_t * get_gnss_sol();
		inceptio_iN_t * get_ins_sol();
	};
}
