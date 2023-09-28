#pragma once
#include <stdint.h>
#include <map>
#include <string>
#include "kml.h"


namespace RTK350LA_Tool {
#ifdef R2D
#define R2D         (57.295779513082320)
#endif
#define MAX_RR_PER_PACKET       64
#define MAX_OUTPUT_MSG_LEN		1024
	enum emPackageType {
		em_S2 = 0x3253,
        em_SI = 0x4953,
		em_gN = 0x4E67,
		em_iN = 0x4E69,
		em_iS = 0x5369,        
		em_o1 = 0x316F,
		em_pv = 0x7670,
        em_rR = 0x5272,
        em_iB = 0x4269,
        em_sT = 0x5473,
	};
#pragma pack(push, 1)
	typedef struct {
		uint8_t nmea_flag;
		uint8_t flag;
		uint8_t header_len;
		uint8_t header[4];
		uint32_t nbyte;
		uint32_t length;
		uint8_t buff[256];
		uint32_t nmeabyte;
		uint8_t nmea[256];
		uint8_t ntype;
		uint16_t packet_type;
	} usrRaw;

	typedef struct {
        uint16_t    week;
        uint32_t    timeOfWeek;
        float       accel_g[3];
        float       rate_dps[3];
        float       temp;
        uint32_t    status;
	}out_S2_struct, out_SI_struct;

    typedef struct {
        uint16_t week;
        uint32_t timeOfWeek;
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
    } out_iN_struct;

    typedef struct {
        uint16_t week;
        uint32_t timeOfWeek;
        uint16_t latitude_std;
        uint16_t longitude_std;
        uint16_t height_std;
        uint16_t north_velocity_std;
        uint16_t east_velocity_std;
        uint16_t up_velocity_std;
        uint16_t lat_velocity_std;
        uint16_t long_velocity_std;
        uint16_t roll_std;
        uint16_t pitch_std;
        uint16_t heading_std;
    } out_iS_struct;

    typedef struct {
        uint16_t    week;
        uint32_t    timeOfWeek;
        uint8_t     positionMode;
        int32_t     latitude;
        int32_t     longitude;
        float       height;
        uint8_t     numberOfSVs;
        uint16_t    hdop;
        uint16_t    vdop;
        uint16_t    tdop;
        uint16_t    diffage;
        int16_t     north_vel;
        int16_t     east_vel;
        int16_t     up_vel;
        uint16_t    latitude_std;
        uint16_t    longitude_std;
        uint16_t    height_std;
        uint16_t    north_vel_std;
        uint16_t    east_vel_std;
        uint16_t    up_vel_std;
    #ifdef GNSS_ALGORITHM_TEST
        uint16_t    alo_time;
    #endif

    } output_gN_t;

    typedef struct {
        uint16_t    gps_week;               // GPS Week number
        uint32_t    gps_millisecs;          // Milliseconds into week
        uint8_t     mode;
        double      speed;
        uint8_t     fwd;
        uint64_t    wheel_tick;
    } output_o1_t;

    typedef struct ucb_pvt_t_
    {
        uint16_t week;
        uint32_t timeOfWeek;
        uint8_t nspp_use;
        double  pvt_time;
        double  pvt_pos[6];
        double  pvt_std[6];
    }ucb_pvt_t;

    typedef struct {
        uint8_t len;
        uint8_t data[MAX_RR_PER_PACKET];
    } ucb_rr_t;

    typedef struct ucb_bias_t_
    {
        uint16_t    week;
        uint32_t    timeOfWeek;
        float       gyro_bias_x;
        float       gyro_bias_y;
        float       gyro_bias_z;
        float       acc_bias_x;
        float       acc_bias_y;
        float       acc_bias_z;
    }ucb_bias_t;


	typedef struct {
		/* IMU */
		uint32_t imu_temp_status : 1; // imu temperature status
		uint32_t imu_acce_status : 1;  // imu accelerometer status
		uint32_t imu_gyro_status : 1; // imu gyro status
		uint32_t imu_sensor_status1 : 1; // imu sensor (#1, #2 #3) status
		uint32_t imu_sensor_status2 : 1; // imu sensor (#1, #2 #3) status
		uint32_t imu_sensor_status3 : 1; // imu sensor (#1, #2 #3) status
		uint32_t imu_overall_status : 1;

		/* GNSS status */
		uint32_t gnss_data_status : 1;
		uint32_t gnss_signal_status : 1;

		/* operation */
		uint32_t power : 1; // for the whole device, any component has no power then 0
		uint32_t MCU_status : 1;
		uint32_t pps_status : 1;

		uint32_t zupt_det : 1; // 1  zupt detected
		uint32_t odo_used : 1; // 1  odometer update used
		uint32_t odo_recv : 1; // 1  odometer data received

		/* IMU sensor fault flags:
		0  fault; 1 - normal*/
		uint32_t imu_s1_state : 1;
		uint32_t imu_s2_state : 1;
		uint32_t imu_s3_state : 1;
		/* Time valid flag: 0  fault; 1 - normal */
		uint32_t time_valid : 1;
		/* Antenna sensing status:
		 0  UNINIT
		 1  NORMAL
		 2 - OPEN
		 3 - SHORT
		 4  THERMAL SHUTDOWN
		 5 - REVERSE CURRENT
		 6 - OVERCURRENT
		 7 - RESERVED */
		uint32_t antenna_sensing : 3;
		/* GNSS chipset fault flag: 0  fault; 1 - normal */
		uint32_t gnss_chipset : 1;
		/* MCU and peripheal power up self-test: 1 - valid, 0 - invalid */
		//uint32_t pust_check : 1;
		uint32_t post : 1;
		uint32_t gnss_receiver_bootmode : 1;
		uint32_t rexerved : 3;
		uint32_t adc_power : 4;
	} status_bit_t;

	typedef struct output_sT_t_
	{
		uint16_t GPS_Week;
		uint32_t GPS_TimeOfWeek;
		uint16_t year;
		uint8_t	 mouth;
		uint8_t	 day;
		uint8_t	 hour;
		uint8_t  min;
		uint8_t  sec;
		status_bit_t status_bit;
		float  imu_temperature;
		float  mcu_temperature;
	} output_sT_t;

#pragma pack(pop)
	typedef std::map<std::string, FILE*> FilesMap;
	class RTK350LA_decoder
	{
	public:
		RTK350LA_decoder();
		~RTK350LA_decoder();
	private:
		bool is_pruned;
		int data_version;
		char base_file_name[256];
		char output_msg[MAX_OUTPUT_MSG_LEN];
		usrRaw raw;
		out_S2_struct pak_S2;
        out_SI_struct pak_SI;
		output_gN_t pak_gN;
		out_iN_struct pak_iN;
		output_o1_t pak_o1;
		ucb_rr_t pak_rr;        
        ucb_pvt_t pak_pvt;
        ucb_bias_t pak_bias;
        output_sT_t pak_sT;
        out_iS_struct pak_iS;
		kml_gnss_t gnss_kml;
		kml_ins_t ins_kml;
		bool show_format_time;
		int pack_num;
		int crc_right_num;
		int crc_error_num;
		std::map<uint16_t, int> all_type_pack_num;
		std::map<uint16_t, int> all_type_file_output;
		FilesMap output_file_map;	
		bool m_isOutputFile;
	private:
		void close_all_files();
		void create_file(FILE * &file, const char * suffix, const char * title, bool format_time);
		FILE* get_file(std::string suffix, std::string title, bool format_time);
		void parse_packet_payload();
		void save_novatel_raw_imu();
		int parse_nmea(uint8_t data);
		void append_early_gnss_kml();
		void append_gnss_kml();
		void append_ins_kml();
		void output_S2();
        void output_SI();
		void output_gN();
		void output_iN();
		void output_o1();
        void output_rr();
        void output_pvt();
        void output_bias();
        void output_sT();
        void output_iS();
	public:
		void init();
		void set_output_file(bool output);
		void set_base_file_name(char * file_name);
		void set_pruned(bool pruned);
		int input_raw(uint8_t data);
		void finish();
	public:
		int get_current_type();
	};
}
