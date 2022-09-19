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

	typedef enum {
		header_old = 0,
		header_new
	} unicoHeaderType;

	typedef struct {
		uint8_t flag;
        unicoHeaderType header_type;
        uint8_t header_to_read;
		uint8_t header_len;
        uint32_t data_len;
		uint8_t header[100];
		uint32_t nbyte;
		uint8_t buff[100*1024];
		uint16_t ntype;
	} unicoRaw;



    typedef struct UnicoHeader_old_t_
    {
        uint8_t sync1;            //!< start of packet first byte (0xAA)
        uint8_t sync2;            //!< start of packet second byte (0x44)
        uint8_t sync3;            //!< start of packet third  byte (0x12)
        uint8_t header_length;
        uint16_t message_id;
        uint8_t message_type;
        uint8_t reserved1;
        uint16_t message_length;
        uint16_t reserved2;
        uint8_t idle_time;
        uint8_t time_status; 
        uint16_t week;
        uint32_t time_of_week;
        uint32_t reserved3;
        uint16_t time_offset_gps;
        uint16_t reserved4;
    } UnicoHeader_old_t;

    typedef struct UnicoHeader_t_
    {
        uint8_t sync1;            //!< start of packet first byte (0xAA)
        uint8_t sync2;            //!< start of packet second byte (0x44)
        uint8_t sync3;            //!< start of packet third  byte (0x12)
        uint8_t cpu_idle;
        uint16_t message_id;
        uint16_t message_length;
        uint8_t time_ref;
        uint8_t time_status;
        uint16_t week;
        uint32_t time_of_week;
        uint32_t reserved3;
        uint8_t version;
        uint8_t leap_sec;
        uint16_t delay_ms;
    } UnicoHeader_t;


    typedef struct RangehData_t_
    {
        uint16_t prn;                   //卫星PRN号（GPS ：1到32 ，GLONASS: 38到61，BDS1到63，SBAS120到141及183到187，QZSS193到197）
        uint16_t glofreq;               //（GLONASS 频率+ 7），GPS，BDS和Galileo不使用
        double psr;                     //码伪距测量值（米）
        float psr_std;                  //码伪距标准差（米）
        double adr;                     //载波相位，周（积分多普勒）
        float adr_std;                  //载波相位标准差（周）
        float dopp;                     //瞬时多普勒（Hz）
        float c_no;                     //载噪比 C/No = 10[log10(S/N0)]（dBHz）
        float locktime;                 //秒，连续跟踪时间（无周跳）
        uint32_t ch_tr_status;          //通道跟踪状态
    }RangehData_t;

    typedef struct RANGEHB_t_
    {
        UnicoHeader_t header;
        uint32_t satellite_num;     //卫星数量
        RangehData_t **data;
        uint8_t crc[4];
    }RANGEHB_t;

    typedef struct BD2SEPHEM_t_                  //北斗星历数据
    {
        UnicoHeader_t header;           //头信息
        uint32_t prn;                   //卫星 PRN 编号（ BDS： 1 到63）
        double tow;                     //子帧1的时间标识（基于GPS时间）， s
        uint32_t health;                //健康状态–在北斗 ICD 中定义的一个 1 比特的健康代码
        uint32_t aode_1;                //星历数据龄期
        uint32_t aode_2;                //星历数据龄期
        uint32_t week;                  //GPS 周计数（GPS Week）
        uint32_t z_week;                //基于 GPS 周的 Z 计数周数，为星历子帧 1 的周数。
        double toe;                     //星历参考时刻（基于 GPS 时间）， s
        double a;                       //轨道长半轴， m
        double delat_n;                 //卫星平均角速度的改正值 ，rad/s
        double m_0;                     //参考时间的平近点角， rad
        double ecc;                     //偏心率
        double omega;                   //近地点幅角， rad
        double cuc;                     //纬度幅角（余弦振幅， rad）
        double cus;                     //纬度幅角（正弦振幅， rad）
        double crc;                     //轨道半径（余弦振幅， m）
        double crs;                     //轨道半径（正弦振幅， m）
        double cic;                     //倾角（余弦振幅， rad）
        double cis;                     //倾角（正弦振幅， rad）
        double i_0;                     //TOE 时间轨道倾角， rad
        double idot;                    //轨道倾角变化率， rad/s
        double omega_0;                 //升交点赤经， rad
        double omega_dot;               //升交点赤经变化率， rad/s
        uint32_t iodc;                  //时钟数据龄期
        double toc;                     //卫星钟差参考时间， s
        double tgd_1;                   //B1 群延迟（B1 星上设备时延差）， s
        double tgd_2;                   //B2 群延迟（B2 星上设备时延差）， s
        double af_0;                    //卫星钟差参数， s
        double af_1;                    //卫星钟速参数， s/s
        double af_2;                    //卫星钟漂参数， s/s/s
        uint32_t as;                    //反欺骗：0 = FALSE;1 = TRUE
        double n;                       //改正平均角速度， rad/s
        double ura;                     //用户距离精度， m2
        uint8_t crc32[4];
    }BD2SEPHEM_t;


    typedef struct obsVM_data_t_
    {
        uint16_t system_freq;
        uint16_t prn;
        double psr;
        double adr;
        uint16_t psr_std;
        uint16_t adr_std;
        float dopp;
        uint16_t c_no;
        uint16_t reserved;
        float lock_time;
        uint32_t ch_tr_status;
    }obsVM_data_t;

    typedef struct OBSVM_t_
    {
        UnicoHeader_t header;
        uint32_t obs_num;
        // uint16_t system_freq;
        obsVM_data_t **data;
        uint8_t crc[4];
    }OBSVM_t;


#pragma pack(pop)

	extern void init_beidou_data();
	extern void set_output_beidou_file(int output);
	extern void set_base_beidou_file_name(char* file_name);
	extern void write_beidou_kml_files();
	extern void close_beidou_all_log_file();

	extern int input_beidou_raw(uint8_t data);
    extern int input_unico_raw(uint8_t data);

}