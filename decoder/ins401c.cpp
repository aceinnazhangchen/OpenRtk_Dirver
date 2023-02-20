#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <memory.h>
#include <string.h>
#include <math.h>
#include "common.h"
#include "rtklib_core.h" //R2D
#include "kml.h"
#include "ins401c.h"


namespace ins401c_Tool {
	static FILE* fs_canfd = NULL;
    static FILE* fs_imu = NULL;
    static FILE* fs_ins = NULL;
	static FILE* f_log = NULL;
    static FILE* fs_gnss = NULL;
    static FILE* fs_dia = NULL;
    static FILE* fs_odo = NULL;
    static uint16_t can_mess_flag = 0x0;
	static char ins401c_output_msg_inspva[1024] = { 0 };
    static char ins401c_output_msg_imu[1024] = { 0 };
    static char ins401c_output_msg_ins[1024] = { 0 };
    static char ins401c_output_msg_gnss[1024] = { 0 };
    static char ins401c_output_msg_diagnostic[1024] = { 0 };
    static char ins401c_output_msg_odo[1024] = { 0 };
    static can_imu_t imu_mess = {0};
    static can_ins_t ins_mess = {0};
    static can_gnss_t gnss_mess = {0};
    FILE* f_ins_txt = NULL;
    kml_ins_t ins_kml;
    kml_gnss_t gnss_kml;

    static const dbc_msg_hdr_t list_canfd_dbc_msgs[] = {
        {0x180, 53,           dbc_decode_INSPVAX},
        {0x181, 53,           dbc_decode_Gnss_Message},
        {0x282, 53,           dbc_decode_Diagnostic_Message},
        {0x600, 53,           dbc_decode_odo_Message},
    };
    static const dbc_msg_hdr_t list_can_dbc_msgs[] = {
        { 0x180, 6,           dbc_decode_INS_ACC},
        { 0x181, 6,          dbc_decode_INS_GYRO},
        { 0x182, 6, dbc_decode_INS_HeadingPitchRoll},
        { 0x183, 4, dbc_decode_INS_HeightAndIMUStatus},
        { 0x184, 4, dbc_decode_INS_LatitudeLongitude},
        { 0x185, 6,         dbc_decode_INS_Speed},
        { 0x186, 6,      dbc_decode_INS_DataInfo},
        { 0x187, 8,           dbc_decode_INS_Std},
        { 0x188, 4,          dbc_decode_INS_Time},
    };
	static char base_ins401c_file_name[256] = { 0 };

	extern void init_ins401c_data() {
		memset(&imu_mess, 0, sizeof(imu_mess));
		memset(&ins_mess, 0, sizeof(ins_mess));
        memset(&gnss_mess, 0, sizeof(gnss_mess));
		Kml_Generator::Instance()->init();
	}

	void create_file(FILE* &file, const char* suffix, const char* title, bool format_time = false) {
		if (strlen(base_ins401c_file_name) == 0) return;
		if (file == NULL) {
			char file_name[256] = { 0 };
			sprintf(file_name, "%s_%s", base_ins401c_file_name, suffix);
			file = fopen(file_name, "wb");
			if (file && title) {
				if(format_time)
					fprintf(file, "DateTime(),");
				fprintf(file, title);
			}
		}
	}

	void append_ins_kml()
	{
		ins_kml.gps_week = ins_mess.week;
		ins_kml.gps_secs = (double)ins_mess.time_of_week / 1000 ;
        /* Tag20230118 modify by wrj, change ins pos type to ins status, to show ins status on kml */
		// ins_kml.ins_status = ins_mess.ins_position_status;
        ins_kml.ins_status = ins_mess.ins_status;
		ins_kml.ins_position_type = ins_mess.ins_position_status;
		ins_kml.latitude = ins_mess.latitude;
		ins_kml.longitude = ins_mess.longitude;
		ins_kml.height = ins_mess.height;
		ins_kml.north_velocity = ins_mess.north_vel;
		ins_kml.east_velocity = ins_mess.east_vel;
		ins_kml.up_velocity = ins_mess.up_vel;
		ins_kml.roll = ins_mess.roll;
		ins_kml.pitch = ins_mess.pitch;
		ins_kml.heading = ins_mess.heading;
		Kml_Generator::Instance()->append_ins(ins_kml);
	}

	void append_gnss_kml()
	{
		gnss_kml.gps_week = gnss_mess.gps_week;
		gnss_kml.gps_secs = (double)gnss_mess.gps_millisecs / 1000.0;
		gnss_kml.position_type = gnss_mess.position_type;
		gnss_kml.latitude = gnss_mess.latitude;
		gnss_kml.longitude = gnss_mess.longitude;
		gnss_kml.height = gnss_mess.height;
		gnss_kml.north_vel = gnss_mess.north_vel;
		gnss_kml.east_vel = gnss_mess.east_vel;
		gnss_kml.up_vel = gnss_mess.up_vel;
		Kml_Generator::Instance()->append_gnss(gnss_kml);
	}


	void set_base_ins401c_file_name(char* file_name)
	{
        init_ins401c_data();
		strcpy(base_ins401c_file_name, file_name);
		if (strlen(base_ins401c_file_name) == 0) return;
		char log_file_name[256] = { 0 };
		if (f_log == NULL) {
			sprintf(log_file_name, "%s.log", base_ins401c_file_name);
			f_log = fopen(log_file_name, "w");
		}
	}
    /* Decode Vector__XXX* INSPVAX message
    * @param hdr  The header of the message to validate its DLC and MID; this can be NULL to skip this check */
    bool dbc_decode_INSPVAX(uint8_t *pstu, const uint8_t *bytes)
    {
        const bool success = true;
        // If msg header is provided, check if the DLC and the MID match
        if (NULL == pstu || NULL == bytes) {
            return !success;
        }
        uint32_t raw;
        INSPVAX_t *to = (INSPVAX_t *)pstu;

        raw  = ((uint32_t)((bytes[0]))) << 8;    //< 8 bit(s) from B7
        raw |= ((uint32_t)((bytes[1])));    //< 8 bit(s) from B15
        to->ACC_X = ((raw * 0.0001220703125) + (-4)) * 9.7803267714;

        raw  = ((uint32_t)((bytes[2]))) << 8;    //< 8 bit(s) from B23
        raw |= ((uint32_t)((bytes[3])));    //< 8 bit(s) from B31
        to->ACC_Y = ((raw * 0.0001220703125) + (-4)) * 9.7803267714;

        raw  = ((uint32_t)((bytes[4]))) << 8;    //< 8 bit(s) from B39
        raw |= ((uint32_t)((bytes[5])));    //< 8 bit(s) from B47
        to->ACC_Z = ((raw * 0.0001220703125) + (-4)) * 9.7803267714;

        raw  = ((uint32_t)((bytes[6]))) << 8;    //< 8 bit(s) from B55
        raw |= ((uint32_t)((bytes[7])));    //< 8 bit(s) from B63
        to->GYRO_X = ((raw * 0.0076293) + (-250));

        raw  = ((uint32_t)((bytes[8]))) << 8;    //< 8 bit(s) from B71
        raw |= ((uint32_t)((bytes[9])));    //< 8 bit(s) from B79
        to->GYRO_Y = ((raw * 0.0076293) + (-250));

        raw  = ((uint32_t)((bytes[10]))) << 8;    //< 8 bit(s) from B87
        raw |= ((uint32_t)((bytes[11])));    //< 8 bit(s) from B95
        to->GYRO_Z = ((raw * 0.0076293) + (-250));

        raw  = ((uint32_t)((bytes[12]))) << 8;    //< 8 bit(s) from B103
        raw |= ((uint32_t)((bytes[13])));    //< 8 bit(s) from B111
        to->INS_PitchAngle = ((raw * 0.01098649576) + (-360));

        raw  = ((uint32_t)((bytes[14]))) << 8;    //< 8 bit(s) from B119
        raw |= ((uint32_t)((bytes[15])));    //< 8 bit(s) from B127
        to->INS_RollAngle = ((raw * 0.01098649576) + (-360));

        raw  = ((uint32_t)((bytes[16]))) << 8;    //< 8 bit(s) from B135
        raw |= ((uint32_t)((bytes[17])));    //< 8 bit(s) from B143
        to->INS_HeadingAngle = ((raw * 0.01098649576) + (-360));

        raw  = ((uint32_t)((bytes[18]))) << 24;    //< 8 bit(s) from B87
        raw |= ((uint32_t)((bytes[19]))) << 16;    //< 8 bit(s) from B95
        raw |= ((uint32_t)((bytes[20]))) << 8;    //< 8 bit(s) from B103
        raw |= ((uint32_t)((bytes[21])));    //< 8 bit(s) from B111
        to->INS_LocatHeight = ((raw * 0.001) + (-10000));

        raw  = ((uint32_t)((bytes[22]))) << 24;    //< 8 bit(s) from B135
        raw |= ((uint32_t)((bytes[23]))) << 16;    //< 8 bit(s) from B143
        raw |= ((uint32_t)((bytes[24]))) << 8;    //< 8 bit(s) from B151
        raw |= ((uint32_t)((bytes[25])));    //< 8 bit(s) from B159
        to->IMU_Status = ((raw));

        raw  = ((uint32_t)((bytes[26]))) << 24;    //< 8 bit(s) from B167
        raw |= ((uint32_t)((bytes[27]))) << 16;    //< 8 bit(s) from B175
        raw |= ((uint32_t)((bytes[28]))) << 8;    //< 8 bit(s) from B183
        raw |= ((uint32_t)((bytes[29])));    //< 8 bit(s) from B191
        to->INS_Latitude = ((raw * 1e-07) + (-180));

        raw  = ((uint32_t)((bytes[30]))) << 24;    //< 8 bit(s) from B199
        raw |= ((uint32_t)((bytes[31]))) << 16;    //< 8 bit(s) from B207
        raw |= ((uint32_t)((bytes[32]))) << 8;    //< 8 bit(s) from B215
        raw |= ((uint32_t)((bytes[33])));    //< 8 bit(s) from B223
        to->INS_Longitude = ((raw * 1e-07) + (-180));

        raw  = ((uint32_t)((bytes[34]))) << 8;    //< 8 bit(s) from B247
        raw |= ((uint32_t)((bytes[35])));    //< 8 bit(s) from B255
        to->INS_NorthSpd = ((raw * 0.0030518) + (-100));

        raw  = ((uint32_t)((bytes[36]))) << 8;    //< 8 bit(s) from B263
        raw |= ((uint32_t)((bytes[37])));    //< 8 bit(s) from B271
        to->INS_EastSpd = ((raw * 0.0030518) + (-100));

        raw  = ((uint32_t)((bytes[38]))) << 8;    //< 8 bit(s) from B279
        raw |= ((uint32_t)((bytes[39])));    //< 8 bit(s) from B287
        to->INS_ToGroundSpd = ((raw * 0.0030518) + (-100));

        raw  = ((uint32_t)((bytes[40])));    //< 8 bit(s) from B303
        to->INS_GpsFlag_Pos = ((raw));

        raw  = ((uint32_t)((bytes[41])));    //< 8 bit(s) from B311
        to->INS_NumSV = ((raw));

        raw  = ((uint32_t)((bytes[42])));    //< 8 bit(s) from B319
        to->INS_GpsFlag_Heading = ((raw));

        raw  = ((uint32_t)((bytes[43])));    //< 8 bit(s) from B327
        to->INS_Gps_Age = ((raw));

        raw  = ((uint32_t)((bytes[44])));    //< 8 bit(s) from B335
        to->INS_Car_Status = ((raw));

        raw  = ((uint32_t)((bytes[45])));    //< 8 bit(s) from B391
        to->INS_Status = ((raw));

        raw = ((uint32_t)((bytes[46]))) << 8;;    //< 8 bit(s) from B399
        raw |= ((uint32_t)((bytes[47])));    //< 8 bit(s) from B399
        to->INS_Std_Lat = ((raw * 0.001));

        raw = ((uint32_t)((bytes[48]))) << 8;;    //< 8 bit(s) from B399
        raw |= ((uint32_t)((bytes[49])));    //< 8 bit(s) from B399
        to->INS_Std_Lon = ((raw * 0.001));

        raw = ((uint32_t)((bytes[50]))) << 8;;    //< 8 bit(s) from B399
        raw |= ((uint32_t)((bytes[51])));    //< 8 bit(s) from B399
        to->INS_Std_LocatHeight = ((raw * 0.001));

        raw = ((uint32_t)((bytes[52]))) << 8;;    //< 8 bit(s) from B399
        raw |= ((uint32_t)((bytes[53])));    //< 8 bit(s) from B399
        to->INS_Std_Heading = ((raw * 0.001));

        raw = ((uint16_t)((bytes[54]))) << 8;    //< 8 bit(s) from B399
        raw |= ((uint16_t)((bytes[55]))) ;    //< 8 bit(s) from B399
        to->Week = (raw);

        raw  = ((uint32_t)((bytes[56]))) << 24;    //< 8 bit(s) from B87
        raw |= ((uint32_t)((bytes[57]))) << 16;    //< 8 bit(s) from B95
        raw |= ((uint32_t)((bytes[58]))) << 8;    //< 8 bit(s) from B103
        raw |= ((uint32_t)((bytes[59])));    //< 8 bit(s) from B111
        to->TimeOfWeek = (raw);
        to->mia_info.mia_counter_ms = 0; ///< Reset the MIA counter
#if 0
        printf("ACC_X:%11.4f\n", to->ACC_X);
        printf("ACC_Y:%11.4f\n", to->ACC_Y);
        printf("ACC_Z:%11.4f\n", to->ACC_Z);
        printf("GYRO_X:%0.4f\n", to->GYRO_X);
        printf("GYRO_Y:%0.4f\n", to->GYRO_Y);
        printf("GYRO_Z:%0.4f\n", to->GYRO_Z);
        printf("INS_PitchAngle:%0.7f\n", to->INS_PitchAngle);
        printf("INS_RollAngle:%0.7f\n", to->INS_RollAngle);
        printf("INS_HeadingAngle:%0.7f\n", to->INS_HeadingAngle);
        printf("INS_LocatHeight:%0.7f\n", to->INS_LocatHeight);
        printf("IMU_Status:%d\n", to->IMU_Status);
        printf("INS_Latitude:%0.7f\n", to->INS_Latitude);
        printf("INS_Longitude:%0.7f\n", to->INS_Longitude);
        printf("INS_NorthSpd:%0.3f\n", to->INS_NorthSpd);
        printf("INS_EastSpd:%0.3f\n", to->INS_EastSpd);
        printf("INS_ToGroundSpd:%0.3f\n", to->INS_ToGroundSpd);
        printf("INS_GpsFlag_Pos:%d\n", to->INS_GpsFlag_Pos);
        printf("INS_NumSV:%d\n", to->INS_NumSV);
        printf("INS_GpsFlag_Heading:%d\n", to->INS_GpsFlag_Heading);
        printf("INS_Gps_Age:%d\n", to->INS_Gps_Age);
        printf("INS_Car_Status:%d\n", to->INS_Car_Status);
        printf("INS_Status:%d\n", to->INS_Status);
        printf("INS_Std_Lat:%0.2f\n", to->INS_Std_Lat);
        printf("INS_Std_Lon:%0.2f\n", to->INS_Std_Lon);
        printf("INS_Std_LocatHeight:%0.2f\n", to->INS_Std_LocatHeight);
        printf("INS_Std_Heading:%0.2f\n", to->INS_Std_Heading);
#endif
		sprintf(ins401c_output_msg_inspva, 
			"%11.4f,%11.4f,%11.4f,"
			"%11.4f,%11.4f,%11.4f,"
			"%11.4f,%11.4f,%11.4f,"
			"%11.7f,%u,"
			"%11.7f,%11.7f,"
			"%11.4f,%11.4f,%11.4f,"
			"%d,%d,%d,%d,%d,%d,"
			"%11.4f,%11.4f,%11.4f,%11.4f,"
			"%6d,%d\n",\
        to->ACC_X, to->ACC_Y, to->ACC_Z,\
        to->GYRO_X, to->GYRO_Y, to->GYRO_Z,\
        to->INS_PitchAngle, to->INS_RollAngle, to->INS_HeadingAngle,\
        to->INS_LocatHeight, to->IMU_Status,\
        to->INS_Latitude, to->INS_Longitude,\
        to->INS_NorthSpd, to->INS_EastSpd, to->INS_ToGroundSpd,\
        to->INS_GpsFlag_Pos, to->INS_NumSV, to->INS_GpsFlag_Heading, to->INS_Gps_Age, to->INS_Car_Status, to->INS_Status,\
        to->INS_Std_Lat, to->INS_Std_Lon, to->INS_Std_LocatHeight, to->INS_Std_Heading, \
        to->Week, to->TimeOfWeek\
        );
        sprintf(ins401c_output_msg_imu, "%d,%d,%11.4f,%11.4f,%11.4f,%11.4f,%11.4f,%11.4f,%d\n",to->Week,\
        to->TimeOfWeek,\
        to->ACC_X, to->ACC_Y, to->ACC_Z,\
        to->GYRO_X, to->GYRO_Y, to->GYRO_Z,\
        to->IMU_Status\
        );

        sprintf(ins401c_output_msg_ins, "%d,%d,%d,%d,%11.7f,%11.7f,%11.7f,%11.4f,%11.4f,%11.4f,%11.4f,%11.4f,%11.4f,%11.4f,%11.4f,%11.4f\n",
        to->Week, to->TimeOfWeek,\
        to->INS_GpsFlag_Pos, to->INS_Status,\
        to->INS_Latitude, to->INS_Longitude, to->INS_LocatHeight,\
        to->INS_NorthSpd, to->INS_EastSpd, to->INS_ToGroundSpd,\
        to->INS_RollAngle, to->INS_PitchAngle, to->INS_HeadingAngle,\
        to->INS_Std_Lat, to->INS_Std_Lon, to->INS_Std_LocatHeight\
        );
        ins_mess.week = to->Week;
        ins_mess.time_of_week = to->TimeOfWeek;
        ins_mess.ins_car_status = to->INS_Car_Status; // odo used or not
        /* Tag20230118 add by wrj, add ins status */
        ins_mess.ins_status = to->INS_Status;
        ins_mess.ins_position_status = to->INS_GpsFlag_Pos; // ins gps position type
        ins_mess.latitude = to->INS_Latitude;
        ins_mess.longitude = to->INS_Longitude;
        ins_mess.height = to->INS_LocatHeight;
        ins_mess.north_vel = to->INS_NorthSpd;
        ins_mess.east_vel = to->INS_EastSpd;
        ins_mess.up_vel = to->INS_ToGroundSpd;
        ins_mess.roll = to->INS_RollAngle;
        ins_mess.pitch = to->INS_PitchAngle;
        ins_mess.heading = to->INS_HeadingAngle;
        ins_mess.latitude_std = to->INS_Std_Lat;
        ins_mess.longitude_std = to->INS_Std_Lon;
        ins_mess.height_std = to->INS_Std_LocatHeight;

        write_ins401c_log_file(ins401c_output_msg_inspva);
        write_ins401c_imu_file(ins401c_output_msg_imu);
        write_ins401c_ins_file(ins401c_output_msg_ins);
		append_ins_kml();

#if 1
		if ((uint32_t)(ins_mess.time_of_week) % 100 < 10) {
			create_file(f_ins_txt, "ins.txt", NULL);
			fprintf(f_ins_txt, "%d,%11.4f,%14.9f,%14.9f,%10.4f,%10.4f,%10.4f,%10.4f,%14.9f,%14.9f,%14.9f,%3d,%3d\n",
				ins_mess.week, (double)ins_mess.time_of_week / 1000.0, ins_mess.latitude, ins_mess.longitude, ins_mess.height,
				ins_mess.north_vel, ins_mess.east_vel, ins_mess.up_vel,
				ins_mess.roll, ins_mess.pitch, ins_mess.heading, ins_mess.ins_car_status, ins_mess.ins_position_status);
		}
#endif

        return success;
    }

    /* Decode Vector__XXX* Gnss_Message message
    * @param hdr  The header of the message to validate its DLC and MID; this can be NULL to skip this check */
    bool dbc_decode_Gnss_Message(uint8_t *pstu, const uint8_t *bytes)
    {
        const bool success = true;
        // If msg header is provided, check if the DLC and the MID match
        if (NULL == pstu || NULL == bytes) {
            return !success;
        }
        uint32_t raw;
        Gnss_Message_t *to = (Gnss_Message_t *)pstu;

        raw  = ((uint32_t)((bytes[0]))) << 8;    //< 8 bit(s) from B7
        raw |= ((uint32_t)((bytes[1])));    //< 8 bit(s) from B15
        to->Gnss_Gps_Week = ((raw));

        raw  = ((uint32_t)((bytes[2]))) << 24;    //< 8 bit(s) from B23
        raw |= ((uint32_t)((bytes[3]))) << 16;    //< 8 bit(s) from B31
        raw |= ((uint32_t)((bytes[4]))) << 8;    //< 8 bit(s) from B39
        raw |= ((uint32_t)((bytes[5])));    //< 8 bit(s) from B47
        to->Gnss_Gps_Milliseconds = ((raw));

        /* utc: hhmmss */
        raw  = ((uint32_t)((bytes[6]))) << 24;    //< 8 bit(s) from B55
        raw |= ((uint32_t)((bytes[7]))) << 16;    //< 8 bit(s) from B63
        raw |= ((uint32_t)((bytes[8]))) << 8;    //< 8 bit(s) from B71
        raw |= ((uint32_t)((bytes[9])));    //< 8 bit(s) from B79
        to->Gnss_UTC = ((raw));

        raw  = ((uint32_t)((bytes[10])));    //< 8 bit(s) from B87
        to->Gnss_Leap_Second = ((raw));

        raw  = ((uint32_t)((bytes[11]))) << 24;    //< 8 bit(s) from B95
        raw |= ((uint32_t)((bytes[12]))) << 16;    //< 8 bit(s) from B103
        raw |= ((uint32_t)((bytes[13]))) << 8;    //< 8 bit(s) from B111
        raw |= ((uint32_t)((bytes[14])));    //< 8 bit(s) from B119
        to->Gnss_MCU_Time_Stamp = ((raw));

        raw  = ((uint32_t)((bytes[15])));    //< 8 bit(s) from B127
        to->Gnss_Position_Type = ((raw));

        raw  = ((uint32_t)((bytes[16])));    //< 8 bit(s) from B135
        to->Gnss_NumberOfSVs = ((raw));

        raw  = ((uint32_t)((bytes[17]))) << 8;    //< 8 bit(s) from B143
        raw |= ((uint32_t)((bytes[18])));    //< 8 bit(s) from B151
        to->Gnss_Hdop = ((raw * 0.0015259021));

        raw  = ((uint32_t)((bytes[19]))) << 8;    //< 8 bit(s) from B159
        raw |= ((uint32_t)((bytes[20])));    //< 8 bit(s) from B167
        to->Gnss_Speed_Over_Ground = ((raw * 0.00183108));

        raw  = ((uint32_t)((bytes[21]))) << 8;    //< 8 bit(s) from B175
        raw |= ((uint32_t)((bytes[22])));    //< 8 bit(s) from B183
        to->Gnss_GPS_Course = ((raw * 0.1));

        raw  = ((uint32_t)((bytes[23]))) << 24;    //< 8 bit(s) from B191
        raw |= ((uint32_t)((bytes[24]))) << 16;    //< 8 bit(s) from B199
        raw |= ((uint32_t)((bytes[25]))) << 8;    //< 8 bit(s) from B207
        raw |= ((uint32_t)((bytes[26])));    //< 8 bit(s) from B215
        to->Gnss_Latitude = ((raw * 1e-07) + (-180));

        raw  = ((uint32_t)((bytes[27]))) << 24;    //< 8 bit(s) from B223
        raw |= ((uint32_t)((bytes[28]))) << 16;    //< 8 bit(s) from B231
        raw |= ((uint32_t)((bytes[29]))) << 8;    //< 8 bit(s) from B239
        raw |= ((uint32_t)((bytes[30])));    //< 8 bit(s) from B247
        to->Gnss_Longitude = ((raw * 1e-07) + (-180));

        raw  = ((uint32_t)((bytes[31]))) << 24;    //< 8 bit(s) from B255
        raw |= ((uint32_t)((bytes[32]))) << 16;    //< 8 bit(s) from B263
        raw |= ((uint32_t)((bytes[33]))) << 8;    //< 8 bit(s) from B271
        raw |= ((uint32_t)((bytes[34])));    //< 8 bit(s) from B279
        to->Gnss_Height = ((raw * 0.001) + (-10000));

        raw  = ((uint32_t)((bytes[35]))) << 8;    //< 8 bit(s) from B287
        raw |= ((uint32_t)((bytes[36])));    //< 8 bit(s) from B295
        to->Gnss_Latitude_Std = ((raw * 0.001));

        raw  = ((uint32_t)((bytes[37]))) << 8;    //< 8 bit(s) from B303
        raw |= ((uint32_t)((bytes[38])));    //< 8 bit(s) from B311
        to->Gnss_Longitude_Std = ((raw * 0.001));

        raw  = ((uint32_t)((bytes[39]))) << 8;    //< 8 bit(s) from B319
        raw |= ((uint32_t)((bytes[40])));    //< 8 bit(s) from B327
        to->Gnss_Height_Std = ((raw * 0.001));

        /* sep in gga */
        raw  = (uint32_t)(bytes[41] << 24);
        raw |= (uint32_t)(bytes[42] << 16);
        raw |= (uint32_t)(bytes[43] << 8);
        raw |= (uint32_t)(bytes[44] << 0);
        to->sep = raw * 0.001 + (-10000);

        /* utc: year mouth day */
        raw  = (uint32_t)(bytes[45] << 24);
        raw |= (uint32_t)(bytes[46] << 16);
        raw |= (uint32_t)(bytes[47] << 8);
        raw |= (uint32_t)(bytes[48] << 0);
        to->Gnss_UTC_YMD = raw;

        /* north spd */
        raw  = (uint32_t)(bytes[49] << 8);
        raw |= (uint32_t)(bytes[50] << 0);
        to->Gnss_North_Spd = raw * 0.0030518 + (-100);

        /* east spd */
        raw  = (uint32_t)(bytes[51] << 8);
        raw |= (uint32_t)(bytes[52] << 0);
        to->Gnss_East_Spd = raw * 0.0030518 + (-100);

        sprintf(ins401c_output_msg_gnss, "%d,%d,%d,%d,%d,%d,%d,%11.4f,%11.4f,%11.4f,%11.7f,%11.7f,%11.7f,%11.4f,%11.4f,%11.4f,%11.4f,%d,%11.4f,%11.4f\n",to->Gnss_Gps_Week,\
        to->Gnss_Gps_Milliseconds,\
        (to->Gnss_UTC), to->Gnss_Leap_Second,\
        to->Gnss_MCU_Time_Stamp, to->Gnss_Position_Type, to->Gnss_NumberOfSVs,\
        to->Gnss_Hdop, to->Gnss_Speed_Over_Ground, to->Gnss_GPS_Course, to->Gnss_Latitude, to->Gnss_Longitude, to->Gnss_Height, to->Gnss_Latitude_Std, to->Gnss_Longitude_Std, to->Gnss_Height_Std,\
        to->sep, to->Gnss_UTC_YMD, to->Gnss_North_Spd, to->Gnss_East_Spd\
        );

        to->mia_info.mia_counter_ms = 0; ///< Reset the MIA counter
        write_ins401c_gnss_file(ins401c_output_msg_gnss);

        gnss_mess.gps_week = to->Gnss_Gps_Week;
        gnss_mess.gps_millisecs = to->Gnss_Gps_Milliseconds;
        gnss_mess.position_type = to->Gnss_Position_Type;
        gnss_mess.latitude = to->Gnss_Latitude;
        gnss_mess.longitude = to->Gnss_Longitude;
        gnss_mess.height = to->Gnss_Height;
        /* Tag20230118 modify by wrj */
        // gnss_mess.north_vel = cos(to->Gnss_GPS_Course * D2R) * to->Gnss_Speed_Over_Ground;
        // gnss_mess.east_vel = sin(to->Gnss_GPS_Course * D2R) * to->Gnss_Speed_Over_Ground;
        gnss_mess.north_vel = to->Gnss_North_Spd;
        gnss_mess.east_vel = to->Gnss_East_Spd;
        gnss_mess.up_vel = 0;

		append_gnss_kml();
        return success;
    }

    bool dbc_decode_Diagnostic_Message(uint8_t *pstu, const uint8_t *bytes)
    {
        const bool success = true;
        // If msg header is provided, check if the DLC and the MID match
        if (NULL == pstu || NULL == bytes) {
            return !success;
        }

        uint32_t raw;
        Diagnostic_Message_t *to = (Diagnostic_Message_t *)pstu;

        raw  = ((uint32_t)((bytes[0]))) << 8;    //< 8 bit(s) from B7
        raw |= ((uint32_t)((bytes[1])));    //< 8 bit(s) from B15
        to->Gnss_Gps_Week = ((raw));
        raw  = ((uint32_t)((bytes[2]))) << 24;    //< 8 bit(s) from B23
        raw |= ((uint32_t)((bytes[3]))) << 16;    //< 8 bit(s) from B31
        raw |= ((uint32_t)((bytes[4]))) << 8;    //< 8 bit(s) from B39
        raw |= ((uint32_t)((bytes[5])));    //< 8 bit(s) from B47
        to->Gnss_Gps_Milliseconds = ((raw));
        raw  = ((uint32_t)((bytes[6]))) << 24;    //< 8 bit(s) from B55
        raw |= ((uint32_t)((bytes[7]))) << 16;    //< 8 bit(s) from B63
        raw |= ((uint32_t)((bytes[8]))) << 8;    //< 8 bit(s) from B71
        raw |= ((uint32_t)((bytes[9])));    //< 8 bit(s) from B79
        to->Diagnostic_Device_Status_Bit = ((raw));
        raw  = ((uint32_t)((bytes[10]))) << 8;    //< 8 bit(s) from B87
        raw |= ((uint32_t)((bytes[11])));    //< 8 bit(s) from B95
        to->Diagnostic_IMU_Temperature = ((raw * 0.01) + (-100));
        raw  = ((uint32_t)((bytes[12]))) << 8;    //< 8 bit(s) from B103
        raw |= ((uint32_t)((bytes[13])));    //< 8 bit(s) from B111
        to->Diagnostic_MCU_Temperature = ((raw * 0.01) + (-100));
        raw  = ((uint32_t)((bytes[14]))) << 8;    //< 8 bit(s) from B119
        raw |= ((uint32_t)((bytes[15])));    //< 8 bit(s) from B127
        to->Diagnostic_STA_Temperature = ((raw * 0.01) + (-100));

        to->mia_info.mia_counter_ms = 0; ///< Reset the MIA counter
        sprintf(ins401c_output_msg_diagnostic, "%11.4f,%d,%11.4f,%11.4f,%11.4f,%d\n",\
        (double)(to->Gnss_Gps_Milliseconds)/1000,\
        to->Gnss_Gps_Week, to->Diagnostic_STA_Temperature, to->Diagnostic_MCU_Temperature,\
        to->Diagnostic_IMU_Temperature, to->Diagnostic_Device_Status_Bit\
        );
        write_ins401c_diagnostic_file(ins401c_output_msg_diagnostic);
        return success;
    }

    bool dbc_decode_odo_Message(uint8_t *pstu, const uint8_t *bytes)
    {
        const bool success = true;
        // If msg header is provided, check if the DLC and the MID match
        if (NULL == pstu || NULL == bytes) {
            return !success;
        }

        uint32_t raw;
        odo_t *to = (odo_t *)pstu;

        /* gps week */
        raw  = (uint32_t)(bytes[0] << 8);
        raw |= (uint32_t)bytes[1];
        to->GPS_Week = raw * 1 + 0;

        /* time of week */
        raw  = (uint32_t)(bytes[2] << 24);
        raw |= (uint32_t)(bytes[3] << 16);
        raw |= (uint32_t)(bytes[4] << 8);
        raw |= (uint32_t)(bytes[5] << 0);
        to->GPS_TimeOfWeek = raw * 0.001 + 0;

        /* mode */
        raw  = (uint32_t)(bytes[6] << 0);
        to->mode = raw * 1 + 0;

        /* speed */
        raw  = (uint32_t)(bytes[7] << 8);
        raw |= (uint32_t)(bytes[8] << 0);
        to->speed = raw * 0.0030518 + (-100);

        /* fwd */
        raw  = (uint32_t)(bytes[9] << 0);
        to->fwd = raw * 1 + 0;

        sprintf(ins401c_output_msg_odo, "%d,%11.4f,%d,%11.4f,%d\n",\
        to->GPS_Week, to->GPS_TimeOfWeek, to->mode, to->speed, to->fwd\
        );
        write_ins401c_odo_file(ins401c_output_msg_odo);
        return success;
    }


    /* Decode Vector__XXX* INS_ACC message
    * @param hdr  The header of the message to validate its DLC and MID; this can be NULL to skip this check */
    bool dbc_decode_INS_ACC(uint8_t *pstu, const uint8_t *bytes)
    {
        const bool success = true;
        // If msg header is provided, check if the DLC and the MID match
        if (NULL == pstu || NULL == bytes) {
            return !success;
        }

        uint32_t raw;
        INS_ACC_t *to = (INS_ACC_t *)pstu;

        raw  = ((uint32_t)((bytes[0]))) << 8;    //< 8 bit(s) from B7
        raw |= ((uint32_t)((bytes[1])));    //< 8 bit(s) from B15
        to->ACC_X = ((raw * 0.0001220703125) + (-4));
        raw  = ((uint32_t)((bytes[2]))) << 8;    //< 8 bit(s) from B23
        raw |= ((uint32_t)((bytes[3])));    //< 8 bit(s) from B31
        to->ACC_Y = ((raw * 0.0001220703125) + (-4));
        raw  = ((uint32_t)((bytes[4]))) << 8;    //< 8 bit(s) from B39
        raw |= ((uint32_t)((bytes[5])));    //< 8 bit(s) from B47
        to->ACC_Z = ((raw * 0.0001220703125) + (-4));

        to->mia_info.mia_counter_ms = 0; ///< Reset the MIA counter
        // printf("ACC_X:%0.2f\n", to->ACC_X * 9.7803267714);
        // printf("ACC_Y:%0.2f\n", to->ACC_Y * 9.7803267714);
        // printf("ACC_Z:%0.2f\n", to->ACC_Z * 9.7803267714);
        can_mess_flag = 0x01;
        imu_mess.acc_x = to->ACC_X * 9.7803267714;
        imu_mess.acc_y = to->ACC_Y * 9.7803267714;
        imu_mess.acc_z = to->ACC_Z * 9.7803267714;
        return success;
    }

    /* Decode Vector__XXX* INS_GYRO message
    * @param hdr  The header of the message to validate its DLC and MID; this can be NULL to skip this check */
    bool dbc_decode_INS_GYRO(uint8_t *pstu, const uint8_t *bytes)
    {
        const bool success = true;
        // If msg header is provided, check if the DLC and the MID match
        if (NULL == pstu || NULL == bytes) {
            return !success;
        }

        uint32_t raw;
        INS_GYRO_t *to = (INS_GYRO_t *)pstu;

        raw  = ((uint32_t)((bytes[0]))) << 8;    //< 8 bit(s) from B7
        raw |= ((uint32_t)((bytes[1])));    //< 8 bit(s) from B15
        to->GYRO_X = ((raw * 0.0076293) + (-250));
        raw  = ((uint32_t)((bytes[2]))) << 8;    //< 8 bit(s) from B23
        raw |= ((uint32_t)((bytes[3])));    //< 8 bit(s) from B31
        to->GYRO_Y = ((raw * 0.0076293) + (-250));
        raw  = ((uint32_t)((bytes[4]))) << 8;    //< 8 bit(s) from B39
        raw |= ((uint32_t)((bytes[5])));    //< 8 bit(s) from B47
        to->GYRO_Z = ((raw * 0.0076293) + (-250));

        to->mia_info.mia_counter_ms = 0; ///< Reset the MIA counter

        if(can_mess_flag == 0x01)
        {
            can_mess_flag|= 0x02;
        }
        else
        {
            can_mess_flag = 0x00;
        }
        imu_mess.gryo_x = to->GYRO_X ;
        imu_mess.gryo_y = to->GYRO_Y ;
        imu_mess.gryo_z = to->GYRO_Z ;
        return success;
    }

    /* Decode Vector__XXX* INS_HeadingPitchRoll message
    * @param hdr  The header of the message to validate its DLC and MID; this can be NULL to skip this check */
    bool dbc_decode_INS_HeadingPitchRoll(uint8_t *pstu, const uint8_t *bytes)
    {
        const bool success = true;
        // If msg header is provided, check if the DLC and the MID match
        if (NULL == pstu || NULL == bytes) {
            return !success;
        }

        uint32_t raw;
        INS_HeadingPitchRoll_t *to = (INS_HeadingPitchRoll_t *)pstu;

        raw  = ((uint32_t)((bytes[0]))) << 8;    //< 8 bit(s) from B7
        raw |= ((uint32_t)((bytes[1])));    //< 8 bit(s) from B15
        to->INS_PitchAngle = ((raw * 0.01098649576) + (-360));
        raw  = ((uint32_t)((bytes[2]))) << 8;    //< 8 bit(s) from B23
        raw |= ((uint32_t)((bytes[3])));    //< 8 bit(s) from B31
        to->INS_RollAngle = ((raw * 0.01098649576) + (-360));
        raw  = ((uint32_t)((bytes[4]))) << 8;    //< 8 bit(s) from B39
        raw |= ((uint32_t)((bytes[5])));    //< 8 bit(s) from B47
        to->INS_HeadingAngle = ((raw * 0.01098649576) + (-360));

        to->mia_info.mia_counter_ms = 0; ///< Reset the MIA counter
        // printf("INS_PitchAngle:%0.2f\n", to->INS_PitchAngle);
        // printf("INS_RollAngle:%0.2f\n", to->INS_RollAngle);
        // printf("INS_HeadingAngle:%0.2f\n", to->INS_HeadingAngle);
        if(can_mess_flag == 0x03)
        {
            can_mess_flag|= 0x04;
        }
        else
        {
            can_mess_flag = 0x00;
        }
        ins_mess.pitch = to->INS_PitchAngle;
        ins_mess.roll = to->INS_RollAngle;
        ins_mess.heading = to->INS_HeadingAngle;
        return success;
    }

    /* Decode Vector__XXX* INS_HeightAndIMUStatus message
    * @param hdr  The header of the message to validate its DLC and MID; this can be NULL to skip this check */
    bool dbc_decode_INS_HeightAndIMUStatus(uint8_t *pstu, const uint8_t *bytes)
    {
        const bool success = true;
        // If msg header is provided, check if the DLC and the MID match
        if (NULL == pstu || NULL == bytes) {
            return !success;
        }

        uint32_t raw;
        INS_HeightAndIMUStatus_t *to = (INS_HeightAndIMUStatus_t *)pstu;

        raw  = ((uint32_t)((bytes[0]))) << 24;    //< 8 bit(s) from B7
        raw |= ((uint32_t)((bytes[1]))) << 16;    //< 8 bit(s) from B15
        raw |= ((uint32_t)((bytes[2]))) << 8;    //< 8 bit(s) from B23
        raw |= ((uint32_t)((bytes[3])));    //< 8 bit(s) from B31
        to->INS_LocatHeight = ((raw * 0.001) + (-10000));
        raw  = ((uint32_t)((bytes[4]))) << 24;    //< 8 bit(s) from B39
        raw |= ((uint32_t)((bytes[5]))) << 16;    //< 8 bit(s) from B47
        raw |= ((uint32_t)((bytes[6]))) << 8;    //< 8 bit(s) from B55
        raw |= ((uint32_t)((bytes[7])));    //< 8 bit(s) from B63
        to->IMU_Status = ((raw));

        to->mia_info.mia_counter_ms = 0; ///< Reset the MIA counter
        // printf("INS_LocatHeight:%0.2f\n", to->INS_LocatHeight);
        // printf("IMU_Status:%d\n", to->IMU_Status);
        if(can_mess_flag == 0x07)
        {
            can_mess_flag|= 0x08;
        }
        else
        {
            can_mess_flag = 0x00;
        }
        ins_mess.height = to->INS_LocatHeight;
        imu_mess.imu_status = to->IMU_Status;
        return success;
    }

    /* Decode Vector__XXX* INS_LatitudeLongitude message
    * @param hdr  The header of the message to validate its DLC and MID; this can be NULL to skip this check */
    bool dbc_decode_INS_LatitudeLongitude(uint8_t *pstu, const uint8_t *bytes)
    {
        const bool success = true;
        // If msg header is provided, check if the DLC and the MID match
        if (NULL == pstu || NULL == bytes) {
            return !success;
        }

        uint32_t raw;
        INS_LatitudeLongitude_t *to = (INS_LatitudeLongitude_t *)pstu;

        raw  = ((uint32_t)((bytes[0]))) << 24;    //< 8 bit(s) from B7
        raw |= ((uint32_t)((bytes[1]))) << 16;    //< 8 bit(s) from B15
        raw |= ((uint32_t)((bytes[2]))) << 8;    //< 8 bit(s) from B23
        raw |= ((uint32_t)((bytes[3])));    //< 8 bit(s) from B31
        to->INS_Latitude = ((raw * 1e-07) + (-180));
        raw  = ((uint32_t)((bytes[4]))) << 24;    //< 8 bit(s) from B39
        raw |= ((uint32_t)((bytes[5]))) << 16;    //< 8 bit(s) from B47
        raw |= ((uint32_t)((bytes[6]))) << 8;    //< 8 bit(s) from B55
        raw |= ((uint32_t)((bytes[7])));    //< 8 bit(s) from B63
        to->INS_Longitude = ((raw * 1e-07) + (-180));

        to->mia_info.mia_counter_ms = 0; ///< Reset the MIA counter
        // printf("INS_Latitude:%0.4f\n", to->INS_Latitude);
        // printf("INS_Longitude:%0.4f\n", to->INS_Longitude);
        if(can_mess_flag == 0x0f)
        {
            can_mess_flag|= 0x10;
        }
        else
        {
            can_mess_flag = 0x00;
        }
        ins_mess.latitude = to->INS_Latitude;
        ins_mess.longitude = to->INS_Longitude;
        return success;
    }

    /* Decode Vector__XXX* INS_Speed message
    * @param hdr  The header of the message to validate its DLC and MID; this can be NULL to skip this check */
    bool dbc_decode_INS_Speed(uint8_t *pstu, const uint8_t *bytes)
    {
        const bool success = true;
        // If msg header is provided, check if the DLC and the MID match
        if (NULL == pstu || NULL == bytes) {
            return !success;
        }

        uint32_t raw;
        INS_Speed_t *to = (INS_Speed_t *)pstu;

        raw  = ((uint32_t)((bytes[0]))) << 8;    //< 8 bit(s) from B7
        raw |= ((uint32_t)((bytes[1])));    //< 8 bit(s) from B15
        to->INS_NorthSpd = ((raw * 0.0030517) + (-100));
        raw  = ((uint32_t)((bytes[2]))) << 8;    //< 8 bit(s) from B23
        raw |= ((uint32_t)((bytes[3])));    //< 8 bit(s) from B31
        to->INS_EastSpd = ((raw * 0.0030517) + (-100));
        raw  = ((uint32_t)((bytes[4]))) << 8;    //< 8 bit(s) from B39
        raw |= ((uint32_t)((bytes[5])));    //< 8 bit(s) from B47
        to->INS_ToGroundSpd = ((raw * 0.0030517) + (-100));

        to->mia_info.mia_counter_ms = 0; ///< Reset the MIA counter
        // printf("INS_NorthSpd:%0.2f\n", to->INS_NorthSpd);
        // printf("INS_EastSpd:%0.2f\n", to->INS_EastSpd);
        // printf("INS_ToGroundSpd:%0.2f\n", to->INS_ToGroundSpd);
        if(can_mess_flag == 0x1f)
        {
            can_mess_flag|= 0x20;
        }
        else
        {
            can_mess_flag = 0x00;
        }
        ins_mess.north_vel = to->INS_NorthSpd;
        ins_mess.east_vel = to->INS_EastSpd;
        ins_mess.up_vel = to->INS_ToGroundSpd;
        return success;
    }

    /* Decode Vector__XXX* INS_DataInfo message
    * @param hdr  The header of the message to validate its DLC and MID; this can be NULL to skip this check */
    bool dbc_decode_INS_DataInfo(uint8_t *pstu, const uint8_t *bytes)
    {
        const bool success = true;
        // If msg header is provided, check if the DLC and the MID match
        if (NULL == pstu || NULL == bytes) {
            return !success;
        }

        uint32_t raw;
        INS_DataInfo_t *to = (INS_DataInfo_t *)pstu;

        raw  = ((uint32_t)((bytes[0])));    //< 8 bit(s) from B7
        to->INS_GpsFlag_Pos = ((raw));
        raw  = ((uint32_t)((bytes[1])));    //< 8 bit(s) from B15
        to->INS_NumSV = ((raw));
        raw  = ((uint32_t)((bytes[2])));    //< 8 bit(s) from B23
        to->INS_GpsFlag_Heading = ((raw));
        raw  = ((uint32_t)((bytes[3])));    //< 8 bit(s) from B31
        to->INS_Gps_Age = ((raw));
        raw  = ((uint32_t)((bytes[4])));    //< 8 bit(s) from B39
        to->INS_Car_Status = ((raw));
        raw  = ((uint32_t)((bytes[5])));    //< 8 bit(s) from B47
        to->INS_Status = ((raw));

        to->mia_info.mia_counter_ms = 0; ///< Reset the MIA counter
        // printf("INS_GpsFlag_Pos:%d\n", to->INS_GpsFlag_Pos);
        // printf("INS_NumSV:%d\n", to->INS_NumSV);
        // printf("INS_GpsFlag_Heading:%d\n", to->INS_GpsFlag_Heading);
        // printf("INS_Gps_Age:%d\n", to->INS_Gps_Age);
        // printf("INS_Car_Status:%d\n", to->INS_Car_Status);
        // printf("INS_Status:%d\n", to->INS_Status);
        if(can_mess_flag == 0x3f)
        {
            can_mess_flag|= 0x40;
        }
        else
        {
            can_mess_flag = 0x00;
        }
        ins_mess.ins_position_status = to->INS_GpsFlag_Pos;
        ins_mess.ins_car_status = to->INS_Car_Status;
        return success;
    }

    /* Decode Vector__XXX* INS_Std message
    * @param hdr  The header of the message to validate its DLC and MID; this can be NULL to skip this check */
    bool dbc_decode_INS_Std(uint8_t *pstu, const uint8_t *bytes)
    {
        const bool success = true;
        // If msg header is provided, check if the DLC and the MID match
        if (NULL == pstu || NULL == bytes) {
            return !success;
        }

        uint32_t raw;
        INS_Std_t *to = (INS_Std_t *)pstu;

        raw  = ((uint32_t)((bytes[0]))) << 8;    //< 8 bit(s) from B7
        raw |= ((uint32_t)((bytes[1])));    //< 8 bit(s) from B15
        to->INS_Std_Lat = ((raw * 0.001));
        raw  = ((uint32_t)((bytes[2]))) << 8;    //< 8 bit(s) from B23
        raw |= ((uint32_t)((bytes[3])));    //< 8 bit(s) from B31
        to->INS_Std_Lon = ((raw * 0.001));
        raw  = ((uint32_t)((bytes[4]))) << 8;    //< 8 bit(s) from B39
        raw |= ((uint32_t)((bytes[5])));    //< 8 bit(s) from B47
        to->INS_Std_LocatHeight = ((raw * 0.001));
        raw  = ((uint32_t)((bytes[6]))) << 8;    //< 8 bit(s) from B55
        raw |= ((uint32_t)((bytes[7])));    //< 8 bit(s) from B63
        to->INS_Std_Heading = ((raw * 0.001));

        to->mia_info.mia_counter_ms = 0; ///< Reset the MIA counter
        // printf("INS_Std_Lat:%0.2f\n", to->INS_Std_Lat);
        // printf("INS_Std_Lon:%0.2f\n", to->INS_Std_Lon);
        // printf("INS_Std_LocatHeight:%0.2f\n", to->INS_Std_LocatHeight);
        // printf("INS_Std_Heading:%0.2f\n", to->INS_Std_Heading);
        if(can_mess_flag == 0x7f)
        {
            can_mess_flag|= 0x80;
        }
        else
        {
            can_mess_flag = 0x00;
        }
        ins_mess.latitude_std = to->INS_Std_Lat;
        ins_mess.longitude_std = to->INS_Std_Lon;
        ins_mess.height_std = to->INS_Std_LocatHeight;
        return success;
    }

    /* Decode Vector__XXX* INS_Time message
    * @param hdr  The header of the message to validate its DLC and MID; this can be NULL to skip this check */
    bool dbc_decode_INS_Time(uint8_t *pstu, const uint8_t *bytes)
    {
        const bool success = true;
        // If msg header is provided, check if the DLC and the MID match
        if (NULL == pstu || NULL == bytes) {
            return !success;
        }

        uint32_t raw;
        INS_Time_t *to = (INS_Time_t *)pstu;

        raw  = ((uint32_t)((bytes[0]))) << 8;    //< 8 bit(s) from B7
        raw |= ((uint32_t)((bytes[1])));    //< 8 bit(s) from B15
        to->Week = ((raw));
        raw  = ((uint32_t)((bytes[2]))) << 24;    //< 8 bit(s) from B7
        raw |= ((uint32_t)((bytes[3]))) << 16;    //< 8 bit(s) from B15
        raw |= ((uint32_t)((bytes[4]))) << 8;    //< 8 bit(s) from B23
        raw |= ((uint32_t)((bytes[5])));    //< 8 bit(s) from B31
        to->TimeOfWeek = ((raw));

        to->mia_info.mia_counter_ms = 0; ///< Reset the MIA counter

        imu_mess.week = to->Week;
        imu_mess.time_of_week = (double)(to->TimeOfWeek) / 1000;
        ins_mess.week = to->Week;
        ins_mess.time_of_week = (double)(to->TimeOfWeek) / 1000;
        if(can_mess_flag == 0xff)
        {
            can_mess_flag|= 0x100;
        }
        else
        {
            can_mess_flag = 0x00;
        }
        if(can_mess_flag == 0x1ff)
        {
            sprintf(ins401c_output_msg_imu, "%d,%11.4f,%11.4f,%11.4f,%11.4f,%11.4f,%11.4f,%11.4f,%d\n",imu_mess.week,\
            (double)(imu_mess.time_of_week),\
            imu_mess.acc_x, imu_mess.acc_y, imu_mess.acc_z,\
            imu_mess.gryo_x, imu_mess.gryo_y, imu_mess.gryo_z,\
            imu_mess.imu_status\
            );

            sprintf(ins401c_output_msg_ins, "%d,%11.4f,%d,%d,%11.7f,%11.7f,%11.7f,%11.4f,%11.4f,%11.4f,%11.4f,%11.4f,%11.4f,%11.4f,%11.4f,%11.4f\n",
            ins_mess.week, (double)(ins_mess.time_of_week),\
            ins_mess.ins_car_status, ins_mess.ins_position_status,\
            ins_mess.latitude, ins_mess.longitude, ins_mess.height,\
            ins_mess.north_vel, ins_mess.east_vel, ins_mess.up_vel,\
            ins_mess.roll, ins_mess.pitch, ins_mess.heading,\
            ins_mess.latitude_std, ins_mess.longitude_std, ins_mess.longitude_std\
            );

            write_ins401c_imu_file(ins401c_output_msg_imu);
            write_ins401c_ins_file(ins401c_output_msg_ins);
#if 1
		if ((uint32_t)(ins_mess.time_of_week) % 100 < 10) {
			create_file(f_ins_txt, "ins.txt", NULL);
			fprintf(f_ins_txt, "%d,%11.4f,%14.9f,%14.9f,%10.4f,%10.4f,%10.4f,%10.4f,%14.9f,%14.9f,%14.9f,%3d,%3d\n",
				ins_mess.week, (double)ins_mess.time_of_week / 1000.0, ins_mess.latitude, ins_mess.longitude, ins_mess.height,
				ins_mess.north_vel, ins_mess.east_vel, ins_mess.up_vel,
				ins_mess.roll, ins_mess.pitch, ins_mess.heading, ins_mess.ins_car_status, ins_mess.ins_position_status);
		}
#endif
    		append_ins_kml();
            can_mess_flag = 0;
        }
        return success;
    }

    bool canfd_dbc_msg_decode(uint32_t mid, uint8_t *from, uint8_t *to)
    {
        if (NULL == from || NULL == to)
            return false;
        for (int i = 0; i < sizeof(list_canfd_dbc_msgs)/sizeof(list_canfd_dbc_msgs[0]); i++) {
            if (mid == list_canfd_dbc_msgs[i].mid)
            {
                list_canfd_dbc_msgs[i].func(to, from);
            }
        }
        return true;
    }

    bool can_dbc_msg_decode(uint32_t mid, uint8_t *from, uint8_t *to)
    {
        if (NULL == from || NULL == to)
            return false;

        for (int i = 0; i < sizeof(list_can_dbc_msgs)/sizeof(list_can_dbc_msgs[0]); i++) {
            if (mid == list_can_dbc_msgs[i].mid)
            {
                list_can_dbc_msgs[i].func(to, from);
            }
        }
        return true;
    }

    int input_ins401c_line(uint8_t* data)
    {
        uint32_t mid = 0;
        uint8_t valid_data[1024];
        uint8_t result[1024];
        uint32_t count = 0;
        uint32_t data_len = (uint32_t)strlen((const char*)data);
        char* str = strtok((char*)data, " ");
        char* endptr = NULL;
        static uint8_t data_frame_start_flag = 0;
        uint16_t data_index = 0;
        while (str != NULL)
        {
            count++; 
            if(data_len > 150)
            {
                if(count == 4)
                {
                    mid = strtol(str, &endptr, 16);
                }
                if( (count >= 10) && (count <= 76) && (data_frame_start_flag == 1))
                {
                    valid_data[data_index++] = strtol(str, &endptr, 16);
                }
                if(strcmp(str, "64") == 0)
                {
                    data_frame_start_flag = 1;
                }
            }
            else
            {
                if(count == 4)
                {
                    mid = strtol(str, &endptr, 16);
                }
                if( (count >= 8) && (count <= 50) )
                {
                    valid_data[count-8] = strtol(str, &endptr, 16);
                }
            }
            str = strtok(NULL, " ");

        }
        if(count >= 20)
        {
            canfd_dbc_msg_decode(mid, valid_data, result);
        }
        else
        {
            can_dbc_msg_decode(mid, valid_data, result);
        }
        count = 0;
        data_frame_start_flag = 0;
        return 0;
    }
	void write_ins401c_log_file(char* log) {
		if (strlen(base_ins401c_file_name) == 0) return;
		char file_name[256] = { 0 };
        if (fs_canfd == NULL) {
            sprintf(file_name, "%s_INSPVA.csv", base_ins401c_file_name);
            fs_canfd = fopen(file_name, "w");
            if (fs_canfd) fprintf(fs_canfd, "ACC_X(s),ACC_Y(s),ACC_Z(s),GYRO_X(s),GYRO_Y(s),GYRO_Z(s),INS_PitchAngle(s),INS_RollAngle(s),INS_HeadingAngle(s),INS_LocatHeight(s),IMU_Status(s),INS_Latitude(s),INS_Longitude(s),INS_NorthSpd(s),INS_EastSpd(s),INS_ToGroundSpd(s),INS_GpsFlag_Pos(s),INS_NumSV(s),INS_GpsFlag_Heading(s),INS_Gps_Age(s),INS_Car_Status(s),INS_Status(s),INS_Std_Lat(s),INS_Std_Lon(s),INS_Std_LocatHeight(s),INS_Std_Heading(s),Week(s),TimeOfWeek(ms)\n");
        }
        if (fs_canfd) fprintf(fs_canfd, log);
	}
	void write_ins401c_imu_file(char* log) {
		if (strlen(base_ins401c_file_name) == 0) return;
		char file_name[256] = { 0 };
        if (fs_imu == NULL) {
            sprintf(file_name, "%s_imu.csv", base_ins401c_file_name);
            fs_imu = fopen(file_name, "w");
            if (fs_imu) fprintf(fs_imu, "GPS_Week(),GPS_TimeofWeek(ms),x_accel(m/s^2),y_accel(m/s^2),z_accel(m/s^2),x_rate(deg/s),y_rate(deg/s),z_rate(deg/s)\n");
        }
        if (fs_imu) fprintf(fs_imu, log);
	}
	void write_ins401c_ins_file(char* log) {
		if (strlen(base_ins401c_file_name) == 0) return;
		char file_name[256] = { 0 };
        if (fs_ins == NULL) {
            sprintf(file_name, "%s_ins.csv", base_ins401c_file_name);
            fs_ins = fopen(file_name, "w");
            /* Tag20230118 modiy by wrj, change insPositionType() to ins_status() */
            if (fs_ins) fprintf(fs_ins, "GPS_Week(),GPS_TimeofWeek(ms),INS_GpsFlag_Pos(),INS_Status(),latitude(deg),longitude(deg),height(m),velocityNorth(m/s),velocityEast(m/s),velocityUp(m/s),roll(deg),pitch(deg),heading(deg),latitude_std(m),longitude_std(m),height_std(m)\n");
        }
        if (fs_ins) fprintf(fs_ins, log);
	}

	void write_ins401c_gnss_file(char* log) {
		if (strlen(base_ins401c_file_name) == 0) return;
		char file_name[256] = { 0 };
        if (fs_gnss == NULL) {
            sprintf(file_name, "%s_gnss.csv", base_ins401c_file_name);
            fs_gnss = fopen(file_name, "w");
            if (fs_gnss) fprintf(fs_gnss, "Gnss_Gps_Week(),Gnss_Gps_Milliseconds(ms),Gnss_UTC(hhmmss),Gnss_Leap_Second(),Gnss_MCU_Time_Stamp(s),Gnss_Position_Type(),Gnss_NumberOfSVs(),Gnss_Hdop(), Gnss_Speed_Over_Ground(m/s), Gnss_GPS_Course(deg), Gnss_Latitude(deg), Gnss_Longitude(deg), Gnss_Height(m), Gnss_Latitude_Std(m), Gnss_Longitude_Std(m), Gnss_Height_Std(m), Gnss_sep(m), Gnss_UTC_YMD(YYYYMMDD), Gnss_North_Spd(m/s), Gnss_East_Spd(m/s)\n");
        }
        if (fs_gnss) fprintf(fs_gnss, log);
	}

	void write_ins401c_diagnostic_file(char* log) {
		if (strlen(base_ins401c_file_name) == 0) return;
		char file_name[256] = { 0 };
        if (fs_dia == NULL) {
            sprintf(file_name, "%s_diagnostic.csv", base_ins401c_file_name);
            fs_dia = fopen(file_name, "w");
            if (fs_dia) fprintf(fs_dia, "Gnss_Gps_Milliseconds(ms),Gnss_Gps_Week(),Diagnostic_STA_Temperature(), Diagnostic_MCU_Temperature(), Diagnostic_IMU_Temperature(), Diagnostic_Device_Status_Bit()\n");
        }
        if (fs_dia) fprintf(fs_dia, log);
	}

    void write_ins401c_odo_file(char* log) {
		if (strlen(base_ins401c_file_name) == 0) return;
		char file_name[256] = { 0 };
        if (fs_odo == NULL) {
            sprintf(file_name, "%s_odo.csv", base_ins401c_file_name);
            fs_odo = fopen(file_name, "w");
            if (fs_odo) fprintf(fs_odo, "Gps_Week(), GPS_TimeOfWeek(s), mode(), speed(m/s), fwd()\n");
        }
        if (fs_odo) fprintf(fs_odo, log);
	}


	void write_ins401c_kml_files() {
		Kml_Generator::Instance()->open_files(base_ins401c_file_name);
		Kml_Generator::Instance()->write_files();
		Kml_Generator::Instance()->close_files();
	}

	void close_ins401c_all_log_file()
	{
		if (fs_canfd) { fclose(fs_canfd); fs_canfd = NULL; }
		if (fs_imu) { fclose(fs_imu); fs_imu = NULL; }
		if (fs_ins) { fclose(fs_ins); fs_ins = NULL; }
		if (f_log) { fclose(f_log); f_log = NULL; }
		if (fs_gnss) { fclose(fs_gnss); fs_gnss = NULL; }
		if (fs_dia) { fclose(fs_dia); fs_dia = NULL; }
        if (fs_odo) { fclose(fs_odo); fs_odo = NULL; }
	}

}