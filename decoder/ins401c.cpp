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
	static char ins401c_output_msg[1024] = { 0 };
    static char ins401c_output_msg_imu[1024] = { 0 };
    static char ins401c_output_msg_ins[1024] = { 0 };
    static const dbc_msg_hdr_t list_dbc_msgs[] = {
        {0x180, 53,           dbc_decode_INSPVAX},
    };
	static char base_ins401c_file_name[256] = { 0 };
	void set_base_ins401c_file_name(char* file_name)
	{
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
        to->INS_PitchAngle = ((raw * 0.010986) + (-250));
        raw  = ((uint32_t)((bytes[14]))) << 8;    //< 8 bit(s) from B119
        raw |= ((uint32_t)((bytes[15])));    //< 8 bit(s) from B127
        to->INS_RollAngle = ((raw * 0.010986) + (-250));
        raw  = ((uint32_t)((bytes[16]))) << 8;    //< 8 bit(s) from B135
        raw |= ((uint32_t)((bytes[17])));    //< 8 bit(s) from B143
        to->INS_HeadingAngle = ((raw * 0.010986) + (-250));
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
        to->INS_NorthSpd = ((raw * 0.0030517) + (-100));
        raw  = ((uint32_t)((bytes[36]))) << 8;    //< 8 bit(s) from B263
        raw |= ((uint32_t)((bytes[37])));    //< 8 bit(s) from B271
        to->INS_EastSpd = ((raw * 0.0030517) + (-100));
        raw  = ((uint32_t)((bytes[38]))) << 8;    //< 8 bit(s) from B279
        raw |= ((uint32_t)((bytes[39])));    //< 8 bit(s) from B287
        to->INS_ToGroundSpd = ((raw * 0.0030517) + (-100));
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
        to->INS_Status = ((raw * 0.001));
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
        // printf("timeofweek = %d, %f\r\n", to->TimeOfWeek, (double)(to->TimeOfWeek)/1000);
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
        printf("INS_Time:%d\n", to->INS_Time);
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
		sprintf(ins401c_output_msg, "%11.4f,%11.4f,%11.4f,%11.4f,%11.4f,%11.4f,%11.4f,%11.4f,%11.4f,%d,%d,%11.7f,%11.7f,%11.4f,%11.4f,%11.4f,%d,%d,%d,%d,%d,%d,%11.4f,%11.4f,%11.4f,%11.4f\n",to->ACC_X, to->ACC_Y, to->ACC_Z,\
        to->GYRO_X, to->GYRO_Y, to->GYRO_Z,\
        to->INS_PitchAngle, to->INS_RollAngle, to->INS_HeadingAngle,\
        to->INS_LocatHeight, to->IMU_Status,\
        to->INS_Latitude, to->INS_Longitude,\
        to->INS_NorthSpd, to->INS_EastSpd, to->INS_ToGroundSpd,\
        to->INS_GpsFlag_Pos, to->INS_NumSV, to->INS_GpsFlag_Heading, to->INS_Gps_Age, to->INS_Car_Status, to->INS_Status,\
        to->INS_Std_Lat, to->INS_Std_Lon, to->INS_Std_LocatHeight, to->INS_Std_Heading, \
        to->Week, (double)(to->TimeOfWeek) / 1000\
        );
        sprintf(ins401c_output_msg_imu, "%d,%11.4f,%11.4f,%11.4f,%11.4f,%11.4f,%11.4f,%11.4f\n",to->Week,\
        (double)(to->TimeOfWeek)/1000,\
        to->ACC_X, to->ACC_Y, to->ACC_Z,\
        to->GYRO_X, to->GYRO_Y, to->GYRO_Z,\
        to->IMU_Status\
        );

        sprintf(ins401c_output_msg_ins, "%d,%11.4f,%d,%d,%11.7f,%11.7f,%11.7f,%11.4f,%11.4f,%11.4f,%11.4f,%11.4f,%11.4f,%11.4f,%11.4f,%11.4f\n",
        to->Week, (double)(to->TimeOfWeek)/1000,\
        to->INS_Car_Status, to->INS_Status,\
        to->INS_Latitude, to->INS_Longitude, to->INS_LocatHeight,\
        to->INS_NorthSpd, to->INS_EastSpd, to->INS_ToGroundSpd,\
        to->INS_RollAngle, to->INS_PitchAngle, to->INS_HeadingAngle,\
        to->INS_Std_Lat, to->INS_Std_Lon, to->INS_Std_LocatHeight\
        );

        write_ins401c_log_file(ins401c_output_msg);
        write_ins401c_imu_file(ins401c_output_msg_imu);
        write_ins401c_ins_file(ins401c_output_msg_ins);
        return success;
    }

    bool dbc_msg_decode(uint32_t mid, uint8_t *from, uint8_t *to)
    {
        if (NULL == from || NULL == to)
            return false;
        for (int i = 0; i < sizeof(list_dbc_msgs); i++) {
            if (mid == list_dbc_msgs[i].mid)
            {
                list_dbc_msgs[i].func(to, from);
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
        char* str = strtok((char*)data, " ");
        char* endptr = NULL;
        while (str != NULL)
        {
            count++; 
            if(count == 4)
            {
                mid = strtol(str, &endptr, 16);
            }
            if( (count >= 10) && (count <= 76) )
            {
                valid_data[count-10] = strtol(str, &endptr, 16);
                // printf("valid_data = %d\r\n", valid_data[count-10]);
            }
            str = strtok(NULL, " ");

        }
        dbc_msg_decode(mid, valid_data, result);
        count = 0;
        return 0;
    }
	void write_ins401c_log_file(char* log) {
		if (strlen(base_ins401c_file_name) == 0) return;
		char file_name[256] = { 0 };
        if (fs_canfd == NULL) {
            sprintf(file_name, "%s_INSPVA.csv", base_ins401c_file_name);
            fs_canfd = fopen(file_name, "w");
            if (fs_canfd) fprintf(fs_canfd, "ACC_X(s),ACC_Y(s),ACC_Z(s),GYRO_X(s),GYRO_Y(s),GYRO_Z(s),INS_PitchAngle(s),INS_RollAngle(s),INS_HeadingAngle(s),INS_LocatHeight(s),INS_Time(s),INS_Latitude(s),INS_Longitude(s),INS_NorthSpd(s),INS_EastSpd(s),INS_ToGroundSpd(s),INS_GpsFlag_Pos(s),INS_NumSV(s),INS_GpsFlag_Heading(s),INS_Gps_Age(s),INS_Car_Status(s),INS_Status(s),INS_Std_Lat(s),INS_Std_Lon(s),INS_Std_LocatHeight(s),INS_Std_Heading(s),Week(s),TimeOfWeek(s)\n");
        }
        if (fs_canfd) fprintf(fs_canfd, log);
	}
	void write_ins401c_imu_file(char* log) {
		if (strlen(base_ins401c_file_name) == 0) return;
		char file_name[256] = { 0 };
        if (fs_imu == NULL) {
            sprintf(file_name, "%s_imu.csv", base_ins401c_file_name);
            fs_imu = fopen(file_name, "w");
            if (fs_imu) fprintf(fs_imu, "GPS_Week(),GPS_TimeofWeek(s),x_accel(m/s^2),y_accel(m/s^2),z_accel(m/s^2),x_rate(deg/s),y_rate(deg/s),z_rate(deg/s)\n");
        }
        if (fs_imu) fprintf(fs_imu, log);
	}
	void write_ins401c_ins_file(char* log) {
		if (strlen(base_ins401c_file_name) == 0) return;
		char file_name[256] = { 0 };
        if (fs_ins == NULL) {
            sprintf(file_name, "%s_ins.csv", base_ins401c_file_name);
            fs_ins = fopen(file_name, "w");
            if (fs_ins) fprintf(fs_ins, "GPS_Week(),GPS_TimeofWeek(s),insCarStatus(),insPositionType(),latitude(deg),longitude(deg),height(m),velocityNorth(m/s),velocityEast(m/s),velocityUp(m/s),roll(deg),pitch(deg),heading(deg),latitude_std(m),longitude_std(m),height_std(m)\n");
        }
        if (fs_ins) fprintf(fs_ins, log);
	}
}