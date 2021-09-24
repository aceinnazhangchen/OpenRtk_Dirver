#include "ins_save_parse.h"
#include <stdint.h>
#include <string.h>

#ifndef CRC32_POLYNOMIAL
#define CRC32_POLYNOMIAL 0xEDB88320L
#endif // !CRC32_POLYNOMIAL

#pragma pack(1)
typedef struct  SaveConfig
{
	int16_t gnss_week;
	int32_t gnss_second;
	int8_t solution_type;
	int8_t position_type;
	double latitude;
	double longitude;
	float height;
	int16_t north_velocity;
	int16_t east_velocity;
	int16_t down_velocity;
	int16_t roll;
	int16_t pitch;
	int16_t azimuth;
	int16_t latitude_std;
	int16_t longitude_std;
	int16_t altitude_std;
	int16_t north_velocity_std;
	int16_t east_velocity_std;
	int16_t down_velocity_std;
	int16_t roll_std;
	int16_t pitch_std;
	int16_t azimuth_std;
	int16_t gyro_bias_x;
	int16_t gyro_bias_y;
	int16_t gyro_bias_z;
	int16_t acc_bias_x;
	int16_t acc_bias_y;
	int16_t acc_bias_z;
	int16_t std_gyro_bias_x;
	int16_t std_gyro_bias_y;
	int16_t std_gyro_bias_z;
	int16_t std_acc_bias_x;
	int16_t std_acc_bias_y;
	int16_t std_acc_bias_z;
	int8_t static_type;
	double reserve1;
	double reserve2;
}SaveConfig;

typedef struct SaveMsg
{
	uint8_t sync1;            //!< start of packet first byte (0xAA)
	uint8_t sync2;            //!< start of packet second byte (0x44)
	uint8_t sync3;            //!< start of packet third  byte (0x12)
	uint16_t message_length;  //!< Message length (Not including header or CRC)
	uint16_t message_id;      //!< Message ID number
	SaveConfig saveConfig;
	uint8_t crc[4];                           //!< 32-bit cyclic redundancy check (CRC)
}SaveMsg;
#pragma pack()


uint8_t ins_save_data_head[3] = {0xaa,0x44,0x12};
uint8_t frame_data[512];
uint8_t crc_rev[4];
SaveMsg* ins_save_data;
static char ins_save_str[512];

static unsigned long CRC32Value(int i)
{
	int j;
	unsigned long ulCRC;
	ulCRC = i;
	for (j = 8; j > 0; j--)
	{
		if (ulCRC & 1)
			ulCRC = (ulCRC >> 1) ^ CRC32_POLYNOMIAL;
		else
			ulCRC >>= 1;
	}
	return ulCRC;
}

static unsigned long CalculateBlockCRC32(unsigned long ulCount,   /* Number of bytes in the data block */
	unsigned char *ucBuffer) /* Data block */
{
	unsigned long ulTemp1;
	unsigned long ulTemp2;
	unsigned long ulCRC = 0;
	while (ulCount-- != 0)
	{
		ulTemp1 = (ulCRC >> 8) & 0x00FFFFFFL;
		ulTemp2 = CRC32Value(((int)ulCRC ^ *ucBuffer++) & 0xff);
		ulCRC = ulTemp1 ^ ulTemp2;
	}
	return (ulCRC);
}


int printasciisavebuf(SaveConfig* msaveconfig, char* buff)
{
	double dms1[3], dms2[3], amag = 0.0;
	char *p = buff, *q, sum, *emag = "E";

	//%02.0f%02.0f%05.2f,A,%02.0f%010.7f,%s,%03.0f%010.7f,%s,%4.2f,%4.2f,%02.0f%02.0f%02d,%.1f,%s,%s
	p+= sprintf(p, "$developer,%4d,%7d,%d,%d,%14.10f,%14.10f,%9.4f,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%10.5f,%10.5f",
		msaveconfig->gnss_week, msaveconfig->gnss_second, msaveconfig->solution_type, msaveconfig->position_type,
		msaveconfig->latitude, msaveconfig->longitude, msaveconfig->height,
		msaveconfig->north_velocity, msaveconfig->east_velocity, msaveconfig->down_velocity,
		msaveconfig->roll, msaveconfig->pitch, msaveconfig->azimuth,
		msaveconfig->longitude_std, msaveconfig->longitude_std, msaveconfig->altitude_std,
		msaveconfig->north_velocity_std, msaveconfig->east_velocity_std, msaveconfig->down_velocity_std,
		msaveconfig->roll_std, msaveconfig->pitch_std, msaveconfig->azimuth_std,
		msaveconfig->gyro_bias_x, msaveconfig->gyro_bias_y, msaveconfig->gyro_bias_z,
		msaveconfig->acc_bias_x, msaveconfig->acc_bias_y, msaveconfig->acc_bias_z,
		msaveconfig->std_gyro_bias_x, msaveconfig->std_gyro_bias_y, msaveconfig->std_gyro_bias_z,
		msaveconfig->std_acc_bias_x, msaveconfig->std_acc_bias_y, msaveconfig->std_acc_bias_z,
		msaveconfig->static_type, msaveconfig->reserve1, msaveconfig->reserve2);
	for (q = (char *)buff + 1, sum = 0; *q; q++)
		sum ^= *q; /* check-sum */

	p += sprintf(p, "*%02X%c%c", sum, 0x0D, 0x0A);
	return p - (char *)buff;
}

static int input_ins_save_data(unsigned char data)
{
    static uint8_t frame_rev_flag = 0;
    static uint16_t frame_data_len = 0;
    static uint16_t data_rev_index = 0;
    static uint16_t crc_rev_index = 0;
    static uint8_t time_cnt = 0;
    if (frame_rev_flag == 0)
    {
        if (data == ins_save_data_head[0])
        {
            frame_rev_flag = 1;
            frame_data_len = 0;
            data_rev_index = 0;
            crc_rev_index = 0;
            frame_data[data_rev_index++] = data;
            return 0;
        }
    }
    if(frame_rev_flag)
    {
        frame_data[data_rev_index++] = data;
    }
    switch (frame_rev_flag)
    {
        case 1:
            if (data == ins_save_data_head[1])
            {
                frame_rev_flag = 2;
            }
            else
            {
                frame_rev_flag = 0;
            }
            break;
        case 2:
            if (data == ins_save_data_head[2])
            {
                frame_rev_flag = 3;
            }
            else
            {
                frame_rev_flag = 0;
            }
            break;
        case 3:
            frame_data_len = data;
            frame_rev_flag = 4;
            break;
        case 4:
            frame_data_len = ((uint16_t)data << 8) + frame_data_len;
            if(frame_data_len > 156)
            {
                frame_rev_flag = 0;
                frame_data_len = 0;
                data_rev_index = 0;
                crc_rev[0] = 0;
                crc_rev[1] = 0;
                crc_rev_index = 0;
            }
            else
            {
                frame_rev_flag = 5;
            }
            break;
        case 5:
            // frame_data[data_rev_index++] = data;
            if (data_rev_index == frame_data_len)
            {
                frame_rev_flag = 6;
            }
            break;
        case 6:
            crc_rev[crc_rev_index++] = data;
            if (crc_rev_index == 4)
            {
                uint32_t crc_check = CalculateBlockCRC32(frame_data_len,frame_data);
                if (crc_check == (crc_rev[3] << 24 | crc_rev[2] << 16 | crc_rev[1] << 8 | crc_rev[0] << 0 ) )
                {
                    ins_save_data = (SaveMsg *)(frame_data);
                    printf("get ins save data\r\n");
                    printasciisavebuf(&ins_save_data->saveConfig,ins_save_str);
                }
                frame_rev_flag = 0;
                frame_data_len = 0;
                data_rev_index = 0;
                crc_rev[0] = 0;
                crc_rev[1] = 0;
                crc_rev_index = 0;
            }
            break;
    }
}


char* parse_ins_save_data(char *buff, int length)
{
    int pos = 0;
    while (length)
    {
        length--;
        int ret = input_ins_save_data(buff[pos++]);
    }
    return ins_save_str;
}

static int getFileSize(FILE * file)
{
	fseek(file, 0L, SEEK_END);
	int file_size = ftell(file);
	fseek(file, 0L, SEEK_SET);
	return file_size;
}

#if 0
int main(int argc, char*argv[])
{
    uint8_t* parse_str;
	char filename[100];
    if(argc < 2)
    {
        printf("please select file to parse\r\n");
        exit(-1);
    }
	memcpy(filename,argv[1],strlen(argv[1]) + 1);
	FILE* file = fopen(filename, "rb");
	if (file) {
		int ret = 0;
		int file_size = getFileSize(file);
		int read_size = 0;
		int readcount = 0;
		char read_cache[1024] = { 0 };
		char dirname[256] = { 0 };
		if(strstr(filename, "ins_save") != NULL)
		{
			while (!feof(file)) {
				readcount = fread(read_cache, sizeof(char), 1024, file);
				read_size += readcount;          
				parse_str = parse_ins_save_data(read_cache, readcount);
				double percent = (double)read_size / (double)file_size * 100;
                printf("%s\r\n",parse_str);
				printf("Process : %4.1f %%\r", percent);
			}
		}
		fclose(file);
		printf("\nfinished\r\n");
	}
}
#endif


