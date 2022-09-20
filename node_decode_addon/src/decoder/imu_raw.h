#pragma once
#include <stdint.h>
#include "common.h"
#ifdef __cplusplus
extern "C"
{
#endif
	extern void set_base_imu_file_name(char* file_name);
	extern void close_imu_all_log_file();

	extern int input_imu_raw(uint8_t c, char* out_msg);

	extern imu_t* getImuPak();

#ifdef __cplusplus
}
#endif