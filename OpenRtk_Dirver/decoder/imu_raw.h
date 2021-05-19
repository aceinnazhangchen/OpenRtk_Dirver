#pragma once
#include <stdint.h>
#include "openrtk_user.h"

#ifdef __cplusplus
extern "C"
{
#endif
	extern void set_base_imu_file_name(char* file_name);
	extern void close_imu_all_log_file();

	extern int input_imu_raw(uint8_t c, char* out_msg);

	extern user_s1_t* getImuPak();

#ifdef __cplusplus
}
#endif