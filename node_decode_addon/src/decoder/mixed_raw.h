#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

#define MAX_BUFFER_SIZE 1024

#define ACEINNA_HEAD_SIZE 4
//#define IMU_CONST_SIZE 23
#define IMU_CONST_SIZE 30

#define TYPE_ROV 1
#define TYPE_BAS 2
#define TYPE_IMU 3

#define ROV_FLAG  "$ROV"
#define BAS_FLAG  "$BAS"
#define IMU_FLAG  "$IMU"

	extern void set_aceinna_decoding(int decoding);
	extern int is_aceinna_decoding();

	extern void set_output_aceinna_file(int output);
	extern void set_aceinna_file_basename(char* input_name);
	extern void open_aceinna_log_file();
	extern void close_aceinna_all_file();

	extern int input_aceinna_format_raw(uint8_t c, uint8_t* outbuff, uint32_t* outlen);

#ifdef __cplusplus
}
#endif