#include "decoder_dll.h"
#include "decode_interface.h"

USERDECODERLIB_API void decode_openrtk_user(char* filename)
{
	decode_openrtk330li_interface(filename);
}

USERDECODERLIB_API void decode_openrtk_inceptio(char* filename)
{
	decode_rtk330la_interface(filename);
}

USERDECODERLIB_API void decode_ins401(char* filename, char* is_parse_dr)
{
	decode_ins401_interface(filename, is_parse_dr);
}

USERDECODERLIB_API void decode_beidou(char* filename)
{
	decode_beidou_interface(filename);
}

USERDECODERLIB_API void decode_npos122(char* filename)
{
	decode_npos122_interface(filename);
}

