#include <stdio.h>
#include <string.h>
#include "decode_interface.h"

void decode_openrtk_user(char* filename)
{
	decode_openrtk330li_interface(filename);
}

void decode_openrtk_inceptio(char* filename)
{
	decode_rtk330la_interface(filename);
}

void decode_ins401(char* filename)
{
	char is_parse_dr[] = "false";
	decode_ins401_interface(filename, is_parse_dr);
}

int main(int argc, char* argv[]) {
	if (argc > 2) {
		char* filename = argv[2];
		if (strcmp(argv[1], "-u") == 0) {
			decode_openrtk_user(filename);
		}
		else if (strcmp(argv[1], "-i") == 0) {
			decode_openrtk_inceptio(filename);
		}
		else if (strcmp(argv[1], "-ins401") == 0) {
			decode_ins401(filename);
		}
		else if (strcmp(argv[1], "-beidou") == 0) {
			decode_beidou_interface(filename);
		}
		else if (strcmp(argv[1], "-npos122") == 0) {
			decode_npos122_interface(filename);
		}
		else {
			printf("User_Decoder_Console.exe -u(OpenRTK330LI)|i(RTK330LA)|ins401|beidou <filename> \n");
		}
	}
	else {
		printf("User_Decoder_Console.exe -u(OpenRTK330LI)|i(RTK330LA)|ins401|beidou <filename> \n");
	}
	return 0;
}