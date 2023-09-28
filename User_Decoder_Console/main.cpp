#include <stdio.h>
#include <string.h>
#include "getopt.h"
#include "decode_interface.h"

void decode_openrtk_user(char* filename)
{
	decode_openrtk330li_interface(filename);
}

void decode_rtk330la(char* filename,bool pruned)
{
	decode_rtk330la_interface(filename,pruned);
}

void decode_rtk350la(char* filename,bool pruned)
{
	decode_rtk350la_interface(filename, pruned);
}

void decode_ins401(char* filename, bool pruned)
{
	char is_parse_dr[] = "false";
	decode_ins401_interface(filename, is_parse_dr, pruned);
}

void decode_ins401c(char* filename)
{
	decode_ins401c_interface(filename);
}

void print_help() {
	printf("User_Decoder_Console.exe -t [filetype] -f [filename] -p(pruned) \n");
	printf("-t <filetype>\tSelect [openrtk|rtk330la|ins401|beidou|npos122|ins401c] \n");
	printf("-f <filename>\tPath of the file to be decoded.\n");
	printf("-p <pruned>\tCut out all the files that do not need.\n");
}

int main(int argc, char* argv[]) {
	if (argc == 3) {
		char* filename = argv[2];
		if (strcmp(argv[1], "-u") == 0) {
			decode_openrtk_user(filename);
		}
		else if (strcmp(argv[1], "-i") == 0) {
			decode_rtk330la(filename, false);
		}
		else if (strcmp(argv[1], "-ins401") == 0) {
			decode_ins401(filename, false);
		}
		else if (strcmp(argv[1], "-beidou") == 0) {
			decode_beidou_interface(filename);
		}
		else if (strcmp(argv[1], "-npos122") == 0) {
			decode_npos122_interface(filename);
		}
		else if (strcmp(argv[1], "-ins401c") == 0) {
			decode_ins401c(filename);
		}
		else if (strcmp(argv[1], "-RTK350LA") == 0) {
			decode_rtk350la(filename, false);
		}        
		else {
			printf("User_Decoder_Console.exe -u(OpenRTK330LI)|i(RTK330LA)|ins401|ins401c|beidou|npos122 <filename> \n");
		}
	}
	else if (argc > 3) {
		char ch;
		char* filepath = NULL;
		char* filetype = NULL;
		bool is_pruned = false;
		while ((ch = getopt(argc, argv, "hpt:f:")) != EOF) {
			switch (ch)
			{
			case 'h':
				print_help();
			case 'p':
				is_pruned = true;
				//puts("enter p");
				break;
			case 't':
				//puts("enter t");
				filetype = optarg;
				printf("file type is %s\n", optarg);
				break;
			case 'f':
				//puts("enter f");
				filepath = optarg;
				printf("file path is %s\n", optarg);
				break;
			default:
				return 1;
			}
		}

		if (filepath == NULL || filetype == NULL) return 0;

		if (strcmp(filetype, "openrtk") == 0) {
			decode_openrtk_user(filepath);
		}
		else if (strcmp(filetype, "rtk330la") == 0) {
			decode_rtk330la(filepath, is_pruned);
		}
		else if (strcmp(filetype, "ins401") == 0) {
			decode_ins401(filepath, is_pruned);
		}
		else if (strcmp(filetype, "beidou") == 0) {
			decode_beidou_interface(filepath);
		}
		else if (strcmp(filetype, "npos122") == 0) {
			decode_npos122_interface(filepath);
		}
		else if (strcmp(filetype, "ins401c") == 0) {
			decode_ins401c(filepath);
		}

	}
	else {
		print_help();
	}
	return 0;
}