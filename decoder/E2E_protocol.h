#pragma once
#include <stdint.h>
#include <stdio.h>

namespace E2E {

#pragma pack(push, 1)
	struct e2e_raw {
		uint8_t flag;
		uint8_t header_len;
		uint8_t header[16];
		uint32_t length;
		uint8_t buff[1280];
	};

	struct e2e_header {
		uint16_t length;
		uint16_t counter;
		uint32_t data_id;
		uint32_t crc;
	};
#pragma pack(pop)

	class E2E_protocol
	{
	public:
		E2E_protocol();
		~E2E_protocol();
	protected:
		void close_all_files();
		void input_Preamble(uint8_t data);
		void create_file(FILE *& file, const char * suffix);
		void write_header_log();
		void write_rtcm();
	private:
		char base_file_name[256];
		e2e_raw m_e2e;
		e2e_header m_header;
		FILE* m_f_rtcm;
		FILE* m_f_log;
	public:
		void init();
		void set_base_file_name(char* file_name);
		void input_data(uint8_t data);
		void finish();
	};
}

