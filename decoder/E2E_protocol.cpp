#include "E2E_protocol.h"
#include <string.h>

namespace E2E {

	const uint8_t E2E_Preamble[3] = { 0x0F,0xF0,0x5A };

	E2E_protocol::E2E_protocol()
	{
		m_f_rtcm = NULL;
		m_f_log = NULL;
		init();
	}

	E2E_protocol::~E2E_protocol()
	{
	}

	void E2E_protocol::close_all_files() {
		if (m_f_rtcm)fclose(m_f_rtcm); m_f_rtcm = NULL;
		if (m_f_log)fclose(m_f_log); m_f_log = NULL;
	}

	void E2E_protocol::input_Preamble(uint8_t data)
	{
		if (m_e2e.header_len == 0) {
			if (E2E_Preamble[0] == data) {
				m_e2e.header[m_e2e.header_len++] = data;
			}
			else {
				m_e2e.header_len = 0;
			}
		}
		else if (m_e2e.header_len == 1) {
			if (E2E_Preamble[1] == data) {
				m_e2e.header[m_e2e.header_len++] = data;
			}
			else {
				m_e2e.header_len = 0;
			}
		}
		else if (m_e2e.header_len == 2) {
			if (E2E_Preamble[2] == data) {
				m_e2e.header[m_e2e.header_len++] = data;
			}
			else {
				m_e2e.header_len = 0;
			}
		}
	}

	void E2E_protocol::create_file(FILE* &file, const char* suffix) {
		if (strlen(base_file_name) == 0) return;
		if (file == NULL) {
			char file_name[256] = { 0 };
			sprintf(file_name, "%s_%s", base_file_name, suffix);
			file = fopen(file_name, "wb");
		}
	}

	void E2E_protocol::write_header_log() {
		create_file(m_f_log, "header.log");
		fprintf(m_f_log, "%5d, %6d, 0x%08x, 0x%08x\n", m_header.length, m_header.counter, m_header.data_id, m_header.crc);
	}

	void E2E_protocol::write_rtcm()
	{
		create_file(m_f_rtcm, "rtcm.bin");
		fwrite(m_e2e.buff,1, m_e2e.length,m_f_rtcm);
	}

	void E2E_protocol::init()
	{
		memset(base_file_name, 0, 256);
		memset(&m_e2e, 0, sizeof(m_e2e));
		memset(&m_header, 0, sizeof(m_header));
	}

	void E2E_protocol::set_base_file_name(char * file_name)
	{
		strcpy(base_file_name, file_name);
	}

	void E2E_protocol::input_data(uint8_t data)
	{
		if (m_e2e.flag == 0) {
			if (m_e2e.header_len < 3) {
				input_Preamble(data);
			}
			else {
				m_e2e.header[m_e2e.header_len++] = data;
			}
			if (m_e2e.header_len == 16) {
				e2e_header header;
				memcpy(&header, &m_e2e.header[4], sizeof(e2e_header));
				m_header.length = (header.length << 8 & 0xFF00) | (header.length >> 8 & 0x00FF);
				m_header.counter = (header.counter << 8 & 0xFF00) | (header.counter >> 8 & 0x00FF);
				m_header.data_id = (header.data_id << 24 & 0xFF000000) | (header.data_id << 8 & 0x00FF0000) | (header.data_id >> 8 & 0x0000FF00) | (header.data_id >>24 & 0x000000FF);
				m_header.crc = (header.crc << 24 & 0xFF000000) | (header.crc << 8 & 0x00FF0000) | (header.crc >> 8 & 0x0000FF00) | (header.crc >> 24 & 0x000000FF);
				m_e2e.flag = 1;
				m_e2e.length = 0;
				write_header_log();
			}
		}
		else {
			if (m_e2e.length < m_header.length - sizeof(e2e_header)) {
				m_e2e.buff[m_e2e.length++] = data;
				if (m_e2e.length == m_header.length - sizeof(e2e_header)) {
					write_rtcm();
					m_e2e.flag = 0;
					m_e2e.header_len = 0;					
					memset(&m_header, 0, sizeof(m_header));
				}
			}
		}
	}

	void E2E_protocol::finish()
	{
		close_all_files();
	}

}