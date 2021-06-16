#include "DecodeThread.h"
#include "StreamManager.h"
#include "decoder\openrtk_user.h"
#include "decoder\openrtk_inceptio.h"
#include "decoder\mixed_raw.h"
#include "decoder\imu_raw.h"
#include <QList>

#define READ_CACHE_SIZE 4*1024

DecodeThread::DecodeThread(QObject *parent)
	: QThread(parent)
	, m_isStop(false)
	, m_FileFormat(emDecodeFormat_openrtk_user)
{
}

DecodeThread::~DecodeThread()
{
}

void DecodeThread::run()
{
	m_isStop = false;
	if (!m_FileName.isEmpty()) {
		m_TimeCounter.start();
		makeOutPath(m_FileName);
		switch (m_FileFormat)
		{
		case emDecodeFormat_openrtk_user:
			decode_openrtk_user();
			break;
		case emDecodeFormat_openrtk_inceptio:
			decode_openrtk_inceptio();
			break;
		case emDecodeFormat_mixed_raw:
			decode_mixed_raw();
			break;
		case emDecodeFormat_imu:
			decode_imu();
			break;
		default:
			break;
		}
	}
	emit sgnFinished();
}

void DecodeThread::stop()
{
	m_isStop = true;
}

void DecodeThread::setFileFormat(int format)
{
	m_FileFormat = format;
}

void DecodeThread::setFileName(QString file)
{
	m_FileName = file;
}

void DecodeThread::makeOutPath(QString filename)
{
	QFileInfo outDir(filename);
	if (outDir.isFile()) {
		QString basename = outDir.baseName();
		QString absoluteDir = outDir.absoluteDir().absolutePath();
		QDir outPath = absoluteDir + QDir::separator() + basename + "_d";
		if (!outPath.exists()) {
			outPath.mkpath(outPath.absolutePath());
		}
		m_OutBaseName = outPath.absolutePath() + QDir::separator() + basename;
	}
}

void DecodeThread::decode_openrtk_user()
{
	FILE* file = fopen(m_FileName.toLocal8Bit().data(), "rb");
	if (file) {
		int ret = 0;
		int file_size = getFileSize(file);
		int read_size = 0;
		int readcount = 0;
		char read_cache[READ_CACHE_SIZE] = { 0 };
		QList<user_g1_t> gnss_list;
		QList<user_i1_t> ins_list;
		set_output_user_file(1);
		set_save_bin(1);
		set_base_user_file_name(m_OutBaseName.toLocal8Bit().data());
		while (!feof(file)) {
			if (m_isStop) break;
			readcount = fread(read_cache, sizeof(char), READ_CACHE_SIZE, file);
			read_size += readcount;
			for (int i = 0; i < readcount; i++) {
				ret = input_user_raw(read_cache[i]);
				if (ret == 1) {
					if (get_user_packet_type() == USR_OUT_BESTGNSS) {
						user_g1_t* g1 = get_user_packet_g1();
						gnss_list << *g1;
					}
					else if (get_user_packet_type() == USR_OUT_INSPVAX) {
						user_i1_t* i1 = get_user_packet_i1();
						ins_list << *i1;
					}
				}
			}
			double percent = (double)read_size / (double)file_size * 10000;
			emit sgnProgress((int)percent, m_TimeCounter.elapsed());
		}
		//gnss kml
		for (int i = 0; i < gnss_list.size(); ++i){
			if (i == 0) {
				write_gnss_kml_line(&gnss_list[i], 1);
			}else if (i == gnss_list.size() - 1) {
				write_gnss_kml_line(&gnss_list[i], -1);
			}else {
				write_gnss_kml_line(&gnss_list[i], 0);
			}			
		}
		for (int i = 0; i < gnss_list.size(); ++i){
			if (i == 0) {
				write_gnss_kml_file(&gnss_list[i], 1);
			}else if (i == gnss_list.size() - 1) {
				write_gnss_kml_file(&gnss_list[i], -1);
			}else {
				write_gnss_kml_file(&gnss_list[i], 0);
			}
		}
		//ins kml
		for (int i = 0; i < ins_list.size(); ++i){
			if (i == 0) {
				write_ins_kml_line(&ins_list[i], 1);
			}else if (i == ins_list.size() - 1) {
				write_ins_kml_line(&ins_list[i], -1);
			}else {
				write_ins_kml_line(&ins_list[i], 0);
			}
		}
		for (int i = 0; i < ins_list.size(); ++i){
			if (i == 0) {
				write_ins_kml_file(&ins_list[i], 1);
			}else if (i == ins_list.size() - 1) {
				write_ins_kml_file(&ins_list[i], -1);
			}else {
				write_ins_kml_file(&ins_list[i], 0);
			}
		}
		gnss_list.clear();
		ins_list.clear();
		close_user_all_log_file();
		fclose(file);
	}
}

void DecodeThread::decode_openrtk_inceptio()
{
	FILE* file = fopen(m_FileName.toLocal8Bit().data(), "rb");
	if (file) {
		int ret = 0;
		int file_size = getFileSize(file);
		int read_size = 0;
		int readcount = 0;
		char read_cache[READ_CACHE_SIZE] = { 0 };
		QList<inceptio_gN_t> gnss_list;
		QList<inceptio_iN_t> ins_list;
		set_output_inceptio_file(1);
		set_base_inceptio_file_name(m_OutBaseName.toLocal8Bit().data());
		while (!feof(file)) {
			if (m_isStop) break;
			readcount = fread(read_cache, sizeof(char), READ_CACHE_SIZE, file);
			read_size += readcount;
			for (int i = 0; i < readcount; i++) {
				ret = input_inceptio_raw(read_cache[i]);
				if (ret == 1) {
					if (get_inceptio_packet_type() == INCEPTIO_OUT_GNSS) {
						inceptio_gN_t* g1 = get_inceptio_packet_gN();
						gnss_list << *g1;
					}
					else if (get_inceptio_packet_type() == INCEPTIO_OUT_INSPVA) {
						inceptio_iN_t* i1 = get_inceptio_packet_iN();
						ins_list << *i1;
					}
				}
			}
			double percent = (double)read_size / (double)file_size * 10000;
			emit sgnProgress((int)percent, m_TimeCounter.elapsed());
		}
		//gnss kml
		for (int i = 0; i < gnss_list.size(); ++i) {
			if (i == 0) {
				write_inceptio_gnss_kml_line(&gnss_list[i], 1);
			}
			else if (i == gnss_list.size() - 1) {
				write_inceptio_gnss_kml_line(&gnss_list[i], -1);
			}
			else {
				write_inceptio_gnss_kml_line(&gnss_list[i], 0);
			}
		}
		for (int i = 0; i < gnss_list.size(); ++i) {
			if (i == 0) {
				write_inceptio_gnss_kml_file(&gnss_list[i], 1);
			}
			else if (i == gnss_list.size() - 1) {
				write_inceptio_gnss_kml_file(&gnss_list[i], -1);
			}
			else {
				write_inceptio_gnss_kml_file(&gnss_list[i], 0);
			}
		}
		//ins kml
		for (int i = 0; i < ins_list.size(); ++i) {
			if (i == 0) {
				write_inceptio_ins_kml_line(&ins_list[i], 1);
			}
			else if (i == ins_list.size() - 1) {
				write_inceptio_ins_kml_line(&ins_list[i], -1);
			}
			else {
				write_inceptio_ins_kml_line(&ins_list[i], 0);
			}
		}
		for (int i = 0; i < ins_list.size(); ++i) {
			if (i == 0) {
				write_inceptio_ins_kml_file(&ins_list[i], 1);
			}
			else if (i == ins_list.size() - 1) {
				write_inceptio_ins_kml_file(&ins_list[i], -1);
			}
			else {
				write_inceptio_ins_kml_file(&ins_list[i], 0);
			}
		}
		gnss_list.clear();
		ins_list.clear();
		close_inceptio_all_log_file();
		fclose(file);
	}
}

void DecodeThread::decode_mixed_raw()
{
	if (is_aceinna_decoding()) return;
	set_aceinna_decoding(1);
	FILE* file = fopen(m_FileName.toLocal8Bit().data(), "rb");
	if (file) {
		int file_size = getFileSize(file);
		int read_size = 0;
		int readcount = 0;
		char read_cache[READ_CACHE_SIZE] = { 0 };
		set_output_aceinna_file(1);
		set_aceinna_file_basename(m_OutBaseName.toLocal8Bit().data());
		open_aceinna_log_file();
		while (!feof(file)) {
			if (m_isStop) break;
			readcount = fread(read_cache, sizeof(char), READ_CACHE_SIZE, file);
			read_size += readcount;
			for (int i = 0; i < readcount; i++) {
				input_aceinna_format_raw(read_cache[i],NULL,NULL);
			}
			double percent = (double)read_size / (double)file_size * 10000;
			emit sgnProgress((int)percent, m_TimeCounter.elapsed());
		}
		close_aceinna_all_file();
		fclose(file);
	}
	set_aceinna_decoding(0);
}

void DecodeThread::decode_imu()
{
	FILE* file = fopen(m_FileName.toLocal8Bit().data(), "rb");
	if (file) {
		int file_size = getFileSize(file);
		int read_size = 0;
		int readcount = 0;
		char read_cache[READ_CACHE_SIZE] = { 0 };
		char out_msg[1024] = { 0 };
		set_base_imu_file_name(m_OutBaseName.toLocal8Bit().data());
		while (!feof(file)) {
			if (m_isStop) break;
			readcount = fread(read_cache, sizeof(char), READ_CACHE_SIZE, file);
			read_size += readcount;
			for (int i = 0; i < readcount; i++) {
				input_imu_raw(read_cache[i], out_msg);
			}
			double percent = (double)read_size / (double)file_size * 10000;
			emit sgnProgress((int)percent, m_TimeCounter.elapsed());
		}
		close_imu_all_log_file();
		fclose(file);
	}
}

int DecodeThread::getFileSize(FILE * file)
{
	fseek(file, 0L, SEEK_END);
	int file_size = ftell(file);
	fseek(file, 0L, SEEK_SET);
	return file_size;
}