#include "DecodeThread.h"
#include "StreamManager.h"
#include "openrtk_user.h"
#include "openrtk_inceptio.h"
#include "mixed_raw.h"
#include "imu_raw.h"
#include <QList>

#define READ_CACHE_SIZE 4*1024

DecodeThread::DecodeThread(QObject *parent)
	: QThread(parent)
	, m_isStop(false)
	, m_show_time(false)
	, m_FileFormat(emDecodeFormat_openrtk_user)
	, ins_kml_frequency(1000)
{
	ins401_decoder = new Ins401_Tool::Ins401_decoder();
	rtcm_split = new Rtcm_Split();
	e2e_deocder = new E2E::E2E_protocol();
}

DecodeThread::~DecodeThread()
{
	if (ins401_decoder) {delete ins401_decoder; ins401_decoder = NULL;}
	if (rtcm_split) { delete rtcm_split; rtcm_split = NULL; }
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
		case emDecodeFormat_ins401:
			decode_ins401();
			break;
		case emDecodeFormat_RTCM_Split:
			split_rtcm();
			break;
		case emDecodeFormat_E2E_protocol:
			decode_e2e_protocol();
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

void DecodeThread::setShowTime(bool show)
{
	m_show_time = show;
}

void DecodeThread::setKmlFrequency(int frequency)
{
	ins_kml_frequency = frequency;
	Kml_Generator::Instance()->set_kml_frequency(ins_kml_frequency);
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
		set_output_user_file(1);
		set_save_bin(1);
		set_base_user_file_name(m_OutBaseName.toLocal8Bit().data());
		Kml_Generator::Instance()->set_kml_frequency(ins_kml_frequency);
		while (!feof(file)) {
			if (m_isStop) break;
			readcount = fread(read_cache, sizeof(char), READ_CACHE_SIZE, file);
			read_size += readcount;
			for (int i = 0; i < readcount; i++) {
				ret = input_user_raw(read_cache[i]);
			}
			double percent = (double)read_size / (double)file_size * 10000;
			emit sgnProgress((int)percent, m_TimeCounter.elapsed());
		}
		write_kml_files();
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
		set_output_inceptio_file(1);
		set_base_inceptio_file_name(m_OutBaseName.toLocal8Bit().data());
		Kml_Generator::Instance()->set_kml_frequency(ins_kml_frequency);
		while (!feof(file)) {
			if (m_isStop) break;
			readcount = fread(read_cache, sizeof(char), READ_CACHE_SIZE, file);
			read_size += readcount;
			for (int i = 0; i < readcount; i++) {
				ret = input_inceptio_raw(read_cache[i]);
			}
			double percent = (double)read_size / (double)file_size * 10000;
			emit sgnProgress((int)percent, m_TimeCounter.elapsed());
		}
		write_inceptio_kml_files();
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
		Kml_Generator::Instance()->set_kml_frequency(ins_kml_frequency);
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

void DecodeThread::decode_ins401()
{
	FILE* file = fopen(m_FileName.toLocal8Bit().data(), "rb");
	if (file && ins401_decoder) {
		int ret = 0;
		int file_size = getFileSize(file);
		int read_size = 0;
		int readcount = 0;
		char read_cache[READ_CACHE_SIZE] = { 0 };
		ins401_decoder->init();
		ins401_decoder->set_show_format_time(m_show_time);
		ins401_decoder->set_base_file_name(m_OutBaseName.toLocal8Bit().data());
		Kml_Generator::Instance()->set_kml_frequency(ins_kml_frequency);
		while (!feof(file)) {
			if (m_isStop) break;
			readcount = fread(read_cache, sizeof(char), READ_CACHE_SIZE, file);
			read_size += readcount;
			for (int i = 0; i < readcount; i++) {
				ret = ins401_decoder->input_data(read_cache[i]);
			}
			double percent = (double)read_size / (double)file_size * 10000;
			emit sgnProgress((int)percent, m_TimeCounter.elapsed());
		}
		ins401_decoder->finish();
		fclose(file);
	}
}

void DecodeThread::split_rtcm()
{
	FILE* file = fopen(m_FileName.toLocal8Bit().data(), "rb");
	if (file && rtcm_split) {
		int64_t file_size = getFileSize(file);
		int64_t read_size = 0;
		int readcount = 0;
		char read_cache[READ_CACHE_SIZE] = { 0 };
		rtcm_split->init();
		rtcm_split->set_base_file_name(m_OutBaseName.toLocal8Bit().data());
		while (!feof(file)) {
			if (m_isStop) break;
			readcount = fread(read_cache, sizeof(char), READ_CACHE_SIZE, file);
			read_size += readcount;
			for (int i = 0; i < readcount; i++) {
				rtcm_split->input_data(read_cache[i]);
			}
			double percent = (double)read_size / (double)file_size * 10000;
			emit sgnProgress((int)percent, m_TimeCounter.elapsed());
		}
		rtcm_split->close_files();
		fclose(file);
	}
}

void DecodeThread::decode_e2e_protocol() {
	FILE* file = fopen(m_FileName.toLocal8Bit().data(), "rb");
	if (file && rtcm_split) {
		int64_t file_size = getFileSize(file);
		int64_t read_size = 0;
		int readcount = 0;
		char read_cache[READ_CACHE_SIZE] = { 0 };
		e2e_deocder->init();
		e2e_deocder->set_base_file_name(m_OutBaseName.toLocal8Bit().data());
		while (!feof(file)) {
			if (m_isStop) break;
			readcount = fread(read_cache, sizeof(char), READ_CACHE_SIZE, file);
			read_size += readcount;
			for (int i = 0; i < readcount; i++) {
				e2e_deocder->input_data(read_cache[i]);
			}
			double percent = (double)read_size / (double)file_size * 10000;
			emit sgnProgress((int)percent, m_TimeCounter.elapsed());
		}
		e2e_deocder->finish();
		fclose(file);
	}
}

int64_t DecodeThread::getFileSize(FILE * file)
{
	int64_t file_size = 0;
#if defined(_WIN32) || defined(_WIN64)
#if _MSC_VER >= 1400
	_fseeki64(file, (int64_t)(0), SEEK_END);
	file_size = _ftelli64(file);
	_fseeki64(file, (int64_t)(0), SEEK_SET);
	return file_size;
#else
#error Visual Studio version is less than 8.0(VS 2005) !
#endif
#else
	fseeko(file, (int64_t)(0), SEEK_END);
	file_size = ftello(fp);
	fseeko(file, (int64_t)(0), SEEK_SET);
#endif
}