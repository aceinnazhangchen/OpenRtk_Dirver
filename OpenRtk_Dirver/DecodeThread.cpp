#include "DecodeThread.h"
#include <QDir>
#include <math.h>
#include "common.h"
#include "openrtk_user.h"
#include "openrtk_inceptio.h"
#include "mixed_raw.h"
#include "imu_raw.h"
#include "rtcm.h"

DecodeThread::DecodeThread(QObject *parent)
	: QThread(parent)
	, m_isStop(false)
	, m_show_time(false)
	, m_FileFormat(emDecodeFormat_openrtk_user)
	, ins_kml_frequency(1000)
{
	m_static_point_ecp = true;
	m_Ins401_Analysis = new Ins401_Analysis(this);
	ins401_decoder = new Ins401_Tool::Ins401_decoder();
	e2e_deocder = new E2E::E2E_protocol();
}

DecodeThread::~DecodeThread()
{
	if (ins401_decoder) {delete ins401_decoder; ins401_decoder = NULL;}
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
		RTK330LA_Tool::set_output_inceptio_file(1);
		RTK330LA_Tool::set_base_inceptio_file_name(m_OutBaseName.toLocal8Bit().data());
		Kml_Generator::Instance()->set_kml_frequency(ins_kml_frequency);
		while (!feof(file)) {
			if (m_isStop) break;
			readcount = fread(read_cache, sizeof(char), READ_CACHE_SIZE, file);
			read_size += readcount;
			for (int i = 0; i < readcount; i++) {
				ret = RTK330LA_Tool::input_inceptio_raw(read_cache[i]);
			}
			double percent = (double)read_size / (double)file_size * 10000;
			emit sgnProgress((int)percent, m_TimeCounter.elapsed());
		}
		RTK330LA_Tool::write_inceptio_kml_files();
		RTK330LA_Tool::close_inceptio_all_log_file();
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
		int64_t file_size = getFileSize(file);
		int64_t read_size = 0;
		int readcount = 0;
		char read_cache[READ_CACHE_SIZE] = { 0 };
		ins401_decoder->init();
		ins401_decoder->set_show_format_time(m_show_time);
		ins401_decoder->set_base_file_name(m_OutBaseName.toLocal8Bit().data());
		Kml_Generator::Instance()->set_kml_frequency(ins_kml_frequency);
		m_Ins401_Analysis->init();
		m_Ins401_Analysis->set_out_base_name(m_OutBaseName);
		while (!feof(file)) {
			if (m_isStop) break;
			readcount = fread(read_cache, sizeof(char), READ_CACHE_SIZE, file);
			read_size += readcount;
			for (int i = 0; i < readcount; i++) {
				ret = ins401_decoder->input_data(read_cache[i]);
				if (m_static_point_ecp && ret == 1) {
					if (Ins401_Tool::em_GNSS_SOL == ins401_decoder->get_current_type()) {
						m_Ins401_Analysis->append_gnss_sol(ins401_decoder->get_gnss_sol());
					}
				}
			}
			double percent = (double)read_size / (double)file_size * 10000;
			emit sgnProgress((int)percent, m_TimeCounter.elapsed());
		}
		ins401_decoder->finish();
		m_Ins401_Analysis->static_point_cep();
		fclose(file);
	}
}

void DecodeThread::decode_e2e_protocol() {
	FILE* file = fopen(m_FileName.toLocal8Bit().data(), "rb");
	if (file) {
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

void DecodeThread::decode_Rtcm_EPVT() {
	FILE* file = fopen(m_FileName.toLocal8Bit().data(), "rb");
	if (file) {
		int64_t file_size = getFileSize(file);
		int64_t read_size = 0;
		int readcount = 0;
		char read_cache[READ_CACHE_SIZE] = { 0 };
		gnss_rtcm_t rtcm = { 0 };
		rtcm_t* rcv0 = rtcm.rcv;
		type_999_t* teseo = &rcv0->teseo;
		double blh[3] = { 0.0 };
		double last_epvt_time = 0.0;
		while (!feof(file)) {
			if (m_isStop) break;
			readcount = fread(read_cache, sizeof(char), READ_CACHE_SIZE, file);
			read_size += readcount;
			for (int i = 0; i < readcount; i++) {
				//e2e_deocder->input_data(read_cache[i]);
				int ret = input_rtcm3(read_cache[i],0, &rtcm);
				if (ret != 1) {
					continue;
				}
				/*total_obs_num(&rtcm.obs[0], 0);*/
				//if (norm(rtcm.obs[0].pos, 3) < 0.01)
				//{
				//	continue;
				//}
				//ecef2pos(teseo->pos, blh);
				//fprintf(fCSV, "%d,%11.4f,%3d,%14.9f,%14.9f,%10.4f,%3d,%5.1f,%5.1f,%5.1f,%5.1f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f\n",
				//	week, teseo->time - week * SECONDS_IN_WEEK, 1, blh[0] * R2D, blh[1] * R2D, blh[2],
				//	teseo->nsat_use, (float)teseo->hdop*0.1, (float)teseo->vdop*0.1, (float)teseo->pdop*0.1, teseo->age,
				//	teseo->vel_enu[1], teseo->vel_enu[0], teseo->vel_enu[2], teseo->std_neu[0], teseo->std_neu[1], teseo->std_neu[2]);

				last_epvt_time = teseo->time;
			}
			double percent = (double)read_size / (double)file_size * 10000;
			emit sgnProgress((int)percent, m_TimeCounter.elapsed());
		}
		fclose(file);
	}
}