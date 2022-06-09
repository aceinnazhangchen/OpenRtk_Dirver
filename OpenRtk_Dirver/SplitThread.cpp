#include "SplitThread.h"
#include <QDir>
#include "common.h"

SplitThread::SplitThread(QObject *parent)
	: QThread(parent)
	, m_isStop(false)
	, m_isRepeatData(false)
	, m_FileFormat(emSplitFormat_RTCM)
{
	rtcm_split = new Rtcm_Split();
	ins401_decoder = new Ins401_Tool::Ins401_decoder();
}

SplitThread::~SplitThread()
{
	if (rtcm_split) { delete rtcm_split; rtcm_split = NULL; }
}

void SplitThread::run()
{
	m_isStop = false;
	if (!m_FileName.isEmpty()) {
		m_TimeCounter.start();
		makeOutPath(m_FileName);
		switch (m_FileFormat)
		{
		case emSplitFormat_RTCM:
			split_rtcm();
			break;
		case emSplitFormat_INS401:
			if (m_isRepeatData) {
				split_ins401_repeat();
			}
			break;
		default:
			break;
		}
	}
	emit sgnFinished();
}

void SplitThread::stop()
{
	m_isStop = true;
}

void SplitThread::setFileFormat(int format)
{
	m_FileFormat = format;
}

void SplitThread::setFileName(QString file)
{
	m_FileName = file;
}

void SplitThread::setRepeatData(bool isRepeat)
{
	m_isRepeatData = isRepeat;
}

void SplitThread::set_time_ref(uint32_t ref_time) {
	rtcm_split->set_time_ref(ref_time);
}

void SplitThread::set_time_range(uint32_t start_time, uint32_t end_time)
{
	rtcm_split->set_time_range(start_time, end_time);
}

void SplitThread::set_time_silce(uint32_t time_silce)
{
	rtcm_split->set_time_silce(time_silce);
}

void SplitThread::makeOutPath(QString filename)
{
	QFileInfo outDir(filename);
	if (outDir.isFile()) {
		QString basename = outDir.baseName();
		QString absoluteDir = outDir.absoluteDir().absolutePath();
		QDir outPath = absoluteDir + QDir::separator() + basename + "_s";
		if (!outPath.exists()) {
			outPath.mkpath(outPath.absolutePath());
		}
		m_OutBaseName = outPath.absolutePath() + QDir::separator() + basename;
	}
}

void SplitThread::split_rtcm()
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
				if (m_isRepeatData) {
					rtcm_split->split_data_repeat(read_cache[i]);
				}
				else {
					rtcm_split->split_data(read_cache[i]);
				}
			}
			double percent = (double)read_size / (double)file_size * 10000;
			emit sgnProgress((int)percent, m_TimeCounter.elapsed());
		}
		rtcm_split->close_files();
		fclose(file);
	}
}

void SplitThread::split_ins401_repeat() {
	FILE* file = fopen(m_FileName.toLocal8Bit().data(), "rb");
	if (file && ins401_decoder) {
		int ret = 0;
		int64_t file_size = getFileSize(file);
		int64_t read_size = 0;
		int readcount = 0;
		char read_cache[READ_CACHE_SIZE] = { 0 };
		ins401_decoder->init();
		ins401_decoder->set_base_file_name(m_OutBaseName.toLocal8Bit().data());
		ins401_decoder->set_output_file(false);
		int32_t last_gps_millisecs = 0;
		char split_file_name[256] = { 0 };
		int32_t split_index = 0;
		sprintf(split_file_name, "%s_%d.bin", m_OutBaseName.toLocal8Bit().data(), split_index);
		FILE* split_file = fopen(split_file_name, "wb");
		QByteArray write_cache;
		while (!feof(file)) {
			if (m_isStop) break;
			readcount = fread(read_cache, sizeof(char), READ_CACHE_SIZE, file);
			read_size += readcount;
			for (int i = 0; i < readcount; i++) {
				ret = ins401_decoder->input_data(read_cache[i]);
				write_cache.append(read_cache[i]);
				if (ret == 1) {
					if (Ins401_Tool::em_RAW_IMU == ins401_decoder->get_current_type()) {
						Ins401_Tool::raw_imu_t* imu = ins401_decoder->get_imu_raw();
						if (last_gps_millisecs > imu->gps_millisecs) {
							if (split_file) fclose(split_file); split_file = NULL;
							split_index++;
							sprintf(split_file_name, "%s_%d.bin", m_OutBaseName.toLocal8Bit().data(), split_index);
							split_file = fopen(split_file_name, "wb");
						}
						last_gps_millisecs = imu->gps_millisecs;
					}
				}
				if (ret == 1 || ret == 2) {
					fwrite(write_cache.data(), 1, write_cache.size(), split_file);
					write_cache.clear();
				}
			}
			double percent = (double)read_size / (double)file_size * 10000;
			emit sgnProgress((int)percent, m_TimeCounter.elapsed());
		}
		ins401_decoder->finish();
		fclose(file);
	}
}