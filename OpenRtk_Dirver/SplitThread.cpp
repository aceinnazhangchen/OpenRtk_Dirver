#include "SplitThread.h"
#include <QDir>
#include "common.h"

SplitThread::SplitThread(QObject *parent)
	: QThread(parent)
	, m_isStop(false)
	, m_FileFormat(emSplitFormat_RTCM)
{
	rtcm_split = new Rtcm_Split();
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
				rtcm_split->input_data(read_cache[i]);
			}
			double percent = (double)read_size / (double)file_size * 10000;
			emit sgnProgress((int)percent, m_TimeCounter.elapsed());
		}
		rtcm_split->close_files();
		fclose(file);
	}
}
