#include "AnalysisThread.h"
#include "StreamManager.h"
#include "mixed_raw.h"

AnalysisThread::AnalysisThread(QObject *parent)
	: QThread(parent)
	, m_isStop(false)
	, m_FileFormat(emAnalysisFormat_mixed)
{
}

AnalysisThread::~AnalysisThread()
{
}

void AnalysisThread::run()
{
	m_isStop = false;
	if (!m_FileName.isEmpty()) {
		m_TimeCounter.start();
		switch (m_FileFormat)
		{
		case emAnalysisFormat_mixed:
			analysis_mixed_raw();
			break;
		default:
			break;
		}
	}
	emit sgnFinished();
}

void AnalysisThread::stop()
{
	m_isStop = true;
}

void AnalysisThread::setFileFormat(int format)
{
	m_FileFormat = format;
}

void AnalysisThread::setFileName(QString file)
{
	m_FileName = file;
}

void AnalysisThread::analysis_mixed_raw()
{
	if (is_aceinna_decoding()) return;
	set_aceinna_decoding(1);
	StreamManager::Instance()->SetDecode(true);
	StreamManager::Instance()->SetReplayFileName(m_FileName);
	StreamManager::Instance()->OpenReplayFile();
	while (StreamManager::Instance()->SendReplayData()) {
		if (m_isStop) break;
	}
	StreamManager::Instance()->StopReplayOpenRTK();
	StreamManager::Instance()->SetDecode(false);
	set_aceinna_decoding(0);
}

int AnalysisThread::getFileSize(FILE * file)
{
	fseek(file, 0L, SEEK_END);
	int file_size = ftell(file);
	fseek(file, 0L, SEEK_SET);
	return file_size;
}
