#include "CsvAnalysisThread.h"
#include <QFile>

CsvAnalysisThread::CsvAnalysisThread(QObject *parent)
	: QThread(parent)
	, m_isStop(false)
{
	m_SkipLine = 1;
	m_StaticAnalysis = new StaticAnalysis(this);
}

CsvAnalysisThread::~CsvAnalysisThread()
{
	delete m_StaticAnalysis;
}

void CsvAnalysisThread::run()
{
	m_isStop = false;
	m_TimeCounter.start();
	AnalysisGnssCsv();
	emit sgnFinished();
}

void CsvAnalysisThread::stop()
{
	m_isStop = true;
}

void CsvAnalysisThread::setFileFormat(int format)
{
	m_FileFormat = format;
}

void CsvAnalysisThread::setFileName(QString file)
{
	m_FileName = file;
}

void CsvAnalysisThread::AnalysisGnssCsv()
{
	if (m_FileName.isEmpty()) return;
	if (!QFile::exists(m_FileName))return;
	QFile CsvFile(m_FileName);
	if (CsvFile.open(QIODevice::ReadOnly | QIODevice::Text)) {
		int32_t line_num = 0;
		while (!CsvFile.atEnd() && !m_isStop) {
			QByteArray byte_line = CsvFile.readLine(512);
			line_num++;
			if (line_num <= m_SkipLine) continue;
			QByteArrayList byte_items = byte_line.trimmed().split(',');

		}
		CsvFile.close();
	}
}
