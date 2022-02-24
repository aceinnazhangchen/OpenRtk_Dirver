#pragma once

#include <QThread>
#include <QTime>
#include "StaticAnalysis.h"

class CsvAnalysisThread : public QThread
{
	Q_OBJECT

public:
	CsvAnalysisThread(QObject *parent);
	~CsvAnalysisThread();
	void run();
	void stop();
	void setFileFormat(int format);
	void setFileName(QString file);
	QString getBasePath();
protected:
	void AnalysisGnssCsv();
private:
	bool m_isStop;
	int m_FileFormat;
	QString m_FileName;
	QTime m_TimeCounter;
	StaticAnalysis* m_StaticAnalysis;
	int32_t m_SkipLine;
	int start_column;
signals:
	void sgnProgress(int present, int msecs);
	void sgnFinished();
};
