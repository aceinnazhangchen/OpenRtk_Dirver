#pragma once

#include <QThread>
#include <QTime>

enum emAnalysisFormat {
	emAnalysisFormat_mixed,
};


class AnalysisThread : public QThread
{
	Q_OBJECT

public:
	AnalysisThread(QObject *parent);
	~AnalysisThread();
	void run();
	void stop();
	void setFileFormat(int format);
	void setFileName(QString file);
protected:
	void analysis_mixed_raw();
private:
	int getFileSize(FILE* file);
private:
	bool m_isStop;
	int m_FileFormat;
	QString m_FileName;
	QTime m_TimeCounter;
signals:
	void sgnProgress(int present, int msecs);
	void sgnFinished();
};
