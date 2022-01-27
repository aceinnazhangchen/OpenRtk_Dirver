#pragma once

#include <QThread>
#include <QTime>
#include "rtcm_split.h"

enum emSplitFormat {
	emSplitFormat_RTCM,
};

class SplitThread : public QThread
{
	Q_OBJECT

public:
	SplitThread(QObject *parent);
	~SplitThread();
	void run();
	void stop();
	void setFileFormat(int format);
	void setFileName(QString file);
	void set_time_range(uint32_t start_time, uint32_t end_time);
	void set_time_silce(uint32_t time_silce);
protected:
	void makeOutPath(QString filename);
	void split_rtcm();
private:
	bool m_isStop;
	int m_FileFormat;
	QString m_FileName;
	QString m_OutBaseName;
	QTime m_TimeCounter;
	Rtcm_Split* rtcm_split;
signals:
	void sgnProgress(int present, int msecs);
	void sgnFinished();
};
