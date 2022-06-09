#pragma once

#include <QThread>
#include <QTime>
#include "rtcm_split.h"
#include "ins401.h"

enum emSplitFormat {
	emSplitFormat_RTCM,
	emSplitFormat_INS401,
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
	void setRepeatData(bool isRepeat);
	void set_time_ref(uint32_t ref_time);
	void set_time_range(uint32_t start_time, uint32_t end_time);
	void set_time_silce(uint32_t time_silce);
protected:
	void makeOutPath(QString filename);
	void split_rtcm();
	void split_ins401_repeat();
private:
	bool m_isStop;
	bool m_isRepeatData;
	int m_FileFormat;
	QString m_FileName;
	QString m_OutBaseName;
	QTime m_TimeCounter;
	Rtcm_Split* rtcm_split;
	Ins401_Tool::Ins401_decoder* ins401_decoder;
signals:
	void sgnProgress(int present, int msecs);
	void sgnFinished();
};
