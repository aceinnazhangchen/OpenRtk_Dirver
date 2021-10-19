#pragma once

#include <QThread>
#include <QTime>
#include "ins401.h"
#include "rtcm_split.h"
#include "E2E_protocol.h"

enum emDecodeFormat {
	emDecodeFormat_openrtk_user,
	emDecodeFormat_openrtk_inceptio,
	emDecodeFormat_mixed_raw,
	emDecodeFormat_imu,
	emDecodeFormat_ins401,
	emDecodeFormat_RTCM_Split,
	emDecodeFormat_E2E_protocol,
};

class DecodeThread : public QThread
{
	Q_OBJECT

public:
	DecodeThread(QObject *parent);
	~DecodeThread();
	void run();
	void stop();
	void setFileFormat(int format);
	void setFileName(QString file);
	void setShowTime(bool show);
	void setKmlFrequency(int frequency);
	void makeOutPath(QString filename);
protected:
	void decode_openrtk_user();
	void decode_openrtk_inceptio();
	void decode_mixed_raw();
	void decode_imu();
	void decode_ins401();
	void split_rtcm();
	void decode_e2e_protocol();
private:
	int64_t getFileSize(FILE* file);
private:
	bool m_isStop;
	int m_FileFormat;
	QString m_FileName;
	QString m_OutBaseName;
	QTime m_TimeCounter;
	Ins401::Ins401_decoder* ins401_decoder;
	Rtcm_Split* rtcm_split;
	E2E::E2E_protocol* e2e_deocder;
	bool m_show_time;
	int ins_kml_frequency;
signals:
	void sgnProgress(int present, int msecs);
	void sgnFinished();
};
