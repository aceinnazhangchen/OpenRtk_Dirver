#pragma once

#include <QThread>
#include <QTime>
#include <QList>
#include "ins401.h"
#include "Ins401_Analysis.h"
#include "E2E_protocol.h"

enum emDecodeFormat {
	emDecodeFormat_openrtk_user,
	emDecodeFormat_openrtk_inceptio,
	emDecodeFormat_mixed_raw,
	emDecodeFormat_imu,
	emDecodeFormat_ins401,
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
protected:
	void makeOutPath(QString filename);
	void decode_openrtk_user();
	void decode_openrtk_inceptio();
	void decode_mixed_raw();
	void decode_imu();
	void decode_ins401();
	void decode_e2e_protocol();
	void decode_Rtcm_EPVT();
private:
	bool m_isStop;
	int m_FileFormat;
	QString m_FileName;
	QString m_OutBaseName;
	QTime m_TimeCounter;
	Ins401_Tool::Ins401_decoder* ins401_decoder;
	E2E::E2E_protocol* e2e_deocder;
	bool m_show_time;
	int ins_kml_frequency;
public:
	bool m_static_point_ecp;
	Ins401_Analysis* m_Ins401_Analysis;
signals:
	void sgnProgress(int present, int msecs);
	void sgnFinished();
};
