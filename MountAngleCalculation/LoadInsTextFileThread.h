#pragma once

#include <QThread>
#include "MountAngle.h"

class LoadInsTextFileThread : public QThread
{
	Q_OBJECT

public:
	LoadInsTextFileThread(QObject *parent);
	~LoadInsTextFileThread();
	void run();
	void stop();
	void setFileName(QString file);
	QString getBasePath();
	std::vector<stTimeSlice>& get_time_slices();
protected:
	void LoadInsText();
private:
	bool m_isStop;
	QString m_InsTextFileName;
	MountAngle* m_MountAngle;
signals:
	void sgnFinished();
};
