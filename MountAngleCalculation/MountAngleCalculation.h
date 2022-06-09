#pragma once

#include <QtWidgets/QMainWindow>
#include "ui_MountAngleCalculation.h"
#include <QDragEnterEvent>
#include <QDropEvent>
#include <QMimeData>
#include <QTime>
#include "SimpleDecodeThread.h"
#include "LoadInsTextFileThread.h"
#include "JsonFileLoader.h"
#include <vector>

struct stAngle
{
	float roll;
	float pitch;
	float heading;
};

class MountAngleCalculation : public QMainWindow
{
    Q_OBJECT

public:
    MountAngleCalculation(QWidget *parent = Q_NULLPTR);
	~MountAngleCalculation();
protected:
	void dragEnterEvent(QDragEnterEvent * event);
	void dropEvent(QDropEvent * event);
	void LoadConfigureJsonFile(QString filename);
	void LoadJsonFile();
	void SaveJsonFile();
	void setOperable(bool enable);
	void setSplitOperable(bool enable);
	void readAngleFromFile(QString file_path);
	void readAngleFromFile_1(QString file_path);
	void CalculateAverageAngle();
	QString ConvertNovatelPosType(QString novateType);
private slots:
	void onSelectFileClicked();
	void onSelectNovatelInsFileClicked();
	void onSelectProcessFileClicked();
	void onSelectResultFileClicked();
	void onSaveClicked();
	void onConvertClicked();
	void onDecodeClicked();
	void onProcessClicked();
	void onSplitClicked();
	void onCalculateClicked();
	void onCalculateAllClicked();
	void onCalculateNext();
	void onTimeSlicesChanged(const QString & time_str);
	void onProcess(int present, int msecs);
	void onDecodeFinished();
	void onSplitFinished();
signals:
	void sgnCalculateNext();
private:
    Ui::MountAngleCalculationClass ui;
	SimpleDecodeThread* m_SimpleDecodeThread;
	LoadInsTextFileThread* m_LoadInsTextFileThread;
	JsonFileLoader* m_JsonFileLoader;
	QTime m_TimeShow;
	QString m_ProcessFilePath;
	QString m_InsResultFilePath;
	std::vector<stAngle> angle_list;
};
