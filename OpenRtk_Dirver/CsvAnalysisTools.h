#pragma once

#include <QWidget>
#include "ui_CsvAnalysisTools.h"
#include <QDragEnterEvent>
#include <QDropEvent>
#include <QMimeData>
#include <QTime>
class CsvAnalysisTools : public QWidget
{
	Q_OBJECT

public:
	CsvAnalysisTools(QWidget *parent = Q_NULLPTR);
	~CsvAnalysisTools();
	void setOperable(bool enable);
protected:
	void dragEnterEvent(QDragEnterEvent * event);
	void dropEvent(QDropEvent * event);
private:
	Ui::CsvAnalysisTools ui;
	QTime m_TimeShow;
public slots:
	void onSelectFileClicked();
	void onAnalysisClicked();
	void onProcess(int present, int msecs);
	void onFinished();
};
