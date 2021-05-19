#pragma once

#include <QWidget>
#include "ui_AnalysisTools.h"
#include <QDragEnterEvent>
#include <QDropEvent>
#include <QMimeData>
#include <QTime>
#include "AnalysisThread.h"

class AnalysisTools : public QWidget
{
	Q_OBJECT

public:
	AnalysisTools(QWidget *parent = Q_NULLPTR);
	~AnalysisTools();
	void setOperable(bool enable);
protected:
	void dragEnterEvent(QDragEnterEvent * event);
	void dropEvent(QDropEvent * event);
private:
	Ui::AnalysisTools ui;
	AnalysisThread* m_AnalysisThread;
	QTime m_TimeShow;
public slots:
	void onSelectFileClicked();
	void onAnalysisClicked();
	void onProcess(int present, int msecs);
	void onFinished();
};
