#pragma once

#include <QWidget>
#include "ui_DecodeTools.h"
#include <QDragEnterEvent>
#include <QDropEvent>
#include <QMimeData>
#include <QTime>
#include "DecodeThread.h"
#include "AnalysisConfigUI.h"

class DecodeTools : public QWidget
{
	Q_OBJECT

public:
	DecodeTools(QWidget *parent = Q_NULLPTR);
	~DecodeTools();
	void setOperable(bool enable);
protected:
	void dragEnterEvent(QDragEnterEvent * event);
	void dropEvent(QDropEvent * event);
private:
	Ui::DecodeTools ui;
	DecodeThread* m_DecodeThread;
	QTime m_TimeShow;
	AnalysisConfigUI* m_AnalysisConfigUI;
public slots:
	void onSelectFileClicked();
	void onDecodeClicked();
	void onProcess(int present, int msecs);
	void onFinished();
	void onClickedSettingButton();
};
