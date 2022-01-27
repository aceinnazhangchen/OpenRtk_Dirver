#pragma once

#include <QWidget>
#include <QDragEnterEvent>
#include <QDropEvent>
#include <QMimeData>
#include <QTime>
#include "ui_SplitTools.h"
#include "SplitThread.h"

class SplitTools : public QWidget
{
	Q_OBJECT

public:
	SplitTools(QWidget *parent = Q_NULLPTR);
	~SplitTools();
	void setOperable(bool enable);
protected:
	void dragEnterEvent(QDragEnterEvent * event);
	void dropEvent(QDropEvent * event);
private:
	Ui::SplitTools ui;
	SplitThread* m_SplitThread;
	QTime m_TimeShow;
public slots:
	void onSelectFileClicked();
	void onSplitClicked();
	void onProcess(int present, int msecs);
	void onFinished();
};
