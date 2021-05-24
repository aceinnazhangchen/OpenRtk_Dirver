#pragma once

#include <QWidget>
#include "ui_MergeTools.h"
#include <QDragEnterEvent>
#include <QDropEvent>
#include <QMimeData>
#include <QTime>
#include "MergeThread.h"

class MergeTools : public QWidget
{
	Q_OBJECT

public:
	MergeTools(QWidget *parent = Q_NULLPTR);
	~MergeTools();
	void setOperable(bool enable);
protected:
	void dragEnterEvent(QDragEnterEvent * event);
	void dropEvent(QDropEvent * event);
private:
	Ui::MergeTools ui;
	MergeThread* m_MergeThread;
	QTime m_TimeShow;
public slots:
	void onSelectFileClicked();
	void onSelectFileClicked2();
	void onMergeClicked();
	void onProcess(int present, int msecs);
	void onFinished();
	void onFileFormatChanged(int index);
};