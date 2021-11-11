#pragma once

#include <QWidget>
#include <QDragEnterEvent>
#include <QDropEvent>
#include <QMimeData>
#include <QTime>
#include "ui_SplitTools.h"

class SplitTools : public QWidget
{
	Q_OBJECT

public:
	SplitTools(QWidget *parent = Q_NULLPTR);
	~SplitTools();
protected:
	void dragEnterEvent(QDragEnterEvent * event);
	void dropEvent(QDropEvent * event);
private:
	Ui::SplitTools ui;
};
