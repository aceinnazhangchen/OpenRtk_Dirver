#pragma once

#include <QWidget>
#include "ui_StreamConfigUI.h"

class StreamConfigUI : public QWidget
{
	Q_OBJECT

public:
	StreamConfigUI(QWidget *parent = Q_NULLPTR);
	~StreamConfigUI();
	void SaveConfig();
	void LoadConfig();
	void SearchComPort();
	void OpenIndex(int index);
private:
	Ui::StreamConfigUI ui;
	int m_CurrentIndex;
public slots:
	void onCloseUI();
signals:
	void sgnClosed();
};
