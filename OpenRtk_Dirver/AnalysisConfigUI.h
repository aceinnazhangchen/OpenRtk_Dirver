#pragma once

#include <QWidget>
#include "ui_AnalysisConfigUI.h"
#include "Ins401_Analysis.h"

class AnalysisConfigUI : public QWidget
{
	Q_OBJECT

public:
	AnalysisConfigUI(QWidget *parent = Q_NULLPTR);
	~AnalysisConfigUI();
	void set_thres(Ins401_Analysis * analyzer);
private:
	Ui::AnalysisConfigUI ui;
};
