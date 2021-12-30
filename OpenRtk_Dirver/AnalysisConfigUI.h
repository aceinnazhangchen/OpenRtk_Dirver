#pragma once

#include <QWidget>
#include "ui_AnalysisConfigUI.h"
#include "Ins401_Analysis.h"
#include "RTK330LA_Analysis.h"

class AnalysisConfigUI : public QWidget
{
	Q_OBJECT

public:
	AnalysisConfigUI(QWidget *parent = Q_NULLPTR);
	~AnalysisConfigUI();
	void set_thres_Ins401(Ins401_Tool::Ins401_Analysis * analyzer);
	void set_thres_RTK330LA(RTK330LA_Tool::RTK330LA_Analysis * analyzer);
private:
	Ui::AnalysisConfigUI ui;
};
