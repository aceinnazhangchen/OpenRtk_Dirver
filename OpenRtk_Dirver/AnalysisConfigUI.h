#pragma once

#include <QWidget>
#include "ui_AnalysisConfigUI.h"
#include "StaticAnalysis.h"

class AnalysisConfigUI : public QWidget
{
	Q_OBJECT

public:
	AnalysisConfigUI(QWidget *parent = Q_NULLPTR);
	~AnalysisConfigUI();
	void set_thres_StaticAnalysis(StaticAnalysis * analyzer);
	bool isStaticTotalChecked();
	bool isMITableChecked();
private:
	Ui::AnalysisConfigUI ui;
};
