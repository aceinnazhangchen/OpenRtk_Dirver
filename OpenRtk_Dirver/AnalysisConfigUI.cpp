#include "AnalysisConfigUI.h"

AnalysisConfigUI::AnalysisConfigUI(QWidget *parent)
	: QWidget(parent)
{
	ui.setupUi(this);
	ui.tableWidget->horizontalHeader()->setStretchLastSection(true);
	ui.tableWidget->setItem(0, 0, new QTableWidgetItem("99.7"));
	ui.tableWidget->setItem(1, 0, new QTableWidgetItem("0.2"));
	ui.tableWidget->setItem(2, 0, new QTableWidgetItem("0.2"));
	ui.tableWidget->setItem(3, 0, new QTableWidgetItem("0.2"));
	ui.tableWidget->setItem(4, 0, new QTableWidgetItem("0.2"));
	ui.tableWidget->setItem(5, 0, new QTableWidgetItem("30"));
}

AnalysisConfigUI::~AnalysisConfigUI()
{
}

void AnalysisConfigUI::set_thres_Ins401(Ins401_Tool::Ins401_Analysis* analyzer) {

	QString cep_level = ui.tableWidget->item(0, 0)->text();
	QString hor_pos_err_thres = ui.tableWidget->item(1, 0)->text();
	QString ver_pos_err_thres = ui.tableWidget->item(2, 0)->text();
	QString hor_vel_err_thres = ui.tableWidget->item(3, 0)->text();
	QString ver_vel_err_thres = ui.tableWidget->item(4, 0)->text();
	QString start_line = ui.tableWidget->item(5, 0)->text();

	analyzer->set_thres(cep_level.toDouble(), hor_pos_err_thres.toDouble(), ver_pos_err_thres.toDouble(), hor_vel_err_thres.toDouble(), ver_vel_err_thres.toDouble(), start_line.toInt());
}

void AnalysisConfigUI::set_thres_RTK330LA(RTK330LA_Tool::RTK330LA_Analysis * analyzer)
{
	QString cep_level = ui.tableWidget->item(0, 0)->text();
	QString hor_pos_err_thres = ui.tableWidget->item(1, 0)->text();
	QString ver_pos_err_thres = ui.tableWidget->item(2, 0)->text();
	QString hor_vel_err_thres = ui.tableWidget->item(3, 0)->text();
	QString ver_vel_err_thres = ui.tableWidget->item(4, 0)->text();
	QString start_line = ui.tableWidget->item(5, 0)->text();

	analyzer->set_thres(cep_level.toDouble(), hor_pos_err_thres.toDouble(), ver_pos_err_thres.toDouble(), hor_vel_err_thres.toDouble(), ver_vel_err_thres.toDouble(), start_line.toInt());
}

bool AnalysisConfigUI::isStaticTotalChecked()
{
	return ui.checkBox_static_total->isChecked();
}

bool AnalysisConfigUI::isMITableChecked()
{
	return ui.checkBox_MI_table->isChecked();
}