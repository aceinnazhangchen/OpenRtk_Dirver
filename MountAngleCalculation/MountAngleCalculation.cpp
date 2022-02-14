#include "MountAngleCalculation.h"
#include "ConfigFile.h"
#include <QProgressBar>
#include <QFileDialog>
#include "CalculationCall.h"

#define DIMENSION	3

MountAngleCalculation::MountAngleCalculation(QWidget *parent)
    : QMainWindow(parent)
{
    ui.setupUi(this);
	setAcceptDrops(true);
	m_ProcessFilePath = "";
	m_SimpleDecodeThread = new SimpleDecodeThread(this);
	m_LoadInsTextFileThread = new LoadInsTextFileThread(this);
	LoadJsonFile();
	ui.tableWidget_config->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
	ui.tableWidget_config->verticalHeader()->setSectionResizeMode(QHeaderView::Stretch);
	//connect(ui.pushButton_save, SIGNAL(clicked()), this, SLOT(onSaveClicked()));
	connect(ui.select_btn, SIGNAL(clicked()), this, SLOT(onSelectFileClicked()));
	connect(ui.pushButton_decode, SIGNAL(clicked()), this, SLOT(onDecodeClicked()));
	connect(ui.pushButton_split, SIGNAL(clicked()), this, SLOT(onSplitClicked()));
	connect(ui.process_select_btn, SIGNAL(clicked()), this, SLOT(onSelectProcessFileClicked()));
	connect(ui.pushButton_calculate, SIGNAL(clicked()), this, SLOT(onCalculateClicked()));
	connect(ui.pushButton_calculate_all, SIGNAL(clicked()), this, SLOT(onCalculateAllClicked()));
	connect(this, SIGNAL(sgnCalculateNext()), this, SLOT(onCalculateNext()));
	connect(ui.time_slices_comb, SIGNAL(currentIndexChanged(const QString &)), this, SLOT(onTimeSlicesChanged(const QString &)));
	connect(m_SimpleDecodeThread, SIGNAL(sgnProgress(int, int)), this, SLOT(onProcess(int, int)));
	connect(m_SimpleDecodeThread, SIGNAL(sgnFinished()), this, SLOT(onFinished()));
	connect(m_LoadInsTextFileThread, SIGNAL(sgnFinished()), this, SLOT(onSplitFinished()));
}

MountAngleCalculation::~MountAngleCalculation()
{
	if (m_SimpleDecodeThread->isRunning()) {
		m_SimpleDecodeThread->stop();
		m_SimpleDecodeThread->terminate();
	}
	if (m_LoadInsTextFileThread->isRunning()) {
		m_LoadInsTextFileThread->stop();
		m_LoadInsTextFileThread->terminate();
	}	
	delete m_SimpleDecodeThread;
	delete m_LoadInsTextFileThread;
}

void MountAngleCalculation::dragEnterEvent(QDragEnterEvent * event)
{
	event->acceptProposedAction();
}

void MountAngleCalculation::dropEvent(QDropEvent * event)
{
	if (ui.filepath_edt->isEnabled() && ui.filepath_edt->geometry().contains(event->pos())) {
		QString name = event->mimeData()->urls().first().toLocalFile();
		ui.filepath_edt->setText(name);
	}
	else if (ui.process_filepath_edt->isEnabled() && ui.process_filepath_edt->geometry().contains(event->pos())) {
		QString name = event->mimeData()->urls().first().toLocalFile();
		ui.process_filepath_edt->setText(name);
	}
}

void MountAngleCalculation::LoadJsonFile()
{
	QJsonObject& configJson = ConfigFile::getInstance()->getConfig();
	ui.tableWidget_config->clearContents();
	if (configJson["priLeverArm"].isArray()) {
		//priLeverArm
		QJsonArray priLeverArm = configJson["priLeverArm"].toArray();
		if (priLeverArm.size() == DIMENSION) {
			for (int i = 0; i < DIMENSION; i++) {
				double priLeverArm_value = priLeverArm[i].toDouble();
				QTableWidgetItem *item = new QTableWidgetItem(QString::number(priLeverArm_value));
				ui.tableWidget_config->setItem(0, i, item);
			}
		}
		//rotationRBV
		QJsonArray rotationRBV = configJson["rotationRBV"].toArray();
		if (rotationRBV.size() == DIMENSION) {
			for (int i = 0; i < DIMENSION; i++) {
				double rotationRBV_value = rotationRBV[i].toDouble();
				QTableWidgetItem *item = new QTableWidgetItem(QString::number(rotationRBV_value));
				ui.tableWidget_config->setItem(1, i, item);
			}
		}
		//odoLeverarm
		QJsonArray odoLeverarm = configJson["odoLeverarm"].toArray();
		if (odoLeverarm.size() == DIMENSION) {
			for (int i = 0; i < DIMENSION; i++) {
				double odoLeverarm_value = odoLeverarm[i].toDouble();
				QTableWidgetItem *item = new QTableWidgetItem(QString::number(odoLeverarm_value));
				ui.tableWidget_config->setItem(2, i, item);
			}
		}
		//userLeverArm
		QJsonArray userLeverArm = configJson["userLeverArm"].toArray();
		if (userLeverArm.size() == DIMENSION) {
			for (int i = 0; i < DIMENSION; i++) {
				double userLeverArm_value = userLeverArm[i].toDouble();
				QTableWidgetItem *item = new QTableWidgetItem(QString::number(userLeverArm_value));
				ui.tableWidget_config->setItem(3, i, item);
			}
		}
	}
}

void MountAngleCalculation::SaveJsonFile()
{
	QJsonObject& configJson = ConfigFile::getInstance()->getConfig();
	configJson["procfileNme"] = m_ProcessFilePath;
	//priLeverArm
	QJsonArray priLeverArm;
	for (int i = 0; i < DIMENSION; i++) {
		QTableWidgetItem *item = ui.tableWidget_config->item(0, i);
		priLeverArm.append(item->text().toDouble());
	}
	configJson["priLeverArm"] = priLeverArm;
	//rotationRBV
	QJsonArray rotationRBV;
	for (int i = 0; i < DIMENSION; i++) {
		QTableWidgetItem *item = ui.tableWidget_config->item(1, i);
		rotationRBV.append(item->text().toDouble());
	}
	configJson["rotationRBV"] = rotationRBV;
	//odoLeverarm
	QJsonArray odoLeverarm;
	for (int i = 0; i < DIMENSION; i++) {
		QTableWidgetItem *item = ui.tableWidget_config->item(2, i);
		odoLeverarm.append(item->text().toDouble());
	}
	configJson["odoLeverarm"] = odoLeverarm;
	//userLeverArm
	QJsonArray userLeverArm;
	for (int i = 0; i < DIMENSION; i++) {
		QTableWidgetItem *item = ui.tableWidget_config->item(3, i);
		userLeverArm.append(item->text().toDouble());
	}
	configJson["userLeverArm"] = userLeverArm;
	ConfigFile::getInstance()->writeConfigFile();
}

void MountAngleCalculation::setOperable(bool enable)
{
	ui.filepath_edt->setEnabled(enable);
	ui.select_btn->setEnabled(enable);
	//ui.decode_btn->setEnabled(enable);
	if (enable) {
		ui.pushButton_decode->setText("decode");
	}
	else {
		ui.pushButton_decode->setText("stop");
	}
}

void MountAngleCalculation::setSplitOperable(bool enable) {
	ui.process_filepath_edt->setEnabled(enable);
	ui.process_select_btn->setEnabled(enable);
	ui.pushButton_split->setEnabled(enable);
}

void MountAngleCalculation::onSelectFileClicked()
{
	QString current_path = ".";
	QString file_name = ui.filepath_edt->text();
	if (!file_name.isEmpty()) {
		current_path = QDir(file_name).absolutePath();
	}
	QString path = QFileDialog::getOpenFileName(this, tr("Open Files"), current_path, tr("Data Files(*.* )"));
	if (path.length() == 0) {
		return;
	}
	ui.filepath_edt->setText(path);
}

void MountAngleCalculation::onSelectProcessFileClicked()
{
	QString current_path = ".";
	QString file_name = ui.process_filepath_edt->text();
	if (!file_name.isEmpty()) {
		current_path = QDir(file_name).absolutePath();
	}
	QString path = QFileDialog::getOpenFileName(this, tr("Open Files"), current_path, tr("Data Files(*.* )"));
	if (path.length() == 0) {
		return;
	}
	ui.process_filepath_edt->setText(path);
}

void MountAngleCalculation::onSaveClicked() {
	SaveJsonFile();
}

void MountAngleCalculation::onDecodeClicked()
{
	if (m_SimpleDecodeThread->isRunning())
	{
		m_SimpleDecodeThread->stop();
		return;
	}
	QString filename = ui.filepath_edt->text();
	if (filename.isEmpty()) {
		return;
	}
	ui.progressBar->setValue(0);
	m_SimpleDecodeThread->setFileFormat(ui.fileformat_cmb->currentIndex());
	m_SimpleDecodeThread->setFileName(filename);
	m_ProcessFilePath = m_SimpleDecodeThread->getOutBaseName()+"_process";
	SaveJsonFile();
	setOperable(false);
	m_SimpleDecodeThread->start();
}

void MountAngleCalculation::onSplitClicked()
{
	QString filename = ui.process_filepath_edt->text();
	if (filename.isEmpty()) {
		return;
	}
	if (m_LoadInsTextFileThread->isRunning())
	{
		m_LoadInsTextFileThread->stop();
		return;
	}
	m_LoadInsTextFileThread->setFileName(filename);
	setSplitOperable(false);
	m_LoadInsTextFileThread->start();
}

void MountAngleCalculation::onCalculateClicked() {
	m_InsResultFilePath = ui.process_filepath_edt->text();
	QFileInfo result_file(m_InsResultFilePath);
	if (!result_file.isFile()) return;
	QString basename = result_file.baseName();
	QString absoluteDir = result_file.absoluteDir().absolutePath();

	QString starttime_str = ui.starttime_edt->text();
	QString endtime_str = ui.endtime_edt->text();
	if (starttime_str.isEmpty())return;
	if (endtime_str.isEmpty())return;
	CalculationCall::call_dr_mountangle_start(m_InsResultFilePath.toLocal8Bit().data(), starttime_str.toLocal8Bit().data(), endtime_str.toLocal8Bit().data());
	QString out_file_path = absoluteDir + QDir::separator() + basename + "_ima_" + starttime_str + "-" + endtime_str + ".txt";
	readAngleFromFile(out_file_path);
}

void MountAngleCalculation::onCalculateAllClicked() {
	if (ui.time_slices_comb->count() > 0) {
		angle_list.clear();
		ui.time_slices_comb->setCurrentIndex(0);
		emit sgnCalculateNext();
	}
}

void MountAngleCalculation::onCalculateNext()
{
	int index = ui.time_slices_comb->currentIndex();
	onCalculateClicked();
	if (index+1 < ui.time_slices_comb->count()) {
		ui.time_slices_comb->setCurrentIndex(index+1);
		emit sgnCalculateNext();
	}
	else {
		//取平均
		CalculateAverageAngle();
	}
}

void MountAngleCalculation::readAngleFromFile(QString file_path) {
	QFileInfo out_file(file_path);
	if (!out_file.isFile()) return;
	FILE* f_out = fopen(file_path.toLocal8Bit().data(), "r");
	char line[256] = { 0 };
	if (f_out) {
		ui.result_edt->setText(file_path);
		fgets(line, 256, f_out);//先读一行
		size_t line_size = strlen(line);//获取一行的长度
		fseek(f_out, -(line_size+1), SEEK_END);//定位到文件最后,并且向前偏移一行的长度加1(因为最后有个空行)
		fgets(line, 256, f_out);//读最后一行
		fclose(f_out);
	}
	QString last_line = QString(line);
	QStringList items = last_line.split(',');
	if (items.size() < 8) return;
	stAngle angle = { 0 };
	angle.roll = items[5].trimmed().toFloat();
	angle.pitch = items[6].trimmed().toFloat();
	angle.heading = items[7].trimmed().toFloat();
	angle_list.push_back(angle);
	ui.result_edt->setText(items[5].trimmed() + "," + items[6].trimmed() + "," + items[7].trimmed());
}

void MountAngleCalculation::CalculateAverageAngle()
{
	stAngle sum_angle = { 0 };
	stAngle avg_angle = { 0 };
	for (int i = 0; i < angle_list.size(); i++) {
		sum_angle.heading += angle_list[i].heading;
		sum_angle.pitch += angle_list[i].pitch;
		sum_angle.roll += angle_list[i].roll;
	}
	avg_angle.heading = sum_angle.heading / angle_list.size();
	avg_angle.pitch = sum_angle.pitch / angle_list.size();
	avg_angle.roll = sum_angle.roll / angle_list.size();
	ui.result_edt->setText(QString::asprintf("%f,%f,%f", avg_angle.roll, avg_angle.pitch, avg_angle.heading));
}

void MountAngleCalculation::onTimeSlicesChanged(const QString & time_str) {
	QStringList time_sp = time_str.split(':');
	if (time_sp.size() >= 2) {
		ui.starttime_edt->setText(time_sp[0]);
		ui.endtime_edt->setText(time_sp[1]);
	}
}

void MountAngleCalculation::onProcess(int present, int msecs)
{
	ui.progressBar->setValue(present);
	double dProgress = ui.progressBar->value() * 100.0 / ui.progressBar->maximum();
	ui.progressBar->setFormat(QString("%1%").arg(QString::number(dProgress, 'f', 2)));
	m_TimeShow.setHMS(0, 0, 0, 0);
	ui.time_lb->setText(m_TimeShow.addMSecs(msecs).toString("hh:mm:ss:zzz"));
}

void MountAngleCalculation::onFinished()
{
	if (m_ProcessFilePath.isEmpty() == false) {
		m_InsResultFilePath = m_ProcessFilePath + "_ins.txt";
		QFileInfo file(m_InsResultFilePath);
		if (file.isFile()) {
			ui.process_filepath_edt->setText(m_InsResultFilePath);
		}
	}
	ui.time_slices_comb->clear();
	std::vector<stTimeSlice>& time_slices = m_SimpleDecodeThread->get_time_slices();
	for (int i = 0; i < time_slices.size();i++) {
		QString time_str = QString::asprintf("%d:%d:%d", time_slices[i].starttime, time_slices[i].endtime, time_slices[i].during);
		ui.time_slices_comb->addItem(time_str);
	}
	setOperable(true);
}

void MountAngleCalculation::onSplitFinished()
{
	ui.time_slices_comb->clear();
	std::vector<stTimeSlice>& time_slices = m_LoadInsTextFileThread->get_time_slices();
	for (int i = 0; i < time_slices.size(); i++) {
		QString time_str = QString::asprintf("%d:%d:%d", time_slices[i].starttime, time_slices[i].endtime,time_slices[i].during);
		ui.time_slices_comb->addItem(time_str);
	}
	setSplitOperable(true);
}
