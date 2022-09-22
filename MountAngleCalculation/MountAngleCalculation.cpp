#include "MountAngleCalculation.h"
#include "ConfigFile.h"
#include <QProgressBar>
#include <QFileDialog>
#include <QMessageBox>
#include "CalculationCall.h"
#include <QDebug>

#define DIMENSION	3

MountAngleCalculation::MountAngleCalculation(QWidget *parent)
    : QMainWindow(parent)
{
    ui.setupUi(this);
	setAcceptDrops(true);
	m_DecodeImuFilePath = "";
	m_DecodeOdoFilePath = "";
	m_DecodeInsFilePath = "";
	m_SimpleDecodeThread = new SimpleDecodeThread(this);
	m_LoadInsTextFileThread = new LoadInsTextFileThread(this);
	m_CalculationThread = new CalculationThread(this, m_LoadInsTextFileThread->get_time_slices());
	m_JsonFileLoader = new JsonFileLoader(this);
	LoadJsonFile();
	ui.tableWidget_config->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
	ui.tableWidget_config->verticalHeader()->setSectionResizeMode(QHeaderView::Stretch);
	connect(ui.pushButton_save, SIGNAL(clicked()), this, SLOT(onSaveClicked()));
	connect(ui.select_btn, SIGNAL(clicked()), this, SLOT(onSelectFileClicked()));
	connect(ui.novatel_ins_convert_btn, SIGNAL(clicked()), this, SLOT(onConvertClicked()));
	connect(ui.pushButton_decode, SIGNAL(clicked()), this, SLOT(onDecodeClicked()));
	connect(ui.pushButton_process, SIGNAL(clicked()), this, SLOT(onProcessClicked()));
	connect(ui.pushButton_split, SIGNAL(clicked()), this, SLOT(onSplitClicked()));
	connect(ui.novatel_ins_select_btn, SIGNAL(clicked()), this, SLOT(onSelectNovatelInsFileClicked()));
	connect(ui.process_select_btn, SIGNAL(clicked()), this, SLOT(onSelectProcessFileClicked()));
	connect(ui.result_select_btn, SIGNAL(clicked()), this, SLOT(onSelectResultFileClicked()));
	connect(ui.pushButton_calculate, SIGNAL(clicked()), this, SLOT(onCalculateOneClicked()));
	connect(ui.pushButton_calculate_all, SIGNAL(clicked()), this, SLOT(onCalculateAllClicked()));
	connect(ui.pushButton_average, SIGNAL(clicked()), this, SLOT(onAverageClicked()));
	connect(ui.pushButton_save_json, SIGNAL(clicked()), this, SLOT(onSaveJsonClicked()));
	connect(ui.time_slices_comb, SIGNAL(currentIndexChanged(const QString &)), this, SLOT(onTimeSlicesChanged(const QString &)));

	connect(m_SimpleDecodeThread, SIGNAL(sgnProgress(int, int)), this, SLOT(onDecodingProcess(int, int)));
	connect(m_SimpleDecodeThread, SIGNAL(sgnFinished()), this, SLOT(onDecodeFinished()));	
	connect(m_LoadInsTextFileThread, SIGNAL(sgnFinished()), this, SLOT(onSplitFinished()));
	connect(m_CalculationThread, SIGNAL(sgnFinished()), this, SLOT(onCalculateFinished()));

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
	delete m_CalculationThread;
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
		FindRtcmFilesInSameDir(name);
	}
	else if (ui.novatel_ins_path_edt->isEnabled() && ui.novatel_ins_path_edt->geometry().contains(event->pos())) {
		QString name = event->mimeData()->urls().first().toLocalFile();
		ui.novatel_ins_path_edt->setText(name);
	}
	else if (ui.tableWidget_config->isEnabled() && ui.tableWidget_config->geometry().contains(event->pos())) {
		QString name = event->mimeData()->urls().first().toLocalFile();
		LoadConfigureJsonFile(name);
	}
	else if (ui.gnssposvel_filepath_edt->isEnabled() && ui.gnssposvel_filepath_edt->geometry().contains(event->pos())) {
		QString name = event->mimeData()->urls().first().toLocalFile();
		ui.gnssposvel_filepath_edt->setText(name);
	}
	else if (ui.movbs_filepath_edt->isEnabled() && ui.movbs_filepath_edt->geometry().contains(event->pos())) {
		QString name = event->mimeData()->urls().first().toLocalFile();
		ui.movbs_filepath_edt->setText(name);
	}
	else if (ui.process_filepath_edt->isEnabled() && ui.process_filepath_edt->geometry().contains(event->pos())) {
		QString name = event->mimeData()->urls().first().toLocalFile();
		ui.process_filepath_edt->setText(name);
	}
	else if (ui.post_ins_filepath_edt->isEnabled() && ui.post_ins_filepath_edt->geometry().contains(event->pos())) {
		QString name = event->mimeData()->urls().first().toLocalFile();
		ui.post_ins_filepath_edt->setText(name);
	}
	else if (ui.post_movbs_filepath_edt->isEnabled() && ui.post_movbs_filepath_edt->geometry().contains(event->pos())) {
		QString name = event->mimeData()->urls().first().toLocalFile();
		ui.post_movbs_filepath_edt->setText(name);
	}
}

void MountAngleCalculation::LoadConfigureJsonFile(QString filename)
{
	if (m_JsonFileLoader == NULL) return;
	if (m_JsonFileLoader->readJsonArrayFile(filename) == false) return;
	ui.label_Load_json_name->setText("Loaded: " + filename);
	QJsonObject& configJson = m_JsonFileLoader->getConfig();
	if (configJson["parameters"].isObject()) {
		//ui.tableWidget_config->clearContents();
		QJsonObject parameters = configJson["parameters"].toObject();
		//rtk330 pri lever
		if (parameters["pri lever arm x"].isDouble()) {
			QTableWidgetItem *item = ui.tableWidget_config->item(0, 0);
			item->setText(QString::number(parameters["pri lever arm x"].toDouble()));
		}
		if (parameters["pri lever arm y"].isDouble()) {
			QTableWidgetItem *item = ui.tableWidget_config->item(0, 1);
			item->setText(QString::number(parameters["pri lever arm y"].toDouble()));
		}
		if (parameters["pri lever arm z"].isDouble()) {
			QTableWidgetItem *item = ui.tableWidget_config->item(0, 2);
			item->setText(QString::number(parameters["pri lever arm z"].toDouble()));
		}
		//ins401 gnss lever
		if (parameters["gnss lever arm x"].isDouble()) {
			QTableWidgetItem *item = ui.tableWidget_config->item(0, 0);
			item->setText(QString::number(parameters["gnss lever arm x"].toDouble()));
		}
		if (parameters["gnss lever arm y"].isDouble()) {
			QTableWidgetItem *item = ui.tableWidget_config->item(0, 1);
			item->setText(QString::number(parameters["gnss lever arm y"].toDouble()));
		}
		if (parameters["gnss lever arm z"].isDouble()) {
			QTableWidgetItem *item = ui.tableWidget_config->item(0, 2);
			item->setText(QString::number(parameters["gnss lever arm z"].toDouble()));
		}
		//vrp lever arm
		if (parameters["vrp lever arm x"].isDouble()) {
			QTableWidgetItem *item = ui.tableWidget_config->item(1, 0);
			item->setText(QString::number(parameters["vrp lever arm x"].toDouble()));
		}
		if (parameters["vrp lever arm y"].isDouble()) {
			QTableWidgetItem *item = ui.tableWidget_config->item(1, 1);
			item->setText(QString::number(parameters["vrp lever arm y"].toDouble()));
		}
		if (parameters["vrp lever arm z"].isDouble()) {
			QTableWidgetItem *item = ui.tableWidget_config->item(1, 2);
			item->setText(QString::number(parameters["vrp lever arm z"].toDouble()));
		}
		//user lever arm
		if (parameters["user lever arm x"].isDouble()) {
			QTableWidgetItem *item = ui.tableWidget_config->item(2, 0);
			item->setText(QString::number(parameters["user lever arm x"].toDouble()));
		}
		if (parameters["user lever arm y"].isDouble()) {
			QTableWidgetItem *item = ui.tableWidget_config->item(2, 1);
			item->setText(QString::number(parameters["user lever arm y"].toDouble()));
		}
		if (parameters["user lever arm z"].isDouble()) {
			QTableWidgetItem *item = ui.tableWidget_config->item(2, 2);
			item->setText(QString::number(parameters["user lever arm z"].toDouble()));
		}
		//rotation rbvx
		if (parameters["rotation rbvx"].isDouble()) {
			QTableWidgetItem *item = ui.tableWidget_config->item(3, 0);
			item->setText(QString::number(parameters["rotation rbvx"].toDouble()));
		}
		if (parameters["rotation rbvy"].isDouble()) {
			QTableWidgetItem *item = ui.tableWidget_config->item(3, 1);
			item->setText(QString::number(parameters["rotation rbvy"].toDouble()));
		}
		if (parameters["rotation rbvz"].isDouble()) {
			QTableWidgetItem *item = ui.tableWidget_config->item(3, 2);
			item->setText(QString::number(parameters["rotation rbvz"].toDouble()));
		}
		//sec lever arm
		if (parameters["sec lever arm x"].isDouble()) {
			QTableWidgetItem *item = ui.tableWidget_config->item(4, 0);
			item->setText(QString::number(parameters["sec lever arm x"].toDouble()));
		}
		else {
			QTableWidgetItem *item = ui.tableWidget_config->item(4, 0);
			item->setText("0.0");
		}
		if (parameters["sec lever arm y"].isDouble()) {
			QTableWidgetItem *item = ui.tableWidget_config->item(4, 1);
			item->setText(QString::number(parameters["sec lever arm y"].toDouble()));
		}
		else {
			QTableWidgetItem *item = ui.tableWidget_config->item(4, 1);
			item->setText("0.0");
		}
		if (parameters["sec lever arm z"].isDouble()) {
			QTableWidgetItem *item = ui.tableWidget_config->item(4, 2);
			item->setText(QString::number(parameters["sec lever arm z"].toDouble()));
		}
		else {
			QTableWidgetItem *item = ui.tableWidget_config->item(4, 2);
			item->setText("0.0");
		}
	}
	else {
		QMessageBox::critical(NULL, "Error", "The Json File don't have the key \"parameters\"!");
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
		//odoLeverarm
		QJsonArray odoLeverarm = configJson["odoLeverArm"].toArray();
		if (odoLeverarm.size() == DIMENSION) {
			for (int i = 0; i < DIMENSION; i++) {
				double odoLeverarm_value = odoLeverarm[i].toDouble();
				QTableWidgetItem *item = new QTableWidgetItem(QString::number(odoLeverarm_value));
				ui.tableWidget_config->setItem(1, i, item);
			}
		}
		//userLeverArm
		QJsonArray userLeverArm = configJson["userLeverArm"].toArray();
		if (userLeverArm.size() == DIMENSION) {
			for (int i = 0; i < DIMENSION; i++) {
				double userLeverArm_value = userLeverArm[i].toDouble();
				QTableWidgetItem *item = new QTableWidgetItem(QString::number(userLeverArm_value));
				ui.tableWidget_config->setItem(2, i, item);
			}
		}
		//rotationRBV
		QJsonArray rotationRBV = configJson["rotationRBV"].toArray();
		if (rotationRBV.size() == DIMENSION) {
			for (int i = 0; i < DIMENSION; i++) {
				double rotationRBV_value = rotationRBV[i].toDouble();
				QTableWidgetItem *item = new QTableWidgetItem(QString::number(rotationRBV_value));
				ui.tableWidget_config->setItem(3, i, item);
			}
		}
		//secLeverArm
		QJsonArray secLeverArm = configJson["secLeverArm"].toArray();
		if (secLeverArm.size() == DIMENSION) {
			for (int i = 0; i < DIMENSION; i++) {
				double secLeverArm_value = secLeverArm[i].toDouble();
				QTableWidgetItem *item = new QTableWidgetItem(QString::number(secLeverArm_value));
				ui.tableWidget_config->setItem(4, i, item);
			}
		}
	}
}

void MountAngleCalculation::SaveJsonFile()
{
	QJsonObject& configJson = ConfigFile::getInstance()->getConfig();
	QString ProcessFilePath = ui.process_filepath_edt->text();
	QFileInfo fileinfo(ProcessFilePath);
	QString file_dir = fileinfo.absoluteDir().absolutePath();
	QString result_dir = file_dir + QDir::separator() + "resultsodo";
	QString outputFileName = result_dir + QDir::separator() + "post";
	configJson["procfileName"] = ProcessFilePath;
	configJson["outputfileName"] = outputFileName;

	configJson["gnssfileName"] = ui.gnssposvel_filepath_edt->text();
	configJson["insfileName"] = m_DecodeImuFilePath;
	configJson["odofileName"] = m_DecodeOdoFilePath;
	configJson["initfile"] = m_DecodeInsFilePath;

	configJson["rovfileName"] = ui.rover1_filepath_edt->text();
	configJson["rov2fileName"] = ui.rover2_filepath_edt->text();
	configJson["reffileName"] = ui.base_filepath_edt->text();
	//priLeverArm
	QJsonArray priLeverArm;
	for (int i = 0; i < DIMENSION; i++) {
		QTableWidgetItem *item = ui.tableWidget_config->item(0, i);
		priLeverArm.append(item->text().toDouble());
	}
	configJson["priLeverArm"] = priLeverArm;
	//odoLeverarm
	QJsonArray odoLeverarm;
	for (int i = 0; i < DIMENSION; i++) {
		QTableWidgetItem *item = ui.tableWidget_config->item(1, i);
		odoLeverarm.append(item->text().toDouble());
	}
	configJson["odoLeverArm"] = odoLeverarm;
	//userLeverArm
	QJsonArray userLeverArm;
	for (int i = 0; i < DIMENSION; i++) {
		QTableWidgetItem *item = ui.tableWidget_config->item(2, i);
		userLeverArm.append(item->text().toDouble());
	}
	configJson["userLeverArm"] = userLeverArm;
	//rotationRBV
	QJsonArray rotationRBV;
	for (int i = 0; i < DIMENSION; i++) {
		QTableWidgetItem *item = ui.tableWidget_config->item(3, i);
		rotationRBV.append(item->text().toDouble());
	}
	configJson["rotationRBV"] = rotationRBV;
	//secLeverArm
	QJsonArray secLeverArm;
	int isUseDuaAnt = 0;
	for (int i = 0; i < DIMENSION; i++) {
		QTableWidgetItem *item = ui.tableWidget_config->item(4, i);
		if (item->text().toDouble() != 0.0) {
			isUseDuaAnt = 1;
		}
		secLeverArm.append(item->text().toDouble());
	}
	configJson["secLeverArm"] = secLeverArm;
	configJson["isUseDuaAnt"] = isUseDuaAnt;
	configJson["isDualAntBaselineCalibration"] = 0;
	configJson["dualAntBaselineLenth"] = 0.0;
	configJson["gnssHeadingCalibration"] = 0.0;
	ConfigFile::getInstance()->writeConfigFile();
}

void MountAngleCalculation::SaveNewJsonFile(QString file_path){
	QJsonObject& configJson = ConfigFile::getInstance()->getConfig();
	stAngle& avg_angle = m_CalculationThread->getAvgAngle();
	configJson["isDualAntBaselineCalibration"] = avg_angle.movbs_length != 0.0? 1:0;
	configJson["dualAntBaselineLenth"] = avg_angle.movbs_length;
	configJson["gnssHeadingCalibration"] = avg_angle.gnssposvel_movbs_diff;
	ConfigFile::getInstance()->writeNewConfigFile(file_path);
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

void MountAngleCalculation::setProcessOperable(bool enable) {
	ui.process_files_widget->setEnabled(enable);
}

QString MountAngleCalculation::ConvertNovatelPosType(QString novateType)
{
	QString pos_type = "";
	if (novateType == "16" || novateType == "53") {
		pos_type = "1";
	}
	else if (novateType == "17" || novateType == "54") {
		pos_type = "2";
	}
	else if (novateType == "50" || novateType == "56") {
		pos_type = "4";
	}
	else if (novateType == "34" || novateType == "55") {
		pos_type = "5";
	}
	return pos_type;
}

void MountAngleCalculation::FindRtcmFilesInSameDir(QString file_path)
{
	onClearAll();
	QString current_path = ".";
	if (!file_path.isEmpty()) {
		current_path = QFileInfo(file_path).absoluteDir().absolutePath();
	}
	QDir dir(current_path);
	QFileInfoList fileInfoList = dir.entryInfoList(QDir::Files | QDir::Dirs | QDir::NoDotAndDotDot);
	ui.base_filepath_edt->setText("");
	ui.rover2_filepath_edt->setText("");
	ui.rover1_filepath_edt->setText("");
	foreach(auto fileInfo, fileInfoList) {
		if (fileInfo.isFile())
		{
			QString current_file_path = fileInfo.absoluteFilePath();
			QString file_name = fileInfo.fileName();
			if (file_name.endsWith(".bin")) {
				if (file_name.startsWith("rtcm_base_")) {
					ui.base_filepath_edt->setText(current_file_path);
				}
				else if (file_name.startsWith("rtcm_rover2_")) {
					ui.rover2_filepath_edt->setText(current_file_path);
				}
				else if (file_name.startsWith("rtcm_rover1_")) {
					ui.rover1_filepath_edt->setText(current_file_path);
				}
				else if (file_name.startsWith("rtcm_rover_")) {
					ui.rover1_filepath_edt->setText(current_file_path);
				}
			}
		}
		if (fileInfo.isDir()) {
			QString current_file_path = fileInfo.absoluteFilePath();
			QString file_name = fileInfo.fileName();
			if (file_name.endsWith("_d")) {
				FindDocodeFiles(current_file_path);
			}
		}
	}
}

void MountAngleCalculation::FindDocodeFiles(QString decoderFilesDir)
{
	QDir dir(decoderFilesDir);
	if (!dir.exists()) {
		return;
	}
	QFileInfoList fileInfoList = dir.entryInfoList(QDir::Files | QDir::NoDotAndDotDot);
	ui.gnssposvel_filepath_edt->setText("");
	ui.movbs_filepath_edt->setText("");
	ui.process_filepath_edt->setText("");
	m_DecodeImuFilePath = "";
	m_DecodeOdoFilePath = "";
	m_DecodeInsFilePath = "";
	foreach(auto fileInfo, fileInfoList) {
		if (fileInfo.isFile())
		{
			QString current_file_path = fileInfo.absoluteFilePath();
			QString file_name = fileInfo.fileName();
			if (file_name.endsWith("_process")) {
				ui.process_filepath_edt->setText(current_file_path);
			}
			else if (file_name.endsWith("_gnssposvel.txt")) {
				ui.gnssposvel_filepath_edt->setText(current_file_path);
			}
			else if (file_name.endsWith("_movbs.txt")) {
				ui.movbs_filepath_edt->setText(current_file_path);
			}
			else if (file_name.endsWith("_imu.txt")) {
				m_DecodeImuFilePath = current_file_path;
			}
			else if (file_name.endsWith("_odo.txt")) {
				m_DecodeOdoFilePath = current_file_path;
			}
			else if (file_name.endsWith("_ins.txt")) {
				m_DecodeInsFilePath = current_file_path;
			}
		}
	}
	FindResultFiles();
}

void MountAngleCalculation::FindResultFiles() {
	QString ProcessFilePath = ui.process_filepath_edt->text();
	QFileInfo fileinfo(ProcessFilePath);
	QString file_dir = fileinfo.absoluteDir().absolutePath();
	QString result_dir = file_dir + QDir::separator() + "resultsodo";
	QDir dir(result_dir);
	if (!dir.exists()) {
		return;
	}
	ui.post_ins_filepath_edt->setText("");
	ui.post_movbs_filepath_edt->setText("");
	QFileInfoList fileInfoList = dir.entryInfoList(QDir::Files | QDir::NoDotAndDotDot | QDir::Dirs);
	foreach(auto fileInfo, fileInfoList) {
		if (fileInfo.isFile())
		{
			if (fileInfo.absoluteFilePath().endsWith("_ins_odo.txt")) {
				ui.post_ins_filepath_edt->setText(fileInfo.absoluteFilePath());
			}
			else  if (fileInfo.absoluteFilePath().endsWith("_mvb.csv")) {
				ui.post_movbs_filepath_edt->setText(fileInfo.absoluteFilePath());
			}
		}
	}
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
	FindRtcmFilesInSameDir(path);
}

void MountAngleCalculation::onSelectNovatelInsFileClicked()
{
	QString current_path = ".";
	QString file_name = ui.novatel_ins_path_edt->text();
	if (!file_name.isEmpty()) {
		current_path = QDir(file_name).absolutePath();
	}
	QString path = QFileDialog::getOpenFileName(this, tr("Open Files"), current_path, tr("Data Files(*.* )"));
	if (path.length() == 0) {
		return;
	}
	ui.novatel_ins_path_edt->setText(path);
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

void MountAngleCalculation::onSelectResultFileClicked()
{
	QString current_path = ".";
	QString file_name = ui.post_ins_filepath_edt->text();
	if (!file_name.isEmpty()) {
		current_path = QDir(file_name).absolutePath();
	}
	QString path = QFileDialog::getOpenFileName(this, tr("Open Files"), current_path, tr("Data Files(*.* )"));
	if (path.length() == 0) {
		return;
	}
	ui.post_ins_filepath_edt->setText(path);
}

void MountAngleCalculation::onSaveClicked() {
	SaveJsonFile();
}

void MountAngleCalculation::onConvertClicked()
{
	QString filename = ui.novatel_ins_path_edt->text();
	if (filename.isEmpty()) {
		return;
	}
	QString ins_result_filename = filename;
	if (filename.endsWith("_ins.txt")) {
		ins_result_filename = ins_result_filename.replace("_ins.txt", "_process_ins.txt");
	}
	else {
		int index = ins_result_filename.lastIndexOf(".");
		ins_result_filename = ins_result_filename.left(index);
		ins_result_filename = ins_result_filename + "_process_ins.txt";
	}
	QFile novatel_ins_file(filename);
	QFile ins_result_file(ins_result_filename);
	if (novatel_ins_file.open(QIODevice::ReadOnly | QIODevice::Text)) {
		if (ins_result_file.open(QIODevice::WriteOnly | QIODevice::Text)) {
			char cline[256] = { 0 };
			while (!novatel_ins_file.atEnd()) {
				novatel_ins_file.readLine(cline, 256);
				QString line(cline);
				line = line.trimmed();
				QStringList items = line.split(",");
				if (items.size() == 13) {
					QStringList new_items;
					for (int i = 0; i < 11; i++) {
						new_items.append(items[i]);
					}
					for (int i = 11; i < 17;i++) {
						new_items.append("0.00000");
					}
					for (int i = 17; i < 19; i++) {
						new_items.append("0");
					}
					new_items.append(items[11]);
					QString pos_type = ConvertNovatelPosType(items[12]);
					new_items.append(pos_type);
					new_items.append("0");
					new_items.append("1.00000");
					QString new_line = new_items.join(',');
					new_line.append("\n");
					ins_result_file.write(new_line.toLocal8Bit());
				}
			}
			ui.post_ins_filepath_edt->setText(ins_result_filename);
			ins_result_file.close();
		}
		novatel_ins_file.close();
	}
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
	setOperable(false);
	m_SimpleDecodeThread->start();
}

void MountAngleCalculation::onDecodeFinished()
{
	QString decoderFilesDir = m_SimpleDecodeThread->getOutDir();
	FindDocodeFiles(decoderFilesDir);
	setOperable(true);
}

void MountAngleCalculation::onProcessClicked()
{
	QString ProcessFilePath = ui.process_filepath_edt->text();
	if (ProcessFilePath.isEmpty()) {
		return;
	}
	SaveJsonFile();
	CalculationCall::call_ins_start("./openins.json");
	FindResultFiles();
}

void MountAngleCalculation::onSplitClicked()
{
	QString InsResultFilePath = ui.post_ins_filepath_edt->text();
	if (InsResultFilePath.isEmpty()) {
		return;
	}
	if (m_LoadInsTextFileThread->isRunning())
	{
		m_LoadInsTextFileThread->stop();
		return;
	}
	m_LoadInsTextFileThread->setFileName(InsResultFilePath);
	setProcessOperable(false);
	m_LoadInsTextFileThread->start();
}

void MountAngleCalculation::onSplitFinished()
{
	ui.time_slices_comb->clear();
	std::vector<stTimeSlice>& time_slices = m_LoadInsTextFileThread->get_time_slices();
	for (int i = 0; i < time_slices.size(); i++) {
		QString time_str = QString::asprintf("%d:%d:%d:%d", time_slices[i].week, time_slices[i].starttime / 1000, time_slices[i].endtime / 1000, time_slices[i].during / 1000);
		ui.time_slices_comb->addItem(time_str);
	}
	setProcessOperable(true);
	//Êä³öÇÐ¸îÎÄ¼þ
	QString InsResultFilePath = ui.post_ins_filepath_edt->text();
	QFileInfo result_file(InsResultFilePath);
	if (!result_file.isFile()) return;
	QString basename = result_file.completeBaseName();
	QString absoluteDir = result_file.absoluteDir().absolutePath();
	QString file_name = absoluteDir + QDir::separator() + basename + "_split_time.txt";
	QFile file(file_name);
	if (file.open(QIODevice::WriteOnly | QIODevice::Text)) {
		for (int i = 0; i < time_slices.size(); i++) {
			file.write(QString::asprintf(
				"%d,%8d,%8d,%3d,"
				"%9.3f,%9.3f,%2d,"
				"%9.3f,%9.3f,%9.3f,%9.3f\n",
				time_slices[i].week, time_slices[i].starttime / 1000, time_slices[i].endtime / 1000, time_slices[i].during / 1000,
				time_slices[i].distance, time_slices[i].speed_distance, time_slices[i].used_in_roll,
				time_slices[i].start_heading, time_slices[i].end_heading, time_slices[i].last_heading, time_slices[i].angle_diff).toLocal8Bit());
		}
		file.close();
	}
}

void MountAngleCalculation::onCalculateOneClicked() {
	//check post_ins_filepath
	QString PostInsFilePath = ui.post_ins_filepath_edt->text();
	if (PostInsFilePath.isEmpty()) {
		QMessageBox::warning(NULL, tr("warnning"), tr("Post ins file path is Empty."));
		return;
	}
	QFileInfo post_ins_file(PostInsFilePath);
	if (!post_ins_file.isFile()) {
		QMessageBox::warning(NULL, tr("warnning"), tr("Post ins file path is not exists."));
		return;
	}
	//check gnssposvel_filepath
	QString gnssposvel_file = ui.gnssposvel_filepath_edt->text();
	if (gnssposvel_file.isEmpty()) {
		QMessageBox::warning(NULL, tr("warnning"), tr("Gnss posvel file path is Empty."));
		return;
	}
	QFileInfo gnss_file(gnssposvel_file);
	if (!gnss_file.isFile()) {
		QMessageBox::warning(NULL, tr("warnning"), tr("Gnss posvel file path is not exists."));
		return;
	}

	QString week_str = ui.week_edt->text();
	QString starttime_str = ui.starttime_edt->text();
	QString endtime_str = ui.endtime_edt->text();
	if (week_str.isEmpty())return;
	if (starttime_str.isEmpty())return;
	if (endtime_str.isEmpty())return;

	//running
	if (m_CalculationThread->isRunning())
	{
		QMessageBox::warning(NULL, tr("warnning"), tr("Calculate is running."));
		return;
	}
	m_CalculationThread->setRunMode(emRunMode_One);
	m_CalculationThread->setTimeRange(week_str, starttime_str, endtime_str);
	m_CalculationThread->setPostInsFilePath(ui.post_ins_filepath_edt->text());
	m_CalculationThread->setGnssposvelFilePath(ui.gnssposvel_filepath_edt->text());
	m_CalculationThread->setPostMovbsFilePath(ui.post_movbs_filepath_edt->text());
	m_CalculationThread->start();
	setProcessOperable(false);
}

void MountAngleCalculation::onCalculateAllClicked() {
	if (ui.time_slices_comb->count() == 0) {
		QMessageBox::warning(NULL, tr("warnning"), tr("No time slices."));
		return;
	}
	//check post_ins_filepath
	QString PostInsFilePath = ui.post_ins_filepath_edt->text();
	if (PostInsFilePath.isEmpty()) {
		QMessageBox::warning(NULL, tr("warnning"), tr("Post ins file path is Empty."));
		return;
	}
	QFileInfo post_ins_file(PostInsFilePath);
	if (!post_ins_file.isFile()) {
		QMessageBox::warning(NULL, tr("warnning"), tr("Post ins file path is not exists."));
		return;
	}
	//check gnssposvel_filepath
	QString gnssposvel_file = ui.gnssposvel_filepath_edt->text();
	if (gnssposvel_file.isEmpty()) {
		QMessageBox::warning(NULL, tr("warnning"), tr("Gnss posvel file path is Empty."));
		return;
	}
	QFileInfo gnss_file(gnssposvel_file);
	if (!gnss_file.isFile()) {
		QMessageBox::warning(NULL, tr("warnning"), tr("Gnss posvel file path is not exists."));
		return;
	}
	//running
	if (m_CalculationThread->isRunning())
	{
		QMessageBox::warning(NULL, tr("warnning"), tr("Calculate is running."));
		return;
	}
	m_CalculationThread->setRunMode(emRunMode_All);
	m_CalculationThread->setPostInsFilePath(ui.post_ins_filepath_edt->text());
	m_CalculationThread->setGnssposvelFilePath(ui.gnssposvel_filepath_edt->text());
	m_CalculationThread->setPostMovbsFilePath(ui.post_movbs_filepath_edt->text());
	m_CalculationThread->start();
	setProcessOperable(false);
}

void MountAngleCalculation::onCalculateFinished()
{
	setProcessOperable(true);
	stAngle& avg_angle = m_CalculationThread->getAvgAngle();
	double rotationRBV[DIMENSION] = { 0 };
	for (int i = 0; i < DIMENSION; i++) {
		QTableWidgetItem *item = ui.tableWidget_config->item(3, i);
		rotationRBV[i] = item->text().toDouble();
	}
	ui.offset_edt->setText(QString::asprintf("%.3f,%.3f,%.3f", avg_angle.roll, avg_angle.pitch, avg_angle.heading));
	ui.dua_ant_edt->setText(QString::asprintf("%.3f,%.3f", avg_angle.gnssposvel_movbs_diff, avg_angle.movbs_length));
	ui.result_edt->setText(QString::asprintf("%.3f,%.3f,%.3f", rotationRBV[0] - avg_angle.roll, rotationRBV[1] - avg_angle.pitch, rotationRBV[2] - avg_angle.heading));
}

void MountAngleCalculation::onAverageClicked() {
	//running
	if (m_CalculationThread->isRunning())
	{
		QMessageBox::warning(NULL, tr("warnning"), tr("Calculate is running."));
		return;
	}
	m_CalculationThread->setRunMode(emRunMode_Avg);
	m_CalculationThread->setPostInsFilePath(ui.post_ins_filepath_edt->text());
	m_CalculationThread->setGnssposvelFilePath(ui.gnssposvel_filepath_edt->text());
	m_CalculationThread->setPostMovbsFilePath(ui.post_movbs_filepath_edt->text());
	m_CalculationThread->start();
	setProcessOperable(false);
}

void MountAngleCalculation::onTimeSlicesChanged(const QString & time_str) {
	QStringList time_sp = time_str.split(':');
	if (time_sp.size() >= 2) {
		ui.week_edt->setText(time_sp[0]);
		ui.starttime_edt->setText(time_sp[1]);
		ui.endtime_edt->setText(time_sp[2]);
	}
}

void MountAngleCalculation::onDecodingProcess(int present, int msecs)
{
	ui.progressBar->setValue(present);
	double dProgress = ui.progressBar->value() * 100.0 / ui.progressBar->maximum();
	ui.progressBar->setFormat(QString("%1%").arg(QString::number(dProgress, 'f', 2)));
	m_TimeShow.setHMS(0, 0, 0, 0);
	ui.time_lb->setText(m_TimeShow.addMSecs(msecs).toString("hh:mm:ss:zzz"));
}

void MountAngleCalculation::onSaveJsonClicked()
{
	QString user_file_path = ui.filepath_edt->text();
	QFileInfo file_info(user_file_path);
	if (!file_info.isFile()) {
		QMessageBox::warning(NULL, tr("warnning"), tr("Please select a file to calculate."));
		return;
	}
	QString file_dir = file_info.absoluteDir().absolutePath();
	QString new_json_file_path = file_dir + QDir::separator() + "openins.json";
	new_json_file_path = QFileDialog::getSaveFileName(this,
		tr("Save Config"),
		new_json_file_path,
		tr("Config Files (*.json)"));
	if (!new_json_file_path.isNull())
	{
		SaveNewJsonFile("./temporary.json");
		CalculationCall::call_json_translate(new_json_file_path);
	}
}

void MountAngleCalculation::onClearAll()
{
	m_DecodeImuFilePath = "";
	m_DecodeOdoFilePath = "";
	m_DecodeInsFilePath = "";

	ui.base_filepath_edt->clear();
	ui.rover2_filepath_edt->clear();
	ui.rover1_filepath_edt->clear();

	ui.gnssposvel_filepath_edt->clear();
	ui.movbs_filepath_edt->clear();
	ui.process_filepath_edt->clear();

	ui.post_ins_filepath_edt->clear();
	ui.post_movbs_filepath_edt->clear();

	ui.time_slices_comb->clear();
	m_LoadInsTextFileThread->init();
	ui.week_edt->clear();
	ui.starttime_edt->clear();
	ui.endtime_edt->clear();

	ui.offset_edt->clear();
	ui.dua_ant_edt->clear();
	ui.result_edt->clear();

}
