#include "MountAngleCalculation.h"
#include "ConfigFile.h"
#include <QProgressBar>
#include <QFileDialog>
#include <QMessageBox>
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
	m_JsonFileLoader = new JsonFileLoader(this);
	LoadJsonFile();
	ui.tableWidget_config->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
	ui.tableWidget_config->verticalHeader()->setSectionResizeMode(QHeaderView::Stretch);
	//connect(ui.pushButton_save, SIGNAL(clicked()), this, SLOT(onSaveClicked()));
	connect(ui.select_btn, SIGNAL(clicked()), this, SLOT(onSelectFileClicked()));
	connect(ui.novatel_ins_convert_btn, SIGNAL(clicked()), this, SLOT(onConvertClicked()));
	connect(ui.pushButton_decode, SIGNAL(clicked()), this, SLOT(onDecodeClicked()));
	connect(ui.pushButton_process, SIGNAL(clicked()), this, SLOT(onProcessClicked()));
	connect(ui.pushButton_split, SIGNAL(clicked()), this, SLOT(onSplitClicked()));
	connect(ui.novatel_ins_select_btn, SIGNAL(clicked()), this, SLOT(onSelectNovatelInsFileClicked()));
	connect(ui.process_select_btn, SIGNAL(clicked()), this, SLOT(onSelectProcessFileClicked()));
	connect(ui.result_select_btn, SIGNAL(clicked()), this, SLOT(onSelectResultFileClicked()));
	connect(ui.pushButton_calculate, SIGNAL(clicked()), this, SLOT(onCalculateClicked()));
	connect(ui.pushButton_calculate_all, SIGNAL(clicked()), this, SLOT(onCalculateAllClicked()));
	connect(this, SIGNAL(sgnCalculateNext()), this, SLOT(onCalculateNext()));
	connect(ui.time_slices_comb, SIGNAL(currentIndexChanged(const QString &)), this, SLOT(onTimeSlicesChanged(const QString &)));
	connect(m_SimpleDecodeThread, SIGNAL(sgnProgress(int, int)), this, SLOT(onProcess(int, int)));
	connect(m_SimpleDecodeThread, SIGNAL(sgnFinished()), this, SLOT(onDecodeFinished()));
	
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
	else if (ui.novatel_ins_path_edt->isEnabled() && ui.novatel_ins_path_edt->geometry().contains(event->pos())) {
		QString name = event->mimeData()->urls().first().toLocalFile();
		ui.novatel_ins_path_edt->setText(name);
	}
	else if (ui.tableWidget_config->isEnabled() && ui.tableWidget_config->geometry().contains(event->pos())) {
		QString name = event->mimeData()->urls().first().toLocalFile();
		LoadConfigureJsonFile(name);
	}
	else if (ui.gnss_filepath_edt->isEnabled() && ui.gnss_filepath_edt->geometry().contains(event->pos())) {
		QString name = event->mimeData()->urls().first().toLocalFile();
		ui.gnss_filepath_edt->setText(name);
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
	else if (ui.result_filepath_edt->isEnabled() && ui.result_filepath_edt->geometry().contains(event->pos())) {
		QString name = event->mimeData()->urls().first().toLocalFile();
		ui.result_filepath_edt->setText(name);
	}
}

void MountAngleCalculation::LoadConfigureJsonFile(QString filename)
{
	if (m_JsonFileLoader == NULL) return;
	if (m_JsonFileLoader->readJsonArrayFile(filename) == false) return;
	ui.label_9->setText(filename);
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
	}
}

void MountAngleCalculation::SaveJsonFile()
{
	QJsonObject& configJson = ConfigFile::getInstance()->getConfig();
	configJson["procFileName"] = m_ProcessFilePath;
	configJson["outputFileName"] = m_ProcessFilePath;
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
	ui.result_filepath_edt->setEnabled(enable);
	ui.process_select_btn->setEnabled(enable);
	ui.pushButton_split->setEnabled(enable);
}

void MountAngleCalculation::readAngleFromFile(QString file_path)
{
	QFileInfo out_file(file_path);
	if (!out_file.isFile()) return;
	FILE* f_out = fopen(file_path.toLocal8Bit().data(), "r");
	if (f_out) {
		char line[256] = { 0 };
		stAngle angle = { 0 };
		int index = 0;
		while (fgets(line, 256, f_out) != 0){
			index++;
			if (index == 2) {
				QString value = QString(line).trimmed();
				value = value.left(value.size() - 1);
				angle.roll = value.toFloat();
			}
			if (index == 3) {
				QString value = QString(line).trimmed();
				value = value.left(value.size() - 1);
				angle.pitch = value.toFloat();
			}
			if (index == 4) {
				QString value = QString(line).trimmed();
				angle.heading = value.toFloat();
			}
		}
		angle_list.push_back(angle);
		double rotationRBV[DIMENSION] = { 0 };
		for (int i = 0; i < DIMENSION; i++) {
			QTableWidgetItem *item = ui.tableWidget_config->item(3, i);
			rotationRBV[i] = item->text().toDouble();
		}
		ui.offset_edt->setText(QString::asprintf("%.3f,%.3f,%.3f", angle.roll, angle.pitch, angle.heading));
		ui.result_edt->setText(QString::asprintf("%.3f,%.3f,%.3f", rotationRBV[0]-angle.roll, rotationRBV[1]-angle.pitch, rotationRBV[2]-angle.heading));
		fclose(f_out);
	}
}

void MountAngleCalculation::readAngleFromFile_1(QString file_path) {
	QFileInfo out_file(file_path);
	if (!out_file.isFile()) return;
	FILE* f_out = fopen(file_path.toLocal8Bit().data(), "r");
	char line[256] = { 0 };
	if (f_out) {
		ui.result_edt->setText(file_path);
		fgets(line, 256, f_out);//先读一行
		size_t line_size = strlen(line);//获取一行的长度
		fseek(f_out, -(line_size + 1), SEEK_END);//定位到文件最后,并且向前偏移一行的长度加1(因为最后有个空行)
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

void MountAngleCalculation::readHeadingFromGnss(QString file_path)
{
	QFileInfo out_file(file_path);
	if (!out_file.isFile()) return;
	FILE* f_out = fopen(file_path.toLocal8Bit().data(), "r");
	char line[256] = { 0 };
	if (f_out) {
		while (fgets(line, 256, f_out) != NULL) {
		}
		if (angle_list.size() > 0) {
			angle_list[angle_list.size()-1].gnss_heading = atof(line);//读取最后一行
		}
		fclose(f_out);
	}
}

void MountAngleCalculation::readHeadingFromIns(QString file_path) {
	QFileInfo out_file(file_path);
	if (!out_file.isFile()) return;
	FILE* f_out = fopen(file_path.toLocal8Bit().data(), "r");
	if (f_out) {
		char line[256] = { 0 };
		int line_num = 0;
		int time_index = 0;
		double sum_heading = 0.0;
		double sum_roll = 0.0;
		int heading_num = 0;
		int roll_num = 0;
		std::vector<stTimeSlice>& time_slices = m_LoadInsTextFileThread->get_time_slices();
		while (fgets(line, 256, f_out) != NULL) {
			line_num++;
			if (line_num == 1) continue;
			QString q_line(line);
			QStringList items = q_line.split(",");
			if (items.size() < 13) continue;
			uint32_t time = (uint32_t)(items[1].trimmed().toDouble()*1000);
			if (time < time_slices[time_index].starttime)
				continue;
			if (time > time_slices[time_index].endtime) {
				if (time_index < angle_list.size()) {
					if (heading_num > 0) {
						angle_list[time_index].ins_heading = sum_heading / heading_num;
						if (angle_list[time_index].ins_heading < 0) {
							angle_list[time_index].ins_heading += 360;
						}
					}
					if (roll_num > 0) {
						angle_list[time_index].ins_roll = sum_roll / roll_num;
					}
				}
				time_index++;
				sum_heading = 0.0;
				sum_roll = 0.0;
				heading_num = 0;
				roll_num = 0;
				if (time_index >= time_slices.size())
					break;
			}
			if (time % 1000 != 0)
				continue;
			if (time >= time_slices[time_index].starttime && time < time_slices[time_index].endtime) {
				double heading = items[10].trimmed().toDouble();
				double roll = items[8].trimmed().toDouble();
				sum_heading += heading;
				heading_num++;
				sum_roll += roll;
				roll_num++;
			}
		}
		fclose(f_out);
	}
}

void MountAngleCalculation::readGnssPosVel() {
	QString file_path = ui.gnssposvel_filepath_edt->text();
	QFileInfo file_info(file_path);
	if (!file_info.isFile()) return;
	std::vector<stTimeSlice>& time_slices = m_LoadInsTextFileThread->get_time_slices();
	FILE* f_gnssposvel = fopen(file_path.toLocal8Bit().data(), "r");
	if (f_gnssposvel) {
		char line[256] = { 0 };
		int line_num = 0;
		int time_index = 0;
		double sum_heading = 0.0;
		int heading_num = 0;
		while (fgets(line, 256, f_gnssposvel) != NULL) {
			line_num++;
			QString q_line(line);
			QStringList items = q_line.split(",");
			if (items.size() < 13) 
				continue;
			uint32_t time = (uint32_t)(items[1].trimmed().toDouble() * 1000);
			if (time < time_slices[time_index].starttime)
				continue;
			if (time > time_slices[time_index].endtime) {
				if (time_index < angle_list.size()) {
					if (heading_num > 0) {
						//统计平均heading
						angle_list[time_index].gnssposvel_heading = sum_heading / heading_num;
						if (angle_list[time_index].gnssposvel_heading < 0) {
							angle_list[time_index].gnssposvel_heading += 360;
						}
					}
				}
				time_index++;//递增区间id
				sum_heading = 0.0;
				heading_num = 0;
				if (time_index >= time_slices.size())//最后一个区间计算完成
					break;
			}
			if (time % 1000 != 0)
				continue;
			if (time >= time_slices[time_index].starttime && time < time_slices[time_index].endtime) {//累计区间内的heading
				double heading = items[12].trimmed().toDouble();
				int type = items[8].trimmed().toInt();
				if (type == 4) {
					sum_heading += heading;
					heading_num++;
				}
			}
		}
		fclose(f_gnssposvel);
	}
}

void MountAngleCalculation::readMovbsTxt() {
	QString file_path = ui.movbs_filepath_edt->text();
	QFileInfo file_info(file_path);
	if (!file_info.isFile()) return;
	std::vector<stTimeSlice>& time_slices = m_LoadInsTextFileThread->get_time_slices();
	FILE* f_movbs = fopen(file_path.toLocal8Bit().data(), "r");
	if (f_movbs) {
		char line[256] = { 0 };
		int line_num = 0;
		int time_index = 0;
		double sum_heading = 0.0;
		int heading_num = 0;
		while (fgets(line, 256, f_movbs) != NULL) {
			line_num++;
			QString q_line(line);
			QStringList items = q_line.split(",");
			if (items.size() < 10)
				continue;
			uint32_t time = (uint32_t)(items[2].trimmed().toDouble() * 1000);
			if (time < time_slices[time_index].starttime)
				continue;
			if (time > time_slices[time_index].endtime) {
				if (time_index < angle_list.size()) {
					if (heading_num > 0) {
						//统计平均heading
						angle_list[time_index].movbs_heading = sum_heading / heading_num;
						if (angle_list[time_index].movbs_heading < 0) {
							angle_list[time_index].movbs_heading += 360;
						}
					}
				}
				time_index++;//递增区间id
				sum_heading = 0.0;
				heading_num = 0;
				if (time_index >= time_slices.size())//最后一个区间计算完成
					break;
			}
			//if (time % 1000 != 0)
			//	continue;
			if (time >= time_slices[time_index].starttime && time < time_slices[time_index].endtime) {//累计区间内的heading
				double heading = items[9].trimmed().toDouble();
				int type1 = items[5].trimmed().toInt();
				int type2 = items[6].trimmed().toInt();
				if (type1 == 4 && type2 == 4) {
					sum_heading += heading;
					heading_num++;
				}
			}
		}
		fclose(f_movbs);
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
	QString file_name = ui.result_filepath_edt->text();
	if (!file_name.isEmpty()) {
		current_path = QDir(file_name).absolutePath();
	}
	QString path = QFileDialog::getOpenFileName(this, tr("Open Files"), current_path, tr("Data Files(*.* )"));
	if (path.length() == 0) {
		return;
	}
	ui.result_filepath_edt->setText(path);
}

void MountAngleCalculation::onSaveClicked() {
	SaveJsonFile();
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
			ui.result_filepath_edt->setText(ins_result_filename);
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

void MountAngleCalculation::onProcessClicked()
{
	m_ProcessFilePath = ui.process_filepath_edt->text();
	if (m_ProcessFilePath.isEmpty()) {
		return;
	}
	SaveJsonFile();
	CalculationCall::call_ins_start("./content_aceinna_config.json");
	m_InsResultFilePath = m_ProcessFilePath + "_ins.txt";
	QFileInfo file(m_InsResultFilePath);
	if (file.isFile()) {
		ui.result_filepath_edt->setText(m_InsResultFilePath);
	}
}

void MountAngleCalculation::onSplitClicked()
{
	m_InsResultFilePath = ui.result_filepath_edt->text();
	if (m_InsResultFilePath.isEmpty()) {
		return;
	}
	if (m_LoadInsTextFileThread->isRunning())
	{
		m_LoadInsTextFileThread->stop();
		return;
	}
	m_LoadInsTextFileThread->setFileName(m_InsResultFilePath);
	setSplitOperable(false);
	m_LoadInsTextFileThread->start();
}

void MountAngleCalculation::onCalculateClicked() {
	QString GnssFilePath = ui.gnss_filepath_edt->text();
	if(GnssFilePath.isEmpty()) return;
	QFileInfo gnss_file(GnssFilePath);
	if (!gnss_file.isFile()) return;
	m_InsResultFilePath = ui.result_filepath_edt->text();
	QFileInfo result_file(m_InsResultFilePath);
	if (!result_file.isFile()) return;
	QString basename = result_file.completeBaseName();
	QString absoluteDir = result_file.absoluteDir().absolutePath();
	QString week_str = ui.week_edt->text();
	QString starttime_str = ui.starttime_edt->text();
	QString endtime_str = ui.endtime_edt->text();
	if (week_str.isEmpty())return;
	if (starttime_str.isEmpty())return;
	if (endtime_str.isEmpty())return;
	CalculationCall::call_dr_mountangle_start(m_InsResultFilePath.toLocal8Bit().data(), week_str.toLocal8Bit().data(), starttime_str.toLocal8Bit().data(), endtime_str.toLocal8Bit().data());
	CalculationCall::call_gnss_calc_heading(GnssFilePath.toLocal8Bit().data(), starttime_str.toLocal8Bit().data(), endtime_str.toLocal8Bit().data());
	QString out_file_path = "result001_content_misalign.txt";
	QString gnss_heading_file = "gnss_" + starttime_str + "-" + endtime_str + ".txt";
	QString gnss_path = gnss_file.absolutePath();
	QString gnss_basename = gnss_file.completeBaseName();
	QString gnss_heading_path = gnss_path + "\\" + gnss_basename + "_heading\\" + gnss_heading_file;
	readAngleFromFile(out_file_path);
	readHeadingFromGnss(gnss_heading_path);
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
		if (!m_InsResultFilePath.isEmpty()) {
			readHeadingFromIns(m_InsResultFilePath);
		}
		readGnssPosVel();
		readMovbsTxt();
		//取平均
		CalculateAverageAngle();
	}
}

void MountAngleCalculation::WriteAngleFile() 
{
	QFileInfo result_file(m_InsResultFilePath);
	if (!result_file.isFile()) return;
	QString basename = result_file.completeBaseName();
	QString absoluteDir = result_file.absoluteDir().absolutePath();
	QString file_name = absoluteDir + "/" + basename + "_Angle.txt";
	QFile file(file_name);
	if (file.open(QIODevice::WriteOnly | QIODevice::Text)) {
		file.write(QString::asprintf(
			"%4s,%6s,%6s,"
			"%6s,%8s,%8s,%4s,"
			"%8s,%8s,"
			"%7s,%7s,%7s,"
			"%8s,%8s,%10s,%7s,"
			"%8s,%8s,%14s\n",
			"week","start","end",
			"during","dis", "dis_s","turn",
			"dst_s","dst_e",
			"roll","pitch","heading",
			"gnss_h","ins_h","ins-gnss_h", "diff_h",
			"posvel_h","movbs_h","posvel-movbs_h").toLocal8Bit());
		std::vector<stTimeSlice>& time_slices = m_LoadInsTextFileThread->get_time_slices();
		if (time_slices.size() == angle_list.size()) {
			for (int i = 0; i < angle_list.size(); i++) {
				file.write(QString::asprintf(
					"%4d,%6d,%6d,"
					"%6d,%8.2f,%8.2f,%4d,"
					"%8.2f,%8.2f,"
					"%7.3f,%7.3f,%7.3f,"
					"%8.3f,%8.3f,%10.3f,%7.3f,"
					"%8.3f,%8.3f,%14.3f\n",
					time_slices[i].week, time_slices[i].starttime / 1000, time_slices[i].endtime / 1000, 
					time_slices[i].during / 1000, time_slices[i].distance, time_slices[i].speed_distance, time_slices[i].used_in_roll,
					time_slices[i].dst_s, time_slices[i].dst_e,
					angle_list[i].roll, angle_list[i].pitch, angle_list[i].heading,
					angle_list[i].gnss_heading, angle_list[i].ins_heading, angle_list[i].ins_gnss_diff,(angle_list[i].heading-angle_list[i].ins_gnss_diff),
					angle_list[i].gnssposvel_heading, angle_list[i].movbs_heading, angle_list[i].gnssposvel_movbs_diff).toLocal8Bit());
			}
		}
		file.close();
	}
}

void MountAngleCalculation::WriteAverageAngleFile(stAngle avg_angle){
	QFileInfo result_file(m_InsResultFilePath);
	if (!result_file.isFile()) return;
	QString basename = result_file.completeBaseName();
	QString absoluteDir = result_file.absoluteDir().absolutePath();
	QString file_name = absoluteDir + "/" + basename + "_AverageAngle.txt";
	QFile file(file_name);
	if (file.open(QIODevice::WriteOnly | QIODevice::Text)) {
		file.write(QString::asprintf("%7.3f,%7.3f,%7.3f,%7.3f",
			avg_angle.roll, avg_angle.pitch, avg_angle.heading, avg_angle.gnssposvel_movbs_diff).toLocal8Bit());
		file.close();
	}
}

void MountAngleCalculation::CalculateAverageAngle()
{
	std::vector<stTimeSlice>& time_slices = m_LoadInsTextFileThread->get_time_slices();
	for (int i = 0; i < angle_list.size(); i++) {
		angle_list[i].roll = angle_list[i].ins_roll;
		angle_list[i].ins_gnss_diff = angle_list[i].ins_heading - angle_list[i].gnss_heading;
		if (angle_list[i].movbs_heading != 0.0) {
			angle_list[i].gnssposvel_movbs_diff = angle_list[i].gnssposvel_heading - angle_list[i].movbs_heading;
		}
	}
	stAngle sum_angle = { 0 };
	stAngle avg_angle = { 0 };
	int num_in_cal = 0;
	int num_in_cal_roll = 0;
	int num_in_gnssposvel_movbs = 0;
	for (int i = 0; i < angle_list.size(); i++) {
		if (fabs(angle_list[i].ins_gnss_diff - angle_list[i].heading) > 1.0) continue;
		sum_angle.heading += angle_list[i].heading;
		sum_angle.pitch += angle_list[i].pitch;
		num_in_cal++;
	}
	for (int i = 0; i < angle_list.size(); i++) {
		if (time_slices[i].used_in_roll == 0 || time_slices[i].distance > 300) continue;
		sum_angle.roll += angle_list[i].roll;
		num_in_cal_roll++;
	}
	for (int i = 0; i < angle_list.size(); i++) {
		if(angle_list[i].gnssposvel_movbs_diff == 0.0) continue;
		if(fabs(angle_list[i].gnssposvel_movbs_diff) >= 5.0) continue;
		sum_angle.gnssposvel_movbs_diff += angle_list[i].gnssposvel_movbs_diff;
		num_in_gnssposvel_movbs++;
	}
	if (num_in_cal > 0) {
		avg_angle.heading = sum_angle.heading / num_in_cal;
		avg_angle.pitch = sum_angle.pitch / num_in_cal;
	}
	if (num_in_cal_roll > 0) {
		avg_angle.roll = sum_angle.roll / num_in_cal_roll;
	}
	if (num_in_gnssposvel_movbs > 0) {
		avg_angle.gnssposvel_movbs_diff = sum_angle.gnssposvel_movbs_diff / num_in_gnssposvel_movbs;
	}
	double rotationRBV[DIMENSION] = { 0 };
	for (int i = 0; i < DIMENSION; i++) {
		QTableWidgetItem *item = ui.tableWidget_config->item(3, i);
		rotationRBV[i] = item->text().toDouble();
	}
	WriteAngleFile();
	ui.offset_edt->setText(QString::asprintf("%.3f,%.3f,%.3f,%.3f", avg_angle.roll, avg_angle.pitch, avg_angle.heading, avg_angle.gnssposvel_movbs_diff));
	ui.result_edt->setText(QString::asprintf("%.3f,%.3f,%.3f", rotationRBV[0] - avg_angle.roll, rotationRBV[1] - avg_angle.pitch, rotationRBV[2] - avg_angle.heading));
	WriteAverageAngleFile(avg_angle);
}

void MountAngleCalculation::onTimeSlicesChanged(const QString & time_str) {
	QStringList time_sp = time_str.split(':');
	if (time_sp.size() >= 2) {
		ui.week_edt->setText(time_sp[0]);
		ui.starttime_edt->setText(time_sp[1]);
		ui.endtime_edt->setText(time_sp[2]);
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

void MountAngleCalculation::onDecodeFinished()
{
	m_ProcessFilePath = m_SimpleDecodeThread->getOutBaseName() + "_process";
	QString Gnss_Csv_Path;
	QString GnssPosVel_Txt_Path;
	QString Movbs_Txt_Path;
	if (ui.fileformat_cmb->currentIndex() == emDecodeFormat_Ins401){
		Gnss_Csv_Path = m_SimpleDecodeThread->getOutBaseName() + "_gnss.csv";
	}
	else if (ui.fileformat_cmb->currentIndex() == emDecodeFormat_RTK330LA) {
		Gnss_Csv_Path = m_SimpleDecodeThread->getOutBaseName() + "_gN.csv";
	}	
	GnssPosVel_Txt_Path = m_SimpleDecodeThread->getOutBaseName() + "_gnssposvel.txt";
	Movbs_Txt_Path = m_SimpleDecodeThread->getOutBaseName() + "_movbs.txt";
	ui.process_filepath_edt->setText(m_ProcessFilePath);
	ui.gnss_filepath_edt->setText(Gnss_Csv_Path);
	ui.gnssposvel_filepath_edt->setText(GnssPosVel_Txt_Path);
	ui.movbs_filepath_edt->setText(Movbs_Txt_Path);	
	setOperable(true);
}


void MountAngleCalculation::onSplitFinished()
{
	ui.time_slices_comb->clear();
	std::vector<stTimeSlice>& time_slices = m_LoadInsTextFileThread->get_time_slices();
	for (int i = 0; i < time_slices.size(); i++) {
		QString time_str = QString::asprintf("%d:%d:%d:%d", time_slices[i].week, time_slices[i].starttime / 1000, time_slices[i].endtime / 1000,time_slices[i].during / 1000);
		ui.time_slices_comb->addItem(time_str);
	}
	setSplitOperable(true);
	//输出切割文件
	QFileInfo result_file(m_InsResultFilePath);
	if (!result_file.isFile()) return;
	QString basename = result_file.completeBaseName();
	QString absoluteDir = result_file.absoluteDir().absolutePath();
	QString file_name = absoluteDir + "/" + basename + "_split_time.txt";
	QFile file(file_name);
	if (file.open(QIODevice::WriteOnly | QIODevice::Text)) {
		for (int i = 0; i < time_slices.size(); i++) {
			file.write(QString::asprintf(
				"%d,%8d,%8d,%3d,"
				"%9.3f,%9.3f,%2d,"
				"%9.3f,%9.3f,%9.3f,%9.3f\n",
				time_slices[i].week, time_slices[i].starttime / 1000, time_slices[i].endtime / 1000, time_slices[i].during / 1000,
				time_slices[i].distance, time_slices[i].speed_distance, time_slices[i].used_in_roll,
				time_slices[i].start_heading, time_slices[i].end_heading,time_slices[i].last_heading, time_slices[i].angle_diff).toLocal8Bit());
		}
		file.close();
	}

}
