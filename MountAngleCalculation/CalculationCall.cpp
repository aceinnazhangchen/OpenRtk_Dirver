#include "CalculationCall.h"
#include <QMessageBox>
#include <QProcess>

HMODULE CalculationCall::INS_DLL;
HMODULE CalculationCall::DR_DLL;
ins_start_f CalculationCall::ins_start = NULL;
dr_mountangle_start_f CalculationCall::dr_mountangle_start = NULL;

CalculationCall::CalculationCall(QObject *parent)
	: QObject(parent)
{
}

CalculationCall::~CalculationCall()
{
}

void CalculationCall::call_gnss_calc_heading(QString file_name, QString start_time, QString end_time) {
	try
	{
		//QString cmd = QString::asprintf("calc_heading.exe \"%s\" %s %s", file_name, start_time, end_time);
		//cmd.replace("/", "\\");
		//system(cmd.toLocal8Bit().data());
		QString filename = file_name;
		filename.replace("/", "\\");
		QProcess process;
		process.startDetached("calc_heading.exe", QStringList() << filename << start_time << end_time);//启动可执行文件
		process.waitForFinished();
	}
	catch (std::exception& e)
	{
		QMessageBox::information(NULL, "Information", e.what());
	}
}

void CalculationCall::call_ins_start(const char * file_name)
{
	//改成调用exe
	try
	{
		QString cmd = QString::asprintf("INS.exe \"%s\"", file_name);
		system(cmd.toLocal8Bit().data());
		//INS_DLL = LoadLibrary(L"INS.dll");
		//ins_start = (ins_start_f)GetProcAddress(INS_DLL, "ins_start");
		//ins_start(file_name, type);
		//ins_start = NULL;
		//FreeLibrary(INS_DLL);
	}
	catch (std::exception& e)
	{
		QMessageBox::information(NULL, "Information", e.what());
	}
}

void CalculationCall::call_dr_mountangle_start(QString file_name, QString week, QString start_time, QString end_time,QString outprefix)
{
	//改成调用exe
	try
	{
		//QString cmd = QString::asprintf("solveMisalign.exe -t \"%s\" -rng %s %s %s", file_name, start_time, end_time, week);
		//system(cmd.toLocal8Bit().data());
		QProcess process;
		process.startDetached("solveMisalign.exe", QStringList() << "-t" << file_name << "-rng" << start_time << end_time << week << "-o" << outprefix);//启动可执行文件
		process.waitForFinished();
	}
	catch (std::exception& e)
	{
		QMessageBox::information(NULL, "Information", e.what());
	}
}

void CalculationCall::call_json_translate(QString file_name)
{
	try
	{
		QProcess process;
		process.startDetached("json_translate.exe", QStringList() << file_name);//转换json
		process.waitForFinished();
	}
	catch (std::exception& e)
	{
		QMessageBox::information(NULL, "Information", e.what());
	}
}

