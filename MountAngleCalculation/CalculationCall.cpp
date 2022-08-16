#include "CalculationCall.h"
#include <QMessageBox>

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

void CalculationCall::call_gnss_calc_heading(const char *file_name, const char *start_time, const char *end_time) {
	try
	{
		QString cmd = QString::asprintf("calc_heading.exe \"%s\" %s %s", file_name, start_time, end_time);
		cmd.replace("/", "\\");
		system(cmd.toLocal8Bit().data());
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

void CalculationCall::call_dr_mountangle_start(const char *file_name, const char *week, const char *start_time, const char *end_time)
{
	//改成调用exe
	try
	{
		//QString cmd = QString::asprintf("DR_MountAngle.exe %s %s %s", file_name, start_time, end_time);
		QString cmd = QString::asprintf("solveMisalign.exe -t \"%s\" -rng %s %s %s", file_name, start_time, end_time, week);
		system(cmd.toLocal8Bit().data());
		//DR_DLL = LoadLibrary(L"DR_MountAngle");
		//dr_mountangle_start = (dr_mountangle_start_f)GetProcAddress(DR_DLL, "dr_mountangle_start");
		//dr_mountangle_start(file_name, start_time, end_time);
		//dr_mountangle_start = NULL;
		//FreeLibrary(DR_DLL);
	}
	catch (std::exception& e)
	{
		QMessageBox::information(NULL, "Information", e.what());
	}
}
