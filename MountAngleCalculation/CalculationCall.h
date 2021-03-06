#pragma once

#include <QObject>
#include <windows.h>

typedef int(*ins_start_f)(const char*, int);
typedef int(*dr_mountangle_start_f)(const char*, const char*, const char*);

class CalculationCall : public QObject
{
	Q_OBJECT

public:
	CalculationCall(QObject *parent);
	~CalculationCall();
	static void call_ins_start(const char*);
	static void call_dr_mountangle_start(const char * file_name, const char * week, const char * start_time, const char * end_time);
private:
	static HMODULE INS_DLL;
	static HMODULE DR_DLL;
	static ins_start_f ins_start;
	static dr_mountangle_start_f dr_mountangle_start;
};
