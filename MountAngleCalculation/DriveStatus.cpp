#include "DriveStatus.h"
#include <memory.h>
#include <math.h>

DriveStatus::DriveStatus()
{
	init();
}

DriveStatus::~DriveStatus()
{
}

void DriveStatus::init()
{
	statusstage = 0;
	memset(&curinsresult, 0, sizeof(ins_sol_data));
	memset(&lastinsresult, 0, sizeof(ins_sol_data));
	memset(&estdata, 0, sizeof(est_t));
	memset(&liveresult, 0, sizeof(liveresult_t));
	init_patterns();
}

void DriveStatus::init_patterns()
{
	patterns.clear();
	clear_pattern();
}

void DriveStatus::clear_pattern() {
	if (pattern.starttime != -1) {
		memset(&pattern, 0, sizeof(pattern_t));
		pattern.gps_week = -1;
		pattern.starttime = -1;
		pattern.endtime = -1;
		pattern.curtime = -1;
	}
}

void DriveStatus::addrawdata(ins_sol_data* data)
{
	memcpy(&curinsresult, data,sizeof(ins_sol_data));
	if (curinsresult.gps_millisecs % 1000 < 10) {
		if (lastinsresult.gps_week > 0) {
			addestcheckdata();
			addestdata2patterns();
		}
		memcpy(&lastinsresult, &curinsresult, sizeof(ins_sol_data));
	}
}

void DriveStatus::addestcheckdata() {
	float angle = curinsresult.heading - lastinsresult.heading;
	while (angle > 180) {
		angle = angle - 360;
	}
	while (angle < -180) {
		angle = angle + 360;
	}
	double distance = sqrt(0.25*(curinsresult.north_velocity+ lastinsresult.north_velocity)*(curinsresult.north_velocity + lastinsresult.north_velocity)
		+0.25*(curinsresult.east_velocity + lastinsresult.east_velocity)*(curinsresult.east_velocity + lastinsresult.east_velocity));

	estdata.status = curinsresult.ins_status;
	estdata.postype = curinsresult.ins_position_type;
	estdata.gps_week = curinsresult.gps_week;
	estdata.time = curinsresult.gps_millisecs;
	estdata.angle = angle;
	estdata.distance = distance;
}

void DriveStatus::addestdata2patterns() {
	liveresult.type = 0;
	if (statusstage == 0) {
		if (estdata.status == 2) {
			liveresult.type = 1;
			statusstage = 1;
		}
		else if (estdata.status == 3) {
			liveresult.type = 2;
			statusstage = 2;
		}
		else if (estdata.status == 4) {
			statusstage = 0;
		}
		else {
			statusstage = 0;
		}
	}
	else if (statusstage == 1) {
		if (estdata.status == 3) {
			liveresult.type = 2;
			statusstage = 2;
		}
	}
	else if (statusstage == 2) {
		if (estdata.status == 2 || estdata.status == 3) {
			checkdatapattern(1);
			int turnflag = calpatterns(1);
			if (turnflag == 3) {
				statusstage = 3;
				liveresult.type = 12;
			}
			else if (turnflag == 2) {
				liveresult.type = 11;
			}
			else if (turnflag == 1) {
				liveresult.type = 10;
			}
			else if (turnflag == 0) {
				liveresult.type = 9;
			}
		}
		else {
			liveresult.type = 7;
			statusstage = 0;
			init_patterns();
		}
	}
	else if (statusstage == 3) {
		if (estdata.status == 2 || estdata.status == 3) {
			checkdatapattern(2);
			int lineflag = calpatterns(2);
			if (lineflag == 7) {
				liveresult.type = 15;
			}
			else if (lineflag == 6) {
				liveresult.type = 14;
			}
			else if (lineflag == 5) {
				liveresult.type = 13;
			}
		}
		else {
			liveresult.type = 8;
			liveresult.gps_week = 0;
			liveresult.starttime = 0;
			liveresult.curtime = 0;
			liveresult.distance = 0.0;
			clear_pattern();
		}
	}
}

int DriveStatus::checkdatapattern(int mode)
{
	float threshold[4] = { 0 };
	if (mode == 1) {  //turn
		threshold[0] = 5.0f;
		threshold[1] = 360.0f;
		threshold[2] = 0.0f;
		threshold[3] = 2.0f;
	}
	else if (mode == 2) { //line
		threshold[0] = 0.0f;
		threshold[1] = 5.0f;
		threshold[2] = 5.0f;
		threshold[3] = 360.0f;
	}
	int ret = 0;
	if (fabs(estdata.angle) >= threshold[0] && fabs(estdata.angle) <= threshold[1]) {
		if (mode == 1 || estdata.distance > 0.3) {
			if (pattern.starttime == -1) {
				pattern.mode = mode;
				pattern.starttime = estdata.time;
			}
			pattern.gps_week = estdata.gps_week;
			pattern.curtime = estdata.time;
			pattern.angle = pattern.angle + estdata.angle;
			pattern.distance = pattern.distance + estdata.distance;
		}
		else {
			if (pattern.distance < 500) {
				ret = -2;
			}
			else {
				ret = 2;
			}
		}
	}
	else if (fabs(estdata.angle) >= threshold[2] && fabs(estdata.angle) <= threshold[3]) {
		if (mode == 1) {
			if (fabs(pattern.angle) < 30) {
				ret = -1;
			}
			else {
				ret = 1;
			}
		}
		else if(mode == 2){
			if (pattern.distance < 500) {
				ret = -2;
			}
			else {
				ret = 2;
			}
		}
	}
	if (ret == 1 || ret == 2) {
		pattern.gps_week = estdata.gps_week;
		pattern.endtime = estdata.time;
		patterns.push_back(pattern);
	}
	if (ret != 0) {
		clear_pattern();
	}
	return ret;
}

int DriveStatus::calpatterns(int mode) {
	int estflag = 0;
	if (mode == 1) {
		float totalangle[2] = { 0,0 };
		for (int i = 0; i < patterns.size(); i++) {
			if (patterns[i].mode == 1) {
				if (patterns[i].angle > 0) {
					totalangle[0] += patterns[i].angle;
				}
				else {
					totalangle[1] += patterns[i].angle;
				}
			}
		}
		liveresult.totalangle_r = totalangle[0];
		liveresult.totalangle_l = totalangle[1];
		if (totalangle[0] > 220 && totalangle[1] < -200) {
			estflag = 3;
		}
		else if (totalangle[0] > 220) {
			estflag = 2;
		}
		else if (totalangle[1] < -220) {
			estflag = 1;
		}
		else {
			estflag = 0;
		}
	}
	else if (mode == 2) {
		if (pattern.mode == 2) {
			if (pattern.distance > 300) {
				liveresult.gps_week = pattern.gps_week;
				liveresult.starttime = pattern.starttime;
				liveresult.curtime = pattern.curtime;
			}
			else {
				liveresult.gps_week = 0;
				liveresult.starttime = 0;
				liveresult.curtime = 0;
			}
			liveresult.distance = pattern.distance;
			if (pattern.distance > 700) {
				estflag = 7;
			}
			else if (pattern.distance > 500) {
				estflag = 6;
			}
			else if (pattern.distance > 300) {
				estflag = 5;
			}
			else {
				estflag = 4;
			}
		}
	}
	return estflag;
}

liveresult_t* DriveStatus::getresult() {
	if (curinsresult.gps_millisecs % 1000 < 10) {
		return &liveresult;
	}
	return NULL;
}