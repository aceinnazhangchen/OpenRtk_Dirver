#pragma once
#include <stdint.h>
#include <vector>

struct ins_sol_data {
	uint16_t	gps_week;
	uint32_t	gps_millisecs;
	uint8_t		ins_status;
	uint8_t		ins_position_type;
	double		latitude;
	double		longitude;
	double		height;
	float		north_velocity;
	float		east_velocity;
	float		up_velocity;
	float		roll;
	float		pitch;
	float		heading;
};

struct est_t {
	uint8_t		status;
	uint8_t		postype;
	uint32_t	time;		// millisecs
	float		angle;
	double		distance;
};

struct pattern_t {
	uint8_t		mode;
	uint32_t	starttime;	// millisecs
	uint32_t	curtime;	// millisecs
	uint32_t	endtime;	// millisecs
	float		angle;
	double		distance;
};

struct liveresult_t {
	uint8_t		type;
	float		totalangle_r;
	float		totalangle_l;
	double		distance;
	uint32_t	starttime;	// millisecs
	uint32_t	curtime;	// millisecs
};

class DriveStatus
{
public:
	DriveStatus();
	~DriveStatus();
	void init();
	void init_patterns();
	void clear_pattern();
	void addrawdata(ins_sol_data * data);
	void addestcheckdata();
	void addestdata2patterns();
	int checkdatapattern(int mode);
	int calpatterns(int mode);
	liveresult_t * getresult();
private:
	ins_sol_data curinsresult;
	ins_sol_data lastinsresult;
	est_t estdata;
	pattern_t pattern;
	std::vector<pattern_t> patterns;
	liveresult_t liveresult;
	int statusstage;
};

