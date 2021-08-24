#pragma once
#include <stdio.h>
#include <stdint.h>
#include <vector>

#define HEADKML1 "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n"
#define HEADKML2 "<kml xmlns=\"http://www.opengis.net/kml/2.2\">\n"
#define MARKICON "http://maps.google.com/mapfiles/kml/shapes/track.png"
//#define R2D   (180/3.1415926)
#define SIZP     0.3            /* mark size of rover positions */
#define SIZR     0.3            /* mark size of reference position */
#define TINT     30.0           /* time label interval (sec) */

#pragma pack(push, 1)
struct kml_gnss_t {
	uint16_t	gps_week;
	double		gps_secs;
	uint8_t		position_type;
	double		latitude;
	double		longitude;
	double		height;
	float		north_vel;
	float		east_vel;
	float		up_vel;
};

struct kml_ins_t {
	uint16_t	gps_week;
	double		gps_secs;
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
#pragma pack(pop)

class Kml_Generator
{
private:
	Kml_Generator();
	~Kml_Generator();
public:
	static Kml_Generator* Instance();
	static void createInstance();
private:
	static Kml_Generator* m_instance;
public:
	void init();
	void open_files(char* file_base_name);
	void write_files();
	void close_files();
	void append_gnss(kml_gnss_t& gnss);
	void append_ins(kml_ins_t& ins);
private:
	void write_header(FILE *kml_file, int ntype);
	void write_end(FILE *kml_file);
	void write_gnss_line(kml_gnss_t* gnss, int begin_end);
	void write_gnss_point(kml_gnss_t* gnss, int begin_end);
	void write_ins_line(kml_ins_t* ins, int begin_end);
	void write_ins_point(kml_ins_t* ins, int begin_end);
	void write_gnss_kml();
	void write_ins_kml();
private:
	int kml_description;
	FILE* f_gnss_kml;
	FILE* f_ins_kml;
	std::vector<kml_gnss_t> gnss_sol_list;
	std::vector<kml_ins_t> ins_sol_list;
};