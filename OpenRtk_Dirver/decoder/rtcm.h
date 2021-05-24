#ifndef _RTCM_H_
#define _RTCM_H_

#include <time.h>
#include <stdio.h>
#include <stdint.h>
#include <math.h>

#ifdef __cplusplus
extern "C" {
#endif

#include "rtklib_core.h"
#include "rtkcmn.h"
//#include "model.h"

#ifdef RTK_ENABLE
#define fprintf(format,args...)        //add function to avoid armcc compiler
#define printf(format,args...)        //add function to avoid armcc compiler
#define fclose  //
#define fopen  //
#endif

#ifndef WIN32
#define ARM_MCU
#endif


// #ifdef ARM_MCU
// #pragma GCC diagnostic ignored "-Wunused-but-set-variable"
// #pragma GCC diagnostic ignored "-Wunused-variable"
// #pragma GCC diagnostic ignored "-Wunused-function"
// #pragma GCC diagnostic ignored "-Wunused-const-variable="
// #endif
//#endif

/*-----------------------------------------------------------*/
/* from rtklib to decode RTCM3 */
#define RTCM2PREAMB 0x66 /* rtcm ver.2 frame preamble */
#define RTCM3PREAMB 0xD3 /* rtcm ver.3 frame preamble */

typedef struct {
    /* move the observation data struct out of rtcm definiton, to save more memory for PPP only mode */
    obs_t obs[MAXSTN];
    rtcm_t rcv[MAXSTN];
    nav_t  nav;
	double time;
} gnss_rtcm_t;
typedef struct
{
	int32_t message_number;
	int32_t sub_type_id;
	int32_t gps_epoch_time;
	int32_t week;
	int32_t leap_seconds;
	int32_t safety_info;
	int32_t protocol_version_flags;
	int32_t firmware_version;
	int32_t reserved1;
	int32_t reserved2;
	int32_t reserved3;
	int32_t pps_status;
	int32_t time_validity;
	int32_t constellation_alarm_mask;
	int32_t reserved4;
	int32_t gnss_constellation_mask;
	int32_t gnss_mf_constellation_mask;
} RTCM_999_RSS_STRUCT;
extern RTCM_999_RSS_STRUCT rtcm_999_rss;
void decode_type999_subtype1(rtcm_t *rtcm, obs_t *obs);

int decode_rtcm3(rtcm_t *rtcm, obs_t *obs, nav_t *nav);
int input_rtcm3_data(rtcm_t *rtcm, unsigned char data, obs_t *obs, nav_t *nav);

/* interface to GNSS db */
int input_rtcm3(unsigned char data, unsigned int stnID, gnss_rtcm_t *gnss);

unsigned int rtcm_getbitu(const unsigned char *buff, int pos, int len);

/* glo frquent number function */
extern void set_glo_frq(int prn, int frq);
extern int get_glo_frq(int prn);

 void set_week_number(int week);
 int  get_week_number();

int satno(int sys, int prn);

/* satellite function */
int  satsys(int sat, int *prn);
int  satidx(int sat, int *prn);
char satid (int sat, int *prn);
char sys2char(int sys);
extern double satwavelen(int sat, int code);

extern double satwavelenbyfreq(int sat, int frq);

unsigned char obs2code(int sys, const char * obs, int * freq);
unsigned char obs2coderinex(int sys, const char *obs, int *freq);

extern int code2frq(int sys, int code);

int getcodepri(int sys, unsigned char code, const char * opt);

void ecef2pos(const double *r, double *pos);
void pos2ecef(const double *pos, double *r);

void set_approximate_time(int year, int doy, rtcm_t *rtcm);

int add_obs(obsd_t* obsd, obs_t* obs);
int add_eph(eph_t* eph, nav_t* nav);
int add_geph(geph_t* eph, nav_t* nav);

int is_complete_rtcm();
//int gen_rtcm3(rtcm_t* rtcm, obs_t *obs, int type, int sync);
extern unsigned int rtk_crc24q(const unsigned char *buff, int len);

#ifdef __cplusplus
}
#endif
#endif