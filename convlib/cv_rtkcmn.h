#ifndef RTKCMN_H
#define RTKCMN_H

#include "rtklib_core.h"
#include <time.h>
#include <string.h>
#include <stdint.h>
#ifdef WIN32
#include <windows.h>
#endif

#ifdef __cplusplus
extern "C" {
#endif
/* time and string functions -------------------------------------------------*/
gtime_t cv_timeget();
void    cv_time2str(gtime_t t, char *str, int n);
RTK_RAM_CODE gtime_t cv_epoch2time(const double *ep);
RTK_RAM_CODE void    cv_time2epoch(gtime_t t, double *ep);
RTK_RAM_CODE gtime_t cv_gpst2time(int week, double sec);
RTK_RAM_CODE double  cv_time2gpst(gtime_t t, int *week);
RTK_RAM_CODE gtime_t cv_gst2time(int week, double sec);
RTK_RAM_CODE gtime_t cv_bdt2time(int week, double sec);
RTK_RAM_CODE double  cv_time2bdt(gtime_t t, int *week);
RTK_RAM_CODE gtime_t cv_timeadd(gtime_t t, double sec);
RTK_RAM_CODE double  cv_timediff(gtime_t t1, gtime_t t2);
RTK_RAM_CODE gtime_t cv_gpst2utc(gtime_t t);
RTK_RAM_CODE gtime_t cv_utc2gpst(gtime_t t);
RTK_RAM_CODE gtime_t cv_gpst2bdt(gtime_t t);
RTK_RAM_CODE gtime_t cv_bdt2gpst(gtime_t t);

int cv_adjgpsweek(int week);

uint32_t cv_rtk_crc24q(const unsigned char *buff, int len);
uint32_t cv_rtk_crc32(const uint8_t* buff, int len);
/* satellites, systems, codes functions --------------------------------------*/

int  cv_satno(int sys, int prn);
int  cv_satsys(int sat, int *prn);

#ifdef __cplusplus
}
#endif

#endif