#ifndef _MODEL_H_
#define _MODEL_H_

#include <stdio.h>
#include "rtklib_core.h"
#ifdef __cplusplus
extern "C" {
#endif


extern void blh2C_en(const double *blh, double C_en[3][3]);
extern void xyz2ned(double C_en[3][3], double *xyz, double *covXYZ, double *ned, double *covNED);
extern void blhdiff(double *blh, double *blh_ref, double *ned);
extern void ecef2enu(const double *pos, const double *r, double *e);

extern void xyz2enu_(const double *pos, double *E);
extern void enu2ecef(const double *pos, const double *e, double *r);
extern void blh2xyz(const double *blh, double *xyz);
extern void xyz2blh(const double *xyz, double *blh);
extern void covecef(const double *pos, const double *Q, double *P);

extern void xyz2enu(const double *pos, double *E);

extern double satazel(const double *pos, const double *e, double *azel);

extern double geodist(const double *rs, const double *rr, double *e);

extern double geovel(const double *rs, const double *rr, double *e);

extern double tropmodel(const double *blh, const double *azel, double humi);
extern double tropmapf(gtime_t time, const double pos[], const double azel[], double *mapfw);
void deg2dms(double deg, double *dms, int ndec);

/* output NMEA GGA */
int print_nmea_gga(double *ep, double *xyz, int nsat, int type, double dop, double age, char *buff);
int print_nmea_gst(double *ep, float* var_llh, char* buff);

#ifdef __cplusplus
}
#endif
#endif