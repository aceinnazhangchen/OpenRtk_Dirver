#ifndef _GNSS_DATATYPE_H_
#define _GNSS_DATATYPE_H_
#include "rtklib_core.h"
#ifdef __cplusplus
extern "C" {
#endif

#ifndef MAXOBS
#define MAXOBS      48
#endif

#define NFREQ       2
#ifndef NSYS
#define NSYS        4  /* only use GPS, GLO, GAL, BDS */
#endif

#ifdef _POST_RTK_
#define MAXAMB       (30)
#else
#define MAXAMB       (20)
#endif // _POST_RTK_
#define MAXAMBINSET (MAXAMB)

#define MI(i, j, n) ((i) * (n) + (j))
#define SMD(i) ((i)*((i)+1)/2)
#define SMI(i, j)	((i) > (j) ? (SMD(i) + (j)) : (SMD(j) + (i)))

#define NX_PHB (NSYS*NFREQ)              /*refsat ambiguity + phase bias*/
#define NX_RTD (9+1+NX_CLK+NX_PHB)       /* p(3),v(3),a(3),cdt_rate(1), cdt(NSYS*NFREQ),
                                            glonass_bias(NFREQ), sd_refamb(NSYS*NFREQ) */
#define NP_RTD (SMD(NX_RTD))
#ifndef NX_RTK
#define NX_RTK (NX_RTD+MAXAMB)           /*  p(3),v(3),a(3),cdt_rate(1), cdt(NSYS*NFREQ),
                                            glonass_bias(NFREQ), sd_refamb(NSYS*NFREQ),
                                            dd_amb(MAXAMB) */
#endif
#define NP_RTK (SMD(NX_RTK))

#define NX_CLK (NSYS*NFREQ)
#define NX_SPP (6+NX_CLK)
#define NX     (3+NX_CLK)         /* # of estimated parameters */
#define NP_SPP (SMD(NX_SPP))

typedef struct {
	unsigned char sat;   /*prn*/
    double rs[6];
    double dts[2];
    double var;
    int svh;    
	double azel[2];    /*azimuth,elevation*/
	double e[3];       /*partial deviation*/
	////double tgd;        /* tgd*/
	double r;          /* vector */
	double rate;
	double tro;        /* tropospheric */
}vec_t;

typedef struct
{
    obs_t obs;
    vec_t vec[MAXOBS];
}epoch_t;

typedef struct {        /* observation data */
    double x[NX_SPP];
    double P[NP_SPP];
    double time;
    unsigned char n_used;
    unsigned char solType;
} rcv_spp_t;

typedef struct
{
    unsigned char s1, s2, f, nlock; /*nlock=0-> cycleslip*/
    double data;
    double var;
    double time;
}ambdata_t;

typedef struct
{
    unsigned char loc;
    unsigned char s1;
    unsigned char s2;
    unsigned char f;
    double value;
}ambloc_t;

typedef struct
{
    unsigned char n;
    ambdata_t amb[MAXAMBINSET];
    double ratio;
    unsigned char nsat;
}ambset_t;

typedef struct
{
    unsigned char  sat;               /* satellite status type */
    unsigned char slip[NFREQ];        /* cycle-slip flag */
    unsigned char nlock[NFREQ];       /* data lock epoch */
    double  gf;                       /* geometry-free phase L1-L2 (m) */
    double  gf2;                      /* geometry-free phase L1-L2 (m) for rover*/
    double  L[NFREQ];                 /* phase (cycle) */
    double  D[NFREQ];                 /* doppler (cycle) */
    double dph[NFREQ];                /* delta phase (cycle)*/
} slipset_t;

typedef struct
{
    unsigned char glo_ifbflag;
    double glo_ifb;
    double glo_ifb_var;
    int ifb_num;
} glo_ifb_t;

#define NX_MAX (60)
#define MAXPAR 10 /* maximum parameters in one design matrix row */

typedef struct
{
    unsigned int s1, s2, f;
    double time;
} state_tag_t;

typedef struct
{
    double H[MAXPAR];
    int L[MAXPAR];
    int numl;
    double z;
    double R;
    double elev;
    double azim;
    int SNR;
    int refsat;
    int sat, sys, prn, code, flag, cslip, nlock;
    unsigned char isd;
} measure_t;


typedef struct
{
    double x[NX_RTK];
    double P[NP_RTK];
    state_tag_t tag[MAXAMB];
    double x_fixed[NX_RTK];
    slipset_t slip[MAXOBS];
    ambset_t ambset;
    rcv_spp_t spp;
    glo_ifb_t glo_ifb;
    unsigned int np;
    unsigned int ns;
    double time;
    double tt;
    double code_blunder_rate;
    double age;
    int imuflag;
    unsigned int num_fix;
    unsigned int num_sat;
    double dop[5];
    int fixType;
    double lastTimeRef;
}rcv_rtk_t;

#ifdef __cplusplus
}
#endif
#endif