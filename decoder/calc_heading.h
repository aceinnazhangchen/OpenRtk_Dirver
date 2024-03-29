#ifndef _CALC_HEADING_H_
#define _CALC_HEADING_H_

#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif

#ifndef PI
#define PI          3.1415926535897932   /* pi */
#endif // !PI

	uint8_t UpdateMN(const double *BLH, double *M, double *N);
	double get_heading(double prepos[3], double curpos[3], double dt);


#ifdef __cplusplus
}
#endif

#endif
