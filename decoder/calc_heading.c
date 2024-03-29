#include "calc_heading.h"
#include <math.h>

typedef  struct EarthParameter
{
	double a;       //Ellipsoid long axis
	double b;       //Ellipsoid short axis
	double f;       //Ellipsoidal oblate 
	double e;       //first Eccentricity of Elliopsoid 
	double e2;
	//double ep;
	//double ep2;     //second Eccentricity of Elliopsoid 
	double wie;     //rotational angular velocity of the earths  
	double GM;      //geocentric gravitational constant 
} EarthParameter;

typedef struct GravityParameter
{
	double g0;
	double g1;
	double g2;
	double g3;
	double g4;
	double g5;
} GravityParameter;

const GravityParameter normg = { 9.7803267715 ,0.0052790414 ,0.0000232718,-0.000003087691089,0.00000000439773 , 0.000000000000721 };
const  EarthParameter WGS84 = { 6378137.0, 6356752.3142, 0.0033528106643315515,0.081819190837555025,0.0066943799893122479 ,  7.2922115147e-5,398600441800000.00 };


uint8_t UpdateMN(const double *BLH, double *M, double *N)
{
	double sinB = sin(*BLH);
	double temp = 1 - WGS84.e2 * sinB * sinB;
	double sqrttemp = sqrt(temp);
	*M = WGS84.a * (1 - WGS84.e2) / (sqrttemp*temp);
	*N = WGS84.a / sqrttemp;
	return 1;
};

double get_heading(double prepos[3], double curpos[3], double dt)
{
	double M = 0.0, N = 0.0;
	UpdateMN(prepos, &M, &N);
	double vn = (curpos[0] - prepos[0])* (M + curpos[2]) / dt;
	double ve = (curpos[1] - prepos[1])* (N + curpos[2])*cos(curpos[0] * PI / 180.0) / dt;
	double heading = atan2(ve, vn);
	return heading;
}
